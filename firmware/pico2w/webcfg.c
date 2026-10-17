/*
 * webcfg - settings page at http://192.168.20.1 over a USB network adapter.
 *
 * The Pico shows up as a USB network card (CDC-NCM, built into Windows 10/11,
 * macOS and Linux) next to the sound card. A tiny DHCP server gives the PC
 * 192.168.20.2; the page and a small JSON API are served from here.
 *
 * Threading: TinyUSB runs in interrupt context (USB IRQ + 250 us timer), lwIP
 * runs only in the core 0 main loop (webcfg_poll). Frames cross between them
 * through one receive buffer and a small transmit queue, so neither side ever
 * calls into the other's stack.
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/sync.h"
#include "tusb.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "lwip/netif.h"
#include "lwip/ethip6.h"
#include "lwip/tcp.h"
#include "netif/ethernet.h"
#include "dhserver.h"
#include "webcfg.h"

#define INIT_IP4(a, b, c, d) { PP_HTONL(LWIP_MAKEU32(a, b, c, d)) }
static const ip4_addr_t s_ip = INIT_IP4(192, 168, 20, 1);
static const ip4_addr_t s_mask = INIT_IP4(255, 255, 255, 0);
static const ip4_addr_t s_gw = INIT_IP4(0, 0, 0, 0);
static dhcp_entry_t s_leases[] = {
    { {0}, INIT_IP4(192, 168, 20, 2), 24 * 60 * 60 },
    { {0}, INIT_IP4(192, 168, 20, 3), 24 * 60 * 60 },
};
static const dhcp_config_t s_dhcp = {
    .router = INIT_IP4(0, 0, 0, 0),           /* no gateway: the PC keeps its internet route */
    .port = 67,
    .dns = INIT_IP4(0, 0, 0, 0),
    .domain = "lakehp",
    .num_entry = TU_ARRAY_SIZE(s_leases),
    .entries = s_leases,
};

uint8_t tud_network_mac_address[6] = { 0x02, 0x4C, 0x48, 0x50, 0x00, 0x01 };

static struct netif s_netif;
static const webcfg_hooks_t *s_hooks;
static bool s_up;

/* ------------------------------------------------------------------ USB <-> lwIP glue */
static uint8_t  s_rx[CFG_TUD_NET_MTU];
static volatile uint16_t s_rx_len;          /* !=0: a frame waits for the main loop */
static volatile bool s_rx_renew;            /* main loop done with it: renew in USB context */

#define TXQ 2
static uint8_t  s_tx[TXQ][CFG_TUD_NET_MTU];
static uint16_t s_tx_len[TXQ];
static volatile uint32_t s_tx_w, s_tx_r;

bool tud_network_recv_cb(const uint8_t *src, uint16_t size)            /* USB context */
{
    if (s_rx_len || !size) return size == 0;
    if (size > sizeof s_rx) return true;                                  /* drop */
    memcpy(s_rx, src, size);
    __atomic_store_n(&s_rx_len, size, __ATOMIC_RELEASE);
    return true;
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg)   /* USB context */
{
    memcpy(dst, ref, arg);
    return arg;
}

void tud_network_init_cb(void)                                          /* USB context */
{
    s_rx_len = 0;
    s_tx_r = s_tx_w;
}

void webcfg_usb_service(void)                                           /* USB context */
{
    if (s_rx_renew) { s_rx_renew = false; tud_network_recv_renew(); }
    while (s_tx_r != __atomic_load_n(&s_tx_w, __ATOMIC_ACQUIRE)) {
        const uint32_t i = s_tx_r % TXQ;
        if (!tud_network_can_xmit(s_tx_len[i])) break;
        tud_network_xmit(s_tx[i], s_tx_len[i]);
        __atomic_store_n(&s_tx_r, s_tx_r + 1, __ATOMIC_RELEASE);
    }
}

static err_t linkoutput(struct netif *netif, struct pbuf *p)            /* main loop */
{
    (void)netif;
    if (!tud_ready() || p->tot_len > CFG_TUD_NET_MTU) return ERR_IF;
    const uint32_t deadline = to_ms_since_boot(get_absolute_time()) + 20;
    while (s_tx_w - __atomic_load_n(&s_tx_r, __ATOMIC_ACQUIRE) >= TXQ)   /* USB drains it */
        if ((int32_t)(to_ms_since_boot(get_absolute_time()) - deadline) > 0) return ERR_MEM;
    const uint32_t i = s_tx_w % TXQ;
    s_tx_len[i] = (uint16_t)pbuf_copy_partial(p, s_tx[i], p->tot_len, 0);
    __atomic_store_n(&s_tx_w, s_tx_w + 1, __ATOMIC_RELEASE);
    return ERR_OK;
}

static err_t netif_init_cb(struct netif *n)
{
    n->mtu = CFG_TUD_NET_MTU - 14;
    n->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_UP;
    n->name[0] = 'u'; n->name[1] = 's';
    n->linkoutput = linkoutput;
    n->output = etharp_output;
    return ERR_OK;
}

u32_t sys_now(void) { return to_ms_since_boot(get_absolute_time()); }

/* ------------------------------------------------------------------ HTTP */
extern const char lhp_web_ui[];
extern const unsigned lhp_web_ui_len;

#define REQ_MAX  1536
#define RESP_MAX 1536
typedef struct {
    struct tcp_pcb *pcb;
    char  req[REQ_MAX];
    int   req_len;
    char  head[256];
    char  resp[RESP_MAX];
    const char *body;          /* what is still to be sent */
    int   body_left;
    bool  head_sent, used;
} conn_t;
static conn_t s_conn[2];

static void conn_close(conn_t *c)
{
    if (c->pcb) {
        tcp_arg(c->pcb, NULL); tcp_recv(c->pcb, NULL); tcp_sent(c->pcb, NULL); tcp_err(c->pcb, NULL);
        if (tcp_close(c->pcb) != ERR_OK) tcp_abort(c->pcb);
    }
    c->pcb = NULL;
    c->used = false;
}

static void conn_push(conn_t *c)
{
    if (!c->head_sent) {
        if (tcp_write(c->pcb, c->head, (u16_t)strlen(c->head), TCP_WRITE_FLAG_COPY) != ERR_OK) return;
        c->head_sent = true;
    }
    while (c->body_left > 0) {
        int n = tcp_sndbuf(c->pcb);
        if (n > c->body_left) n = c->body_left;
        if (n > TCP_MSS) n = TCP_MSS;
        if (n <= 0 || tcp_write(c->pcb, c->body, (u16_t)n, 0) != ERR_OK) break;   /* body is static */
        c->body += n; c->body_left -= n;
    }
    tcp_output(c->pcb);
}

static void respond(conn_t *c, const char *status, const char *type, const char *body, int len)
{
    snprintf(c->head, sizeof c->head,
             "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %d\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
             status, type, len);
    c->body = body; c->body_left = len; c->head_sent = false;
    conn_push(c);
}

static void handle(conn_t *c)
{
    char *hdr_end = strstr(c->req, "\r\n\r\n");
    const char *body = hdr_end + 4;
    if (!strncmp(c->req, "GET / ", 6) || !strncmp(c->req, "GET /index.html", 15)) {
        respond(c, "200 OK", "text/html; charset=utf-8", lhp_web_ui, (int)lhp_web_ui_len);
    } else if (!strncmp(c->req, "GET /api/state", 14)) {
        s_hooks->state_json(c->resp, RESP_MAX);
        respond(c, "200 OK", "application/json", c->resp, (int)strlen(c->resp));
    } else if (!strncmp(c->req, "POST /api/set", 13)) {
        s_hooks->apply_json(body, c->resp, RESP_MAX);
        respond(c, "200 OK", "application/json", c->resp, (int)strlen(c->resp));
    } else {
        static const char nf[] = "not found";
        respond(c, "404 Not Found", "text/plain", nf, (int)sizeof nf - 1);
    }
}

static err_t on_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
    (void)pcb; (void)len;
    conn_t *c = arg;
    if (!c) return ERR_OK;
    conn_push(c);
    if (c->head_sent && c->body_left == 0 && tcp_sndbuf(c->pcb) == TCP_SND_BUF) conn_close(c);
    return ERR_OK;
}

static err_t on_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    conn_t *c = arg;
    if (!p || err != ERR_OK) { if (p) pbuf_free(p); if (c) conn_close(c); return ERR_OK; }
    if (!c) { tcp_recved(pcb, p->tot_len); pbuf_free(p); return ERR_OK; }
    int room = REQ_MAX - 1 - c->req_len;
    int n = p->tot_len < room ? p->tot_len : room;
    pbuf_copy_partial(p, c->req + c->req_len, (u16_t)n, 0);
    c->req_len += n;
    c->req[c->req_len] = 0;
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    char *he = strstr(c->req, "\r\n\r\n");
    if (!he) { if (c->req_len >= REQ_MAX - 1) conn_close(c); return ERR_OK; }
    int want = 0;
    const char *cl = strstr(c->req, "Content-Length:");
    if (!cl) cl = strstr(c->req, "content-length:");
    if (cl && cl < he) want = atoi(cl + 15);
    if ((he + 4 - c->req) + want <= c->req_len || c->req_len >= REQ_MAX - 1) handle(c);
    return ERR_OK;
}

static void on_err(void *arg, err_t err)
{
    (void)err;
    conn_t *c = arg;
    if (c) { c->pcb = NULL; c->used = false; }
}

static err_t on_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
    (void)arg;
    if (err != ERR_OK || !pcb) return ERR_VAL;
    conn_t *c = NULL;
    for (unsigned i = 0; i < TU_ARRAY_SIZE(s_conn); i++) if (!s_conn[i].used) { c = &s_conn[i]; break; }
    if (!c) { tcp_abort(pcb); return ERR_ABRT; }
    memset(c, 0, sizeof *c);
    c->used = true; c->pcb = pcb;
    tcp_arg(pcb, c); tcp_recv(pcb, on_recv); tcp_sent(pcb, on_sent); tcp_err(pcb, on_err);
    return ERR_OK;
}

/* ------------------------------------------------------------------ public */
void webcfg_init(const webcfg_hooks_t *hooks)
{
    s_hooks = hooks;
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    memcpy(&tud_network_mac_address[2], &id.id[4], 4);   /* stable, locally administered */
    lwip_init();
    s_netif.hwaddr_len = 6;
    memcpy(s_netif.hwaddr, tud_network_mac_address, 6);
    s_netif.hwaddr[5] ^= 0x01;                              /* differs from the host side */
    netif_add(&s_netif, &s_ip, &s_mask, &s_gw, NULL, netif_init_cb, ethernet_input);
    netif_set_default(&s_netif);
    netif_set_up(&s_netif);
    dhserv_init(&s_dhcp);
    struct tcp_pcb *l = tcp_new();
    tcp_bind(l, IP_ADDR_ANY, 80);
    l = tcp_listen(l);
    tcp_accept(l, on_accept);
    s_up = true;
}

void webcfg_poll(void)
{
    if (!s_up) return;
    const uint16_t n = __atomic_load_n(&s_rx_len, __ATOMIC_ACQUIRE);
    if (n) {
        struct pbuf *p = pbuf_alloc(PBUF_RAW, n, PBUF_POOL);
        if (p) {
            pbuf_take(p, s_rx, n);
            if (s_netif.input(p, &s_netif) != ERR_OK) pbuf_free(p);
        }
        __atomic_store_n(&s_rx_len, 0, __ATOMIC_RELEASE);
        s_rx_renew = true;
    }
    sys_check_timeouts();
}

/* tiny JSON helpers for the hooks (our own page is the only client) */
bool webcfg_json_num(const char *json, const char *key, float *out)
{
    char pat[40];
    snprintf(pat, sizeof pat, "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (!p) return false;
    p = strchr(p + strlen(pat), ':');
    if (!p) return false;
    p++;
    while (*p == ' ') p++;
    if (!strncmp(p, "true", 4)) { *out = 1; return true; }
    if (!strncmp(p, "false", 5)) { *out = 0; return true; }
    char *end;
    *out = strtof(p, &end);
    return end != p;
}

bool webcfg_json_str(const char *json, const char *key, char *out, int max)
{
    char pat[40];
    snprintf(pat, sizeof pat, "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (!p) return false;
    p = strchr(p + strlen(pat), ':');
    if (!p) return false;
    p = strchr(p, '"');
    if (!p) return false;
    p++;
    int i = 0;
    while (*p && *p != '"' && i < max - 1) out[i++] = *p++;
    out[i] = 0;
    return true;
}
