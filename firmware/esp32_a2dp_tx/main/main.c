/*
 * LakeHP 5.1 - classic ESP32 half: Bluetooth A2DP transmitter.
 *
 * I2S slave input (48 kHz, 32-bit slots, from the ESP32-S3 running
 * firmware/esp32s3_usb_dsp) -> ring buffer -> adaptive resampler (to whatever
 * rate the headset negotiated, usually 44.1 kHz, plus clock-drift tracking
 * between the two boards) -> Bluedroid A2DP source (SBC).
 *
 * Pairing: scans for the first Audio/Video "rendering" device (optionally
 * filtered by name), remembers it in NVS and reconnects to it automatically.
 * Hold BOOT for 2 s to forget it and pair a different headset.
 *
 * Structure follows ESP-IDF v5.5 examples/bluetooth/bluedroid/classic_bt/a2dp_source.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "resampler.h"

static const char *TAG = "lakehp_tx";

/* ------------------------------------------------------------------ ring (I2S -> A2DP) */
#define RING_FRAMES   8192                    /* power of 2, ~170 ms at 48 kHz */
#define TARGET_FILL   CONFIG_LHP_TX_TARGET_FILL   /* slack between the two boards' clocks */
static int16_t s_ring[RING_FRAMES * 2];
static volatile uint32_t s_rw, s_rr;
#define LD(x)    __atomic_load_n(&(x), __ATOMIC_ACQUIRE)
#define ST(x, v) __atomic_store_n(&(x), (v), __ATOMIC_RELEASE)

static i2s_chan_handle_t s_rx;

static void i2s_rx_task(void *arg)
{
    static int32_t buf[240 * 2];
    for (;;) {
        size_t got = 0;
        if (i2s_channel_read(s_rx, buf, sizeof buf, &got, portMAX_DELAY) != ESP_OK) continue;
        uint32_t frames = got / 8, w = s_rw;
        uint32_t space = RING_FRAMES - (w - LD(s_rr));
        if (frames > space) frames = space;           /* overflow: resampler will speed up */
        for (uint32_t i = 0; i < frames; i++) {
            uint32_t idx = ((w + i) & (RING_FRAMES - 1)) * 2;
#if CONFIG_LHP_SWAP_LR
            s_ring[idx] = (int16_t)(buf[2 * i + 1] >> 16);
            s_ring[idx + 1] = (int16_t)(buf[2 * i] >> 16);
#else
            s_ring[idx] = (int16_t)(buf[2 * i] >> 16);
            s_ring[idx + 1] = (int16_t)(buf[2 * i + 1] >> 16);
#endif
        }
        ST(s_rw, w + frames);
    }
}

static void i2s_init(void)
{
    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    ESP_ERROR_CHECK(i2s_new_channel(&cc, NULL, &s_rx));
    i2s_std_config_t sc = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = CONFIG_LHP_I2S_BCLK,
            .ws = CONFIG_LHP_I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = CONFIG_LHP_I2S_DIN,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_rx, &sc));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx));
}

/* ------------------------------------------------------------------ A2DP PCM source */
/* Two resamplers: tables are rebuilt off the audio path (in a2d_cb) and swapped in. */
static resampler_t s_rs_buf[2];
static resampler_t *volatile s_rs;
static uint32_t s_rs_rate[2];
static volatile uint32_t s_out_rate = 44100;   /* updated from the negotiated SBC config */
static volatile bool s_rs_reset = true;

static void select_rate(uint32_t rate)          /* not called from the PCM callback */
{
    if (s_rs && s_rs_rate[s_rs - s_rs_buf] == rate) return;
    int idx = (s_rs == &s_rs_buf[0]) ? 1 : 0;
    rs_init(&s_rs_buf[idx], 48000, rate);
    s_rs_rate[idx] = rate;
    __atomic_store_n(&s_rs, &s_rs_buf[idx], __ATOMIC_RELEASE);
    s_out_rate = rate;
    s_rs_reset = true;
}
static bool s_primed;
static float s_integ;                          /* integral term (ppm) */

/* Called by Bluedroid's A2DP source task: fill `len` bytes of 16-bit stereo. */
static int32_t a2d_data_cb(uint8_t *data, int32_t len)
{
    if (!data || len < 0) { s_rs_reset = true; return 0; }
    int16_t *out = (int16_t *)data;
    const int frames = len / 4;

    resampler_t *rs = __atomic_load_n(&s_rs, __ATOMIC_ACQUIRE);
    if (s_rs_reset) {
        rs_reset(rs);
        s_rs_reset = false;
        s_primed = false;
        s_integ = 0;
    }
    uint32_t level = LD(s_rw) - s_rr;
    if (!s_primed) {                            /* wait for slack before starting */
        if (level < TARGET_FILL) { memset(data, 0, len); return len; }
        ST(s_rr, s_rr + level - TARGET_FILL);   /* drop stale audio */
        level = TARGET_FILL;
        s_primed = true;
    }

    /* PI control of the ring level: compensates the S3/ESP32 crystal difference. */
    float err = ((float)level - TARGET_FILL) / TARGET_FILL;
    s_integ += err * 2.0f;
    if (s_integ > 800) s_integ = 800;
    if (s_integ < -800) s_integ = -800;
    float ppm = 400.0f * err + s_integ;
    if (ppm > 2000) ppm = 2000;
    if (ppm < -2000) ppm = -2000;
    rs_trim_ppm(rs, ppm);

    int made = 0;
    while (made < frames) {
        uint32_t avail = LD(s_rw) - s_rr;
        if (!avail) {                           /* underrun: pad, re-prime */
            memset(&out[2 * made], 0, (frames - made) * 4);
            s_primed = false;
            break;
        }
        uint32_t r = s_rr & (RING_FRAMES - 1);
        uint32_t chunk = RING_FRAMES - r;
        if (chunk > avail) chunk = avail;
        int used = 0;
        made += rs_process(rs, &s_ring[2 * r], (int)chunk, &used, &out[2 * made], frames - made);
        ST(s_rr, s_rr + used);
        if (!used && made < frames) break;
    }
    return len;
}

/* ------------------------------------------------------------------ connection state */
typedef enum { ST_IDLE, ST_SCANNING, ST_CONNECTING, ST_CONNECTED, ST_STREAMING } app_state_t;
static volatile app_state_t s_state;
static esp_bd_addr_t s_peer;
static bool s_have_peer, s_found;
static esp_timer_handle_t s_retry_timer;
static SemaphoreHandle_t s_lock;               /* serialises state between BTC, timer and UI tasks */
static esp_bd_addr_t s_forgotten;
static bool s_has_forgotten;
#define LOCK()   xSemaphoreTakeRecursive(s_lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGiveRecursive(s_lock)

static void save_peer(const esp_bd_addr_t a)
{
    nvs_handle_t h;
    if (nvs_open("lakehp", NVS_READWRITE, &h) == ESP_OK) { nvs_set_blob(h, "peer", a, 6); nvs_commit(h); nvs_close(h); }
}
static bool load_peer(esp_bd_addr_t a)
{
    nvs_handle_t h; size_t n = 6; bool ok = false;
    if (nvs_open("lakehp", NVS_READONLY, &h) == ESP_OK) { ok = nvs_get_blob(h, "peer", a, &n) == ESP_OK && n == 6; nvs_close(h); }
    return ok;
}
static void erase_peer(void)
{
    nvs_handle_t h;
    if (nvs_open("lakehp", NVS_READWRITE, &h) == ESP_OK) { nvs_erase_key(h, "peer"); nvs_commit(h); nvs_close(h); }
}

static void start_scan(void)
{
    ESP_LOGI(TAG, "scanning - put your headset in pairing mode");
    s_found = false;
    s_state = ST_SCANNING;
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
}

static void connect_peer(void)
{
    ESP_LOGI(TAG, "connecting to %02x:%02x:%02x:%02x:%02x:%02x",
             s_peer[0], s_peer[1], s_peer[2], s_peer[3], s_peer[4], s_peer[5]);
    s_state = ST_CONNECTING;
    esp_a2d_source_connect(s_peer);
}

/* esp_timer callback: reconnect or resume. Bluedroid APIs are thread-safe (they post to BTC). */
static void retry_cb(void *arg)
{
    LOCK();
    switch (s_state) {
    case ST_CONNECTED:  esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY); break;
    case ST_IDLE:
    case ST_CONNECTING: if (s_have_peer) connect_peer(); else start_scan(); break;
    default: break;
    }
    UNLOCK();
}
static void retry_in(int ms)
{
    esp_timer_stop(s_retry_timer);
    esp_timer_start_once(s_retry_timer, (uint64_t)ms * 1000);
}

/* ------------------------------------------------------------------ GAP */
static bool name_matches(const char *name)
{
    const char *want = CONFIG_LHP_HEADSET_NAME;
    if (!want[0]) return true;
    return name && strstr(name, want) != NULL;
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    LOCK();
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        if (s_state != ST_SCANNING || s_found) break;
        uint32_t cod = 0;
        char name[ESP_BT_GAP_MAX_BDNAME_LEN + 1] = {0};
        for (int i = 0; i < param->disc_res.num_prop; i++) {
            esp_bt_gap_dev_prop_t *p = &param->disc_res.prop[i];
            if (p->type == ESP_BT_GAP_DEV_PROP_COD) cod = *(uint32_t *)p->val;
            else if (p->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
                int n = p->len < ESP_BT_GAP_MAX_BDNAME_LEN ? p->len : ESP_BT_GAP_MAX_BDNAME_LEN;
                memcpy(name, p->val, n);
            } else if (p->type == ESP_BT_GAP_DEV_PROP_EIR && !name[0]) {
                uint8_t n = 0;
                uint8_t *s = esp_bt_gap_resolve_eir_data((uint8_t *)p->val, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &n);
                if (!s) s = esp_bt_gap_resolve_eir_data((uint8_t *)p->val, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &n);
                if (s) { if (n > ESP_BT_GAP_MAX_BDNAME_LEN) n = ESP_BT_GAP_MAX_BDNAME_LEN; memcpy(name, s, n); }
            }
        }
        if (!esp_bt_gap_is_valid_cod(cod) || esp_bt_gap_get_cod_major_dev(cod) != ESP_BT_COD_MAJOR_DEV_AV ||
            !(esp_bt_gap_get_cod_srvc(cod) & ESP_BT_COD_SRVC_RENDERING)) break;
        if (!name_matches(name)) { ESP_LOGI(TAG, "skipping '%s' (name filter)", name); break; }
        ESP_LOGI(TAG, "found headset '%s'", name);
        memcpy(s_peer, param->disc_res.bda, 6);
        s_found = true;
        esp_bt_gap_cancel_discovery();
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED && s_state == ST_SCANNING) {
            if (s_found) connect_peer();
            else esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
        }
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) ESP_LOGI(TAG, "paired with '%s'", param->auth_cmpl.device_name);
        else ESP_LOGW(TAG, "pairing failed (%d)", param->auth_cmpl.stat);
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {                /* legacy headsets: 0000 */
        esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
        break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT:                  /* SSP just-works */
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    default:
        break;
    }
    UNLOCK();
}

/* ------------------------------------------------------------------ A2DP events */
static void a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *p)
{
    LOCK();
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        if (p->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            if (s_has_forgotten && !memcmp(p->conn_stat.remote_bda, s_forgotten, 6) && !s_found) {
                /* a connection to the headset we were just told to forget completed */
                esp_a2d_source_disconnect(p->conn_stat.remote_bda);
                break;
            }
            esp_bt_gap_cancel_discovery();       /* no-op if not discovering */
            memcpy(s_peer, p->conn_stat.remote_bda, 6);
            s_have_peer = true;
            save_peer(s_peer);
            s_state = ST_CONNECTED;
            ESP_LOGI(TAG, "connected");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
        } else if (p->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            ESP_LOGI(TAG, "disconnected");
            s_rs_reset = true;
            if (s_state == ST_SCANNING) break;    /* forget in progress */
            s_state = s_have_peer ? ST_CONNECTING : ST_IDLE;
            retry_in(3000);
        }
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
        if (p->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            uint8_t sf = p->audio_cfg.mcc.cie.sbc_info.samp_freq;
            select_rate((sf & ESP_A2D_SBC_CIE_SF_48K) ? 48000 : (sf & ESP_A2D_SBC_CIE_SF_44K) ? 44100 :
                        (sf & ESP_A2D_SBC_CIE_SF_32K) ? 32000 : 16000);
            ESP_LOGI(TAG, "SBC configured at %" PRIu32 " Hz", (uint32_t)s_out_rate);
        }
        break;
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        if (p->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY) {
            if (p->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
            else retry_in(1000);
        } else if (p->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_START) {
            if (p->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) { s_state = ST_STREAMING; ESP_LOGI(TAG, "streaming"); }
            else retry_in(1000);
        }
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        if (p->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED) s_state = ST_STREAMING;
        else if (s_state == ST_STREAMING) {      /* headset suspended us: resume */
            s_state = ST_CONNECTED;
            s_rs_reset = true;
            retry_in(1000);
        }
        break;
    default:
        break;
    }
    UNLOCK();
}

/* ------------------------------------------------------------------ button + LED */
static void forget_headset(void)
{
    LOCK();
    ESP_LOGW(TAG, "forgetting headset");
    int n = esp_bt_gap_get_bond_device_num();
    if (n > 0) {
        esp_bd_addr_t *list = malloc(sizeof(esp_bd_addr_t) * n);
        if (list && esp_bt_gap_get_bond_device_list(&n, list) == ESP_OK)
            for (int i = 0; i < n; i++) esp_bt_gap_remove_bond_device(list[i]);
        free(list);
    }
    erase_peer();
    bool linked = s_state == ST_CONNECTED || s_state == ST_STREAMING || s_state == ST_CONNECTING;
    memcpy(s_forgotten, s_peer, 6);
    s_has_forgotten = true;
    s_have_peer = false;
    esp_timer_stop(s_retry_timer);
    if (linked) esp_a2d_source_disconnect(s_peer);   /* also aborts a pending connect */
    start_scan();
    UNLOCK();
}

static void ui_task(void *arg)
{
    gpio_config_t in = { .pin_bit_mask = 1ULL << CONFIG_LHP_BUTTON_GPIO, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&in);
#if CONFIG_LHP_LED_GPIO >= 0
    gpio_config_t out = { .pin_bit_mask = 1ULL << CONFIG_LHP_LED_GPIO, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&out);
#endif
    int64_t t_down = 0; bool down = false, fired = false;
    for (uint32_t tick = 0;; tick++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        bool p = gpio_get_level(CONFIG_LHP_BUTTON_GPIO) == 0;
        int64_t now = esp_timer_get_time() / 1000;
        if (p && !down) { down = true; fired = false; t_down = now; }
        else if (p && down && !fired && now - t_down >= 2000) { fired = true; forget_headset(); }
        else if (!p) down = false;
#if CONFIG_LHP_LED_GPIO >= 0
        int on;
        switch (s_state) {
        case ST_STREAMING: on = 1; break;                       /* solid      */
        case ST_CONNECTED: on = (now / 1000) & 1; break;         /* slow blink */
        default:           on = (now / 125) & 1; break;          /* fast blink */
        }
        gpio_set_level(CONFIG_LHP_LED_GPIO, on);
#endif
        if (tick % 250 == 0) {
            ESP_LOGI(TAG, "state %d, ring %" PRIu32 " frames, out %" PRIu32 " Hz",
                     (int)s_state, LD(s_rw) - LD(s_rr), (uint32_t)s_out_rate);
        }
    }
}

/* ------------------------------------------------------------------ main */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    i2s_init();
    xTaskCreatePinnedToCore(i2s_rx_task, "i2s_rx", 4096, NULL, 15, NULL, 1);

    s_lock = xSemaphoreCreateRecursiveMutex();
    select_rate(44100);                         /* Bluedroid's usual SBC rate; updated on AUDIO_CFG */
    const esp_timer_create_args_t ta = { .callback = retry_cb, .name = "bt_retry" };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_retry_timer));

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    esp_bluedroid_config_t bd_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bd_cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;          /* just-works pairing */
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_bt_pin_code_t pin = {0};
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, pin);

    esp_bt_gap_set_device_name(CONFIG_LHP_DEVICE_NAME);
    esp_bt_gap_register_callback(gap_cb);
    ESP_ERROR_CHECK(esp_a2d_source_init());
    esp_a2d_register_callback(a2d_cb);
    esp_a2d_source_register_data_callback(a2d_data_cb);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

    LOCK();
    if (load_peer(s_peer)) { s_have_peer = true; connect_peer(); }
    else start_scan();
    UNLOCK();

    xTaskCreatePinnedToCore(ui_task, "ui", 3072, NULL, 2, NULL, 0);
}
