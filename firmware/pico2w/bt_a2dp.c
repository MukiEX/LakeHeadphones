/*
 * bt_a2dp.c - A2DP source (SBC) for the Pico 2 W. Structure follows BTstack's
 * example/a2dp_source_demo.c (BSD-style BlueKitchen licence), trimmed to what
 * a headphone transmitter needs: auto-pair, auto-reconnect, and an AVRCP
 * target so the headset's buttons reach the PC as media keys.
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "btstack.h"
#include "btstack_tlv.h"
#include "classic/btstack_sbc.h"
#include "classic/btstack_sbc_bluedroid.h"
#include "bt_a2dp.h"
#include "resampler.h"

#define AUDIO_TIMEOUT_MS        5
#define INQUIRY_1280MS          4        /* one listening window = 5.1 s */
#define RECONNECT_DELAY_MS      3000
#define RESUME_DELAY_MS         1000
#define TAG_LAST_DEVICE         (((uint32_t)'L' << 24) | ((uint32_t)'H' << 16) | ((uint32_t)'P' << 8) | 'A')
#define SBC_STORAGE_SIZE        1030

/* ------------------------------------------------------------------ shared between the encoder (core 0) and BTstack (core 1) */
#define SBC_SLOTS   16
#define SBC_SLOT_SZ 256          /* 16 blocks, stereo (not joint), bitpool 53 = 224 bytes */
typedef struct { uint8_t data[SBC_SLOT_SZ]; uint16_t len; uint16_t samples; uint32_t gen; } sbc_slot_t;

static sbc_slot_t s_slot[SBC_SLOTS];
static volatile uint32_t s_slot_w, s_slot_r;

static struct {
    volatile uint32_t gen;               /* bumped by core 0 on new config */
    int rate, blocks, subbands, bitpool;
    btstack_sbc_allocation_method_t alloc;
    btstack_sbc_channel_mode_t mode;
} s_cfg;
static volatile uint32_t s_enc_gen;      /* config generation the encoder is using */
static volatile bool s_streaming;
static volatile bt_state_t s_state;
static volatile uint32_t s_chime_req;     /* bumped on each stream start: play the "connected" chime */

/* debug counters */
static volatile uint32_t d_enc, d_pkts, d_skip_cfg, d_skip_pcm, d_last_len, d_pad;

#define LD(x)     __atomic_load_n(&(x), __ATOMIC_ACQUIRE)
#define ST(x, v)  __atomic_store_n(&(x), (v), __ATOMIC_RELEASE)

/* ------------------------------------------------------------------ core 0 state */
static uint8_t sdp_a2dp_source_service_buffer[150];
static uint8_t sdp_avrcp_target_service_buffer[150];
static uint8_t media_sbc_codec_capabilities[] = {
    (AVDTP_SBC_44100 << 4) | (AVDTP_SBC_48000 << 4) | AVDTP_SBC_STEREO | AVDTP_SBC_JOINT_STEREO,
    0xFF,      /* all block lengths, subbands, allocation methods */
    2, 53
};
static uint8_t media_sbc_codec_configuration[4];
static btstack_packet_callback_registration_t hci_event_callback_registration;

static struct {
    uint16_t a2dp_cid;
    uint16_t avrcp_cid;
    bool     playing;         /* what we report to the headset (our guess of the PC's state) */
    uint8_t  local_seid;
    bd_addr_t addr;
    bool     have_addr;
    bool     connecting_stored;
    btstack_timer_source_t audio_timer, retry_timer;
    /* pairing: best candidate seen during the current inquiry window */
    bool     cand_valid;
    bd_addr_t cand_addr;
    int      cand_score;
    uint32_t cand_cod;
    int8_t   cand_rssi;
    char     cand_name[33];
    bool     chimed;          /* chime played for this connection */
    uint32_t time_audio_data_sent;
    uint32_t acc_num_missed_samples;
    uint32_t samples_ready;
    int      max_media_payload_size;
    uint32_t rtp_timestamp;
    uint8_t  sbc_storage[SBC_STORAGE_SIZE];
    uint16_t sbc_storage_count;
    uint8_t  sbc_frames;
    bool     sbc_ready_to_send;
} m;

static void start_scan(void);
static void connect_stored(void);

bt_state_t bt_a2dp_state(void) { return s_state; }
bool bt_a2dp_streaming(void)    { return s_streaming; }

/* ------------------------------------------------------------------ TLV */
static bool load_device(bd_addr_t a)
{
    const btstack_tlv_t *tlv; void *ctx;
    btstack_tlv_get_instance(&tlv, &ctx);
    return tlv && tlv->get_tag(ctx, TAG_LAST_DEVICE, a, 6) == 6;
}
static void store_device(const bd_addr_t a)
{
    /* Flash writes pause both cores briefly: only write when it changed. */
    bd_addr_t old;
    if (load_device(old) && memcmp(old, a, 6) == 0) return;
    const btstack_tlv_t *tlv; void *ctx;
    btstack_tlv_get_instance(&tlv, &ctx);
    if (tlv) tlv->store_tag(ctx, TAG_LAST_DEVICE, a, 6);
}
static void delete_device(void)
{
    const btstack_tlv_t *tlv; void *ctx;
    btstack_tlv_get_instance(&tlv, &ctx);
    if (tlv) tlv->delete_tag(ctx, TAG_LAST_DEVICE);
}

/* ------------------------------------------------------------------ media (core 0) */
static void send_media_packet(void)
{
    if (!m.sbc_frames) { m.sbc_ready_to_send = false; return; }
    m.sbc_storage[0] = m.sbc_frames;           /* SBC media payload header */
    d_pkts++;
    a2dp_source_stream_send_media_payload_rtp(m.a2dp_cid, m.local_seid, 0, m.rtp_timestamp,
                                              m.sbc_storage, m.sbc_storage_count + 1);
    m.sbc_storage_count = 0;
    m.sbc_frames = 0;
    m.sbc_ready_to_send = false;
}

static void audio_timeout_handler(btstack_timer_source_t *timer)
{
    btstack_run_loop_set_timer(timer, AUDIO_TIMEOUT_MS);
    btstack_run_loop_add_timer(timer);
    uint32_t now = btstack_run_loop_get_time_ms();
    uint32_t period = m.time_audio_data_sent ? now - m.time_audio_data_sent : AUDIO_TIMEOUT_MS;
    uint32_t rate = (uint32_t)s_cfg.rate;
    uint32_t n = (period * rate) / 1000;
    m.acc_num_missed_samples += (period * rate) % 1000;
    while (m.acc_num_missed_samples >= 1000) { n++; m.acc_num_missed_samples -= 1000; }
    m.time_audio_data_sent = now;
    m.samples_ready += n;
    if (m.samples_ready > rate / 10) m.samples_ready = rate / 10;   /* cap catch-up after stalls */
    if (m.sbc_ready_to_send) return;

    const uint32_t gen = LD(s_cfg.gen);
    for (;;) {
        uint32_t r = s_slot_r;
        if (LD(s_slot_w) == r) break;                      /* core 1 behind */
        sbc_slot_t *s = &s_slot[r % SBC_SLOTS];
        if (s->gen != gen) { ST(s_slot_r, r + 1); continue; } /* stale config */
        if (m.samples_ready < s->samples) break;
        if (m.sbc_storage_count + s->len > m.max_media_payload_size - 1) {
            m.sbc_ready_to_send = true;
            a2dp_source_stream_endpoint_request_can_send_now(m.a2dp_cid, m.local_seid);
            return;
        }
        memcpy(&m.sbc_storage[1 + m.sbc_storage_count], s->data, s->len);
        m.sbc_storage_count += s->len;
        m.sbc_frames++;
        m.samples_ready -= s->samples;
        m.rtp_timestamp += s->samples;
        ST(s_slot_r, r + 1);
        if (m.sbc_frames == 15) {            /* 4-bit frame counter in SBC header */
            m.sbc_ready_to_send = true;
            a2dp_source_stream_endpoint_request_can_send_now(m.a2dp_cid, m.local_seid);
            return;
        }
    }
}

static void timer_start(void)
{
    m.max_media_payload_size = btstack_min(a2dp_max_media_payload_size(m.a2dp_cid, m.local_seid), SBC_STORAGE_SIZE);
    m.sbc_storage_count = 0; m.sbc_frames = 0; m.sbc_ready_to_send = false;
    m.time_audio_data_sent = 0; m.acc_num_missed_samples = 0; m.samples_ready = 0;
    btstack_run_loop_remove_timer(&m.audio_timer);
    btstack_run_loop_set_timer_handler(&m.audio_timer, audio_timeout_handler);
    btstack_run_loop_set_timer(&m.audio_timer, AUDIO_TIMEOUT_MS);
    btstack_run_loop_add_timer(&m.audio_timer);
    ST(s_streaming, true);
}

static void timer_stop(void)
{
    btstack_run_loop_remove_timer(&m.audio_timer);
    ST(s_streaming, false);
}

/* ------------------------------------------------------------------ connection mgmt */
static void retry_handler(btstack_timer_source_t *t)
{
    (void)t;
    if (s_state == BT_STREAMING || s_state == BT_CONNECTED) {
        if (!s_streaming && m.a2dp_cid) a2dp_source_start_stream(m.a2dp_cid, m.local_seid); /* resume */
        return;
    }
    if (m.have_addr) connect_stored(); else start_scan();
}
static void schedule_retry(uint32_t ms)
{
    btstack_run_loop_remove_timer(&m.retry_timer);
    btstack_run_loop_set_timer_handler(&m.retry_timer, retry_handler);
    btstack_run_loop_set_timer(&m.retry_timer, ms);
    btstack_run_loop_add_timer(&m.retry_timer);
}

static void start_scan(void)
{
    printf("BT: pairing mode - looking for headsets (put yours in pairing mode)\n");
    s_state = BT_SCANNING;
    m.cand_valid = false;
    gap_inquiry_start(INQUIRY_1280MS);
}

/* Pairing picks one device per 5 s listening window instead of grabbing the
 * first audio device that answers: headphones and headsets beat speakers,
 * TVs and car kits, and between equals the strongest signal (the one next to
 * the Pico) wins. Other people's headsets are normally not discoverable unless
 * they are in pairing mode, so in practice this finds yours. */
static int cod_score(uint32_t cod)
{
    if (!(cod & 0x200000) || ((cod >> 8) & 0x1f) != 0x04) return -1;   /* Audio service + Audio/Video */
    switch ((cod >> 2) & 0x3f) {                                          /* minor class */
    case 0x01: case 0x06: return 3;      /* wearable headset, headphones */
    case 0x07: return 2;                 /* portable audio */
    case 0x05: case 0x0a: return 1;      /* loudspeaker, HiFi */
    case 0x02: case 0x08: return 0;      /* hands-free, car audio */
    default:   return 1;
    }
}

static void connect_candidate(void)
{
    printf("BT: pairing with %s \"%s\" (CoD %06" PRIx32 ", RSSI %d dBm)\n",
           bd_addr_to_str(m.cand_addr), m.cand_name[0] ? m.cand_name : "?", m.cand_cod, m.cand_rssi);
    memcpy(m.addr, m.cand_addr, 6);
    s_state = BT_CONNECTING;
    m.connecting_stored = false;
    if (a2dp_source_establish_stream(m.addr, &m.a2dp_cid) != ERROR_CODE_SUCCESS) {
        printf("BT: could not start connection, rescanning\n");
        start_scan();
    }
}

static void connect_stored(void)
{
    printf("BT: reconnecting to %s\n", bd_addr_to_str(m.addr));
    s_state = BT_CONNECTING;
    m.connecting_stored = true;
    if (a2dp_source_establish_stream(m.addr, &m.a2dp_cid) != ERROR_CODE_SUCCESS) schedule_retry(RECONNECT_DELAY_MS);
}

static void forget_on_bt_thread(void *ctx)
{
    (void)ctx;
    printf("BT: forgetting paired headset\n");
    gap_delete_all_link_keys();
    delete_device();
    m.have_addr = false;
    btstack_run_loop_remove_timer(&m.retry_timer);
    if (m.a2dp_cid) a2dp_source_disconnect(m.a2dp_cid);   /* release event will rescan */
    else { gap_inquiry_stop(); start_scan(); }
}

void bt_a2dp_forget_and_scan(void)
{
    static btstack_context_callback_registration_t reg;
    reg.callback = forget_on_bt_thread;
    reg.context = NULL;
    btstack_run_loop_execute_on_main_thread(&reg);
}

/* ------------------------------------------------------------------ HCI events */
static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel; (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    bd_addr_t address;
    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
        gap_local_bd_addr(address);
        printf("BT: up, local address %s\n", bd_addr_to_str(address));
        if (load_device(m.addr)) { m.have_addr = true; connect_stored(); }
        else start_scan();
        break;
    case HCI_EVENT_PIN_CODE_REQUEST:              /* legacy headsets: try 0000 */
        hci_event_pin_code_request_get_bd_addr(packet, address);
        gap_pin_code_response(address, "0000");
        break;
    case HCI_EVENT_USER_CONFIRMATION_REQUEST:     /* SSP just-works */
        hci_event_user_confirmation_request_get_bd_addr(packet, address);
        gap_ssp_confirmation_response(address);
        break;
    case GAP_EVENT_INQUIRY_RESULT: {
        if (s_state != BT_SCANNING) break;
        uint32_t cod = gap_event_inquiry_result_get_class_of_device(packet);
        gap_event_inquiry_result_get_bd_addr(packet, address);
        int cls = cod_score(cod);
        if (cls < 0) break;
        int8_t rssi = gap_event_inquiry_result_get_rssi_available(packet) ? gap_event_inquiry_result_get_rssi(packet) : -100;
        int score = cls * 1000 + (rssi + 128);
        char name[33] = "";
        if (gap_event_inquiry_result_get_name_available(packet)) {
            int n = gap_event_inquiry_result_get_name_len(packet);
            if (n > 32) n = 32;
            memcpy(name, gap_event_inquiry_result_get_name(packet), n);
            name[n] = 0;
        }
        printf("BT:   seen %s \"%s\" CoD %06" PRIx32 " RSSI %d\n", bd_addr_to_str(address), name, cod, rssi);
        if (!m.cand_valid || score > m.cand_score ||
            (memcmp(address, m.cand_addr, 6) == 0 && name[0] && !m.cand_name[0])) {
            m.cand_valid = true; m.cand_score = score; m.cand_cod = cod; m.cand_rssi = rssi;
            memcpy(m.cand_addr, address, 6);
            memcpy(m.cand_name, name, sizeof name);
        }
        break;
    }
    case GAP_EVENT_INQUIRY_COMPLETE:
        if (s_state != BT_SCANNING) break;
        if (m.cand_valid) connect_candidate();
        else gap_inquiry_start(INQUIRY_1280MS);   /* nothing yet: keep listening */
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------ A2DP events */
static void a2dp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel; (void)size;
    if (packet_type != HCI_EVENT_PACKET || hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) return;
    uint8_t status;
    bd_addr_t address;

    switch (hci_event_a2dp_meta_get_subevent_code(packet)) {
    case A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED:
        a2dp_subevent_signaling_connection_established_get_bd_addr(packet, address);
        status = a2dp_subevent_signaling_connection_established_get_status(packet);
        if (status != ERROR_CODE_SUCCESS) {
            printf("BT: connection failed (0x%02x)\n", status);
            m.a2dp_cid = 0;
            if (m.connecting_stored) schedule_retry(RECONNECT_DELAY_MS);   /* headset off? keep trying */
            else start_scan();
            break;
        }
        m.a2dp_cid = a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
        m.chimed = false;
        memcpy(m.addr, address, 6);
        m.have_addr = true;
        store_device(address);
        s_state = BT_CONNECTED;
        printf("BT: connected to %s\n", bd_addr_to_str(address));
        if (!m.avrcp_cid) avrcp_connect(address, &m.avrcp_cid);   /* for the headset's buttons; it may connect first */
        break;

    case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION: {
        avdtp_channel_mode_t cm = (avdtp_channel_mode_t)a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet);
        s_cfg.rate     = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
        s_cfg.blocks   = a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet);
        s_cfg.subbands = a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet);
        s_cfg.bitpool  = a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet);
        s_cfg.alloc    = (btstack_sbc_allocation_method_t)(a2dp_subevent_signaling_media_codec_sbc_configuration_get_allocation_method(packet) - 1);
        switch (cm) {
        case AVDTP_CHANNEL_MODE_JOINT_STEREO: s_cfg.mode = SBC_CHANNEL_MODE_JOINT_STEREO; break;
        case AVDTP_CHANNEL_MODE_STEREO:       s_cfg.mode = SBC_CHANNEL_MODE_STEREO; break;
        case AVDTP_CHANNEL_MODE_DUAL_CHANNEL: s_cfg.mode = SBC_CHANNEL_MODE_DUAL_CHANNEL; break;
        default:                              s_cfg.mode = SBC_CHANNEL_MODE_MONO; break;
        }
        ST(s_cfg.gen, s_cfg.gen + 1);
        printf("BT: SBC %d Hz, %d blocks, %d subbands, bitpool %d, mode %d\n",
               s_cfg.rate, s_cfg.blocks, s_cfg.subbands, s_cfg.bitpool, (int)s_cfg.mode);
        break;
    }

    case A2DP_SUBEVENT_STREAM_ESTABLISHED:
        status = a2dp_subevent_stream_established_get_status(packet);
        if (status != ERROR_CODE_SUCCESS) { printf("BT: stream failed 0x%02x\n", status); break; }
        a2dp_source_start_stream(m.a2dp_cid, m.local_seid);
        break;

    case A2DP_SUBEVENT_STREAM_RECONFIGURED:
        if (a2dp_subevent_stream_reconfigured_get_status(packet) == ERROR_CODE_SUCCESS)
            a2dp_source_start_stream(m.a2dp_cid, m.local_seid);
        break;

    case A2DP_SUBEVENT_STREAM_STARTED:
        if (s_state != BT_STREAMING && !m.chimed) { m.chimed = true; ST(s_chime_req, s_chime_req + 1); }
        timer_start();
        s_state = BT_STREAMING;
        printf("BT: streaming\n");
        break;

    case A2DP_SUBEVENT_STREAMING_CAN_SEND_MEDIA_PACKET_NOW:
        send_media_packet();
        break;

    case A2DP_SUBEVENT_STREAM_SUSPENDED:           /* sink paused us: ask to resume */
        timer_stop();
        s_state = BT_CONNECTED;
        schedule_retry(RESUME_DELAY_MS);
        break;

    case A2DP_SUBEVENT_STREAM_RELEASED:
        /* The sink closed the stream (idle timeout, source switch...). BTstack
         * can't simply restart a released endpoint, so drop the link; the
         * SIGNALING_CONNECTION_RELEASED path reconnects and renegotiates. */
        timer_stop();
        s_state = BT_CONNECTED;
        if (m.a2dp_cid) a2dp_source_disconnect(m.a2dp_cid);
        break;

    case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
        timer_stop();
        m.a2dp_cid = 0;
        if (m.avrcp_cid) avrcp_disconnect(m.avrcp_cid);
        printf("BT: disconnected\n");
        if (m.have_addr) { s_state = BT_CONNECTING; schedule_retry(RECONNECT_DELAY_MS); }
        else start_scan();
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------ AVRCP: headset buttons */
__attribute__((weak)) void bt_a2dp_media_key_cb(uint16_t usage, bool pressed) { (void)usage; (void)pressed; }

static uint16_t avrcp_to_usage(avrcp_operation_id_t op)
{
    switch (op) {
    case AVRCP_OPERATION_ID_PLAY:
    case AVRCP_OPERATION_ID_PAUSE:        return 0xCD;   /* Play/Pause: the key Windows and apps act on */
    case AVRCP_OPERATION_ID_STOP:         return 0xB7;
    case AVRCP_OPERATION_ID_FORWARD:      return 0xB5;   /* next track */
    case AVRCP_OPERATION_ID_BACKWARD:     return 0xB6;   /* previous track */
    case AVRCP_OPERATION_ID_FAST_FORWARD: return 0xB3;
    case AVRCP_OPERATION_ID_REWIND:       return 0xB4;
    case AVRCP_OPERATION_ID_VOLUME_UP:    return 0xE9;
    case AVRCP_OPERATION_ID_VOLUME_DOWN:  return 0xEA;
    case AVRCP_OPERATION_ID_MUTE:         return 0xE2;
    default:                              return 0;
    }
}

static void avrcp_report_status(void)
{
    if (m.avrcp_cid)
        avrcp_target_set_playback_status(m.avrcp_cid, m.playing ? AVRCP_PLAYBACK_STATUS_PLAYING : AVRCP_PLAYBACK_STATUS_PAUSED);
}

static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel; (void)size;
    if (packet_type != HCI_EVENT_PACKET || hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;
    switch (packet[2]) {
    case AVRCP_SUBEVENT_CONNECTION_ESTABLISHED: {
        const uint16_t cid = avrcp_subevent_connection_established_get_avrcp_cid(packet);
        if (avrcp_subevent_connection_established_get_status(packet) != ERROR_CODE_SUCCESS) {
            if (m.avrcp_cid == cid) m.avrcp_cid = 0;
            printf("BT: AVRCP connection failed\n");
            break;
        }
        m.avrcp_cid = cid;
        m.playing = true;
        avrcp_target_support_event(cid, AVRCP_NOTIFICATION_EVENT_PLAYBACK_STATUS_CHANGED);
        avrcp_report_status();
        printf("BT: AVRCP connected (headset buttons -> media keys)\n");
        break;
    }
    case AVRCP_SUBEVENT_CONNECTION_RELEASED:
        if (avrcp_subevent_connection_released_get_avrcp_cid(packet) == m.avrcp_cid) m.avrcp_cid = 0;
        break;
    default:
        break;
    }
}

static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel; (void)size;
    if (packet_type != HCI_EVENT_PACKET || hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;
    switch (packet[2]) {
    case AVRCP_SUBEVENT_PLAY_STATUS_QUERY:
        avrcp_target_play_status(m.avrcp_cid, 0xFFFFFFFF, 0xFFFFFFFF,
                                 m.playing ? AVRCP_PLAYBACK_STATUS_PLAYING : AVRCP_PLAYBACK_STATUS_PAUSED);
        break;
    case AVRCP_SUBEVENT_OPERATION: {
        const avrcp_operation_id_t op = (avrcp_operation_id_t)avrcp_subevent_operation_get_operation_id(packet);
        const bool pressed = avrcp_subevent_operation_get_button_pressed(packet) > 0;
        const uint16_t usage = avrcp_to_usage(op);
        printf("BT: headset button %s %s\n", avrcp_operation2str(op), pressed ? "down" : "up");
        if (!usage) break;
        bt_a2dp_media_key_cb(usage, pressed);
        if (pressed && (op == AVRCP_OPERATION_ID_PLAY || op == AVRCP_OPERATION_ID_PAUSE || op == AVRCP_OPERATION_ID_STOP)) {
            m.playing = op == AVRCP_OPERATION_ID_PLAY;     /* the headset picks play or pause from this */
            avrcp_report_status();
        }
        break;
    }
    default:
        break;
    }
}

/* ------------------------------------------------------------------ init */
void bt_a2dp_init(void)
{
    s_cfg.rate = 48000;
    hci_set_master_slave_policy(0);
    hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);
    l2cap_init();

    a2dp_source_init();
    a2dp_source_register_packet_handler(&a2dp_packet_handler);
    avdtp_stream_endpoint_t *ep = a2dp_source_create_stream_endpoint(AVDTP_AUDIO, AVDTP_CODEC_SBC,
        media_sbc_codec_capabilities, sizeof media_sbc_codec_capabilities,
        media_sbc_codec_configuration, sizeof media_sbc_codec_configuration);
    if (!ep) { printf("BT: no memory for stream endpoint\n"); return; }
    avdtp_set_preferred_sampling_frequency(ep, 48000);     /* matches the DSP, no resampling */
    m.local_seid = avdtp_local_seid(ep);

    sdp_init();
    memset(sdp_a2dp_source_service_buffer, 0, sizeof sdp_a2dp_source_service_buffer);
    a2dp_source_create_sdp_record(sdp_a2dp_source_service_buffer, sdp_create_service_record_handle(),
                                  AVDTP_SOURCE_FEATURE_MASK_PLAYER, NULL, NULL);
    sdp_register_service(sdp_a2dp_source_service_buffer);

    /* AVRCP target (category 1, player): the headset sends play/pause, next,
     * previous... to us. No controller record, so the headset keeps its own
     * volume control instead of switching to absolute volume. */
    avrcp_init();
    avrcp_register_packet_handler(&avrcp_packet_handler);
    avrcp_target_init();
    avrcp_target_register_packet_handler(&avrcp_target_packet_handler);
    avrcp_controller_init();                 /* needed for the shared AVRCP connection */
    memset(sdp_avrcp_target_service_buffer, 0, sizeof sdp_avrcp_target_service_buffer);
    avrcp_target_create_sdp_record(sdp_avrcp_target_service_buffer, sdp_create_service_record_handle(),
                                   AVRCP_FEATURE_MASK_CATEGORY_PLAYER_OR_RECORDER, NULL, NULL);
    sdp_register_service(sdp_avrcp_target_service_buffer);

    gap_set_local_name("LakeHP 5.1 00:00:00:00:00:00");
    gap_discoverable_control(1);
    gap_set_class_of_device(0x200408);

    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    hci_power_control(HCI_POWER_ON);
}

/* ------------------------------------------------------------------ encoder (core 0 main loop) */
static btstack_sbc_encoder_bluedroid_t s_enc_state;
static const btstack_sbc_encoder_t *s_enc;
static resampler_t s_rs;
static int s_rs_active;
static int16_t s_rs_in[256 * 2];
static int s_rs_in_n;

/* fetch n stereo frames at the SBC rate; silence-padded if `force` */
static bool get_pcm(dhp_t *p, int16_t *pcm, int n, bool force)
{
    if (!s_rs_active) {
        if (!force && dhp_out_available(p) < (uint32_t)n) return false;
        dhp_read_stereo(p, pcm, (uint32_t)n);
        return true;
    }
    /* 48k -> 44.1k fallback for sinks that refuse 48 kHz */
    if (!force && dhp_out_available(p) + (uint32_t)s_rs_in_n < (uint32_t)(n * 48000 / 44100 + 40)) return false;
    int made = 0;
    while (made < n) {
        if (s_rs_in_n == 0) { dhp_read_stereo(p, s_rs_in, 64); s_rs_in_n = 64; }
        int used;
        made += rs_process(&s_rs, s_rs_in + 2 * (64 - s_rs_in_n), s_rs_in_n, &used, pcm + 2 * made, n - made);
        s_rs_in_n -= used;
    }
    return true;
}

/* Two rising tones when a headset connects, so you hear that it worked
 * (and which side of a multi-device setup the Pico grabbed). Mixed in at the
 * encoder, so it plays even when the PC is silent. */
static void add_chime(int16_t *pcm, int n)
{
    static uint32_t seen, pos, total;
    static float phase;
    const uint32_t req = LD(s_chime_req);
    if (req != seen) { seen = req; pos = 0; total = (uint32_t)s_cfg.rate * 3 / 10; phase = 0; }
    if (pos >= total) return;
    const uint32_t half = total / 2, fade = (uint32_t)s_cfg.rate / 200;
    for (int i = 0; i < n && pos < total; i++, pos++) {
        const uint32_t t = pos % half;
        const float f = pos < half ? 784.0f : 1175.0f;            /* G5 then D6 */
        float env = 1.0f;
        if (t < fade) env = (float)t / fade;
        else if (half - t < fade) env = (float)(half - t) / fade;
        phase += 6.2831853f * f / (float)s_cfg.rate;
        if (phase > 6.2831853f) phase -= 6.2831853f;
        const int v = (int)(0.15f * 32767.0f * env * sinf(phase));
        for (int c = 0; c < 2; c++) {
            int x = pcm[2 * i + c] + v;
            pcm[2 * i + c] = (int16_t)(x > 32767 ? 32767 : x < -32768 ? -32768 : x);
        }
    }
}

bool bt_a2dp_encoder_service(dhp_t *p)
{
    uint32_t gen = LD(s_cfg.gen);
    if (gen != s_enc_gen) {
        s_enc = btstack_sbc_encoder_bluedroid_init_instance(&s_enc_state);
        s_enc->configure(&s_enc_state, SBC_MODE_STANDARD, (uint8_t)s_cfg.blocks, (uint8_t)s_cfg.subbands,
                         s_cfg.alloc, (uint16_t)s_cfg.rate, (uint8_t)s_cfg.bitpool, s_cfg.mode);
        s_rs_active = (s_cfg.rate != 48000);
        if (s_rs_active) rs_init(&s_rs, 48000, s_cfg.rate);
        s_rs_in_n = 0;
        ST(s_enc_gen, gen);
    }
    if (!s_enc || !LD(s_streaming)) return false;

    const uint32_t level = LD(s_slot_w) - LD(s_slot_r);
    const bool ll = dhp_low_latency(p);
    if (level >= (ll ? 3u : SBC_SLOTS / 2)) return false;   /* ~8 ms (low latency) or ~21 ms queued */
    const int n = s_enc->num_audio_frames(&s_enc_state);
    int16_t pcm[256 * 2];
    if (n > 256 || s_enc->sbc_buffer_length(&s_enc_state) > SBC_SLOT_SZ) { d_skip_cfg++; return false; }
    const bool force = level < (ll ? 1u : 3u);
    if (!force && dhp_out_available(p) < (uint32_t)n) { d_skip_pcm++; }
    if (!get_pcm(p, pcm, n, force)) return false;  /* starving: pad with silence */
    if (force) d_pad++;

    add_chime(pcm, n);

    sbc_slot_t *s = &s_slot[s_slot_w % SBC_SLOTS];
    s_enc->encode_signed_16(&s_enc_state, pcm, s->data);
    s->len = s_enc->sbc_buffer_length(&s_enc_state);
    s->samples = (uint16_t)n;
    s->gen = gen;
    d_enc++; d_last_len = s->len;
    ST(s_slot_w, s_slot_w + 1);
    return true;
}

/* ------------------------------------------------------------------ debug */
void bt_a2dp_debug_print(void (*out)(const char *fmt, ...))
{
    out("  bt: state %d streaming %d | sbc %d Hz blk %d sb %d bp %d mode %d cfg-gen %u/%u | "
           "frames enc %u (len %u, padded %u, cfg-skip %u, pcm-wait %u) | packets %u, max payload %d, slots %u\n",
           (int)s_state, (int)s_streaming, s_cfg.rate, s_cfg.blocks, s_cfg.subbands, s_cfg.bitpool, (int)s_cfg.mode,
           (unsigned)s_cfg.gen, (unsigned)s_enc_gen, (unsigned)d_enc, (unsigned)d_last_len, (unsigned)d_pad,
           (unsigned)d_skip_cfg, (unsigned)d_skip_pcm, (unsigned)d_pkts, m.max_media_payload_size, (unsigned)(s_slot_w - s_slot_r));
}

void bt_a2dp_peer_info(char addr[20], int *rate, int *bitpool)
{
    if (s_state >= BT_CONNECTED) snprintf(addr, 20, "%s", bd_addr_to_str(m.addr));
    else addr[0] = 0;
    *rate = s_state >= BT_CONNECTED ? s_cfg.rate : 0;
    *bitpool = s_cfg.bitpool;
}
