/*
 * usbout - USB audio output on the PIO-USB host port (core 1). See usbout.h.
 */
#include "usbout.h"
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/sync.h"
#include "tusb.h"
#include "pio_usb.h"
#include "resampler.h"
uint32_t pio_usb_host_get_frame_number(void);
#if LHP_DEBUG_CDC
#include "dbg_cdc.h"
#define LOG1(...) dbg_logf(__VA_ARGS__)
#else
#define LOG1(...) do { } while (0)
#endif

#ifndef USBOUT_PIO_USB_DP_PIN
#ifdef PICO_DEFAULT_PIO_USB_DP_PIN
#define USBOUT_PIO_USB_DP_PIN PICO_DEFAULT_PIO_USB_DP_PIN   /* RP2350-USB-A: GP12/GP13 */
#else
#define USBOUT_PIO_USB_DP_PIN 0
#endif
#endif

/* Playback FIFO target: enough that a full packet is always queued, no more. */
#define FIFO_TARGET_MS   4
#define MOUNT_DELAY_MS   100
#define RETRY_DELAY_MS   500

static dhp_t *s_pipe;
static volatile usbout_state_t s_state;
static volatile bool s_restart_req;

/* info for core 0 (settings page): written under a seqlock */
static usbout_info_t s_info;
static volatile uint32_t s_info_seq;

static uint8_t s_idx = TUSB_INDEX_INVALID_8, s_stream = TUSB_INDEX_INVALID_8, s_cfg_i;
static tuh_audio_stream_config_t s_cfg;
static uint32_t s_frame_bytes, s_fifo_frames;
static uint32_t s_mount_at, s_retry_at;          /* ms, 0 = none */
static uint8_t  s_mount_idx = TUSB_INDEX_INVALID_8;

static resampler_t s_rs;
static bool s_rs_on;

/* ------------------------------------------------------------------ info */
static void info_begin(void) { s_info_seq++; __dmb(); }
static void info_end(void)   { __dmb(); s_info_seq++; }
static void set_state(usbout_state_t st)
{
    s_state = st;
    info_begin(); s_info.state = st; info_end();
}
usbout_state_t usbout_state(void) { return s_state; }
void usbout_get_info(usbout_info_t *out)
{
    for (int tries = 0; tries < 8; tries++) {
        const uint32_t a = s_info_seq;
        __dmb();
        *out = s_info;
        __dmb();
        if (!(a & 1) && a == s_info_seq) return;
    }
}
void usbout_restart(void) { s_restart_req = true; }
static volatile bool s_tone;
void usbout_set_tone(bool on) { s_tone = on; }
bool usbout_tone(void) { return s_tone; }

/* 1 kHz sine at -12 dBFS, L only then R only every second: tests the output
 * path alone (the pipeline is still drained so the PC side keeps its pace). */
static void tone_fill(int16_t *lr, uint32_t n, uint32_t rate)
{
    static uint32_t ph, t;
    static int16_t tab[48];
    if (!tab[12]) for (int i = 0; i < 48; i++) {
        /* one cycle of a sine in 48 points (quarter wave below, -12 dBFS) */
        static const int16_t q[13] = {0, 1070, 2122, 3135, 4096, 4985, 5793, 6503, 7094, 7568, 7912, 8122, 8192};
        const int k = i % 24, v = k <= 12 ? q[k] : q[24 - k];
        tab[i] = (int16_t)(i < 24 ? v : -v);
    }
    for (uint32_t i = 0; i < n; i++) {
        const int16_t v = tab[ph / rate];              /* ph in table-index units x rate */
        ph = (ph + 48u * 1000u) % (48u * rate);
        const bool left = (t / rate) % 2 == 0;
        lr[2 * i] = left ? v : 0;
        lr[2 * i + 1] = left ? 0 : v;
        t++;
    }
}

/* ------------------------------------------------------------------ format conversion */
static uint8_t fmt_bytes(tuh_audio_format_t f)
{
    switch (f) {
    case TUH_AUDIO_FORMAT_S8: return 1;
    case TUH_AUDIO_FORMAT_S16_LE: return 2;
    case TUH_AUDIO_FORMAT_S24_3LE: return 3;
    case TUH_AUDIO_FORMAT_S24_LE: case TUH_AUDIO_FORMAT_S32_LE: return 4;
    default: return 0;
    }
}
static uint8_t fmt_bits(tuh_audio_format_t f)
{
    switch (f) {
    case TUH_AUDIO_FORMAT_S8: return 8;
    case TUH_AUDIO_FORMAT_S16_LE: return 16;
    case TUH_AUDIO_FORMAT_S24_3LE: case TUH_AUDIO_FORMAT_S24_LE: return 24;
    case TUH_AUDIO_FORMAT_S32_LE: return 32;
    default: return 0;
    }
}

/* stereo int16 -> the device's frame layout (L, R, then silent channels) */
static uint32_t convert(const int16_t *lr, uint32_t n, uint8_t *dst)
{
    const uint8_t ch = s_cfg.channels;
    uint8_t *d = dst;
    for (uint32_t i = 0; i < n; i++) {
        const int32_t l = lr[2 * i], r = lr[2 * i + 1];
        for (uint8_t c = 0; c < ch; c++) {
            const int32_t v = ch == 1 ? (l + r) / 2 : c == 0 ? l : c == 1 ? r : 0;
            switch (s_cfg.format) {
            case TUH_AUDIO_FORMAT_S8:      *d++ = (uint8_t)(v >> 8); break;
            case TUH_AUDIO_FORMAT_S16_LE:  *d++ = (uint8_t)v; *d++ = (uint8_t)(v >> 8); break;
            case TUH_AUDIO_FORMAT_S24_3LE: *d++ = 0; *d++ = (uint8_t)v; *d++ = (uint8_t)(v >> 8); break;
            case TUH_AUDIO_FORMAT_S24_LE: { const uint32_t w = (uint32_t)v << 8;
                *d++ = (uint8_t)w; *d++ = (uint8_t)(w >> 8); *d++ = (uint8_t)(w >> 16); *d++ = (uint8_t)(w >> 24); break; }
            case TUH_AUDIO_FORMAT_S32_LE: { const uint32_t w = (uint32_t)v << 16;
                *d++ = (uint8_t)w; *d++ = (uint8_t)(w >> 8); *d++ = (uint8_t)(w >> 16); *d++ = (uint8_t)(w >> 24); break; }
            default: break;
            }
        }
    }
    return (uint32_t)(d - dst);
}

/* ------------------------------------------------------------------ configuration choice */
static int config_score(const tuh_audio_stream_config_t *c)
{
    if (!fmt_bytes(c->format) || !c->channels) return -1;
    if ((uint32_t)c->channels * fmt_bytes(c->format) * (c->sample_rate / 1000 + 1) > CFG_TUH_AUDIO_EPOUT_BUFSIZE) return -1;
    int s;
    if (c->sample_rate == 48000) s = 1000;
    else if (c->sample_rate == 44100) s = 500;           /* through the resampler */
    else return -1;
    s += c->channels == 2 ? 100 : c->channels > 2 ? 60 : 30;
    switch (c->format) {
    case TUH_AUDIO_FORMAT_S16_LE: s += 10; break;        /* least bus time, plenty for 16-bit input */
    case TUH_AUDIO_FORMAT_S24_3LE: s += 8; break;
    case TUH_AUDIO_FORMAT_S24_LE: s += 7; break;
    case TUH_AUDIO_FORMAT_S32_LE: s += 6; break;
    default: s += 1; break;
    }
    return s;
}

static void setup_controls(uint8_t idx, uint8_t st)
{
    if (tuh_audio_mute_supported(idx, st)) {
        if (tuh_audio_mute_set_sync(idx, st, false) != XFER_RESULT_SUCCESS) LOG1("usbout: unmute failed\n");
    }
    tuh_audio_volume_range_t range;
    if (tuh_audio_volume_range_get(idx, st, &range)) {
        /* The PC's volume is applied before us; the device stays at 0 dB
         * (its own hardware volume wheel, if any, still works). */
        int32_t target = 0;
        if (target > range.max) target = range.max;
        if (target < range.min) target = range.min;
        info_begin(); s_info.has_vol = true; s_info.vol = (int16_t)target;
        s_info.vol_min = range.min; s_info.vol_max = range.max; info_end();
        if (tuh_audio_volume_set_sync(idx, st, TUH_AUDIO_CHANNEL_MASTER, (int16_t)target) != XFER_RESULT_SUCCESS)
            LOG1("usbout: volume set failed\n");
        else
            LOG1("usbout: device volume %d/256 dB (range %d..%d)\n", (int)target, range.min, range.max);
    }
}

static void read_name(uint8_t idx, char *out, size_t n)
{
    static uint16_t buf[48];
    out[0] = 0;
    memset(buf, 0, sizeof buf);
    const uint8_t daddr = tuh_audio_get_dev_addr(idx);
    if (tuh_descriptor_get_product_string_sync(daddr, 0x0409, buf, sizeof buf) != XFER_RESULT_SUCCESS) return;
    const int chars = ((buf[0] & 0xFF) - 2) / 2;
    size_t k = 0;
    for (int i = 0; i < chars && i < 47 && k + 1 < n; i++) {
        const uint16_t c = buf[1 + i];
        out[k++] = (c >= 0x20 && c < 0x7F && c != '"' && c != '\\') ? (char)c : '?';
    }
    out[k] = 0;
}

static bool configure_and_start(uint8_t idx)
{
    int best = -1, best_score = -1;
    uint8_t best_stream = TUSB_INDEX_INVALID_8;
    tuh_audio_stream_config_t best_cfg = {0};
    for (uint8_t st = 0; st < tuh_audio_stream_count(idx); st++) {
        if (!tuh_audio_stream_exists(idx, st) || tuh_audio_stream_direction(idx, st) != TUH_AUDIO_STREAM_PLAYBACK) continue;
        for (uint8_t i = 0; i < tuh_audio_config_count(idx, st); i++) {
            tuh_audio_stream_config_t c;
            if (!tuh_audio_config_get(idx, st, i, &c)) continue;
            LOG1("usbout:   stream %u cfg %u: %lu Hz, %u ch, format %u\n", st, i,
                 (unsigned long)c.sample_rate, c.channels, (unsigned)c.format);
            const int sc = config_score(&c);
            if (sc > best_score) { best_score = sc; best = i; best_stream = st; best_cfg = c; }
        }
    }
    if (best < 0) { set_state(USBOUT_UNSUPPORTED); LOG1("usbout: no usable playback format\n"); return false; }
    if (!tuh_audio_configure(idx, best_stream, (uint8_t)best)) {
        LOG1("usbout: configure failed\n"); set_state(USBOUT_FAILED); return false;
    }
    s_idx = idx; s_stream = best_stream; s_cfg_i = (uint8_t)best; s_cfg = best_cfg;
    s_frame_bytes = (uint32_t)s_cfg.channels * fmt_bytes(s_cfg.format);
    s_fifo_frames = CFG_TUH_AUDIO_STREAM_BUFSIZE / s_frame_bytes;
    s_rs_on = s_cfg.sample_rate != 48000;
    if (s_rs_on) rs_init(&s_rs, 48000.0, (double)s_cfg.sample_rate);
    setup_controls(idx, best_stream);

    usbout_info_t in = s_info;
    tuh_vid_pid_get(tuh_audio_get_dev_addr(idx), &in.vid, &in.pid);
    read_name(idx, in.name, sizeof in.name);
    in.rate = s_cfg.sample_rate; in.channels = s_cfg.channels;
    in.bits = fmt_bits(s_cfg.format); in.container = fmt_bytes(s_cfg.format);
    in.resampled = s_rs_on;
    info_begin(); s_info = in; info_end();
    LOG1("usbout: %04x:%04x \"%s\": %lu Hz, %u ch, %u-bit%s\n", in.vid, in.pid, in.name,
         (unsigned long)in.rate, in.channels, in.bits, s_rs_on ? " (resampled from 48 kHz)" : "");

    set_state(USBOUT_STARTING);
    if (!tuh_audio_start(idx, best_stream)) { set_state(USBOUT_FAILED); return false; }
    return true;
}

/* ------------------------------------------------------------------ TinyUSB callbacks (core 1) */
void tuh_audio_mount_cb(uint8_t idx)
{
    s_mount_idx = idx;
    s_mount_at = to_ms_since_boot(get_absolute_time()) + MOUNT_DELAY_MS;
    if (!s_mount_at) s_mount_at = 1;
}

void tuh_audio_umount_cb(uint8_t idx)
{
    if (idx == s_idx || idx == s_mount_idx) {
        s_idx = s_stream = s_mount_idx = TUSB_INDEX_INVALID_8;
        s_mount_at = s_retry_at = 0;
        usbout_info_t z = {0};
        info_begin(); s_info = z; info_end();
        set_state(USBOUT_NONE);
        LOG1("usbout: device removed\n");
    }
}

void tuh_audio_playback_cb(uint8_t idx, uint8_t stream_idx, uint16_t xferred_bytes)
{
    (void)idx; (void)stream_idx;
    /* Frames that passed without a packet: a synchronous device plays a gap. */
    static uint32_t last_frame;
    const uint32_t fr = pio_usb_host_get_frame_number();
    if (s_info.packets && fr - last_frame > 1 && fr - last_frame < 1000) s_info.skipped += fr - last_frame - 1;
    last_frame = fr;
    if (!s_info.pkt_min || xferred_bytes < s_info.pkt_min) s_info.pkt_min = xferred_bytes;
    if (xferred_bytes > s_info.pkt_max) s_info.pkt_max = xferred_bytes;
    s_info.packets++;                 /* single writer (core 1); a torn read only miscounts */
}

void tuh_audio_capture_cb(uint8_t idx, uint8_t stream_idx, uint16_t xferred_bytes)
{
    (void)idx; (void)stream_idx; (void)xferred_bytes;
}

void tuh_audio_event_cb(uint8_t idx, uint8_t stream_idx, tuh_audio_event_t event, tusb_xfer_result_t result)
{
    if (idx != s_idx || stream_idx != s_stream) return;
    if (event == TUH_AUDIO_EVENT_START_COMPLETE && result == XFER_RESULT_SUCCESS) {
        set_state(USBOUT_STREAMING);
        LOG1("usbout: streaming\n");
        return;
    }
    if (event == TUH_AUDIO_EVENT_STOP_COMPLETE) return;
    LOG1("usbout: stream %s (result %u), retrying\n", event == TUH_AUDIO_EVENT_START_COMPLETE ? "start failed" : "failed",
         (unsigned)result);
    set_state(USBOUT_FAILED);
    s_retry_at = to_ms_since_boot(get_absolute_time()) + RETRY_DELAY_MS;
    if (!s_retry_at) s_retry_at = 1;
}

/* ------------------------------------------------------------------ feeding */
static void feed(void)
{
    static int16_t lr[64 * 2], rs_out[64 * 2];
    static int16_t rs_in[32 * 2];
    static int rs_in_n, rs_in_pos;
    static uint8_t pkt[64 * 8 * 4];
    const uint32_t target = s_cfg.sample_rate / 1000 * FIFO_TARGET_MS;
    for (int guard = 0; guard < 8; guard++) {
        const uint32_t free_frames = tuh_audio_write_available(s_idx, s_stream);
        const uint32_t level = s_fifo_frames > free_frames ? s_fifo_frames - free_frames : 0;
        if (level >= target) return;
        uint32_t n = target - level;
        if (n > 64) n = 64;
        if (n > free_frames) n = free_frames;
        if (!n) return;
        const int16_t *src = lr;
        /* Take only what the DSP has finished: dhp_read_stereo() pads a
         * shortfall with silence, which would splice gaps into the audio.
         * TinyUSB's FIFO (FIFO_TARGET_MS) rides out the DSP's bursts. */
        if (!s_rs_on) {
            const uint32_t avail = dhp_out_available(s_pipe);
            if (n > avail && !s_tone) n = avail;           /* the tone plays regardless */
            if (!n) break;
            dhp_read_stereo(s_pipe, lr, n < avail ? n : avail);
        } else {
            /* 48 kHz pipeline -> device rate */
            int produced = 0;
            while (produced < (int)n) {
                if (rs_in_n == 0) {
                    uint32_t k = dhp_out_available(s_pipe);
                    if (k > 32) k = 32;
                    if (!k) break;
                    dhp_read_stereo(s_pipe, rs_in, k);
                    rs_in_n = (int)k; rs_in_pos = 0;
                }
                int used = 0;
                const int got = rs_process(&s_rs, rs_in + rs_in_pos * 2, rs_in_n, &used,
                                           rs_out + produced * 2, (int)n - produced);
                rs_in_n -= used; rs_in_pos += used;
                produced += got;
                if (!got && !used) break;
            }
            n = (uint32_t)produced;
            src = rs_out;
            if (!n) break;
        }
        if (s_tone) { tone_fill(rs_out, n, s_cfg.sample_rate); src = rs_out; }
        convert(src, n, pkt);
        tuh_audio_write(s_idx, s_stream, pkt, n);
    }
    /* about to run short of a whole packet: the driver would send silence */
    const uint32_t free_frames = tuh_audio_write_available(s_idx, s_stream);
    const uint32_t level = s_fifo_frames > free_frames ? s_fifo_frames - free_frames : 0;
    static bool was_low;
    const bool low = level < s_cfg.sample_rate / 1000;
    if (low && !was_low) s_info.fifo_dry++;
    was_low = low;
}

/* No device streaming: drain the pipeline at 48 kHz (the USB input side and
 * its feedback keep running at a sane rate). */
static void drain_virtual(bool reset)
{
    static uint64_t t0, done;
    if (reset) { t0 = 0; return; }
    if (!t0) { t0 = time_us_64(); done = 0; }
    const uint64_t due = (time_us_64() - t0) * 48 / 1000;
    while (due - done >= 48) {
        int16_t junk[48 * 2];
        dhp_read_stereo(s_pipe, junk, 48);
        done += 48;
    }
}

/* ------------------------------------------------------------------ core 1 */
void usbout_core1_main(dhp_t *pipe)
{
    s_pipe = pipe;
    flash_safe_execute_core_init();          /* core 0 can pause us for flash writes / BOOTSEL */

    pio_usb_configuration_t cfg = PIO_USB_DEFAULT_CONFIG;
    cfg.pin_dp = USBOUT_PIO_USB_DP_PIN;
    cfg.pio_tx_num = 0;                      /* PIO2 stays free for the status LED */
    cfg.pio_rx_num = 1;
    tuh_configure(BOARD_TUH_RHPORT, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &cfg);
    tusb_rhport_init_t host_init = { .role = TUSB_ROLE_HOST, .speed = TUSB_SPEED_AUTO };
    tusb_init(BOARD_TUH_RHPORT, &host_init);
    LOG1("usbout: USB host on GP%d/GP%d\n", USBOUT_PIO_USB_DP_PIN, USBOUT_PIO_USB_DP_PIN + 1);

    for (;;) {
        tuh_task_ext(0, false);
        const uint32_t now = to_ms_since_boot(get_absolute_time());

        if (s_mount_at && (int32_t)(now - s_mount_at) >= 0) {
            s_mount_at = 0;
            LOG1("usbout: audio device mounted\n");
            configure_and_start(s_mount_idx);
        }
        if (s_retry_at && (int32_t)(now - s_retry_at) >= 0 && s_idx != TUSB_INDEX_INVALID_8) {
            s_retry_at = 0;
            set_state(USBOUT_STARTING);
            if (!tuh_audio_start(s_idx, s_stream)) set_state(USBOUT_FAILED);
        }
        if (s_restart_req) {
            s_restart_req = false;
            if (s_idx != TUSB_INDEX_INVALID_8) {
                LOG1("usbout: restart requested\n");
                tuh_audio_stop(s_idx, s_stream);
                s_mount_idx = s_idx;
                s_mount_at = now + MOUNT_DELAY_MS;
            }
        }

        if (s_state == USBOUT_STREAMING) {
            drain_virtual(true);
            feed();
        } else {
            drain_virtual(false);
        }
    }
}
