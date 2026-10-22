/*
 * LakeHP 5.1/7.1 -> USB DAC / 2.4 GHz headset, Waveshare RP2350-USB-A.
 *
 *   USB-C (native USB, device): the 5.1/7.1 (UAC2) or stereo (UAC1) sound card,
 *           settings page network adapter, optional debug serial port.
 *   USB-A (PIO-USB, host):      the output - a USB DAC dongle or the USB
 *           dongle of a wireless headset (any USB Audio Class 1/2 playback
 *           device at full speed).
 *
 *   core 0: TinyUSB device from a 250 us timer interrupt plus the USB
 *           interrupt (see firmware/pico2w/main.c for why), and the headphone
 *           virtualizer DSP in the main loop, with the button, LED and settings.
 *   core 1: PIO-USB host. Its bit-banged USB needs a whole core: SOF every
 *           millisecond, one isochronous packet per frame (usbout.c).
 * 240 MHz at 1.20 V: PIO-USB needs a multiple of 12 MHz, and the DSP gets
 * a whole core instead of sharing one with the radio.
 *
 * Button (BOOTSEL, or an optional GPIO button to GND):
 *   single press -> cycle Small / Medium / Large room (1/2/3 blue blinks)
 *   double press -> low-latency stereo on/off (4 blinks = on)
 *   triple press -> USB mode 5.1/7.1 <-> stereo (restarts)
 *   4 presses    -> speaker placement preset Factory <-> Theater (saved)
 *   5 presses    -> settings page network adapter on/off (restarts)
 *   hold 2 s     -> restart the output stream
 *
 * LED: green = playing (cyan in stereo mode), yellow = starting, red blip = nothing plugged in,
 * magenta = device without a usable playback format, fast red = retrying.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/flash.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "tusb.h"

#include "lakehp.h"
#include "dh_pipeline.h"
#include "usb_audio51.h"
#include "usbout.h"
#include "status_led.h"
#if LHP_FILTERS_EMBEDDED
#include "lakehp_filters.h"
#endif
#include "lhp_filter_region.h"
#if LHP_WEBCFG
#include "webcfg.h"
#endif
#if LHP_DEBUG_CDC
#include "dbg_cdc.h"
#endif

/* ------------------------------------------------------------------ hang checkpoints */
/* Where core 0 is, kept in watchdog scratch registers (they survive a reset):
 * after a watchdog reset (core 0 stuck for 2 s) the log and the settings page
 * say which step it was stuck in. scratch[0] magic, [1] main loop step,
 * [2] USB interrupt step, [3] core 1 step. */
#define CP_MAGIC 0x4C485043u
#define CP_MAIN(n) (watchdog_hw->scratch[1] = (n))
#define CP_IRQ(n)  (watchdog_hw->scratch[2] = (n))
static char g_last_reset[96];
static const char *cp_name(uint32_t n)
{
    switch (n) {
    case 0: return "idle";
    case 1: return "DSP"; case 2: return "button"; case 3: return "LED"; case 4: return "restart check";
    case 5: return "network (lwIP)"; case 6: return "debug log";
    case 21: return "button: waiting for USB frame"; case 22: return "button: freezing core 1";
    case 23: return "button: reading BOOTSEL"; case 24: return "button: releasing core 1";
    case 50: return "settings: applying a change"; case 51: return "settings: building state";
    case 60: return "saving settings to flash";
    case 101: return "USB irq: tud_task"; case 102: return "USB irq: audio pump";
    case 103: return "USB irq: network"; case 104: return "USB irq: debug log"; case 110: return "USB irq (USBCTRL)";
    case 201: return "core 1: USB host task"; case 202: return "core 1: mount/start"; case 203: return "core 1: feed";
    default: return "?";
    }
}

/* 2 MB flash: firmware below 1 MB, filters at 1 MB, settings near the end. */
#define FILTER_REGION_OFFSET 0x100000u
#define FILTER_REGION_MAX    (448u * 1024u)
LHP_REGION_TAG(XIP_BASE + FILTER_REGION_OFFSET, FILTER_REGION_MAX);

#ifndef LHP_SYS_KHZ
#define LHP_SYS_KHZ 240000
#endif
#ifndef LHP_DEFAULT_ROOM
#define LHP_DEFAULT_ROOM 1
#endif
#ifndef LHP_BUTTON_GPIO
#define LHP_BUTTON_GPIO -1
#endif
#if LHP_DEBUG_CDC
#define LOG0(...) dbg_logf(__VA_ARGS__)
#else
#define LOG0(...) do { } while (0)
#endif
#define LONG_PRESS_MS 2000
#define MULTI_GAP_MS  350

/* ------------------------------------------------------------------ settings in flash */
/* Same layout as the Pico 2 W (v3), so the two builds read each other's sector. */
#define MODE_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - 8u * FLASH_SECTOR_SIZE)
#define MODE_MAGIC        0x4D50484Cu            /* "LHPM" */
typedef struct {
    uint32_t stereo;
    lhp_surround_t sur;
    uint32_t web_off, room, lowlat;
    uint32_t ch8;
    lhp_surround_t back;
} settings_t;
static bool placement_ok(const lhp_surround_t *s)
{
    return s->near_delay >= -12 && s->near_delay <= 24 && s->far_hf_db >= -6 && s->far_hf_db <= 12 &&
           s->back_eq >= -1 && s->back_eq <= 2;
}
static void read_placement(const volatile uint32_t *p, lhp_surround_t *s)
{
    memcpy(&s->near_delay, (const void *)&p[0], 4);
    memcpy(&s->far_hf_db, (const void *)&p[1], 4);
    memcpy(&s->back_eq, (const void *)&p[2], 4);
    if (!placement_ok(s)) *s = LHP_SURROUND_FACTORY;
}
static settings_t g_set;
/* The settings sector holds 16 page-sized slots, written in turn; the last
 * valid one wins. Programming one page freezes core 1 (the USB output) for
 * under a millisecond. Erasing the sector takes ~50 ms - long enough for
 * the output device to see the bus as suspended - so it happens only when
 * all slots are used (every 16th save), and the output is restarted after. */
#define SET_SLOTS (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)
static const volatile uint32_t *slot_ptr(int i)
{
    return (const volatile uint32_t *)(XIP_BASE + MODE_FLASH_OFFSET + (uint32_t)i * FLASH_PAGE_SIZE);
}
static bool slot_valid(int i) { const volatile uint32_t *p = slot_ptr(i); return p[0] == MODE_MAGIC && p[3] == 3; }
static bool slot_erased(int i)
{
    const volatile uint32_t *p = slot_ptr(i);
    for (unsigned k = 0; k < FLASH_PAGE_SIZE / 4; k++) if (p[k] != 0xFFFFFFFFu) return false;
    return true;
}
static void settings_load(void)
{
    memset(&g_set, 0, sizeof g_set);
    g_set.room = LHP_DEFAULT_ROOM;
    int last = -1;
    for (int i = 0; i < SET_SLOTS; i++) if (slot_valid(i)) last = i;
    if (last < 0) return;
    const volatile uint32_t *p = slot_ptr(last);
    g_set.stereo = p[1] == 1;
    read_placement(&p[4], &g_set.sur);
    g_set.web_off = p[7] == 1;
    if (p[8] >= 1 && p[8] <= 3) g_set.room = p[8];
    g_set.lowlat = p[9] == 1;
    g_set.ch8 = p[10] == 1;
    read_placement(&p[11], &g_set.back);
}
typedef struct { const settings_t *v; int slot; bool erase; } settings_write_t;
static void settings_write_cb(void *param)
{
    static uint32_t page[FLASH_PAGE_SIZE / 4];
    const settings_write_t *w = param;
    const settings_t *v = w->v;
    memset(page, 0xFF, sizeof page);
    page[0] = MODE_MAGIC;
    page[1] = v->stereo;
    page[2] = dhp_surround_is_factory(&v->sur) ? 0 : 1;
    page[3] = 3;
    memcpy(&page[4], &v->sur.near_delay, 4);
    memcpy(&page[5], &v->sur.far_hf_db, 4);
    memcpy(&page[6], &v->sur.back_eq, 4);
    page[7] = v->web_off;
    page[8] = v->room;
    page[9] = v->lowlat;
    page[10] = v->ch8;
    memcpy(&page[11], &v->back.near_delay, 4);
    memcpy(&page[12], &v->back.far_hf_db, 4);
    memcpy(&page[13], &v->back.back_eq, 4);
    if (w->erase) flash_range_erase(MODE_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(MODE_FLASH_OFFSET + (uint32_t)w->slot * FLASH_PAGE_SIZE, (const uint8_t *)page, FLASH_PAGE_SIZE);
}
static void settings_save(void)
{
    settings_write_t w = { &g_set, -1, false };
    for (int i = 0; i < SET_SLOTS; i++) if (slot_erased(i)) { w.slot = i; break; }
    /* a slot is only usable if every later one is erased too (keeps "last valid wins") */
    if (w.slot >= 0) for (int i = w.slot + 1; i < SET_SLOTS; i++) if (!slot_erased(i)) { w.slot = -1; break; }
    if (w.slot < 0) { w.slot = 0; w.erase = true; }
    const uint32_t prev = watchdog_hw->scratch[1];
    watchdog_hw->scratch[1] = 60;
    usbout_core0_flash_busy = true;     /* core 1 skips its BOOTSEL reads meanwhile */
    busy_wait_us_32(500);               /* let a read in progress finish */
    const int rc = flash_safe_execute(settings_write_cb, &w, 1000);
    usbout_core0_flash_busy = false;
    watchdog_hw->scratch[1] = prev;
    LOG0("Settings saved to slot %d%s (rc %d)\n", w.slot, w.erase ? " after erasing the sector" : "", rc);
    if (w.erase) usbout_restart();      /* the device may have suspended during the erase */
}
static uint32_t g_reboot_at;

static lhp_engine g_eng;
static dhp_t      g_pipe;
static dhp_filter_t g_filters[16];
static bool g_have_filters;
static char g_filter_src[112];          /* where the room filters came from (settings page) */

/* ------------------------------------------------------------------ USB device glue */
static volatile uint32_t g_usb_frames;
void usb_audio51_on_volume(int16_t vol, bool mute)
{
    dhp_set_usb_volume(&g_pipe, vol, mute);
    LOG0("USB: volume %d.%02d dB, mute %d\n", vol / 256, (abs(vol) % 256) * 100 / 256, (int)mute);
}
void usb_audio51_on_stream(bool active) { (void)active; LOG0("USB: host %s the audio stream\n", active ? "opened" : "closed"); }

static void usb_pump(void)
{
    static int16_t tmp[64 * USB_AUDIO51_MAX_CHANNELS];
    for (;;) {
        const uint32_t cap = usb_audio51_fifo_frames();
        uint32_t want = dhp_usb_frames_wanted_guarded(&g_pipe, usb_audio51_frames_available(),
                                                      cap * 3 / 4, cap / 2);
        if (!want) break;
        if (want > 64) want = 64;
        uint32_t got = usb_audio51_read(tmp, want);
        if (!got) break;
        dhp_usb_write(&g_pipe, tmp, got);
        g_usb_frames += got;
    }
}

static bool usb_service_cb(repeating_timer_t *t)
{
    (void)t;
    CP_IRQ(101);
    tud_task();
    CP_IRQ(102);
    usb_pump();
#if LHP_WEBCFG
    CP_IRQ(103);
    if (usb_audio51_has_network()) webcfg_usb_service();
#endif
#if LHP_DEBUG_CDC
    CP_IRQ(104);
    dbg_cdc_task();
#endif
    CP_IRQ(0);
    return true;
}
static void usb_irq_task(void) { CP_IRQ(110); tud_task(); CP_IRQ(0); }

/* ------------------------------------------------------------------ DSP (core 0 main loop) */
static volatile uint32_t g_dsp_us_sum, g_dsp_n, g_dsp_us_max, g_dsp_us_total;
static bool dsp_task(void)
{
    const uint32_t t0 = time_us_32();
    const bool did = dhp_dsp_poll(&g_pipe);
    if (did) {
        const uint32_t dt = time_us_32() - t0;
        g_dsp_us_sum += dt; g_dsp_n++; g_dsp_us_total += dt;
        if (dt > g_dsp_us_max) g_dsp_us_max = dt;
    }
    return did;
}

/* ------------------------------------------------------------------ button */

static bool button_pressed(void)
{
#if LHP_BUTTON_GPIO == -2
    return false;                       /* no button (diagnostics) */
#elif LHP_BUTTON_GPIO >= 0
    return !gpio_get(LHP_BUTTON_GPIO);
#else
    return usbout_bootsel();            /* read by core 1 at a safe point, see usbout.c */
#endif
}

/* ------------------------------------------------------------------ LED */
static int led_blinks;
static void led_task(uint32_t now)
{
    static uint32_t t0;
    if (led_blinks) {                                   /* N blue blinks */
        if (!t0) t0 = now;
        const uint32_t ph = (now - t0) / 150;
        if (ph >= (uint32_t)led_blinks * 2 + 2) { led_blinks = 0; t0 = 0; }
        else { const bool on = ph < (uint32_t)led_blinks * 2 && !(ph & 1); status_led_rgb(0, 0, on ? 40 : 0); return; }
    }
    switch (usbout_state()) {
    case USBOUT_STREAMING:                              /* cyan in stereo (console) mode */
        if (usb_audio51_is_stereo()) status_led_rgb(0, 16, 20); else status_led_rgb(0, 24, 0);
        break;
    case USBOUT_STARTING:    status_led_rgb(24, 16, 0); break;
    case USBOUT_UNSUPPORTED: status_led_rgb(24, 0, 24); break;
    case USBOUT_FAILED:      status_led_rgb((now / 100) & 1 ? 32 : 0, 0, 0); break;
    default:                 status_led_rgb(now % 1000 < 80 ? 24 : 0, 0, 0); break;
    }
}

static void button_task(uint32_t now)
{
    static uint32_t last_poll, down_at, released_at;
    static bool down, long_fired;
    static int clicks;
    if (g_reboot_at) return;
    static bool raw_prev, p;
    if (now - last_poll < 50) return;
    last_poll = now;
    /* Debounce: the state changes only when two reads 50 ms apart agree. */
    const bool raw = button_pressed();
    if (raw == raw_prev) p = raw;
    raw_prev = raw;
    if (p && !down) { down = true; down_at = now; long_fired = false; }
    else if (p && down && !long_fired && now - down_at >= LONG_PRESS_MS) {
        long_fired = true;
        clicks = 0;
        usbout_restart();
        led_blinks = 3;
        LOG0("Restarting the output stream\n");
    } else if (!p && down) {
        down = false;
        if (!long_fired) { clicks++; released_at = now; }
    }
    if (!clicks || down || now - released_at <= MULTI_GAP_MS) return;
    if ((clicks < 3 || clicks == 4) && !g_have_filters) {
        led_blinks = 8;
        LOG0("No room filters loaded: add them with lakehp_flasher.html\n");
    } else if (clicks == 1) {
        const int next = dhp_cycle_room(&g_pipe);
        led_blinks = next;
        LOG0("Room -> %s\n", dhp_room_name(next));
    } else if (clicks == 2) {
        const bool on = dhp_toggle_low_latency(&g_pipe);
        led_blinks = on ? 4 : g_pipe.room;
        LOG0("Low-latency stereo %s\n", on ? "ON" : "OFF (rooms)");
    } else if (clicks == 3) {
        g_set.stereo = usb_audio51_is_stereo() ? 0 : 1;
        LOG0("Button x3: USB mode -> %s, restarting\n", g_set.stereo ? "stereo (UAC1)" : g_set.ch8 ? "7.1" : "5.1");
        led_blinks = 6;
        g_reboot_at = now + 6 * 300 + 200;
    } else if (clicks == 4) {
        lhp_surround_t sur, back;
        dhp_get_placement(&g_pipe, &sur, &back);
        const int to = dhp_preset_of(&sur, &back) == DHP_PRESET_FACTORY ? DHP_PRESET_THEATER : DHP_PRESET_FACTORY;
        dhp_preset(to, &g_set.sur, &g_set.back);
        dhp_set_placement(&g_pipe, &g_set.sur, &g_set.back, true);
        settings_save();
        led_blinks = to == DHP_PRESET_THEATER ? 2 : 1;
        LOG0("Button x4: placement -> %s\n", to == DHP_PRESET_THEATER ? "Theater" : "Factory");
    } else {
        g_set.web_off = !g_set.web_off;
        LOG0("Button x%d: settings adapter -> %s, restarting\n", clicks, g_set.web_off ? "off" : "on");
        led_blinks = 5;
        g_reboot_at = now + 5 * 300 + 200;
    }
    clicks = 0;
}

static void reboot_task(uint32_t now)
{
    if (!g_reboot_at || (int32_t)(now - g_reboot_at) < 0) return;
    settings_save();
    watchdog_hw->scratch[0] = 0;                        /* a planned restart, not a hang */
    watchdog_reboot(0, 0, 10);
    for (;;) tight_loop_contents();
}

/* ------------------------------------------------------------------ status (debug build) */
#if LHP_DEBUG_CDC
static void status_task(uint32_t now)
{
    static uint32_t last, last_frames, last_blocks, last_pk;
    if (now - last < 2000) return;
    const uint32_t dt = now - last;
    last = now;
    const uint32_t fr = g_usb_frames, bl = g_pipe.blocks;
    usbout_info_t o;
    usbout_get_info(&o);
    LOG0("[%lu s] usb in: %s, %lu frames/s, fifo %lu | dsp: %lu blocks/s, room %d, LL %d, in-ovr %lu, "
         "out-under %lu, queued %lu\n",
         (unsigned long)(now / 1000), usb_audio51_streaming() ? "streaming" : "idle",
         (unsigned long)((fr - last_frames) * 1000u / dt), (unsigned long)usb_audio51_frames_available(),
         (unsigned long)((bl - last_blocks) * 1000u / dt), g_pipe.room, (int)dhp_low_latency(&g_pipe),
         (unsigned long)g_pipe.in_overruns, (unsigned long)g_pipe.out_underruns,
         (unsigned long)dhp_out_available(&g_pipe));
    LOG0("  out: state %d, %04x:%04x \"%s\" %lu Hz %u ch %u-bit, %lu pkts/s, fifo dips %lu\n",
         (int)o.state, o.vid, o.pid, o.name, (unsigned long)o.rate, o.channels, o.bits,
         (unsigned long)((o.packets - last_pk) * 1000u / dt), (unsigned long)o.fifo_dry);
    const uint32_t dn = g_dsp_n, ds = g_dsp_us_sum, dm = g_dsp_us_max;
    g_dsp_n = g_dsp_us_sum = g_dsp_us_max = 0;
    LOG0("  load: dsp %lu us/block avg (max %lu) = core0 %lu%%\n",
         (unsigned long)(dn ? ds / dn : 0), (unsigned long)dm, (unsigned long)(ds / (dt * 10u)));
#if LHP_WEBCFG
    { char nb[200]; webcfg_debug(nb, sizeof nb); LOG0("  %s\n", nb); }
#endif
    last_frames = fr; last_blocks = bl; last_pk = o.packets;
}
#endif

/* ------------------------------------------------------------------ settings page */
#if LHP_WEBCFG
#ifndef LHP_FW_VERSION
#define LHP_FW_VERSION "2026.10-usbhost"
#endif
static void json_safe(char *s) { for (; *s; s++) if (*s == '"' || *s == '\\' || (unsigned char)*s < 0x20) *s = '?'; }

static void web_state(char *b, int max)
{
    CP_MAIN(51);
    static uint32_t last_ms, last_dsp, last_rx, rx_rate;
    const uint32_t now = to_ms_since_boot(get_absolute_time()), dt = now - last_ms ? now - last_ms : 1;
    const uint32_t dsp = g_dsp_us_total;
    uint32_t rxp, rxb, fbp;
    usb_audio51_debug_counters(&rxp, &rxb, &fbp);
    const int dsp_pct = (int)((dsp - last_dsp) / (dt * 10u));
    if (dt > 200) { rx_rate = (rxp - last_rx) * 1000u / dt; last_ms = now; last_dsp = dsp; last_rx = rxp; }
    lhp_surround_t s, k;
    dhp_get_placement(&g_pipe, &s, &k);
    usbout_info_t o;
    usbout_get_info(&o);
    json_safe(o.name);
    char volstr[48] = "null";
    if (o.has_vol) snprintf(volstr, sizeof volstr, "\"%.1f dB (range %.1f..%.1f)\"",
                            o.vol / 256.0, o.vol_min / 256.0, o.vol_max / 256.0);
    const char *arr = usb_audio51_is_stereo() ? "stereo" : usb_audio51_channels() == 8 ? "71" : "51";
    const char *saved_arr = g_set.stereo ? "stereo" : g_set.ch8 ? "71" : "51";
    snprintf(b, max,
        "{\"fw\":\"%s\",\"reset\":\"%s\",\"arr\":\"%s\",\"stereo\":%s,\"filters\":%s,\"fsrc\":\"%s\",\"room\":%d,\"lowlat\":%s,"
        "\"preset\":\"%s\","
        "\"sur\":{\"delay\":%.2f,\"hf\":%.2f,\"eq\":%.2f,\"az\":%.1f},"
        "\"back\":{\"delay\":%.2f,\"hf\":%.2f,\"eq\":%.2f,\"az\":%.1f},"
        "\"saved\":{\"arr\":\"%s\",\"preset\":\"%s\",\"room\":%u,\"lowlat\":%s,"
        "\"sur\":{\"delay\":%.2f,\"hf\":%.2f,\"eq\":%.2f},\"back\":{\"delay\":%.2f,\"hf\":%.2f,\"eq\":%.2f}},"
        "\"usb\":{\"stream\":%s,\"rx\":%lu,\"in_ovr\":%lu,\"out_under\":%lu,\"blocks\":%lu},"
        "\"out\":{\"state\":%d,\"name\":\"%s\",\"vid\":\"%04x\",\"pid\":\"%04x\",\"rate\":%lu,\"ch\":%u,\"bits\":%u,"
        "\"rs\":%s,\"dry\":%lu,\"tone\":%s,\"pkts\":%lu,\"skip\":%lu,\"pmin\":%u,\"pmax\":%u,\"vol\":%s,\"sofp\":%s,\"sofmax\":%lu,\"sofover\":%lu},"
        "\"bt\":{\"state\":0,\"addr\":\"\",\"rate\":0,\"bitpool\":0},"
        "\"load\":{\"dsp\":%d,\"enc\":0}}",
        LHP_FW_VERSION, g_last_reset, arr, usb_audio51_is_stereo() ? "true" : "false", g_have_filters ? "true" : "false", g_filter_src,
        dhp_room(&g_pipe), dhp_low_latency(&g_pipe) ? "true" : "false",
        dhp_preset_name(dhp_preset_of(&s, &k)),
        (double)s.near_delay, (double)s.far_hf_db, (double)s.back_eq, (double)lhp_surround_azimuth(&s),
        (double)k.near_delay, (double)k.far_hf_db, (double)k.back_eq, (double)lhp_surround_azimuth(&k),
        saved_arr, dhp_preset_name(dhp_preset_of(&g_set.sur, &g_set.back)),
        (unsigned)g_set.room, g_set.lowlat ? "true" : "false",
        (double)g_set.sur.near_delay, (double)g_set.sur.far_hf_db, (double)g_set.sur.back_eq,
        (double)g_set.back.near_delay, (double)g_set.back.far_hf_db, (double)g_set.back.back_eq,
        usb_audio51_streaming() ? "true" : "false", (unsigned long)rx_rate,
        (unsigned long)g_pipe.in_overruns, (unsigned long)g_pipe.out_underruns, (unsigned long)g_pipe.blocks,
        (int)o.state, o.name, o.vid, o.pid, (unsigned long)o.rate, o.channels, o.bits,
        o.resampled ? "true" : "false", (unsigned long)o.fifo_dry, usbout_tone() ? "true" : "false",
        (unsigned long)o.packets, (unsigned long)o.skipped, o.pkt_min, o.pkt_max, volstr,
        o.sof_precise ? "true" : "false", (unsigned long)o.sof_dev_max_us, (unsigned long)o.sof_dev_over,
        dsp_pct > 100 ? 100 : dsp_pct);
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
static bool apply_pair(const char *body, const char *prefix, lhp_surround_t *s)
{
    char key[12];
    float v;
    bool any = false;
    snprintf(key, sizeof key, "%saz", prefix);
    if (webcfg_json_num(body, key, &v)) { *s = lhp_surround_for_azimuth(v); any = true; }
    snprintf(key, sizeof key, "%sdelay", prefix);
    if (webcfg_json_num(body, key, &v)) { s->near_delay = clampf(v, -12, 24); any = true; }
    snprintf(key, sizeof key, "%shf", prefix);
    if (webcfg_json_num(body, key, &v)) { s->far_hf_db = clampf(v, -6, 12); any = true; }
    snprintf(key, sizeof key, "%seq", prefix);
    if (webcfg_json_num(body, key, &v)) { s->back_eq = clampf(v, -1, 2); any = true; }
    return any;
}

static void web_apply(const char *body, char *resp, int max)
{
    CP_MAIN(50);
    float v;
    char str[24];
    const char *msg = "ok";
    if (g_have_filters) {
        if (webcfg_json_num(body, "room", &v) && v >= 1 && v <= 3) dhp_request_room(&g_pipe, (int)v);
        if (webcfg_json_num(body, "lowlat", &v)) dhp_set_low_latency(&g_pipe, v != 0);
        lhp_surround_t s, k;
        dhp_get_placement(&g_pipe, &s, &k);
        bool moved = apply_pair(body, "", &s);
        moved |= apply_pair(body, "b", &k);
        if (webcfg_json_str(body, "preset", str, sizeof str)) {
            dhp_preset(!strcmp(str, "theater") ? DHP_PRESET_THEATER : DHP_PRESET_FACTORY, &s, &k);
            moved = true;
        }
        if (moved) dhp_set_placement(&g_pipe, &s, &k, false);
    }
    if (webcfg_json_str(body, "arr", str, sizeof str)) {
        const uint32_t stereo = !strcmp(str, "stereo"), ch8 = !strcmp(str, "71");
        const char *cur = usb_audio51_is_stereo() ? "stereo" : usb_audio51_channels() == 8 ? "71" : "51";
        if (strcmp(str, cur) && (stereo || ch8 || !strcmp(str, "51"))) {
            g_set.stereo = stereo;
            if (!stereo) g_set.ch8 = ch8;
            g_reboot_at = to_ms_since_boot(get_absolute_time()) + 500;
            msg = stereo ? "Restarting as a stereo device" : ch8 ? "Restarting as a 7.1 device" : "Restarting as a 5.1 device";
        }
    }
    if (webcfg_json_str(body, "action", str, sizeof str)) {
        if (!strcmp(str, "save")) {
            dhp_get_placement(&g_pipe, &g_set.sur, &g_set.back);
            g_set.room = (uint32_t)dhp_room(&g_pipe);
            g_set.lowlat = dhp_low_latency(&g_pipe);
            settings_save();
            msg = "Saved";
        } else if (!strcmp(str, "revert")) {
            dhp_set_placement(&g_pipe, &g_set.sur, &g_set.back, false);
        } else if (!strcmp(str, "tone")) {
            usbout_set_tone(!usbout_tone());
            msg = usbout_tone() ? "Test tone on" : "Test tone off";
        } else if (!strcmp(str, "restart")) {
            usbout_restart();
            msg = "Restarting the output";
        } else if (!strcmp(str, "weboff")) {
            g_set.web_off = 1;
            g_reboot_at = to_ms_since_boot(get_absolute_time()) + 500;
        } else if (!strcmp(str, "reboot")) {
            g_reboot_at = to_ms_since_boot(get_absolute_time()) + 500;
        }
    }
    snprintf(resp, max, "{\"ok\":true,\"msg\":\"%s\"}", msg);
}
static const webcfg_hooks_t s_web_hooks = { web_state, web_apply };
#endif

/* ------------------------------------------------------------------ core 1 */
static void core1_entry(void) { usbout_core1_main(&g_pipe); }

/* ------------------------------------------------------------------ main */
int main(void)
{
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(2);
    set_sys_clock_khz(LHP_SYS_KHZ, true);
    stdio_init_all();
#if LHP_DEBUG_CDC
    dbg_cdc_init();
#endif
#if LHP_BUTTON_GPIO >= 0
    gpio_init(LHP_BUTTON_GPIO);
    gpio_pull_up(LHP_BUTTON_GPIO);
#endif
    if (watchdog_enable_caused_reboot() && watchdog_hw->scratch[0] == CP_MAGIC) {
        snprintf(g_last_reset, sizeof g_last_reset, "stuck in %s / %s / %s",
                 cp_name(watchdog_hw->scratch[1]), cp_name(watchdog_hw->scratch[2]), cp_name(watchdog_hw->scratch[3]));
        LOG0("*** Restarted by the watchdog: core 0 was %s\n", g_last_reset);
        printf("Watchdog reset: %s\n", g_last_reset);
    }
    watchdog_hw->scratch[0] = 0;
    watchdog_hw->scratch[1] = watchdog_hw->scratch[2] = watchdog_hw->scratch[3] = 0;
    status_led_init();
    status_led_rgb(16, 16, 16);
    LHP_REGION_TAG_KEEP();
    settings_load();
    int nfilt;
#if LHP_FILTERS_EMBEDDED
    nfilt = (int)LHP_NUM_FILTERS;
#ifdef LHP_FILTER_SOURCE
    snprintf(g_filter_src, sizeof g_filter_src, "%s (built in)", LHP_FILTER_SOURCE);
#else
    snprintf(g_filter_src, sizeof g_filter_src, "built in");
#endif
    for (int i = 0; i < nfilt && i < 16; i++)
        g_filters[i] = (dhp_filter_t){ lhp_filters[i].rate, lhp_filters[i].room,
                                       lhp_filters[i].blob, (uint32_t)lhp_filters[i].len };
#else
    nfilt = lhp_region_parse((const uint8_t *)(XIP_BASE + FILTER_REGION_OFFSET), FILTER_REGION_MAX, g_filters, 16);
    lhp_region_label((const uint8_t *)(XIP_BASE + FILTER_REGION_OFFSET), FILTER_REGION_MAX, g_filter_src, sizeof g_filter_src);
    if (nfilt < 0) {
        printf("No room filters in flash (code %d): add them with tools/lakehp_flasher.html\n", nfilt);
        nfilt = 0;
    }
#endif
    g_have_filters = dhp_init(&g_pipe, &g_eng, g_filters, nfilt, 48000, (int)g_set.room) == 0;
    if (!g_have_filters) {
        g_pipe.lowlat = true;
        led_blinks = 8;
        printf("WARNING: no 48 kHz room filters - running as plain low-latency stereo\n");
    }
    if (g_have_filters) {
        if (!g_set.stereo && g_set.ch8) {
            lhp_set_surround(&g_eng, &g_set.sur);
            lhp_set_back(&g_eng, &g_set.back);
            g_pipe.sur = g_pipe.sur_req = g_set.sur;
            g_pipe.back = g_pipe.back_req = g_set.back;
            dhp_set_channels(&g_pipe, 8);
        } else if (dhp_preset_of(&g_set.sur, &g_set.back) != DHP_PRESET_FACTORY) {
            dhp_set_placement_now(&g_pipe, &g_set.sur, &g_set.back);
        }
        if (g_set.lowlat) g_pipe.lowlat = true;
    } else if (!g_set.stereo && g_set.ch8) {
        dhp_set_channels(&g_pipe, 8);
    }
    printf("LakeHP USB host: %s, %d filters, %lu MHz\n", g_have_filters ? dhp_room_name(g_pipe.room) : "no rooms",
           nfilt, (unsigned long)(clock_get_hz(clk_sys) / 1000000));

    multicore_lockout_victim_init();          /* core 1 parks us briefly to read BOOTSEL */
    static uint32_t core1_stack[2048];
    multicore_launch_core1_with_stack(core1_entry, core1_stack, sizeof core1_stack);

    const bool stereo = g_set.stereo;
    usb_audio51_set_stereo(stereo);
    usb_audio51_set_channels(!stereo && g_set.ch8 ? 8 : 6);
#if LHP_WEBCFG
    const bool web = !stereo && !g_set.web_off;
    usb_audio51_set_network(web);
    if (web) webcfg_init(&s_web_hooks);
#endif
    g_pipe.stereo_input = stereo;
    tusb_rhport_init_t dev_init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    static repeating_timer_t usb_timer;
    add_repeating_timer_us(-250, usb_service_cb, NULL, &usb_timer);
    irq_add_shared_handler(USBCTRL_IRQ, usb_irq_task, PICO_SHARED_IRQ_HANDLER_LOWEST_ORDER_PRIORITY);

    watchdog_hw->scratch[0] = CP_MAGIC;
    watchdog_enable(2000, true);                        /* core 0 stuck for 2 s: reset */
    for (;;) {
        watchdog_update();
        const uint32_t now = to_ms_since_boot(get_absolute_time());
        CP_MAIN(1);
        const bool did = dsp_task();
        CP_MAIN(2);
        button_task(now);
        CP_MAIN(3);
        led_task(now);
        CP_MAIN(4);
        reboot_task(now);
#if LHP_WEBCFG
        CP_MAIN(5);
        if (web) webcfg_poll();
#endif
#if LHP_DEBUG_CDC
        CP_MAIN(6);
        status_task(now);
#endif
        CP_MAIN(0);
        (void)did;
    }
}
