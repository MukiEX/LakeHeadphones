/*
 * usb_audio51.c - UAC2 5.1/7.1 speaker for TinyUSB 0.18.
 * Based on the structure of TinyUSB's examples/device/uac2_speaker_fb (MIT).
 */
#include "tusb.h"
#include "usb_audio51_tusb_compat.h"
#include "usb_audio51.h"
#include <string.h>

/* ---------------------------------------------------------------- IDs */
#define ENT_CLOCK        0x04
#define ENT_INPUT_TERM   0x01
#define ENT_FEATURE      0x02
#define ENT_OUTPUT_TERM  0x03

#ifndef USB_AUDIO51_CDC
#define USB_AUDIO51_CDC 0          /* debug builds: CDC serial log port */
#endif
#ifndef USB_AUDIO51_NET
#define USB_AUDIO51_NET 0          /* RNDIS network adapter for the settings page */
#endif
enum { ITF_NUM_AUDIO_CONTROL = 0, ITF_NUM_AUDIO_STREAMING };
#define EPNUM_CDC_NOTIF  0x82
#define EPNUM_CDC_OUT    0x03
#define EPNUM_CDC_IN     0x83
#define EPNUM_NET_NOTIF  0x84
#define EPNUM_NET_OUT    0x05
#define EPNUM_NET_IN     0x85
#define EPNUM_HID_IN     0x86
#ifndef USB_AUDIO51_HID
#define USB_AUDIO51_HID  0
#endif
#if USB_AUDIO51_HID
#define HID_DESC_LEN     TUD_HID_DESC_LEN
#else
#define HID_DESC_LEN     0
#endif
#define STR_HID          12
#if USB_AUDIO51_CDC
#define CDC_DESC_LEN     TUD_CDC_DESC_LEN
#else
#define CDC_DESC_LEN     0
#endif
#define STR_NET          10
#define STR_MAC          11

#define EPNUM_AUDIO_OUT  0x01
#define EPNUM_AUDIO_FB   0x01

/* FL FR FC LFE BL BR = KSAUDIO_SPEAKER_5POINT1 */
#define CH_CFG_51  (AUDIO_CHANNEL_CONFIG_FRONT_LEFT | AUDIO_CHANNEL_CONFIG_FRONT_RIGHT | \
                    AUDIO_CHANNEL_CONFIG_FRONT_CENTER | AUDIO_CHANNEL_CONFIG_LOW_FRQ_EFFECTS | \
                    AUDIO_CHANNEL_CONFIG_BACK_LEFT | AUDIO_CHANNEL_CONFIG_BACK_RIGHT)
/* FL FR FC LFE BL BR SL SR = KSAUDIO_SPEAKER_7POINT1_SURROUND */
#define CH_CFG_71  (CH_CFG_51 | AUDIO_CHANNEL_CONFIG_SIDE_LEFT | AUDIO_CHANNEL_CONFIG_SIDE_RIGHT)

#ifndef USB_AUDIO51_VID
#define USB_AUDIO51_VID 0x1209      /* pid.codes */
#endif
#ifndef USB_AUDIO51_PID
#define USB_AUDIO51_PID 0x0001      /* pid.codes test PID: personal use only */
#endif

/* feature unit: master mute+volume, no per-channel controls */
#define FU_CTRL_MASTER ((AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS) | \
                        (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS))
#define FU_CH0 U32_TO_U8S_LE(0)
#define TUD_AUDIO_DESC_FEATURE_UNIT_SIX_CHANNEL(_unitid, _srcid, _c0, _stridx) \
    USB_AUDIO51_FU_LEN(6), TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, _unitid, _srcid, \
    U32_TO_U8S_LE(_c0), FU_CH0, FU_CH0, FU_CH0, FU_CH0, FU_CH0, FU_CH0, _stridx
#define TUD_AUDIO_DESC_FEATURE_UNIT_EIGHT_CHANNEL(_unitid, _srcid, _c0, _stridx) \
    USB_AUDIO51_FU_LEN(8), TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, _unitid, _srcid, \
    U32_TO_U8S_LE(_c0), FU_CH0, FU_CH0, FU_CH0, FU_CH0, FU_CH0, FU_CH0, FU_CH0, FU_CH0, _stridx


/* ---------------------------------------------------------------- descriptors */
static tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_AUDIO51_VID,
    .idProduct = USB_AUDIO51_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

/* ---------------------------------------------------------------- host OS guess
 * Apple hosts (macOS, iPadOS, iOS) want the full-speed feedback value as 10.14
 * in 3 bytes with a 3-byte feedback endpoint; Windows wants 16.16 in 4 bytes.
 * Sending the wrong one makes the host compute a nonsense packet rate, and
 * iOS then plays nothing. Apple asks for a string descriptor right after the
 * device descriptor, Windows and Linux ask for the configuration first
 * (the same heuristic as TinyUSB's uac2_speaker_fb example), and the answer
 * arrives before we have to hand out the configuration descriptor. */
static bool s_first_after_device_seen;
static bool s_apple_host = USB_AUDIO51_MACOS;
bool usb_audio51_host_is_apple(void) { return s_apple_host; }

/* Stereo (UAC1) mode, chosen once before tud_init(): see the UAC1 section. */
static bool s_stereo;
static uint8_t s_nch = USB_AUDIO51_CHANNELS;   /* 6 or 8, chosen once before tud_init() */
#if USB_AUDIO51_UAC1
void usb_audio51_set_stereo(bool on) { s_stereo = on; }
#endif
bool usb_audio51_is_stereo(void) { return s_stereo; }

static bool s_net;

uint8_t const *tud_descriptor_device_cb(void)
{
    s_first_after_device_seen = false;
    if (s_stereo && !USB_AUDIO51_CDC) {
        /* plain single-function audio device, as consoles expect */
        desc_device.bDeviceClass = 0; desc_device.bDeviceSubClass = 0; desc_device.bDeviceProtocol = 0;
    }
    /* each layout gets its own bcdDevice so Windows doesn't reuse a cached one */
    desc_device.bcdDevice = s_stereo ? 0x0101 : (uint16_t)(0x0100 | (s_net ? 0x02 : 0) | (s_nch == 8 ? 0x10 : 0) |
                                                       (USB_AUDIO51_HID ? 0x20 : 0));
    return (uint8_t const *)&desc_device;
}

/* The 5.1 configuration is put together at run time: the audio function,
 * then optionally the settings network adapter and the debug serial port. */
#define AUDIO_FN(_nch, _chcfg, _FU) \
    TUD_AUDIO_DESC_IAD(ITF_NUM_AUDIO_CONTROL, 0x02, 0x00), \
    TUD_AUDIO_DESC_STD_AC(ITF_NUM_AUDIO_CONTROL, 0x00, 4), \
    TUD_AUDIO_DESC_CS_AC(0x0200, AUDIO_FUNC_DESKTOP_SPEAKER, \
                         TUD_AUDIO_DESC_CLK_SRC_LEN + TUD_AUDIO_DESC_INPUT_TERM_LEN + \
                         TUD_AUDIO_DESC_OUTPUT_TERM_LEN + USB_AUDIO51_FU_LEN(_nch), \
                         AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS), \
    TUD_AUDIO_DESC_CLK_SRC(ENT_CLOCK, AUDIO_CLOCK_SOURCE_ATT_INT_FIX_CLK, \
                           (AUDIO_CTRL_R << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS), ENT_INPUT_TERM, 0x00), \
    TUD_AUDIO_DESC_INPUT_TERM(ENT_INPUT_TERM, AUDIO_TERM_TYPE_USB_STREAMING, 0x00, ENT_CLOCK, \
                              _nch, _chcfg, 0x00, 0x0000, 0x00), \
    TUD_AUDIO_DESC_OUTPUT_TERM(ENT_OUTPUT_TERM, AUDIO_TERM_TYPE_OUT_GENERIC_SPEAKER, ENT_INPUT_TERM, \
                               ENT_FEATURE, ENT_CLOCK, 0x0000, 0x00), \
    _FU(ENT_FEATURE, ENT_INPUT_TERM, FU_CTRL_MASTER, 0x00), \
    TUD_AUDIO_DESC_STD_AS_INT(ITF_NUM_AUDIO_STREAMING, 0x00, 0x00, 0x05), \
    TUD_AUDIO_DESC_STD_AS_INT(ITF_NUM_AUDIO_STREAMING, 0x01, 0x02, 0x05), \
    TUD_AUDIO_DESC_CS_AS_INT(ENT_INPUT_TERM, AUDIO_CTRL_NONE, AUDIO_FORMAT_TYPE_I, \
                             AUDIO_DATA_FORMAT_TYPE_I_PCM, _nch, _chcfg, 0x00), \
    TUD_AUDIO_DESC_TYPE_I_FORMAT(2, 16), \
    TUD_AUDIO_DESC_STD_AS_ISO_EP(EPNUM_AUDIO_OUT, \
        (uint8_t)(TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA), \
        USB_AUDIO51_EP_SIZE(USB_AUDIO51_RATE, 2, _nch), 0x01), \
    TUD_AUDIO_DESC_CS_AS_ISO_EP(AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, AUDIO_CTRL_NONE, \
                                AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC, 0x0001), \
    TUD_AUDIO_DESC_STD_AS_ISO_FB_EP(EPNUM_AUDIO_FB | 0x80, 4, 0x01)

static uint8_t const cfg_audio_fn51[] = { AUDIO_FN(6, CH_CFG_51, TUD_AUDIO_DESC_FEATURE_UNIT_SIX_CHANNEL) };
static uint8_t const cfg_audio_fn71[] = { AUDIO_FN(8, CH_CFG_71, TUD_AUDIO_DESC_FEATURE_UNIT_EIGHT_CHANNEL) };
TU_VERIFY_STATIC(sizeof(cfg_audio_fn51) == USB_AUDIO51_FN_LEN(6), "5.1 audio descriptor length mismatch");
TU_VERIFY_STATIC(sizeof(cfg_audio_fn71) == USB_AUDIO51_FN_LEN(8), "7.1 audio descriptor length mismatch");
TU_VERIFY_STATIC(USB_AUDIO51_EP_SIZE(USB_AUDIO51_RATE, 2, 8) <= CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX, "EP too small for 7.1");

uint16_t usb_audio51_fn_desc_len = USB_AUDIO51_FN_LEN(6);   /* read by TinyUSB's audio driver */
void usb_audio51_set_channels(int nch)
{
    s_nch = nch == 8 ? 8 : 6;
    usb_audio51_fn_desc_len = s_nch == 8 ? USB_AUDIO51_FN_LEN(8) : USB_AUDIO51_FN_LEN(6);
}
int usb_audio51_channels(void) { return s_stereo ? USB_AUDIO51_CHANNELS : s_nch; }
#define FRAME_BYTES ((uint32_t)s_nch * 2u)

#if USB_AUDIO51_NET
static uint8_t const cfg_net_fn[] = { TUD_RNDIS_DESCRIPTOR(0, STR_NET, EPNUM_NET_NOTIF, 8, EPNUM_NET_OUT, EPNUM_NET_IN, 64) };
#endif
#if USB_AUDIO51_CDC
static uint8_t const cfg_cdc_fn[] = { TUD_CDC_DESCRIPTOR(0, 6, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64) };
#endif
#if USB_AUDIO51_HID
static uint8_t const desc_hid_report[] = { TUD_HID_REPORT_DESC_CONSUMER() };
static uint8_t const cfg_hid_fn[] = { TUD_HID_DESCRIPTOR(0, STR_HID, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report),
                                                         EPNUM_HID_IN, CFG_TUD_HID_EP_BUFSIZE, 10) };
#endif
static uint8_t desc_configuration[TUD_CONFIG_DESC_LEN + USB_AUDIO51_FN_LEN(8) + CDC_DESC_LEN + HID_DESC_LEN
#if USB_AUDIO51_NET
                                  + TUD_RNDIS_DESC_LEN
#endif
                                  ];
#if USB_AUDIO51_NET
void usb_audio51_set_network(bool on) { s_net = on; }
#endif
bool usb_audio51_has_network(void) { return s_net; }

/* Audio streaming interface number: 1, or 3 when the network adapter
 * takes interfaces 0-1 (Windows' RNDIS driver fails with Code 10 unless
 * RNDIS is the first function of a composite device). */
static uint8_t s_itf_as = ITF_NUM_AUDIO_STREAMING;

/* Append a function descriptor built for interface 0 and renumber it to
 * start at `base`. cdc: also fix the interface numbers inside the CDC call
 * management and union descriptors. Returns the number of interfaces. */
static uint8_t append_fn(uint16_t *len, const uint8_t *fn, uint16_t fn_len, uint8_t base, bool cdc)
{
    uint8_t *d = desc_configuration + *len;
    memcpy(d, fn, fn_len);
    uint8_t n = 0;
    for (uint16_t i = 0; i + 2 < fn_len && d[i]; i += d[i]) {
        const uint8_t t = d[i + 1];
        if (t == TUSB_DESC_INTERFACE_ASSOCIATION) d[i + 2] += base;
        if (t == TUSB_DESC_INTERFACE) { if (d[i + 3] == 0) n++; d[i + 2] += base; }
        if (cdc && t == TUSB_DESC_CS_INTERFACE) {
            if (d[i + 2] == CDC_FUNC_DESC_CALL_MANAGEMENT) d[i + 4] += base;
            if (d[i + 2] == CDC_FUNC_DESC_UNION) { d[i + 3] += base; d[i + 4] += base; }
        }
    }
    *len += fn_len;
    return n;
}

static uint16_t build_configuration(void)
{
    uint16_t len = TUD_CONFIG_DESC_LEN;
    uint8_t nitf = 0;
#if USB_AUDIO51_NET
    if (s_net) nitf += append_fn(&len, cfg_net_fn, sizeof cfg_net_fn, nitf, true);
#endif
    s_itf_as = (uint8_t)(ITF_NUM_AUDIO_STREAMING + nitf);
    if (s_nch == 8) nitf += append_fn(&len, cfg_audio_fn71, sizeof cfg_audio_fn71, nitf, false);
    else            nitf += append_fn(&len, cfg_audio_fn51, sizeof cfg_audio_fn51, nitf, false);
#if USB_AUDIO51_HID
    nitf += append_fn(&len, cfg_hid_fn, sizeof cfg_hid_fn, nitf, false);
#endif
#if USB_AUDIO51_CDC
    nitf += append_fn(&len, cfg_cdc_fn, sizeof cfg_cdc_fn, nitf, true);
#endif
    const uint8_t head[] = { TUD_CONFIG_DESCRIPTOR(1, nitf, 0, len, 0x00, 100) };
    memcpy(desc_configuration, head, sizeof head);
    desc_configuration[2] = (uint8_t)len; desc_configuration[3] = (uint8_t)(len >> 8);
    return len;
}

/* ---------------------------------------------------------------- media keys */
#if USB_AUDIO51_HID
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) { (void)instance; return desc_hid_report; }
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t id, hid_report_type_t type, uint8_t *buf, uint16_t len)
{ (void)instance; (void)id; (void)type; (void)buf; (void)len; return 0; }
void tud_hid_set_report_cb(uint8_t instance, uint8_t id, hid_report_type_t type, uint8_t const *buf, uint16_t len)
{ (void)instance; (void)id; (void)type; (void)buf; (void)len; }

/* events from any core: multi-producer, so a spin lock guards the write side */
#define MK_Q 16
static uint32_t s_mk_q[MK_Q];               /* usage | pressed << 16 */
static volatile uint32_t s_mk_w, s_mk_r;
static volatile uint32_t s_mk_lock;
#endif
void usb_audio51_media_key(uint16_t usage, bool pressed)
{
#if USB_AUDIO51_HID
    if (s_stereo) return;
    while (__atomic_exchange_n(&s_mk_lock, 1, __ATOMIC_ACQUIRE)) { }
    const uint32_t w = s_mk_w;
    if (w - __atomic_load_n(&s_mk_r, __ATOMIC_ACQUIRE) < MK_Q) {
        s_mk_q[w % MK_Q] = usage | (pressed ? 1u << 16 : 0u);
        __atomic_store_n(&s_mk_w, w + 1, __ATOMIC_RELEASE);
    }
    __atomic_store_n(&s_mk_lock, 0, __ATOMIC_RELEASE);
#else
    (void)usage; (void)pressed;
#endif
}

void usb_audio51_service(void)
{
#if USB_AUDIO51_HID
    static uint16_t down;                    /* key currently held, 0 = none */
    static uint32_t down_ticks;
    if (s_stereo || !tud_mounted()) { s_mk_r = s_mk_w; down = 0; return; }
    if (!tud_hid_ready()) return;
    if (down && ++down_ticks > 12000) {      /* ~3 s at 4 kHz: a lost release, let go */
        uint16_t z = 0; tud_hid_report(0, &z, 2); down = 0; return;
    }
    const uint32_t r = s_mk_r;
    if (r == __atomic_load_n(&s_mk_w, __ATOMIC_ACQUIRE)) return;
    const uint32_t ev = s_mk_q[r % MK_Q];
    const uint16_t usage = (uint16_t)ev;
    const bool pressed = ev >> 16;
    __atomic_store_n(&s_mk_r, r + 1, __ATOMIC_RELEASE);
    if (pressed) {
        if (down && down != usage) { uint16_t z = 0; tud_hid_report(0, &z, 2); down = 0;   /* release the old one first */
                                     __atomic_store_n(&s_mk_r, r, __ATOMIC_RELEASE); return; }
        tud_hid_report(0, &usage, 2); down = usage; down_ticks = 0;
    } else if (down == usage) {
        uint16_t z = 0; tud_hid_report(0, &z, 2); down = 0;
    }
#endif
}

#if USB_AUDIO51_UAC1
static uint8_t const *uac1_configuration(void);
#endif
uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
#if USB_AUDIO51_UAC1
    if (s_stereo) return uac1_configuration();
#endif
    if (!s_first_after_device_seen) { s_first_after_device_seen = true; s_apple_host = USB_AUDIO51_MACOS; }
    const uint16_t len = build_configuration();
    /* find the feedback endpoint descriptor and set its wMaxPacketSize */
    for (size_t i = 0; i + 7 <= len; i += desc_configuration[i] ? desc_configuration[i] : 1) {
        if (desc_configuration[i] == 7 && desc_configuration[i + 1] == TUSB_DESC_ENDPOINT &&
            desc_configuration[i + 2] == (EPNUM_AUDIO_FB | 0x80)) {
            desc_configuration[i + 4] = s_apple_host ? 3 : 4;
            desc_configuration[i + 5] = 0;
            break;
        }
    }
    return desc_configuration;
}

bool tud_audio_feedback_format_correction_cb(uint8_t func_id) { (void)func_id; return s_apple_host; }

static char const *string_desc[] = {
    (const char[]){0x09, 0x04},
    "DIY Audio",                         /* 1 manufacturer */
    "LakeHP 5.1 Headphone Virtualizer",  /* 2 product      */
    "000001",                            /* 3 serial       */
    "LakeHP 5.1",                        /* 4 audio control */
    "5.1 Speakers",                      /* 5 audio streaming */
    "LakeHP debug log",                  /* 6 CDC (debug builds) */
    "LakeHP Stereo",                     /* 7 product in stereo (UAC1) mode */
    "000002",                            /* 8 serial in stereo mode */
    "",                                  /* 9 */
    "LakeHP settings (192.168.20.1)",    /* 10 network adapter */
    "",                                  /* 11 MAC address, filled in below */
    "LakeHP media keys",                 /* 12 HID consumer control (headset buttons) */
    "LakeHP 7.1",                        /* 13 audio control in 7.1 mode */
    "7.1 Speakers",                      /* 14 audio streaming in 7.1 mode */
};

static uint16_t _desc_str[33];
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    if (!s_first_after_device_seen) { s_first_after_device_seen = true; s_apple_host = true; }
    size_t n;
    if (index == 0) { memcpy(&_desc_str[1], string_desc[0], 2); n = 1; }
    else {
        if (index >= sizeof(string_desc) / sizeof(string_desc[0])) return NULL;
#if USB_AUDIO51_NET
        if (index == STR_MAC) {                      /* RNDIS/ECM: MAC as 12 hex digits */
            extern uint8_t tud_network_mac_address[6];
            for (int i = 0; i < 12; i++) _desc_str[1 + i] = "0123456789ABCDEF"[(tud_network_mac_address[i / 2] >> ((i & 1) ? 0 : 4)) & 15];
            _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * 12 + 2));
            return _desc_str;
        }
#endif
        if (s_stereo && index == 2) index = 7;
        if (s_stereo && index == 3) index = 8;
        if (s_nch == 8 && !s_stereo && (index == 4 || index == 5)) index += 9;
        const char *s = string_desc[index];
        n = strlen(s); if (n > 32) n = 32;
        for (size_t i = 0; i < n; i++) _desc_str[1 + i] = s[i];
    }
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    return _desc_str;
}

/* ---------------------------------------------------------------- state */
static int8_t  s_mute;          /* master */
static int16_t s_volume;        /* 1/256 dB */
static volatile bool s_streaming;

__attribute__((weak)) void usb_audio51_on_volume(int16_t v, bool m) { (void)v; (void)m; }
__attribute__((weak)) void usb_audio51_on_stream(bool a) { (void)a; }

#if USB_AUDIO51_UAC1
static uint32_t uac1_available(void);
static uint32_t uac1_read(int16_t *dst, uint32_t max_frames);
#define UAC1_RING 2048u
#endif

uint32_t usb_audio51_frames_available(void)
{
#if USB_AUDIO51_UAC1
    if (s_stereo) return uac1_available();
#endif
    return tud_audio_available() / FRAME_BYTES;
}
bool usb_audio51_streaming(void) { return s_streaming; }
uint32_t usb_audio51_fifo_frames(void)
{
#if USB_AUDIO51_UAC1
    if (s_stereo) return UAC1_RING;
#endif
    return CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / FRAME_BYTES;
}

uint32_t usb_audio51_read(int16_t *dst, uint32_t max_frames)
{
#if USB_AUDIO51_UAC1
    if (s_stereo) return uac1_read(dst, max_frames);
#endif
    uint32_t n = usb_audio51_frames_available();
    if (n > max_frames) n = max_frames;
    if (!n) return 0;
    uint16_t got = tud_audio_read(dst, (uint16_t)(n * FRAME_BYTES));
    return got / FRAME_BYTES;
}

/* ---------------------------------------------------------------- requests */
static bool clock_get(uint8_t rhport, audio_control_request_t const *req)
{
    if (req->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        if (req->bRequest == AUDIO_CS_REQ_CUR) {
            audio_control_cur_4_t cur = { (int32_t)tu_htole32(USB_AUDIO51_RATE) };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)req, &cur, sizeof cur);
        }
        if (req->bRequest == AUDIO_CS_REQ_RANGE) {
            audio_control_range_4_n_t(1) r = { .wNumSubRanges = tu_htole16(1) };
            r.subrange[0].bMin = USB_AUDIO51_RATE;
            r.subrange[0].bMax = USB_AUDIO51_RATE;
            r.subrange[0].bRes = 0;
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)req, &r, sizeof r);
        }
    } else if (req->bControlSelector == AUDIO_CS_CTRL_CLK_VALID && req->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t v = { .bCur = 1 };
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)req, &v, sizeof v);
    }
    return false;
}

static bool feature_get(uint8_t rhport, audio_control_request_t const *req)
{
    if (req->bChannelNumber != 0) return false;
    if (req->bControlSelector == AUDIO_FU_CTRL_MUTE && req->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t m = { .bCur = s_mute };
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)req, &m, sizeof m);
    }
    if (req->bControlSelector == AUDIO_FU_CTRL_VOLUME) {
        if (req->bRequest == AUDIO_CS_REQ_RANGE) {
            audio_control_range_2_n_t(1) r = { .wNumSubRanges = tu_htole16(1) };
            r.subrange[0].bMin = tu_htole16(-50 * 256);
            r.subrange[0].bMax = tu_htole16(0);
            r.subrange[0].bRes = tu_htole16(256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)req, &r, sizeof r);
        }
        if (req->bRequest == AUDIO_CS_REQ_CUR) {
            audio_control_cur_2_t v = { .bCur = tu_htole16(s_volume) };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)req, &v, sizeof v);
        }
    }
    return false;
}

/* ---------------------------------------------------------------- debug trace */
#if USB_AUDIO51_CDC
#include <stdio.h>
#define TRACE(...) printf(__VA_ARGS__)
#else
#define TRACE(...) do { } while (0)
#endif
static volatile uint32_t s_rx_pkts, s_rx_bytes, s_fb_pkts;
void usb_audio51_debug_counters(uint32_t *rx_pkts, uint32_t *rx_bytes, uint32_t *fb_pkts)
{
    *rx_pkts = s_rx_pkts; *rx_bytes = s_rx_bytes; *fb_pkts = s_fb_pkts;
}
bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n, uint8_t func_id, uint8_t ep, uint8_t alt)
{
    (void)rhport; (void)func_id; (void)ep; (void)alt;
    s_rx_pkts++; s_rx_bytes += n;
    return true;
}
void tud_audio_fb_done_cb(uint8_t func_id) { (void)func_id; s_fb_pkts++; }

static bool set_req_entity(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf);

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    audio_control_request_t const *req = (audio_control_request_t const *)p_request;
    bool ok = false;
    if (req->bEntityID == ENT_CLOCK) ok = clock_get(rhport, req);
    else if (req->bEntityID == ENT_FEATURE) ok = feature_get(rhport, req);
    TRACE("USB: GET %s entity %u sel %u ch %u len %u -> %s\n", req->bRequest == AUDIO_CS_REQ_CUR ? "CUR" : "RANGE",
          req->bEntityID, req->bControlSelector, req->bChannelNumber, (unsigned)req->wLength, ok ? "ok" : "STALL");
    return ok;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
    audio_control_request_t const *req = (audio_control_request_t const *)p_request;
    bool ok = set_req_entity(rhport, p_request, buf);
    TRACE("USB: SET entity %u sel %u ch %u len %u -> %s\n", req->bEntityID, req->bControlSelector,
          req->bChannelNumber, (unsigned)req->wLength, ok ? "ok" : "STALL");
    return ok;
}

/* Requests addressed to an interface or endpoint: none are supported, log them. */
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *r)
{ (void)rhport; TRACE("USB: GET itf req %02x val %04x idx %04x -> STALL\n", r->bRequest, r->wValue, r->wIndex); return false; }
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *r, uint8_t *b)
{ (void)rhport; (void)b; TRACE("USB: SET itf req %02x val %04x idx %04x -> STALL\n", r->bRequest, r->wValue, r->wIndex); return false; }
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *r)
{ (void)rhport; TRACE("USB: GET ep req %02x val %04x idx %04x -> STALL\n", r->bRequest, r->wValue, r->wIndex); return false; }
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *r, uint8_t *b)
{ (void)rhport; (void)b; TRACE("USB: SET ep req %02x val %04x idx %04x -> STALL\n", r->bRequest, r->wValue, r->wIndex); return false; }

static bool set_req_entity(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
    (void)rhport;
    audio_control_request_t const *req = (audio_control_request_t const *)p_request;
    if (req->bRequest != AUDIO_CS_REQ_CUR) return false;
    if (req->bEntityID == ENT_CLOCK && req->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        TU_VERIFY(req->wLength == sizeof(audio_control_cur_4_t));
        return ((audio_control_cur_4_t const *)buf)->bCur == USB_AUDIO51_RATE;
    }
    if (req->bEntityID == ENT_FEATURE && req->bChannelNumber == 0) {
        if (req->bControlSelector == AUDIO_FU_CTRL_MUTE) {
            TU_VERIFY(req->wLength == sizeof(audio_control_cur_1_t));
            s_mute = ((audio_control_cur_1_t const *)buf)->bCur;
            usb_audio51_on_volume(s_volume, s_mute);
            return true;
        }
        if (req->bControlSelector == AUDIO_FU_CTRL_VOLUME) {
            TU_VERIFY(req->wLength == sizeof(audio_control_cur_2_t));
            s_volume = ((audio_control_cur_2_t const *)buf)->bCur;
            usb_audio51_on_volume(s_volume, s_mute);
            return true;
        }
    }
    return false;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));
    TRACE("USB: set interface %u alt %u (host looks like %s)\n", itf, alt, s_apple_host ? "Apple" : "Windows/Linux");
    if (itf == s_itf_as && alt != 0) { s_streaming = true; usb_audio51_on_stream(true); }
    return true;
}

#if CFG_TUSB_MCU == OPT_MCU_RP2040
/* RP2040/RP2350 workaround. TinyUSB 0.18's audio class never closes its iso
 * endpoints on this port (it pre-allocates them, and the RP2 driver has no
 * per-endpoint close). When the host switches the stream off, the OUT and
 * feedback buffers stay armed in hardware, and the next time the host starts
 * the stream the driver re-arms them and panics with "ep 01 was already
 * available", freezing the whole device. Windows always opens, closes and
 * reopens the stream when playback starts, so this hit on the first sound.
 * Fix: abort both endpoints and clear their buffer-control words on close. */
#include "hardware/structs/usb.h"
#include "hardware/address_mapped.h"
static void rp2_disarm_audio_eps(void)
{
    const uint32_t mask = (1u << (2 * EPNUM_AUDIO_OUT + 1))      /* EP1 OUT */
                        | (1u << (2 * EPNUM_AUDIO_FB));          /* EP1 IN (feedback) */
    hw_set_bits(&usb_hw->abort, mask);
    for (int i = 0; i < 10000 && (usb_hw->abort_done & mask) != mask; i++) { }
    usb_dpram->ep_buf_ctrl[EPNUM_AUDIO_OUT].out = 0;
    usb_dpram->ep_buf_ctrl[EPNUM_AUDIO_FB].in = 0;
    hw_clear_bits(&usb_hw->abort_done, mask);
    hw_clear_bits(&usb_hw->abort, mask);
}
#endif

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
#if CFG_TUSB_MCU == OPT_MCU_RP2040
    rp2_disarm_audio_eps();
#endif
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));
    TRACE("USB: close endpoints, interface %u -> alt %u\n", itf, alt);
    if (itf == s_itf_as && alt == 0) { s_streaming = false; usb_audio51_on_stream(false); }
    return true;
}

void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t *fb)
{
    (void)func_id; (void)alt_itf;
    fb->method = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    fb->sample_freq = USB_AUDIO51_RATE;
}

#if USB_AUDIO51_UAC1
/* ======================================================================
 * Stereo mode: a USB Audio Class 1.0 stereo speaker (48 kHz, 16-bit).
 *
 * Consoles (Nintendo Switch / Switch 2, PlayStation) and some other hosts
 * only drive UAC1 devices, and TinyUSB 0.18 has no UAC1 class, so this is a
 * small app-level class driver. The stereo stream feeds the headphone engine's
 * front left/right inputs (virtual front speakers in the room).
 *
 * The endpoint is "adaptive": the host sends at its own clock with no
 * feedback, so the small difference to the Bluetooth clock is absorbed by
 * dropping or repeating one frame when the ring drifts away from its target
 * (at most once per 50 ms; real drift needs one every few seconds).
 * ====================================================================== */
#include "device/usbd_pvt.h"

#define UAC1_EP_OUT     0x01
#define UAC1_EP_SIZE    196                      /* 49 frames x 2 ch x 2 bytes */
#define UAC1_TARGET     576u                     /* ring fill to aim for (12 ms) */
#define UAC1_SLACK      192u
#define UAC1_ENT_FU     2
#define UAC1_AC_CS_LEN  (9 + 12 + 10 + 9)        /* header + input term + feature unit + output term */
#define UAC1_ITF_TOTAL  (2 + (USB_AUDIO51_CDC ? 2 : 0))
#define UAC1_CFG_LEN    (9 + 9 + UAC1_AC_CS_LEN + 9 + 9 + 7 + 11 + 9 + 7 + CDC_DESC_LEN)

static uint8_t const desc_cfg_uac1[] = {
    TUD_CONFIG_DESCRIPTOR(1, UAC1_ITF_TOTAL, 0, UAC1_CFG_LEN, 0x00, 100),
    /* interface 0: audio control */
    9, TUSB_DESC_INTERFACE, 0, 0, 0, TUSB_CLASS_AUDIO, 0x01, 0x00, 4,
    9, TUSB_DESC_CS_INTERFACE, 0x01, U16_TO_U8S_LE(0x0100), U16_TO_U8S_LE(UAC1_AC_CS_LEN), 1, 1,
    12, TUSB_DESC_CS_INTERFACE, 0x02, 1, U16_TO_U8S_LE(0x0101), 0, 2, U16_TO_U8S_LE(0x0003), 0, 0,
    10, TUSB_DESC_CS_INTERFACE, 0x06, UAC1_ENT_FU, 1, 1, 0x03, 0x00, 0x00, 0,
    9, TUSB_DESC_CS_INTERFACE, 0x03, 3, U16_TO_U8S_LE(0x0302), 0, UAC1_ENT_FU, 0,
    /* interface 1: audio streaming, alt 0 = idle, alt 1 = 48 kHz 16-bit stereo */
    9, TUSB_DESC_INTERFACE, 1, 0, 0, TUSB_CLASS_AUDIO, 0x02, 0x00, 5,
    9, TUSB_DESC_INTERFACE, 1, 1, 1, TUSB_CLASS_AUDIO, 0x02, 0x00, 5,
    7, TUSB_DESC_CS_INTERFACE, 0x01, 1, 1, U16_TO_U8S_LE(0x0001),
    11, TUSB_DESC_CS_INTERFACE, 0x02, 1, 2, 2, 16, 1, 0x80, 0xBB, 0x00,
    9, TUSB_DESC_ENDPOINT, UAC1_EP_OUT, 0x09 /* iso, adaptive */, U16_TO_U8S_LE(UAC1_EP_SIZE), 1, 0, 0,
    7, 0x25, 0x01, 0x01 /* sampling-frequency control */, 0, U16_TO_U8S_LE(0),
#if USB_AUDIO51_CDC
    TUD_CDC_DESCRIPTOR(2, 6, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
#endif
};
TU_VERIFY_STATIC(sizeof(desc_cfg_uac1) == UAC1_CFG_LEN, "UAC1 descriptor length mismatch");
static uint8_t const *uac1_configuration(void) { return desc_cfg_uac1; }

static struct {
    tusb_desc_endpoint_t const *ep_desc;
    uint8_t  alt;
    bool     active, primed;
    uint32_t w, r, since_adj;
    uint32_t drops, dups, overflows;
    int16_t  ring[UAC1_RING * 2];
} u1;
static uint8_t u1_buf[UAC1_EP_SIZE] __attribute__((aligned(4)));
static uint8_t u1_ctrl[4] __attribute__((aligned(4)));

void usb_audio51_stereo_counters(uint32_t *drops, uint32_t *dups, uint32_t *overflows)
{
    *drops = u1.drops; *dups = u1.dups; *overflows = u1.overflows;
}

/* Reader side (usb_pump), same context as the TinyUSB task. */
static uint32_t uac1_available(void)
{
    uint32_t level = u1.w - u1.r;
    if (!u1.primed) {                       /* start/restart only once the ring holds the target */
        if (level < UAC1_TARGET) return 0;
        u1.primed = true;
    }
    return level;
}

static uint32_t uac1_read(int16_t *dst, uint32_t max_frames)
{
    uint32_t n = uac1_available();
    if (n > max_frames) n = max_frames;
    for (uint32_t i = 0; i < n; i++) {
        const int16_t *f = &u1.ring[((u1.r + i) % UAC1_RING) * 2];
        int16_t *o = &dst[i * USB_AUDIO51_CHANNELS];
        o[0] = f[0]; o[1] = f[1];           /* FL FR */
        o[2] = o[3] = o[4] = o[5] = 0;      /* FC LFE BL BR */
    }
    u1.r += n;
    u1.since_adj += n;
    if (u1.since_adj >= 2400) {             /* drift correction, at most every 50 ms */
        u1.since_adj = 0;
        uint32_t level = u1.w - u1.r;
        if (level > UAC1_TARGET + UAC1_SLACK) { u1.r++; u1.drops++; }
        else if (level + UAC1_SLACK < UAC1_TARGET && n) { u1.r--; u1.dups++; }
    }
    if (u1.w == u1.r) u1.primed = false;    /* ran dry: refill before resuming */
    return n;
}

#if CFG_TUSB_MCU == OPT_MCU_RP2040
static void uac1_rp2_disarm(void)          /* same RP2 issue as the UAC2 path */
{
    const uint32_t mask = 1u << (2 * UAC1_EP_OUT + 1);
    hw_set_bits(&usb_hw->abort, mask);
    for (int i = 0; i < 10000 && (usb_hw->abort_done & mask) != mask; i++) { }
    usb_dpram->ep_buf_ctrl[UAC1_EP_OUT].out = 0;
    hw_clear_bits(&usb_hw->abort_done, mask);
    hw_clear_bits(&usb_hw->abort, mask);
}
#endif

static void uac1_init(void) { memset(&u1, 0, sizeof u1); }
static void uac1_reset(uint8_t rhport)
{
    (void)rhport;
    if (u1.active) { s_streaming = false; usb_audio51_on_stream(false); }
    memset(&u1, 0, sizeof u1);
}

static uint16_t uac1_open(uint8_t rhport, tusb_desc_interface_t const *itf, uint16_t max_len)
{
    if (!s_stereo || itf->bInterfaceClass != TUSB_CLASS_AUDIO || itf->bInterfaceProtocol != 0) return 0;
    uint8_t const *p = (uint8_t const *)itf;
    uint16_t len = 0;
    if (itf->bInterfaceSubClass == 0x01) {                 /* control: interface + CS descriptors */
        len = tu_desc_len(p); p = tu_desc_next(p);
        while (len < max_len && tu_desc_type(p) == TUSB_DESC_CS_INTERFACE) { len += tu_desc_len(p); p = tu_desc_next(p); }
        return len;
    }
    if (itf->bInterfaceSubClass != 0x02) return 0;
    while (len < max_len) {                                /* streaming: all its alt settings */
        if (len && tu_desc_type(p) == TUSB_DESC_INTERFACE &&
            ((tusb_desc_interface_t const *)p)->bInterfaceNumber != itf->bInterfaceNumber) break;
        if (len && tu_desc_type(p) == TUSB_DESC_INTERFACE_ASSOCIATION) break;
        if (tu_desc_type(p) == TUSB_DESC_ENDPOINT) {
            u1.ep_desc = (tusb_desc_endpoint_t const *)p;
#ifdef TUP_DCD_EDPT_ISO_ALLOC
            usbd_edpt_iso_alloc(rhport, UAC1_EP_OUT, UAC1_EP_SIZE);
#endif
        }
        len += tu_desc_len(p); p = tu_desc_next(p);
    }
    return len;
}

static void uac1_set_alt(uint8_t rhport, uint8_t alt)
{
    TRACE("USB: stereo set interface 1 alt %u\n", alt);
    if (u1.active) {
#if CFG_TUSB_MCU == OPT_MCU_RP2040
        uac1_rp2_disarm();
#endif
#ifndef TUP_DCD_EDPT_ISO_ALLOC
        usbd_edpt_close(rhport, UAC1_EP_OUT);
#endif
        u1.active = false;
        s_streaming = false; usb_audio51_on_stream(false);
    }
    u1.alt = alt;
    if (alt == 1 && u1.ep_desc) {
#ifdef TUP_DCD_EDPT_ISO_ALLOC
        usbd_edpt_iso_activate(rhport, u1.ep_desc);
#else
        usbd_edpt_open(rhport, u1.ep_desc);
#endif
        usbd_edpt_clear_stall(rhport, UAC1_EP_OUT);
        u1.w = u1.r = 0; u1.primed = false; u1.since_adj = 0;
        u1.active = USB_AUDIO51_EDPT_XFER(rhport, UAC1_EP_OUT, u1_buf, UAC1_EP_SIZE);
        s_streaming = u1.active; usb_audio51_on_stream(u1.active);
    }
}

static bool uac1_control(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req)
{
    const uint8_t type = req->bmRequestType_bit.type, rcpt = req->bmRequestType_bit.recipient;
    const uint8_t cs = tu_u16_high(req->wValue), ch = tu_u16_low(req->wValue);

    if (type == TUSB_REQ_TYPE_STANDARD && rcpt == TUSB_REQ_RCPT_INTERFACE) {
        const uint8_t itf = tu_u16_low(req->wIndex);
        if (req->bRequest == TUSB_REQ_SET_INTERFACE) {
            if (stage == CONTROL_STAGE_SETUP) {
                if (itf == 1) uac1_set_alt(rhport, (uint8_t)req->wValue);
                tud_control_status(rhport, req);
            }
            return true;
        }
        if (req->bRequest == TUSB_REQ_GET_INTERFACE) {
            if (stage == CONTROL_STAGE_SETUP) { u1_ctrl[0] = itf == 1 ? u1.alt : 0; tud_control_xfer(rhport, req, u1_ctrl, 1); }
            return true;
        }
        return false;
    }
    if (type != TUSB_REQ_TYPE_CLASS) return false;

    const bool get = req->bmRequestType_bit.direction == TUSB_DIR_IN;
    if (rcpt == TUSB_REQ_RCPT_INTERFACE && tu_u16_high(req->wIndex) == UAC1_ENT_FU) {
        if (get) {
            if (stage != CONTROL_STAGE_SETUP) return true;
            int16_t v;
            if (cs == 0x01 && req->bRequest == 0x81) { u1_ctrl[0] = (uint8_t)s_mute; return tud_control_xfer(rhport, req, u1_ctrl, 1); }
            if (cs != 0x02) return false;
            switch (req->bRequest) {
            case 0x81: v = s_volume; break;           /* GET_CUR */
            case 0x82: v = -50 * 256; break;          /* GET_MIN */
            case 0x83: v = 0; break;                  /* GET_MAX */
            case 0x84: v = 256; break;                /* GET_RES: 1 dB */
            default: return false;
            }
            u1_ctrl[0] = (uint8_t)v; u1_ctrl[1] = (uint8_t)(v >> 8);
            return tud_control_xfer(rhport, req, u1_ctrl, 2);
        }
        if (req->bRequest != 0x01 || req->wLength > sizeof u1_ctrl || (cs != 0x01 && cs != 0x02)) return false;
        if (stage == CONTROL_STAGE_SETUP) return tud_control_xfer(rhport, req, u1_ctrl, req->wLength);
        if (stage == CONTROL_STAGE_DATA) {
            if (ch != 0 && ch != 0xFF) return true;   /* only master controls exist */
            if (cs == 0x01) s_mute = (int8_t)u1_ctrl[0];
            else s_volume = (int16_t)(u1_ctrl[0] | (u1_ctrl[1] << 8));
            usb_audio51_on_volume(s_volume, s_mute);
        }
        return true;
    }
    if (rcpt == TUSB_REQ_RCPT_ENDPOINT && cs == 0x01) {    /* sampling frequency: only 48 kHz */
        if (stage != CONTROL_STAGE_SETUP) return true;
        if (get) {
            if (req->bRequest != 0x81) return false;
            u1_ctrl[0] = 0x80; u1_ctrl[1] = 0xBB; u1_ctrl[2] = 0x00;
            return tud_control_xfer(rhport, req, u1_ctrl, 3);
        }
        if (req->bRequest != 0x01 || req->wLength > sizeof u1_ctrl) return false;
        return tud_control_xfer(rhport, req, u1_ctrl, req->wLength);
    }
    TRACE("USB: stereo request %02x type %u rcpt %u val %04x idx %04x -> STALL\n",
          req->bRequest, type, rcpt, req->wValue, req->wIndex);
    return false;
}

static bool uac1_xfer(uint8_t rhport, uint8_t ep, xfer_result_t result, uint32_t n)
{
    if (ep != UAC1_EP_OUT) return false;
    if (result == XFER_RESULT_SUCCESS) {
        s_rx_pkts++; s_rx_bytes += n;
        const int16_t *in = (const int16_t *)u1_buf;
        for (uint32_t i = 0; i < n / 4; i++) {
            if (u1.w - u1.r >= UAC1_RING) { u1.overflows++; break; }
            int16_t *f = &u1.ring[(u1.w % UAC1_RING) * 2];
            f[0] = in[2 * i]; f[1] = in[2 * i + 1];
            u1.w++;
        }
    }
    if (u1.active) USB_AUDIO51_EDPT_XFER(rhport, UAC1_EP_OUT, u1_buf, UAC1_EP_SIZE);
    return true;
}

static usbd_class_driver_t const s_uac1_driver = {
    .name = "UAC1",
    .init = uac1_init,
    .deinit = NULL,
    .reset = uac1_reset,
    .open = uac1_open,
    .control_xfer_cb = uac1_control,
    .xfer_cb = uac1_xfer,
    .sof = NULL,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *count)
{
    *count = 1;
    return &s_uac1_driver;
}
#endif /* USB_AUDIO51_UAC1 */
