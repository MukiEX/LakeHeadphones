/*
 * usb_audio51 - TinyUSB (0.18) UAC2 5.1/7.1 speaker: 6 or 8 ch, 48 kHz, 16-bit,
 * async isochronous OUT with feedback endpoint. Shared by the Pico 2 W and the
 * ESP32-S3 builds. Channel order on the wire: FL FR FC LFE BL BR [SL SR].
 *
 * Include usb_audio51_tusb_config.h from your tusb_config.h.
 */
#ifndef USB_AUDIO51_H
#define USB_AUDIO51_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_AUDIO51_RATE       48000
#define USB_AUDIO51_CHANNELS   6               /* default (5.1) */
#define USB_AUDIO51_MAX_CHANNELS 8             /* 7.1 */

/* 6 = 5.1 (default) or 8 = 7.1. Call before tud_init(). */
void usb_audio51_set_channels(int nch);
int  usb_audio51_channels(void);   /* frame width usb_audio51_read() returns */
/* Read up to max_frames whole frames from the TinyUSB FIFO. */
uint32_t usb_audio51_read(int16_t *dst, uint32_t max_frames);
uint32_t usb_audio51_frames_available(void);
bool     usb_audio51_streaming(void);     /* host has selected the streaming alt setting */
uint32_t usb_audio51_fifo_frames(void);   /* capacity of the TinyUSB FIFO in frames */
/* Stereo mode (builds with USB_AUDIO51_UAC1=1): enumerate as a USB Audio
 * Class 1.0 stereo speaker instead of the UAC2 5.1 device, for consoles.
 * Call before tud_init(). usb_audio51_read() still returns 6-channel frames
 * (L, R and four zero channels). */
void usb_audio51_set_stereo(bool on);
/* Builds with USB_AUDIO51_NET=1: add the settings network adapter (RNDIS) to
 * the 5.1 configuration. Call before tud_init(). */
void usb_audio51_set_network(bool on);
bool usb_audio51_has_network(void);
bool usb_audio51_is_stereo(void);
void usb_audio51_stereo_counters(uint32_t *drops, uint32_t *dups, uint32_t *overflows);
/* Builds with USB_AUDIO51_HID=1 (5.1/7.1 mode only): a media-key interface
 * (HID consumer control). usage = HID consumer usage, e.g. 0xCD play/pause,
 * 0xB5 next, 0xB6 previous, 0xB7 stop, 0xE9/0xEA volume, 0xE2 mute.
 * Safe from any thread or core; reports go out from usb_audio51_service(). */
void usb_audio51_media_key(uint16_t usage, bool pressed);
/* Call from the TinyUSB task context, after tud_task(). */
void usb_audio51_service(void);
/* true if enumeration looked like macOS/iOS (feedback sent as 10.14). */
bool usb_audio51_host_is_apple(void);
/* Debug: isochronous packets/bytes received and feedback packets sent so far. */
void usb_audio51_debug_counters(uint32_t *rx_pkts, uint32_t *rx_bytes, uint32_t *fb_pkts);

/* Called by the module when the host changes master volume / mute.
 * vol is in 1/256 dB (UAC2), range -50 dB..0 dB. Weak default does nothing. */
void usb_audio51_on_volume(int16_t vol_db_q8, bool mute);
/* Called when streaming starts/stops. Weak default does nothing. */
void usb_audio51_on_stream(bool active);

#ifdef __cplusplus
}
#endif
#endif
