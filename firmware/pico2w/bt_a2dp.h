/*
 * bt_a2dp - A2DP source for the Pico 2 W (BTstack + CYW43439, Classic BT).
 *
 * Core 1 (BTstack/CYW43 background IRQ): discovery, pairing, stream control,
 *   and the media timer that packs SBC frames into RTP packets.
 * Core 0 main loop: bt_a2dp_encoder_service() pulls PCM from the DSP pipeline,
 *   encodes one SBC frame and hands it to core 1 through a lock-free ring.
 */
#ifndef BT_A2DP_H
#define BT_A2DP_H
#include <stdbool.h>
#include "dh_pipeline.h"

typedef enum {
    BT_OFF = 0,
    BT_SCANNING,       /* looking for a headset in pairing mode */
    BT_CONNECTING,
    BT_CONNECTED,      /* signalling up, stream not running     */
    BT_STREAMING,
} bt_state_t;

void       bt_a2dp_init(void);             /* on the core that ran cyw43_arch_init() */
bt_state_t bt_a2dp_state(void);
bool       bt_a2dp_streaming(void);
void       bt_a2dp_forget_and_scan(void);
void       bt_a2dp_peer_info(char addr[20], int *rate, int *bitpool); /* for the settings page */  /* any thread                       */

/* Headset buttons (AVRCP pass-through) as HID consumer usages: 0xCD play/pause,
 * 0xB5 next, 0xB6 previous, 0xB7 stop, 0xB3/0xB4 fast forward/rewind,
 * 0xE9/0xEA volume up/down, 0xE2 mute. Called on the BTstack core; weak
 * default does nothing. */
void       bt_a2dp_media_key_cb(uint16_t usage, bool pressed);

bool       bt_a2dp_encoder_service(dhp_t *p); /* one thread only; true if a frame was encoded */
void       bt_a2dp_debug_print(void (*out)(const char *fmt, ...)); /* one status line */

#endif
