/*
 * usbout - core 1 of the RP2350-USB-A build: the PIO-USB host on the USB-A
 * port and the USB audio output to a DAC or 2.4 GHz headset dongle.
 *
 * Picks a playback configuration (48 kHz stereo, 16-bit preferred; 24/32-bit
 * and multichannel accepted; 44.1 kHz through the resampler as a last resort),
 * unmutes it, sets its volume to 0 dB (or its maximum below that), and keeps
 * TinyUSB's playback FIFO topped up from the pipeline's stereo output. The
 * device's clock (feedback endpoint) or ours (adaptive / synchronous devices)
 * sets the drain rate, and the USB input side locks the PC to it.
 * With no device attached the pipeline is drained at 48 kHz so the PC side
 * keeps running.
 */
#ifndef USBOUT_H
#define USBOUT_H
#include <stdint.h>
#include <stdbool.h>
#include "dh_pipeline.h"

typedef enum {
    USBOUT_NONE = 0,       /* nothing plugged in (or not a playback device) */
    USBOUT_STARTING,       /* mounted, configuring / starting the stream    */
    USBOUT_STREAMING,
    USBOUT_UNSUPPORTED,    /* audio device without a usable playback format */
    USBOUT_FAILED,         /* stream failed; retrying                        */
} usbout_state_t;

typedef struct {
    usbout_state_t state;
    uint16_t vid, pid;
    uint32_t rate;
    uint8_t  channels, bits, container;   /* container: bytes per sample   */
    bool     resampled;                   /* 44.1 kHz device: resampler on */
    char     name[40];
    uint32_t packets;                     /* isochronous packets sent       */
    uint32_t fifo_dry;                    /* times the playback FIFO ran low */
} usbout_info_t;

void usbout_core1_main(dhp_t *pipe);      /* never returns */
usbout_state_t usbout_state(void);
void usbout_get_info(usbout_info_t *out); /* any core */
void usbout_restart(void);                /* any core: stop and set the stream up again */
void usbout_set_tone(bool on);            /* any core: 1 kHz test tone straight to the output */
bool usbout_tone(void);

#endif
