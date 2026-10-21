#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_
/* RP2350-USB-A: device on the native USB-C port (rhport 0), host on the
 * PIO-USB USB-A port (rhport 1). */
#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS               OPT_OS_PICO
#endif
#undef  CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG            0
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))

/* ---- device: the sound card the PC sees */
#define CFG_TUD_ENABLED           1
#define BOARD_TUD_RHPORT          0
#define CFG_TUD_MAX_SPEED         OPT_MODE_FULL_SPEED
#define CFG_TUD_ENDPOINT0_SIZE    64
#include "usb_audio51_tusb_config.h"

/* ---- host: the USB DAC / wireless headset dongle */
#define CFG_TUH_ENABLED           1
#define CFG_TUH_RPI_PIO_USB       1
#define BOARD_TUH_RHPORT          1
#define CFG_TUH_MAX_SPEED         OPT_MODE_FULL_SPEED
#define CFG_TUH_ENUMERATION_BUFSIZE 512
#define CFG_TUH_HUB               1      /* a hub in between works too */
#define CFG_TUH_DEVICE_MAX        4
#define CFG_TUH_CDC               0
#define CFG_TUH_HID               0
#define CFG_TUH_MSC               0
#define CFG_TUH_VENDOR            0
#define CFG_TUH_AUDIO             1
#define CFG_TUH_AUDIO_PROTOCOLS   (TUH_AUDIO_PROTOCOL_UAC1 | TUH_AUDIO_PROTOCOL_UAC2)
#define CFG_TUH_AUDIO_MAX         1
#define CFG_TUH_AUDIO_MAX_SAM_FREQ 8
#define CFG_TUH_AUDIO_MAX_AS      6
#define CFG_TUH_AUDIO_EPIN_BUFSIZE  64   /* feedback / small captures only */
#define CFG_TUH_AUDIO_EPOUT_BUFSIZE 400  /* one 48 kHz stereo packet, up to 32-bit samples */
#define CFG_TUH_AUDIO_STREAM_BUFSIZE 2048
#endif
