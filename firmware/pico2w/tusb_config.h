#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS               OPT_OS_PICO
#endif
#define CFG_TUSB_DEBUG            0
#define CFG_TUD_ENABLED           1
#define CFG_TUD_MAX_SPEED         OPT_MODE_FULL_SPEED
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))
#define CFG_TUD_ENDPOINT0_SIZE    64

#include "usb_audio51_tusb_config.h"

#endif
