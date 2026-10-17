#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_
#include "sdkconfig.h"

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif
#define CFG_TUSB_RHPORT0_MODE     (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_OS               OPT_OS_FREERTOS
#define CFG_TUSB_OS_INC_PATH      freertos/
#ifndef ESP_PLATFORM
#define ESP_PLATFORM 1
#endif
#define CFG_TUSB_DEBUG            0
#define CFG_TUD_ENABLED           1
#define CFG_TUD_MAX_SPEED         OPT_MODE_FULL_SPEED
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))
#define CFG_TUD_ENDPOINT0_SIZE    64

#include "usb_audio51_tusb_config.h"
#endif
