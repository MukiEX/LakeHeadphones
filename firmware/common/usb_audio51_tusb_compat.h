/*
 * Lets usb_audio51.c build against both the TinyUSB in pico-sdk 2.3 (0.18)
 * and TinyUSB 0.20+ (used by the RP2350-USB-A build for its audio host
 * class), which renamed the UAC2 names to AUDIO20_* / TUD_AUDIO20_*,
 * dropped audio_control_request_t and gave usbd_edpt_xfer() an is_isr flag.
 * Include after tusb.h.
 */
#ifndef USB_AUDIO51_TUSB_COMPAT_H
#define USB_AUDIO51_TUSB_COMPAT_H
#include "tusb.h"

#if TUSB_VERSION_NUMBER >= 2000
#define USB_AUDIO51_TUSB_NEW 1

#define AUDIO_CHANNEL_CONFIG_FRONT_LEFT        AUDIO20_CHANNEL_CONFIG_FRONT_LEFT
#define AUDIO_CHANNEL_CONFIG_FRONT_RIGHT       AUDIO20_CHANNEL_CONFIG_FRONT_RIGHT
#define AUDIO_CHANNEL_CONFIG_FRONT_CENTER      AUDIO20_CHANNEL_CONFIG_FRONT_CENTER
#define AUDIO_CHANNEL_CONFIG_LOW_FRQ_EFFECTS   AUDIO20_CHANNEL_CONFIG_LOW_FRQ_EFFECTS
#define AUDIO_CHANNEL_CONFIG_BACK_LEFT         AUDIO20_CHANNEL_CONFIG_BACK_LEFT
#define AUDIO_CHANNEL_CONFIG_BACK_RIGHT        AUDIO20_CHANNEL_CONFIG_BACK_RIGHT
#define AUDIO_CHANNEL_CONFIG_SIDE_LEFT         AUDIO20_CHANNEL_CONFIG_SIDE_LEFT
#define AUDIO_CHANNEL_CONFIG_SIDE_RIGHT        AUDIO20_CHANNEL_CONFIG_SIDE_RIGHT
#define AUDIO_CLOCK_SOURCE_ATT_INT_FIX_CLK     AUDIO20_CLOCK_SOURCE_ATT_INT_FIX_CLK
#define AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS    AUDIO20_CLOCK_SOURCE_CTRL_CLK_FRQ_POS
#define AUDIO_CS_AC_INTERFACE_FEATURE_UNIT     AUDIO20_CS_AC_INTERFACE_FEATURE_UNIT
#define AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS AUDIO20_CS_AS_INTERFACE_CTRL_LATENCY_POS
#define AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK   AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK
#define AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC
#define AUDIO_CS_CTRL_CLK_VALID                AUDIO20_CS_CTRL_CLK_VALID
#define AUDIO_CS_CTRL_SAM_FREQ                 AUDIO20_CS_CTRL_SAM_FREQ
#define AUDIO_CS_REQ_CUR                       AUDIO20_CS_REQ_CUR
#define AUDIO_CS_REQ_RANGE                     AUDIO20_CS_REQ_RANGE
#define AUDIO_CTRL_NONE                        AUDIO20_CTRL_NONE
#define AUDIO_CTRL_R                           AUDIO20_CTRL_R
#define AUDIO_CTRL_RW                          AUDIO20_CTRL_RW
#define AUDIO_DATA_FORMAT_TYPE_I_PCM           AUDIO20_DATA_FORMAT_TYPE_I_PCM
#define AUDIO_FEATURE_UNIT_CTRL_MUTE_POS       AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS
#define AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS     AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS
#define AUDIO_FORMAT_TYPE_I                    AUDIO20_FORMAT_TYPE_I
#define AUDIO_FUNC_DESKTOP_SPEAKER             AUDIO20_FUNC_DESKTOP_SPEAKER
#define AUDIO_FU_CTRL_MUTE                     AUDIO20_FU_CTRL_MUTE
#define AUDIO_FU_CTRL_VOLUME                   AUDIO20_FU_CTRL_VOLUME

#define TUD_AUDIO_DESC_IAD                     TUD_AUDIO20_DESC_IAD
#define TUD_AUDIO_DESC_IAD_LEN                 TUD_AUDIO20_DESC_IAD_LEN
#define TUD_AUDIO_DESC_STD_AC                  TUD_AUDIO20_DESC_STD_AC
#define TUD_AUDIO_DESC_STD_AC_LEN              TUD_AUDIO20_DESC_STD_AC_LEN
#define TUD_AUDIO_DESC_CS_AC                   TUD_AUDIO20_DESC_CS_AC
#define TUD_AUDIO_DESC_CS_AC_LEN               TUD_AUDIO20_DESC_CS_AC_LEN
#define TUD_AUDIO_DESC_CLK_SRC                 TUD_AUDIO20_DESC_CLK_SRC
#define TUD_AUDIO_DESC_CLK_SRC_LEN             TUD_AUDIO20_DESC_CLK_SRC_LEN
#define TUD_AUDIO_DESC_INPUT_TERM              TUD_AUDIO20_DESC_INPUT_TERM
#define TUD_AUDIO_DESC_INPUT_TERM_LEN          TUD_AUDIO20_DESC_INPUT_TERM_LEN
#define TUD_AUDIO_DESC_OUTPUT_TERM             TUD_AUDIO20_DESC_OUTPUT_TERM
#define TUD_AUDIO_DESC_OUTPUT_TERM_LEN         TUD_AUDIO20_DESC_OUTPUT_TERM_LEN
#define TUD_AUDIO_DESC_STD_AS_INT              TUD_AUDIO20_DESC_STD_AS_INT
#define TUD_AUDIO_DESC_STD_AS_INT_LEN          TUD_AUDIO20_DESC_STD_AS_LEN
#define TUD_AUDIO_DESC_CS_AS_INT               TUD_AUDIO20_DESC_CS_AS_INT
#define TUD_AUDIO_DESC_CS_AS_INT_LEN           TUD_AUDIO20_DESC_CS_AS_INT_LEN
#define TUD_AUDIO_DESC_TYPE_I_FORMAT           TUD_AUDIO20_DESC_TYPE_I_FORMAT
#define TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN       TUD_AUDIO20_DESC_TYPE_I_FORMAT_LEN
#define TUD_AUDIO_DESC_STD_AS_ISO_EP           TUD_AUDIO20_DESC_STD_AS_ISO_EP
#define TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN       TUD_AUDIO20_DESC_STD_AS_ISO_EP_LEN
#define TUD_AUDIO_DESC_CS_AS_ISO_EP            TUD_AUDIO20_DESC_CS_AS_ISO_EP
#define TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN        TUD_AUDIO20_DESC_CS_AS_ISO_EP_LEN
#define TUD_AUDIO_DESC_STD_AS_ISO_FB_EP        TUD_AUDIO20_DESC_STD_AS_ISO_FB_EP
#define TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_LEN    TUD_AUDIO20_DESC_STD_AS_ISO_FB_EP_LEN

typedef audio20_control_cur_1_t audio_control_cur_1_t;
typedef audio20_control_cur_2_t audio_control_cur_2_t;
typedef audio20_control_cur_4_t audio_control_cur_4_t;
#define audio_control_range_2_n_t              audio20_control_range_2_n_t
#define audio_control_range_4_n_t              audio20_control_range_4_n_t

/* UAC2 5.2.2 control request layout (removed from TinyUSB's audio.h) */
typedef struct TU_ATTR_PACKED {
    union {
        struct TU_ATTR_PACKED {
            uint8_t recipient : 5;
            uint8_t type      : 2;
            uint8_t direction : 1;
        } bmRequestType_bit;
        uint8_t bmRequestType;
    };
    uint8_t bRequest;
    uint8_t bChannelNumber;
    uint8_t bControlSelector;
    union { uint8_t bInterface; uint8_t bEndpoint; };
    uint8_t bEntityID;
    uint16_t wLength;
} audio_control_request_t;

#define USB_AUDIO51_EDPT_XFER(rh, ep, buf, len) usbd_edpt_xfer(rh, ep, buf, len, false)
#else
#define USB_AUDIO51_TUSB_NEW 0
#define USB_AUDIO51_EDPT_XFER(rh, ep, buf, len) usbd_edpt_xfer(rh, ep, buf, len)
#endif

#endif
