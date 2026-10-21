/* Audio-class part of tusb_config.h for usb_audio51 (TinyUSB 0.18 API).
 * Include this from the platform tusb_config.h after the common settings. */
#ifndef USB_AUDIO51_TUSB_CONFIG_H
#define USB_AUDIO51_TUSB_CONFIG_H

#define CFG_TUD_AUDIO                                   1
/* 1 = also include the UAC1 stereo mode (app-level class driver in usb_audio51.c) */
#ifndef USB_AUDIO51_UAC1
#define USB_AUDIO51_UAC1                                0
#endif
#if defined(USB_AUDIO51_CDC) && USB_AUDIO51_CDC
#define CFG_TUD_CDC                                     1
#define CFG_TUD_CDC_RX_BUFSIZE                          64
#define CFG_TUD_CDC_TX_BUFSIZE                          1024
#define CFG_TUD_CDC_EP_BUFSIZE                          64
#else
#define CFG_TUD_CDC                                     0
#endif
#define CFG_TUD_MSC                                     0
#if defined(USB_AUDIO51_NET) && USB_AUDIO51_NET
#define CFG_TUD_ECM_RNDIS                               1      /* settings page network adapter */
#define CFG_TUD_NCM                                     0
#endif
#ifndef USB_AUDIO51_HID
#define USB_AUDIO51_HID                                 0      /* 1 = media keys (consumer control) */
#endif
#if USB_AUDIO51_HID
#define CFG_TUD_HID                                     1
#define CFG_TUD_HID_EP_BUFSIZE                          8
#else
#define CFG_TUD_HID                                     0
#endif
#define CFG_TUD_MIDI                                    0
#define CFG_TUD_VENDOR                                  0

/* Descriptor length: see usb_audio51.c. The feature unit has one control
 * word per channel + master, so 5.1 and 7.1 differ; TinyUSB reads the length
 * when the configuration is set, so it can be a run-time value. */
#define USB_AUDIO51_FU_LEN(nch)     (6 + ((nch) + 1) * 4)
#define USB_AUDIO51_FN_LEN(nch)     (TUD_AUDIO_DESC_IAD_LEN \
    + TUD_AUDIO_DESC_STD_AC_LEN + TUD_AUDIO_DESC_CS_AC_LEN + TUD_AUDIO_DESC_CLK_SRC_LEN \
    + TUD_AUDIO_DESC_INPUT_TERM_LEN + TUD_AUDIO_DESC_OUTPUT_TERM_LEN + USB_AUDIO51_FU_LEN(nch) \
    + TUD_AUDIO_DESC_STD_AS_INT_LEN + TUD_AUDIO_DESC_STD_AS_INT_LEN + TUD_AUDIO_DESC_CS_AS_INT_LEN \
    + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN \
    + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN + TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_LEN)
#ifndef __ASSEMBLER__
#include <stdint.h>
extern uint16_t usb_audio51_fn_desc_len;
#endif
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN  usb_audio51_fn_desc_len

#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT                   1
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ                64
#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE            48000
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX              8      /* 7.1 max; 5.1 uses 6 */
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX      2
#define CFG_TUD_AUDIO_FUNC_1_RESOLUTION_RX              16

#define CFG_TUD_AUDIO_ENABLE_EP_OUT                     1
/* full-speed isochronous packet size (TinyUSB's TUD_AUDIO_EP_SIZE changed arity in 0.20) */
#define USB_AUDIO51_EP_SIZE(_rate, _bytes, _ch) ((((_rate) + 999) / 1000 + 1) * (_bytes) * (_ch))
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX \
    USB_AUDIO51_EP_SIZE(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)
/* ~16 ms (5.1) / ~12 ms (7.1) of software FIFO; feedback keeps it half full. Bigger stalls on the
 * sink side spill into the pipeline (dhp_usb_frames_wanted_guarded). */
#ifndef USB_AUDIO51_FIFO_PACKETS
#define USB_AUDIO51_FIFO_PACKETS 12   /* 7.1-size packets: = 16 x 5.1 packets, 12 x 7.1 */
#endif
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ           (USB_AUDIO51_FIFO_PACKETS * CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX)

#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP                1
/* Feedback format is picked per host automatically (see usb_audio51.c).
 * -DUSB_AUDIO51_MACOS=1 forces the Apple format if the guess ever fails. */
#ifndef USB_AUDIO51_MACOS
#define USB_AUDIO51_MACOS 0
#endif
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_FORMAT_CORRECTION USB_AUDIO51_MACOS

#endif
