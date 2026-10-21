cmake_minimum_required(VERSION 3.13)

# LakeHP on the Waveshare RP2350-USB-A: USB 5.1/7.1 sound card on the USB-C
# port, rendered stereo out of the USB-A port to a USB DAC or a 2.4 GHz
# wireless headset dongle (USB Audio Class 1/2 device, full speed).
#
# Needs current TinyUSB (USB audio host class) and Pico-PIO-USB with the
# isochronous patch in patches/: run tools/vendor_usbhost_libs.py first.

set(PICO_BOARD waveshare_rp2350_usb_a CACHE STRING "Board type")
set(PICO_TINYUSB_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/tinyusb)
set(PICO_PIO_USB_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/Pico-PIO-USB)
if (NOT EXISTS ${PICO_TINYUSB_PATH}/src/class/audio/audio_host.c OR NOT EXISTS ${PICO_PIO_USB_PATH}/src/pio_usb.c)
    message(FATAL_ERROR "lib/tinyusb or lib/Pico-PIO-USB missing: run  python3 tools/vendor_usbhost_libs.py  first")
endif()

include(pico_sdk_import.cmake)
project(lakehp_usbhost C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(LHP_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(LHP_P2W ${LHP_ROOT}/firmware/pico2w)          # shared: settings page, debug log
set(TINYUSB ${PICO_TINYUSB_PATH})
set(LHP_FILTERS ${LHP_ROOT}/firmware/generated/lakehp_filters.h)
if(EXISTS ${LHP_FILTERS})
    set(LHP_FILTERS_EMBEDDED 1)
else()
    set(LHP_FILTERS_EMBEDDED 0)
    message(STATUS "LakeHP: filters are read from flash at 0x10100000 (this board has 2 MB of flash).\n"
                   "        Add them with tools/lakehp_flasher.html or extract_filters.py --uf2 ... --addr 0x10100000.")
endif()

set(LHP_BUTTON_GPIO -1 CACHE STRING "GPIO for the room button, -1 = BOOTSEL")
set(LHP_DEFAULT_ROOM 1 CACHE STRING "Room at power-up: 1 small, 2 medium, 3 large")
set(LHP_DEBUG_CDC 0 CACHE STRING "1 = log over a USB serial port next to the sound card")
set(LHP_SYS_KHZ 240000 CACHE STRING "System clock in kHz (a multiple of 12 MHz for PIO-USB)")
set(LHP_WEBCFG 1 CACHE STRING "1 = settings web page over USB networking (needs the SDK's lib/lwip)")
if (LHP_WEBCFG AND NOT EXISTS ${PICO_SDK_PATH}/lib/lwip/src/core/init.c)
    message(WARNING "lib/lwip is missing in the SDK: building without the settings page")
    set(LHP_WEBCFG 0)
endif()
file(READ ${LHP_P2W}/web/index.html LHP_WEB_HEX HEX)
string(LENGTH "${LHP_WEB_HEX}" LHP_WEB_HEXLEN)
math(EXPR LHP_WEB_LEN "${LHP_WEB_HEXLEN} / 2")
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," LHP_WEB_BYTES "${LHP_WEB_HEX}")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/web_ui.c "/* generated from web/index.html */\nconst char lhp_web_ui[] = {${LHP_WEB_BYTES}0};\nconst unsigned lhp_web_ui_len = ${LHP_WEB_LEN};\n")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${LHP_P2W}/web/index.html)

add_executable(lakehp_usbhost
    main.c
    usbout.c
    status_led.c
    ${LHP_ROOT}/engine/lakehp.c
    ${LHP_ROOT}/firmware/common/dh_pipeline.c
    ${LHP_ROOT}/firmware/common/usb_audio51.c
    ${LHP_ROOT}/firmware/common/resampler.c
    ${LHP_P2W}/dbg_cdc.c
)
pico_generate_pio_header(lakehp_usbhost ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)
if (LHP_WEBCFG)
    target_sources(lakehp_usbhost PRIVATE
        ${LHP_P2W}/webcfg.c
        ${CMAKE_CURRENT_BINARY_DIR}/web_ui.c
        ${TINYUSB}/lib/networking/dhserver.c
        ${TINYUSB}/lib/networking/rndis_reports.c
    )
    target_include_directories(lakehp_usbhost PRIVATE ${TINYUSB}/lib/networking)
    target_link_libraries(lakehp_usbhost PRIVATE pico_lwip_core pico_lwip_core4 pico_lwip_netif pico_lwip_arch pico_unique_id)
endif()
target_include_directories(lakehp_usbhost PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}          # tusb_config.h
    ${LHP_P2W}                         # lwipopts.h, webcfg.h, dbg_cdc.h
    ${LHP_ROOT}/engine
    ${LHP_ROOT}/firmware/common
    ${LHP_ROOT}/firmware/generated
)
target_compile_definitions(lakehp_usbhost PRIVATE
    CFG_TUSB_OS=OPT_OS_PICO
    LHP_PICO_RAMFUNC=1
    LHP_BUTTON_GPIO=${LHP_BUTTON_GPIO}
    LHP_DEFAULT_ROOM=${LHP_DEFAULT_ROOM}
    LHP_DEBUG_CDC=${LHP_DEBUG_CDC}
    LHP_FILTERS_EMBEDDED=${LHP_FILTERS_EMBEDDED}
    LHP_SYS_KHZ=${LHP_SYS_KHZ}
    USB_AUDIO51_CDC=${LHP_DEBUG_CDC}
    USB_AUDIO51_UAC1=1
    USB_AUDIO51_HID=0
    LHP_WEBCFG=${LHP_WEBCFG}
    USB_AUDIO51_NET=${LHP_WEBCFG}
    # no Bluetooth retries to ride out: much smaller buffers (see docs/TECHNICAL.md)
    DHP_TARGET_FILL=1792
    DHP_TARGET_FILL_LL=160
    DHP_LL_CHUNK=64
    USB_AUDIO51_FIFO_PACKETS=6
    PICO_STACK_SIZE=0x1000
)
target_compile_options(lakehp_usbhost PRIVATE -O2 -Wall)
target_link_libraries(lakehp_usbhost PRIVATE
    pico_stdlib
    pico_multicore
    hardware_sync
    hardware_pio
    pico_flash
    hardware_vreg
    hardware_flash
    tinyusb_device
    tinyusb_host
    tinyusb_pico_pio_usb
)
pico_enable_stdio_usb(lakehp_usbhost 0)
if (LHP_DEBUG_CDC)
    pico_enable_stdio_uart(lakehp_usbhost 0)
else()
    pico_enable_stdio_uart(lakehp_usbhost 1)
endif()
pico_add_extra_outputs(lakehp_usbhost)
