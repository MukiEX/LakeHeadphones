cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(lakehp_pico2w C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(LHP_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(LHP_FILTERS ${LHP_ROOT}/firmware/generated/lakehp_filters.h)
if(EXISTS ${LHP_FILTERS})
    set(LHP_FILTERS_EMBEDDED 1)
    message(STATUS "LakeHP: room filters compiled in from ${LHP_FILTERS}")
else()
    set(LHP_FILTERS_EMBEDDED 0)
    message(STATUS "LakeHP: no filters header - the firmware reads them from flash at 0x10300000.\n"
                   "        Add them with tools/lakehp_flasher.html (or extract_filters.py --region).")
endif()

# Optional: external room button on a GPIO (to GND) instead of BOOTSEL
set(LHP_BUTTON_GPIO -1 CACHE STRING "GPIO for the room button, -1 = BOOTSEL")
set(LHP_DEFAULT_ROOM 1 CACHE STRING "Room at power-up: 1 small, 2 medium, 3 large")
# Debug: adds a USB serial port ("LakeHP debug log") with a status line every 2 s
set(LHP_DEBUG_CDC 0 CACHE STRING "1 = log over a USB serial port next to the sound card")

# Settings page (http://192.168.20.1 over a USB network adapter)
set(LHP_WEBCFG 1 CACHE STRING "1 = settings web page over USB networking (needs the SDK's lib/lwip)")
if (LHP_WEBCFG AND NOT EXISTS ${PICO_SDK_PATH}/lib/lwip/src/core/init.c)
    message(WARNING "lib/lwip is missing in the SDK (git submodule update --init lib/lwip): building without the settings page")
    set(LHP_WEBCFG 0)
endif()
# web/index.html -> C array (no extra tools needed)
file(READ ${CMAKE_CURRENT_LIST_DIR}/web/index.html LHP_WEB_HEX HEX)
string(LENGTH "${LHP_WEB_HEX}" LHP_WEB_HEXLEN)
math(EXPR LHP_WEB_LEN "${LHP_WEB_HEXLEN} / 2")
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," LHP_WEB_BYTES "${LHP_WEB_HEX}")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/web_ui.c "/* generated from web/index.html */\nconst char lhp_web_ui[] = {${LHP_WEB_BYTES}0};\nconst unsigned lhp_web_ui_len = ${LHP_WEB_LEN};\n")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/web/index.html)

add_executable(lakehp_pico2w
    main.c
    bt_a2dp.c
    ${LHP_ROOT}/engine/lakehp.c
    ${LHP_ROOT}/firmware/common/dh_pipeline.c
    ${LHP_ROOT}/firmware/common/usb_audio51.c
    ${LHP_ROOT}/firmware/common/resampler.c
    dbg_cdc.c
)
if (LHP_WEBCFG)
    target_sources(lakehp_pico2w PRIVATE
        webcfg.c
        ${CMAKE_CURRENT_BINARY_DIR}/web_ui.c
        ${PICO_SDK_PATH}/lib/tinyusb/lib/networking/dhserver.c
        ${PICO_SDK_PATH}/lib/tinyusb/lib/networking/rndis_reports.c
    )
    target_include_directories(lakehp_pico2w PRIVATE ${PICO_SDK_PATH}/lib/tinyusb/lib/networking)
    target_link_libraries(lakehp_pico2w PRIVATE pico_lwip_core pico_lwip_core4 pico_lwip_netif pico_lwip_arch pico_unique_id)
endif()
target_include_directories(lakehp_pico2w PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}          # tusb_config.h, btstack_config.h
    ${LHP_ROOT}/engine
    ${LHP_ROOT}/firmware/common
    ${LHP_ROOT}/firmware/generated
)
target_compile_definitions(lakehp_pico2w PRIVATE
    CFG_TUSB_OS=OPT_OS_PICO
    LHP_PICO_RAMFUNC=1                 # DSP inner loops in SRAM
    LHP_BUTTON_GPIO=${LHP_BUTTON_GPIO}
    LHP_DEFAULT_ROOM=${LHP_DEFAULT_ROOM}
    LHP_DEBUG_CDC=${LHP_DEBUG_CDC}
    LHP_FILTERS_EMBEDDED=${LHP_FILTERS_EMBEDDED}
    USB_AUDIO51_CDC=${LHP_DEBUG_CDC}
    USB_AUDIO51_UAC1=1                 # triple press: stereo UAC1 mode for consoles
    USB_AUDIO51_HID=1                  # headset buttons -> USB media keys
    LHP_WEBCFG=${LHP_WEBCFG}
    USB_AUDIO51_NET=${LHP_WEBCFG}
    PICO_STACK_SIZE=0x1000             # core 0: USB, SBC encoder, button
    CYW43_PIO_CLOCK_DIV_INT=2          # 200 MHz sys clock: 2 + 171/256 = 2.67,
    CYW43_PIO_CLOCK_DIV_FRAC8=171      # same CYW43 SPI speed as stock 150 MHz / 2
)
target_compile_options(lakehp_pico2w PRIVATE -O2 -Wall)
target_link_libraries(lakehp_pico2w PRIVATE
    pico_stdlib
    pico_multicore
    hardware_sync
    pico_flash
    hardware_vreg
    hardware_flash
    tinyusb_device
    pico_cyw43_arch_none               # threadsafe background: BT runs in a low-prio IRQ
    pico_btstack_cyw43
    pico_btstack_classic
    pico_btstack_sbc_encoder
)
pico_enable_stdio_usb(lakehp_pico2w 0)    # USB is the sound card
# UART logs (GP0 TX, 115200) in normal builds; debug builds log over USB instead,
# because a blocking 115200-baud printf on core 0 drops USB audio packets.
if (LHP_DEBUG_CDC)
    pico_enable_stdio_uart(lakehp_pico2w 0)
else()
    pico_enable_stdio_uart(lakehp_pico2w 1)
endif()
pico_add_extra_outputs(lakehp_pico2w)
