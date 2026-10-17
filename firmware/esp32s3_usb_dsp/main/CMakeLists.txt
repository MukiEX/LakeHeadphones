set(LHP_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
set(LHP_FILTERS ${LHP_ROOT}/firmware/generated/lakehp_filters.h)
if(NOT EXISTS ${LHP_FILTERS})
    message(FATAL_ERROR "\n${LHP_FILTERS} is missing.\nRun:  python3 tools/extract_filters.py /path/to/DolbyHph.dll --rates 48000 -o firmware/generated/lakehp_filters.h\n")
endif()

idf_component_register(
    SRCS "main.c"
         "${LHP_ROOT}/engine/lakehp.c"
         "${LHP_ROOT}/firmware/common/dh_pipeline.c"
         "${LHP_ROOT}/firmware/common/usb_audio51.c"
    INCLUDE_DIRS "." "${LHP_ROOT}/engine" "${LHP_ROOT}/firmware/common" "${LHP_ROOT}/firmware/generated"
    REQUIRES tinyusb usb esp_driver_i2s esp_driver_gpio nvs_flash esp_timer)

# TinyUSB itself must see our tusb_config.h
idf_component_get_property(tusb_lib tinyusb COMPONENT_LIB)
target_include_directories(${tusb_lib} PUBLIC "${CMAKE_CURRENT_LIST_DIR}" "${LHP_ROOT}/firmware/common")
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wno-unused-function)
target_compile_definitions(${COMPONENT_LIB} PRIVATE DHP_TARGET_FILL=${CONFIG_LHP_TARGET_FILL})
