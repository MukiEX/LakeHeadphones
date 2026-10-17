set(LHP_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
idf_component_register(
    SRCS "main.c" "${LHP_ROOT}/firmware/common/resampler.c"
    INCLUDE_DIRS "." "${LHP_ROOT}/firmware/common"
    REQUIRES bt nvs_flash esp_driver_i2s esp_driver_gpio esp_timer)
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
