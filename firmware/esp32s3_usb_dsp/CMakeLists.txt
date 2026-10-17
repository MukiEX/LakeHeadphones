cmake_minimum_required(VERSION 3.16)
# Local components: components/tinyusb is created by tools/vendor_tinyusb_s3.sh
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/components)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lakehp_s3)
