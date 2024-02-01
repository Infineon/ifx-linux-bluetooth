#This CMakeLists file is added for stagging_repo "Bluetooth_linux"  to build Linux porting layer
cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(linux_stack_porting_layer VERSION 1.1.0 DESCRIPTION "Linux platform layer")

set(WICED_INC "${CMAKE_CURRENT_SOURCE_DIR}/${WICED_INCLUDE}")
message("WICED_INCLUDE Path=${WICED_INCLUDE}")

add_definitions(-DBTSTACK_LINUX -DBT_TRACE_MASK=0xFFFFFFFF)

add_library(${PROJECT_NAME}
        patch_download.c
        wiced_bt_app.c
        hci_uart_linux.c
        linux_main.c
        spy_ux.c
        linux_tcp_server.c
        wiced_hal_nvram.c
        nvram.c
        utils_arg_parser.c
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/)
include_directories(${WICED_INC})
