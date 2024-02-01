cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(linux-example-btstack-spp VERSION 1.0.0 DESCRIPTION "Linux CE SPP Application")


# specify the C++ standard
set (CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

set (BUILD_SHARED_LIBS ON)
set (BASE_LIB_DIR_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set (BTSTACK_INCLUDE ${BASE_LIB_DIR_PATH}/btstack/wiced_include)
set (BTSTACK_LIB ${BASE_LIB_DIR_PATH}/btstack/stack/COMPONENT_WICED_DUALMODE/COMPONENT_ARMv8_LINUX/COMPONENT_GCC)
set (PORTING_LAYER ${BASE_LIB_DIR_PATH}/bluetooth-linux)
set (SPP_PROFILE_LAYER ${BASE_LIB_DIR_PATH}/bt-audio-profiles/COMPONENT_spp_profile)

# control where the static and shared libraries are built so that on windows
# we don't need to tinker with the path to run the executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

link_directories(${BTSTACK_LIB}/)
add_executable(${PROJECT_NAME}
	${CMAKE_CURRENT_SOURCE_DIR}/app_bt_config/wiced_bt_cfg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/spp.c
    ${SPP_PROFILE_LAYER}/wiced_spp_api.c
    ${SPP_PROFILE_LAYER}/wiced_spp_rw_data.c
    ${SPP_PROFILE_LAYER}/../utils/wiced_bt_utils.c
    ${PORTING_LAYER}/patch_download.c
    ${PORTING_LAYER}/wiced_bt_app.c
    ${PORTING_LAYER}/hci_uart_linux.c
    ${PORTING_LAYER}/linux_main.c
    ${PORTING_LAYER}/spy_ux.c
    ${PORTING_LAYER}/linux_tcp_server.c
    ${PORTING_LAYER}/wiced_hal/wiced_hal_nvram.c
    ${PORTING_LAYER}/nvram.c
    ${PORTING_LAYER}/utils_arg_parser.c
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/app_bt_utils)
include_directories(${SPP_PROFILE_LAYER}/)
include_directories(${SPP_PROFILE_LAYER}/../profiles_include/)
include_directories(${SPP_PROFILE_LAYER}/../utils/)
include_directories(${BTSTACK_INCLUDE}/)
include_directories(${PORTING_LAYER}/)
include_directories(${PORTING_LAYER}/wiced_hal/)

target_link_libraries(${PROJECT_NAME} PRIVATE btstack)
target_link_libraries(${PROJECT_NAME} PRIVATE pthread rt)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
