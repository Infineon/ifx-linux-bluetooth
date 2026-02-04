cmake_minimum_required(VERSION 3.20)

# set the project name and version
project(linux-example-btstack-hello-sensor VERSION 1.0.0 DESCRIPTION "Linux Code Example HelloSensor Application")

# specify the C++ standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

function(CheckDependencies HAS_DEPENDENCIES)
    if(
        NOT EXISTS "${BASE_LIB_DIR_PATH}/btstack"
        OR NOT EXISTS "${BASE_LIB_DIR_PATH}/bluetooth-linux"
    )
        set(HAS_DEPENDENCIES FALSE PARENT_SCOPE)
    else()
        set(HAS_DEPENDENCIES TRUE PARENT_SCOPE)
    endif()
endfunction(CheckDependencies)

cmake_path(GET CMAKE_CURRENT_SOURCE_DIR PARENT_PATH PARENT_DIR)
cmake_path(GET PARENT_DIR PARENT_PATH GRANDPARENT_DIR)

# Check if dependencies exists in the right path
set(BASE_LIB_DIR_PATH "" CACHE PATH "Path to other dependencies: btstack, bluetooth-linux, ...")
if(BASE_LIB_DIR_PATH)
    # User provide the base lib dir
    CheckDependencies(HAS_DEPENDENCIES)
else()
    # defualt to parent dir
    set(BASE_LIB_DIR_PATH ${PARENT_DIR})
    CheckDependencies(HAS_DEPENDENCIES)
    if(NOT HAS_DEPENDENCIES)
        set(BASE_LIB_DIR_PATH ${GRANDPARENT_DIR})
        CheckDependencies(HAS_DEPENDENCIES)
    endif()
endif()
if (NOT HAS_DEPENDENCIES)
    message(WARNING "Dependencies not found. Please set BASE_LIB_DIR_PATH correctly.")
endif()

set(BUILD_SHARED_LIBS ON)
set(BTSTACK_INCLUDE ${BASE_LIB_DIR_PATH}/btstack/wiced_include)
set(BTSTACK_LIB ${BASE_LIB_DIR_PATH}/btstack/stack/COMPONENT_WICED_DUALMODE/COMPONENT_ARMv8_LINUX/COMPONENT_GCC)
set(PORTING_LAYER ${BASE_LIB_DIR_PATH}/bluetooth-linux)

add_definitions(-DLINUX_PLATFORM)

# control where the static and shared libraries are built so that on windows
# we don't need to tinker with the path to run the executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

link_directories(${BTSTACK_LIB}/)
add_executable(${PROJECT_NAME}
	${CMAKE_CURRENT_SOURCE_DIR}/app_bt_config/wiced_bt_cfg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/main.c
	${CMAKE_CURRENT_SOURCE_DIR}/app_bt_utils/app_bt_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/hello_sensor_gatt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_bt_config/hello_sensor_gatt_cfg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/hello_sensor.c
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
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/app_bt_utils)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/app_bt_config)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${BTSTACK_INCLUDE}/)
include_directories(${PORTING_LAYER}/)
include_directories(${PORTING_LAYER}/wiced_hal/)

target_link_libraries(${PROJECT_NAME} PRIVATE btstack_dual_mode)
target_link_libraries(${PROJECT_NAME} PRIVATE pthread rt)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
