cmake_minimum_required(VERSION 3.12)

# set the project name and version
project(linux-example-btstack-broadcast-source VERSION 1.0.0 DESCRIPTION "Linux CE Broadcast Source Application")


# specify the C++ standard
set (CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

set (BUILD_SHARED_LIBS OFF)

add_definitions(-DLINUX_PLATFORM -DHCI_RX_QUEUE -DRX_PKT_NUM=30 -DENABLE_SCO_QUEUE -DSCO_PKT_NUM=5)

set (ARM_GCC_PATH               "COMPONENT_ARM_LINUX/COMPONENT_GCC/")
set (BASE_LIB_DIR_PATH          ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set (BTSTACK_INCLUDE            ${BASE_LIB_DIR_PATH}/btstack/wiced_include)
set (BTSTACK_LIB                ${BASE_LIB_DIR_PATH}/btstack/stack/COMPONENT_WICED_DUALMODE/COMPONENT_ARMv8_LINUX/COMPONENT_GCC)
set (PORTING_LAYER              ${BASE_LIB_DIR_PATH}/bluetooth-linux)
set (BT_AUDIO_COMPONENT         ${BASE_LIB_DIR_PATH}/bt-audio-profiles)
set (PROFILE_UTILS_COMPONENT    ${BASE_LIB_DIR_PATH}/bt-audio-profiles/utils)
set (ALSA_LIB                   ${BASE_LIB_DIR_PATH}/bluetooth-linux/linux-audio/lib/${ARM_GCC_PATH})
set (ALSA_INCLUDE               ${BASE_LIB_DIR_PATH}/bluetooth-linux/linux-audio/include)
set (SBC_INCLUDE                ${BASE_LIB_DIR_PATH}/bt-audio-profiles/sbc/include)
set (ALSA_SND_INCLUDE           ${BASE_LIB_DIR_PATH}/bluetooth-linux/linux-audio/include/alsa)
set (APP_BT_CFG                 ${CMAKE_CURRENT_SOURCE_DIR}/app_bt_config)
set (AUDIO_MODULE_COMPONENT     ${PORTING_LAYER}/COMPONENT_audio_module)
set (LE_AUDIO_COMPONENT         ${BASE_LIB_DIR_PATH}/le-audio-profiles-linux/COMPONENT_le_audio_profiles)
set (ISO_MODULE                 ${BASE_LIB_DIR_PATH}/le-audio-profiles-linux/iso_data_handler_module_lib)
set (LC3_CODEC_COMPONENT        ${CMAKE_CURRENT_SOURCE_DIR}/COMPONENT_LC3_CODEC)
set (GATT_INTERFACE_INCLUDE     ${BASE_LIB_DIR_PATH}/le-audio-profiles-linux/COMPONENT_gatt_interface_linux)
set (GATT_INTERFACE_LIB         ${BASE_LIB_DIR_PATH}/le-audio-profiles-linux/COMPONENT_gatt_interface_linux/COMPONENT_ARMv8_LINUX/COMPONENT_GCC)
set (ALSA_LIB                   ${BASE_LIB_DIR_PATH}/bluetooth-linux/linux-audio/lib/${ARM_GCC_PATH})
set (LE_AUDIO_INCLUDE           ${BASE_LIB_DIR_PATH}/le-audio-profiles-linux/COMPONENT_le_audio_profiles_linux)
set (LE_AUDIO_LIB               ${BASE_LIB_DIR_PATH}/le-audio-profiles-linux/COMPONENT_le_audio_profiles_linux/COMPONENT_ARMv8_LINUX/COMPONENT_GCC)
set (ALSA_INCLUDE               ${BASE_LIB_DIR_PATH}/bluetooth-linux/linux-audio/include)

#set (LC3CODEC                  "ittiam")

if(LC3CODEC STREQUAL "ittiam")
    set (ITTIAMLC3                  "ittiam_lc3")
else()
    set (GOOGLELC3                  "google_lc3")
endif()

if(ITTIAMLC3)
    set (LC3_FOLDER             ${ITTIAMLC3})
elseif(GOOGLELC3)
    set (LC3_FOLDER             ${GOOGLELC3})
endif()

add_subdirectory(${LC3_CODEC_COMPONENT}/)

# control where the static and shared libraries are built so that on windows
# we don't need to tinker with the path to run the executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

file(GLOB APP_UTILS     ${CMAKE_CURRENT_SOURCE_DIR}/app_bt_utils/*.c)
file(GLOB APP_SRC       ${CMAKE_CURRENT_SOURCE_DIR}/app/*.c)
file(GLOB_RECURSE       PORTING_LAYER_SRC       ${PORTING_LAYER}/*.c)
file(GLOB_RECURSE       AUDIO_MODULE_SRC        ${AUDIO_MODULE_COMPONENT}/*.c)
file(GLOB_RECURSE       APP_BT_CONFIG           ${APP_BT_CFG}/*.c)
file(GLOB_RECURSE       PROFILE_UTILS           ${PROFILE_UTILS_COMPONENT}/*.c)

# linking
link_directories(${BTSTACK_LIB}/)
link_directories(${ALSA_LIB}/)
link_directories(${LC3_CODEC_COMPONENT}/${LC3_FOLDER}/)
link_directories(${LE_AUDIO_LIB}/)
link_directories(${GATT_INTERFACE_LIB}/)

add_executable(${PROJECT_NAME} 
    ${PORTING_LAYER_SRC}
    ${AUDIO_MODULE_SRC}
    ${APP_UTILS}
    ${APP_SRC}
    ${APP_BT_CONFIG}
    ${PROFILE_UTILS}
    ${ISO_MODULE_SRC}
)

find_library(
  LC3_LIBRARY
  NAMES ia_lc3_enc_dec ia_lc3_enc_dec_lib
  HINTS
  ${AUDIO_MODULE_COMPONENT}/ittiam_lc3)

find_library(
    ASOUND_LIB
    asound
    HINTS
    ${ALSA_LIB})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/app)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/app_bt_utils)
include_directories(${LE_AUDIO_COMPONENT}/include)
include_directories(${LE_AUDIO_COMPONENT}/internal)
include_directories(${ISO_MODULE}/)
include_directories(${LC3_CODEC_COMPONENT}/)
include_directories(${LE_AUDIO_INCLUDE}/include)
include_directories(${AUDIO_MODULE_COMPONENT}/)
include_directories(${AUDIO_MODULE_COMPONENT}/include)
include_directories(${GATT_INTERFACE_INCLUDE}/include)
include_directories(${BTSTACK_INCLUDE}/)
include_directories(${PORTING_LAYER}/)
include_directories(${PORTING_LAYER}/wiced_hal)
include_directories(${APP_BT_CFG}/)
include_directories(${BT_AUDIO_COMPONENT}/profiles_include)
include_directories(${BT_AUDIO_COMPONENT}/utils)
include_directories(${ALSA_INCLUDE}/)
include_directories(${ALSA_INCLUDE}/alsa)
include_directories(${ALSA_INCLUDE}/sound/)
include_directories(${ALSA_INCLUDE}/sound/uapi/)
include_directories(${ALSA_SND_INCLUDE}/)
include_directories(${PROFILE_UTILS_COMPONENT}/)
include_directories(${LC3_LIB}/)

target_link_libraries(${PROJECT_NAME} PRIVATE btstack)
target_link_libraries(${PROJECT_NAME} PRIVATE pthread rt)

target_link_libraries(${PROJECT_NAME} PRIVATE le_audio_profiles gatt_interface ${ASOUND_LIB} lc3_codec)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
