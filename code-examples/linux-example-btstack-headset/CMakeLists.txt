cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(linux-example-btstack-headset VERSION 1.0.0 DESCRIPTION "Linux CE Headset Application")


# specify the C++ standard
set (CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

set (BUILD_SHARED_LIBS ON)

add_definitions(-DPLATFORM_LED_DISABLED -DLINUX_PLATFORM -DCT_HANDLE_PASSTHROUGH_COMMANDS -DWICED_APP_LE_INCLUDED)
add_definitions(-DWICED_BT_HFP_HF_WBS_INCLUDED=TRUE)
add_definitions(-DWICED_BT_HFP_HF_DEBUG=TRUE)

#add_definitions(-DHCI_RX_QUEUE -DRX_PKT_NUM=30 -DENABLE_SCO_QUEUE -DSCO_PKT_NUM=30)

set (BASE_LIB_DIR_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set (BTSTACK_INCLUDE ${BASE_LIB_DIR_PATH}/btstack/wiced_include)
set (ARM_GCC_PATH "COMPONENT_ARM_LINUX/COMPONENT_GCC/")
set (BTSTACK_LIB ${BASE_LIB_DIR_PATH}/btstack/stack/COMPONENT_WICED_DUALMODE/COMPONENT_ARMv8_LINUX/COMPONENT_GCC)
set (PORTING_LAYER ${BASE_LIB_DIR_PATH}/bluetooth-linux)
set (BT_AUDIO_COMPONENT ${BASE_LIB_DIR_PATH}/bt-audio-profiles)
set (HFU_AUDIO_COMPONENT ${BASE_LIB_DIR_PATH}/bt-audio-profiles/COMPONENT_handsfree_profile)
set (A2DP_SINK_COMPONENT ${BASE_LIB_DIR_PATH}/bt-audio-profiles/COMPONENT_a2dp_sink_profile)
set (AVRCP_COMPONENT ${BASE_LIB_DIR_PATH}/bt-audio-profiles/COMPONENT_avrc_controller)
set (PROFILE_UTILS_COMPONENT ${BASE_LIB_DIR_PATH}/bt-audio-profiles/utils)
set (ALSA_LIB ${BASE_LIB_DIR_PATH}/bluetooth-linux/linux-audio/lib/${ARM_GCC_PATH})
set (ALSA_INCLUDE ${BASE_LIB_DIR_PATH}/bluetooth-linux/linux-audio/include)
set (SBC_INCLUDE ${BASE_LIB_DIR_PATH}/bt-audio-profiles/sbc/include)
set (SBC_LIB ${BASE_LIB_DIR_PATH}/bt-audio-profiles/sbc/COMPONENT_ARMv8_LINUX/COMPONENT_GCC)
set (BTSDK-GFPS ${BASE_LIB_DIR_PATH}/btsdk-gfps)

# control where the static and shared libraries are built so that on windows
# we don't need to tinker with the path to run the executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

# linking
link_directories(${BTSTACK_LIB}/)
link_directories(${ALSA_LIB}/)
link_directories(${SBC_LIB}/)

add_executable(${PROJECT_NAME}
    ${A2DP_SINK_COMPONENT}/wiced_bt_a2dp_m12.c
    ${A2DP_SINK_COMPONENT}/wiced_bt_a2dp_m24.c
    ${A2DP_SINK_COMPONENT}/wiced_bt_a2dp_sbc.c
    ${A2DP_SINK_COMPONENT}/wiced_bt_a2dp_sink_act.c
    ${A2DP_SINK_COMPONENT}/wiced_bt_a2dp_sink_api.c
    ${A2DP_SINK_COMPONENT}/wiced_bt_a2dp_sink_cfg.c
    ${A2DP_SINK_COMPONENT}/wiced_bt_a2dp_sink_main.c
    ${A2DP_SINK_COMPONENT}/wiced_bt_a2dp_sink_ssm.c
    ${AVRCP_COMPONENT}/wiced_bt_avrc_ct.c
    ${PROFILE_UTILS_COMPONENT}/wiced_bt_utils.c
    ${PROFILE_UTILS_COMPONENT}/wiced_bt_utils.h
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_act.c
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_api.c
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_at.c
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_cmd.c
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_main.c
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_ssm.c
    ${HFU_AUDIO_COMPONENT}/wiced_bt_hfp_hf_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_bt_config/wiced_bt_cfg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_bt_config/wiced_app_cfg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_bt_config/wiced_app_cfg.h
    ${CMAKE_CURRENT_SOURCE_DIR}/app/wiced_hal_nvram_wrapper.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/headset_control_le.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/headset_control_le.h
    ${CMAKE_CURRENT_SOURCE_DIR}/app/headset_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/headset_control.h
    ${CMAKE_CURRENT_SOURCE_DIR}/app/platform_audio_interface.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/headset.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/linux_platform/bt_decode_queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/linux_platform/alsa_playback.c
    ${CMAKE_CURRENT_SOURCE_DIR}/linux_platform/alsa_capture.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bt_hs_spk_lib/bt_hs_spk_audio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bt_hs_spk_lib/bt_hs_spk_audio.h
    ${CMAKE_CURRENT_SOURCE_DIR}/bt_hs_spk_lib/bt_hs_spk_button.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bt_hs_spk_lib/bt_hs_spk_button.h
    ${CMAKE_CURRENT_SOURCE_DIR}/bt_hs_spk_lib/bt_hs_spk_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bt_hs_spk_lib/bt_hs_spk_control.h
    ${CMAKE_CURRENT_SOURCE_DIR}/bt_hs_spk_lib/bt_hs_spk_handsfree.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bt_hs_spk_lib/bt_hs_spk_handsfree.h
    ${CMAKE_CURRENT_SOURCE_DIR}/bt_hs_spk_lib/bt_hs_spk_handsfree_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bt_hs_spk_lib/bt_hs_spk_handsfree_utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/audiomanager/audio_effects.c
    ${CMAKE_CURRENT_SOURCE_DIR}/audiomanager/audio_effects.h
    ${CMAKE_CURRENT_SOURCE_DIR}/audiomanager/platform_audio_device.h
    ${CMAKE_CURRENT_SOURCE_DIR}/audiomanager/platform_audio_device_wrapper.c
    ${CMAKE_CURRENT_SOURCE_DIR}/audiomanager/platform_audio_effects.h
    ${CMAKE_CURRENT_SOURCE_DIR}/audiomanager/wiced_audio_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/audiomanager/wiced_audio_manager.h
    ${PORTING_LAYER}/patch_download.c
    ${PORTING_LAYER}/wiced_bt_app.c
    ${PORTING_LAYER}/hci_uart_linux.c
    ${PORTING_LAYER}/linux_main.c
    ${PORTING_LAYER}/spy_ux.c
    ${PORTING_LAYER}/linux_tcp_server.c
    ${PORTING_LAYER}/sco_process.c
    ${PORTING_LAYER}/wiced_hal/wiced_hal_nvram.c
    ${PORTING_LAYER}/wiced_hal/wiced_hal_rand.c
    ${PORTING_LAYER}/wiced_hal/wiced_hal_memory.c
    ${PORTING_LAYER}/wiced_hal/wiced_hal_sha256.c
    ${PORTING_LAYER}/wiced_hal/wiced_hal_utils.c
    ${PORTING_LAYER}/nvram.c
    ${PORTING_LAYER}/utils_arg_parser.c
    ${ALSA_INCLUDE}/alsa/asoundlib.h
    ${CMAKE_CURRENT_SOURCE_DIR}/app_bt_utils/app_bt_utils.c
    ${BTSDK-GFPS}/COMPONENT_gfps_provider/gfps_provider.c
    ${BTSDK-GFPS}/COMPONENT_gfps_provider/fastpair_sec_utils.c
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/linux_platform)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/app_bt_utils)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/app_bt_config)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/bt_hs_spk_lib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/audio_sink)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/audiomanager)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/app_bt_utils/)
include_directories(${BTSTACK_INCLUDE}/)
include_directories(${PORTING_LAYER}/)
include_directories(${PORTING_LAYER}/wiced_hal)
include_directories(${BT_AUDIO_COMPONENT}/profiles_include)
include_directories(${BT_AUDIO_COMPONENT}/utils)
include_directories(${HFU_AUDIO_COMPONENT}/)
include_directories(${A2DP_SINK_COMPONENT}/)
include_directories(${ALSA_INCLUDE}/)
include_directories(${ALSA_INCLUDE}/sound/)
include_directories(${ALSA_INCLUDE}/sound/uapi/)
include_directories(${SBC_INCLUDE}/)
include_directories(${BTSDK-GFPS}/COMPONENT_gfps_provider)

target_link_libraries(${PROJECT_NAME} PRIVATE btstack)
target_link_libraries(${PROJECT_NAME} PRIVATE pthread rt)
target_link_libraries(${PROJECT_NAME} PRIVATE asound)
target_link_libraries(${PROJECT_NAME} PRIVATE sbc)
