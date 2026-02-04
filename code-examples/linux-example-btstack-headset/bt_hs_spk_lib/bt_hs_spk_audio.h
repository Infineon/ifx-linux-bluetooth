/*
* Copyright 2014-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*/

 /**************************************************************************************************
 *
 * File Name:       bt_hs_spk_audio.h
 *
 * Abstract:        Utilities and definition used for Audio Control Module.
 *
 * Special Notices:
 *
 **************************************************************************************************
 */

#pragma once
/**************************************************************************************************
*  Includes
**************************************************************************************************/
#include "wiced_result.h"
#include "wiced_bt_a2dp_sink.h"
#include "bt_hs_spk_control.h"

/**************************************************************************************************
*  Macros and Literal Definitions
**************************************************************************************************/
#define BT_HS_SPK_AUDIO_VOLUME_MAX      127
#define BT_HS_SPK_AUDIO_VOLUME_MIN      0
#define BT_HS_SPK_AUDIO_VOLUME_STEP     8

#define BT_HS_SPK_AUDIO_AVRC_APP_ATTR_SETTINGS_MAX  4
#define BT_HS_SPK_AUDIO_AVRC_APP_ATTR_VALUES_MAX    4

#define DELAY_DECODE_MILLISEC         1000
/**************************************************************************************************
*  Type Definitions and Enums
**************************************************************************************************/
/* Callback to report A2DP Stream Activity event */
typedef void (bt_hs_spk_audio_a2dp_state_callback_t)(void);

/** A2DP state */
typedef enum
{
    BT_HS_SPK_AUDIO_A2DP_STATE_IDLE,            /* Initial state (channel is unused) */
    BT_HS_SPK_AUDIO_A2DP_STATE_CONFIGURED,      /* Remote has sent configuration request */
    BT_HS_SPK_AUDIO_A2DP_STATE_CONNECTED,       /* Signaling Channel is connected and active */
    BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING,   /* Start initiated, intermediate state waiting on response */
    BT_HS_SPK_AUDIO_A2DP_STATE_STARTED,         /* Data streaming */
    BT_HS_SPK_AUDIO_A2DP_STATE_PAUSE_PENDING,   /* Pause initiated, intermediate state waiting on response */
} bt_hs_spk_audio_a2dp_state_t;

/* Volume effect event of external codec for A2DP stream
 * It should be handled in user application if VOLUME_EFFECT enabled
 */
typedef enum
{
    VOLUME_EFFECT_NONE,                     /* No volume effect, bt_hs_spk_lib will set volume immediately and inform app */
    VOLUME_EFFECT_INIT_MUTE,                /* Init mute of audio start, app should handle the event and set effect timer */
    VOLUME_EFFECT_RAMP_UP,                  /* Set the volume due to ramp up volume by app's effect timer */
    VOLUME_EFFECT_UNDERRUN_MUTE,            /* Mute due to undderun, app should handle the event and set effect timer */
} bt_hs_spk_audio_volume_effect_t;

//=================================================================================================
//  Structure
//=================================================================================================
typedef struct bt_hs_spk_audio_info
{
    wiced_bt_device_address_t   bdaddr; /* active source device's BT address */
    uint16_t                    handle; /* a2dp sink profile handle value for the active source device*/
    uint16_t                    cp_type;/* Content Protection Type */
    wiced_bt_a2dp_codec_info_t  codec;  /* codec information. */
} bt_hs_spk_audio_info_t;

typedef struct
{
    wiced_bool_t available;
    uint8_t      current_index;
    uint8_t      num_possible_values;
    uint8_t      possible_values[BT_HS_SPK_AUDIO_AVRC_APP_ATTR_VALUES_MAX]; /* Values are all 1 based */
} bt_hs_spk_audio_avrc_app_setting_t;

typedef struct
{
    wiced_bt_device_address_t       peerBda;        /* Peer bd address */

    struct
    {
        bt_hs_spk_audio_a2dp_state_t    state;      /* AVDT State machine state */
        uint16_t                        handle;     /* A2DP connection handle */
        uint16_t                        cp_type;    /* Content Protection Type */
        wiced_bt_a2dp_codec_info_t      codec_info; /* codec_info used for a2dp_sink_profile */
        wiced_bool_t                    interrupted;    /* TRUE if the stream has been interrupted
                                                           by call session. */
        wiced_bool_t                    is_streaming_started; /* WICED_TRUE if the streaming is started. */
    } a2dp;

    struct
    {
        wiced_bt_avrc_ct_connection_state_t state;  /* AVRC Connection State */
        uint16_t                            handle; /* AVRC Connection Handle */
        wiced_bt_avrc_playstate_t           playstate;  /* AVRC Playstate */
        uint8_t                             num_app_settings;
        uint8_t                             num_app_settings_init;
        uint8_t                             abs_vol_supported;
        bt_hs_spk_audio_avrc_app_setting_t  app_setting[BT_HS_SPK_AUDIO_AVRC_APP_ATTR_SETTINGS_MAX + 1];
    } avrc;

    audio_config_t                  audio_config;   /* Audio Configuration */
    uint8_t                         abs_vol;        /* Absolute volume. */
} bt_hs_spk_audio_context_t;

typedef struct bt_hs_spk_audio_context_info
{
    bt_hs_spk_audio_context_t   context[BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS];
    uint8_t                     active_audio_context_index;
} bt_hs_spk_audio_context_info_t;

/**************************************************************************************************
*  Function Declaration
**************************************************************************************************/

/*
* Function:     bt_hs_spk_audio_init
*
* Abstract:     Initialize the Audio Control Module.
*
* Input/Output:
*   bt_hs_spk_control_config_audio_t *p_config - configuration
*   BT_HS_SPK_CONTROL_LOCAL_VOLUME_CHANGE_CB *p_vol_chg_cb - local volume change callback
*
*
* Return:
*   WICED_SUCCESS - Success
*
* Notices:
*
* BIT pool calculation
* ********************
* bit_rate = 8*frame_length*fs(in kHz)/nrof_subbands/nrof_blocks,
* where,
* frame_length = 4+(4*nrof_subbands*nrof_channels)/8
*                 +[nrof_blocks*nrof_channels*bitpool/8]
*
* for the MONO and DUAL_CHANNEL channel modes
* frame_length = 4+(4*nrof_subbands*nrof_channels)/8
*                 +[(join*nrof_subbands + nrof_blocks*nrof_channels*bitpool)/8]
*
* for the STEREO a and JOIN_STEREO channel modes
* join = 1 when join stereo is used else 0
*
* for fs = 16kHz, nrof_subbands = 8, nrof_blocks = 16, nrof_channels = 1
* and channel mode = MONO
* => bit_rate = frame_length.
* => frame_length = 8+2*bitpool.
*
* Therefore, bitpool = (bit_rate-8)/2
* For bit_rate of 128kbps, bitpool = 60
*
* reference : A2DP spec v12
*/
wiced_result_t bt_hs_spk_audio_init(bt_hs_spk_control_config_audio_t *p_config, BT_HS_SPK_CONTROL_LOCAL_VOLUME_CHANGE_CB *p_vol_chg_cb);

/*
 *  Set the app current service to audio module.
 */
void bt_hs_spk_audio_app_service_set(void);

/*
 * bt_hs_spk_audio_streaming_stop
 *
 * Stop current A2DP streaming and reset the lite host immediately.
 */
void bt_hs_spk_audio_streaming_stop(void);

/*
 * bt_hs_spk_audio_streaming_recover
 *
 * Recover the interrupted streaming
 */
void bt_hs_spk_audio_streaming_recover(void);

void bt_hs_spk_audio_streaming_suspend(void);

void bt_hs_spk_audio_streaming_resume(void);

void bt_hs_spk_audio_a2dp_status_register(bt_hs_spk_audio_a2dp_state_callback_t *p_callback);

wiced_bool_t bt_hs_spk_audio_a2dp_connection_check(wiced_bt_device_address_t bd_addr, wiced_bool_t *p_connected);
wiced_bool_t bt_hs_spk_audio_avrc_connection_check(wiced_bt_device_address_t bd_addr, wiced_bool_t *p_connected);

/*
 * Check if the target link is allowed to enter sniff mode.
 *
 * @param[in]   bd_addr: peer device's BT address
 */
wiced_bool_t bt_hs_spk_audio_bt_sniff_mode_allowance_check(wiced_bt_device_address_t bd_addr);

/*
 * bt_hs_spk_audio_disconnect
 *
 * Disconnect target peer device.
 *
 * @param bdaddr - target device's BT address
 *                 If this is set to NULL, all the connected devices will be disconnected
 */
void bt_hs_spk_audio_disconnect(wiced_bt_device_address_t bdaddr);

/*
 * bt_hs_spk_audio_vse_jitter_buffer_event_handler
 *
 * Audio module Jitter buffer VSE event handler.
 */
void bt_hs_spk_audio_vse_jitter_buffer_event_handler(uint8_t *p);

/**
 * bt_hs_spk_audio_connection_check
 *
 * Check if the audio (either A2DP or AVRC) is connected.
 *
 * @param bdaddr - specific source/TG's BT address
 *                 If this is set to NULL, the return value would be TRUE if any source/TG is
 *                 connected.
 *
 * @return  WICED_TRUE
 *          WICED_FALSE
 */
wiced_bool_t bt_hs_spk_audio_connection_check(wiced_bt_device_address_t bdaddr);

/**
 * bt_hs_spk_audio_volume_sync
 *
 * Synchronize the current AVRC absolute volume with the active TG.
 *
 */
void bt_hs_spk_audio_volume_sync(void);

/**
 * bt_hs_spk_audio_volume_get
 *
 * Get current absolute volume for the active source device.
 *
 * @return  absolute volume
 */
uint8_t bt_hs_spk_audio_volume_get(void);

/**
 * Get the AVRC play state of the active context.
 *
 * @return The AVRC play state.
 */
wiced_bt_avrc_playstate_t bt_hs_spk_audio_avrc_playstate_get(void);

/**
 * bt_hs_spk_audio_a2dp_info_get
 *
 * Get the related A2DP connection information for the active source device
 * @param[out] p_info - refer to bt_hs_spk_audio_info_t
 *
 * @return  WICED_TRUE- success
 *          WICED_FALSE - there is no active source device
 */
wiced_bool_t bt_hs_spk_audio_a2dp_info_get(bt_hs_spk_audio_info_t *p_info);

/**
 * bt_hs_spk_audio_streaming_check
 *
 * Check if the device is doing Audio Streaming.
 *
 * @param bdaddr - target device's BT address
 *                 If this is set to NULL, the return value will be set to TRUE if any
 *                 audio streaming is ongoing
 *
 * @return  WICED_NOT_FOUND - if a2dp context is invalid or not found.
 *          WICED_ALREADY_CONNECTED
 *                          - if a2dp state is BT_HS_SPK_AUDIO_A2DP_STATE_START_PENDING
 *                               or BT_HS_SPK_AUDIO_A2DP_STATE_STARTED
 *          WICED_NOT_CONNECTED - Otherwise (no audio streaming is ongoing
 */
wiced_result_t bt_hs_spk_audio_streaming_check(wiced_bt_device_address_t bdaddr);

/**
 * bt_hs_spk_audio_is_a2dp_streaming_started
 *
 * Check if A2DP streaming is started
 */
wiced_bool_t bt_hs_spk_audio_is_a2dp_streaming_started(void);

/**
 * bt_hs_spk_audio_is_a2dp_streaming_interrupted
 *
 * Check if the a2dp streaming is interrupted.
 *
 * When the a2dp streaming is interrupted, the a2dp may or may not be suspended.
 *
 * @param[in]   bdaddr - sink device's address
 *                       If this is set to NULL, the return value will be set to TRUE if any
 *                       audio streaming is interrupted
 *
 * @return      WICED_TRUE
 *              WICED_FALSE
 */
wiced_bool_t bt_hs_spk_audio_is_a2dp_streaming_interrupted(wiced_bt_device_address_t bdaddr);

/**
 *
 * Volume levels passed from the application to Audio Manager should be in the range 0 to 10
 * calculating from 0 to 127 levels to 0 to 10 levels
 *
 * @param           int32_t  : vol from app.
 *
 * @return          volume in AM Level
 */
int32_t bt_hs_spk_audio_utils_abs_volume_to_am_volume(int32_t vol);

/**
 * bt_hs_spk_audio_a2dp_delay_update
 *
 * Update the A2DP delay with the active source device (by sending the delay report).
 *
 */
void bt_hs_spk_audio_a2dp_delay_update(void);

/**
 * Emulate the A2DP sink event.
 *
 * @param[in] event - refer to wiced_bt_a2dp_sink_event_t
 * @param[in] p_data - refer to wiced_bt_a2dp_sink_event_data_t
 */
void bt_hs_spk_audio_a2dp_sink_event_emulator(wiced_bt_a2dp_sink_event_t event, wiced_bt_a2dp_sink_event_data_t* p_data);

/**
 * bt_hs_spk_audio_a2dp_codec_info_to_audio_config
 *
 * Transform the A2DP codec info. to the Audio Manager audio config.
 *
 * @param[in] p_codec_info
 * @param[out] p_audio_config
 */
void bt_hs_spk_audio_a2dp_codec_info_to_audio_config(wiced_bt_a2dp_codec_info_t *p_codec_info, audio_config_t *p_audio_config);

/**
 * bt_hs_spk_audio_audio_manager_stream_start
 *
 * Start the external codec via Audio Manager
 *
 * @param[in] p_audio_config
 */
void bt_hs_spk_audio_audio_manager_stream_start(audio_config_t *p_audio_config);

/**
 * Stop and close the external codec via the Audio Manager module.
 */
void bt_hs_spk_audio_audio_manager_stream_stop(void);

/**
 * bt_hs_spk_audio_audio_manager_stream_volume_set
 *
 * Set the external codec streaming gain via the Audio Manager module.
 *
 * @param[in] am_vol_level - from AM_VOL_LEVEL_LOW to AM_VOL_LEVEL_HIGH
 * @param[in] am_vol_effect_event - indicate the reason of VOLUME_EFFECT,
 *            it should be handled in user application if VOLUME_EFFECT enabled.
 */
void bt_hs_spk_audio_audio_manager_stream_volume_set(int32_t am_vol_level, uint8_t am_vol_effect_event);

/**
 * bt_hs_spk_audio_audio_context_info_set
 *
 * Set the context of Audio Context
 *
 * Note: Do NOT use this utility unless you certainly understand what you are doing.
 *       Using this utility MAY cause unexpected behavior and crash.
 *
 * @param[in] p_info
 */
void bt_hs_spk_audio_audio_context_info_set(bt_hs_spk_audio_context_info_t *p_info);

/**
 * bt_hs_spk_audio_audio_context_info_get
 *
 * Get the content of Audio Context
 *
 * @param[out] p_info
 */
void bt_hs_spk_audio_audio_context_info_get(bt_hs_spk_audio_context_info_t *p_info);

/**
 * bt_hs_spk_audio_activate_decode_thread
 *
 * Activate decode thread
 */
void bt_hs_spk_audio_activate_decode_thread(void);

/**
 * bt_hs_spk_audio_current_streaming_addr_get
 *
 * Get the current active streaming address
 */
wiced_bool_t bt_hs_spk_audio_current_streaming_addr_get(wiced_bt_device_address_t bdaddr);

wiced_result_t bt_hs_spk_audio_send_unit_info(void);
wiced_result_t bt_hs_spk_audio_get_attr_info(void);
wiced_result_t bt_hs_spk_audio_button_handler_volume_up(void);
wiced_result_t bt_hs_spk_audio_button_handler_volume_down(void);
wiced_result_t bt_hs_spk_audio_button_handler_pause_play(void);
wiced_result_t bt_hs_spk_audio_button_handler(app_service_action_t action);
wiced_result_t bt_hs_spk_audio_handler_skip_stop_fastforward_rewind(uint8_t op_id, uint8_t key_state);

//=================================================================================================
//  End of File (bt_hs_spk_audio.h)
//=================================================================================================
