/*
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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


/** @file
 *
 *  This file implements the handsfree functions
 *
 */

#include "bt_hs_spk_handsfree.h"

#include <pthread.h>

#include "wiced_bt_hfp_hf.h"
#include "wiced_timer.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sco.h"

#include "bt_hs_spk_control.h"
#include "bt_hs_spk_handsfree_utils.h"
#include "bt_hs_spk_audio.h"
#include "wiced_result.h"
#include "wiced_audio_manager.h"
#include "app_bt_utils.h"
#include "platform_audio_interface.h"
#include "app_bt_utils.h"
#include "alsa_playback.h"
#include "log.h"

#define BT_HS_SPK_SCO_INDEX                                           0
#define BT_HS_SPK_HANDSFREE_SCO_CONNECTING_STATE_PROTECTION_TIMEOUT 500  // ms

typedef void (*bt_hs_spk_handsfree_btm_event_sco_handler_t)(handsfree_app_state_t *p_ctx, wiced_bt_management_evt_data_t *p_data);
typedef void (*bt_hs_spk_handsfree_event_handler)(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t *p_data);

typedef struct
{
    handsfree_app_state_t       context[WICED_BT_HFP_HF_MAX_CONN];
    handsfree_app_state_t       *p_active_context;
    wiced_app_service_t         app_service;
    wiced_bt_voice_path_setup_t sco_voice_path;

    /* Timer used to protect the SCO connection from connecting to connected.
     * Due to the limit of maximum concurrent SCO connection numbers in the controller,
     * the duration shall be protected to avoid accepting another SCO connection request
     * when a SCO connection is already under connecting state. */
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE) || WIN_EMULATOR
    wiced_timer_t   sco_connecting_protection_timer;
#else
    wiced_timer_t   sco_connecting_protection_timer;
#endif

    int32_t                     stream_id;

    /* BTM SCO event handlers */
    bt_hs_spk_handsfree_btm_event_sco_handler_t btm_sco_event_handler[BTM_SCO_CONNECTION_CHANGE_EVT - BTM_SCO_CONNECTED_EVT + 1];

    /* Handsfree event handlers */
    bt_hs_spk_handsfree_event_handler handsfree_event_handler[WICED_BT_HFP_HF_BIND_EVT + 2];

    bt_hs_spk_control_config_hfp_t  config;
    BT_HS_SPK_CONTROL_LOCAL_VOLUME_CHANGE_CB    *p_local_volume_change_cb;
    bt_hs_spk_handsfree_mic_data_add_cb_t       *p_mic_data_add_cb;
} bt_hs_spk_handsfree_cb_t;

/******************************************************
 *               Function Declarations
 ******************************************************/

static void handsfree_event_callback( wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data);
//void hci_control_send_sco_confirmation_request_evt( BD_ADDR bda, uint16_t sco_index );
static wiced_result_t bt_audio_hfp_button_event_handler(app_service_action_t action);
static wiced_result_t bt_audio_hfp_reject_call_active_no(handsfree_app_state_t *p_ctx);

static void bt_hs_spk_handsfree_active_call_session_set(handsfree_app_state_t *p_ctx);
static wiced_result_t   bt_hs_spk_handsfree_at_cmd_send(uint16_t handle, char *cmd, uint8_t arg_type, uint8_t arg_format, const char *p_arg, int16_t int_arg);
static void bt_hs_spk_handsfree_audio_connection_establish(handsfree_app_state_t *p_ctx);
static void bt_hs_spk_handsfree_call_hang_up(handsfree_app_state_t *p_ctx);
static void bt_hs_spk_handsfree_event_handler_ag_feature_support(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_battery_status_ind(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_cnum(handsfree_app_state_t* p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_bind(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_binp(handsfree_app_state_t* p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_cops(handsfree_app_state_t* p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_clcc(handsfree_app_state_t* p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_call_setup(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_codec_set(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_connection_state(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_clip_ind(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_inband_ring_state(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_ring(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_rssi_ind(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_network_name(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void bt_hs_spk_handsfree_event_handler_voice_recognition(handsfree_app_state_t* p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);

static void             bt_hs_spk_handsfree_event_handler_service_state(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_service_type(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static void             bt_hs_spk_handsfree_event_handler_volume_change(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data);
static wiced_bool_t     bt_hs_spk_handsfree_incoming_call_notification_handler(handsfree_app_state_t *p_ctx);
static wiced_bool_t     bt_hs_spk_handsfree_outgoing_call_notification_handler(handsfree_app_state_t *p_ctx);
#if defined(STACK_INSIDE_FREE_RTOS) && (STACK_INSIDE_FREE_RTOS == TRUE)
static void             bt_hs_spk_handsfree_sco_connecting_protection_timeout_cb(uint32_t arg);
#elif defined(WIN_EMULATOR)
static void bt_hs_spk_handsfree_sco_connecting_protection_timeout_cb(WICED_TIMER_PARAM_TYPE arg);
#else
static void bt_hs_spk_handsfree_sco_connecting_protection_timeout_cb(TIMER_PARAM_TYPE arg);
#endif
static void             bt_hs_spk_handsfree_sco_management_callback_connected(handsfree_app_state_t *p_ctx, wiced_bt_management_evt_data_t *p_data);
static void             bt_hs_spk_handsfree_sco_management_callback_connection_request(handsfree_app_state_t *p_ctx, wiced_bt_management_evt_data_t *p_data);
static void             bt_hs_spk_handsfree_sco_management_callback_disconnected(handsfree_app_state_t *p_ctx, wiced_bt_management_evt_data_t *p_data);

static uint8_t  bt_hs_spk_handsfree_speaker_volume_level_get(handsfree_app_state_t *p_ctx);
static void     bt_hs_spk_handsfree_speaker_volume_level_set(handsfree_app_state_t *p_ctx, uint8_t volume_level);
static void bt_hs_spk_handsfree_sco_data_app_callback(uint16_t sco_channel, uint16_t length, uint8_t* p_data);
wiced_result_t hci_control_HFP_inband_status_event(uint16_t sco_idx, uint8_t status);
wiced_result_t hci_control_HFP_sco_connection_event(uint16_t sco_idx, uint8_t isconnected, uint8_t spk_vol, uint8_t mic_vol);
wiced_result_t hci_control_HFP_connection_status_event(wiced_bt_device_address_t bd_addr, uint16_t handle, uint8_t isconnected);
wiced_result_t hci_control_HFP_callsetup_status_event(uint16_t sco_idx, uint8_t active_Call, uint8_t  held_call, uint8_t call_setup_status);
wiced_result_t hci_control_HFP_volume_change_event(wiced_bt_device_address_t bd_addr, uint8_t vol_type, uint8_t vol, uint8_t percent);
wiced_result_t hci_control_HFP_call_alert_event(wiced_bt_device_address_t bd_addr, uint8_t alert_type, uint8_t* caller_num);
wiced_result_t hci_control_HFP_volume_up_down_event(wiced_bt_device_address_t bd_addr, uint8_t vol, uint8_t percent, uint8_t am_vol, uint8_t is_up);
wiced_result_t hci_control_HFP_voice_recognition_status_event(uint16_t sco_idx, uint8_t status);
extern wiced_result_t wiced_transport_send_data(uint16_t type, uint8_t* p_data, uint16_t data_size);

/******************************************************
 *               Variables Definitions
 ******************************************************/

static bt_hs_spk_handsfree_cb_t bt_hs_spk_handsfree_cb;
uint8_t vrec_enable = FALSE;
pthread_t sco_write_thread_handle;
pthread_cond_t cond_call_active = PTHREAD_COND_INITIALIZER; 
pthread_mutex_t cond_lock = PTHREAD_MUTEX_INITIALIZER;
uint8_t *mic_data = NULL;
int mic_buffer_size = ALSA_MIC_FRAMES_SIZE * 4;
static bool sco_flag = true;  //flag to control wiced_bt_sco_write_buffer
uint32_t count = 0;
/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 * Initialize HFP structure
 */
static void bt_hs_spk_handsfree_cb_init_context(void)
{
    uint16_t i = 0;
    handsfree_app_state_t *p_context;

    for (i = 0 ; i < WICED_BT_HFP_HF_MAX_CONN ; i++)
    {
        p_context = &bt_hs_spk_handsfree_cb.context[i];

        memset(p_context, 0, sizeof(handsfree_app_state_t));

        p_context->call_setup           = WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE;
        p_context->connection_status    = WICED_BT_HFP_HF_STATE_DISCONNECTED;
        p_context->spkr_volume          = (WICED_HANDSFREE_VOLUME_MAX - WICED_HANDSFREE_VOLUME_MIN + 1) / 2;
        p_context->ringtone_volume      = (WICED_HANDSFREE_VOLUME_MAX - WICED_HANDSFREE_VOLUME_MIN + 1) / 2;
        p_context->mic_volume           = (WICED_HANDSFREE_VOLUME_MAX - WICED_HANDSFREE_VOLUME_MIN + 1) / 2;
        p_context->sco_index            = WICED_INVALID_SCO_INDEX;

#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        p_context->audio_config.sr              = AM_PLAYBACK_SR_16K;
#else
        p_context->audio_config.sr              = AM_PLAYBACK_SR_8K;
#endif
        p_context->audio_config.channels        = 1;
        p_context->audio_config.bits_per_sample = DEFAULT_BITSPSAM;
        p_context->audio_config.volume          = AM_VOL_LEVEL_HIGH - 2;
        p_context->audio_config.mic_gain        = AM_VOL_LEVEL_HIGH - 2;

#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        p_context->sco_params.max_latency       = 0x000D;   /* Latency: 13 ms (T2) refer to Table 5.11 of HFP v1.7.1 */
#else
        p_context->sco_params.max_latency       = 0x000C;   /* Latency: 12 ms ( HS/HF can use EV3, 2-EV3, 3-EV3 ) ( S4 ),
                                                               Refer to Table 5.9 of HFP v1.7.1 */
#endif
        p_context->sco_params.packet_types      = WICED_SCO_PKT_TYPES;
        p_context->sco_params.retrans_effort    = WICED_ESCO_RETRANS_QUALITY;   /* Use retransmit effort link quality to support S4/T2. */
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        p_context->sco_params.use_wbs           = WICED_TRUE;
#else
        p_context->sco_params.use_wbs           = WICED_FALSE;
#endif
    }
}

/*
 * bt_hs_spk_handsfree_sco_data_app_callback
 */
static void bt_hs_spk_handsfree_sco_data_app_callback(uint16_t sco_channel, uint16_t length, uint8_t *p_data)
{
    uint8_t *p_mic_data = NULL;
    uint16_t ret_value = WICED_BT_ERROR;
    uint32_t mic_data_len = 0;
    uint16_t micLength = length;

    /* Check if the active call session exists. */
    if (bt_hs_spk_handsfree_cb.p_active_context != NULL)
    {
        hf_audio_process_sco_data(sco_channel, length,  p_data);

        if((bt_hs_spk_handsfree_cb.p_active_context->call_active) && (bt_hs_spk_handsfree_cb.p_active_context->sco_connected == WICED_TRUE)){
            mic_data_len = audio_capture_mic_data(mic_data, micLength);
            if (mic_data_len > 0){
    #ifdef AUDIO_DEBUG
                TRACE_LOG("Get mic data length = (%d)", mic_data_len);
    #endif
                ret_value = bt_hs_spk_handsfree_send_sco_data(mic_data, mic_data_len);
                usleep(5000);  //workaround for RPI4 Uart Issue
                if (WICED_BT_SUCCESS != ret_value)
                {
                    TRACE_WNG("bt_hs_spk_handsfree_send_sco_data error, result = %d", ret_value);
                }
            }
        }
    }
}

static void bt_hs_spk_handsfree_cb_init(void)
{
    // context
    bt_hs_spk_handsfree_cb_init_context();

    // active context
    bt_hs_spk_handsfree_active_call_session_set(NULL);

    // wiced app service
    bt_hs_spk_handsfree_cb.app_service.active_service = SERVICE_BT_HFP;
    bt_hs_spk_handsfree_cb.app_service.button_handler = bt_audio_hfp_button_event_handler;

    // SCO voice path
    if (bt_hs_spk_get_audio_sink() == AM_UART)
    {

        bt_hs_spk_handsfree_cb.sco_voice_path.path = WICED_BT_SCO_OVER_HCI;
        bt_hs_spk_handsfree_cb.sco_voice_path.p_sco_data_cb = &bt_hs_spk_handsfree_sco_data_app_callback;

    }
    else
    {
        bt_hs_spk_handsfree_cb.sco_voice_path.path = WICED_BT_SCO_OVER_PCM;
        bt_hs_spk_handsfree_cb.sco_voice_path.p_sco_data_cb = NULL;


    }

    // stream id used for Audio Manager
    bt_hs_spk_handsfree_cb.stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;

    // BTM SCO event handler
    bt_hs_spk_handsfree_cb.btm_sco_event_handler[BTM_SCO_CONNECTED_EVT - BTM_SCO_CONNECTED_EVT] = \
            &bt_hs_spk_handsfree_sco_management_callback_connected;

    bt_hs_spk_handsfree_cb.btm_sco_event_handler[BTM_SCO_DISCONNECTED_EVT - BTM_SCO_CONNECTED_EVT] = \
            &bt_hs_spk_handsfree_sco_management_callback_disconnected;

    bt_hs_spk_handsfree_cb.btm_sco_event_handler[BTM_SCO_CONNECTION_REQUEST_EVT - BTM_SCO_CONNECTED_EVT] = \
            &bt_hs_spk_handsfree_sco_management_callback_connection_request;

    bt_hs_spk_handsfree_cb.btm_sco_event_handler[BTM_SCO_CONNECTION_CHANGE_EVT - BTM_SCO_CONNECTED_EVT] = NULL;

    // Handsfree event handlers
    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_CONNECTION_STATE_EVT] = \
            &bt_hs_spk_handsfree_event_handler_connection_state;

    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_AG_FEATURE_SUPPORT_EVT] = \
            &bt_hs_spk_handsfree_event_handler_ag_feature_support;

    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_SERVICE_STATE_EVT] = \
            &bt_hs_spk_handsfree_event_handler_service_state;

    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_SERVICE_TYPE_EVT] = \
            &bt_hs_spk_handsfree_event_handler_service_type;

    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_CALL_SETUP_EVT] = \
            &bt_hs_spk_handsfree_event_handler_call_setup;

    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_RING_EVT] = \
            &bt_hs_spk_handsfree_event_handler_ring;

    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_INBAND_RING_STATE_EVT] = \
            &bt_hs_spk_handsfree_event_handler_inband_ring_state;

    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_RSSI_IND_EVT] = \
            &bt_hs_spk_handsfree_event_handler_rssi_ind;

    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_BATTERY_STATUS_IND_EVT] = \
            &bt_hs_spk_handsfree_event_handler_battery_status_ind;

    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_VOLUME_CHANGE_EVT] = \
            &bt_hs_spk_handsfree_event_handler_volume_change;

    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_CLIP_IND_EVT]  = \
            &bt_hs_spk_handsfree_event_handler_clip_ind;

    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HFP_CODEC_SET_EVT] =\
            &bt_hs_spk_handsfree_event_handler_codec_set;

    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HFP_ACTIVE_CALL_EVT] = \
            &bt_hs_spk_handsfree_event_handler_clcc;
    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_OK_EVT] = NULL;
    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_ERROR_EVT] = NULL;
    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_CME_ERROR_EVT] = NULL;
    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_CNUM_EVT] = \
            &bt_hs_spk_handsfree_event_handler_cnum;
    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_BINP_EVT] = \
            &bt_hs_spk_handsfree_event_handler_binp;
    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_VOICE_RECOGNITION_EVT] = \
            &bt_hs_spk_handsfree_event_handler_voice_recognition;
    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_BIND_EVT] = \
            &bt_hs_spk_handsfree_event_handler_bind;
    bt_hs_spk_handsfree_cb.handsfree_event_handler[WICED_BT_HFP_HF_COPS_EVT] = \
            &bt_hs_spk_handsfree_event_handler_cops;
}

/*
 * Find and return handsfree_app_state_t using connection handle
 */
handsfree_app_state_t* get_context_ptr_from_handle(uint16_t handle)
{
    uint16_t i = 0;

    for (i = 0 ; i < WICED_BT_HFP_HF_MAX_CONN ; i++)
    {
        if ((bt_hs_spk_handsfree_cb.context[i].rfcomm_handle == handle) &&
            (bt_hs_spk_handsfree_cb.context[i].connection_status >= WICED_BT_HFP_HF_STATE_CONNECTED))
        {
            return &bt_hs_spk_handsfree_cb.context[i];
        }
    }

    return NULL;
}

/*
 * get_context_ptr_from_address
 *
 * Find the corresponding context.
 */
static handsfree_app_state_t *get_context_ptr_from_address(wiced_bt_device_address_t bdaddr, wiced_bool_t allocate)
{
    uint16_t i;

    /* Check if the corresponding context exists. */
    for (i = 0 ; i < WICED_BT_HFP_HF_MAX_CONN ; i++)
    {
        if (memcmp((void *) bt_hs_spk_handsfree_cb.context[i].peer_bd_addr,
                   (void *) bdaddr,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            return &bt_hs_spk_handsfree_cb.context[i];
        }
    }

    if (allocate == WICED_FALSE)
    {
        return NULL;
    }

    /* Allocate a free space. */
    for (i = 0 ; i < WICED_BT_HFP_HF_MAX_CONN ; i++)
    {
        if (bt_hs_spk_handsfree_cb.context[i].connection_status == WICED_BT_HFP_HF_STATE_DISCONNECTED)
        {
            memcpy((void *) bt_hs_spk_handsfree_cb.context[i].peer_bd_addr,
                   (void *) bdaddr,
                   sizeof(wiced_bt_device_address_t));

            bt_hs_spk_handsfree_cb.context[i].spkr_volume       = (WICED_HANDSFREE_VOLUME_MAX - WICED_HANDSFREE_VOLUME_MIN + 1) / 2;
            bt_hs_spk_handsfree_cb.context[i].ringtone_volume   = (WICED_HANDSFREE_VOLUME_MAX - WICED_HANDSFREE_VOLUME_MIN + 1) / 2;
            bt_hs_spk_handsfree_cb.context[i].mic_volume        = (WICED_HANDSFREE_VOLUME_MAX - WICED_HANDSFREE_VOLUME_MIN + 1) / 2;

#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
            bt_hs_spk_handsfree_cb.context[i].audio_config.sr               = AM_PLAYBACK_SR_16K;
#else
            bt_hs_spk_handsfree_cb.context[i].audio_config.sr               = AM_PLAYBACK_SR_8K;
#endif
            bt_hs_spk_handsfree_cb.context[i].audio_config.volume           = AM_VOL_LEVEL_HIGH - 2;
            bt_hs_spk_handsfree_cb.context[i].audio_config.mic_gain         = AM_VOL_LEVEL_HIGH - 2;
            bt_hs_spk_handsfree_cb.context[i].audio_config.bits_per_sample  = DEFAULT_BITSPSAM;

            bt_hs_spk_handsfree_cb.context[i].call_hanging_up   = WICED_FALSE;

            return &bt_hs_spk_handsfree_cb.context[i];
        }
    }

    return NULL;
}

/*
 * Find and return handsfree_app_state_t using sco index
 */
handsfree_app_state_t* get_context_ptr_from_sco_index(uint16_t sco_index)
{
    int i = 0;

    if( sco_index == WICED_INVALID_SCO_INDEX )
        return NULL;

    for( ; i< WICED_BT_HFP_HF_MAX_CONN; i++ )
    {
        if( sco_index == bt_hs_spk_handsfree_cb.context[i].sco_index && bt_hs_spk_handsfree_cb.context[i].connection_status != WICED_BT_HFP_HF_STATE_DISCONNECTED)
        {
            return &bt_hs_spk_handsfree_cb.context[i];
        }
    }
    return NULL;
}

/*
 * Initialize HFP
 *
 * @param[in]   p_config - configuration
 * @param[in]   *p_vol_chg_cb - local volume change callback
 */
void handsfree_hfp_init(bt_hs_spk_control_config_hfp_t *p_config, BT_HS_SPK_CONTROL_LOCAL_VOLUME_CHANGE_CB *p_vol_chg_cb)
{
    wiced_result_t result = WICED_BT_ERROR;
    wiced_bt_hfp_hf_config_data_t config;
    uint8_t index;

    bt_hs_spk_handsfree_cb_init();

    memcpy((void *) &bt_hs_spk_handsfree_cb.config,
           (void *) p_config,
           sizeof(bt_hs_spk_control_config_hfp_t));

    bt_hs_spk_handsfree_cb.p_local_volume_change_cb = p_vol_chg_cb;
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
    /* Perform the rfcomm init before hf and spp start up */
    result = wiced_bt_rfcomm_set_buffer_pool(p_config->rfcomm.buffer_size, p_config->rfcomm.buffer_count);
    if (result != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("Error Initializing RFCOMM - HFP failed\n");
        return;
    }
#endif
    result = WICED_BT_SUCCESS;
    /* Init. the SCO connecting duration protection timer. */
    result = wiced_init_timer(&bt_hs_spk_handsfree_cb.sco_connecting_protection_timer,
                              bt_hs_spk_handsfree_sco_connecting_protection_timeout_cb,
                              0,
                              WICED_MILLI_SECONDS_TIMER);
    if (result != WICED_SUCCESS)
    {
        WICED_BT_TRACE("Error Initializing SCO connecting protection timer1 (%d)\n", result);
        assert(0);
        return;
    }

    config.feature_mask     = p_config->feature_mask;
    config.speaker_volume   = bt_hs_spk_handsfree_speaker_volume_level_get(&bt_hs_spk_handsfree_cb.context[0]);
    config.mic_volume       = bt_hs_spk_handsfree_cb.context[0].mic_volume;
    config.num_server       = WICED_BT_HFP_HF_MAX_CONN;

    for (index = 0; index < WICED_BT_HFP_HF_MAX_CONN; index ++)
    {
        config.scn[index]   = WICED_HANDSFREE_SCN;
        config.uuid[index]   = UUID_SERVCLASS_HF_HANDSFREE;
    }

    result = wiced_bt_hfp_hf_init(&config, handsfree_event_callback);
    WICED_BT_TRACE("[INIT] wiced_bt_hfp_hf_init [Done]\n",__func__);

    /* init mic data buffer */
    mic_data = (uint8_t *)malloc(mic_buffer_size);
    memset(mic_data, 0, mic_buffer_size);

    WICED_BT_TRACE("[INIT] %s %d [Done]\n",__func__, result);
}

static wiced_result_t setup_call_audio()
{
    wiced_result_t result = WICED_SUCCESS;

    return result;
}

/*
 * Handle the BTM_SCO_CONNECTION_REQUEST_EVT event. 
 */
static void bt_hs_spk_handsfree_sco_management_callback_connection_request(handsfree_app_state_t *p_ctx, wiced_bt_management_evt_data_t *p_data)
{
    wiced_bool_t sco_protected = WICED_FALSE;
    wiced_app_service_t*  service = NULL;
    wiced_result_t result;

    app_set_current_service(&bt_hs_spk_handsfree_cb.app_service);

    /* Check if SCO connecting/disconnecting protection timer is running*/
    if (wiced_is_timer_in_use(&bt_hs_spk_handsfree_cb.sco_connecting_protection_timer))
    {
        sco_protected = WICED_TRUE;
    }
    else
    {
        if (bt_hs_spk_handsfree_cb.p_active_context != NULL)
        {
            /* Check if there is an active call, the active call will be pushed to held and remove existent SCO connection
             * in call_notification_handler, protect it until the SCO disconnecting procedure done */
            if (bt_hs_spk_handsfree_cb.p_active_context->call_active && bt_hs_spk_handsfree_cb.p_active_context != p_ctx)
            {
                sco_protected = WICED_TRUE;
            }
        }
    }

    WICED_BT_TRACE("bt_hs_spk_handsfree_sco_management_callback_connection_request (%d, %d, 0x%08X 0x%08X, %d)\n",
        p_data->sco_connection_request.sco_index,
        p_data->sco_connection_request.link_type,
        bt_hs_spk_handsfree_cb.p_active_context,
        p_ctx,
        sco_protected);

    result = setup_call_audio();
    /* Check parameter. */
    if ((p_data->sco_connection_request.link_type != BTM_LINK_TYPE_SCO) &&
        (p_data->sco_connection_request.link_type != BTM_LINK_TYPE_ESCO))
    {
        /* Reject this connection request. */
        wiced_bt_sco_accept_connection(p_ctx->sco_index,
                                       WICED_BT_SCO_CONNECTION_REJECT_RESOURCES,
                                       (wiced_bt_sco_params_t *) &p_ctx->sco_params);

        WICED_BT_TRACE("[%s] : Rejuct BTM_SCO_CONNECTION_REQUEST_EVT",__func__);
        return;
    }

    /* Check if the SCO is already in the connecting state. */
    if (sco_protected)
    {
        WICED_BT_TRACE("[%s] : Rejuct sco_protected",__func__);
        return;
    }

    /* If this SCO connection is for the active call session and the active call session
     * already has a SCO connection, reject this new SCO connection request if the
     * requested SCO index is different from the existent SCO connection. */
    if ((bt_hs_spk_handsfree_cb.p_active_context == p_ctx) &&
        (bt_hs_spk_handsfree_cb.p_active_context->sco_connected) &&
        (bt_hs_spk_handsfree_cb.p_active_context->sco_index != p_data->sco_connection_request.sco_index))
    {
        /* Reject this connection request. */
        wiced_bt_sco_accept_connection(p_ctx->sco_index,
                                       WICED_BT_SCO_CONNECTION_REJECT_RESOURCES,
                                       (wiced_bt_sco_params_t *) &p_ctx->sco_params);
        WICED_BT_TRACE("[%s] : Rejuct status",__func__);
        return;
    }

    /* Stop existent Audio Streaming if there is. */
    bt_hs_spk_audio_streaming_stop();

    /*
     * Note:
     * Unless the parameter in the SCO connection request is invalid, do NOT
     * reject the incoming SCO connection request. Otherwise, the iPhone device
     * will NOT trigger the audio connection with the IUT anymore.
     */
    /* In the multi-point scenario, the sniff attempt(s) for iPhone (only one sniff attempt)
     * may be interrupted by the ongoing SCO data due to the priority setting in the controller.
     * That is, if one Active Call Session exists with the voice connection, the other ACL
     * connection which is in sniff mode may loss the sniff attempts and leads to the ACL
     * disconnection (caused by supervision timeout) or the ACL data may be lost due to the
     * collision of sniff attempt and the SCO data.
     * To achieve this, we need to set all the other ACL links to active mode and forbidden
     * the sniff mode during the voice call session.
     *
     * Moreover, the LMP Unsniff command shall be sent before accepting the SCO connection. */
    bt_hs_spk_control_acl_link_policy_sniff_mode_set_exclusive(p_ctx->peer_bd_addr, WICED_FALSE);
    bt_hs_spk_control_bt_power_mode_set_exclusive(WICED_TRUE, p_ctx->peer_bd_addr, NULL);
    if(result == WICED_SUCCESS)
    {
        wiced_bt_sco_accept_connection(p_ctx->sco_index, WICED_BT_SCO_CONNECTION_ACCEPT, (wiced_bt_sco_params_t *) &p_ctx->sco_params);
        /* Start the SCO connecting protection timer to protect this duration. */
        result = wiced_start_timer(&bt_hs_spk_handsfree_cb.sco_connecting_protection_timer,
                                   BT_HS_SPK_HANDSFREE_SCO_CONNECTING_STATE_PROTECTION_TIMEOUT);

        if (result != WICED_SUCCESS)
        {
            WICED_BT_TRACE("Error fail to start the SCO connecting protection timer (%d)\n", result);
        }
        else{
            WICED_BT_TRACE("wiced_bt_sco_accept_connection success!!!\n", result);
        }
    }
    else
    {
        wiced_bt_sco_accept_connection(p_ctx->sco_index, WICED_BT_SCO_CONNECTION_REJECT_RESOURCES, (wiced_bt_sco_params_t *) &p_ctx->sco_params);
        WICED_BT_TRACE("[Check]WICED_BT_SCO_CONNECTION_REJECT_RESOURCES %d\n", result);
    }
}

static void bt_hs_spk_handsfree_sco_management_callback_connected(handsfree_app_state_t *p_ctx, wiced_bt_management_evt_data_t *p_data)
{
    wiced_bool_t allowed = WICED_FALSE;
    wiced_bt_dev_status_t status  = WICED_BT_ERROR;

    WICED_BT_TRACE("bt_hs_spk_handsfree_sco_management_callback_connected (%d, 0x%08X 0x%08X, %d)\n",
                   p_data->sco_connected.sco_index,
                   bt_hs_spk_handsfree_cb.p_active_context,
                   p_ctx,
                   p_ctx->call_setup);

    /* Stop the SCO protection timer. */
    if (wiced_is_timer_in_use(&bt_hs_spk_handsfree_cb.sco_connecting_protection_timer))
    {
        wiced_stop_timer(&bt_hs_spk_handsfree_cb.sco_connecting_protection_timer);
    }

    /* Note: Do NOT delete the following debug message.
     * This debug message is used for PTS automation. */
    /* Verify in-band ringtone status from PUART Log. */
    if ((p_ctx->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING) ||
        (p_ctx->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING) ||
        (p_ctx->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING))
    {
        WICED_BT_TRACE("In-band ringtone is audible\n");
        hci_control_HFP_inband_status_event(p_data->sco_connected.sco_index, WICED_SUCCESS);
    }

    /* Update information. */
    p_ctx->sco_connected = WICED_TRUE;

    if (p_ctx->sco_params.use_wbs == WICED_TRUE)
    {
        p_ctx->audio_config.sr = AM_PLAYBACK_SR_16K;
    }
    else
    {
        p_ctx->audio_config.sr = AM_PLAYBACK_SR_8K;
    }

    p_ctx->audio_config.volume = bt_hs_spk_handsfree_utils_hfp_volume_to_am_volume(bt_hs_spk_handsfree_speaker_volume_level_get(p_ctx));
    p_ctx->audio_config.mic_gain = bt_hs_spk_handsfree_utils_hfp_volume_to_am_volume(p_ctx->mic_volume);
    p_ctx->audio_config.sink = bt_hs_spk_get_audio_sink();

    /* Note: Do NOT delete the following debug message.
     * This debug message is used for PTS automation. */
    WICED_BT_TRACE("HF Volume Current Level (%d, %d)\n",
                   bt_hs_spk_handsfree_speaker_volume_level_get(p_ctx),
                   p_ctx->mic_volume);
    hci_control_HFP_sco_connection_event(p_data->sco_connected.sco_index, p_ctx->sco_connected, bt_hs_spk_handsfree_speaker_volume_level_get(p_ctx), p_ctx->mic_volume);

    /*
     * To support multi-point, we cannot reject the incoming SCO connection request due to the
     * constraint of iPhone devices.
     * That is, if one SCO connection exists for one phone and the 2nd SCO connection request comes
     * from the iPhone, we have to accept it but not to handle the SCO data from the 2nd AG.
     * We need to reject the 2nd AG's SCO connection according to the multi-point behavior
     * defined in the development document.
     */
    /* Check the purpose of this SCO connection. */
    switch (p_ctx->call_setup)
    {
    case WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE:
        /* Check if the active call session exists. */
        if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
        {
            allowed = WICED_TRUE;
        }
        else
        {
            if (bt_hs_spk_handsfree_cb.p_active_context == p_ctx)
            {
                allowed = WICED_TRUE;
            }
            else
            {
                /* Check if the active call session already has the audio connection. */
                if (bt_hs_spk_handsfree_cb.p_active_context->sco_connected)
                {
                    /* Terminate this audio connection. */
                    status = wiced_bt_sco_remove(p_ctx->sco_index);
                    WICED_BT_TRACE("wiced_bt_sco_remove (%d, %d)\n", p_ctx->sco_index, status);
                    allowed = WICED_FALSE;
                }
                else
                {
                    /* Accept the SCO Connection. */
                    allowed = WICED_TRUE;
                }
            }
        }
        break;

    /* This SCO connection is used for the incoming call in-band ring tone. */
    case WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING:
        allowed = bt_hs_spk_handsfree_incoming_call_notification_handler(p_ctx);
        break;

    /* This SCO connection is used for the outgoing call in-band alert. */
    case WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING:
    case WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING:
        allowed = bt_hs_spk_handsfree_outgoing_call_notification_handler(p_ctx);
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_WAITING:
        break;

    default:
        break;
    }

    if (allowed == WICED_FALSE)
    {
        return;
    }

#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
    bt_hs_spk_pm_disable();
#endif

    bt_hs_spk_handsfree_active_call_session_set(p_ctx);

    /* Configure the Audio Manager. */
    bt_hs_spk_handsfree_audio_manager_stream_start(&p_ctx->audio_config);

#ifdef NREC_ENABLE
    if (bt_hs_spk_handsfree_audio_manager_nrec_enable() == WICED_TRUE)
    {
        /* Disable the NREC on peer device*/
        if (bt_hs_spk_handsfree_at_cmd_send(p_ctx->rfcomm_handle,
                                            "+NREC",
                                            WICED_BT_HFP_HF_AT_SET,
                                            WICED_BT_HFP_HF_AT_FMT_INT,
                                            NULL,
                                            0) != WICED_SUCCESS)
        {
            WICED_BT_TRACE("NREC disable AT CMD to AG failed\n");
        }
    }
#endif // NREC_ENABLE

    /* Set current service to HFP.
     * Sometimes, the HFP call setup event (no corresponding +CIEV command issued by AG)
     * will NOT be received in OOR reconnection scenario. We need to set to active service
     * to HFP once the SCO/eSCO connection is connected. */
    app_set_current_service(&bt_hs_spk_handsfree_cb.app_service);
}

static void bt_hs_spk_handsfree_sco_management_callback_disconnected(handsfree_app_state_t *p_ctx, wiced_bt_management_evt_data_t *p_data)
{
    wiced_bt_dev_status_t status = WICED_BT_ERROR;
    WICED_BT_TRACE("bt_hs_spk_handsfree_sco_management_callback_disconnected (%d, 0x%08X, 0x%08X)\n",
                   p_data->sco_disconnected.sco_index,
                   bt_hs_spk_handsfree_cb.p_active_context,
                   p_ctx);


    /* Stop the SCO protection timer. */
    if (wiced_is_timer_in_use(&bt_hs_spk_handsfree_cb.sco_connecting_protection_timer))
    {
        wiced_stop_timer(&bt_hs_spk_handsfree_cb.sco_connecting_protection_timer);
    }

    if (bt_hs_spk_handsfree_cb.p_active_context == p_ctx)
    {
        uint16_t i = 0;
        /* Check if there is another active call session. */
        for (i = 0 ; i < WICED_BT_HFP_HF_MAX_CONN ; i++)
        {
            if (p_ctx != &bt_hs_spk_handsfree_cb.context[i])
            {
                if ((bt_hs_spk_handsfree_cb.context[i].sco_connected == WICED_TRUE) ||
                    (bt_hs_spk_handsfree_cb.context[i].call_active == WICED_TRUE) ||
                    (bt_hs_spk_handsfree_cb.context[i].call_held == WICED_TRUE) ||
                    (bt_hs_spk_handsfree_cb.context[i].call_setup >= WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING))
                {
                    break;
                }
            }
        }

        if (i >= WICED_BT_HFP_HF_MAX_CONN)
        {   /* There is no other active call session. */
            bt_hs_spk_handsfree_active_call_session_set(NULL);

            /* Set current service back to A2DP. */
            bt_hs_spk_audio_app_service_set();
        }
        else
        {
            bt_hs_spk_handsfree_active_call_session_set(&bt_hs_spk_handsfree_cb.context[i]);
        }
    }

    p_ctx->sco_connected = WICED_FALSE;

    WICED_BT_TRACE("bt_hs_spk_handsfree_sco_management_callback_disconnected : bt_audio_end_loop");

    //hci_control_HFP_sco_connection_event(p_data->sco_connected.sco_index, p_ctx->sco_connected, bt_hs_spk_handsfree_speaker_volume_level_get(p_ctx), p_ctx->mic_volume);

    if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        bt_hs_spk_handsfree_audio_manager_stream_stop();

        bt_hs_spk_control_link_key_nvram_update();

#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
        bt_hs_spk_pm_enable();
#endif

        /* Recover the audio streaming if it is interrupted. */
        bt_hs_spk_audio_streaming_recover();
    }
    else
    {
        /* Re-establish the voice connection since we disconnect it in the previous incoming
         * call set up process. */
        if (bt_hs_spk_handsfree_cb.p_active_context->call_held == WICED_TRUE)
        {
            WICED_BT_TRACE("Re-establish the previous voice connection\n");

            /* Push the held call to be active. */
            wiced_bt_hfp_hf_perform_call_action(bt_hs_spk_handsfree_cb.p_active_context->rfcomm_handle,
                                                WICED_BT_HFP_HF_CALL_ACTION_HOLD_2,
                                                NULL);

            /* Recovery the Voice stream. */
            bt_hs_spk_handsfree_audio_connection_establish(bt_hs_spk_handsfree_cb.p_active_context);
        }
        /* Recover the voice stream for the active call. */
        else if (bt_hs_spk_handsfree_cb.p_active_context->call_active)
        {
            /* Establish the voice connection with the active call session. */
            bt_hs_spk_handsfree_audio_connection_establish(bt_hs_spk_handsfree_cb.p_active_context);
        }

    }
    status = wiced_bt_sco_create_as_acceptor_ex(p_ctx->peer_bd_addr, &p_ctx->sco_index);

    print_bd_address(p_ctx->peer_bd_addr);
    WICED_BT_TRACE("%s:status [%d] SCO INDEX [%d] \n",
                   __func__,
                   status,
                   p_ctx->sco_index);

    /* Set ACL(s) to sniff mode if the connection is in idle state.
     * Since we set all the other ACL(s) to active mode due to multi-point
     * sniff interval collision issue, we have to set the link(s) back to sniff
     * mode if possible. */
    if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        bt_hs_spk_control_acl_link_policy_sniff_mode_set(NULL, WICED_TRUE);
        bt_hs_spk_control_bt_power_mode_set(WICED_FALSE, NULL, NULL);
    }
    //#endif
}

/**
 * @brief       Process BTM events for SCO management.
 *
 * @param[in]   event
 *              1. BTM_SCO_CONNECTED_EVT - Event data: #wiced_bt_sco_connected_t
 *              2. BTM_SCO_DISCONNECTED_EVT - Event data: #wiced_bt_sco_disconnected_t
 *              3. BTM_SCO_CONNECTION_REQUEST_EVT - Event data: #wiced_bt_sco_connection_request_t
 *              4. BTM_SCO_CONNECTION_CHANGE_EVT -  Event data: #wiced_bt_sco_connection_change_t
 *
 * @param[in]   p_event_data: refer to the description of parameter, event.
 */
void hf_sco_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    WICED_BT_TRACE("[FLOW]hf_sco_management_callback handsfree_management_callback. Event: 0x%x\n", event);
    handsfree_app_state_t* p_ctx = NULL;
    uint16_t sco_index;

    /* Check parameter. */
    if (p_event_data == NULL)
    {
        WICED_BT_TRACE("[ERROR]hf_sco_management_callback handsfree_management_callback. p_event_data = NULL!!!\n");
        return;
    }

    if ((event < BTM_SCO_CONNECTED_EVT) ||
        (event > BTM_SCO_CONNECTION_CHANGE_EVT))
    {
        WICED_BT_TRACE("[ERROR]hf_sco_management_callback handsfree_management_callback. event out of handling!!!\n");
        return;
    }

    /* Check if the corresponding context exist. */
    sco_index = event == BTM_SCO_CONNECTED_EVT ? p_event_data->sco_connected.sco_index :
                event == BTM_SCO_DISCONNECTED_EVT ? p_event_data->sco_disconnected.sco_index :
                event == BTM_SCO_CONNECTION_REQUEST_EVT ? p_event_data->sco_connection_request.sco_index :
                p_event_data->sco_connection_change.sco_index;

    p_ctx = get_context_ptr_from_sco_index(sco_index);

    if (p_ctx == NULL)
    {
        WICED_BT_TRACE("Invalid sco index %d in hf_sco_management_callback\n", sco_index);
        return;
    }

    if (bt_hs_spk_handsfree_cb.btm_sco_event_handler[event - BTM_SCO_CONNECTED_EVT])
    {
        WICED_BT_TRACE("[FLOW]Function bt_hs_spk_handsfree_sco_management_callback_connection_request\n");
        (*bt_hs_spk_handsfree_cb.btm_sco_event_handler[event - BTM_SCO_CONNECTED_EVT])(p_ctx, p_event_data);
    }
    else
    {
        WICED_BT_TRACE("[ERROR]Not Find CallBack Function!\n");
    }
    WICED_BT_TRACE("[FLOW]End hf_sco_management_callback handsfree_management_callback.\n");
}

/*
 * This function handles the HF event, WICED_BT_HFP_HF_CONNECTION_STATE_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_connection_state(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    wiced_bt_dev_status_t status  = WICED_BT_ERROR;
    wiced_bt_device_address_t reconnect_peer_bdaddr;

    switch (p_data->conn_data.conn_state)
    {
    case WICED_BT_HFP_HF_STATE_DISCONNECTED:
        hci_control_HFP_connection_status_event(p_ctx->peer_bd_addr,p_data->handle, FALSE);
        if(p_ctx->sco_index != WICED_INVALID_SCO_INDEX)
        {
            status = wiced_bt_sco_remove(p_ctx->sco_index);
            WICED_BT_TRACE("%s: remove sco status [%d] \n", __func__, status);
            p_ctx->sco_index = WICED_INVALID_SCO_INDEX;

        }

        p_ctx->sco_connected        = WICED_FALSE;
        p_ctx->connection_status    = p_data->conn_data.conn_state;
        p_ctx->ag_indicator_mask    = 0;

        /* Check if all the HFP connection state are set to DISCONNECTED. */
        if (bt_hs_spk_handsfree_hfp_connection_check(NULL, WICED_FALSE) == WICED_FALSE)
        {
            bt_hs_spk_audio_app_service_set();
        }
        break;

    case WICED_BT_HFP_HF_STATE_CONNECTED:
        p_ctx->rfcomm_handle        = p_data->handle;
        p_ctx->connection_status    = p_data->conn_data.conn_state;

        status = wiced_bt_sco_create_as_acceptor_ex(p_ctx->peer_bd_addr, &p_ctx->sco_index);
        print_bd_address(p_data->conn_data.remote_address);
        WICED_BT_TRACE("%s:status [%d] RFCOMM Handle:%d SCO INDEX [%d] \n", __func__, status, p_ctx->rfcomm_handle , p_ctx->sco_index);

        /* To simply the controller QoS, enforce the IUT be the slave if
         * IUT support multi-point. */
        /*
         * We do it here to make it happen as soon as possible.
         * If we let the Main module doing it once SLC is fully connected it may be
         * too late because the SCO link may have been already established (Role Switch is not
         * allowed if a SCO link is connected)
         */
        if (BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS > 1)
        {
            bt_hs_spk_control_bt_role_set(p_data->conn_data.remote_address, HCI_ROLE_PERIPHERAL);
        }
        break;

    case WICED_BT_HFP_HF_STATE_SLC_CONNECTED:
        WICED_BT_TRACE("HF Connection State: conn_state:%d, call_active:%d, sco (sco_connected:%s, sco_index:%d, use_wbs:%d), %d\n",
                       p_data->conn_data.conn_state,
                       p_ctx->call_active,
                       p_ctx->sco_connected ? "connected" : "disconnected",
                       p_ctx->sco_index,
                       p_ctx->sco_params.use_wbs,
                       bt_hs_spk_control_br_edr_last_disconnection_reason_get(p_data->conn_data.remote_address));

        hci_control_HFP_connection_status_event(p_data->conn_data.remote_address,p_data->handle, TRUE);

        /* Check if there is an active call but the SCO connection is dropped. */
        if ((p_ctx->call_active == WICED_TRUE) &&
            (p_ctx->sco_connected == WICED_FALSE))
        {
            if (bt_hs_spk_control_br_edr_last_disconnection_reason_get(p_data->conn_data.remote_address) == HCI_ERR_CONNECTION_TOUT)
            {
                /* Recovery the Voice stream. */
                bt_hs_spk_handsfree_audio_connection_establish(p_ctx);
            }
        }

        p_ctx->rfcomm_handle        = p_data->handle;
        p_ctx->connection_status    = p_data->conn_data.conn_state;
        break;
    default:
        break;
    }

    /* Check if device is under re-connection state. */
    if (bt_hs_spk_control_reconnect_peer_bdaddr_get(reconnect_peer_bdaddr) == WICED_TRUE)
    {
        if (memcmp((void *) p_ctx->peer_bd_addr,
                   (void *) reconnect_peer_bdaddr,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            bt_hs_spk_control_reconnect();
        }
    }
}

/*
 * This function handles the HF event, WICED_BT_HFP_HF_SERVICE_STATE_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_service_state(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    WICED_BT_TRACE("HF Service State (%d)\n", p_data->service_state);
}

/*
 * This function handles the HF event, WICED_BT_HFP_HF_SERVICE_TYPE_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_service_type(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    WICED_BT_TRACE("HF Service Type (%d)\n", p_data->service_type);
}

/*
 * This function handles the HF event, WICED_BT_HFP_HFP_CODEC_SET_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_codec_set(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    if (bt_hs_spk_handsfree_cb.p_active_context)
    {
        WICED_BT_TRACE("HF Codec Set (%d, %d) (%d)\n",
            bt_hs_spk_handsfree_cb.p_active_context->sco_index,
            p_ctx->sco_index,
            p_data->selected_codec);
    }

    /* Check parameter. */
    if ((p_data->selected_codec != WICED_BT_HFP_HF_CVSD_CODEC) &&
        (p_data->selected_codec != WICED_BT_HFP_HF_MSBC_CODEC))
    {
        return;
    }

    /* Update information. */
    if (p_data->selected_codec == WICED_BT_HFP_HF_MSBC_CODEC)
    {
        p_ctx->sco_params.use_wbs = WICED_TRUE;
        p_ctx->audio_config.sr = AM_PLAYBACK_SR_16K;
    }
    else
    {
        p_ctx->sco_params.use_wbs = WICED_FALSE;
        p_ctx->audio_config.sr = AM_PLAYBACK_SR_8K;
    }

    p_ctx->audio_config.channels =  1;
    p_ctx->audio_config.bits_per_sample = DEFAULT_BITSPSAM;
    p_ctx->audio_config.volume = bt_hs_spk_handsfree_utils_hfp_volume_to_am_volume(bt_hs_spk_handsfree_speaker_volume_level_get(p_ctx));
    p_ctx->audio_config.sink = bt_hs_spk_get_audio_sink();

    /* Set active context. */
    if ((bt_hs_spk_handsfree_cb.p_active_context == NULL) ||
        (bt_hs_spk_handsfree_cb.p_active_context == p_ctx))
    {
        bt_hs_spk_handsfree_active_call_session_set(p_ctx);
    }
}

static void bt_hs_spk_handsfree_event_handler_call_setup_idle(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    uint16_t i;

    if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        if ((p_data->call_data.active_call_present == WICED_FALSE) &&
            (p_data->call_data.held_call_present == WICED_FALSE))
        {
            return;
        }

        bt_hs_spk_handsfree_active_call_session_set(p_ctx);

        app_set_current_service(&bt_hs_spk_handsfree_cb.app_service);

        return;
    }

    if (bt_hs_spk_handsfree_cb.p_active_context == p_ctx)
    {
        if ((p_data->call_data.active_call_present == WICED_FALSE) &&
            (p_data->call_data.held_call_present == WICED_FALSE))
        {
            /* Check if there is another call session. */
            for (i = 0 ; i < WICED_BT_HFP_HF_MAX_CONN ; i++)
            {
                if (bt_hs_spk_handsfree_cb.context[i].connection_status >= WICED_BT_HFP_HF_STATE_CONNECTED)
                {
                    if (bt_hs_spk_handsfree_cb.context[i].sco_index != p_ctx->sco_index)
                    {
                        if ((bt_hs_spk_handsfree_cb.context[i].sco_connected == WICED_TRUE) ||
                            (bt_hs_spk_handsfree_cb.context[i].call_active == WICED_TRUE) ||
                            (bt_hs_spk_handsfree_cb.context[i].call_held == WICED_TRUE) ||
                            (bt_hs_spk_handsfree_cb.context[i].call_setup >= WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING))
                        {
                            bt_hs_spk_handsfree_active_call_session_set(&bt_hs_spk_handsfree_cb.context[i]);

                            break;
                        }
                    }
                }
            }

            if (i >= WICED_BT_HFP_HF_MAX_CONN)
            {
                bt_hs_spk_handsfree_active_call_session_set(NULL);

                /* Set current service back to A2DP. */
                bt_hs_spk_audio_app_service_set();
            }
        }
    }
}

/*
 * This function handles the HF event, WICED_BT_HFP_HF_CALL_SETUP_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_call_setup(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    wiced_bool_t speaker_volume_switch = WICED_FALSE;
    uint8_t hfp_volume_level;
    int32_t am_volume_level;
    wiced_bool_t allowed = WICED_FALSE;

    WICED_BT_TRACE("HF Call Setup (sco_index: %d) (active_call: %d, held_call: %d, setup_state: %d, 0x%08X 0x%08X)\n",
                   p_ctx->sco_index,
                   p_data->call_data.active_call_present,
                   p_data->call_data.held_call_present,
                   p_data->call_data.setup_state,
                   bt_hs_spk_handsfree_cb.p_active_context,
                   p_ctx);

    hci_control_HFP_callsetup_status_event(p_ctx->sco_index ,p_data->call_data.active_call_present,p_data->call_data.held_call_present, p_data->call_data.setup_state);

    switch (p_data->call_data.setup_state)
    {
        case WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING:
            WICED_BT_TRACE("Call(incoming) setting-up");
            allowed = bt_hs_spk_handsfree_incoming_call_notification_handler(p_ctx);
            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE:
            bt_hs_spk_handsfree_event_handler_call_setup_idle(p_ctx, p_data);
            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING:
        case WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING:
            allowed = bt_hs_spk_handsfree_outgoing_call_notification_handler(p_ctx);
            break;

        default:
            return;
    }

    if (allowed)
    {
        if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
        {
            app_set_current_service(&bt_hs_spk_handsfree_cb.app_service);
        }

        bt_hs_spk_handsfree_active_call_session_set(p_ctx);
    }

    /* Check if the speaker volume (ring tone <-> call) shall be switched. */
    if (p_ctx->call_active == WICED_FALSE)
    {
        if (p_data->call_data.active_call_present == WICED_TRUE)
        {  
            TRACE_LOG("call is answered!!!!!");
            start_alsa_capture();
            sco_flag = true;
            speaker_volume_switch = WICED_TRUE;
        }
    }
    else
    {
        if (p_data->call_data.active_call_present == WICED_FALSE)
        {   // call is terminated
            TRACE_LOG("call is Terminated!!!!!");
            sco_flag = false;
            stop_alsa_capture();
            speaker_volume_switch = WICED_TRUE;
        }
    }

    /* Update state. */
    p_ctx->call_active = p_data->call_data.active_call_present;
    p_ctx->call_setup  = p_data->call_data.setup_state;
    p_ctx->call_held   = p_data->call_data.held_call_present;

    /* Update the speaker volume if required. */
    if (speaker_volume_switch == WICED_TRUE)
    {
        /* Update the speaker volume with the AG. */
        /* Although HFP v1.7.1 Section 4.28.2 asks the HF to inform the AG of it current
         * gain setting on Service Level Connection Establishment. the PTS test cases
         * HFP/HF/ATH/BV-05-I,HFP/HF/ACS/BV-15-I,HFP/HF/OOR/BV-01-I only allows the HF
         * to update its current gain after establishment of the Service Level Connection. */
        if (p_ctx->connection_status != WICED_BT_HFP_HF_STATE_SLC_CONNECTED)
        {
            return;
        }
        hfp_volume_level = bt_hs_spk_handsfree_speaker_volume_level_get(p_ctx);

        wiced_bt_hfp_hf_notify_volume(p_ctx->rfcomm_handle,
                                      WICED_BT_HFP_HF_SPEAKER,
                                      hfp_volume_level);

        /* Update external codec's volume. */
        if ((p_ctx == bt_hs_spk_handsfree_cb.p_active_context) ||
            (p_ctx->sco_connected == WICED_TRUE))
        {
            /* Changing from 0 to 15 volume level to 0 to 10 volume level for Audio Manager */
            am_volume_level = bt_hs_spk_handsfree_utils_hfp_volume_to_am_volume(hfp_volume_level);

            bt_hs_spk_handsfree_audio_manager_stream_volume_set(am_volume_level);
        }
    }

    /* Set ACL to sniff mode if the connection is in idle state. */
    if (p_data->call_data.setup_state == WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE)
    {
        bt_hs_spk_control_bt_power_mode_set(WICED_FALSE, p_ctx->peer_bd_addr, NULL);
    }

    /* Reset call hanging up mutex. */
    if (p_ctx->call_hanging_up)
    {
        if ((p_ctx->call_active == WICED_FALSE) &&
            (p_ctx->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE))
        {
            p_ctx->call_hanging_up = WICED_FALSE;
        }
    }
}

/*
 * This function handles the HF event, WICED_BT_HFP_HF_RING_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_ring(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    WICED_BT_TRACE("HF RING \n");
    app_set_current_service(&bt_hs_spk_handsfree_cb.app_service);
}

/*
 * This function handles the HF event, WICED_BT_HFP_HF_INBAND_RING_STATE_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_inband_ring_state(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    WICED_BT_TRACE("HF INBAND RING STATE (%d)\n", p_data->inband_ring);

    p_ctx->inband_ring_status = p_data->inband_ring;

}

/*
 * This function handles the HF event, WICED_BT_HFP_HF_RSSI_IND_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_rssi_ind(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    WICED_BT_TRACE("HF RSSI Ind (%d)\n", p_data->rssi);
}

/*
 * This function handles the HF event, WICED_BT_HFP_HF_BATTERY_STATUS_IND_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_battery_status_ind(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    WICED_BT_TRACE("HF Battery Status Ind (%d)\n", p_data->battery_level);
}

 /* This function handles the HF event, WICED_BT_HFP_HF_COPS_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_cops(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    WICED_BT_TRACE("Rx : Network Name  \n");
}

   /* This function handles the HF event, WICED_BT_HFP_HF_CLCC_EVT.
   */
  static void bt_hs_spk_handsfree_event_handler_clcc(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
  {
      WICED_BT_TRACE("Rx : Current call id = %d \n",p_data->active_call.idx);

      p_data->active_call.dir ?WICED_BT_TRACE("dir :INCOMING CALL"):WICED_BT_TRACE("dir :OUTGOING CALL");
      p_data->active_call.mode ?WICED_BT_TRACE("mode :OTHER"):WICED_BT_TRACE("mode :VOICE");
      WICED_BT_TRACE("Number : %s",p_data->active_call.num);
  }


  /* This function handles the HF event, WICED_BT_HFP_HF_CNUM_EVT.
  */
 static void bt_hs_spk_handsfree_event_handler_cnum(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
 {
     WICED_BT_TRACE("Rx :  Subscriber Number Information  \n");
 }

   /* This function handles the HF event, WICED_BT_HFP_HF_VOICE_RECOGNITION_EVT.
  */
 static void bt_hs_spk_handsfree_event_handler_voice_recognition(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
 {
     WICED_BT_TRACE("Voice Recognition Event =  %s \n", p_data->voice_recognition? "ACTIVATED" : "DEACTIVATED");
     if (p_data->voice_recognition)
         vrec_enable = TRUE;
     else
         vrec_enable = FALSE;
     hci_control_HFP_voice_recognition_status_event(p_ctx->sco_index, p_data->voice_recognition);
 }

 /* This function handles the HF event, WICED_BT_HFP_HF_BINP_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_binp(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    WICED_BT_TRACE("Rx : Phone Number  \n");
}

/*
 * This function handles the HF event, WICED_BT_HFP_HF_BIND_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_bind(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    WICED_BT_TRACE("BT Indicator (%d, %d)\n",
                   p_data->bind_data.ind_id,
                   p_data->bind_data.ind_value);

    switch(p_data->bind_data.ind_id)
    {
    case WICED_BT_HFP_HF_IND_ID_ENHANCED_SAFETY:
        /* Save the Enabled/Disabled value of this Indicator */
        if (p_data->bind_data.ind_value)
        {
            p_ctx->ag_indicator_mask |= 1 << (WICED_BT_HFP_HF_IND_ID_ENHANCED_SAFETY);
        }
        else
        {
            p_ctx->ag_indicator_mask &= ~(1 << (WICED_BT_HFP_HF_IND_ID_ENHANCED_SAFETY));
        }
        break;
    case WICED_BT_HFP_HF_IND_ID_BATTERY:
        /* Save the Enabled/Disabled value of this Indicator */
        if (p_data->bind_data.ind_value)
        {
            p_ctx->ag_indicator_mask |= 1 << (WICED_BT_HFP_HF_IND_ID_BATTERY);
        }
        else
        {
            p_ctx->ag_indicator_mask &= ~(1 << (WICED_BT_HFP_HF_IND_ID_BATTERY));
        }
        break;
    default:
        break;
    }
}

/*
 * This function handles the HF event, WICED_BT_HFP_HF_VOLUME_CHANGE_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_volume_change(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    int32_t volume_level;

    WICED_BT_TRACE("HF Volume Change (0x%08X 0x%08X, %d, %d (%d percent))\n",
                   bt_hs_spk_handsfree_cb.p_active_context,
                   p_ctx,
                   p_data->volume.type,
                   p_data->volume.level,
                   p_data->volume.level * 100 / WICED_HANDSFREE_VOLUME_MAX);

    hci_control_HFP_volume_change_event(p_ctx->peer_bd_addr,p_data->volume.type,p_data->volume.level,(uint8_t)(p_data->volume.level * 100 / WICED_HANDSFREE_VOLUME_MAX));

    /* Check parameter. */
    if ((p_data->volume.type != WICED_BT_HFP_HF_SPEAKER) &&
        (p_data->volume.type != WICED_BT_HFP_HF_MIC))
    {
        return;
    }

    if (p_data->volume.level > WICED_HANDSFREE_VOLUME_MAX)
    {
        return;
    }

    /* Process this event. */
    if (p_data->volume.type == WICED_BT_HFP_HF_MIC)
    {   // MIC
        /* Set the external codec only when the context is the active context. */
        if (bt_hs_spk_handsfree_cb.p_active_context == p_ctx)
        {
            /* Changing from 0 to 15 volume level to 0 to 10 volume level for Audio Manager */
            volume_level = bt_hs_spk_handsfree_utils_hfp_volume_to_am_volume(p_data->volume.level);
            if( WICED_SUCCESS != wiced_am_stream_set_param(bt_hs_spk_handsfree_cb.stream_id, AM_MIC_GAIN_LEVEL, &volume_level))
            {
                WICED_BT_TRACE("wiced_am_set_param failed\n");
                return;
            }
        }

        /* Update volume. */
        p_ctx->mic_volume = p_data->volume.level;
    }
    else
    {   // SPEAKER
        /* Set the external codec only when the context is the active context. */
        if (bt_hs_spk_handsfree_cb.p_active_context == p_ctx)
        {
            /* Changing from 0 to 15 volume level to 0 to 10 volume level for Audio Manager */
            volume_level = bt_hs_spk_handsfree_utils_hfp_volume_to_am_volume(p_data->volume.level);

            bt_hs_spk_handsfree_audio_manager_stream_volume_set(volume_level);
        }

        /* Update volume. */
        bt_hs_spk_handsfree_speaker_volume_level_set(p_ctx, p_data->volume.level);
    }
}

/*
 * This function handles the HF event, WICED_BT_HFP_HF_CLIP_IND_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_clip_ind(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    WICED_BT_TRACE("HF CLIP Ind (%d, %s)\n", p_data->clip.type, p_data->clip.caller_num);
    hci_control_HFP_call_alert_event(p_ctx->peer_bd_addr, p_data->clip.type,p_data->clip.caller_num);
}

/*
 * This function handles the HF event, WICED_BT_HFP_HF_AG_FEATURE_SUPPORT_EVT.
 */
static void bt_hs_spk_handsfree_event_handler_ag_feature_support(handsfree_app_state_t *p_ctx, wiced_bt_hfp_hf_event_data_t* p_data)
{
    wiced_bt_hfp_hf_scb_t    *p_scb = NULL;

    WICED_BT_TRACE("HF AG Feature (0x%08X)\n", p_data->ag_feature_flags);

    if (p_data->ag_feature_flags & WICED_BT_HFP_AG_FEATURE_INBAND_RING_TONE_CAPABILITY)
    {
        p_ctx->inband_ring_status = WICED_BT_HFP_HF_INBAND_RING_ENABLED;
    }
    else
    {
        p_ctx->inband_ring_status = WICED_BT_HFP_HF_INBAND_RING_DISABLED;
    }

#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
    p_scb = wiced_bt_hfp_hf_get_scb_by_handle(p_data->handle);

    if (p_scb == NULL)
    {
        return;
    }

    if ((p_data->ag_feature_flags & WICED_BT_HFP_AG_FEATURE_CODEC_NEGOTIATION) &&
        (p_scb->feature_mask & WICED_BT_HFP_AG_FEATURE_CODEC_NEGOTIATION))
    {
        p_ctx->sco_params.use_wbs = WICED_TRUE;
    }
    else
    {
        p_ctx->sco_params.use_wbs = WICED_FALSE;
    }
#endif

    /* Save information. */
    p_ctx->ag_features = p_data->ag_feature_flags;
}



/*
 * Control callback supplied by the handsfree profile code.
 */
static void handsfree_event_callback( wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data)
{
    handsfree_app_state_t *p_ctx = NULL;

    WICED_BT_TRACE("handsfree_event_callback (handle: %d event: %d)\n",
                   p_data ? p_data->handle: 0xffff,
                   event);

    /* Check paramter. */
    if ((event > WICED_BT_HFP_HF_BIND_EVT) ||
        (p_data == NULL))
    {
        return;
    }

    if (bt_hs_spk_handsfree_cb.config.p_pre_handler)
    {
        if ((*bt_hs_spk_handsfree_cb.config.p_pre_handler)(event, p_data) == WICED_FALSE)
        {
            return;
        }
    }

    /* Check if the corresponding context exist. */
    if (event == WICED_BT_HFP_HF_CONNECTION_STATE_EVT)
    {
        p_ctx = get_context_ptr_from_address(p_data->conn_data.remote_address, WICED_TRUE);
    }
    else
    {
        p_ctx = get_context_ptr_from_handle(p_data->handle);
    }

    if (p_ctx == NULL)
    {
        WICED_BT_TRACE("%s: Can't get HFP context pointer\n", __func__);
        return;
    }

    if (bt_hs_spk_handsfree_cb.handsfree_event_handler[event])
    {
        (*bt_hs_spk_handsfree_cb.handsfree_event_handler[event])(p_ctx, p_data);
    }

    if (bt_hs_spk_handsfree_cb.config.post_handler)
    {
        bt_hs_spk_handsfree_cb.config.post_handler(event, p_data);
    }

}

static wiced_result_t bt_audio_hfp_reject_call_active_yes(handsfree_app_state_t *p_ctx)
{
    switch (p_ctx->call_setup)
    {
    case WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE:     /* No call set up in progress */
        /* Check if there is any held call. */
        if (p_ctx->call_held)
        {   // There is a held call.
            /* Release the active call and active the held call. */
            return wiced_bt_hfp_hf_perform_call_action(p_ctx->rfcomm_handle,
                                                       WICED_BT_HFP_HF_CALL_ACTION_HOLD_1,
                                                       NULL);
        }
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING: /* There is an incoming call */
        /* Reject the incoming call. */
        return wiced_bt_hfp_hf_perform_call_action(p_ctx->rfcomm_handle,
                                                   WICED_BT_HFP_HF_CALL_ACTION_HOLD_0,
                                                   NULL);

    case WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING:  /* Outgoing call is being setup up */
    case WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING: /* Remote party is being alterted of the call */
        /* Terminate/Cancel the outgoing call. */
        bt_hs_spk_handsfree_call_hang_up(p_ctx);
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_WAITING:  /* Incoming call is waiting (received when a call is already active) */
    default:
        break;
    }

    return WICED_BT_SUCCESS;
}

static wiced_result_t bt_audio_hfp_reject_call_active_no(handsfree_app_state_t *p_ctx)
{
    switch (p_ctx->call_setup)
    {
    case WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE:     /* No call set up in progress */
        /* Check if there is any held call. */
        if (p_ctx->call_held)
        {   // There is a held call.
            /* Release the held call. */
            return wiced_bt_hfp_hf_perform_call_action(p_ctx->rfcomm_handle,
                                                       WICED_BT_HFP_HF_CALL_ACTION_HOLD_0,
                                                       NULL);
        }
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING: /* There is an incoming call */
        /* Reject the incoming call. */
        bt_hs_spk_handsfree_call_hang_up(p_ctx);
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING:  /* Outgoing call is being setup up */
    case WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING: /* Remote party is being alterted of the call */
        /* Terminate/Cancel the outgoing call. */
        bt_hs_spk_handsfree_call_hang_up(p_ctx);
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_WAITING:  /* Incoming call is waiting (received when a call is already active) */
    default:
        break;
    }

    return WICED_BT_SUCCESS;
}

static wiced_result_t bt_audio_hfp_reject(void)
{
    if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        return WICED_ERROR;
    }

    /* Process this request according to current active call state. */
    if (bt_hs_spk_handsfree_cb.p_active_context->call_active)
    {
        return bt_audio_hfp_reject_call_active_yes(bt_hs_spk_handsfree_cb.p_active_context);
    }
    else
    {
        return bt_audio_hfp_reject_call_active_no(bt_hs_spk_handsfree_cb.p_active_context);
    }
}

static wiced_result_t bt_audio_hfp_accept_hangup_call_active_yes(handsfree_app_state_t *p_ctx)
{
    switch (p_ctx->call_setup)
    {
    case WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE:     /* No call set up in progress */
        /* Check if there is any held call. */
        if (p_ctx->call_held)
        {   // There is a held call.
            /* Switch the active call. */
            return wiced_bt_hfp_hf_perform_call_action(p_ctx->rfcomm_handle,
                                                       WICED_BT_HFP_HF_CALL_ACTION_HOLD_2,
                                                       NULL);
        }
        else
        {   // There is no held call.
            if (p_ctx->sco_connected == WICED_TRUE)
            {   // A SCO/eSCO connection exists.
                /* Hang up the call. */
                bt_hs_spk_handsfree_call_hang_up(p_ctx);
            }
            else
            {   // There is no existent SCO/eSCO connection.
                /* For some device, like the PTS test tool HFP ACC test cases, the AG will
                 * not trigger the HFP Codec Connection process. We shall add a user interface
                 * to start the HFP Codec Connection process. Refer to HFP test cases,
                 * HFP/HF/ACC/BV-01-I, HFP/HF/ACC/BV-02-I, HFP/HF/ACC/BV-03-I,
                 * HFP/HF/ACC/BV-04-I, HFP/HF/ACC/BV-05-I, HFP/HF/ACC/BV-06-I, and
                 * HFP/HF/ACC/BV-07-I. */
                bt_hs_spk_handsfree_audio_connection_establish(p_ctx);
            }
        }
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING: /* There is an incoming call */
        /* Switch the active call. */
        return wiced_bt_hfp_hf_perform_call_action(p_ctx->rfcomm_handle,
                                                   WICED_BT_HFP_HF_CALL_ACTION_HOLD_2,
                                                   NULL);

    case WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING:  /* Outgoing call is being setup up */
    case WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING: /* Remote party is being alterted of the call */
        /* Do nothing since an outgoing call is waiting now. */
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_WAITING:  /* Incoming call is waiting (received when a call is already active) */
    default:
        break;
    }

    return WICED_BT_SUCCESS;
}

static wiced_result_t bt_audio_hfp_accept_hangup_call_active_no(handsfree_app_state_t *p_ctx)
{
    switch (p_ctx->call_setup)
    {
    case WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE:     /* No call set up in progress */
        /* Check if there is any held call. */
        if (p_ctx->call_held)
        {   // There is a held call.
            /* Active the held call. */
            return wiced_bt_hfp_hf_perform_call_action(p_ctx->rfcomm_handle,
                                                       WICED_BT_HFP_HF_CALL_ACTION_HOLD_2,
                                                       NULL);
        }
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING: /* There is an incoming call */
        /* Hang on this incoming call. */
        return wiced_bt_hfp_hf_perform_call_action(p_ctx->rfcomm_handle,
                                                   WICED_BT_HFP_HF_CALL_ACTION_ANSWER,
                                                   NULL);

    case WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING:  /* Outgoing call is being setup up */
    case WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING: /* Remote party is being alterted of the call */
        /* Do nothing since an outgoing call is waiting now. */
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_WAITING:  /* Incoming call is waiting (received when a call is already active) */
    default:
        break;
    }

    return WICED_BT_SUCCESS;
}

static wiced_result_t bt_audio_hfp_accept_hangup(void)
{
    if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        return WICED_ERROR;
    }

    /* Process this request according to current active call state. */
    if (bt_hs_spk_handsfree_cb.p_active_context->call_active)
    {
        return bt_audio_hfp_accept_hangup_call_active_yes(bt_hs_spk_handsfree_cb.p_active_context);
    }
    else
    {
        return bt_audio_hfp_accept_hangup_call_active_no(bt_hs_spk_handsfree_cb.p_active_context);
    }
}

static wiced_result_t bt_audio_hfp_volume_up(void)
{
    int32_t volume_level;
    uint8_t hfp_volume_level;

    if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        return WICED_ERROR;
    }

    hfp_volume_level = bt_hs_spk_handsfree_speaker_volume_level_get(bt_hs_spk_handsfree_cb.p_active_context);

    hfp_volume_level++;

    if (hfp_volume_level > WICED_HANDSFREE_VOLUME_MAX)
    {
        hfp_volume_level = WICED_HANDSFREE_VOLUME_MAX;
    }

    bt_hs_spk_handsfree_speaker_volume_level_set(bt_hs_spk_handsfree_cb.p_active_context,
                                                 hfp_volume_level);

    wiced_bt_hfp_hf_notify_volume(bt_hs_spk_handsfree_cb.p_active_context->rfcomm_handle,
                                  WICED_BT_HFP_HF_SPEAKER,
                                  hfp_volume_level);

    /* Changing from 0 to 15 volume level to 0 to 10 volume level for Audio Manager */
    volume_level = bt_hs_spk_handsfree_utils_hfp_volume_to_am_volume(hfp_volume_level);

    WICED_BT_TRACE("bt_audio_hfp_volume_up - abs:%d (%d percent), am:%d\n",
                   hfp_volume_level,
                   hfp_volume_level * 100 /  WICED_HANDSFREE_VOLUME_MAX,
                   volume_level);
    hci_control_HFP_volume_up_down_event(bt_hs_spk_handsfree_cb.p_active_context->peer_bd_addr,hfp_volume_level,(uint8_t)(hfp_volume_level * 100 /  WICED_HANDSFREE_VOLUME_MAX),volume_level, 1);

    bt_hs_spk_handsfree_audio_manager_stream_volume_set(volume_level);

    return WICED_SUCCESS;
}

static wiced_result_t bt_audio_hfp_volume_down(void)
{
    int32_t volume_level;
    uint8_t hfp_volume_level;

    if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        return WICED_ERROR;
    }

    hfp_volume_level = bt_hs_spk_handsfree_speaker_volume_level_get(bt_hs_spk_handsfree_cb.p_active_context);

    if (hfp_volume_level != WICED_HANDSFREE_VOLUME_MIN)
    {
        hfp_volume_level--;
    }

    bt_hs_spk_handsfree_speaker_volume_level_set(bt_hs_spk_handsfree_cb.p_active_context,
                                                 hfp_volume_level);

    wiced_bt_hfp_hf_notify_volume(bt_hs_spk_handsfree_cb.p_active_context->rfcomm_handle,
                                  WICED_BT_HFP_HF_SPEAKER,
                                  hfp_volume_level);

    /* Changing from 0 to 15 volume level to 0 to 10 volume level for Audio Manager */
    volume_level = bt_hs_spk_handsfree_utils_hfp_volume_to_am_volume(hfp_volume_level);

    WICED_BT_TRACE("bt_audio_hfp_volume_down - abs:%d (%d percent), am:%d\n",
                   hfp_volume_level,
                   hfp_volume_level * 100 /  WICED_HANDSFREE_VOLUME_MAX,
                   volume_level);

    hci_control_HFP_volume_up_down_event(bt_hs_spk_handsfree_cb.p_active_context->peer_bd_addr,hfp_volume_level,(uint8_t)(hfp_volume_level * 100 /  WICED_HANDSFREE_VOLUME_MAX),volume_level, 0);

    bt_hs_spk_handsfree_audio_manager_stream_volume_set(volume_level);

    return WICED_SUCCESS;
}

/*
 * bt_hs_spk_handsfree_voice_recognition_activate
 *
 * Activate the voice recognition function resident in the AG.
 *
 * Refer to Section 4.25 of Hands-Free Profile 1.7.1
 */
wiced_result_t bt_hs_spk_handsfree_voice_recognition_activate(void)
{
    uint16_t idx;

    /* Note here that we don't have to stop the existent audio streaming here,
     * the audio streaming will be interrupted when the AG tries to establish
     * the audio connection with the headset after the AG accepts the voice
     * recognition request. */

    if (bt_hs_spk_handsfree_cb.p_active_context)
    {   /* Active call session exists. */
        return bt_hs_spk_handsfree_at_cmd_send(bt_hs_spk_handsfree_cb.p_active_context->rfcomm_handle,
                                               "+BVRA",
                                               WICED_BT_HFP_HF_AT_SET,
                                               WICED_BT_HFP_HF_AT_FMT_INT,
                                               NULL,
                                               VOICE_RECOGNITION_ENABLE);
    }
    else
    {
        /* Find the first connected AG with SLC connection and activate its voice recognition
         * function. */
        for (idx = 0 ; idx < WICED_BT_HFP_HF_MAX_CONN; idx++)
        {
            if (bt_hs_spk_handsfree_cb.context[idx].connection_status >= WICED_BT_HFP_HF_STATE_SLC_CONNECTED)
            {
                return bt_hs_spk_handsfree_at_cmd_send(bt_hs_spk_handsfree_cb.context[idx].rfcomm_handle,
                                                       "+BVRA",
                                                       WICED_BT_HFP_HF_AT_SET,
                                                       WICED_BT_HFP_HF_AT_FMT_INT,
                                                       NULL,
                                                       VOICE_RECOGNITION_ENABLE);
            }
        }

    }

    return WICED_ERROR;
}



wiced_result_t bt_hs_spk_handsfree_voice_recognition_deactivate(void)
{
    uint16_t idx;

    if (bt_hs_spk_handsfree_cb.p_active_context)
    {
        /* Check connection state. */
        if (bt_hs_spk_handsfree_cb.p_active_context->connection_status >= WICED_BT_HFP_HF_STATE_SLC_CONNECTED)
        {

            /* Active call session exists. */
            return bt_hs_spk_handsfree_at_cmd_send(bt_hs_spk_handsfree_cb.p_active_context->rfcomm_handle,
                                                   "+BVRA",
                                                   WICED_BT_HFP_HF_AT_SET,
                                                   WICED_BT_HFP_HF_AT_FMT_INT,
                                                   NULL,
                                                   VOICE_RECOGNITION_DISABLE);
       }
    }
    else
    {
        /* Find the first connected AG with SLC connection and activate its voice recognition
         * function. */
        for (idx = 0 ; idx < WICED_BT_HFP_HF_MAX_CONN; idx++)
        {
            if (bt_hs_spk_handsfree_cb.context[idx].connection_status >= WICED_BT_HFP_HF_STATE_SLC_CONNECTED)
            {
                return bt_hs_spk_handsfree_at_cmd_send(bt_hs_spk_handsfree_cb.context[idx].rfcomm_handle,
                                                       "+BVRA",
                                                       WICED_BT_HFP_HF_AT_SET,
                                                       WICED_BT_HFP_HF_AT_FMT_INT,
                                                       NULL,
                                                       VOICE_RECOGNITION_DISABLE);

            }
        }

    }

    return WICED_ERROR;
}

wiced_result_t bt_hs_spk_handsfree_enhanced_voice_rec_cmd(void)
{

    if (bt_hs_spk_handsfree_cb.p_active_context)
    {   /* Active call session exists. */
        return bt_hs_spk_handsfree_at_cmd_send(bt_hs_spk_handsfree_cb.p_active_context->rfcomm_handle,
                                               "+BVRA",
                                               WICED_BT_HFP_HF_AT_SET,
                                               WICED_BT_HFP_HF_AT_FMT_INT,
                                               NULL,
                                               ENHANCED_VOICE_RECOGNITION_ACTIVATE);
    }
     return WICED_ERROR;
}

static wiced_result_t bt_audio_hfp_held_active_call(handsfree_app_state_t *p_ctx)
{
    switch (p_ctx->call_setup)
    {
    case WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE:     /* No call set up in progress */
        /* Check if there is any held call. */
        if(p_ctx->call_active || p_ctx->call_held)
        {
            /* held the active call or active the held call. */
            return wiced_bt_hfp_hf_perform_call_action(p_ctx->rfcomm_handle,
                                                       WICED_BT_HFP_HF_CALL_ACTION_HOLD_2,
                                                       NULL);
        }
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING: /* There is an incoming call */
        /* Check if there is any held call. */
        if(p_ctx->call_active || p_ctx->call_held)
        {
            /* held the active call or active the held call. */
            return wiced_bt_hfp_hf_perform_call_action(p_ctx->rfcomm_handle,
                                                       WICED_BT_HFP_HF_CALL_ACTION_HOLD_2,
                                                       NULL);
        }
        break;


    case WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING:  /* Outgoing call is being setup up */
    case WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING: /* Remote party is being alterted of the call */
        /* Terminate/Cancel the outgoing call. */
        bt_hs_spk_handsfree_call_hang_up(p_ctx);
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_WAITING:  /* Incoming call is waiting (received when a call is already active) */
    default:
        break;
    }

    return WICED_BT_SUCCESS;
}

static wiced_result_t bt_audio_hfp_call_held(void)
{
    if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        return WICED_ERROR;
    }

    /* Process this request according to current active call state. */
    return bt_audio_hfp_held_active_call(bt_hs_spk_handsfree_cb.p_active_context);

}

static wiced_result_t bt_audio_hfp_send_cmd_enhanced_vrec(void)
{
    if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        return WICED_ERROR;
    }

    /* Process this request according to current active call state. */
    return bt_audio_hfp_held_active_call(bt_hs_spk_handsfree_cb.p_active_context);

}



/* This function can be called when one call is active and and want to release the active call and
    switch to held call*/
static wiced_result_t bt_audio_hfp_switch_active_held_call(void)
{
    if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        return WICED_ERROR;
    }
    /* Check if there is any held call. */
    if(bt_hs_spk_handsfree_cb.p_active_context ->call_active || bt_hs_spk_handsfree_cb.p_active_context ->call_held)
    {
        /* release the active call and active the held call. */
        return wiced_bt_hfp_hf_perform_call_action(bt_hs_spk_handsfree_cb.p_active_context ->rfcomm_handle,
                                                   WICED_BT_HFP_HF_CALL_ACTION_HOLD_1,
                                                   NULL);
    }
    return WICED_BT_SUCCESS;

}



static wiced_result_t bt_audio_hfp_outgoing_call(handsfree_app_state_t *p_ctx)
{
    switch (p_ctx->call_setup)
    {
    case WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE:     /* No call set up in progress */
        {
            /* held the active call or active the held call. */
            return wiced_bt_hfp_hf_perform_call_action(p_ctx->rfcomm_handle,
                                                       WICED_BT_HFP_HF_CALL_ACTION_DIAL,
                                                       NULL);
        }
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING: /* There is an incoming call */
        /* Reject the incoming call. */
        return wiced_bt_hfp_hf_perform_call_action(p_ctx->rfcomm_handle,
                                                   WICED_BT_HFP_HF_CALL_ACTION_HOLD_0,
                                                   NULL);

    case WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING:  /* Outgoing call is being setup up */
    case WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING: /* Remote party is being alterted of the call */
        /* Terminate/Cancel the outgoing call. */
        bt_hs_spk_handsfree_call_hang_up(p_ctx);
        break;

    case WICED_BT_HFP_HF_CALLSETUP_STATE_WAITING:  /* Incoming call is waiting (received when a call is already active) */
    default:
        break;
    }

    return WICED_BT_SUCCESS;
}


 wiced_result_t bt_audio_hfp_dial_last_call(void)
{
     /*since there will not be any active call session , get the first context to start  a outgoing call
     * TODO : context can be searched by bd_addr, passed by application
      */
     bt_hs_spk_handsfree_active_call_session_set(&bt_hs_spk_handsfree_cb.context[0]);
    /* Process this request according to current active call state. */
    return bt_audio_hfp_outgoing_call(bt_hs_spk_handsfree_cb.p_active_context);

}

/*Button event handler for the HFP*/
static wiced_result_t bt_audio_hfp_button_event_handler(app_service_action_t action)
{
    wiced_result_t ret = WICED_ERROR;

    WICED_BT_TRACE("bt_audio_hfp_button_event_handler (action: 0x%02X)\n", action);

    switch( action )
    {
        case ACTION_VOLUME_UP:
            ret = bt_audio_hfp_volume_up();
            break;

        case ACTION_VOLUME_DOWN:
          ret = bt_audio_hfp_volume_down();
            break;

        case ACTION_PAUSE_PLAY:
            ret = bt_audio_hfp_accept_hangup();
            break;

        case ACTION_BT_CALL_REJECT:
            ret = bt_audio_hfp_reject();
            break;

        case ACTION_FORWARD: /* for call held and release */
            ret = bt_audio_hfp_call_held();
            break;
        case ACTION_MULTI_FUNCTION_LONG_RELEASE:
            ret = bt_audio_hfp_send_cmd_enhanced_vrec();
            break;
       default:
            WICED_BT_TRACE("%s -- No Action\n",__func__);
           break;
    }

    return ret;
}

wiced_bool_t bt_hs_spk_handsfree_target_connection_status_check(wiced_bt_device_address_t *p_bd_addr,
        wiced_bool_t *p_connected)
{
    uint16_t idx;

    for (idx = 0 ; idx < WICED_BT_HFP_HF_MAX_CONN; idx++)
    {
        if (memcmp((void *) &bt_hs_spk_handsfree_cb.context[idx].peer_bd_addr,
                   (void *) p_bd_addr,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            if (bt_hs_spk_handsfree_cb.context[idx].connection_status >= WICED_BT_HFP_HF_STATE_CONNECTED)
            {
                *p_connected = WICED_TRUE;
            }
            else
            {
                *p_connected = WICED_FALSE;
            }

            return WICED_TRUE;
        }
    }

    return WICED_FALSE;
}

/**
 * bt_hs_spk_handsfree_sco_connection_check
 *
 * Check if the sco connection for any/specific AG is established (voice call session exists).
 *
 * @param[in]   bdaddr - specific AG's BT address.
 *                       If this is set to NULL, the return value would be set to TURE if any
 *                       sco connection has been established
 *
 * @return WICED_TRUE - (at least one) sco connection is established
 *         WICED_FALSE - there is no established sco connection
 */
wiced_bool_t bt_hs_spk_handsfree_sco_connection_check(wiced_bt_device_address_t bdaddr)
{
    uint16_t idx;
    wiced_bool_t match = WICED_FALSE;

    for (idx = 0 ; idx < WICED_BT_HFP_HF_MAX_CONN; idx++)
    {
        match = WICED_FALSE;

        if (bdaddr != NULL)
        {
            if (memcmp((void *) bt_hs_spk_handsfree_cb.context[idx].peer_bd_addr,
                       (void *) bdaddr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {
                match = WICED_TRUE;
            }
        }

        if ((bdaddr == NULL) ||
            (match == WICED_TRUE))
        {
            if ((bt_hs_spk_handsfree_cb.context[idx].connection_status >= WICED_BT_HFP_HF_STATE_CONNECTED) &&
                (bt_hs_spk_handsfree_cb.context[idx].sco_connected == WICED_TRUE))
            {
                return WICED_TRUE;
            }
        }
    }

    return WICED_FALSE;
}

wiced_app_service_t *bt_hs_spk_handsfree_app_service_get(void)
{
    return &bt_hs_spk_handsfree_cb.app_service;
}

/*
 * This API will send Handsfree AT commands to HF profile
 *
 * @param           handle      : RFCOMM Handle
 * @param           cmd         : AT command to be sent.
 * @param           arg_type    : Type of the argument to be set.
 * @param           arg_format  : Argument format
 * @param           p_arg       : p_arg
 * @param           int_arg     : any number arguments to be set.
 *
 * @return          result
 */
static wiced_result_t bt_hs_spk_handsfree_at_cmd_send(uint16_t handle, char *cmd, uint8_t arg_type, uint8_t arg_format, const char *p_arg, int16_t int_arg)
{
    char    buf[WICED_BT_HFP_HF_AT_MAX_LEN + 16];
    char    *p = buf;

    memset (buf, 0, (WICED_BT_HFP_HF_AT_MAX_LEN+16));

    *p++ = 'A';
    *p++ = 'T';

    /* copy result code string */
    memcpy(p,cmd, strlen(cmd));
    p += strlen(cmd);

    if(arg_type == WICED_BT_HFP_HF_AT_SET)
    {
        *p++ = '=';

    }
    else if(arg_type == WICED_BT_HFP_HF_AT_READ)
    {
        *p++ = '?';

    }
    else if(arg_type == WICED_BT_HFP_HF_AT_TEST)
    {
        *p++ = '=';
        *p++ = '?';

    }

    /* copy argument if any */
    if (arg_format == WICED_BT_HFP_HF_AT_FMT_INT)
    {
        p += util_itoa((uint16_t) int_arg, p);
    }
    else if (arg_format == WICED_BT_HFP_HF_AT_FMT_STR)
    {
        utl_strcpy(p, p_arg);
        p += strlen(p_arg);
    }

    /* finish with \r*/
    *p++ = '\r';

    return wiced_bt_hfp_hf_send_at_cmd(handle,buf);
}

/*
 * bt_hs_spk_handsfree_call_session_check
 *
 * Check if any call session exists.
 * The call session includes
 * 1. a SCO/eSCO connection exists, or
 * 2. a HFP active call exists, or
 * 3. a HFP held call exists, or
 * 4. a incoming call setup process exists, or
 * 5  a outgoing call setup process exists
 *
 * @return  WICED_TRUE: There is at least one call session.
 *          WICED_FALSE: There is no call session.
 */
wiced_bool_t bt_hs_spk_handsfree_call_session_check(void)
{
    if (bt_hs_spk_handsfree_cb.p_active_context)
    {
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

/*
 * Check if the target link is allowed to enter sniff mode.
 *
 * The device is not suggested to enter sniff mode when one of the following conditions exists:
 * 1. a SCO/eSCO connection exists
 * 2. a HFP active call exists
 * 3. a HFP held call exists
 * 4. a incoming call setup process exists
 * 5  a outgoing call setup process exists
 *
 * @param[in]   bd_addr: peer device's BT address
 */
wiced_bool_t bt_hs_spk_handsfree_bt_sniff_mode_allowance_check(wiced_bt_device_address_t bd_addr)
{
    handsfree_app_state_t *p_ctx = NULL;

    p_ctx = get_context_ptr_from_address(bd_addr, WICED_FALSE);

    if (p_ctx == NULL)
    {
        return WICED_TRUE;
    }

    if ((p_ctx->sco_connected == WICED_TRUE) ||
        (p_ctx->call_active == WICED_TRUE) ||
        (p_ctx->call_held == WICED_TRUE) ||
        (p_ctx->call_setup >= WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING))
    {
        return WICED_FALSE;
    }

    return WICED_TRUE;
}

/**
 *
 * Transmit device's battery level to the AG.
 *
 * @param bdaddr - AG's BT address
 * @param battery_level - device's battery level (0 ~ 100)
 */
void bt_hs_spk_handsfree_battery_level_tx(wiced_bt_device_address_t bdaddr, uint8_t battery_level)
{
    handsfree_app_state_t *p_ctx = NULL;
    char at_cmd_param[WICED_BT_HFP_HF_AT_MAX_LEN];
    char *p;

    /* Check parameter. */
    if (battery_level > 100)
    {
        return;
    }

    /* Get the context. */
    p_ctx = get_context_ptr_from_address(bdaddr, WICED_FALSE);

    if (p_ctx == NULL)
    {
        return;
    }

    /* Check connection state. */
    if (p_ctx->connection_status == WICED_BT_HFP_HF_STATE_DISCONNECTED)
    {
        return;
    }

    /* Check if the AG supports HF Indicators. */
    if ((p_ctx->ag_features & WICED_BT_HFP_AG_FEATURE_HF_INDICATORS) == 0)
    {
        return;
    }

    /* Check if the AG supports Battery Level indicator. */
    if ((p_ctx->ag_indicator_mask & (1 << (WICED_BT_HFP_HF_IND_ID_BATTERY))) == 0)
    {
        return;
    }

    /* Build the AT Command Parameter. The command looks like "AT+BIEV=1,level" */
    p = at_cmd_param;
    /* Write Battery Level Identity */
    p += util_itoa(WICED_BT_HFP_HF_IND_ID_BATTERY, p);
    /* Write separator */
    UINT8_TO_STREAM(p, ',');
    /* Write Battery Level value */
    p += util_itoa(battery_level, p);

    /* Send the AT command to the AG. */
    bt_hs_spk_handsfree_at_cmd_send(p_ctx->rfcomm_handle,
                                    "+BIEV",
                                    WICED_BT_HFP_HF_AT_SET,
                                    WICED_BT_HFP_HF_AT_FMT_STR,
                                    at_cmd_param,
                                    0);
}

/**
 * bt_hs_spk_handsfree_ag_nrec_disable
 *
 * Disable AG's NREC capability if the local device supports and enables its NREC capability.
 *
 * @param[in]   bdaddr - target AG's BT address
 */
void bt_hs_spk_handsfree_ag_nrec_disable(wiced_bt_device_address_t bdaddr)
{
#ifdef NREC_ENABLE
    handsfree_app_state_t *p_ctx = NULL;

    /* Get the context. */
    p_ctx = get_context_ptr_from_address(bdaddr, WICED_FALSE);

    if (p_ctx == NULL)
    {
        return;
    }

    /* Check connection state. */
    if (p_ctx->connection_status == WICED_BT_HFP_HF_STATE_DISCONNECTED)
    {
        return;
    }

    /* Check if the AG supports NREC. */
    if ((p_ctx->ag_features & WICED_BT_HFP_AG_FEATURE_ECNR) == 0)
    {
        return;
    }

    /* Transmit the AT+NREC=0 command to the AG to disable its NREC capability. */
    bt_hs_spk_handsfree_at_cmd_send(p_ctx->rfcomm_handle,
                                    "+NREC",
                                    WICED_BT_HFP_HF_AT_SET,
                                    WICED_BT_HFP_HF_AT_FMT_INT,
                                    NULL,
                                    0);
#endif
}

/**
 * bt_hs_spk_handsfree_hfp_connection_check
 *
 * Check if the HFP is connected.
 *
 * @param[in] bdaddr - specific AG's BT address (valid only when any is to FALSE)
 *                 If this is set to NULL, the return value would be TRUE if any AG is connected.
 *
 * @param[in] slc_included - TRUE if the connection shall finish its Service Level Connection
 *                           establishment defined in the HFP spec.
 *
 * @return  WICED_TRUE
 *          WICED_FALSE
 */
wiced_bool_t bt_hs_spk_handsfree_hfp_connection_check(wiced_bt_device_address_t bdaddr, wiced_bool_t slc_included)
{
    uint16_t idx;
    wiced_bool_t match = WICED_FALSE;
    wiced_bt_hfp_hf_connection_state_t target_state = slc_included ? WICED_BT_HFP_HF_STATE_SLC_CONNECTED : WICED_BT_HFP_HF_STATE_CONNECTED;

    for (idx = 0 ; idx < WICED_BT_HFP_HF_MAX_CONN; idx++)
    {
        match = WICED_FALSE;

        if (bdaddr)
        {
            if (memcmp((void *) bdaddr,
                       (void *) bt_hs_spk_handsfree_cb.context[idx].peer_bd_addr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {
                match = WICED_TRUE;
            }
        }

        if ((bdaddr == NULL) ||
            (match == WICED_TRUE))
        {
            if (bt_hs_spk_handsfree_cb.context[idx].connection_status >= target_state)
            {
                return WICED_TRUE;
            }
        }
    }

    return WICED_FALSE;
}

/**
 * bt_hs_spk_handsfree_volume_get
 *
 * Get current speaker volume for the active call session.
 *
 * @return  speaker volume level (0 ~ 15)
 */
uint8_t bt_hs_spk_handsfree_volume_get(void)
{
    if (bt_hs_spk_handsfree_cb.p_active_context)
    {
        return bt_hs_spk_handsfree_speaker_volume_level_get(bt_hs_spk_handsfree_cb.p_active_context);
    }

    return 0;
}

/*
 * bt_hs_spk_control_disconnect
 *
 * Disconnect target peer device.
 *
 * @param bdaddr - target device's BT address
 *                 If this is set to NULL, all the connected devices will be disconnected
 */
void bt_hs_spk_handsfree_disconnect(wiced_bt_device_address_t bdaddr)
{
    uint16_t i;
    wiced_bool_t match;

    for (i = 0 ; i < WICED_BT_HFP_HF_MAX_CONN ; i++)
    {
        match = WICED_TRUE;

        if (bdaddr != NULL)
        {
            if (memcmp((void *) bt_hs_spk_handsfree_cb.context[i].peer_bd_addr,
                       (void *) bdaddr,
                       sizeof(wiced_bt_device_address_t)) != 0)
            {
                match = WICED_FALSE;
            }
        }

        if (match == WICED_TRUE)
        {
            if (bt_hs_spk_handsfree_cb.context[i].connection_status >= WICED_BT_HFP_HF_STATE_CONNECTED)
            {
                wiced_bt_hfp_hf_disconnect(bt_hs_spk_handsfree_cb.context[i].rfcomm_handle);
            }
        }
    }
}

/**
 * bt_hs_spk_handsfree_active_call_session_info_get
 *
 * Get the related call session information for the active call session
 *
 * @param[out] p_info - refer to bt_hs_spk_handsfree_active_call_session_info_t
 *
 * @return  WICED_TRUE- success
 *          WICED_FALSE - there is no active call session
 */
wiced_bool_t bt_hs_spk_handsfree_active_call_session_info_get(bt_hs_spk_handsfree_active_call_session_info_t *p_info)
{
    if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        return WICED_FALSE;
    }

    memcpy((void *) p_info->bdaddr,
           (void *) bt_hs_spk_handsfree_cb.p_active_context->peer_bd_addr,
           sizeof(wiced_bt_device_address_t));

    p_info->sco_idx = bt_hs_spk_handsfree_cb.p_active_context->sco_index;

    p_info->wide_band = bt_hs_spk_handsfree_cb.p_active_context->sco_params.use_wbs;

    return WICED_TRUE;
}

/**
 * bt_hs_spk_handsfree_audio_manager_stream_start
 *
 * Start the external codec via Audio Manager
 *
 * @param[in] p_audio_config
 */
void bt_hs_spk_handsfree_audio_manager_stream_start(audio_config_t *p_audio_config)
{
    wiced_result_t status;

    if (bt_hs_spk_handsfree_cb.stream_id != WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        return;
    }

    bt_hs_spk_handsfree_cb.stream_id = wiced_am_stream_open(HFP);

    if (bt_hs_spk_handsfree_cb.stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        WICED_BT_TRACE("wiced_am_stream_open fail\n");

        return;
    }

    /* Set PCM parameters. */
    status = wiced_am_stream_set_param(bt_hs_spk_handsfree_cb.stream_id,
                                       AM_AUDIO_CONFIG,
                                       (void *) p_audio_config);

    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_set_param %d failed (%d)\n", AM_AUDIO_CONFIG, status);
    }

    /* Start external codec sampling. */
    status = wiced_am_stream_start(bt_hs_spk_handsfree_cb.stream_id);

    if (status  != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_start failed (%d)\n", status);
    }

    /* Set speaker volume. */
    bt_hs_spk_handsfree_audio_manager_stream_volume_set(p_audio_config->volume);

    /* Set MIC gain. */
    status = wiced_am_stream_set_param(bt_hs_spk_handsfree_cb.stream_id,
                                       AM_MIC_GAIN_LEVEL,
                                       (void *) &p_audio_config->mic_gain);
    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_set_param %d failed (%d)\n", AM_MIC_GAIN_LEVEL, status);
    }
}

/**
 * Stop and close the external codec via the Audio Manager module.
 */
void bt_hs_spk_handsfree_audio_manager_stream_stop(void)
{
    wiced_result_t status;

    if (bt_hs_spk_handsfree_cb.stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        return;
    }

    status = wiced_am_stream_stop(bt_hs_spk_handsfree_cb.stream_id);

    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_stop failed (%d)\n", status);
    }

#ifdef NREC_ENABLE
    status = wiced_am_stream_set_param(bt_hs_spk_handsfree_cb.stream_id,
                                       AM_NREC_DISABLE,
                                       NULL);

    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_set_param %d failed (%d)\n", AM_NREC_DISABLE, status);
    }
#endif

    status = wiced_am_stream_close(bt_hs_spk_handsfree_cb.stream_id);

    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_close failed (%d)\n", status);
    }

    bt_hs_spk_handsfree_cb.stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
}

/**
 * bt_hs_spk_handsfree_audio_manager_stream_volume_set
 *
 * Set the external codec streaming gain via the Audio Manager module.
 *
 * @param[in] am_vol_level - from AM_VOL_LEVEL_LOW to AM_VOL_LEVEL_HIGH
 */
void bt_hs_spk_handsfree_audio_manager_stream_volume_set(int32_t am_vol_level)
{
    int32_t new_vol_level;
    wiced_result_t status;

    /* Check parameter. */
    if (bt_hs_spk_handsfree_cb.stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        return;
    }

    if (am_vol_level < AM_VOL_LEVEL_LOW)
    {
        new_vol_level = AM_VOL_LEVEL_LOW;
    }
    else if (am_vol_level > AM_VOL_LEVEL_HIGH)
    {
        new_vol_level = AM_VOL_LEVEL_HIGH;
    }
    else
    {
        new_vol_level = am_vol_level;
    }

    /* Inform Control Module. */
    if (bt_hs_spk_handsfree_cb.p_local_volume_change_cb)
    {
        (*bt_hs_spk_handsfree_cb.p_local_volume_change_cb)(new_vol_level, VOLUME_EFFECT_NONE);
    }

    /* Set volume. */
    status = wiced_am_stream_set_param(bt_hs_spk_handsfree_cb.stream_id,
                                       AM_SPEAKER_VOL_LEVEL,
                                       (void *) &new_vol_level);

    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_set_param %d failed (%d)\n", AM_SPEAKER_VOL_LEVEL, status);
    }
}

#ifdef NREC_ENABLE
/**
 * bt_hs_spk_handsfree_audio_manager_nrec_enable
 *
 * Enable externalcodec's NREC capability via Audio Manager Module.
 *
 * @return  WICED_TRUE - Success
 *          WICED_FALSE - Fail
 */
wiced_bool_t bt_hs_spk_handsfree_audio_manager_nrec_enable(void)
{
    wiced_result_t status;

    /* Check parameter. */
    if (bt_hs_spk_handsfree_cb.stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        return WICED_FALSE;
    }

    /* setting NULL config here will allow stream config to be used */
    status = wiced_am_stream_set_param(bt_hs_spk_handsfree_cb.stream_id,
                                       AM_NREC_CONFIG,
                                       NULL);
    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_set_param %d failed (%d)\n", AM_NREC_CONFIG, status);
        return WICED_FALSE;
    }

    /* Enable external codec NREC capability. */
    status = wiced_am_stream_set_param(bt_hs_spk_handsfree_cb.stream_id,
                                       AM_NREC_ENABLE,
                                       NULL);

    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("wiced_am_stream_set_param %d failed (%d)\n", AM_NREC_ENABLE, status);
        return WICED_FALSE;
    }

    return WICED_TRUE;
}
#endif // NREC_ENABLE

/**
 * bt_hs_spk_handsfree_call_session_info_set
 *
 * Set the context of Call Session
 *
 * Note: Do NOT use this utility unless you certainly understand what you are doing.
 *       Using this utility MAY cause unexpected behavior and crash.
 *
 * @param[in] p_info
 */
void bt_hs_spk_handsfree_call_session_info_set(bt_hs_spk_handsfree_call_session_info_t *p_info)
{
    if (p_info == NULL)
    {
        return;
    }

    /* Copy call session. */
    memcpy((void *) &bt_hs_spk_handsfree_cb.context[0],
           (void *) &p_info->session[0],
           sizeof(handsfree_app_state_t) * WICED_BT_HFP_HF_MAX_CONN);

    /* Set active call session. */
    if (p_info->active_call_session_index >= WICED_BT_HFP_HF_MAX_CONN)
    {
        bt_hs_spk_handsfree_active_call_session_set(NULL);
    }
    else
    {
        bt_hs_spk_handsfree_active_call_session_set(&bt_hs_spk_handsfree_cb.context[p_info->active_call_session_index]);
        /* Set current service to HFP. */
        app_set_current_service(&bt_hs_spk_handsfree_cb.app_service);
    }
}

/**
 * bt_hs_spk_handsfree_call_session_info_get
 *
 * Get the content of Call Session
 *
 * @param[out] p_info
 */
void bt_hs_spk_handsfree_call_session_info_get(bt_hs_spk_handsfree_call_session_info_t *p_info)
{
    uint8_t i;

    if (p_info == NULL)
    {
        return;
    }

    /* Copy call session. */
    memcpy((void *) &p_info->session[0],
           (void *) &bt_hs_spk_handsfree_cb.context[0],
           sizeof(handsfree_app_state_t) * WICED_BT_HFP_HF_MAX_CONN);

    /* Find active call session index. */
    for (i = 0 ; i < WICED_BT_HFP_HF_MAX_CONN ; i++)
    {
        if (bt_hs_spk_handsfree_cb.p_active_context == &bt_hs_spk_handsfree_cb.context[i])
        {
            break;
        }
    }

    p_info->active_call_session_index = i;
}

/*
 * bt_hs_spk_handsfree_speaker_volume_level_get
 *
 * Get current context HFP Speaker volume level.
 *
 * @return uint8_t: call volume level if call is active
 *                  ring tone volume level if call is under ringing state.
 */
static uint8_t bt_hs_spk_handsfree_speaker_volume_level_get(handsfree_app_state_t *p_ctx)
{
    if (p_ctx == NULL)
    {
        return 0;
    }

    if ((p_ctx->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING) ||
        (p_ctx->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING) ||
        (p_ctx->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING))
    {
        return p_ctx->ringtone_volume;
    }
    else
    {
        if (p_ctx->call_active)
        {
            return p_ctx->spkr_volume;
        }
        else
        {
            return p_ctx->ringtone_volume;
        }
    }
}

/*
 * bt_hs_spk_handsfree_volume_level_set
 *
 * Set the HFP speaker volume level for the target context
 *
 * @param[in] p_ctx - target context
 * @param[in] volume_level - HFP volume level
 */
static void bt_hs_spk_handsfree_speaker_volume_level_set(handsfree_app_state_t *p_ctx, uint8_t volume_level)
{
    if (p_ctx == NULL || bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        return;
    }

    if ((p_ctx->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING) ||
        (p_ctx->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING) ||
        (p_ctx->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING))
    {
        bt_hs_spk_handsfree_cb.p_active_context->ringtone_volume = volume_level;
    }
    else
    {
        if (p_ctx->call_active)
        {
            bt_hs_spk_handsfree_cb.p_active_context->spkr_volume = volume_level;
        }
        else
        {
            bt_hs_spk_handsfree_cb.p_active_context->ringtone_volume = volume_level;
        }
    }
}

/*
 * bt_hs_spk_handsfree_outgoing_call_notification_handler
 *
 * Handle the outgoing call notification (alert).
 *
 * @return  WICED_TRUE: the outgoing call is allowed
 *          WICED_FALSE: the outgoing call is not allowed
 */
static wiced_bool_t bt_hs_spk_handsfree_outgoing_call_notification_handler(handsfree_app_state_t *p_ctx)
{
    wiced_result_t result;

    /* Check if active call session exists. */
    if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        return WICED_TRUE;
    }

    /* Check if this notification belong to the active call session. */
    if (bt_hs_spk_handsfree_cb.p_active_context == p_ctx)
    {
        return WICED_TRUE;
    }

    /* Check if the active call session has voice connection. */
    if (bt_hs_spk_handsfree_cb.p_active_context->sco_connected == WICED_FALSE)
    {
        return WICED_TRUE;
    }

    /* Check if the active call session has an active call. */
    if (bt_hs_spk_handsfree_cb.p_active_context->call_active)
    {
        /* Check if the active call session is doing the Three-Way Calls operation. */
        if (bt_hs_spk_handsfree_cb.p_active_context->call_held)
        {
            /* Check if the existent call session is doing the Three-Way Calls operation. */
            if (bt_hs_spk_handsfree_cb.p_active_context->call_held)
            {
                /* The active call session is doing the Three-Way Calls operation now.
                 * It is supposed that the 3-way call shall not to be interrupted.
                 * Therefore, terminate this outgoing call. */
                bt_hs_spk_handsfree_call_hang_up(p_ctx);

                return WICED_FALSE;
            }
        }

        /* Since this new outgoing call is triggered by user in the AG, we assume
         * that the user knows the existent active call and want to make an outgoing
         * call in the different AG anyway. Therefore, Push current active call to held.
         *
         * Note here that we need not to check the situation that call_active is true
         * and call_setup is not idle since a phone call NOT make an outgoing call or
         * receiving a new incoming call when the user is doing voice communication now. */
        wiced_bt_hfp_hf_perform_call_action(bt_hs_spk_handsfree_cb.p_active_context->rfcomm_handle,
                                            WICED_BT_HFP_HF_CALL_ACTION_HOLD_2,
                                            NULL);

        /* Due to current firmware constraint, the headset can have only one
         * active SCO/eSCO connection at the same time.
         * Hence, temporarily disconnect the active SCO/eSCO connection. */

        /* Start the SCO connecting/disconnecting protection timer to protect this duration. */
        result = wiced_start_timer(&bt_hs_spk_handsfree_cb.sco_connecting_protection_timer,
                               BT_HS_SPK_HANDSFREE_SCO_CONNECTING_STATE_PROTECTION_TIMEOUT);

        if (result != WICED_SUCCESS)
        {
            WICED_BT_TRACE("Error fail to start the SCO connecting protection timer (%d)\n", result);
        }
        wiced_bt_sco_remove(bt_hs_spk_handsfree_cb.p_active_context->sco_index);
        return WICED_TRUE;
    }

    /* Check if the active call session has a held call. */
    if (bt_hs_spk_handsfree_cb.p_active_context->call_held)
    {
        /* Due to current firmware constraint, the headset can have only one
         * active SCO/eSCO connection at the same time.
         * Hence, temporarily disconnect the active SCO/eSCO connection. */

        /* Start the SCO connecting/disconnecting protection timer to protect this duration. */
        result = wiced_start_timer(&bt_hs_spk_handsfree_cb.sco_connecting_protection_timer,
                               BT_HS_SPK_HANDSFREE_SCO_CONNECTING_STATE_PROTECTION_TIMEOUT);

        if (result != WICED_SUCCESS)
        {
            WICED_BT_TRACE("Error fail to start the SCO connecting protection timer (%d)\n", result);
        }
        wiced_bt_sco_remove(bt_hs_spk_handsfree_cb.p_active_context->sco_index);
        return WICED_TRUE;
    }

    /* Check if the active call session is under incoming call setup process. */
    if (bt_hs_spk_handsfree_cb.p_active_context->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING)
    {
       /* Terminate this new outgoing call. */
        bt_hs_spk_handsfree_call_hang_up(p_ctx);

       return WICED_FALSE;
    }

    /* Check if the active call session is under outgoing call setup process. */
    if ((bt_hs_spk_handsfree_cb.p_active_context->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING) ||
        (bt_hs_spk_handsfree_cb.p_active_context->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING))
    {
        /* Due to current firmware constraint, the headset can have only one
         * active SCO/eSCO connection at the same time.
         * Besides, the Google Pixel series phones will try to enter sniff mode
         * even the SCO/eSCO is connected. This means we cannot remove the existent
         * SCO/eSCO connection and wait for new calling device's in-band ringtone.
         * It has a great possibility that the LMP Remove eSCO Link command will be sent out
         * later than receiving the LMP eSCO Link request from the calling device.
         * Therefore, We need to terminate the new calling. */
        bt_hs_spk_handsfree_call_hang_up(p_ctx);

        return WICED_FALSE;
    }

    return WICED_FALSE;
}

/*
 * bt_hs_spk_handsfree_incoming_call_notification_handler
 *
 * Handle the incoming call notification.
 *
 * @return  WICED_TRUE: the incoming call is allowed
 *          WICED_FALSE: the incoming call is not allowed
 */
static wiced_bool_t bt_hs_spk_handsfree_incoming_call_notification_handler(handsfree_app_state_t *p_ctx)
{
    wiced_result_t result;

    /* Check if the active call session exists. */
    if (bt_hs_spk_handsfree_cb.p_active_context == NULL)
    {
        return WICED_TRUE;
    }

    /* Check if this incoming call belongs to the active call session. */
    if (bt_hs_spk_handsfree_cb.p_active_context == p_ctx)
    {
        return WICED_TRUE;
    }

    /* Check if the active call session has voice connection. */
    if (bt_hs_spk_handsfree_cb.p_active_context->sco_connected == WICED_FALSE)
    {
        return WICED_TRUE;
    }

    /* Check if the active call session has an active call. */
    if (bt_hs_spk_handsfree_cb.p_active_context->call_active)
    {
        /* Check if the active call session is doing the Three-Way Calls operation. */
        if (bt_hs_spk_handsfree_cb.p_active_context->call_held)
        {
            /* The active call session is doing the Three-Way Calls operation now.
             * It is supposed that the 3-way calling shall not to be interrupted.
             * Therefore, reject this incoming call from another AG. */
            bt_hs_spk_handsfree_call_hang_up(p_ctx);

            return WICED_FALSE;
        }

        /* Push current active call session to held. */
        wiced_bt_hfp_hf_perform_call_action(bt_hs_spk_handsfree_cb.p_active_context->rfcomm_handle,
                                            WICED_BT_HFP_HF_CALL_ACTION_HOLD_2,
                                            NULL);

        /* Due to current firmware constraint, the headset can have only one
         * active SCO/eSCO connection at the same time.
         * Hence, temporarily disconnect the active SCO/eSCO connection. */

        /* Start the SCO connecting/disconnecting protection timer to protect this duration. */
        result = wiced_start_timer(&bt_hs_spk_handsfree_cb.sco_connecting_protection_timer,
                               BT_HS_SPK_HANDSFREE_SCO_CONNECTING_STATE_PROTECTION_TIMEOUT);

        if (result != WICED_SUCCESS)
        {
            WICED_BT_TRACE("Error fail to start the SCO connecting protection timer (%d)\n", result);
        }
        wiced_bt_sco_remove(bt_hs_spk_handsfree_cb.p_active_context->sco_index);
        return WICED_TRUE;
    }

    /* Check if the active call session has a held call. */
    if (bt_hs_spk_handsfree_cb.p_active_context->call_held)
    {
        /* Due to current firmware constraint, the headset can have only one
         * active SCO/eSCO connection at the same time.
         * Hence, temporarily disconnect the active SCO/eSCO connection. */

        /* Start the SCO connecting/disconnecting protection timer to protect this duration. */
        result = wiced_start_timer(&bt_hs_spk_handsfree_cb.sco_connecting_protection_timer,
                               BT_HS_SPK_HANDSFREE_SCO_CONNECTING_STATE_PROTECTION_TIMEOUT);

        if (result != WICED_SUCCESS)
        {
            WICED_BT_TRACE("Error fail to start the SCO connecting protection timer (%d)\n", result);
        }
        wiced_bt_sco_remove(bt_hs_spk_handsfree_cb.p_active_context->sco_index);
        return WICED_TRUE;
    }

    /* Check if the active call session has call setup indication. */
    if (bt_hs_spk_handsfree_cb.p_active_context->call_setup != WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE)
    {
        /* No matter this active call session has an incoming call or is doing outgoing call,
         * reject this incoming call from another AG. */
        bt_hs_spk_handsfree_call_hang_up(p_ctx);

        return WICED_FALSE;
    }

    return WICED_FALSE;
}

/*
 * bt_hs_spk_handsfree_active_call_session_set
 *
 * Set the target call session to be the active call session.
 *
 * @param[in] p_ctx: target call session to be the active call session
 */
static void bt_hs_spk_handsfree_active_call_session_set(handsfree_app_state_t *p_ctx)
{
    WICED_BT_TRACE("bt_hs_spk_handsfree_active_call_session_set (0x%08X)\n", p_ctx);

    bt_hs_spk_handsfree_cb.p_active_context = p_ctx;
}

/*
 * bt_hs_spk_handsfree_audio_connection_establish
 *
 * Establish the Audio Connection with the target AG.
 */
static void bt_hs_spk_handsfree_audio_connection_establish(handsfree_app_state_t *p_ctx)
{
    wiced_bt_dev_status_t status;
    wiced_result_t result;

    WICED_BT_TRACE("bt_hs_spk_handsfree_audio_connection_establish (%d)\n", p_ctx->sco_connected);
    print_bd_address(p_ctx->peer_bd_addr);

    if (p_ctx->sco_connected)
    {
        return;
    }

    if (p_ctx->sco_params.use_wbs == WICED_TRUE)
    {
        /* Send the AT+BCC command to request the Audio Gateway to trigger the AG to
         * initiate the Audio Connection Setup Process. */
        result = bt_hs_spk_handsfree_at_cmd_send(p_ctx->rfcomm_handle,
                                                 "+BCC",
                                                 WICED_BT_HFP_HF_AT_NONE,
                                                 WICED_BT_HFP_HF_AT_FMT_NONE,
                                                 NULL,
                                                 0);

        if (result != WICED_SUCCESS)
        {
            WICED_BT_TRACE("Err: Send AT+BCC fail (%d)\n", result);
        }
    }
    else
    {
        /* Since the AG doesn't support the Codec Connection process, we just
         * establish the SCO connection for HFP Audio Conneciton. */
        status = wiced_bt_sco_remove(p_ctx->sco_index);
        WICED_BT_TRACE("wiced_bt_sco_remove status: %d\n", status);
        status = wiced_bt_sco_create_as_initiator(p_ctx->peer_bd_addr,
                                                  &p_ctx->sco_index,
                                                  &p_ctx->sco_params);


        if (status != WICED_BT_PENDING)
        {
            WICED_BT_TRACE("wiced_bt_sco_create_as_initiator fail (%d)\n", status);
        }
        else
        {
            result = setup_call_audio();
            WICED_BT_TRACE("setup_call_audio: %d\n", status);
        }
    }
}

#if defined(STACK_INSIDE_FREE_RTOS) && (STACK_INSIDE_FREE_RTOS == TRUE)
static void bt_hs_spk_handsfree_sco_connecting_protection_timeout_cb(uint32_t arg)
{

}
#elif defined(WIN_EMULATOR)
static void bt_hs_spk_handsfree_sco_connecting_protection_timeout_cb(WICED_TIMER_PARAM_TYPE arg)
{

}
#else
static void bt_hs_spk_handsfree_sco_connecting_protection_timeout_cb(TIMER_PARAM_TYPE arg)
{
    WICED_BT_TRACE("[%s]\n",__func__);
    /* if sco is not created as an acceptor then remove the sco and create it as initiator. */
    if (bt_hs_spk_handsfree_cb.p_active_context->call_active && !bt_hs_spk_handsfree_cb.p_active_context->sco_connected)
    {
        WICED_BT_TRACE("[%s] Reset\n",__func__);
        wiced_bt_sco_remove(bt_hs_spk_handsfree_cb.p_active_context->sco_index);
        wiced_bt_sco_create_as_initiator(bt_hs_spk_handsfree_cb.p_active_context->peer_bd_addr, &bt_hs_spk_handsfree_cb.p_active_context->sco_index,
                                            (wiced_bt_sco_params_t *) &bt_hs_spk_handsfree_cb.p_active_context->sco_params);
    }
}
#endif


/*
 * bt_hs_spk_handsfree_call_hang_up
 *
 * Hang up the active call, reject the incoming call, or terminate the outgoing call.
 */
static void bt_hs_spk_handsfree_call_hang_up(handsfree_app_state_t *p_ctx)
{
    /* Check parameter. */
    if (p_ctx == NULL)
    {
        return;
    }

    /* Check state do avoid sending duplicate AT+CHUP commands to the AG.
     * Some AG may replies the +CME:0 command to indicates the AG error if
     * receiving duplicate AT+CHUP command. */
    if (p_ctx->call_hanging_up)
    {
        trace_log_with_bdaddr("Err: already hanging up", p_ctx->peer_bd_addr);
        return;
    }

    if (wiced_bt_hfp_hf_perform_call_action(p_ctx->rfcomm_handle,
                                            WICED_BT_HFP_HF_CALL_ACTION_HANGUP,
                                            NULL) == WICED_SUCCESS)
    {
        p_ctx->call_hanging_up = WICED_TRUE;
    }
}

/*
 * bt_hs_spk_handsfree_sco_voice_path_update
 *
 * Update the SCO voice path.
 *
 * @param[in]   uart - WICED_TRUE: the SCO data will be route to HCI UART interface
 *                     WICED_FALSE: the SCO data will be route to PCM interface
 */
void bt_hs_spk_handsfree_sco_voice_path_update(wiced_bool_t uart)
{
    wiced_result_t result = WICED_BT_ERROR;
    if (uart)
    {
        bt_hs_spk_handsfree_cb.sco_voice_path.path = WICED_BT_SCO_OVER_HCI;
        bt_hs_spk_handsfree_cb.sco_voice_path.p_sco_data_cb = &bt_hs_spk_handsfree_sco_data_app_callback;
    }
    else
    {
        bt_hs_spk_handsfree_cb.sco_voice_path.path = WICED_BT_SCO_OVER_PCM;
        bt_hs_spk_handsfree_cb.sco_voice_path.p_sco_data_cb = NULL;
    }
    result = wiced_bt_sco_setup_voice_path(&bt_hs_spk_handsfree_cb.sco_voice_path);
    WICED_BT_TRACE("[%s] SCO Setting up voice path = %d\n",__func__, result);
}

/**
 * bt_hs_spk_handsfree_sco_mic_data_add_callback_register
 *
 * Register the callback to insert user specific MIC data (PCM) into the HFP audio stream.
 * The inserted MIC data will be forwarded to the HFP AG.
 *
 * In the user callback, the user application need to provide the MIC data (with specific data
 * length in bytes) and return TRUE.
 *
 * If the user application doesn't have MIC data to be sent, the user callback shall return FALSE.
 *
 * @param p_cb - user callback to fill the MIC data
 */
void bt_hs_spk_handsfree_sco_mic_data_add_callback_register(bt_hs_spk_handsfree_mic_data_add_cb_t *p_cb)
{
    bt_hs_spk_handsfree_cb.p_mic_data_add_cb = p_cb;
}

/**
 *
 * This function allows HF to query the phone number
 *
 * @param bdaddr - AG's BT address
 */

wiced_result_t bt_hs_spk_handsfree_get_phone_number(wiced_bt_device_address_t bdaddr)
{
    handsfree_app_state_t *p_ctx = NULL;

    if(bdaddr == NULL|| bt_hs_spk_handsfree_cb.p_active_context != NULL)
    {
        p_ctx = bt_hs_spk_handsfree_cb.p_active_context;
    }
    else
    {
        return WICED_ERROR;
    }

    if (p_ctx)
    {
        return bt_hs_spk_handsfree_at_cmd_send(p_ctx->rfcomm_handle,
                                               "+BINP",
                                               WICED_BT_HFP_HF_AT_SET,
                                               WICED_BT_HFP_HF_AT_FMT_INT,
                                               NULL,
                                               1);
    }
    return WICED_ERROR;
}

/**
 *
 * This function allows HF to query the AG network operator
 *
 * @param bdaddr - AG's BT address
 */

wiced_result_t bt_hs_spk_handsfree_get_network_name(wiced_bt_device_address_t bdaddr)
{
    handsfree_app_state_t *p_ctx = NULL;

    if(bdaddr == NULL|| bt_hs_spk_handsfree_cb.p_active_context != NULL)
    {
        p_ctx = bt_hs_spk_handsfree_cb.p_active_context;
    }
    else
    {
        return WICED_ERROR;
    }

    if(p_ctx)
    {

        return bt_hs_spk_handsfree_at_cmd_send(p_ctx->rfcomm_handle,
                                               "+COPS",
                                               WICED_BT_HFP_HF_AT_READ,
                                               0,
                                               NULL,
                                               0);
    }

    return WICED_ERROR;
}

/**
 *
 * This function allows HF to query the AG subscriber number
 *
 * @param bdaddr - AG's BT address
 */


wiced_result_t bt_hs_spk_handsfree_get_sub_number_info(wiced_bt_device_address_t bdaddr)
{
    handsfree_app_state_t *p_ctx = NULL;

    if(bdaddr == NULL|| bt_hs_spk_handsfree_cb.p_active_context != NULL)
    {
        p_ctx = bt_hs_spk_handsfree_cb.p_active_context;
    }
    else
    {
        return WICED_ERROR;
    }

    if (p_ctx)
    {
        return bt_hs_spk_handsfree_at_cmd_send(p_ctx->rfcomm_handle,
                                               "+CNUM",
                                               WICED_BT_HFP_HF_AT_READ,
                                               0,
                                               NULL,
                                               0);
    }
    return WICED_ERROR;
}

/**
 *
 * This function sends command to enable/disable the “Call Waiting notification” function in the AG.
 *
 * @param bdaddr - AG's BT address
 * @param activate[in] - Either 0 or 1
 */

wiced_result_t bt_hs_spk_handsfree_call_waiting_activate(wiced_bt_device_address_t bdaddr, uint8_t activate)
{
    handsfree_app_state_t *p_ctx = NULL;

    if(bdaddr == NULL|| bt_hs_spk_handsfree_cb.p_active_context != NULL)
    {
        p_ctx = bt_hs_spk_handsfree_cb.p_active_context;
    }
    else
    {
        return WICED_ERROR;
    }

    if (p_ctx)
    {
        return bt_hs_spk_handsfree_at_cmd_send(bt_hs_spk_handsfree_cb.p_active_context->rfcomm_handle,
                                               "+CCWA",
                                               WICED_BT_HFP_HF_AT_SET,
                                               WICED_BT_HFP_HF_AT_FMT_INT,
                                               NULL,
                                               activate);
    }
    return WICED_ERROR;
}

/**
 *
 * Dial a call with a phone number.
 *
 * @param bdaddr - AG's BT address
 * @param ph_number[in] - phone number to dial
 */

wiced_result_t bt_hs_spk_handsfree_dial_phone_number(wiced_bt_device_address_t bdaddr, char *ph_number)
{

    handsfree_app_state_t *p_ctx = NULL;

    if(bdaddr == NULL|| bt_hs_spk_handsfree_cb.p_active_context != NULL)
    {
        p_ctx = bt_hs_spk_handsfree_cb.p_active_context;
    }
    else
    {
        return WICED_ERROR;
    }

    if((ph_number == NULL) || (strlen(ph_number) > 10) )
        return WICED_ERROR;

    if (p_ctx)
    {
        return bt_hs_spk_handsfree_at_cmd_send(bt_hs_spk_handsfree_cb.p_active_context->rfcomm_handle,
                                               "D",
                                               WICED_BT_HFP_HF_AT_FREE,
                                               WICED_BT_HFP_HF_AT_FMT_STR,
                                               ph_number,
                                               0);
    }
    return WICED_ERROR;
}


/**
 * bt_hs_spk_handsfree_update_hf_indicator
 *
 * This function is used to enable/disable individual indicator which is supported by AG
(AT+BIND?
 *
 * @param[in]   bdaddr - target AG's BT address
 * @param[in]   indicator_status - this contains list of values  suchas {1,1} or {0,0}
 */

wiced_result_t bt_hs_spk_handsfree_update_hf_indicator(wiced_bt_device_address_t bdaddr, char* indicator_status)
{
    wiced_result_t result = WICED_ERROR;

    handsfree_app_state_t *p_ctx = NULL;
    if(bdaddr == NULL || bt_hs_spk_handsfree_cb.p_active_context != NULL)
    {
        p_ctx = bt_hs_spk_handsfree_cb.p_active_context;
    }
    else
    {
        return result;
    }
    /* Get the context. */
    if (bdaddr != NULL)
    {
        p_ctx = get_context_ptr_from_address(bdaddr, WICED_FALSE);
    }

     if (p_ctx == NULL)
    {
        return result;
    }

    /* Check connection state. */
    if (p_ctx->connection_status == WICED_BT_HFP_HF_STATE_DISCONNECTED)
    {
        return result;
    }

    /* Check if the AG supports HF Indicators. */
    if ((p_ctx->ag_features & WICED_BT_HFP_AG_FEATURE_HF_INDICATORS) == 0)
    {
        return result;
    }

    return bt_hs_spk_handsfree_at_cmd_send(bt_hs_spk_handsfree_cb.p_active_context->rfcomm_handle,
                                           "+BIA",
                                           WICED_BT_HFP_HF_AT_SET,
                                           WICED_BT_HFP_HF_AT_FMT_STR,
                                           indicator_status,
                                           0);
    return WICED_ERROR;

}

/**
 * bt_hs_spk_handsfree_enahnced_call_status
 *
 * This function is used to retrieve current call status
 *
 * @param[in]   bdaddr - target AG's BT address
 * @param[in]   indicator_status - this contains list of values  suchas {1,1} or {0,0}
 */

wiced_result_t bt_hs_spk_handsfree_enahnced_call_status(void)
{

    if (bt_hs_spk_handsfree_cb.p_active_context)
    {
        return bt_hs_spk_handsfree_at_cmd_send(bt_hs_spk_handsfree_cb.p_active_context->rfcomm_handle,
                                               "+CLCC",
                                               WICED_BT_HFP_HF_AT_NONE,
                                               WICED_BT_HFP_HF_AT_FMT_NONE,
                                               NULL,
                                               0);
    }
    return WICED_ERROR;


}

wiced_result_t bt_hs_spk_handsfree_send_sco_data(uint8_t *buffer, uint8_t length)
{
    wiced_result_t result = WICED_SUCCESS;

    if((sco_flag) && (bt_hs_spk_handsfree_cb.p_active_context != NULL) && (bt_hs_spk_handsfree_cb.p_active_context->sco_connected))
    {
        result = wiced_bt_sco_write_buffer(bt_hs_spk_handsfree_cb.p_active_context->sco_index, buffer, length);
    }

    return result;

}

/*
 *  send HFP SLC connect complete event to UART
 */
wiced_result_t hci_control_HFP_connection_status_event(wiced_bt_device_address_t bd_addr, uint16_t handle, uint8_t is_connected)
{
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(handle) ;
    uint8_t event_data[BD_ADDR_LEN + sizeof(handle)];

    for (i = 0; i < BD_ADDR_LEN; i++)                     // bd address
        event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

    event_data[i++] = handle & 0xff;                        //handle
    event_data[i++] = (handle >> 8) & 0xff;
    if(is_connected)
        return wiced_transport_send_data(HCI_CONTROL_HF_EVENT_CONNECTED, event_data, cmd_size);
    else
        return wiced_transport_send_data(HCI_CONTROL_HF_EVENT_CLOSE, event_data, cmd_size);
}

/*
 *  send HFP SCO connected/disconnected complete event to UART
 */
wiced_result_t hci_control_HFP_sco_connection_event(uint16_t sco_idx, uint8_t isconnected, uint8_t spk_vol, uint8_t mic_vol)
{
    int i = 0;
    uint8_t cmd_size = sizeof(sco_idx) +2;
    uint8_t event_data[sizeof(sco_idx) +2];

    event_data[i++] = sco_idx & 0xff;                        //handle
    event_data[i++] = (sco_idx >> 8) & 0xff;
    event_data[i++] = spk_vol;
    event_data[i++] = mic_vol;
    if(isconnected)
        return wiced_transport_send_data(HCI_CONTROL_HF_EVENT_AUDIO_OPEN, event_data,cmd_size);
    else
        return wiced_transport_send_data(HCI_CONTROL_HF_EVENT_AUDIO_CLOSE, event_data, cmd_size);
}

/*
 *  send HFP call status event to UART
 */
wiced_result_t hci_control_HFP_callsetup_status_event(uint16_t sco_idx, uint8_t active_Call, uint8_t  held_call, uint8_t call_setup_status)
{
    int i=0;
    const int cmd_size = sizeof(sco_idx) +sizeof(active_Call) +sizeof(held_call)+sizeof(call_setup_status);
    uint8_t event_data[sizeof(sco_idx) +sizeof(active_Call) +sizeof(held_call)+sizeof(call_setup_status)];

    event_data[i++] = sco_idx & 0xff;                        //handle
    event_data[i++] = (sco_idx >> 8) & 0xff;
    event_data[i++] = active_Call;
    event_data[i++] = held_call;
    event_data[i++] = call_setup_status;
    return wiced_transport_send_data(HCI_CONTROL_HF_EVENT_CALL_SETUP_STATUS, event_data, cmd_size);
}

/*
 *  send HFP INBAND Ringtone status event to UART
 */
wiced_result_t hci_control_HFP_inband_status_event(uint16_t sco_idx, uint8_t status)
{
    int i = 0;
    const int cmd_size = sizeof(sco_idx) +sizeof(status);
    uint8_t event_data[sizeof(sco_idx) + sizeof(status)];

    event_data[i++] = sco_idx & 0xff;
    event_data[i++] = (sco_idx >> 8) & 0xff;
    event_data[i++] = status;

    return wiced_transport_send_data(HCI_CONTROL_HF_EVENT_INBAND_RING_STATUS, event_data, cmd_size);
}


/*
 *  send HFP Volume change event to UART
 */
wiced_result_t hci_control_HFP_volume_change_event(wiced_bt_device_address_t bd_addr, uint8_t vol_type, uint8_t vol, uint8_t percent)
{
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(vol_type) +2 ;
    uint8_t event_data[BD_ADDR_LEN + sizeof(vol_type)+2];

    for (i = 0; i < BD_ADDR_LEN; i++)                     // bd address
        event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

    event_data[i++] = vol_type;
    event_data[i++] = vol;
    event_data[i++] = percent;
    return wiced_transport_send_data(HCI_CONTROL_HF_EVENT_VOLUME_CHANGE, event_data, cmd_size);
}

/*
 *  send HFP Volume UP/down event to UART
 */
wiced_result_t hci_control_HFP_volume_up_down_event(wiced_bt_device_address_t bd_addr, uint8_t vol, uint8_t percent,uint8_t am_vol, uint8_t is_up)
{
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(vol) +2 ;
    uint8_t event_data[BD_ADDR_LEN + sizeof(vol)+2];

    for (i = 0; i < BD_ADDR_LEN; i++)                     // bd address
        event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

    event_data[i++] = vol;
    event_data[i++] = percent;
    event_data[i++] = am_vol;
    if(is_up)
        return wiced_transport_send_data(HCI_CONTROL_HF_EVENT_VOLUME_UP, event_data, cmd_size);
    else
        return wiced_transport_send_data(HCI_CONTROL_HF_EVENT_VOLUME_DOWN, event_data, cmd_size);
}


/*
 *  send HFP  call_alert indication to UART
 */
wiced_result_t hci_control_HFP_call_alert_event(wiced_bt_device_address_t bd_addr, uint8_t alert_type, uint8_t *caller_num)
{
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(alert_type) + 32;
    uint8_t event_data[BD_ADDR_LEN + sizeof(alert_type) + 32];

    for (i = 0; i < BD_ADDR_LEN; i++)                     // bd address
        event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

    event_data[i++] = alert_type;
    if (strlen((char *)caller_num) <= 32){
        memcpy(&event_data[i], caller_num, strlen((char *)caller_num));
    }
    else{
        return WICED_ERROR;
    }

    return wiced_transport_send_data(HCI_CONTROL_HF_EVENT_CALL_ALERT, event_data, cmd_size);
}

/*
 *  send HFP voice recognition status event to UART
 */
wiced_result_t hci_control_HFP_voice_recognition_status_event(uint16_t sco_idx, uint8_t status)
{
    int i = 0;
    const int cmd_size = sizeof(sco_idx) + sizeof(status);
    uint8_t event_data[sizeof(sco_idx) + sizeof(status)];

    event_data[i++] = sco_idx & 0xff;
    event_data[i++] = (sco_idx >> 8) & 0xff;
    event_data[i++] = status;

    return wiced_transport_send_data(HCI_CONTROL_HF_EVENT_VOICE_RECOG_EVENT, event_data, cmd_size);
}

/*****************************************************************************
* Function Name: bt_hs_spk_handsfree_get_rfcomm_handle
******************************************************************************
* Summary: Get the RF COMM handle index
*          
*     
*
* Parameters: none
*   
* 
* Return:  uint16_t:
*               the index ID, 0 on no connect
*   
*
****************************************************************************/
uint16_t bt_hs_spk_handsfree_get_rfcomm_handle(){
    if (bt_hs_spk_handsfree_cb.p_active_context){
        return bt_hs_spk_handsfree_cb.p_active_context->rfcomm_handle;
    }
    else{
        handsfree_app_state_t* app_state  = get_context_ptr_from_sco_index(BT_HS_SPK_SCO_INDEX);
        if (app_state != NULL){
            return app_state->rfcomm_handle;
        }
        else{
            return 0;
        }
    }
}


/*****************************************************************************
* Function Name: bt_hs_spk_handsfree_is_call_active
******************************************************************************
* Summary: get the call active or not 
*          
*     
*
* Parameters: none
*   
* 
* Return:  bool:
*               true: active  false: no active
*   
*
****************************************************************************/
wiced_bool_t bt_hs_spk_handsfree_is_call_active(){
if (bt_hs_spk_handsfree_cb.p_active_context){
        return true;
    }
    else{
        return false;
    }
}