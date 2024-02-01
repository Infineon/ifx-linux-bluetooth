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
 * This file provides the private interface definitions for handsfree
 *
 */
#pragma once

#include <stdbool.h>

#include "wiced_bt_types.h"
#include "wiced_bt_hfp_hf_int.h"
#include "wiced_bt_sco.h"
#include "bt_hs_spk_control.h"

#if ( HCI_CONTROL_HF_VERSION >= HFP_VERSION_1_7 && HCI_CONTROL_HF_IND_SUPPORTED == TRUE )
/* Max number of peer HF indicator */
#define BTA_HS_MAX_NUM_PEER_HF_IND     20
#endif

#define VOICE_RECOGNITION_ENABLE        1
#define VOICE_RECOGNITION_DISABLE       0
#define ENHANCED_VOICE_RECOGNITION_ACTIVATE        2


/* feature mask that matches spec */
#if ( WICED_BT_HFP_HF_WBS_INCLUDED == TRUE )
#define HCI_CONTROL_HF_FEAT_SPEC        ( HCI_CONTROL_HF_FEAT_ECNR | HCI_CONTROL_HF_FEAT_3WAY | \
                                  HCI_CONTROL_HF_FEAT_CLIP | HCI_CONTROL_HF_FEAT_VREC | \
                                  HCI_CONTROL_HF_FEAT_RVOL | HCI_CONTROL_HF_FEAT_ECS  | \
                                  HCI_CONTROL_HF_FEAT_ECC  | HCI_CONTROL_HF_FEAT_CODEC| \
                                  HCI_CONTROL_HF_FEAT_HF_IND | HCI_CONTROL_HF_FEAT_ESCO )

#else
#define HCI_CONTROL_HF_FEAT_SPEC        ( HCI_CONTROL_HF_FEAT_ECNR | HCI_CONTROL_HF_FEAT_3WAY | \
                                  HCI_CONTROL_HF_FEAT_CLIP | HCI_CONTROL_HF_FEAT_VREC | \
                                  HCI_CONTROL_HF_FEAT_RVOL | HCI_CONTROL_HF_FEAT_ECS  | \
                                  HCI_CONTROL_HF_FEAT_ECC  | HCI_CONTROL_HF_FEAT_HF_IND | \
                                  HCI_CONTROL_HF_FEAT_ESCO )
#endif


#define HCI_CONTROL_HF_AT_TIMEOUT_VALUE     5          /* 5 seconds */

/* AT command argument capabilities */
#define HCI_CONTROL_HF_AT_NONE              0x01        /* no argument */
#define HCI_CONTROL_HF_AT_SET               0x02        /* set value */
#define HCI_CONTROL_HF_AT_READ              0x04        /* read value */
#define HCI_CONTROL_HF_AT_TEST              0x08        /* test value range */
#define HCI_CONTROL_HF_AT_FREE              0x10        /* freeform argument */

/* AT argument format */
#define HCI_CONTROL_HF_AT_FMT_NONE          0           /* no arguments */
#define HCI_CONTROL_HF_AT_FMT_STR           1           /* string */
#define HCI_CONTROL_HF_AT_FMT_INT           2           /* integer */

#define WICED_ESCO_RETRANS_POWER          1
#define WICED_ESCO_RETRANS_QUALITY        2

#define WICED_SCO_PKT_TYPES    ( WICED_SCO_PKT_TYPES_MASK_HV3 | \
                                 WICED_SCO_PKT_TYPES_MASK_EV3 | \
                                 WICED_SCO_PKT_TYPES_MASK_EV4 | \
                                 WICED_SCO_PKT_TYPES_MASK_EV5 | \
                                 WICED_SCO_PKT_TYPES_MASK_NO_3_EV3 | \
                                 WICED_SCO_PKT_TYPES_MASK_NO_3_EV5 )

// SDP Record for Hands-Free Unit
#define WICED_HANDSFREE_HDLR_UNIT                     0x10004
#define WICED_HANDSFREE_SCN                           0x01
#define WICED_HANDSFREE_VOLUME_MIN                    0
#define WICED_HANDSFREE_VOLUME_MAX                    15

/** HF device indicators. */
typedef enum
{
    WICED_BT_HFP_HF_SERVICE_IND     =   1,
    WICED_BT_HFP_HF_CALL_IND        =   2,
    WICED_BT_HFP_HF_CALL_SETUP_IND  =   3,
    WICED_BT_HFP_HF_CALL_HELD_IND   =   4,
    WICED_BT_HFP_HF_SIGNAL_IND      =   5,
    WICED_BT_HFP_HF_ROAM_IND        =   6,
    WICED_BT_HFP_HF_BATTERY_IND     =   7
}wiced_bt_hfp_hf_indicator_t;

/* Callback when the sco state is changed. */
typedef void (bt_hs_spk_sco_state_callback_t)(void);

/** Call back function for pcm MIC data transfer */
typedef wiced_bool_t (bt_hs_spk_handsfree_mic_data_add_cb_t)(uint8_t *p_data, uint32_t len);

/* data associated with HF_OPEN_EVT */
typedef struct
{
    wiced_bt_device_address_t             bd_addr;                                /* Peer BD address  */
    uint8_t             status;                                 /* Event status   */
} hci_control_hf_open_t;

/* data associated with AT command response event */
typedef struct
{
    uint16_t            num;                                    /* Numeric Value : Please refer HF Response Section of WICED-HCI-Control-Protocol Document */
    char                str[WICED_BT_HFP_HF_MAX_AT_CMD_LEN];    /* Optional String */
} hci_control_hf_value_t;

/* data associated with HF_CONNECTED_EVT */
typedef struct
{
    uint16_t           peer_features;                           /* Peer supported feature */
} hci_control_hf_connect_t;

typedef struct bt_hs_spk_handsfree_active_call_session_info
{
    wiced_bt_device_address_t   bdaddr;     /** Active AG's BT address. */
    uint16_t                    sco_idx;    /** SCO index used for the active call session. */
    wiced_bool_t                wide_band;  /** TURE if the active call session uses wide band speech. */
} bt_hs_spk_handsfree_active_call_session_info_t;

typedef struct
{
    wiced_bt_device_address_t           peer_bd_addr;       /* Peer BD address  */
    wiced_bt_hfp_hf_connection_state_t  connection_status;  /* Connection status  */
    wiced_bool_t                        call_active;        /* Is call active  */
    wiced_bool_t                        call_held;          /* Is call on hold  */
    wiced_bt_hfp_hf_callsetup_state_t   call_setup;         /* call setup state  */
    wiced_bt_hfp_hf_inband_ring_state_t inband_ring_status; /* Inband ringtone status  */
    uint8_t                             mic_volume;         /* Mic volume  */
    uint8_t                             spkr_volume;        /* Speaker volume (call) */
    uint8_t                             ringtone_volume;    /* Ring tone volume */
    uint16_t                            sco_index;          /* HF SCO index  */
    wiced_bool_t                        sco_connected;      /* Is SCO is connected */
    uint16_t                            rfcomm_handle;      /* RFCOMM handle  */
    audio_config_t                      audio_config;       /* Audio Configuration used for Audio Manager control. */
    wiced_bt_sco_params_t               sco_params;
    uint32_t                            ag_features;        /* AG supported features. */
    uint8_t                             ag_indicator_mask;
    wiced_bool_t                        call_hanging_up;    /* Hanging up call*/
} handsfree_app_state_t;

typedef struct bt_hs_spk_handsfree_call_session_info
{
    handsfree_app_state_t   session[WICED_BT_HFP_HF_MAX_CONN];
    uint8_t                 active_call_session_index;
} bt_hs_spk_handsfree_call_session_info_t;

void handsfree_hfp_init(bt_hs_spk_control_config_hfp_t *p_config, BT_HS_SPK_CONTROL_LOCAL_VOLUME_CHANGE_CB *p_vol_chg_cb);

void bt_audio_hfp_register_sco_event(bt_hs_spk_sco_state_callback_t *p_callback);

wiced_bool_t bt_hs_spk_handsfree_target_connection_status_check(wiced_bt_device_address_t *p_bd_addr,
        wiced_bool_t *p_connected);

/**
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
wiced_bool_t bt_hs_spk_handsfree_call_session_check(void);

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
wiced_bool_t bt_hs_spk_handsfree_bt_sniff_mode_allowance_check(wiced_bt_device_address_t bd_addr);

wiced_app_service_t *bt_hs_spk_handsfree_app_service_get(void);

/*
 * bt_hs_spk_control_disconnect
 *
 * Disconnect target peer device.
 *
 * @param bdaddr - target device's BT address
 *                 If this is set to NULL, all the connected devices will be disconnected
 */
void bt_hs_spk_handsfree_disconnect(wiced_bt_device_address_t bdaddr);

/*
 * bt_hs_spk_handsfree_voice_recognition_activate
 *
 * Activate the voice recognition function resident in the AG.
 *
 * Refer to Section 4.25 of Hands-Free Profile 1.7.1
 */
wiced_result_t bt_hs_spk_handsfree_voice_recognition_activate(void);

/*
 * bt_hs_spk_handsfree_voice_recognition_activate
 *
 * Deactivate the voice recognition function resident in the AG.
 *
 */
wiced_result_t bt_hs_spk_handsfree_voice_recognition_deactivate(void);

/*
 * bt_hs_spk_handsfree_enhanced_voice_rec_cmd
 *
 * send the AT+BVRA=2 Enchanced voice recognition command to the  AG.
 *
 */

wiced_result_t bt_hs_spk_handsfree_enhanced_voice_rec_cmd(void);

/*
 * bt_hs_spk_handsfree_sco_voice_path_update
 *
 * Update the SCO voice path.
 *
 * @param[in]   uart - WICED_TRUE: the SCO data will be route to HCI UART interface
 *                     WICED_FALSE: the SCO data will be route to PCM interface
 */
void bt_hs_spk_handsfree_sco_voice_path_update(wiced_bool_t uart);

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
void hf_sco_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);


/**
 * bt_hs_spk_handsfree_send_sco_data
 *
 * Transmit handsfree sco data to the AG.
 *
 * @param buffer - data buffer
 * @param length - length of the buffer
 *
 */

wiced_result_t bt_hs_spk_handsfree_send_sco_data(uint8_t *buffer, uint8_t length);


/**
 * bt_hs_spk_handsfree_battery_level_tx
 *
 * Transmit device's battery level to the AG.
 *
 * @param bdaddr - AG's BT address
 * @param battery_level - device's battery level (0 ~ 100)
 */
void bt_hs_spk_handsfree_battery_level_tx(wiced_bt_device_address_t bdaddr, uint8_t battery_level);

/**
 * bt_hs_spk_handsfree_ag_nrec_disable
 *
 * Disable AG's NREC capability if the local device supports and enables its NREC capability.
 *
 * @param[in]   bdaddr - target AG's BT address
 */
void bt_hs_spk_handsfree_ag_nrec_disable(wiced_bt_device_address_t bdaddr);

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
wiced_bool_t bt_hs_spk_handsfree_hfp_connection_check(wiced_bt_device_address_t bdaddr, wiced_bool_t slc_included);

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
wiced_bool_t bt_hs_spk_handsfree_sco_connection_check(wiced_bt_device_address_t bdaddr);

/**
 * bt_hs_spk_handsfree_volume_get
 *
 * Get current speaker volume for the active call session.
 *
 * @return  speaker volume level (0 ~ 15)
 */
uint8_t bt_hs_spk_handsfree_volume_get(void);

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
wiced_bool_t bt_hs_spk_handsfree_active_call_session_info_get(bt_hs_spk_handsfree_active_call_session_info_t *p_info);

/**
 * bt_hs_spk_handsfree_audio_manager_stream_start
 *
 * Start the external codec via Audio Manager
 *
 * @param[in] p_audio_config
 */
void bt_hs_spk_handsfree_audio_manager_stream_start(audio_config_t *p_audio_config);

/**
 * Stop and close the external codec via the Audio Manager module.
 */
void bt_hs_spk_handsfree_audio_manager_stream_stop(void);

/**
 * bt_hs_spk_handsfree_audio_manager_stream_volume_set
 *
 * Set the external codec streaming gain via the Audio Manager module.
 *
 * @param[in] am_vol_level - from AM_VOL_LEVEL_LOW to AM_VOL_LEVEL_HIGH
 */
void bt_hs_spk_handsfree_audio_manager_stream_volume_set(int32_t am_vol_level);

#ifdef NREC_ENABLE
/**
 * bt_hs_spk_handsfree_audio_manager_nrec_enable
 *
 * Enable externalcodec's NREC capability via Audio Manager Module.
 *
 * @return  WICED_TRUE - Success
 *          WICED_FALSE - Fail
 */
wiced_bool_t bt_hs_spk_handsfree_audio_manager_nrec_enable(void);
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
void bt_hs_spk_handsfree_call_session_info_set(bt_hs_spk_handsfree_call_session_info_t *p_info);

/**
 * bt_hs_spk_handsfree_call_session_info_get
 *
 * Get the content of Call Session
 *
 * @param[out] p_info
 */
void bt_hs_spk_handsfree_call_session_info_get(bt_hs_spk_handsfree_call_session_info_t *p_info);

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
void bt_hs_spk_handsfree_sco_mic_data_add_callback_register(bt_hs_spk_handsfree_mic_data_add_cb_t *p_cb);


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
uint16_t bt_hs_spk_handsfree_get_rfcomm_handle();


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
wiced_bool_t bt_hs_spk_handsfree_is_call_active(void);