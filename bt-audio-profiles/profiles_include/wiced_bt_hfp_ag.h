/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company)
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @file
 *
 * This is the public interface file for the handsfree profile Audio Gateway(AG) subsystem of
 * HCI_CONTROL application.
 */
#ifndef WICED_BT_HFP_AG_H
#define WICED_BT_HFP_AG_H

#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_timer.h"
#include "wiced_bt_utils.h"

#if BTSTACK_VER >= 0x03000001
#ifndef BOOLEAN
#define BOOLEAN uint32_t
#endif
#endif

/**
 * @defgroup  wicedbt_ag        Hands Free Profile (HFP) Audio Gateway
 *
 * This section describes the API's required to add Hands Free Profile to the user application.
 * The typical use case for this profile is, a headset wirelessly connected to a mobile phone
 * enabling the user to perform telephone functions through the headset .
 * This library can also connect to an AG supporting HSP only, if the application using this library supports HSP in the SDP.
 * @addtogroup wicedbt_ag Hands Free Profile (HFP)
 * @ingroup wicedbt_av
 * @{
 */


/******************************************************
 *                     Constants
 ******************************************************/
#define HFP_VERSION_1_5                 0x0105
#define HFP_VERSION_1_6                 0x0106
#define HFP_VERSION_1_7                 0x0107
#define HFP_VERSION_1_8                 0x0108
#define HFP_AG_VERSION                  HFP_VERSION_1_8

/* HF feature masks */
#define HFP_HF_FEAT_ECNR                0x00000001   /* Echo cancellation and/or noise reduction */
#define HFP_HF_FEAT_3WAY                0x00000002   /* Call waiting and three-way calling */
#define HFP_HF_FEAT_CLIP                0x00000004   /* Caller ID presentation capability  */
#define HFP_HF_FEAT_VREC                0x00000008   /* Voice recoginition activation capability  */
#define HFP_HF_FEAT_RVOL                0x00000010   /* Remote volume control capability  */
#define HFP_HF_FEAT_ECS                 0x00000020   /* Enhanced Call Status  */
#define HFP_HF_FEAT_ECC                 0x00000040   /* Enhanced Call Control  */
#define HFP_HF_FEAT_CODEC               0x00000080   /* Codec negotiation */
#define HFP_HF_FEAT_HF_IND              0x00000100   /* HF Indicators */
#define HFP_HF_FEAT_ESCO                0x00000200   /* eSCO S4 ( and T2 ) setting supported */

/* AG feature masks */
#define HFP_AG_FEAT_3WAY                0x00000001   /* Three-way calling */
#define HFP_AG_FEAT_ECNR                0x00000002   /* Echo cancellation and/or noise reduction */
#define HFP_AG_FEAT_VREC                0x00000004   /* Voice recognition */
#define HFP_AG_FEAT_INBAND              0x00000008   /* In-band ring tone */
#define HFP_AG_FEAT_VTAG                0x00000010   /* Attach a phone number to a voice tag */
#define HFP_AG_FEAT_REJECT              0x00000020   /* Ability to reject incoming call */
#define HFP_AG_FEAT_ECS                 0x00000040   /* Enhanced Call Status */
#define HFP_AG_FEAT_ECC                 0x00000080   /* Enhanced Call Control */
#define HFP_AG_FEAT_EXTERR              0x00000100   /* Extended error codes */
#define HFP_AG_FEAT_CODEC               0x00000200   /* Codec Negotiation */
#define HFP_AG_FEAT_HF_IND              0x00000400   /* HF Indicators */
#define HFP_AG_FEAT_ESCO                0x00000800   /* eSCO S4 ( and T2 ) setting supported */

/* SCO Codec Types */
#define HFP_CODEC_CVSD                  0x0001
#define HFP_CODEC_MSBC                  0x0002

/* ASCII charcter string of arguments to the AT command or response */
#define HFP_AG_AT_MAX_LEN               200

/* HFP VGS and VGM Max/Min value */
#define HFP_VGM_VGS_MAX                 15
#define HFP_VGM_VGS_MIN                 0

#undef  BTM_WBS_INCLUDED
#define BTM_WBS_INCLUDED                TRUE

/* SDP SupportedFeatures attribute bit mapping for HF.
   Table 5.4 of Hand-Free Profile 1.8 */
#define WICED_BT_HFP_AG_SDP_FEATURE_3WAY_CALLING    0x0001  /* Call waiting or three-way calling (yes:1, no:0) */
#define WICED_BT_HFP_AG_SDP_FEATURE_ECNR            0x0002  /* EC and/or NR function (yes:1, no:0) */
#define WICED_BT_HFP_AG_SDP_FEATURE_VRECG           0x0004  /* Voice recognition activation (yes:1, no:0) */
#define WICED_BT_HFP_AG_SDP_FEATURE_INBAND          0x0008  /* Inband Ringtone (yes:1, no:0) */
#define WICED_BT_HFP_AG_SDP_FEATURE_VTAG            0x0010  /* Attach a phone number to a voice tag (yes:1, no:0) */
#define WICED_BT_HFP_AG_SDP_FEATURE_WIDEBAND_SPEECH 0x0020  /* Wide band speech (yes:1, no:0) */
#define WICED_BT_HFP_AG_SDP_FEATURE_EVRS            0x0040  /* Enhanced Voice Recognition Status (yes/no, 1 = yes, 0 = no)  */
#define WICED_BT_HFP_AG_SDP_FEATURE_VRC_TEXT        0x0080  /* Voice Recognition Text (yes/no, 1 = yes, 0 = no) */

/* type for each service control block */
/* Handsfree device control block */
typedef struct
{
#define HFP_AG_STATE_IDLE 0
#define HFP_AG_STATE_OPENING 1
#define HFP_AG_STATE_OPEN 2
#define HFP_AG_STATE_CLOSING 3

    uint8_t state;       /* state machine state */
    uint16_t app_handle; /* Handle used to identify with the app */

    uint8_t b_is_initiator;    /* initiator of the connection ( true ) or acceptor ( false ) */
    wiced_bool_t b_slc_is_up;  /* set to TRUE when service level connection up */
    wiced_bool_t b_call_is_up; /* set to TRUE when phone call connection up for HSP */
    uint16_t hf_profile_uuid;  /* Used to store HS/HF profile ID for Initiator Role */

    uint16_t rfc_serv_handle;          /* RFCOMM server handle */
    uint16_t rfc_conn_handle;          /* RFCOMM handle of connected service */
    uint8_t hf_scn;                    /* HF's scn */
    wiced_bt_device_address_t hf_addr; /* HF's bd address */

    char res_buf[HFP_AG_AT_MAX_LEN + 1]; /* temp parsing buffer */
    int res_len;                         /* length of data in temp buffer */

    wiced_bt_sdp_discovery_db_t *p_sdp_discovery_db; /* pointer to discovery database */

    uint32_t hf_features; /* HF device features */
    uint16_t hf_version;  /* HF device profile version */

#if (BTM_WBS_INCLUDED == TRUE)
    wiced_timer_t cn_timer; /* codec negotiation timer */

    wiced_bool_t peer_supports_msbc; /* TRUE if peer supports mSBC */
    wiced_bool_t msbc_selected;      /* TRUE if we have selected mSBC */
#endif

    uint16_t sco_idx;          /* SCO handle */
    wiced_bool_t b_sco_opened; /* set to TRUE when SCO connection is open */

    wiced_bool_t retry_with_sco_only; /* ind to try with SCO only if eSCO fails */

    wiced_bool_t clip_enabled; /* set to TRUE if HF enables CLIP reporting */
    wiced_bool_t cmer_enabled; /* set to TRUE if HF enables CMER reporting */
    wiced_bool_t cmee_enabled; /* set to TRUE if HF enables CME ERROR reporting */
    uint8_t indicator_bit_map; /* Indicator bit map */
#if BTSTACK_VER >= 0x03000001
    /* TODO : for now fifo size if fixed, need to update the required max memory for rfcomm_fifo */
    uint8_t rfcomm_fifo[400];
#endif

} wiced_bt_hfp_ag_session_cb_t;

/* HS settings */
typedef struct
{
    uint8_t        spk_vol;
    uint8_t        mic_vol;
    wiced_bool_t        ecnr_enabled;
} wiced_bt_hci_control_ag_settings_t;


/* data associated with AG_OPEN_EVT */
typedef struct
{
    wiced_bt_device_address_t bd_addr;
    uint8_t status;
} wiced_bt_hfp_ag_open_t;

/* data associated with AG_CONNECTED_EVT */
typedef struct
{
    uint32_t peer_features;
} wiced_bt_wiced_bt_hfp_ag_connect_t;

/* AT command */
typedef struct
{
    uint8_t *cmd_ptr;
    uint8_t cmd_len;
} wiced_bt_hfp_ag_at_cmd_t;

/* data associated with WICED_BT_HFP_AG_EVENT_AUDIO_OPEN */
typedef struct
{
    uint8_t wbs_supported;
    uint8_t wbs_used;
} wiced_bt_hfp_ag_audio_open_t;
/* union of data associated with AG callback */
typedef union
{
    wiced_bt_hfp_ag_open_t open;
    wiced_bt_wiced_bt_hfp_ag_connect_t conn;
    wiced_bt_hfp_ag_at_cmd_t at_cmd;
    wiced_bt_hfp_ag_audio_open_t audio_open;
} wiced_bt_hfp_ag_event_data_t;

typedef enum
{
    WICED_BT_HFP_AG_EVENT_OPEN,
    WICED_BT_HFP_AG_EVENT_CLOSE,
    WICED_BT_HFP_AG_EVENT_CONNECTED,
    WICED_BT_HFP_AG_EVENT_AUDIO_OPEN,
    WICED_BT_HFP_AG_EVENT_AUDIO_CLOSE,
    WICED_BT_HFP_AG_EVENT_AT_CMD,
    WICED_BT_HFP_AG_EVENT_CLCC_REQ
}wiced_bt_hfp_ag_event_t;

typedef enum
{
    WICED_BT_HFP_AG_STATUS_SUCCESS,
    WICED_BT_HFP_AG_STATUS_FAIL_SDP,
    WICED_BT_HFP_AG_STATUS_FAIL_RFCOMM,
    WICED_BT_HFP_AG_STATUS_FAIL_CONN_TOUT
}wiced_bt_wiced_bt_hfp_ag_connection_status_t;

/** Callback for events */
typedef void (*wiced_bt_hfp_ag_hci_send_ag_event_cback_t)(wiced_bt_hfp_ag_event_t evt, uint16_t handle, wiced_bt_hfp_ag_event_data_t *p_data);

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
/*
 * Start the handsfree service. This function must be called once to initialize
 * the handsfree device.  Internally function starts SDP and RFCOMM processing.
 * This function must be called before other function in the HF API called.
 */
extern void wiced_bt_hfp_ag_startup( wiced_bt_hfp_ag_session_cb_t *p_scb, uint8_t num_scb, uint32_t features, wiced_bt_hfp_ag_hci_send_ag_event_cback_t p_app_cback );

/*
 * Opens a connection to a audio gateway.  When connection is open callback
 * function is called with a HCI_CONTROL_HF_EVENT_OPEN event indicating
 * success or failure of the operation. Only the service level data connection
 * is opened. The audio connection is not opened.
 */
extern void wiced_bt_hfp_ag_connect(wiced_bt_device_address_t bd_addr);

/*
 * Close the current connection to a audio gateway.  Any current audio
 * connection will also be closed.
 */
extern void wiced_bt_hfp_ag_disconnect( uint16_t handle );

/*
 * Opens an audio connection to the currently connected audio gateway specified
 * by the handle.
 */
extern  void wiced_bt_hfp_ag_audio_open( uint16_t handle );

/*
 * Close the currently active audio connection to a audio gateway specified by
 * the handle. The data connection remains opened.
 */
extern void wiced_bt_hfp_ag_audio_close( uint16_t handle );

/*
 * Current Call list status response
 */
extern void wiced_bt_hfp_ag_send_cmd_str( uint16_t handle , uint8_t *data, uint8_t len);
extern void wiced_bt_hfp_ag_sco_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

/* AT functions */
extern void wiced_bt_hfp_ag_send_BCS_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb);
extern void wiced_bt_hfp_ag_send_BVRA_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb, wiced_bool_t b_is_active);
extern void wiced_bt_hfp_ag_send_RING_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb);
extern uint8_t wiced_bt_hfp_ag_send_VGM_to_hf(wiced_bt_hfp_ag_session_cb_t *p_scb, int16_t gain);
extern uint8_t wiced_bt_hfp_ag_send_VGS_to_hf(wiced_bt_hfp_ag_session_cb_t *p_scb, int16_t gain);
extern uint8_t wiced_bt_hfp_ag_send_cmd_str_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb, char *data);
extern void wiced_bt_hfp_ag_send_OK_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb);
extern void wiced_bt_hfp_ag_set_cind(char *cind_str, uint8_t length);
extern void wiced_bt_hfp_ag_send_Error_to_hf (wiced_bt_hfp_ag_session_cb_t *p_scb, int error_code);
    /**
     * @} wicedbt_ag
     */
    /* end of hfp_ag */
#ifdef __cplusplus
    }
    /* extern "C" */
#endif
#endif/* WICED_BT_HFP_AG_H */
