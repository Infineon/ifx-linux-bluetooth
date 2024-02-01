/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company)
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
 * This is the private interface file for the audio source profile.
 */

#pragma once

#include "wiced_bt_avdt.h"
#include "wiced_bt_a2dp_int.h"
#include "wiced_bt_a2dp_source.h"
#include "wiced_bt_avrc.h"
#include "wiced_memory.h"
#include "wiced_bt_trace.h"

/*****************************************************************************
**  Constants
*****************************************************************************/

/* Set it to TRUE for adding debugging traces */
//#define WICED_BT_A2DP_SOURCE_DEBUG

#ifndef WICED_BT_A2DP_SOURCE_DEBUG
#define WICED_BT_A2DP_SOURCE_DEBUG FALSE
#endif

/* Maximun number of AVDTP signaling connections */
#define WICED_BT_A2DP_SOURCE_MAX_NUM_CONN            1
#define WICED_BT_A2DP_SOURCE_MAX_NUM_CODECS          1
/* Max stream end-points */
#define WICED_BT_A2DP_SOURCE_MAX_SEPS                WICED_BT_A2DP_SOURCE_MAX_NUM_CONN*WICED_BT_A2DP_SOURCE_MAX_NUM_CODECS
#define WICED_BT_A2DP_SOURCE_SEP_MEDIA_TYPE_INDEX    2

/* The timer in milliseconds to guard against link busy and AVDT_CloseReq failed to be sent */
#define WICED_A2DP_SOURCE_CLOSE_REQ_TIME_VAL  4000

#define WICED_BT_A2DP_SOURCE_TASK_ID          BTU_TASK

/* Define the task message box for A2DP Sink */
#define WICED_BT_A2DP_SOURCE_TASK_MBOX        TASK_MBOX_2

/* Maximum number of SEPS in stream discovery results */
#define WICED_BT_A2DP_SOURCE_NUM_SEPS         3

/* Size of database for service discovery */
#define WICED_BT_A2DP_SOURCE_DISC_BUF_SIZE    1000

/* Maximum length of AVDTP security data */
#define WICED_BT_A2DP_SOURCE_SECURITY_MAX_LEN 400

/**************************/
/* Audio channel configuration */
/**************************/
/* AVDTP signaling channel MTU at L2CAP */
#define WICED_BT_A2DP_SOURCE_AVDT_SIG_CH_MTU              wiced_bt_avrc_get_ctrl_mtu()

/* AVDTP audio transport channel MTU at L2CAP */
#define WICED_BT_A2DP_SOURCE_AVDT_AUDIO_MAX_MTU           1008

/* AVDTP audio transport channel flush timeout */
#define WICED_BT_A2DP_SOURCE_AVDT_AUDIO_FLUSH_TOUT        200

/* AVDTP audio channel max data queue size */
#define WICED_BT_A2DP_SOURCE_AVDT_AUDIO_MAX_QUEUE_SIZE    6


/* AVDTP protocol timeout values */
#define WICED_BT_A2DP_SOURCE_RET_TOUT         4
#define WICED_BT_A2DP_SOURCE_SIG_TOUT         4
#define WICED_BT_A2DP_SOURCE_IDLE_TOUT        10

/* Initiator/acceptor role for adaption */
#define WICED_BT_A2DP_SOURCE_ROLE_OPEN_ACP       0x00     /**< Connection Open Acceptor */
#define WICED_BT_A2DP_SOURCE_ROLE_OPEN_INT       0x01     /**< Connection Open Initiator */

/* Initiator/acceptor signaling roles */
#define WICED_BT_A2DP_SOURCE_ROLE_START_ACP      0x00
#define WICED_BT_A2DP_SOURCE_ROLE_START_INT      0x10     /**< Do not change this value */

/*****************************************************************************
**  Enums
*****************************************************************************/

/* These events are global to A2DP source and not handled by state machine */
typedef enum
{
    WICED_BT_A2DP_SOURCE_INVALID_EVT,             /**< Invalid event, used to signify an event from AVDT which is not handled */
    WICED_BT_A2DP_SOURCE_API_DEINIT_EVT,          /**< De-init API event */
    WICED_BT_A2DP_SOURCE_SIG_CHG_EVT,             /**< AVDT Signalling change event */
    WICED_BT_A2DP_SOURCE_AVDT_REPOPT_CONN_EVT,    /**< AVDT Reporting channel connected/disconnected */
    WICED_BT_A2DP_SOURCE_GLOBAL_EVT_END           /**< IMPORTANT: This should always be the last event in this enum */
} wiced_bt_a2dp_source_global_evt_t;

/* These events are handled by the A2DP source state machine for each connection control block */
typedef enum
{
    WICED_BT_A2DP_SOURCE_API_CONNECT_EVT = WICED_BT_A2DP_SOURCE_GLOBAL_EVT_END,
    WICED_BT_A2DP_SOURCE_API_DISCONNECT_EVT,
    WICED_BT_A2DP_SOURCE_API_START_EVT,
    WICED_BT_A2DP_SOURCE_API_START_RESP_EVT,
    WICED_BT_A2DP_SOURCE_API_SUSPEND_EVT,
    WICED_BT_A2DP_SOURCE_SDP_DISC_OK_EVT,
    WICED_BT_A2DP_SOURCE_SDP_DISC_FAIL_EVT,
    WICED_BT_A2DP_SOURCE_STR_OPEN_OK_EVT,
    WICED_BT_A2DP_SOURCE_STR_OPEN_FAIL_EVT,
    WICED_BT_A2DP_SOURCE_STR_START_IND_EVT,
    WICED_BT_A2DP_SOURCE_STR_START_CFM_EVT,
    WICED_BT_A2DP_SOURCE_STR_START_FAIL_EVT,
    WICED_BT_A2DP_SOURCE_STR_CLOSE_IND_EVT,
    WICED_BT_A2DP_SOURCE_STR_CLOSE_CFM_EVT,
    WICED_BT_A2DP_SOURCE_STR_SUSPEND_CFM_EVT,
    WICED_BT_A2DP_SOURCE_AVDT_CONNECT_EVT,
    WICED_BT_A2DP_SOURCE_AVDT_DISCONNECT_EVT,
    WICED_BT_A2DP_SOURCE_API_AVDT_SETCONFIG,
    WICED_BT_A2DP_SOURCE_STR_DISCOVER_CFM_EVT,
    WICED_BT_A2DP_SOURCE_STR_GETCAP_CFM_EVT,
    WICED_BT_A2DP_SOURCE_API_RECONFIG_CMD_EVT,
    WICED_BT_A2DP_SOURCE_STR_RECONFIG_CFM_EVT,
    WICED_BT_A2DP_SOURCE_STR_CONFIG_IND_EVT,
    WICED_BT_A2DP_SOURCE_LAST_EVENT                   /**< IMPORTANT: This should always be the last event in this enum */
} wiced_bt_a2dp_source_state_evt_t;

/* State machine states - Applicable for each connection control block */
typedef enum
{
    WICED_BT_A2DP_SOURCE_INIT_SST,
    WICED_BT_A2DP_SOURCE_SIG_OPENING_SST,
    WICED_BT_A2DP_SOURCE_SIG_OPEN_SST,
    WICED_BT_A2DP_SOURCE_INCOMING_SST,
    WICED_BT_A2DP_SOURCE_OUTGOING_SST,
    WICED_BT_A2DP_SOURCE_OPEN_SST,
    WICED_BT_A2DP_SOURCE_CLOSING_SST
} wiced_bt_a2dp_source_state_t;

/*****************************************************************************
**  Data types
*****************************************************************************/

/* Data type for
 * WICED_BT_A2DP_SOURCE_API_CONNECT_EVT
 * WICED_BT_A2DP_SOURCE_API_DISCONNECT_EVT
 * WICED_BT_A2DP_SOURCE_API_SUSPEND_EVT
 */
typedef struct
{
    wiced_bt_avdt_evt_hdr_t     hdr;
    wiced_bt_device_address_t   bd_address;
    uint8_t                     label;  /* Transaction label for sending the start response */
    uint8_t                     status; /* Status of the start response*/
    uint16_t                    handle;
} wiced_bt_a2dp_source_api_data_t;

/* Data type for
 * WICED_BT_A2DP_SOURCE_API_START_EVT
 */
typedef struct
{
    wiced_bt_avdt_evt_hdr_t     hdr;
    uint16_t                    handle;
    wiced_bt_a2dp_codec_info_t  codec_params;
}wiced_bt_a2dp_source_api_start_t;

/* Data type for
 * WICED_BT_A2DP_SOURCE_STR_CONFIG_RSP_OK_EVT and
 * WICED_BT_A2DP_SOURCE_STR_CONFIG_RSP_FAIL_EVT
 */
typedef struct
{
    wiced_bt_avdt_evt_hdr_t     hdr;
    uint8_t                     err_code;
    uint8_t                     category;
    wiced_bt_device_address_t   peer_addr;      /* peer BD address */
} wiced_bt_a2dp_source_setconfig_rsp_t;

/* Data type for all stream events from AVDTP */
typedef struct {
    wiced_bt_avdt_evt_hdr_t     hdr;
    wiced_bt_avdt_cfg_t         cfg;        /* configuration/capabilities parameters */
    wiced_bt_avdt_ctrl_t        msg;        /* AVDTP callback message parameters */
    wiced_bt_device_address_t   bd_addr;    /* bd address */
    uint8_t                     handle;
    uint8_t                     avdt_event;
} wiced_bt_a2dp_source_str_msg_t;

/* Data type for WICED_BT_A2DP_SOURCE_STR_DISC_OK_EVT */
typedef struct
{
    wiced_bt_avdt_evt_hdr_t     hdr;
    uint16_t                    avdt_version;   /* AVDTP protocol version */
} wiced_bt_a2dp_source_sdp_res_t;

/* Union of all event datatypes */
typedef union
{
    wiced_bt_avdt_evt_hdr_t              hdr;
    wiced_bt_a2dp_source_api_start_t     start_req;
    wiced_bt_a2dp_source_api_data_t      api_data;
    wiced_bt_a2dp_source_setconfig_rsp_t setconfig_rsp;
    wiced_bt_a2dp_source_str_msg_t       str_msg;
    wiced_bt_a2dp_source_sdp_res_t       sdp_res;
} wiced_bt_a2dp_source_data_t;

/* type for SEP control block */
typedef struct
{
    wiced_bool_t                in_use;             /* SEP in use */
    uint8_t                     av_handle;          /* AVDTP handle */
    wiced_bt_a2dp_codec_t       codec_type;         /* codec type */
    uint16_t                    codec_id;           /* codec id for codec_type=WICED_BT_A2DP_SOURCE_CODEC_VENDOR_SPECIFIC */
    uint8_t                     codec_cap_index;    /* Respective CODEC index id */
} wiced_bt_a2dp_source_sep_t;

typedef struct
{
    uint8_t seid;
    wiced_bt_avdt_cfg_t peer_caps;
    wiced_bool_t caps_already_updated;
} wiced_bt_a2dp_source_av_sep_info;


/* Type for A2DP source stream control block */
typedef struct
{
    wiced_bt_sdp_discovery_db_t     *p_sdp_db;       /* pointer to SDP database */
    wiced_bt_avdt_cfg_t              *p_cap;          /* buffer used for get capabilities */
    wiced_bt_avdt_sep_info_t         *sep_info;      /* stream discovery results */
    wiced_bt_avdt_cfg_t              cfg;            /* local SEP configuration */
    wiced_bt_device_address_t        peer_addr;      /* peer BD address */
    uint16_t                         l2c_cid;        /* L2CAP channel ID */
    uint16_t                         stream_mtu;     /* MTU of stream */
    uint16_t                         avdt_version;   /* the avdt version of peer device */
    wiced_bt_a2dp_codec_t            codec_type;     /* codec type */
    uint16_t                         codec_id;       /* codec id for codec_type=WICED_BT_A2DP_SOURCE_CODEC_VENDOR_SPECIFIC */
    uint8_t                          media_type;     /* Media type */
    wiced_result_t                   open_status;    /* open failure status */
    uint16_t                         cur_psc_mask;   /* Protocol service capabilities mask for current connection */
    uint8_t                          avdt_handle;    /* AVDTP handle */
    uint8_t                          num_seps;       /* number of seps returned by stream discovery */
    uint8_t                          sep_info_idx;   /* current index into sep_info */
    uint8_t                          sep_idx;        /* current index into local seps[] */
    uint8_t                          rcfg_idx;       /* reconfig requested index into sep_info */
    uint8_t                          state;          /* state machine state */
    uint8_t                          avdt_label;     /* AVDTP label */
    uint8_t                          app_id;         /* application id */
    uint8_t                          num_recfg;      /* number of reconfigure sent */
    uint8_t                          role;
    uint8_t                          l2c_bufs;       /* the number of buffers queued to L2CAP */
    wiced_bool_t                     started;        /* TRUE if stream started */
    wiced_bool_t                     co_started;     /* TRUE if stream started from call-out perspective */
    wiced_bool_t                     recfg_sup;      /* TRUE if the first attempt to reconfigure the stream was successfull, else False if command fails */
    wiced_bool_t                     suspend_sup;    /* TRUE if Suspend stream is supported, else FALSE if suspend command fails */
    wiced_bool_t                     deregistring;   /* TRUE if deregistering */
    wiced_bool_t                     recfg_ind;      /* TRUE if reconfigure attempt happens in open state */
    wiced_bool_t                     is_api_close;   /* Whether the close is called by local device through API or not */
    wiced_bt_a2dp_source_av_sep_info av_sep_info[WICED_BT_A2DP_SOURCE_MAX_NUM_CODECS];
    uint8_t                          configured_sep; /* Stored configured SEP id */
    wiced_bt_a2dp_codec_info_t       cap_configured; /* Stored configured CODEC capabilities */
    wiced_bool_t                     is_accepter;    /* TRUE if device is accepter otherwise false */
} wiced_bt_a2dp_source_scb_t;

/* Type for A2DP source connection control block */
typedef struct
{
    wiced_bool_t                     in_use;         /* whether ccb is in use or not */
    wiced_bt_device_address_t        peer_addr;      /* peer BD address */
    uint8_t                          state;          /* state machine state */
    uint8_t                          ccb_handle;     /* ccb handle to layer above */
    uint8_t                          avdt_handle;    /* avdt handle from layer below */
    uint8_t                          scb_hdl;        /* stream control block associated with this connection */
    wiced_bt_avdt_ctrl_cback_t      *p_dt_cback;     /* the callback function to receive events from AVDT control channel */
    wiced_bool_t                     is_str_active;  /* whether stream channel is active */
    wiced_bt_a2dp_source_scb_t        *p_scb;          /* pointer to the stream control block */
} wiced_bt_a2dp_source_ccb_t;

/* Type for A2DP source control block */
typedef struct
{
    wiced_bt_a2dp_source_sep_t         seps[WICED_BT_A2DP_SOURCE_MAX_SEPS];
    wiced_bt_a2dp_source_config_data_t     *p_config_data;  /* Configuration data from the application */
    wiced_bt_a2dp_source_control_cb_t  control_cb;     /* Application registered control callback function */
    wiced_bool_t                     is_init;        /* A2DP Sink is initialized or not */
    wiced_bt_device_address_t        sdp_bd_addr;    /* peer BD address */
    wiced_bt_a2dp_source_ccb_t         ccb[WICED_BT_A2DP_SOURCE_MAX_NUM_CONN];  /* AVDTP connection control block */
    wiced_bt_a2dp_source_scb_t         p_scb[WICED_BT_A2DP_SOURCE_MAX_NUM_CONN];/* Pointer to a stream control block */
    wiced_bt_heap_t*                   heap; /* Heap to be used for dynamic memory */
} wiced_bt_a2dp_source_cb_t;

/* Type for non state machine action functions */
typedef void (*wiced_bt_a2sp_source_nsm_act_t)(uint16_t avdt_event, wiced_bt_a2dp_source_data_t *p_data);

/* Type for state machine action functions */
typedef void (*wiced_bt_a2dp_source_sm_act_t)(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data);

typedef struct {
    uint8_t                     event;      /**< One of wiced_bt_a2dp_source_state_evt_t events */
    uint8_t                     next_state; /**< One of wiced_bt_a2dp_source_state_t states */
    wiced_bt_a2dp_source_sm_act_t pfhandler;  /**< Event Handler */
} wiced_bt_a2dp_source_event_map_t;

typedef struct  {
    const wiced_bt_a2dp_source_event_map_t *state_table;
    uint8_t                               size_of_table;
} wiced_bt_a2dp_source_sst_tbl_entry_t;

/*****************************************************************************
**  Global data
*****************************************************************************/

/* Control block declaration */
extern wiced_bt_a2dp_source_cb_t wiced_bt_a2dp_source_cb;

/*****************************************************************************
**  Function prototypes
*****************************************************************************/

/* Main functions */
wiced_result_t wiced_bt_a2dp_source_register(void);
extern void wiced_bt_a2dp_source_hdl_event(uint8_t event, wiced_bt_avdt_evt_hdr_t *p_msg);
extern void wiced_bt_a2dp_source_dereg_comp(void);
extern void wiced_bt_a2dp_source_dealloc_ccb(wiced_bt_a2dp_source_ccb_t *p_ccb);
extern void wiced_bt_a2dp_source_dealloc_scb(wiced_bt_a2dp_source_scb_t *p_scb);
extern void wiced_bt_a2dp_source_conn_cback(uint8_t handle,
    wiced_bt_device_address_t bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data);

/* State machine related function */
extern void wiced_bt_a2dp_source_ssm_execute(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data, uint8_t event);
extern wiced_result_t wiced_bt_a2dp_source_init_state_machine(void);


/* State machine action functions */
extern void wiced_bt_a2dp_source_do_sdp(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_cleanup(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_free_sdb(wiced_bt_a2dp_source_ccb_t *p_cb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_reconfig_cfm(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_disconnect_req(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_reconfig_rsp(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_str_opened(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_str_open_fail(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_do_close(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_close_str(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_connect_req(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_sdp_failed(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_sig_hdl_ap_close(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_do_start(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_send_start_resp( wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data );
extern void wiced_bt_a2dp_source_str_stopped(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_start_ind(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_start_ok(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_start_failed(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_suspend_cfm(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_hdl_str_close_ind(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_hdl_str_close_cfm(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_rej_conn(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_sig_opened(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_sig_closed(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_sig_open_fail(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_sig_hdl_ap_close_disconnect_req(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_sig_closed_cleanup(wiced_bt_a2dp_source_ccb_t *p_ccb,
    wiced_bt_a2dp_source_data_t *p_data);
extern void wiced_bt_a2dp_source_stream_chg(wiced_bt_a2dp_source_scb_t *p_scb,
    wiced_bool_t started);

/* Data path and avdt control callback functions */
extern void wiced_bt_a2dp_source_ctrl_cback(uint8_t handle, wiced_bt_device_address_t bd_addr,
    uint8_t event, wiced_bt_avdt_ctrl_t *p_data);

/* Codec configuration related functions */
extern wiced_bool_t wiced_bt_a2dp_source_cfg_init(wiced_bt_a2dp_codec_info_t *p_codec_info,
    uint8_t *p_built_codec_info, uint8_t *p_num_protect, uint8_t *p_protect_info);

/* Utility functions */
extern void wiced_bt_a2dp_source_utils_bdcpy(wiced_bt_device_address_t a,
    const wiced_bt_device_address_t b);

extern int wiced_bt_a2dp_source_utils_bdcmp(const wiced_bt_device_address_t a,
    const wiced_bt_device_address_t b);

extern void wiced_bt_a2dp_source_discover_req(wiced_bt_a2dp_source_ccb_t *p_ccb);

extern void wiced_bt_a2dp_source_sig_hdl_setconfig (wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data);

extern void wiced_bt_a2dp_source_sig_hdl_reconfig (wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data);

extern void wiced_bt_a2dp_source_sig_str_hdl_setconfig (wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data);

extern void wiced_bt_a2dp_source_sig_str_hdl_discovery_cfm (wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data);

extern void wiced_bt_a2dp_source_sig_str_hdl_getcap_cfm (wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data);

/* Debug functions */
#if (defined(WICED_BT_A2DP_SOURCE_DEBUG) && WICED_BT_A2DP_SOURCE_DEBUG == TRUE)
extern char *wiced_bt_a2dp_source_evt_code(uint8_t evt_code);
extern char *wiced_bt_a2dp_source_st_code(uint8_t state);
#endif

extern void wiced_bt_a2dp_source_set_route_codec_config( uint32_t audio_route, wiced_bt_a2dp_codec_info_t *codec_config, uint16_t handle, uint16_t cp_type, wiced_bool_t is_master );
extern void wiced_bt_a2dp_source_set_handle( uint16_t handle );
extern wiced_result_t wiced_bt_a2dp_source_streaming_configure_route( uint16_t handle );
extern wiced_result_t wiced_bt_a2dp_source_streaming_stop( uint16_t handle );
extern void wiced_bt_a2dp_source_stream_close( uint16_t handle );

#if (defined(WICED_BT_A2DP_SOURCE_DEBUG) && WICED_BT_A2DP_SOURCE_DEBUG == TRUE)
#define WICED_BTA2DP_SRC_TRACE   WICED_BT_TRACE
#else
#define WICED_BTA2DP_SRC_TRACE(...)
#endif

#define WICED_BTA2DP_SRC_ERROR   WICED_BT_TRACE
