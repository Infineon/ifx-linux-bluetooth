/**
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
 *
 * Bluetooth Wiced AVRC Remote Control Controller interface
 *
 */

#include "wiced_bt_avrc_ct.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_trace.h"
#include "string.h"
#include "wiced_memory.h"
#include "wiced_bt_avrc.h"


#define CASE_RETURN_STR(const) case const: return #const;

#define BTAVRCP_TRACE_DEBUG

#ifdef BTAVRCP_TRACE_DEBUG
#define WICED_BTAVRCP_TRACE WICED_BT_TRACE
#else
#define WICED_BTAVRCP_TRACE(...)
#endif

#ifndef MAX_CONNECTED_RCC_DEVICES
#define MAX_CONNECTED_RCC_DEVICES       2
#endif

#ifndef AVRC_CT_SECURITY_REQUIRED
#define AVRC_CT_SECURITY_REQUIRED       (BTM_SEC_IN_AUTHENTICATE |      \
                                         BTM_SEC_OUT_AUTHENTICATE |     \
                                         BTM_SEC_ENCRYPT)
#endif

#define MAX_TRANSACTIONS_PER_SESSION    16
#define INVALID_CB_INDEX                0xFF
#define INVALID_TRANSACTION_LABEL       0xFF
#define PLAYBACK_POSITION_CHANGE_SEC    1
#define DEFAULT_METADATA_SCRATCH_SZ     1024
#define MAX_METADATA_RCV_MSG_SIZE       512
#define MAX_AVCT_RCV_PKT_SIZE           512
#define DEFAULT_METADATA_CMD_SIZE       512
#define DEFAULT_METADATA_RSP_SIZE       512

#define APP_AVRC_TEMP_BUF           128

 /* size of database for service discovery */
#define RCC_DISC_BUF_SIZE               512
#define RCC_SERVICE_NAME_LEN            35

/* Definitions for flags used in rcc_cb */
#define RCC_FLAG_DISABLING              0x01    /* Deinit called */
#define RCC_FLAG_DISC_CB_IN_USE         0x02    /* Discovery database p_disc_db is in use */
#define RCC_FLAG_RC_API_OPEN_PENDING    0x04    /* AVRCP API open is pending */
#define RCC_FLAG_RC_OPENED              0x08    /* AVRCP connection opened */

/* WICED-REMOTE-CONTROL features masks */
#define RCC_FEAT_PROTECT     0x0004  /* Streaming media content protection */
#define RCC_FEAT_VENDOR      0x0008  /* Remote control vendor dependent commands */
#define RCC_FEAT_BROWSE      0x0010  /* Browsing channel support */
#define RCC_FEAT_REPORT      0x0020  /* Use reporting service for VDP */
#define RCC_FEAT_DELAY_RPT   0x0040  /* Use delay reporting */
#define RCC_FEAT_METADATA    0x0080  /* AVRC Metadata transfer supported */

#define RCC_STATUS_NO_RSP           0xFF
#define RCC_REMOTE_RSP_EVT          1
#define RCC_META_MSG_EVT            2
#define RCC_REMOTE_CMD_EVT          3
#define RCC_VENDOR_CMD_EVT          4
#define RCC_VENDOR_RSP_EVT          5

/* Pass Through commands */
typedef enum {
    PASS_CMD_PLAY = 0x44,
    PASS_CMD_STOP = 0x45,
    PASS_CMD_PAUSE = 0x46,
    PASS_CMD_RECORD = 0x47,
    PASS_CMD_REWIND = 0x48,
    PASS_CMD_FFWD = 0x49,
    PASS_CMD_EJECT = 0x4A,
    PASS_CMD_FORWARD = 0x4B,
    PASS_CMD_BACKWARD = 0x4C,
} btrcc_pass_cmd;

/* global constant for "any" bd addr */
wiced_bt_device_address_t bd_addr_any0 = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

#define isValidPassThruComd(x) (((x) >= PASS_CMD_PLAY) && ((x) <= PASS_CMD_BACKWARD))

/* Valid commands for AVRCP 1.5 */
#if AVRC_ADV_CTRL_INCLUDED == TRUE
#define isValidCapability(x) ((((x) >= AVRC_EVT_PLAY_STATUS_CHANGE) &&  \
                                ((x) <= AVRC_EVT_PLAY_POS_CHANGED)) ||   \
                               (((x) >= AVRC_EVT_APP_SETTING_CHANGE) && \
                                ((x) <= AVRC_EVT_VOLUME_CHANGE)))
#else

/* Valid commands for AVRCP 1.3 */
#define isValidCapability(x) ((((x) >= AVRC_EVT_PLAY_STATUS_CHANGE) &&  \
                                ((x) <= AVRC_EVT_PLAY_POS_CHANGED)) ||   \
                                ((x) == AVRC_EVT_APP_SETTING_CHANGE))
#endif

#if AVRC_ADV_CTRL_INCLUDED == TRUE
#define isValidMediaScope(x) ((x) <= AVRC_SCOPE_NOW_PLAYING)
#define isValidMediaAttribute(x) (((x) >= AVRC_MEDIA_ATTR_ID_TITLE) && \
                                  ((x) <= AVRC_MEDIA_ATTR_ID_PLAYING_TIME))

#define CHECK_BROWSING_SUPPORTED(x) \
        if (((x)->peer_version < AVRC_REV_1_4) || \
            ((x)->peer_features & AVRC_SUPF_TG_BROWSE) == 0)\
        {\
            WICED_BT_TRACE("%s: Not AVRCP 1.4 BROWSE capable: 0x%x 0x%x", \
                        __FUNCTION__, (x)->peer_version, (x)->peer_features ); \
            return WICED_UNSUPPORTED;\
        }

#define isValidAbsoluteVolume(x) ((x) <= AVRC_MAX_VOLUME)
#endif

const uint32_t  rcc_meta_caps_co_ids[] = {
    AVRC_CO_METADATA,
    AVRC_CO_BROADCOM
};

/*
 * If the number of event IDs is changed in this array, NUM_RC_EVT_IDS also needs to be changed.
 */
const uint8_t  rcc_meta_caps_evt_ids[] = {
#if AVRC_ADV_CTRL_INCLUDED == TRUE
    AVRC_EVT_VOLUME_CHANGE,
#endif
};

#ifndef NUM_RC_EVT_IDS
#define NUM_RC_EVT_IDS   (sizeof(rcc_meta_caps_evt_ids) / sizeof(rcc_meta_caps_evt_ids[0]))
#endif /* NUM_RC_EVT_IDS */

/* AVRC configuration structure */
typedef struct
{
    uint32_t  company_id;         /* AVRCP Company ID */
    uint16_t  avrc_mtu;           /* AVRCP MTU at L2CAP */
    uint16_t  avrc_br_mtu;        /* AVRCP MTU at L2CAP for the Browsing channel */
    uint16_t  avrc_ct_cat;        /* AVRCP controller categories */
    uint16_t  avrc_tg_cat;        /* AVRCP target categories */

    uint8_t   num_co_ids;         /* company id count in p_meta_co_ids */
    uint8_t   num_evt_ids;        /* event id count in p_meta_evt_ids */
    const uint32_t* p_meta_co_ids;/* the metadata Get Capabilities response for company id */
    const uint8_t* p_meta_evt_ids;/* the the metadata Get Capabilities response for event id */

    char          avrc_controller_name[RCC_SERVICE_NAME_LEN]; /* Default AVRCP controller name */
    char          avrc_target_name[RCC_SERVICE_NAME_LEN];     /* Default AVRCP target name */
} REMOTE_CONTROL_CFG;

const REMOTE_CONTROL_CFG remote_control_config =
{
    AVRC_CO_BROADCOM,                           /* AVRCP Company ID */
#if AVRC_METADATA_INCLUDED == TRUE
    672,                                        /* AVRCP MTU at L2CAP for control channel */
#else
    48,                                         /* AVRCP MTU at L2CAP for control channel */
#endif
    672,                                       /* AVRCP MTU at L2CAP for the Browsing channel */
    (AVRC_SUPF_CT_CAT1 | AVRC_SUPF_CT_BROWSE),     /* AVRCP controller categories */
    (AVRC_SUPF_TG_CAT2 | AVRC_SUPF_TG_BROWSE),     /* AVRCP target categories */

    2,
    NUM_RC_EVT_IDS,
    rcc_meta_caps_co_ids,
    rcc_meta_caps_evt_ids,

    "AVRC Controller",      /* Default AVRCP controller name */
    "AVRC Target"           /* Default AVRCP target name*/
};

REMOTE_CONTROL_CFG* p_remote_control_config = (REMOTE_CONTROL_CFG*)&remote_control_config;

const uint16_t remote_control_config_id[] =
{
    0x0000, /* bit mask: 0=SELECT, 1=UP, 2=DOWN, 3=LEFT,
                         4=RIGHT, 5=RIGHT_UP, 6=RIGHT_DOWN, 7=LEFT_UP,
                         8=LEFT_DOWN, 9=ROOT_MENU, 10=SETUP_MENU, 11=CONT_MENU,
                         12=FAV_MENU, 13=EXIT */

    0,      /* not used */

    0x0000, /* bit mask: 0=0, 1=1, 2=2, 3=3,
                         4=4, 5=5, 6=6, 7=7,
                         8=8, 9=9, 10=DOT, 11=ENTER,
                         12=CLEAR */

    0x0000, /* bit mask: 0=CHAN_UP, 1=CHAN_DOWN, 2=PREV_CHAN, 3=SOUND_SEL,
                         4=INPUT_SEL, 5=DISP_INFO, 6=HELP, 7=PAGE_UP,
                         8=PAGE_DOWN */

    0x0006, /* bit mask: 0=POWER, 1=VOL_UP, 2=VOL_DOWN, 3=MUTE,
                         4=PLAY, 5=STOP, 6=PAUSE, 7=RECORD,
                         8=REWIND, 9=FAST_FOR, 10=EJECT, 11=FORWARD,
                         12=BACKWARD */

    0x0000, /* bit mask: 0=ANGLE, 1=SUBPICT */

    0,      /* not used */

    0x0000  /* bit mask: 0=not used, 1=F1, 2=F2, 3=F3,
                         4=F4, 5=F5 */
};

uint16_t* p_remote_control_config_id = (uint16_t*)remote_control_config_id;


typedef enum
{
    RCC_STATE_IDLE,
    RCC_STATE_CONNECTING,
    RCC_STATE_CONNECTED
} tRCC_STATE;

typedef struct {
    wiced_bool_t in_use;
    uint8_t      label;
    uint8_t      handle;
    union
    {
        uint8_t attribute_id;
        uint8_t event_id;   /* event Id for notification to be tracked */
    }u;
} rcc_transaction_t;

typedef struct
{
    tRCC_STATE                          state;
    uint8_t                             rc_handle;
    wiced_bt_device_address_t           peer_bda;
    uint16_t                            peer_version;
    uint16_t                            peer_features;
    uint16_t                            peer_ct_version;
    uint16_t                            peer_ct_features;
    uint16_t                            last_UID_counter;
    wiced_bool_t                        app_event_enabled;
#if AVRC_ADV_CTRL_INCLUDED == TRUE
    uint8_t                             abs_volume_reg_label;
    uint8_t                             abs_volume_supported;
#endif
    rcc_transaction_t                   transaction[MAX_TRANSACTIONS_PER_SESSION];
    uint8_t                             current_volume;
    uint8_t                             role;
} rcc_device_t;

/* Info from peer's AVRC SDP record (included in RC_OPEN_EVT) */
typedef struct
{
    uint16_t version;         /* AVRCP version */
    uint32_t features;        /* Supported features (see AVRC_SUPF_* definitions in avrc_api.h) */
} REMOTE_CONTROL_INFO;


typedef struct
{
    wiced_bool_t                is_initialized;
    uint8_t                     rc_acp_handle[MAX_CONNECTED_RCC_DEVICES];
    uint32_t                    local_features;
    uint32_t                    remote_features;
    uint32_t                    features;
    REMOTE_CONTROL_INFO         peer_ct;        /* peer CT role info */
    REMOTE_CONTROL_INFO         peer_tg;        /* peer TG role info */
    uint8_t                     flags;
    wiced_bt_device_address_t   sdb_bd_addr;

    wiced_bt_avrc_ct_connection_state_cback_t   connection_cb;
    wiced_bt_avrc_ct_cmd_cback_t                cmd_cb;
    wiced_bt_avrc_ct_rsp_cback_t                rsp_cb;
    wiced_bt_avrc_ct_pt_rsp_cback_t             pt_rsp_cb;
#if AVRC_ADV_CTRL_INCLUDED == TRUE
    wiced_bt_avrc_ct_features_cback_t           features_callback;
#endif

    rcc_device_t                                device[MAX_CONNECTED_RCC_DEVICES];
    wiced_bt_sdp_discovery_db_t* p_disc_db;

    uint8_t* supported_events;
#define MAX_QUEUED_TRANSACTIONS     6
    uint16_t                    num_held_trans;
    struct
    {
        uint8_t                     ctype;
        uint8_t                     handle;
        uint8_t                     label;
        wiced_bt_avrc_xmit_buf_t* p_buf;
    } xmit_queue[MAX_QUEUED_TRANSACTIONS];

    void* p_avct_buf[MAX_CONNECTED_RCC_DEVICES];
    void* p_avrc_buf[MAX_CONNECTED_RCC_DEVICES];
    tDRB* p_browse_drb[MAX_CONNECTED_RCC_DEVICES]; /*Pointer to the Browsing DRB buffer*/

#ifdef AVCT_MAP_FRAGMENTED_RESPONSE_ALLOCATION
    tAVCT_GET_FRAG_BUFFER_ALLOC     orig_get_buf;
    uint16_t                        orig_buffer_size;
    void* p_pool;
#endif
} rcc_cb_t;

typedef struct
{
    uint32_t                        db_len;   /* Length, in bytes, of the discovery database */
    wiced_bt_sdp_discovery_db_t* p_db;    /* Pointer to the discovery database */
    uint16_t                        num_attr; /* The number of attributes in p_attrs */
    uint16_t* p_attrs; /* The attributes filter. If NULL, AVRCP API sets the attribute filter
                                               * to be ATTR_ID_SERVICE_CLASS_ID_LIST, ATTR_ID_BT_PROFILE_DESC_LIST,
                                               * ATTR_ID_SUPPORTED_FEATURES, ATTR_ID_SERVICE_NAME and ATTR_ID_PROVIDER_NAME.
                                               * If not NULL, the input is taken as the filter. */
} RCC_SDP_DB_PARAMS;

/*****************************************************************************
**  Static variables
******************************************************************************/
static rcc_cb_t rcc_cb;

#ifdef CT_HANDLE_PASSTHROUGH_COMMANDS
static wiced_bt_avrc_ct_pt_evt_cback_t wiced_bt_avrc_ct_pt_evt_cback = NULL;
#endif

/******************************************************
 *               Function declarations
 ******************************************************/
static rcc_device_t* wiced_bt_avrc_ct_device_for_handle(uint8_t rc_handle);
static uint8_t wiced_bt_avrc_ct_device_index_for_address(wiced_bt_device_address_t bd_addr, uint8_t* free_cb_indx);
#ifdef TBD
static wiced_result_t wiced_bt_avrc_ct_device_for_address(wiced_bt_device_address_t bd_addr, rcc_device_t** prcc_dev);
#endif
static rcc_transaction_t* wiced_bt_avrc_ct_get_transaction_by_label(rcc_device_t* rcc_dev, uint8_t label);
static wiced_result_t  wiced_bt_avrc_ct_get_transaction_for_device(rcc_device_t* rcc_dev, rcc_transaction_t** ptransaction);
static void wiced_bt_avrc_ct_release_transaction_for_device(rcc_device_t* rcc_dev, uint8_t label);

static void wiced_bt_avrc_ct_start_discovery(wiced_bt_device_address_t  peer_addr);
static wiced_result_t wiced_bt_avrc_ct_send_getcaps_cmd(rcc_device_t* prcc_dev);
static uint8_t wiced_bt_avrc_ct_process_meta_cmd(wiced_bt_avrc_metadata_rsp_t* p_rc_rsp, wiced_bt_avrc_cmd_t* p_msg, uint8_t* p_ctype);
#if AVRC_ADV_CTRL_INCLUDED == TRUE
static wiced_bt_avrc_xmit_buf_t* wiced_bt_avrc_ct_set_absolute_volume_cmd(rcc_device_t* rcc_dev, uint8_t label, wiced_bt_avrc_cmd_t* p_cmd, uint8_t* p_code);
#endif
static uint32_t wiced_bt_avrc_ct_check_peer_features(wiced_bt_sdp_discovery_db_t* p_disc_db, uint16_t service_uuid, REMOTE_CONTROL_INFO* p_rc_peer_info);
static void wiced_bt_avrc_ct_register_for_notifications(rcc_device_t* rcc_dev, wiced_bt_avrc_metadata_rsp_t * p_rsp);
static void wiced_bt_avrc_ct_handle_list_player_app_values_rsp(rcc_device_t* rcc_dev, rcc_transaction_t* transaction, wiced_bt_avrc_rsp_t * p_rsp);
static wiced_result_t wiced_bt_avrc_ct_register_for_notification(rcc_device_t* prcc_dev, uint8_t event_id, rcc_transaction_t* ptransaction);

static void wiced_bt_avrc_ct_sdp_cback(uint16_t status);
static void wiced_bt_avrc_ct_metadata_event_cback(uint8_t handle, uint8_t label, wiced_bt_avrc_msg_t* p_meta);
static void wiced_bt_avrc_ct_pass_through_response_event_cback(uint8_t handle, uint8_t label, wiced_bt_avrc_hdr_t *p_hdr, wiced_bt_avrc_pass_thru_hdr_t *p_msg);
#if AVRC_ADV_CTRL_INCLUDED == TRUE
static void wiced_bt_avrc_ct_metadata_command_event_cback(uint8_t handle, uint8_t label, wiced_bt_avrc_msg_t* p_data);
#endif
static void wiced_bt_avrc_ct_metadata_response_event_cback(uint8_t handle, uint8_t label, wiced_bt_avrc_msg_t* p_data);

static void wiced_bt_avrc_ct_discovery_done(void);
static void wiced_bt_avrc_ct_handle_getcaps_rsp(rcc_device_t* rcc_dev, wiced_bt_avrc_metadata_rsp_t * p_rsp);
static void wiced_bt_avrc_ct_handle_notification_rsp(rcc_device_t* rcc_dev, uint8_t label, uint8_t code, wiced_bt_avrc_rsp_t * p_rsp);
static void wiced_bt_avrc_ct_forward_rsp(rcc_device_t* rcc_dev, uint8_t label, uint8_t code, wiced_bt_avrc_rsp_t* p_rsp);

static void wiced_bt_avrc_ct_handle_msg(wiced_bt_avrc_msg_t* p_msg);
static uint8_t wiced_bt_avrc_ct_operation_supported(uint8_t rc_id);
#if AVRC_ADV_CTRL_INCLUDED == TRUE
static wiced_bt_avrc_xmit_buf_t* wiced_bt_avrc_ct_receive_notification_registration(rcc_device_t* rcc_dev, uint8_t label, wiced_bt_avrc_cmd_t* p_cmd, wiced_bt_avrc_metadata_rsp_t* p_rsp, uint8_t* p_code);
#endif
typedef void (tAVRC_FIND_CBACK)(uint16_t status);
static uint16_t wiced_bt_avrc_ct_find_service(uint16_t service_uuid, wiced_bt_device_address_t bd_addr, RCC_SDP_DB_PARAMS* p_db, tAVRC_FIND_CBACK* p_cback);

void wiced_bt_avrc_msg_cback(wiced_bt_avrc_msg_t* p_msg);

extern void AVCT_Register(uint16_t mtu, uint16_t mtu_br, uint8_t sec_mask);
void bdcpy(wiced_bt_device_address_t a, const wiced_bt_device_address_t b);

static void wiced_bt_avrc_ct_rcc_display(void);

static void wiced_bt_avrc_ct_connection_open(uint8_t dev_idx, uint8_t role, wiced_bt_device_address_t bdaddr, uint8_t local_feature);
#ifdef __cplusplus
extern "C" {
#endif

#ifdef AVCT_MAP_FRAGMENTED_RESPONSE_ALLOCATION
    /******************************************************
     *  Fragment buffer Allocation
     ******************************************************/

    void* wiced_getbufpool(void* p_cb);
    typedef void (*wiced_buffer_free_cb)(void* p, char* from);
    void* wiced_create_pool(uint32_t buffer_size, uint32_t buffer_cnt, wiced_buffer_free_cb free_callback);

#define AVCT_FRAG_BUFFER_SIZE   (sizeof(wiced_bt_avrc_xmit_buf_t) + 2048)
#define MAX_AVCT_FRAG_BUFFERS   2

    static void* get_fragmented_buffer(uint16_t buf_size)
    {
        void* p_buffer = NULL;

        if (buf_size == AVCT_FRAG_BUFFER_SIZE)
        {
            p_buffer = wiced_getbufpool(rcc_cb.p_pool);
        }

        if (p_buffer == NULL)
            WICED_BT_TRACE("%s: FAILED!!! No buffers available\n", __FUNCTION__);

        return p_buffer;
    }

    static wiced_result_t create_avct_buffer_pool(void)
    {
        wiced_result_t result = WICED_OUT_OF_HEAP_SPACE;
        int i;

        WICED_BT_TRACE("%s: creating pool \n", __FUNCTION__);

        rcc_cb.p_pool = wiced_create_pool(AVCT_FRAG_BUFFER_SIZE, MAX_AVCT_FRAG_BUFFERS, NULL);
        if (rcc_cb.p_pool != NULL)
        {
            uint16_t status;
            tAVCT_GET_FRAG_BUFFER_ALLOC get_buf = get_fragmented_buffer;
            uint16_t buffer_size = AVCT_FRAG_BUFFER_SIZE;

            result = WICED_SUCCESS;
            WICED_BT_TRACE("%s: replacing allocation functions. buffer size: %d\n", __FUNCTION__, buffer_size);

            status = AVCT_SetFragAlloc(&get_buf, &buffer_size);
            if (status == AVCT_SUCCESS)
            {
                rcc_cb.orig_get_buf = get_buf;
                rcc_cb.orig_buffer_size = buffer_size;

                /* TODO: in future use we could cascade the request to the original functions */

            }
        }

        return result;
    }
#endif

#ifdef BTAVRCP_TRACE_DEBUG
    static const char* dump_event_name(uint8_t event)
    {
        switch ((int)event)
        {
            CASE_RETURN_STR(AVRC_OPEN_IND_EVT)
                CASE_RETURN_STR(AVRC_CLOSE_IND_EVT)
                CASE_RETURN_STR(AVRC_CONG_IND_EVT)
                CASE_RETURN_STR(AVRC_UNCONG_IND_EVT)
                CASE_RETURN_STR(AVRC_BROWSE_OPEN_IND_EVT)
                CASE_RETURN_STR(AVRC_BROWSE_CLOSE_IND_EVT)
                CASE_RETURN_STR(AVRC_BROWSE_CONG_IND_EVT)
                CASE_RETURN_STR(AVRC_BROWSE_UNCONG_IND_EVT)
                CASE_RETURN_STR(AVRC_CMD_TIMEOUT_EVT)
                CASE_RETURN_STR(AVRC_APP_BUFFER_TX_EVT)
        default:
            return ("Unknown Event");
        }
    }
#endif


    void wiced_avrc_send_or_enqueue(uint8_t handle, uint8_t label, uint8_t ctype, wiced_bt_avrc_xmit_buf_t* p_msgbuf)
    {
        uint16_t status;

        if ((status = wiced_bt_avrc_send_metadata_msg(handle, label, ctype, p_msgbuf)) != 0)
        {
            WICED_BTAVRCP_TRACE("wiced_avrc_send_or_enqueue - status: 0x%x  num_held: %d",
                status, rcc_cb.num_held_trans);

            if (rcc_cb.num_held_trans < MAX_QUEUED_TRANSACTIONS)
            {
                rcc_cb.xmit_queue[rcc_cb.num_held_trans].ctype = ctype;
                rcc_cb.xmit_queue[rcc_cb.num_held_trans].label = label;
                rcc_cb.xmit_queue[rcc_cb.num_held_trans].handle = handle;
                rcc_cb.xmit_queue[rcc_cb.num_held_trans].p_buf = p_msgbuf;

                rcc_cb.num_held_trans++;
            }
            else
                wiced_bt_free_buffer(p_msgbuf);
        }
    }

void wiced_avrc_send_next_metadata_msg()
{
    uint8_t                     ctype, handle, label;
    wiced_bt_avrc_xmit_buf_t* p_msgbuf;
    int                         xx;
    uint16_t                    status;

    if (rcc_cb.num_held_trans > 0)
    {
        ctype = rcc_cb.xmit_queue[0].ctype;
        label = rcc_cb.xmit_queue[0].label;
        handle = rcc_cb.xmit_queue[0].handle;
        p_msgbuf = rcc_cb.xmit_queue[0].p_buf;

        rcc_cb.num_held_trans--;

        // Move everything up one
        for (xx = 0; xx < rcc_cb.num_held_trans; xx++)
        {
            rcc_cb.xmit_queue[xx].ctype = rcc_cb.xmit_queue[xx + 1].ctype;
            rcc_cb.xmit_queue[xx].label = rcc_cb.xmit_queue[xx + 1].label;
            rcc_cb.xmit_queue[xx].handle = rcc_cb.xmit_queue[xx + 1].handle;
            rcc_cb.xmit_queue[xx].p_buf = rcc_cb.xmit_queue[xx + 1].p_buf;
        }

        if ((status = wiced_bt_avrc_send_metadata_msg(handle, label, ctype, p_msgbuf)) != 0)
        {
            WICED_BTAVRCP_TRACE("wiced_avrc_send_next_metadata_msg dropping - status: 0x%x  num_held: %d",
                status, rcc_cb.num_held_trans);

            wiced_bt_free_buffer(p_msgbuf);
        }
        else
        {
            WICED_BTAVRCP_TRACE("wiced_avrc_send_next_metadata_msg  num_held now: %d", rcc_cb.num_held_trans);
        }
    }
}


    wiced_bt_avrc_sts_t wiced_avrc_build_and_send_metadata_cmd(void* p_cmd, uint8_t ctype, uint8_t handle, uint8_t label)
    {
        wiced_bt_avrc_sts_t         avrc_status;
        wiced_bt_avrc_xmit_buf_t* p_cmdbuf;

        if ((p_cmdbuf = (wiced_bt_avrc_xmit_buf_t*)wiced_bt_get_buffer(DEFAULT_METADATA_CMD_SIZE + WICED_AVRC_XMIT_BUF_OVERHEAD)) == NULL)
            return (AVRC_STS_NO_RESOURCES);

        p_cmdbuf->buffer_size = DEFAULT_METADATA_CMD_SIZE;

        avrc_status = wiced_bt_avrc_bld_metadata_cmd((wiced_bt_avrc_metadata_cmd_t*)p_cmd, p_cmdbuf);

        if (avrc_status == AVRC_STS_NO_ERROR)
        {
            wiced_avrc_send_or_enqueue(handle, label, ctype, p_cmdbuf);
        }
        else
        {
            wiced_bt_free_buffer (p_cmdbuf);
        }

        return (avrc_status);
    }

    wiced_bt_avrc_sts_t wiced_avrc_build_and_send_browse_cmd(void *p_cmd, uint8_t ctype, uint8_t handle, uint8_t label)
    {
        wiced_bt_avrc_sts_t         avrc_status;
        wiced_bt_avrc_xmit_buf_t *p_cmdbuf;

        if ((p_cmdbuf = (wiced_bt_avrc_xmit_buf_t *)wiced_bt_get_buffer(DEFAULT_METADATA_CMD_SIZE + WICED_AVRC_XMIT_BUF_OVERHEAD)) == NULL)
            return (AVRC_STS_NO_RESOURCES);

        p_cmdbuf->buffer_size = DEFAULT_METADATA_CMD_SIZE;

        avrc_status = wiced_bt_avrc_bld_browse_command((wiced_bt_avrc_browse_cmd_t *)p_cmd, p_cmdbuf);

        if (avrc_status == AVRC_STS_NO_ERROR)
        {
            wiced_bt_avrc_send_browse_data(handle, label, AVRC_CMD,  p_cmdbuf);
        }

        return (avrc_status);
    }


wiced_bt_avrc_sts_t wiced_avrc_build_and_send_browse_rsp (uint8_t handle, uint8_t label, void *p_rsp, wiced_bt_avrc_xmit_buf_t **p_rspbuf)
      {
      wiced_bt_avrc_sts_t         avrc_status;

      if ((*p_rspbuf = (wiced_bt_avrc_xmit_buf_t *)wiced_bt_get_buffer (DEFAULT_METADATA_RSP_SIZE + WICED_AVRC_XMIT_BUF_OVERHEAD)) == NULL)
          return (AVRC_STS_NO_RESOURCES);

      (*p_rspbuf)->buffer_size = DEFAULT_METADATA_RSP_SIZE;

      avrc_status = wiced_bt_avrc_bld_browse_response((wiced_bt_avrc_browse_rsp_t *)p_rsp, *p_rspbuf);

      if (avrc_status != AVRC_STS_NO_ERROR)
      {
          wiced_bt_free_buffer (*p_rspbuf);
          *p_rspbuf = NULL;
      }

      if (avrc_status == AVRC_STS_NO_ERROR)
      {
          wiced_bt_avrc_send_browse_data(handle, label, AVRC_RSP, *p_rspbuf);
      }

      return (avrc_status);
  }

wiced_bt_avrc_sts_t wiced_bt_avrc_build_browse_rsp (void *p_rsp, wiced_bt_avrc_xmit_buf_t **p_rspbuf)
{
   wiced_bt_avrc_sts_t         avrc_status;

   if ((*p_rspbuf = (wiced_bt_avrc_xmit_buf_t *)wiced_bt_get_buffer (DEFAULT_METADATA_RSP_SIZE + WICED_AVRC_XMIT_BUF_OVERHEAD)) == NULL)
       return (AVRC_STS_NO_RESOURCES);

   (*p_rspbuf)->buffer_size = DEFAULT_METADATA_RSP_SIZE;

   avrc_status = wiced_bt_avrc_bld_browse_response((wiced_bt_avrc_browse_rsp_t *)p_rsp, *p_rspbuf);

   if (avrc_status != AVRC_STS_NO_ERROR)
   {
       wiced_bt_free_buffer (*p_rspbuf);
       *p_rspbuf = NULL;
   }

   return (avrc_status);
}


wiced_bt_avrc_sts_t wiced_avrc_build_metadata_rsp (void *p_rsp, wiced_bt_avrc_xmit_buf_t **p_rspbuf)
{
   wiced_bt_avrc_sts_t         avrc_status;

   if ((*p_rspbuf = (wiced_bt_avrc_xmit_buf_t *)wiced_bt_get_buffer (DEFAULT_METADATA_RSP_SIZE + WICED_AVRC_XMIT_BUF_OVERHEAD)) == NULL)
       return (AVRC_STS_NO_RESOURCES);

   (*p_rspbuf)->buffer_size = DEFAULT_METADATA_RSP_SIZE;

   avrc_status = wiced_bt_avrc_bld_metadata_response ((wiced_bt_avrc_metadata_rsp_t *)p_rsp, *p_rspbuf);

   if (avrc_status != AVRC_STS_NO_ERROR)
   {
       wiced_bt_free_buffer (*p_rspbuf);
       *p_rspbuf = NULL;
   }

   return (avrc_status);
}


    wiced_bt_avrc_sts_t wiced_avrc_build_and_send_metadata_rsp (void *p_rsp, uint8_t ctype, uint8_t handle, uint8_t label)
    {
        wiced_bt_avrc_sts_t         avrc_status;
        wiced_bt_avrc_xmit_buf_t    *p_rspbuf;

        if ((p_rspbuf = (wiced_bt_avrc_xmit_buf_t *)wiced_bt_get_buffer (DEFAULT_METADATA_RSP_SIZE + WICED_AVRC_XMIT_BUF_OVERHEAD)) == NULL)
            return (AVRC_STS_NO_RESOURCES);

        p_rspbuf->buffer_size = WICED_AVRC_XMIT_BUF_OVERHEAD;

        avrc_status = wiced_avrc_build_metadata_rsp (p_rsp, &p_rspbuf);

        if (avrc_status == AVRC_STS_NO_ERROR)
{
            wiced_avrc_send_or_enqueue (handle, label, ctype, p_rspbuf);
        }
        else
    {
            wiced_bt_free_buffer (p_rspbuf);
    }

        return (avrc_status);
}

    /******************************************************
     *               Callback implementations
     ******************************************************/
     /**
      *
      * Function         wiced_bt_avrc_ctrl_cback
      *
      *                  AVRC control callback function.
      *
      * @param[in]       handle      : Connection handle
      * @param[in]       event       : AVRC event (see @ref AVRC_EVT "AVRC events")
      * @param[in]       result      : Result code (see @ref AVRC_RESULT "AVRC result codes")
      * @param[in]       peer_addr   : Peer device address
      *
      * @return          Nothing
      */

    void wiced_bt_avrc_ctrl_cback(uint8_t handle, wiced_bt_avrc_ctrl_evt_t event, uint16_t result, wiced_bt_avrc_xmit_buf_t* p_buf, wiced_bt_device_address_t peer_addr)
    {
        rcc_device_t* prcc_dev = NULL;
        uint8_t cb_indx = INVALID_CB_INDEX;
        uint8_t free_cb_indx = INVALID_CB_INDEX;
#if AVRC_ADV_CTRL_INCLUDED == TRUE
        wiced_bt_avrc_ct_features_data_t features_data;
#endif
        uint8_t txn_indx;

        wiced_bt_avrc_ct_connection_state_t connection_state = REMOTE_CONTROL_DISCONNECTED;

        WICED_BTAVRCP_TRACE("%s event: %s[%d]\n", __FUNCTION__, dump_event_name(event), event);
        if (event == AVRC_APP_BUFFER_TX_EVT)
        {
            WICED_BTAVRCP_TRACE("%s handle[%d] buf: %p  sent_ok: %d\n", __FUNCTION__, handle, p_buf, result);
            wiced_bt_free_buffer(p_buf);
            return;
        }
        /* Locate the address in our cb list. If not there, this is remote connection */
        cb_indx = wiced_bt_avrc_ct_device_index_for_address(peer_addr, &free_cb_indx);
        if (cb_indx == INVALID_CB_INDEX)
        {
            if (free_cb_indx != INVALID_CB_INDEX)
            {
                /* Assume this is a remote connect attempt. Accept it if we have room. */
                prcc_dev = &rcc_cb.device[free_cb_indx];
            }
            else
            {
                WICED_BTAVRCP_TRACE(" [%s] ERROR : DEVICE RECORD NOT AVAILABLE/ALLOCATED \n", __FUNCTION__);
                return;
            }
        }
        else
        {
            prcc_dev = &rcc_cb.device[cb_indx];
        }

        WICED_BTAVRCP_TRACE("%s: [%p] \n", __FUNCTION__, prcc_dev);

        switch (event)
        {
        case AVRC_OPEN_IND_EVT:
            connection_state = REMOTE_CONTROL_CONNECTED;
            prcc_dev->state = RCC_STATE_CONNECTED;
            prcc_dev->rc_handle = handle;
            bdcpy(prcc_dev->peer_bda, peer_addr);
            if (rcc_cb.flags & RCC_FLAG_RC_API_OPEN_PENDING)
            {
                rcc_cb.flags &= ~RCC_FLAG_RC_API_OPEN_PENDING;
                wiced_bt_avrc_ct_send_getcaps_cmd(prcc_dev);
            }
            else
            {
                wiced_bt_avrc_ct_start_discovery(peer_addr);
            }
            if (rcc_cb.connection_cb)
            {
                (rcc_cb.connection_cb)(handle, peer_addr,
                    (wiced_result_t)result,
                    connection_state,
                    (uint32_t)prcc_dev->peer_features);
            }
#if AVRC_ADV_CTRL_INCLUDED == TRUE
            /* We don't know, yet, if the peer device supports Absolute volume */
            if (rcc_cb.features_callback)
            {
                /* Tell the app that it's not supported */
                features_data.abs_vol_supported.handle = handle;
                features_data.abs_vol_supported.supported = WICED_FALSE;
                rcc_cb.features_callback(WICED_BT_AVRC_CT_FEATURES_ABS_VOL_SUPPORTED,
                    &features_data);
            }
#endif
            break;

        case AVRC_CLOSE_IND_EVT:
            prcc_dev->state = RCC_STATE_IDLE;
            rcc_cb.remote_features = 0;
            rcc_cb.flags = 0;
#if AVRC_ADV_CTRL_INCLUDED == TRUE
            prcc_dev->abs_volume_supported = 0;
#endif
            /* Need to clean up the transaction list. */
            for (txn_indx = 0; txn_indx < MAX_TRANSACTIONS_PER_SESSION; txn_indx++)
            {
                memset(&prcc_dev->transaction[txn_indx], 0, sizeof(rcc_transaction_t));
                prcc_dev->transaction[txn_indx].label = txn_indx;
            }
            WICED_BTAVRCP_TRACE("%s:  Clear %d held_trans from Xmit queue\n", __FUNCTION__, rcc_cb.num_held_trans);

            for (int xx = 0; xx < rcc_cb.num_held_trans; xx++)
            {
                wiced_bt_avrc_xmit_buf_t *p_msgbuf = rcc_cb.xmit_queue[xx].p_buf;
                wiced_bt_free_buffer(p_msgbuf);

            }
            rcc_cb.num_held_trans = 0;
            if (rcc_cb.connection_cb)
            {
                (rcc_cb.connection_cb)(handle, peer_addr,
                    (wiced_result_t)result,
                    connection_state,
                    (uint32_t)prcc_dev->peer_features);
            }

            if (prcc_dev->role == AVRC_CONN_INITIATOR)
            {
                int i;

                for (i = 0; i < MAX_CONNECTED_RCC_DEVICES; i++)
                {
                    if (rcc_cb.rc_acp_handle[i] == handle)
                    {

                        if (rcc_cb.p_avct_buf[i] == NULL)
                            rcc_cb.p_avct_buf[i] = wiced_bt_get_buffer(MAX_AVCT_RCV_PKT_SIZE);

                        if (rcc_cb.p_avrc_buf[i] == NULL)
                            rcc_cb.p_avrc_buf[i] = wiced_bt_get_buffer(MAX_METADATA_RCV_MSG_SIZE);

                        wiced_bt_avrc_ct_connection_open(i,
                            AVRC_CONN_ACCEPTOR,
                            bd_addr_any0,
                            rcc_cb.local_features);

                        prcc_dev->role = AVRC_CONN_ACCEPTOR;
                        break;
                    }
                }
            }

            break;

        case AVRC_CMD_TIMEOUT_EVT:
            /* Release the transaction that timed out */
            wiced_bt_avrc_ct_release_transaction_for_device(prcc_dev, (uint8_t)result);
            break;

        case AVRC_CONG_IND_EVT:
#if 0
            WICED_BTAVRCP_TRACE("%s size: %d count %d free: %d utilization: %d", __FUNCTION__,
                GKI_get_pool_bufsize(HCI_ACL_POOL_ID),
                GKI_poolcount(HCI_ACL_POOL_ID),
                GKI_poolfreecount(HCI_ACL_POOL_ID),
                GKI_poolutilization(HCI_ACL_POOL_ID));
#endif
            break;
#if AVRC_ADV_CTRL_INCLUDED == TRUE
        case AVRC_BROWSE_OPEN_IND_EVT:
            if (rcc_cb.connection_cb)
            {
               int i;

                for (i = 0; i < MAX_CONNECTED_RCC_DEVICES; i++)
                {
                     if (rcc_cb.rc_acp_handle[i] == handle)
                    {
                        if(rcc_cb.p_browse_drb[i] != NULL)
                            wiced_bt_avrc_set_browse_drb(handle ,rcc_cb.p_browse_drb[i], remote_control_config.avrc_br_mtu, NULL);
                    }
                }

                (rcc_cb.connection_cb)(handle, peer_addr,
                    (wiced_result_t)result,
                    REMOTE_CONTROL_BROWSE_CONNECTED,
                    (uint32_t)prcc_dev->peer_features);
            }
            break;
        case AVRC_BROWSE_CLOSE_IND_EVT:
            if( rcc_cb.connection_cb )
            {
                (rcc_cb.connection_cb)( handle, peer_addr,
                                   (wiced_result_t)result,
                                   REMOTE_CONTROL_BROWSE_DISCONNECTED,
                                   (uint32_t)prcc_dev->peer_features);
            }
            break;
        case AVRC_BROWSE_CONG_IND_EVT:
            WICED_BT_TRACE("%s: Rx browsing Event peer-addr: <%B> \n", __FUNCTION__, peer_addr);
            break;
#endif
        default:
            WICED_BTAVRCP_TRACE("%s: **** unhandled event received event [%d] ****\n", __FUNCTION__, event);
            break;
        }

        WICED_BTAVRCP_TRACE("%s: Exit peer-addr: <%B> \n", __FUNCTION__, peer_addr);
    }

    /**
     *
     * Function         wiced_bt_avrc_msg_cback_t
     *
     *                  AVRC message callback function.  It is executed when AVCTP has
     *                  a message packet ready for the application.  The implementation of this
     *                  callback function must copy the wiced_bt_avrc_msg_t structure passed to it as it
     *                  is not guaranteed to remain after the callback function exits.
     *
     * @param[in]       handle  : Connection handle
     * @param[in]       label   : Message label
     * @param[in]       opcode  : Message opcode (see @ref AVRC_OPCODES "AVRC opcodes")
     * @param[in]       p_msg   : AVRC message
     *
     * @return          Nothing
     */
    void wiced_bt_avrc_msg_cback(wiced_bt_avrc_msg_t* p_msg)
    {
        WICED_BTAVRCP_TRACE("%s handle[%d] label[%d] opcode [%x]\n", __FUNCTION__, p_msg->handle, p_msg->label, p_msg->opcode);
        wiced_bt_avrc_ct_handle_msg(p_msg);
    }

    /*******************************************************************************
     *
     *                      Internal Functions
     *
     *******************************************************************************/

     /*******************************************************************************
     **
     ** Function         bdcpy
     **
     ** Description      Copy bd addr b to a.
     **
     **
     ** Returns          void
     **
     *******************************************************************************/
    void bdcpy(wiced_bt_device_address_t a, const wiced_bt_device_address_t b)
    {
        int i;

        for (i = BD_ADDR_LEN; i != 0; i--)
        {
            *a++ = *b++;
        }
    }

    /*******************************************************************************
    **
    ** Function         bdcmp
    **
    ** Description      Compare bd addr b to a.
    **
    **
    ** Returns          Zero if b==a, nonzero otherwise (like memcmp).
    **
    *******************************************************************************/
    int bdcmp(const wiced_bt_device_address_t a, const wiced_bt_device_address_t b)
    {
        int i;

        for (i = BD_ADDR_LEN; i != 0; i--)
        {
            if (*a++ != *b++)
            {
                return -1;
            }
        }
        return 0;
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_device_for_handle
    **
    ** Description    Retreives the device structure for the handle
    **
    ** Returns          rcc_device_t *
    *******************************************************************************/
    static rcc_device_t* wiced_bt_avrc_ct_device_for_handle(uint8_t rc_handle)
    {
        rcc_device_t* rcc_dev = NULL;
        uint8_t i;

        if (rc_handle == INVALID_CB_INDEX)
        {
            return NULL;
        }
        for (i = 0; i < MAX_CONNECTED_RCC_DEVICES; i++)
        {
            if (rcc_cb.device[i].rc_handle == rc_handle)
            {
                rcc_dev = &rcc_cb.device[i];
                break;
            }
        }

        return rcc_dev;
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_device_index_for_address
    **
    ** Description    retreives the device structure index for the requested bluetooth address
    **
    ** Returns          uint8_t - index of device structure
    *******************************************************************************/
    static uint8_t wiced_bt_avrc_ct_device_index_for_address(
        wiced_bt_device_address_t bd_addr,
        uint8_t* free_cb_indx)
    {
        // uint8_t cb_index = INVALID_CB_INDEX;
        uint8_t i;

        WICED_BTAVRCP_TRACE("wiced_bt_avrc_ct_device_index_for_address(%B)\n", bd_addr);
        if (NULL == bd_addr)
        {
            return INVALID_CB_INDEX;
        }
        wiced_bt_avrc_ct_rcc_display();

        /* Check if this device is already in use. */
        for (i = 0; i < MAX_CONNECTED_RCC_DEVICES; i++)
        {
            if (memcmp((void*)bd_addr,
                (void*)rcc_cb.device[i].peer_bda,
                sizeof(wiced_bt_device_address_t)) == 0)
            {
                if (rcc_cb.device[i].state == RCC_STATE_IDLE)
                {
                    rcc_cb.device[i].state = RCC_STATE_CONNECTING;
                }

                return i;
            }
        }

        if (free_cb_indx == NULL)
        {
            return INVALID_CB_INDEX;
        }
        else
        {
            *free_cb_indx = INVALID_CB_INDEX;
        }

        /* Find a free space. */
        for (i = 0; i < MAX_CONNECTED_RCC_DEVICES; i++)
        {
            if (rcc_cb.device[i].state == RCC_STATE_IDLE)
            {
                *free_cb_indx = i;
                rcc_cb.device[i].state = RCC_STATE_CONNECTING;
                memcpy((void*)rcc_cb.device[i].peer_bda,
                    (void*)bd_addr,
                    sizeof(wiced_bt_device_address_t));

                break;
            }
        }

        return INVALID_CB_INDEX;
    }

#ifdef TBD
    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_device_for_address
    **
    ** Description    retreives the device structure pointer for the requested bluetooth address
    **
    ** Returns          wiced_result_t
    *******************************************************************************/
    static wiced_result_t wiced_bt_avrc_ct_device_for_address(wiced_bt_device_address_t bd_addr, rcc_device_t** prcc_dev)
    {
        wiced_result_t result = WICED_NOT_FOUND;
        uint8_t cb_indx;

        WICED_BTAVRCP_TRACE("%s \n", __FUNCTION__);

        /* Find the remote device in the device list */
        cb_indx = wiced_bt_avrc_ct_device_index_for_address(bd_addr, NULL);

        WICED_BTAVRCP_TRACE("cb_indx: [%d] \n", cb_indx);
        if (cb_indx != INVALID_CB_INDEX)
        {
            *prcc_dev = &rcc_cb.device[cb_indx];
            result = WICED_SUCCESS;
        }

        return result;
    }
#endif
    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_get_transaction_by_label
    **
    ** Description    Will return a transaction for a particular device based on the label. If not inuse
    **                      will return an error.
    **
    ** Returns          bt_status_t
    *******************************************************************************/
    static rcc_transaction_t* wiced_bt_avrc_ct_get_transaction_by_label(
        rcc_device_t* rcc_dev,
        uint8_t label)
    {
        rcc_transaction_t* transaction = NULL;

        WICED_BTAVRCP_TRACE("%s device: 0x%x label: %d\n", __FUNCTION__, (unsigned long)rcc_dev, label);

        /* Determine if this is a valid label */
        if (label < MAX_TRANSACTIONS_PER_SESSION)
        {
            transaction = &rcc_dev->transaction[label];
            if (!transaction->in_use)
            {
                transaction = NULL;
            }
        }

        return transaction;
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_get_transaction_for_device
    **
    ** Description    Will acquire a transaction for a particular device for use. AVRCP spec stipulates that there
    **                      are only 16 outstanding trasactions/labels at any one time so there are only 16
    **                      transaction structures available.
    **
    ** Returns          bt_status_t
    *******************************************************************************/
    static wiced_result_t  wiced_bt_avrc_ct_get_transaction_for_device(rcc_device_t* rcc_dev, rcc_transaction_t** ptransaction)
    {
        wiced_result_t result = WICED_ERROR;
        int i;

        for (i = 0; i < MAX_TRANSACTIONS_PER_SESSION; i++)
        {
            if (!rcc_dev->transaction[i].in_use)
            {
                rcc_dev->transaction[i].in_use = TRUE;
                rcc_dev->transaction[i].handle = rcc_dev->rc_handle;

                *ptransaction = &rcc_dev->transaction[i];
                result = WICED_SUCCESS;

                WICED_BTAVRCP_TRACE("%s: label: %d\n", __FUNCTION__, rcc_dev->transaction[i].label);

                break;
            }
        }
        return result;
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_release_transaction_for_device
    **
    ** Description    Will release a transaction for a particular device for reuse.
    **
    ** Returns          bt_status_t
    *******************************************************************************/
    static void wiced_bt_avrc_ct_release_transaction_for_device(
        rcc_device_t* rcc_dev,
        uint8_t label)
    {
        rcc_transaction_t* transaction = wiced_bt_avrc_ct_get_transaction_by_label(rcc_dev, label);

        /* If the transaction is in use... */
        if (transaction != NULL)
        {
            WICED_BTAVRCP_TRACE("%s: label: %d\n", __FUNCTION__, transaction->label);

            memset(transaction, 0, sizeof(rcc_transaction_t));
            transaction->label = label;
        }
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_start_discovery
    **
    ** Description      start AVRC SDP discovery.
    **
    ** Returns          void
    **
    *******************************************************************************/
    void wiced_bt_avrc_ct_start_discovery(wiced_bt_device_address_t peer_addr)
    {
        RCC_SDP_DB_PARAMS db_params;
        uint16_t attr_list[] = { ATTR_ID_SERVICE_CLASS_ID_LIST, ATTR_ID_BT_PROFILE_DESC_LIST, ATTR_ID_SUPPORTED_FEATURES };

        WICED_BTAVRCP_TRACE("%s, features = %d\n", __FUNCTION__, (unsigned int)rcc_cb.remote_features);

        /* Start SDP for AVRCP if not already started */
        if ((rcc_cb.remote_features == 0) && !(rcc_cb.flags & RCC_FLAG_DISC_CB_IN_USE))
        {
            /* allocate discovery database */
            if (rcc_cb.p_disc_db == NULL)
            {
                rcc_cb.p_disc_db = (wiced_bt_sdp_discovery_db_t*)wiced_bt_get_buffer(RCC_DISC_BUF_SIZE);
            }
            else
            {
                WICED_BTAVRCP_TRACE("%s: WARNING!!! Discovery database already in use!!! \n", __FUNCTION__);
            }

            if (rcc_cb.p_disc_db != NULL)
            {
                /* set up parameters */
                db_params.db_len = RCC_DISC_BUF_SIZE;
                db_params.p_db = rcc_cb.p_disc_db;
                db_params.num_attr = sizeof_array(attr_list);
                db_params.p_attrs = attr_list;

                if (wiced_bt_avrc_ct_find_service(UUID_SERVCLASS_AV_REM_CTRL_TARGET, peer_addr, &db_params,
                    wiced_bt_avrc_ct_sdp_cback) == 0)
                {
                    rcc_cb.flags |= RCC_FLAG_DISC_CB_IN_USE;
                    bdcpy(rcc_cb.sdb_bd_addr, peer_addr);
                    WICED_BTAVRCP_TRACE("%s started\n", __FUNCTION__);
                }
                else
                {
                    wiced_bt_free_buffer(rcc_cb.p_disc_db);
                    rcc_cb.p_disc_db = NULL;

                    WICED_BTAVRCP_TRACE("%s, failed \n", __FUNCTION__);
                }
            }
            else
            {
                WICED_BTAVRCP_TRACE("%s: ERROR!!! Could not allocate the sdb database!!!\n", __FUNCTION__);
            }
        }

        else
        {
            WICED_BTAVRCP_TRACE("%s, not started\n", __FUNCTION__);
        }
    }

    /******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_find_service
    **
    ** Description      This function is called by the application to perform service
    **                  discovery and retrieve AVRCP SDP record information from a
    **                  peer device.
    **
    ** Returns          AVRC_SUCCESS if successful.
    **                  AVRC_BAD_PARAMS if discovery database parameters are invalid.
    **                  AVRC_NO_RESOURCES if there are not enough resources to
    **                                    perform the service search.
    **
    ******************************************************************************/
    uint16_t wiced_bt_avrc_ct_find_service(uint16_t service_uuid, wiced_bt_device_address_t bd_addr,
        RCC_SDP_DB_PARAMS* p_db, tAVRC_FIND_CBACK* p_cback)
    {
        wiced_bt_uuid_t uuid_list;

        wiced_bool_t     result = TRUE;
        uint16_t    a2d_attr_list[] = { ATTR_ID_SERVICE_CLASS_ID_LIST,
                                       ATTR_ID_BT_PROFILE_DESC_LIST,
                                       ATTR_ID_SUPPORTED_FEATURES };


        WICED_BTAVRCP_TRACE("%s enter... <%B> service_uuid: 0x%x\n", __FUNCTION__, bd_addr, service_uuid);

        if (((service_uuid != UUID_SERVCLASS_AV_REM_CTRL_TARGET) &&
            (service_uuid != UUID_SERVCLASS_AV_REMOTE_CONTROL)) ||
            (p_db == NULL) || (p_db->p_db == NULL) || (p_cback == NULL))
        {
            WICED_BTAVRCP_TRACE("%s AVRC_BAD_PARAM %d\n", __FUNCTION__, AVRC_BAD_PARAM);

            return AVRC_BAD_PARAM;
        }

        /* set up discovery database */
        uuid_list.len = LEN_UUID_16;
        uuid_list.uu.uuid16 = service_uuid;

        if (p_db->p_attrs == NULL || p_db->num_attr == 0)
        {
            p_db->p_attrs = a2d_attr_list;
            p_db->num_attr = sizeof_array(a2d_attr_list);
        }

        result = wiced_bt_sdp_init_discovery_db(p_db->p_db, p_db->db_len, 1, &uuid_list, p_db->num_attr, p_db->p_attrs);
        WICED_BTAVRCP_TRACE("%s result 1 %d\n", __FUNCTION__, result);

        if (result == TRUE)
        {
            /* perform service search */
            result = wiced_bt_sdp_service_search_attribute_request(bd_addr, p_db->p_db, p_cback);

            WICED_BTAVRCP_TRACE("%s result 2 %d\n", __FUNCTION__, result);
        }

        return (result ? AVRC_SUCCESS : AVRC_FAIL);
    }


    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_sdp_cback
    **
    ** Description      AVRCP service sdp callback.
    **
    ** Returns          void
    **
    *******************************************************************************/
    static void wiced_bt_avrc_ct_sdp_cback(uint16_t status)
    {
        uint8_t i;

        WICED_BTAVRCP_TRACE("%s status %d\n", __FUNCTION__, status);

        /* If SDP failed and SDP was triggered by API Open RC, notify upper layer */
        if ((status != WICED_BT_SDP_SUCCESS) && (rcc_cb.flags & RCC_FLAG_RC_API_OPEN_PENDING))
        {
            rcc_cb.flags &= ~(RCC_FLAG_RC_API_OPEN_PENDING | RCC_FLAG_DISC_CB_IN_USE);

            /* Free the database */
            wiced_bt_free_buffer(rcc_cb.p_disc_db);
            rcc_cb.p_disc_db = NULL;

            WICED_BTAVRCP_TRACE("%s SDP failed for AVRCP\n", __FUNCTION__);
            if (rcc_cb.connection_cb)
            {
                (rcc_cb.connection_cb)(0, rcc_cb.sdb_bd_addr, (wiced_result_t)status,
                    REMOTE_CONTROL_DISCONNECTED, 0);
            }
            rcc_cb.remote_features = 0;

            /* Create a AVCT connection as acceptor. */
            for (i = 0; i < MAX_CONNECTED_RCC_DEVICES; i++)
            {
                if (rcc_cb.device[i].rc_handle == INVALID_TRANSACTION_LABEL)
                {
                    wiced_bt_avrc_ct_connection_open(i,
                        AVRC_CONN_ACCEPTOR,
                        bd_addr_any0,
                        rcc_cb.local_features | RCC_FEAT_METADATA | RCC_FEAT_VENDOR);
                    break;
                }
            }
            return;
        }

        wiced_bt_avrc_ct_discovery_done();

    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_discovery_done
    **
    ** Description      Handle AVRCP service discovery results.  If matching
    **                  service found, open AVRCP connection.
    **
    ** Returns          void
    **
    *******************************************************************************/
    void wiced_bt_avrc_ct_discovery_done()
    {
        uint8_t cb_indx = INVALID_CB_INDEX;
        uint8_t free_cb_indx = INVALID_CB_INDEX;
        uint8_t  dev_inx = INVALID_CB_INDEX;

        rcc_device_t* prcc_dev = NULL;

        if (!(rcc_cb.flags & RCC_FLAG_DISC_CB_IN_USE))
            return;

        rcc_cb.flags &= ~RCC_FLAG_DISC_CB_IN_USE;

        /* Locate the address in our cb list. If not there, this is remote connection */
        cb_indx = wiced_bt_avrc_ct_device_index_for_address(rcc_cb.sdb_bd_addr, &free_cb_indx);
        if (cb_indx == INVALID_CB_INDEX)
        {
            if (free_cb_indx != INVALID_CB_INDEX)
            {
                /* Assume this is a remote connect attempt. Accept it if we have room. */
                prcc_dev = &rcc_cb.device[free_cb_indx];
                dev_inx = free_cb_indx;
            }
            else
            {
                WICED_BTAVRCP_TRACE("[%s] ERROR: NO SPACE FOR STORING %B DEVICE RECORD \n", __FUNCTION__, rcc_cb.sdb_bd_addr);
                wiced_bt_free_buffer(rcc_cb.p_disc_db);
                rcc_cb.p_disc_db = NULL;
                return;
            }
        }
        else
        {
            prcc_dev = &rcc_cb.device[cb_indx];
            dev_inx = cb_indx;
        }


        WICED_BTAVRCP_TRACE("%s rcc-dev [%x]\n", __FUNCTION__, (unsigned long)prcc_dev);

        /* find peer features */
        rcc_cb.remote_features |= wiced_bt_avrc_ct_check_peer_features(rcc_cb.p_disc_db,
            UUID_SERVCLASS_AV_REMOTE_CONTROL, &rcc_cb.peer_ct);

        prcc_dev->peer_ct_features = rcc_cb.peer_ct.features;
        prcc_dev->peer_ct_version  = rcc_cb.peer_ct.version;


        rcc_cb.remote_features |= wiced_bt_avrc_ct_check_peer_features(rcc_cb.p_disc_db,
            UUID_SERVCLASS_AV_REM_CTRL_TARGET, &rcc_cb.peer_tg);

        prcc_dev->peer_features = rcc_cb.peer_tg.features;
        prcc_dev->peer_version= rcc_cb.peer_tg.version;
        /* if we have no rc connection */
        if (rcc_cb.flags &= RCC_FLAG_RC_API_OPEN_PENDING)
        {
            /* SDP was performed in preparation for initiating AVRCP connection. Initiate AVRCP connection now */
            WICED_BTAVRCP_TRACE("%s matching services: Local: 0x%x Remote: 0x%x\n", __FUNCTION__,
                rcc_cb.local_features, rcc_cb.remote_features);

            /* if peer remote control service matches ours */
            if (((rcc_cb.local_features & REMOTE_CONTROL_FEATURE_CONTROLLER) &&
                (rcc_cb.remote_features & REMOTE_CONTROL_FEATURE_TARGET)) ||
                ((rcc_cb.local_features & REMOTE_CONTROL_FEATURE_TARGET) &&
                (rcc_cb.remote_features & REMOTE_CONTROL_FEATURE_CONTROLLER)))
            {
                if (rcc_cb.p_avct_buf[dev_inx] == NULL)
                    rcc_cb.p_avct_buf[dev_inx] = wiced_bt_get_buffer(MAX_AVCT_RCV_PKT_SIZE);

                if (rcc_cb.p_avrc_buf[dev_inx] == NULL)
                    rcc_cb.p_avrc_buf[dev_inx] = wiced_bt_get_buffer(MAX_METADATA_RCV_MSG_SIZE);

                wiced_bt_avrc_ct_connection_open(dev_inx,
                    AVRC_CONN_INITIATOR,
                    rcc_cb.sdb_bd_addr,
                    rcc_cb.local_features & (REMOTE_CONTROL_FEATURE_CONTROLLER | REMOTE_CONTROL_FEATURE_TARGET));

                prcc_dev->role = AVRC_CONN_INITIATOR;
            }
        }
        else
        {
            /* AVRC connection was initiated by peer. Notify app of peer avrc features */
            WICED_BTAVRCP_TRACE("%s connection was initiated by peer. Notify app of peer avrc features\n",
                __FUNCTION__);

            /* Now send the command to get the capabilities of the remote */
            wiced_bt_avrc_ct_send_getcaps_cmd(prcc_dev);
        }

        rcc_cb.remote_features = 0;
        /* Done with discovery information. Free the buffer */
        wiced_bt_free_buffer(rcc_cb.p_disc_db);
        rcc_cb.p_disc_db = NULL;
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_check_peer_features
    **
    ** Description      Get info from sdp database for specified service
    **                  (UUID_SERVCLASS_AV_REMOTE_CONTROL or UUID_SERVCLASS_AV_REM_CTRL_TARGET)
    **
    ** Returns          peer features mask
    **
    *******************************************************************************/
    uint32_t wiced_bt_avrc_ct_check_peer_features(wiced_bt_sdp_discovery_db_t* p_disc_db, uint16_t service_uuid, REMOTE_CONTROL_INFO* p_rc_peer_info)
    {
        uint32_t peer_features = 0;
        wiced_bt_sdp_discovery_record_t* p_rec = NULL;
        wiced_bt_sdp_discovery_attribute_t* p_attr;

        WICED_BTAVRCP_TRACE("%s service_uuid:0x%x\n", __FUNCTION__, service_uuid);

        /* Look for request UUID in database */
        p_rc_peer_info->version = 0;
        if ((p_rec = wiced_bt_sdp_find_service_in_db(p_disc_db, service_uuid, p_rec)) != NULL)
        {
            WICED_BTAVRCP_TRACE("%s p_rec: 0x%x\n", __FUNCTION__, p_rec);

            peer_features |= ((service_uuid == UUID_SERVCLASS_AV_REMOTE_CONTROL) ?
                REMOTE_CONTROL_FEATURE_CONTROLLER : REMOTE_CONTROL_FEATURE_TARGET);

            if ((wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_SERVICE_CLASS_ID_LIST)) != NULL)
            {
                WICED_BTAVRCP_TRACE("%s CLASS_ID_LIST\n", __FUNCTION__);
                if ((wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_BT_PROFILE_DESC_LIST)) != NULL)
                {
                    /* get profile version (if failure, version parameter is not updated) */
                    wiced_bt_sdp_find_profile_version_in_rec(p_rec, UUID_SERVCLASS_AV_REMOTE_CONTROL, &p_rc_peer_info->version);
                    WICED_BTAVRCP_TRACE("peer_rc_version 0x%x\n", (unsigned int)p_rc_peer_info->version);
                }

                if (p_rc_peer_info->version >= AVRC_REV_1_3)
                {
                    peer_features |= (RCC_FEAT_VENDOR | RCC_FEAT_METADATA);
                }

                if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_SUPPORTED_FEATURES)) != NULL)
                {
                    WICED_BTAVRCP_TRACE("%s SUPPORTED_FEATURES\n", __FUNCTION__);

                    p_rc_peer_info->features = p_attr->attr_value.v.u16;
                    WICED_BTAVRCP_TRACE("peer_rc_features: 0x%x\n",
                        (unsigned int)p_rc_peer_info->features);
                    if ((p_rc_peer_info->version >= AVRC_REV_1_4) &&
                        (p_rc_peer_info->features & AVRC_SUPF_CT_BROWSE))
                    {
                        peer_features |= RCC_FEAT_BROWSE;
                    }
                }
            }
        }

        WICED_BTAVRCP_TRACE("%s peer_features: 0x%x \n", __FUNCTION__, (unsigned int)peer_features);

        return (peer_features);
    }


    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_send_getcaps_cmd
    **
    ** Description      Send command to retreive the getcaps response.
    **
    ** Returns          void
    *******************************************************************************/
    static wiced_result_t wiced_bt_avrc_ct_send_getcaps_cmd(rcc_device_t* prcc_dev)
    {
        rcc_transaction_t* ptransaction = NULL;
        wiced_bt_avrc_metadata_cmd_t cmd;
        wiced_bt_avrc_sts_t avrc_status;
        wiced_result_t status = WICED_SUCCESS;

        WICED_BTAVRCP_TRACE("%s %p\n", __FUNCTION__, prcc_dev);

        /* Get a transaction label for the request */
        status = wiced_bt_avrc_ct_get_transaction_for_device(prcc_dev, &ptransaction);
        if (status != WICED_SUCCESS)
        {
            WICED_BTAVRCP_TRACE("%s Could not allocate transaction\n", __FUNCTION__);
            return status;
        }

        /* Build and send GetCapabilities command */
        cmd.metadata_hdr.pdu = AVRC_PDU_GET_CAPABILITIES;
        //cmd.status = AVRC_STS_NO_ERROR;
        cmd.u.get_caps.capability_id = AVRC_CAP_EVENTS_SUPPORTED;

        /* This call will allocate the packet. */
        avrc_status = wiced_avrc_build_and_send_metadata_cmd((wiced_bt_avrc_metadata_cmd_t*)&cmd, AVRC_CMD_STATUS, prcc_dev->rc_handle, ptransaction->label);
        if (avrc_status != AVRC_STS_NO_ERROR)
        {
            /* Release the acquired transaction */
            wiced_bt_avrc_ct_release_transaction_for_device(prcc_dev, ptransaction->label);
            status = WICED_ERROR;
        }

        WICED_BTAVRCP_TRACE("%s status[%d]\n", __FUNCTION__, status);
        return status;
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_handle_getcaps_rsp
    **
    ** Description      Handle the response to the getcaps command sent on open.
    **                  Call to register for reported supported events
    **
    ** Returns          void
    *******************************************************************************/
    static void wiced_bt_avrc_ct_handle_getcaps_rsp(
        rcc_device_t* rcc_dev,
        wiced_bt_avrc_metadata_rsp_t * p_rsp)
    {
        wiced_bt_avrc_metadata_get_caps_rsp_t * p_gcrsp = (wiced_bt_avrc_metadata_get_caps_rsp_t *)&p_rsp->u.get_caps;

        WICED_BTAVRCP_TRACE("[%s]: Enter... count [%d]\n", __FUNCTION__, p_gcrsp->count);

         /* Register for notifications for supported events */
         wiced_bt_avrc_ct_register_for_notifications(rcc_dev, p_rsp);

        /* Inform the app using this API that the initialization sequence is complete. */
        if (rcc_cb.connection_cb != NULL)
        {
            (rcc_cb.connection_cb)(rcc_dev->rc_handle, rcc_dev->peer_bda,
                WICED_SUCCESS,
                REMOTE_CONTROL_INITIALIZED,
                0);
        }
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_handle_notification_rsp
    **
    ** Description     When a notification response occurs this routine will convey the information from that
    **                      notification up to the service AND re-register for that notification if the message is a
    **                      CHANGED response.
    **
    ** Returns          void
    *******************************************************************************/
    static void wiced_bt_avrc_ct_handle_notification_rsp(
        rcc_device_t* rcc_dev,
        uint8_t label,
        uint8_t code,
        wiced_bt_avrc_rsp_t *p_rsp)
    {
        rcc_transaction_t* transaction = wiced_bt_avrc_ct_get_transaction_by_label(rcc_dev, label);
        wiced_bt_avrc_metadata_reg_notif_rsp_t * reg_notif = (wiced_bt_avrc_metadata_reg_notif_rsp_t *)&p_rsp->type.metadata.u.reg_notif;
        uint8_t event_id;

        WICED_BTAVRCP_TRACE("%s: Enter...Code: label: %d event_id [%d] code [%d]\n", __FUNCTION__, label, reg_notif->event_id, code);

        WICED_BTAVRCP_TRACE("%s: play status: %d \n", __FUNCTION__, reg_notif->param.play_status);

        /* If this is just a UID counter update handle it separately since the app
              doesn't really need to know about it.

              NOTE: If Cover Art (AVRCP 1.6) is ever implemented this will have to change.
        */
        if (reg_notif->event_id == AVRC_EVT_UIDS_CHANGE)
        {
            switch (code)
            {
            case AVRC_RSP_INTERIM:
                rcc_dev->last_UID_counter = reg_notif->param.uid_counter;
                break;

            case AVRC_RSP_CHANGED:
                rcc_dev->last_UID_counter = reg_notif->param.uid_counter;

            case AVRC_RSP_REJ:
                /* Resubmit the notification cmd if not disconnecting. Reuse label */
                wiced_bt_avrc_ct_register_for_notification(rcc_dev, reg_notif->event_id, transaction);
                break;

            default:
                /* Unknown. Log it and release the transaction */
                WICED_BTAVRCP_TRACE("%s: Unknown code: %d\n", __FUNCTION__, code);
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, label);
                break;
            }
        }
        else
        {
            /* Check for Reject responses, Interim responses and Changed responses. */
            switch (code)
            {
            case AVRC_RSP_INTERIM:
                if (reg_notif->event_id != AVRC_EVT_NOW_PLAYING_CHANGE &&
                    reg_notif->event_id != AVRC_EVT_AVAL_PLAYERS_CHANGE &&
                    reg_notif->event_id != AVRC_EVT_ADDR_PLAYER_CHANGE)
                {
                    if (reg_notif->event_id == AVRC_EVT_VOLUME_CHANGE) {
                        // As per the spec, consider only the lower 7 bits.
                        p_rsp->type.metadata.u.reg_notif.param.volume &= 0x7F;
                    }
                    /* Send event to service */
                    rcc_cb.rsp_cb(rcc_dev->rc_handle, p_rsp);
                }
                break;

            case AVRC_RSP_CHANGED:
            case AVRC_RSP_REJ:
                if (/*reg_notif->status == AVRC_STS_ADDR_PLAYER_CHG && */transaction->u.event_id != 0)
                {
                    WICED_BTAVRCP_TRACE("btif_rcc_hndl_notification_rsp event ID  %d\n", transaction->u.event_id);
                    event_id = transaction->u.event_id;
                }
                else
                    event_id = reg_notif->event_id;

                if(event_id == AVRC_EVT_VOLUME_CHANGE)
                {
                    // As per the spec, consider only the lower 7 bits.
                    p_rsp->type.metadata.u.reg_notif.param.volume &= 0x7F;
                }
                /* Send event to service */
                rcc_cb.rsp_cb(rcc_dev->rc_handle, p_rsp);

                /* Resubmit the notification cmd if not disconnecting. Reuse label */
                wiced_bt_avrc_ct_register_for_notification(rcc_dev, event_id, transaction);
                break;

            default:
                /* Unknown. Log it and release the transaction */
                WICED_BTAVRCP_TRACE("%s: Unknown code: %d\n", __FUNCTION__, code);
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, label);
                break;
            }
        }
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_register_for_notifications
    **
    ** Description    Will walk through the list of events acquired through the call to getcaps and
    **                      call the function to register each for notifications for each one.
    **
    ** Returns          void
    *******************************************************************************/
    static void wiced_bt_avrc_ct_register_for_notifications(
        rcc_device_t* rcc_dev,
        wiced_bt_avrc_metadata_rsp_t * p_rsp)
    {
        wiced_bt_avrc_metadata_get_caps_rsp_t * p_gcrsp = (wiced_bt_avrc_metadata_get_caps_rsp_t *)&p_rsp->u.get_caps;
        rcc_transaction_t* transaction = NULL;
        wiced_result_t result;
        uint8_t cap_indx;
        uint8_t event_id;

        /* For each of the event capabilities, register for notifications */
        for (cap_indx = 0; cap_indx < p_gcrsp->count; cap_indx++)
        {
            event_id = p_gcrsp->param.event_id[cap_indx];

            /* Make sure this is a valid capability before trying to register it.
                        Restrict this to AVRCP 1.3 and volume*/
            if (isValidCapability(event_id) && rcc_cb.supported_events[event_id])
            {
                WICED_BT_TRACE("%s event_id [%d]\n", __FUNCTION__, event_id);

                /* Get a transaction label for the request */
                result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
                if (result == WICED_SUCCESS)
                {
                    transaction->u.event_id = event_id;
                    wiced_bt_avrc_ct_register_for_notification(rcc_dev, event_id, transaction);
                }

                /* Determine if the event ID reflects capability to report player updates */
                if (event_id == AVRC_EVT_APP_SETTING_CHANGE)
                {
                    rcc_dev->app_event_enabled = TRUE;
                }
            }
            else
            {
                WICED_BTAVRCP_TRACE("%s event_id [%d] not registered valid:%d supported:%d",
                    __FUNCTION__, event_id,
                    isValidCapability(event_id), rcc_cb.supported_events[event_id]);
            }
        }
    }

#ifdef CT_HANDLE_PASSTHROUGH_COMMANDS
    void wiced_bt_avrc_ct_register_passthrough_event_callback(wiced_bt_avrc_ct_pt_evt_cback_t pt_evt_cb)
    {
        wiced_bt_avrc_ct_pt_evt_cback = pt_evt_cb;
    }
#endif

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_handle_msg
    **
    ** Description      Process an AVRCP message from the peer.
    **
    ** Returns          void
    **
    *******************************************************************************/
    void wiced_bt_avrc_ct_handle_msg(wiced_bt_avrc_msg_t* p_msg)
    {
        uint8_t evt = 0;
		uint8_t handle = p_msg->handle;
	    uint8_t label = p_msg->label;
	    uint8_t opcode = p_msg->opcode;
#if( AVRC_METADATA_INCLUDED == TRUE )
        wiced_bt_avrc_xmit_buf_t* p_pkt = NULL;
        uint8_t       ctype = 0;
        wiced_bt_avrc_metadata_rsp_t  rc_rsp;
        wiced_bt_avrc_hdr_t *hdr = NULL;
        wiced_bt_avrc_cmd_t *p_cmd = NULL;
        wiced_bt_avrc_rsp_t *p_res = NULL;
        uint32_t company_id = 0;
#endif
        if(p_msg->msg_type == AVRC_CMD)
        {
            p_cmd =  &p_msg->type.command;
            hdr = &p_msg->type.command.hdr;
            company_id = p_cmd->type.metadata.metadata_hdr.company_id;
        }
        else if (p_msg->msg_type == AVRC_RSP)
        {
            p_res =  &p_msg->type.response;
            hdr = &p_msg->type.response.hdr;
            company_id = p_res->type.metadata.metadata_hdr.company_id;
        }
        WICED_BTAVRCP_TRACE("%s opcode: %d handle [%d]\n", __FUNCTION__, opcode, p_msg->handle);
       /* if this is a pass thru command */
        if ((opcode == AVRC_OP_PASS_THRU) && hdr->ctype == AVRC_CMD_CTRL)
        {
            /* check if operation is supported */
            hdr->ctype = wiced_bt_avrc_ct_operation_supported(p_cmd->type.pass_thru.hdr.operation_id);

            /* send response */
            wiced_bt_avrc_send_passthrough_rsp(handle, label, hdr->ctype, &p_msg->type.command.type.pass_thru);

            /* set up for callback if supported */
            if (p_msg->type.command.hdr.ctype == AVRC_RSP_ACCEPT)
            {
                evt = RCC_REMOTE_CMD_EVT;
#ifdef CT_HANDLE_PASSTHROUGH_COMMANDS
                if (wiced_bt_avrc_ct_pt_evt_cback)
                    (wiced_bt_avrc_ct_pt_evt_cback)(handle, p_cmd->type.pass_thru.hdr.operation_id);
#endif
            }
        }
        /* else if this is a pass thru response */
        else if (opcode == AVRC_OP_PASS_THRU && hdr->ctype >= AVRC_RSP_NOT_IMPL)
        {
            /* set up for callback */
            evt = RCC_REMOTE_RSP_EVT;
        }

        /* else if this is a vendor specific command or response */
        if (opcode == AVRC_OP_VENDOR)
        {
            /* set up for callback */
            /* Check for metadata */
            WICED_BTAVRCP_TRACE("%s handling vendor OP commands local_features [%x] remote_features [%x] \n", __FUNCTION__,
                (unsigned int)rcc_cb.local_features, (unsigned int)rcc_cb.remote_features);

            if ((rcc_cb.local_features & RCC_FEAT_METADATA) && (company_id == AVRC_CO_METADATA))
            {
                if (((rcc_cb.local_features & REMOTE_CONTROL_FEATURE_TARGET) && hdr->ctype <= AVRC_CMD_GEN_INQ) ||
                    ((rcc_cb.local_features & REMOTE_CONTROL_FEATURE_CONTROLLER) && hdr->ctype >= AVRC_RSP_NOT_IMPL))
                {
                    if (hdr->ctype <= AVRC_CMD_GEN_INQ)
                    {
                        evt = wiced_bt_avrc_ct_process_meta_cmd(&rc_rsp, p_cmd, &ctype);
                    }
                    else
                    {
                        evt = RCC_META_MSG_EVT;
                    }
                    /* If we got a response, send any enqueued command */
                    if (hdr->ctype >= AVRC_RSP_NOT_IMPL)
                        wiced_avrc_send_next_metadata_msg();

                    WICED_BTAVRCP_TRACE("%s handling vendor OP commands evt [%d] handle [%d]\n", __FUNCTION__, evt, p_msg->handle);
                }
            }
            else
            {
                /* if configured to support vendor specific and it's a command */
                if ((rcc_cb.local_features & RCC_FEAT_VENDOR) &&
                    hdr->ctype <= AVRC_CMD_GEN_INQ)
                {
                    evt = RCC_VENDOR_CMD_EVT;
                }
                /* else if configured to support vendor specific and it's a response */
                else if ((rcc_cb.local_features & RCC_FEAT_VENDOR) &&
                    hdr->ctype >= AVRC_RSP_NOT_IMPL)
                {
                    evt = RCC_VENDOR_RSP_EVT;
                }
                /* else if not configured to support vendor specific and it's a command */
                else if (!(rcc_cb.local_features & RCC_FEAT_VENDOR) &&
                    hdr->ctype <= AVRC_CMD_GEN_INQ)
                {
                    /* reject it */
                    wiced_avrc_build_and_send_metadata_rsp(&p_msg->type.response, AVRC_RSP_NOT_IMPL, handle, label);
                }
            }
        }
#if AVRC_BROWSE_INCLUDED == TRUE
        else if (opcode == AVRC_OP_BROWSE)
        {
            /* set up for callback */
            evt = RCC_META_MSG_EVT;
        }
#endif /* AVCT_BROWSE_INCLUDED */

#if( AVRC_METADATA_INCLUDED == TRUE )
        WICED_BTAVRCP_TRACE("%s handling vendor OP evt [%d]\n", __FUNCTION__, evt);

        /* if already handled, send the response without sending the event to app */
        if (evt == 0)
        {
            if (p_pkt == NULL)
            {
                wiced_avrc_build_and_send_metadata_rsp (&rc_rsp, ctype, handle, label);

            }
        }
#endif

        /* call callback */
        if (evt != 0)
        {
            switch (evt)
            {
            case RCC_REMOTE_RSP_EVT: /* remote control response */
                wiced_bt_avrc_ct_pass_through_response_event_cback(handle, label, &p_msg->type.response.hdr, &p_msg->type.response.type.pass);
                break;
            case RCC_META_MSG_EVT: /* metadata command/response received */
                wiced_bt_avrc_ct_metadata_event_cback(handle, label, p_msg);
                break;
            case RCC_REMOTE_CMD_EVT: /* remote control command */
            case RCC_VENDOR_CMD_EVT: /* vendor dependent remote control command */
            case RCC_VENDOR_RSP_EVT: /* vendor dependent remote control response */
                break;
            default:
                WICED_BTAVRCP_TRACE("%s: Unhandled event=%d\n", __FUNCTION__, evt);
                break;
            }

        }
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_operation_supported
    **
    ** Description      Check if remote control operation is supported.
    **
    ** Returns          AVRC_RSP_ACCEPT of supported, AVRC_RSP_NOT_IMPL if not.
    **
    *******************************************************************************/
    static uint8_t wiced_bt_avrc_ct_operation_supported(uint8_t rc_id)
    {
        if (p_remote_control_config_id[rc_id >> 4] & (1 << (rc_id & 0x0F)))
        {
            return AVRC_RSP_ACCEPT;
        }
        else
        {
            return AVRC_RSP_NOT_IMPL;
        }
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_process_meta_cmd
    **
    ** Description      Process an AVRCP metadata command from the peer.
    **
    ** Returns          0              : handled internally
    **                  non-zero value : event to be passed to app for handling
    **
    *******************************************************************************/
    uint8_t wiced_bt_avrc_ct_process_meta_cmd(wiced_bt_avrc_metadata_rsp_t* p_rc_rsp, wiced_bt_avrc_cmd_t* p_msg, uint8_t* p_ctype)
    {
        uint8_t evt = RCC_META_MSG_EVT;
        wiced_bt_avrc_metadata_cmd_t* p_vendor = &p_msg->type.metadata;

        WICED_BTAVRCP_TRACE("[%s] : PDU : %x\n", __FUNCTION__, p_msg->type.metadata.metadata_hdr.pdu);
        p_rc_rsp->metadata_hdr.pdu = p_msg->type.metadata.metadata_hdr.pdu;
        *p_ctype = AVRC_RSP_REJ;

        if (!wiced_bt_avrc_is_valid_avc_type(p_msg->type.metadata.metadata_hdr.pdu, p_msg->hdr.ctype))
        {
            WICED_BTAVRCP_TRACE("Invalid pdu/ctype: 0x%x, %d\n", p_msg->type.metadata.metadata_hdr.pdu, p_msg->hdr.ctype);
            /* reject invalid message without reporting to app */
            evt = 0;
            p_rc_rsp->u.status = AVRC_STS_BAD_CMD;
        }
        else
        {
            switch (p_msg->type.metadata.metadata_hdr.pdu)
            {
            case AVRC_PDU_GET_CAPABILITIES:
                /* process GetCapabilities command without reporting the event to app */
                evt = 0;
                p_rc_rsp->u.get_caps.capability_id = p_vendor->u.get_caps.capability_id;

                WICED_BTAVRCP_TRACE("%s, GET_CAPABILITIES case u8 [%d]\n", __FUNCTION__, p_vendor->u.get_caps.capability_id);
                if (p_vendor->u.get_caps.capability_id == AVRC_CAP_COMPANY_ID)
                {
                    *p_ctype = AVRC_RSP_IMPL_STBL;
                    p_rc_rsp->u.get_caps.count = p_remote_control_config->num_co_ids;
                    memcpy(p_rc_rsp->u.get_caps.param.company_id, p_remote_control_config->p_meta_co_ids,
                        (p_remote_control_config->num_co_ids << 2));
                }
                else if (p_vendor->u.get_caps.capability_id == AVRC_CAP_EVENTS_SUPPORTED)
                {
                    *p_ctype = AVRC_RSP_IMPL_STBL;
                    WICED_BTAVRCP_TRACE("%s, GET_CAPABILITIES case num_evt_ids [%d] \n", __FUNCTION__,
                        p_remote_control_config->num_evt_ids);
                    p_rc_rsp->u.get_caps.count = p_remote_control_config->num_evt_ids;
                    memcpy(p_rc_rsp->u.get_caps.param.event_id, p_remote_control_config->p_meta_evt_ids,
                        p_remote_control_config->num_evt_ids);

                    WICED_BTAVRCP_TRACE("%s, GET_CAPABILITIES case event_id [%d] \n", __FUNCTION__, p_rc_rsp->u.get_caps.param.event_id[0]);
                }
                else
                {
                    WICED_BTAVRCP_TRACE("Invalid capability ID: 0x%x\n", p_vendor->u.get_caps.capability_id);
                    /* reject - unknown capability ID */
                    p_rc_rsp->u.status = AVRC_STS_BAD_PARAM;
                }
                break;
            }
        }
        return evt;
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_metadata_event_cback
    **
    ** Description      Metadata event callback
    **
    ** Returns          void
    *******************************************************************************/
    static void wiced_bt_avrc_ct_metadata_event_cback(uint8_t handle, uint8_t label, wiced_bt_avrc_msg_t* p_meta)
    {
        wiced_bt_avrc_hdr_t *hdr = NULL;
        if (p_meta->msg_type == AVRC_CMD)
        {
            hdr = &p_meta->type.command.hdr;

        }
        if (p_meta->msg_type == AVRC_RSP)
        {
            hdr = &p_meta->type.response.hdr;
        }
        if (p_meta->opcode == AVRC_OP_BROWSE)
        {
            if (p_meta->msg_type== AVRC_RSP)
            {
                wiced_bt_avrc_ct_metadata_response_event_cback(handle, label, p_meta);
            }
            else if (p_meta->msg_type == AVRC_CMD)
            {
                WICED_BTAVRCP_TRACE("%s Received Browse command! NOT HANDLED\n", __FUNCTION__);
            }
        }
        else if (hdr->ctype >= AVRC_RSP_NOT_IMPL)
        {
            wiced_bt_avrc_ct_metadata_response_event_cback(handle, label, p_meta);
        }
        else
        {

#if AVRC_ADV_CTRL_INCLUDED == TRUE
            wiced_bt_avrc_ct_metadata_command_event_cback(handle, label, p_meta);
#else
            WICED_BTAVRCP_TRACE("%s Received metadata command! NOT HANDLED\n", __FUNCTION__);
#endif
        }
    }

    /*******************************************************************************
    **
    ** Function         rcc_remote_rsp_evt_cback
    **
    ** Description      PassThru command event callback
    **
    ** Returns          void
    *******************************************************************************/
    static void wiced_bt_avrc_ct_pass_through_response_event_cback(uint8_t handle, uint8_t label, wiced_bt_avrc_hdr_t *p_hdr, wiced_bt_avrc_pass_thru_hdr_t * p_msg)
    {
        rcc_device_t* rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);

        /* Ensure the device is valid */
        if (rcc_dev != NULL)
        {
            WICED_BTAVRCP_TRACE("%s: Enter...Operation: %d Key State: %d\n",
                __FUNCTION__, p_hdr->ctype, p_msg->state);

            /* Release the transaction */
            wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, label);

            /* Inform app of completion. */
            rcc_cb.pt_rsp_cb(rcc_dev->rc_handle, p_hdr->ctype, p_msg);
        }
    }

wiced_bt_avrc_sts_t wiced_bt_avrc_build_rejected_rsp(wiced_bt_avrc_xmit_buf_t **p_rspbuf,
                                                        uint8_t pdu,
                                                        uint8_t status)
{
    uint8_t *p_data;

    if ((*p_rspbuf = (wiced_bt_avrc_xmit_buf_t *)wiced_bt_get_buffer(1 + WICED_AVRC_XMIT_BUF_OVERHEAD)) == NULL)
        return (AVRC_STS_NO_RESOURCES);

    (*p_rspbuf)->buffer_size = 1;
    p_data = (*p_rspbuf)->payload;

    WICED_BTAVRCP_TRACE("[%s]: status=%d ", __FUNCTION__, status);
    (*p_rspbuf)->meta_pdu_id = pdu;
    *p_data = status;
    (*p_rspbuf)->len_used = 1;
    return AVRC_STS_NO_ERROR;
}


#if AVRC_ADV_CTRL_INCLUDED == TRUE
    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_metadata_command_event_cback
    **
    ** Description     Metadata command event callback
    **
    ** Returns          void
    *******************************************************************************/
    static void wiced_bt_avrc_ct_metadata_command_event_cback(uint8_t handle, uint8_t label, wiced_bt_avrc_msg_t* p_data)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_device_t* rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);

        /* Get the device pointer */
        if (rcc_dev == NULL)
        {
            WICED_BTAVRCP_TRACE("%s: Invalid device \n ", __FUNCTION__);
            result = WICED_ERROR;
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_sts_t avrc_status =  AVRC_STS_NO_ERROR;
            wiced_bt_avrc_metadata_rsp_t avrc_rsp;
            wiced_bt_avrc_xmit_buf_t* p_rsp_pkt = NULL;
            uint8_t rsp_code = AVRC_RSP_REJ;

            /* Use the AVRC utilities to parse the metadata response
             * Note that there is no need for a scratch buffer for the PDUs
             */

            WICED_BTAVRCP_TRACE("%s: PDU: %d Parse status: %d\n", __FUNCTION__, p_data->type.command.type.metadata.metadata_hdr.pdu, avrc_status);

            avrc_rsp.metadata_hdr.pdu = p_data->type.command.type.metadata.metadata_hdr.pdu;

            if (avrc_status == AVRC_STS_NO_ERROR)
            {
                switch (p_data->type.command.type.metadata.metadata_hdr.pdu)
                {
                case AVRC_PDU_SET_ABSOLUTE_VOLUME:
                    p_rsp_pkt = wiced_bt_avrc_ct_set_absolute_volume_cmd(rcc_dev, label, &p_data->type.command, &rsp_code);
                    break;

                case AVRC_PDU_REGISTER_NOTIFICATION:
                    p_rsp_pkt = wiced_bt_avrc_ct_receive_notification_registration(rcc_dev, label, &p_data->type.command, &avrc_rsp, &rsp_code);
                    break;

                default:
                    /* If an invalid value reject the request directly */
                    avrc_rsp.metadata_hdr.pdu = p_data->type.command.type.metadata.metadata_hdr.pdu;
                    avrc_rsp.u.status = AVRC_STS_BAD_CMD;

                    wiced_avrc_build_metadata_rsp (&avrc_rsp, &p_rsp_pkt);
                    break;
                }
            }
            else
            {
                avrc_rsp.u.status = avrc_status;
                if (p_data->type.command.hdr.opcode == AVRC_OP_VENDOR && avrc_status == AVRC_STS_BAD_CMD)
                {
                    avrc_rsp.metadata_hdr.pdu = AVRC_PDU_GENERAL_REJECT;
                }
                wiced_bt_avrc_build_rejected_rsp (&p_rsp_pkt , AVRC_PDU_GENERAL_REJECT, avrc_status);

            }

            if (rsp_code == AVRC_RSP_NOT_IMPL)
            {

#if AVRC_BROWSE_INCLUDED == TRUE
                if (p_data->opcode == AVRC_OP_BROWSE)
                {
                    /* use general reject */
                    wiced_bt_avrc_build_rejected_rsp(&p_rsp_pkt, AVRC_PDU_GENERAL_REJECT, AVRC_STS_INTERNAL_ERR);
                }
                else
#endif
                {
                    return;
                }
            }

            if (p_rsp_pkt)
            {
                wiced_bt_avrc_send_metadata_msg(handle, label, rsp_code, p_rsp_pkt);
            }
        }
    }


    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_set_absolute_volume_cmd
    **
    ** Description      Convey the request to change absolute volume up to service
    **
    ** Returns          void
    *******************************************************************************/
    static wiced_bt_avrc_xmit_buf_t* wiced_bt_avrc_ct_set_absolute_volume_cmd(
        rcc_device_t* rcc_dev, uint8_t label, wiced_bt_avrc_cmd_t* p_cmd, uint8_t* p_code)
    {
        uint8_t volume = p_cmd->type.metadata.u.volume;

        /* response packet to be sent */
        wiced_bt_avrc_xmit_buf_t* p_rsp_pkt = NULL;
        wiced_bt_avrc_metadata_rsp_t p_rsp;
        wiced_bt_avrc_sts_t aStatus;
        WICED_BTAVRCP_TRACE("%s: Enter... label: %d volume: %d\n", __FUNCTION__, label, volume);

        /* Peg volume request if set past maximum */
        if (volume > AVRC_MAX_VOLUME)
        {
            volume = AVRC_MAX_VOLUME;
        }

        /* Update the volume cache so this value does not get resent on the following update. */
        rcc_dev->current_volume = volume;

        /* Callback to service with play status */
        rcc_cb.cmd_cb(rcc_dev->rc_handle, &p_cmd->type.metadata);
        *p_code = AVRC_RSP_ACCEPT;

        /* filling the response packet */
        p_rsp.u.volume = rcc_dev->current_volume;
        p_rsp.metadata_hdr.pdu = p_cmd->type.metadata.metadata_hdr.pdu;

        /* build response packet */
        aStatus = wiced_avrc_build_metadata_rsp (&p_rsp, &p_rsp_pkt);
        if (aStatus != AVRC_STS_NO_ERROR)
        {
            WICED_BTAVRCP_TRACE(" %s: Failed to create response packet: %d\n", __func__, aStatus);
        }

        /* send response packet */
        return p_rsp_pkt;
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_receive_notification_registration
    **
    ** Description     Handle the request to register for notification
    **
    ** Returns          void
    *******************************************************************************/
    static wiced_bt_avrc_xmit_buf_t* wiced_bt_avrc_ct_receive_notification_registration(
        rcc_device_t* rcc_dev, uint8_t label, wiced_bt_avrc_cmd_t* p_cmd, wiced_bt_avrc_metadata_rsp_t * p_rsp, uint8_t* p_code)
    {
        wiced_bt_avrc_xmit_buf_t* p_rsp_pkt = NULL;
        *p_code = AVRC_RSP_REJ;
        wiced_bt_avrc_ct_features_data_t features_data;

        WICED_BTAVRCP_TRACE("%s: Enter... label: %d event_id: %d\n",
            __FUNCTION__, label, p_cmd->type.metadata.u.reg_notif.event_id);

        p_rsp->u.reg_notif.event_id = p_cmd->type.metadata.u.reg_notif.event_id;

        if (p_cmd->type.metadata.u.reg_notif.event_id == AVRC_EVT_VOLUME_CHANGE)
        {
            wiced_bt_avrc_sts_t aStatus;

            /* Respond initially with the interim value */
            *p_code = AVRC_RSP_INTERIM;
            p_rsp->u.reg_notif.param.volume = rcc_dev->current_volume;

            /* Store the label for the final returned value */
            rcc_dev->abs_volume_reg_label = label;

            /* Build the response packet */
             aStatus = wiced_avrc_build_metadata_rsp(p_rsp, &p_rsp_pkt);
            if (aStatus != AVRC_STS_NO_ERROR)
            {
                WICED_BTAVRCP_TRACE("%s: Failed to create response packet: %d\n", __FUNCTION__, aStatus);
            }

            /* Tell the application that the peer device supports Absolute volume */
            if ((rcc_dev->abs_volume_supported == 0) &&
                (rcc_cb.features_callback))
            {
                WICED_BTAVRCP_TRACE("%s: Absolute Volume Supported handle:%d\n",
                    __FUNCTION__, rcc_dev->rc_handle);
                /* We will sent this event only once (first time the peer register for this event) */
                rcc_dev->abs_volume_supported = 1;
                features_data.abs_vol_supported.handle = rcc_dev->rc_handle;
                features_data.abs_vol_supported.supported = WICED_TRUE;
                rcc_cb.features_callback(WICED_BT_AVRC_CT_FEATURES_ABS_VOL_SUPPORTED,
                    &features_data);
            }
        }
        else
        {
            *p_code = AVRC_RSP_NOT_IMPL;
        }
        return p_rsp_pkt;

    }
#endif

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_metadata_response_event_cback
    **
    ** Description      Metadata response event callback
    **
    ** Returns          void
    *******************************************************************************/
    static void wiced_bt_avrc_ct_metadata_response_event_cback(uint8_t handle, uint8_t label, wiced_bt_avrc_msg_t* p_data)
    {
        wiced_result_t result = WICED_SUCCESS;

        rcc_device_t* rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        rcc_transaction_t* transaction = NULL;
        uint8_t  pdu;

        WICED_BTAVRCP_TRACE("%s: Enter handle; %d label: %d PDU: 0x%x\n",
            __FUNCTION__, handle, label, p_data->type.response.type.metadata.metadata_hdr.pdu);

        /* Only handle responses */

        if (p_data->opcode == AVRC_OP_BROWSE)
        {
            pdu = p_data->type.response.type.browse_rsp.pdu_id;
            p_data->type.response.hdr.opcode = p_data->opcode;
            if (p_data->type.response.hdr.ctype == AVRC_RSP)
                WICED_BTAVRCP_TRACE("%s browse response handler not a response\n", __FUNCTION__);
        }
        else
        {
            pdu = p_data->type.response.type.metadata.metadata_hdr.pdu;
        }

        /* Get the device pointer */
        if (rcc_dev == NULL)
        {
            WICED_BTAVRCP_TRACE("%s Invalid device ptr\n", __FUNCTION__);
            result = WICED_ERROR;
        }

        transaction = wiced_bt_avrc_ct_get_transaction_by_label(rcc_dev, label);
        if (transaction == NULL)
        {
            WICED_BTAVRCP_TRACE("%s Invalid transaction label [%d]\n", __FUNCTION__, label);
            result = WICED_ERROR;
        }

        if (result == WICED_SUCCESS)
        {
           // if (scratch_buffer != NULL)
            {
                /* Use the AVRC utilities to parse the metadata response */
               // wiced_bt_avrc_parse_response(p_data, &avrc_rsp, scratch_buffer, (uint16_t)scratch_sz);

                WICED_BTAVRCP_TRACE("%s: PDU: 0x%x \n", __FUNCTION__,
                    pdu);

                switch (pdu)
                {
                case AVRC_PDU_GET_CAPABILITIES:
                    wiced_bt_avrc_ct_handle_getcaps_rsp(rcc_dev, &p_data->type.response.type.metadata);
                    break;

                case AVRC_PDU_REGISTER_NOTIFICATION:
                    wiced_bt_avrc_ct_handle_notification_rsp(rcc_dev, label, p_data->type.response.hdr.ctype, &p_data->type.response);
                    break;

                case AVRC_PDU_GET_PLAY_STATUS:
                case AVRC_PDU_GET_ELEMENT_ATTR:
                case AVRC_PDU_LIST_PLAYER_APP_ATTR:
                case AVRC_PDU_GET_CUR_PLAYER_APP_VALUE:
                    wiced_bt_avrc_ct_forward_rsp(rcc_dev, label, p_data->type.response.hdr.ctype, &p_data->type.response);
                    break;

                case AVRC_PDU_LIST_PLAYER_APP_VALUES:
                    wiced_bt_avrc_ct_handle_list_player_app_values_rsp(rcc_dev, transaction, &p_data->type.response);
                    break;

                case AVRC_PDU_GET_PLAYER_APP_ATTR_TEXT:
                case AVRC_PDU_GET_PLAYER_APP_VALUE_TEXT:
                case AVRC_PDU_SET_ADDRESSED_PLAYER:
                case AVRC_PDU_SET_BROWSED_PLAYER:
                case AVRC_PDU_GET_FOLDER_ITEMS:
                case AVRC_PDU_CHANGE_PATH:
                case AVRC_PDU_GET_ITEM_ATTRIBUTES:
                case AVRC_PDU_PLAY_ITEM:
                case AVRC_PDU_SEARCH:
                case AVRC_PDU_ADD_TO_NOW_PLAYING:
                case AVRC_PDU_SET_PLAYER_APP_VALUE:
                case AVRC_PDU_INFORM_DISPLAY_CHARSET:
                case AVRC_PDU_INFORM_BATTERY_STAT_OF_CT:
                case AVRC_PDU_GET_TOTAL_NUM_OF_ITEMS:
                    wiced_bt_avrc_ct_forward_rsp(rcc_dev, label, p_data->type.response.hdr.ctype, &p_data->type.response);
                    break;

                default:
                    break;
                }

#ifdef AVCT_MAP_FRAGMENTED_RESPONSE_ALLOCATION
                wiced_bt_free_buffer(scratch_buffer);
#endif
            }

            if ((p_data->type.response.type.metadata.metadata_hdr.pdu != AVRC_PDU_REGISTER_NOTIFICATION))
            {
                /* Release the transaction */
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, label);
            }
        }
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_forward_rsp
    **
    ** Description     Convey the response for an avrc request to the servicing app
    **
    ** Returns          void
    *******************************************************************************/
    static void wiced_bt_avrc_ct_forward_rsp(rcc_device_t* rcc_dev, uint8_t label, uint8_t code, wiced_bt_avrc_rsp_t* p_rsp)
    {
        WICED_BTAVRCP_TRACE("%s: In... label: %d \n", __FUNCTION__, label);

        /* Callback to service with play status */
        rcc_cb.rsp_cb(rcc_dev->rc_handle, p_rsp);
        return;
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_handle_list_player_app_values_rsp
    **
    ** Description      Convey the response for list player app values per attribute up to
    **                  the service
    **
    ** Returns          void
    *******************************************************************************/
    static void wiced_bt_avrc_ct_handle_list_player_app_values_rsp(rcc_device_t* rcc_dev, rcc_transaction_t* transaction, wiced_bt_avrc_rsp_t* p_rsp)
    {
        WICED_BTAVRCP_TRACE("%s: In...attribute: %d\n", __FUNCTION__,  transaction->u.attribute_id);

        p_rsp->hdr.opcode = transaction->u.attribute_id;

        /* Callback to service with attribute list */
        rcc_cb.rsp_cb(rcc_dev->rc_handle, p_rsp);
        return;
    }

    /*******************************************************************************
    **
    ** Function         wiced_bt_avrc_ct_register_for_notification
    **
    ** Description    Will register/reregister for notification from the specified event
    **
    ** Returns          bt_status_t
    *******************************************************************************/
    static wiced_result_t wiced_bt_avrc_ct_register_for_notification(rcc_device_t* prcc_dev, uint8_t event_id, rcc_transaction_t* ptransaction)
    {
        wiced_bt_avrc_metadata_cmd_t cmd;
        wiced_bt_avrc_sts_t avrc_status;
        wiced_result_t status = WICED_SUCCESS;

        /* For each of the event capabilities, register for notifications */
        WICED_BTAVRCP_TRACE("%s event_id [%d] label [%d] rc_handle[%d]\n", __FUNCTION__, event_id, ptransaction->label, prcc_dev->rc_handle);

        /* Build and send Register Notification command */
        cmd.metadata_hdr.pdu = AVRC_PDU_REGISTER_NOTIFICATION;
       // cmd.status = AVRC_STS_NO_ERROR;
        cmd.u.reg_notif.event_id = event_id;

        /* Playback interval needs to be set when playback pos change event set
                Otherwise it is ignored so just setting it is quicker than checking  the PDU */
        cmd.u.reg_notif.playback_interval = PLAYBACK_POSITION_CHANGE_SEC;

        /* This call will allocate the packet. */
        avrc_status = wiced_avrc_build_and_send_metadata_cmd((wiced_bt_avrc_metadata_cmd_t*)&cmd, AVRC_CMD_NOTIF, prcc_dev->rc_handle, ptransaction->label);
        if (avrc_status != AVRC_STS_NO_ERROR)
        {
            wiced_bt_avrc_ct_release_transaction_for_device(prcc_dev, ptransaction->label);
            status = WICED_ERROR;
        }

        return status;
    }


    /****************************************************************************/
    /**
     * AVRC remote control functions
     *
     * @addtogroup  wicedbt_RemoteControl       Remote control
     *
     * @{
     */

     /****************************************************************************/

     /**
      * Function         wiced_bt_avrc_ct_init
      *
      *                  Initialize the AVRC controller and start listening for incoming connections
      *
      * @param[in]       local_features      : Local supported features mask
      *                                        Combination of wiced_bt_avrc_ct_features_t
      * @param[in]       p_connection_cback  : Callback for connection state
      * @param[in]       p_rsp_cb            : Callback from peer device in response to AVRCP commands
      * @param[in]       p_cmd_cb            : Callback when peer device sends AVRCP commands
      *
      * @return          wiced_result_t
      *
      */
    wiced_result_t wiced_bt_avrc_ct_init(uint32_t local_features,
        uint8_t* supported_events,
        wiced_bt_avrc_ct_connection_state_cback_t p_connection_cb,
        wiced_bt_avrc_ct_cmd_cback_t p_cmd_cb,
        wiced_bt_avrc_ct_rsp_cback_t p_rsp_cb,
        wiced_bt_avrc_ct_pt_rsp_cback_t p_ptrsp_cb)
    {
        uint8_t dev_indx;
        uint8_t txn_indx;
        wiced_result_t result = WICED_SUCCESS;

        WICED_BTAVRCP_TRACE("%s: [%d] \n", __FUNCTION__, (int)local_features);

        if (rcc_cb.is_initialized == WICED_TRUE)
        {
            WICED_BTAVRCP_TRACE("%s AVRC controller already initialized\n", __FUNCTION__);
            return WICED_ERROR;
        }

       memset(&rcc_cb, 0, sizeof(rcc_cb));

        rcc_cb.supported_events = supported_events;
        rcc_cb.is_initialized = WICED_TRUE;

       /* Initialize individual device control blocks */
        for (dev_indx = 0; dev_indx < MAX_CONNECTED_RCC_DEVICES; dev_indx++)
        {
            rcc_device_t* rcc_dev = &rcc_cb.device[dev_indx];

            rcc_dev->state = RCC_STATE_IDLE;
            rcc_dev->rc_handle = INVALID_TRANSACTION_LABEL;
            rcc_dev->current_volume = 65;
            rcc_dev->role = AVRC_CONN_ACCEPTOR;

#if AVRC_ADV_CTRL_INCLUDED == TRUE
            rcc_dev->abs_volume_reg_label = INVALID_TRANSACTION_LABEL;
            /* Suppose by default that the peer does not support Absolute Volume */
            rcc_dev->abs_volume_supported = 0;
#endif

            for (txn_indx = 0; txn_indx < MAX_TRANSACTIONS_PER_SESSION; txn_indx++)
            {
                memset(&rcc_dev->transaction[txn_indx], 0, sizeof(rcc_transaction_t));
                rcc_dev->transaction[txn_indx].label = txn_indx;
            }

            if ((rcc_cb.rc_acp_handle[dev_indx] == 0) && (local_features & (REMOTE_CONTROL_FEATURE_CONTROLLER | REMOTE_CONTROL_FEATURE_TARGET)))
            {
                if (rcc_cb.p_avct_buf[dev_indx] == NULL)
                    rcc_cb.p_avct_buf[dev_indx] = wiced_bt_get_buffer(MAX_AVCT_RCV_PKT_SIZE);

                if (rcc_cb.p_avrc_buf[dev_indx] == NULL)
                    rcc_cb.p_avrc_buf[dev_indx] = wiced_bt_get_buffer(MAX_METADATA_RCV_MSG_SIZE);

                wiced_bt_avrc_ct_connection_open(dev_indx,
                    AVRC_CONN_ACCEPTOR,
                    bd_addr_any0,
                    local_features | RCC_FEAT_METADATA | RCC_FEAT_VENDOR);
            }
        }


        rcc_cb.connection_cb = p_connection_cb;
        rcc_cb.cmd_cb = p_cmd_cb;
        rcc_cb.rsp_cb = p_rsp_cb;
        rcc_cb.pt_rsp_cb = p_ptrsp_cb;
        rcc_cb.local_features = local_features | RCC_FEAT_METADATA | RCC_FEAT_VENDOR;
        rcc_cb.p_disc_db = NULL;
        rcc_cb.flags = 0;

        /* Setup the AVCT fragmented buffer handlers. */
#ifdef AVCT_MAP_FRAGMENTED_RESPONSE_ALLOCATION
        result = create_avct_buffer_pool();
#endif

        return result;
    }

#if AVRC_ADV_CTRL_INCLUDED == TRUE
    /**
     * Function         wiced_bt_avrc_ct_features_register
     *
     *                  Register for AVRC Feature events.
     *                  This, optional, function must be called after wiced_bt_avrc_ct_init
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_features_register(
        wiced_bt_avrc_ct_features_cback_t features_callback)
    {
        rcc_cb.features_callback = features_callback;

        return WICED_BT_SUCCESS;
    }
#endif

    /**
     * Function         wiced_bt_avrc_ct_deinit
     *
     *                  Cleanup the AVRC controller and stop listening for incoming connections
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_deinit(void)
    {
        uint8_t dev_indx;

        if (rcc_cb.is_initialized == WICED_FALSE)
        {
            WICED_BTAVRCP_TRACE("%s AVRCP controller already cleaned-up\n", __FUNCTION__);
            return WICED_ERROR;
        }

        for (dev_indx = 0; dev_indx < MAX_CONNECTED_RCC_DEVICES; dev_indx++)
        {
            rcc_device_t* rcc_dev = &rcc_cb.device[dev_indx];

            if (rcc_cb.p_avct_buf[dev_indx] != NULL)
            {
                wiced_bt_free_buffer(rcc_cb.p_avct_buf[dev_indx]);
            }

            if (rcc_cb.p_avrc_buf[dev_indx] != NULL)
            {
                wiced_bt_free_buffer(rcc_cb.p_avrc_buf[dev_indx]);
             }

            if (rcc_cb.p_browse_drb[dev_indx] != NULL)
            {
                    wiced_bt_free_buffer(rcc_cb.p_browse_drb[dev_indx]);
            }
            for (int xx = 0; xx < rcc_cb.num_held_trans; xx++)
            {
                wiced_bt_avrc_xmit_buf_t *p_msgbuf = rcc_cb.xmit_queue[xx].p_buf;
                wiced_bt_free_buffer(p_msgbuf);
            }

            if (rcc_dev->rc_handle != INVALID_TRANSACTION_LABEL)
            {
                WICED_BTAVRCP_TRACE("%s handle: %d\n", __FUNCTION__, rcc_dev->rc_handle);

                wiced_bt_avrc_close(rcc_dev->rc_handle);
                rcc_dev->rc_handle = INVALID_TRANSACTION_LABEL;

                // TODO: Transaction cleanup
            }
        }

        if (rcc_cb.p_disc_db != NULL)
        {
            wiced_bt_free_buffer(rcc_cb.p_disc_db);
            rcc_cb.p_disc_db = NULL;
        }

        memset(&rcc_cb, 0, sizeof(rcc_cb));
        return WICED_SUCCESS;
    }

    static void wiced_bt_avrc_ct_rcc_display(void)
    {
        uint8_t i;

        WICED_BTAVRCP_TRACE("wiced_bt_avrc_ct_rcc_display\n");

        for (i = 0; i < MAX_CONNECTED_RCC_DEVICES; i++)
        {
            WICED_BTAVRCP_TRACE("[%d] %B, state: %d, role: %d, handle: %d\n",
                i,
                rcc_cb.device[i].peer_bda,
                rcc_cb.device[i].state,
                rcc_cb.device[i].role,
                rcc_cb.device[i].rc_handle);
        }
    }

    /**
     * Function         wiced_bt_avrc_ct_connect
     *
     *                  Initiate connection to the peer AVRC target device.
     *                  After connection establishment, stop listening for incoming connections
     *
     * @param[in]       remote_addr : Bluetooth address of peer device
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_connect(wiced_bt_device_address_t remote_addr)
    {
        uint8_t dev_indx;

        WICED_BTAVRCP_TRACE("Initiating AVRCP connection\n", __FUNCTION__);

        wiced_bt_avrc_ct_rcc_display();

        /* To avoid the race condition (remote device and the HOST are both doing the AVRC connection),
         * we need the check if the connection with target device has already been triggered.*/
        for (dev_indx = 0; dev_indx < MAX_CONNECTED_RCC_DEVICES; dev_indx++)
        {
            if (memcmp((void*)remote_addr,
                (void*)rcc_cb.device[dev_indx].peer_bda,
                sizeof(wiced_bt_device_address_t)) == 0)
            {
                if (rcc_cb.device[dev_indx].state != RCC_STATE_IDLE)
                {
                    return WICED_BT_SUCCESS;
                }
                else
                {
                    if (rcc_cb.device[dev_indx].role == AVRC_CONN_ACCEPTOR)
                    {
                        wiced_bt_avrc_close(rcc_cb.rc_acp_handle[dev_indx]);
                        rcc_cb.device[dev_indx].rc_handle = INVALID_TRANSACTION_LABEL;
                    }

                    wiced_bt_avrc_ct_start_discovery(remote_addr);
                    rcc_cb.flags |= RCC_FLAG_RC_API_OPEN_PENDING;

                    return WICED_PENDING;
                }
            }
        }

        /* Find a free space for this connection request. */
        for (dev_indx = 0; dev_indx < MAX_CONNECTED_RCC_DEVICES; dev_indx++)
        {
            if ((rcc_cb.device[dev_indx].state == RCC_STATE_IDLE) &&
                (rcc_cb.device[dev_indx].role == AVRC_CONN_ACCEPTOR))
            {
                wiced_bt_avrc_close(rcc_cb.rc_acp_handle[dev_indx]);
                rcc_cb.device[dev_indx].rc_handle = INVALID_TRANSACTION_LABEL;
                wiced_bt_avrc_ct_start_discovery(remote_addr);
                rcc_cb.flags |= RCC_FLAG_RC_API_OPEN_PENDING;

                return WICED_PENDING;
            }
        }

        WICED_BTAVRCP_TRACE("%s Failed to initiate AVRCP connection...No free link\n", __FUNCTION__);
        /* if reached here - means that there is no remaining link on which AVRCP connection can be initiated */
        return WICED_BT_NO_RESOURCES;
    }

    /**
     * Function         wiced_bt_avrc_ct_disconnect
     *
     *                  Disconnect from the peer AVRC target device
     *                  After disconnection , start listening for incoming connections
     *
     * @param[in]       handle : Connection handle of peer device
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_disconnect(uint8_t handle)
    {
        rcc_device_t* prcc_dev = NULL;
        WICED_BTAVRCP_TRACE("%s\n", __FUNCTION__);

        prcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (prcc_dev != 0)
        {
            WICED_BTAVRCP_TRACE("%s Calling avrc-close [%d]\n", __FUNCTION__, prcc_dev->rc_handle);
            wiced_bt_avrc_close(prcc_dev->rc_handle);
            prcc_dev->rc_handle = INVALID_TRANSACTION_LABEL;

            // TODO: Transaction cleanup
        }

        return WICED_SUCCESS;
    }

    /**
     * Function         wiced_bt_avrc_ct_send_pass_through_cmd
     *
     *                  Send PASS THROUGH command
     *
     * @param[in]       handle          : Connection handle
     * @param[in]       cmd             : Pass through command id (see #AVRC_ID_XX)
     * @param[in]       state           : State of the pass through command (see #AVRC_STATE_XX)
     * @param[in]       data_field_len  : Data field length
     * @param[in]       data_field      : Data field
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_send_pass_through_cmd(
        uint8_t handle, uint8_t cmd, uint8_t state,
        uint16_t grp_vendor_op_id)
    {
          wiced_bt_avrc_cmd_t msg;
        rcc_device_t* prcc_dev = NULL;
        rcc_transaction_t* ptransaction = NULL;

        prcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);

        WICED_BTAVRCP_TRACE("%s prcc_dev [%p] CMD[%x]\n", __FUNCTION__, prcc_dev, cmd);

        if (prcc_dev != NULL)
        {
            uint16_t snd_result = 0;

            if (wiced_bt_avrc_ct_get_transaction_for_device(prcc_dev, &ptransaction) == WICED_SUCCESS)
            {
                msg.type.pass_thru.hdr.operation_id= cmd;
                if (msg.type.pass_thru.hdr.operation_id == AVRC_ID_VENDOR)
                {
                    msg.type.pass_thru.group_nav.vendor_operation_id = grp_vendor_op_id;
                }
                msg.type.pass_thru.hdr.state = state;
                WICED_BTAVRCP_TRACE("%s label[%d] rc_handle[%d]\n", __FUNCTION__, ptransaction->label, prcc_dev->rc_handle);

                snd_result = wiced_bt_avrc_send_passthrough_cmd(prcc_dev->rc_handle, ptransaction->label, &msg.type.pass_thru);
                WICED_BTAVRCP_TRACE("%s snd_result = %d\n", __FUNCTION__, snd_result);
            }
            UNUSED_VARIABLE(snd_result);
        }

        return WICED_SUCCESS;
    }

    /**
     * Function         wiced_bt_avrc_ct_send_unit_info_cmd
     *
     *                  Send Unit Info Command
     *
     * @param[in]       handle          : Connection handle
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_send_unit_info_cmd(uint16_t handle)
    {
        rcc_device_t* prcc_dev = NULL;
        rcc_transaction_t* ptransaction = NULL;
        uint16_t snd_result = 0;

        prcc_dev = wiced_bt_avrc_ct_device_for_handle((uint8_t)handle);
        WICED_BTAVRCP_TRACE("%s prcc_dev [%p] \n", __FUNCTION__, prcc_dev);

        if (prcc_dev != NULL)
        {
            if (wiced_bt_avrc_ct_get_transaction_for_device(prcc_dev, &ptransaction) == WICED_SUCCESS)
            {
                snd_result = wiced_bt_avrc_send_unit_cmd(prcc_dev->rc_handle, ptransaction->label);
                WICED_BTAVRCP_TRACE("%s label[%d] rc_handle[%d] snd_result = %d\n",
                    __FUNCTION__,
                    ptransaction->label,
                    prcc_dev->rc_handle,
                    snd_result);

                return snd_result == AVRC_SUCCESS ? WICED_SUCCESS : WICED_NOT_CONNECTED;
            }

            return WICED_ERROR;
        }

        return WICED_BADARG;
    }

    /**
     * Function         wiced_bt_avrc_ct_send_sub_unit_info_cmd
     *
     *                  Send Sub Unit Info Command
     *
     * @param[in]       handle          : Connection handle
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_send_sub_unit_info_cmd(uint16_t handle)
    {
        rcc_device_t* prcc_dev = NULL;
        rcc_transaction_t* ptransaction = NULL;
        uint16_t snd_result = 0;

        prcc_dev = wiced_bt_avrc_ct_device_for_handle((uint8_t)handle);
        WICED_BTAVRCP_TRACE("%s prcc_dev [%p] \n", __FUNCTION__, prcc_dev);

        if (prcc_dev != NULL)
        {
            if (wiced_bt_avrc_ct_get_transaction_for_device(prcc_dev, &ptransaction) == WICED_SUCCESS)
            {
                snd_result = wiced_bt_avrc_send_subunit_cmd(prcc_dev->rc_handle, ptransaction->label, 0);
                WICED_BTAVRCP_TRACE("%s label[%d] rc_handle[%d] snd_result = %d\n",
                    __FUNCTION__,
                    ptransaction->label,
                    prcc_dev->rc_handle,
                    snd_result);

                return snd_result == AVRC_SUCCESS ? WICED_SUCCESS : WICED_NOT_CONNECTED;
            }

            return WICED_ERROR;
        }

        return WICED_BADARG;
    }

    /**
     * Function         wiced_bt_avrc_ct_get_element_attr_cmd
     *
     *                  Requests the target device to provide the attributes
     *                  of the element specified in the parameter
     *
     * @param[in]       handle      : Connection handle
     * @param[in]       element_id  : Element id
     * @param[in]       num_attr    : Number of attributes
     * @param[in]       p_attrs     : Media attribute ids (see #AVRC_MEDIA_ATTR_ID)
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_get_element_attr_cmd(uint8_t handle, wiced_bt_avrc_uid_t element_id, uint8_t num_attr, uint8_t* p_attrs)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;
        int attrCount;

        WICED_BTAVRCP_TRACE("%s num_attr: %d\n", __FUNCTION__, num_attr);

        /* TODO: First we need to validate that the peer is capable of AVRCP 1.3 or greater */

        /* Validate the requested count and attributes (num_attr can be 0) */
        if (num_attr != 0)
        {
            if (num_attr <= AVRC_MAX_ELEM_ATTR_SIZE)
            {
                for (attrCount = 0; ((attrCount < num_attr) && (result == WICED_SUCCESS)); attrCount++)
                {
                    if (!AVRC_IS_VALID_MEDIA_ATTRIBUTE(p_attrs[attrCount]))
                    {
                        WICED_BTAVRCP_TRACE("%s: Bad Attribute: %d: %d", __FUNCTION__, attrCount, p_attrs[attrCount]);
                        result = WICED_BADVALUE;
                    }
                }
            }
            else
            {
                result = WICED_BADVALUE;
            }
        }

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        /* Acquire a transaction label for this transaction */
        if (rcc_dev != NULL)
        {
            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            //wiced_bt_avrc_get_elem_attrs_cmd_t get_elem_attr;
            wiced_bt_avrc_metadata_cmd_t cmd;
            wiced_bt_avrc_sts_t  avrc_status;

            /* Build and send Get Element Attribute command */
            cmd.metadata_hdr.pdu = AVRC_PDU_GET_ELEMENT_ATTR;
            //get_elem_attr.status = AVRC_STS_NO_ERROR;
            cmd.u.get_elem_attrs.num_attr = num_attr;
            for (attrCount = 0; attrCount < num_attr; attrCount++)
            {
                cmd.u.get_elem_attrs.attrs[attrCount] = p_attrs[attrCount];
            }

            /* This call will allocate the packet */
            avrc_status = wiced_avrc_build_and_send_metadata_cmd(&cmd, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);
            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }

        return result;
    }


    /**
     * Function         wiced_bt_avrc_ct_get_play_status_cmd
     *
     *                  Get the status of the currently playing media at the TG
     *
     * @param[in]       handle      : Connection handle
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_get_play_status_cmd(uint8_t handle)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;

        WICED_BTAVRCP_TRACE("%s \n", __FUNCTION__);

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {
            //CHECK_AVRCP_1_3(rcc_dev);

            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_metadata_cmd_t cmd;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send play status command */
            cmd.metadata_hdr.pdu = AVRC_PDU_GET_PLAY_STATUS;
            //get_play_status.status = AVRC_STS_NO_ERROR;

            /* This call will allocate the packet. */
            avrc_status = wiced_avrc_build_and_send_metadata_cmd(&cmd, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }

        return result;
    }

    wiced_result_t wiced_bt_avrc_ct_inform_displayable_charset_cmd(uint8_t handle, uint8_t num_charset, uint16_t *p_charsets )
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;

        WICED_BTAVRCP_TRACE("%s \n", __FUNCTION__);

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {


            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_metadata_cmd_t cmd;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send play status command */
            cmd.metadata_hdr.pdu = AVRC_PDU_INFORM_DISPLAY_CHARSET;
            //get_play_status.status = AVRC_STS_NO_ERROR;
            cmd.u.inform_charset.num_id=1;
            cmd.u.inform_charset.p_charsets = p_charsets;
            /* This call will allocate the packet. */
            avrc_status = wiced_avrc_build_and_send_metadata_cmd(&cmd, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }

        return result;

    }

wiced_result_t wiced_bt_avrc_ct_inform_battery_status_ct_cmd(uint8_t handle, uint8_t battery_status)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;

        WICED_BTAVRCP_TRACE("%s \n", __FUNCTION__);

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {


            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_metadata_cmd_t cmd;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send play status command */
            cmd.metadata_hdr.pdu = AVRC_PDU_INFORM_BATTERY_STAT_OF_CT;
            cmd.u.inform_battery_status.battery_status = battery_status;
            /* This call will allocate the packet. */
            avrc_status = wiced_avrc_build_and_send_metadata_cmd(&cmd, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }

        return result;

    }

    /*****************************************************************************
     *  APPLICATION SETTINGS FUNCTIONS
     ****************************************************************************/

     /**
      * Function         wiced_bt_avrc_ct_list_player_attrs_cmd
      *
      *                  Request the target device to provide target supported
      *                  player application setting attributes
      *
      * @param[in]       handle      : Connection handle
      *
      * @return          wiced_result_t
      *
      */
    wiced_result_t wiced_bt_avrc_ct_list_player_attrs_cmd(uint8_t handle)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;

        WICED_BTAVRCP_TRACE("%s: Handle: %d\n", __FUNCTION__, handle);

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        WICED_BTAVRCP_TRACE("%s: result1: %d\n", __FUNCTION__, result);

        /* Acquire a transaction label for this transaction */
        if (result == WICED_SUCCESS)
        {
            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        WICED_BTAVRCP_TRACE("%s: result2: %d\n", __FUNCTION__, result);

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_metadata_cmd_t list_app_attr;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send GetCapabilities command */
            list_app_attr.metadata_hdr.pdu = AVRC_PDU_LIST_PLAYER_APP_ATTR;

            /* This call will allocate the packet. */
        /* This call will allocate the packet. */
            avrc_status = wiced_avrc_build_and_send_metadata_cmd(&list_app_attr, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }

        }

        WICED_BTAVRCP_TRACE("%s: Exit result3: %d\n", __FUNCTION__, result);

        return result;
    }


    /**
     * Function         wiced_bt_avrc_ct_list_player_values_cmd
     *
     *                  Requests the target device to list the
     *                  set of possible values for the requested player application setting attribute
     *
     * @param[in]       handle      : Connection handle
     * @param[in]       attr        : Player application setting attribute
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_list_player_values_cmd(uint8_t handle, uint8_t attr)
    {
        wiced_result_t result = WICED_ERROR;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;

        if (wiced_bt_avrc_is_valid_player_attr(attr))
        {
            result = WICED_SUCCESS;
        }

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        /* Acquire a transaction label for this transaction */
        if (result == WICED_SUCCESS)
        {
            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_metadata_cmd_t cmd;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send listPlayerApp command */
            cmd.metadata_hdr.pdu = AVRC_PDU_LIST_PLAYER_APP_VALUES;
            cmd.u.list_app_values.attr_id = attr;
            transaction->u.attribute_id = attr;

            /* This call will allocate the packet. */
            avrc_status = wiced_avrc_build_and_send_metadata_cmd(&cmd, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }

        return result;
    }


    /**
     * Function         wiced_bt_avrc_ct_get_player_value_cmd
     *
     *                  Requests the target device to provide the current set values
     *                  on the target for the provided player application setting attributes list
     *
     * @param[in]       handle      : Connection handle
     * @param[in]       num_attr    : Number of attributes
     * @param[in]       p_attrs     : Player attribute ids (see #AVRC_PLAYER_SETTING_XX)
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_get_player_value_cmd(uint8_t handle,
        uint8_t num_attr, uint8_t* p_attrs)
    {
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;
        int attrIndex;
        wiced_result_t result = WICED_SUCCESS;

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {

            /* If there is no app event, do not allow setting of app values */
            if (!rcc_dev->app_event_enabled)
            {
                result = WICED_UNSUPPORTED;
            }
        }

        /* Validate the parameters */
        if (result == WICED_SUCCESS)
        {
            if ((num_attr > 0) && (num_attr <= AVRC_MAX_APP_ATTR_SIZE))
            {
                for (attrIndex = 0; ((attrIndex < num_attr) &&
                    wiced_bt_avrc_is_valid_player_attr(p_attrs[attrIndex])); attrIndex++);
                if (attrIndex != num_attr)
                {
                    result = WICED_BADARG;
                }
            }
        }

        /* Acquire a transaction label for this transaction */
        if (result == WICED_SUCCESS)
        {
            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_metadata_cmd_t cmd;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send GetCapabilities command */
            cmd.metadata_hdr.pdu = AVRC_PDU_GET_CUR_PLAYER_APP_VALUE;
            cmd.u.get_cur_app_val.num_attr = num_attr;

             cmd.u.get_cur_app_val.p_vals = p_attrs;

            /* This call will allocate the packet */
            avrc_status = wiced_avrc_build_and_send_metadata_cmd(&cmd, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);
            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }

        return result;
    }


    /**
     * Function         wiced_bt_avrc_ct_set_player_value_cmd
     *
     *                  Requests to set the player application setting list
     *                  of player application setting values on the target device
     *
     * @param[in]       handle      : Connection handle
     * @param[in]       p_vals      : Player application setting values
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_set_player_value_cmd(uint8_t handle,
        wiced_bt_avrc_metadata_set_app_value_cmd_t* app_val)
    {
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;
        int attrIndex;
        wiced_result_t result = WICED_SUCCESS;

        WICED_BTAVRCP_TRACE("%s attr_id:%d", __FUNCTION__, app_val->num_val);

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", __FUNCTION__, handle);
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {
            /* If there is no app event, do not allow setting of app values */
            if (!rcc_dev->app_event_enabled)
            {
                result = WICED_UNSUPPORTED;
            }
        }

        /* Validate the parameters */
        if (result == WICED_SUCCESS &&(app_val->p_vals))
        {
            if ((app_val->num_val > 0) &&
                (app_val->num_val <= AVRC_MAX_APP_SETTINGS))
            {
                for (attrIndex = 0; ((attrIndex < app_val->num_val)
                    && wiced_bt_avrc_is_valid_player_attr(app_val->p_vals[attrIndex].attr_val)); attrIndex++);
                if (attrIndex != app_val->num_val)
                {
                    result = WICED_BADARG;
                }
            }
        }

        /* Acquire a transaction label for this transaction */
        if (result == WICED_SUCCESS)
        {
            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_metadata_cmd_t cmd;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send GetCapabilities command */
            cmd.metadata_hdr.pdu = AVRC_PDU_SET_PLAYER_APP_VALUE;
            cmd.u.set_app_val.num_val = app_val->num_val;
            cmd.u.set_app_val.num_val = app_val->num_val;
            cmd.u.set_app_val.p_vals = app_val->p_vals;

            /* This call will allocate the packet */
            avrc_status = wiced_avrc_build_and_send_metadata_cmd(&cmd, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }

        return result;
    }


    /**
     * Function         wiced_bt_avrc_ct_get_player_attrs_text_cmd
     *
     *                  Requests the target device to provide the current set values
     *                  on the target for the provided player application setting attributes list
     *
     * @param[in]       handle      : Connection handle
     * @param[in]       num_attr    : Number of attributes
     * @param[in]       p_attrs     : Player attribute ids (see #AVRC_PLAYER_SETTING_XX)
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_get_player_attrs_text_cmd(
        uint8_t handle,
        uint8_t num_attr, uint8_t* p_attrs)
    {
        wiced_result_t result = WICED_ERROR;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;
        int attrIndex;


        /* Validate the parameters */
        if ((num_attr > 0) && (num_attr <= AVRC_MAX_APP_ATTR_SIZE))
        {
            for (attrIndex = 0; ((attrIndex < num_attr) &&
                wiced_bt_avrc_is_valid_player_attr(p_attrs[attrIndex])); attrIndex++);
            if (attrIndex == num_attr)
            {
                result = WICED_SUCCESS;
            }
        }

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", __FUNCTION__, handle);
            result = WICED_BADARG;
        }

        /* Acquire a transaction label for this transaction */
        if (result == WICED_SUCCESS)
        {
            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_metadata_cmd_t cmd;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send GetCapabilities command */
            cmd.metadata_hdr.pdu = AVRC_PDU_GET_PLAYER_APP_ATTR_TEXT;
            cmd.u.get_app_attr_txt.num_attr = num_attr;
            cmd.u.get_app_attr_txt.p_attrs = p_attrs;

            /* This call will allocate the packet */
            avrc_status = wiced_avrc_build_and_send_metadata_cmd(&cmd, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }

        return result;
    }


int wiced_bt_avrc_parse_attr_text_value_rsp_from_stream(uint8_t *p_rcvd_data, uint16_t rcvd_data_len, wiced_bt_avrc_app_setting_text_t *p_attr_text_val)
{
    uint8_t *p_start = p_rcvd_data;
    if(rcvd_data_len >= 4)
    {
        BE_STREAM_TO_UINT8(p_attr_text_val->attr_id, p_rcvd_data);

        BE_STREAM_TO_UINT16(p_attr_text_val->name.charset_id, p_rcvd_data);
        BE_STREAM_TO_UINT8(p_attr_text_val->name.name.str_len, p_rcvd_data);
        p_attr_text_val->name.name.p_str = p_rcvd_data;

        p_rcvd_data += p_attr_text_val->name.name.str_len;
        return p_rcvd_data - p_start;
    }
    else
    {
        return -1;
    }
}

int wiced_bt_avrc_parse_get_element_attr_rsp_from_stream(uint8_t *p_rcvd_data, uint16_t rcvd_data_len, wiced_bt_avrc_attr_entry_t *p_attr)
{
    uint8_t *p_start = p_rcvd_data;
    int read;
    if(rcvd_data_len >= 8)
    {
        read = avrc_read_attr_entry_from_stream(p_rcvd_data, rcvd_data_len, p_attr);
        if (read < 0) {
            return read;
        }
        p_rcvd_data += read;
    }
    return p_rcvd_data - p_start;
}

int wiced_bt_avrc_parse_get_folder_items_rsp_from_stream(uint8_t *p_rcvd_data, uint16_t rcvd_data_len, wiced_bt_avrc_item_t  *p_item)
{
    uint8_t *p_start = p_rcvd_data;
    int read;
    read = avrc_read_browse_item_from_stream(p_rcvd_data, rcvd_data_len, p_item);
    if (read < 0) {
        return read;
    }

    return p_rcvd_data -p_start;
}


int wiced_bt_avrc_parse_folder_name_from_stream(uint8_t *p_rcvd_data, uint16_t rcvd_data_len, wiced_bt_avrc_name_t *p_name)
{
    int read;
    read = avrc_read_name_from_stream(p_rcvd_data, rcvd_data_len, p_name);
    if (read < 0) {
        return read;
    }
    return read;

}

    /**
     * Function         wiced_bt_avrc_ct_get_player_values_text_cmd
     *
     *                  Request the target device to provide target supported player
     *                  application setting value displayable text
     *
     * @param[in]       handle      : Connection handle
     * @param[in]       attr        : player application setting attribute
     * @param[in]       num_attr    : Number of values
     * @param[in]       p_attrs     : Player value scan value ids (see #AVRC_PLAYER_VAL_XX)
     *
     * @return          wiced_result_t
     *
     */
wiced_result_t wiced_bt_avrc_ct_get_player_values_text_cmd(uint8_t handle, uint8_t attr, uint8_t num_val, uint8_t* p_values)
{
    wiced_result_t result = WICED_ERROR;
    rcc_transaction_t* transaction = NULL;
    rcc_device_t* rcc_dev = NULL;

    /* Validate the parameters */
    if (wiced_bt_avrc_is_valid_player_attr(attr))
    {
        /* Limited by the size of the vals array in the tAVRC_GET_APP_VAL_TXT_CMD
                    structure below. Should be sufficient for all standard attributes */
        if ((num_val > 0) && (num_val <= AVRC_MAX_APP_ATTR_SIZE))
        {
            result = WICED_SUCCESS;
        }
    }

    rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
    if (!rcc_dev)
    {
        WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
        result = WICED_BADARG;
    }

    /* Acquire a transaction label for this transaction */
    if (result == WICED_SUCCESS)
    {
        /* Acquire a transaction label for this transaction */
        result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
    }

    if (result == WICED_SUCCESS)
    {
        wiced_bt_avrc_metadata_cmd_t cmd;
        wiced_bt_avrc_sts_t avrc_status;

        /* Build and send GetCapabilities command */
        cmd.metadata_hdr.pdu = AVRC_PDU_GET_PLAYER_APP_VALUE_TEXT;
        cmd.u.get_app_val_txt.attr_id = attr;
        cmd.u.get_app_val_txt.num_val = num_val;
        cmd.u.get_app_val_txt.p_vals = p_values;


        /* This call will allocate the packet */
        avrc_status = wiced_avrc_build_and_send_metadata_cmd(&cmd, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

        if (avrc_status != AVRC_STS_NO_ERROR)
        {
            wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
            result = WICED_ERROR;
        }
    }

    return result;

}

#if AVRC_ADV_CTRL_INCLUDED == TRUE
    /*****************************************************************************
     *  AVRCP 1.5 BROWSING FUNCTIONS
     ****************************************************************************/

     /**
      * Function         wiced_bt_avrc_ct_set_addressed_player_cmd
      *
      *                  Set the player id to the player to be addressed on the target device
      *
      * @param[in]       handle      : Connection handle
      * @param[in]       player_id   : Player id
      *
      * @return          wiced_result_t
      *
      */
    wiced_result_t wiced_bt_avrc_ct_set_addressed_player_cmd(uint8_t handle, uint16_t player_id)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {
            CHECK_BROWSING_SUPPORTED(rcc_dev);

            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_metadata_cmd_t cmd;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and set player id command */
            cmd.metadata_hdr.pdu = AVRC_PDU_SET_ADDRESSED_PLAYER;
            cmd.u.player_id = player_id;

            /* This call will allocate the packet. */
            avrc_status = wiced_avrc_build_and_send_metadata_cmd(&cmd, AVRC_CMD_CTRL, rcc_dev->rc_handle, transaction->label);

            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }
        return result;
    }


    /**
     * Function         wiced_bt_avrc_ct_set_browsed_player_cmd
     *
     *                  Set the player id to the browsed player to be addressed on the target device
     *
     * @param[in]       handle      : Connection handle
     * @param[in]       player_id   : Player id
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_set_browsed_player_cmd(uint8_t handle, uint16_t player_id)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {
            CHECK_BROWSING_SUPPORTED(rcc_dev);

            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_browse_cmd_t set_browsed_player;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send play status command */
            set_browsed_player.pdu = AVRC_PDU_SET_BROWSED_PLAYER;
            set_browsed_player.browse_cmd.player_id = player_id;

            /* This call will allocate the packet. */
            avrc_status = wiced_avrc_build_and_send_browse_cmd(&set_browsed_player, AVRC_CMD_CTRL, rcc_dev->rc_handle, transaction->label);
            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }
        return result;
    }


    /**
     * Function         wiced_bt_avrc_ct_change_path_cmd
     *
     *                  Change the path in the Virtual file system being browsed
     *
     * @param[in]       handle      : Connection handle
     * @param[in]       direction   : Direction of path change
     * @param[in]       path_uid    : Path uid
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_change_path_cmd(uint8_t handle,
        uint8_t direction, wiced_bt_avrc_uid_t path_uid)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;


        /* Make sure we have a valid direction */
        if ((direction == AVRC_DIR_UP) || (direction == AVRC_DIR_DOWN))
        {
            rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
            if (!rcc_dev)
            {
                WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
                result = WICED_BADARG;
            }
        }
        else
        {
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {
            CHECK_BROWSING_SUPPORTED(rcc_dev);

            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_browse_cmd_t change_path_cmd;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send play status command */
            change_path_cmd.pdu = AVRC_PDU_CHANGE_PATH;
            change_path_cmd.browse_cmd.chg_path.uid_counter = rcc_dev->last_UID_counter;
            change_path_cmd.browse_cmd.chg_path.direction = direction;
            memcpy(change_path_cmd.browse_cmd.chg_path.folder_uid, path_uid, sizeof(wiced_bt_avrc_uid_t));

            /* This call will allocate the packet. */
            avrc_status = wiced_avrc_build_and_send_browse_cmd(&change_path_cmd, AVRC_CMD_CTRL, rcc_dev->rc_handle, transaction->label);

            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }
        return result;
    }


    /**
     * Function         wiced_bt_avrc_ct_get_folder_items_cmd
     *
     *                  Retrieves a listing of the contents of a folder
     *
     * @param[in]       handle      : Connection handle
     * @param[in]       scope       : Scope of the folder
     * @param[in]       start_item  : Start item index
     * @param[in]       end_item    : End item index
     * @param[in]       num_attr    : Number of attributes
     * @param[in]       p_attrs     : Media attribute ids (see #AVRC_MEDIA_ATTR_ID)
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_get_folder_items_cmd(uint8_t handle,
        uint8_t scope, uint32_t start_item, uint32_t end_item,
        uint8_t num_attr, uint32_t* p_attrs)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;

        if (!isValidMediaScope(scope))
        {
            result = WICED_BADARG;
        }
        else if ((num_attr != 0) && (num_attr != 0xFF))
        {
            int attrIndex;

            for (attrIndex = 0; ((attrIndex < num_attr) && isValidMediaAttribute(p_attrs[attrIndex])); attrIndex++);
            if (attrIndex != num_attr)
            {
                result = WICED_BADARG;
            }
        }

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {
            CHECK_BROWSING_SUPPORTED(rcc_dev);

            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_browse_cmd_t get_folder_items;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send play status command */
            get_folder_items.pdu = AVRC_PDU_GET_FOLDER_ITEMS;
            get_folder_items.browse_cmd.get_folder_items.scope = scope;
            get_folder_items.browse_cmd.get_folder_items.start_item = start_item;
            get_folder_items.browse_cmd.get_folder_items.end_item = end_item;
            get_folder_items.browse_cmd.get_folder_items.attr_count = num_attr;
            get_folder_items.browse_cmd.get_folder_items.p_attr_list = (uint32_t*)p_attrs;

            /* This call will allocate the packet. */
            avrc_status = wiced_avrc_build_and_send_browse_cmd(&get_folder_items, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);
            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }

        return result;
    }


    /**
     * Function         wiced_bt_avrc_ct_get_item_attributes_cmd
     *
     *                  Retrieves the metadata attributes for a
     *                  particular media element item or folder item
     *
     * @param[in]       handle      : Connection handle
     * @param[in]       scope       : Scope of the item
     * @param[in]       path_uid    : Path of the item
     * @param[in]       num_attr    : Number of attributes
     * @param[in]       p_attrs     : Media attribute ids (see #AVRC_MEDIA_ATTR_ID)
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_get_item_attributes_cmd(
        uint8_t handle, uint8_t scope, wiced_bt_avrc_uid_t path_uid,
        uint8_t num_attr, uint32_t* p_attrs)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;

        if (!isValidMediaScope(scope))
        {
            result = WICED_BADARG;
        }
        else if ((num_attr != 0) && (num_attr != 0xFF))
        {
            int attrIndex;

            for (attrIndex = 0; ((attrIndex < num_attr) && isValidMediaAttribute(p_attrs[attrIndex])); attrIndex++);
            if (attrIndex != num_attr)
            {
                result = WICED_BADARG;
            }
        }

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {
            CHECK_BROWSING_SUPPORTED(rcc_dev);

            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_browse_cmd_t get_item_attr_cmd;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send play status command */
            get_item_attr_cmd.pdu = AVRC_PDU_GET_ITEM_ATTRIBUTES;
            get_item_attr_cmd.browse_cmd.get_item_attrs.scope = scope;
            get_item_attr_cmd.browse_cmd.get_item_attrs.uid_counter = rcc_dev->last_UID_counter;

            memcpy(get_item_attr_cmd.browse_cmd.get_item_attrs.uid, path_uid, sizeof(wiced_bt_avrc_uid_t));

            get_item_attr_cmd.browse_cmd.get_item_attrs.attr_count = num_attr;
            get_item_attr_cmd.browse_cmd.get_item_attrs.p_attr_list = (uint32_t*)p_attrs;

            /* This call will allocate the packet. */
            avrc_status = wiced_avrc_build_and_send_browse_cmd(&get_item_attr_cmd, AVRC_CMD_STATUS, rcc_dev->rc_handle, transaction->label);

            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }

        return result;
    }


    /**
     * Function         wiced_bt_avrc_ct_search_cmd
     *
     *                  Performs search from the current folder
     *                  in the Browsed Player's virtual file system
     *
     * @param[in]       handle          : Connection handle
     * @param[in]       search_string   : Search string
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_search_cmd(uint8_t handle, wiced_bt_avrc_full_name_t search_string)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }
        if (result == WICED_SUCCESS)
        {
            CHECK_BROWSING_SUPPORTED(rcc_dev);

            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_browse_cmd_t search_cmd;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send play status command */
            search_cmd.pdu = AVRC_PDU_SEARCH;
            search_cmd.browse_cmd.search.string.charset_id = search_string.charset_id;
            search_cmd.browse_cmd.search.string.name.str_len = search_string.name.str_len;
            search_cmd.browse_cmd.search.string.name.p_str = search_string.name.p_str;

            /* This call will allocate the packet. */
            avrc_status = wiced_avrc_build_and_send_browse_cmd(&search_cmd, AVRC_CMD_CTRL, rcc_dev->rc_handle, transaction->label);

            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }
        return result;
    }


    /**
     * Function         wiced_bt_avrc_ct_play_item_cmd
     *
     *                  Starts playing an item indicated by the UID
     *
     * @param[in]       handle      : Connection handle
     * @param[in]       scope       : Scope of the item (see #AVRC_SCOPE_XX)
     * @param[in]       item_uid    : UID of the item
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_play_item_cmd(uint8_t handle, uint8_t scope, wiced_bt_avrc_uid_t item_uid)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;

        if (!isValidMediaScope(scope))
        {
            result = WICED_BADARG;
        }

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {
            CHECK_BROWSING_SUPPORTED(rcc_dev);

            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_metadata_cmd_t play_item_cmd;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send play status command */
            play_item_cmd.metadata_hdr.pdu = AVRC_PDU_PLAY_ITEM;
            play_item_cmd.u.play_item.scope = scope;
            play_item_cmd.u.play_item.uid_counter = rcc_dev->last_UID_counter;

            memcpy(play_item_cmd.u.play_item.uid, item_uid, sizeof(wiced_bt_avrc_uid_t));

            avrc_status = wiced_avrc_build_and_send_metadata_cmd(&play_item_cmd, AVRC_CMD_CTRL, rcc_dev->rc_handle, transaction->label);

            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }
        }

        return result;
    }


    /**
     * Function         wiced_bt_avrc_ct_get_total_num_items
     *
     *                 get the total number of items in current folder for given scope
     *
     * @param[in]       handle      : Connection handle
     * @param[in]       scope       : Scope of the item (see #AVRC_SCOPE_XX)
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_get_total_num_items(uint8_t handle, uint8_t scope)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;

        if (!isValidMediaScope(scope))
        {
            result = WICED_BADARG;
        }

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {
            CHECK_BROWSING_SUPPORTED(rcc_dev);

            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_browse_cmd_t get_total_items;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send play status command */
            get_total_items.pdu = AVRC_PDU_GET_TOTAL_NUM_OF_ITEMS;
            get_total_items.browse_cmd.get_num_of_items.scope = scope;

               /* This call will allocate the packet. */
               avrc_status = wiced_avrc_build_and_send_browse_cmd(&get_total_items, AVRC_CMD_CTRL, rcc_dev->rc_handle, transaction->label);
               if (avrc_status != AVRC_STS_NO_ERROR)
               {
                   wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                   result = WICED_ERROR;
               }
        }

        return result;
    }


    /**
     * Function         wiced_bt_avrc_ct_add_to_now_playing_cmd
     *
     *                  Adds an item indicated by the UID to the Now Playing queue
     *
     * @param[in]       handle      : Connection handle
     * @param[in]       scope       : Scope of the item (see #AVRC_SCOPE_XX)
     * @param[in]       item_uid    : UID of the item
     *
     * @return          wiced_result_t
     *
     */
    wiced_result_t wiced_bt_avrc_ct_add_to_now_playing_cmd(uint8_t handle, uint8_t scope, wiced_bt_avrc_uid_t item_uid)
    {
        wiced_result_t result = WICED_SUCCESS;
        rcc_transaction_t* transaction = NULL;
        rcc_device_t* rcc_dev = NULL;

        if (!isValidMediaScope(scope))
        {
            result = WICED_BADARG;
        }

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", handle);
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {
            CHECK_BROWSING_SUPPORTED(rcc_dev);

            /* Acquire a transaction label for this transaction */
            result = wiced_bt_avrc_ct_get_transaction_for_device(rcc_dev, &transaction);
        }

        if (result == WICED_SUCCESS)
        {
            wiced_bt_avrc_metadata_cmd_t add_to_play_cmd;
            wiced_bt_avrc_sts_t avrc_status;

            /* Build and send play status command */
            add_to_play_cmd.metadata_hdr.pdu = AVRC_PDU_ADD_TO_NOW_PLAYING;
            add_to_play_cmd.u.play_item.scope = scope;
            add_to_play_cmd.u.play_item.uid_counter = rcc_dev->last_UID_counter;

            memcpy(add_to_play_cmd.u.play_item.uid, item_uid, sizeof(wiced_bt_avrc_uid_t));

            avrc_status = wiced_avrc_build_and_send_metadata_cmd(&add_to_play_cmd, AVRC_CMD_CTRL, rcc_dev->rc_handle, transaction->label);

            if (avrc_status != AVRC_STS_NO_ERROR)
            {
                wiced_bt_avrc_ct_release_transaction_for_device(rcc_dev, transaction->label);
                result = WICED_ERROR;
            }

        }

        return result;
    }


    /*****************************************************************************
     *  VOLUME FUNCTIONS
     ****************************************************************************/
     /**
      * Function         wiced_bt_avrc_ct_set_volume_cmd
      *
      *                  Set volume for peer device
      *
      * @param[in]       handle      : Connection handle
      * @param[in]       volume      : Volume
      *
      * @return          wiced_result_t
      *
      */
    wiced_result_t wiced_bt_avrc_ct_set_volume_cmd(uint8_t handle, uint8_t volume)
    {
        wiced_result_t result = WICED_SUCCESS;
        // int connectedDeviceIndex;
        wiced_bt_avrc_metadata_rsp_t avrc_rsp;
        wiced_bt_avrc_xmit_buf_t* p_rsp_pkt = NULL;
        rcc_device_t* rcc_dev = NULL;

        if (!isValidAbsoluteVolume(volume))
        {
            result = WICED_BADARG;
        }

        rcc_dev = wiced_bt_avrc_ct_device_for_handle(handle);
        if (!rcc_dev)
        {
            WICED_BTAVRCP_TRACE("[%s] Device not found for handle:%d\n", __FUNCTION__, handle);
            result = WICED_BADARG;
        }

        if (result == WICED_SUCCESS)
        {
            uint8_t label;

            WICED_BTAVRCP_TRACE("%s: volume: %d current: %d\n", __FUNCTION__, volume, rcc_dev->current_volume);
            /* Only update if the volume setting is different from the current value */
            if (rcc_dev->current_volume == volume)
            {
                WICED_BTAVRCP_TRACE("[%s] Volume level already attained:%d\n", __FUNCTION__, rcc_dev->current_volume);
                return result;
            }

            label = rcc_dev->abs_volume_reg_label;
            /*  Cache the new value */
            rcc_dev->current_volume = volume;

            // Device has registered for notification. Send the CHANGED response.
            if (label != INVALID_TRANSACTION_LABEL)
            {
                /* Enable for future use */
                rcc_dev->abs_volume_reg_label = INVALID_TRANSACTION_LABEL;

                avrc_rsp.metadata_hdr.pdu = AVRC_PDU_REGISTER_NOTIFICATION;
                avrc_rsp.u.reg_notif.event_id = AVRC_EVT_VOLUME_CHANGE;
                avrc_rsp.u.reg_notif.param.volume = volume;

                result = wiced_avrc_build_metadata_rsp (&avrc_rsp, &p_rsp_pkt);

                if( p_rsp_pkt != NULL )
                {
                        wiced_avrc_send_or_enqueue (rcc_dev->rc_handle, label, AVRC_RSP_CHANGED, p_rsp_pkt);
                }

            }
            else
            {
                result = WICED_BT_WRONG_MODE;
                WICED_BTAVRCP_TRACE("%s: wrong label: %d \n", __FUNCTION__, label);
            }
        }

        return result;
    }
#endif

    /*
     * LRAC Switch Structure
     */
    typedef struct
    {
        uint8_t                     rc_acp_handle[MAX_CONNECTED_RCC_DEVICES];
        uint32_t                    remote_features;
        uint32_t                    features;
        REMOTE_CONTROL_INFO         peer_ct;        /* peer CT role info */
        REMOTE_CONTROL_INFO         peer_tg;        /* peer TG role info */
        uint8_t                     flags;
        rcc_device_t                device[MAX_CONNECTED_RCC_DEVICES];
    } wiced_bt_avrc_ct_lrac_switch_data_t;

    /** API To get LRAC Switch data
     *
     *  Called by the application to get the LRAC Switch Data
     *
     *  @param p_opaque     Pointer to a buffer which will be filled with LRAC Switch data (current
     *                      A2DP Sink State)
     *  @param p_opaque     Size of the buffer (IN), size filled (OUT)
     *
     *  @return none
     */
    wiced_result_t wiced_bt_avrc_ct_lrac_switch_get(void* p_opaque, uint16_t* p_sync_data_len)
    {
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
        wiced_bt_avrc_ct_lrac_switch_data_t switch_data;

        if (p_opaque == NULL)
        {
            WICED_BT_TRACE("%s Err: p_opaque is NULL\n", __FUNCTION__);
            return WICED_BT_BADARG;
        }

        if (p_sync_data_len == NULL)
        {
            WICED_BT_TRACE("%s Err: p_sync_data_len is NULL\n", __FUNCTION__);
            return WICED_BT_BADARG;
        }

        if (*p_sync_data_len < sizeof(switch_data))
        {
            WICED_BT_TRACE("%s Err: buffer too small (%d/%d)\n", __FUNCTION__,
                *p_sync_data_len, sizeof(switch_data));
            return WICED_BT_BADARG;
        }

        /*
         * We don't need every data of the Control Block. Create a, temporary, structure containing
         * the data we need
         */
        memcpy(switch_data.rc_acp_handle, &rcc_cb.rc_acp_handle, sizeof(switch_data.rc_acp_handle));
        switch_data.remote_features = rcc_cb.remote_features;
        switch_data.features = rcc_cb.features;
        switch_data.flags = rcc_cb.flags;
        memcpy(&switch_data.peer_ct, &rcc_cb.peer_ct, sizeof(switch_data.peer_ct));
        memcpy(&switch_data.peer_tg, &rcc_cb.peer_tg, sizeof(switch_data.peer_tg));
        memcpy(switch_data.device, &rcc_cb.device, sizeof(switch_data.device));

        memcpy(p_opaque, &switch_data, sizeof(switch_data));

        *p_sync_data_len = sizeof(switch_data);

        return WICED_BT_SUCCESS;
#else
        return WICED_UNSUPPORTED;
#endif
    }

    /** API To set LRAC Switch data
     *
     *  Called by the application to set the LRAC Switch Data
     *
     *  @param p_opaque     Pointer to a buffer which contains LRAC Switch data (new
     *                      A2DP Sink State)
     *  @param p_opaque     Size of the buffer (IN)
     *
     *  @return none
     */
    wiced_result_t wiced_bt_avrc_ct_lrac_switch_set(void* p_opaque, uint16_t sync_data_len)
    {
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
        wiced_bt_avrc_ct_lrac_switch_data_t* p_switch_data;

        if (p_opaque == NULL)
        {
            WICED_BT_TRACE("%s Err: p_opaque is NULL\n", __FUNCTION__);
            return WICED_BT_BADARG;
        }

        if (sync_data_len != sizeof(wiced_bt_avrc_ct_lrac_switch_data_t))
        {
            WICED_BT_TRACE("%s Err: bad buffer size (%d/%d)\n", __FUNCTION__,
                sync_data_len, sizeof(wiced_bt_avrc_ct_lrac_switch_data_t));
            return WICED_BT_BADARG;
        }
        p_switch_data = (wiced_bt_avrc_ct_lrac_switch_data_t*)p_opaque;

        memcpy(&rcc_cb.rc_acp_handle, p_switch_data->rc_acp_handle, sizeof(rcc_cb.rc_acp_handle));
        rcc_cb.remote_features = p_switch_data->remote_features;
        rcc_cb.features = p_switch_data->features;
        rcc_cb.flags = p_switch_data->flags;
        memcpy(&rcc_cb.peer_ct, &p_switch_data->peer_ct, sizeof(rcc_cb.peer_ct));
        memcpy(&rcc_cb.peer_tg, &p_switch_data->peer_tg, sizeof(rcc_cb.peer_tg));
        memcpy(&rcc_cb.device, p_switch_data->device, sizeof(rcc_cb.device));

        return WICED_BT_SUCCESS;
#else
        return WICED_UNSUPPORTED;
#endif
    }

    /*
     * wiced_bt_avrc_ct_connection_open
     *
     * Open AVCT connection for target device
     *
     * @param[in]   p_handle: assigned handle for the AVRC connection
     *
     * @param[in]   role: AVRC_CONN_INITIATOR or AVRC_CONN_ACCEPTOR
     *
     * @param[in]   bdaddr: target device's bdaddr
     *
     * @param[in]   local_feature
     */
    static void wiced_bt_avrc_ct_connection_open(uint8_t dev_idx, uint8_t role, wiced_bt_device_address_t bdaddr, uint8_t local_feature)
    {
        wiced_bt_avrc_config_t ccb;
        uint16_t result;
        uint8_t* p_handle;

        if (role == AVRC_CONN_ACCEPTOR)
        {
            p_handle = &rcc_cb.rc_acp_handle[dev_idx];
        }
        else
        {
            p_handle = &rcc_cb.device[dev_idx].rc_handle;
        }

        ccb.p_ctrl_cback = wiced_bt_avrc_ctrl_cback;
        ccb.p_msg_cback = wiced_bt_avrc_msg_cback;
        ccb.company_id = AVRC_CO_METADATA;
        ccb.connection_role = role;
        ccb.control = local_feature;

        ccb.p_avct_buff = rcc_cb.p_avct_buf[dev_idx];
        ccb.avct_seg_buf_len = MAX_AVCT_RCV_PKT_SIZE;

        ccb.p_avrc_buff = rcc_cb.p_avrc_buf[dev_idx];
        ccb.avrc_seg_buf_len = MAX_METADATA_RCV_MSG_SIZE;
        if (rcc_cb.p_browse_drb[dev_idx] == NULL)
             rcc_cb.p_browse_drb[dev_idx] = wiced_bt_get_buffer(remote_control_config.avrc_br_mtu + DRB_OVERHEAD_SIZE);

        result  = wiced_bt_avrc_init(remote_control_config.avrc_mtu, remote_control_config.avrc_br_mtu);
        if(result != AVRC_SUCCESS)
        {
            WICED_BTAVRCP_TRACE("%s:wiced_bt_avrc_init : FAILED (%d) \n", __FUNCTION__, result);
        }
        result = wiced_bt_avrc_open(p_handle, &ccb, (BD_ADDR_PTR)bdaddr);

        if (result != AVRC_SUCCESS)
        {
            WICED_BTAVRCP_TRACE("%s (%B, 0x%02X, %d)\n", __FUNCTION__, bdaddr, *p_handle, result);
        }
    }

    /** @}*/

#ifdef __cplusplus
}
#endif
