/*
 * handsfree.c
 *
 *  Created on: Aug 20, 2024
 *      Author: ChauhanHardi
 */
#ifdef HFP_ENABLED
#include "lepl.h"
#include "handsfree.h"
#include "wiced_transport.h"
#include "wiced_hal_nvram.h"

#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1) || defined(CYW55500A1)
#define HFP_VOLUME_HIGH 15
#include "wiced_audio_manager.h"
#endif

static int32_t stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
bluetooth_hfp_context_t handsfree_ctxt_data;
handsfrees_app_globals handsfree_app_states;

const uint8_t handsfree_sdp_db[] =
{
    SDP_ATTR_SEQUENCE_1(75 + 2),

    // SDP Record for Hands-Free Unit
    SDP_ATTR_SEQUENCE_1(75),
        SDP_ATTR_RECORD_HANDLE(HDLR_HANDS_FREE_UNIT),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST), SDP_ATTR_SEQUENCE_1(6),
            SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
            SDP_ATTR_UUID16(UUID_SERVCLASS_GENERIC_AUDIO),
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(HANDS_FREE_SCN),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
                SDP_ATTR_VALUE_UINT2(0x0108),
        SDP_ATTR_SERVICE_NAME(15),
            'W', 'I', 'C', 'E', 'D', ' ', 'H', 'F', ' ', 'D', 'E', 'V', 'I', 'C', 'E',
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, SUPPORTED_FEATURES_ATT),
};

static void hfp_timer_expiry_handler( TIMER_PARAM_TYPE param );


wiced_bt_sco_params_t handsfree_esco_params =
{
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        0x000D,             /* Latency: 13 ms ( HS/HF can use EV3, 2-EV3, 3-EV3 ) ( T2 ) */
#else
        0x000C,             /* Latency: 12 ms ( HS/HF can use EV3, 2-EV3, 3-EV3 ) ( S4 ) */
#endif
        HANDS_FREE_SCO_PKT_TYPES,
        BTM_ESCO_RETRANS_POWER, /* Retrans Effort ( At least one retrans, opt for power ) ( S4 ) */
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        WICED_TRUE
#else
        WICED_FALSE
#endif
};

static audio_config_t audio_config =
    {
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        .sr = AM_PLAYBACK_SR_16K,
#else
        .sr = AM_PLAYBACK_SR_8K,
#endif
       .channels = 1,
       .bits_per_sample = DEFAULT_BITSPSAM,
       .volume = AM_VOL_LEVEL_HIGH-2,
       .mic_gain = AM_VOL_LEVEL_HIGH-2,
       .sink = AM_HEADPHONES,
    };


void hci_control_send_hf_event(uint16_t evt, uint16_t handle, hci_control_hf_event_t *p_data)
{
    uint8_t   tx_buf[300];
    uint8_t  *p = tx_buf;
    int       i;

    WICED_BT_TRACE("[%u]hci_control_send_hf_event: Sending Event: %u  to UART\n", handle, evt);

    *p++ = (uint8_t)(handle);
    *p++ = (uint8_t)(handle >> 8);

    switch (evt)
    {
        case HCI_CONTROL_HF_EVENT_OPEN:                 /* HS connection opened or connection attempt failed  */
            for (i = 0; i < BD_ADDR_LEN; i++)
                *p++ = p_data->open.bd_addr[BD_ADDR_LEN - 1 - i];
            *p++ = p_data->open.status;
            break;

        case HCI_CONTROL_HF_EVENT_CLOSE:                /* HS connection closed */
            break;

        case HCI_CONTROL_HF_EVENT_AUDIO_OPEN:           /* Audio connection open */
            break;

        case HCI_CONTROL_HF_EVENT_AUDIO_CLOSE:          /* Audio connection closed */
            break;

        case HCI_CONTROL_HF_EVENT_CONNECTED:            /* HS Service Level Connection is UP */
            UINT32_TO_STREAM(p,p_data->conn.peer_features);
            break;

        case HCI_CONTROL_HF_EVENT_PROFILE_TYPE:
            UINT8_TO_STREAM(p,p_data->conn.profile_selected);
            break;
        default:                                        /* AT response */
            if (p_data)
            {
                *p++ = (uint8_t)(p_data->val.num);
                *p++ = (uint8_t)(p_data->val.num >> 8);
                utl_strcpy((char *)p, p_data->val.str);
                p += strlen(p_data->val.str) + 1;
            }
            else
            {
                *p++ = 0;               // val.num
                *p++ = 0;
                *p++ = 0;               // empty val.str
            }
            break;
    }
    wiced_transport_send_data(evt, tx_buf, (int)(p - tx_buf));
}

static void handsfree_connection_event_handler(wiced_bt_hfp_hf_event_data_t* p_data)
{
    wiced_bt_dev_status_t status;

    if(p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_CONNECTED)
    {
        hci_control_hf_open_t    open;
        wiced_bt_hfp_hf_scb_t *p_scb = wiced_bt_hfp_hf_get_scb_by_bd_addr (p_data->conn_data.remote_address);
        memcpy(open.bd_addr,p_data->conn_data.remote_address,BD_ADDR_LEN);
        open.status = WICED_BT_SUCCESS;
        handsfree_ctxt_data.rfcomm_handle = p_scb->rfcomm_handle;
        wiced_bt_dev_switch_role(p_data->conn_data.remote_address, HCI_ROLE_CENTRAL, NULL);
        hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_OPEN, p_scb->rfcomm_handle, (hci_control_hf_event_t *) &open);

        if( p_data->conn_data.connected_profile == WICED_BT_HFP_PROFILE )
        {
            handsfree_app_states.connect.profile_selected = WICED_BT_HFP_PROFILE;
            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_PROFILE_TYPE, p_scb->rfcomm_handle, (hci_control_hf_event_t *) &handsfree_app_states.connect);
        }
        else
        {
            handsfree_app_states.connect.profile_selected = WICED_BT_HSP_PROFILE;
            memcpy( handsfree_ctxt_data.peer_bd_addr, p_data->conn_data.remote_address, sizeof(wiced_bt_device_address_t));
            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_PROFILE_TYPE, p_scb->rfcomm_handle, (hci_control_hf_event_t *) &handsfree_app_states.connect);
        }

        status = wiced_bt_sco_create_as_acceptor(&handsfree_ctxt_data.sco_index);
        WICED_BT_TRACE("%s: status [%d] SCO INDEX [%d] \n", __func__, status, handsfree_ctxt_data.sco_index);
    }
    else if(p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_SLC_CONNECTED)
    {
        WICED_BT_TRACE("%s: Peer BD Addr [%B]\n", __func__,p_data->conn_data.remote_address);

        memcpy( handsfree_ctxt_data.peer_bd_addr, p_data->conn_data.remote_address, sizeof(wiced_bt_device_address_t));
    }
    else if(p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_DISCONNECTED)
    {
        memset(handsfree_ctxt_data.peer_bd_addr, 0, sizeof(wiced_bt_device_address_t));
        if(handsfree_ctxt_data.sco_index != BT_AUDIO_INVALID_SCO_INDEX)
        {
            status = wiced_bt_sco_remove(handsfree_ctxt_data.sco_index);
            handsfree_ctxt_data.sco_index = BT_AUDIO_INVALID_SCO_INDEX;
            WICED_BT_TRACE("%s: remove sco status [%d] \n", __func__, status);
        }
        hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_CLOSE, handsfree_ctxt_data.rfcomm_handle, NULL);
    }
    UNUSED_VARIABLE(status);
}


static void handsfree_call_setup_event_handler(wiced_bt_hfp_hf_call_data_t* call_data)
{
    switch (call_data->setup_state)
    {
        case WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING:
            WICED_BT_TRACE("%s: Call(incoming) setting-up\n", __func__);
            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE:
            if(call_data->active_call_present == 0)
            {
                if(handsfree_ctxt_data.call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING ||
                        handsfree_ctxt_data.call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING ||
                        handsfree_ctxt_data.call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING )
                {
                    WICED_BT_TRACE("Call: Inactive; Call Set-up: IDLE\n");
                    break;
                }
                /* If previous context has an active-call and active_call_present is 0 */
                if(handsfree_ctxt_data.call_active == 1)
                {
                    WICED_BT_TRACE("Call Terminated\n");
                    break;
                }
            }
            else if( call_data->active_call_present == 1)
            {
                WICED_BT_TRACE("Call: Active; Call-setup: DONE\n");
            }
            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING:
            WICED_BT_TRACE("Call(outgoing) setting-up\n");
            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING:
            WICED_BT_TRACE("Remote(outgoing) ringing\n");
            break;

        default:
            break;
    }
    handsfree_ctxt_data.call_active = call_data->active_call_present;
    handsfree_ctxt_data.call_setup  = call_data->setup_state;
    handsfree_ctxt_data.call_held   = call_data->held_call_present;
}

static void handsfree_send_ciev_cmd (uint16_t handle, uint8_t ind_id,uint8_t ind_val,hci_control_hf_value_t *p_val)
{
    wiced_bt_hfp_hf_scb_t    *p_scb = wiced_bt_hfp_hf_get_scb_by_handle(handle);
    p_val->str[0] = '0'+ind_id;
    p_val->str[1] = ',';
    p_val->str[2] = '0'+ind_val;
    p_val->str[3] = '\0';
    hci_control_send_hf_event( HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CIEV, p_scb->rfcomm_handle, (hci_control_hf_event_t *)p_val );
}

static void handsfree_send_clcc_evt (uint16_t handle, wiced_bt_hfp_hf_active_call_t *active_call,hci_control_hf_value_t *p_val)
{
    wiced_bt_hfp_hf_scb_t    *p_scb = wiced_bt_hfp_hf_get_scb_by_handle(handle);
    int i = 0;

    p_val->str[i++] = '0'+active_call->idx;
    p_val->str[i++] = ',';
    p_val->str[i++] = '0'+active_call->dir;
    p_val->str[i++] = ',';
    p_val->str[i++] = '0'+active_call->status;
    p_val->str[i++] = ',';
    p_val->str[i++] = '0'+active_call->mode;
    p_val->str[i++] = ',';
    p_val->str[i++] = '0'+active_call->is_conference;

    if(active_call->type)
    {
        p_val->str[i++] = ',';
        memcpy(&p_val->str[i],active_call->num,strlen(active_call->num));
        i +=  strlen(active_call->num);
        p_val->str[i++] = ',';
        i += utl_itoa (active_call->type,&p_val->str[i]);
    }
    p_val->str[i++] = '\0';
    hci_control_send_hf_event( HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CLCC, p_scb->rfcomm_handle, (hci_control_hf_event_t *)p_val );
}

static void handsfree_event_callback( wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data)
{
    hci_control_hf_event_t     p_val;
    int res = 0;

    memset(&p_val,0,sizeof(hci_control_hf_event_t));

    switch(event)
    {
        case WICED_BT_HFP_HF_CONNECTION_STATE_EVT:
            handsfree_connection_event_handler(p_data);
            break;

        case WICED_BT_HFP_HF_AG_FEATURE_SUPPORT_EVT:
            res = HCI_CONTROL_HF_EVENT_CONNECTED;
            p_val.conn.peer_features = p_data->ag_feature_flags;

            if(p_data->ag_feature_flags & WICED_BT_HFP_AG_FEATURE_INBAND_RING_TONE_CAPABILITY)
            {
                handsfree_ctxt_data.inband_ring_status = WICED_BT_HFP_HF_INBAND_RING_ENABLED;
            }
            else
            {
                handsfree_ctxt_data.inband_ring_status = WICED_BT_HFP_HF_INBAND_RING_DISABLED;
            }
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
            {
                wiced_bt_hfp_hf_scb_t    *p_scb = wiced_bt_hfp_hf_get_scb_by_handle(p_data->handle);
                if( (p_data->ag_feature_flags & WICED_BT_HFP_AG_FEATURE_CODEC_NEGOTIATION) &&
                        (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION) )
                {
                    handsfree_esco_params.use_wbs = WICED_TRUE;
                }
                else
                {
                    handsfree_esco_params.use_wbs = WICED_FALSE;
                }
            }
#endif
            break;

        case WICED_BT_HFP_HF_SERVICE_STATE_EVT:
            handsfree_send_ciev_cmd (p_data->handle,WICED_BT_HFP_HF_SERVICE_IND,p_data->service_state,&p_val.val);
            break;

        case WICED_BT_HFP_HF_CALL_SETUP_EVT:
        {
            if (handsfree_ctxt_data.call_active != p_data->call_data.active_call_present)
                handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_CALL_IND,p_data->call_data.active_call_present,&p_val.val);

            if (handsfree_ctxt_data.call_held != p_data->call_data.held_call_present)
                handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_CALL_HELD_IND,p_data->call_data.held_call_present,&p_val.val);

            if (handsfree_ctxt_data.call_setup != p_data->call_data.setup_state)
                handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_CALL_SETUP_IND,p_data->call_data.setup_state,&p_val.val);

            handsfree_call_setup_event_handler(&p_data->call_data);
        }
            break;

        case WICED_BT_HFP_HF_RSSI_IND_EVT:
            handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_SIGNAL_IND,p_data->rssi,&p_val.val);
            break;

        case WICED_BT_HFP_HF_SERVICE_TYPE_EVT:
            handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_ROAM_IND,p_data->service_type,&p_val.val);
            break;

        case WICED_BT_HFP_HF_BATTERY_STATUS_IND_EVT:
            handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_BATTERY_IND,p_data->battery_level,&p_val.val);
            break;

        case WICED_BT_HFP_HF_RING_EVT:
            WICED_BT_TRACE("%s: RING \n", __func__);
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_RING;
            break;

        case WICED_BT_HFP_HF_INBAND_RING_STATE_EVT:
            handsfree_ctxt_data.inband_ring_status = p_data->inband_ring;
            break;

        case WICED_BT_HFP_HF_OK_EVT:
            WICED_BT_TRACE("%s: OK \n", __func__);
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_OK;
            break;

        case WICED_BT_HFP_HF_ERROR_EVT:
            WICED_BT_TRACE("%s: Error \n", __func__);
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_ERROR;
            break;

        case WICED_BT_HFP_HF_CME_ERROR_EVT:
            WICED_BT_TRACE("%s: CME Error \n", __func__);
            p_val.val.num = p_data->error_code;
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CMEE;
            break;

        case WICED_BT_HFP_HF_CLIP_IND_EVT:
            p_val.val.num = p_data->clip.type;
            strncpy( p_val.val.str, p_data->clip.caller_num, sizeof( p_val.val.str ) );
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CLIP;
            WICED_BT_TRACE("%s: CLIP - number %s, type %d\n", __func__, p_data->clip.caller_num, p_data->clip.type);
            break;

        case WICED_BT_HFP_HF_BINP_EVT:
            p_val.val.num = p_data->binp_data.type;
            strncpy( p_val.val.str, p_data->binp_data.caller_num, sizeof( p_val.val.str ) );
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BINP;
            WICED_BT_TRACE("%s: BINP - number %s, type %d\n", __func__, p_data->binp_data.caller_num, p_data->binp_data.type);
            break;

        case WICED_BT_HFP_HF_VOLUME_CHANGE_EVT:
            WICED_BT_TRACE("%s: %s VOLUME - %d \n", __func__, (p_data->volume.type == WICED_BT_HFP_HF_SPEAKER)?"SPK":"MIC",  p_data->volume.level);
            if (p_data->volume.type == WICED_BT_HFP_HF_MIC )
            {
                res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_VGM;
            }
            else
            {
                res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_VGS;
            }
            p_val.val.num = p_data->volume.level;
            break;

        case WICED_BT_HFP_HFP_CODEC_SET_EVT:
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BCS;
            if ( p_data->selected_codec == WICED_BT_HFP_HF_MSBC_CODEC )
                handsfree_esco_params.use_wbs = WICED_TRUE;
            else
                handsfree_esco_params.use_wbs = WICED_FALSE;
            p_val.val.num = p_data->selected_codec;


            if (handsfree_ctxt_data.init_sco_conn == WICED_TRUE)
            {
                /* timer started here to check if the sco has been created as an acceptor*/
                wiced_start_timer(&handsfree_app_states.hfp_timer,SCO_CONNECTION_WAIT_TIMEOUT);

                handsfree_ctxt_data.init_sco_conn = WICED_FALSE;
            }
#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1) || defined(CYW55500A1)
            WICED_BT_TRACE("%s - CODEC_SET: %d\n", __func__, p_data->selected_codec);
            if ( p_data->selected_codec == WICED_BT_HFP_HF_MSBC_CODEC ) {
                handsfree_esco_params.use_wbs = WICED_TRUE;
                audio_config.sr = 16000;
            }
            else {
                handsfree_esco_params.use_wbs = WICED_FALSE;
                audio_config.sr = 8000;
            }

            audio_config.channels =  1;
            audio_config.bits_per_sample = DEFAULT_BITSPSAM;
            audio_config.volume = AM_VOL_LEVEL_HIGH-2;
            if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                stream_id = wiced_am_stream_open(HFP);
            }

            if( WICED_SUCCESS != wiced_am_stream_set_param(stream_id,AM_AUDIO_CONFIG, &audio_config))
                WICED_BT_TRACE("wiced_am_set_param failed\n");
#endif
            break;

        case WICED_BT_HFP_HFP_ACTIVE_CALL_EVT:
            handsfree_send_clcc_evt(p_data->handle,&p_data->active_call,&p_val.val);
            break;

        case WICED_BT_HFP_HF_CNUM_EVT:
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CNUM;
            memcpy(p_val.val.str, p_data->cnum_data, strlen(p_data->cnum_data));
            break;

        case WICED_BT_HFP_HF_BIND_EVT:
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BIND;
            p_val.val.str[0] = p_data->bind_data.ind_id + '0';
            p_val.val.str[1] = ',';
            p_val.val.str[2] = p_data->bind_data.ind_value + '0';
            break;

        default:
            break;
    }
    if ( res && (res <= (HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_MAX)) )
    {
        wiced_bt_hfp_hf_scb_t    *p_scb = wiced_bt_hfp_hf_get_scb_by_handle(p_data->handle);
        hci_control_send_hf_event( res, p_scb->rfcomm_handle, (hci_control_hf_event_t *)&p_val );
    }
}

void handsfree_init_context_data(void)
{
    handsfree_ctxt_data.call_active         = 0;
    handsfree_ctxt_data.call_held           = 0;
    handsfree_ctxt_data.call_setup          = WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE;
    handsfree_ctxt_data.connection_status   = WICED_BT_HFP_HF_STATE_DISCONNECTED;
    handsfree_ctxt_data.spkr_volume         = 8;
    handsfree_ctxt_data.mic_volume          = 8;
    handsfree_ctxt_data.sco_index           = BT_AUDIO_INVALID_SCO_INDEX;
    handsfree_ctxt_data.init_sco_conn       = WICED_FALSE;
}

wiced_bt_voice_path_setup_t handsfree_sco_path = {
#ifdef CYW20706A2
    .path = WICED_BT_SCO_OVER_I2SPCM,
#else
    .path = WICED_BT_SCO_OVER_PCM,
#endif
#if defined(CYW20721B2) || defined (CYW43012C0) || defined(CYW55572A1) || defined(CYW55500A1)
    .p_sco_data_cb = NULL
#endif
};

#include "wiced_hal_pcm.h"

#define WICED_HAL_MXTDM0_I2S_MODE        (2)
#define WICED_HAL_MXTDM0_PCM_MODE        (3)
#define WICED_HAL_MXTDM1_I2S_MODE        (4)
#define WICED_HAL_MXTDM1_PCM_MODE        (5)
#define HCI_OPCODE_WRITE_SCO_TS                             (0xFC22)
#define HCI_OPCODE_WRITE_SCO_PCM_INT_PARAMS                 (0xFC1C)

void handsfree_hfp_init(void)
{
    wiced_result_t result = WICED_BT_ERROR;
    wiced_bt_hfp_hf_config_data_t config;

    handsfree_init_context_data();

    /* Perform the rfcomm init before hf and spp start up */

    config.feature_mask     = BT_AUDIO_HFP_SUPPORTED_FEATURES;
    config.speaker_volume   = handsfree_ctxt_data.spkr_volume;
    config.mic_volume       = handsfree_ctxt_data.mic_volume;
#ifdef WICED_ENABLE_BT_HSP_PROFILE
    config.num_server       = 2;
#else
    config.num_server       = 1;
#endif
    config.scn[0]           = HANDS_FREE_SCN;
    config.uuid[0]          = UUID_SERVCLASS_HF_HANDSFREE;
#ifdef WICED_ENABLE_BT_HSP_PROFILE
    config.scn[1]           = HEADSET_SCN;
    config.uuid[1]          = UUID_SERVCLASS_HEADSET;
#endif

    result = wiced_bt_hfp_hf_init(&config, handsfree_event_callback);
    WICED_BT_TRACE("[%s] SCO Setting up voice path = %d\n",__func__, result);

    // set the MXTDM1 I2S Mode
    wiced_hal_pcm_config_t wiced_hal_pcm_params =
    {
        WICED_HAL_MXTDM1_I2S_MODE,
        WICED_HAL_PCM_MASTER,
        // Below settings are not applicable for I2S MODE
        {
            WICED_HAL_PCM_MSB_FIRST,
            0,
            WICED_HAL_PCM_FILL_0S,
            3,
            WICED_HAL_PCM_DISABLE_RIGHT_JUSTIFY,
            WICED_HAL_PCM_FRAME_TYPE_SHORT
        }
    };

    wiced_hal_set_pcm_config(&wiced_hal_pcm_params);
#if 0
    // Set channle number for SCO
    uint8_t channel_num = 1;
    wiced_bt_dev_vendor_specific_command(HCI_OPCODE_WRITE_SCO_TS, sizeof(channel_num),&channel_num,NULL);

#define MXTDM_0                         0
#define MXTDM_1                         1

#define MXTDM_TDM                       0
#define MXTDM_I2S                       1
    uint8_t pcm_int_params[] = {0xFF, MXTDM_1,0, MXTDM_TDM, 1 }; // sub_command (0xFF), pair (MXTDM0->0 or 1->1), channel (0 to 7), mode (I2S/TDM), enable(0/1)
    wiced_bt_dev_vendor_specific_command(HCI_OPCODE_WRITE_SCO_PCM_INT_PARAMS, sizeof(pcm_int_params),pcm_int_params,NULL);
#endif
}

void handsfree_write_eir()
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;

    pBuf = (uint8_t*)wiced_bt_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE );
    WICED_BT_TRACE( "hci_control_write_eir %x\n", pBuf );

    if ( !pBuf )
    {
        return;
    }
    p = pBuf;

    //p = ( uint8_t * )( pBuf + 1 );
    //p += 4;

    length = strlen( (char *)lepl_cfg_settings.device_name );

    *p++ = length + 1;
    *p++ = 0x09;            // EIR type full name
    memcpy( p, lepl_cfg_settings.device_name, length );
    p += length;
    *p++ = ( 1 * 2 ) + 1;     // length of services + 1
    *p++ =   0x02;            // EIR type full list of 16 bit service UUIDs
#ifdef WICED_ENABLE_BT_HSP_PROFILE
    *p++ =   UUID_SERVCLASS_HEADSET         & 0xff;
    *p++ = ( UUID_SERVCLASS_HEADSET >> 8 ) & 0xff;
#endif
    *p++ =   UUID_SERVCLASS_HF_HANDSFREE        & 0xff;
    *p++ = ( UUID_SERVCLASS_HF_HANDSFREE >> 8 ) & 0xff;
    *p++ =   UUID_SERVCLASS_GENERIC_AUDIO        & 0xff;
    *p++ = ( UUID_SERVCLASS_GENERIC_AUDIO >> 8 ) & 0xff;
    *p++ = 0;

    // print EIR data
    WICED_BT_TRACE_ARRAY( ( uint8_t* )( pBuf+1 ), MIN( p-( uint8_t* )pBuf,100 ), "EIR :" );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    return;
}

extern void hci_control_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
void handsfree_post_bt_init(wiced_bt_management_evt_data_t *p_event_data)
{
    if(p_event_data->enabled.status == WICED_BT_SUCCESS)
    {
        //disable pairing
        wiced_bt_set_pairable_mode(0,0);

        WICED_BT_TRACE("Bluetooth stack initialized\n");

        handsfree_app_states.pairing_allowed = WICED_FALSE;
        wiced_init_timer( &handsfree_app_states.hfp_timer, hfp_timer_expiry_handler, 0,
                        WICED_MILLI_SECONDS_TIMER );

        /* Set-up EIR data */
        handsfree_write_eir();
        /* Set-up SDP database */
        wiced_bt_sdp_db_init((uint8_t *)handsfree_sdp_db, sizeof(handsfree_sdp_db));

        handsfree_hfp_init();


        wiced_bt_sco_setup_voice_path(&handsfree_sco_path);

        wiced_am_init();
        stream_id = wiced_am_stream_open(HFP);
        if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
        {
            WICED_BT_TRACE("wiced_am_stream_open failed\n");
        }
        else
        {
            if (wiced_am_stream_close(stream_id) != WICED_SUCCESS)
            {
                WICED_BT_TRACE("Err: wiced_am_stream_close\n");
            }
            else
            {
                WICED_BT_TRACE("Init external codec done\n");
            }
            stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
        }
    }
    else
    {
        WICED_BT_TRACE("Bluetooth stack initialization failure!!\n");
        return;
    }
}

#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1) || defined(CYW55500A1)
static int32_t handsfree_utils_hfp_volume_to_am_volume(int32_t vol)
{
    uint32_t remainder;
    int32_t am_level;

    am_level    = (vol * AM_VOL_LEVEL_HIGH) / HFP_VOLUME_HIGH;
    remainder   = (vol * AM_VOL_LEVEL_HIGH) % HFP_VOLUME_HIGH;

    if (remainder >= AM_VOL_LEVEL_HIGH)
    {
        am_level++;
    }

    return am_level;
}
#endif

/*
 * Process SCO management callback
 */
void hf_sco_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_hfp_hf_scb_t *p_scb = wiced_bt_hfp_hf_get_scb_by_bd_addr (handsfree_ctxt_data.peer_bd_addr);
    int status;

    WICED_BT_TRACE("hf_sco_management_callback: event=%d\n", event);

    switch ( event )
    {
        case BTM_SCO_CONNECTED_EVT:             /**< SCO connected event. Event data: #wiced_bt_sco_connected_t */
#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1) || defined(CYW55500A1)
            /* setup audio path */
            if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                stream_id = wiced_am_stream_open(HFP);
                WICED_BT_TRACE("wiced_am_stream_open completed stream_id: %d\n", stream_id);
            }

            /* Set sample rate. */
            if (handsfree_esco_params.use_wbs == WICED_TRUE)
            {
                audio_config.sr = AM_PLAYBACK_SR_16K;
            }
            else
            {
                audio_config.sr = AM_PLAYBACK_SR_8K;
            }

            audio_config.volume = handsfree_utils_hfp_volume_to_am_volume(AM_VOL_LEVEL_HIGH - 2);
            audio_config.mic_gain = handsfree_utils_hfp_volume_to_am_volume(AM_VOL_LEVEL_HIGH - 2);

            if( WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_AUDIO_CONFIG, &audio_config))
                WICED_BT_TRACE("wiced_am_set_param failed\n");

            if( WICED_SUCCESS != wiced_am_stream_start(stream_id))
                WICED_BT_TRACE("wiced_am_stream_start failed stream_id : %d \n", stream_id);

            /* Set speaker volume and MIC gain to make the volume consistency between call
             * sessions. */
            if (WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_SPEAKER_VOL_LEVEL, (void *) &audio_config.volume))
                WICED_BT_TRACE("wiced_am_set_param failed\n");

            if (WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_MIC_GAIN_LEVEL, (void *) &audio_config.mic_gain))
                WICED_BT_TRACE("wiced_am_set_param failed\n");
#endif

            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_AUDIO_OPEN, p_scb->rfcomm_handle, NULL );
            WICED_BT_TRACE("%s: SCO Audio connected, sco_index = %d [in context sco index=%d]\n", __func__, p_event_data->sco_connected.sco_index, handsfree_ctxt_data.sco_index);
            handsfree_ctxt_data.is_sco_connected = WICED_TRUE;

            break;

        case BTM_SCO_DISCONNECTED_EVT:          /**< SCO disconnected event. Event data: #wiced_bt_sco_disconnected_t */
#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1)  || defined(CYW55500A1)
            if (stream_id != WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                if( WICED_SUCCESS != wiced_am_stream_stop(stream_id))
                    WICED_BT_TRACE("wiced_am_stream_stop failed stream_id : %d \n", stream_id);

                if( WICED_SUCCESS != wiced_am_stream_close(stream_id))
                    WICED_BT_TRACE("wiced_am_stream_close failed stream_id : %d \n", stream_id);

                stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
            }
#endif
            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_AUDIO_CLOSE, p_scb->rfcomm_handle, NULL );
            WICED_BT_TRACE("%s: SCO disconnection change event handler\n", __func__);

            status = wiced_bt_sco_create_as_acceptor(&handsfree_ctxt_data.sco_index);
            WICED_BT_TRACE("%s: status [%d] SCO INDEX [%d] \n", __func__, status, handsfree_ctxt_data.sco_index);
            handsfree_ctxt_data.is_sco_connected = WICED_FALSE;
            break;

        case BTM_SCO_CONNECTION_REQUEST_EVT:    /**< SCO connection request event. Event data: #wiced_bt_sco_connection_request_t */
            WICED_BT_TRACE("%s: SCO connection request event handler \n", __func__);

            if( wiced_is_timer_in_use(&handsfree_app_states.hfp_timer) )
            {
                wiced_stop_timer(&handsfree_app_states.hfp_timer);
            }

            if(handsfree_app_states.connect.profile_selected == WICED_BT_HFP_PROFILE)
            {
                wiced_bt_sco_accept_connection(p_event_data->sco_connection_request.sco_index, HCI_SUCCESS, (wiced_bt_sco_params_t *) &handsfree_esco_params);
            }
#ifdef WICED_ENABLE_BT_HSP_PROFILE
            else
            {
                wiced_bt_sco_accept_connection(p_event_data->sco_connection_request.sco_index, HCI_SUCCESS, (wiced_bt_sco_params_t *) &headset_sco_params);
            }
#endif
            break;

        case BTM_SCO_CONNECTION_CHANGE_EVT:     /**< SCO connection change event. Event data: #wiced_bt_sco_connection_change_t */
            WICED_BT_TRACE("%s: SCO connection change event handler\n", __func__);
            break;
    }
    UNUSED_VARIABLE(status);
}

static void hfp_timer_expiry_handler( TIMER_PARAM_TYPE param )
{
    /* if sco is not created as an acceptor then remove the sco and create it as initiator. */
    if( handsfree_ctxt_data.call_active && !handsfree_ctxt_data.is_sco_connected )
    {
        wiced_bt_sco_remove( handsfree_ctxt_data.sco_index );
        wiced_bt_sco_create_as_initiator( handsfree_ctxt_data.peer_bd_addr, &handsfree_ctxt_data.sco_index, (wiced_bt_sco_params_t *) &handsfree_esco_params );
    }
}

/*
 * Write NVRAM function is called to store information in the NVRAM.
 */
int handsfree_write_nvram( int nvram_id, int data_len, void *p_data)
{
    wiced_result_t  result;
    int             bytes_written = wiced_hal_write_nvram( nvram_id, data_len, (uint8_t*)p_data, &result );

    WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\n", nvram_id, bytes_written, result);
    return (bytes_written);
}

/*
 * Read data from the NVRAM and return in the passed buffer
 */
int handsfree_read_nvram( int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;
    wiced_result_t  result;

    if (data_len >= sizeof(wiced_bt_device_link_keys_t))
    {
        read_bytes = wiced_hal_read_nvram( nvram_id, sizeof(wiced_bt_device_link_keys_t), p_data, &result );
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n", nvram_id, sizeof(wiced_bt_device_link_keys_t), read_bytes, result );
    }
    return (read_bytes);
}

/*
* transfer command status event to UART
*/
void handsfree_send_command_status_evt( uint16_t code, uint8_t status )
{
    wiced_transport_send_data( code, &status, 1 );
}

/*
 *  Handle Inquiry result callback from teh stack, format and send event over UART
 */
void handsfree_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data )
{
    int       i;
    uint8_t   len;
    uint8_t   tx_buf[300];
    uint16_t  code;
    uint8_t   *p = tx_buf;

    if ( p_inquiry_result == NULL )
    {
        code = HCI_CONTROL_EVENT_INQUIRY_COMPLETE;
        WICED_BT_TRACE( "inquiry complete \n");
    }
    else
    {
        code = HCI_CONTROL_EVENT_INQUIRY_RESULT;
        WICED_BT_TRACE( "inquiry result %B\n", p_inquiry_result->remote_bd_addr );
        for ( i = 0; i < 6; i++ )
            *p++ = p_inquiry_result->remote_bd_addr[5 - i];
        for ( i = 0; i < 3; i++ )
            *p++ = p_inquiry_result->dev_class[2 - i];
        *p++ = p_inquiry_result->rssi;

        // currently callback does not pass the data of the adv data, need to go through the data
        // zero len in the LTV means that there is no more data
        while ( ( p_eir_data != NULL ) && ( len = *p_eir_data ) != 0 )
        {
            // In the HCI event all parameters should fit into 255 bytes
            if ( p + len + 1 > tx_buf + 255 )
            {
                WICED_BT_TRACE( "Bad data\n" );
                break;
            }
            for ( i = 0; i < len + 1; i++ )
                *p++ = *p_eir_data++;
        }
    }
    wiced_transport_send_data( code, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  Handle Inquiry command received over UART
 */
void handsfree_inquiry( uint8_t enable )
{
    wiced_result_t           result;
    wiced_bt_dev_inq_parms_t params;

    if ( enable )
    {

        memset( &params, 0, sizeof( params ) );

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = 5;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry( &params, &handsfree_inquiry_result_cback );
        WICED_BT_TRACE( "inquiry started:%d\n", result );
    }
    else
    {
        result = wiced_bt_cancel_inquiry( );
        WICED_BT_TRACE( "cancel inquiry:%d\n", result );
    }
    handsfree_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    UNUSED_VARIABLE(result);
}

/*
 *  Handle Set Visibility command received over UART
 */
void handsfree_handle_set_visibility( uint8_t discoverability, uint8_t connectability )
{
    // we cannot be discoverable and not connectable
    if ( ( ( discoverability != 0 ) && ( connectability == 0 ) ) ||
           ( discoverability > 1 ) ||
           ( connectability > 1 ) )
    {
        handsfree_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
    else
    {
        wiced_bt_dev_set_discoverability( ( discoverability != 0 ) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE ,
                                            BTM_DEFAULT_DISC_WINDOW,
                                            BTM_DEFAULT_DISC_INTERVAL);

        wiced_bt_dev_set_connectability( ( connectability != 0 ) ? WICED_TRUE : WICED_FALSE ,
                                            BTM_DEFAULT_CONN_WINDOW,
                                            BTM_DEFAULT_CONN_INTERVAL);

        handsfree_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }
}

/*
 *  Handle Set Pairability command received over UART
 */
void handsfree_handle_set_pairability ( uint8_t allowed )
{
    uint8_t                   status = HCI_CONTROL_STATUS_SUCCESS;

    if ( handsfree_app_states.pairing_allowed != allowed )
    {
        handsfree_app_states.pairing_allowed = allowed;
        wiced_bt_set_pairable_mode( handsfree_app_states.pairing_allowed, 0 );
        WICED_BT_TRACE( " Set the pairing allowed to %d \n", handsfree_app_states.pairing_allowed );
    }

    handsfree_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, status );
}

#endif // HFP_ENABLED
