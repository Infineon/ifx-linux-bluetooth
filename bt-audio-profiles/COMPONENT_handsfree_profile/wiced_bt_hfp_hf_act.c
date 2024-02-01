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
 * This file contains action functions for Handsfree.
 */

#include <string.h>
#include "wiced_bt_hfp_hf_int.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "wiced_memory.h"
#if defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW20706A2)
#include "wiced_bt_event.h"
#endif

extern wiced_bool_t wiced_bt_hfp_close_rfcomm_server_port();
extern wiced_bool_t wiced_bt_hfp_open_rfcomm_server_port();


/*******************************************************************************
** Function         wiced_bt_hfp_hf_rfcomm_mgmt_cback
** Description      Handle RFCOMM management callback
*******************************************************************************/
void wiced_bt_hfp_hf_rfcomm_mgmt_cback(wiced_bt_rfcomm_result_t code, uint16_t handle)
{
    wiced_bt_hfp_rfcomm_evt_t                    msg;
    wiced_result_t            res = WICED_SUCCESS;
    wiced_bt_device_address_t bd_addr;
    wiced_bt_hfp_hf_scb_t    *p_scb = NULL;
    uint16_t                  lcid = 0;

    WICED_BTHFP_TRACE("%s: handle:0x%x, code:0x%x\n", __FUNCTION__, handle, code);

    memset(&msg, 0, sizeof(msg));
    if(code == WICED_BT_RFCOMM_SUCCESS)
    {
        /* Connection Established */
        WICED_BTHFP_TRACE("%s: Connected\n", __FUNCTION__);
        res = (wiced_result_t) wiced_bt_rfcomm_check_connection(handle, bd_addr, &lcid);
        if(res != WICED_SUCCESS)
        {
            WICED_BTHFP_ERROR("%s: Cannot retrieve connected BDA: %d\n", __FUNCTION__, res);
            return;
        }
        // search if we already have scb record
        p_scb = wiced_bt_hfp_hf_get_scb_by_handle(handle);
        if(p_scb == NULL)
        {
            // we don't have scb for this handle, so allocate
            p_scb = wiced_bt_hfp_hf_scb_alloc();
            if (p_scb == NULL)
                return;

            p_scb->rfcomm_handle = handle;
            p_scb->is_server = TRUE;

            wiced_bt_rfcomm_set_rx_fifo (handle, (char *)p_scb->rfcomm_fifo, sizeof (p_scb->rfcomm_fifo));
        }

        wiced_bt_hfp_hf_utils_bdcpy(p_scb->peer_addr, bd_addr);
        msg.hf_evt = WICED_BT_HFP_HF_RFC_CONNECT_EVT;

    } else if ( (code == WICED_BT_RFCOMM_PEER_CONNECTION_FAILED)
             || (code == WICED_BT_RFCOMM_CLOSED) )
    {
        /* Disconnected */
        WICED_BTHFP_TRACE("%s: Disconnected", __FUNCTION__);
        msg.hf_evt = WICED_BT_HFP_HF_RFC_DISCONNECT_EVT;
    }
    else
    {
        return;
    }
    msg.handle = handle;
    wiced_bt_hfp_hf_hdl_event((wiced_bt_hfp_hf_data_t *)&msg);
}

void wiced_bt_hfp_hf_rfcomm_port_tx_cmpl_cback(uint16_t handle, void* p_data)
{
    WICED_BTHFP_TRACE("wiced_bt_hfp_hf_rfcomm_port_tx_cmpl_cback()  p_data:%x ", p_data);


}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_rfcomm_data_cback
** Description      Handle RFCOMM data callback
*******************************************************************************/
void wiced_bt_hfp_hf_rfcomm_data_cback(wiced_bt_rfcomm_port_event_t code, uint16_t handle)
{
    wiced_bt_hfp_hf_scb_t    *p_scb = NULL;
    wiced_bt_hfp_rfcomm_evt_t   rfc_data;
    static char buff[256];
    uint16_t    len_read;

    p_scb = wiced_bt_hfp_hf_get_scb_by_handle(handle);
    if(p_scb == NULL)
    {
        WICED_BTHFP_ERROR("%s: No SCB found for handle:%d\n", __FUNCTION__, handle);
        return;
    }

    if (code & PORT_EV_RXCHAR)
    {
        wiced_bt_rfcomm_read_data (handle, buff, 256, &len_read);

        if (len_read != 0)
        {
            rfc_data.hf_evt = WICED_BT_HFP_HF_RFC_DATA_EVT;
            rfc_data.handle = handle;
            rfc_data.p_data = buff;
            rfc_data.len = len_read;

            WICED_BTHFP_TRACE("[%s] rfcomm data callback\n", __func__);
            wiced_bt_hfp_hf_hdl_event((wiced_bt_hfp_hf_data_t*)&rfc_data);
        }
    }
    else
    {
        WICED_BTHFP_ERROR("%s: Handle: %x  Code: 0x%08x\n", __FUNCTION__, handle, code);
    }
}

/******************************************************************************
 ** Function        wiced_bt_do_sdp_again
 ** Description     Used to serialize and perform SDP again
 *****************************************************************************/
int wiced_bt_do_sdp_again (void *data)
{
    wiced_bt_hfp_hf_do_sdp( (wiced_bt_hfp_hf_scb_t *)data, NULL );
    return 0;
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_sdp_complete_cback
** Description      Service discovery complete callback.
*******************************************************************************/
void wiced_bt_hfp_hf_sdp_complete_cback(uint16_t sdp_res)
{
    wiced_bt_sdp_discovery_record_t    *p_rec = NULL;
    wiced_bt_sdp_protocol_elem_t        elem;
    wiced_bt_hfp_hf_sdp_res_t           msg;
    wiced_bt_sdp_discovery_attribute_t *p_attr = NULL;
    uint16_t                            peer_version;
    wiced_bt_hfp_hf_scb_t              *p_scb = NULL;

    memset(&msg, 0, sizeof(wiced_bt_hfp_hf_sdp_res_t));
    memset(&elem,0,sizeof(wiced_bt_sdp_protocol_elem_t));

    WICED_BTHFP_TRACE("%s status: %d\n", __FUNCTION__, sdp_res);

    if (sdp_res != WICED_BT_SDP_SUCCESS)
    {
        msg.hf_evt= WICED_BT_HFP_HF_SDP_DISC_FAIL_EVT;
        p_scb = wiced_bt_hfp_hf_get_scb_by_bd_addr(wiced_bt_hfp_hf_cb.sdp_bd_addr);
        if (p_scb == NULL)
        {
            WICED_BTHFP_ERROR("%s No SCB found\n", __FUNCTION__);
        }
        goto wiced_hfp_hf_sdp_complete;
    }

    p_scb = wiced_bt_hfp_hf_get_scb_by_bd_addr(wiced_bt_hfp_hf_cb.sdp_bd_addr);
    if (p_scb == NULL)
    {
        WICED_BTHFP_ERROR("%s No SCB found\n", __FUNCTION__);
        msg.hf_evt= WICED_BT_HFP_HF_SDP_DISC_FAIL_EVT;

        wiced_bt_hfp_hf_cb.ag_profile_uuid = 0;
        wiced_bt_hfp_hf_hdl_event((wiced_bt_hfp_hf_data_t*)&msg);
        return;
    }

    /* Check if service is available */
    if ((p_rec = wiced_bt_sdp_find_service_in_db(p_scb->p_sdp_db,
            wiced_bt_hfp_hf_cb.ag_profile_uuid, p_rec)) == NULL)
   {
       msg.hf_evt = WICED_BT_HFP_HF_SDP_DISC_FAIL_EVT;
       goto wiced_hfp_hf_sdp_complete;
   }

    msg.hf_evt = WICED_BT_HFP_HF_SDP_DISC_OK_EVT;

    /* Get scn */
    if (wiced_bt_sdp_find_protocol_list_elem_in_rec(p_rec, UUID_PROTOCOL_RFCOMM, &elem) == TRUE)
    {
        msg.peer_scn = (uint8_t) elem.params[0];
        WICED_BTHFP_TRACE("%s: peer scn: 0x%x\n", __FUNCTION__, msg.peer_scn);
    }

    if( wiced_bt_hfp_hf_cb.ag_profile_uuid == UUID_SERVCLASS_AG_HANDSFREE )
    {
        WICED_BTHFP_TRACE("[%s] do this for HFP only\n",__func__);

        /* Get supported features */
        if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_SUPPORTED_FEATURES)) != NULL)
        {
            msg.peer_feature_mask = p_attr->attr_value.v.u16;
            WICED_BTHFP_TRACE("%s: peer supported feature mask: 0x%04x\n", __FUNCTION__,
                msg.peer_feature_mask);
        }
        /* Get profile version */
        if (wiced_bt_sdp_find_profile_version_in_rec(p_rec, UUID_SERVCLASS_HF_HANDSFREE,
                &peer_version) == TRUE)
        {
            msg.peer_version = peer_version;
            WICED_BTHFP_TRACE("%s: peer hf version: 0x%04x\n", __FUNCTION__, msg.peer_version);
        }
    }

wiced_hfp_hf_sdp_complete:
    if( msg.hf_evt == WICED_BT_HFP_HF_SDP_DISC_FAIL_EVT )
    {
#ifdef WICED_ENABLE_BT_HSP_PROFILE
        if( wiced_bt_hfp_hf_cb.ag_profile_uuid == UUID_SERVCLASS_AG_HANDSFREE )
        {
            wiced_bt_hfp_hf_cb.ag_profile_uuid = UUID_SERVCLASS_HEADSET_AUDIO_GATEWAY;
            memset(p_scb->p_sdp_db,0,WICED_BT_HFP_HF_DISC_BUF_SIZE);
            wiced_app_event_serialize(wiced_bt_do_sdp_again,(void *)p_scb);
        }
        else
        {
            /* If both Profiles are not found */
            wiced_bt_hfp_hf_cb.ag_profile_uuid = 0;
            wiced_bt_hfp_hf_hdl_event((wiced_bt_hfp_rfcomm_evt_t*)&msg);
        }
#else   // WICED_ENABLE_BT_HSP_PROFILE
        wiced_bt_hfp_hf_cb.ag_profile_uuid = 0;
        wiced_bt_hfp_hf_hdl_event((wiced_bt_hfp_hf_data_t*)&msg);
#endif  // WICED_ENABLE_BT_HSP_PROFILE
    }
    else
        wiced_bt_hfp_hf_hdl_event((wiced_bt_hfp_hf_data_t*)&msg);

    if((p_scb != NULL) && (p_scb->p_sdp_db != NULL))
    {
        wiced_bt_free_buffer(p_scb->p_sdp_db);
        p_scb->p_sdp_db = NULL;
    }
}


/*******************************************************************************
** Function         wiced_bt_hfp_hf_do_sdp
** Description      Perform SDP.
*******************************************************************************/
void wiced_bt_hfp_hf_do_sdp(wiced_bt_hfp_hf_scb_t *p_scb,
        wiced_bt_hfp_hf_data_t *p_data)
{
    uint16_t                    sdp_res   = WICED_BT_SDP_GENERIC_ERROR;
    wiced_bool_t                ret = FALSE;
    wiced_bt_uuid_t             uuid_list;
    uint16_t                    attr_list[] = {ATTR_ID_SERVICE_CLASS_ID_LIST,
                                               ATTR_ID_PROTOCOL_DESC_LIST,
                                               ATTR_ID_BT_PROFILE_DESC_LIST,
                                               ATTR_ID_SUPPORTED_FEATURES};
    WICED_BTHFP_TRACE("%s\n", __FUNCTION__);

    /* Save peer bd_addr in sdp_bd_addr */
    wiced_bt_hfp_hf_utils_bdcpy(wiced_bt_hfp_hf_cb.sdp_bd_addr, p_scb->peer_addr);

    uuid_list.len       = LEN_UUID_16;
    uuid_list.uu.uuid16 = wiced_bt_hfp_hf_cb.ag_profile_uuid;

    if (p_scb->p_sdp_db == NULL)
    {
        p_scb->p_sdp_db = (wiced_bt_sdp_discovery_db_t *)
        wiced_bt_get_buffer(WICED_BT_HFP_HF_DISC_BUF_SIZE);
    }

    if (p_scb->p_sdp_db == NULL)
    {
        sdp_res = WICED_BT_SDP_NO_RESOURCES;
        goto wiced_bt_hfp_hf_sdp_error;
    }

    ret = wiced_bt_sdp_init_discovery_db (p_scb->p_sdp_db, WICED_BT_HFP_HF_DISC_BUF_SIZE,
            1,&uuid_list,
            sizeof(attr_list)/sizeof(attr_list[0]), attr_list);
    if(ret == FALSE)
    {
        sdp_res = WICED_BT_SDP_GENERIC_ERROR;
        goto wiced_bt_hfp_hf_sdp_error;
    }

    ret = wiced_bt_sdp_service_search_attribute_request(wiced_bt_hfp_hf_cb.sdp_bd_addr,
            p_scb->p_sdp_db, wiced_bt_hfp_hf_sdp_complete_cback);

wiced_bt_hfp_hf_sdp_error:
    if(ret == FALSE)
    {
        wiced_bt_hfp_hf_sdp_complete_cback(sdp_res);
    }
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_sdp_failed
** Description      Service discovery failed.
*******************************************************************************/
void wiced_bt_hfp_hf_sdp_failed(wiced_bt_hfp_hf_scb_t *p_scb,
        wiced_bt_hfp_hf_data_t *p_data)
{
    wiced_bt_hfp_hf_event_data_t connect_fail;

    WICED_BTHFP_TRACE("%s\n", __FUNCTION__);

    wiced_bt_hfp_hf_utils_bdcpy(connect_fail.conn_data.remote_address, p_scb->peer_addr);
    connect_fail.conn_data.conn_state = WICED_BT_HFP_HF_STATE_DISCONNECTED;
    connect_fail.handle = p_scb->rfcomm_handle;

    /* Deallocate scb */
    wiced_bt_hfp_hf_scb_dealloc(p_scb);
    (*wiced_bt_hfp_hf_cb.p_event_cback)(WICED_BT_HFP_HF_CONNECTION_STATE_EVT,
        (wiced_bt_hfp_hf_event_data_t*)&connect_fail);
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_rfc_connect_req
** Description      Issue RFCOMM Connection to peer device
*******************************************************************************/
void wiced_bt_hfp_hf_rfc_connect_req(wiced_bt_hfp_hf_scb_t *p_scb,
        wiced_bt_hfp_hf_data_t *p_data)
{
    wiced_bt_hfp_hf_connect_fail_t connect_fail;
    wiced_bt_rfcomm_result_t       res = WICED_BT_RFCOMM_SUCCESS;
    uint16_t                       handle = 0;
    wiced_bt_hfp_hf_sdp_res_t     *sdp_res = (wiced_bt_hfp_hf_sdp_res_t *)p_data;
    uint16_t uuid;

    WICED_BTHFP_TRACE("%s to scn:%d\n", __FUNCTION__, sdp_res->peer_scn);

    // In starting we are opening all ports as server, so here we will close one available port and reopen that as client
    if (!wiced_bt_hfp_close_rfcomm_server_port())
    {
        WICED_BTHFP_ERROR("%s: Failed to close rfcomm server port \n", __FUNCTION__);
        goto connect_req_fail;
    }

    uuid = ( wiced_bt_hfp_hf_cb.ag_profile_uuid == UUID_SERVCLASS_AG_HANDSFREE ) ? UUID_SERVCLASS_HF_HANDSFREE : UUID_SERVCLASS_HEADSET;

    res = wiced_bt_rfcomm_create_connection(uuid,
        sdp_res->peer_scn, FALSE, WICED_BT_HFP_HF_MTU, p_scb->peer_addr, &handle,
        wiced_bt_hfp_hf_rfcomm_mgmt_cback);

    WICED_BTHFP_TRACE("[%s][create connection for %x result = %d]\n",__func__, uuid, res);

    if(res != WICED_BT_RFCOMM_SUCCESS)
    {
        WICED_BTHFP_ERROR("%s: Failed to connect\n", __FUNCTION__);
        goto connect_req_fail;
    }

    res = wiced_bt_rfcomm_set_event_callback(handle,
        wiced_bt_hfp_hf_rfcomm_data_cback, wiced_bt_hfp_hf_rfcomm_port_tx_cmpl_cback);
    if(res != WICED_BT_RFCOMM_SUCCESS)
    {
        wiced_bt_rfcomm_remove_connection(handle, TRUE);
        WICED_BTHFP_ERROR("%s: Failed to set RFCOMM data callback\n", __FUNCTION__);
        goto connect_req_fail;
    }

    res = wiced_bt_rfcomm_set_event_mask(handle, WICED_BT_HFP_HF_PORT_EV_MASK);
    if(res != WICED_BT_RFCOMM_SUCCESS)
    {
        wiced_bt_rfcomm_remove_connection(handle, TRUE);
        WICED_BTHFP_ERROR("%s: Failed to set RFCOMM mask\n", __FUNCTION__);
        goto connect_req_fail;
    }

    res = wiced_bt_rfcomm_set_rx_fifo (handle, (char *)p_scb->rfcomm_fifo,  sizeof (p_scb->rfcomm_fifo));
    if (res != WICED_BT_RFCOMM_SUCCESS)
    {
        wiced_bt_rfcomm_remove_connection(handle, TRUE);
        WICED_BTHFP_ERROR("%s: Failed to set RFCOMM FIFO\n", __FUNCTION__);
        goto connect_req_fail;
    }

    /* Set the handle in SCB */
    p_scb->rfcomm_handle  = handle;
    p_scb->is_server = FALSE;

    WICED_BTHFP_TRACE("%s: scb:%p, handle:0x%x is_server:%d\n", __FUNCTION__,
        p_scb, p_scb->rfcomm_handle, p_scb->is_server);

    return;

connect_req_fail:
    connect_fail.hf_evt = WICED_BT_HFP_HF_RFC_CONNECT_FAIL_EVT;
    wiced_bt_hfp_hf_utils_bdcpy(connect_fail.bd_address, p_scb->peer_addr);
    wiced_bt_hfp_hf_hdl_event((wiced_bt_hfp_hf_data_t*)&connect_fail);
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_rfc_connection_fail
** Description      RFCOMM connection to peer failed
*******************************************************************************/
void wiced_bt_hfp_hf_rfc_connection_fail(wiced_bt_hfp_hf_scb_t *p_scb,
        wiced_bt_hfp_hf_data_t *p_data)
{
    wiced_bt_hfp_hf_event_data_t connect_fail;

    WICED_BTHFP_TRACE("%s\n", __FUNCTION__);

    wiced_bt_hfp_hf_utils_bdcpy(connect_fail.conn_data.remote_address, p_scb->peer_addr);
    connect_fail.conn_data.conn_state = WICED_BT_HFP_HF_STATE_DISCONNECTED;
    connect_fail.handle = p_scb->rfcomm_handle;

    /* Port is closed, so re-open as server */
    if (!p_scb->is_server)
        wiced_bt_hfp_open_rfcomm_server_port();

    /* Deallocate scb */
    wiced_bt_hfp_hf_scb_dealloc(p_scb);
    (*wiced_bt_hfp_hf_cb.p_event_cback)(WICED_BT_HFP_HF_CONNECTION_STATE_EVT,
        (wiced_bt_hfp_hf_event_data_t*)&connect_fail);
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_rfc_connected
** Description      RFCOMM connection complete
*******************************************************************************/
void wiced_bt_hfp_hf_rfc_connected(wiced_bt_hfp_hf_scb_t *p_scb,
    wiced_bt_hfp_hf_data_t *p_data)
{
    wiced_bt_hfp_hf_event_data_t connect;
    uint16_t uuid;

    if( p_scb->is_server)
    {
        uuid = wiced_bt_hfp_hf_get_uuid_by_handle(p_scb->rfcomm_handle);
    }
    else
    {
        uuid = ( wiced_bt_hfp_hf_cb.ag_profile_uuid == UUID_SERVCLASS_AG_HANDSFREE ) ? UUID_SERVCLASS_HF_HANDSFREE : UUID_SERVCLASS_HEADSET;
    }

    /* Set up AT command interpreter */
    p_scb->at_cb.p_user = p_scb;
    p_scb->at_cb.res_max_len = WICED_BT_HFP_HF_AT_MAX_LEN;
    wiced_bt_hfp_hf_at_init(&p_scb->at_cb);

    /* Set up scb with global settings */
    p_scb->feature_mask = wiced_bt_hfp_hf_cb.config_data.feature_mask;
    p_scb->mic_volume = wiced_bt_hfp_hf_cb.config_data.mic_volume;
    p_scb->speaker_volume = wiced_bt_hfp_hf_cb.config_data.speaker_volume;

    wiced_bt_hfp_hf_utils_bdcpy(connect.conn_data.remote_address, p_scb->peer_addr);
    connect.conn_data.conn_state = WICED_BT_HFP_HF_STATE_CONNECTED;
    connect.handle = p_scb->rfcomm_handle;

    if( uuid == UUID_SERVCLASS_HEADSET  )
    {
        connect.conn_data.connected_profile = WICED_BT_HSP_PROFILE;
    }
    else
    {
        connect.conn_data.connected_profile = WICED_BT_HFP_PROFILE;
    }

    (*wiced_bt_hfp_hf_cb.p_event_cback)(WICED_BT_HFP_HF_CONNECTION_STATE_EVT,
        (wiced_bt_hfp_hf_event_data_t*)&connect);


    /* TODO: Start timer for service level conn */
    // bta_sys_start_timer(&p_scb->act_timer, BTA_HS_SVC_TOUT_EVT, p_bta_hs_cfg->conn_tout);

    WICED_BTHFP_TRACE("%s\n", __FUNCTION__);

    WICED_BTHFP_TRACE("\t        Feature                                      Supported \n");
    WICED_BTHFP_TRACE("\t WICED_BT_HFP_HF_FEATURE_ECNR                            %d \n", (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_ECNR) ? 1 : 0 );
    WICED_BTHFP_TRACE("\t WICED_BT_HFP_HF_FEATURE_3WAY_CALLING                    %d \n", (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_3WAY_CALLING) ? 1 : 0 );
    WICED_BTHFP_TRACE("\t WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY                 %d \n", (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY) ? 1 : 0 );
    WICED_BTHFP_TRACE("\t WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION    %d \n", (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION) ? 1 : 0 );
    WICED_BTHFP_TRACE("\t WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL           %d \n", (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL) ? 1 : 0 );
    WICED_BTHFP_TRACE("\t WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_STATUS            %d \n", (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_STATUS) ? 1 : 0 );
    WICED_BTHFP_TRACE("\t WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_CONTROL           %d \n", (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_CONTROL) ? 1 : 0 );
    WICED_BTHFP_TRACE("\t WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION               %d \n", (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION) ? 1 : 0 );
    WICED_BTHFP_TRACE("\t WICED_BT_HFP_HF_FEATURE_HF_INDICATORS                   %d \n", (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_HF_INDICATORS) ? 1 : 0 );
    WICED_BTHFP_TRACE("\t WICED_BT_HFP_HF_FEATURE_ESCO_S4_SETTINGS_SUPPORT     %d \n", (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_ESCO_S4_SETTINGS_SUPPORT) ? 1 : 0 );
    WICED_BTHFP_TRACE("\t WICED_BT_HFP_HF_FEATURE_ENHANCED_VOICE_RECOGNITION      %d \n", (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_ENHANCED_VOICE_RECOGNITION) ? 1 : 0 );

    if( uuid == UUID_SERVCLASS_HEADSET )
    {
        /* send initial set of HSP commands */
        WICED_BTHFP_TRACE("[%s] HSP profile handling initial set of commands\n",__func__);

        wiced_bt_hsp_at_slc(p_scb);
    }
    else
    {
        /* send initial set of HFP commands */
        wiced_bt_hfp_hf_at_slc(p_scb);
    }
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_rfc_disconnect_req
** Description      Issue RFCOMM disconnection to peer connected device
*******************************************************************************/
void wiced_bt_hfp_hf_rfc_disconnect_req(wiced_bt_hfp_hf_scb_t *p_scb,
    wiced_bt_hfp_hf_data_t *p_data)
{
    wiced_bt_rfcomm_result_t res = WICED_BT_RFCOMM_SUCCESS;
    WICED_BTHFP_TRACE("%s\n", __FUNCTION__);
    res = wiced_bt_rfcomm_remove_connection (p_scb->rfcomm_handle, FALSE);
    if(res != WICED_BT_RFCOMM_SUCCESS)
    {
        WICED_BTHFP_ERROR("%s: Failed to disconnect\n", __FUNCTION__);
    }
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_rfc_disconnected
** Description      RFCOMM disconnection complete.
*******************************************************************************/
void wiced_bt_hfp_hf_rfc_disconnected(wiced_bt_hfp_hf_scb_t *p_scb,
    wiced_bt_hfp_hf_data_t *p_data)
{
    wiced_bt_hfp_hf_event_data_t disconnect;

    /* Reset the AT command interpreter */
    p_scb->peer_feature_mask = 0;
    p_scb->slc_open = FALSE;
    p_scb->slc_at_init_state = 0;
    p_scb->ind_string_received = WICED_FALSE;
    wiced_bt_hfp_hf_at_reinit(&p_scb->at_cb);

#if (WICED_BT_HFP_HF_VERSION >= WICED_BT_HFP_HF_VERSION_1_7 \
  && WICED_BT_HFP_HF_IND_SUPPORTED == TRUE)
    memset(&(p_scb->peer_ind), 0, sizeof(p_scb->peer_ind));
#endif

    WICED_BTHFP_TRACE("%s\n", __FUNCTION__);

    wiced_bt_hfp_hf_utils_bdcpy(disconnect.conn_data.remote_address, p_scb->peer_addr);
    disconnect.conn_data.conn_state = WICED_BT_HFP_HF_STATE_DISCONNECTED;
    disconnect.handle = p_scb->rfcomm_handle;

    /* Clear the connected bda information from SCB */
    memset(p_scb->peer_addr, 0, sizeof(wiced_bt_device_address_t));

    /* Deallocate scb */
    wiced_bt_hfp_hf_scb_dealloc(p_scb);

    // Port is closed so reopen that port as server
    if (!p_scb->is_server)
        wiced_bt_hfp_open_rfcomm_server_port();

    (*wiced_bt_hfp_hf_cb.p_event_cback)(WICED_BT_HFP_HF_CONNECTION_STATE_EVT,
        (wiced_bt_hfp_hf_event_data_t*)&disconnect);
}


/*******************************************************************************
** Function         wiced_bt_hfp_hf_rfc_data
** Description      Process data from RFCOMM
*******************************************************************************/
void wiced_bt_hfp_hf_rfc_data(wiced_bt_hfp_hf_scb_t *p_scb,
    wiced_bt_hfp_hf_data_t *p_data)
{
    void *buf = p_data->rfc_data.p_data;
    uint16_t len = p_data->rfc_data.len;

    WICED_BTHFP_TRACE("%s: len:0x%x\n", __FUNCTION__, len);

    /* Run AT command response interpreter on data */
    wiced_bt_hfp_hf_at_parse(&p_scb->at_cb, buf, len);
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_cmd_timeout
** Description      Handle API cmd timeout
*******************************************************************************/
void wiced_bt_hfp_hf_cmd_timeout(wiced_bt_hfp_hf_scb_t *p_scb,
    wiced_bt_hfp_hf_data_t *p_data)
{
    wiced_bt_hfp_hf_at_cmd_queue_timeout(p_scb);
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_slc_open
** Description      Service level connection opened
*******************************************************************************/
void wiced_bt_hfp_hf_slc_open(wiced_bt_hfp_hf_scb_t *p_scb,
    wiced_bt_hfp_hf_data_t *p_data)
{
    wiced_bt_hfp_hf_event_data_t connect;

    if (p_scb->slc_open == TRUE)
    {
        WICED_BTHFP_ERROR("%s: SLC already opened\n", __FUNCTION__);
        return;
    }

    WICED_BTHFP_TRACE("%s\n", __FUNCTION__);

    p_scb->slc_open = TRUE;

    /* Call callback */
    wiced_bt_hfp_hf_utils_bdcpy(connect.conn_data.remote_address, p_scb->peer_addr);
    connect.conn_data.conn_state = WICED_BT_HFP_HF_STATE_SLC_CONNECTED;
    connect.handle = p_scb->rfcomm_handle;

    (*wiced_bt_hfp_hf_cb.p_event_cback)(WICED_BT_HFP_HF_CONNECTION_STATE_EVT,
        (wiced_bt_hfp_hf_event_data_t*)&connect);
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_do_call_action
** Description      Perform call action
*******************************************************************************/
void wiced_bt_hfp_hf_do_call_action(wiced_bt_hfp_hf_scb_t *p_scb,
    wiced_bt_hfp_hf_data_t *p_data)
{
    wiced_bt_hfp_hf_api_call_action_t *call_action = (wiced_bt_hfp_hf_api_call_action_t *)p_data;
    WICED_BTHFP_TRACE ("%s\n", __FUNCTION__);

    switch(call_action->action)
    {
        case WICED_BT_HFP_HF_CALL_ACTION_DIAL:
        {
            int num_len = (int) strlen(call_action->number);
            if( (num_len != 0) && (num_len < WICED_BT_HFP_HF_MAX_PHONE_NUMBER_LEN) )
            {

                char buf[WICED_BT_HFP_HF_MAX_PHONE_NUMBER_LEN+1];
                memset(buf, 0, sizeof(buf));
                wiced_bt_hfp_hf_utils_strcpy(buf, call_action->number);

                if(buf[num_len - 1] != ';')
                {
                    buf[num_len] = ';';
                    buf[num_len + 1] = 0;
                }
                wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_D,
                    WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_STR, buf, 0);
            }
            else
            {
                wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_BLDN,
                    WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            }
        }
        break;
        case WICED_BT_HFP_HF_CALL_ACTION_ANSWER:
        {
            wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_A, WICED_BT_HFP_HF_AT_NONE,
                WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
        }
        break;
        case WICED_BT_HFP_HF_CALL_ACTION_HANGUP:
        {
            wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_CHUP, WICED_BT_HFP_HF_AT_NONE,
                WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
        }
        break;
        case WICED_BT_HFP_HF_CALL_ACTION_HOLD_0:
        case WICED_BT_HFP_HF_CALL_ACTION_HOLD_1:
        case WICED_BT_HFP_HF_CALL_ACTION_HOLD_2:
        case WICED_BT_HFP_HF_CALL_ACTION_HOLD_3:
        case WICED_BT_HFP_HF_CALL_ACTION_HOLD_4:
        {
            char buf[WICED_BT_HFP_HF_MAX_PHONE_NUMBER_LEN+1];
            memset(buf, 0, sizeof(buf));
            buf[0] = '0' + call_action->action-WICED_BT_HFP_HF_CALL_ACTION_HOLD_0;
            wiced_bt_hfp_hf_utils_strcpy(&buf[1], call_action->number);

            wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_CHLD, WICED_BT_HFP_HF_AT_SET,
                WICED_BT_HFP_HF_AT_FMT_STR, buf, 0);
        }
        break;
    }
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_do_notify_volume
** Description      Notify volume
*******************************************************************************/
void wiced_bt_hfp_hf_do_notify_volume(wiced_bt_hfp_hf_scb_t *p_scb,
    wiced_bt_hfp_hf_data_t *p_data)
{
    wiced_bt_hfp_hf_api_notify_vol_t *notify_vol = (wiced_bt_hfp_hf_api_notify_vol_t *)p_data;

    WICED_BTHFP_TRACE ("%s\n", __FUNCTION__);
    if(notify_vol->volume_type == WICED_BT_HFP_HF_SPEAKER)
    {
        wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_VGS, WICED_BT_HFP_HF_AT_SET,
            WICED_BT_HFP_HF_AT_FMT_INT, NULL, notify_vol->volume_level);
        p_scb->speaker_volume = notify_vol->volume_level;
    }
    else if(notify_vol->volume_type == WICED_BT_HFP_HF_MIC)
    {
        wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_VGM, WICED_BT_HFP_HF_AT_SET,
            WICED_BT_HFP_HF_AT_FMT_INT, NULL, notify_vol->volume_level);
        p_scb->mic_volume = notify_vol->volume_level;
    }
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_do_send_at_cmd
** Description      Send custom AT command
*******************************************************************************/
void wiced_bt_hfp_hf_do_send_at_cmd(wiced_bt_hfp_hf_scb_t *p_scb,
    wiced_bt_hfp_hf_data_t *p_data)
{
    wiced_bt_hfp_hf_api_send_at_cmd_t *at_cmd = (wiced_bt_hfp_hf_api_send_at_cmd_t *)p_data;

    WICED_BTHFP_TRACE ("%s\n", __FUNCTION__);
    wiced_bt_hfp_hf_at_cmd_queue_enqueue(p_scb, 0/*TODO*/,
        at_cmd->at_cmd, (uint16_t)strlen(at_cmd->at_cmd));
}
