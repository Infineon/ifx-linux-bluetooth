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
 * This file contains action functions for audio source state machine.
 */


#include <string.h>
#include "wiced_memory.h"
//#include "wiced_bt_l2c.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_a2dp_src_int.h"
#include "wiced_bt_sdp.h"
#include "wiced_timer.h"

static wiced_timer_t a2dp_src_setconfig_timer;

/* These tables translate AVDT events to state machine events */
static const uint8_t wiced_bt_a2dp_source_stream_evt_ok[] = {
    WICED_BT_A2DP_SOURCE_STR_DISCOVER_CFM_EVT,     /**< AVDT_DISCOVER_CFM_EVT */
    WICED_BT_A2DP_SOURCE_STR_GETCAP_CFM_EVT,       /**< AVDT_GETCAP_CFM_EVT */
    WICED_BT_A2DP_SOURCE_STR_OPEN_OK_EVT,          /**< AVDT_OPEN_CFM_EVT */
    WICED_BT_A2DP_SOURCE_STR_OPEN_OK_EVT,          /**< AVDT_OPEN_IND_EVT */
    WICED_BT_A2DP_SOURCE_STR_CONFIG_IND_EVT,       /**< AVDT_CONFIG_IND_EVT */
    WICED_BT_A2DP_SOURCE_STR_START_CFM_EVT,        /**< AVDT_START_CFM_EVT */
    WICED_BT_A2DP_SOURCE_STR_START_IND_EVT,        /**< AVDT_START_IND_EVT */
    WICED_BT_A2DP_SOURCE_STR_SUSPEND_CFM_EVT,      /**< AVDT_SUSPEND_CFM_EVT */
    WICED_BT_A2DP_SOURCE_STR_SUSPEND_CFM_EVT,      /**< AVDT_SUSPEND_IND_EVT */
    WICED_BT_A2DP_SOURCE_STR_CLOSE_CFM_EVT,         /**< AVDT_CLOSE_CFM_EVT */
    WICED_BT_A2DP_SOURCE_STR_CLOSE_IND_EVT,         /**< AVDT_CLOSE_IND_EVT */
    WICED_BT_A2DP_SOURCE_STR_RECONFIG_CFM_EVT,     /**< AVDT_RECONFIG_CFM_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_RECONFIG_IND_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_SECURITY_CFM_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_SECURITY_IND_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_WRITE_CFM_EVT */
    WICED_BT_A2DP_SOURCE_AVDT_CONNECT_EVT,         /**< AVDT_CONNECT_IND_EVT */
    WICED_BT_A2DP_SOURCE_AVDT_DISCONNECT_EVT,      /**< AVDT_DISCONNECT_IND_EVT */
    WICED_BT_A2DP_SOURCE_AVDT_REPOPT_CONN_EVT,     /**< AVDT_REPORT_CONN_EVT */
    WICED_BT_A2DP_SOURCE_AVDT_REPOPT_CONN_EVT,     /**< AVDT_REPORT_DISCONN_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_DELAY_REPORT_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_DELAY_REPORT_CFM_EVT */
};

static const uint8_t wiced_bt_a2dp_source_stream_evt_fail[] = {
    WICED_BT_A2DP_SOURCE_STR_DISCOVER_CFM_EVT,     /**< AVDT_DISCOVER_CFM_EVT */
    WICED_BT_A2DP_SOURCE_STR_GETCAP_CFM_EVT,       /**< AVDT_GETCAP_CFM_EVT */
    WICED_BT_A2DP_SOURCE_STR_OPEN_FAIL_EVT,        /**< AVDT_OPEN_CFM_EVT */
    WICED_BT_A2DP_SOURCE_STR_OPEN_OK_EVT,          /**< AVDT_OPEN_IND_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_CONFIG_IND_EVT */
    WICED_BT_A2DP_SOURCE_STR_START_FAIL_EVT,       /**< AVDT_START_CFM_EVT */
    WICED_BT_A2DP_SOURCE_STR_START_IND_EVT,        /**< AVDT_START_IND_EVT */
    WICED_BT_A2DP_SOURCE_STR_SUSPEND_CFM_EVT,      /**< AVDT_SUSPEND_CFM_EVT */
    WICED_BT_A2DP_SOURCE_STR_SUSPEND_CFM_EVT,      /**< AVDT_SUSPEND_IND_EVT */
    WICED_BT_A2DP_SOURCE_STR_CLOSE_CFM_EVT,         /**< AVDT_CLOSE_CFM_EVT */
    WICED_BT_A2DP_SOURCE_STR_CLOSE_IND_EVT,         /**< AVDT_CLOSE_IND_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_RECONFIG_CFM_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_RECONFIG_IND_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_SECURITY_CFM_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_SECURITY_IND_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_WRITE_CFM_EVT */
    WICED_BT_A2DP_SOURCE_AVDT_CONNECT_EVT,         /**< AVDT_CONNECT_IND_EVT */
    WICED_BT_A2DP_SOURCE_AVDT_DISCONNECT_EVT,      /**< AVDT_DISCONNECT_IND_EVT */
    WICED_BT_A2DP_SOURCE_AVDT_REPOPT_CONN_EVT,     /**< AVDT_REPORT_CONN_EVT */
    WICED_BT_A2DP_SOURCE_AVDT_REPOPT_CONN_EVT,     /**< AVDT_REPORT_DISCONN_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_DELAY_REPORT_EVT */
    WICED_BT_A2DP_SOURCE_INVALID_EVT,              /**< AVDT_DELAY_REPORT_CFM_EVT */
};

wiced_bool_t wiced_bt_a2dp_source_next_gatcap(wiced_bt_a2dp_source_ccb_t *p_ccb,
                wiced_bt_avdt_sep_info_t *p_sep_info, wiced_bt_avdt_cfg_t *avdt_sep_config );
extern wiced_bool_t wiced_bt_a2dp_src_sbc_format_check(uint8_t *peer_codec_info,
                                                       wiced_bt_a2d_sbc_cie_t supported_cap,
                                                       wiced_bt_a2d_sbc_cie_t *default_cap,
                                                       wiced_bt_a2d_sbc_cie_t *output_cap);
/*******************************************************************************
** Function         wiced_bt_a2dp_source_proc_stream_evt
** Description      Utility function to compose stream events.
*******************************************************************************/
static void wiced_bt_a2dp_source_proc_stream_evt(uint8_t handle,
    wiced_bt_device_address_t bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data)
{
    wiced_bt_a2dp_source_str_msg_t *p_msg   = NULL;
    uint16_t                      sec_len = 0;
    wiced_bt_a2dp_source_global_evt_t source_event;

    WICED_BTA2DP_SRC_TRACE ("[%s] event %d handle %d \n",__FUNCTION__, event, handle);

    if (p_data)
    {
        if (event == AVDT_SECURITY_IND_EVT)
        {
            sec_len = (p_data->security_ind.len < WICED_BT_A2DP_SOURCE_SECURITY_MAX_LEN) ?
                   p_data->security_ind.len : WICED_BT_A2DP_SOURCE_SECURITY_MAX_LEN;
        }
        else if (event == AVDT_SECURITY_CFM_EVT && p_data->hdr.err_code == 0)
        {
            sec_len = (p_data->security_cfm.len < WICED_BT_A2DP_SOURCE_SECURITY_MAX_LEN) ?
                   p_data->security_cfm.len : WICED_BT_A2DP_SOURCE_SECURITY_MAX_LEN;
        }
    }

    if ((p_msg = (wiced_bt_a2dp_source_str_msg_t *)
                 wiced_bt_get_buffer_from_heap(wiced_bt_a2dp_source_cb.heap, (uint16_t) (sizeof(wiced_bt_a2dp_source_str_msg_t) + sec_len))) != NULL)
    {
        /* Copy event data, bd addr, and handle to event message buffer */
        if (bd_addr != NULL)
        {
            wiced_bt_a2dp_source_utils_bdcpy(p_msg->bd_addr, bd_addr);
            WICED_BTA2DP_SRC_TRACE("bd_addr:%02x-%02x-%02x-%02x-%02x-%02x \n",
                          bd_addr[0], bd_addr[1],
                          bd_addr[2], bd_addr[3],
                          bd_addr[4], bd_addr[5]);
        }
        /* Copy config params to event message buffer */
        if (p_data != NULL)
        {
            WICED_BTA2DP_SRC_TRACE ("[%s] error code %d \n",__FUNCTION__,p_data->hdr.err_code);

            memcpy(&p_msg->msg, p_data, sizeof (wiced_bt_avdt_ctrl_t));

            switch (event)
            {
                case AVDT_CONFIG_IND_EVT:
                memcpy(&p_msg->cfg, p_data->config_ind.p_cfg, sizeof(wiced_bt_avdt_cfg_t));
                break;

                case AVDT_RECONFIG_CFM_EVT:
                memcpy(&p_msg->cfg, p_data->reconfig_cfm.p_cfg, sizeof(wiced_bt_avdt_cfg_t));
                break;

                case AVDT_RECONFIG_IND_EVT:
                memcpy(&p_msg->cfg, p_data->reconfig_ind.p_cfg, sizeof(wiced_bt_avdt_cfg_t));
                break;

                case AVDT_DISCOVER_CFM_EVT:
                memcpy(&p_msg->msg.discover_cfm, &p_data->discover_cfm, sizeof(wiced_bt_avdt_discover_t));
                break;

                case AVDT_GETCAP_CFM_EVT:
                memcpy(&p_msg->msg.getcap_cfm, &p_data->getcap_cfm, sizeof(wiced_bt_avdt_config_t));
                break;

                case AVDT_OPEN_CFM_EVT:
                memcpy(&p_msg->msg.open_cfm, &p_data->open_cfm, sizeof(wiced_bt_avdt_config_t));
                break;

                case AVDT_CONNECT_IND_EVT:
                memcpy(&p_msg->msg.connect_ind, &p_data->connect_ind, sizeof(wiced_bt_avdt_evt_hdr_t));
                break;

                case AVDT_SECURITY_IND_EVT:
                p_msg->msg.security_ind.p_data = (uint8_t *) (p_msg + 1);
                memcpy(p_msg->msg.security_ind.p_data,
                       p_data->security_ind.p_data, sec_len);
                break;

                case AVDT_SECURITY_CFM_EVT:
                p_msg->msg.security_cfm.p_data = (uint8_t *) (p_msg + 1);
                if (p_data->hdr.err_code == 0)
                {
                    memcpy(p_msg->msg.security_cfm.p_data,
                           p_data->security_cfm.p_data, sec_len);
                }
                break;

                case AVDT_SUSPEND_IND_EVT:
                if (p_data)
                    p_msg->msg.hdr.err_code = 0;
                break;

                default:
                break;
            }
        }
        else
            p_msg->msg.hdr.err_code = 0;

        /* Look up a2dp source event */
        if ((p_data == NULL) || (p_data->hdr.err_code == 0))
        {
            source_event = (wiced_bt_a2dp_source_global_evt_t)wiced_bt_a2dp_source_stream_evt_ok[event];
        }
        else
        {
            source_event = (wiced_bt_a2dp_source_global_evt_t)wiced_bt_a2dp_source_stream_evt_fail[event];
        }

        p_msg->handle = handle;

        wiced_bt_a2dp_source_hdl_event(source_event, (wiced_bt_avdt_evt_hdr_t *)&p_msg->hdr);

        /* Free the buffer used, since it was allocated here */
        wiced_bt_free_buffer(p_msg);
    }

    /* If signalling channel closed, then also notify so that a2dp source cleans up any open stream.
     * If sig channel was closed locally; a scb may have been left in INCOMING_ST during
     * incoming connection for SNK
     */
    if (event == AVDT_DISCONNECT_IND_EVT)
    {
        wiced_bt_avdt_ctrl_t ctrl_data;
        ctrl_data.hdr.err_param = 0; /*TODO*/
        WICED_BTA2DP_SRC_TRACE("Notifying signalling channel close (locally initiated) \n");
        wiced_bt_a2dp_source_conn_cback(handle, bd_addr, AVDT_DISCONNECT_IND_EVT, &ctrl_data);
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_ctrl_cback
** Description      This is the AVDTP callback function for stream events.
*******************************************************************************/
void wiced_bt_a2dp_source_ctrl_cback(uint8_t handle, wiced_bt_device_address_t bd_addr,
                              uint8_t event, wiced_bt_avdt_ctrl_t *p_data)
{
    wiced_bt_avdt_delay_rpt_t     *delay_rpt_cmd;
    wiced_bt_a2dp_source_event_data_t evt_data;
    WICED_BTA2DP_SRC_TRACE("%s: avdt_handle: %d event=0x%x \n", __FUNCTION__, handle, event);
    wiced_bt_a2dp_source_proc_stream_evt(handle, bd_addr, event, p_data);

    if (event == AVDT_WRITE_CFM_EVT)
    {
        (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_WRITE_CFM_EVT,
            NULL);
    }
    if (event == AVDT_DELAY_REPORT_EVT)
    {
        delay_rpt_cmd = (wiced_bt_avdt_delay_rpt_t*)p_data;
        evt_data.delay_ms = delay_rpt_cmd->delay;
        WICED_BTA2DP_SRC_TRACE("delay report cmd received, reported delay is %d", delay_rpt_cmd->delay);
        (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_DELAY_RPT_EVT,
            &evt_data);
    }
}

#if AVDT_REPORTING == TRUE
/*******************************************************************************
** Function         wiced_bt_a2dp_source_report_cback
** Description      Report callback.
*******************************************************************************/
static void wiced_bt_a2dp_source_report_cback(uint8_t handle, AVDT_REPORT_TYPE type,
                                    wiced_bt_avdt_report_data_t *p_data)
{
    /* Will be implemented later if needed. */
    WICED_BTA2DP_SRC_TRACE("%s: NOT IMPLEMENTED \n", __FUNCTION__);
}
#endif

/*******************************************************************************
** Function         wiced_bt_a2dp_source_sdp_complete_cback
** Description      Service discovery complete callback.
*******************************************************************************/
void wiced_bt_a2dp_source_sdp_complete_cback(uint16_t sdp_res)
{
    wiced_bt_sdp_discovery_record_t *p_rec = NULL;
    wiced_bt_sdp_protocol_elem_t     elem;
    wiced_bt_a2dp_source_sdp_res_t     msg;
    uint16_t                         avdt_version;
    wiced_bt_a2dp_source_global_evt_t source_event = WICED_BT_A2DP_SOURCE_SDP_DISC_FAIL_EVT;
    wiced_bt_a2dp_source_scb_t        *p_scb = wiced_bt_a2dp_source_cb.p_scb;

    memset(&msg, 0, sizeof(wiced_bt_a2dp_source_sdp_res_t));

    WICED_BTA2DP_SRC_TRACE("%s status: %d \n", __FUNCTION__, sdp_res);

    if (sdp_res != WICED_BT_SDP_SUCCESS)
    {
        source_event  = WICED_BT_A2DP_SOURCE_SDP_DISC_FAIL_EVT;
        goto wiced_a2dp_source_sdp_complete;
    }

    /* Check if service is available */
    if ((p_rec = wiced_bt_sdp_find_service_in_db(p_scb->p_sdp_db,
            UUID_SERVCLASS_AUDIO_SINK, p_rec)) == NULL)
    {
        source_event = WICED_BT_A2DP_SOURCE_SDP_DISC_FAIL_EVT;
        goto wiced_a2dp_source_sdp_complete;
    }

    /* Get AVDTP version */
    if (wiced_bt_sdp_find_protocol_list_elem_in_rec(p_rec,
        UUID_PROTOCOL_AVDTP, &elem) == WICED_TRUE)
    {
        avdt_version = elem.params[0];
        WICED_BTA2DP_SRC_TRACE("%s: avdt_version: 0x%04x \n", __FUNCTION__, avdt_version);
        source_event = WICED_BT_A2DP_SOURCE_SDP_DISC_OK_EVT;
        msg.avdt_version = avdt_version;
    }

wiced_a2dp_source_sdp_complete:
    wiced_bt_a2dp_source_hdl_event(source_event, (wiced_bt_avdt_evt_hdr_t*)&msg.hdr);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_do_sdp
** Description      Perform SDP.
*******************************************************************************/
void wiced_bt_a2dp_source_do_sdp(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    uint16_t                    sdp_res   = WICED_BT_SDP_GENERIC_ERROR;
    wiced_bt_a2dp_source_scb_t   *p_scb = p_ccb->p_scb;
    wiced_bool_t                ret = WICED_FALSE;
    wiced_bt_uuid_t             uuid_list;
    uint16_t                    attr_list[] = {ATTR_ID_SERVICE_CLASS_ID_LIST,
                                               ATTR_ID_PROTOCOL_DESC_LIST,
                                               ATTR_ID_BT_PROFILE_DESC_LIST};

    /* Save peer bd_addr in sdp_bd_addr */
    wiced_bt_a2dp_source_utils_bdcpy(wiced_bt_a2dp_source_cb.sdp_bd_addr, p_ccb->peer_addr);

    uuid_list.len       = LEN_UUID_16;
    uuid_list.uu.uuid16 = UUID_SERVCLASS_AUDIO_SINK;

    if (p_scb->p_sdp_db == NULL)
    {
        p_scb->p_sdp_db = (wiced_bt_sdp_discovery_db_t *)
            wiced_bt_get_buffer_from_heap(wiced_bt_a2dp_source_cb.heap, WICED_BT_A2DP_SOURCE_DISC_BUF_SIZE);
    }

    if (p_scb->p_sdp_db == NULL)
    {
        sdp_res = WICED_BT_SDP_NO_RESOURCES;
        goto wiced_bt_a2dp_source_sdp_error;
    }

    ret = wiced_bt_sdp_init_discovery_db (p_scb->p_sdp_db, WICED_BT_A2DP_SOURCE_DISC_BUF_SIZE,
            1,&uuid_list,
            sizeof(attr_list)/sizeof(attr_list[0]), attr_list);
    if(ret == WICED_FALSE)
    {
        sdp_res = WICED_BT_SDP_GENERIC_ERROR;
        goto wiced_bt_a2dp_source_sdp_error;
    }

    ret = wiced_bt_sdp_service_search_attribute_request(wiced_bt_a2dp_source_cb.sdp_bd_addr,
            p_scb->p_sdp_db, wiced_bt_a2dp_source_sdp_complete_cback);

wiced_bt_a2dp_source_sdp_error:
    if(ret == WICED_FALSE)
    {
        wiced_bt_a2dp_source_sdp_complete_cback(sdp_res);
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_clean_sep_record
** Description      Clean sep record
*******************************************************************************/
void wiced_bt_a2dp_source_clean_sep_record(uint16_t avdt_handle)
{
    int xx;
    for ( xx = 0; xx < WICED_BT_A2DP_SOURCE_MAX_SEPS; xx++ )
    {
        if (  wiced_bt_a2dp_source_cb.seps[xx].av_handle == avdt_handle )
        {
            wiced_bt_a2dp_source_cb.seps[xx].in_use = 0;
        }
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_cleanup
** Description      Cleanup stream control block.
*******************************************************************************/
void wiced_bt_a2dp_source_cleanup(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    int                       xx;
    wiced_bt_a2dp_source_scb_t *p_scb = NULL;

    WICED_BTA2DP_SRC_TRACE("%s, %d \n", __FUNCTION__, p_ccb->state );

    if( p_ccb->state != WICED_BT_A2DP_SOURCE_INIT_SST )
        return;

    p_scb = p_ccb->p_scb;

    if (p_scb != NULL)
    {
        /* Free any buffers */
        if(p_scb->p_cap != NULL)
        {
            memset(p_scb->p_cap,0,sizeof(wiced_bt_avdt_cfg_t)*WICED_BT_A2DP_SOURCE_NUM_SEPS);
        }

        if(p_scb->p_sdp_db != NULL)
        {
            wiced_bt_free_buffer(p_scb->p_sdp_db);
            p_scb->p_sdp_db = NULL;
        }

        p_scb->avdt_version = 0;

        /* Initialize some control block variables */
        p_scb->open_status = WICED_SUCCESS;

        /* if de-registering shut everything down */
        p_scb->started  = WICED_FALSE;
        p_scb->role = 0;
        p_scb->is_accepter = WICED_FALSE;
        p_scb->cur_psc_mask = 0;
        p_scb->recfg_ind = WICED_FALSE;

        wiced_bt_a2dp_source_clean_sep_record(p_scb->avdt_handle);

        if (p_scb->deregistring)
        {
            /* Remove stream */
            if(p_scb->avdt_handle)
                wiced_bt_avdt_remove_stream(p_scb->avdt_handle);
            p_scb->avdt_handle = 0;
            WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);
        }
    }

    for (xx = 0; xx < WICED_BT_A2DP_SOURCE_MAX_NUM_CONN; xx++)
    {
        if (wiced_bt_a2dp_source_cb.ccb[xx].state != WICED_BT_A2DP_SOURCE_INIT_SST)
            return;

        if (wiced_bt_a2dp_source_cb.ccb[xx].p_scb->avdt_handle != 0)
            return;

    }
    /*
     * should not reach here if there are still some avdt_handles
     * that are non-null otherwise, we will end up deregistering from AVDT itself.
     */
    if (p_scb->deregistring)
        wiced_bt_a2dp_source_dereg_comp();
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_free_sdb
** Description      Free service discovery db buffer.
*******************************************************************************/
void wiced_bt_a2dp_source_free_sdb(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_scb_t *p_scb = p_ccb->p_scb;
    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);

    if(p_scb->p_sdp_db != NULL)
    {
        wiced_bt_free_buffer(p_scb->p_sdp_db);
        p_scb->p_sdp_db = NULL;
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_config_ind
** Description      Handle a stream configuration indication from the peer.
*******************************************************************************/
void wiced_bt_a2dp_source_reconfig_cfm(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_avdt_config_t  *reconfig_cfm = (wiced_bt_avdt_config_t *)&p_data->str_msg.msg.reconfig_cfm;
    wiced_bt_a2dp_source_config_t cap_configured;
    WICED_BTA2DP_SRC_TRACE( "[%s]: <%B> error_code: %d\n\r", __FUNCTION__, p_ccb->peer_addr, reconfig_cfm->hdr.err_code);

    if (reconfig_cfm->hdr.err_code == AVDT_SUCCESS)
    {
        p_ccb->p_scb->recfg_ind = WICED_FALSE;
        cap_configured.result = WICED_BT_SUCCESS;
        cap_configured.cp_type = 0;
        cap_configured.codec_config = &p_ccb->p_scb->cap_configured;
        cap_configured.stream_mtu = p_ccb->p_scb->stream_mtu;
        (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_CONFIGURE_EVT,
                                              (wiced_bt_a2dp_source_event_data_t *)&cap_configured);
        // Update the configuration
       // wiced_bt_a2dp_source_set_codec_config(&p_ccb->p_scb->cap_configured,p_ccb->p_scb->avdt_handle,0);

        /* Reconfig was successful. restart */
        wiced_bt_avdt_start_req(&p_ccb->p_scb->avdt_handle, 1);
    }
    else
    {
        // Reconfig failed. Try closing and re-starting
        wiced_bt_a2dp_source_close_str(p_ccb,NULL);
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_disconnect_req
** Description      Disconnect AVDTP connection.
*******************************************************************************/
void wiced_bt_a2dp_source_disconnect_req(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);
    wiced_bt_avdt_disconnect_req(p_ccb->peer_addr, p_ccb->p_dt_cback);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_sig_opened
** Description      This function indicates that A2DP signaling channel has opened
*******************************************************************************/
void wiced_bt_a2dp_source_sig_opened(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    WICED_BTA2DP_SRC_TRACE("%s ccb avdt_handle = %d \n", __FUNCTION__, p_data->str_msg.handle);

    if (p_ccb->p_scb->is_accepter)
        p_ccb->p_scb->avdt_version = p_data->sdp_res.avdt_version;

    wiced_bt_a2dp_source_free_sdb(p_ccb, p_data);
    memset(&p_ccb->p_scb->cap_configured,0,sizeof(p_ccb->p_scb->cap_configured));

    wiced_bt_a2dp_source_discover_req(p_ccb);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_sig_closed
** Description      This function sends a callback event to the upper layer to
**                  notify that A2DP signaling channel has closed
*******************************************************************************/
void wiced_bt_a2dp_source_sig_closed(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_status_t disconnect;

    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);

    /* Send callback event to the upper layer to inform streaming channel is closed */
    wiced_bt_a2dp_source_utils_bdcpy(disconnect.bd_addr, p_ccb->peer_addr);
    disconnect.result = WICED_SUCCESS;
    disconnect.handle = p_ccb->p_scb->avdt_handle;
    disconnect.is_accepter   = p_ccb->p_scb->is_accepter;

    /*TODO: */
#if 0
    /* If ACL is down and ACL is disconnected due to conneciton time out */
    if ( (HCI_ERR_CONNECTION_TOUT == btm_get_acl_disc_reason_code()) &&
         (BTM_IsAclConnectionUp(p_ccb->peer_addr, BT_TRANSPORT_BR_EDR) == WICED_FALSE))
    {
        disconnect.result = WICED_BT_TIMEOUT;
    }
#endif

    (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_DISCONNECT_EVT,
        (wiced_bt_a2dp_source_event_data_t*) &disconnect);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_sig_open_fail
** Description      This function sends a callback event to the upper layer to
**                  notify that A2DP signaling channel fails to open
*******************************************************************************/
void wiced_bt_a2dp_source_sig_open_fail(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_status_t connect;

    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);

    wiced_bt_a2dp_source_utils_bdcpy(connect.bd_addr, p_ccb->peer_addr);
    connect.result = WICED_BT_ERROR;
    connect.handle = p_ccb->p_scb->avdt_handle;
    connect.is_accepter = p_ccb->p_scb->is_accepter;

    /* Deallocate ccb */
    wiced_bt_a2dp_source_dealloc_ccb(p_ccb);

    (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_CONNECT_EVT,
        (wiced_bt_a2dp_source_event_data_t*) &connect);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_str_opened
** Description      Stream opened OK (incoming).
*******************************************************************************/
void wiced_bt_a2dp_source_str_opened(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_scb_t   *p_scb = p_ccb->p_scb;
    wiced_bt_a2dp_source_status_t connect;
    wiced_bt_a2dp_source_config_t cap_configured;

    p_scb->avdt_handle = p_data->str_msg.handle;
    p_scb->l2c_cid = wiced_bt_avdt_get_l2cap_channel(p_scb->avdt_handle);
    p_scb->stream_mtu = p_data->str_msg.msg.open_ind.peer_mtu - AVDT_MEDIA_HDR_SIZE;

    WICED_BTA2DP_SRC_TRACE("%s: l2c_cid:%d mtu:%d \n", __FUNCTION__, p_scb->l2c_cid, p_scb->stream_mtu);

    p_scb->l2c_bufs = 0;

    if (p_scb->role & WICED_BT_A2DP_SOURCE_ROLE_OPEN_INT)
        p_scb->role &= ~WICED_BT_A2DP_SOURCE_ROLE_OPEN_INT;

    cap_configured.result = WICED_BT_SUCCESS;
    cap_configured.cp_type = 0;
    cap_configured.codec_config = &p_ccb->p_scb->cap_configured;
    cap_configured.stream_mtu = p_scb->stream_mtu;
    (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_CONFIGURE_EVT,
                                          (wiced_bt_a2dp_source_event_data_t *)&cap_configured);
    //wiced_bt_a2dp_source_set_codec_config(&p_ccb->p_scb->cap_configured,p_ccb->p_scb->avdt_handle,0);

    /* TODO check if other audio channel is open.
     * If yes, check if reconfig is needed
     * Right now we do not do this kind of checking.
     * A2dp source is INT for 2nd audio connection.
     * The application needs to make sure the current codec_info is proper.
     * If one audio connection is open and another SNK attempts to connect to AV,
     * the connection will be rejected.
     */
    /* check if other audio channel is started. If yes, start */
    wiced_bt_a2dp_source_utils_bdcpy(connect.bd_addr, p_ccb->peer_addr);
    connect.result = WICED_SUCCESS;
    connect.handle = p_scb->avdt_handle;
    connect.lcid = p_data->str_msg.msg.open_cfm.lcid;
    connect.is_accepter = p_scb->is_accepter;


    (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_CONNECT_EVT,
        (wiced_bt_a2dp_source_event_data_t*)&connect);

    if (p_scb->recfg_ind)
    {
        wiced_bt_avdt_start_req(&p_scb->avdt_handle, 1);
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_str_open_fail
** Description      Fail to open stream in the incoming state.
*******************************************************************************/
void wiced_bt_a2dp_source_str_open_fail(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_scb_t    *p_scb = p_ccb->p_scb;
    wiced_bt_a2dp_source_status_t  connect;

    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);

    // make sure we are updating reconfig flag
    p_scb->recfg_ind = WICED_FALSE;

    wiced_bt_a2dp_source_utils_bdcpy(connect.bd_addr, p_scb->peer_addr);
    connect.result = WICED_BT_ERROR; /*TODO: Find a better error code */
    connect.handle = p_scb->avdt_handle;
    connect.is_accepter = p_scb->is_accepter;

    (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_CONNECT_EVT,
        (wiced_bt_a2dp_source_event_data_t*) &connect);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_do_close
** Description      Close stream and signaling channel
*******************************************************************************/
void wiced_bt_a2dp_source_do_close(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_scb_t *p_scb = p_ccb->p_scb;

    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);

    /* stop stream if started */
    if (p_scb->co_started)
    {
        wiced_bt_a2dp_source_str_stopped(p_ccb, NULL);
    }

    /* close stream */
    p_scb->started = WICED_FALSE;

    /* This action function is only triggered by AP_CLOSE_EVT, set the flag */
    p_scb->is_api_close = WICED_TRUE;

    wiced_bt_avdt_close_req(p_scb->avdt_handle);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_close_str
** Description      Close stream only
*******************************************************************************/
void wiced_bt_a2dp_source_close_str(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_scb_t *p_scb = p_ccb->p_scb;

    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);

    /* stop stream if started */
    if (p_scb->co_started)
    {
        wiced_bt_a2dp_source_str_stopped(p_ccb, NULL);
    }

    /* close stream */
    p_scb->started = WICED_FALSE;
    wiced_bt_avdt_close_req(p_scb->avdt_handle);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_connect_req
** Description      Connect AVDTP signaling channel.
*******************************************************************************/
void wiced_bt_a2dp_source_connect_req(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_scb_t *p_scb = p_ccb->p_scb;

    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);

    p_scb->avdt_version = p_data->sdp_res.avdt_version;
    wiced_bt_a2dp_source_free_sdb(p_ccb, p_data);

    wiced_bt_avdt_connect_req(p_ccb->peer_addr, 0/*Not Used*/, p_ccb->p_dt_cback);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_sdp_failed
** Description      Service discovery failed.
*******************************************************************************/
void wiced_bt_a2dp_source_sdp_failed(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_status_t connect;

    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);
    wiced_bt_a2dp_source_free_sdb(p_ccb, p_data);

    if (p_ccb->p_scb->is_accepter)
    {
        // In accepter role, SDP was trigger just to get peer A2DP version.
        // As SDP is failed, we will assume peer A2DP version is 1.0
        p_ccb->p_scb->avdt_version = 0x100;
        memset(&p_ccb->p_scb->cap_configured,0,sizeof(p_ccb->p_scb->cap_configured));
        wiced_bt_a2dp_source_discover_req(p_ccb);
        return;
    }

    wiced_bt_a2dp_source_utils_bdcpy(connect.bd_addr, p_ccb->peer_addr);
    connect.result = WICED_BT_ERROR; /*TODO: Find a better error code */
    connect.handle = p_ccb->p_scb->avdt_handle;
    connect.is_accepter   = p_ccb->p_scb->is_accepter;

    /* Deallocate ccb */
    wiced_bt_a2dp_source_dealloc_ccb(p_ccb);
    (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_CONNECT_EVT,
        (wiced_bt_a2dp_source_event_data_t*)&connect);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_rej_conn
** Description      .
*******************************************************************************/
void wiced_bt_a2dp_source_rej_conn(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);
    wiced_bt_avdt_config_rsp(p_data->str_msg.handle, p_data->str_msg.msg.hdr.label,
        AVDT_ERR_BAD_STATE, 0);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_open_failed
** Description      Failed to open an AVDT stream
*******************************************************************************/
void wiced_bt_a2dp_source_open_failed(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);
    wiced_bt_a2dp_source_cb.p_scb->open_status = WICED_BT_ERROR;
    wiced_bt_avdt_disconnect_req(p_ccb->peer_addr, p_ccb->p_dt_cback);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_sig_hdl_ap_close
** Description      This function handles the AP_CLOSE_EVT in the signaling channel
**                  opened state
*******************************************************************************/
void wiced_bt_a2dp_source_sig_hdl_ap_close(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_do_start
** Description      Start stream.
*******************************************************************************/
void wiced_bt_a2dp_source_do_start(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_scb_t *p_scb = p_ccb->p_scb;

    WICED_BTA2DP_SRC_TRACE("%s started %d \n", __FUNCTION__, p_scb->started);

    if (p_scb->started == WICED_FALSE)
    {
        p_scb->role |= WICED_BT_A2DP_SOURCE_ROLE_START_INT;

        if (memcmp(&p_ccb->p_scb->cap_configured, &p_data->start_req.codec_params, sizeof(wiced_bt_a2dp_codec_info_t)))
        {
            // if not same, mean reconfig required
            p_scb->recfg_ind = WICED_TRUE;

            // Clear SEP record as we will configure it again
            wiced_bt_a2dp_source_clean_sep_record(p_scb->avdt_handle);

            memcpy(&wiced_bt_a2dp_source_cb.p_config_data->default_codec_config, &p_data->start_req.codec_params,sizeof(wiced_bt_a2dp_codec_info_t));

            // Send reconfigure command
            wiced_bt_a2dp_source_ssm_execute(p_ccb, NULL, WICED_BT_A2DP_SOURCE_API_RECONFIG_CMD_EVT);
        }
        else
        {
            wiced_bt_avdt_start_req(&p_scb->avdt_handle, 1);
        }
    }
    else
    {
        wiced_bt_a2dp_source_start_ok(p_ccb, p_data);
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_send_start_resp
** Description      Accept or Reject Start stream request.
*******************************************************************************/
void wiced_bt_a2dp_source_send_start_resp( wiced_bt_a2dp_source_ccb_t *p_ccb,
                                                          wiced_bt_a2dp_source_data_t *p_data )
{
    wiced_bt_a2dp_source_scb_t *p_scb = p_ccb->p_scb;
    wiced_result_t result = WICED_SUCCESS;

    /* If start response is ok */
    if ( !p_data->api_data.status )
    {
        p_scb->started = WICED_TRUE;

        if(p_scb->role & WICED_BT_A2DP_SOURCE_ROLE_START_INT)
            p_scb->role &= ~WICED_BT_A2DP_SOURCE_ROLE_START_INT;

        WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);

        wiced_bt_a2dp_source_stream_chg(p_scb, WICED_TRUE);
        p_scb->co_started = WICED_TRUE;

       // result = wiced_bt_a2dp_source_streaming_configure_route( p_scb->avdt_handle );

        if ( result == WICED_SUCCESS )
            wiced_bt_avdt_start_resp( p_scb->avdt_handle, p_data->api_data.label, p_data->api_data.status );
        else
            wiced_bt_avdt_start_resp( p_scb->avdt_handle, p_data->api_data.label, AVDT_ERR_BAD_STATE );
    }
    else  /* If start response is reject */
    {
        wiced_bt_avdt_start_resp( p_scb->avdt_handle, p_data->api_data.label, p_data->api_data.status );
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_str_stopped
** Description      Stream stopped.
*******************************************************************************/
void wiced_bt_a2dp_source_str_stopped(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_scb_t   *p_scb = p_ccb->p_scb;
    wiced_bt_a2dp_source_status_t suspend;

    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);
    wiced_bt_a2dp_source_utils_bdcpy(suspend.bd_addr, p_scb->peer_addr);
    suspend.handle = p_scb->avdt_handle;
    suspend.is_accepter   = p_scb->is_accepter;

    /* If in the middle of a reconfigure cycle, stop it */
    p_scb->recfg_ind = WICED_FALSE;

    if (p_data)
    {
        WICED_BTA2DP_SRC_TRACE("suspending: %d, sup:%d \n", p_scb->started, p_scb->suspend_sup);
        if ((p_scb->started)  && (p_scb->suspend_sup))
        {
            p_scb->l2c_bufs = 0;
            wiced_bt_avdt_suspend_req(&p_scb->avdt_handle, 1);
        }
        else
        {
            if (p_scb->co_started)
            {
                wiced_bt_a2dp_source_stream_chg(p_scb, WICED_FALSE);
                /* if avdtp signalling is used,
                 * callout stop will be called at wiced_bt_a2dp_source_suspend_cfm.
                 */
                p_scb->co_started = WICED_FALSE;
            }

            {
                suspend.result = WICED_SUCCESS;

                //suspend.result = wiced_bt_a2dp_source_streaming_stop(p_scb->avdt_handle);

                (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_SUSPEND_EVT,
                    (wiced_bt_a2dp_source_event_data_t*) &suspend);
            }
        }
    }
    else
    {
        if (p_scb->co_started)
        {
            wiced_bt_a2dp_source_stream_chg(p_scb, WICED_FALSE);
            /* if avdtp signalling is used,
             * callout stop will be called at wiced_bt_a2dp_source_suspend_cfm.
             */
            p_scb->co_started = WICED_FALSE;

            //wiced_bt_a2dp_source_streaming_stop(p_scb->avdt_handle);

            (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_SUSPEND_EVT,
                (wiced_bt_a2dp_source_event_data_t*) &suspend);
        }
    }

}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_start_ind
** Description      Stream started.
*******************************************************************************/

void wiced_bt_a2dp_source_start_ind(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_start_t start;
    wiced_bt_a2dp_source_scb_t   *p_scb = p_ccb->p_scb;

    /* AVDT_START_IND_EVT - start audio */
    WICED_BTA2DP_SRC_TRACE( "[%s]: handle: %d, lcid %d \n", __FUNCTION__, p_ccb->avdt_handle, p_ccb->p_scb->l2c_cid );

    start.result = p_data->str_msg.msg.hdr.err_code;
    start.handle = p_scb->avdt_handle;
    start.label  = p_data->str_msg.msg.hdr.label;

    (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_START_IND_EVT,
            (wiced_bt_a2dp_source_event_data_t *) &start);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_start_ok
** Description      Stream started.
*******************************************************************************/
void wiced_bt_a2dp_source_start_ok(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_scb_t   *p_scb = p_ccb->p_scb;
    wiced_bt_a2dp_source_status_t start;

    p_scb->started = WICED_TRUE;

    if(p_scb->role & WICED_BT_A2DP_SOURCE_ROLE_START_INT)
        p_scb->role &= ~WICED_BT_A2DP_SOURCE_ROLE_START_INT;

    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);

    wiced_bt_a2dp_source_stream_chg(p_scb, WICED_TRUE);
    p_scb->co_started = WICED_TRUE;
    p_scb->recfg_ind = WICED_FALSE;
    start.result = WICED_SUCCESS;
    start.handle = p_scb->avdt_handle;
    start.is_accepter = p_scb->is_accepter;
    wiced_bt_a2dp_source_utils_bdcpy(start.bd_addr, p_scb->peer_addr);

    (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_START_CFM_EVT,
        (wiced_bt_a2dp_source_event_data_t *) &start);

     //wiced_bt_a2dp_source_streaming_configure_route(p_scb->avdt_handle);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_start_failed
** Description      Stream start failed.
*******************************************************************************/
void wiced_bt_a2dp_source_start_failed(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_scb_t   *p_scb = p_ccb->p_scb;
    wiced_bt_a2dp_source_status_t start;

    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);

    if(p_scb->started == WICED_FALSE && p_scb->co_started == WICED_FALSE)
    {
        start.result = WICED_ERROR;
        start.handle = p_scb->avdt_handle;
        start.is_accepter   = p_scb->is_accepter;
        wiced_bt_a2dp_source_utils_bdcpy(start.bd_addr, p_scb->peer_addr);

        //wiced_bt_a2dp_source_streaming_stop(p_scb->avdt_handle);

        (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_START_CFM_EVT,
            (wiced_bt_a2dp_source_event_data_t *) &start);
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_suspend_cfm
** Description      Process the suspend response
*******************************************************************************/
void wiced_bt_a2dp_source_suspend_cfm(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_scb_t   *p_scb = p_ccb->p_scb;
    wiced_bt_a2dp_source_status_t suspend;
    uint8_t                     err_code = p_data->str_msg.msg.hdr.err_code;

    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);

    wiced_bt_a2dp_source_utils_bdcpy(suspend.bd_addr, p_data->str_msg.bd_addr);
    suspend.result = WICED_SUCCESS;
    suspend.handle = p_scb->avdt_handle;
    suspend.is_accepter = p_scb->is_accepter;

    if (err_code)
    {
        /* Disable suspend feature only with explicit rejection(not with timeout) */
        if (err_code != AVDT_ERR_TIMEOUT)
        {
            p_scb->suspend_sup = WICED_FALSE;
        }
        suspend.result = WICED_ERROR;
    }
    else
    {
        /* only set started to WICED_FALSE when suspend is successful */
        p_scb->started = WICED_FALSE;
    }

    /* in case that we received suspend_ind, we may need to call co_stop here */
    if (p_scb->co_started)
    {
        {
            wiced_bt_a2dp_source_stream_chg(p_scb, WICED_FALSE);
            p_scb->co_started = WICED_FALSE;
        }
    }

    // suspend.result = wiced_bt_a2dp_source_streaming_stop(p_scb->avdt_handle);

    (*wiced_bt_a2dp_source_cb.control_cb)(WICED_BT_A2DP_SOURCE_SUSPEND_EVT,
        (wiced_bt_a2dp_source_event_data_t*) &suspend);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_stream_chg
** Description      Audio streaming status changed.
** Parameter        Started: WICED_TRUE   if an audio/video stream starts
**                           WICED_FALSE  if an audio/video stream stops
*******************************************************************************/
void wiced_bt_a2dp_source_stream_chg(wiced_bt_a2dp_source_scb_t *p_scb, wiced_bool_t started)
{
    WICED_BTA2DP_SRC_TRACE("%s: started:%d \n", __FUNCTION__, started);

/* TODO - Set stream priority */
#if 0
    if (started)
    {   /* As long as there is no channel opened */
        /* Let L2CAP know this channel is processed with high priority */
        wiced_bt_l2cap_set_acl_priority_ext(p_scb->peer_addr, L2CAP_PRIORITY_HIGH,
                               L2CAP_DIRECTION_DATA_SOURCE);
    }
    else
    {
        /* Let L2CAP know this channel is processed with low priority */
        wiced_bt_l2cap_set_acl_priority_ext(p_scb->peer_addr, L2CAP_PRIORITY_NORMAL,
                               L2CAP_DIRECTION_DATA_SOURCE);
    }
#endif

}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_hdl_str_close
** Description      This function handles disconnection of A2DP streaming channel
*******************************************************************************/
void wiced_bt_a2dp_source_hdl_str_close_ind(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_scb_t   *p_scb = p_ccb->p_scb;
    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);

    /* Do stop if we were started */
    if (p_scb->co_started)
    {
        wiced_bt_a2dp_source_str_stopped(p_ccb, NULL);
    }

    /* Close the A2DP signaling channel if streaming channel is closed by API
     * close call. Otherwise, stream close event is already sent to the upper layer, let
     * upper layer decides whether to continue to close A2DP signaling channels
     */
    if (p_scb->is_api_close == WICED_TRUE)
    {
        /* Close the A2DP signaling channel */
        wiced_bt_avdt_disconnect_req(p_ccb->peer_addr, p_ccb->p_dt_cback);
        p_scb->is_api_close = WICED_FALSE;
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_hdl_str_close_cfm
** Description      This function handles disconnection of A2DP streaming channel
*******************************************************************************/
void wiced_bt_a2dp_source_hdl_str_close_cfm(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{

    WICED_BTA2DP_SRC_TRACE( "[%s]: <%B> error_code: %d\n\r", __FUNCTION__, p_ccb->peer_addr, p_data->str_msg.msg.close_cfm.err_code);

    if (p_ccb->p_scb->recfg_ind == WICED_TRUE)
    {
        memset(&p_ccb->p_scb->cap_configured, 0, sizeof(wiced_bt_a2dp_codec_info_t));

        /* Assuming the peer device does not disconnect when close happens, re-perform SEP discovery */
        wiced_bt_a2dp_source_discover_req(p_ccb);
    }
    else
    {
        if (p_ccb->p_scb->is_api_close == WICED_TRUE)
        {
            /* Close the A2DP signaling channel */
            wiced_bt_avdt_disconnect_req(p_ccb->peer_addr, p_ccb->p_dt_cback);
            p_ccb->p_scb->is_api_close = WICED_FALSE;
        }
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_sig_hdl_ap_close_disconnect_req
** Description      Perform sig handle close and then disconnect
*******************************************************************************/
void wiced_bt_a2dp_source_sig_hdl_ap_close_disconnect_req(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_sig_hdl_ap_close(p_ccb, p_data);
    wiced_bt_a2dp_source_disconnect_req(p_ccb, p_data);
}

void av_app_send_setconfiguration(WICED_TIMER_PARAM_TYPE  cb_params)
{
    wiced_deinit_timer(&a2dp_src_setconfig_timer);
    wiced_bt_a2dp_source_ssm_execute((wiced_bt_a2dp_source_ccb_t *)cb_params, NULL, WICED_BT_A2DP_SOURCE_API_AVDT_SETCONFIG);
}

void wiced_bt_a2dp_source_sig_str_hdl_getcap_cfm(wiced_bt_a2dp_source_ccb_t *p_ccb, wiced_bt_a2dp_source_data_t *p_data)
{
    uint8_t i;
    wiced_bt_avdt_config_t *gatcap_cfm =  (wiced_bt_avdt_config_t *) &(p_data->str_msg.msg.getcap_cfm);
    uint8_t *peer_codec_info = gatcap_cfm->p_cfg->codec_info;

    /* check the getcap complete response status */
    if ( gatcap_cfm->hdr.err_code != AVDT_SUCCESS )
    {
        WICED_BTA2DP_SRC_ERROR( "[%s]: ERROR: getcap status = %d", __FUNCTION__, gatcap_cfm->hdr.err_code );
        /* TODO: getcap failed, report error to someone? Disconnect? */
        return;
    }

    /* Determine if this is one of the codecs we are interested in */
    for (i=0; i<WICED_BT_A2DP_SOURCE_MAX_NUM_CODECS; i++)
    {
        if ( ( peer_codec_info[2] == A2D_MEDIA_CT_SBC ) &&
               ( p_ccb->p_scb->av_sep_info[i].caps_already_updated != WICED_TRUE) )
        {

            uint8_t seid = p_ccb->p_scb->sep_info[p_ccb->p_scb->sep_info_idx].seid;

            WICED_BTA2DP_SRC_TRACE( "[%s]: Saving SEID %d information for codec id: %d \n",
                            __FUNCTION__, seid, A2D_MEDIA_CT_SBC);

            p_ccb->p_scb->av_sep_info[i].seid = seid;
            p_ccb->p_scb->av_sep_info[i].caps_already_updated = WICED_TRUE;

            /* Save the codec information to the control block */
            memcpy(&p_ccb->p_scb->av_sep_info[i].peer_caps,
                    gatcap_cfm->p_cfg,
                   sizeof(wiced_bt_avdt_cfg_t));
            break;
        }
        else
        {
            WICED_BTA2DP_SRC_TRACE( "[%s]: Codec not saved \n", __FUNCTION__);
        }
    }

    /* update the index for the next getcap call if any */
    p_ccb->p_scb->sep_info_idx += 1;

    if ( p_ccb->p_scb->sep_info_idx < p_ccb->p_scb->num_seps )
    {

        if ( wiced_bt_a2dp_source_next_gatcap( p_ccb, p_ccb->p_scb->sep_info, gatcap_cfm->p_cfg ) == WICED_FALSE )
        {
            /* getcap requests are not done yet. wait for next call */
            return;
        }
    }

    /* Free allocation made for the the getcap call */
    wiced_bt_free_buffer(gatcap_cfm->p_cfg);

    wiced_init_timer(&a2dp_src_setconfig_timer, av_app_send_setconfiguration, (WICED_TIMER_PARAM_TYPE)p_ccb, WICED_SECONDS_TIMER);
       wiced_start_timer(&a2dp_src_setconfig_timer,2);
}

wiced_bt_avdt_cfg_t *peercaps_from_seid(wiced_bt_a2dp_source_ccb_t *p_ccb,uint8_t seid)
{
    wiced_bt_avdt_cfg_t *peer_caps = NULL;
    int i;

    for (i=0; i<WICED_BT_A2DP_SOURCE_MAX_NUM_CODECS;i++)
    {
        if (p_ccb->p_scb->av_sep_info[i].seid == seid)
        {
            peer_caps = &p_ccb->p_scb->av_sep_info[i].peer_caps;
            break;
        }
    }

    return peer_caps;
}

/*
 * Send request to get the capabilities of the next available
 * stream found in the discovery results.
 */
wiced_bool_t wiced_bt_a2dp_source_next_gatcap(wiced_bt_a2dp_source_ccb_t *p_ccb,
                wiced_bt_avdt_sep_info_t *p_sep_info, wiced_bt_avdt_cfg_t *avdt_sep_config )
{
    wiced_bool_t last_getcap = WICED_FALSE;
    uint8_t i;

    WICED_BTA2DP_SRC_TRACE( "[%s]: sep_info_idx = %d\n\r", __FUNCTION__, p_ccb->p_scb->sep_info_idx);

    /* walk the discovery results looking for the next audio sink. */
    for ( i = p_ccb->p_scb->sep_info_idx; i < p_ccb->p_scb->num_seps; i++)
    {
        /* find a stream that is a sink, and is audio */
        if ( ( p_sep_info[i].tsep == AVDT_TSEP_SNK ) &&
             ( p_sep_info[i].media_type == p_ccb->p_scb->media_type ) )
        {
            break;
        }
    }

    p_ccb->p_scb->sep_info_idx = i;

    if (p_ccb->p_scb->sep_info_idx < p_ccb->p_scb->num_seps)
    {
        wiced_bt_avdt_getcap_req_t *p_req;
        uint16_t getcap_status;

        /* attempt GetAllCapReq only if both local and peer version are
         * greater than or equal to 1.3
         */
        if ( ( p_ccb->p_scb->avdt_version >= AVDT_VERSION_1_3 ) && ( AVDT_VERSION >= AVDT_VERSION_1_3 ) )
        {
            p_req = wiced_bt_avdt_get_all_cap_req;
        }
        else
        {
            p_req = wiced_bt_avdt_get_cap_req;
        }

        /* make the call to send the getcap request to the remote */
        getcap_status = (*p_req)( p_ccb->peer_addr,
                                  p_sep_info[p_ccb->p_scb->sep_info_idx].seid,
                                  avdt_sep_config,
                                  p_ccb->p_dt_cback );

        /* if getcap call returns an error , bail out */
        if ( getcap_status != AVDT_SUCCESS )
        {
            WICED_BTA2DP_SRC_ERROR( "[%s]: ERROR: getcap call status = %d", __FUNCTION__, getcap_status );
            last_getcap = WICED_TRUE;
        }
    }
    else
    {
        last_getcap = WICED_TRUE;
    }
    return last_getcap;
}

void wiced_bt_a2dp_source_sig_str_hdl_discovery_cfm (wiced_bt_a2dp_source_ccb_t *p_ccb, wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_avdt_discover_t *p_discover = (wiced_bt_avdt_discover_t *) &(p_data->str_msg.msg.discover_cfm);
    uint8_t sink_cnt = 0, i;

    if (p_discover->hdr.err_code != AVDT_SUCCESS)
    {
        WICED_BTA2DP_SRC_TRACE( "[%s]: Error code: = %d \n", __FUNCTION__, p_discover->hdr.err_code );
        return;
    }

    /* reset the current SEP index before processing the SEP list */
    p_ccb->p_scb->sep_info_idx = 0;

    /* store number of stream endpoints returned */
    p_ccb->p_scb->num_seps = p_discover->num_seps;

    /* trace all discovered SEPs */
    WICED_BTA2DP_SRC_TRACE( "[%s]: Peer Num SEPs: = %d \n", __FUNCTION__, p_discover->num_seps );

    /* save the pointer to the SEP information from the discover */
    if (p_ccb->p_scb->sep_info != NULL)
    {
        wiced_bt_free_buffer(p_ccb->p_scb->sep_info);
    }

    p_ccb->p_scb->sep_info = p_discover->p_sep_info;

    for (i = 0; i < p_discover->num_seps; i++)
    {
        /* find a stream that is a sink, and is audio */
        if ((p_discover->p_sep_info[i].tsep       == AVDT_TSEP_SNK) &&
            (p_discover->p_sep_info[i].media_type == AVDT_MEDIA_AUDIO))
        {
            sink_cnt++;
        }
    }

    WICED_BTA2DP_SRC_TRACE( "[%s]: sink_cnt: = %d \n", __FUNCTION__, sink_cnt );

    if (sink_cnt != 0)
    {
        /* Allocate space for the sink SEP capabilities. */
        wiced_bt_avdt_cfg_t *avdt_sep_config = (wiced_bt_avdt_cfg_t *)wiced_bt_get_buffer_from_heap(wiced_bt_a2dp_source_cb.heap, sizeof(wiced_bt_avdt_cfg_t));

        if (avdt_sep_config != NULL)
       {
            /* clear the audio SEP */
            memset( p_ccb->p_scb->av_sep_info, 0, sizeof(p_ccb->p_scb->av_sep_info) );

            /* clear the getcap results array */
            memset(avdt_sep_config, 0, sizeof(wiced_bt_avdt_cfg_t) );

            if ( wiced_bt_a2dp_source_next_gatcap (p_ccb, p_discover->p_sep_info,avdt_sep_config) == WICED_TRUE )
            {
                wiced_bt_free_buffer(avdt_sep_config);
            }
       }
    }
    else
    {
        WICED_BTA2DP_SRC_ERROR("[%s] WARNING No Available Sink SEPs on remote", __FUNCTION__ );
    }
}

void wiced_bt_a2dp_source_sig_str_hdl_setconfig (wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    int     result;
    uint8_t err_code = AVDT_SUCCESS;
    uint8_t sep_index = p_data->str_msg.msg.config_ind.int_seid - 1;
    uint8_t codec_index = wiced_bt_a2dp_source_cb.seps[sep_index].codec_cap_index;

    WICED_BTA2DP_SRC_TRACE("[%s] seid: %d error code %d \n", __FUNCTION__, p_data->str_msg.msg.config_ind.int_seid, p_data->str_msg.msg.config_ind.hdr.err_code);

    if (AVDT_SUCCESS != p_data->str_msg.msg.config_ind.hdr.err_code)
        return;

    result = wiced_bt_a2dp_src_sbc_format_check(p_data->str_msg.msg.config_ind.p_cfg->codec_info,
                wiced_bt_a2dp_source_cb.p_config_data->codec_capabilities.info[codec_index].cie.sbc,
                NULL, &p_ccb->p_scb->cap_configured.cie.sbc);

    if (result != WICED_TRUE)
        err_code = AVDT_ERR_UNSUP_CFG;

    result = wiced_bt_avdt_config_rsp(wiced_bt_a2dp_source_cb.seps[sep_index].av_handle, p_data->str_msg.msg.config_ind.hdr.label, err_code, AVDT_ASC_CODEC);

    if ( result == AVDT_SUCCESS )
    {
        wiced_bt_a2dp_source_cb.seps[sep_index].in_use = 1;
    }
    else
    {
        WICED_BTA2DP_SRC_ERROR (" [%s] Error %d in sending avdt config response \n", __FUNCTION__, result);
    }
}

int wiced_bt_a2dp_soruce_get_free_sep(int codec_index)
{
    int i = WICED_BT_A2DP_SOURCE_MAX_NUM_CONN;
    int codec_count = wiced_bt_a2dp_source_cb.p_config_data->codec_capabilities.count;

    for ( i = 0; i<WICED_BT_A2DP_SOURCE_MAX_NUM_CONN * codec_count; i++)
    {
        if ( (wiced_bt_a2dp_source_cb.seps[i].in_use == WICED_FALSE) &&  (wiced_bt_a2dp_source_cb.seps[i].codec_cap_index == codec_index) )
        {
            wiced_bt_a2dp_source_cb.seps[i].in_use = 1;
            return i;
        }
    }
    return i;
}

void wiced_bt_a2dp_source_sig_hdl_set_re_config (wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data, wiced_bool_t is_reconfig)
{
    int i,j,result;
    wiced_bool_t found = WICED_FALSE;
   // wiced_bt_a2d_sbc_cie_t codec_info;
    wiced_bt_avdt_cfg_t *peercaps= NULL;

    /* determine if one of the sink found will match our format */
    for ( i = 0; i < p_ccb->p_scb->num_seps; i++ )
    {

        /* find a stream that is a sink, is not in use, and is audio */
        if ( ( p_ccb->p_scb->sep_info[i].tsep == AVDT_TSEP_SNK ) &&
             ( p_ccb->p_scb->sep_info[i].in_use == WICED_FALSE) &&
             ( p_ccb->p_scb->sep_info[i].media_type == p_ccb->p_scb->media_type ) )
        {
            peercaps = peercaps_from_seid(p_ccb, p_ccb->p_scb->sep_info[i].seid);

            if (peercaps != NULL)
            {
                /* we will use the first SBC SEP that we find. */
                WICED_BTA2DP_SRC_TRACE("[%s] check sep: %d seid: %d\n", __FUNCTION__, i, p_ccb->p_scb->sep_info[i].seid);

                for( j = 0; j < wiced_bt_a2dp_source_cb.p_config_data->codec_capabilities.count; j++)
                {
                    if (wiced_bt_a2dp_source_cb.p_config_data->default_codec_config.codec_id == wiced_bt_a2dp_source_cb.p_config_data->codec_capabilities.info[j].codec_id)
                    {
                        switch(wiced_bt_a2dp_source_cb.p_config_data->codec_capabilities.info[j].codec_id)
                        {
                            case WICED_BT_A2DP_CODEC_SBC:
                                result = wiced_bt_a2dp_src_sbc_format_check(peercaps->codec_info,
                                        wiced_bt_a2dp_source_cb.p_config_data->codec_capabilities.info[j].cie.sbc,
                                        &wiced_bt_a2dp_source_cb.p_config_data->default_codec_config.cie.sbc,
                                        &p_ccb->p_scb->cap_configured.cie.sbc);
                                if (result == WICED_TRUE)
                                {
                                    p_ccb->p_scb->sep_info_idx = i;
                                    p_ccb->p_scb->configured_sep = wiced_bt_a2dp_soruce_get_free_sep(j);
                                    found = WICED_TRUE;
                                    break;
                                }
                                break;
                            case WICED_BT_A2DP_CODEC_M12:
                            case WICED_BT_A2DP_CODEC_M24:
                            case WICED_BT_A2DP_CODEC_VENDOR_SPECIFIC:
                                //Not Supported
                                break;
                        }
                    }
                }
                if (found)
                    break;
            }
            else
            {
                WICED_BTA2DP_SRC_TRACE("\t\t peercaps not found for seid %d!\n", p_ccb->p_scb->sep_info[i].seid);
            }
        }
    }

    if ( (i < p_ccb->p_scb->num_seps) && (found == WICED_TRUE))
    {
        wiced_bt_avdt_cfg_t av_cfg;

        WICED_BTA2DP_SRC_TRACE( "[%s]: found SBC SEP on peer seid: %d... \n", __FUNCTION__, p_ccb->p_scb->sep_info[i].seid );

        /* Build the configuration used to open the AVDT/A2DP connection */

        /* Copy the configuration exposed on initialization */
        memcpy( &av_cfg, &p_ccb->p_scb->cfg, sizeof(wiced_bt_avdt_cfg_t) );

        /* Based on peer and our config reset AVDT_PSC_DELAY_RPT */
        if ( ( p_ccb->p_scb->avdt_version < AVDT_VERSION_1_3 ) || ( AVDT_VERSION < AVDT_VERSION_1_3 ) ||
                !(av_cfg.psc_mask & AVDT_PSC_DELAY_RPT) || (peercaps && (!(peercaps->psc_mask & AVDT_PSC_DELAY_RPT))))
        {
            av_cfg.psc_mask &= ( (~AVDT_PSC_DELAY_RPT & 0xFFFF));
        }

        /* Build the config bytes from the configuration */
        wiced_bt_a2d_bld_sbc_info( AVDT_MEDIA_AUDIO, &p_ccb->p_scb->cap_configured.cie.sbc, av_cfg.codec_info );


        if (is_reconfig)
        {
            result = wiced_bt_avdt_reconfig_req( wiced_bt_a2dp_source_cb.seps[p_ccb->p_scb->configured_sep].av_handle, &av_cfg );
            if (result != AVDT_SUCCESS)
            {
                p_ccb->p_scb->recfg_ind = WICED_FALSE;
                WICED_BTA2DP_SRC_ERROR ( "[%s] wiced_bt_avdt_reconfig_req response %d \n",__FUNCTION__, result);
            }
        }
        else
        {
            wiced_bt_avdt_open_req( wiced_bt_a2dp_source_cb.seps[p_ccb->p_scb->configured_sep].av_handle, p_ccb->peer_addr,
                    p_ccb->p_scb->sep_info[p_ccb->p_scb->sep_info_idx].seid, &av_cfg );
        }
    }
    else
    {
        WICED_BTA2DP_SRC_ERROR( "[%s]: ERROR: No peer format match found", __FUNCTION__ );
        wiced_bt_avdt_disconnect_req(p_ccb->peer_addr, p_ccb->p_dt_cback);
    }
}

void wiced_bt_a2dp_source_sig_hdl_setconfig (wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_sig_hdl_set_re_config (p_ccb, p_data, WICED_FALSE);
}

void wiced_bt_a2dp_source_sig_hdl_reconfig (wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_sig_hdl_set_re_config (p_ccb, p_data, WICED_TRUE);
}

void wiced_bt_a2dp_source_discover_req(wiced_bt_a2dp_source_ccb_t *p_ccb)
{
    wiced_bt_avdt_sep_info_t *p_sep_info;
    uint16_t discover_size = sizeof(wiced_bt_avdt_sep_info_t) * WICED_BT_A2DP_SOURCE_NUM_SEPS;
    p_sep_info = (wiced_bt_avdt_sep_info_t *)wiced_bt_get_buffer_from_heap(wiced_bt_a2dp_source_cb.heap, discover_size);

    if ( p_sep_info !=NULL)
    {
        if( AVDT_SUCCESS != wiced_bt_avdt_discover_req( p_ccb->peer_addr, p_sep_info, WICED_BT_A2DP_SOURCE_NUM_SEPS, p_ccb->p_dt_cback ))
        {
            WICED_BTA2DP_SRC_TRACE ("[%s] Failed \n",__FUNCTION__);
            wiced_bt_free_buffer(p_sep_info);
        }
    }
}
/*******************************************************************************
** Function         wiced_bt_a2dp_source_sig_closed_cleanup
** Description      Perform sig closed and then cleanup
*******************************************************************************/
void wiced_bt_a2dp_source_sig_closed_cleanup(wiced_bt_a2dp_source_ccb_t *p_ccb,
        wiced_bt_a2dp_source_data_t *p_data)
{
    WICED_BTA2DP_SRC_TRACE("%s\n", __FUNCTION__);

    //wiced_bt_a2dp_source_stream_close(p_scb->avdt_handle);

    wiced_bt_a2dp_source_sig_closed(p_ccb, p_data);
    wiced_bt_a2dp_source_cleanup(p_ccb, p_data);
    /* Deallocate ccb */
    wiced_bt_a2dp_source_dealloc_ccb(p_ccb);
}
