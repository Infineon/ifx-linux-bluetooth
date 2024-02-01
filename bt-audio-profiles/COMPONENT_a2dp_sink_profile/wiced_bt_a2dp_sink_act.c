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
 * This file contains action functions for audio sink state machine.
 */

#include <string.h>
#include "wiced_memory.h"
//#include "wiced_bt_l2c.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_a2dp_sink_int.h"
#include "wiced_bt_sdp.h"

/* These tables translate AVDT events to state machine events */
static const uint8_t wiced_bt_a2dp_sink_stream_evt_ok[] = {
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_DISCOVER_CFM_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_GETCAP_CFM_EVT */
    WICED_BT_A2DP_SINK_STR_OPEN_OK_EVT,          /**< AVDT_OPEN_CFM_EVT */
    WICED_BT_A2DP_SINK_STR_OPEN_OK_EVT,          /**< AVDT_OPEN_IND_EVT */
    WICED_BT_A2DP_SINK_STR_CONFIG_IND_EVT,       /**< AVDT_CONFIG_IND_EVT */
    WICED_BT_A2DP_SINK_STR_START_CFM_EVT,        /**< AVDT_START_CFM_EVT */
    WICED_BT_A2DP_SINK_STR_START_IND_EVT,        /**< AVDT_START_IND_EVT */
    WICED_BT_A2DP_SINK_STR_SUSPEND_CFM_EVT,      /**< AVDT_SUSPEND_CFM_EVT */
    WICED_BT_A2DP_SINK_STR_SUSPEND_CFM_EVT,      /**< AVDT_SUSPEND_IND_EVT */
    WICED_BT_A2DP_SINK_STR_CLOSE_OK_EVT,         /**< AVDT_CLOSE_CFM_EVT */
    WICED_BT_A2DP_SINK_STR_CLOSE_OK_EVT,         /**< AVDT_CLOSE_IND_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_RECONFIG_CFM_EVT */
    WICED_BT_A2DP_SINK_STR_RECONFIG_IND_EVT,     /**< AVDT_RECONFIG_IND_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_SECURITY_CFM_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_SECURITY_IND_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_WRITE_CFM_EVT */
    WICED_BT_A2DP_SINK_AVDT_CONNECT_EVT,         /**< AVDT_CONNECT_IND_EVT */
    WICED_BT_A2DP_SINK_AVDT_DISCONNECT_EVT,      /**< AVDT_DISCONNECT_IND_EVT */
    WICED_BT_A2DP_SINK_AVDT_REPOPT_CONN_EVT,     /**< AVDT_REPORT_CONN_EVT */
    WICED_BT_A2DP_SINK_AVDT_REPOPT_CONN_EVT,     /**< AVDT_REPORT_DISCONN_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_DELAY_REPORT_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_DELAY_REPORT_CFM_EVT */
};

static const uint8_t wiced_bt_a2dp_sink_stream_evt_fail[] = {
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_DISCOVER_CFM_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_GETCAP_CFM_EVT */
    WICED_BT_A2DP_SINK_STR_OPEN_FAIL_EVT,        /**< AVDT_OPEN_CFM_EVT */
    WICED_BT_A2DP_SINK_STR_OPEN_OK_EVT,          /**< AVDT_OPEN_IND_EVT */
    WICED_BT_A2DP_SINK_STR_CONFIG_IND_EVT,       /**< AVDT_CONFIG_IND_EVT */
    WICED_BT_A2DP_SINK_STR_START_FAIL_EVT,       /**< AVDT_START_CFM_EVT */
    WICED_BT_A2DP_SINK_STR_START_IND_EVT,        /**< AVDT_START_IND_EVT */
    WICED_BT_A2DP_SINK_STR_SUSPEND_CFM_EVT,      /**< AVDT_SUSPEND_CFM_EVT */
    WICED_BT_A2DP_SINK_STR_SUSPEND_CFM_EVT,      /**< AVDT_SUSPEND_IND_EVT */
    WICED_BT_A2DP_SINK_STR_CLOSE_OK_EVT,         /**< AVDT_CLOSE_CFM_EVT */
    WICED_BT_A2DP_SINK_STR_CLOSE_OK_EVT,         /**< AVDT_CLOSE_IND_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_RECONFIG_CFM_EVT */
    WICED_BT_A2DP_SINK_STR_RECONFIG_IND_EVT,     /**< AVDT_RECONFIG_IND_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_SECURITY_CFM_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_SECURITY_IND_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_WRITE_CFM_EVT */
    WICED_BT_A2DP_SINK_AVDT_CONNECT_EVT,         /**< AVDT_CONNECT_IND_EVT */
    WICED_BT_A2DP_SINK_AVDT_DISCONNECT_EVT,      /**< AVDT_DISCONNECT_IND_EVT */
    WICED_BT_A2DP_SINK_AVDT_REPOPT_CONN_EVT,     /**< AVDT_REPORT_CONN_EVT */
    WICED_BT_A2DP_SINK_AVDT_REPOPT_CONN_EVT,     /**< AVDT_REPORT_DISCONN_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_DELAY_REPORT_EVT */
    WICED_BT_A2DP_SINK_INVALID_EVT,              /**< AVDT_DELAY_REPORT_CFM_EVT */
};
/*******************************************************************************
** Function         wiced_bt_a2dp_sink_get_delay
** Description      Provide delay timing in 1/10 ms.
*******************************************************************************/
static uint16_t wiced_bt_a2dp_sink_get_delay(void)
{
    /*
     * Delay = buf_depth_ms * (target_buf_depth / 100)        in ms
     *       = 10 * buf_depth_ms * (target_buf_depth / 100)   in 1/10 ms
     *       = buf_depth_ms * target_buf_depth / 10           in 1/10 ms
     */
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
    const uint32_t delay = wiced_bt_a2dp_sink_cb.p_config_data->p_param.buf_depth_ms
        * wiced_bt_a2dp_sink_cb.p_config_data->p_param.target_buf_depth / 10;
#else
    const uint32_t delay = wiced_bt_a2dp_sink_cb.p_config_data->sink_delay;
#endif

    return delay;
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_save_addr
** Description      Copy the bd_addr and maybe reset the supported flags
*******************************************************************************/
static void wiced_bt_a2dp_sink_save_addr(wiced_bt_a2dp_sink_scb_t *p_scb,
    const wiced_bt_device_address_t b)
{
    WICED_BTA2DP_TRACE("%s: recfg_sup:%d, suspend_sup:%d \n", __FUNCTION__,
        p_scb->recfg_sup, p_scb->suspend_sup);
    if(wiced_bt_a2dp_sink_utils_bdcmp(p_scb->peer_addr, b) != 0)
    {
        WICED_BTA2DP_SINK_ERROR("reset flags \n");
        /* A new addr, reset the supported flags */
        p_scb->recfg_sup    = WICED_TRUE;
        p_scb->suspend_sup  = WICED_TRUE;
    }

    /* Do this copy anyway, just in case the first addr matches
     * the control block one by accident */
    wiced_bt_a2dp_sink_utils_bdcpy(p_scb->peer_addr, b);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_proc_stream_evt
** Description      Utility function to compose stream events.
*******************************************************************************/
static void wiced_bt_a2dp_sink_proc_stream_evt(uint8_t handle,
    wiced_bt_device_address_t bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data)
{
	wiced_bt_a2dp_sink_str_msg_t *p_msg   = NULL;
    uint16_t                      sec_len = 0;
    wiced_bt_a2dp_sink_global_evt_t  sink_event;

    WICED_BTA2DP_TRACE("wiced_bt_a2dp_sink_proc_stream_evt event:%d, p_data:%lu \n", event, p_data);
    if (p_data)
    {
        if (event == AVDT_SECURITY_IND_EVT)
        {
            sec_len = (p_data->security_ind.len < WICED_BT_A2DP_SINK_SECURITY_MAX_LEN) ?
                   p_data->security_ind.len : WICED_BT_A2DP_SINK_SECURITY_MAX_LEN;
        }
        else if (event == AVDT_SECURITY_CFM_EVT && p_data->hdr.err_code == 0)
        {
            sec_len = (p_data->security_cfm.len < WICED_BT_A2DP_SINK_SECURITY_MAX_LEN) ?
                   p_data->security_cfm.len : WICED_BT_A2DP_SINK_SECURITY_MAX_LEN;
        }
    }

    if ((p_msg = (wiced_bt_a2dp_sink_str_msg_t *)
                 wiced_bt_get_buffer((uint16_t) (sizeof(wiced_bt_a2dp_sink_str_msg_t) + sec_len))) != NULL)
    {
        /* Copy event data, bd addr, and handle to event message buffer */
        if (bd_addr != NULL)
        {
            wiced_bt_a2dp_sink_utils_bdcpy(p_msg->bd_addr, bd_addr);
            WICED_BTA2DP_TRACE("bd_addr:%02x-%02x-%02x-%02x-%02x-%02x \n",
                          bd_addr[0], bd_addr[1],
                          bd_addr[2], bd_addr[3],
                          bd_addr[4], bd_addr[5]);
        }
        /* Copy config params to event message buffer */
        if (p_data != NULL)
        {
            memcpy(&p_msg->msg, p_data, sizeof (wiced_bt_avdt_ctrl_t));

            switch (event)
            {
                case AVDT_CONFIG_IND_EVT:
                memcpy(&p_msg->cfg, p_data->config_ind.p_cfg, sizeof(wiced_bt_avdt_cfg_t));
                break;

                case AVDT_RECONFIG_IND_EVT:
                memcpy(&p_msg->cfg, p_data->reconfig_ind.p_cfg, sizeof(wiced_bt_avdt_cfg_t));
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

        /* Look up a2dp sink event */
        if ((p_data == NULL) || (p_data->hdr.err_code == 0))
        {
        	sink_event = (wiced_bt_a2dp_sink_global_evt_t) wiced_bt_a2dp_sink_stream_evt_ok[event];
        }
        else
        {
        	sink_event = (wiced_bt_a2dp_sink_global_evt_t) wiced_bt_a2dp_sink_stream_evt_fail[event];
        }

        p_msg->handle = handle;
        wiced_bt_a2dp_sink_hdl_event(sink_event, (wiced_bt_avdt_evt_hdr_t*)&p_msg->hdr);

        /* Free the buffer used, since it was allocated here */
        wiced_bt_free_buffer(p_msg);
    }

    /* If signalling channel closed, then also notify so that a2dp sink cleans up any open stream.
     * If sig channel was closed locally; a scb may have been left in INCOMING_ST during
     * incoming connection for SNK
     */
    if (event == AVDT_DISCONNECT_IND_EVT)
    {
        wiced_bt_avdt_ctrl_t ctrl_data;
        ctrl_data.hdr.err_param = 0; /*TODO*/
        WICED_BTA2DP_TRACE("Notifying signalling channel close (locally initiated) \n");
        wiced_bt_a2dp_sink_conn_cback(handle, bd_addr, AVDT_DISCONNECT_IND_EVT, &ctrl_data);
    }
}


/*******************************************************************************
** Function         wiced_bt_a2dp_sink_ctrl_cback
** Description      This is the AVDTP callback function for stream events.
*******************************************************************************/
void wiced_bt_a2dp_sink_ctrl_cback(uint8_t handle, wiced_bt_device_address_t bd_addr,
                              uint8_t event, wiced_bt_avdt_ctrl_t *p_data)
{
    WICED_BTA2DP_TRACE("%s: avdt_handle: %d event=0x%x \n", __FUNCTION__, handle, event);
    wiced_bt_a2dp_sink_proc_stream_evt(handle, bd_addr, event, p_data);
}


 /*******************************************************************************
 ** Function         wiced_bt_a2dp_sink_data_cback
 ** Description      This function receives compressed audio data.
 *******************************************************************************/
void wiced_bt_a2dp_sink_data_cback(uint8_t handle, uint16_t seq, uint32_t time_stamp, uint8_t m_pt,
                            uint8_t * p_rx_media, uint16_t media_len)
{
    WICED_BTA2DP_TRACE("%s: handle:%d, stamp:%d, m_pt:%d", __FUNCTION__,
                        handle, time_stamp, m_pt);
    if(wiced_bt_a2dp_sink_cb.data_cb)
        (*wiced_bt_a2dp_sink_cb.data_cb)(p_rx_media, media_len); 
    }


#if AVDT_REPORTING == TRUE
/*******************************************************************************
** Function         wiced_bt_a2dp_sink_report_cback
** Description      Report callback.
*******************************************************************************/
void wiced_bt_a2dp_sink_report_cback(uint8_t handle, AVDT_REPORT_TYPE type,
                                    wiced_bt_avdt_report_data_t *p_data)
{
    /* Will be implemented later if needed. */
    WICED_BTA2DP_TRACE("%s: NOT IMPLEMENTED \n", __FUNCTION__);
}
#endif

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_sdp_complete_cback
** Description      Service discovery complete callback.
*******************************************************************************/
void wiced_bt_a2dp_sink_sdp_complete_cback(uint16_t sdp_res)
{
    wiced_bt_sdp_discovery_record_t *p_rec = NULL;
    wiced_bt_sdp_protocol_elem_t     elem;
    wiced_bt_a2dp_sink_sdp_res_t     msg;
    uint16_t                         avdt_version;
    uint8_t event;
#if 1
    uint16_t                        i;
#else
    wiced_bt_a2dp_sink_scb_t        *p_scb = wiced_bt_a2dp_sink_cb.p_scb;
#endif

    memset(&msg, 0, sizeof(wiced_bt_a2dp_sink_sdp_res_t));

    WICED_BTA2DP_TRACE("%s status: %d \n", __FUNCTION__, sdp_res);

    if (sdp_res != WICED_BT_SDP_SUCCESS)
    {
    	event= WICED_BT_A2DP_SINK_SDP_DISC_FAIL_EVT;
        goto wiced_a2dp_sink_sdp_complete;
    }

    /* Check if service is available */
#if 1
    for (i = 0 ; i < WICED_BT_A2DP_SINK_MAX_SEPS ; i++)
    {
        if (wiced_bt_a2dp_sink_cb.p_scb[i].p_sdp_db)
        {
            p_rec = wiced_bt_sdp_find_service_in_db(wiced_bt_a2dp_sink_cb.p_scb[i].p_sdp_db,
                                                    UUID_SERVCLASS_AUDIO_SOURCE,
                                                    p_rec);

            if (p_rec)
            {
                WICED_BT_TRACE("%s audio source found in %B\n", __FUNCTION__, wiced_bt_a2dp_sink_cb.p_scb[i].peer_addr);
                break;
            }
        }
    }

    if (p_rec == NULL)
    {
    	event = WICED_BT_A2DP_SINK_SDP_DISC_FAIL_EVT;
        goto wiced_a2dp_sink_sdp_complete;
    }
#else
    if ((p_rec = wiced_bt_sdp_find_service_in_db(p_scb->p_sdp_db,
        UUID_SERVCLASS_AUDIO_SOURCE, p_rec)) == NULL)
    {
        msg.hdr.event = WICED_BT_A2DP_SINK_SDP_DISC_FAIL_EVT;
        goto wiced_a2dp_sink_sdp_complete;
    }
#endif

    /* Get AVDTP version */
    if (wiced_bt_sdp_find_protocol_list_elem_in_rec(p_rec,
        UUID_PROTOCOL_AVDTP, &elem) == WICED_TRUE)
    {
        avdt_version = elem.params[0];
        WICED_BTA2DP_TRACE("%s: avdt_version: 0x%04x \n", __FUNCTION__, avdt_version);
        event = WICED_BT_A2DP_SINK_SDP_DISC_OK_EVT;
        msg.avdt_version = avdt_version;
    }
    else
    {
    	event = WICED_BT_A2DP_SINK_SDP_DISC_FAIL_EVT;
    }

wiced_a2dp_sink_sdp_complete:
    wiced_bt_a2dp_sink_hdl_event(event, (wiced_bt_avdt_evt_hdr_t*)&msg.hdr);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_do_sdp
** Description      Perform SDP.
*******************************************************************************/
void wiced_bt_a2dp_sink_do_sdp(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    uint16_t                    sdp_res   = WICED_BT_SDP_GENERIC_ERROR;
    wiced_bt_a2dp_sink_scb_t   *p_scb = p_ccb->p_scb;
    wiced_bool_t                ret = WICED_FALSE;
    wiced_bt_uuid_t             uuid_list;
    uint16_t                    attr_list[] = {ATTR_ID_SERVICE_CLASS_ID_LIST,
                                               ATTR_ID_PROTOCOL_DESC_LIST,
                                               ATTR_ID_BT_PROFILE_DESC_LIST};

    /* Save peer bd_addr in sdp_bd_addr */
    wiced_bt_a2dp_sink_utils_bdcpy(wiced_bt_a2dp_sink_cb.sdp_bd_addr, p_ccb->peer_addr);

    uuid_list.len       = LEN_UUID_16;
    uuid_list.uu.uuid16 = UUID_SERVCLASS_AUDIO_SOURCE;

    if (p_scb->p_sdp_db == NULL)
    {
        p_scb->p_sdp_db = (wiced_bt_sdp_discovery_db_t *)
            wiced_bt_get_buffer(WICED_BT_A2DP_SINK_DISC_BUF_SIZE);
    }

    if (p_scb->p_sdp_db == NULL)
    {
        sdp_res = WICED_BT_SDP_NO_RESOURCES;
        goto wiced_bt_a2dp_sink_sdp_error;
    }

    ret = wiced_bt_sdp_init_discovery_db (p_scb->p_sdp_db, WICED_BT_A2DP_SINK_DISC_BUF_SIZE,
            1,&uuid_list,
            sizeof(attr_list)/sizeof(attr_list[0]), attr_list);
    if(ret == WICED_FALSE)
    {
        sdp_res = WICED_BT_SDP_GENERIC_ERROR;
        goto wiced_bt_a2dp_sink_sdp_error;
    }

    ret = wiced_bt_sdp_service_search_attribute_request(wiced_bt_a2dp_sink_cb.sdp_bd_addr,
            p_scb->p_sdp_db, wiced_bt_a2dp_sink_sdp_complete_cback);

wiced_bt_a2dp_sink_sdp_error:
    if(ret == WICED_FALSE)
    {
        wiced_bt_a2dp_sink_sdp_complete_cback(sdp_res);
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_delay_send
** Description      Send A2DP Delay Report
*******************************************************************************/
void wiced_bt_a2dp_sink_delay_send(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t   *p_scb = p_ccb->p_scb;

    /* If peer device supports Delay Reporting */
    if (p_scb->cur_psc_mask & AVDT_PSC_DELAY_RPT)
    {
        wiced_bt_avdt_delay_report(p_scb->avdt_handle, p_scb->peer_sep_idx,
                wiced_bt_a2dp_sink_get_delay());
    }
}
/*******************************************************************************
** Function         wiced_bt_a2dp_sink_clean_sep_record
** Description      Clean sep record
*******************************************************************************/
void wiced_bt_a2dp_sink_clean_sep_record(uint16_t avdt_handle)
{
    int xx;
    for ( xx = 0; xx < WICED_BT_A2DP_SINK_MAX_SEPS; xx++ )
    {
        if (  wiced_bt_a2dp_sink_cb.seps[xx].av_handle == avdt_handle )
        {
            wiced_bt_a2dp_sink_cb.seps[xx].in_use = 0;
        }
    }
}
/*******************************************************************************
** Function         wiced_bt_a2dp_sink_cleanup
** Description      Cleanup stream control block.
*******************************************************************************/
void wiced_bt_a2dp_sink_cleanup(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    int                       xx;
    wiced_bt_a2dp_sink_scb_t *p_scb = NULL;

    WICED_BTA2DP_TRACE("%s, %d \n", __FUNCTION__, p_ccb->state );

    if( p_ccb->state != WICED_BT_A2DP_SINK_INIT_SST )
        return;

    p_scb = p_ccb->p_scb;

    if (p_scb != NULL)
    {
        /* Free any buffers */
        if(p_scb->p_cap != NULL)
        {
            wiced_bt_free_buffer(p_scb->p_cap);
            p_scb->p_cap = NULL;
        }

        if(p_scb->p_sdp_db != NULL)
        {
            wiced_bt_free_buffer(p_scb->p_sdp_db);
            p_scb->p_sdp_db = NULL;
        }

        p_scb->avdt_version = 0;
        p_scb->co_delay     = 0;

        /* Initialize some control block variables */
        p_scb->open_status = WICED_SUCCESS;

        /* if de-registering shut everything down */
        p_scb->started  = WICED_FALSE;
        p_scb->role = 0;
        p_scb->cur_psc_mask = 0;
        p_scb->recfg_ind = WICED_FALSE;

        wiced_bt_a2dp_sink_clean_sep_record(p_scb->avdt_handle);

        if (p_scb->deregistring)
        {
            /* Remove stream */
            if(p_scb->avdt_handle)
                wiced_bt_avdt_remove_stream(p_scb->avdt_handle);
            p_scb->avdt_handle = 0;
            WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);
        }
    }

    for (xx = 0; xx < WICED_BT_A2DP_SINK_MAX_NUM_CONN; xx++)
    {
        if (wiced_bt_a2dp_sink_cb.ccb[xx].state != WICED_BT_A2DP_SINK_INIT_SST)
            return;

        if (wiced_bt_a2dp_sink_cb.ccb[xx].in_use == WICED_TRUE &&
                wiced_bt_a2dp_sink_cb.ccb[xx].p_scb->avdt_handle != 0)
            return;

    }
    /*
     * should not reach here if there are still some avdt_handles
     * that are non-null otherwise, we will end up deregistering from AVDT itself.
     */
    if ((p_scb) && (p_scb->deregistring))
        wiced_bt_a2dp_sink_dereg_comp();

}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_free_sdb
** Description      Free service discovery db buffer.
*******************************************************************************/
void wiced_bt_a2dp_sink_free_sdb(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t *p_scb = p_ccb->p_scb;
    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    if(p_scb->p_sdp_db != NULL)
    {
        wiced_bt_free_buffer(p_scb->p_sdp_db);
        p_scb->p_sdp_db = NULL;
    }
}


/*******************************************************************************
** Function         wiced_bt_a2dp_sink_config_ind
** Description      Handle a stream configuration indication from the peer.
*******************************************************************************/
void wiced_bt_a2dp_sink_config_ind(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t          *p_scb = p_ccb->p_scb;
    wiced_bt_a2dp_sink_setconfig_rsp_t setconfig;
    wiced_bt_avdt_cfg_t               *p_evt_cfg = &p_data->str_msg.cfg;
    uint8_t   psc_mask = (p_evt_cfg->psc_mask & p_scb->cfg.psc_mask) | p_scb->cfg.psc_mask;

    WICED_BTA2DP_TRACE("%s handle = %d \n", __FUNCTION__, p_data->str_msg.handle);

    p_scb->avdt_label = p_data->str_msg.msg.hdr.label;
    memcpy(p_scb->cfg.codec_info, p_evt_cfg->codec_info, AVDT_CODEC_SIZE);
    p_scb->codec_type = (wiced_bt_a2dp_codec_t)p_evt_cfg->codec_info[AVDT_CODEC_TYPE_INDEX];

    wiced_bt_a2dp_sink_save_addr(p_scb, p_data->str_msg.bd_addr);

    /* If no codec parameters in configuration, fail */
    if ((p_evt_cfg->num_codec == 0) ||
    /* Or the peer requests for a service we do not support */
        ((psc_mask != p_scb->cfg.psc_mask) &&
        (psc_mask != (p_scb->cfg.psc_mask&~AVDT_PSC_DELAY_RPT))))
    {
        setconfig.err_code  = AVDT_ERR_UNSUP_CFG;
        wiced_bt_a2dp_sink_utils_bdcpy(setconfig.peer_addr, p_data->str_msg.bd_addr);
        wiced_bt_a2dp_sink_ssm_execute(p_ccb, (wiced_bt_a2dp_sink_data_t*) &setconfig,
            WICED_BT_A2DP_SINK_STR_CONFIG_RSP_FAIL_EVT);
    }
    else
    {
        if (p_ccb->state == WICED_BT_A2DP_SINK_OPEN_SST)
        {
            /* This is a reconfigure, not a set config. Set flag */
            p_scb->recfg_ind = WICED_TRUE;
            WICED_BTA2DP_TRACE("%s: recfg_ind set: %d \n", __FUNCTION__, p_scb->recfg_ind);

        }

        p_scb->avdt_handle = p_data->str_msg.handle;
        p_scb->peer_sep_idx = p_data->str_msg.msg.config_ind.int_seid;
        WICED_BTA2DP_TRACE("%s: SEID: %d handle = 0x%x \n", __FUNCTION__, p_scb->peer_sep_idx,p_scb->avdt_handle);
        p_scb->cur_psc_mask = p_evt_cfg->psc_mask;

        wiced_bt_a2dp_sink_cfg_setcfg_ind_handler(
            &wiced_bt_a2dp_sink_cb.p_config_data->codec_capabilities, &p_data->str_msg);

    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_disconnect_req
** Description      Disconnect AVDTP connection.
*******************************************************************************/
void wiced_bt_a2dp_sink_disconnect_req(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);
    wiced_bt_avdt_disconnect_req(p_ccb->peer_addr, p_ccb->p_dt_cback);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_setconfig_rsp
** Description      Setconfig is OK
*******************************************************************************/
void wiced_bt_a2dp_sink_setconfig_rsp(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t  *p_scb;

    if(!(p_ccb) || !(p_data))
    {
        WICED_BTA2DP_SINK_ERROR("%s p_ccb or p_data is null \n",__FUNCTION__);
        return;
    }

    p_scb = p_ccb->p_scb;

    WICED_BTA2DP_TRACE("%s: cur_psc_mask:0x%x \n", __FUNCTION__, p_scb->cur_psc_mask);

    wiced_bt_avdt_config_rsp(p_scb->avdt_handle, p_scb->avdt_label,
        p_data->setconfig_rsp.err_code,
        p_data->setconfig_rsp.category);

    /* If delay report is set, send the initial delay report now */
    if (!p_data->setconfig_rsp.err_code && (p_scb->cur_psc_mask & AVDT_PSC_DELAY_RPT) )
    {
        p_scb->co_delay     = WICED_TRUE;
        /* Calculate the Delay (in ms) and sent it to the Source (1/10 ms unit) */
        wiced_bt_avdt_delay_report(p_scb->avdt_handle,p_scb->peer_sep_idx,
                wiced_bt_a2dp_sink_get_delay());
        WICED_BTA2DP_TRACE("%s: avdt_version:0x%04x \n", __FUNCTION__, p_scb->avdt_version);
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_reconfig_rsp
** Description      Setconfig is OK
*******************************************************************************/
void wiced_bt_a2dp_sink_reconfig_rsp(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t *p_scb = p_ccb->p_scb;

    if (p_scb->recfg_ind)
    {
        p_scb->recfg_ind = WICED_FALSE;

        wiced_bt_a2dp_sink_clean_sep_record(p_scb->avdt_handle);

        wiced_bt_avdt_reconfig_rsp(p_scb->avdt_handle, p_scb->avdt_label,
            p_data->setconfig_rsp.err_code, p_data->setconfig_rsp.category);
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_sig_opened
** Description      This function indicates that A2DP signaling channel has opened
*******************************************************************************/
void wiced_bt_a2dp_sink_sig_opened(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    WICED_BTA2DP_TRACE("%s ccb avdt_handle = %d \n", __FUNCTION__, p_data->str_msg.handle);
    p_ccb->avdt_handle = p_data->str_msg.handle;
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_sig_closed
** Description      This function sends a callback event to the upper layer to
**                  notify that A2DP signaling channel has closed
*******************************************************************************/
void wiced_bt_a2dp_sink_sig_closed(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_status_t disconnect;

    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    /* Send callback event to the upper layer to inform streaming channel is closed */
    wiced_bt_a2dp_sink_utils_bdcpy(disconnect.bd_addr, p_ccb->peer_addr);
    disconnect.result = WICED_SUCCESS;
    disconnect.handle = p_ccb->p_scb->avdt_handle;

    /*TODO: */
#if 0
    /* If ACL is down and ACL is disconnected due to conneciton time out */
    if ( (HCI_ERR_CONNECTION_TOUT == btm_get_acl_disc_reason_code()) &&
         (BTM_IsAclConnectionUp(p_ccb->peer_addr, BT_TRANSPORT_BR_EDR) == WICED_FALSE))
    {
        disconnect.result = WICED_BT_TIMEOUT;
    }
#endif

    (*wiced_bt_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_DISCONNECT_EVT,
        (wiced_bt_a2dp_sink_event_data_t*) &disconnect);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_sig_open_fail
** Description      This function sends a callback event to the upper layer to
**                  notify that A2DP signaling channel fails to open
*******************************************************************************/
void wiced_bt_a2dp_sink_sig_open_fail(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_status_t connect;

    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    wiced_bt_a2dp_sink_utils_bdcpy(connect.bd_addr, p_ccb->peer_addr);
    connect.result = WICED_BT_ERROR;
    connect.handle = p_ccb->p_scb->avdt_handle;

    /* Deallocate ccb */
    wiced_bt_a2dp_sink_dealloc_ccb(p_ccb);

    (*wiced_bt_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_CONNECT_EVT,
        (wiced_bt_a2dp_sink_event_data_t*) &connect);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_str_opened
** Description      Stream opened OK (incoming).
*******************************************************************************/
void wiced_bt_a2dp_sink_str_opened(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t   *p_scb = p_ccb->p_scb;
    wiced_bt_a2dp_sink_status_t connect;
    wiced_bt_avdt_ctrl_t ctrl_data;

    p_scb->l2c_cid = wiced_bt_avdt_get_l2cap_channel(p_scb->avdt_handle);
    p_scb->stream_mtu = p_data->str_msg.msg.open_ind.peer_mtu - AVDT_MEDIA_HDR_SIZE;

    WICED_BTA2DP_TRACE("%s: l2c_cid:%d mtu:%d \n", __FUNCTION__, p_scb->l2c_cid, p_scb->stream_mtu);

    if ((p_scb->cur_psc_mask & AVDT_PSC_DELAY_RPT) && !p_scb->co_delay)
    {
        p_scb->co_delay     = WICED_TRUE;
    }

    p_scb->l2c_bufs = 0;

    if (p_scb->role & WICED_BT_A2DP_SINK_ROLE_OPEN_INT)
        p_scb->role &= ~WICED_BT_A2DP_SINK_ROLE_OPEN_INT;

    /* TODO check if other audio channel is open.
     * If yes, check if reconfig is needed
     * Right now we do not do this kind of checking.
     * A2dp sink is INT for 2nd audio connection.
     * The application needs to make sure the current codec_info is proper.
     * If one audio connection is open and another SNK attempts to connect to AV,
     * the connection will be rejected.
     */
    /* check if other audio channel is started. If yes, start */
    wiced_bt_a2dp_sink_utils_bdcpy(connect.bd_addr, p_scb->peer_addr);
    connect.result = WICED_SUCCESS;
    connect.handle = p_scb->avdt_handle;
    ctrl_data.hdr.err_param = 0; /*TODO*/
    wiced_bt_a2dp_sink_conn_cback(0, connect.bd_addr, AVDT_CONNECT_IND_EVT, &ctrl_data);
    (*wiced_bt_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_CONNECT_EVT,
        (wiced_bt_a2dp_sink_event_data_t*)&connect);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_str_open_fail
** Description      Fail to open stream in the incoming state.
*******************************************************************************/
void wiced_bt_a2dp_sink_str_open_fail(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t    *p_scb = p_ccb->p_scb;
    wiced_bt_a2dp_sink_status_t  connect;

    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    wiced_bt_a2dp_sink_utils_bdcpy(connect.bd_addr, p_scb->peer_addr);
    connect.result = WICED_BT_ERROR; /*TODO: Find a better error code */
    connect.handle = p_scb->avdt_handle;

    (*wiced_bt_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_CONNECT_EVT,
        (wiced_bt_a2dp_sink_event_data_t*) &connect);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_do_close
** Description      Close stream and signaling channel
*******************************************************************************/
void wiced_bt_a2dp_sink_do_close(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t *p_scb = p_ccb->p_scb;

    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    /* stop stream if started */
    if (p_scb->co_started)
    {
        wiced_bt_a2dp_sink_str_stopped(p_ccb, NULL);
    }

    /* close stream */
    p_scb->started = WICED_FALSE;

    /* This action function is only triggered by AP_CLOSE_EVT, set the flag */
    p_scb->is_api_close = WICED_TRUE;

    wiced_bt_avdt_close_req(p_scb->avdt_handle);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_close_str
** Description      Close stream only
*******************************************************************************/
void wiced_bt_a2dp_sink_close_str(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t *p_scb = p_ccb->p_scb;

    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    /* stop stream if started */
    if (p_scb->co_started)
    {
        wiced_bt_a2dp_sink_str_stopped(p_ccb, NULL);
    }

    /* close stream */
    p_scb->started = WICED_FALSE;
    wiced_bt_avdt_close_req(p_scb->avdt_handle);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_connect_req
** Description      Connect AVDTP signaling channel.
*******************************************************************************/
void wiced_bt_a2dp_sink_connect_req(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t *p_scb = p_ccb->p_scb;
#if 1
    uint16_t result;
#endif

#if 1
    WICED_BT_TRACE("%s (%B) \n", __FUNCTION__, p_ccb->peer_addr);
#else
    WICED_BTA2DP_TRACE("%s (%B) \n", __FUNCTION__, p_ccb->peer_addr);
#endif

    p_scb->avdt_version = p_data->sdp_res.avdt_version;
    wiced_bt_a2dp_sink_free_sdb(p_ccb, p_data);
#if 1
    result = wiced_bt_avdt_connect_req(p_ccb->peer_addr, 0, p_ccb->p_dt_cback);

    WICED_BT_TRACE("wiced_bt_avdt_connect_req result: %d\n", result);
#else
    wiced_bt_avdt_connect_req(p_ccb->peer_addr, 0/*Not Used*/, p_ccb->p_dt_cback);
#endif
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_sdp_failed
** Description      Service discovery failed.
*******************************************************************************/
void wiced_bt_a2dp_sink_sdp_failed(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_status_t connect;

    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    wiced_bt_a2dp_sink_utils_bdcpy(connect.bd_addr, p_ccb->peer_addr);
    connect.result = WICED_BT_ERROR; /*TODO: Find a better error code */
    connect.handle = p_ccb->p_scb->avdt_handle;

    /* Deallocate ccb */
    wiced_bt_a2dp_sink_dealloc_ccb(p_ccb);
    (*wiced_bt_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_CONNECT_EVT,
        (wiced_bt_a2dp_sink_event_data_t*)&connect);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_rej_conn
** Description      .
*******************************************************************************/
void wiced_bt_a2dp_sink_rej_conn(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);
    wiced_bt_avdt_config_rsp(p_data->str_msg.handle, p_data->str_msg.msg.hdr.label,
        AVDT_ERR_BAD_STATE, 0);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_open_failed
** Description      Failed to open an AVDT stream
*******************************************************************************/
void wiced_bt_a2dp_sink_open_failed(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);
    wiced_bt_a2dp_sink_cb.p_scb->open_status = WICED_BT_ERROR;
    wiced_bt_avdt_disconnect_req(p_ccb->peer_addr, p_ccb->p_dt_cback);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_sig_hdl_ap_close
** Description      This function handles the AP_CLOSE_EVT in the signaling channel
**                  opened state
*******************************************************************************/
void wiced_bt_a2dp_sink_sig_hdl_ap_close(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_do_start
** Description      Start stream.
*******************************************************************************/
void wiced_bt_a2dp_sink_do_start(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t *p_scb = p_ccb->p_scb;

    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    if (p_scb->started == WICED_FALSE)
    {
        p_scb->role |= WICED_BT_A2DP_SINK_ROLE_START_INT;

        wiced_bt_avdt_start_req(&p_scb->avdt_handle, 1);
    }
    else
    {
        wiced_bt_a2dp_sink_start_ok(p_ccb, p_data);
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_send_start_resp
** Description      Accept or Reject Start stream request.
*******************************************************************************/
void wiced_bt_a2dp_sink_send_start_resp( wiced_bt_a2dp_sink_ccb_t *p_ccb,
                                                          wiced_bt_a2dp_sink_data_t *p_data )
{
    wiced_bt_a2dp_sink_scb_t *p_scb = p_ccb->p_scb;

    /* If start response is ok */
    if ((!p_data->api_data.status) ||
        (p_data->api_data.status == WICED_SUCCESS))
    {
        p_scb->started = WICED_TRUE;

        if(p_scb->role & WICED_BT_A2DP_SINK_ROLE_START_INT)
            p_scb->role &= ~WICED_BT_A2DP_SINK_ROLE_START_INT;

        WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

        wiced_bt_a2dp_sink_stream_chg(p_scb, WICED_TRUE);
        p_scb->co_started = WICED_TRUE;

        if (p_data->api_data.status == A2D_SUCCESS_ONLY)
        {
            p_data->api_data.status = A2D_SUCCESS;
        }
         wiced_bt_avdt_start_resp( p_scb->avdt_handle, p_data->api_data.label, p_data->api_data.status );
    }
    else  /* If start response is reject */
    {
        wiced_bt_avdt_start_resp( p_scb->avdt_handle, p_data->api_data.label, p_data->api_data.status );
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_str_stopped
** Description      Stream stopped.
*******************************************************************************/
void wiced_bt_a2dp_sink_str_stopped(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t   *p_scb = p_ccb->p_scb;
    wiced_bt_a2dp_sink_status_t suspend;

    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);
    wiced_bt_a2dp_sink_utils_bdcpy(suspend.bd_addr, p_scb->peer_addr);
    suspend.handle = p_scb->avdt_handle;
#if ( BT_USE_TRACES==TRUE || BT_TRACE_PROTOCOL==TRUE)
//    BT_LOGMSG_ENABLE( TRUE );
#endif

    if (p_data)
    {
        WICED_BTA2DP_TRACE("suspending: %d, sup:%d \n", p_scb->started, p_scb->suspend_sup);
        if ((p_scb->started)  && (p_scb->suspend_sup))
        {
            p_scb->l2c_bufs = 0;
            wiced_bt_avdt_suspend_req(&p_scb->avdt_handle, 1);
        }
        else
        {
            if (p_scb->co_started)
            {
                wiced_bt_a2dp_sink_stream_chg(p_scb, WICED_FALSE);
                /* if avdtp signalling is used,
                 * callout stop will be called at wiced_bt_a2dp_sink_suspend_cfm.
                 */
                p_scb->co_started = WICED_FALSE;
            }

            {
                suspend.result = WICED_SUCCESS;

                (*wiced_bt_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_SUSPEND_EVT,
                    (wiced_bt_a2dp_sink_event_data_t*) &suspend);
            }
        }
    }
    else
    {
        if (p_scb->co_started)
        {
            wiced_bt_a2dp_sink_stream_chg(p_scb, WICED_FALSE);
            /* if avdtp signalling is used,
             * callout stop will be called at wiced_bt_a2dp_sink_suspend_cfm.
             */
            p_scb->co_started = WICED_FALSE;

            (*wiced_bt_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_SUSPEND_EVT,
                (wiced_bt_a2dp_sink_event_data_t*) &suspend);
        }
    }

}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_start_ind
** Description      Stream started.
*******************************************************************************/

void wiced_bt_a2dp_sink_start_ind(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_start_t  start_ind;

    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    start_ind.result    = WICED_SUCCESS;
    start_ind.label     = p_data->str_msg.msg.hdr.label;
    start_ind.handle    = p_ccb->p_scb->avdt_handle;
    memcpy((void *) start_ind.bdaddr, (void *) p_data->str_msg.bd_addr, sizeof(wiced_bt_device_address_t));

    (*wiced_bt_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_START_IND_EVT,
        (wiced_bt_a2dp_sink_event_data_t *) &start_ind);

#if 0
#if ( BT_USE_TRACES==TRUE || BT_TRACE_PROTOCOL==TRUE)
    /* Turn on/off logging during A2DP streaming using BT_USE_TRACES_WITH_MEDIA macro */
    BT_LOGMSG_ENABLE( BT_USE_TRACES_WITH_MEDIA );
#endif
#endif
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_start_ok
** Description      Stream started.
*******************************************************************************/
void wiced_bt_a2dp_sink_start_ok(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t   *p_scb = p_ccb->p_scb;
    wiced_bt_a2dp_sink_status_t start;

    p_scb->started = WICED_TRUE;

    if(p_scb->role & WICED_BT_A2DP_SINK_ROLE_START_INT)
        p_scb->role &= ~WICED_BT_A2DP_SINK_ROLE_START_INT;

    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    wiced_bt_a2dp_sink_stream_chg(p_scb, WICED_TRUE);
    p_scb->co_started = WICED_TRUE;

    start.result = WICED_SUCCESS;
    start.handle = p_scb->avdt_handle;
    wiced_bt_a2dp_sink_utils_bdcpy(start.bd_addr, p_scb->peer_addr);

    (*wiced_bt_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_START_CFM_EVT,
        (wiced_bt_a2dp_sink_event_data_t *) &start);

#if 0
#if ( BT_USE_TRACES==TRUE || BT_TRACE_PROTOCOL==TRUE)
    /* Turn on/off logging during A2DP streaming using BT_USE_TRACES_WITH_MEDIA macro */
    BT_LOGMSG_ENABLE( BT_USE_TRACES_WITH_MEDIA );
#endif
#endif
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_start_failed
** Description      Stream start failed.
*******************************************************************************/
void wiced_bt_a2dp_sink_start_failed(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t   *p_scb = p_ccb->p_scb;
    wiced_bt_a2dp_sink_status_t start;

    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    if(p_scb->started == WICED_FALSE && p_scb->co_started == WICED_FALSE)
    {
        start.result = WICED_ERROR;
        start.handle = p_scb->avdt_handle;
        wiced_bt_a2dp_sink_utils_bdcpy(start.bd_addr, p_scb->peer_addr);

        (*wiced_bt_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_START_CFM_EVT,
            (wiced_bt_a2dp_sink_event_data_t *) &start);
    }
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_suspend_cfm
** Description      Process the suspend response
*******************************************************************************/
void wiced_bt_a2dp_sink_suspend_cfm(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t   *p_scb = p_ccb->p_scb;
    wiced_bt_a2dp_sink_status_t suspend;
    uint8_t                     err_code = p_data->str_msg.msg.hdr.err_code;

#if ( BT_USE_TRACES==TRUE || BT_TRACE_PROTOCOL==TRUE)
//    BT_LOGMSG_ENABLE( TRUE );
#endif
    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    wiced_bt_a2dp_sink_utils_bdcpy(suspend.bd_addr, p_data->str_msg.bd_addr);
    suspend.result = WICED_SUCCESS;
    suspend.handle = p_scb->avdt_handle;

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
            wiced_bt_a2dp_sink_stream_chg(p_scb, WICED_FALSE);
            p_scb->co_started = WICED_FALSE;
        }
    }

    (*wiced_bt_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_SUSPEND_EVT,
        (wiced_bt_a2dp_sink_event_data_t*) &suspend);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_stream_chg
** Description      Audio streaming status changed.
** Parameter        Started: WICED_TRUE   if an audio/video stream starts
**                           WICED_FALSE  if an audio/video stream stops
*******************************************************************************/
void wiced_bt_a2dp_sink_stream_chg(wiced_bt_a2dp_sink_scb_t *p_scb, wiced_bool_t started)
{
    WICED_BTA2DP_TRACE("%s: started:%d \n", __FUNCTION__, started);

/* TODO - Set stream priority */
#if 0
    if (started)
    {   /* As long as there is no channel opened */
        /* Let L2CAP know this channel is processed with high priority */
        wiced_bt_l2cap_set_acl_priority_ext(p_scb->peer_addr, L2CAP_PRIORITY_HIGH,
                               L2CAP_DIRECTION_DATA_SINK);
    }
    else
    {
        /* Let L2CAP know this channel is processed with low priority */
        wiced_bt_l2cap_set_acl_priority_ext(p_scb->peer_addr, L2CAP_PRIORITY_NORMAL,
                               L2CAP_DIRECTION_DATA_SINK);
    }
#endif

}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_hdl_str_close
** Description      This function handles disconnection of A2DP streaming channel
*******************************************************************************/
void wiced_bt_a2dp_sink_hdl_str_close(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_scb_t   *p_scb = p_ccb->p_scb;
    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    /* Do stop if we were started */
    if (p_scb->co_started)
    {
        wiced_bt_a2dp_sink_str_stopped(p_ccb, NULL);
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
** Function         wiced_bt_a2dp_sink_sig_hdl_ap_close_disconnect_req
** Description      Perform sig handle close and then disconnect
*******************************************************************************/
void wiced_bt_a2dp_sink_sig_hdl_ap_close_disconnect_req(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    wiced_bt_a2dp_sink_sig_hdl_ap_close(p_ccb, p_data);
    wiced_bt_a2dp_sink_disconnect_req(p_ccb, p_data);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_sink_sig_closed_cleanup
** Description      Perform sig closed and then cleanup
*******************************************************************************/
void wiced_bt_a2dp_sink_sig_closed_cleanup(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data)
{
    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);
    wiced_bt_a2dp_sink_sig_closed(p_ccb, p_data);
    wiced_bt_a2dp_sink_cleanup(p_ccb, p_data);
    /* Deallocate ccb */
    wiced_bt_a2dp_sink_dealloc_ccb(p_ccb);
}
