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
 * This is the main implementation file for the audio source.
 */

#include "wiced_bt_sdp.h"
#include "wiced_bt_a2dp_src_int.h"
#include "wiced_memory.h"

/*****************************************************************************
** Global data
*****************************************************************************/

/* A2DP source control block */
wiced_bt_a2dp_source_cb_t wiced_bt_a2dp_source_cb;

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_conn_cback
**
** Description      This function handles AVDTP callback events from the underlying layer.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_a2dp_source_conn_cback(uint8_t handle,
    wiced_bt_device_address_t bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data)
{
    wiced_bt_a2dp_source_str_msg_t  str_msg;
    uint16_t                      evt = 0;
    WICED_BTA2DP_SRC_TRACE("%s: %d \n", __FUNCTION__, event);

    if (p_data == NULL)
    {
        WICED_BTA2DP_SRC_ERROR ("%s p_data NULL!! \n", __FUNCTION__);
        return;
    }

    memset(&str_msg, 0, sizeof(wiced_bt_a2dp_source_str_msg_t));

    if (event == AVDT_CONNECT_IND_EVT || event == AVDT_DISCONNECT_IND_EVT)
    {
        evt = WICED_BT_A2DP_SOURCE_SIG_CHG_EVT;
        if (AVDT_CONNECT_IND_EVT == event)
        {
            WICED_BTA2DP_SRC_TRACE("%s: CONN_IND is ACP:%d \n", __FUNCTION__, p_data->hdr.err_param);
        }
        str_msg.avdt_event = (uint8_t)event;
        str_msg.hdr.err_param = p_data->hdr.err_param;
        str_msg.handle = handle;
        memcpy(str_msg.bd_addr, bd_addr, sizeof(wiced_bt_device_address_t));
        WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);
        WICED_BTA2DP_SRC_TRACE("bd_addr:%02x-%02x-%02x-%02x-%02x-%02x \n",
                      bd_addr[0], bd_addr[1],
                      bd_addr[2], bd_addr[3],
                      bd_addr[4], bd_addr[5]);
        wiced_bt_a2dp_source_hdl_event((uint8_t)evt, (wiced_bt_avdt_evt_hdr_t *)&str_msg.hdr);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_init_ccb
**
** Description      Initialize connection control block
**
** Returns          void
**
*******************************************************************************/
static void wiced_bt_a2dp_source_init_ccb(void)
{
    uint8_t   idx;

    /* Initialize connection control block -  - each with the same stream control block */
    for (idx = 0; idx < WICED_BT_A2DP_SOURCE_MAX_NUM_CONN; ++idx)
    {
        wiced_bt_a2dp_source_cb.ccb[idx].in_use = WICED_FALSE;
        wiced_bt_a2dp_source_cb.ccb[idx].p_dt_cback = wiced_bt_a2dp_source_ctrl_cback;
    }
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_api_deinit
**
** Description      De-register audio stream
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_a2dp_source_api_deinit(uint16_t avdt_event, wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_scb_t *p_scb = wiced_bt_a2dp_source_cb.p_scb;
    uint8_t idx;

    if (p_scb)
    {
        p_scb->deregistring = WICED_TRUE;
        for (idx = 0; idx < WICED_BT_A2DP_SOURCE_MAX_NUM_CONN; idx++)
        {
            wiced_bt_a2dp_source_ssm_execute(&wiced_bt_a2dp_source_cb.ccb[idx],
                p_data, WICED_BT_A2DP_SOURCE_API_DISCONNECT_EVT);
        }
        wiced_bt_a2dp_source_dealloc_scb(p_scb);
        wiced_bt_a2dp_source_cb.is_init = WICED_FALSE;
    }
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_alloc_ccb
**
** Description      Allocate a connnection control block
**
** Returns          void
**
*******************************************************************************/
static wiced_bt_a2dp_source_ccb_t *wiced_bt_a2dp_source_alloc_ccb(
    wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_ccb_t *p_ccb = NULL;
    uint8_t idx;

    for(idx = 0; idx < WICED_BT_A2DP_SOURCE_MAX_NUM_CONN; idx++)
    {
        if(wiced_bt_a2dp_source_cb.ccb[idx].in_use == WICED_FALSE)
        {
            wiced_bt_a2dp_source_cb.ccb[idx].in_use     = WICED_TRUE;
            wiced_bt_a2dp_source_cb.ccb[idx].ccb_handle = idx;
            wiced_bt_a2dp_source_cb.ccb[idx].p_scb      = &(wiced_bt_a2dp_source_cb.p_scb[idx]);
            wiced_bt_a2dp_source_cb.ccb[idx].p_dt_cback = wiced_bt_a2dp_source_ctrl_cback;

            wiced_bt_a2dp_source_utils_bdcpy(wiced_bt_a2dp_source_cb.ccb[idx].peer_addr,
                                           p_data->api_data.bd_address);

            WICED_BTA2DP_SRC_TRACE("%s: ccb_handle:%d \n", __FUNCTION__,
                wiced_bt_a2dp_source_cb.ccb[idx].ccb_handle);

            p_ccb = &wiced_bt_a2dp_source_cb.ccb[idx];

            break;
        }
    }
    if(p_ccb == NULL)
        WICED_BTA2DP_SRC_TRACE("%s:out of ccb \n", __FUNCTION__);

    return p_ccb;
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_dealloc_ccb
**
** Description      Deallocate a connnection control block
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_a2dp_source_dealloc_ccb(wiced_bt_a2dp_source_ccb_t *p_ccb)
{
    memset(p_ccb, 0, sizeof(wiced_bt_a2dp_source_ccb_t));
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_dealloc_scb
**
** Description      Deallocate a stream control block
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_a2dp_source_dealloc_scb(wiced_bt_a2dp_source_scb_t *p_scb)
{
    uint8_t    index;

    for (index = 0; index < WICED_BT_A2DP_SOURCE_MAX_NUM_CONN; index++)
    {
        memset(&p_scb[index], 0, sizeof(wiced_bt_a2dp_source_scb_t));
    }
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_dereg_comp
**
** Description      Deregister complete. free the stream control block.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_a2dp_source_dereg_comp(void)
{
    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);

    wiced_bt_avdt_deregister();
    wiced_bt_a2dp_source_dealloc_scb(wiced_bt_a2dp_source_cb.p_scb);
}


/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_get_ccb_by_bd_addr
**
** Description      Get the connnection control block by bd_addr
**
** Returns          void
**
*******************************************************************************/
wiced_bt_a2dp_source_ccb_t *wiced_bt_a2dp_source_get_ccb_by_bd_addr(
    wiced_bt_device_address_t bd_addr)
{
    wiced_bt_a2dp_source_ccb_t *p_ccb = NULL;
    uint8_t idx;

    for(idx = 0; idx < WICED_BT_A2DP_SOURCE_MAX_NUM_CONN; idx++)
    {
        if(wiced_bt_a2dp_source_cb.ccb[idx].in_use == WICED_TRUE &&
            !wiced_bt_a2dp_source_utils_bdcmp(wiced_bt_a2dp_source_cb.ccb[idx].peer_addr, bd_addr))
        {
            WICED_BTA2DP_SRC_TRACE("wiced_bt_a2dp_source_get_ccb_by_bd_addr idx:%d \n", idx);
            p_ccb = &wiced_bt_a2dp_source_cb.ccb[idx];
            break;
        }
    }
    if(p_ccb == NULL)
    {
        WICED_BTA2DP_SRC_TRACE("%s: cannot find ccb \n", __FUNCTION__);
    }

    return p_ccb;
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_get_ccb_by_handle
**
** Description      Get the connnection control block by handle
**
** Returns          void
**
*******************************************************************************/
static wiced_bt_a2dp_source_ccb_t *wiced_bt_a2dp_source_get_ccb_by_handle(uint16_t handle)
{
    wiced_bt_a2dp_source_ccb_t *p_ccb = NULL;
    uint8_t idx;

    for(idx = 0; idx < WICED_BT_A2DP_SOURCE_MAX_NUM_CONN; idx++)
    {
        if(wiced_bt_a2dp_source_cb.ccb[idx].in_use == WICED_TRUE &&
            wiced_bt_a2dp_source_cb.ccb[idx].p_scb->avdt_handle == handle)
        {
            WICED_BTA2DP_SRC_TRACE("wiced_bt_a2dp_source_get_ccb_by_bd_handle idx:%d\n", idx);
            p_ccb = &wiced_bt_a2dp_source_cb.ccb[idx];
            break;
        }
    }
    if(p_ccb == NULL)
    {
        WICED_BTA2DP_SRC_TRACE("%s: cannot find ccb\n", __FUNCTION__);
    }

    return p_ccb;
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_get_ccb
**
** Description      Get the connnection control block for different events
**
** Returns          Connection control block
**
*******************************************************************************/
static wiced_bt_a2dp_source_ccb_t *wiced_bt_a2dp_source_get_ccb(uint16_t event,
    wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_ccb_t *p_ccb = NULL;

    switch (event)
    {
    case WICED_BT_A2DP_SOURCE_API_CONNECT_EVT:
        p_ccb = wiced_bt_a2dp_source_alloc_ccb(p_data);
        break;
    case WICED_BT_A2DP_SOURCE_API_DISCONNECT_EVT:
        p_ccb = wiced_bt_a2dp_source_get_ccb_by_handle(p_data->api_data.handle);
        break;
    case WICED_BT_A2DP_SOURCE_SDP_DISC_OK_EVT:
    case WICED_BT_A2DP_SOURCE_SDP_DISC_FAIL_EVT:
        p_ccb = wiced_bt_a2dp_source_get_ccb_by_bd_addr(wiced_bt_a2dp_source_cb.sdp_bd_addr);
        break;
    case WICED_BT_A2DP_SOURCE_AVDT_CONNECT_EVT:
    case WICED_BT_A2DP_SOURCE_AVDT_DISCONNECT_EVT:
        p_ccb = wiced_bt_a2dp_source_get_ccb_by_bd_addr(p_data->str_msg.bd_addr);
        break;

    case WICED_BT_A2DP_SOURCE_STR_CLOSE_IND_EVT:
        p_ccb = wiced_bt_a2dp_source_get_ccb_by_bd_addr(p_data->str_msg.bd_addr);
        break;

    case WICED_BT_A2DP_SOURCE_STR_CLOSE_CFM_EVT:
        if ((p_ccb = wiced_bt_a2dp_source_get_ccb_by_bd_addr(p_data->str_msg.bd_addr)) != NULL)
            p_ccb->is_str_active = WICED_FALSE;
        break;

    case WICED_BT_A2DP_SOURCE_STR_OPEN_OK_EVT:
    case WICED_BT_A2DP_SOURCE_STR_OPEN_FAIL_EVT:
    case WICED_BT_A2DP_SOURCE_STR_START_IND_EVT:
    case WICED_BT_A2DP_SOURCE_STR_START_CFM_EVT:
    case WICED_BT_A2DP_SOURCE_STR_START_FAIL_EVT:
    case WICED_BT_A2DP_SOURCE_STR_SUSPEND_CFM_EVT:
    case WICED_BT_A2DP_SOURCE_STR_DISCOVER_CFM_EVT:
    case WICED_BT_A2DP_SOURCE_STR_GETCAP_CFM_EVT:
    case WICED_BT_A2DP_SOURCE_STR_CONFIG_IND_EVT:
        p_ccb = wiced_bt_a2dp_source_get_ccb_by_bd_addr(p_data->str_msg.bd_addr);
        break;

    case WICED_BT_A2DP_SOURCE_STR_RECONFIG_CFM_EVT:
        p_ccb = wiced_bt_a2dp_source_get_ccb_by_handle(p_data->str_msg.handle);
        memcpy(p_data->str_msg.bd_addr,p_ccb->peer_addr,BD_ADDR_LEN);
        break;

    case WICED_BT_A2DP_SOURCE_API_START_EVT:
        WICED_BT_TRACE ("Start req event %d handle \n",p_data->start_req.handle);
        p_ccb = wiced_bt_a2dp_source_get_ccb_by_handle(p_data->start_req.handle);
        break;

    case WICED_BT_A2DP_SOURCE_API_START_RESP_EVT:
    case WICED_BT_A2DP_SOURCE_API_SUSPEND_EVT:
        p_ccb = wiced_bt_a2dp_source_get_ccb_by_handle(p_data->api_data.handle);
        break;

    default:
#if (defined(WICED_BT_A2DP_SOURCE_DEBUG) && WICED_BT_A2DP_SOURCE_DEBUG == TRUE)
        WICED_BTA2DP_SRC_TRACE("%s: Unknown event %s \n", __FUNCTION__,
            wiced_bt_a2dp_source_evt_code(event));
#endif
        break;
    }
    return p_ccb;
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_sig_chg
**
** Description      Process AVDT signal channel up/down.
**
** Returns          void
**
*******************************************************************************/
static void wiced_bt_a2dp_source_sig_change(uint16_t avdt_event, wiced_bt_a2dp_source_data_t *p_data)
{
    wiced_bt_a2dp_source_ccb_t *p_ccb = NULL;

    WICED_BTA2DP_SRC_TRACE("%s event: %d bd_addr:%02x-%02x-%02x-%02x-%02x-%02x\n", __FUNCTION__, avdt_event,
                      p_data->str_msg.bd_addr[0], p_data->str_msg.bd_addr[1],
                      p_data->str_msg.bd_addr[2], p_data->str_msg.bd_addr[3],
                      p_data->str_msg.bd_addr[4], p_data->str_msg.bd_addr[5]);

    if (p_data->str_msg.avdt_event == AVDT_CONNECT_IND_EVT)
    {
        //if (p_data->str_msg.hdr.offset == AVDT_ACP)
        {
            WICED_BTA2DP_SRC_TRACE("Incoming L2CAP acquired, set state as sig open \n");

            if ((p_ccb = wiced_bt_a2dp_source_get_ccb_by_bd_addr(p_data->str_msg.bd_addr)) == NULL)
            {
                /* If cannot find, connection is initiated by peer, allocate a new ccb */
                wiced_bt_a2dp_source_utils_bdcpy(p_data->api_data.bd_address,
                                               p_data->str_msg.bd_addr);
                p_ccb = wiced_bt_a2dp_source_alloc_ccb(p_data);
            }

            if (p_ccb)
            {
                p_ccb->p_scb->is_accepter = WICED_TRUE;
                p_ccb->avdt_handle = p_data->str_msg.handle;
                wiced_bt_a2dp_source_ssm_execute(p_ccb, p_data,
                                        WICED_BT_A2DP_SOURCE_AVDT_CONNECT_EVT);
            }
        }

    }
    else
    {
        /* Disconnected. */
        if ((p_ccb = wiced_bt_a2dp_source_get_ccb_by_bd_addr(p_data->str_msg.bd_addr)) != NULL)
        {
            wiced_bt_a2dp_source_ssm_execute(p_ccb, NULL, WICED_BT_A2DP_SOURCE_AVDT_DISCONNECT_EVT);
        }
    }
}


/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_report_conn
**
** Description      AVDT Reporting channel connected/disconnected handler function
**
** Returns          void
**
*******************************************************************************/
static void wiced_bt_a2dp_source_report_conn(uint16_t avdt_event ,wiced_bt_a2dp_source_data_t *p_data)
{
    WICED_BTA2DP_SRC_TRACE("%s \n", __FUNCTION__);
}

/* Non state machine action functions */
const wiced_bt_a2sp_source_nsm_act_t wiced_bt_a2dp_source_nsm_act[] =
{
    NULL,                           /**< WICED_BT_A2DP_SOURCE_INVALID_EVT */
    wiced_bt_a2dp_source_api_deinit,  /**< WICED_BT_A2DP_SOURCE_API_DEINIT_EVT */
    wiced_bt_a2dp_source_sig_change,  /**< WICED_BT_A2DP_SOURCE_SIG_CHG_EVT */
    wiced_bt_a2dp_source_report_conn, /**< WICED_BT_A2DP_SOURCE_AVDT_REPOPT_CONN_EVT */
};

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_hdl_event
**
** Description      Audio source main event handling function.
**
** Returns          wiced_bool_t
**
*******************************************************************************/
void wiced_bt_a2dp_source_hdl_event(uint8_t event, wiced_bt_avdt_evt_hdr_t *p_msg)
{
    wiced_bt_a2dp_source_data_t *p_data = (wiced_bt_a2dp_source_data_t *)p_msg;
    wiced_bt_a2dp_source_ccb_t *p_ccb = NULL;

    if( (event == WICED_BT_A2DP_SOURCE_INVALID_EVT) || (event >= WICED_BT_A2DP_SOURCE_LAST_EVENT) )
    {
        return;
    }

    if(event < WICED_BT_A2DP_SOURCE_GLOBAL_EVT_END)
    {
        /* Non state machine events */
        (*wiced_bt_a2dp_source_nsm_act[event])(event, p_data);
    }
    else
    {
        /* State machine events */
        if ((p_ccb = wiced_bt_a2dp_source_get_ccb(event, p_data)) != NULL)
        {
            wiced_bt_a2dp_source_ssm_execute(p_ccb, p_data, event);
        }
        else
        {
            WICED_BTA2DP_SRC_TRACE("%s No CCB block for Event:%d\n",__FUNCTION__, event);
        }
    }

    /* TODO: need to delete the mailbox message here since there is no actual MBOX.
     *       If we ever go back to multithreaded need to remove the line below.
     */
    // GKI_freebuf(p_msg);
}

/*******************************************************************************
** Function         wiced_bt_a2dp_source_reg_a2dp
**
** Description      Registers A2DP, create A2DP stream
**
** Returns          void
*******************************************************************************/
static void wiced_bt_a2dp_source_reg_a2dp(wiced_bt_avdt_cs_t *p_cs, wiced_bt_a2dp_source_scb_t *p_scb)
{
    uint8_t                    index = 0;
    uint8_t                    c_index =0;
    uint8_t                    codec_count = 0;
    uint8_t                    sep_index = 0;

    WICED_BTA2DP_SRC_TRACE("%s: feature mask 0x%04x \n", __FUNCTION__,
        wiced_bt_a2dp_source_cb.p_config_data->feature_mask);

    /* Set up the audio stream control block */

    for (index = 0; index < WICED_BT_A2DP_SOURCE_MAX_NUM_CONN; index++)
    {
        p_scb[index].media_type = AVDT_MEDIA_AUDIO;
    }

    /* Keep the configuration in the stream control block */
    for (index = 0; index < WICED_BT_A2DP_SOURCE_MAX_NUM_CONN; index++)
    {
        /* for each scb, configure */
        memcpy(&(p_scb[index].cfg), &p_cs->cfg, sizeof(wiced_bt_avdt_cfg_t));
    }

    /* how many codecs are there */
    codec_count = wiced_bt_a2dp_source_cb.p_config_data->codec_capabilities.count;

    for ( index = 0; index < WICED_BT_A2DP_SOURCE_MAX_NUM_CONN; index++ )
    {
        WICED_BTA2DP_SRC_TRACE("index %d codec_count:%d\n", index, codec_count );
        for ( c_index = 0; c_index < codec_count; c_index++ )
        {
            wiced_bool_t ret = WICED_FALSE;
            ret = wiced_bt_a2dp_source_cfg_init(&wiced_bt_a2dp_source_cb.p_config_data->codec_capabilities.info[c_index],
                                              p_cs->cfg.codec_info,
                                              &p_cs->cfg.num_protect,
                                              p_cs->cfg.protect_info);
            if ( ret == WICED_TRUE )
            {
                if( ( sep_index < WICED_BT_A2DP_SOURCE_MAX_SEPS ) && (wiced_bt_avdt_create_stream(&(wiced_bt_a2dp_source_cb.seps[sep_index].av_handle), p_cs) == AVDT_SUCCESS ) )
                {
                    wiced_bt_a2dp_source_cb.seps[sep_index].codec_type = wiced_bt_a2dp_source_cb.p_config_data->codec_capabilities.info[c_index].codec_id;
                    wiced_bt_a2dp_source_cb.seps[sep_index].codec_cap_index = c_index;
                    WICED_BTA2DP_SRC_TRACE("%s: audio[%d] av_handle: %d codec_type: %d \n", __FUNCTION__,
                            sep_index, wiced_bt_a2dp_source_cb.seps[sep_index].av_handle, wiced_bt_a2dp_source_cb.seps[sep_index].codec_type);
                    sep_index++;
                }
            }
        }
    }
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_register
**
** Description      Allocate stream control block and registers the service to stack
**
** Returns          wiced_result_t
**
*******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_register(void)
{
    wiced_bt_a2dp_source_scb_t   *p_scb = NULL;
    wiced_bt_avdt_reg_t         reg;
    wiced_bt_avdt_cs_t          cs;
    uint16_t                    count = 0;

    /* First Initialize the stream control block */
    p_scb = wiced_bt_a2dp_source_cb.p_scb;

    /* Now. Initialize the connection control blocks */
    wiced_bt_a2dp_source_init_ccb();

    /* Register to AVDTP */
    reg.ctrl_mtu = WICED_BT_A2DP_SOURCE_AVDT_SIG_CH_MTU;
    reg.ret_tout = WICED_BT_A2DP_SOURCE_RET_TOUT;
    reg.sig_tout = WICED_BT_A2DP_SOURCE_SIG_TOUT;
    reg.idle_tout= WICED_BT_A2DP_SOURCE_IDLE_TOUT;
    reg.sec_mask = 0; /* NOT USED */
    wiced_bt_avdt_register(&reg, wiced_bt_a2dp_source_conn_cback);

    /* Get stream configuration and create stream */
    memset(&cs.cfg, 0, sizeof(wiced_bt_avdt_cfg_t));
    cs.cfg.psc_mask  = AVDT_PSC_TRANS;
    cs.media_type    = AVDT_MEDIA_AUDIO;
    cs.p_avdt_ctrl_cback  = wiced_bt_a2dp_source_ctrl_cback;
    cs.p_avdt_data_cback = NULL;

    cs.cfg.num_codec  = 1;
    cs.cfg.num_protect  = 0;
    cs.tsep           = AVDT_TSEP_SRC;
#if AVDT_REPORTING == TRUE
    cs.p_report_cback = NULL;
#endif
    if (wiced_bt_a2dp_source_cb.p_config_data->feature_mask & WICED_BT_A2DP_SOURCE_FEAT_DELAY_RPT)
    {
        if (AVDT_VERSION >= AVDT_VERSION_DELAYREPORT)
        {
            cs.cfg.psc_mask |= AVDT_PSC_DELAY_RPT;
        }
        else
        {
            WICED_BTA2DP_SRC_TRACE("AVDTP version 0x%04x does not support"
                                " desired feature 0x%x \n",
                                 AVDT_VERSION, wiced_bt_a2dp_source_cb.p_config_data->feature_mask);
        }
    }

    if (wiced_bt_a2dp_source_cb.p_config_data->feature_mask & WICED_BT_A2DP_SOURCE_FEAT_PROTECT)
    {
        if (AVDT_VERSION >= AVDT_VERSION_CP)
        {
            cs.cfg.psc_mask |= AVDT_PSC_CP;
            cs.cfg.num_protect = 1;
        }
        else
        {
            WICED_BTA2DP_SRC_TRACE("AVDTP version 0x%04x does not support"
                                " desired feature 0x%x \n",
                                AVDT_VERSION, wiced_bt_a2dp_source_cb.p_config_data->feature_mask);
        }
    }

    /* Suspend and reconfig is supported for each scb.*/
    for (count = 0; count < WICED_BT_A2DP_SOURCE_MAX_NUM_CONN; count++)
    {
        p_scb[count].suspend_sup  = WICED_TRUE;
        p_scb[count].recfg_sup    = WICED_TRUE;
    }

    wiced_bt_a2dp_source_reg_a2dp(&cs, p_scb);
    return WICED_SUCCESS;
}
