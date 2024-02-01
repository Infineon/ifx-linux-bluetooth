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
 * This is the main implementation file for the handsfree.
 */

#include <string.h>
#include "wiced_bt_hfp_hf_int.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "wiced_memory.h"
#if defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW20706A2)
#include "wiced_bt_event.h"
#endif
#include "wiced_bt_utils.h"

/*****************************************************************************
** Global data
*****************************************************************************/

/* Handsfree control block */
wiced_bt_hfp_hf_cb_t  wiced_bt_hfp_hf_cb;

wiced_bt_hfp_rfcomm_server_data_t wiced_rfcomm_server_data[WICED_BT_HFP_HF_MAX_CONN];

/*******************************************************************************
** Function         wiced_bt_hfp_hf_api_deinit
** Description      De-init Handsfre
** Returns          void
*******************************************************************************/
void wiced_bt_hfp_hf_api_deinit(wiced_bt_hfp_hf_data_t *p_data)
{
    wiced_bt_hfp_hf_scb_t *p_scb = NULL;

    uint8_t i=0;
    for(i=0; i<WICED_BT_HFP_HF_MAX_CONN; i++)
    {
        p_scb = &wiced_bt_hfp_hf_cb.scb[i];
        if(p_scb->in_use == TRUE)
        {
            /*TODO: If active client, then issue disconnect. If server, close the server */
        }
    }
    wiced_bt_hfp_hf_cb.is_init = FALSE;
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_alloc_scb
** Description      Retrieves a free service control block and marks it in use.
** Returns          Service control block if allocated, else NULL
*******************************************************************************/
wiced_bt_hfp_hf_scb_t *wiced_bt_hfp_hf_scb_alloc(void)
{
    wiced_bt_hfp_hf_scb_t *p_scb = NULL;
    uint8_t                i = 0;

    for(i=0; i<WICED_BT_HFP_HF_MAX_CONN; i++)
    {
        if(wiced_bt_hfp_hf_cb.scb[i].in_use == FALSE)
        {
            p_scb = &wiced_bt_hfp_hf_cb.scb[i];
            memset(p_scb, 0, sizeof(wiced_bt_hfp_hf_scb_t));
            wiced_bt_hfp_hf_at_cmd_queue_init(p_scb);
            p_scb->in_use = TRUE;

            WICED_BTHFP_TRACE("%s: scb allocated:%x\n", __FUNCTION__, p_scb);
            break;
        }
    }
    if(p_scb == NULL)
    {
        WICED_BTHFP_ERROR("%s:out of scb\n", __FUNCTION__);
    }
    return p_scb;
}


/*******************************************************************************
** Function         wiced_bt_hfp_hf_get_scb_by_handle
** Description      Given an RFCOMM handle, return pointer to scb.
** Returns          Pointer to scb or NULL if not allocated.
*******************************************************************************/
wiced_bt_hfp_hf_scb_t *wiced_bt_hfp_hf_get_scb_by_handle(uint16_t handle)
{
    wiced_bt_hfp_hf_scb_t *p_scb = NULL;
    uint8_t                i = 0;

    for(i=0; i<WICED_BT_HFP_HF_MAX_CONN; i++)
    {
        if(wiced_bt_hfp_hf_cb.scb[i].in_use == TRUE &&
            wiced_bt_hfp_hf_cb.scb[i].rfcomm_handle == handle)
        {
            p_scb = &wiced_bt_hfp_hf_cb.scb[i];
            break;
        }
    }

    if(p_scb == NULL)
    {
        WICED_BTHFP_ERROR("%s: cannot find scb\n", __FUNCTION__);
    }

    return p_scb;
}


/*******************************************************************************
** Function         wiced_bt_hfp_hf_get_scb_by_bd_addr
** Description      Get the service control block by bd_addr
** Returns          void
*******************************************************************************/
wiced_bt_hfp_hf_scb_t *wiced_bt_hfp_hf_get_scb_by_bd_addr(
    wiced_bt_device_address_t bd_addr)
{
    wiced_bt_hfp_hf_scb_t *p_scb = NULL;
    uint8_t                i = 0;

    for(i=0; i<WICED_BT_HFP_HF_MAX_CONN; i++)
    {
        if(wiced_bt_hfp_hf_cb.scb[i].in_use == TRUE &&
            !wiced_bt_hfp_hf_utils_bdcmp(wiced_bt_hfp_hf_cb.scb[i].peer_addr, bd_addr))
        {
            p_scb = &wiced_bt_hfp_hf_cb.scb[i];
            break;
        }
    }

    if(p_scb == NULL)
    {
        WICED_BTHFP_ERROR("%s: cannot find scb\n", __FUNCTION__);
    }

    return p_scb;
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_get_uuid_by_handle
** Description      Given an RFCOMM conn handle, return uuid.
** Returns          UUID or 0 if not allocated.
*******************************************************************************/
uint16_t wiced_bt_hfp_hf_get_uuid_by_handle(uint16_t handle)
{
    uint16_t uuid = 0;
    uint8_t  i = 0;

    for(i=0; i<WICED_BT_HFP_HF_MAX_CONN; i++)
    {
        if( wiced_rfcomm_server_data[i].server_handle == handle )
        {
            uuid = wiced_rfcomm_server_data[i].uuid;
        }
    }
    return uuid;
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_get_uuid_by_scn
** Description      Given an scn id, return uuid.
** Returns          UUID or 0 if not allocated.
*******************************************************************************/
uint16_t wiced_bt_hfp_hf_get_uuid_by_scn(uint8_t scn)
{
    uint16_t uuid = 0;
    uint8_t  i = 0;

    for(i=0; i<WICED_BT_HFP_HF_MAX_CONN; i++)
    {
        if( wiced_rfcomm_server_data[i].scn == scn )
        {
            uuid = wiced_rfcomm_server_data[i].uuid;
        }
    }
    return uuid;
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_scb_dealloc
** Description      Deallocate a service control block.
** Returns          void
*******************************************************************************/
void wiced_bt_hfp_hf_scb_dealloc(wiced_bt_hfp_hf_scb_t *p_scb)
{
    WICED_BTHFP_TRACE("%s\n", __FUNCTION__);
    wiced_bt_hfp_hf_at_cmd_queue_free(p_scb);
    p_scb->in_use = FALSE;
    /* Free SDB block */
    if(p_scb->p_sdp_db != NULL)
    {
        wiced_bt_free_buffer(p_scb->p_sdp_db);
        p_scb->p_sdp_db = NULL;
    }
}

void wiced_bt_hfp_set_rfcomm_server_data (uint16_t handle, uint8_t scn, uint16_t uuid)
{
    int index;
    for (index = 0; index<WICED_BT_HFP_HF_MAX_CONN ; index++)
    {
        if( wiced_rfcomm_server_data[index].server_handle == 0 )
        {
            wiced_rfcomm_server_data[index].server_handle = handle;
            wiced_rfcomm_server_data[index].scn = scn;
            wiced_rfcomm_server_data[index].uuid = uuid;
            break;
        }
    }

}

wiced_bool_t wiced_bt_hfp_close_rfcomm_server_port()
{
    int index;
    for (index = 0; index<WICED_BT_HFP_HF_MAX_CONN ; index++)
    {
        // make sure that server port is open and not used by profile
        if( (wiced_rfcomm_server_data[index].server_handle != 0) &&
                (wiced_bt_hfp_hf_get_scb_by_handle(wiced_rfcomm_server_data[index].server_handle) == NULL) )
        {
            if (wiced_bt_rfcomm_remove_connection (wiced_rfcomm_server_data[index].server_handle,WICED_TRUE) == WICED_BT_RFCOMM_SUCCESS)
            {
                wiced_rfcomm_server_data[index].server_handle = 0;
                return WICED_TRUE;
            }
        }
    }
    return WICED_FALSE;
}

int wiced_bt_hfp_serialize_hf_register ( void *data )
{
    int scn;
    uint16_t uuid;

    if (!data)
        return 0;

    scn = *(int *)data;

    uuid = wiced_bt_hfp_hf_get_uuid_by_scn(scn);

    wiced_bt_hfp_hf_register(scn,uuid);
    return 0;
}

wiced_bool_t wiced_bt_hfp_open_rfcomm_server_port()
{
    int index;

    for (index = 0; index<WICED_BT_HFP_HF_MAX_CONN ; index++)
    {
        if( wiced_rfcomm_server_data[index].server_handle == 0 )
        {
           // wiced_app_event_serialize(wiced_bt_hfp_serialize_hf_register, &(wiced_rfcomm_server_data[index].scn));
        	//Replaced to wiced_app_event_serialize
            wiced_bt_hfp_serialize_hf_register(&wiced_rfcomm_server_data[index].scn);
            return WICED_TRUE;
        }
    }
    return WICED_FALSE;
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_register
** Description      Allocate stream control block and registers the service to stack
** Returns          wiced_result_t
*******************************************************************************/
wiced_result_t wiced_bt_hfp_hf_register(uint8_t scn, uint16_t uuid)
{
    wiced_bt_rfcomm_result_t res = WICED_BT_RFCOMM_SUCCESS;
    uint16_t                 handle = 0;

    res = wiced_bt_rfcomm_create_connection(uuid,
            scn, WICED_TRUE, WICED_BT_HFP_HF_MTU, bd_addr_any, &handle,
            wiced_bt_hfp_hf_rfcomm_mgmt_cback);

    if(res != WICED_BT_RFCOMM_SUCCESS)
    {
        WICED_BTHFP_ERROR("%s: Failed to create RFCOMM server\n", __FUNCTION__);
        return res;
    }

    res = wiced_bt_rfcomm_set_event_callback(handle,
        wiced_bt_hfp_hf_rfcomm_data_cback, wiced_bt_hfp_hf_rfcomm_port_tx_cmpl_cback);
    if(res != WICED_BT_RFCOMM_SUCCESS)
    {
        wiced_bt_rfcomm_remove_connection(handle, TRUE);
        WICED_BTHFP_ERROR("%s: Failed to set RFCOMM data callback\n", __FUNCTION__);
        return res;
    }

    res = wiced_bt_rfcomm_set_event_mask(handle, WICED_BT_HFP_HF_PORT_EV_MASK);
    if(res != WICED_BT_RFCOMM_SUCCESS)
    {
        wiced_bt_rfcomm_remove_connection(handle, TRUE);
        WICED_BTHFP_ERROR("%s: Failed to set RFCOMM mask\n", __FUNCTION__);
        return res;
    }

    //Store handle and scn and UUID
    wiced_bt_hfp_set_rfcomm_server_data ( handle, scn, uuid);

    return WICED_SUCCESS;
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_get_scb
** Description      Get the service control block for different events
** Returns          Service control block
*******************************************************************************/
static wiced_bt_hfp_hf_scb_t *wiced_bt_hfp_hf_get_scb(uint16_t event,
    wiced_bt_hfp_hf_data_t *p_data)
{
    wiced_bt_hfp_hf_scb_t *p_scb = NULL;

    switch (event)
    {
    case WICED_BT_HFP_HF_API_CONNECT_EVT:
        p_scb = wiced_bt_hfp_hf_scb_alloc();
        if(p_scb != NULL)
        {
            wiced_bt_hfp_hf_utils_bdcpy(p_scb->peer_addr, p_data->api_data.bd_address);
        }
        break;
    case WICED_BT_HFP_HF_API_DISCONNECT_EVT:
        p_scb = wiced_bt_hfp_hf_get_scb_by_handle(p_data->api_data.handle);
        break;
    case WICED_BT_HFP_HF_SDP_DISC_OK_EVT:
    case WICED_BT_HFP_HF_SDP_DISC_FAIL_EVT:
        p_scb = wiced_bt_hfp_hf_get_scb_by_bd_addr(wiced_bt_hfp_hf_cb.sdp_bd_addr);
        break;
    case WICED_BT_HFP_HF_RFC_DATA_EVT:
    case WICED_BT_HFP_HF_RFC_CONNECT_EVT:
    case WICED_BT_HFP_HF_RFC_DISCONNECT_EVT:
        p_scb = wiced_bt_hfp_hf_get_scb_by_handle(p_data->rfc_data.handle);
        break;
    case WICED_BT_HFP_HF_RFC_CONNECT_FAIL_EVT:
        p_scb = wiced_bt_hfp_hf_get_scb_by_bd_addr(p_data->connect_fail.bd_address);
        break;
    case WICED_BT_HFP_HF_API_CALL_ACTION_EVT:
        p_scb = wiced_bt_hfp_hf_get_scb_by_handle(p_data->api_call_action.handle);
        break;
    case WICED_BT_HFP_HF_API_NOTIFY_VOLUME_EVT:
        p_scb = wiced_bt_hfp_hf_get_scb_by_handle(p_data->api_notify_vol.handle);
        break;
    case WICED_BT_HFP_HF_API_SEND_AT_CMD_EVT:
        p_scb = wiced_bt_hfp_hf_get_scb_by_handle(p_data->api_send_at_cmd.handle);
        break;
    default:
#if (defined(WICED_BT_HFP_HF_DEBUG) && WICED_BT_HFP_HF_DEBUG == TRUE)
        WICED_BTHFP_TRACE("%s: Unknown event %s\n", __FUNCTION__,
            wiced_bt_hfp_hf_evt_code((uint8_t)event));
#endif
        break;
    }
    return p_scb;
}

/* Non state machine action functions */
const wiced_bt_hfp_hf_nsm_act_t wiced_bt_hfp_hf_nsm_act[] =
{
    NULL,                       /**< WICED_BT_HFP_HF_INVALID_EVT */
    wiced_bt_hfp_hf_api_deinit, /**< WICED_BT_HFP_HF_API_DEINIT_EVT */
};


/*******************************************************************************
** Function         wiced_bt_hfp_hf_hdl_event
** Description      Handsfree main event handling function.
** Returns          wiced_bool_t
*******************************************************************************/
wiced_bool_t wiced_bt_hfp_hf_hdl_event(wiced_bt_hfp_hf_data_t *p_msg)
{
    uint8_t event = (uint8_t)p_msg->hf_evt;
    wiced_bt_hfp_hf_data_t *p_data = (wiced_bt_hfp_hf_data_t *)p_msg;
    wiced_bt_hfp_hf_scb_t *p_scb = NULL;

    WICED_BTHFP_TRACE("%s HFP profile event handler (%d)\n", __func__, event);

    if( (event == WICED_BT_HFP_HF_INVALID_EVT) || (event >= WICED_BT_HFP_HF_LAST_EVENT) )
    {
        return TRUE; /* To free p_msg */
    }

    if(event < WICED_BT_HFP_HF_GLOBAL_EVT_END)
    {
        /* Non state machine events */
        (*wiced_bt_hfp_hf_nsm_act[event])(p_data);
    }
    else
    {
        /* State machine events */
        if ((p_scb = wiced_bt_hfp_hf_get_scb(event, p_data)) != NULL)
        {
            wiced_bt_hfp_hf_ssm_execute(p_scb, p_data, (uint8_t)p_msg->hf_evt);
        }

    }
    return TRUE;
}
