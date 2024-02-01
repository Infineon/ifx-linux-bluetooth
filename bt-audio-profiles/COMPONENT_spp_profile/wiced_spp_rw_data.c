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
 * This file implements the SPP Library wrapper functions
 *
 */



#include "wiced_bt_dev.h"
#include "string.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_sdp.h"

#include "wiced_bt_dev.h"
#include "wiced_memory.h"
#include "wiced_spp_int.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_bt_utils.h"

/*****************************************************************************
 * Constants
 *****************************************************************************/

/******************************************************
 * Global Variables
 ******************************************************/
static uint8_t  *p_rx_fifo = NULL;

/*****************************************************************************
 * Data types
 *****************************************************************************/
typedef enum
{
    INDEX_TYPE_PORT_HANDLE,
    INDEX_TYPE_STATE_OPENING,
    INDEX_TYPE_STATE_IDLE,
} spp_scb_index_type_t;

/******************************************************
 * External Global Variables
 ******************************************************/
extern uint32_t    spp_bytes_rxed;

/******************************************************
 * External Functions
 ******************************************************/
extern spp_scb_t* spp_lib_get_scb_pointer( spp_scb_index_type_t index, uint16_t port_handle );
extern void       spp_rfcomm_acceptor_opened(spp_scb_t *p_scb);
extern void       spp_rfcomm_closed(spp_scb_t *p_scb);

/******************************************************
 * Function Declartions
 ******************************************************/
void spp_rfcomm_opened(spp_scb_t *p_scb);

/******************************************************
 * Function Definitions
 ******************************************************/

void spp_rfcomm_port_tx_cmpl_cback(uint16_t handle, void* p_data)
{
    SPP_TRACE("spp_rfcomm_port_tx_cmpl_cback()  p_data:%x ", p_data);
}

/*
 * Process RFCOMM events
 */
void spp_port_event_cback(wiced_bt_rfcomm_port_event_t event, uint16_t handle)
{
    spp_scb_t *p_scb = spp_lib_get_scb_pointer( INDEX_TYPE_PORT_HANDLE , handle );
    static char buff[256];
    uint16_t    len_read;

    if( p_scb == NULL )
    {
        SPP_TRACE( "ERROR spp_port_event_cback - cannot find scb based on handle %d\n", handle );
        return;
    }

    if (event & PORT_EV_RXCHAR)
    {
        wiced_bt_rfcomm_read_data (handle, buff, 256, &len_read);

        if (len_read != 0)
        {
            SPP_TRACE("[%s] rfcomm data callback\n", __func__);
            spp_bytes_rxed += len_read;
            p_scb->p_spp_reg->p_rx_data_callback(handle, (uint8_t *)buff, len_read);
            return;
        }
    }

    if (event & PORT_EV_FC)
    {
        p_scb->flow_control_on = !(event & PORT_EV_FCS) ? WICED_TRUE : WICED_FALSE;
    }

    if (event & PORT_EV_TXEMPTY)
    {
        p_scb->flow_control_on = WICED_FALSE;
    }

    p_scb->event_error = (event & PORT_EV_ERR) ? 1 : 0;
}

/*
 * RFCOMM management callback
 */
void spp_rfcomm_control_callback(uint32_t port_status, uint16_t port_handle)
{
    spp_scb_t *p_scb = spp_lib_get_scb_pointer( INDEX_TYPE_PORT_HANDLE , port_handle );

    if( p_scb == NULL )
    {
        SPP_TRACE( "ERROR spp_rfcomm_control_callback - cannot find scb based on handle %d\n", port_handle );
        return;
    }

    SPP_TRACE("spp_rfcomm_control_callback : Status = %d, port: 0x%04x  SCB state: %u  Srv: 0x%04x  Conn: 0x%04x\n",
                    port_status, port_handle, p_scb->state, p_scb->rfc_serv_handle, p_scb->rfc_conn_handle);

    /* ignore close event for port handles other than connected handle */
    if ((port_status != WICED_BT_RFCOMM_SUCCESS) && (port_handle != p_scb->rfc_conn_handle))
    {
        SPP_TRACE("spp_rfcomm_control_callback ignoring handle:%d", port_handle);
        return;
    }

    if( port_status == WICED_BT_RFCOMM_CLOSED )
    {
        if (p_rx_fifo != NULL)
        {
            wiced_bt_free_buffer (p_rx_fifo);
            p_rx_fifo = NULL;
        }
    }

    if ((port_status == WICED_BT_RFCOMM_SUCCESS) && (p_scb->state != SPP_SESSION_STATE_CLOSING))
    {
        p_rx_fifo = wiced_bt_get_buffer (p_scb->p_spp_reg->rfcomm_mtu * 2);
        if (p_rx_fifo != NULL)
        {
            SPP_TRACE("spp_rfcomm_control_callback p_rx_fifo SUCCESS");
            wiced_bt_rfcomm_set_rx_fifo (port_handle, (char *)p_rx_fifo, p_scb->p_spp_reg->rfcomm_mtu * 2);
        }
        else
        {
            SPP_TRACE("spp_rfcomm_control_callback p_rx_fifo NULL");
            wiced_bt_rfcomm_set_rx_fifo (port_handle, NULL, 0);
        }

        // Register data callback to receive data and set event mask to receive event notifications
        wiced_bt_rfcomm_set_event_mask(port_handle, PORT_MASK_ALL);

        if (p_scb->state == SPP_SESSION_STATE_IDLE)
            spp_rfcomm_acceptor_opened(p_scb);
        else
            spp_rfcomm_opened(p_scb);
    }
    else
    {
        spp_rfcomm_closed(p_scb);
    }
}


/*
 * Handle RFCOMM channel opened.
 */
void spp_rfcomm_opened(spp_scb_t *p_scb)
{
    int i;

    p_scb->state = SPP_SESSION_STATE_OPEN;

    /* store parameters */
    for (i = 0; i < BD_ADDR_LEN; i++)
        bd_addr_connected[i] = p_scb->server_addr[i];

    wiced_bt_rfcomm_set_event_callback(p_scb->rfc_conn_handle, spp_port_event_cback, spp_rfcomm_port_tx_cmpl_cback);

    SPP_TRACE("RFCOMM Connected  isInit: %u  Serv: 0x%04x   Conn: 0x%04x %B\n",
        p_scb->b_is_initiator, p_scb->rfc_serv_handle, p_scb->rfc_conn_handle, bd_addr_connected);

    p_scb->p_spp_reg->p_connection_up_callback(p_scb->rfc_conn_handle, p_scb->server_addr);
}
