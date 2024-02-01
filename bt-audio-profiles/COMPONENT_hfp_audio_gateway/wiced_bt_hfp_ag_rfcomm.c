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
 * RFCOMM for HF Device sample application.
 *
 */

#include "wiced_bt_cfg.h"
#include "wiced_bt_hfp_ag_int.h"
#include "string.h"

static void hfp_ag_rfcomm_closed( wiced_bt_hfp_ag_session_cb_t *p_scb );
static void hfp_ag_rfcomm_opened( wiced_bt_hfp_ag_session_cb_t *p_scb );
static void hfp_ag_rfcomm_acceptor_opened( wiced_bt_hfp_ag_session_cb_t *p_scb );
void hfp_ag_rfcomm_do_open( wiced_bt_hfp_ag_session_cb_t *p_scb );
extern void hfp_ag_process_open_callback( wiced_bt_hfp_ag_session_cb_t *p_scb, uint8_t status );
extern wiced_bt_hfp_ag_hci_send_ag_event_cback_t hfp_ag_hci_send_ag_event;
#if BTSTACK_VER >= 0x03000001
/*
 * RFCOMM TX complete callback
 */
void hfp_ag_rfcomm_port_tx_cmpl_cback(uint16_t handle, void *p_buf)
{
    if(p_buf)
        wiced_bt_free_buffer(p_buf);
}
#endif /* BTSTACK_VER */

/*
 * Process RFCOMM data received from the peer
 */
#if BTSTACK_VER >= 0x03000001
void hfp_ag_rfcomm_data_callback( wiced_bt_rfcomm_port_event_t code, uint16_t handle )
{
    wiced_bt_hfp_ag_session_cb_t *p_scb = hfp_ag_find_scb_by_rfc_handle( handle, HFP_FLAG_RFCOMM_DATA );
    char *p;
    static char buff[256];
    uint16_t len;

    if ( p_scb == NULL )
    {
        WICED_BT_TRACE( "hfp_ag_rfcomm_data_callback: Unknown port handle: 0x%04x\n", handle );
        return;
    }

    if (code & PORT_EV_RXCHAR)
    {
        wiced_bt_rfcomm_read_data (handle, buff, 256, &len);

        p = buff;
    }
    else
    {
        WICED_BT_TRACE("[%u]handle_rcvd_data: unexpected code: 0x%08X\n",
                p_scb->app_handle, code);
        return;
    }

    //Ash DumpData( "RECV:", p_data, len );

    if ( p_scb->res_len == 0 )
    {
        memset( p_scb->res_buf, 0, HFP_AG_AT_MAX_LEN );
    }

    if ( ( p_scb->res_len + len ) > HFP_AG_AT_MAX_LEN )
    {
        WICED_BT_TRACE( "[%u]handle_rcvd_data: too much data res_len %u  len: %u\n", p_scb->app_handle, p_scb->res_len, len );
    }
    else if ( len > 0 )
    {
        memcpy( &p_scb->res_buf[p_scb->res_len], p, len );
        p_scb->res_len += len;

        hfp_ag_parse_AT_command (p_scb);
    }

    return;
}
#else /* !BTSTACK_VER */
int hfp_ag_rfcomm_data_callback( uint16_t port_handle, void *p_data, uint16_t len )
{
    char                *p = ( char * )p_data;
    wiced_bt_hfp_ag_session_cb_t *p_scb = hfp_ag_find_scb_by_rfc_handle( port_handle, HFP_FLAG_RFCOMM_DATA );

    if ( p_scb == NULL )
    {
        WICED_BT_TRACE( "hfp_ag_rfcomm_data_callback: Unknown port handle: 0x%04x\n", port_handle );
        return 0;
    }

    //Ash DumpData( "RECV:", p_data, len );

    if ( p_scb->res_len == 0 )
    {
        memset( p_scb->res_buf, 0, HFP_AG_AT_MAX_LEN );
    }

    if ( ( p_scb->res_len + len ) > HFP_AG_AT_MAX_LEN )
    {
        WICED_BT_TRACE( "[%u]handle_rcvd_data: too much data res_len %u  len: %u\n", p_scb->app_handle, p_scb->res_len, len );
    }
    else if ( len > 0 )
    {
        memcpy( &p_scb->res_buf[p_scb->res_len], p, len );
        p_scb->res_len += len;

        hfp_ag_parse_AT_command (p_scb);
    }

    return ( ( int )len );
}
#endif /* BTSTACK_VER */


/*
 * RFCOMM management callback
 */
static void hfp_ag_rfcomm_control_callback( uint32_t port_status, uint16_t port_handle )
{
    uint8_t i;
    wiced_bt_hfp_ag_session_cb_t *p_scb = hfp_ag_find_scb_by_rfc_handle( port_handle, HFP_FLAG_RFCOMM_CONTROL );

    if ( p_scb == NULL )
    {
        WICED_BT_TRACE( "hfp_ag_rfcomm_data_callback: Unknown port handle: 0x%04x\n", port_handle );
        return;
    }

    WICED_BT_TRACE( "[%u]hfp_ag_rfcomm_control_callback : Status = %d, port: 0x%04x  SCB state: %u  Srv: 0x%04x  Conn: 0x%04x\n",
                    p_scb->app_handle, port_status, port_handle, p_scb->state, p_scb->rfc_serv_handle, p_scb->rfc_conn_handle );

    /* ignore close event for port handles other than connected handle */
    if ( ( port_status != WICED_BT_RFCOMM_SUCCESS ) && ( port_handle != p_scb->rfc_conn_handle ) )
    {
        WICED_BT_TRACE( "hfp_ag_rfcomm_control_callback ignoring handle:%d\n", port_handle );
        return;
    }

    if ( ( port_status == WICED_BT_RFCOMM_SUCCESS ) && ( p_scb->state != HFP_AG_STATE_CLOSING) )
    {
#if BTSTACK_VER >= 0x03000001
        wiced_bt_rfcomm_set_event_callback(port_handle,
                hfp_ag_rfcomm_data_callback,
                hfp_ag_rfcomm_port_tx_cmpl_cback);
#else
        wiced_bt_rfcomm_set_data_callback( port_handle, hfp_ag_rfcomm_data_callback );
#endif

        i = p_scb->state;
        p_scb->state = HFP_AG_STATE_OPEN;

#if BTSTACK_VER >= 0x03000001
        wiced_bt_rfcomm_set_rx_fifo (port_handle, (char *)p_scb->rfcomm_fifo,
                sizeof (p_scb->rfcomm_fifo));
#endif

        if ( i == HFP_AG_STATE_IDLE )
            hfp_ag_rfcomm_acceptor_opened( p_scb );
        else
            hfp_ag_rfcomm_opened( p_scb );

        hfp_ag_sco_create( p_scb, WICED_FALSE );                  /* Listen for a SCO */
    }
    else
    {
        hfp_ag_rfcomm_closed( p_scb );
    }
}

/*
 * Setup RFCOMM server, in listen mode.
 */
void hfp_ag_rfcomm_start_server( wiced_bt_hfp_ag_session_cb_t *p_scb )
{
    wiced_bt_rfcomm_result_t    rfcomm_result;
    uint8_t                     scn;

    p_scb->state = HFP_AG_STATE_IDLE;

    if(p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE)
        scn = HFP_RFCOMM_SCN;
    else
        scn = HSP_RFCOMM_SCN;

    if ( !p_scb->rfc_serv_handle )
    {
        rfcomm_result = wiced_bt_rfcomm_create_connection( p_scb->hf_profile_uuid, //Use connected UUID
                                                           scn, WICED_TRUE, HFP_DEVICE_MTU, bd_addr_any,
                                                           &p_scb->rfc_serv_handle,
                                                           ( wiced_bt_port_mgmt_cback_t * )hfp_ag_rfcomm_control_callback );

        WICED_BT_TRACE( "[%u]hfp_ag_rfcomm_start_server: rfcomm_create Res: 0x%x  Port: 0x%04x UUID: 0x%04x\n",
                        p_scb->app_handle, rfcomm_result, p_scb->rfc_serv_handle, p_scb->hf_profile_uuid );
    }
    else
    {
        WICED_BT_TRACE( "[%u]hfp_ag_rfcomm_start_server: rfcomm_create Port Already set to: 0x%04x\n",
                        p_scb->app_handle, p_scb->rfc_serv_handle );
    }
}

/*
 * Open an RFCOMM connection to the peer device.
 */
void hfp_ag_rfcomm_do_open( wiced_bt_hfp_ag_session_cb_t *p_scb )
{
    wiced_bt_rfcomm_result_t rfcomm_result;

    WICED_BT_TRACE( "[%u]hfp_ag_rfcomm_do_open: rfc_serv_handle: 0x%04x\n",
                    p_scb->app_handle, p_scb->rfc_serv_handle );

    /* Close the server, if listening on this SCB */
    if ( p_scb->rfc_serv_handle )
    {
        wiced_bt_rfcomm_remove_connection(p_scb->rfc_serv_handle, TRUE );
        p_scb->rfc_serv_handle = 0;
    }

    rfcomm_result = wiced_bt_rfcomm_create_connection( p_scb->hf_profile_uuid, //Use connected UUID
                                                      p_scb->hf_scn, WICED_FALSE,
                                                      HFP_DEVICE_MTU,
                                                      p_scb->hf_addr,
                                                      &p_scb->rfc_conn_handle,
                                                      ( wiced_bt_port_mgmt_cback_t * )hfp_ag_rfcomm_control_callback );

    WICED_BT_TRACE( "[%u]hfp_ag_rfcomm_do_open - rfcomm_create Res: 0x%x   Port: 0x%04x UUID: 0x%04x\n",
                     p_scb->app_handle, rfcomm_result, p_scb->rfc_conn_handle, p_scb->hf_profile_uuid );

    if ( rfcomm_result != WICED_BT_RFCOMM_SUCCESS )
    {
        /* Pass back that the connection attempt failed */
        wiced_bt_hfp_ag_open_t    open;

        /* call app callback with open event */
        open.status = rfcomm_result;

        utl_bdcpy( open.bd_addr, p_scb->hf_addr );
        if(hfp_ag_hci_send_ag_event)
            hfp_ag_hci_send_ag_event( WICED_BT_HFP_AG_EVENT_OPEN, p_scb->app_handle, ( wiced_bt_hfp_ag_event_data_t *)&open );

        hfp_ag_rfcomm_start_server( p_scb );
    }

}

/*
 * Close RFCOMM connection.
 */
void hfp_ag_rfcomm_do_close( wiced_bt_hfp_ag_session_cb_t *p_scb )
{
    wiced_bt_rfcomm_result_t rfcomm_result;

    if ( p_scb->rfc_conn_handle )
    {
        p_scb->state = HFP_AG_STATE_CLOSING;

        // Disconnect RFCOMM keeping server listening
        rfcomm_result = wiced_bt_rfcomm_remove_connection( p_scb->rfc_conn_handle, FALSE );

        WICED_BT_TRACE( "[%u]hfp_ag_rfcomm_do_close (0x%04x) result 0x%x\n",
                        p_scb->app_handle, p_scb->rfc_conn_handle, rfcomm_result );
    }
    else
    {
        WICED_BT_TRACE( "[%u]hfp_ag_rfcomm_do_close - conn_handle zero\n", p_scb->app_handle );
        hfp_ag_rfcomm_start_server (p_scb);
    }
}

/*
 * RFCOMM connection closed.
 */
void hfp_ag_rfcomm_closed( wiced_bt_hfp_ag_session_cb_t *p_scb )
{
    wiced_bt_rfcomm_result_t rfcomm_result;

    /* call appropriate close cback */
    if ( p_scb->state == HFP_AG_STATE_OPENING )
        hfp_ag_process_open_callback( p_scb, WICED_BT_HFP_AG_STATUS_FAIL_RFCOMM );
    else
    {
        if(hfp_ag_hci_send_ag_event)
            hfp_ag_hci_send_ag_event( WICED_BT_HFP_AG_EVENT_CLOSE, p_scb->app_handle, NULL );
    }
    if ( p_scb->rfc_conn_handle )
    {
        // Remove RFCOMM server
        rfcomm_result = wiced_bt_rfcomm_remove_connection( p_scb->rfc_conn_handle, TRUE );

        WICED_BT_TRACE( "[%u]hfp_ag_rfcomm_closed (0x%04x) result 0x%x\n",
                        p_scb->app_handle, p_scb->rfc_conn_handle, rfcomm_result );
    }

    /* Clear peer bd_addr */
    utl_bdcpy( p_scb->hf_addr, bd_addr_null );

    hfp_ag_sco_close( p_scb );

    p_scb->sco_idx         = BTM_INVALID_SCO_INDEX;
    p_scb->rfc_conn_handle = 0;
    p_scb->state           = HFP_AG_STATE_IDLE;

    p_scb->rfc_serv_handle = 0; //Let hfp_ag_rfcomm_start_server to restart RFCOMM server
    p_scb->b_call_is_up = WICED_FALSE; //For HSP

    /* Reopen server if needed */
    hfp_ag_rfcomm_start_server (p_scb);
}

/*
 * Handle RFCOMM channel opened.
 */
static void hfp_ag_rfcomm_opened ( wiced_bt_hfp_ag_session_cb_t *p_scb )
{
    p_scb->state    = HFP_AG_STATE_OPEN;

    /* reinitialize stuff */
    //p_scb->hf_features          = 0; //No need to cleanup
    p_scb->b_slc_is_up          = WICED_FALSE;
    p_scb->b_call_is_up         = WICED_FALSE; //For HSP
    p_scb->res_len              = 0;
    p_scb->sco_idx              = BTM_INVALID_SCO_INDEX;
    p_scb->b_sco_opened         = WICED_FALSE;
    p_scb->indicator_bit_map    = 0x7F;
#if (BTM_WBS_INCLUDED == TRUE)
    p_scb->peer_supports_msbc   = WICED_FALSE;
    p_scb->msbc_selected        = WICED_FALSE;
#endif

    hfp_ag_process_open_callback ( p_scb, WICED_BT_HFP_AG_STATUS_SUCCESS );

    WICED_BT_TRACE( "[%u]RFCOMM Connected  isInit: %u  Serv: 0x%04x   Conn: 0x%04x\n",
                    p_scb->app_handle, p_scb->b_is_initiator, p_scb->rfc_serv_handle, p_scb->rfc_conn_handle );
}

/*
 * Handle RFCOMM channel opened when accepting connection.
 */
void hfp_ag_rfcomm_acceptor_opened( wiced_bt_hfp_ag_session_cb_t *p_scb )
{
    uint16_t lcid;
    int      status;

    /* set role and connection handle */
    p_scb->b_is_initiator  = WICED_FALSE;
    p_scb->rfc_conn_handle = p_scb->rfc_serv_handle;

    /* get bd addr of peer */
    if ( WICED_BT_RFCOMM_SUCCESS != ( status = wiced_bt_rfcomm_check_connection( p_scb->rfc_conn_handle, p_scb->hf_addr, &lcid ) ) )
    {
        WICED_BT_TRACE( "[%u]hfp_ag_rfcomm_acceptor_opened error PORT_CheckConnection returned status %d\n", p_scb->app_handle, status );
    }

    /* do service discovery to get features */
    hfp_ag_sdp_start_discovery( p_scb );

    /* continue with common open processing */
    hfp_ag_rfcomm_opened( p_scb );
}
