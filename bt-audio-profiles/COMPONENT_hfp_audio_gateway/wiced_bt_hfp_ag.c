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
 * This file implement handsfree audio gateway application controlled over UART
 *
 */
#include "wiced_bt_hfp_ag_int.h"
#include "string.h"

extern wiced_result_t hci_control_send_script_event(int type, uint8_t *p_data, uint16_t data_size);
void hfp_ag_process_open_callback( wiced_bt_hfp_ag_session_cb_t *p_scb, uint8_t status );

wiced_timer_t               sdp_timer;               /* wiced bt app sdp timer */
uint32_t                    ag_features;
wiced_bt_hfp_ag_session_cb_t         *ag_p_scb = NULL;
uint8_t                     ag_num_scb;
wiced_bt_hfp_ag_hci_send_ag_event_cback_t hfp_ag_hci_send_ag_event = NULL;



/******************************************************
 *               Function Definitions
 ******************************************************/

/* The function invoked on timeout of app seconds timer. */
void sdp_timer_cb(WICED_TIMER_PARAM_TYPE arg)
{
    wiced_bt_hfp_ag_session_cb_t *p_scb = ag_p_scb;
    uint8_t i;

    wiced_stop_timer( &sdp_timer );

    for ( i = 0; i < ag_num_scb; i++, p_scb++ )
    {
        if (p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE)
            break;
    }

    if(p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE)
    {
        p_scb->state = HFP_AG_STATE_OPENING;
        /* close RFCOMM server, if listening on this SCB */
        if ( p_scb->rfc_serv_handle )
        {
            wiced_bt_rfcomm_remove_connection( p_scb->rfc_serv_handle, WICED_TRUE );
            p_scb->rfc_serv_handle = 0;
        }

        /* set role */
        p_scb->b_is_initiator = WICED_TRUE;
        p_scb->hf_profile_uuid = UUID_SERVCLASS_HEADSET; //Try to search Headset service again

        /* do service search */
        hfp_ag_sdp_start_discovery( p_scb );
    }
}

/*
 * Start up the audio gateway service.
 */
void wiced_bt_hfp_ag_startup( wiced_bt_hfp_ag_session_cb_t *p_scb, uint8_t num_scb, uint32_t features, wiced_bt_hfp_ag_hci_send_ag_event_cback_t p_app_cback )
{
    uint8_t i;

    WICED_BT_TRACE( "%s\n", __FUNCTION__ );
    ag_p_scb    = p_scb;
    ag_num_scb  = num_scb;
    ag_features  = features;
    hfp_ag_hci_send_ag_event = p_app_cback;

    for ( i = 0; i < ag_num_scb; i++, p_scb++ )
    {
#if (BTM_WBS_INCLUDED == TRUE)
        wiced_init_timer(&p_scb->cn_timer, hfp_cn_timeout, (WICED_TIMER_PARAM_TYPE)p_scb, WICED_SECONDS_TIMER);
#endif

        p_scb->sco_idx = BTM_INVALID_SCO_INDEX;

        /* start RFCOMM server */
        hfp_ag_rfcomm_start_server( p_scb );
    }

    wiced_init_timer( &sdp_timer, &sdp_timer_cb, 0, WICED_SECONDS_TIMER );

//    Default mode is set to I2S Master so no need to call this API
//    To change the mode please call below API and to update PCM configuration use wiced_hal_set_pcm_config API
//    result = wiced_bt_sco_setup_voice_path(&ag_sco_path);
//    WICED_BT_TRACE("[%s] SCO Setting up voice path = %d\n",__func__, result);
}

/*
 * Opens a connection to an HF device.  When connection is opened callback
 * function is called with a HCI_CONTROL_HF_EVENT_CONNECTED. Only the service
 * level data connection is opened. The audio connection is not.
 */
void wiced_bt_hfp_ag_connect( wiced_bt_device_address_t bd_addr )
{
    wiced_bt_hfp_ag_session_cb_t *p_scb = ag_p_scb;
    uint8_t i;
    WICED_BT_TRACE("[%s]: BD_ADDR = %B", __FUNCTION__, bd_addr);
    for ( i = 0; i < ag_num_scb; i++, p_scb++ )
    {
        if ( p_scb->state == HFP_AG_STATE_IDLE )
            break;
    }

    if ( i == ag_num_scb )
    {
        return;
    }

    p_scb->state = HFP_AG_STATE_OPENING;

    /* store parameters */
    WICED_MEMCPY(p_scb->hf_addr, bd_addr, sizeof(wiced_bt_device_address_t));

    /* close RFCOMM server, if listening on this SCB */
    if ( p_scb->rfc_serv_handle )
    {
        wiced_bt_rfcomm_remove_connection( p_scb->rfc_serv_handle, WICED_TRUE );
        p_scb->rfc_serv_handle = 0;
    }

    /* set role */
    p_scb->b_is_initiator = WICED_TRUE;
    p_scb->hf_profile_uuid = UUID_SERVCLASS_HF_HANDSFREE;

    /* do service search */
    hfp_ag_sdp_start_discovery( p_scb );
}

/*
 * Close the current connection to an audio gateway.  Any current audio
 * connection will also be closed
 */
void wiced_bt_hfp_ag_disconnect( uint16_t handle )
{
    wiced_bt_hfp_ag_session_cb_t *p_scb = hfp_ag_find_scb_by_app_handle( handle );

    if (p_scb == NULL)
        return;

    WICED_BT_TRACE( "[%u]wiced_bt_hfp_ag_disconnect   State: %u\n", p_scb->app_handle, p_scb->state );

    if ( p_scb->state == HFP_AG_STATE_OPENING )
    {
        p_scb->state = HFP_AG_STATE_CLOSING;
        hfp_ag_rfcomm_do_close( p_scb );
    }
    else if ( p_scb->state == HFP_AG_STATE_OPEN )
    {
        p_scb->state = HFP_AG_STATE_CLOSING;

        /* if SCO is open close SCO and wait on RFCOMM close */
        if ( !p_scb->b_sco_opened )
            hfp_ag_rfcomm_do_close( p_scb );

        /* always do SCO shutdown to handle all SCO corner cases */
        hfp_ag_sco_close( p_scb );
    }
}

/*
 * Opens an audio connection to the currently connected audio gateway
 */
void wiced_bt_hfp_ag_audio_open( uint16_t handle )
{
    wiced_bt_hfp_ag_session_cb_t *p_scb = hfp_ag_find_scb_by_app_handle( handle );
    wiced_bt_hfp_ag_event_data_t ap_event;

    if (p_scb == NULL)
        return;

    WICED_BT_TRACE( "wiced_bt_hfp_ag_audio_open - state: %u  SCO inx: 0x%02x\n", p_scb->state, p_scb->sco_idx );

    /* If already open, just return success */
    if ( p_scb->b_sco_opened )
    {

#if (BTM_WBS_INCLUDED == TRUE)
        ap_event.audio_open.wbs_supported = p_scb->peer_supports_msbc;
        ap_event.audio_open.wbs_used = p_scb->msbc_selected;
#else
        ap_event.audio_open.wbs_supported = WICED_FALSE;
        ap_event.audio_open.wbs_used = WICED_FALSE;
#endif
        if(hfp_ag_hci_send_ag_event)
            hfp_ag_hci_send_ag_event( WICED_BT_HFP_AG_EVENT_AUDIO_OPEN, handle, (wiced_bt_hfp_ag_event_data_t*)&ap_event );
        return;
    }

    if ( p_scb->state == HFP_AG_STATE_OPEN )
    {
        /* Assume we are bringing up a SCO for voice recognition, so send BVRA */
        if ( (p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE) &&
             (ag_features & HFP_AG_FEAT_VREC) && (p_scb->hf_features & HFP_HF_FEAT_VREC) )
        {
            wiced_bt_hfp_ag_send_BVRA_to_hf( p_scb, TRUE );
            hfp_ag_sco_create( p_scb, TRUE );
        }
    }
}

/*
 * Close the currently active audio connection to a audio gateway. The data
 * connection remains open
 */
void wiced_bt_hfp_ag_audio_close( uint16_t handle )
{
    wiced_bt_hfp_ag_session_cb_t *p_scb = hfp_ag_find_scb_by_app_handle( handle );

    if (p_scb == NULL)
        return;

    if ( p_scb->b_sco_opened )
    {
        /* Assume we had brought up the SCO for voice recognition, so send BVRA */
        if(p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE)
            wiced_bt_hfp_ag_send_BVRA_to_hf( p_scb, FALSE );

        hfp_ag_sco_close( p_scb );
    }
}

/*
 * Sends Given Command string
 */
void wiced_bt_hfp_ag_send_cmd_str( uint16_t handle , uint8_t *data, uint8_t len)
{
	wiced_bt_hfp_ag_session_cb_t *p_scb = hfp_ag_find_scb_by_app_handle( handle );

	if (p_scb == NULL)
        return;

	data[len] = '\0';
    wiced_bt_hfp_ag_send_cmd_str_to_hf(p_scb, (char*)data);
}
/*
 * Find SCB associated with AG BD address.
 */
wiced_bt_hfp_ag_session_cb_t *hfp_ag_find_scb_by_sco_index( uint16_t sco_idx )
{
    wiced_bt_hfp_ag_session_cb_t *p_scb = ag_p_scb;
    uint16_t i;

    for ( i = 0; i < ag_num_scb; i++, p_scb++ )
    {
        if ( p_scb->sco_idx == sco_idx )
            return ( p_scb );
    }

    /* no scb found */
    WICED_BT_TRACE( "No scb for SCO inx: %u\n", sco_idx );
    return NULL;
}

/*
 * Find SCB associated with rfcomm handle.
 */
wiced_bt_hfp_ag_session_cb_t *hfp_ag_find_scb_by_rfc_handle( uint16_t rfc_handle, uint8_t flag )
{
    wiced_bt_hfp_ag_session_cb_t *p_scb = ag_p_scb;
    uint16_t i;

    for ( i = 0; i < ag_num_scb; i++, p_scb++ )
    {
        if ( ( flag == HFP_FLAG_RFCOMM_DATA ) && ( rfc_handle == p_scb->rfc_conn_handle ) )
            return ( p_scb );

        if ( ( flag == HFP_FLAG_RFCOMM_CONTROL ) &&
                ( ( rfc_handle == p_scb->rfc_serv_handle ) || ( rfc_handle == p_scb->rfc_conn_handle ) ) )
            return ( p_scb );
    }

    /* no scb found */
    WICED_BT_TRACE( "No scb for rfcomm handle: %u\n", rfc_handle );
    return NULL;
}

/*
 * Find SCB associated with app handle.
 */
wiced_bt_hfp_ag_session_cb_t *hfp_ag_find_scb_by_app_handle( uint16_t app_handle )
{
    wiced_bt_hfp_ag_session_cb_t *p_scb = ag_p_scb;
    uint16_t i;

    for ( i = 0; i < ag_num_scb; i++, p_scb++ )
    {
        if ( ( app_handle == p_scb->app_handle ) )
            return ( p_scb );
    }

    /* no scb found */
    WICED_BT_TRACE( "No scb for app handle: %u\n", app_handle );
    return NULL;
}


/*
 * Send open callback event to application.
 */
void hfp_ag_process_open_callback( wiced_bt_hfp_ag_session_cb_t *p_scb, uint8_t status )
{
    wiced_bt_hfp_ag_open_t open;

    /* call app callback with open event */
    open.status = status;

    WICED_BT_TRACE("hfp_ag_process_open_callback status=%d\n", status);

    if ( status == WICED_BT_HFP_AG_STATUS_SUCCESS )
    {
        utl_bdcpy( open.bd_addr, p_scb->hf_addr );
        if(hfp_ag_hci_send_ag_event)
            hfp_ag_hci_send_ag_event( WICED_BT_HFP_AG_EVENT_OPEN, p_scb->app_handle, (wiced_bt_hfp_ag_event_data_t *) &open );
    }
    else
    {
        if(p_scb->b_is_initiator && p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE)
        {
            WICED_BT_TRACE("hfp_ag_process_open_callback: Try HSP\n");
            wiced_start_timer( &sdp_timer, 1 );
        }
        else
        {
            utl_bdcpy( p_scb->hf_addr, (BD_ADDR_PTR) bd_addr_null );
            hfp_ag_rfcomm_start_server( p_scb ); //Restart RFCOMM Server
            if(hfp_ag_hci_send_ag_event)
                hfp_ag_hci_send_ag_event( WICED_BT_HFP_AG_EVENT_OPEN, p_scb->app_handle, ( wiced_bt_hfp_ag_event_data_t * ) &open );
        }
    }
}

/*
 * Service level connection opened
 */
void hfp_ag_service_level_up( wiced_bt_hfp_ag_session_cb_t *p_scb )
{
    wiced_bt_wiced_bt_hfp_ag_connect_t evt;

    /* Only tell it once */
    if ( !p_scb->b_slc_is_up )
    {
        p_scb->b_slc_is_up = WICED_TRUE;

        evt.peer_features = p_scb->hf_features;

        /* call callback */
        if(hfp_ag_hci_send_ag_event)
            hfp_ag_hci_send_ag_event( WICED_BT_HFP_AG_EVENT_CONNECTED, p_scb->app_handle, ( wiced_bt_hfp_ag_event_data_t * ) &evt );
    }
}
