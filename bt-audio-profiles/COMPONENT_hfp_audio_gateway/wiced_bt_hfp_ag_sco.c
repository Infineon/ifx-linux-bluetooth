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
 * This file contains functions for managing the SCO connection used in
 * handsfree profile
 *
 */
#include "wiced_bt_hfp_ag_int.h"

static const wiced_bt_sco_params_t hf_control_esco_params =
{
#if ( HFP_AG_VERSION >= HFP_VERSION_1_7 )
        0x000C,             /* Latency: 12 ms ( HS/HF can use EV3, 2-EV3, 3-EV3 ) ( S4 ) */
#else
        0x000A,             /* Latency: 10 ms ( HS/HF can use EV3, 2-EV3, 3-EV3 ) ( S3 ) */
#endif

    BTA_AG_SCO_PKT_TYPES,

#if ( HFP_AG_VERSION >= HFP_VERSION_1_7 )
        BTM_ESCO_RETRANS_QUALITY, /* Retrans Effort ( Quality ) ( S4 ) */
#else
        BTM_ESCO_RETRANS_POWER, /* Retrans Effort ( At least one retrans, opt for power ) ( S3 ) */
#endif
    WICED_FALSE
};

extern wiced_bt_hfp_ag_hci_send_ag_event_cback_t hfp_ag_hci_send_ag_event;
/*
 * Create SCO connection as an originator or an acceptor
 */
void hfp_ag_sco_create(wiced_bt_hfp_ag_session_cb_t *p_scb, wiced_bool_t is_orig)
{
    wiced_bt_dev_status_t   status;
    wiced_bt_sco_params_t   params = hf_control_esco_params;

    /* remove listening SCO */
    if ( p_scb->sco_idx != BTM_INVALID_SCO_INDEX )
        wiced_bt_sco_remove( p_scb->sco_idx );

    /* Attempt to use eSCO if remote host supports HFP >= 1.5 */
    if ( is_orig )
    {
        if (p_scb->retry_with_sco_only)
        {
            p_scb->retry_with_sco_only = WICED_FALSE;
            params.packet_types        = BTM_SCO_LINK_ONLY_MASK;
        }
        else
        {
#if (BTM_WBS_INCLUDED == TRUE)
            if ( p_scb->peer_supports_msbc && !p_scb->msbc_selected )
            {
                /* Send +BCS to the peer and start a 3-second timer waiting for AT+BCS */
                wiced_bt_hfp_ag_send_BCS_to_hf (p_scb);
                wiced_start_timer (&p_scb->cn_timer, 3);
                return;
            }

            /* If WBS is negotiated, override the necessary defaults */
            if ( p_scb->msbc_selected )
            {
                params.use_wbs        = WICED_TRUE;
                params.max_latency    = 13;
                params.retrans_effort = BTM_ESCO_RETRANS_QUALITY;
                params.packet_types   = ( BTM_SCO_PKT_TYPES_MASK_EV3     |  /* EV3 + 2-EV3 */
                                        BTM_SCO_PKT_TYPES_MASK_NO_3_EV3 |
                                        BTM_SCO_PKT_TYPES_MASK_NO_2_EV5 |
                                        BTM_SCO_PKT_TYPES_MASK_NO_3_EV5 );
                WICED_BT_TRACE("Use WBS\n");
            }
            else
                WICED_BT_TRACE("Use NBS\n");
#endif
            /* Igf setup fails, fall back to regular SCO */
            p_scb->retry_with_sco_only = WICED_TRUE;
        }
        status = wiced_bt_sco_create_as_initiator( p_scb->hf_addr, &p_scb->sco_idx, &params );
    }
    else
    {
        p_scb->retry_with_sco_only = WICED_FALSE;
        status = wiced_bt_sco_create_as_acceptor_ex(p_scb->hf_addr, (uint16_t *) &p_scb->sco_idx);
    }

    WICED_BT_TRACE( "hfp_ag_sco_create  is_orig: %u   sco_idx: 0x%0x  status:0x%x retry_with_sco_only: %u\n",
                    is_orig, p_scb->sco_idx, status, p_scb->retry_with_sco_only );
}

/*
 * Close SCO connection
 */
void hfp_ag_sco_close( wiced_bt_hfp_ag_session_cb_t *p_scb )
{
    wiced_bt_dev_status_t status;

    WICED_BT_TRACE( "hfp_ag_sco_close : sco_idx = %d\n", p_scb->sco_idx );

    if ( p_scb->sco_idx != BTM_INVALID_SCO_INDEX )
    {
        status = wiced_bt_sco_remove( p_scb->sco_idx );

        /* BTM will return immediately for listening sco.       */
        /* If removing open SCO, sco_idx will be updated later. */
        if ( ( status == WICED_BT_SUCCESS ) || ( status == WICED_BT_UNKNOWN_ADDR ) )
        {
            p_scb->sco_idx      = BTM_INVALID_SCO_INDEX;
            p_scb->b_sco_opened = WICED_FALSE;
        }
    }
}


/*
 * Process SCO management callback
 */
void wiced_bt_hfp_ag_sco_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_hfp_ag_session_cb_t *p_scb;
    wiced_bt_hfp_ag_event_data_t ap_event;

    switch ( event )
    {
        case BTM_SCO_CONNECTED_EVT:             /**< SCO connected event. Event data: #wiced_bt_sco_connected_t */
            WICED_BT_TRACE( "Got BTM_SCO_CONNECTED_EVT: sco_idx: %u\n", p_event_data->sco_connected.sco_index );

            if ( ( p_scb = hfp_ag_find_scb_by_sco_index( p_event_data->sco_connected.sco_index ) ) != NULL )
            {
                p_scb->retry_with_sco_only        = WICED_FALSE;
                p_scb->b_sco_opened               = WICED_TRUE;
#if (BTM_WBS_INCLUDED == TRUE)
                /* call app callback */
                ap_event.audio_open.wbs_supported = p_scb->peer_supports_msbc;
                ap_event.audio_open.wbs_used      = p_scb->msbc_selected;
#else
                ap_event.audio_open.wbs_supported = WICED_FALSE;
                ap_event.audio_open.wbs_used = WICED_FALSE;
#endif
                if(hfp_ag_hci_send_ag_event)
                    hfp_ag_hci_send_ag_event( WICED_BT_HFP_AG_EVENT_AUDIO_OPEN, p_scb->app_handle, &ap_event );
            }
            break;

        case BTM_SCO_DISCONNECTED_EVT:          /**< SCO disconnected event. Event data: #wiced_bt_sco_disconnected_t */
            WICED_BT_TRACE( "Got BTM_SCO_DISCONNECTED_EVT: sco_idx: %u\n", p_event_data->sco_disconnected.sco_index );

            if ( ( p_scb = hfp_ag_find_scb_by_sco_index( p_event_data->sco_disconnected.sco_index ) ) != NULL )
            {
                p_scb->sco_idx = BTM_INVALID_SCO_INDEX;

                if ( p_scb->state == HFP_AG_STATE_CLOSING )
                {
                    hfp_ag_rfcomm_do_close( p_scb );
                    return;
                }

                if ( p_scb->retry_with_sco_only )
                    hfp_ag_sco_create( p_scb, WICED_TRUE );
                else
                {
                    hfp_ag_sco_create( p_scb, WICED_FALSE );
                    p_scb->b_sco_opened = WICED_FALSE;
                    if(hfp_ag_hci_send_ag_event)
                        hfp_ag_hci_send_ag_event( WICED_BT_HFP_AG_EVENT_AUDIO_CLOSE, p_scb->app_handle, NULL );
                }
            }
            break;

        case BTM_SCO_CONNECTION_REQUEST_EVT:    /**< SCO connection request event. Event data: #wiced_bt_sco_connection_request_t */
            WICED_BT_TRACE( "Got BTM_SCO_CONNECTION_REQUEST_EVT: sco_idx: %u  %B  %C 0x%02x\n", p_event_data->sco_connection_request.sco_index,
                    p_event_data->sco_connection_request.bd_addr, p_event_data->sco_connection_request.dev_class, p_event_data->sco_connection_request.link_type );

            if ( ( p_scb = hfp_ag_find_scb_by_sco_index( p_event_data->sco_connection_request.sco_index ) ) != NULL )
            {
                /* Automatically accept SCO */
                wiced_bt_sco_params_t   resp;
                uint8_t                 hci_status = HCI_SUCCESS;

                /* Start with narrow band defaults */
                resp = hf_control_esco_params;

#if ( BTM_WBS_INCLUDED == WICED_TRUE )
                /* If WBS is negotiated, override the necessary defaults */
                if ( p_scb->msbc_selected )
                {
                    resp.use_wbs        = WICED_TRUE;
                    resp.max_latency    = 13;
                    resp.retrans_effort = BTM_ESCO_RETRANS_QUALITY;
                    resp.packet_types   = ( BTM_SCO_PKT_TYPES_MASK_EV3     |  /* EV3 + 2-EV3 */
                                           BTM_SCO_PKT_TYPES_MASK_NO_3_EV3 |
                                           BTM_SCO_PKT_TYPES_MASK_NO_2_EV5 |
                                           BTM_SCO_PKT_TYPES_MASK_NO_3_EV5 );
                }
#endif
                wiced_bt_sco_accept_connection( p_scb->sco_idx, hci_status, &resp );
            }
            break;

        case BTM_SCO_CONNECTION_CHANGE_EVT:     /**< SCO connection change event. Event data: #wiced_bt_sco_connection_change_t */
            WICED_BT_TRACE( "Got BTM_SCO_CONNECTION_CHANGE_EVT: sco_idx: %u\n", p_event_data->sco_connection_change.sco_index );
            break;
    }

}

#if (BTM_WBS_INCLUDED == TRUE)
/*******************************************************************************
**
** Function         hfp_cn_timeout
**
** Description      called when the codec negotiation timer tripped
**
** Returns          void
**
*******************************************************************************/
void hfp_cn_timeout(WICED_TIMER_PARAM_TYPE scb)
{
    wiced_bt_hfp_ag_session_cb_t *p_scb = ( wiced_bt_hfp_ag_session_cb_t * )scb;

    WICED_BT_TRACE( "[%u]  !!Command Timeout!!\n", p_scb->app_handle );

    /* Codec negotiation failed - retry with plain old SCO */
    p_scb->peer_supports_msbc  = WICED_FALSE;
    p_scb->msbc_selected       = WICED_FALSE;

    hfp_ag_sco_create ( p_scb, WICED_TRUE );
}
#endif
