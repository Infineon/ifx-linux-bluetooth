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
 * This file contains SDP functionality required HF Device sample application.
 * SDP database definition is contained in this file and is not changed from
 * the controlling MCU.
 *
 */
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_hfp_ag_int.h"
#include "wiced_memory.h"

extern void hfp_ag_process_open_callback( wiced_bt_hfp_ag_session_cb_t *p_scb, uint8_t status );
extern void hfp_ag_rfcomm_do_open( wiced_bt_hfp_ag_session_cb_t *p_scb );
extern wiced_timer_t sdp_timer;               /* wiced bt app sdp timer */

/******************************************************
*                 Global Variables
******************************************************/
static wiced_bt_hfp_ag_session_cb_t* sdp_p_scb;

/******************************************************
*               Function Declarations
******************************************************/

/* declare sdp callback functions */
static void hfp_ag_sdp_free_db( wiced_bt_hfp_ag_session_cb_t *p_scb );
static wiced_bool_t hfp_ag_sdp_find_attr(wiced_bt_hfp_ag_session_cb_t *p_scb);

/*
 * SDP callback function.
 */
static void hfp_ag_sdp_cback( uint16_t sdp_status )
{
    wiced_bt_hfp_ag_session_cb_t     *p_scb = sdp_p_scb;

    WICED_BT_TRACE( "hci_control_ag_sdp_cback status:0x%x, p_scb %x\n", sdp_status, p_scb );

    /* set event according to int/acp */
    if ( !p_scb->b_is_initiator )
    {
        if ( ( sdp_status == WICED_BT_SDP_SUCCESS ) || ( sdp_status == WICED_BT_SDP_DB_FULL ) )
        {
            hfp_ag_sdp_find_attr ( p_scb );
        }
    }
    else
    {
        if ( ( sdp_status == WICED_BT_SDP_SUCCESS ) || ( sdp_status == WICED_BT_SDP_DB_FULL ) )
        {
            if ( hfp_ag_sdp_find_attr( p_scb ) )
            {
                hfp_ag_rfcomm_do_open( p_scb );
            }
            else
            {
                /* reopen server and notify app of the failure */
                hfp_ag_rfcomm_start_server( p_scb );
                hfp_ag_process_open_callback( p_scb, WICED_BT_HFP_AG_STATUS_FAIL_SDP );
            }
        }
        else
        {
            /* reopen server and notify app of the failure */
            hfp_ag_rfcomm_start_server(p_scb);
            hfp_ag_process_open_callback(p_scb, WICED_BT_HFP_AG_STATUS_FAIL_SDP);
        }
    }
    hfp_ag_sdp_free_db( p_scb );
}

/*
 * Process SDP discovery results to find requested attributes for requested service.
 * Returns TRUE if results found, FALSE otherwise.
 */
wiced_bool_t hfp_ag_sdp_find_attr( wiced_bt_hfp_ag_session_cb_t *p_scb )
{
    wiced_bt_sdp_discovery_record_t     *p_rec = ( wiced_bt_sdp_discovery_record_t * ) NULL;
    wiced_bt_sdp_protocol_elem_t        pe;
    wiced_bt_sdp_discovery_attribute_t  *p_attr;
    wiced_bool_t                             result = WICED_TRUE;
    wiced_bt_uuid_t                     uuid_list;

    if(p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE)
    {
        WICED_BT_TRACE( "Looking for HFP service\n" );
        uuid_list.len       = LEN_UUID_16;
        uuid_list.uu.uuid16 = p_scb->hf_profile_uuid;
    }
    else
    {
        WICED_BT_TRACE( "Looking for HSP service\n" );
        uuid_list.len       = LEN_UUID_16;
        uuid_list.uu.uuid16 = p_scb->hf_profile_uuid;
    }

    p_rec = wiced_bt_sdp_find_service_uuid_in_db( p_scb->p_sdp_discovery_db, &uuid_list, p_rec );
    if ( p_rec == NULL )
    {
        WICED_BT_TRACE( "hci_control_ag_sdp_find_attr( ) - could not find AG service\n" );
        return ( WICED_FALSE );
    }

    /*** Look up the server channel number in the protocol list element ***/
    if ( wiced_bt_sdp_find_protocol_list_elem_in_rec( p_rec, UUID_PROTOCOL_RFCOMM, &pe ) )
    {
        WICED_BT_TRACE( "hci_control_ag_sdp_find_attr - num of proto elements -RFCOMM =0x%x\n",  pe.num_params );
        if ( pe.num_params > 0 )
        {
            p_scb->hf_scn = ( uint8_t )pe.params[0];
            WICED_BT_TRACE( "hci_control_ag_sdp_find_attr - found SCN in SDP record. SCN=0x%x\n", p_scb->hf_scn );
        }
        else
            result = WICED_FALSE;
    }
    else
    {
        result = WICED_FALSE;
    }

    /* get HFP supported features ( attribute ID 0x0311 ) */
    if ( p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE)
    {
        if ( ( p_attr = wiced_bt_sdp_find_attribute_in_rec( p_rec, 0x0311 ) ) != NULL )
        {
            /* Found attribute. Get value. but do not overwrite peer_feature if we already received +BRSF */
            if ( p_scb->hf_features == 0 )
                p_scb->hf_features = p_attr->attr_value.v.u16;
        }

        if ( wiced_bt_sdp_find_profile_version_in_rec( p_rec, UUID_SERVCLASS_HF_HANDSFREE, &p_scb->hf_version ) )
        {
            WICED_BT_TRACE( "HF device profile version: 0x%x\n", p_scb->hf_version );
        }
    }

    return result;
}


/*
 * Do service discovery.
 */
void hfp_ag_sdp_start_discovery( wiced_bt_hfp_ag_session_cb_t *p_scb )
{
    uint16_t        attr_list[4];
    uint8_t         num_attr;
    wiced_bt_uuid_t uuid_list;

    /* initiator - get proto list and features */
    if ( p_scb->b_is_initiator )
    {
        attr_list[0] = ATTR_ID_SERVICE_CLASS_ID_LIST;
        attr_list[1] = ATTR_ID_PROTOCOL_DESC_LIST;
        attr_list[2] = ATTR_ID_BT_PROFILE_DESC_LIST;
        attr_list[3] = ATTR_ID_SUPPORTED_FEATURES;
        num_attr = 4;
    }
    /* HFP acceptor; get features */
    else
    {
        attr_list[0] = ATTR_ID_SERVICE_CLASS_ID_LIST;
        attr_list[1] = ATTR_ID_BT_PROFILE_DESC_LIST;
        attr_list[2] = ATTR_ID_SUPPORTED_FEATURES;
        num_attr = 3;
    }

    /* allocate buffer for sdp database */
    p_scb->p_sdp_discovery_db = ( wiced_bt_sdp_discovery_db_t * ) wiced_bt_get_buffer( WICED_BUFF_MAX_SIZE );

    uuid_list.len       = LEN_UUID_16;
    uuid_list.uu.uuid16 = p_scb->hf_profile_uuid;
    /* set up service discovery database; attr happens to be attr_list len */
    wiced_bt_sdp_init_discovery_db( p_scb->p_sdp_discovery_db, WICED_BUFF_MAX_SIZE, 1, &uuid_list, num_attr, attr_list );

    WICED_BT_TRACE("  initiate service discovery app_handle = %x\n",p_scb->app_handle);

    /* save the p_scb in this service discovery, sdp_cback need this */
    sdp_p_scb = p_scb;

    /* initiate service discovery */
    if ( !wiced_bt_sdp_service_search_attribute_request( p_scb->hf_addr, p_scb->p_sdp_discovery_db, hfp_ag_sdp_cback ) )
    {
        WICED_BT_TRACE("hfp_ag_sdp_start_discovery: wiced_bt_sdp_service_search_attribute_request fail\n");
        /* Service discovery not initiated - free discover db, reopen server, tell app  */
        hfp_ag_sdp_free_db( p_scb );

        if ( p_scb->b_is_initiator )
        {
            hfp_ag_rfcomm_start_server( p_scb );
            hfp_ag_process_open_callback( p_scb, WICED_BT_HFP_AG_STATUS_FAIL_SDP );
        }
    }
}

/*
 * Free discovery database.
 */
void hfp_ag_sdp_free_db( wiced_bt_hfp_ag_session_cb_t *p_scb )
{
    if ( p_scb->p_sdp_discovery_db != NULL )
    {
        wiced_bt_free_buffer( p_scb->p_sdp_discovery_db );
        p_scb->p_sdp_discovery_db = NULL;
    }
}
