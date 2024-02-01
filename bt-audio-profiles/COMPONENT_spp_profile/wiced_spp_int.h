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
* Prototypes and definitions for WICED SPP implementation
*
*/

#ifndef __SPP_INT_H
#define __SPP_INT_H

#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_spp.h"
#include "wiced_bt_types.h"
#include "wiced_bt_rfcomm.h"


#define     SPP_BUFFER_POOL                    2
#define     SPP_MAX_CONNECTIONS                2

//#define SPP_TRACE_ENABLE
#if defined(WICED_BT_TRACE_ENABLE) && defined(SPP_TRACE_ENABLE)
#define     SPP_TRACE                          WICED_BT_TRACE
#else
#define     SPP_TRACE(...)
#endif

#define L2CAP_MIN_OFFSET                        13      /* from l2c_api.h */
#define RFCOMM_MIN_OFFSET                       5       /* from rfc_defs.h */
#define PORT_SUCCESS                            0       /* from port_api.h */
#define RFCOMM_INVALID_HANDLE                   0xFFFF

/* SPP control block */
typedef struct
{
#define     SPP_SESSION_STATE_IDLE       0
#define     SPP_SESSION_STATE_OPENING    1
#define     SPP_SESSION_STATE_OPEN       2
#define     SPP_SESSION_STATE_CLOSING    3

    uint8_t     state;                  /* state machine state */
    wiced_bool_t       in_use;                 /* indicates if control block is in use */

    uint8_t     b_is_initiator;         /* initiator of the connection ( true ) or acceptor ( false ) */

    uint16_t    rfc_serv_handle;        /* RFCOMM server handle */
    uint16_t    rfc_conn_handle;        /* RFCOMM handle of connected service */
    uint8_t     server_scn;             /* server's scn */
    wiced_bt_device_address_t     server_addr;            /* server's bd address */

    void        *p_sdp_discovery_db;                /* pointer to discovery database */
    uint32_t    flow_control_on;
    wiced_bt_spp_reg_t *p_spp_reg;             /* pointer to application call backs */
    wiced_bt_rfcomm_port_event_t event_error;  /* reflect PORT_EV_ERR */
} spp_scb_t;

extern wiced_bt_device_address_t              bd_addr_connected;

extern int PORT_Purge (uint16_t handle, uint8_t purge_flags);



#endif
