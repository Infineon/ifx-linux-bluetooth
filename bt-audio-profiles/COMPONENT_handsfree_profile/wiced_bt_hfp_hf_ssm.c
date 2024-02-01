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
 * This is the service state machine for Handsfree.
 */

#include "wiced_bt_hfp_hf_int.h"

/*****************************************************************************
** Constants and types
*****************************************************************************/

/* State table for init state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_hfp_hf_state_evt_t enum*/
static const wiced_bt_hfp_hf_event_map_t wiced_bt_hfp_hf_sst_init[] =
{
    {WICED_BT_HFP_HF_API_CONNECT_EVT       , WICED_BT_HFP_HF_OPENING_SST , wiced_bt_hfp_hf_do_sdp},
    {WICED_BT_HFP_HF_RFC_CONNECT_EVT       , WICED_BT_HFP_HF_OPEN_SST    , wiced_bt_hfp_hf_rfc_connected},
};

/* State table for signal opening state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_hfp_hf_state_evt_t enum*/
static const wiced_bt_hfp_hf_event_map_t wiced_bt_hfp_hf_sst_opening[] =
{
    {WICED_BT_HFP_HF_API_DISCONNECT_EVT    , WICED_BT_HFP_HF_CLOSING_SST , wiced_bt_hfp_hf_rfc_disconnect_req},
    {WICED_BT_HFP_HF_RFC_CONNECT_EVT       , WICED_BT_HFP_HF_OPEN_SST    , wiced_bt_hfp_hf_rfc_connected},
    {WICED_BT_HFP_HF_RFC_CONNECT_FAIL_EVT  , WICED_BT_HFP_HF_INIT_SST    , wiced_bt_hfp_hf_rfc_connection_fail},
    {WICED_BT_HFP_HF_RFC_DISCONNECT_EVT    , WICED_BT_HFP_HF_INIT_SST    , wiced_bt_hfp_hf_rfc_connection_fail},
    {WICED_BT_HFP_HF_SDP_DISC_OK_EVT       , WICED_BT_HFP_HF_OPENING_SST , wiced_bt_hfp_hf_rfc_connect_req},
    {WICED_BT_HFP_HF_SDP_DISC_FAIL_EVT     , WICED_BT_HFP_HF_INIT_SST    , wiced_bt_hfp_hf_sdp_failed},
};

/* State table for signaling channel opened state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_hfp_hf_state_evt_t enum*/
static const wiced_bt_hfp_hf_event_map_t wiced_bt_hfp_hf_sst_open[] =
{
    {WICED_BT_HFP_HF_API_DISCONNECT_EVT    , WICED_BT_HFP_HF_CLOSING_SST , wiced_bt_hfp_hf_rfc_disconnect_req},
    {WICED_BT_HFP_HF_API_CALL_ACTION_EVT   , WICED_BT_HFP_HF_OPEN_SST    , wiced_bt_hfp_hf_do_call_action},
    {WICED_BT_HFP_HF_API_NOTIFY_VOLUME_EVT , WICED_BT_HFP_HF_OPEN_SST    , wiced_bt_hfp_hf_do_notify_volume},
    {WICED_BT_HFP_HF_API_SEND_AT_CMD_EVT   , WICED_BT_HFP_HF_OPEN_SST    , wiced_bt_hfp_hf_do_send_at_cmd},
    {WICED_BT_HFP_HF_RFC_DISCONNECT_EVT    , WICED_BT_HFP_HF_INIT_SST    , wiced_bt_hfp_hf_rfc_disconnected},
    {WICED_BT_HFP_HF_RFC_DATA_EVT          , WICED_BT_HFP_HF_OPEN_SST    , wiced_bt_hfp_hf_rfc_data},
    {WICED_BT_HFP_HF_CMD_TIMEOUT_EVT       , WICED_BT_HFP_HF_OPEN_SST    , wiced_bt_hfp_hf_cmd_timeout},
};

/* State table for closing state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_hfp_hf_state_evt_t enum*/
static const wiced_bt_hfp_hf_event_map_t wiced_bt_hfp_hf_sst_closing[] =
{
    {WICED_BT_HFP_HF_RFC_DISCONNECT_EVT    , WICED_BT_HFP_HF_INIT_SST    , wiced_bt_hfp_hf_rfc_disconnected},
};

/* State table */
static const wiced_bt_hfp_hf_sst_tbl_entry_t wiced_bt_hfp_hf_sst_tbl[] =
{
    {wiced_bt_hfp_hf_sst_init,   sizeof(wiced_bt_hfp_hf_sst_init)/sizeof(wiced_bt_hfp_hf_event_map_t)},
    {wiced_bt_hfp_hf_sst_opening,sizeof(wiced_bt_hfp_hf_sst_opening)/sizeof(wiced_bt_hfp_hf_event_map_t)},
    {wiced_bt_hfp_hf_sst_open,   sizeof(wiced_bt_hfp_hf_sst_open)/sizeof(wiced_bt_hfp_hf_event_map_t)},
    {wiced_bt_hfp_hf_sst_closing,sizeof(wiced_bt_hfp_hf_sst_closing)/sizeof(wiced_bt_hfp_hf_event_map_t)},
};

/*******************************************************************************
**
** Function         wiced_bt_hfp_hf_search_event
**
** Description      Search for the event in the state table and return the
**                       next state and handler if the event is handled by the state table.
**
** Returns      Returns WICED_TRUE if the event is handled by the state table. Else return WICED_FALSE.
**
*******************************************************************************/
static wiced_bool_t wiced_bt_hfp_hf_search_event(
    wiced_bt_hfp_hf_sst_tbl_entry_t state_table_entry, uint8_t event,
    uint8_t *p_next_state, wiced_bt_hfp_hf_sm_act_t *pfhandler)
{
    int l = 0;
    int r = state_table_entry.size_of_table - 1;
    int m = 0;
    wiced_bool_t found = FALSE;
    const wiced_bt_hfp_hf_event_map_t *state_table = state_table_entry.state_table;

    while (l <= r)
    {
        m = l + (r-l)/2;
        if (state_table[m].event == event) /* Check if x is present at mid */
        {
            found = TRUE;
            break;
        }
        else if (state_table[m].event < event) /* If x greater, ignore left half */
        {
            l = m + 1;
        }
        else /* If x is smaller, ignore right half */
        {
            r = m - 1;
        }
    }

    if(found == FALSE)
    {
        return FALSE;
    }

    *p_next_state = state_table[m].next_state;
    *pfhandler = state_table[m].pfhandler;
    return TRUE;
}


/*******************************************************************************
**
** Function         wiced_bt_hfp_hf_ssm_execute
**
** Description      Service state machine event handling function for Handsfree
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_hfp_hf_ssm_execute(wiced_bt_hfp_hf_scb_t *p_scb,
        wiced_bt_hfp_hf_data_t *p_data, uint8_t event)
{
    wiced_bool_t                    found      = WICED_FALSE;
    wiced_bt_hfp_hf_sm_act_t        pfhandler  = NULL;
    wiced_bt_hfp_hf_sst_tbl_entry_t state_table_entry;
    uint8_t                         next_state;


    if (p_scb == NULL)
    {
        /* This stream is not registered */
        WICED_BTHFP_ERROR("%s: ERROR: channel not registered\n", __FUNCTION__);
        return;
    }

    WICED_BTHFP_TRACE("%s executing event: %d state: %d, next_state:%d\n",__func__, event, p_scb->state);
    /* Look up the state table for the current state */
    state_table_entry = wiced_bt_hfp_hf_sst_tbl[p_scb->state];

    /* Search whether this event is handled by the current state */
    found = wiced_bt_hfp_hf_search_event(state_table_entry, event, &next_state, &pfhandler);
    if(found == FALSE)
    {
        WICED_BTHFP_TRACE("%s: event=0x%x state=0x%x not handled\n", __FUNCTION__,
            event, p_scb->state);
        return;
    }

#if (defined(WICED_BT_HFP_HF_DEBUG) && WICED_BT_HFP_HF_DEBUG == TRUE)
    WICED_BTHFP_TRACE("%s: p_scb:%x current-state=%s event=%s next-state=%s\n",
            __FUNCTION__, (void *)p_scb,
            wiced_bt_hfp_hf_st_code(p_scb->state),
            wiced_bt_hfp_hf_evt_code((uint8_t)event),
            wiced_bt_hfp_hf_st_code(next_state));
#else
    WICED_BTHFP_TRACE("%s: p_scb:%p current-state=0x%x event=0x%x next-state=0x%x\n",
            __FUNCTION__, p_scb, p_scb->state, event, next_state);
#endif

    p_scb->state  = next_state;

    if(pfhandler != NULL)
    {
        pfhandler(p_scb, p_data);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_hfp_hf_init_state_machine
**
** Description    Initialize state machine.
**
** Returns          WICED_SUCCESS if successfully initialized, else return WICED_FAILURE.
**
*******************************************************************************/
wiced_result_t wiced_bt_hfp_hf_init_state_machine(void)
{
    /* Loop through each state and ensure that the events are listed in order */
    int num_states = sizeof(wiced_bt_hfp_hf_sst_tbl)/sizeof(wiced_bt_hfp_hf_sst_tbl_entry_t);
    int i=0;

    for(i=0; i<num_states; i++) {
        wiced_bt_hfp_hf_sst_tbl_entry_t state_table_entry = wiced_bt_hfp_hf_sst_tbl[i];
        const wiced_bt_hfp_hf_event_map_t *state_table = state_table_entry.state_table;
        uint8_t size_of_table = state_table_entry.size_of_table;
        int j=0;

        for(j=1; j<size_of_table; j++)
        {
            /* If value of jth event is less than (j-1)th event, then the order is incorrect */
            if(state_table[j].event < state_table[j-1].event)
            {
                WICED_BTHFP_ERROR("%s: Incorrect state table entry. state:%d, entry:%d. \
                    IMPORTANT: THE EVENTS MUST BE LISTED IN ASCENDING ORDER\n", __FUNCTION__,i,j);
                return WICED_BADARG;
            }
        }
    }
    return WICED_SUCCESS;
}


/*****************************************************************************
**  Debug Functions
*****************************************************************************/
#if (defined(WICED_BT_HFP_HF_DEBUG) && WICED_BT_HFP_HF_DEBUG == TRUE)
/*******************************************************************************
** Function         wiced_bt_hfp_hf_st_code
** Description
** Returns          char *
*******************************************************************************/
char *wiced_bt_hfp_hf_st_code(uint8_t state)
{
    switch(state)
    {
        case WICED_BT_HFP_HF_INIT_SST:    return "INIT";
        case WICED_BT_HFP_HF_OPENING_SST: return "OPENING";
        case WICED_BT_HFP_HF_OPEN_SST:    return "OPEN";
        case WICED_BT_HFP_HF_CLOSING_SST: return "CLOSING";
        default:                          return "unknown";
    }
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_evt_code
** Description
** Returns          char *
*******************************************************************************/
char *wiced_bt_hfp_hf_evt_code(uint8_t evt_code)
{
    switch(evt_code)
    {
        case WICED_BT_HFP_HF_INVALID_EVT:           return "INVALID";
        case WICED_BT_HFP_HF_API_DEINIT_EVT:        return "API_DEINIT";
        case WICED_BT_HFP_HF_API_CONNECT_EVT:       return "API_CONNECT";
        case WICED_BT_HFP_HF_API_DISCONNECT_EVT:    return "API_DISCONNECT";
        case WICED_BT_HFP_HF_API_CALL_ACTION_EVT:   return "API_CALL_ACTION";
        case WICED_BT_HFP_HF_API_NOTIFY_VOLUME_EVT: return "API_NOTIFY_VOLUME";
        case WICED_BT_HFP_HF_API_SEND_AT_CMD_EVT:   return "API_SEND_AT_CMD";
        case WICED_BT_HFP_HF_RFC_CONNECT_EVT:       return "RFC_CONNECT";
        case WICED_BT_HFP_HF_RFC_DISCONNECT_EVT:    return "RFC_DISCONNECT";
        case WICED_BT_HFP_HF_RFC_DATA_EVT:          return "RFC_DATA";
        case WICED_BT_HFP_HF_SDP_DISC_OK_EVT:       return "SDP_DISC_OK";
        case WICED_BT_HFP_HF_SDP_DISC_FAIL_EVT:     return "SDP_DISC_FAIL";
        case WICED_BT_HFP_HF_CMD_TIMEOUT_EVT:       return "CMD_TIMEOUT";
        default:                                    return "unknown";
    }
}

#endif
