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
 * This is the stream state machine for audio sink.
 */

#include "wiced_bt_a2dp_sink_int.h"

/*****************************************************************************
** Constants and types
*****************************************************************************/

/* State table for init state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_a2dp_sink_state_evt_t enum*/
static const wiced_bt_a2dp_sink_event_map_t wiced_bt_a2dp_sink_sst_init[] =
{
    {WICED_BT_A2DP_SINK_API_CONNECT_EVT          , WICED_BT_A2DP_SINK_SIG_OPENING_SST, wiced_bt_a2dp_sink_do_sdp},
    {WICED_BT_A2DP_SINK_API_DISCONNECT_EVT       , WICED_BT_A2DP_SINK_INIT_SST       , wiced_bt_a2dp_sink_cleanup},
    {WICED_BT_A2DP_SINK_STR_CLOSE_OK_EVT         , WICED_BT_A2DP_SINK_INIT_SST       , wiced_bt_a2dp_sink_hdl_str_close},
    {WICED_BT_A2DP_SINK_STR_CONFIG_IND_EVT       , WICED_BT_A2DP_SINK_INCOMING_SST   , wiced_bt_a2dp_sink_config_ind},
    {WICED_BT_A2DP_SINK_AVDT_CONNECT_EVT         , WICED_BT_A2DP_SINK_SIG_OPEN_SST   , wiced_bt_a2dp_sink_sig_opened}
};

/* State table for signal opening state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_a2dp_sink_state_evt_t enum*/
static const wiced_bt_a2dp_sink_event_map_t wiced_bt_a2dp_sink_sst_sig_opening[] =
{
    {WICED_BT_A2DP_SINK_API_DISCONNECT_EVT       , WICED_BT_A2DP_SINK_INIT_SST       , wiced_bt_a2dp_sink_cleanup},
    {WICED_BT_A2DP_SINK_SDP_DISC_OK_EVT          , WICED_BT_A2DP_SINK_SIG_OPENING_SST, wiced_bt_a2dp_sink_connect_req},
    {WICED_BT_A2DP_SINK_SDP_DISC_FAIL_EVT        , WICED_BT_A2DP_SINK_INIT_SST       , wiced_bt_a2dp_sink_sdp_failed},
    {WICED_BT_A2DP_SINK_STR_CLOSE_OK_EVT         , WICED_BT_A2DP_SINK_SIG_OPENING_SST, wiced_bt_a2dp_sink_hdl_str_close},
    {WICED_BT_A2DP_SINK_STR_CONFIG_IND_EVT       , WICED_BT_A2DP_SINK_INCOMING_SST   , wiced_bt_a2dp_sink_config_ind},
    {WICED_BT_A2DP_SINK_AVDT_CONNECT_EVT         , WICED_BT_A2DP_SINK_SIG_OPEN_SST   , wiced_bt_a2dp_sink_sig_opened},
    {WICED_BT_A2DP_SINK_AVDT_DISCONNECT_EVT      , WICED_BT_A2DP_SINK_INIT_SST       , wiced_bt_a2dp_sink_sig_open_fail}
};

/* State table for signaling channel opened state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_a2dp_sink_state_evt_t enum*/
static const wiced_bt_a2dp_sink_event_map_t wiced_bt_a2dp_sink_sst_sig_open[] =
{
    {WICED_BT_A2DP_SINK_API_DISCONNECT_EVT       , WICED_BT_A2DP_SINK_SIG_OPEN_SST   , wiced_bt_a2dp_sink_sig_hdl_ap_close_disconnect_req},
    {WICED_BT_A2DP_SINK_API_DELAY_EVT            , WICED_BT_A2DP_SINK_SIG_OPEN_SST   , wiced_bt_a2dp_sink_delay_send},
    {WICED_BT_A2DP_SINK_STR_CLOSE_OK_EVT         , WICED_BT_A2DP_SINK_SIG_OPEN_SST   , wiced_bt_a2dp_sink_hdl_str_close},
    {WICED_BT_A2DP_SINK_STR_CONFIG_IND_EVT       , WICED_BT_A2DP_SINK_INCOMING_SST   , wiced_bt_a2dp_sink_config_ind},
    {WICED_BT_A2DP_SINK_AVDT_DISCONNECT_EVT      , WICED_BT_A2DP_SINK_INIT_SST       , wiced_bt_a2dp_sink_sig_closed_cleanup}
};

/* State table for incoming state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_a2dp_sink_state_evt_t enum*/
static const wiced_bt_a2dp_sink_event_map_t wiced_bt_a2dp_sink_sst_incoming[] =
{
    {WICED_BT_A2DP_SINK_API_DISCONNECT_EVT       , WICED_BT_A2DP_SINK_CLOSING_SST    , wiced_bt_a2dp_sink_disconnect_req},
    {WICED_BT_A2DP_SINK_API_DELAY_EVT            , WICED_BT_A2DP_SINK_INCOMING_SST   , wiced_bt_a2dp_sink_delay_send},
    {WICED_BT_A2DP_SINK_SDP_DISC_OK_EVT          , WICED_BT_A2DP_SINK_INCOMING_SST   , wiced_bt_a2dp_sink_free_sdb},
    {WICED_BT_A2DP_SINK_SDP_DISC_FAIL_EVT        , WICED_BT_A2DP_SINK_INCOMING_SST   , wiced_bt_a2dp_sink_free_sdb},
    {WICED_BT_A2DP_SINK_STR_OPEN_OK_EVT          , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_str_opened},
    {WICED_BT_A2DP_SINK_STR_CLOSE_OK_EVT         , WICED_BT_A2DP_SINK_INIT_SST       , wiced_bt_a2dp_sink_str_open_fail},
    {WICED_BT_A2DP_SINK_STR_CONFIG_RSP_OK_EVT    , WICED_BT_A2DP_SINK_INCOMING_SST   , wiced_bt_a2dp_sink_setconfig_rsp},
    {WICED_BT_A2DP_SINK_STR_CONFIG_RSP_FAIL_EVT  , WICED_BT_A2DP_SINK_INIT_SST       , wiced_bt_a2dp_sink_setconfig_rsp},
    {WICED_BT_A2DP_SINK_AVDT_DISCONNECT_EVT      , WICED_BT_A2DP_SINK_INIT_SST       , wiced_bt_a2dp_sink_sig_closed_cleanup}
};

/* State table for open state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_a2dp_sink_state_evt_t enum*/
static const wiced_bt_a2dp_sink_event_map_t wiced_bt_a2dp_sink_sst_open[] =
{
    {WICED_BT_A2DP_SINK_API_DISCONNECT_EVT       , WICED_BT_A2DP_SINK_CLOSING_SST    , wiced_bt_a2dp_sink_do_close},
    {WICED_BT_A2DP_SINK_API_START_EVT            , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_do_start},
    {WICED_BT_A2DP_SINK_API_START_RESP_EVT       , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_send_start_resp},
    {WICED_BT_A2DP_SINK_API_SUSPEND_EVT          , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_str_stopped},
    {WICED_BT_A2DP_SINK_API_DELAY_EVT            , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_delay_send},
    {WICED_BT_A2DP_SINK_SDP_DISC_OK_EVT          , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_free_sdb},
    {WICED_BT_A2DP_SINK_SDP_DISC_FAIL_EVT        , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_free_sdb},
    {WICED_BT_A2DP_SINK_STR_START_IND_EVT        , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_start_ind},
    {WICED_BT_A2DP_SINK_STR_START_CFM_EVT        , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_start_ok},
    {WICED_BT_A2DP_SINK_STR_START_FAIL_EVT       , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_start_failed},
    {WICED_BT_A2DP_SINK_STR_CLOSE_OK_EVT         , WICED_BT_A2DP_SINK_CLOSING_SST    , wiced_bt_a2dp_sink_hdl_str_close},
    {WICED_BT_A2DP_SINK_STR_CONFIG_IND_EVT       , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_rej_conn},
    {WICED_BT_A2DP_SINK_STR_CONFIG_RSP_OK_EVT    , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_reconfig_rsp},
    {WICED_BT_A2DP_SINK_STR_CONFIG_RSP_FAIL_EVT  , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_reconfig_rsp},
    {WICED_BT_A2DP_SINK_STR_SUSPEND_CFM_EVT      , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_suspend_cfm},
    {WICED_BT_A2DP_SINK_STR_RECONFIG_IND_EVT     , WICED_BT_A2DP_SINK_OPEN_SST       , wiced_bt_a2dp_sink_config_ind},
    {WICED_BT_A2DP_SINK_AVDT_DISCONNECT_EVT      , WICED_BT_A2DP_SINK_INIT_SST       , wiced_bt_a2dp_sink_sig_closed_cleanup}
};

/* State table for closing state */
/* IMPORTANT:THE EVENTS MUST BE LISTED IN ASCENDING ORDER OF wiced_bt_a2dp_sink_state_evt_t enum*/
static const wiced_bt_a2dp_sink_event_map_t wiced_bt_a2dp_sink_sst_closing[] =
{
    {WICED_BT_A2DP_SINK_API_DISCONNECT_EVT       , WICED_BT_A2DP_SINK_CLOSING_SST    , wiced_bt_a2dp_sink_disconnect_req},
    {WICED_BT_A2DP_SINK_STR_OPEN_OK_EVT          , WICED_BT_A2DP_SINK_CLOSING_SST    , wiced_bt_a2dp_sink_do_close},
    {WICED_BT_A2DP_SINK_STR_OPEN_FAIL_EVT        , WICED_BT_A2DP_SINK_CLOSING_SST    , wiced_bt_a2dp_sink_disconnect_req},
    {WICED_BT_A2DP_SINK_STR_CLOSE_OK_EVT         , WICED_BT_A2DP_SINK_CLOSING_SST    , wiced_bt_a2dp_sink_hdl_str_close},
    {WICED_BT_A2DP_SINK_STR_CONFIG_IND_EVT       , WICED_BT_A2DP_SINK_INCOMING_SST   , wiced_bt_a2dp_sink_config_ind},
    {WICED_BT_A2DP_SINK_AVDT_DISCONNECT_EVT      , WICED_BT_A2DP_SINK_INIT_SST       , wiced_bt_a2dp_sink_sig_closed_cleanup}
};

/* State table */
static const wiced_bt_a2dp_sink_sst_tbl_entry_t wiced_bt_a2dp_sst_tbl[] =
{
    {wiced_bt_a2dp_sink_sst_init,       sizeof(wiced_bt_a2dp_sink_sst_init)/sizeof(wiced_bt_a2dp_sink_event_map_t)},
    {wiced_bt_a2dp_sink_sst_sig_opening,sizeof(wiced_bt_a2dp_sink_sst_sig_opening)/sizeof(wiced_bt_a2dp_sink_event_map_t)},
    {wiced_bt_a2dp_sink_sst_sig_open,   sizeof(wiced_bt_a2dp_sink_sst_sig_open)/sizeof(wiced_bt_a2dp_sink_event_map_t)},
    {wiced_bt_a2dp_sink_sst_incoming,   sizeof(wiced_bt_a2dp_sink_sst_incoming)/sizeof(wiced_bt_a2dp_sink_event_map_t)},
    {wiced_bt_a2dp_sink_sst_open,       sizeof(wiced_bt_a2dp_sink_sst_open)/sizeof(wiced_bt_a2dp_sink_event_map_t)},
    {wiced_bt_a2dp_sink_sst_closing,    sizeof(wiced_bt_a2dp_sink_sst_closing)/sizeof(wiced_bt_a2dp_sink_event_map_t)},
};

/*******************************************************************************
**
** Function         wiced_bt_a2dp_sink_search_event
**
** Description      Search for the event in the state table and return the
**                       next state and handler if the event is handled by the state table.
**
** Returns      Returns WICED_TRUE if the event is handled by the state table. Else return WICED_FALSE.
**
*******************************************************************************/
static wiced_bool_t wiced_bt_a2dp_sink_search_event(
    wiced_bt_a2dp_sink_sst_tbl_entry_t state_table_entry, uint8_t event,
    uint8_t *p_next_state, wiced_bt_a2dp_sink_sm_act_t *pfhandler)
{
    int l = 0;
    int r = state_table_entry.size_of_table - 1;
    int m = 0;
    wiced_bool_t found = WICED_FALSE;
    const wiced_bt_a2dp_sink_event_map_t *state_table = state_table_entry.state_table;

    while (l <= r)
    {
        m = l + (r-l)/2;
        if (state_table[m].event == event) /* Check if x is present at mid */
        {
            found = WICED_TRUE;
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

    if(found == WICED_FALSE)
    {
        return WICED_FALSE;
    }

    *p_next_state = state_table[m].next_state;
    *pfhandler = state_table[m].pfhandler;
    return WICED_TRUE;
}


/*******************************************************************************
**
** Function         wiced_bt_a2dp_csm_execute
**
** Description      Stream state machine event handling function for A2DP sink
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_a2dp_sink_ssm_execute(wiced_bt_a2dp_sink_ccb_t *p_ccb,
        wiced_bt_a2dp_sink_data_t *p_data, uint8_t event)
{
    wiced_bool_t                       found      = WICED_FALSE;
    wiced_bt_a2dp_sink_sm_act_t        pfhandler  = NULL;
    wiced_bt_a2dp_sink_sst_tbl_entry_t state_table_entry;
    uint8_t                            next_state;

    if (p_ccb->p_scb == NULL)
    {
        /* This stream is not registered */
        WICED_BTA2DP_SINK_ERROR("%s: ERROR: channel not registered \n", __FUNCTION__);
        return;
    }

    /* Look up the state table for the current state */
    state_table_entry = wiced_bt_a2dp_sst_tbl[p_ccb->state];

    /* Search whether this event is handled by the current state */
    found = wiced_bt_a2dp_sink_search_event(state_table_entry, event, &next_state, &pfhandler);
    if(found == WICED_FALSE)
    {
        WICED_BTA2DP_TRACE("%s: event=0x%x state=0x%x not handled \n", __FUNCTION__,
            event, p_ccb->state);
        return;
    }

#if (defined(WICED_BT_A2DP_SINK_DEBUG) && WICED_BT_A2DP_SINK_DEBUG == TRUE)
        WICED_BTA2DP_TRACE("%s: current-state=%s event=%s next-state=%s \n",
            __FUNCTION__,
            wiced_bt_a2dp_sink_st_code(p_ccb->state),
            wiced_bt_a2dp_sink_evt_code(event),
            wiced_bt_a2dp_sink_st_code(next_state));
#else
        WICED_BTA2DP_TRACE("%s: current-state=0x%x event=0x%x next-state=0x%x \n",
            __FUNCTION__, p_ccb->state, event, next_state);
#endif

    p_ccb->state  = next_state;

    if(pfhandler != NULL)
    {
        pfhandler(p_ccb, p_data);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_sink_init_state_machine
**
** Description    Initialize state machine.
**
** Returns          WICED_SUCCESS if successfully initialized, else return WICED_FAILURE.
**
*******************************************************************************/
wiced_result_t wiced_bt_a2dp_sink_init_state_machine(void)
{
    /* Loop through each state and ensure that the events are listed in order */
    int num_states = sizeof(wiced_bt_a2dp_sst_tbl)/sizeof(wiced_bt_a2dp_sink_sst_tbl_entry_t);
    int i=0;

    for(i=0; i<num_states; i++) {
        wiced_bt_a2dp_sink_sst_tbl_entry_t state_table_entry = wiced_bt_a2dp_sst_tbl[i];
        const wiced_bt_a2dp_sink_event_map_t *state_table = state_table_entry.state_table;
        uint8_t size_of_table = state_table_entry.size_of_table;
        int j=0;

        for(j=1; j<size_of_table; j++)
        {
            /* If value of jth event is less than (j-1)th event, then the order is incorrect */
            if(state_table[j].event < state_table[j-1].event)
            {
                WICED_BTA2DP_SINK_ERROR("%s: Incorrect state table entry. state:%d, entry:%d. \
                    IMPORTANT: THE EVENTS MUST BE LISTED IN ASCENDING ORDER \n", __FUNCTION__,i,j);
                return WICED_BADARG;
            }
        }
    }
    return WICED_SUCCESS;
}


/*****************************************************************************
**  Debug Functions
*****************************************************************************/
#if (defined(WICED_BT_A2DP_SINK_DEBUG) && WICED_BT_A2DP_SINK_DEBUG == TRUE)
/*******************************************************************************
**
** Function         wiced_bt_a2dp_sink_st_code
**
** Description
**
** Returns          char *
**
*******************************************************************************/
char *wiced_bt_a2dp_sink_st_code(uint8_t state)
{
    switch(state)
    {
        case WICED_BT_A2DP_SINK_INIT_SST:        return "INIT";
        case WICED_BT_A2DP_SINK_SIG_OPENING_SST: return "SIG_OPENING";
        case WICED_BT_A2DP_SINK_SIG_OPEN_SST:    return "SIG_OPEN";
        case WICED_BT_A2DP_SINK_INCOMING_SST:    return "INCOMING";
        case WICED_BT_A2DP_SINK_OPEN_SST:        return "OPEN";
        case WICED_BT_A2DP_SINK_CLOSING_SST:     return "CLOSING";
        default:                                 return "unknown";
    }
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_evt_code
**
** Description
**
** Returns          char *
**
*******************************************************************************/
char *wiced_bt_a2dp_sink_evt_code(uint8_t evt_code)
{
    switch(evt_code)
    {
        case WICED_BT_A2DP_SINK_INVALID_EVT:            return "INVALID";
        case WICED_BT_A2DP_SINK_API_DEINIT_EVT:         return "API_DEINIT";
        case WICED_BT_A2DP_SINK_SIG_CHG_EVT:            return "SIG_CHG";
        case WICED_BT_A2DP_SINK_AVDT_REPOPT_CONN_EVT:   return "AVDT_REPOPT_CONN";
        case WICED_BT_A2DP_SINK_API_CONNECT_EVT:        return "API_CONNECT";
        case WICED_BT_A2DP_SINK_API_DISCONNECT_EVT:     return "API_DISCONNECT";
        case WICED_BT_A2DP_SINK_API_START_EVT:          return "API_START";
        case WICED_BT_A2DP_SINK_API_SUSPEND_EVT:        return "API_SUSPEND";
        case WICED_BT_A2DP_SINK_API_DELAY_EVT:          return "API_DELAY";
        case WICED_BT_A2DP_SINK_SDP_DISC_OK_EVT:        return "SDP_DISC_OK";
        case WICED_BT_A2DP_SINK_SDP_DISC_FAIL_EVT:      return "SDP_DISC_FAIL";
        case WICED_BT_A2DP_SINK_STR_OPEN_OK_EVT:        return "STR_OPEN_OK";
        case WICED_BT_A2DP_SINK_STR_OPEN_FAIL_EVT:      return "STR_OPEN_FAIL";
        case WICED_BT_A2DP_SINK_STR_START_IND_EVT:      return "STR_START_IND";
        case WICED_BT_A2DP_SINK_STR_START_CFM_EVT:      return "STR_START_CFM";
        case WICED_BT_A2DP_SINK_STR_START_FAIL_EVT:     return "STR_START_FAIL";
        case WICED_BT_A2DP_SINK_STR_CLOSE_OK_EVT:       return "STR_CLOSE_OK";
        case WICED_BT_A2DP_SINK_STR_CONFIG_IND_EVT:     return "STR_CONFIG_IND";
        case WICED_BT_A2DP_SINK_STR_CONFIG_RSP_OK_EVT:  return "STR_CONFIG_RSP_OK";
        case WICED_BT_A2DP_SINK_STR_CONFIG_RSP_FAIL_EVT:return "STR_CONFIG_RSP_FAIL";
        case WICED_BT_A2DP_SINK_STR_SUSPEND_CFM_EVT:    return "STR_SUSPEND_CFM";
        case WICED_BT_A2DP_SINK_STR_RECONFIG_IND_EVT:   return "STR_RECONFIG_IND";
        case WICED_BT_A2DP_SINK_AVDT_CONNECT_EVT:       return "AVDT_CONNECT";
        case WICED_BT_A2DP_SINK_AVDT_DISCONNECT_EVT:    return "AVDT_DISCONNECT";
        default:             return "unknown";
    }
}

#endif
