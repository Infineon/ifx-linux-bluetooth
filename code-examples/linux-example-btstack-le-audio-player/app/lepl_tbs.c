
/*
 * $ Copyright Cypress Semiconductor $
 */

#include "lepl.h"

#define WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT 10

lepl_ccs_states_t lepl_ccs_state = CALL_CONTROL_SERVER_STATE_IDLE;
extern wiced_bool_t lepl_ccs_pacs_does_peer_support_ringtone(uint16_t conn_id);

#define CODEC_CONFIG BAP_CODEC_CONFIG_32_1_2

void lepl_set_call_control_server_state(lepl_ccs_states_t state)
{
    lepl_ccs_state = state;
}

lepl_ccs_states_t lepl_get_call_control_server_state()
{
    return lepl_ccs_state;
}

char default_bearer_provider_name[] = "CyIFX_provider";
char default_bearer_uci[] = "uci";
char default_bearer_uri[] = "skype,phone";
char default_friendly_name[] = "tel:+19991110011";
char default_tg_caller_id[] = "aaa:77777";

void get_call_states(wiced_bt_ga_tbs_call_state_list_t *p_data, tbs_call_state_data_t *p_call_list, uint8_t num_calls);
tbs_call_state_data_t *get_call_state(uint8_t call_id, tbs_call_state_data_t *p_call_list, uint8_t num_calls);
void get_call_list(wiced_bt_ga_tbs_current_call_list_t *p_list, tbs_call_state_data_t *p_call_list, uint8_t num_calls);
lepl_tbs_data_t *telephone_bearer_data = &g_lepl_gatt_cb.tbs_data;

wiced_bt_ga_tbs_call_operation_result_t place_call(wiced_bt_ga_string_t data, uint8_t *call_id)
{
    uint8_t len;
    if (telephone_bearer_data->current_call_id != 255)
        telephone_bearer_data->current_call_id++;
    else
        telephone_bearer_data->current_call_id = 1;

    *call_id = telephone_bearer_data->current_call_id;

    telephone_bearer_data->num_calls++;

    //fill call friendly name
    len = (uint8_t)strlen(default_friendly_name);
    memcpy(telephone_bearer_data->call_friendly_name.name, default_friendly_name, len);

    telephone_bearer_data->call_friendly_name.call_id = telephone_bearer_data->current_call_id;

    //place the call and return the status back
    return WICED_BT_CALL_SUCCESS;
}

wiced_bool_t check_URI_validity(wiced_bt_ga_string_t data)
{
    if (telephone_bearer_data->is_caller_id_invalid_enabled)
        return WICED_FALSE;
    else
        return WICED_TRUE;
}

wiced_bt_ga_tbs_call_operation_result_t terminate_call(uint8_t call_id,
                                                       wiced_bt_ga_tbs_call_termination_reason_t *reason)
{

    //terminate the call and return the result and the reason for termination
    *reason = (wiced_bt_ga_tbs_call_termination_reason_t)WICED_BT_GA_TBS_CLIENT_TERMINATED;
    telephone_bearer_data->num_calls--;
    return WICED_BT_CALL_SUCCESS;
}

wiced_bt_ga_tbs_call_operation_result_t change_call_state(uint8_t call_id, wiced_bt_ga_tbs_call_state_t call_state)
{
    wiced_bt_ga_tbs_call_operation_result_t res;
    switch (call_state)
    {
    case WICED_BT_GA_TBS_CALL_STATE_ACTIVE:
        //make the call active and return the status
        res = WICED_BT_CALL_SUCCESS;
        break;

    case WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD:
        //locally hold the call and return the status
        res = WICED_BT_CALL_SUCCESS;
        break;

    case WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD:
        //locally and remotely hold the call and return the status
        res = WICED_BT_CALL_SUCCESS;
        break;

    case WICED_BT_GA_TBS_CALL_STATE_REMOTELY_HELD:
        //remotely hold the call and return the status
        res = WICED_BT_CALL_SUCCESS;
        break;

    case WICED_BT_GA_TBS_CALL_STATE_ALERTING:
        //the call is in alerting state
        res = WICED_BT_CALL_SUCCESS;
        break;

    default:
        res = WICED_BT_CALL_OPCODE_NOT_SUPPORTED;
        break;
    }
    if (res == WICED_BT_CALL_SUCCESS)
    {
        le_audio_rpc_update_call_state(0, call_id, NULL, call_state); // conn id 0 as LEPL is TBS server
    }
    return res;
}

void get_call_list(wiced_bt_ga_tbs_current_call_list_t *p_list, tbs_call_state_data_t *p_call_list, uint8_t num_calls)
{
    uint8_t call_index = 0, i;
    wiced_bt_ga_tbs_current_call_t *p_call_dst;
    tbs_call_state_data_t *p_call_src;

    for (i = 0; i < num_calls; i++)
    {
        p_call_src = (tbs_call_state_data_t *)(p_call_list + i);

        if (p_call_src->in_use)
        {

            p_call_dst = &(p_list->call_data[call_index]);
            // allocating memory
            p_call_dst->remote_caller_id.str = (char*)calloc(1,strlen(p_call_src->URI));

            p_call_dst->call_id = p_call_src->call_id;
            p_call_dst->call_state = p_call_src->call_state;
            p_call_dst->call_flags = p_call_src->call_flags;

            p_call_dst->remote_caller_id.len = strlen(p_call_src->URI);
            if (p_call_dst->remote_caller_id.str && p_call_dst->remote_caller_id.len)
            {
                memcpy(p_call_dst->remote_caller_id.str, p_call_src->URI, strlen(p_call_src->URI));
            }
            p_call_dst->remote_caller_id.len = strlen(p_call_src->URI);
            call_index++;
        }
    }
    p_list->num_calls = call_index;
    WICED_BT_TRACE("[%s] num calls %d ", __FUNCTION__, call_index);
}

void get_call_states(wiced_bt_ga_tbs_call_state_list_t *p_data, tbs_call_state_data_t *p_call_list, uint8_t num_calls)
{
    tbs_call_state_data_t *p_call_state_src;
    uint8_t call_index = 0, i;
    wiced_bt_ga_tbs_call_state_data_t *p_call_state_dst;
    for (i = 0; i < num_calls; i++)
    {
        p_call_state_src = (tbs_call_state_data_t *)(p_call_list + i);
        if (p_call_state_src->in_use)
        {
            p_call_state_dst = &(p_data->call_state[call_index]);

            p_call_state_dst->call_id = p_call_state_src->call_id;
            p_call_state_dst->call_state = p_call_state_src->call_state;
            p_call_state_dst->call_flags = p_call_state_src->call_flags;
            call_index++;
        }
    }
    p_data->num_calls = call_index;
}

static void notify_tbs_characteristics(gatt_intf_attribute_t *characteristics, wiced_bt_ga_tbs_data_t *p_data)
{
    for (int i = 0; i < MAX_CONNECTION_INSTANCE; i++)
    {
        if (g_lepl_gatt_cb.unicast_clcb[i].in_use)
        {

            gatt_interface_notify_characteristic(g_lepl_gatt_cb.unicast_clcb[i].conn_id,
                                                 g_lepl_gatt_cb.local_profiles.p_gtbs,
                                                 characteristics,
                                                 p_data);
        }
    }
}

tbs_call_state_data_t *get_call_state(uint8_t call_id, tbs_call_state_data_t *p_call_list, uint8_t num_calls)
{
    uint8_t i;
    tbs_call_state_data_t *p_call_state_src;
    WICED_BT_TRACE("[%s] ", __FUNCTION__);

    for (i = 0; i < num_calls; i++)
    {
        p_call_state_src = (tbs_call_state_data_t *)(p_call_list + i);
        if (p_call_state_src->in_use && p_call_state_src->call_id == call_id)
        {
            return p_call_state_src;
        }
    }
    return NULL;
}

static wiced_bt_ga_tbs_call_operation_result_t lepl_tbs_accept_call(uint16_t conn_id,
                                                                    uint8_t call_id,
                                                                            tbs_call_state_data_t *p_call_list,
                                                                            uint8_t num_calls,
                                                                            wiced_bool_t is_server_initiated)
{
    wiced_bt_ga_tbs_call_operation_result_t result = WICED_BT_CALL_INVALID_CALL_ID;
    tbs_call_state_data_t *p_call_state;
    tbs_call_state_data_t *p_call_state_accept;

    WICED_BT_TRACE("[%s] ", __FUNCTION__);

    p_call_state_accept = get_call_state(call_id, p_call_list, num_calls);

    if (p_call_state_accept != NULL)
    {
        if (p_call_state_accept->call_state != WICED_BT_GA_TBS_CALL_STATE_INCOMING &&
            p_call_state_accept->call_state != WICED_BT_GA_TBS_CALL_STATE_DIALING)
        {
            result = WICED_BT_CALL_STATE_MISMATCH;
        }
        else
        {
            for (int i = 0; i < num_calls; i++)
            {
                p_call_state = (tbs_call_state_data_t *)(p_call_list + i);
                if (p_call_state->in_use == WICED_TRUE && p_call_state->call_id != call_id)
                {
                    if (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_ACTIVE)
                    {
                        //call the application's hold call function
                        result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD);

                        if (result == WICED_BT_CALL_SUCCESS)
                        {
                            //for all the other caller ID having call state active must be moved to locally held
                            p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD;
                        }
                    }
                    else if (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_REMOTELY_HELD)
                    {
                        //call the application's hold call function
                        result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD);

                        if (result == WICED_BT_CALL_SUCCESS)
                        {
                            //for all the other caller ID having call state active must be moved to locally held
                            p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD;
                        }
                    }
                }
            }

            //call the application's accept call function which will actually accept the call
            result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_ACTIVE);

            if (result == WICED_BT_CALL_SUCCESS)
            {
                //move call to active state
                p_call_state_accept->call_state = WICED_BT_GA_TBS_CALL_STATE_ACTIVE;
            }
        }
    }

    if (lepl_get_call_control_server_state() == CALL_CONTROL_SERVER_STATE_INBAND_RINGTONE)
    {
        lepl_cap_stop_media_streaming(conn_id); //Stop sending ringtone Media
    }
    else
    {
        lepl_ccs_start_streaming(conn_id);
    }

    return result;
}

static wiced_bt_ga_tbs_call_operation_result_t lepl_tbs_terminate_call(
    uint16_t conn_id,
    uint8_t call_id,
    wiced_bt_ga_tbs_call_termination_reason_t *reason,
    tbs_call_state_data_t *p_call_list,
    uint8_t num_calls,
    wiced_bool_t is_server_initiated)
{
    WICED_BT_TRACE("[%s] ", __FUNCTION__);

    wiced_bt_ga_tbs_call_operation_result_t result = WICED_BT_CALL_INVALID_CALL_ID;
    tbs_call_state_data_t *p_call_state;

    p_call_state = get_call_state(call_id, p_call_list, num_calls);

    if (p_call_state != NULL)
    {
        if (!is_server_initiated)
        {
            //call the application's terminate call function
            result = terminate_call(call_id, reason);
        }
        else
        {
            result = WICED_BT_CALL_SUCCESS;
        }

        //make state machine changes only if terminate call was successful
        if (result == WICED_BT_CALL_SUCCESS)
        {
            //clear data
            p_call_state->in_use = 0;

            wiced_bt_ga_tbs_data_t data = {0};
            gatt_intf_attribute_t characteristic = {0};
            characteristic.characteristic_type = TBS_CALL_TERMINATION_REASON_CHARACTERISTIC;
            data.call_termination_reason.call_id = call_id;
            data.call_termination_reason.termination_reason = *reason;
            notify_tbs_characteristics(&characteristic, &data);

            le_audio_rpc_send_call_terminated_event(0, call_id, *reason);
        }
    }
    lepl_ccs_states_t call_state = lepl_get_call_control_server_state();
    if (call_state == CALL_CONTROL_SERVER_STATE_INBAND_RINGTONE)
    {
        lepl_cap_stop_media_streaming(conn_id);
    }
    else
    {
        lepl_cap_stop_conv_streaming(conn_id);
    }
    lepl_set_call_control_server_state(CALL_CONTROL_SERVER_STATE_IDLE);
    lepl_cap_set_next_application_state(LEPL_APP_STATE_IDLE, 0xFF);
    return result;
}

static wiced_bt_ga_tbs_call_operation_result_t lepl_tbs_hold_call(uint8_t call_id,
                                                           tbs_call_state_data_t *p_call_list,
                                                           uint8_t num_calls)
{
    WICED_BT_TRACE("[%s] ", __FUNCTION__);

    wiced_bt_ga_tbs_call_operation_result_t result = WICED_BT_CALL_INVALID_CALL_ID;
    tbs_call_state_data_t *p_call_state;

    p_call_state = get_call_state(call_id, p_call_list, num_calls);

    if (p_call_state != NULL)
    {

        if (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_INCOMING ||
            p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_ACTIVE)
        {
            //call the application's hold call function
            result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD);

            //make state machine changes only if hold call was successful
            if (result == WICED_BT_CALL_SUCCESS)
            {
                // incoming / active ----> locally held
                p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD;
            }
        }
        else if (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_REMOTELY_HELD)
        {
            //call the application's hold call function
            result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD);

            //make state machine changes only if hold call was successful
            if (result == WICED_BT_CALL_SUCCESS)
            {
                // remotely----> locally and remotely held
                p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD;
            }
        }
        else
        {
            return WICED_BT_CALL_STATE_MISMATCH;
        }
    }
    return result;
}

static wiced_bt_ga_tbs_call_operation_result_t lepl_tbs_retrieve_call(uint8_t call_id,
                                                               tbs_call_state_data_t *p_call_list,
                                                               uint8_t num_calls)
{
    WICED_BT_TRACE("[%s] ", __FUNCTION__);
    uint8_t i;
    wiced_bt_ga_tbs_call_operation_result_t result = WICED_BT_CALL_INVALID_CALL_ID;
    tbs_call_state_data_t *p_call_state;

    p_call_state = get_call_state(call_id, p_call_list, num_calls);

    if (p_call_state != NULL)
    {
        if (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD)
        {
            result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_ACTIVE);

            if (result == WICED_BT_CALL_SUCCESS)
            {
                p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_ACTIVE;
                //move all other calls to inactive state
                for (i = 0; i < num_calls; i++)
                {
                    p_call_state = (tbs_call_state_data_t *)(p_call_list + i);

                    if (p_call_state->in_use == 1 && p_call_state->call_id != call_id &&
                        p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_ACTIVE)
                    {
                        result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD);
                        if (result == WICED_BT_CALL_SUCCESS)
                        {
                            p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD;
                        }
                    }
                    else if (p_call_state->in_use == 1 && p_call_state->call_id != call_id &&
                             p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_REMOTELY_HELD)
                    {
                        result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD);
                        if (result == WICED_BT_CALL_SUCCESS)
                        {
                            p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD;
                        }
                    }
                }
            }
        }
        else if (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD)
        {
            result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_REMOTELY_HELD);
            if (result == WICED_BT_CALL_SUCCESS)
            {
                p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_REMOTELY_HELD;

                //move all other calls to inactive state
                for (i = 0; i < num_calls; i++)
                {
                    p_call_state = (tbs_call_state_data_t *)(p_call_list + i);

                    if (p_call_state->in_use == 1 && p_call_state->call_id != call_id &&
                        p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_ACTIVE)
                    {
                        result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD);
                        if (result == WICED_BT_CALL_SUCCESS)
                        {
                            p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD;
                        }
                    }
                    else if (p_call_state->in_use == 1 && p_call_state->call_id != call_id &&
                             p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_REMOTELY_HELD)
                    {
                        result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD);
                        if (result == WICED_BT_CALL_SUCCESS)
                        {
                            p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD;
                        }
                    }
                }
            }
        }
    }
    return result;
}

static wiced_bt_ga_tbs_call_operation_result_t lepl_tbs_retrieve_remotely_hold_call(uint8_t call_id,
                                                                             tbs_call_state_data_t *p_call_list,
                                                                             uint8_t num_calls)
{
    WICED_BT_TRACE("[%s] ", __FUNCTION__);
    uint8_t i;
    wiced_bt_ga_tbs_call_operation_result_t result = WICED_BT_CALL_INVALID_CALL_ID;
    tbs_call_state_data_t *p_call_state;

    p_call_state = get_call_state(call_id, p_call_list, num_calls);

    if (p_call_state != NULL)
    {
        if (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_REMOTELY_HELD)
        {
            result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_ACTIVE);

            if (result == WICED_BT_CALL_SUCCESS)
            {
                p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_ACTIVE;
                //move all other calls to inactive state
                for (i = 0; i < num_calls; i++)
                {
                    p_call_state = (tbs_call_state_data_t *)(p_call_list + i);

                    if (p_call_state->in_use == 1 && p_call_state->call_id != call_id &&
                        p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_ACTIVE)
                    {
                        result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD);
                        if (result == WICED_BT_CALL_SUCCESS)
                        {
                            p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD;
                        }
                    }
                }
            }
        }
        else if (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD)
        {
            result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD);
            if (result == WICED_BT_CALL_SUCCESS)
            {
                p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD;
            }
        }
    }
    return result;
}

#if 0
static wiced_bt_ga_tbs_call_operation_result_t lepl_tbs_alert_call(uint8_t call_id,
                                                            tbs_call_state_data_t *p_call_list,
                                                            uint8_t num_calls)
{
    WICED_BT_TRACE("[%s] ", __FUNCTION__);
    wiced_bt_ga_tbs_call_operation_result_t result = WICED_BT_CALL_INVALID_CALL_ID;
    tbs_call_state_data_t *p_call_state;

    p_call_state = get_call_state(call_id, p_call_list, num_calls);

    if (p_call_state != NULL)
    {
        if (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_DIALING)
        {
            result = change_call_state(call_id, WICED_BT_GA_TBS_CALL_STATE_ALERTING);

            if (result == WICED_BT_CALL_SUCCESS)
            {
                p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_ALERTING;
            }
        }
    }
    return result;
}
#endif

static wiced_bt_ga_tbs_call_operation_result_t lepl_tbs_place_call(tbs_call_state_data_t *p_call_list,
                                                            uint8_t num_calls,
                                                            wiced_bt_ga_string_t data)
{
    WICED_BT_TRACE("[%s] ", __FUNCTION__);
    uint8_t i;
    wiced_bt_ga_tbs_call_operation_result_t result = WICED_BT_CALL_LACK_OF_RESOURCES;
    tbs_call_state_data_t *p_call_state;

    if (!check_URI_validity(data))
    {
        result = WICED_BT_CALL_INVALID_URI;
    }
    else
    {
        // for the caller ID in the parameter , we have to set the state to alerting
        for (i = 0; i < num_calls; i++)
        {
            p_call_state = (tbs_call_state_data_t *)(p_call_list + i);

            if (p_call_state->in_use == WICED_FALSE)
            {
                result = place_call(data, &p_call_state->call_id);

                if (result == WICED_BT_CALL_SUCCESS)
                {
                    p_call_state->call_flags = WICED_BT_GA_TBS_OUTGOING_CALL;
                    memcpy(p_call_state->URI, data.str, data.len);

                    p_call_state->URI[data.len] = '\0';
                    p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_DIALING;
                    p_call_state->in_use = WICED_TRUE;

                    //place the call using remote caller id, remote caller id has to be populated before calling place call
                    WICED_BT_TRACE("[%s] outgoing place call uri %s", __FUNCTION__, p_call_state->URI);
                    result = WICED_BT_CALL_SUCCESS;
                    break;
                }
                else
                {
                    return result;
                }
            }
        }

        //move all active calls to locally held state
        for (i = 0; i < num_calls; i++)
        {
            p_call_state = (tbs_call_state_data_t *)(p_call_list + i);

            if (p_call_state->in_use == WICED_FALSE)
            {
                if (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_ACTIVE)
                {
                    p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD;
                }
            }
        }
    }
    return result;
}

static wiced_bt_ga_tbs_call_operation_result_t lepl_tbs_join_call(tbs_call_state_data_t *p_call_list,
                                                           uint8_t num_calls,
                                                           wiced_bt_ga_tbs_join_call_data_t join_data)
{
    WICED_BT_TRACE("[%s] ", __FUNCTION__);
    wiced_bt_ga_tbs_call_operation_result_t result = WICED_BT_CALL_SUCCESS;
    tbs_call_state_data_t *p_call_state;
    uint8_t i = 0, j = 0;

    if (join_data.num_call_ids < 2) return WICED_BT_CALL_OPERATION_NOT_POSSIBLE;

    //check if all calls are valid
    for (i = 0; i < join_data.num_call_ids; i++)
    {
        p_call_state = get_call_state(join_data.call_ids[i], p_call_list, num_calls);
        if (p_call_state == NULL) return WICED_BT_CALL_INVALID_CALL_ID;
    }

    //check if any of the call state is incoming, if so return op not possible
    for (i = 0; i < num_calls; i++)
    {
        p_call_state = get_call_state(join_data.call_ids[i], p_call_list, num_calls);
        if (p_call_state != NULL && p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_INCOMING)
            return WICED_BT_CALL_OPERATION_NOT_POSSIBLE;
    }

    //move all the calls which is in active state but not in the join call list to locally held state
    for (i = 0; i < num_calls; i++)
    {
        p_call_state = (tbs_call_state_data_t *)(p_call_list + i);
        if (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_ACTIVE)
        {
            for (j = 0; j < join_data.num_call_ids; j++)
            {
                if (p_call_state->call_id == join_data.call_ids[j]) break;
            }

            if (j == join_data.num_call_ids)
            {
                result = change_call_state(p_call_state->call_id, WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD);
                if (result == WICED_BT_CALL_SUCCESS) p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD;
            }
        }
    }

    //for all the calls in the join list
    for (i = 0; i < join_data.num_call_ids; i++)
    {
        if (join_data.call_ids[i] == 0) // reserved for the call from the client
        {
            result = WICED_BT_CALL_INVALID_CALL_ID;
        }
        else
        {
            p_call_state = get_call_state(join_data.call_ids[i], p_call_list, num_calls);

            if (p_call_state != NULL)
            {
                if (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD)
                {
                    result = change_call_state(p_call_state->call_id, WICED_BT_GA_TBS_CALL_STATE_ACTIVE);
                    if (result == WICED_BT_CALL_SUCCESS)
                    {
                        p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_ACTIVE;
                    }
                }
                else if (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD)
                {
                    result = change_call_state(p_call_state->call_id, WICED_BT_GA_TBS_CALL_STATE_REMOTELY_HELD);
                    if (result == WICED_BT_CALL_SUCCESS)
                    {
                        p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_REMOTELY_HELD;
                    }
                }
            }
        }
    }
    return result;
}

/* actions performed by server */
static wiced_bt_ga_tbs_call_operation_result_t lepl_tbs_remotely_hold_call(uint8_t call_id,
                                                                    tbs_call_state_data_t *p_call_list,
                                                                    uint8_t num_calls)
{
    WICED_BT_TRACE("[%s] ", __FUNCTION__);

    wiced_bt_ga_tbs_call_operation_result_t result = WICED_BT_CALL_SUCCESS;
    tbs_call_state_data_t *p_call_state;

    p_call_state = get_call_state(call_id, p_call_list, num_calls);

    if (p_call_state != NULL)
    {
        if ((p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_ALERTING) ||
            (p_call_state->call_state == WICED_BT_GA_TBS_CALL_STATE_DIALING))
            return WICED_BT_CALL_STATE_MISMATCH;

        if (p_call_state->call_state != WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD)
            p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_REMOTELY_HELD;
        else
            p_call_state->call_state = WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD;
    }
    else
    {
        result = WICED_BT_CALL_INVALID_CALL_ID;
    }
    return result;
}

void lepl_tbs_set_incoming_remote_call(lepl_tbs_data_t *p_tbs, char *uri_scheme, char *friendly_name)

{
    uint8_t i;
    uint32_t prefix_len;

    if (p_tbs->num_calls >= WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT) return;

    if (p_tbs->current_call_id >= 255)
        p_tbs->current_call_id = 1;
    else
        p_tbs->current_call_id++;

    for (i = 0; i < WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT; i++)
    {
        if (p_tbs->call_state_data[i].in_use == WICED_FALSE) break;
    }

    if (i == WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT)
    {
        //no resourse for new call
        p_tbs->current_call_id--;
        return;
    }

    p_tbs->num_calls++;

    WICED_BT_TRACE("[%s] call_id value:%d  Call count:%d  i : %d",
                   __FUNCTION__,
                   p_tbs->current_call_id,
                   p_tbs->num_calls,
                   i);

    prefix_len = (uint8_t)strlen(uri_scheme);
    if (prefix_len > WICED_BT_GA_TBS_RM_CALLERID_MAX_SIZE) prefix_len = WICED_BT_GA_TBS_RM_CALLERID_MAX_SIZE;

    //------------------------updatecall URI---------------------------------
    memset(p_tbs->bearer_URI, 0, WICED_BT_GA_TBS_BEARER_URI_MAX_SIZE);
    memcpy(p_tbs->bearer_URI, uri_scheme, prefix_len);

    //------------------------update golbal TBS Data---------------------------------
    p_tbs->call_state_data[i].call_id = p_tbs->current_call_id;
    p_tbs->call_state_data[i].call_type = INCOMING_CALL; //1 = Call is an outgoingcall
    p_tbs->call_state_data[i].call_state = WICED_BT_GA_TBS_CALL_STATE_INCOMING;
    p_tbs->call_state_data[i].call_flags = 1; //inband ringtone is enabled
    p_tbs->call_state_data[i].in_use = WICED_TRUE;

    //--------------------------friendly name -------------------------------
    memset(p_tbs->call_friendly_name.name, 0, WICED_BT_GA_TBS_FRIENDLY_NAME_MAX_SIZE);
    strcpy(p_tbs->call_friendly_name.name, friendly_name);
    p_tbs->call_friendly_name.call_id = p_tbs->current_call_id;
    WICED_BT_TRACE("[%s] friendly name %s ", __FUNCTION__, p_tbs->call_friendly_name.name);

    //--------------------------Target caller ID -------------------------------
    strcpy(p_tbs->incoming_tg_caller_id, default_tg_caller_id);
    WICED_BT_TRACE("[%s] Tg caller ID %s ", __FUNCTION__, p_tbs->incoming_tg_caller_id);
}

static wiced_result_t tbs_handle_write_req_evt(uint16_t conn_id,
                                        const gatt_intf_service_object_t *p_service,
                                        gatt_intf_attribute_t *p_char,
                                        wiced_bt_ga_tbs_data_t *p_evt_data)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_bt_ga_tbs_call_action_t opcode;
    WICED_BT_TRACE("[%s] opcode %d \n", __FUNCTION__, p_evt_data->call_action.opcode);

    switch (p_char->characteristic_type)
    {
#if 0
        case TBS_BEARER_SIGNAL_STRENGTH_REPORTING_INTERVAL_CHARACTERISTIC:
            WICED_BT_TRACE("[%s] event interval val %x \n",
                           __FUNCTION__,
                           p_evt_data->bearer_signal_strength_reporting_interval);
            Send_TBS_set_bearer_signal_strength_reporting_interval(
                p_char,
                p_service,
                p_evt_data->bearer_signal_strength_reporting_interval);
break;
#endif
        case TBS_CALL_CONTROL_POINT_CHARACTERISTIC:
            opcode = p_evt_data->call_action.opcode;
            switch (opcode)
            {
                case WICED_BT_GA_CCP_ACTION_ACCEPT_CALL:
                result = lepl_tbs_accept_call(conn_id, p_evt_data->call_action.call_id,
                                                  telephone_bearer_data->call_state_data,
                                                  WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT,
                                                  WICED_FALSE);

                    break;
                case WICED_BT_GA_CCP_ACTION_TERMINATE_CALL:
                    result =
                        lepl_tbs_terminate_call(conn_id, p_evt_data->call_action.call_id,
                                                               &p_evt_data->call_termination_reason.termination_reason,
                                                               telephone_bearer_data->call_state_data,
                                                               WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT,
                                                               FALSE);
                    break;
                case WICED_BT_GA_CCP_ACTION_HOLD_CALL:
                    if (IS_HOLD_SUPPORTED(telephone_bearer_data->bearer_status_flag))
                        result = lepl_tbs_hold_call(p_evt_data->call_action.call_id,
                                                                   telephone_bearer_data->call_state_data,
                                                                   WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);
                    else
                        result = WICED_BT_CALL_OPCODE_NOT_SUPPORTED;
                    break;
                case WICED_BT_GA_CCP_ACTION_RETRIEVE_CALL:
                    result = lepl_tbs_retrieve_call(p_evt_data->call_action.call_id,
                                                                   telephone_bearer_data->call_state_data,
                                                                   WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);
                    break;
                case WICED_BT_GA_CCP_ACTION_ORIGINATE:
                    result = lepl_tbs_place_call(telephone_bearer_data->call_state_data,
                                                                WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT,
                                                                p_evt_data->call_action.place_call_uri);
                    p_evt_data->call_action.call_id = telephone_bearer_data->current_call_id;
                    break;
                case WICED_BT_GA_CCP_ACTION_JOIN_CALL:
                    if (IS_JOIN_SUPPORTED(telephone_bearer_data->bearer_status_flag))
                        result = lepl_tbs_join_call(telephone_bearer_data->call_state_data,
                                                                   WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT,
                                                                   p_evt_data->call_action.join_call_data);
                    else
                        result = WICED_BT_CALL_OPCODE_NOT_SUPPORTED;
                    break;
                default:
                    result = WICED_BT_CALL_OPCODE_NOT_SUPPORTED;
                    break;
            }
            if (result == WICED_SUCCESS)
            {

                lepl_tbs_data_t *p_tbs = &g_lepl_gatt_cb.tbs_data;
                gatt_intf_attribute_t characteristic = {0};
                wiced_bt_ga_tbs_data_t data = {0};

                get_call_states(&data.call_state_list,
                                p_tbs->call_state_data,
                                WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);

                characteristic.characteristic_type = TBS_CALL_STATE_CHARACTERISTIC;
                notify_tbs_characteristics(&characteristic, &data);
            }
            else
            {
                WICED_BT_TRACE_CRIT("[%s] res %d", __FUNCTION__, result);
            }
            break;
    }
    return result;
}

wiced_result_t tbs_handle_read_req_evt(uint16_t conn_id,
                                       const gatt_intf_service_object_t *p_service,
                                       gatt_intf_attribute_t *p_char,
                                       wiced_bt_ga_tbs_data_t *p_evt_data)
{
    wiced_result_t result = WICED_SUCCESS;

    if (p_char->included_service_type != INCLUDED_SERVICE_NONE) return WICED_ERROR;

    switch (p_char->characteristic_type)
    {
        case TBS_BEARER_PROVIDER_NAME_CHARACTERISTIC:
            p_evt_data->bearer_provider_name.len = strlen(telephone_bearer_data->bearer_provider_name);
            p_evt_data->bearer_provider_name.str = telephone_bearer_data->bearer_provider_name;
            break;
        case TBS_BEARER_UCI_CHARACTERISTIC:
            p_evt_data->bearer_UCI.len = strlen(telephone_bearer_data->bearer_UCI);
            p_evt_data->bearer_UCI.str = telephone_bearer_data->bearer_UCI;
            break;
        case TBS_BEARER_URI_SUPPORTED_SCHEMES_CHARACTERISTIC:
            p_evt_data->bearer_URI_supported_schemes_list.len = strlen(telephone_bearer_data->bearer_URI);
            p_evt_data->bearer_URI_supported_schemes_list.str = telephone_bearer_data->bearer_URI;
            break;
        case TBS_BEARER_TECHNOLOGY_CHARACTERISTIC:
            p_evt_data->bearer_technology = telephone_bearer_data->bearer_technology;
            break;
        case TBS_BEARER_SIGNAL_STRENGTH_CHARACTERISTIC:
            p_evt_data->bearer_signal_strength = telephone_bearer_data->prev_bearer_signal_strength;
            break;
        case TBS_BEARER_SIGNAL_STRENGTH_REPORTING_INTERVAL_CHARACTERISTIC:
            p_evt_data->bearer_signal_strength_reporting_interval =
                telephone_bearer_data->bearer_signal_strength_reporting_interval;
            break;
        case TBS_STATUS_FLAGS_CHARACTERISTIC:
            p_evt_data->status_flag = telephone_bearer_data->bearer_status_flag;
            break;
        case TBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE_CHARACTERISTIC:
            p_evt_data->ccp_supported_opcode = telephone_bearer_data->ccp_supported_opcode;
            break;
        case TBS_CONTENT_CONTROL_ID_CHARACTERISTIC:
            p_evt_data->content_control_id = telephone_bearer_data->content_control_id;
            break;
        case TBS_INCOMING_CALL_TG_BEARER_URI_CHARACTERISTIC: {
            tbs_call_state_data_t *p_state = get_call_state(telephone_bearer_data->latest_incoming_remote_call_id,
                                                            telephone_bearer_data->call_state_data,
                                                            WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);
            if (p_state != NULL)
            {
                p_evt_data->incoming_call.call_id = p_state->call_id;
                p_evt_data->incoming_call.URI.str = p_state->URI;
                p_evt_data->incoming_call.URI.len = strlen(p_state->URI);
            }
        }
        break;
        case TBS_CALL_FRIENDLY_NAME_CHARACTERISTIC:
            p_evt_data->call_friendly_name.call_id = telephone_bearer_data->call_friendly_name.call_id;
            p_evt_data->call_friendly_name.name.len = strlen(telephone_bearer_data->call_friendly_name.name);
            p_evt_data->call_friendly_name.name.str = telephone_bearer_data->call_friendly_name.name;
            break;
        case TBS_BEARER_LIST_CURRENT_CALLS_CHARACTERISTIC:
            memset(&p_evt_data->current_call_list, 0 ,sizeof(wiced_bt_ga_tbs_current_call_list_t));
            get_call_list(&p_evt_data->current_call_list,
                          telephone_bearer_data->call_state_data,
                          WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);
            break;
        case TBS_CALL_STATE_CHARACTERISTIC:
            memset(&p_evt_data->call_state_list, 0, sizeof(wiced_bt_ga_tbs_call_state_list_t));
            get_call_states(&p_evt_data->call_state_list,
                            telephone_bearer_data->call_state_data,
                            WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);
            break;
    }

    return result;
}

wiced_result_t lepl_tbs_callback(uint16_t conn_id,
                                 void *p_app_ctx,
                                 gatt_intf_service_object_t *p_service,
                                 wiced_bt_gatt_status_t status,
                                 uint32_t evt_type,
                                 gatt_intf_attribute_t *p_char,
                                 void *p_data,
                                 int len)
{
    wiced_result_t result = WICED_SUCCESS;

    WICED_BT_TRACE("[%s] event 0x%x \n", __FUNCTION__, evt_type);

    switch (evt_type)
    {
        case WRITE_REQ_EVT:
            result = tbs_handle_write_req_evt(conn_id, p_service, p_char, p_data);
            break;

        case READ_REQ_EVT:
            result = tbs_handle_read_req_evt(conn_id, p_service, p_char, p_data);
            break;

        default:
            break;
    }
    return result;
}

wiced_result_t lepl_ccs_set_incoming_remote_call(uint16_t conn_id)
{
    wiced_bt_ga_tbs_data_t data = {0};
    wiced_bt_ga_tbs_data_t *p_data = &data;
    lepl_tbs_data_t *p_tbs = &g_lepl_gatt_cb.tbs_data;
    gatt_intf_attribute_t characteristic = {0};

    WICED_BT_TRACE("[%s] call_id value:%d  Call count:%d ", __FUNCTION__, p_tbs->current_call_id, p_tbs->num_calls);

    // we need friendly name update in the incoming call notification, hence sending this first
    p_data->call_friendly_name.name.str = p_tbs->call_friendly_name.name;
    p_data->call_friendly_name.name.len = strlen(p_tbs->call_friendly_name.name);
    p_data->call_friendly_name.call_id = p_tbs->call_friendly_name.call_id;

    WICED_BT_TRACE("[%s] friendly name %s ", __FUNCTION__, p_data->call_friendly_name.name);

    characteristic.characteristic_type = TBS_CALL_FRIENDLY_NAME_CHARACTERISTIC;
    notify_tbs_characteristics(&characteristic, p_data);
    //---------------------------------------------------------

    p_data->bearer_URI_supported_schemes_list.str = p_tbs->bearer_URI;
    p_data->bearer_URI_supported_schemes_list.len = strlen(p_tbs->bearer_URI);

    p_data->incoming_call.call_id = p_tbs->current_call_id;
    p_data->incoming_call.URI.str = p_tbs->bearer_URI;
    p_data->incoming_call.URI.len = strlen(p_tbs->bearer_URI);

    WICED_BT_TRACE("[%s] call_id value %d p_data->incoming_call.URI.str %s ",
                   __FUNCTION__,
                   p_data->incoming_call.call_id,
                   p_data->incoming_call.URI.str);

    characteristic.characteristic_type = TBS_INCOMING_CALL_CHARACTERISTIC;

    notify_tbs_characteristics(&characteristic, p_data);
    //---------------------------------------------------------

    p_data->incoming_tg_caller_id.call_id = p_tbs->current_call_id;
    p_data->incoming_tg_caller_id.target_caller_id.str = p_tbs->incoming_tg_caller_id;
    p_data->incoming_tg_caller_id.target_caller_id.len = strlen(p_tbs->incoming_tg_caller_id);

    WICED_BT_TRACE("[%s] Tg caller ID %s ", __FUNCTION__, p_data->incoming_tg_caller_id.target_caller_id);
    characteristic.characteristic_type = TBS_INCOMING_CALL_TG_BEARER_URI_CHARACTERISTIC;
    notify_tbs_characteristics(&characteristic, p_data);
    //---------------------------------------------------------

    get_call_states(&p_data->call_state_list, p_tbs->call_state_data, WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);

    characteristic.characteristic_type = TBS_CALL_STATE_CHARACTERISTIC;
    notify_tbs_characteristics(&characteristic, p_data);
    //---------------------------------------------------------

    get_call_list(&p_data->current_call_list, p_tbs->call_state_data, WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);

    characteristic.characteristic_type = TBS_BEARER_LIST_CURRENT_CALLS_CHARACTERISTIC;
    notify_tbs_characteristics(&characteristic, p_data);

    //---------------------------------------------------------
    #if 1
    if (lepl_ccs_pacs_does_peer_support_ringtone(conn_id))
    {
        //start_inband_ringtone
        lepl_ccs_start_inband_ringtone(conn_id);
    }
    #else
	    lepl_ccs_start_streaming_convo(conn_id);
    #endif

    wiced_bt_ga_string_t call_URI = {.str = p_tbs->bearer_URI, .len = strlen(p_tbs->bearer_URI)};
    le_audio_rpc_update_call_state(conn_id,
                                   p_tbs->current_call_id,
                                   &call_URI,
                                   WICED_BT_GA_TBS_CALL_STATE_DIALING);
    return WICED_SUCCESS;
}

void lepl_ccs_set_remote_hold_call(uint8_t call_id)
{
    wiced_bt_ga_tbs_call_operation_result_t result;
    wiced_bt_ga_tbs_data_t data = {0};
    wiced_bt_ga_tbs_data_t *p_data = &data;
    lepl_tbs_data_t *p_tbs = &g_lepl_gatt_cb.tbs_data;
    gatt_intf_attribute_t characteristic = {0};

    //update state
    result = lepl_tbs_remotely_hold_call(call_id,
                                         p_tbs->call_state_data,
                                         WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);

    if (result == WICED_BT_CALL_SUCCESS)
    {
        WICED_BT_TRACE("[%s] result %d call_id %d call_state %d\n", __FUNCTION__, result, call_id,
                                                                   telephone_bearer_data->call_state_data->call_state);
        get_call_states(&p_data->call_state_list, p_tbs->call_state_data, WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);

        characteristic.characteristic_type = TBS_CALL_STATE_CHARACTERISTIC;
        notify_tbs_characteristics(&characteristic, p_data);
    }
    else
        WICED_BT_TRACE("[%s] result %d Call state not updated for call_id %d\n", __FUNCTION__, result, call_id);
}

void lepl_ccs_set_retrieve_remote_hold_call(uint8_t call_id)
{
    wiced_bt_ga_tbs_call_operation_result_t result;
    wiced_bt_ga_tbs_data_t data = {0};
    wiced_bt_ga_tbs_data_t *p_data = &data;
    lepl_tbs_data_t *p_tbs = &g_lepl_gatt_cb.tbs_data;
    gatt_intf_attribute_t characteristic = {0};

    //update state
    result = lepl_tbs_retrieve_remotely_hold_call(call_id,
                                                  p_tbs->call_state_data,
                                                  WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);

    if (result == WICED_BT_CALL_SUCCESS)
    {
        WICED_BT_TRACE("[%s] Remote call rertieve Result %d call_id %d call_state %d\n",
                       __FUNCTION__,
                       result,
                       call_id,
                       telephone_bearer_data->call_state_data->call_state);
        get_call_states(&p_data->call_state_list, p_tbs->call_state_data, WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);

        characteristic.characteristic_type = TBS_CALL_STATE_CHARACTERISTIC;
        notify_tbs_characteristics(&characteristic, p_data);
    }
    else
        WICED_BT_TRACE("[%s] result %d Call state not updated for call_id %d\n", __FUNCTION__, result, call_id);
}

wiced_result_t lepl_ccs_terminate_call(uint16_t conn_id, uint8_t call_id, uint8_t termination_reason)
{
    tbs_call_state_data_t *p_call_state;
    lepl_tbs_data_t *p_tbs = &g_lepl_gatt_cb.tbs_data;
    gatt_intf_attribute_t characteristic = {0};
    wiced_bt_ga_tbs_data_t data = {0};
    wiced_bt_ga_tbs_data_t *p_data = &data;

    p_call_state = get_call_state(call_id, p_tbs->call_state_data, WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);
    if (!p_call_state)
    {
        return WICED_BT_CALL_INVALID_CALL_ID;
    }

    lepl_ccs_states_t call_state = lepl_get_call_control_server_state();
    WICED_BT_TRACE("[%s] state : %d\n", __FUNCTION__, call_state);

    if (call_state == CALL_CONTROL_SERVER_STATE_INBAND_RINGTONE)
    {
        lepl_cap_stop_media_streaming(conn_id);
    }
    else
    {
        lepl_cap_stop_conv_streaming(conn_id);
    }
    lepl_set_call_control_server_state(CALL_CONTROL_SERVER_STATE_IDLE);

    if (p_call_state != NULL)
    {
        //clear data
        p_call_state->in_use = 0;
    }

    get_call_states(&p_data->call_state_list, p_tbs->call_state_data, WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);

    characteristic.characteristic_type = TBS_CALL_STATE_CHARACTERISTIC;
    notify_tbs_characteristics(&characteristic, p_data);

    get_call_list(&p_data->current_call_list, p_tbs->call_state_data, WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);

    characteristic.characteristic_type = TBS_BEARER_LIST_CURRENT_CALLS_CHARACTERISTIC;
    notify_tbs_characteristics(&characteristic, p_data);

    return WICED_BT_CALL_SUCCESS;
}

void lepl_tbs_initialize_data()
{
    WICED_BT_TRACE("[%s] \n", __FUNCTION__);

    //set initial default values
    memset(&g_lepl_gatt_cb.tbs_data, 0, sizeof(lepl_tbs_data_t));
    telephone_bearer_data->content_control_id = 2;
    strcpy(telephone_bearer_data->bearer_provider_name, default_bearer_provider_name);
    strcpy(telephone_bearer_data->bearer_UCI, default_bearer_uci);
    strcpy(telephone_bearer_data->bearer_URI, default_bearer_uri);
    telephone_bearer_data->bearer_signal_strength = 5;
    telephone_bearer_data->bearer_signal_strength_reporting_interval = 0;
    telephone_bearer_data->prev_bearer_signal_strength = 0;
    telephone_bearer_data->bearer_technology = WICED_BT_GA_TBS_3G_TECHNOLOGY;
    telephone_bearer_data->bearer_status_flag =
        WICED_BT_GA_TBS_FEATURE_BIT_INBAND_RINGTONE | WICED_BT_GA_TBS_FEATURE_BIT_SILENT_MODE;
    telephone_bearer_data->ccp_supported_opcode =
        WICED_BT_GA_TBS_FEATURE_BIT_LOCAL_HOLD | WICED_BT_GA_TBS_FEATURE_BIT_JOIN; //bit 0 and 1 set to 1
    strcpy(telephone_bearer_data->incoming_tg_caller_id, default_tg_caller_id);
    memset(telephone_bearer_data->call_friendly_name.name, 0, sizeof(telephone_bearer_data->call_friendly_name.name));
}

void lepl_ccs_start_streaming_convo(uint16_t conn_id)
{
    lepl_stream_config_t ccs_stream_config = {0};
    lepl_device_config_t config1[2] = {0};
    wiced_bt_ga_bap_stream_config_t stream_config;
    ccs_stream_config.num_devices = 0;

    wiced_bt_ga_bap_get_unicast_stream_config(CODEC_CONFIG, &stream_config);

    WICED_BT_TRACE("[%s] conn id %x\n", __FUNCTION__, conn_id);
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);

     if (p_clcb->peer_profiles.p_csis &&
        !lepl_if_sirk_zero(p_clcb->csis_data.sirk_data.sirk))
     {
         for (int i = 0; i < MAX_CONNECTION_INSTANCE; i++)
         {
             if (lepl_csis_device_belongs_to_coordinated_set(g_lepl_gatt_cb.unicast_clcb[i].conn_id,
                                                             p_clcb->csis_data.sirk_data.sirk))
             {
                 config1[ccs_stream_config.num_devices].conn_id = g_lepl_gatt_cb.unicast_clcb[i].conn_id;
                 ccs_stream_config.num_devices++;
             }
         }
     }
     else
     {
         config1[ccs_stream_config.num_devices].conn_id = conn_id;
         ccs_stream_config.num_devices = 1;
     }

    ccs_stream_config.ctx_type = BAP_CONTEXT_TYPE_CONVERSATIONAL;
    ccs_stream_config.config_list = config1;
    ccs_stream_config.stream_config = &stream_config;

    lepl_set_call_control_server_state(CALL_CONTROL_SERVER_STATE_INBAND_RINGTONE_CONVO);

    lepl_cap_start_conv_streaming(&ccs_stream_config);
}

void lepl_ccs_start_streaming(uint16_t conn_id)
{
    lepl_stream_config_t ccs_stream_config = {0};
    lepl_device_config_t config1[2] = {0};
    wiced_bt_ga_bap_stream_config_t stream_config;
    ccs_stream_config.num_devices = 0;

    wiced_bt_ga_bap_get_unicast_stream_config(CODEC_CONFIG, &stream_config);

    WICED_BT_TRACE("call_control_server_start_streaming %x\n", conn_id);
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);

    if (p_clcb->peer_profiles.p_csis &&
        !lepl_if_sirk_zero(p_clcb->csis_data.sirk_data.sirk))
    {
        for (int i = 0; i < MAX_CONNECTION_INSTANCE; i++)
        {
            if (lepl_csis_device_belongs_to_coordinated_set(g_lepl_gatt_cb.unicast_clcb[i].conn_id,
                                                            p_clcb->csis_data.sirk_data.sirk))
            {
                config1[ccs_stream_config.num_devices].conn_id = g_lepl_gatt_cb.unicast_clcb[i].conn_id;
                ccs_stream_config.num_devices++;
            }
        }
    }
    else
    {
        config1[ccs_stream_config.num_devices].conn_id = conn_id;
        ccs_stream_config.num_devices = 1;
    }

    ccs_stream_config.ctx_type = BAP_CONTEXT_TYPE_CONVERSATIONAL;
    ccs_stream_config.config_list = config1;
    ccs_stream_config.stream_config = &stream_config;

    lepl_set_call_control_server_state(CALL_CONTROL_SERVER_STATE_CONVO);
    lepl_cap_start_conv_streaming(&ccs_stream_config);
}


void lepl_ccs_start_inband_ringtone(uint16_t conn_id)
{
    lepl_stream_config_t ccs_stream_config = {0};
    lepl_device_config_t config1[2] = {0};
    wiced_bt_ga_bap_stream_config_t stream_config;
    ccs_stream_config.num_devices = 0;

    wiced_bt_ga_bap_get_unicast_stream_config(CODEC_CONFIG, &stream_config);

    WICED_BT_TRACE("call_control_server_start_inband_ringtone %x\n", conn_id);

    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);

    if (p_clcb->peer_profiles.p_csis &&
        !lepl_if_sirk_zero(p_clcb->csis_data.sirk_data.sirk))
    {
        for (int i = 0; i < MAX_CONNECTION_INSTANCE; i++)
        {
            if (lepl_csis_device_belongs_to_coordinated_set(g_lepl_gatt_cb.unicast_clcb[i].conn_id,
                                                            p_clcb->csis_data.sirk_data.sirk))
            {
                config1[ccs_stream_config.num_devices].conn_id = g_lepl_gatt_cb.unicast_clcb[i].conn_id;
                ccs_stream_config.num_devices++;
            }
        }
    }
    else
    {
        config1[ccs_stream_config.num_devices].conn_id = conn_id;
        ccs_stream_config.num_devices = 1;
    }

    ccs_stream_config.ctx_type = BAP_CONTEXT_TYPE_RINGTONE;
    ccs_stream_config.config_list = config1;
    ccs_stream_config.stream_config = &stream_config;

    lepl_set_call_control_server_state(CALL_CONTROL_SERVER_STATE_INBAND_RINGTONE);
    lepl_cap_start_media_streaming(&ccs_stream_config);
}

uint8_t lepl_ccs_get_active_call_id(void)
{
    uint8_t num_calls;
    wiced_bt_ga_tbs_data_t tbs_data = {0};

    get_call_states(&tbs_data.call_state_list,
                    telephone_bearer_data->call_state_data,
                    WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT);

    num_calls = tbs_data.call_state_list.num_calls;

    if (num_calls == 0)
    {
        return 0;
    }
    for (uint8_t i=0; i < num_calls; i++)
    {
        if(tbs_data.call_state_list.call_state[i].call_state == WICED_BT_GA_TBS_CALL_STATE_ACTIVE)
        {
            return tbs_data.call_state_list.call_state[i].call_id;
        }
    }

    return 0;
}
