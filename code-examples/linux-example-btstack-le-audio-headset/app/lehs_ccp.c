/*
 * $ Copyright Cypress Semiconductor $
 */

#include "lehs.h"
#include "le_audio_rpc.h"
#include "wiced_bt_ga_tbs.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_tbs.h"

#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"

void lehs_rpc_update_call_frindly_name(uint16_t conn_id, char *friendly_name);

void lehs_handle_call_control_point_action(uint16_t conn_id,
                                           uint8_t call_id,
                                           gatt_intf_service_object_t *p_service,
                                           gatt_intf_attribute_t *p_char , uint8_t opcode)
{
    wiced_bt_ga_tbs_data_t data;
    wiced_bt_ga_tbs_data_t *p_data = &data;

    p_data->call_action.opcode = opcode;
    p_data->call_action.call_id = call_id;

    wiced_bt_gatt_status_t status = gatt_interface_write_characteristic(conn_id, p_service, p_char, p_data);
    WICED_BT_TRACE("[%s] status %d \n", __FUNCTION__, status);
}

void lehs_terminate_incoming_call(uint16_t conn_id,
                                                 uint8_t call_id,
                                                 uint8_t reason,
                                                 gatt_intf_service_object_t *p_service,
                                                 gatt_intf_attribute_t *p_char)
{
    wiced_bt_ga_tbs_data_t data;
    wiced_bt_ga_tbs_data_t *p_data = &data;

    p_data->call_action.opcode = WICED_BT_GA_CCP_ACTION_TERMINATE_CALL;
    p_data->call_action.termination_data.call_id = call_id;
    p_data->call_action.termination_data.termination_reason = reason;

    wiced_bt_gatt_status_t status = gatt_interface_write_characteristic(conn_id, p_service, p_char, p_data);
    WICED_BT_TRACE("[%s] status %d \n", __FUNCTION__, status);
}

wiced_result_t lehs_handle_tbs_data(uint16_t conn_id,
                                                   void *p_app_ctx,
                                                   const gatt_intf_service_object_t *p_service,
                                                   wiced_bt_gatt_status_t status,
                                                   uint32_t evt_type,
                                                   gatt_intf_attribute_t *p_char,
                                                   wiced_bt_ga_tbs_data_t *p_data_ptr,
                                                   int len)
{
    wiced_bt_ga_tbs_data_t *p_event_data = (wiced_bt_ga_tbs_data_t *)p_data_ptr;

    if (evt_type == WRITE_CMPL_EVT)
    {
        //update data to Client control

        return WICED_BT_SUCCESS;
    }

    WICED_BT_TRACE("[%s] event %x status %x \n", __FUNCTION__, p_char, status);

    if (status == WICED_BT_GATT_SUCCESS)
    {
        switch (p_char->included_service_type)
        {
            case INCLUDED_SERVICE_NONE: {
                switch (p_char->characteristic_type)
                {
                    case TBS_BEARER_PROVIDER_NAME_CHARACTERISTIC:
                        WICED_BT_TRACE("[%s] event %d bearer provider name:[%s]",
                                       __FUNCTION__,
                                       p_char,
                                       p_event_data->bearer_provider_name);
                        break;
                    case TBS_BEARER_UCI_CHARACTERISTIC:
                        WICED_BT_TRACE("[%s] event_id %d bearer UCI:[%s]",
                                       __FUNCTION__,
                                       p_char,
                                       p_event_data->bearer_UCI);
                        break;
                    case TBS_BEARER_TECHNOLOGY_CHARACTERISTIC:
                        WICED_BT_TRACE("[%s] event_id %d bearer technology:%d \n",
                                       __FUNCTION__,
                                       p_char,
                                       p_event_data->bearer_technology);
                        break;
                    case TBS_BEARER_URI_SUPPORTED_SCHEMES_CHARACTERISTIC:
                        WICED_BT_TRACE("[%s] event_id %d bearer URI:%d \n",
                                       __FUNCTION__,
                                       p_char,
                                       p_event_data->bearer_URI_supported_schemes_list.str);
                        break;
                    case TBS_BEARER_SIGNAL_STRENGTH_CHARACTERISTIC:
                        WICED_BT_TRACE("[%s] event_id %d bearer signal strength %d  \n",
                                       __FUNCTION__,
                                       p_char,
                                       p_event_data->bearer_signal_strength);
                        break;
                    case TBS_BEARER_SIGNAL_STRENGTH_REPORTING_INTERVAL_CHARACTERISTIC:
                        WICED_BT_TRACE("[%s] event_id %d bearer signal strength reporting interval %d  \n",
                                       __FUNCTION__,
                                       p_char,
                                       p_event_data->bearer_signal_strength_reporting_interval);
                        break;
                    case TBS_BEARER_LIST_CURRENT_CALLS_CHARACTERISTIC: {
                        int i;
                        WICED_BT_TRACE("[%s] ---- current call list begin----\n", __FUNCTION__);
                        for (i = 0; i < p_event_data->current_call_list.num_calls; i++)
                            WICED_BT_TRACE(
                                "[%s] event_id %d call id %d, call state %d, call flags %d, remote caller id %s\n",
                                __FUNCTION__,
                                p_char,
                                p_event_data->current_call_list.call_data[i].call_id,
                                p_event_data->current_call_list.call_data[i].call_state,
                                p_event_data->current_call_list.call_data[i].call_flags,
                                p_event_data->current_call_list.call_data[i].remote_caller_id.str);
                        WICED_BT_TRACE("[%s] ---- current call list end ----\n", __FUNCTION__);
                        return WICED_SUCCESS;
                    }
                    case TBS_CONTENT_CONTROL_ID_CHARACTERISTIC:
                        WICED_BT_TRACE("[%s] event_id %d content control id %x \n",
                                       __FUNCTION__,
                                       p_char,
                                       p_event_data->content_control_id);
                        break;
                    case TBS_INCOMING_CALL_TG_BEARER_URI_CHARACTERISTIC: {
                        WICED_BT_TRACE("[%s]  incoming call target caller id \n", __FUNCTION__);
                        if (p_event_data->incoming_tg_caller_id.target_caller_id.len == 0) return WICED_SUCCESS;
                        WICED_BT_TRACE("[%s] event_id %d caller id: %d  bearer incoming target call id: %s\n",
                                       __FUNCTION__,
                                       p_char,
                                       p_event_data->incoming_tg_caller_id.call_id,
                                       p_event_data->incoming_tg_caller_id.target_caller_id);
                        break;
                    }
                    case TBS_STATUS_FLAGS_CHARACTERISTIC:
                        WICED_BT_TRACE("[%s] event_id %d bearer status_flags %d \n",
                                       __FUNCTION__,
                                       p_char,
                                       p_event_data->status_flag);
                        break;
                    case TBS_CALL_STATE_CHARACTERISTIC: {
                        WICED_BT_TRACE("[%s] ---- call state begin----\n", __FUNCTION__);
                        for (int i = 0; i < p_event_data->call_state_list.num_calls; i++)
                        {
                            WICED_BT_TRACE("[%s] event_id %d call state change id %d, state %d , call_flags %d \n",
                                           __FUNCTION__,
                                           p_char,
                                           p_event_data->call_state_list.call_state[i].call_id,
                                           p_event_data->call_state_list.call_state[i].call_state,
                                           p_event_data->call_state_list.call_state[i].call_flags);
                            WICED_BT_TRACE("[%s] ---- call state end----\n", __FUNCTION__);

                            if (p_event_data->call_state_list.call_state[i].call_state !=
                                WICED_BT_GA_TBS_CALL_STATE_INCOMING)
                            {
                                WICED_BT_TRACE("[%s] Sending call state event to client control\n", __FUNCTION__);
                                le_audio_rpc_update_call_state(0x8000,
                                                           p_event_data->call_state_list.call_state[i].call_id,
                                                           NULL,
                                                           p_event_data->call_state_list.call_state[i].call_state);
                            }
                        }
                    }
                    break;
                    case TBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE_CHARACTERISTIC:
                        WICED_BT_TRACE("[%s] event_id %d control point supported opcode %d \n",
                                       __FUNCTION__,
                                       p_char,
                                       p_event_data->ccp_supported_opcode);
                        break;
                    case TBS_CALL_TERMINATION_REASON_CHARACTERISTIC:
                        WICED_BT_TRACE("[%s] event_id %d call id %d bearer termination reason %d \n",
                                       __FUNCTION__,
                                       p_char,
                                       p_event_data->call_termination_reason.call_id,
                                       p_event_data->call_termination_reason.termination_reason);
                        le_audio_rpc_send_call_terminated_event(conn_id,
                                                                p_event_data->call_termination_reason.call_id,
                                                                p_event_data->call_termination_reason.termination_reason);
                        break;
                    case TBS_INCOMING_CALL_CHARACTERISTIC:
                        if (p_event_data->incoming_call.URI.len == 0) return WICED_BT_ERROR;

                        WICED_BT_TRACE("[%s] event_id %d bearer incoming caller id  %d uri %s \n",
                                       __FUNCTION__,
                                       p_char,
                                       p_event_data->incoming_call.call_id,
                                       p_event_data->incoming_call.URI);

                        le_audio_rpc_update_call_state(conn_id,
                                                   p_event_data->incoming_call.call_id,
                                                   &p_event_data->incoming_call.URI,
                                                   WICED_BT_GA_TBS_CALL_STATE_INCOMING);
                        break;
                    case TBS_CALL_FRIENDLY_NAME_CHARACTERISTIC: {
                        WICED_BT_TRACE("[%s] event_id %d bearer call friendly name id %d name %s\n",
                                       __FUNCTION__,
                                       p_char,
                                       p_event_data->call_friendly_name.call_id,
                                       p_event_data->call_friendly_name.name);
                        lehs_rpc_update_call_frindly_name(conn_id, p_event_data->call_friendly_name.name.str);
                        break;
                    }
                }
            }
            break;
        }
    }
    return WICED_BT_SUCCESS;
}

wiced_result_t lehs_ccp_callback(uint16_t conn_id,
                                                void *p_app_ctx,
                                                gatt_intf_service_object_t *p_service,
                                                wiced_bt_gatt_status_t status,
                                                uint32_t evt_type,
                                                gatt_intf_attribute_t *p_char,
                                                void *p_data,
                                                int len)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;
    WICED_BT_TRACE("[%s] event 0x%x \n", __FUNCTION__, p_char);

    switch (evt_type)
    {
        case READ_CMPL_EVT:
        case WRITE_CMPL_EVT:
        case NOTIFICATION_EVT:
            result = lehs_handle_tbs_data(conn_id,
                                        p_app_ctx,
                                        p_service,
                                        status,
                                        evt_type,
                                        p_char,
                                        (wiced_bt_ga_tbs_data_t *)p_data,
                                        len);
            break;
    }
        return result;
}
