/*
 * $ Copyright Cypress Semiconductor $
 */
#include "lepl.h"

wiced_result_t lepl_vcs_set_volume(uint16_t conn_id, volume_control_opcodes_t opcode, uint8_t abs_vol)
{
    WICED_BT_TRACE("[%s] opcode %d vol %d\n", __FUNCTION__, opcode, abs_vol);
    lepl_cap_get_coordinated_set_members(conn_id);
    return le_audio_cap_set_volume(&g_lepl_gatt_cb.cap_profile_data, opcode, abs_vol);
}

wiced_result_t lepl_vcs_set_mute_state(uint16_t conn_id, wiced_bt_ga_mute_val_t mute_state)
{
    WICED_BT_TRACE("[%s] mute state %d \n", __FUNCTION__, mute_state);
    lepl_cap_get_coordinated_set_members(conn_id);
    return le_audio_cap_set_volume_mute_state(&g_lepl_gatt_cb.cap_profile_data, mute_state);
}

wiced_result_t lepl_handle_vcs_data(uint16_t conn_id,
                                              void *p_app_ctx,
                                              const gatt_intf_service_object_t *p_volume,
                                              wiced_bt_gatt_status_t status,
                                              uint32_t evt_type,
                                              gatt_intf_attribute_t *p_char,
                                              wiced_bt_ga_vcs_data_t *p_data_ptr,
                                              int len)
{
    wiced_bt_ga_vcs_data_t *p_event_data = (wiced_bt_ga_vcs_data_t *)p_data_ptr;

    WICED_BT_TRACE("[%s] event %x status %x \n", __FUNCTION__, evt_type, status);

    if (evt_type == WRITE_CMPL_EVT) {
        return WICED_BT_SUCCESS;
    }

    if (status == WICED_BT_GATT_SUCCESS)
    {
        switch (p_char->included_service_type)
        {
        case INCLUDED_SERVICE_NONE:
        {
            switch (p_char->characteristic_type)
            {
            case VCS_VOLUME_STATE_CHARACTERISTIC:
                    WICED_BT_TRACE("[%s] event %d volume_setting:%x mute_state:%x \n",
                                   __FUNCTION__,
                                   p_char,
                                   p_event_data->control_point_data.volume_state.volume_setting,
                                   p_event_data->control_point_data.volume_state.mute_state);
                le_audio_rpc_send_vcs_state_update(conn_id,
                                                   p_event_data->control_point_data.volume_state.volume_setting,
                                                   p_event_data->control_point_data.volume_state.mute_state,
                                                   HCI_CONTROL_LEA_MUTE_AND_VOLUME_STATUS);
                break;
            case VCS_VOLUME_FLAG_CHARACTERISTIC:
                WICED_BT_TRACE("[%s] event %d volume_flag:%x \n", __FUNCTION__, p_char, p_event_data->volume_flag);
                break;
            case VCS_CONTROL_POINT_CHARACTERISTIC:
                WICED_BT_TRACE("[%s] event %d write_complete:%x \n", __FUNCTION__, p_char, status);
                break;
            }
        }break;
        }
    }
    return WICED_BT_SUCCESS;
}

wiced_result_t lepl_vcs_callback(uint16_t conn_id,
                                           void *p_app_ctx,
                                           gatt_intf_service_object_t *p_service,
                                           wiced_bt_gatt_status_t status,
                                           uint32_t evt_type,
                                           gatt_intf_attribute_t *p_char,
                                           void *p_data,
                                           int len)
{
    WICED_BT_TRACE("[%s] event 0x%x \n", __FUNCTION__, p_char);

    switch (evt_type) {
        case READ_CMPL_EVT:
        case WRITE_CMPL_EVT:
        case NOTIFICATION_EVT:
            return lepl_handle_vcs_data(conn_id,
                                                  p_app_ctx,
                                                  p_service,
                                                  status,
                                                  evt_type,
                                                  p_char,
                                                  (wiced_bt_ga_vcs_data_t *)p_data,
                                                  len);
            break;
    }
    return WICED_BT_SUCCESS;
}
