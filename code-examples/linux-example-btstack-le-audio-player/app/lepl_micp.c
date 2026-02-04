/*
 * $ Copyright Cypress Semiconductor $
 */

#include "lepl.h"

#define AICS_AUDIO_INPUT_DESCRIPTION "LEPL MICP AICS"

enum
{
    MICP_AICS_GAIN_INCREMENT = 1,
    MICP_AICS_GAIN_DECREMENT,
    MICP_AICS_SET_GAIN,
}micp_aisc_gain_op_t;

void lepl_micp_mute(uint16_t conn_id, uint8_t mute)
{
    WICED_BT_TRACE("[%s] mute state %d", __FUNCTION__, mute);
    lepl_cap_get_coordinated_set_members(conn_id);
    le_audio_cap_set_mics_mute_state(&g_lepl_gatt_cb.cap_profile_data, mute);
}

void lepl_micp_aics_mute(uint16_t conn_id, uint32_t instance, uint8_t mute)
{
    WICED_BT_TRACE("[%s] mute state %d", __FUNCTION__, mute);
    lepl_cap_get_coordinated_set_members(conn_id);
    le_audio_cap_set_mics_aics_mute_state(&g_lepl_gatt_cb.cap_profile_data, instance, mute);
}

void lepl_micp_aics_set_gain(uint16_t conn_id, uint32_t instance, uint8_t opcode, int8_t input_gain)
{
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    if (!p_clcb) return;
    lepl_aics_data_t *p_aics = &p_clcb->mics_data.aics[instance];

    if (opcode == MICP_AICS_GAIN_INCREMENT)
    {
        int gain = p_aics->input_state.gain_setting + (p_aics->gain_setting.gain_setting_units);
        input_gain = (gain > 127) ? 127 : (int8_t)gain;
    }
    else if (opcode == MICP_AICS_GAIN_DECREMENT)
    {
        int gain = p_aics->input_state.gain_setting - (p_aics->gain_setting.gain_setting_units);
        input_gain = (gain < -128) ? -128 : (int8_t)gain;
    }
    if (input_gain > p_aics->gain_setting.max_gain_setting) input_gain = p_aics->gain_setting.max_gain_setting;
    if (input_gain < p_aics->gain_setting.min_gain_setting) input_gain = p_aics->gain_setting.min_gain_setting;

    WICED_BT_TRACE("[%s] gain %d", __FUNCTION__, input_gain);
    lepl_cap_get_coordinated_set_members(conn_id);
    le_audio_cap_set_mics_aics_gain(&g_lepl_gatt_cb.cap_profile_data, instance, input_gain);
}


int lepl_get_service_instance(gatt_intf_service_object_t **p_store,
                              int limit,
                              const gatt_intf_service_object_t *p_search)
{
    int i = 0;
    while (limit--)
    {
        if (p_store[i] == p_search)
        {
            return i;
        }
        i++;
    }
    return -1;
}

wiced_result_t lepl_mics_callback(uint16_t conn_id,
                                  void *p_app_ctx,
                                  gatt_intf_service_object_t *p_volume,
                                  wiced_bt_gatt_status_t status,
                                  uint32_t evt_type,
                                  gatt_intf_attribute_t *p_char,
                                  void *p_data_ptr,
                                  int len)
{
    wiced_bt_ga_mics_data_t *p_event_data = (wiced_bt_ga_mics_data_t *)p_data_ptr;
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    lepl_mics_data_t *mics_data = &p_clcb->mics_data;

    WICED_BT_TRACE("[%s] event %d char %d status %x", __FUNCTION__, evt_type, p_char->characteristic_type, status);

    if (evt_type == WRITE_CMPL_EVT)
    {
        return WICED_BT_SUCCESS;
    }

    if (p_char->characteristic_type == MICS_MUTE_STATE_CHARACTERISTIC)
    {
        WICED_BT_TRACE("[%s] char %d  mute_state:%x \n", __FUNCTION__, p_char, p_event_data->mute_val);
        mics_data->mute_state = p_event_data->mute_val;
        le_audio_rpc_send_micp_mute_state(conn_id, mics_data->mute_state);
    }
    return WICED_BT_SUCCESS;
}

wiced_result_t lepl_mics_aics_callback(uint16_t conn_id,
                                       void *p_app_ctx,
                                       gatt_intf_service_object_t *p_aics,
                                       wiced_bt_gatt_status_t status,
                                       uint32_t evt_type,
                                       gatt_intf_attribute_t *p_char,
                                       void *p_data_ptr,
                                       int len)
{
    wiced_bt_ga_aics_data_t *p_in = (wiced_bt_ga_aics_data_t *)p_data_ptr;
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    lepl_peer_profiles_t *p_profiles = &p_clcb->peer_profiles;
    int instance = lepl_get_service_instance(p_profiles->p_mics_aics, p_profiles->num_mics_aics, p_aics);
    lepl_aics_data_t *p_lepl_aics = &p_clcb->mics_data.aics[instance];
    uint8_t type = p_char->characteristic_type;

    WICED_BT_TRACE("[%s] evt_type %d event %x status %x inst %d", __FUNCTION__, evt_type, type, status, instance);

    if (evt_type == WRITE_CMPL_EVT)
    {
        return WICED_BT_SUCCESS;
    }

    switch (p_char->characteristic_type)
    {
    case AICS_INPUT_STATE_CHARACTERISTIC:
    {
        wiced_bt_ga_aics_input_state_t *p_st = &p_in->input_state;
        WICED_BT_TRACE("[%s] st (%d %d %d)", __FUNCTION__, p_st->gain_setting, p_st->mute_mode, p_st->gain_mode);

        WICED_MEMCPY(&p_lepl_aics->input_state, p_st, sizeof(wiced_bt_ga_aics_input_state_t));
        le_audio_rpc_send_micp_aics_input_state(conn_id, instance, &p_lepl_aics->input_state);
    }
    break;
    case AICS_GAIN_SETTING_PROPERTIES_CHARACTERISTIC:
    {
        wiced_bt_ga_aics_gain_settings_params_t *p_gs = &p_in->gain_setting;
        WICED_BT_TRACE("[%s] prop %d %d %d",
                       __FUNCTION__,
                       p_gs->gain_setting_units,
                       p_gs->min_gain_setting,
                       p_gs->max_gain_setting);

        WICED_MEMCPY(&p_lepl_aics->gain_setting, p_gs, sizeof(wiced_bt_ga_aics_gain_settings_params_t));
    }
    break;
    case AICS_INPUT_TYPE_CHARACTERISTIC:
    {
        WICED_BT_TRACE("[%s] input_type %x",
                       __FUNCTION__,
                       p_in->input_type);
        p_lepl_aics->input_type = p_in->input_type;
    }
    break;
    case AICS_INPUT_STATUS_CHARACTERISTIC:
    {
        WICED_BT_TRACE("[%s] input_status %x", __FUNCTION__, p_in->input_status);
        p_lepl_aics->input_status = p_in->input_status;
    }
    break;
    case AICS_INPUT_DESCRIPTION_CHARACTERISTIC:
    {
        WICED_BT_TRACE("[%s] desc [%s]", __FUNCTION__, p_in->description.str);

        p_lepl_aics->description_len =
            (p_in->description.len > (MAX_DESCRIPTION - 1)) ? MAX_DESCRIPTION - 1 : p_in->description.len;
        WICED_MEMCPY(p_lepl_aics->description, p_in->description.str, p_lepl_aics->description_len);
        p_lepl_aics->description[p_lepl_aics->description_len] = '\0';

        le_audio_rpc_send_micp_aics_description(conn_id, p_char->included_service_instance, &p_in->description);
    }
    break;
    case AICS_INPUT_CONTROL_POINT_CHARACTERISTIC:
    {
        WICED_BT_TRACE("[%s] event %d write_complete:%x \n", __FUNCTION__, p_char, status);
    }
    break;
    default:
        break;
    }

#ifdef WICED_BT_GA_CAP_ENABLED
    le_audio_cap_mics_update_event(&ga_script_cb.cap_app_data, conn_id, status, evt_type, p_char, p_data_ptr, len);
#endif
    return WICED_BT_SUCCESS;
}