/*
 * $ Copyright Cypress Semiconductor $
 */


#include "lehs.h"
#include "audio_driver.h"

#define AICS_AUDIO_INPUT_DESCRIPTION "MICP AICS"

enum
{
    MICP_AICS_GAIN_INCREMENT,
    MICP_AICS_GAIN_DECREMENT,
    MICP_AICS_SET_GAIN,
} micp_aisc_gain_op_t;

wiced_bt_gatt_status_t lehs_mics_mute(uint16_t conn_id, uint8_t mute)
{
    wiced_bt_ga_mics_data_t mics_data;
    lehs_mics_t *p_mics = &g_lehs_gatt_cb.mics_data;

    audio_driver_set_mic_mute_state(mute, p_mics->aics->input_state.gain_setting);

    mics_data.mute_val = mute;
    p_mics->mute_state = mute;

    gatt_intf_attribute_t characteristic = {0};
    characteristic.characteristic_type = MICS_MUTE_STATE_CHARACTERISTIC;
    le_audio_rpc_send_micp_mute_state(conn_id, mute);
    return gatt_interface_notify_characteristic(conn_id,
                                                g_lehs_gatt_cb.local_profiles.p_mics,
                                                &characteristic,
                                                &mics_data);
}

wiced_bt_gatt_status_t lehs_mics_aics_mute(uint16_t conn_id, uint32_t instance, uint8_t mute)
{
    wiced_bt_ga_mics_data_t mics_data;
    wiced_bt_ga_aics_data_t aics_data;
    mics_data.mics_included.p_aics = &aics_data;
    lehs_mics_t *p_mics = &g_lehs_gatt_cb.mics_data;

    audio_driver_set_mic_mute_state(mute, p_mics->aics->input_state.gain_setting);

    p_mics->aics[instance].input_state.mute_mode = mute;
    WICED_MEMCPY(&aics_data.input_state, &p_mics->aics[instance].input_state, sizeof(wiced_bt_ga_aics_input_state_t));

    gatt_intf_attribute_t characteristic = {0};
    characteristic.characteristic_type = AICS_INPUT_STATE_CHARACTERISTIC;
    characteristic.included_service_instance = instance;
    characteristic.included_service_type = INCLUDED_SERVICE_NONE;
    le_audio_rpc_send_micp_aics_input_state(conn_id,
                                            instance,
                                            &aics_data.input_state);
    return gatt_interface_notify_characteristic(conn_id,
                                                g_lehs_gatt_cb.local_profiles.p_mics,
                                                &characteristic,
                                                &mics_data);
}
wiced_bt_gatt_status_t lehs_mics_aics_set_gain(uint16_t conn_id, uint32_t instance, uint8_t opcode, int8_t input_gain)
{
    wiced_bt_ga_mics_data_t mics_data;
    wiced_bt_ga_aics_data_t aics_data;
    mics_data.mics_included.p_aics = &aics_data;
    lehs_mics_t *p_mics = &g_lehs_gatt_cb.mics_data;

    if (opcode == MICP_AICS_GAIN_INCREMENT)
    {
        int gain = (int)(p_mics->aics[instance].input_state.gain_setting +
                       (p_mics->aics[instance].gain_setting.gain_setting_units * 0.1));
        input_gain = (gain > 127) ? 127 : gain;
    }
    else if (opcode == MICP_AICS_GAIN_DECREMENT)
    {
        int gain = (int)(p_mics->aics[instance].input_state.gain_setting -
                       (p_mics->aics[instance].gain_setting.gain_setting_units * 0.1));
        input_gain = (gain < -128) ? -128 : gain;
    }

    if (input_gain > p_mics->aics[instance].gain_setting.max_gain_setting)
        input_gain = p_mics->aics[instance].gain_setting.max_gain_setting;
    else if (input_gain < p_mics->aics[instance].gain_setting.min_gain_setting)
        input_gain = p_mics->aics[instance].gain_setting.min_gain_setting;

    audio_driver_set_mic_gain(input_gain);

    p_mics->aics[instance].input_state.gain_setting = input_gain;
    WICED_MEMCPY(&aics_data.input_state, &p_mics->aics[instance].input_state, sizeof(wiced_bt_ga_aics_input_state_t));

    le_audio_rpc_send_micp_aics_input_state(conn_id,
                                            instance,
                                            &aics_data.input_state);

    gatt_intf_attribute_t characteristic = {0};
    characteristic.characteristic_type = AICS_INPUT_STATE_CHARACTERISTIC;
    characteristic.included_service_instance = instance;
    characteristic.included_service_type = INCLUDED_SERVICE_NONE;
    return gatt_interface_notify_characteristic(conn_id,
                                                g_lehs_gatt_cb.local_profiles.p_mics,
                                                &characteristic,
                                                &mics_data);
}

wiced_result_t handle_aics_write_req(uint16_t conn_id,
                                     lehs_aics_t *p_aics,
                                     gatt_intf_attribute_t *p_char,
                                     wiced_bt_ga_aics_data_t *p_evt_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;

    switch (p_char->characteristic_type)
    {
    case AICS_INPUT_CONTROL_POINT_CHARACTERISTIC: {
        switch (p_evt_data->control_point.opcode)
        {
        case WICED_BT_GA_AICS_OPCODE_SET_GAIN_SETTINGS:
            p_aics->input_state.gain_setting = p_evt_data->control_point.input_state.gain_setting;
            audio_driver_set_mic_gain(p_aics->input_state.gain_setting);
            break;
        case WICED_BT_GA_AICS_OPCODE_SET_UNMUTE:
            p_aics->input_state.mute_mode = WICED_BT_GA_AICS_UNMUTE;
            audio_driver_set_mic_mute_state(FALSE, p_aics->input_state.gain_setting);
            break;
        case WICED_BT_GA_AICS_OPCODE_SET_MUTE:
            p_aics->input_state.mute_mode = WICED_BT_GA_AICS_MUTE;
            audio_driver_set_mic_mute_state(TRUE, p_aics->input_state.gain_setting);
            break;
        case WICED_BT_GA_AICS_OPCODE_SET_MANUAL_GAIN_MODE:
        case WICED_BT_GA_AICS_OPCODE_SET_AUTO_GAIN_MODE:
            p_aics->input_state.gain_mode = p_evt_data->control_point.input_state.gain_mode;
            break;
        }
        p_evt_data->input_state.gain_mode = p_aics->input_state.gain_mode;
        p_evt_data->input_state.gain_setting = p_aics->input_state.gain_setting;
        p_evt_data->input_state.mute_mode = p_aics->input_state.mute_mode;
        le_audio_rpc_send_micp_aics_input_state(conn_id,
                                                p_char->included_service_instance,
                                                &p_aics->input_state);
    }
    break;
    case AICS_INPUT_DESCRIPTION_CHARACTERISTIC: {
        memset(&p_aics->description, 0, sizeof(p_aics->description));
        p_aics->description_len = (uint8_t)MIN(strlen(p_aics->description), p_evt_data->description.len);
        WICED_MEMCPY(p_aics->description, p_evt_data->description.str, p_aics->description_len);
        le_audio_rpc_send_micp_aics_description(conn_id,
                                               p_char->included_service_instance,
                                               &p_evt_data->description);
    }
    break;
    default:
        result = WICED_BT_ERROR;
    }

    return result;
}

wiced_result_t handle_aics_read_req(uint16_t conn_id,
                                    lehs_aics_t *p_aics,
                                    gatt_intf_attribute_t *p_char,
                                    wiced_bt_ga_aics_data_t *p_evt_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    switch (p_char->characteristic_type)
    {
    case AICS_INPUT_STATE_CHARACTERISTIC:
        p_evt_data->input_state.gain_mode = p_aics->input_state.gain_mode;
        p_evt_data->input_state.gain_setting = p_aics->input_state.gain_setting;
        p_evt_data->input_state.mute_mode = p_aics->input_state.mute_mode;
        break;
    case AICS_GAIN_SETTING_PROPERTIES_CHARACTERISTIC:
        p_evt_data->gain_setting = p_aics->gain_setting;
        break;
    case AICS_INPUT_TYPE_CHARACTERISTIC:
        p_evt_data->input_type = p_aics->input_type;
        break;
    case AICS_INPUT_STATUS_CHARACTERISTIC:
        p_evt_data->input_status = p_aics->input_status;
        break;
    case AICS_INPUT_DESCRIPTION_CHARACTERISTIC:
        p_evt_data->description.len = p_aics->description_len;
        p_evt_data->description.str = p_aics->description;
        break;
    default:
        result = WICED_BT_ERROR;
        break;
    }

    return result;
}

wiced_result_t mics_handle_write_req_evt(uint16_t conn_id,
                                         lehs_mics_t *p_mics,
                                         gatt_intf_attribute_t *p_char,
                                         wiced_bt_ga_mics_data_t *p_evt_data)
{
    wiced_result_t result = WICED_SUCCESS;

    switch (p_char->characteristic_type)
    {
    case MICS_MUTE_STATE_CHARACTERISTIC:
    {
        WICED_BT_TRACE("[%s] event 0x%x mute state:%x \n",
                       __FUNCTION__,
                       p_char->characteristic_type,
                       p_evt_data->mute_val);
        p_mics->mute_state = p_evt_data->mute_val;
        audio_driver_set_mic_mute_state(p_mics->mute_state, p_mics->aics->input_state.gain_setting);
        le_audio_rpc_send_micp_mute_state(conn_id, p_mics->mute_state);
    }
    break;
    default:
        return WICED_BT_GATT_ERROR;
    }
    return result;
}

wiced_result_t mics_handle_read_req_evt(uint16_t conn_id,
                                        lehs_mics_t *p_mics,
                                        gatt_intf_attribute_t *p_char,
                                        wiced_bt_ga_mics_data_t *p_evt_data)
{
    wiced_result_t result = WICED_SUCCESS;

    WICED_BT_TRACE("[%s] char %x ", __FUNCTION__, p_char->characteristic_type);
    switch (p_char->characteristic_type)
    {
    case MICS_MUTE_STATE_CHARACTERISTIC:
        WICED_BT_TRACE("[%s] mute state:%x \n", __FUNCTION__, p_evt_data->mute_val);
        p_evt_data->mute_val = p_mics->mute_state;
        break;
    default:
        break;
    }

    return result;
}

wiced_result_t lehs_mics_callback(uint16_t conn_id,
                                          void *p_app_ctx,
                                          gatt_intf_service_object_t *p_service,
                                          wiced_bt_gatt_status_t status,
                                          uint32_t evt_type,
                                          gatt_intf_attribute_t *p_char,
                                          void *p_evt_data,
                                          int len)
{
    wiced_result_t result = WICED_SUCCESS;
    lehs_mics_t *p_mics = &g_lehs_gatt_cb.mics_data;

    WICED_BT_TRACE("[%s] event 0x%x \n", __FUNCTION__, evt_type);

    switch (evt_type)
    {
    case WRITE_REQ_EVT:
        result = mics_handle_write_req_evt(conn_id, p_mics, p_char, (wiced_bt_ga_mics_data_t *)p_evt_data);
        break;
    case READ_REQ_EVT:
        result = mics_handle_read_req_evt(conn_id, p_mics, p_char, (wiced_bt_ga_mics_data_t *)p_evt_data);
        break;
    }
    return result;
}

void lehs_mics_initialize_data(void)
{
    WICED_BT_TRACE("[%s] ", __FUNCTION__);
    lehs_mics_t *mics_app_data = &g_lehs_gatt_cb.mics_data;
    mics_app_data->mute_state = WICED_BT_MUTE_STATE_MUTED;

    lehs_aics_t *aics_app_data = g_lehs_gatt_cb.mics_data.aics;
    aics_app_data->description_len = (uint8_t)strlen(AICS_AUDIO_INPUT_DESCRIPTION);
    memcpy(aics_app_data->description, AICS_AUDIO_INPUT_DESCRIPTION, strlen(AICS_AUDIO_INPUT_DESCRIPTION));
    aics_app_data->description[aics_app_data->description_len] = '\0';

    aics_app_data->gain_setting.gain_setting_units = 10;
    aics_app_data->gain_setting.max_gain_setting = 127;
    aics_app_data->gain_setting.min_gain_setting = -128;
    aics_app_data->input_state.gain_mode = WICED_BT_GA_AICS_GAIN_MODE_MANUAL;
    aics_app_data->input_state.gain_setting = 5;
    aics_app_data->input_type = WICED_BT_GA_AICS_INPUT_TYPE_BLUETOOTH;
    aics_app_data->input_status = WICED_BT_GA_AICS_INPUT_STATUS_ACTIVE;
}

int lehs_get_service_instance(gatt_intf_service_object_t **p_store,
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

wiced_result_t lehs_mics_aics_callback(uint16_t conn_id,
                                       void *p_app_ctx,
                                       gatt_intf_service_object_t *p_service,
                                       wiced_bt_gatt_status_t status,
                                       uint32_t evt_type,
                                       gatt_intf_attribute_t *p_char,
                                       void *p_evt_data,
                                       int len)
{
    wiced_result_t result = WICED_SUCCESS;
    lehs_local_profiles_t *p_profiles = &g_lehs_gatt_cb.local_profiles;
    int instance = lehs_get_service_instance(p_profiles->p_mics_aics, p_profiles->num_mics_aics, p_service);
    lehs_aics_t *p_aics = &g_lehs_gatt_cb.mics_data.aics[instance];

    WICED_BT_TRACE("[%s] event %d char %d status %x", __FUNCTION__, evt_type, p_char->characteristic_type, status);

    switch (evt_type)
    {
    case WRITE_REQ_EVT:
        result = handle_aics_write_req(conn_id, p_aics, p_char, (wiced_bt_ga_aics_data_t *)p_evt_data);
        break;
    case READ_REQ_EVT:
        result = handle_aics_read_req(conn_id, p_aics, p_char, (wiced_bt_ga_aics_data_t *)p_evt_data);
        break;
    default:
        break;
    }
    return result;
}