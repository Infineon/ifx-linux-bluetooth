/*
 * $ Copyright Cypress Semiconductor $
 */
#include "lehs.h"
#include "audio_driver.h"

static void vcs_set_volume(lehs_volume_t *p_vcs, int incr)
{
    int new_volume = p_vcs->state.volume_setting + incr;

    /* ensure that the updated volume is within the supported range */
    new_volume = (new_volume < WICED_BT_GA_VCS_MINIMUM_VOLUME) ? WICED_BT_GA_VCS_MINIMUM_VOLUME : new_volume;
    new_volume = (new_volume > WICED_BT_GA_VCS_MAXIMUM_VOLUME) ? WICED_BT_GA_VCS_MAXIMUM_VOLUME : new_volume;

    p_vcs->state.volume_setting = new_volume;

    WICED_BT_TRACE("[%s] new_volume %d (0 ~ 255) \n", __FUNCTION__, new_volume);

    /* VCS profile defines volume range on 0-255, hence mapping the received value to 0-100 */
    audio_driver_set_volume((p_vcs->state.volume_setting * 100) / 255);
}

static wiced_result_t vcs_handle_write_req_evt(uint16_t conn_id,
                                               lehs_volume_t *p_vcs,
                                               gatt_intf_attribute_t *p_char,
                                               wiced_bt_ga_vcs_data_t *p_evt_data)
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t char_type = 0;

    if (p_char->characteristic_type != VCS_CONTROL_POINT_CHARACTERISTIC) return WICED_ERROR;

    WICED_BT_TRACE("[%s] opcode %d \n", __FUNCTION__, p_evt_data->control_point_data.opcode);

    switch (p_evt_data->control_point_data.opcode)
    {
        case VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_DOWN:
            vcs_set_volume(p_vcs, -VCS_STEP_SIZE);
            char_type = HCI_CONTROL_LEA_VOLUME_STATUS;
            break;

        case VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_UP:
            vcs_set_volume(p_vcs, +VCS_STEP_SIZE);
            char_type = HCI_CONTROL_LEA_VOLUME_STATUS;
            break;

        case VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_DOWN:
            p_vcs->state.mute_state = WICED_BT_GA_VCS_NOT_MUTED;
            audio_driver_set_mute_state(p_vcs->state.mute_state);
            vcs_set_volume(p_vcs, -VCS_STEP_SIZE);
            char_type = HCI_CONTROL_LEA_MUTE_AND_VOLUME_STATUS;
            break;

        case VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_UP:
            p_vcs->state.mute_state = WICED_BT_GA_VCS_NOT_MUTED;
            audio_driver_set_mute_state(p_vcs->state.mute_state);
            vcs_set_volume(p_vcs, +VCS_STEP_SIZE);
            char_type = HCI_CONTROL_LEA_MUTE_AND_VOLUME_STATUS;
            break;

        case VOLUME_CONTROL_OPCODE_SET_ABSOLUTE_VOLUME: {
            // update volume flag
            if (p_evt_data->control_point_data.volume_state.volume_setting <= WICED_BT_GA_VCS_MAXIMUM_VOLUME)
            {
                p_vcs->state.volume_setting = p_evt_data->control_point_data.volume_state.volume_setting;
                vcs_set_volume(p_vcs, 0);
            }
            char_type = HCI_CONTROL_LEA_VOLUME_STATUS;
        }
        break;

        case VOLUME_CONTROL_OPCODE_UNMUTE:
            p_vcs->state.mute_state = WICED_BT_GA_VCS_NOT_MUTED;
            audio_driver_set_mute_state(p_vcs->state.mute_state);
            char_type = HCI_CONTROL_LEA_MUTE_STATUS;
            break;

        case VOLUME_CONTROL_OPCODE_MUTE:
            p_vcs->state.mute_state = WICED_BT_GA_VCS_MUTED;
            audio_driver_set_mute_state(p_vcs->state.mute_state);
            char_type = HCI_CONTROL_LEA_MUTE_STATUS;
            break;
    }

    le_audio_rpc_send_vcs_state_update(conn_id, p_vcs->state.volume_setting, p_vcs->state.mute_state, char_type);

    return result;
}

static wiced_result_t vcs_handle_read_req_evt(uint16_t conn_id,
                                              lehs_volume_t *p_vcs,
                                              gatt_intf_attribute_t *p_char,
                                              wiced_bt_ga_vcs_data_t *p_evt_data)
{
    wiced_result_t result = WICED_SUCCESS;

    if (p_char->included_service_type != INCLUDED_SERVICE_NONE) return WICED_ERROR;

    switch (p_char->characteristic_type)
    {
        case VCS_VOLUME_STATE_CHARACTERISTIC:
            p_evt_data->control_point_data.volume_state = p_vcs->state;
            break;

        case VCS_VOLUME_FLAG_CHARACTERISTIC:
            p_evt_data->volume_flag = p_vcs->flag;
            break;
    }

    return result;
}

wiced_result_t lehs_vcs_callback(uint16_t conn_id,
                                         void *p_app_ctx,
                                         gatt_intf_service_object_t *p_service,
                                         wiced_bt_gatt_status_t status,
                                         uint32_t evt_type,
                                         gatt_intf_attribute_t *p_char,
                                         void *p_data,
                                         int len)
{
    wiced_result_t result = WICED_SUCCESS;
    lehs_volume_t *p_vcs = &g_lehs_gatt_cb.vcs_data;

    WICED_BT_TRACE("[%s] event 0x%x \n", __FUNCTION__, p_char);

    switch (evt_type)
    {
        case WRITE_REQ_EVT:
            result = vcs_handle_write_req_evt(conn_id, p_vcs, p_char, p_data);
            break;

        case READ_REQ_EVT:
            result = vcs_handle_read_req_evt(conn_id, p_vcs, p_char, p_data);
            break;

        default:
            break;
    }
    return result;
}

void lehs_vcs_initialize_data(void)
{
    lehs_volume_t *p_vcs = &g_lehs_gatt_cb.vcs_data;

    p_vcs->flag = 1;
    p_vcs->state.mute_state = WICED_BT_MUTE_STATE_NOT_MUTED;
    p_vcs->state.volume_setting = 130;

    // initialize volume in alsa driver
    vcs_set_volume(p_vcs, 0);
}

wiced_result_t lehs_vcs_set_volume(uint16_t conn_id, volume_control_opcodes_t vcs_opcode, uint8_t abs_vol)
{
    wiced_bt_ga_vcs_data_t vcs_data;
    lehs_volume_t *p_vcs = &g_lehs_gatt_cb.vcs_data;
    uint8_t char_type = 0;

    switch (vcs_opcode)
    {
        case VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_DOWN:
            vcs_set_volume(p_vcs, -VCS_STEP_SIZE);
            char_type = HCI_CONTROL_LEA_VOLUME_STATUS;
            break;

        case VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_UP:
            vcs_set_volume(p_vcs, VCS_STEP_SIZE);
            char_type = HCI_CONTROL_LEA_VOLUME_STATUS;
            break;

        case VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_DOWN:
            p_vcs->state.mute_state = WICED_BT_GA_VCS_NOT_MUTED;
            audio_driver_set_mute_state(p_vcs->state.mute_state);
            vcs_set_volume(p_vcs, -VCS_STEP_SIZE);
            char_type = HCI_CONTROL_LEA_MUTE_AND_VOLUME_STATUS;
            break;

        case VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_UP:
            p_vcs->state.mute_state = WICED_BT_GA_VCS_NOT_MUTED;
            audio_driver_set_mute_state(p_vcs->state.mute_state);
            vcs_set_volume(p_vcs, +VCS_STEP_SIZE);
            char_type = HCI_CONTROL_LEA_MUTE_AND_VOLUME_STATUS;
            break;

        case VOLUME_CONTROL_OPCODE_SET_ABSOLUTE_VOLUME: {
            if (abs_vol <= WICED_BT_GA_VCS_MAXIMUM_VOLUME)
            {
                p_vcs->state.volume_setting = abs_vol;
                vcs_set_volume(p_vcs, 0);
            }
            char_type = HCI_CONTROL_LEA_VOLUME_STATUS;
        }
            break;

        case VOLUME_CONTROL_OPCODE_UNMUTE:
            p_vcs->state.mute_state = WICED_BT_GA_VCS_NOT_MUTED;
            audio_driver_set_mute_state(p_vcs->state.mute_state);
            char_type = HCI_CONTROL_LEA_MUTE_STATUS;
            break;

        case VOLUME_CONTROL_OPCODE_MUTE:
            p_vcs->state.mute_state = WICED_BT_GA_VCS_MUTED;
            audio_driver_set_mute_state(p_vcs->state.mute_state);
            char_type = HCI_CONTROL_LEA_MUTE_STATUS;
            break;
        default:
            WICED_BT_TRACE("[%s] Invalid opcode %d",__FUNCTION__, vcs_opcode);
            return WICED_ERROR;
    }
     le_audio_rpc_send_vcs_state_update(conn_id,
                                        p_vcs->state.volume_setting,
                                        p_vcs->state.mute_state,
                                        char_type);

    vcs_data.control_point_data.volume_state.volume_setting = p_vcs->state.volume_setting;
    vcs_data.control_point_data.volume_state.mute_state = p_vcs->state.mute_state;
    vcs_data.control_point_data.opcode = vcs_opcode;

    gatt_intf_attribute_t characteristic = {0};
    characteristic.characteristic_type = VCS_VOLUME_STATE_CHARACTERISTIC;
    return gatt_interface_notify_characteristic(conn_id,
                                                g_lehs_gatt_cb.local_profiles.p_vcs,
                                                &characteristic,
                                                &vcs_data);
}

void lehs_set_vol(volume_control_opcodes_t vcs_opcode, uint8_t abs_vol)
{
    static uint8_t vol = DEFAULT_VOL;
    static uint8_t mute_state = WICED_BT_MUTE_STATE_NOT_MUTED;
    uint8_t char_type = 0;
    switch (vcs_opcode)
    {
    case VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_DOWN:
    case VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_DOWN:
        char_type = HCI_CONTROL_LEA_VOLUME_STATUS;
        if (vcs_opcode == VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_DOWN)
        {
            audio_driver_set_mute_state(0);
            mute_state = WICED_BT_MUTE_STATE_NOT_MUTED;
            char_type = HCI_CONTROL_LEA_MUTE_AND_VOLUME_STATUS;
        }

        if (vol >= VCS_STEP_SIZE)
        {
            vol -= VCS_STEP_SIZE;
            audio_driver_set_volume(((vol)*100) / 255);
        }
        break;
    case VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_UP:
    case VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_UP:
        char_type = HCI_CONTROL_LEA_VOLUME_STATUS;
        if (vcs_opcode == VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_UP)
        {
            audio_driver_set_mute_state(0);
            mute_state = WICED_BT_MUTE_STATE_NOT_MUTED;
            char_type = HCI_CONTROL_LEA_MUTE_AND_VOLUME_STATUS;
        }

        if ((vol + VCS_STEP_SIZE) <= 255)
        {
            vol += VCS_STEP_SIZE;
            audio_driver_set_volume(((vol)*100) / 255);
        }
        break;
    case VOLUME_CONTROL_OPCODE_SET_ABSOLUTE_VOLUME:
        vol = abs_vol;
        audio_driver_set_volume((vol * 100) / 255);
        char_type = HCI_CONTROL_LEA_VOLUME_STATUS;
        break;
    case VOLUME_CONTROL_OPCODE_MUTE:
        audio_driver_set_mute_state(1);
        mute_state = WICED_BT_MUTE_STATE_MUTED;
        char_type = HCI_CONTROL_LEA_MUTE_STATUS;
        break;
    case VOLUME_CONTROL_OPCODE_UNMUTE:
        audio_driver_set_mute_state(0);
        mute_state = WICED_BT_MUTE_STATE_NOT_MUTED;
        char_type = HCI_CONTROL_LEA_MUTE_STATUS;
        break;
    default:
        WICED_BT_TRACE("[%s] Invalid opcode %d", __FUNCTION__, vcs_opcode);
        break;
    }

    le_audio_rpc_send_vcs_state_update(0, vol, mute_state, char_type);
}
