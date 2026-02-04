/*
 * $ Copyright Cypress Semiconductor $
 */
#include "lehs.h"
#include "audio_driver.h"

#define MAX_PRESET_RECORDS 5
#define UNIVERSAL_PRESET "Universal"
#define OUTDOOR_PRESET "Outdoor"
#define NOISY_ENVIRONMENT_PRESET "Noisy Environment"

typedef struct
{
    uint8_t preset_index;
    uint8_t properties;
    uint8_t name_len;
    char name[HAS_MAX_PRESET_RECORD_NAME_LENGTH + 1];
} lehs_has_preset_rec_t;

typedef struct
{
    uint8_t procedure_in_progress;
    uint8_t num_rec;
    uint8_t hearing_aid_features;
    uint8_t active_preset_index;
    lehs_has_preset_rec_t *p_preset_rec_list;
}lehs_has_data_t;

lehs_has_preset_rec_t preset_record[] = {
    {.preset_index = 1, .properties = 3, .name = UNIVERSAL_PRESET, .name_len = sizeof(UNIVERSAL_PRESET)},
    {.preset_index = 2, .properties = 3, .name = OUTDOOR_PRESET, .name_len = sizeof(OUTDOOR_PRESET)},
    {.preset_index = 3,
     .properties = 3,
     .name = NOISY_ENVIRONMENT_PRESET,
     .name_len = sizeof(NOISY_ENVIRONMENT_PRESET)}};

lehs_has_data_t lehs_has_data = {.procedure_in_progress = 0,
                                 .active_preset_index = 0,
                                 .hearing_aid_features = HAS_BINAURAL_HEARING_AID |
                                                         HAS_PRESET_SYNCHRONIZATION_NOT_SUPPORTED |
                                                         HAS_IDENTICAL_PRESET_IN_COORDINATED_SET | HAS_STATIC_PRESET |
                                                         HAS_WRITABLE_PRESETS_SUPPORTED,
                                 .num_rec = 3,
                                 .p_preset_rec_list = preset_record};


static int get_position_of_preset_in_list(uint8_t preset_index)
{
    for (int index = 0; index < lehs_has_data.num_rec; index++)
    {
        if (lehs_has_data.p_preset_rec_list[index].preset_index == preset_index)
            return index;
    }
    return -1;
}

static uint8_t get_prevIndex_for_preset_rec(uint8_t preset_index)
{
    if (lehs_has_data.num_rec == 0 || preset_index == 0)
        return 0;

    lehs_has_preset_rec_t *p_rec_list = lehs_has_data.p_preset_rec_list;
    for (int index = 0; index < lehs_has_data.num_rec; index++)
    {
        if (p_rec_list[index].preset_index >= preset_index)
        {
            if (index == 0) return 0;
            return p_rec_list[index - 1].preset_index;
        }
    }
    return p_rec_list[lehs_has_data.num_rec - 1].preset_index;
}

static wiced_result_t has_handle_read_preset_record(uint16_t conn_id, wiced_bt_ga_has_cp_read_preset_t *p_read_pset)
{
    uint8_t index = 0;
    uint8_t prevIndex = 0;

    if (lehs_has_data.procedure_in_progress)
    {
        return WICED_BT_GATT_PRC_IN_PROGRESS;
    }
    if (!lehs_has_data.num_rec)
    {
        return WICED_BT_GATT_OUT_OF_RANGE;
    }

    prevIndex = get_prevIndex_for_preset_rec(p_read_pset->start_index);
    index = get_position_of_preset_in_list(prevIndex);
    index += 1;

    if (lehs_has_data.num_rec == index)
    {
        return WICED_BT_GATT_OUT_OF_RANGE;
    }

    lehs_has_data.procedure_in_progress = WICED_BT_GA_HAS_OPCODE_READ_PRESETS_REQUEST;
    for (int i = 0; (i < p_read_pset->num_presets) && ((index + i) < lehs_has_data.num_rec); i++)
    {
        wiced_bt_ga_has_data_t event_data;
        wiced_bt_ga_has_preset_records_t *p_n = &event_data.evt_data.read_rsp_prest;
        lehs_has_preset_rec_t *p_rec = &lehs_has_data.p_preset_rec_list[index + i];
        event_data.evt_data.opcode = WICED_BT_GA_HAS_OPCODE_READ_PRESET_RESPONSE;
        if (i == (p_read_pset->num_presets - 1) || index == (lehs_has_data.num_rec - 1))
            p_n->is_last = 1;
        else
            p_n->is_last = 0;

        p_n->preset_index = p_rec->preset_index;
        p_n->properties = p_rec->properties;
        p_n->name.str = p_rec->name;
        p_n->name.len = p_rec->name_len;

        gatt_intf_attribute_t characteristic = {.characteristic_type =
                                                    HAS_HEARING_AID_PRESET_CONTROL_POINT_CHARACTERISTIC};
        gatt_interface_notify_characteristic(conn_id,
                                             g_lehs_gatt_cb.local_profiles.p_has,
                                             &characteristic,
                                             &event_data);
    }
    lehs_has_data.procedure_in_progress = WICED_BT_GA_HAS_OPCODE_INVALID;
    return WICED_BT_SUCCESS;
}

static wiced_result_t has_handle_write_preset_name(uint16_t conn_id, wiced_bt_ga_has_preset_records_t *p_preset_rec)
{
    int index = 0;

    if (lehs_has_data.procedure_in_progress)
    {
        return WICED_BT_GATT_PRC_IN_PROGRESS;
    }
    if (!lehs_has_data.num_rec)
    {
        return WICED_BT_GATT_OUT_OF_RANGE;
    }
    if (!(lehs_has_data.hearing_aid_features & HAS_WRITABLE_PRESETS_SUPPORTED))
    {
        return WICED_BT_GA_HAS_ERROR_WRITE_NAME_NOT_ALLOWED;
    }

    index = get_position_of_preset_in_list(p_preset_rec->preset_index);
    if (index == -1)
    {
        return WICED_BT_GATT_OUT_OF_RANGE;
    }

    lehs_has_preset_rec_t *p_rec = &lehs_has_data.p_preset_rec_list[index];
    if (!(p_rec->properties & 1))
    {
        return WICED_BT_GA_HAS_ERROR_WRITE_NAME_NOT_ALLOWED;
    }

    WICED_MEMCPY(p_rec->name, p_preset_rec->name.str, p_preset_rec->name.len);
    p_rec->name_len = p_preset_rec->name.len;
    p_rec->name[p_rec->name_len] = '\0';

    wiced_bt_ga_has_evt_data_t event_data;
    event_data.opcode = WICED_BT_GA_HAS_OPCODE_PRESET_CHANGED;
    event_data.preset_changed.change_id = WICED_BT_GA_HAS_GENERIC_UPDATE;
    event_data.preset_changed.preset_rec.preset_index = p_preset_rec->preset_index;
    if (index == 0)
    {
        event_data.preset_changed.prev_index = 0;
    }
    else
    {
        event_data.preset_changed.prev_index = lehs_has_data.p_preset_rec_list[index-1].preset_index;
    }
    event_data.preset_changed.preset_rec.properties = p_rec->properties;
    event_data.preset_changed.preset_rec.name.str = p_rec->name;
    event_data.preset_changed.preset_rec.name.len = p_rec->name_len;
    gatt_intf_attribute_t characteristic = {.characteristic_type = HAS_HEARING_AID_PRESET_CONTROL_POINT_CHARACTERISTIC};
    gatt_interface_notify_characteristic(conn_id,
                                         g_lehs_gatt_cb.local_profiles.p_has,
                                         &characteristic,
                                         &event_data);
   // le_audio_rpc_send_preset_record(conn_id, p_rec->preset_index, &event_data.preset_changed.preset_rec.name);

    return WICED_SUCCESS;
}

static wiced_result_t has_set_active_preset(uint16_t conn_id, uint8_t preset_index)
{
    wiced_bt_ga_has_evt_data_t event_data = {0};
    gatt_intf_attribute_t characteristic = {0};
    gatt_intf_service_object_t *p_has = g_lehs_gatt_cb.local_profiles.p_has;

    int index = get_position_of_preset_in_list(preset_index);

    if (index == -1)
        return WICED_BT_GATT_OUT_OF_RANGE;

    lehs_has_preset_rec_t *p_rec = &lehs_has_data.p_preset_rec_list[index];
    if (!(p_rec->properties & 2))
    {
        return WICED_BT_GA_HAS_ERROR_PRESET_OPERATION_NOT_POSSIBLE;
    }

    characteristic.characteristic_type = HAS_ACTIVE_PRESET_INDEX_CHARACTERISTIC;
    event_data.active_preset_index = preset_index;
    gatt_interface_notify_characteristic(conn_id, p_has, &characteristic, &event_data);

    p_rec->properties &= ~(1 << 1); // Clear IsAvailable bit
    lehs_has_data.active_preset_index = preset_index;
    event_data.opcode = WICED_BT_GA_HAS_OPCODE_PRESET_CHANGED;
    event_data.preset_changed.change_id = WICED_BT_GA_HAS_PRESET_RECORD_UNAVAILABLE;
    event_data.preset_changed.preset_index = preset_index;
    characteristic.characteristic_type = HAS_HEARING_AID_PRESET_CONTROL_POINT_CHARACTERISTIC;
    gatt_interface_notify_characteristic(conn_id, p_has, &characteristic, &event_data);

    if (lehs_has_data.active_preset_index)
    {
        index = get_position_of_preset_in_list(lehs_has_data.active_preset_index);
        if (index != -1)
        {
            p_rec = &lehs_has_data.p_preset_rec_list[index];
            p_rec->properties |= (1 << 1); // Set isAvalaible bit
            event_data.opcode = WICED_BT_GA_HAS_OPCODE_PRESET_CHANGED;
            event_data.preset_changed.change_id = WICED_BT_GA_HAS_PRESET_RECORD_AVAILABLE;
            event_data.preset_changed.preset_index = p_rec->preset_index;
            gatt_interface_notify_characteristic(conn_id,
                                                 g_lehs_gatt_cb.local_profiles.p_has,
                                                 &characteristic,
                                                 &event_data);
        }
    }

   return WICED_SUCCESS;
}

static wiced_result_t has_set_next_preset(uint16_t conn_id)
{
    int index = get_position_of_preset_in_list(lehs_has_data.active_preset_index) + 1;
    while (index < lehs_has_data.num_rec)
    {
        lehs_has_preset_rec_t *p_rec = &lehs_has_data.p_preset_rec_list[index];
        if (p_rec->properties & 2)
        {
            return has_set_active_preset(conn_id, p_rec->preset_index);
        }
        index++;
    }

    // First available preset record
    for (index = 0; index < lehs_has_data.active_preset_index; index++)
    {
        lehs_has_preset_rec_t *p_rec = &lehs_has_data.p_preset_rec_list[index];
        if (p_rec->properties & 2)
        {
            return has_set_active_preset(conn_id, p_rec->preset_index);
        }
    }

    // No available preset record
    return WICED_BT_GA_HAS_ERROR_PRESET_OPERATION_NOT_POSSIBLE;
}

static wiced_result_t has_set_previous_preset(uint16_t conn_id)
{
    int index = get_position_of_preset_in_list(lehs_has_data.active_preset_index) - 1;

    while (index >= 0)
    {
        lehs_has_preset_rec_t *p_rec = &lehs_has_data.p_preset_rec_list[index];
        if (p_rec->properties & 2)
        {
            return has_set_active_preset(conn_id, p_rec->preset_index);
        }
        index--;
    }

    // Last available preset record
    index = lehs_has_data.num_rec - 1;
    for (index = (lehs_has_data.num_rec - 1); index > 0; index--)
    {
        lehs_has_preset_rec_t *p_rec = &lehs_has_data.p_preset_rec_list[index];
        if (p_rec->properties & 2)
        {
            return has_set_active_preset(conn_id, p_rec->preset_index);
        }
    }

    // No available preset record
    return WICED_BT_GA_HAS_ERROR_PRESET_OPERATION_NOT_POSSIBLE;
}


static wiced_result_t has_handle_write_req_evt(uint16_t conn_id,
                                               gatt_intf_attribute_t* p_char,
                                               wiced_bt_ga_has_control_point_t *p_evt_data)
{
    wiced_result_t result = WICED_SUCCESS;

    if (p_char->characteristic_type != HAS_HEARING_AID_PRESET_CONTROL_POINT_CHARACTERISTIC) return WICED_ERROR;

    WICED_BT_TRACE("[%s] opcode %d \n", __FUNCTION__, p_evt_data->opcode);

    switch (p_evt_data->opcode)
    {
    case WICED_BT_GA_HAS_OPCODE_READ_PRESETS_REQUEST:
        result = has_handle_read_preset_record(conn_id, &p_evt_data->read_preset);
        break;
    case WICED_BT_GA_HAS_OPCODE_WRITE_PRESET_NAME:
        result = has_handle_write_preset_name(conn_id, &p_evt_data->preset_rec);
        break;
    case WICED_BT_GA_HAS_OPCODE_SET_ACTIVE_PRESET:
        result = has_set_active_preset(conn_id, p_evt_data->preset_index);
        break;
    case WICED_BT_GA_HAS_OPCODE_SET_NEXT_PRESET:
        result = has_set_next_preset(conn_id);
        break;
    case WICED_BT_GA_HAS_OPCODE_SET_PREVIOUS_PRESET:
        result = has_set_previous_preset(conn_id);
        break;
    case WICED_BT_GA_HAS_OPCODE_SET_ACTIVE_PRESET_SYNCHRONIZED_LOCALLY:
        if (!(lehs_has_data.hearing_aid_features & HAS_PRESET_SYNCHRONIZATION_SUPPORTED))
        {
            return WICED_BT_GA_HAS_ERROR_PRESET_SYNCHRONIZATION_NOT_SUPPORTED;
        }
        break;
    case WICED_BT_GA_HAS_OPCODE_SET_NEXT_PRESET_SYNCHRONIZED_LOCALLY:
        if (!(lehs_has_data.hearing_aid_features & HAS_PRESET_SYNCHRONIZATION_SUPPORTED))
        {
            return WICED_BT_GA_HAS_ERROR_PRESET_SYNCHRONIZATION_NOT_SUPPORTED;
        }
        break;
    case WICED_BT_GA_HAS_OPCODE_SET_PREVIOUS_PRESET_SYNCHRONIZED_LOCALLY:
        if (!(lehs_has_data.hearing_aid_features & HAS_PRESET_SYNCHRONIZATION_SUPPORTED))
        {
            return WICED_BT_GA_HAS_ERROR_PRESET_SYNCHRONIZATION_NOT_SUPPORTED;
        }
        break;
    }

    return result;
}

static wiced_result_t has_handle_read_req_evt(uint16_t conn_id,
                                              gatt_intf_attribute_t * p_char,
                                              wiced_bt_ga_has_evt_data_t *p_evt_data)
{
    wiced_result_t result = WICED_SUCCESS;

    switch (p_char->characteristic_type)
    {
    case HAS_HEARING_AID_FEATURES_CHARACTERISTIC:
        p_evt_data->hearing_aid_feature = lehs_has_data.hearing_aid_features;
        break;

    case HAS_ACTIVE_PRESET_INDEX_CHARACTERISTIC:
        p_evt_data->active_preset_index = lehs_has_data.active_preset_index;
        break;
    }
    return result;
}

wiced_result_t lehs_has_callback(uint16_t conn_id,
    void* p_app_ctx,
    gatt_intf_service_object_t* p_service,
    wiced_bt_gatt_status_t status,
    uint32_t evt_type,
    gatt_intf_attribute_t* p_char,
    void* p_data,
    int len)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_bt_ga_has_data_t *p_has_data = (wiced_bt_ga_has_data_t *)p_data;
    WICED_BT_TRACE("[%s] event 0x%x \n", __FUNCTION__, p_char);

    switch (evt_type)
    {
    case WRITE_REQ_EVT:
        result = has_handle_write_req_evt(conn_id, p_char, &p_has_data->has_cp_cmd);
        break;

    case READ_REQ_EVT:
        result = has_handle_read_req_evt(conn_id, p_char, &p_has_data->evt_data);
        break;

    default:
        break;
    }
    return result;
}

void lehs_has_initialize_data(void)
{

}
