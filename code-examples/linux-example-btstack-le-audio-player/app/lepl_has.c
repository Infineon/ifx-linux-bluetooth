/*
 * $ Copyright Cypress Semiconductor $
 */
#include "lepl.h"

enum
{
    SET_ACTIVE_PRESET = 1,
    SET_NEXT_PRESET,
    SET_PREVIOUS_PRESET,
};

static int get_position_of_preset_in_list(uint16_t conn_id, uint8_t preset_index)
{
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    if (!p_clcb)
    {
        return -1;
    }

    lepl_has_data_t *p_has = &p_clcb->has_data;
    if (!preset_index || !p_has->num_rec)
    {
        WICED_BT_TRACE("[%s] Not valid preset index: %d num rec %d", __FUNCTION__, preset_index, p_has->num_rec);
        return -1;
    }

    for (int index = 0; index < p_has->num_rec; index++)
    {
        lepl_has_preset_rec_t *p_rec = &p_has->preset_rec_list[index];
        if (p_rec->preset_index == preset_index)
        {
            WICED_BT_TRACE("[%s] preset index %d pos %d", __FUNCTION__, preset_index, index);
            return index;
        }
    }

    WICED_BT_TRACE("[%s] Not available in the list", __FUNCTION__);
    return -1;
}

static uint8_t get_prevIndex_for_preset_rec(uint16_t conn_id, uint8_t preset_index)
{
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    if (!p_clcb )
    {
        return 0;
    }

    lepl_has_data_t *p_has = &p_clcb->has_data;
    if (!preset_index || !p_has->num_rec)
    {
        WICED_BT_TRACE("[%s] Not valid preset index: %d", __FUNCTION__, preset_index);
        return 0;
    }

    lepl_has_preset_rec_t *p_rec_list = p_has->preset_rec_list;
    for (int index = 0; index < p_has->num_rec; index++)
    {
        if (p_rec_list[index].preset_index >= preset_index)
        {
            if (index == 0) return 0;
            return p_rec_list[index - 1].preset_index;
        }
    }
    return p_rec_list[p_has->num_rec - 1].preset_index;
}

void lepl_hap_read_preset_records(uint16_t conn_id)
{
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    if (!p_clcb)
    {
        return;
    }
    gatt_intf_service_object_t *p_service = p_clcb->peer_profiles.p_has;
    if (!p_service)
    {
        WICED_BT_TRACE("HAS not supported by peer device !");
        return;
    }
    gatt_intf_attribute_t characteristic = {.characteristic_type = HAS_HEARING_AID_PRESET_CONTROL_POINT_CHARACTERISTIC};
    wiced_bt_ga_has_control_point_t has_cp_cmd = {.opcode = WICED_BT_GA_HAS_OPCODE_READ_PRESETS_REQUEST,
                                                  .read_preset.start_index = 1,
                                                  .read_preset.num_presets = 5};

    wiced_bt_gatt_status_t status =
        gatt_interface_write_characteristic(conn_id, p_service, &characteristic, &has_cp_cmd);
    WICED_BT_TRACE("[%s] status %d \n", __FUNCTION__, status);
}

void lepl_hap_set_active_preset(uint16_t conn_id, uint8_t opcode, uint8_t preset_index)
{
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    if (!p_clcb)
    {
        WICED_BT_TRACE_CRIT("No clcb");
        return;
    }
    gatt_intf_service_object_t *p_service = p_clcb->peer_profiles.p_has;
    if (!p_service)
    {
        WICED_BT_TRACE_CRIT("HAS not supported by peer device !");
        return;
    }
    gatt_intf_attribute_t characteristic = {.characteristic_type = HAS_HEARING_AID_PRESET_CONTROL_POINT_CHARACTERISTIC};
    wiced_bt_ga_has_control_point_t has_cp_cmd = {0};
    switch (opcode)
    {
    case SET_ACTIVE_PRESET:
        has_cp_cmd.opcode = WICED_BT_GA_HAS_OPCODE_SET_ACTIVE_PRESET;
        has_cp_cmd.preset_index = preset_index;
        break;
    case SET_NEXT_PRESET:
        has_cp_cmd.opcode = WICED_BT_GA_HAS_OPCODE_SET_NEXT_PRESET;
        break;
    case SET_PREVIOUS_PRESET:
        has_cp_cmd.opcode = WICED_BT_GA_HAS_OPCODE_SET_PREVIOUS_PRESET;
        break;
    }
    wiced_bt_gatt_status_t status =
        gatt_interface_write_characteristic(conn_id, p_service, &characteristic, &has_cp_cmd);
    WICED_BT_TRACE("[%s] status %d \n", __FUNCTION__, status);
}

void lepl_hap_set_preset_name(uint16_t conn_id, uint8_t preset_index, gatt_intf_string_t *p_name)
{
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    if (!p_clcb)
    {
        WICED_BT_TRACE_CRIT("No clcb");
        return;
    }
    gatt_intf_service_object_t *p_service = p_clcb->peer_profiles.p_has;
    if (!p_service)
    {
        WICED_BT_TRACE_CRIT("HAS not supported by peer device !");
        return;
    }
    gatt_intf_attribute_t characteristic = {.characteristic_type = HAS_HEARING_AID_PRESET_CONTROL_POINT_CHARACTERISTIC};
    int index = get_position_of_preset_in_list(conn_id, preset_index);
    if (!p_clcb->has_data.preset_rec_list[index].properties & 1)
    {
        WICED_BT_TRACE("[%s] write is not allowed", __FUNCTION__);
        return;
    }

    wiced_bt_ga_has_control_point_t has_cp_cmd = {
        .opcode = WICED_BT_GA_HAS_OPCODE_WRITE_PRESET_NAME,
        .preset_rec.preset_index = preset_index,
        .preset_rec.properties =
               p_clcb->has_data.preset_rec_list[index].properties,
        .preset_rec.name.str = p_name->str,
        .preset_rec.name.len = p_name->len};
    wiced_bt_gatt_status_t status =
        gatt_interface_write_characteristic(conn_id, p_service, &characteristic, &has_cp_cmd);
    WICED_BT_TRACE("[%s] status %d \n", __FUNCTION__, status);
}

static void hap_handle_delete_preset_rec(uint16_t conn_id, uint8_t preset_index)
{
    int index = get_position_of_preset_in_list(conn_id, preset_index);
    if (index == -1)
    {
        WICED_BT_TRACE_CRIT("[%s] Not present index %d", __FUNCTION__, preset_index);
        return;
    }

    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    lepl_has_preset_rec_t *p_rec_list = p_clcb->has_data.preset_rec_list;
    uint8_t num_rec = p_clcb->has_data.num_rec;
    for (int i = index; i < num_rec - 1; i++)
    {
        WICED_MEMCPY(&p_rec_list[i], &p_rec_list[i + 1], sizeof(lepl_has_preset_rec_t));
    }
    WICED_MEMSET(&p_rec_list[num_rec - 1], 0, sizeof(lepl_has_preset_rec_t));
    p_clcb->has_data.num_rec -= 1;
}

static void hap_handle_add_preset_rec_to_list(uint16_t conn_id,
                                            uint8_t prevIndex,
                                            wiced_bt_ga_has_preset_records_t *p_rec)
{
    int index = get_position_of_preset_in_list(conn_id, prevIndex);
    index += 1;

    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    lepl_has_preset_rec_t *p_rec_list = p_clcb->has_data.preset_rec_list;
    uint8_t num_rec = p_clcb->has_data.num_rec;
    while (index < num_rec && (p_rec_list[index].preset_index < p_rec->preset_index))
    {
        hap_handle_delete_preset_rec(conn_id, p_rec_list[index].preset_index);
        num_rec -= 1;
    }
    if (num_rec == MAX_PRESET_RECORDS)
    {
        WICED_BT_TRACE_CRIT("Storing capacity Exceeded");
        return;
    }

    for (int i = num_rec; i > index && i > 0; i--)
    {
        WICED_MEMCPY(&p_rec_list[i], &p_rec_list[i - 1], sizeof(lepl_has_preset_rec_t));
    }

    WICED_BT_TRACE("[%s] index %d", __FUNCTION__, p_rec->preset_index);
    p_rec_list[index].preset_index = p_rec->preset_index;
    p_rec_list[index].properties = p_rec->properties;
    p_rec_list[index].name_len = p_rec->name.len;
    WICED_MEMCPY(p_rec_list[index].name, p_rec->name.str, p_rec->name.len);
    p_rec_list[index].name[p_rec->name.len] = '\0';
    p_clcb->has_data.num_rec += 1;
    le_audio_rpc_send_preset_record(conn_id, p_rec->preset_index, &p_rec->name);
}

static void hap_handle_read_preset_response(uint16_t conn_id, wiced_bt_ga_has_preset_records_t *p_rec)
{
    int index = get_position_of_preset_in_list(conn_id, p_rec->preset_index);
    if (index != -1)
    {
        lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
        lepl_has_preset_rec_t *p_pset = &p_clcb->has_data.preset_rec_list[index];
        p_pset->properties = p_rec->properties;
        WICED_MEMCPY(p_pset->name, p_rec->name.str, p_rec->name.len);
        p_pset->name[p_rec->name.len] = '\0';
        p_pset->name_len = p_rec->name.len;
        le_audio_rpc_send_preset_record(conn_id, p_rec->preset_index, &p_rec->name);
    }
    else
    {
        uint8_t prevIndex = get_prevIndex_for_preset_rec(conn_id, p_rec->preset_index);
        hap_handle_add_preset_rec_to_list(conn_id, prevIndex, p_rec);
    }
}

static void hap_handle_preset_record_changed(uint16_t conn_id, wiced_bt_ga_has_cp_rsp_preset_changed_t *p_pset_changed)
{
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    lepl_has_data_t *p_has = &p_clcb->has_data;
    switch (p_pset_changed->change_id)
    {
    case WICED_BT_GA_HAS_GENERIC_UPDATE: {
        int index = get_position_of_preset_in_list(conn_id, p_pset_changed->preset_rec.preset_index);
        if (index != -1)
        {
            lepl_has_preset_rec_t *p_pset = &p_has->preset_rec_list[index];
            wiced_bt_ga_has_preset_records_t *p_rec = &p_pset_changed->preset_rec;
            p_pset->properties = p_rec->properties;
            p_pset->name_len = p_rec->name.len;
            WICED_MEMCPY(p_pset->name,  p_rec->name.str, p_pset->name_len);
            p_pset->name[p_pset->name_len] = '\0';
            le_audio_rpc_send_preset_record(conn_id, p_pset_changed->preset_index, &p_pset_changed->preset_rec.name);
        }
        else
        {
            hap_handle_add_preset_rec_to_list(conn_id, p_pset_changed->prev_index, &p_pset_changed->preset_rec);
        }
    }
        break;
    case WICED_BT_GA_HAS_PRESET_RECORD_DELETED:
        hap_handle_delete_preset_rec(conn_id, p_pset_changed->preset_index);
        break;
    case WICED_BT_GA_HAS_PRESET_RECORD_AVAILABLE: {
            int index = get_position_of_preset_in_list(conn_id, p_pset_changed->preset_index);
            if (index != -1)
            {
                p_has->preset_rec_list[index].properties |= 2;
            }
        }
        break;
    case WICED_BT_GA_HAS_PRESET_RECORD_UNAVAILABLE: {
            int index = get_position_of_preset_in_list(conn_id, p_pset_changed->preset_index);
            if (index != -1)
            {
                p_has->preset_rec_list[index].properties &= ~(2);
            }
        }
        break;
    }

}
wiced_result_t lepl_handle_has_data(uint16_t conn_id,
                                    void *p_app_ctx,
                                    const gatt_intf_service_object_t *p_service,
                                    wiced_bt_gatt_status_t status,
                                    uint32_t evt_type,
                                    gatt_intf_attribute_t *p_char,
                                    wiced_bt_ga_has_evt_data_t *p_data_ptr,
                                    int len)
{
    wiced_bt_ga_has_evt_data_t *p_event_data = (wiced_bt_ga_has_evt_data_t *)p_data_ptr;

    WICED_BT_TRACE("[%s] event %x status %x \n", __FUNCTION__, evt_type, status);

    if (evt_type == WRITE_CMPL_EVT)
    {
        return WICED_BT_SUCCESS;
    }

    if (status == WICED_BT_GATT_SUCCESS)
    {
        lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
        switch (p_char->characteristic_type)
        {
        case HAS_HEARING_AID_FEATURES_CHARACTERISTIC:
            p_clcb->has_data.hearing_aid_features = p_event_data->hearing_aid_feature;
            break;
        case HAS_HEARING_AID_PRESET_CONTROL_POINT_CHARACTERISTIC:
            switch (p_event_data->opcode)
            {
            case WICED_BT_GA_HAS_OPCODE_READ_PRESET_RESPONSE:
                hap_handle_read_preset_response(conn_id, &p_event_data->read_rsp_prest);
                break;
            case WICED_BT_GA_HAS_OPCODE_PRESET_CHANGED:
                hap_handle_preset_record_changed(conn_id, &p_event_data->preset_changed);
                break;
            }
            break;
        case HAS_ACTIVE_PRESET_INDEX_CHARACTERISTIC:
            p_clcb->has_data.active_preset_index = p_event_data->active_preset_index;
            le_audio_rpc_update_active_preset(conn_id, p_event_data->active_preset_index);
            break;
        }
    }
    return WICED_BT_SUCCESS;
}

wiced_result_t lepl_has_callback(uint16_t conn_id,
                                 void *p_app_ctx,
                                 gatt_intf_service_object_t *p_service,
                                 wiced_bt_gatt_status_t status,
                                 uint32_t evt_type,
                                 gatt_intf_attribute_t *p_char,
                                 void *p_data,
                                 int len)
{
    WICED_BT_TRACE("[%s] event 0x%x \n", __FUNCTION__, p_char);

    switch (evt_type)
    {
    case WRITE_CMPL_EVT:
        break;

    case READ_CMPL_EVT:
    case NOTIFICATION_EVT:
    case INDICATION_EVT:
        return lepl_handle_has_data(conn_id,
                                    p_app_ctx,
                                    p_service,
                                    status,
                                    evt_type,
                                    p_char,
                                    (wiced_bt_ga_has_evt_data_t *)p_data,
                                    len);
        break;
    }
    return WICED_BT_SUCCESS;
}
