/*
 * $ Copyright Cypress Semiconductor $
 */
#include "lepl.h"


static void lepl_pacs_handle_read_complete_opeartion(uint16_t conn_id,
                                                     pacs_characteristics_t pacs_char,
                                                     wiced_bt_ga_pacs_data_t *p_evt_data)
{
    lepl_clcb_t *p_clcb = (lepl_clcb_t *)lepl_gatt_get_clcb_by_conn_id(conn_id);
    int current_num_records = 0;
    if (!p_clcb) return;

    WICED_BT_TRACE("[%s] p_clcb 0x%x cap 0x%x pacs_data 0x%x pacs_char %d",
                   __FUNCTION__,
                   p_clcb,
                   &(p_clcb->p_cap),
                   &(p_clcb->p_cap->p_pacs_data),
                   pacs_char);

    switch (pacs_char)
    {
    case PACS_AVAILABILE_AUDIO_CONTEXTS_CHARACTERISTIC:
        WICED_BT_TRACE("AVAILABILE Sink %d Source %d",
                       p_evt_data->available.sink_contexts,
                       p_evt_data->available.source_contexts);
        p_clcb->p_cap->p_pacs_data->available.sink_contexts = p_evt_data->available.sink_contexts;
        p_clcb->p_cap->p_pacs_data->available.source_contexts = p_evt_data->available.source_contexts;
        break;

    case PACS_SUPPORTED_AUDIO_CONTEXTS_CHARACTERISTIC:
        WICED_BT_TRACE("Supported Sink %d Source %d",
                       p_evt_data->supported.sink_contexts,
                       p_evt_data->supported.source_contexts);
        p_clcb->p_cap->p_pacs_data->supported.sink_contexts = p_evt_data->supported.sink_contexts;
        p_clcb->p_cap->p_pacs_data->supported.source_contexts = p_evt_data->supported.source_contexts;
        break;

    case PACS_SINK_AUDIO_LOCATIONS_CHARACTERISTIC:
        p_clcb->p_cap->p_pacs_data->sink_audio_location = p_evt_data->sink_audio_location;
        break;

    case PACS_SOURCE_AUDIO_LOCATIONS_CHARACTERISTIC:
        p_clcb->p_cap->p_pacs_data->source_audio_location = p_evt_data->source_audio_location;
        break;

    case PACS_SINK_CAPABILITY_CHARACTERISTIC: {
        WICED_BT_TRACE("SINK PACS num_of_records %d", p_evt_data->sink_pac_list.num_of_records);
        current_num_records = p_clcb->p_cap->p_pacs_data->sink_pac_list.num_of_records;

        if (current_num_records == MAX_PACS_SINK_CAP_SUPPORTED)
        {
            WICED_BT_TRACE("SNK PACS num_of_records %d reached max", current_num_records);
            return;
        }
        p_evt_data->sink_pac_list.num_of_records =
            (p_evt_data->sink_pac_list.num_of_records < (MAX_PACS_SINK_CAP_SUPPORTED - current_num_records))
                ? p_evt_data->sink_pac_list.num_of_records
                : MAX_PACS_SINK_CAP_SUPPORTED - current_num_records;

        if (p_evt_data->sink_pac_list.num_of_records && p_evt_data->sink_pac_list.record_list &&
            p_clcb->p_cap->p_pacs_data->sink_pac_list.record_list)
        {
            WICED_MEMCPY(&p_clcb->p_cap->p_pacs_data->sink_pac_list.record_list[current_num_records],
                         p_evt_data->sink_pac_list.record_list,
                         sizeof(wiced_bt_ga_pacs_record_t) * p_evt_data->sink_pac_list.num_of_records);
            p_clcb->p_cap->p_pacs_data->sink_pac_list.num_of_records += p_evt_data->sink_pac_list.num_of_records;
        }
    }
    break;

    case PACS_SOURCE_CAPABILITY_CHARACTERISTIC: {
        WICED_BT_TRACE("SRC PACS num_of_records %d", p_evt_data->source_pac_list.num_of_records);
        current_num_records = p_clcb->p_cap->p_pacs_data->source_pac_list.num_of_records;

        if (current_num_records == MAX_PACS_SOURCE_CAP_SUPPORTED)
        {
            WICED_BT_TRACE("SRC PACS num_of_records %d reached max", current_num_records);
            return;
        }

        p_evt_data->source_pac_list.num_of_records =
            (p_evt_data->source_pac_list.num_of_records < (MAX_PACS_SOURCE_CAP_SUPPORTED - current_num_records))
                ? p_evt_data->source_pac_list.num_of_records
                : MAX_PACS_SOURCE_CAP_SUPPORTED - current_num_records;

        if (p_evt_data->source_pac_list.num_of_records && p_evt_data->source_pac_list.record_list &&
            p_clcb->p_cap->p_pacs_data->source_pac_list.record_list)
        {
            WICED_BT_TRACE("SRC PACS num_of_records %x ", p_clcb->p_cap->p_pacs_data->source_pac_list.record_list);
            WICED_MEMCPY(&p_clcb->p_cap->p_pacs_data->source_pac_list.record_list[current_num_records],
                         p_evt_data->source_pac_list.record_list,
                         sizeof(wiced_bt_ga_pacs_record_t) * p_evt_data->source_pac_list.num_of_records);
            p_clcb->p_cap->p_pacs_data->source_pac_list.num_of_records += p_evt_data->source_pac_list.num_of_records;
        }
    }
    break;
    }
}

wiced_result_t lepl_pacs_callback(uint16_t conn_id,
                                  void *p_app_ctx,
                                  gatt_intf_service_object_t *p_service,
                                  wiced_bt_gatt_status_t status,
                                  uint32_t evt_type,
                                  gatt_intf_attribute_t *p_char,
                                  void *p_data,
                                  int len)
{
    switch (evt_type)
    {
    case WRITE_CMPL_EVT:
        WICED_BT_TRACE("[%s] WRITE_CMPL_EVT characteristic %x\n", __FUNCTION__, p_char->characteristic_type);
        break;
    case READ_CMPL_EVT:
    case NOTIFICATION_EVT:
        WICED_BT_TRACE("[%s] Read/NOTIFICATION_EVT characteristic %d\n", __FUNCTION__, p_char->characteristic_type);
        lepl_pacs_handle_read_complete_opeartion(conn_id, p_char->characteristic_type, p_data);
        break;
    default:
        WICED_BT_TRACE("[%s] event %d", __FUNCTION__, p_char);
        break;
    }
    return WICED_BT_SUCCESS;
}

wiced_bool_t lepl_ccs_pacs_does_peer_support_ringtone(uint16_t conn_id)
{
    lepl_clcb_t *p_clcb = (lepl_clcb_t *)lepl_gatt_get_clcb_by_conn_id(conn_id);

    if (p_clcb->p_cap->p_pacs_data->available.sink_contexts & BAP_CONTEXT_TYPE_RINGTONE)
    {
        WICED_BT_TRACE("[%s] peer supports inband", __FUNCTION__);
        return WICED_TRUE;
    }
    WICED_BT_TRACE("[%s] peer doesnot supports inband", __FUNCTION__);
    return WICED_FALSE;
}
