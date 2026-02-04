/*
 * $ Copyright Cypress Semiconductor $
 */
#include "lehs.h"

#ifndef MULTIPLEX_AUDIO_AUPPORTED
#define MULTIPLEX_AUDIO_SUPPORTED 1
#endif


// left earbud with a microphone
static const wiced_bt_ga_pacs_record_t sink_pac_records_supported[] = {{
    .codec_id =
        {
            .coding_format = LC3_CODEC_ID,
            .company_id = 0,
            .vendor_specific_codec_id = 0,
        },
    .codec_specific_capabilities = {0x03,
                                    BAP_CODEC_CAPABILITIES_SUPPORTED_SAMPLING_FREQUENCIES_TYPE,
                                    (BAP_SUPPORTED_SAMPLING_FREQ_8_KHZ | BAP_SUPPORTED_SAMPLING_FREQ_16_KHZ |
                                    BAP_SUPPORTED_SAMPLING_FREQ_24_KHZ | BAP_SUPPORTED_SAMPLING_FREQ_32_KHZ |
                                    BAP_SUPPORTED_SAMPLING_FREQ_44_1_KHZ | BAP_SUPPORTED_SAMPLING_FREQ_48_KHZ),
                                    0x00,
                                    0x02,
                                    BAP_CODEC_CAPABILITIES_SUPPORTED_FRAME_DURATIONS_TYPE,
                                    (BAP_SUPPORTED_FRAME_DURATION_10MS | BAP_SUPPORTED_FRAME_DURATION_7_5MS ),
                                    0x02,
                                    BAP_CODEC_CAPABILITIES_SUPPORTED_AUDIO_CHANNEL_COUNTS_TYPE,
    #if defined MULTIPLEX_AUDIO_SUPPORTED && (MULTIPLEX_AUDIO_SUPPORTED == 1)
                                    0x03, // Channel count greater than one to support multiplex audio
    #else
                                    0x01, // Supports mono audio
    #endif
                                    0x05,
                                    BAP_CODEC_CAPABILITIES_SUPPORTED_OCTETS_PER_CODEC_FRAME_TYPE,
                                    0x1A,
                                    0x00,
                                    MAX_SUPPORTED_OCTETS_PER_CODEC_FRAME,
                                    0x00},
    .codec_specific_capabilities_length = 16,
    .metadata_length = 4,
    .metadata =
        {0x3, BAP_METADATA_PREFERRED_AUDIO_CONTEXTS_TYPE,
        BAP_CONTEXT_TYPE_MEDIA | BAP_CONTEXT_TYPE_UNSPECIFIED,
        0x02}, //BAP_CONTEXT_TYPE_RINGTONE 0x0200
}
};

static const wiced_bt_ga_pacs_record_t source_pac_records_supported[] = {{
    .codec_id =
        {
            .coding_format = LC3_CODEC_ID,
            .company_id = 0,
            .vendor_specific_codec_id = 0,
        },
    .codec_specific_capabilities = {0x03,
                                    BAP_CODEC_CAPABILITIES_SUPPORTED_SAMPLING_FREQUENCIES_TYPE,
                                    (BAP_SUPPORTED_SAMPLING_FREQ_8_KHZ | BAP_SUPPORTED_SAMPLING_FREQ_16_KHZ |
                                     BAP_SUPPORTED_SAMPLING_FREQ_24_KHZ | BAP_SUPPORTED_SAMPLING_FREQ_32_KHZ |
                                     BAP_SUPPORTED_SAMPLING_FREQ_48_KHZ),
                                    0x00,
                                    0x02,
                                    BAP_CODEC_CAPABILITIES_SUPPORTED_FRAME_DURATIONS_TYPE,
                                    ( BAP_SUPPORTED_FRAME_DURATION_10MS | BAP_SUPPORTED_FRAME_DURATION_7_5MS ),
                                    0x02,
                                    BAP_CODEC_CAPABILITIES_SUPPORTED_AUDIO_CHANNEL_COUNTS_TYPE,
                                    0x03,
                                    0x05,
                                    BAP_CODEC_CAPABILITIES_SUPPORTED_OCTETS_PER_CODEC_FRAME_TYPE,
                                    0x1A,
                                    0x00,
                                    MAX_SUPPORTED_OCTETS_PER_CODEC_FRAME,
                                    0x00},
    .codec_specific_capabilities_length = 16,
    .metadata_length = 4,
    .metadata = {0x3,
                 BAP_METADATA_PREFERRED_AUDIO_CONTEXTS_TYPE,
                 BAP_CONTEXT_TYPE_CONVERSATIONAL | BAP_CONTEXT_TYPE_UNSPECIFIED,
                 0},
}};

wiced_bt_ga_pacs_data_t lehs_pacs_app_data = {
    .source_pac_list = {1, (wiced_bt_ga_pacs_record_t *)source_pac_records_supported},
    .source_audio_location = BAP_AUDIO_LOCATION_FRONT_LEFT,
    .sink_pac_list = {1, (wiced_bt_ga_pacs_record_t *)sink_pac_records_supported},
    .sink_audio_location = (BAP_AUDIO_LOCATION_FRONT_LEFT | BAP_AUDIO_LOCATION_FRONT_RIGHT),
    .supported = {.source_contexts = 0x0FFF, .sink_contexts = 0x0FFF}, // 0x0FFF : Supports all valid context type
    .available = {.source_contexts = 0x0FFF, .sink_contexts = 0x0FFF},
};

wiced_result_t lehs_pacs_handle_read_req_evt(uint16_t conn_id,
                                                     gatt_intf_attribute_t *p_char,
                                                     wiced_bt_ga_pacs_data_t *p_evt_data)
{
    wiced_result_t result = WICED_ERROR;
    wiced_bt_ga_pacs_data_t *p_my_pacs = &lehs_pacs_app_data;
    lehs_clcb_t *p_clcb = lehs_gatt_get_clcb_by_conn_id(conn_id);
    CHECK_FOR_NULL_AND_RETURN_VALUE(p_clcb, result);
    CHECK_FOR_NULL_AND_RETURN_VALUE(p_my_pacs, result);
    if (INCLUDED_SERVICE_NONE == p_char->included_service_type)
    {
        memcpy(p_evt_data, p_my_pacs, sizeof(wiced_bt_ga_pacs_data_t));
        result = WICED_SUCCESS;
    }

    return result;
}

wiced_result_t lehs_pacs_handle_write_req_evt(uint16_t conn_id,
                                                      gatt_intf_attribute_t *p_char,
                                                      wiced_bt_ga_pacs_data_t *p_evt_data)
{
    wiced_result_t result = WICED_ERROR;
    wiced_bt_ga_pacs_data_t *p_my_pacs = &lehs_pacs_app_data;

    WICED_BT_TRACE("[%s] char %d \n", __FUNCTION__, p_char->characteristic_type);
    CHECK_FOR_NULL_AND_RETURN_VALUE(p_my_pacs, result);

    //TODO: Validate the received audio location
    switch (p_char->characteristic_type)
    {
        case PACS_SINK_AUDIO_LOCATIONS_CHARACTERISTIC:
            p_my_pacs->sink_audio_location = p_evt_data->sink_audio_location;
            result = WICED_SUCCESS;
            break;

        case PACS_SOURCE_AUDIO_LOCATIONS_CHARACTERISTIC:
            p_my_pacs->source_audio_location = p_evt_data->source_audio_location;
            result = WICED_SUCCESS;
            break;
        default:
            break;
    }

    return result;
}

wiced_result_t lehs_pacs_callback(uint16_t conn_id,
                                          void *p_app_ctx,
                                          gatt_intf_service_object_t *p_service,
                                          wiced_bt_gatt_status_t status,
                                          uint32_t evt_type,
                                          gatt_intf_attribute_t *p_char,
                                          void *p_data,
                                          int len)
{
    wiced_result_t result = WICED_SUCCESS;
    WICED_BT_TRACE("[%s] evt %d", __FUNCTION__, evt_type);

    switch (evt_type)
    {
        case WRITE_REQ_EVT:
            result = lehs_pacs_handle_write_req_evt(conn_id, p_char, p_data);
            break;
        case READ_REQ_EVT:
            result = lehs_pacs_handle_read_req_evt(conn_id, p_char, p_data);
            break;
        default:
            WICED_BT_TRACE("[%s] unsupported event %d received\n", __FUNCTION__, evt_type);
            break;
    }
    return result;
}

void lehs_pacs_init_data(void)
{
    g_lehs_gatt_cb.p_pacs_data = &lehs_pacs_app_data;
}

void lehs_pacs_alloc_memory()
{
    // nothing to do
}

void lehs_set_audio_location(uint32_t audio_location)
{
    lehs_pacs_app_data.sink_audio_location = audio_location;
    lehs_pacs_app_data.source_audio_location = audio_location;
}
