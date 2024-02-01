/*
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*/

#include "unicast_sink_gatt.h"

#include "wiced_bt_ga_bap.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_pacs.h"

#include "wiced_memory.h"

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
                                    0x03,
                                    0x05,
                                    BAP_CODEC_CAPABILITIES_SUPPORTED_OCTETS_PER_CODEC_FRAME_TYPE,
                                    0x1A,
                                    0x00,
                                    0xA0,
                                    0x00},
    .codec_specific_capabilities_length = 16,
    .metadata_length = 4,
    .metadata = {0x3, BAP_METADATA_PREFERRED_AUDIO_CONTEXTS_TYPE, BAP_CONTEXT_TYPE_MEDIA, 0},
}};

static const wiced_bt_ga_pacs_record_t source_pac_records_supported[] = {{
    .codec_id =
        {
            .coding_format = LC3_CODEC_ID,
            .company_id = 0,
            .vendor_specific_codec_id = 0,
        },
    .codec_specific_capabilities = {0x03,
                                    BAP_CODEC_CAPABILITIES_SUPPORTED_SAMPLING_FREQUENCIES_TYPE,
                                    BAP_SUPPORTED_SAMPLING_FREQ_16_KHZ,
                                    0x00,
                                    0x02,
                                    BAP_CODEC_CAPABILITIES_SUPPORTED_FRAME_DURATIONS_TYPE,
                                    BAP_SUPPORTED_FRAME_DURATION_10MS,
                                    0x02,
                                    BAP_CODEC_CAPABILITIES_SUPPORTED_AUDIO_CHANNEL_COUNTS_TYPE,
                                    0x03,
                                    0x05,
                                    BAP_CODEC_CAPABILITIES_SUPPORTED_OCTETS_PER_CODEC_FRAME_TYPE,
                                    0x28,
                                    0x00,
                                    0x78,
                                    0x00},
    .codec_specific_capabilities_length = 16,
    .metadata_length = 4,
    .metadata = {0x3, BAP_METADATA_PREFERRED_AUDIO_CONTEXTS_TYPE, BAP_CONTEXT_TYPE_CONVERSATIONAL, 0},
}};

// clang-format on
wiced_bt_ga_pacs_data_t unicast_sink_pacs_app_data = {
    .source_pac_list = {0, NULL},
    .sink_pac_list = {1, (wiced_bt_ga_pacs_record_t *)sink_pac_records_supported},
    .sink_audio_location = (BAP_AUDIO_LOCATION_FRONT_LEFT),
    .supported = {.source_contexts = BAP_CONTEXT_TYPE_CONVERSATIONAL, .sink_contexts = BAP_CONTEXT_TYPE_MEDIA},
    .available = {BAP_CONTEXT_TYPE_CONVERSATIONAL, BAP_CONTEXT_TYPE_MEDIA},
};

/******************************************************************************
 * Function Name: unicast_sink_pacs_handle_read_req_evt 
 ******************************************************************************
 * Summary: pacs handle read request
 *
 * Parameters:
 *  uint16_t conn_id
 *  gatt_intf_attribute_t *p_char
 *  wiced_bt_ga_pacs_data_t *p_evt_data
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_sink_pacs_handle_read_req_evt(uint16_t conn_id,
                                                     gatt_intf_attribute_t *p_char,
                                                     wiced_bt_ga_pacs_data_t *p_evt_data)
{
    wiced_result_t result = WICED_ERROR;
    wiced_bt_ga_pacs_data_t *pacs_app_data_ptr = NULL;
    unicast_sink_clcb_t *p_clcb = unicast_sink_gatt_get_clcb_by_conn_id(conn_id);
    CHECK_FOR_NULL_AND_RETURN_VALUE(p_clcb, result);

    pacs_app_data_ptr = &p_clcb->pacs_data;
    CHECK_FOR_NULL_AND_RETURN_VALUE(pacs_app_data_ptr, result);

    if (INCLUDED_SERVICE_NONE == p_char->included_service_type)
    {
        memcpy(p_evt_data, pacs_app_data_ptr, sizeof(wiced_bt_ga_pacs_data_t));
        result = WICED_SUCCESS;
    }

    return result;
}

/******************************************************************************
 * Function Name: unicast_sink_pacs_handle_write_req_evt 
 ******************************************************************************
 * Summary: handle pacs write request
 *
 * Parameters:
 *  uint16_t conn_id
 *  gatt_intf_attribute_t *p_char
 *  wiced_bt_ga_pacs_data_t *p_evt_data
 *  None
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_sink_pacs_handle_write_req_evt(uint16_t conn_id,
                                                      gatt_intf_attribute_t *p_char,
                                                      wiced_bt_ga_pacs_data_t *p_evt_data)
{
    wiced_result_t result = WICED_ERROR;
    unicast_sink_clcb_t *p_clcb = unicast_sink_gatt_get_clcb_by_conn_id(conn_id);
    wiced_bt_ga_pacs_data_t *pacs_app_data_ptr = &p_clcb->pacs_data;

    WICED_BT_TRACE("[%s] char %d \n", __FUNCTION__, p_char->characteristic_type);
    CHECK_FOR_NULL_AND_RETURN_VALUE(pacs_app_data_ptr, result);

    //TODO: Validate the received audio location
    switch (p_char->characteristic_type)
    {
        case PACS_SINK_AUDIO_LOCATIONS_CHARACTERISTIC:
            pacs_app_data_ptr->sink_audio_location = p_evt_data->sink_audio_location;
            result = WICED_SUCCESS;
            break;

        default:
            break;
    }

    return result;
}

/******************************************************************************
 * Function Name: unicast_sink_pacs_callback 
 ******************************************************************************
 * Summary: pacs callback use in unicast_sink_store_service_ref_local
 *
 * Parameters:
 *  uint16_t conn_id
 *  void *p_app_ctx
 *  const gatt_intf_service_object_t *p_service
 *  wiced_bt_gatt_status_t status
 *  uint32_t evt_type
 *  gatt_intf_attribute_t *p_char
 *  void *p_data
 *  int len
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_sink_pacs_callback(uint16_t conn_id,
                                          void *p_app_ctx,
                                          const gatt_intf_service_object_t *p_service,
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
            result = unicast_sink_pacs_handle_write_req_evt(conn_id, p_char, p_data);
            break;
        case READ_REQ_EVT:
            result = unicast_sink_pacs_handle_read_req_evt(conn_id, p_char, p_data);
            break;
        default:
            WICED_BT_TRACE("[%s] unsupported event %d received\n", __FUNCTION__, evt_type);
            break;
    }
    return result;
}

/******************************************************************************
 * Function Name: unicast_sink_pacs_init_data
 ******************************************************************************
 * Summary: init sink device pacs data
 *
 * Parameters:
 *  unicast_sink_clcb_t *p_clcb
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_pacs_init_data(unicast_sink_clcb_t *p_clcb)
{
#define SINK_PAC_RECORD_LIST(x) x->sink_pac_list.record_list
#define SOURCE_PAC_RECORD_LIST(x) x->source_pac_list.record_list
    wiced_bt_ga_pacs_data_t *g_pacs_app_data_ptr = &unicast_sink_pacs_app_data;

    wiced_bt_ga_pacs_data_t *pacs_app_data_ptr = &p_clcb->pacs_data;
    if (!pacs_app_data_ptr) return;

    WICED_BT_TRACE("[%s] sink_rec_cnt %d src_rec_cnt %d\n",
                   __FUNCTION__,
                   g_pacs_app_data_ptr->sink_pac_list.num_of_records,
                   g_pacs_app_data_ptr->source_pac_list.num_of_records);

    pacs_app_data_ptr->sink_pac_list.num_of_records = g_pacs_app_data_ptr->sink_pac_list.num_of_records;
    WICED_MEMCPY(SINK_PAC_RECORD_LIST(pacs_app_data_ptr),
                 SINK_PAC_RECORD_LIST(g_pacs_app_data_ptr),
                 sizeof(wiced_bt_ga_pacs_record_t) * g_pacs_app_data_ptr->sink_pac_list.num_of_records);

    pacs_app_data_ptr->source_pac_list.num_of_records = 0;
    pacs_app_data_ptr->source_audio_location = 0;

    pacs_app_data_ptr->sink_audio_location = g_pacs_app_data_ptr->sink_audio_location;
    pacs_app_data_ptr->available = g_pacs_app_data_ptr->available;
    pacs_app_data_ptr->supported = g_pacs_app_data_ptr->supported;
}

/******************************************************************************
 * Function Name: unicast_sink_pacs_alloc_memory
 ******************************************************************************
 * Summary: allocate memory for pacs at gatt_init
 *
 * Parameters:
 *  ga_cfg_t *p_cfg
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_pacs_alloc_memory(ga_cfg_t *p_cfg)
{
    int index;
    int req_size = (p_cfg->pacs_max_sink_capabilities_supported + p_cfg->pacs_max_source_capabilities_supported) *
                   sizeof(wiced_bt_ga_pacs_record_t) * MAX_CONNECTION_INSTANCE;
    wiced_bt_ga_pacs_record_t *p_pacs_data = wiced_memory_alloc_long_term_mem_block(req_size, "PACS");

    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        if (p_cfg->pacs_max_sink_capabilities_supported)
        {
            g_unicast_sink_gatt_cb.unicast_clcb[index].pacs_data.sink_pac_list.record_list = p_pacs_data;
            p_pacs_data += p_cfg->pacs_max_sink_capabilities_supported;
        }
        if (p_cfg->pacs_max_source_capabilities_supported)
        {
            g_unicast_sink_gatt_cb.unicast_clcb[index].pacs_data.source_pac_list.record_list = p_pacs_data;
            p_pacs_data += p_cfg->pacs_max_source_capabilities_supported;
        }
    }
}
