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

#include "unicast_source_gatt.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_pacs.h"


/******************************************************************************
 * Function Name: unicast_source_pacs_handle_read_complete_opeartion
 *
 * Summary: handle pacs read complete
 *          support pacs characteristic:
 *              PACS_AVAILABILE_AUDIO_CONTEXTS_CHARACTERISTIC 
 *              PACS_SUPPORTED_AUDIO_CONTEXTS_CHARACTERISTIC
 *              PACS_SINK_AUDIO_LOCATIONS_CHARACTERISTIC
 *              PACS_SOURCE_AUDIO_LOCATIONS_CHARACTERISTIC
 *              PACS_SINK_CAPABILITY_CHARACTERISTIC
 *              PACS_SOURCE_CAPABILITY_CHARACTERISTIC
 *
 * Parameters:
 *  uint16_t                conn_id
 *  pacs_characteristics_t  pacs_char
 *  wiced_bt_ga_pacs_data_t *p_evt_data
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_pacs_handle_read_complete_opeartion(uint16_t conn_id,
                                                               pacs_characteristics_t pacs_char,
                                                               wiced_bt_ga_pacs_data_t *p_evt_data)
{
    unicast_source_clcb_t *p_clcb = (unicast_source_clcb_t *)unicast_source_gatt_get_clcb_by_conn_id(conn_id);
    int current_num_records = 0;
    if (!p_clcb) return;

    WICED_BT_TRACE("[%s] p_clcb 0x%x cap 0x%x pacs_data 0x%x pacs_char %d",
                   __FUNCTION__,
                   p_clcb,
                   &(p_clcb->cap_data),
                   &(p_clcb->cap_data->pacs_data),
                   pacs_char);

    switch (pacs_char) {
        case PACS_AVAILABILE_AUDIO_CONTEXTS_CHARACTERISTIC:
            WICED_BT_TRACE("AVAILABILE Sink %d Source %d",
                           p_evt_data->available.sink_contexts,
                           p_evt_data->available.source_contexts);
            p_clcb->cap_data->pacs_data->available.sink_contexts = p_evt_data->available.sink_contexts;
            p_clcb->cap_data->pacs_data->available.source_contexts = p_evt_data->available.source_contexts;
            break;

        case PACS_SUPPORTED_AUDIO_CONTEXTS_CHARACTERISTIC:
            WICED_BT_TRACE("Supported Sink %d Source %d",
                           p_evt_data->supported.sink_contexts,
                           p_evt_data->supported.source_contexts);
            p_clcb->cap_data->pacs_data->supported.sink_contexts = p_evt_data->supported.sink_contexts;
            p_clcb->cap_data->pacs_data->supported.source_contexts = p_evt_data->supported.source_contexts;
            break;

        case PACS_SINK_AUDIO_LOCATIONS_CHARACTERISTIC:
            p_clcb->cap_data->pacs_data->sink_audio_location = p_evt_data->sink_audio_location;
            break;

        case PACS_SOURCE_AUDIO_LOCATIONS_CHARACTERISTIC:
            p_clcb->cap_data->pacs_data->source_audio_location = p_evt_data->source_audio_location;
            break;

        case PACS_SINK_CAPABILITY_CHARACTERISTIC: {
            WICED_BT_TRACE("SINK PACS num_of_records %d", p_evt_data->sink_pac_list.num_of_records);
            current_num_records = p_clcb->cap_data->pacs_data->sink_pac_list.num_of_records;

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
                p_clcb->cap_data->pacs_data->sink_pac_list.record_list)
            {
                WICED_MEMCPY(&p_clcb->cap_data->pacs_data->sink_pac_list.record_list[current_num_records],
                             p_evt_data->sink_pac_list.record_list,
                             sizeof(wiced_bt_ga_pacs_record_t) * p_evt_data->sink_pac_list.num_of_records);
                p_clcb->cap_data->pacs_data->sink_pac_list.num_of_records += p_evt_data->sink_pac_list.num_of_records;
            }
        } break;

        case PACS_SOURCE_CAPABILITY_CHARACTERISTIC: {
            WICED_BT_TRACE("SRC PACS num_of_records %d", p_evt_data->source_pac_list.num_of_records);
            current_num_records = p_clcb->cap_data->pacs_data->source_pac_list.num_of_records;

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
                p_clcb->cap_data->pacs_data->source_pac_list.record_list)
            {
                WICED_BT_TRACE("SRC PACS num_of_records %x ", p_clcb->cap_data->pacs_data->source_pac_list.record_list);
                WICED_MEMCPY(&p_clcb->cap_data->pacs_data->source_pac_list.record_list[current_num_records],
                             p_evt_data->source_pac_list.record_list,
                            sizeof(wiced_bt_ga_pacs_record_t) * p_evt_data->source_pac_list.num_of_records);
                p_clcb->cap_data->pacs_data->source_pac_list.num_of_records +=
                        p_evt_data->source_pac_list.num_of_records;

            }
        } break;
    }
}

/******************************************************************************
 * Function Name: unicast_source_pacs_callback
 *
 * Summary: source pacs callback function
 *
 * Parameters:
 *  uint16_t conn_id
 *  void *p_app_ctx,
 *  const gatt_intf_service_object_t *p_service,
 *  wiced_bt_gatt_status_t status,
 *  uint32_t evt_type,
 *  gatt_intf_attribute_t *p_char,
 *  void *p_data,
 *  int len
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_source_pacs_callback(uint16_t conn_id,
                                            void *p_app_ctx,
                                            const gatt_intf_service_object_t *p_service,
                                            wiced_bt_gatt_status_t status,
                                            uint32_t evt_type,
                                            gatt_intf_attribute_t *p_char,
                                            void *p_data,
                                            int len)
{
    switch (evt_type) {
        case WRITE_CMPL_EVT:
            WICED_BT_TRACE("[%s] WRITE_CMPL_EVT characteristic %x\n", __FUNCTION__, p_char->characteristic_type);
            break;
        case READ_CMPL_EVT:
        case NOTIFICATION_EVT:
            WICED_BT_TRACE("[%s] Read/NOTIFICATION_EVT characteristic %d\n", __FUNCTION__, p_char->characteristic_type);
            unicast_source_pacs_handle_read_complete_opeartion(conn_id, p_char->characteristic_type, p_data);
            break;
        default:
            WICED_BT_TRACE("[%s] event %d", __FUNCTION__, p_char);
            break;
    }
    return WICED_BT_SUCCESS;
}
