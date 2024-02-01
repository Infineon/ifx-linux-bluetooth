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

/******************************************************************************/

#pragma once

#include <stdbool.h>
#include "wiced_bt_types.h"
#include "wiced_bt_ga_bap_broadcast.h"

typedef struct
{
    // app info
    wiced_bool_t in_use;
    wiced_bt_device_address_t bd_addr;
    uint8_t big_handle;
    uint8_t adv_handle;
    wiced_bool_t b_base_updated;
    wiced_bool_t b_biginfo_updated;
    uint8_t *p_periodic_adv_data;
    uint8_t periodic_adv_data_offset;
    uint8_t base_len;

    // controller info
    wiced_bt_ble_periodic_adv_sync_handle_t sync_handle;
    wiced_bool_t sync_in_progress;
    uint8_t number_of_subevents; // for sink only (received in BIGInfo)
    uint8_t bis_conn_id_count;
    uint16_t bis_conn_id_list[BROADCAST_MAX_BIS_PER_SUB_GROUP * BROADCAST_MAX_SUB_GROUP];
    uint8_t bis_index_list[BROADCAST_MAX_BIS_PER_SUB_GROUP * BROADCAST_MAX_SUB_GROUP];
    wiced_bool_t b_encryption;
    wiced_bt_bap_broadcast_code_t broadcast_code;

    // profile info
    wiced_bt_ga_bap_broadcast_base_t base;
} broadcast_sink_cb_t;

typedef struct
{
    uint32_t broadcast_id;
    wiced_bt_bap_broadcast_code_t broadcast_code;
} broadcast_source_t;

void broadcast_sink_bis_menu_discover_sources(bool start);
void broadcast_sink_bis_menu_sync_to_source(uint32_t broadcast_id, bool listen, uint8_t* broadcast_code);
void broadcast_sink_bis_init(wiced_bt_cfg_isoc_t *p_isoc_cfg);
void broadcast_sink_bis_discover_sources(wiced_bt_ble_scan_type_t scan_type);
void broadcast_sink_clear_data();
void broadcast_sink_bis_sync_to_source(wiced_bt_ble_scan_type_t scan_type, broadcast_source_t source);

broadcast_sink_cb_t *broadcast_sink_bis_get_big_by_broadcast_id(uint32_t br_id);

broadcast_sink_cb_t *broadcast_sink_bis_alloc_big(uint32_t broadcast_id,
                                                  wiced_bt_device_address_t bd_addr,
                                                  uint8_t adv_sid);

void broadcast_sink_bis_free_big(broadcast_sink_cb_t *p_big);

broadcast_sink_cb_t *broadcast_sink_bis_get_big_by_sync_handle(wiced_bt_ble_periodic_adv_sync_handle_t sync_handle);
