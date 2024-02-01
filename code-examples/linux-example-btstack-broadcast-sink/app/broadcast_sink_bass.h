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

#include "wiced_bt_types.h"

#include "broadcast_sink_bis.h"
#include "wiced_bt_ga_bass.h"

#define MAX_BASS 2

typedef struct
{
    wiced_bool_t is_used;
    uint16_t conn_id;
    wiced_bool_t waiting_broadcast_code;
    uint16_t sync_handle;
    wiced_bt_ga_bass_receive_state_data_t recv_state_data;
    wiced_bt_ga_bass_sub_group_data_t sub_group_data[WICED_BT_GA_BASS_MAX_SUBGROUP_COUNT]; /**< Subgroup Data */
} broadcast_sink_bass_data_t;

typedef struct
{
    broadcast_sink_bass_data_t bass_data[MAX_BASS];
    wiced_bt_ga_bass_operation_t operation_data;
} broadcast_sink_bass_t;

wiced_result_t broadcast_sink_bass_callback(uint16_t conn_id,
                                            void *p_app_ctx,
                                            const gatt_intf_service_object_t *p_service,
                                            wiced_bt_gatt_status_t status,
                                            uint32_t evt_type,
                                            gatt_intf_attribute_t *p_char,
                                            void *p_data,
                                            int len);

void broadcast_sink_bass_notify_pa_sync_state(wiced_bt_ble_periodic_adv_sync_established_event_data_t *p_sync);
void broadcast_sink_bass_broadcast_code_check(uint16_t sync_handle);
void broadcast_sink_sync_to_source(broadcast_sink_cb_t *p_big, uint8_t *p_brdcst_code);
void broadcast_sink_bass_notify_sync_established(uint8_t *p_addr);
void broadcast_sink_bass_notify_big_sync_lost(uint8_t *p_addr);
void broadcast_sink_bass_notify_pa_sync_lost(uint16_t sync_handle);
broadcast_sink_bass_t *broadcast_sink_bass_get_bass_data();
void broadcast_sink_bass_request_pa_sync_info();
static void broadcast_sink_notify_recv_state_char(uint16_t conn_id,
                                                  const gatt_intf_service_object_t *p_service,
                                                  broadcast_sink_bass_data_t *p_bass_data,
                                                  uint8_t char_instance);
