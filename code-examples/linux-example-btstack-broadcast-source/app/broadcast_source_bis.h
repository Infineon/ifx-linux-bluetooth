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

/******************************************************************************
 * File Name: broadcast_source_bis.h
 *
 * Description: This is the header file for Broadcast_source CE application.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

#pragma once

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

    // controller info
    wiced_bt_ble_periodic_adv_sync_handle_t sync_handle;
    uint8_t number_of_subevents; // for sink only (received in BIGInfo)
    uint8_t bis_conn_id_count;
    uint16_t bis_conn_id_list[BROADCAST_MAX_BIS_PER_SUB_GROUP * BROADCAST_MAX_SUB_GROUP];
    uint8_t bis_index_list[BROADCAST_MAX_BIS_PER_SUB_GROUP * BROADCAST_MAX_SUB_GROUP];
    wiced_bool_t b_encryption;
    wiced_bt_bap_broadcast_code_t broadcast_code;

    // profile info
    wiced_bt_ga_bap_broadcast_base_t base;
} broadcast_source_cb_t;

enum
{
    CODEC_CONFIG_16_2,
    // CODEC_CONFIG_48_1,
    CODEC_CONFIG_48_2,
    // CODEC_CONFIG_48_3,
    CODEC_CONFIG_48_4,
    // CODEC_CONFIG_48_5,
    CODEC_CONFIG_48_6,
};

void broadcast_source_bis_init(wiced_bt_cfg_isoc_t *p_isoc_cfg);
void broadcast_source_bis_start_stream(wiced_bt_ga_bap_stream_config_t *p_stream_config);
wiced_result_t broadcast_source_bis_disable_stream(void);
wiced_result_t broadcast_source_bis_release_stream(void);
wiced_result_t broadcast_source_bis_configure_stream(uint32_t broadcast_id,
                                                     uint8_t *broadcast_code,
                                                     uint8_t bis_count,
                                                     uint32_t channel_counts,
                                                     uint32_t sampling_freq,
                                                     uint32_t frame_duration,
                                                     uint16_t octets_per_codec_frame,
                                                     wiced_bool_t enable_encryption);
