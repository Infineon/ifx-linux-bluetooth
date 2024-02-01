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

#pragma once

#include "wiced_bt_isoc.h"
#include "wiced_bt_types.h"

/* App Library includes */
#include "wiced_bt_ga_ascs.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_mcs.h"
#include "wiced_bt_ga_mcs_common.h"
#include "wiced_bt_ga_pacs.h"
#include "wiced_bt_ga_vcs.h"
#include "unicast_sink_rpc.h"

#define UNICAST_SINK_EXT_ADV_HANDLE 1
#define MAX_CONNECTION_INSTANCE 2

typedef struct
{
    uint16_t cis_conn_hdl;              // cis connection handle
    uint8_t lc3_index;                  // lc3 codec index ?  What is this one exactly ?
    uint32_t data_path_established : 1; // data path established ?
    wiced_bt_ga_ascs_ase_t data;
} unicast_sink_ase_data_t;

typedef struct
{
    gatt_intf_service_object_t *p_pacs;
    gatt_intf_service_object_t *p_ascs;
    gatt_intf_service_object_t *p_vcs;
} unicast_sink_local_profiles_t;

typedef struct
{
    gatt_intf_service_object_t *p_mcs;
    gatt_intf_service_object_t *p_gmcs;
} unicast_sink_peer_profiles_t;

typedef struct
{
    char media_player_name[MAX_MEDIA_PLAYER_NAME_LEN];       /**< Media Player Name */
    char track_title[MAX_MEDIA_TRACK_TITLE_LEN];             /**< Track Title */
    int32_t track_duration;                                  /**< Track Duration */
    int32_t track_position;                                  /**< Track Position */
    int8_t playback_speed;                                   /**< Playback Speed */
    int8_t seeking_speed;                                    /**< Seeking Speed */
    wiced_bt_ga_media_control_playing_order_t playing_order; /**< Playing Order */
    uint16_t playing_order_supported;                        /**< Playing Order Supported bit field */
    wiced_bt_ga_media_control_state_t media_state;           /**< Media State */
    uint32_t media_control_supported_opcodes;                /**< Media Control Supported Opcodes */
    uint8_t content_control_id;                              /**< Content Control ID */
} unicast_sink_mcs_data_t;

typedef struct
{
    uint8_t in_use;
    uint16_t conn_id;
    uint32_t addr_type;
    wiced_bt_device_address_t bda;
    wiced_bool_t b_is_central;
    uint32_t num_local_ases;
    unicast_sink_peer_profiles_t peer_profiles;
    wiced_bt_ga_pacs_data_t pacs_data;
    unicast_sink_ase_data_t *p_local_ase_data;
    unicast_sink_mcs_data_t mcs_data;
} unicast_sink_clcb_t;

typedef struct
{
    wiced_bt_ga_vcs_control_point_t cp;
    wiced_bt_ga_vcs_volume_state_t state;
    uint8_t flag;
} unicast_sink_volume_t;

typedef struct
{
    unicast_sink_local_profiles_t local_profiles;
    unicast_sink_volume_t vcs_data;
    unicast_sink_clcb_t unicast_clcb[MAX_CONNECTION_INSTANCE];
} unicast_sink_gatt_cb_t;

extern unicast_sink_gatt_cb_t g_unicast_sink_gatt_cb;

wiced_bt_gatt_status_t unicast_sink_gatt_init(int max_connections, int max_mtu, ga_cfg_t *p_ga_cfg);

void unicast_sink_gatt_start_discovery(uint8_t *p_bd_addr);

void unicast_sink_gatt_start_stop_adv(uint32_t b_start);

extern wiced_result_t unicast_sink_mcs_callback(uint16_t conn_id,
                                                void *p_app_ctx,
                                                const gatt_intf_service_object_t *p_service,
                                                wiced_bt_gatt_status_t status,
                                                uint32_t evt_type,
                                                gatt_intf_attribute_t *p_char,
                                                void *p_data,
                                                int len);

extern wiced_result_t unicast_sink_ascs_callback(uint16_t conn_id,
                                                 void *p_app_ctx,
                                                 const gatt_intf_service_object_t *p_service,
                                                 wiced_bt_gatt_status_t status,
                                                 uint32_t evt_type,
                                                 gatt_intf_attribute_t *p_char,
                                                 void *p_data,
                                                 int len);

extern wiced_result_t unicast_sink_pacs_callback(uint16_t conn_id,
                                                 void *p_app_ctx,
                                                 const gatt_intf_service_object_t *p_service,
                                                 wiced_bt_gatt_status_t status,
                                                 uint32_t evt_type,
                                                 gatt_intf_attribute_t *p_char,
                                                 void *p_data,
                                                 int len);

extern wiced_result_t unicast_sink_vcs_callback(uint16_t conn_id,
                                                void *p_app_ctx,
                                                const gatt_intf_service_object_t *p_service,
                                                wiced_bt_gatt_status_t status,
                                                uint32_t evt_type,
                                                gatt_intf_attribute_t *p_char,
                                                void *p_data,
                                                int len);

void unicast_sink_ascs_alloc_memory(ga_cfg_t *p_cfg);
void unicast_sink_pacs_alloc_memory(ga_cfg_t *p_cfg);
void unicast_sink_ascs_init_data(unicast_sink_clcb_t *p_clcb);
void unicast_sink_pacs_init_data(unicast_sink_clcb_t *p_clcb);
void unicast_sink_vcs_initialize_data(void);

extern unicast_sink_clcb_t *unicast_sink_gatt_get_clcb_by_conn_id(uint16_t conn_id);
gatt_intf_attribute_t *ascs_init_characteristic(gatt_intf_attribute_t *p_char,
                                                     unicast_sink_ase_data_t *p_ase);
void unicast_sink_isoc_event_handler(wiced_bt_isoc_event_t event, wiced_bt_isoc_event_data_t *p_event_data);
void iso_audio_init(const wiced_bt_cfg_isoc_t *p_iso_cfg);
