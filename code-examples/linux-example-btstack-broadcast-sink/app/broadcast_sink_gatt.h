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
#include "broadcast_sink_bass.h"
#include "wiced_bt_types.h"

/* App Library includes */
#include "wiced_bt_ga_common.h"

typedef struct
{
    gatt_intf_service_object_t *p_pacs;
    gatt_intf_service_object_t *p_bass;
} profiles_t;

typedef struct
{
    uint8_t in_use;
    uint16_t conn_id;
    uint32_t addr_type;
    wiced_bt_device_address_t bda;
    wiced_bool_t b_is_central;
} broadcast_sink_gatt_cb_t;

wiced_bt_gatt_status_t broadcast_sink_gatt_init(int max_connections, int max_mtu, ga_cfg_t *p_ga_cfg);
broadcast_sink_gatt_cb_t *broadcast_sink_gatt_get_cb_by_conn_id(uint16_t conn_id);
gatt_intf_service_object_t *broadcast_sink_gatt_get_bass_service_instance();
