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

#include "wiced_bt_types.h"
#include "unicast_sink_gatt.h"
#include "wiced_hal_nvram.h"

#define MAX_NUM_DEVICES_IN_NVRAM 2
#define MAX_SERVICES_PER_DEVICE_NVRAM 16

#define UNICAST_APP_MAX_IDS_PER_DEVICE                                                                                  \
    (1 +                                        /* Link keys */                                                        \
     1 +                                        /* Client DB Meta data */                                              \
     1 +                                        /* Server DB Meta data */                                              \
     MAX_SERVICES_PER_DEVICE_NVRAM +            /* Client DB Data */                                                   \
     MAX_SERVICES_PER_DEVICE_NVRAM              /* Server DB Data */                                                   \
    )
#define UNICAST_APP_MAX_IDS_PER_DEVICE_ALIGNED ALIGN_SIZE(UNICAST_APP_MAX_IDS_PER_DEVICE, (1 << 8))

enum
{
    UNICAST_APP_NVRAM_ID_START = (WICED_NVRAM_VSID_START),
    UNICAST_APP_NVRAM_ID_LOCAL_IRK,      // 0x201
    UNICAST_APP_NVRAM_ID_LAST_PAIRED_KEY, // 0x202

    UNICAST_APP_NVRAM_ID_PAIRED_KEYS = ALIGN_SIZE(UNICAST_APP_NVRAM_ID_LAST_PAIRED_KEY + 1, 0x100), // 0x203
    UNICAST_APP_NVRAM_ID_PAIRED_KEYS_MAX =
        (UNICAST_APP_NVRAM_ID_PAIRED_KEYS + MAX_NUM_DEVICES_IN_NVRAM * UNICAST_APP_MAX_IDS_PER_DEVICE_ALIGNED),

    UNICAST_APP_NVRAM_ID_END
};

#define SCRIPT_APP_MAX_NVRAM_ENTRIES (UNICAST_APP_NVRAM_ID_END - UNICAST_APP_NVRAM_ID_START)

typedef struct
{
    wiced_bt_device_link_keys_t link_keys;
} unicast_sink_nvram_data_t;

int unicast_sink_nvram_read(int nvram_id, wiced_bt_device_address_t bdaddr, uint8_t *p_data, uint32_t len);
int unicast_sink_nvram_write(int nvram_id, wiced_bt_device_address_t bdaddr, uint8_t *p_data, uint32_t len);
void unicast_sink_nvram_delete(int nvram_id, wiced_bt_device_address_t bdaddr);

int unicast_sink_nvram_read_keys(wiced_bt_device_link_keys_t *p_linkkeys);
int unicast_sink_nvram_write_keys(wiced_bt_device_link_keys_t *p_linkkeys);

