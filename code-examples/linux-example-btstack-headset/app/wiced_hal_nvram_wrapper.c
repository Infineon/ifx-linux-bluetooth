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

#include "wiced_data_types.h"
#include "wiced_bt_trace.h"

#define VS_CONFIG_MAX_ITEMS     10
#define VS_CONFIG_MAX_LEN       200

struct tNVRAM
{
    uint16_t config_id[VS_CONFIG_MAX_ITEMS];
    uint8_t data[VS_CONFIG_MAX_ITEMS][VS_CONFIG_MAX_LEN];
}nvram_cache;
struct tNVRAM *p_nvram_cache = &nvram_cache;

extern uint8_t config_VS_Delete(uint16_t config_item_id);
extern uint32_t config_VS_Write(uint16_t config_item_id, uint32_t len, uint8_t* buf);
extern uint32_t config_VS_Read(uint16_t config_item_id, uint32_t len, uint8_t* buf);

uint32_t wiced_hal_wrapper_write_nvram(uint16_t config_item_id, uint32_t len, uint8_t* buf)
{
    uint32_t ret;
    ret = config_VS_Write(config_item_id, len, (uint8_t*)buf);
    return ret;

}

uint32_t wiced_hal_wrapper_read_nvram(uint16_t config_item_id, uint32_t len, uint8_t *buf)
{
    uint32_t ret;

    ret = (uint32_t)config_VS_Read(config_item_id, len, (uint8_t*)buf);
    return ret;

}

uint8_t wiced_hal_wrapper_delete_nvram(uint16_t config_item_id)
{
    uint32_t ret;
    ret = (uint32_t)config_VS_Delete(config_item_id);
    return ret;

}

