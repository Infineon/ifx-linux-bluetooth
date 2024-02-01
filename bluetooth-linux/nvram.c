/*
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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

/*****************************************************************************
 **
 **  Name:          nvram.c
 **
 **  Description: nvram control api
 **
 ******************************************************************************/

#include "stdio.h"
#include "string.h"
#include "wiced_data_types.h"
#include "log.h"

extern int wicedx_emulator_instance;
void config_get_file_name(char* p_name_buf, int buf_len, uint16_t config_item_id)
{
    snprintf(p_name_buf, buf_len, "nvram%d_%04d.bin", wicedx_emulator_instance, config_item_id);
}

FILE * config_get_file(uint16_t config_item_id, char * read_option)
{
    FILE* p_file;
    char nvram_file_name[30] = { 0 };

    config_get_file_name(nvram_file_name, sizeof(nvram_file_name), config_item_id);
    p_file = fopen(nvram_file_name, read_option);
    if (!p_file)
    {
        TRACE_ERR("Unable to open NVRAM file %s with %s\n", nvram_file_name, read_option);
        return NULL;
    }

    return p_file;
}

uint32_t config_VS_Read (uint16_t config_item_id, uint32_t len, uint8_t *buf)
{
    FILE* p_file = config_get_file(config_item_id, "rb");
    size_t read_len;

    if (!p_file) {
        return 0;
    }

    read_len = fread(buf, 1, len, p_file);
    fclose(p_file);

    TRACE_LOG("id 0x%x len %zu\n", config_item_id, read_len);
    return read_len ;
}

uint32_t config_VS_Write(uint16_t config_item_id, uint32_t len, uint8_t* buf)
{
    FILE* p_file = config_get_file(config_item_id, "wb");
    size_t write_len;

    if (!p_file) {
        return 0;
    }

    write_len = fwrite(buf, 1, len, p_file);

    fclose(p_file);

    TRACE_LOG("id 0x%x len %zu\n", config_item_id, write_len);
    return write_len;
}

wiced_bool_t config_VS_Delete (uint16_t config_item_id)
{
    char nvram_file_name[30] = { 0 };

    config_get_file_name(nvram_file_name, sizeof(nvram_file_name), config_item_id);

    TRACE_LOG("%s\n", nvram_file_name);

    if(remove(nvram_file_name) != 0 )
        return FALSE;
    else
        return TRUE;

}
