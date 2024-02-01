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
 **  Name:          wiced_hal_nvram.c
 **
 **  Description:   Implements the interface for reading 
 **                 and writing any data to the NVRAM
 **
 ******************************************************************************/

#include "wiced_bt_types.h"
#include "wiced_hal_nvram.h"

extern uint32_t config_VS_Write (uint16_t config_item_id, uint32_t len, uint8_t* buf);
extern uint32_t config_VS_Read (uint16_t config_item_id, uint32_t len, uint8_t* buf);
extern uint32_t config_VS_Delete (uint16_t config_item_id);

/*****************************************************************************
 *                   Function Definitions
 ****************************************************************************/

/**
 * Writes the data to NVRAM,
 *
 * @param[in] vs_id        : Volatile Section Identifier. Application can use
 *                           the VS ids from WICED_NVRAM_VSID_START to
 *                           WICED_NVRAM_VSID_END
 *
 * @param[in] data_length  : Length of the data to be written to the NVRAM,
 *                           Application can only write up to 255 bytes
 *
 * @param[in] p_data       : Pointer to the data to be written to the NVRAM
 *
 * @param[out] p_status    : Pointer to location where status of the call
 *                           is returned
 *
 * @return  number of bytes written, 0 on error
 */
uint16_t write_nvram_utility( uint16_t vs_id,
            uint16_t         data_length,
            uint8_t        * p_data,
            wiced_result_t * p_status )
{
    uint16_t written = 0;
    wiced_result_t uResult = WICED_ERROR;

    written = config_VS_Write ( vs_id, data_length, p_data );

    if( written )
        uResult = WICED_SUCCESS;

    if( p_status )
        *p_status = uResult;

    return written;
}

/** Reads the data from NVRAM
 *
 * @param[in]  vs_id       : Volatile Section Identifier. Application can use
 *                           the VS ids from WICED_NVRAM_VSID_START to
 *                           WICED_NVRAM_VSID_END
 *
 * @param[in]  data_length : Length of the data to be read from NVRAM
 *
 * @param[out] p_data      : Pointer to the buffer to which data will be copied
 *
 * @param[out] p_status    : Pointer to location where status of the call
 *                           is returned
 *
 * @return  the number of bytes read, 0 on failure
 */
uint16_t read_nvram_utility( uint16_t vs_id,
        uint16_t         data_length,
        uint8_t        * p_data,
        wiced_result_t * p_status)
{
    uint16_t        read = 0;
    wiced_result_t  uResult = WICED_ERROR;

    read = config_VS_Read ( vs_id, data_length, p_data );

    if( read )
        uResult = WICED_SUCCESS;

    if( p_status )
        *p_status = uResult;

    return read;
}

/**
 * Delete the data from NVRAM
 * 
 * Delete the NVRAM data at specified VS id
 *
 * @param[in] vs_id        : Volatile Section Identifier. Application can use
 *                           the VS ids from WICED_NVRAM_VSID_START to
 *                           WICED_NVRAM_VSID_END
 *
 * @param[out] p_status    : Pointer to location where status of the call
 *                           is returned
 */
void delete_nvram_utility( uint16_t vs_id, wiced_result_t * p_status)
{
    uint8_t deleted = 0;
    wiced_result_t uResult = WICED_ERROR;

    deleted = config_VS_Delete ( vs_id );

    if(deleted)
        uResult = WICED_SUCCESS;

    if(p_status)
        *p_status = uResult;
}

/* WICED applications call the below APIs. WICED applications allowed to operate vs ids range WICED_NVRAM_VSID_START to WICED_NVRAM_VSID_END */
/**
 * Writes the data to NVRAM,
 * Application can write up to 255 bytes in one VS  id section
 *
 * @param[in] vs_id        : Volatile Section Identifier. Application can use
 *                           the VS ids from WICED_NVRAM_VSID_START to
 *                           WICED_NVRAM_VSID_END
 *
 * @param[in] data_length  : Length of the data to be written to the NVRAM,
 *                           Application can only write up to 255 bytes
 *
 * @param[in] p_data       : Pointer to the data to be written to the NVRAM
 *
 * @param[out] p_status    : Pointer to location where status of the call
 *                           is returned
 *
 *
 * @return  number of bytes written, 0 on error
 */
uint16_t wiced_hal_write_nvram( uint16_t vs_id,
        uint16_t        data_length,
        uint8_t        * p_data,
        wiced_result_t * p_status)
{
    if ( ( vs_id < WICED_NVRAM_VSID_START ) ||
            ( vs_id > WICED_NVRAM_VSID_END ) )
    {
        if( p_status )
        {
            *p_status = WICED_BADARG;
        }
        return 0;
    }

    return write_nvram_utility( vs_id, data_length, p_data, p_status );
}

/** Reads the data from NVRAM
 *
 * @param[in]  vs_id       : Volatile Section Identifier. Application can use
 *                           the VS ids from WICED_NVRAM_VSID_START to
 *                           WICED_NVRAM_VSID_END
 *
 * @param[in]  data_length : Length of the data to be read from NVRAM
 *
 * @param[out] p_data      : Pointer to the buffer to which data will be copied
 *
 * @param[out] p_status    : Pointer to location where status of the call
 *                           is returned
 *
 * @return  the number of bytes read, 0 on failure
 */
uint16_t wiced_hal_read_nvram( uint16_t vs_id,
        uint16_t        data_length,
        uint8_t        * p_data,
        wiced_result_t * p_status)
{
    if ( ( vs_id < WICED_NVRAM_VSID_START ) ||
            ( vs_id > WICED_NVRAM_VSID_END ) )
    {
        if( p_status )
        {
            *p_status = WICED_BADARG;
        }
        return 0;
    }
    return read_nvram_utility( vs_id, data_length, p_data, p_status );
}

/**
 * Delete the data from NVRAM
 *
 * @param[in] vs_id        : Volatile Section Identifier. Application can use
 *                           the VS ids from WICED_NVRAM_VSID_START to
 *                           WICED_NVRAM_VSID_END
 *
 * @param[out] p_status    : Pointer to location where status of the call
 *                           is returned
 *
 */
void wiced_hal_delete_nvram ( uint16_t vs_id, wiced_result_t * p_status )
{
    if ( ( vs_id < WICED_NVRAM_VSID_START ) ||
            ( vs_id > WICED_NVRAM_VSID_END ) )
    {
        if( p_status )
        {
            *p_status = WICED_BADARG;
        }
    }
    else
    {
        delete_nvram_utility( vs_id, p_status );
    }
}
