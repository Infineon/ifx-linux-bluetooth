/*
* Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/**************************************************************************//**
* \file
*
* \brief Provides definitions for application configuration items and API.
* \details Some per device information may be programmed in the flash during manufacturing.
* The data is saved in the Static Section of the flash and is not changed during
* the DFU or a factory reset. This interface provides standard method for an application
* to retrieve this data.
*
* Programming of the flash can be done using ss_json script which is provided
* in the tools directory of the SDK.
*
*
******************************************************************************/

#ifndef WICED_BT_FACTORY_APP_CONFIG__H
#define WICED_BT_FACTORY_APP_CONFIG__H

#ifdef __cplusplus
extern "C" {
#endif

/**
*
* \addtogroup  wiced_bt_factory_config_api_functions        Factory Config Library API
* \ingroup     wicedsys
* @{
*
* The Factory Config library of the AIROC BTSDK provides a simple method for an application to read
* from the static section of the flash items that has been typically programmed in the factory.
*/

/**
 * @anchor WICED_BT_FACTORY_CONFIG_IDS
 * @name Factory Configuration Item Types
 * \details The following is the list of items types that can be programmed in the flash
 * @{
 */
#define WICED_BT_FACTORY_CONFIG_ITEM_FIRST              0x80

/* Definition of te provisioning records IDs
   Corresponding standard definitions are in wiced_bt_mesh_core.h file */
#define WICED_BT_FACTORY_CONFIG_ITEM_BASE_URI                     0xC0
#define WICED_BT_FACTORY_CONFIG_ITEM_DEVICE_CERTIFICATE           0xC1
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_1   0xC2
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_2   0xC3
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_3   0xC4
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_4   0xC5
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_5   0xC6
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_6   0xC7
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_7   0xC8
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_8   0xC9
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_9   0xCA
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_A   0xC8
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_B   0xCC
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_C   0xCD
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_D   0xCE
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_E   0xCF
#define WICED_BT_FACTORY_CONFIG_ITEM_INTERMEDIATE_CERTIFICATE_F   0xD0
#define WICED_BT_FACTORY_CONFIG_ITEM_LOCAL_NAME                   0xD1
#define WICED_BT_FACTORY_CONFIG_ITEM_APPEARANCE                   0xD2
/* End of provisioning records IDs */

#define WICED_BT_FACTORY_CONFIG_ITEM_UUID                         0xE0
#define WICED_BT_FACTORY_CONFIG_ITEM_OOB_STATIC_DATA              0xE1
#define WICED_BT_FACTORY_CONFIG_ITEM_HOMEKIT_TOKEN                0xE2
#define WICED_BT_FACTORY_CONFIG_ITEM_PRIVATE_KEY                  0xE3
#define WICED_BT_FACTORY_CONFIG_ITEM_LAST                         0xEF
/** @} WICED_BT_FACTORY_CONFIG_IDS */

#define WICED_BT_FACTORY_PROVISIONING_RECORD_OFFSET     WICED_BT_FACTORY_CONFIG_ITEM_BASE_URI
#define WICED_BT_FACTORY_PROVISIONING_RECORD_SIZE       WICED_BT_FACTORY_CONFIG_ITEM_APPEARANCE - WICED_BT_FACTORY_CONFIG_ITEM_BASE_URI + 1

/******************************************************************************
*
* Function Name: wiced_bt_factory_config_read
*
***************************************************************************//**
*
* \details The application calls this API to retrieve data programmed in the static
* section of flash. If entry with specified type is stored in the static section,
* the function copies data or requested chunk of it from the flash into the
* application buffer and returns number of bytes that has been copied. If entry
* with specified type does not exist in the static section of the flash, the
* function returns zero.
*
* \param    type : one of the @ref WICED_BT_FACTORY_CONFIG_IDS types
* \param    buffer : application buffer to return
* \param    read_size : number of bytes to read
* \param    read_offset : read offset from the start of the record
* \param    record_size : output parameter indicating total size of the record
* \return   number of bytes copied from the flash, or zero if item not found.
*
******************************************************************************/
uint16_t wiced_bt_factory_config_read(uint8_t item_type, uint8_t* buffer, uint16_t read_size, uint16_t read_offset , uint16_t * record_size);

/******************************************************************************
*
* Function Name: wiced_bt_factory_config_provisioning_records_list
*
***************************************************************************//**
*
* \details The application calls this API to get list of IDs of the provisioning records
* programmed in the static section of flash.
*
* \param    buffer : pointer to the memory buffer to store the record list of IDs
* \return   number of provisioning record IDs found
*
******************************************************************************/
uint16_t wiced_bt_factory_config_provisioning_records_get(uint16_t* buffer);


/******************************************************************************
*
* Function Name: wiced_bt_factry_config_provisioning_record_get
*
***************************************************************************//**
*
* \details The application calls this API to get list of IDs of the provisioning records
* programmed in the static section of flash.
*
* \param    record_id : record list of IDs
* \param    buffer : pointer to the memory buffer to store the record fragment
* \param    fragment_length : length of the record fragment to read
* \param    fragment_offset : start offset of the record fragment
* \param    record_size : pointer to output parameter to save the total record size
* \return   number of provisioning record IDs found
*
******************************************************************************/
uint16_t wiced_bt_factory_config_provisioning_record_req(uint16_t record_id, uint8_t* buffer, uint16_t fragment_length, uint16_t fragment_offset, uint16_t * record_size);

#ifdef __cplusplus
}
#endif

/** @} wiced_bt_factory_config_api_functions */

#endif /* WICED_BT_FACTORY_APP_CONFIG__H */
