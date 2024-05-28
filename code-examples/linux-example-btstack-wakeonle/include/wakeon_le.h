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
 * File Name: wakeon_ble.h
 * 
 * Description: This is the header file for Linux wakeOnLE CE
 *
 ******************************************************************************
* $ Copyright 2022-YEAR Cypress Semiconductor $
 *****************************************************************************/

#ifndef __APP_WAKEON_LE_H__
#define __APP_WAKEON_LE_H__

/******************************************************************************
*       MACRO
******************************************************************************/
#define LE_PCF_MANUFACTURE_DATA_LEN_MAX            29U
#define LE_PCF_COMANY_ID_LEN                       2U
#define LE_PCF_MANUFACTURE_DATA_PATTERN_LEN_MAX    27U
#define COMPANY_ID				   0x0009

/******************************************************************************
*       TYPEDEF 
******************************************************************************/

/******************************************************************************
*       FUNCTION PROTOTYPE
******************************************************************************/
void application_start( void );
void app_disable_wake_on_le();
void app_enable_wake_on_le();
void app_enable_wake_on_le_uuid();
void app_enable_wake_on_le_uuid_manu();
void app_enable_wake_on_connection();

/* BT LE configuration settings */     
extern const  wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

#endif /* __APP_WAKEON_LE_H__ */
