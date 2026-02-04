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

/******************************************************************************
 * File Name: patch_download.h
 *
 * Description: This is the header file for patch_download.c
 *
 ******************************************************************************/

#ifndef PATCH_DOWNLOAD_H
#define PATCH_DOWNLOAD_H

#include "wiced_bt_types.h"

typedef uint8_t tBRCM_PRM_STATUS;

void GKI_delay(uint32_t timeout);

/*******************************************************************************
 **
 ** Function            patch_download_check
 **
 ** Description         This function is called after HCI Reset is complete
 **                     Download patch ram and update serial port baud rate if needed.
 **
 *******************************************************************************/
void patch_download_check (void);

/*******************************************************************************
 **
 ** Function            wait_controller_reset
 **
 ** Description         This is a function for waiting controller reset signaled 
 **                     after patch downloading.
 **
 *******************************************************************************/
void wait_controller_reset (void);

#endif //PATCH_DOWNLOAD_H
