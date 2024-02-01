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

/** @file
 *
 * This file provides the private interface definitions for hci_control
 *
 */
#ifndef HCI_CONTROL_H
#define HCI_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#include "wiced_result.h"

/*****************************************************************************
**  Constants that define the capabilities and configuration
*****************************************************************************/
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI)
#define TRANS_SPI_BUFFER_SIZE           1024
#endif
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_UART)
#define TRANS_UART_BUFFER_SIZE          1024
#endif

/*****************************************************************************
**  Function prototypes
*****************************************************************************/
/* main functions */

/*******************************************************************************
* Function Name: btheadset_control_init
********************************************************************************
* Summary:
*   initial btstack, run at application start
*
* Parameters:
*   void
*
* Return:
*   none
*
*******************************************************************************/
void btheadset_control_init( void );


/*******************************************************************************
* Function Name: btheadset_post_bt_init
********************************************************************************
* Summary:
*   initial profile feature, run when stack initial is done (BTM_ENABLED_EVT)
*
* Parameters:
*   void
*
* Return:
*   wiced_result_t: result
*
*******************************************************************************/
wiced_result_t btheadset_post_bt_init(void);

/*******************************************************************************
* Function Name: bt_hs_spk_control_link_key_display
********************************************************************************
* Summary:
*   Display Link key
*
* Parameters:
*   NONE
*
* Return:
*   NONE
*
*******************************************************************************/
void bt_hs_spk_control_link_key_display(void);

/*******************************************************************************
* Function Name: headset_get_statues
********************************************************************************
* Summary:
*   print current status for connect, sink, call active
*
* Parameters:
*   void
*
* Return:
*   none
*
*******************************************************************************/
void headset_get_statues(void);

/*******************************************************************************
* Function Name: headset_get_br_connect
********************************************************************************
* Summary:
*   return the headset BR/EDR is connected or not
*
* Parameters:
*   void
*
* Return:
*   bool false: disconenct true: connected

*******************************************************************************/
bool headset_get_br_connect(void);

/*******************************************************************************
* Function Name: headset_get_call_active
********************************************************************************
* Summary:
*   return the call in active or not
*
* Parameters:
*   void
*
* Return:
*   bool false: no active true: call active

*******************************************************************************/
bool headset_get_call_active(void);

/*******************************************************************************
* Function Name: headset_get_playstate
********************************************************************************
* Summary:
*   return the headset sink play state
*
* Parameters:
*   void
*
* Return:
*   uint8_t return the sink play state, refer to wiced_bt_avrc_playstate_t
*    0x00    < Stopped 
*    0x01    < Playing
*    0x02    < Paused
*    0x03    < Fwd Seek
*    0x04    < Rev Seek
*    0xFF    < Error
*******************************************************************************/
uint8_t headset_get_playstate(void);
#endif /* BTA_HS_INT_H */
