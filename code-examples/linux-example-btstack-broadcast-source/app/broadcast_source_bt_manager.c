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
 * File Name: broadcast_source_bt_manager.c
 *
 * Description: This is the source file for Broadcast_source CE bt manager.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

/* Application includes */
#include "broadcast_source_bt_manager.h"

/* BT Stack includes */
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_gatt.h"

extern void set_local_bd_addr(void);

wiced_result_t broadcast_source_handle_btm_enabled(wiced_bt_dev_enabled_t *p_btm_enabled)
{
    set_local_bd_addr();
    return WICED_SUCCESS;
}

wiced_result_t broadcast_source_btm_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t res = WICED_ERROR;

    WICED_BT_TRACE("[%s] Received Event [%d] \n", __FUNCTION__, event);

    switch (event)
    {
        case BTM_ENABLED_EVT:
            broadcast_source_handle_btm_enabled(&p_event_data->enabled);
            break;

        default: break;
    }

    return res;
}
