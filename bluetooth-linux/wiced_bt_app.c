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
 **  Name:          wiced_bt_app.c
 **
 **  Description:   wiced_bt_app api for btstack (Linux)
 **
 ******************************************************************************/

#include "wiced_bt_dev.h"
#include "wiced_memory.h"
#include "wiced_bt_types.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_stack_platform.h"
#include <stdio.h>
#include "data_types.h"
#include "platform_linux.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_isoc.h"
#include "log.h"

#define tBTM_EVENT                      wiced_bt_management_evt_t
#define tBTM_ENABLED_EVT                wiced_bt_dev_enabled_t
#define tBTM_DISABLED_EVT               wiced_bt_dev_disabled_t
#define tBTM_EVENT_DATA                 wiced_bt_management_evt_data_t
#define tBTM_STATUS wiced_result_t
#define BTM_SUCCESS                     WICED_BT_SUCCESS                     /* Command succeeded                 */
#define tBTM_ENCRYPTION_STATUS_EVT      wiced_bt_dev_encryption_status_t
#define BTM_BLE_LL_DATA_TX_TIME_MIN     0x0148
#define BTM_BLE_LL_DATA_TX_TIME_MAX     0x0848
#define BTM_BLE_LL_DATA_PDU_MAX         0x00fb
#define BTM_BLE_LL_DATA_PDU_MIN         0x001b

typedef tBTM_STATUS(tBTM_EVENT_CBACK) (tBTM_EVENT event, tBTM_EVENT_DATA* p_event_data);

/*****************************************************************************
 *               Function Declarations
 ****************************************************************************/
BOOL32          wiced_stack_event_handler_cback(uint8_t *p_event);/* BTLE HCI event call back. In Call back we can handle event before host stack get chance to process */
tBTM_STATUS     wiced_internal_btm_management_cback (tBTM_EVENT event, tBTM_EVENT_DATA *p_event_data);/* WICED shim BTLE stack management event call back */
extern BOOL32   btsnd_hcic_ble_write_dflt_data_length(uint16_t tx_octets, uint16_t tx_time);

/* External function declarations */
extern tBTM_STATUS  BTM_BleScatternetEnable (BOOL32 enable);

/****************************************************************************
** Extern Variables
****************************************************************************/


/*****************************************************************************
 *               Variables Definitions
*****************************************************************************/

//WICED BT application Global variables
tBTM_EVENT_CBACK                *p_app_management_callback; /* WICED application BT management call back holder */
int                             wiced_app_le_cache_state = 0;
tBTM_EVENT_DATA                 wiced_app_cached_event;

/*****************************************************************************
 *                                              Function Definitions
 ****************************************************************************/

#if (defined(BTSTACK_VER) && (BTSTACK_VER >= 0x03070000))

/*****************************************************************************
 * Function Name: init_layers
 *
 * Description: this funciton call btstack API to init smp for btstack370
 *
 * Param: void
 *
 * Return: wiced_result_t 
 *
 ****************************************************************************/
static wiced_result_t init_layers(void)
{
    /* handle in porting layer */
    return wiced_bt_smp_module_init();
}

/*****************************************************************************
 * Function Name: app_initialize_btstack_modules
 *
 * Description: this function call init_layers to init stack module
 *
 * Param: void
 *
 * Return: wiced_result_t
 *
 ****************************************************************************/
static wiced_result_t app_initialize_btstack_modules(void)
{
     return init_layers();
}
#endif

/*
 * On stack initalization complete this call back gets called
*/
void wiced_post_stack_init_cback( void )
{
    tBTM_EVENT_DATA event_data;

    TRACE_LOG("init\n");

    memset( &event_data, 0, sizeof(tBTM_EVENT) );
    event_data.enabled.status = BTM_SUCCESS;
    wiced_internal_btm_management_cback ( BTM_ENABLED_EVT, &event_data );
}

/*
* This call back gets called for each HCI event.
*/
BOOL32 wiced_stack_event_handler_cback (uint8_t *p_event)
{
#if 0   // Enable only for debug
    if ( *p_event == HCI_EVENT_CODE_ENCRYPTION_CHANGE_COMPLETE )
    {
        HCI_EVENT_ENCRYPTION_CHANGE_COMPLETE* p_enc = (HCI_EVENT_ENCRYPTION_CHANGE_COMPLETE*)event;

        TRACE_LOG( "HCI ENCRY CHANGE COMPL status\n", p_enc->status);
    }
#endif

    return WICED_FALSE;
}

/* Override wiced_mpaf_ota_callbacks.p_btm_mgmt_callback if need to handle any management events before passing to application */
tBTM_STATUS wiced_internal_btm_management_cback (tBTM_EVENT event, tBTM_EVENT_DATA *p_event_data)
{
    TRACE_LOG("[wiced_internal_btm_management_cback] event:0x%x\n", event);

    switch (event)
    {
        case BTM_ENABLED_EVT:
#if (defined(BTSTACK_VER) && (BTSTACK_VER >= 0x03070000))
            TRACE_LOG("BTSTACK Version:%x\n",  BTSTACK_VER);

            //TODO: call btstack version api (if have) to show library version

            app_initialize_btstack_modules();

            wiced_bt_init_resolution();

            break;
        default:
            break;
#else
    #error "BTSTACK version too old!!!, only support >= 3.7.0"
#endif


    }
    if (p_app_management_callback )
    {
        return (tBTM_STATUS)p_app_management_callback(event, p_event_data);
    }

    return BTM_SUCCESS;
}

wiced_result_t wiced_bt_stack_init (wiced_bt_management_cback_t *p_bt_management_cback,
                                    const wiced_bt_cfg_settings_t *p_bt_cfg_settings)
{
    TRACE_LOG("init\n");

     p_app_management_callback   = p_bt_management_cback;

     wiced_bt_platform_interface_init();

    /* Configure the stack */
    if (wiced_bt_set_stack_config (p_bt_cfg_settings) == 0)
        return (WICED_ERROR);

    wiced_bt_stack_init_internal(wiced_internal_btm_management_cback,
                                 wiced_post_stack_init_cback,
                                 wiced_stack_event_handler_cback);

    return WICED_BT_SUCCESS;
}

wiced_result_t wiced_bt_stack_deinit(void)
{
    wiced_bt_stack_shutdown();
    return WICED_SUCCESS;
}
