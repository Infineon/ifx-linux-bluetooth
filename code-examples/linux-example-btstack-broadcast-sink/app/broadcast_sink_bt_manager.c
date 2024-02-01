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

/******************************************************************************/

/* Application includes */
#include "broadcast_sink_bt_manager.h"
#include "broadcast_sink_bis.h"
#include "broadcast_sink_gatt.h"

/* BT Stack includes */
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"

extern wiced_bt_cfg_ble_t broadcast_sink_ble_cfg;
extern wiced_bt_cfg_isoc_t broadcast_sink_isoc_cfg;
extern void set_local_bd_addr(void);

static void (*bt_ready_callback)(void) = NULL;

ga_cfg_t broadcast_sink_ga_cfg = {
    .pacs_max_sink_capabilities_supported = 2,
    .pacs_max_source_capabilities_supported = 0,

    .bass_max_receive_state_supported = 2,
};


void broadcast_sink_reg_bt_ready_callback(void* callback)
{
    bt_ready_callback = callback;
}

static void broadcast_sink_bt_ready(void)
{
    bt_ready_callback();
}

wiced_result_t broadcast_sink_handle_btm_enabled(wiced_bt_dev_enabled_t *p_btm_enabled)
{
    wiced_bt_gatt_status_t sts = WICED_BT_ERROR;

    set_local_bd_addr();

    /* Initialize GATT */
    sts = broadcast_sink_gatt_init(broadcast_sink_ble_cfg.ble_max_simultaneous_links,
                                   broadcast_sink_ble_cfg.ble_max_rx_pdu_size,
                                   &broadcast_sink_ga_cfg);
    if (sts) WICED_BT_TRACE("[%s] GATT init sts %d\n", __FUNCTION__, sts);

    /* Initialize BIS */
    broadcast_sink_bis_init(&broadcast_sink_isoc_cfg);

    return WICED_SUCCESS;
}

wiced_result_t broadcast_sink_btm_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t res = WICED_ERROR;
    wiced_bt_dev_ble_io_caps_req_t *p_ble_io_caps = &p_event_data->pairing_io_capabilities_ble_request;

    WICED_BT_TRACE("[%s] Received Event [%d] \n", __FUNCTION__, event);

    switch (event)
    {
        case BTM_ENABLED_EVT:
            broadcast_sink_handle_btm_enabled(&p_event_data->enabled);
            broadcast_sink_bt_ready();
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: {
            p_ble_io_caps->local_io_cap = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            p_ble_io_caps->oob_data = BTM_OOB_NONE;
            p_ble_io_caps->auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
            p_ble_io_caps->max_key_size = 16;
            p_ble_io_caps->init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
            p_ble_io_caps->resp_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        }
        break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            WICED_BT_TRACE("[%s] status %d\n",
                           __FUNCTION__,
                           p_event_data->pairing_complete.pairing_complete_info.ble.status);
            break;

        default:
            break;
    }

    return res;
}
