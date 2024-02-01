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


/* Application includes */
#include "unicast_source_bt_manager.h"
#include "unicast_source_gatt.h"
#include "unicast_source_mcs.h"

/* App Library includes */
#include "wiced_bt_ga_common.h"

/* BT Stack includes */
#include "wiced_bt_cfg.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "app_bt_utils.h"

#include "log.h"

/******************************************************************************
 *                              EXTERNS VARIABLE
 *****************************************************************************/
extern wiced_bt_cfg_ble_t unicast_source_ble_cfg;
extern wiced_bt_cfg_isoc_t unicast_source_isoc_cfg;

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
ga_cfg_t unicast_source_ga_cfg = {
                                  .pacs_max_sink_capabilities_supported = MAX_PACS_SINK_CAP_SUPPORTED,
                                  .pacs_max_source_capabilities_supported = MAX_PACS_SOURCE_CAP_SUPPORTED,

                                  .ascs_max_sink_ase_supported = MAX_SINK_ASE_SUPPORTED,
                                  .ascs_max_source_ase_supported = MAX_SOURCE_ASE_SUPPORTED,

                                  .vcs.max_vcs_vocs = 1,
                                  .vcs.max_vcs_aics = 1,

                                  .max_mcs = 1};

/******************************************************************************
 * Function Name: unicast_source_handle_btm_enabled
 *
 * Summary:
 *      set bd_addr, do post init 
 *
 * Parameters:
 *      wiced_bt_dev_enabled_t *p_btm_enabled, for futrue use
 *
 * Return:
 *      wiced_result_t: Init result
 *                  
 *
 ******************************************************************************/
wiced_result_t unicast_source_handle_btm_enabled(wiced_bt_dev_enabled_t *p_btm_enabled)
{
    wiced_bt_gatt_status_t sts = WICED_BT_ERROR;

    set_local_bd_addr();

    /* Initialize GATT */
    sts = unicast_source_gatt_init(unicast_source_ble_cfg.ble_max_simultaneous_links,
                                   unicast_source_ble_cfg.ble_max_rx_pdu_size,
                                   &unicast_source_ga_cfg);
    if (sts) WICED_BT_TRACE("[%s] GATT init sts %d\n", __FUNCTION__, sts);

    /* Initialize profiles */
    unicast_source_mcs_initialize_data();

    return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_source_btm_handle_encryption_sts
 *
 * Summary:
 *  when encryption success, do gatt discovery
 *
 * Parameters:
 *  wiced_bt_dev_encryption_status_t *p_encryption_sts
 *      : encryption status
 *
 * Return:
 *  wiced_result_t: ERROR or SUCCESS
 *
 ******************************************************************************/
wiced_result_t unicast_source_btm_handle_encryption_sts(wiced_bt_dev_encryption_status_t *p_encryption_sts)
{
    wiced_bt_gatt_status_t sts = WICED_BT_ERROR;

    /* Start GATT Discovery */
    if (WICED_SUCCESS != p_encryption_sts->result) return WICED_ERROR;

    unicast_source_gatt_start_discovery(p_encryption_sts->bd_addr);

    return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: uniast_source_stack_module_init
 *
 * Summary:
 *  calling wiced_bt_smp_module_init api to enable smp module. this api only 
 *  in BTSTACK version >= 370
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 ******************************************************************************/
void uniast_source_stack_module_init(void)
{
    uint8_t res;
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);

    res = wiced_bt_smp_module_init();
    if (res != WICED_BT_SUCCESS) WICED_BT_TRACE(" wiced_bt_smp_module_init() failed : %d \n", res);
}

/******************************************************************************
 * Function Name: unicast_source_btm_cback 
 *
 * Summary:
 *  bt event callback
 *
 * Parameters:
 *  wiced_bt_management_evt_t event
 *  wiced_bt_management_evt_data_t *p_event_data
 *
 * Return:
 *  wiced_result_t
 *
 ******************************************************************************/
wiced_result_t unicast_source_btm_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t res = WICED_ERROR;
    wiced_bt_dev_ble_io_caps_req_t *p_ble_io_caps = &p_event_data->pairing_io_capabilities_ble_request;

    WICED_BT_TRACE("[%s] Received Event [%d], %s\n", __FUNCTION__, event, get_bt_event_name(event));

    switch (event) {
        case BTM_ENABLED_EVT:
            uniast_source_stack_module_init();
            unicast_source_handle_btm_enabled(&p_event_data->enabled);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: {
            p_ble_io_caps->local_io_cap = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            p_ble_io_caps->oob_data = BTM_OOB_NONE;
            p_ble_io_caps->auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
            p_ble_io_caps->max_key_size = 16;
            p_ble_io_caps->init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
            p_ble_io_caps->resp_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        } break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            res = unicast_source_btm_handle_encryption_sts(&p_event_data->encryption_status);
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            WICED_BT_TRACE("[%s] status %d\n",
                           __FUNCTION__,
                           p_event_data->pairing_complete.pairing_complete_info.ble.status);
            break;
        case BTM_BLE_DATA_LENGTH_UPDATE_EVENT:
        {
            wiced_bt_ble_phy_data_length_update_t *p_update = &p_event_data->ble_data_length_update_event;
            WICED_BT_TRACE("BTM_BLE_DATA_LENGTH_UPDATE_EVENT: remote bdaddr:%B\n", p_update->bd_address);
            WICED_BT_TRACE("BTM_BLE_DATA_LENGTH_UPDATE_EVENT: max tx octets:%d \n", p_update->max_tx_octets);
            WICED_BT_TRACE("BTM_BLE_DATA_LENGTH_UPDATE_EVENT: max tx times:%d \n", p_update->max_tx_time);
            WICED_BT_TRACE("BTM_BLE_DATA_LENGTH_UPDATE_EVENT: max rx octets:%d \n", p_update->max_rx_octets);
            WICED_BT_TRACE("BTM_BLE_DATA_LENGTH_UPDATE_EVENT: max rx times:%d \n", p_update->max_rx_time);
        }
            break;
        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            if (WICED_SUCCESS == p_event_data->ble_connection_param_update.status)
            {
                TRACE_LOG("Supervision Time Out = %d\n", (p_event_data->ble_connection_param_update.supervision_timeout * 10));
            }
            break;

        default:
            break;
    }

    return res;
}
