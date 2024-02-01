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
#include "unicast_sink_bt_manager.h"
#include "unicast_sink_nvram.h"
#include "unicast_sink_isoc.h"

/* App Library includes */
#include "wiced_bt_ga_common.h"

/* BT Stack includes */
#include "wiced_bt_cfg.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "app_bt_utils.h"

/******************************************************************************
 *                              EXTERNS
 *****************************************************************************/
extern wiced_bt_cfg_ble_t unicast_sink_ble_cfg;
extern wiced_bt_cfg_isoc_t unicast_sink_isoc_cfg;

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
wiced_bt_device_address_t local_bda = {0x11, 0x22, 0x11, 0x22, 0x11, 0x22};

ga_cfg_t unicast_sink_ga_cfg = {
    .pacs_max_sink_capabilities_supported = 1,
    .pacs_max_source_capabilities_supported = 0,

    .ascs_max_sink_ase_supported = 1,
    .ascs_max_source_ase_supported = 0,
    .vcs_step_size = 15,

    .max_mcs = 1,
};

/******************************************************************************
 * Function Name: unicast_sink_btm_handle_encryption_sts 
 ******************************************************************************
 * Summary: handle encryption status from stack
 *
 * Parameters:
 *  wiced_bt_dev_encryption_status_t *p_encryption_sts
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_sink_btm_handle_encryption_sts(wiced_bt_dev_encryption_status_t *p_encryption_sts)
{
    wiced_bt_gatt_status_t sts = WICED_BT_ERROR;

    /* Start GATT Discovery */
    if (WICED_SUCCESS != p_encryption_sts->result) return WICED_ERROR;

    unicast_sink_gatt_start_discovery(p_encryption_sts->bd_addr);

    return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_sink_handle_btm_enabled 
 ******************************************************************************
 * Summary: handle btm_enable case from stack, do some post init here 
 *
 * Parameters:
 *  wiced_bt_dev_enabled_t *p_btm_enabled
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_sink_handle_btm_enabled(wiced_bt_dev_enabled_t *p_btm_enabled)
{
    wiced_bt_gatt_status_t sts = WICED_BT_ERROR;

    set_local_bd_addr();

    /* Initialize GATT */
    sts = unicast_sink_gatt_init(unicast_sink_ble_cfg.ble_max_simultaneous_links,
                                 unicast_sink_ble_cfg.ble_max_rx_pdu_size,
                                 &unicast_sink_ga_cfg);
    if (sts) WICED_BT_TRACE("[%s] GATT init sts %d\n", __FUNCTION__, sts);

    unicast_sink_isoc_dhm_init(&unicast_sink_isoc_cfg);
    wiced_bt_set_pairable_mode(1, 0);

    return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_sink_btm_handle_key_update_event
 ******************************************************************************
 * Summary: handle link key update event from stack
 *
 * Parameters:
 *  wiced_bt_device_link_keys_t *p_event_data
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_sink_btm_handle_key_update_event(wiced_bt_device_link_keys_t *p_event_data)
{
    {
        wiced_bt_device_link_keys_t *keys = p_event_data;

        WICED_BT_TRACE("[%s] linkkey update %B Type: %d  Conn: %B   Key_Mask: 0x%x",
                       __FUNCTION__,
                       keys->bd_addr,
                       keys->key_data.ble_addr_type,
                       keys->conn_addr,
                       keys->key_data.le_keys_available_mask);
        WICED_BT_TRACE("[%s] br_edr %A type %d",
                       __FUNCTION__,
                       keys->key_data.br_edr_key,
                       sizeof(keys->key_data.br_edr_key),
                       keys->key_data.br_edr_key_type);
        WICED_BT_TRACE("[%s] lltk %A sec_level %d %d %d",
                       __FUNCTION__,
                       keys->key_data.le_keys.lltk,
                       sizeof(keys->key_data.le_keys.lltk),
                       keys->key_data.le_keys.sec_level,
                       keys->key_data.le_keys.local_csrk_sec_level,
                       keys->key_data.le_keys.srk_sec_level);
        WICED_BT_TRACE("[%s] pltk %A sec_level %d %d %d",
                       __FUNCTION__,
                       keys->key_data.le_keys.pltk,
                       sizeof(keys->key_data.le_keys.pltk),
                       keys->key_data.le_keys.sec_level,
                       keys->key_data.le_keys.local_csrk_sec_level,
                       keys->key_data.le_keys.srk_sec_level);
    }

    unicast_sink_nvram_write_keys(p_event_data);

    return WICED_BT_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_sink_btm_handle_key_request_event 
 ******************************************************************************
 * Summary: handle link key request event from btstack
 *
 * Parameters:
 *  wiced_bt_device_link_keys_t *p_event_data
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_sink_btm_handle_key_request_event(wiced_bt_device_link_keys_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    if (!unicast_sink_nvram_read_keys(p_event_data))
    {
        WICED_BT_TRACE("[%s] no key for BDA: %B", __FUNCTION__, p_event_data->bd_addr);
        result = WICED_ERROR;
    }
    else
    {
        WICED_BT_TRACE("[%s] found key for BDA: %B", __FUNCTION__, p_event_data->bd_addr);
    }

    return result;
}

/******************************************************************************
 * Function Name: unicast_sink_btm_cback 
 ******************************************************************************
 * Summary: btstack callback function, register in wiced_bt_stack_init
 *
 * Parameters:
 *  wiced_bt_management_evt_t event
 *  wiced_bt_management_evt_data_t *p_event_data
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_sink_btm_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t res = WICED_SUCCESS;
    wiced_bt_dev_ble_io_caps_req_t *p_ble_io_caps = &p_event_data->pairing_io_capabilities_ble_request;
    extern wiced_bt_cfg_settings_t unicast_sink_cfg_settings;

    WICED_BT_TRACE("[%s] Received Event [%d] \n", __FUNCTION__, event);

    switch (event)
    {
        case BTM_ENABLED_EVT:
            unicast_sink_handle_btm_enabled(&p_event_data->enabled);
            if (unicast_sink_cfg_settings.p_ble_cfg->rpa_refresh_timeout)
            {
                wiced_bt_set_local_bdaddr(local_bda, BLE_ADDR_RANDOM);
            }
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

        case BTM_ENCRYPTION_STATUS_EVT:
            res = unicast_sink_btm_handle_encryption_sts(&p_event_data->encryption_status);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: {
            res = unicast_sink_btm_handle_key_update_event(&p_event_data->paired_device_link_keys_update);
            if (res == WICED_BT_SUCCESS)
            {
                WICED_BT_TRACE_ARRAY(p_event_data->paired_device_link_keys_update.key_data.le_keys.lltk,
                                     16,
                                     "***LLTK***");
            }
        }
        break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT: {
            res = unicast_sink_btm_handle_key_request_event(&p_event_data->paired_device_link_keys_request);
        }

        break;
        default:
            break;
    }

    return res;
}
