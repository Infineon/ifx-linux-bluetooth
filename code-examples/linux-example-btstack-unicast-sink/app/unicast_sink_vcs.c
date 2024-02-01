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

#include "unicast_sink_gatt.h"
#include "unicast_sink_rpc.h"

#include "gatt_interface.h"
#include "wiced_bt_ga_common.h"

#include "wiced_bt_ga_vcs.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "le_audio_rpc.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define DEFAULT_VOLUME  130U

/******************************************************************************
 *                              EXTERNS
 *****************************************************************************/
extern ga_cfg_t unicast_sink_ga_cfg;

/****************************************************************************
 *                              FUNCTION DECLARATION
 ***************************************************************************/
extern void audio_driver_set_mute_state(uint8_t mute_enabled);
extern void audio_driver_set_volume(uint8_t volume);

/******************************************************************************
 * Function Name: vcs_set_volume 
 ******************************************************************************
 * Summary: implement the vcs set volume through audio_driver_set_volume 
 *
 * Parameters:
 *  unicast_sink_volume_t *p_vcs
 *  int incr
 *
 * Return:
 *  None
 *
******************************************************************************/
static void vcs_set_volume(unicast_sink_volume_t *p_vcs, int incr)
{
    int new_volume = p_vcs->state.volume_setting + incr;

    /* ensure that the updated volume is within the supported range */
    new_volume = (new_volume < WICED_BT_GA_VCS_MINIMUM_VOLUME) ? WICED_BT_GA_VCS_MINIMUM_VOLUME : new_volume;
    new_volume = (new_volume > WICED_BT_GA_VCS_MAXIMUM_VOLUME) ? WICED_BT_GA_VCS_MAXIMUM_VOLUME : new_volume;

    p_vcs->state.volume_setting = new_volume;

    WICED_BT_TRACE("[%s] new_volume %d \n", __FUNCTION__, new_volume);

    /* VCS profile defines volume range on 0-255, hence mapping the received value to 0-100 */
    audio_driver_set_volume((p_vcs->state.volume_setting * 100) / 255);
    le_audio_rpc_send_vcs_state_update(0, p_vcs->state.volume_setting, p_vcs->state.mute_state, VOLUME_STATUS);
}

/******************************************************************************
 * Function Name: vcs_handle_write_req_evt
 ******************************************************************************
 * Summary: handle vcs write request, use in vcs callback
 *
 * Parameters:
 *  uint16_t conn_id
 *  unicast_sink_volume_t *p_vcs
 *  gatt_intf_attribute_t *p_char
 *  wiced_bt_ga_vcs_data_t *p_evt_data
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
static wiced_result_t vcs_handle_write_req_evt(uint16_t conn_id,
                                               unicast_sink_volume_t *p_vcs,
                                               gatt_intf_attribute_t *p_char,
                                               wiced_bt_ga_vcs_data_t *p_evt_data)
{
    wiced_result_t result = WICED_SUCCESS;

    if (p_char->characteristic_type != VCS_CONTROL_POINT_CHARACTERISTIC) return WICED_ERROR;

    WICED_BT_TRACE("[%s] opcode %d \n", __FUNCTION__, p_evt_data->control_point_data.opcode);

    switch (p_evt_data->control_point_data.opcode)
    {
        case VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_DOWN:
            vcs_set_volume(p_vcs, -unicast_sink_ga_cfg.vcs_step_size);
            break;

        case VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_UP:
            vcs_set_volume(p_vcs, +unicast_sink_ga_cfg.vcs_step_size);
            break;

        case VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_DOWN:
            p_vcs->state.mute_state = WICED_BT_GA_VCS_NOT_MUTED;
            audio_driver_set_mute_state(p_vcs->state.mute_state);
            le_audio_rpc_send_vcs_state_update(conn_id,
                                               p_vcs->state.volume_setting,
                                               p_vcs->state.mute_state,
                                               MUTE_STATUS);
            vcs_set_volume(p_vcs, -unicast_sink_ga_cfg.vcs_step_size);
            break;

        case VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_UP:
            p_vcs->state.mute_state = WICED_BT_GA_VCS_NOT_MUTED;
            audio_driver_set_mute_state(p_vcs->state.mute_state);
            le_audio_rpc_send_vcs_state_update(conn_id,
                                               p_vcs->state.volume_setting,
                                               p_vcs->state.mute_state,
                                               MUTE_STATUS);
            vcs_set_volume(p_vcs, +unicast_sink_ga_cfg.vcs_step_size);
            break;

        case VOLUME_CONTROL_OPCODE_SET_ABSOLUTE_VOLUME: {
            // update volume flag
            if (p_evt_data->control_point_data.volume_state.volume_setting <= WICED_BT_GA_VCS_MAXIMUM_VOLUME)
            {
                p_vcs->state.volume_setting = p_evt_data->control_point_data.volume_state.volume_setting;
                vcs_set_volume(p_vcs, 0);
            }
        }
        break;

        case VOLUME_CONTROL_OPCODE_UNMUTE:
            p_vcs->state.mute_state = WICED_BT_GA_VCS_NOT_MUTED;
            audio_driver_set_mute_state(p_vcs->state.mute_state);
            le_audio_rpc_send_vcs_state_update(conn_id,
                                               p_vcs->state.volume_setting,
                                               p_vcs->state.mute_state,
                                               MUTE_STATUS);
            break;

        case VOLUME_CONTROL_OPCODE_MUTE:
            p_vcs->state.mute_state = WICED_BT_GA_VCS_MUTED;
            audio_driver_set_mute_state(p_vcs->state.mute_state);
            le_audio_rpc_send_vcs_state_update(conn_id,
                                               p_vcs->state.volume_setting,
                                               p_vcs->state.mute_state,
                                               MUTE_STATUS);
            break;
    }

    return result;
}

/******************************************************************************
 * Function Name: vcs_handle_read_req_evt
 ******************************************************************************
 * Summary: handle vcs read request
 *
 * Parameters:
 *  uint16_t conn_id
 *  unicast_sink_volume_t *p_vcs
 *  gatt_intf_attribute_t *p_char
 *  wiced_bt_ga_vcs_data_t *p_evt_data
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
static wiced_result_t vcs_handle_read_req_evt(uint16_t conn_id,
                                              unicast_sink_volume_t *p_vcs,
                                              gatt_intf_attribute_t *p_char,
                                              wiced_bt_ga_vcs_data_t *p_evt_data)
{
    wiced_result_t result = WICED_SUCCESS;

    if (p_char->included_service_type != INCLUDED_SERVICE_NONE) return WICED_ERROR;

    switch (p_char->characteristic_type)
    {
        case VCS_VOLUME_STATE_CHARACTERISTIC:
            p_evt_data->control_point_data.volume_state = p_vcs->state;
            break;

        case VCS_VOLUME_FLAG_CHARACTERISTIC:
            p_evt_data->volume_flag = p_vcs->flag;
            break;
    }

    return result;
}

/******************************************************************************
 * Function Name: unicast_sink_vcs_callback 
 ******************************************************************************
 * Summary: unicast sink vcs callback function, use in unicast_sink_store_service_ref_local
 *          as callback for gatt
 *
 * Parameters:
 *  uint16_t conn_id,
 *  void *p_app_ctx,
 *  const gatt_intf_service_object_t *p_service,
 *  wiced_bt_gatt_status_t status,
 *  uint32_t evt_type,
 *  gatt_intf_attribute_t *p_char,
 *  void *p_data,
 *  int len
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_sink_vcs_callback(uint16_t conn_id,
                                         void *p_app_ctx,
                                         const gatt_intf_service_object_t *p_service,
                                         wiced_bt_gatt_status_t status,
                                         uint32_t evt_type,
                                         gatt_intf_attribute_t *p_char,
                                         void *p_data,
                                         int len)
{
    wiced_result_t result = WICED_SUCCESS;
    unicast_sink_volume_t *p_vcs = &g_unicast_sink_gatt_cb.vcs_data;

    WICED_BT_TRACE("[%s] event 0x%x \n", __FUNCTION__, p_char);

    switch (evt_type)
    {
        case WRITE_REQ_EVT:
            result = vcs_handle_write_req_evt(conn_id, p_vcs, p_char, p_data);
            break;

        case READ_REQ_EVT:
            result = vcs_handle_read_req_evt(conn_id, p_vcs, p_char, p_data);
            break;

        default:
            break;
    }
    return result;
}

/******************************************************************************
 * Function Name: unicast_sink_vcs_initialize_data
 ******************************************************************************
 * Summary: initial vcs data when gatt init
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_vcs_initialize_data(void)
{
    unicast_sink_volume_t *p_vcs = &g_unicast_sink_gatt_cb.vcs_data;

    p_vcs->flag = 1;
    p_vcs->state.mute_state = WICED_BT_MUTE_STATE_NOT_MUTED;
    p_vcs->state.volume_setting = DEFAULT_VOLUME;

    // initialize volume in alsa driver
    vcs_set_volume(p_vcs, 0);
}

/******************************************************************************
 * Function Name: unicast_sink_vcs_vol_up 
 ******************************************************************************
 * Summary: volume up and notify to remote device
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_sink_vcs_vol_up(uint16_t conn_id)
{
    wiced_bt_ga_vcs_data_t vcs_data;
    unicast_sink_volume_t *p_vcs = &g_unicast_sink_gatt_cb.vcs_data;

    vcs_set_volume(p_vcs, +unicast_sink_ga_cfg.vcs_step_size);
    vcs_data.control_point_data.volume_state.mute_state = p_vcs->state.mute_state;
    vcs_data.control_point_data.volume_state.volume_setting = p_vcs->state.volume_setting;
    vcs_data.control_point_data.opcode = VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_UP;

    gatt_intf_attribute_t characteristic = {0};
    characteristic.characteristic_type = VCS_VOLUME_STATE_CHARACTERISTIC;

    return gatt_interface_notify_characteristic(conn_id,
                                                g_unicast_sink_gatt_cb.local_profiles.p_vcs,
                                                &characteristic,
                                                &vcs_data);
}

/******************************************************************************
 * Function Name: unicast_sink_vcs_vol_down 
 ******************************************************************************
 * Summary: volume down and notify to remote device
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_sink_vcs_vol_down(uint16_t conn_id)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    wiced_bt_ga_vcs_data_t data;
    wiced_bt_ga_vcs_data_t *p_data = &data;
    unicast_sink_volume_t *p_vcs = &g_unicast_sink_gatt_cb.vcs_data;

    vcs_set_volume(p_vcs, -unicast_sink_ga_cfg.vcs_step_size);
    p_data->control_point_data.volume_state.mute_state = p_vcs->state.mute_state;
    p_data->control_point_data.volume_state.volume_setting = p_vcs->state.volume_setting;
    p_data->control_point_data.opcode = VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_DOWN;

    gatt_intf_attribute_t characteristic = {0};
    characteristic.characteristic_type = VCS_VOLUME_STATE_CHARACTERISTIC;

    return gatt_interface_notify_characteristic(conn_id,
                                                g_unicast_sink_gatt_cb.local_profiles.p_vcs,
                                                &characteristic,
                                                p_data);
}


/******************************************************************************
 * Function Name: unicast_sink_vcs_mute 
 ******************************************************************************
 * Summary: mute and notify remote device
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_sink_vcs_mute(uint16_t conn_id)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    wiced_bt_ga_vcs_data_t data;
    wiced_bt_ga_vcs_data_t *p_data = &data;
    unicast_sink_volume_t *p_vcs = &g_unicast_sink_gatt_cb.vcs_data;

    p_data->control_point_data.volume_state.mute_state = WICED_BT_GA_VCS_MUTED;
    p_data->control_point_data.volume_state.volume_setting = p_vcs->state.volume_setting;
    p_data->control_point_data.opcode = VOLUME_CONTROL_OPCODE_MUTE;

    gatt_intf_attribute_t characteristic = {0};
    characteristic.characteristic_type = VCS_VOLUME_STATE_CHARACTERISTIC;

    p_vcs->state.mute_state = WICED_BT_GA_VCS_MUTED;

    audio_driver_set_mute_state(p_vcs->state.mute_state);

    return gatt_interface_notify_characteristic(conn_id,
                                                g_unicast_sink_gatt_cb.local_profiles.p_vcs,
                                                &characteristic,
                                                p_data);
}

/******************************************************************************
 * Function Name: unicast_sink_vcs_unmute 
 ******************************************************************************
 * Summary: unmute and notify remote device
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_sink_vcs_unmute(uint16_t conn_id)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    wiced_bt_ga_vcs_data_t data;
    wiced_bt_ga_vcs_data_t *p_data = &data;
    unicast_sink_volume_t *p_vcs = &g_unicast_sink_gatt_cb.vcs_data;

    p_data->control_point_data.volume_state.mute_state = WICED_BT_GA_VCS_NOT_MUTED;
    p_data->control_point_data.volume_state.volume_setting = p_vcs->state.volume_setting;
    p_data->control_point_data.opcode = VOLUME_CONTROL_OPCODE_MUTE;

    gatt_intf_attribute_t characteristic = {0};
    characteristic.characteristic_type = VCS_VOLUME_STATE_CHARACTERISTIC;

    p_vcs->state.mute_state = WICED_BT_GA_VCS_NOT_MUTED;

    audio_driver_set_mute_state(p_vcs->state.mute_state);

    return gatt_interface_notify_characteristic(conn_id,
                                                g_unicast_sink_gatt_cb.local_profiles.p_vcs,
                                                &characteristic,
                                                p_data);
}

/******************************************************************************
 * Function Name: unicast_sink_vcs_volume_set
 ******************************************************************************
 * Summary: set volume and notiy to remote device
 *
 * Parameters:
 *  uint16_t conn_id
 *  uint8_t vol
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_sink_vcs_volume_set(uint16_t conn_id, uint8_t vol)
{
    WICED_BT_TRACE("[%s] vol %x\n", __FUNCTION__, vol);
    wiced_bt_ga_vcs_data_t data;
    wiced_bt_ga_vcs_data_t *p_data = &data;
    unicast_sink_volume_t *p_vcs = &g_unicast_sink_gatt_cb.vcs_data;

    if (vol > WICED_BT_GA_VCS_MAXIMUM_VOLUME) return WICED_ERROR;

    p_vcs->state.volume_setting = vol;
    p_data->control_point_data.volume_state.mute_state = p_vcs->state.mute_state;
    p_data->control_point_data.volume_state.volume_setting = p_vcs->state.volume_setting;
    p_data->control_point_data.opcode = VOLUME_CONTROL_OPCODE_MUTE;

    gatt_intf_attribute_t characteristic = {0};
    characteristic.characteristic_type = VCS_VOLUME_STATE_CHARACTERISTIC;

    vcs_set_volume(p_vcs, 0);
    return gatt_interface_notify_characteristic(conn_id,
                                                g_unicast_sink_gatt_cb.local_profiles.p_vcs,
                                                &characteristic,
                                                p_data);
}

/******************************************************************************
 * Function Name: unicast_sink_vcs_unmute_relative_volume_up 
 ******************************************************************************
 * Summary: unumte and volume up
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_sink_vcs_unmute_relative_volume_up(uint16_t conn_id)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    wiced_bt_ga_vcs_data_t data;
    wiced_bt_ga_vcs_data_t *p_data = &data;
    unicast_sink_volume_t *p_vcs = &g_unicast_sink_gatt_cb.vcs_data;

    vcs_set_volume(p_vcs, +unicast_sink_ga_cfg.vcs_step_size);
    p_data->control_point_data.volume_state.volume_setting = p_vcs->state.volume_setting;
    p_data->control_point_data.opcode = VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_DOWN;

    gatt_intf_attribute_t characteristic = {0};
    characteristic.characteristic_type = VCS_VOLUME_STATE_CHARACTERISTIC;

    p_vcs->state.mute_state = WICED_BT_GA_VCS_NOT_MUTED;
    p_data->control_point_data.volume_state.mute_state = p_vcs->state.mute_state;

    audio_driver_set_mute_state(p_vcs->state.mute_state);
    le_audio_rpc_send_vcs_state_update(conn_id, p_vcs->state.volume_setting, p_vcs->state.mute_state, MUTE_STATUS);

    return gatt_interface_notify_characteristic(conn_id,
                                                g_unicast_sink_gatt_cb.local_profiles.p_vcs,
                                                &characteristic,
                                                p_data);
}

/******************************************************************************
 * Function Name: unicast_sink_vcs_unmute_relative_volume_down
 ******************************************************************************
 * Summary: unmute and volume down
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_sink_vcs_unmute_relative_volume_down(uint16_t conn_id)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    wiced_bt_ga_vcs_data_t data;
    wiced_bt_ga_vcs_data_t *p_data = &data;
    unicast_sink_volume_t *p_vcs = &g_unicast_sink_gatt_cb.vcs_data;

    vcs_set_volume(p_vcs, -unicast_sink_ga_cfg.vcs_step_size);
    p_data->control_point_data.volume_state.volume_setting = p_vcs->state.volume_setting;
    p_data->control_point_data.opcode = VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_UP;

    gatt_intf_attribute_t characteristic = {0};
    characteristic.characteristic_type = VCS_VOLUME_STATE_CHARACTERISTIC;

    p_vcs->state.mute_state = WICED_BT_GA_VCS_NOT_MUTED;
    p_data->control_point_data.volume_state.mute_state = p_vcs->state.mute_state;

    audio_driver_set_mute_state(p_vcs->state.mute_state);
    le_audio_rpc_send_vcs_state_update(conn_id, p_vcs->state.volume_setting, p_vcs->state.mute_state, MUTE_STATUS);

    return gatt_interface_notify_characteristic(conn_id,
                                                g_unicast_sink_gatt_cb.local_profiles.p_vcs,
                                                &characteristic,
                                                p_data);
}
