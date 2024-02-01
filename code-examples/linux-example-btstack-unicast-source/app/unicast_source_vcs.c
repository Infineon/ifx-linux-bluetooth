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

#include "unicast_source_vcs.h"
#include "unicast_source_gatt.h"
#include "wiced_bt_ga_common.h"

#include "wiced_bt_ga_vcp.h"
#include "wiced_bt_ga_vcs.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "le_audio_rpc.h"

/******************************************************************************
 * Function Name: unicast_source_vcs_vol_up
 *
 * Summary: source set volume up
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_source_vcs_vol_up(uint16_t conn_id)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    return wiced_bt_ga_cap_relative_volume_up(&g_unicast_source_gatt_cb.cap_profile_data);
}

/******************************************************************************
 * Function Name: unicast_source_vcs_vol_down
 *
 * Summary: source set volume down
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_source_vcs_vol_down(uint16_t conn_id)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    return wiced_bt_ga_cap_relative_volume_down(&g_unicast_source_gatt_cb.cap_profile_data);
}

/******************************************************************************
 * Function Name: unicast_source_vcs_mute
 *
 * Summary: source set audio mute
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_source_vcs_mute(uint16_t conn_id)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    return wiced_bt_ga_cap_set_volume_mute_state(&g_unicast_source_gatt_cb.cap_profile_data, WICED_BT_MUTE_STATE_MUTED);
}

/******************************************************************************
 * Function Name: unicast_source_vcs_unmute
 *
 * Summary: source set audio unmute
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_source_vcs_unmute(uint16_t conn_id)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    return wiced_bt_ga_cap_set_volume_mute_state(&g_unicast_source_gatt_cb.cap_profile_data,
                                                 WICED_BT_MUTE_STATE_NOT_MUTED);
}

/******************************************************************************
 * Function Name: unicast_source_vcs_unmute_relative_volume_down
 *
 * Summary: source set unmute and volume down
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_source_vcs_unmute_relative_volume_down(uint16_t conn_id)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    return wiced_bt_ga_cap_unmute_relative_volume_down(&g_unicast_source_gatt_cb.cap_profile_data);
}

/******************************************************************************
 * Function Name: unicast_source_vcs_unmute_relative_volume_up
 *
 * Summary: source set unmute and volume up 
 *
 * Parameters:
 *
 * Return:
 *
******************************************************************************/
wiced_result_t unicast_source_vcs_unmute_relative_volume_up(uint16_t conn_id)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    return wiced_bt_ga_cap_unmute_relative_volume_up(&g_unicast_source_gatt_cb.cap_profile_data);
}

/******************************************************************************
 * Function Name: unicast_source_vcs_volume_set
 *
 * Summary: source set abs volume
 *
 * Parameters:
 *  uint16_t    conn_id
 *  uint8_t     vol
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_source_vcs_volume_set(uint16_t conn_id, uint8_t vol)
{
    WICED_BT_TRACE("[%s] vol %x\n", __FUNCTION__, vol);
    return wiced_bt_ga_cap_set_absolute_volume(&g_unicast_source_gatt_cb.cap_profile_data, vol);
}

/******************************************************************************
 * Function Name: unicast_source_handle_vcs_data
 *
 * Summary: source handle data of volume control service 
 *         when read/write complete or get Notify 
 *
 * Parameters:
 *  uint16_t                            conn_id,
 *  void                                *p_app_ctx,
 *  const gatt_intf_service_object_t    *p_volume,
 *  wiced_bt_gatt_status_t              status,
 *  uint32_t                            evt_type
 *  gatt_intf_attribute_t               *p_char
 *  wiced_bt_ga_vcs_data_t              *p_data_ptr
 *  int                                 len
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_source_handle_vcs_data(uint16_t conn_id,
                                              void *p_app_ctx,
                                              const gatt_intf_service_object_t *p_volume,
                                              wiced_bt_gatt_status_t status,
                                              uint32_t evt_type,
                                              gatt_intf_attribute_t *p_char,
                                              wiced_bt_ga_vcs_data_t *p_data_ptr,
                                              int len)
{
    wiced_bt_ga_vcs_data_t *p_event_data = (wiced_bt_ga_vcs_data_t *)p_data_ptr;

    if (evt_type == WRITE_CMPL_EVT) {
        wiced_bt_ga_vcs_volume_state_t *p_v =
            (wiced_bt_ga_vcs_volume_state_t *)&p_event_data->control_point_data.volume_state;

        return WICED_BT_SUCCESS;
    }

    WICED_BT_TRACE("[%s] event %x status %x \n", __FUNCTION__, p_char, status);

    if (status == WICED_BT_GATT_SUCCESS)
    {
        switch (p_char->included_service_type)
        {
        case INCLUDED_SERVICE_NONE:
        {
            switch (p_char->characteristic_type)
            {
            case VCS_VOLUME_STATE_CHARACTERISTIC:
                    WICED_BT_TRACE("[%s] event %d volume_setting:%d mute_state:%x \n",
                                   __FUNCTION__,
                                   p_char,
                                   p_event_data->control_point_data.volume_state.volume_setting,
                                   p_event_data->control_point_data.volume_state.mute_state);
                le_audio_rpc_send_vcs_state_update(conn_id,
                                             p_event_data->control_point_data.volume_state.volume_setting,
                                             p_event_data->control_point_data.volume_state.mute_state,
                                             MUTE_AND_VOLUME_STATUS);
                break;
            case VCS_VOLUME_FLAG_CHARACTERISTIC:
                WICED_BT_TRACE("[%s] event %d volume_flag:%x \n", __FUNCTION__, p_char, p_event_data->volume_flag);
                break;
            case VCS_CONTROL_POINT_CHARACTERISTIC:
                WICED_BT_TRACE("[%s] event %d write_complete:%x \n", __FUNCTION__, p_char, status);
                break;
            }
        }break;
        }
    }
    return WICED_BT_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_source_vcs_callback
 *
 * Summary: source volume control service callback
 *
 * Parameters:
 *  uint16_t conn_id
 *  void                                *p_app_ctx
 *  const gatt_intf_service_object_t    *p_service,
 *  wiced_bt_gatt_status_t              status
 *  uint32_t                            evt_type
 *  gatt_intf_attribute_t               *p_char
 *  void                                *p_data
 *  int                                 len
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_source_vcs_callback(uint16_t conn_id,
                                           void *p_app_ctx,
                                           const gatt_intf_service_object_t *p_service,
                                           wiced_bt_gatt_status_t status,
                                           uint32_t evt_type,
                                           gatt_intf_attribute_t *p_char,
                                           void *p_data,
                                           int len)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;
    WICED_BT_TRACE("[%s] event 0x%x \n", __FUNCTION__, p_char);

    switch (evt_type) {
        case READ_CMPL_EVT:
        case WRITE_CMPL_EVT:
        case NOTIFICATION_EVT:
            return unicast_source_handle_vcs_data(conn_id,
                                                  p_app_ctx,
                                                  p_service,
                                                  status,
                                                  evt_type,
                                                  p_char,
                                                  (wiced_bt_ga_vcs_data_t *)p_data,
                                                  len);
            break;
    }
    return WICED_BT_SUCCESS;
}
