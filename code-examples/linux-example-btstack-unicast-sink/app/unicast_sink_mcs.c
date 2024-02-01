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

#include "unicast_sink_mcs.h"
#include "unicast_sink_gatt.h"
#include "unicast_sink_rpc.h"

#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_mcs.h"

#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"

/******************************************************************************
 * Function Name: unicast_sink_mcs_play_pause( 
 ******************************************************************************
 * Summary: mcs play or pause
 *
 * Parameters:
 *  uint16_t conn_id
 *  wiced_bool_t play
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_sink_mcs_play_pause(uint16_t conn_id, wiced_bool_t play)
{
    wiced_bt_ga_mcs_data_t mcs_data;
    gatt_intf_attribute_t characteristic = {0};
    unicast_sink_clcb_t *p_clcb = unicast_sink_gatt_get_clcb_by_conn_id(conn_id);

    mcs_data.control_point_operation.opcode = play ? WICED_BT_GA_MCS_PLAY : WICED_BT_GA_MCS_PAUSE;
    characteristic.characteristic_type = MCS_MEDIA_CONTROL_POINT_CHARACTERISTIC;

    return gatt_interface_write_characteristic(conn_id, p_clcb->peer_profiles.p_gmcs, &characteristic, &mcs_data);
}

/******************************************************************************
 * Function Name: unicast_sink_mcs_callback
 ******************************************************************************
 * Summary: mcs callback, use in unicast_sink_store_service_ref_peer, as gatt callback function
 *
 * Parameters:
 *  uint16_t conn_id
 *  void *p_app_ctx
 *  const gatt_intf_service_object_t *p_service
 *  wiced_bt_gatt_status_t status
 *  uint32_t evt_type
 *  gatt_intf_attribute_t *p_char
 *  void *p_data
 *  int len
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_sink_mcs_callback(uint16_t conn_id,
                                         void *p_app_ctx,
                                         const gatt_intf_service_object_t *p_service,
                                         wiced_bt_gatt_status_t status,
                                         uint32_t evt_type,
                                         gatt_intf_attribute_t *p_char,
                                         void *p_data,
                                         int len)
{
    wiced_bt_ga_mcs_data_t *p_event_data = (wiced_bt_ga_mcs_data_t *)p_data;
    wiced_bt_ga_mcs_operation_data_t *p_mcs =
        (wiced_bt_ga_mcs_operation_data_t *)&p_event_data->control_point_operation.data;
    unicast_sink_clcb_t *p_clcb = unicast_sink_gatt_get_clcb_by_conn_id(conn_id);
    unicast_sink_mcs_data_t *mcs_data = &p_clcb->mcs_data;

    WICED_BT_TRACE("[%s] characteristic %x p_clcb 0x%x \n", __FUNCTION__, p_char, p_clcb);

    if (evt_type != READ_CMPL_EVT && evt_type != WRITE_CMPL_EVT && evt_type != NOTIFICATION_EVT) return WICED_ERROR;

    switch (p_char->characteristic_type)
    {
        case MCS_MEDIA_PLAYER_NAME_CHARACTERISTIC: {
            unicast_sink_player_t player;
            player.len = (p_event_data->media_player_name.len > MAX_PLAYER_NAME_LEN - 1)
                             ? MAX_PLAYER_NAME_LEN - 1
                             : p_event_data->media_player_name.len;
            memcpy(player.player_name, p_event_data->media_player_name.str, player.len);
            player.player_name[player.len] = '\0';
            WICED_BT_TRACE("Player name %s\n", player.player_name);
            unicast_sink_rpc_send_get_players_event(conn_id, &player, 1);
            break;
        }
        case MCS_MEDIA_STATE_CHARACTERISTIC:
            WICED_BT_TRACE("Media State %d \n", p_event_data->media_state);
            p_clcb->mcs_data.media_state = p_event_data->media_state;
            unicast_sink_rpc_send_play_status(conn_id, p_event_data->media_state);
            break;
        case MCS_MEDIA_CONTROL_POINT_CHARACTERISTIC:
            WICED_BT_TRACE("Media Operation Result Code 0x%x for opcode 0x%x \n",
                           p_event_data->control_point_operation.result,
                           p_event_data->control_point_operation.opcode);
            break;
        case MCS_MEDIA_TRACK_CHANGED_CHARACTERISTIC:
            WICED_BT_TRACE("Track Changed \n");
            break;
        case MCS_MEDIA_TRACK_TITLE_CHARACTERISTIC:
            WICED_MEMSET(mcs_data->track_title, 0, MAX_MEDIA_TRACK_TITLE_LEN);
            WICED_MEMCPY(mcs_data->track_title, p_event_data->track_title.str, p_event_data->track_title.len);
            if (p_event_data->track_title.len) WICED_BT_TRACE("Track Title %s \n", p_event_data->track_title.str);
            break;
        case MCS_MEDIA_TRACK_DURATION_CHARACTERISTIC:
            mcs_data->track_duration = p_event_data->track_duration;
            WICED_BT_TRACE("Track Duration %d \n", p_event_data->track_duration);
            break;
        case MCS_MEDIA_TRACK_POSITION_CHARACTERISTIC:
            mcs_data->track_position = p_event_data->track_position;
            WICED_BT_TRACE("Track Position %d \n", p_event_data->track_position);
            break;
        case MCS_MEDIA_PLAYBACK_SPEED_CHARACTERISTIC:
            mcs_data->playback_speed = p_event_data->playback_speed;
            WICED_BT_TRACE("Track Playback Speed %lf \n", p_event_data->playback_speed);
            break;
        case MCS_MEDIA_SEEKING_SPEED_CHARACTERISTIC:
            mcs_data->seeking_speed = p_event_data->seeking_speed;
            WICED_BT_TRACE("Track Seeking Speed %d \n", p_event_data->seeking_speed);
            break;
        case MCS_MEDIA_PLAYING_ORDER_CHARACTERISTIC:
            mcs_data->playing_order = p_event_data->playing_order;
            WICED_BT_TRACE("Playing Order %d \n", p_event_data->playing_order);
            break;
        case MCS_MEDIA_PLAYING_ORDER_SUPPORTED_CHARACTERISTIC:
            mcs_data->playing_order_supported = p_event_data->playing_order_supported;
            WICED_BT_TRACE("Playing Order Supported %d \n", p_event_data->playing_order_supported);
            break;
        case MCS_CONTENT_CONTROL_ID_CHARACTERISTIC:
            mcs_data->content_control_id = p_event_data->content_control_id;
            WICED_BT_TRACE("Content Control Id %d \n", p_event_data->content_control_id);
            break;
        case MCS_MEDIA_OPCODE_SUPPORTED_CHARACTERISTIC:
            mcs_data->media_control_supported_opcodes = p_event_data->media_control_supported_opcodes;
            WICED_BT_TRACE("Supported Opcode 0x%x \n", p_event_data->media_control_supported_opcodes);
            break;
        default:
            WICED_BT_TRACE_CRIT("[%s] Unknow char %d ", __FUNCTION__, p_char);
            break;
    }

    return WICED_BT_SUCCESS;
}
