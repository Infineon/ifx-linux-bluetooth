/*
 * $ Copyright Cypress Semiconductor $
 */
#include "lehs.h"

wiced_result_t lehs_mcs_play_pause(uint16_t conn_id, wiced_bool_t play)
{
    wiced_bt_ga_mcs_data_t mcs_data;
    gatt_intf_attribute_t characteristic = {0};
    lehs_clcb_t *p_clcb = lehs_gatt_get_clcb_by_conn_id(conn_id);

    if (!p_clcb)
    {
        WICED_BT_TRACE_CRIT("[%s] No p_clcb",__FUNCTION__);
        return WICED_ERROR;
    }

    mcs_data.control_point_operation.opcode = play ? WICED_BT_GA_MCS_PLAY : WICED_BT_GA_MCS_PAUSE;
    characteristic.characteristic_type = MCS_MEDIA_CONTROL_POINT_CHARACTERISTIC;

    return gatt_interface_write_characteristic(conn_id, p_clcb->peer_profiles.p_gmcs, &characteristic, &mcs_data);
}

wiced_result_t lehs_mcs_callback(uint16_t conn_id,
                                         void *p_app_ctx,
                                         gatt_intf_service_object_t *p_service,
                                         wiced_bt_gatt_status_t status,
                                         uint32_t evt_type,
                                         gatt_intf_attribute_t *p_char,
                                         void *p_data,
                                         int len)
{
    wiced_bt_ga_mcs_data_t *p_event_data = (wiced_bt_ga_mcs_data_t *)p_data;
    lehs_clcb_t *p_clcb = lehs_gatt_get_clcb_by_conn_id(conn_id);
    lehs_mcs_data_t *mcs_data = &p_clcb->mcs_data;

    WICED_BT_TRACE("[%s] characteristic %x p_clcb 0x%x \n", __FUNCTION__, p_char, p_clcb);

    if (evt_type != READ_CMPL_EVT && evt_type != WRITE_CMPL_EVT && evt_type != NOTIFICATION_EVT) return WICED_ERROR;

    switch (p_char->characteristic_type)
    {
        case MCS_MEDIA_PLAYER_NAME_CHARACTERISTIC: {
            lehs_player_t player;
            player.len = (p_event_data->media_player_name.len > MAX_PLAYER_NAME_LEN - 1)
                             ? MAX_PLAYER_NAME_LEN - 1
                             : p_event_data->media_player_name.len;
            memcpy(player.player_name, p_event_data->media_player_name.str, player.len);
            player.player_name[player.len] = '\0';
            WICED_BT_TRACE("Player name %s\n", player.player_name);
            lehs_rpc_send_get_players_event(conn_id, &player, 1);
            break;
        }
        case MCS_MEDIA_STATE_CHARACTERISTIC:
            WICED_BT_TRACE("Media State %d \n", p_event_data->media_state);
            p_clcb->mcs_data.media_state = p_event_data->media_state;
            lehs_rpc_send_play_status(conn_id, p_event_data->media_state);
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
