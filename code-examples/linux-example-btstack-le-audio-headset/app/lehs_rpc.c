/*
 * $ Copyright Cypress Semiconductor $
 */

/* Application includes */
#include "lehs.h"
#include "audio_driver.h"
#include "lehs_rpc.h"

/* BT Stack includes */
#include "wiced_bt_ga_tbs.h"

void lehs_rpc_send_play_status(uint16_t conn_id, uint8_t play_status)
{
    uint8_t tx_buff[10];
    uint8_t *p_buff = &tx_buff[0];

    UINT16_TO_STREAM(p_buff, conn_id);
    UINT8_TO_STREAM(p_buff, play_status);

    le_audio_rpc_send_data(HCI_CONTROL_LE_AUDIO_EVENT_PLAY_STATUS, tx_buff, (int)(p_buff - tx_buff));
}

void lehs_rpc_send_device_role_event(uint8_t dev_role)
{
    uint8_t tx_buff[2];
    uint8_t *p_buff = &tx_buff[0];

    UINT8_TO_STREAM(p_buff, dev_role);
    WICED_BT_TRACE("[%s] dev role %d \n", __FUNCTION__, dev_role);

    le_audio_rpc_send_data(HCI_CONTROL_LE_AUDIO_EVENT_DEVICE_ROLE, tx_buff, (int)(p_buff - tx_buff));
}

void lehs_rpc_send_get_players_event(uint16_t conn_id, lehs_player_t *players, int num)
{
    uint8_t tx_buff[50];
    uint8_t *p_buff = &tx_buff[0];

    WICED_BT_TRACE("[%s] connid %d \n", __FUNCTION__, conn_id);
    WICED_BT_TRACE("[%s] num players %d \n", __FUNCTION__, num);

    UINT16_TO_STREAM(p_buff, conn_id);
    UINT8_TO_STREAM(p_buff, num);

    for (int i = 0; i < num; i++)
    {
        UINT8_TO_STREAM(p_buff, players[i].len);
        ARRAY_TO_STREAM(p_buff, players[i].player_name, players[i].len);
        WICED_BT_TRACE("[%s] player name %s \n", __FUNCTION__, players[i].player_name);
    }

    le_audio_rpc_send_data(HCI_CONTROL_LE_AUDIO_EVENT_MEDIA_PLAYER, tx_buff, (int)(p_buff - tx_buff));
}



//commands received from the client control to the application
static void rpc_get_bis_info(uint8_t *p_data, uint8_t payload_len)
{
    uint32_t broadcast_id;
    STREAM_TO_UINT32(broadcast_id, p_data);
    lehs_sync_to_pa(broadcast_id);
}

static void rpc_cancel_pa_sync()
{
    if (lehs_broadcast_get_synk_progress())
        wiced_ble_padv_cancel_sync();
}

static void rpc_terminate_pa_sync(uint8_t* p_data, uint8_t payload_len)
{
    uint32_t broadcast_id;
    STREAM_TO_UINT32(broadcast_id, p_data);

    broadcast_sink_cb_t *p_big = lehs_bis_get_big_by_broadcast_id(broadcast_id);
    if (p_big == NULL)
    {
        WICED_BT_TRACE("[%s] p_big is null", __FUNCTION__);
        return;
    }

    if (p_big->sync_state == HCI_CONTROL_LEA_BROADCAST_PA_SYNC_ESTABLISHED)
        wiced_ble_padv_terminate_sync(p_big->sync_handle);

    lehs_bis_free_big(p_big);
}

static void rpc_set_csis_params(uint8_t *p_data, uint8_t payload_len)
{
    wiced_bt_ga_csis_sirk_data_t sirk;
    uint32_t location;
    uint8_t rank;
    uint8_t size;

    sirk.is_oob = 0;
    STREAM_TO_ARRAY(sirk.sirk, p_data, sizeof(wiced_bt_ga_csis_sirk_t));
    STREAM_TO_UINT32(location, p_data);
    STREAM_TO_UINT8(sirk.sirk_type, p_data);
    STREAM_TO_UINT8(size, p_data);
    STREAM_TO_UINT8(rank, p_data);

    lehs_csis_set_sirk(&sirk);
    lehs_csis_set_rank(rank);
    lehs_csis_set_size(size);
    lehs_set_audio_location(location);

    WICED_BT_TRACE("[%s] stype %d size %d sirk %A loc 0x%04x rank %d", __FUNCTION__,
        sirk.sirk_type, size, sirk.sirk, sizeof(wiced_bt_ga_csis_sirk_t), location, rank);
}

static void rpc_handle_set_media_player(uint8_t *p_data, uint8_t payload_len)
{
    char player_name[MAX_PLAYER_NAME_LEN];
    uint16_t conn_id;
    uint8_t player_len;

    memset(player_name, 0, MAX_PLAYER_NAME_LEN);

    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(player_len, p_data);

    memcpy(player_name, p_data, player_len);

    WICED_BT_TRACE("[%s] conn_id %d len %d player_name %s\n", __FUNCTION__, conn_id, payload_len, player_name);
}

static void rpc_handle_get_media_players(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);
    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    //    unicast_sink_player_t player;
    //    memcpy(player.player_name, player_name, 9);
    //    player.player_name[9] = '\0';
    //    player.len = 9;
    //    unicast_sink_le_audio_get_players_event(0x8000, &player, 1);
}

static void rpc_handle_play(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    lehs_mcs_play_pause(conn_id, TRUE);
}

static void rpc_handle_pause(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    lehs_mcs_play_pause(conn_id, FALSE);
}

static void rpc_handle_set_vol(uint8_t *p_data, uint8_t payload_len, uint32_t opcode)
{
    uint16_t conn_id;
    volume_control_opcodes_t vcs_opcode = 0xFF;
    uint8_t vol = 0;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    switch (opcode)
    {
    case HCI_CONTROL_LE_AUDIO_COMMAND_VOL_DOWN:
        vcs_opcode = VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_DOWN;
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_VOL_UP:
        vcs_opcode =  VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_UP;
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_ABS_VOL:
        STREAM_TO_UINT8(vol, p_data);
        vcs_opcode =  VOLUME_CONTROL_OPCODE_SET_ABSOLUTE_VOLUME;
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_MUTE:
        vcs_opcode =  VOLUME_CONTROL_OPCODE_MUTE;
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE:
        vcs_opcode = VOLUME_CONTROL_OPCODE_UNMUTE;
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_UP:
        vcs_opcode = VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_UP;
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_DOWN:
        vcs_opcode = VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_DOWN;
        break;
    default:
        WICED_BT_TRACE("[%s] opcode %d", __FUNCTION__, opcode);
        break;
    }
    if (conn_id)
    {
        lehs_vcs_set_volume(conn_id, vcs_opcode, vol);
    }
    else
    {
        lehs_set_vol(vcs_opcode, vol);
    }
}

static void rpc_handle_adv(uint8_t *p_data, uint8_t payload_len)
{
    uint8_t adv, swift_pair = 0, tx_power = 0;
    uint16_t adv_data_options = 0xffff;

    STREAM_TO_UINT8(adv, p_data);
    if (payload_len > 1)
    {
        STREAM_TO_UINT8(tx_power, p_data);
    }
    if (payload_len > 2)
    {
        STREAM_TO_UINT8(swift_pair, p_data);
    }
    if (payload_len > 3)
    {
        STREAM_TO_UINT16(adv_data_options, p_data);
    }
    g_lehs_gatt_cb.adv_tx_power = tx_power;
    g_lehs_gatt_cb.adv_data_options = adv_data_options;
    g_lehs_gatt_cb.do_swift_pair = swift_pair;

    WICED_BT_TRACE("[%s] adv %d %d pwr 0x%x opt 0x%x len %d\n", __FUNCTION__,
        adv, swift_pair, tx_power, adv_data_options, payload_len);

    if (!adv)
    {
        g_lehs_gatt_cb.adv_state = ADV_STATE_MAX;
    }

    if (g_lehs_gatt_cb.adv_state > ADV_STATE_MAX)
    {
        g_lehs_gatt_cb.adv_state = 0;
    }

    lehs_move_to_next_adv_state(g_lehs_gatt_cb.adv_state, "rpc");
}


static void rpc_handle_connect(uint8_t *p_data, uint8_t payload_len, int cancel)
{
    wiced_bt_device_address_t bd_addr;
    uint8_t addr_type;
    wiced_result_t status;

    STREAM_TO_UINT8(addr_type, p_data);
    STREAM_TO_BDADDR(bd_addr, p_data);

    if (cancel == 0)
    {
        status = app_create_connection(addr_type, bd_addr);
    }
    else
    {
        status = wiced_bt_gatt_cancel_connect(bd_addr, 1);
    }

    WICED_BT_TRACE("[%s] %sconnect type %d address %B len %d status %d\n", __FUNCTION__,
        cancel ? "cancel" : "", addr_type, bd_addr, payload_len, status);
}

static void rpc_handle_disconnect(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);
    wiced_bt_gatt_disconnect(conn_id);
}

/* Indicate if the device is sink capable of scanning itself or require Broadcast Assistant to scan on its behalf */
static wiced_bool_t is_dev_role_sink = FALSE;
#define MAX_BROADCAST_CODE_LEN 16

wiced_bool_t lehs_rpc_is_dev_role_sink(void)
{
    return is_dev_role_sink;
}

static void rpc_handle_find_broadcast_sources(uint8_t *p_data, uint32_t data_len)
{
    uint8_t start;

    STREAM_TO_UINT8(start, p_data);
    WICED_BT_TRACE("[%s] [start:%d]\n", __FUNCTION__, start);

    is_dev_role_sink = TRUE;

    lehs_bis_discover_sources(start);
}

static void rpc_handle_sync_to_broadcast_source(uint8_t *p_data, uint32_t data_len)
{
    broadcast_source_t source = {0};
    uint8_t listen;

    STREAM_TO_UINT8(listen, p_data);
    STREAM_TO_UINT32(source.broadcast_id, p_data);
    WICED_BT_TRACE("Broadcast ID: %x", source.broadcast_id);


    if (!listen)
    {
        lehs_bis_terminate_sync(source.broadcast_id);
    }
    else
    {
        STREAM_TO_UINT32(source.bis_index_bits, p_data);
        if (!source.bis_index_bits)
        {
            source.bis_index_bits = 1;
        }
        uint8_t br_code_flag;
        STREAM_TO_UINT8(br_code_flag, p_data);
        if (br_code_flag)
        {
            STREAM_TO_ARRAY(&source.broadcast_code, p_data, MAX_BROADCAST_CODE_LEN);
            WICED_BT_TRACE_ARRAY(&source.broadcast_code, MAX_BROADCAST_CODE_LEN, "[%s] broadcast code\n");
        }
        lehs_bis_sync_to_source(source);
    }
}

//call control app
void lehs_rpc_update_call_frindly_name(uint16_t conn_id, char *friendly_name)
{
    uint8_t tx_buf[64];
    uint8_t f_name_len = (uint8_t)strlen(friendly_name);
    uint8_t *p = tx_buf;

    UINT16_TO_STREAM(p, conn_id);
    if (f_name_len >= WICED_BT_GA_TBS_FRIENDLY_NAME_MAX_SIZE) f_name_len = WICED_BT_GA_TBS_FRIENDLY_NAME_MAX_SIZE - 1;
    UINT8_TO_STREAM(p, f_name_len);
    ARRAY_TO_STREAM(p, friendly_name, f_name_len);

    WICED_BT_TRACE("[%s] friendly_name %s f_name_len %d \n", __FUNCTION__, friendly_name, f_name_len);

    le_audio_rpc_send_data(HCI_CONTROL_LE_AUDIO_EVENT_CALL_FRIENDLY_NAME, tx_buf, (int)(p - tx_buf));
}

void rpc_handle_call_control_point_action(uint8_t *p_data, uint8_t data_len, uint8_t action){
    uint8_t call_id;
    uint16_t conn_id;
    gatt_intf_attribute_t characteristic = {0};

    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(call_id, p_data);

    lehs_clcb_t *p_clcb = lehs_gatt_get_clcb_by_conn_id(conn_id);
    WICED_BT_TRACE("[%s] conn_id %d call_id %d\n", __FUNCTION__, conn_id, call_id);
    if (!p_clcb)
    {
        WICED_BT_TRACE_CRIT("[%s] no clcb", __FUNCTION__);
        return;
    }

    characteristic.characteristic_type = TBS_CALL_CONTROL_POINT_CHARACTERISTIC;

    lehs_handle_call_control_point_action(conn_id,
                                          call_id,
                                          p_clcb->peer_profiles.p_gtbs,
                                          &characteristic,
                                          action);

}

static void rpc_handle_call_terminate(uint8_t *p_data, uint8_t data_len)
{
    uint8_t call_id;
    uint16_t conn_id;
    uint8_t reason;
    gatt_intf_attribute_t characteristic = {0};

    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(call_id, p_data);
    STREAM_TO_UINT8(reason, p_data);

    lehs_clcb_t *p_clcb = lehs_gatt_get_clcb_by_conn_id(conn_id);
    WICED_BT_TRACE("[%s] conn_id %d call_id %d\n", __FUNCTION__, conn_id, call_id);

    if (!p_clcb)
    {
        WICED_BT_TRACE_CRIT("[%s] no clcb", __FUNCTION__);
        return;
    }

    characteristic.characteristic_type = TBS_CALL_CONTROL_POINT_CHARACTERISTIC;
    lehs_terminate_incoming_call(conn_id,
                                 call_id,
                                 reason,
                                 p_clcb->peer_profiles.p_gtbs,
                                 &characteristic);
}
//end call control

static void rpc_mics_mute(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    uint8_t mute;
    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(mute, p_data);

    WICED_BT_TRACE("[%s] conn_id %d mute %d\n", __FUNCTION__, conn_id, mute);
    lehs_mics_mute(conn_id, mute);
}

static void rpc_mics_aics_mute(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    uint8_t mute;
    uint32_t instance;
    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(mute, p_data);
    STREAM_TO_UINT32(instance, p_data);

    lehs_mics_aics_mute(conn_id, instance, mute);
}

static void rpc_mics_aics_set_gain(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    uint8_t opcode;
    uint32_t instance;
    int8_t gain = 0;
    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(opcode, p_data);
    STREAM_TO_UINT32(instance, p_data);
    if (data_len - 7) STREAM_TO_INT8(gain, p_data);

    lehs_mics_aics_set_gain(conn_id, instance, opcode, gain);
}

wiced_bool_t lehs_rpc_rx_callback(uint16_t opcode, uint8_t *p_data, uint32_t payload_len)
{
    WICED_BT_TRACE("[%s] [opcode 0x%04x] (%d bytes)\n", __FUNCTION__, opcode, payload_len);

    switch (opcode)
    {
        case HCI_CONTROL_LE_AUDIO_COMMAND_GET_MEDIA_PLAYERS:
            rpc_handle_get_media_players(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_SET_MEDIA_PLAYER:
            rpc_handle_set_media_player(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_PLAY:
            rpc_handle_play(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_PAUSE:
            rpc_handle_pause(p_data, payload_len);
            break;

        case HCI_CONTROL_LE_AUDIO_COMMAND_VOL_UP:
        case HCI_CONTROL_LE_AUDIO_COMMAND_VOL_DOWN:
        case HCI_CONTROL_LE_AUDIO_COMMAND_ABS_VOL:
        case HCI_CONTROL_LE_AUDIO_COMMAND_MUTE:
        case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE:
        case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_UP:
        case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_DOWN:
            rpc_handle_set_vol(p_data, payload_len, opcode);
            break;

        case HCI_CONTROL_LE_COMMAND_ADVERTISE:
            rpc_handle_adv(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_COMMAND_CONNECT:
            rpc_handle_connect(p_data, payload_len, 0);
            break;
        case HCI_CONTROL_LE_COMMAND_CANCEL_CONNECT:
            rpc_handle_connect(p_data, payload_len, 1);
            break;
        case HCI_CONTROL_LE_COMMAND_DISCONNECT:
            rpc_handle_disconnect(p_data, payload_len);
            break;
        case HCI_CONTROL_MISC_COMMAND_PING:
            le_audio_rpc_send_data(HCI_CONTROL_MISC_EVENT_PING_REPLY, p_data, payload_len);
            break;
        case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
            le_audio_rpc_send_dev_role(HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SINK |
                                       HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK |
                                       HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_CLIENT);
            gatt_intf_string_t aics_desc;
            aics_desc.len = g_lehs_gatt_cb.mics_data.aics->description_len;
            aics_desc.str = g_lehs_gatt_cb.mics_data.aics->description;
            le_audio_rpc_send_micp_aics_description(0, 0, &aics_desc);
            le_audio_rpc_send_micp_mute_state(0, g_lehs_gatt_cb.mics_data.mute_state);
            le_audio_rpc_send_micp_aics_input_state(0,
                                                    0,
                                                    &g_lehs_gatt_cb.mics_data.aics->input_state);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_SET_CSIS_PARAMS:
            rpc_set_csis_params(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_BROADCAST_SINK_FIND_SOURCES:
            rpc_handle_find_broadcast_sources(p_data, payload_len);
            break;

        case HCI_CONTROL_LE_AUDIO_COMMAND_BROADCAST_SINK_SYNC_TO_SOURCES:
            rpc_handle_sync_to_broadcast_source(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_GET_BIS_INFO:
            rpc_get_bis_info(p_data, payload_len);
            break;

       case HCI_CONTROL_LE_AUDIO_COMMAND_CANCEL_PA_SYNC:
            rpc_cancel_pa_sync();
            break;

       case HCI_CONTROL_LE_AUDIO_COMMAND_TERMINATE_PA_SYNC:
           rpc_terminate_pa_sync(p_data, payload_len);
           break;

            // CCP operations
        case HCI_CONTROL_LE_AUDIO_COMMAND_ACCEPT_CALL:
            rpc_handle_call_control_point_action(p_data, payload_len, WICED_BT_GA_CCP_ACTION_ACCEPT_CALL);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_HOLD_CALL:
            rpc_handle_call_control_point_action(p_data, payload_len, WICED_BT_GA_CCP_ACTION_HOLD_CALL);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_RETRIEVE_CALL:
            rpc_handle_call_control_point_action(p_data, payload_len, WICED_BT_GA_CCP_ACTION_RETRIEVE_CALL);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_ORIGINATE_CALL:
            rpc_handle_call_control_point_action(p_data, payload_len, WICED_BT_GA_CCP_ACTION_ORIGINATE);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_JOIN_CALL:
            rpc_handle_call_control_point_action(p_data, payload_len, WICED_BT_GA_CCP_ACTION_JOIN_CALL);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_TERMINATE_CALL:
            rpc_handle_call_terminate(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_MICP_MUTE:
            rpc_mics_mute(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_MICP_AICS_MUTE:
            rpc_mics_aics_mute(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_MICP_AICS_SET_GAIN:
            rpc_mics_aics_set_gain(p_data, payload_len);
            break;

        default:
            WICED_BT_TRACE("[%s]: default case cmd_opcode 0x%04x\n",__FUNCTION__, opcode);
            break;
    }

    return 0;
}
