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
#include "unicast_sink_gatt.h"
#include "unicast_sink_mcs.h"
#include "unicast_sink_rpc.h"
#include "unicast_sink_vcs.h"

/* BT Stack includes */
#include "wiced_bt_trace.h"

/* App Library includes */
#include "le_audio_rpc.h"

/******************************************************************************
 * Function Name: unicast_sink_rpc_send_play_status
 ******************************************************************************
 * Summary: send play status to rpc
 *
 * Parameters:
 *  uint16_t conn_id
 *  uint8_t play_status
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_rpc_send_play_status(uint16_t conn_id, uint8_t play_status)
{
    uint8_t tx_buff[10];
    uint8_t *p_buff = &tx_buff[0];

    UINT16_TO_STREAM(p_buff, conn_id);
    UINT8_TO_STREAM(p_buff, play_status);

    le_audio_rpc_send_data(HCI_CONTROL_LE_AUDIO_PLAY_STATUS_EVT, tx_buff, (int)(p_buff - tx_buff));
}

/******************************************************************************
 * Function Name: unicast_sink_rpc_send_device_role_event
 ******************************************************************************
 * Summary: send device role to rpc
 *
 * Parameters:
 *  uint8_t dev_role
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_rpc_send_device_role_event(uint8_t dev_role)
{
    uint8_t tx_buff[2];
    uint8_t *p_buff = &tx_buff[0];

    UINT8_TO_STREAM(p_buff, dev_role);
    WICED_BT_TRACE("[%s] dev role %d \n", __FUNCTION__, dev_role);

    le_audio_rpc_send_data(HCI_CONTROL_LE_AUDIO_DEVICE_ROLE_EVT, tx_buff, (int)(p_buff - tx_buff));
}

/******************************************************************************
 * Function Name: unicast_sink_rpc_send_get_players_event
 ******************************************************************************
 * Summary: send player name to rpc
 *
 * Parameters:
 *  uint16_t conn_id
 *  unicast_sink_player_t *players
 *  int num
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_rpc_send_get_players_event(uint16_t conn_id, unicast_sink_player_t *players, int num)
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

    le_audio_rpc_send_data(HCI_CONTROL_LE_AUDIO_MEDIA_PLAYER_EVT, tx_buff, (int)(p_buff - tx_buff));
}

/******************************************************************************
 * Function Name: unicast_sink_rpc_send_disconnect_evt
 ******************************************************************************
 * Summary: transfer disconnection event to UART
 *
 * Parameters:
 *  uint16_t reason
 *  uint16_t conn_idx
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_rpc_send_disconnect_evt(uint16_t reason, uint16_t conn_idx)
{
    uint8_t tx_buf[4];
    uint8_t *p = tx_buf;

    UINT16_TO_STREAM(p, conn_idx);
    UINT16_TO_STREAM(p, reason);

    le_audio_rpc_send_data(HCI_CONTROL_LE_EVENT_DISCONNECTED, tx_buf, (int)(p - tx_buf));
}

/******************************************************************************
 * Function Name: rpc_handle_set_media_player
 ******************************************************************************
 * Summary: commands received from the client control to the application, set player_name
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
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

/******************************************************************************
 * Function Name: rpc_handle_get_media_players 
 ******************************************************************************
 * Summary: get media players, no use now
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
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

/******************************************************************************
 * Function Name: rpc_handle_paly
 ******************************************************************************
 * Summary: handle play audio
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void rpc_handle_play(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    unicast_sink_mcs_play_pause(conn_id, TRUE);
}

/******************************************************************************
 * Function Name: rpc_handle_pause
 ******************************************************************************
 * Summary: handle pause command
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void rpc_handle_pause(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    unicast_sink_mcs_play_pause(conn_id, FALSE);
}

/******************************************************************************
 * Function Name: rpc_handle_vol_up
 ******************************************************************************
 * Summary: handle volume up
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void rpc_handle_vol_up(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    unicast_sink_vcs_vol_up(conn_id);
}

/******************************************************************************
 * Function Name: rpc_handle_vol_down
 ******************************************************************************
 * Summary: handle volume down
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void rpc_handle_vol_down(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    unicast_sink_vcs_vol_down(conn_id);
}

/******************************************************************************
 * Function Name: rpc_handle_unmute_vol_down
 ******************************************************************************
 * Summary: unmute and volume down
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void rpc_handle_unmute_vol_down(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    unicast_sink_vcs_unmute_relative_volume_down(conn_id);
}

/******************************************************************************
 * Function Name: rpc_handle_unmute_vol_up
 ******************************************************************************
 * Summary: unmute and volume up
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void rpc_handle_unmute_vol_up(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    unicast_sink_vcs_unmute_relative_volume_up(conn_id);
}

/******************************************************************************
 * Function Name: rpc_handle_abs_vol
 ******************************************************************************
 * Summary: handle abs volume set
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void rpc_handle_abs_vol(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;
    uint8_t vol;

    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(vol, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d vol %d\n", __FUNCTION__, conn_id, payload_len, vol);

    unicast_sink_vcs_volume_set(conn_id, vol);
}

/******************************************************************************
 * Function Name: rpc_handle_mute
 ******************************************************************************
 * Summary: handle mute
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void rpc_handle_mute(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    unicast_sink_vcs_mute(conn_id);
}

/******************************************************************************
 * Function Name: rpc_handle_unmute
 ******************************************************************************
 * Summary: handle unmute
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void rpc_handle_unmute(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    unicast_sink_vcs_unmute(conn_id);
}

/******************************************************************************
 * Function Name: rpc_handle_adv
 ******************************************************************************
 * Summary: handle start or stop adv
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void rpc_handle_adv(uint8_t *p_data, uint8_t payload_len)
{
    uint8_t adv;

    STREAM_TO_UINT8(adv, p_data);

    WICED_BT_TRACE("[%s] adv %d len %d\n", __FUNCTION__, adv, payload_len);

    unicast_sink_gatt_start_stop_adv(adv);
}

/******************************************************************************
 * Function Name: rpc_handle_connect
 ******************************************************************************
 * Summary: handle connect, no use now
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void rpc_handle_connect(uint8_t *p_data, uint8_t payload_len)
{
    wiced_bt_device_address_t bd_addr;
    uint8_t addr_type;

    STREAM_TO_UINT8(addr_type, p_data);
    STREAM_TO_BDADDR(bd_addr, p_data);

    WICED_BT_TRACE("[%s] type %d address %B len %d\n", __FUNCTION__, addr_type, bd_addr, payload_len);
}

/******************************************************************************
 * Function Name: rpc_handle_disconnect 
 ******************************************************************************
 * Summary: handle disconnet, no use now
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void rpc_handle_disconnect(uint8_t *p_data, uint8_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);
}

/******************************************************************************
 * Function Name: unicast_sink_rpc_rx_callback 
 ******************************************************************************
 * Summary: rpc rx callback, register in le_audio_rpc_init
 *          receive command from Client Control
 *
 * Parameters:
 *  uint16_t opcode
 *  uint8_t *p_data
 *  uint32_t payload_len
 *
 * Return:
 *  wiced_bool_t
 *
******************************************************************************/
wiced_bool_t unicast_sink_rpc_rx_callback(uint16_t opcode, uint8_t *p_data, uint32_t payload_len)
{
    WICED_BT_TRACE("[%s] [cmd_opcode 0x%04x] (%d bytes)\n", __FUNCTION__, opcode, payload_len);

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
            rpc_handle_vol_up(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_VOL_DOWN:
            rpc_handle_vol_down(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_ABS_VOL:
            rpc_handle_abs_vol(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_MUTE:
            rpc_handle_mute(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE:
            rpc_handle_unmute(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_UP:
            rpc_handle_unmute_vol_up(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_DOWN:
            rpc_handle_unmute_vol_down(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_COMMAND_ADVERTISE:
            rpc_handle_adv(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_COMMAND_CONNECT:
            rpc_handle_connect(p_data, payload_len);
            break;
        case HCI_CONTROL_LE_COMMAND_CANCEL_CONNECT:
            rpc_handle_disconnect(p_data, payload_len);
            break;
        case HCI_CONTROL_MISC_COMMAND_PING:
            le_audio_rpc_send_data(HCI_CONTROL_MISC_EVENT_PING_REPLY, p_data, payload_len);
            break;
        case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
            le_audio_rpc_send_dev_role(HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SINK);
            break;
        default:
            WICED_BT_TRACE("unicast_sink_proc_hci_cmd: default case cmd_opcode 0x%04x\n", opcode);
            break;
    }

    return 0;
}
