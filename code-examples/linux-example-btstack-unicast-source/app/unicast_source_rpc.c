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
#include "unicast_source_rpc.h"
#include "unicast_source_gatt.h"
#include "unicast_source_vcs.h"
#include "unicast_source_mcs.h"
#include "hci_control_api.h"

/* App Library includes */
#include "le_audio_rpc.h"

/* BT Stack includes */
#include "wiced_bt_trace.h"

/******************************************************************************
 * Function Name: unicast_source_rpc_send_device_role_event
 *
 * Summary: send device role to rpc client (Client Control)
 *
 * Parameters:
 *  uint8_t dev_role
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_rpc_send_device_role_event(uint8_t dev_role)
{
    uint8_t tx_buff[2];
    uint8_t *p_buff = &tx_buff[0];

    UINT8_TO_STREAM(p_buff, dev_role);
    WICED_BT_TRACE("[%s] dev role %d \n", __FUNCTION__, dev_role);

    le_audio_rpc_send_data(HCI_CONTROL_LE_AUDIO_DEVICE_ROLE_EVT, tx_buff, (int)(p_buff - tx_buff));
}

/******************************************************************************
 * Function Name: unicast_source_rpc_handle_scan
 *
 * Summary: handle scan start or stop from rpc client
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint32_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_rpc_handle_scan(uint8_t *p_data, uint32_t data_len)
{
    uint8_t scan;

    STREAM_TO_UINT8(scan, p_data);

    WICED_BT_TRACE("[%s] adv %d len %d\n", __FUNCTION__, scan, data_len);

    unicast_source_gatt_start_stop_scan(scan);
}

/******************************************************************************
 * Function Name: unicast_source_rpc_handle_connect
 *
 * Summary: handle connect command from rpc client
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint32_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_rpc_handle_connect(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_device_address_t bd_addr;
    uint8_t addr_type;
    wiced_result_t status;
    STREAM_TO_UINT8(addr_type, p_data);
    STREAM_TO_BDADDR(bd_addr, p_data);

    WICED_BT_TRACE("[%s] type %d address %B len %d\n", __FUNCTION__, addr_type, bd_addr, data_len);
    status = unicast_source_gatt_connect(bd_addr, addr_type);
    if(status!=WICED_SUCCESS)
    WICED_BT_TRACE("CONNECTION FAILED: %d",status);
    else
    WICED_BT_TRACE("CONNECTION CREATED: %d", status);
}


/******************************************************************************
 * Function Name: unicast_source_rpc_handle_disconnect
 *
 * Summary: handle disconnect command from rpc client
 *
 * Parameters:
 * uint8_t *p_data
 * uint32_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_rpc_handle_disconnect(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_device_address_t bd_addr;
    uint8_t addr_type;
    STREAM_TO_UINT8(addr_type, p_data);
    STREAM_TO_BDADDR(bd_addr, p_data);

    WICED_BT_TRACE("[%s] BD addr addr_type %d %B len %d\n", __FUNCTION__, bd_addr, addr_type, data_len);
    unicast_source_gatt_disconnect(bd_addr);
}

/******************************************************************************
 * Function Name: unicast_source_rpc_handle_play
 *
 * Summary: handle play command from rpc client
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint32_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_rpc_handle_play(uint8_t *p_data, uint32_t data_len)
{
    uint16_t conn_id;
    uint32_t codec_config;

    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT32(codec_config, p_data);

    WICED_BT_TRACE("[%s] conn_id %d codec_config %d len %d\n", __FUNCTION__, conn_id, codec_config, data_len);

    unicast_source_mcs_play(conn_id, codec_config);
}

/******************************************************************************
 * Function Name: unicast_source_rpc_handle_pause
 *
 * Summary: handle pause command from rpc client
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint32_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_rpc_handle_pause(uint8_t *p_data, uint32_t data_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    unicast_source_mcs_pause(conn_id);
}

/******************************************************************************
 * Function Name: unicast_source_rpc_handle_vol_up
 *
 * Summary: handle volume up from rpc client
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint32_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_rpc_handle_vol_up(uint8_t *p_data, uint32_t data_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    unicast_source_vcs_vol_up(conn_id);
}

/******************************************************************************
 * Function Name: unicast_source_rpc_handle_vol_down
 *
 * Summary: handle volume down from rpc client
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint32_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_rpc_handle_vol_down(uint8_t *p_data, uint32_t data_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    unicast_source_vcs_vol_down(conn_id);
}

/******************************************************************************
 * Function Name: unicast_source_rpc_handle_abs_vol
 *
 * Summary: handle set abs volume from rpc client
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint32_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_rpc_handle_abs_vol(uint8_t *p_data, uint32_t data_len)
{
    uint16_t conn_id;
    uint8_t vol;

    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(vol, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d vol %d\n", __FUNCTION__, conn_id, data_len, vol);

    unicast_source_vcs_volume_set(conn_id, vol);
}

/******************************************************************************
 * Function Name:
 *
 * Summary: handle mute from rpc client
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint32_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_rpc_handle_mute(uint8_t *p_data, uint32_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    unicast_source_vcs_mute(conn_id);
}

/******************************************************************************
 * Function Name: unicast_source_rpc_handle_unmute
 *
 * Summary: handle unmute from rpc client
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint32_t payload_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_rpc_handle_unmute(uint8_t *p_data, uint32_t payload_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, payload_len);

    unicast_source_vcs_unmute(conn_id);
}

/******************************************************************************
 * Function Name: unicast_source_rpc_handle_unmute_vol_down
 *
 * Summary: unmute and dolume down
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint32_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_rpc_handle_unmute_vol_down(uint8_t *p_data, uint32_t data_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    unicast_source_vcs_unmute_relative_volume_down(conn_id);
}

/******************************************************************************
 * Function Name: unicast_source_rpc_handle_unmute_vol_up
 *
 * Summary: unmute and volume up
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint32_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_rpc_handle_unmute_vol_up(uint8_t *p_data, uint32_t data_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    unicast_source_vcs_unmute_relative_volume_up(conn_id);
}

/******************************************************************************
 * Function Name: unicast_source_rpc_rx_cback
 *
 * Summary: rpc callback handle
 *
 * Parameters:
 *  uint16_t opcode
 *  uint8_t *p_data
 *  uint32_t data_len
 *
 * Return:
 *  wiced_bool_t
 *
******************************************************************************/
wiced_bool_t unicast_source_rpc_rx_cback(uint16_t opcode, uint8_t *p_data, uint32_t data_len)
{
    wiced_bool_t b_response_sent = TRUE;

    WICED_BT_TRACE("[%s] Received fn. code [%d]\n", __FUNCTION__, opcode);
    switch (opcode) {
        case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
            le_audio_rpc_send_dev_role(HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE);
            break;

        case HCI_CONTROL_LE_COMMAND_SCAN:
            unicast_source_rpc_handle_scan(p_data, data_len);
            break;

        case HCI_CONTROL_LE_COMMAND_CONNECT:
            unicast_source_rpc_handle_connect(p_data, data_len);
            break;

        case HCI_CONTROL_MISC_COMMAND_PING:
            le_audio_rpc_send_data(HCI_CONTROL_MISC_EVENT_PING_REPLY, p_data, data_len);
            break;

        case HCI_CONTROL_LE_COMMAND_CANCEL_CONNECT:
            unicast_source_rpc_handle_disconnect(p_data, data_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_PLAY:
            unicast_source_rpc_handle_play(p_data, data_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_PAUSE:
            unicast_source_rpc_handle_pause(p_data, data_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_VOL_UP:
            unicast_source_rpc_handle_vol_up(p_data, data_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_VOL_DOWN:
            unicast_source_rpc_handle_vol_down(p_data, data_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_MUTE:
            unicast_source_rpc_handle_mute(p_data, data_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE:
            unicast_source_rpc_handle_unmute(p_data, data_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_ABS_VOL:
            unicast_source_rpc_handle_abs_vol(p_data, data_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_UP:
            unicast_source_rpc_handle_unmute_vol_up(p_data, data_len);
            break;
        case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_DOWN:
            unicast_source_rpc_handle_unmute_vol_down(p_data, data_len);
            break;
        default:
            b_response_sent = FALSE;
            WICED_BT_TRACE("[%s] Unknown Function code [%d] \n", __FUNCTION__, opcode);
            break;
    }

    return b_response_sent;
}

/******************************************************************************
 * Function Name: unicast_source_rpc_init
 *
 * Summary: init rpc for client control
 *
 * Parameters:
 *  uint8_t app_instance
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_rpc_init(uint8_t app_instance)
{
    /* RPC to work with LE Audio Client Control */
    le_audio_rpc_init(app_instance, unicast_source_rpc_rx_cback, FALSE);
}
