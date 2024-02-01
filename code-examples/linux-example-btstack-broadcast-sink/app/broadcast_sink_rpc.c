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
#include "broadcast_sink_rpc.h"
#include "audio_driver.h"
#include "broadcast_sink_bass.h"
#include "broadcast_sink_bis.h"
#include "broadcast_sink_iso_audio.h"

/* App Library includes */
#include "le_audio_rpc.h"
#include "wiced_bt_ga_bap_broadcast.h"

/* BT Stack includes */
#include "wiced_bt_ble.h"

#include "wiced_bt_trace.h"

#define MAX_BROADCAST_NAME_SIZE 32

uint8_t step_size = 20;
static uint8_t vol = DEFAULT_VOL;

/* Indicate if the device is sink capable of scanning itself or require Broadcast Assistant to scan on its behalf */
static wiced_bool_t is_dev_role_sink = TRUE;
#define MAX_BROADCAST_CODE_LEN 16
char dev_name[] = "ifx_deligator";

extern void audio_driver_set_volume(uint8_t volume);

void broadcast_sink_rpc_handle_start_bass_adv(uint8_t *p_data, uint32_t data_len)
{
#define BASS_ADV_HANDLE 1
    wiced_bt_ga_bap_broadcast_start_solicitation_requests(BASS_ADV_HANDLE, dev_name, (uint8_t)strlen(dev_name));
}

void broadcast_sink_rpc_handle_find_sources(uint8_t *p_data, uint32_t data_len)
{
    uint8_t start;

    STREAM_TO_UINT8(start, p_data);
    WICED_BT_TRACE("[%s] [start:%d]\n", __FUNCTION__, start);

    is_dev_role_sink = TRUE;

    if (start)
        broadcast_sink_clear_data();

    broadcast_sink_bis_discover_sources(start ? BTM_BLE_SCAN_TYPE_HIGH_DUTY : BTM_BLE_SCAN_TYPE_NONE);
}

void broadcast_sink_rpc_handle_sync_to_source(uint8_t *p_data, uint32_t data_len)
{
    broadcast_sink_cb_t *p_big = NULL;
    broadcast_source_t source = {0};
    uint8_t listen;
    uint8_t lc3_index[2] = {0};

    STREAM_TO_UINT8(listen, p_data);
    STREAM_TO_UINT32(source.broadcast_id, p_data);
    WICED_BT_TRACE("Broadcast ID: %x", source.broadcast_id);
    uint8_t br_code_flag;
    STREAM_TO_UINT8(br_code_flag, p_data);
    if (br_code_flag)
    {
        STREAM_TO_ARRAY(&source.broadcast_code, p_data, MAX_BROADCAST_CODE_LEN);
        WICED_BT_TRACE_ARRAY(&source.broadcast_code, MAX_BROADCAST_CODE_LEN, "[%s] broadcast code\n");
    }

    if (!listen)
    {
        p_big = broadcast_sink_bis_get_big_by_broadcast_id(source.broadcast_id);
        if (p_big == NULL)
        {
            WICED_BT_TRACE("[%s] p_big is null", __FUNCTION__);
            return;
        }

        wiced_bool_t res = wiced_bt_isoc_peripheral_big_terminate_sync(p_big->big_handle);
        wiced_bt_ble_terminate_sync_to_periodic_adv(p_big->sync_handle);
        iso_audio_remove_data_path(p_big->bis_conn_id_list[0], WICED_BLE_ISOC_DPD_OUTPUT_BIT, lc3_index);
        broadcast_sink_bis_free_big(p_big);
    }

    broadcast_sink_bis_sync_to_source(listen ? BTM_BLE_SCAN_TYPE_HIGH_DUTY : BTM_BLE_SCAN_TYPE_NONE, source);
}

void broadcast_sink_rpc_listen_to_broadcast(uint8_t *p_data, uint32_t data_len)
{
    uint16_t listen;
    uint8_t broadcast_code[MAX_BROADCAST_CODE_LEN];
    STREAM_TO_UINT16(listen, p_data);
    STREAM_TO_ARRAY(broadcast_code, p_data, MAX_BROADCAST_CODE_LEN);
    int index;

    if (!listen)
    {
        WICED_BT_TRACE("[%s] TERMINATE SYNC\n", __FUNCTION__);
        broadcast_sink_bass_t *p_bass_data = broadcast_sink_bass_get_bass_data();
        for (index = 0; index < MAX_BASS; index++)
        {
            if ((p_bass_data->bass_data[index].is_used) &&
                (p_bass_data->bass_data[index].sub_group_data[0].bis_sync_state & WICED_BT_GA_BASS_SYNC_EST_MASK))
            {
                broadcast_sink_cb_t *big =
                    broadcast_sink_bis_get_big_by_sync_handle(p_bass_data->bass_data[index].sync_handle);
                big->b_biginfo_updated = FALSE;
                big->b_base_updated = FALSE;
                big->in_use = FALSE;
                wiced_bt_isoc_peripheral_big_terminate_sync(big->big_handle);
                //le_audio_rpc_send_data()
                le_audio_rpc_send_status_update(BIG_SYNC_LOST);
            }
        }
    }
    else
    {
        WICED_BT_TRACE("[%s] REQUEST PA SYNC INFO\n", __FUNCTION__);
        broadcast_sink_bass_t *p_bass_data = broadcast_sink_bass_get_bass_data();
        broadcast_sink_bis_alloc_big(p_bass_data->operation_data.data.add_source_param.broadcast_id,
                                     p_bass_data->operation_data.data.add_source_param.source_addr.bda,
                                     p_bass_data->operation_data.data.add_source_param.adv_sid);
        broadcast_sink_bass_request_pa_sync_info();
    }
}

wiced_bool_t broadcast_sink_rpc_is_dev_role_sink()
{
    return is_dev_role_sink;
}

void broadcast_sink_rpc_set_vol(uint8_t *p_data, uint32_t data_len)
{
    /*LE_AUDIO_param_vol *param = (LE_AUDIO_param_vol *)p_data;*/
    uint32_t opcode;
    uint32_t abs_vol;
    STREAM_TO_UINT32(opcode, p_data);
    switch (opcode)
    {
    case HCI_CONTROL_LE_AUDIO_COMMAND_VOL_DOWN:
        if (vol - step_size >= 0)
        {
            vol = vol - step_size;
            audio_driver_set_volume(((vol)*100) / 255);
        }
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_VOL_UP:
        if (vol + step_size <= 255)
        {
            vol = vol + step_size;
            audio_driver_set_volume(((vol)*100) / 255);
        }
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_ABS_VOL:
        STREAM_TO_UINT32(abs_vol, p_data);
        vol = abs_vol;
        audio_driver_set_volume((vol * 100) / 255);
        break;
    }
}

wiced_bool_t broadcast_sink_rpc_rx_cback(uint16_t opcode, uint8_t *p_data, uint32_t data_len)
{
    wiced_bool_t b_response_sent = FALSE;

    WICED_BT_TRACE("[%s] Received fn. code [%d]\n", __FUNCTION__, opcode);

    switch (opcode)
    {
    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        b_response_sent = TRUE;
        le_audio_rpc_send_dev_role(HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SINK);
        break;

    case HCI_CONTROL_LE_COMMAND_ADVERTISE:
        broadcast_sink_rpc_handle_start_bass_adv(p_data, data_len);
        break;

    case HCI_CONTROL_LE_AUDIO_COMMAND_BROADCAST_SINK_FIND_SOURCES:
        broadcast_sink_rpc_handle_find_sources(p_data, data_len);
        break;

    case HCI_CONTROL_LE_AUDIO_COMMAND_BROADCAST_SINK_SYNC_TO_SOURCES:
        broadcast_sink_rpc_handle_sync_to_source(p_data, data_len);
        break;

    case HCI_CONTROL_LE_AUDIO_COMMAND_BROADCAST_SINK_LISTEN_TO_BROADCAST:
        broadcast_sink_rpc_listen_to_broadcast(p_data, data_len);
        break;

    case HCI_CONTROL_LE_AUDIO_COMMAND_VOLUME_SET:
        broadcast_sink_rpc_set_vol(p_data, data_len);
        break;

    default:
        WICED_BT_TRACE("[%s] Unknown Function code [%d] \n", __FUNCTION__, opcode);
        break;
    }

    return b_response_sent;
}

void broadcast_sink_rpc_init(uint8_t app_instance)
{
    /* RPC to work with LE Audio Client Control */
    le_audio_rpc_init(app_instance, broadcast_sink_rpc_rx_cback, FALSE);
}

void broadcast_sink_rpc_send_new_stream_info(uint32_t broadcast_id, uint8_t *br_name)
{
    uint8_t broadcast_name[] = " GEN SOURCE ";
    WICED_BT_TRACE("[%s] broadcastid [%x] br_name %s\n",
                   __FUNCTION__,
                   broadcast_id,
                   br_name ? br_name : broadcast_name);

    uint8_t tx_buff[MAX_BROADCAST_NAME_SIZE + 5];
    uint8_t *p = tx_buff;
    UINT32_TO_STREAM(p, broadcast_id);

    if (br_name != NULL)
    {
        UINT8_TO_STREAM(p, strlen(br_name));
        ARRAY_TO_STREAM(p, br_name, strlen(br_name));
    }
    else
    {
        UINT8_TO_STREAM(p, strlen(broadcast_name));
        ARRAY_TO_STREAM(p, broadcast_name, strlen(broadcast_name));
    }

    le_audio_rpc_send_data(HCI_CONTROL_LE_AUDIO_BROADCAST_STREAM_RSP_EVT, tx_buff, (int)(p - tx_buff));
}
