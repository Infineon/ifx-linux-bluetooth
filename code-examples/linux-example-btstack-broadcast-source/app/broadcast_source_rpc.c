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

/******************************************************************************
 * File Name: broadcast_source_rpc.c
 *
 * Description: This is the source file for Broadcast_source rpc function.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

/* Application includes */
#include "broadcast_source_rpc.h"
#include "broadcast_source_bis.h"

/* App Library includes */
#include "le_audio_rpc.h"

/* BT Stack includes */
#include "wiced_bt_ga_bap_broadcast.h"
#include "wiced_bt_trace.h"

#define MAX_BROADCAST_CODE_LEN 16

/*
    uint8_t start;
    uint32_t codec_config;
    uint8_t enable_encryption;
    uint32_t channel_counts;
    uint32_t broadcast_id;
    uint8_t broadcast_code[MAX_BROADCAST_CODE_LEN];
    uint8_t bis_count;
*/

void broadcast_source_rpc_handle_start_streaming(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_ga_bap_stream_config_t stream_config;
    uint8_t start;
    uint32_t codec_config;
    uint8_t enable_encryption;
    uint32_t channel_counts;
    uint32_t broadcast_id;
    uint8_t broadcast_code[MAX_BROADCAST_CODE_LEN];
    uint8_t bis_count;

    wiced_result_t ret_sts = WICED_ERROR;
    STREAM_TO_UINT8(start, p_data);
    STREAM_TO_UINT32(codec_config, p_data);
    STREAM_TO_UINT8(enable_encryption, p_data);
    STREAM_TO_UINT32(channel_counts, p_data);
    STREAM_TO_UINT32(broadcast_id, p_data);
    STREAM_TO_ARRAY(broadcast_code, p_data, MAX_BROADCAST_CODE_LEN);
    STREAM_TO_UINT8(bis_count, p_data);

    WICED_BT_TRACE("[%s] Broadcast ID: %x", __FUNCTION__,broadcast_id);
    WICED_BT_TRACE("start:%d, codec_config:%d, enable_encryption:%d channel_counts:%d Broadcast ID: %x, bis_count:%d",start, codec_config, enable_encryption, channel_counts, broadcast_id, bis_count);
    if (start)
    {
        wiced_bt_ga_bap_broadcast_get_stream_config(codec_config, &stream_config);

        WICED_BT_TRACE("[%s] Configuring source stream [SF:%d] [FD:%d] [OPF:%d]\n",
                       __FUNCTION__,
                       stream_config.sampling_frequency,
                       stream_config.frame_duration,
                       stream_config.octets_per_codec_frame);

        ret_sts = broadcast_source_bis_configure_stream(broadcast_id,
                                                        broadcast_code,
                                                        bis_count,
                                                        channel_counts,
                                                        stream_config.sampling_frequency,
                                                        stream_config.frame_duration,
                                                        stream_config.octets_per_codec_frame,
                                                        enable_encryption);
        if (ret_sts)
            WICED_BT_TRACE_CRIT("[%s] broadcast_source_bis_configure_stream [ret_sts:0x%x]\n", __FUNCTION__, ret_sts);

        broadcast_source_bis_start_stream(&stream_config);
    }
    else
    {
        ret_sts = broadcast_source_bis_disable_stream();
        if (ret_sts)
            WICED_BT_TRACE_CRIT("[%s] broadcast_source_bis_disable_stream [ret_sts:0x%x]\n", __FUNCTION__, ret_sts);

        ret_sts = broadcast_source_bis_release_stream();
        if (ret_sts)
            WICED_BT_TRACE_CRIT("[%s] broadcast_source_bis_release_stream [ret_sts:0x%x]\n", __FUNCTION__, ret_sts);
    }
}

wiced_bool_t broadcast_source_rx_cback(uint16_t opcode, uint8_t *p_data, uint32_t data_len)
{
    wiced_bool_t b_response_sent = TRUE;

    WICED_BT_TRACE("[%s] Received fn. code [%d]\n", __FUNCTION__, opcode);

    switch (opcode)
    {
        case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
            le_audio_rpc_send_dev_role(HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SOURCE);
            break;

        case HCI_CONTROL_LE_AUDIO_COMMAND_BROADCAST_SOURCE_START_STREAMIMG:
            broadcast_source_rpc_handle_start_streaming(p_data, data_len);
            break;

        default:
            b_response_sent = FALSE;
            WICED_BT_TRACE("[%s] Unknown Function code [%d] \n", __FUNCTION__, opcode);
            break;
    }

    return b_response_sent;
}

void broadcast_source_rpc_init(uint8_t app_instance)
{
    /* RPC to work with LE Audio Client Control */
    le_audio_rpc_init(app_instance, broadcast_source_rx_cback, TRUE);
}
