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

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "wiced_bt_isoc.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "iso_data_handler.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define ISO_DATA_HEADER_SIZE 4

#define ISO_LOAD_HEADER_SIZE_WITH_TS 8
#define ISO_LOAD_HEADER_SIZE_WITHOUT_TS 4

#define ISO_PKT_PB_FLAG_MASK 3
#define ISO_PKT_PB_FLAG_OFFSET 12

#define ISO_PKT_PB_FLAG_FIRST_FRAGMENT 0
#define ISO_PKT_PB_FLAG_CONTINUATION_FRAGMENT 1
#define ISO_PKT_PB_FLAG_COMPLETE 2
#define ISO_PKT_PB_FLAG_LAST_FRAGMENT 3

#define ISO_PKT_TS_FLAG_MASK 1
#define ISO_PKT_TS_FLAG_OFFSET 14

#define ISO_PKT_RESERVED_FLAG_MASK 1
#define ISO_PKT_RESERVED_FLAG_OFFSET 15

#define ISO_PKT_DATA_LOAD_LENGTH_MASK 0x3FFF
#define ISO_PKT_SDU_LENGTH_MASK 0x0FFF

/****************************************************************************
 *                              FUNCTION DECLARATION
 ***************************************************************************/
static iso_dhm_num_complete_evt_cb_t g_num_complete_cb;
static iso_dhm_rx_evt_cb_t g_rx_data_cb;


void iso_dhm_process_rx_data(uint8_t *p_data, uint32_t length)
{
    uint16_t handle_and_flags = 0;
    uint16_t data_load_length = 0;
    uint16_t ts_flag = 0;
    uint16_t pb_flag = 0;
    uint16_t psn = 0;
    uint16_t sdu_len = 0;
    uint32_t ts = 0;

    if (!length) { return; }

    STREAM_TO_UINT16(handle_and_flags, p_data);
    STREAM_TO_UINT16(data_load_length, p_data);

    pb_flag = (handle_and_flags & (ISO_PKT_PB_FLAG_MASK << ISO_PKT_PB_FLAG_OFFSET)) >> ISO_PKT_PB_FLAG_OFFSET;
    ts_flag = (handle_and_flags & (ISO_PKT_TS_FLAG_MASK << ISO_PKT_TS_FLAG_OFFSET)) >> ISO_PKT_TS_FLAG_OFFSET;

    handle_and_flags &= ~(ISO_PKT_PB_FLAG_MASK << ISO_PKT_PB_FLAG_OFFSET);
    handle_and_flags &= ~(ISO_PKT_TS_FLAG_MASK << ISO_PKT_TS_FLAG_OFFSET);
    handle_and_flags &= ~(ISO_PKT_RESERVED_FLAG_MASK << ISO_PKT_RESERVED_FLAG_OFFSET);

    if (ts_flag) { STREAM_TO_UINT32(ts, p_data); }

    STREAM_TO_UINT16(psn, p_data);
    STREAM_TO_UINT16(sdu_len, p_data);

    data_load_length &= ISO_PKT_DATA_LOAD_LENGTH_MASK;
    sdu_len &= ISO_PKT_SDU_LENGTH_MASK;

    // WICED_BT_TRACE("Recv isoc data size %d ", sdu_len);
    // WICED_BT_TRACE_ARRAY(p_data, sdu_len, "ISO Data");
    // WICED_BT_TRACE("TS %d PB flag %d psn %d ", ts, pb_flag, psn);
    (void)ts;
    (void)pb_flag;
    (void)psn;

    //if (!sdu_len) { return; }

    if (g_rx_data_cb) { g_rx_data_cb(handle_and_flags, p_data, sdu_len); }
}

wiced_bool_t iso_dhm_process_num_completed_pkts(uint8_t *p_buf)
{
    if (p_buf == NULL)
    {
        WICED_BT_TRACE("Error: p_buf is NULL");
        return WICED_FALSE;
    }
    uint8_t num_handles, xx;
    uint16_t handle;
    uint16_t num_sent;
    wiced_bool_t complete = WICED_TRUE;

    STREAM_TO_UINT8(num_handles, p_buf);

    for (xx = 0; xx < num_handles; xx++)
    {
        STREAM_TO_UINT16(handle, p_buf);
        STREAM_TO_UINT16(num_sent, p_buf);

        // WICED_BT_TRACE("[%s] handle 0x%x num_sent %d", __FUNCTION__, handle, num_sent);

        //validate handle
        if (wiced_bt_isoc_is_cis_connected_by_conn_id(handle) || wiced_bt_isoc_is_bis_created(handle))
        {
            //callback to app to send more packets
            if (g_num_complete_cb) { g_num_complete_cb(handle, num_sent); }
        }
        else
        {
            complete = WICED_FALSE;
        }
    }
    return complete;
}

void iso_dhm_init(
                  iso_dhm_num_complete_evt_cb_t num_complete_cb,
                  iso_dhm_rx_evt_cb_t rx_data_cb)
{
    wiced_bt_isoc_register_data_cb(iso_dhm_process_rx_data, iso_dhm_process_num_completed_pkts);

    g_num_complete_cb = num_complete_cb;
    g_rx_data_cb = rx_data_cb;
}

uint32_t iso_dhm_get_header_size()
{
    return ISO_LOAD_HEADER_SIZE_WITH_TS + ISO_DATA_HEADER_SIZE;
}

uint32_t iso_dhm_get_buffer_size(const wiced_bt_cfg_isoc_t *p_isoc_cfg)
{
    if (p_isoc_cfg == NULL)
    {
        WICED_BT_TRACE("Error: p_isoc_cfg is NULL");
        return 0;
    }
    int buff_size =
        (p_isoc_cfg->max_sdu_size * p_isoc_cfg->channel_count) + ISO_LOAD_HEADER_SIZE_WITH_TS + ISO_DATA_HEADER_SIZE;
    return buff_size;
}

void iso_dhm_send_packet(wiced_bool_t is_cis,
                         uint16_t conn_handle,
                         uint8_t ts_flag,
                         uint8_t *p_data_buf,
                         uint32_t data_buf_len)
{
    uint8_t *p = NULL;
    p_data_buf = p_data_buf + iso_dhm_get_header_size();
    uint16_t handle_and_flags = conn_handle;
    uint16_t data_load_length = 0;
    uint16_t psn = 0xFFFF;
    uint8_t *p_iso_sdu = NULL;

    uint16_t max_supported_data_len = wiced_bt_isoc_get_max_data_pkt_len();
    if (data_buf_len > max_supported_data_len)
    {
        //TODO: Fragmentation is to be supported
        WICED_BT_TRACE_CRIT("Received packet larger than the ISO SDU len supported");
        return;
    }

    handle_and_flags |= (ISO_PKT_PB_FLAG_COMPLETE << ISO_PKT_PB_FLAG_OFFSET);
    handle_and_flags |= (ts_flag << ISO_PKT_TS_FLAG_OFFSET);

    if (ts_flag)
    {
        //timestamp supported, header size is 4 + 8
        p_iso_sdu = p = p_data_buf - (ISO_LOAD_HEADER_SIZE_WITH_TS + ISO_DATA_HEADER_SIZE);
        data_load_length = data_buf_len + ISO_LOAD_HEADER_SIZE_WITH_TS;
    }
    else
    {
        //timestamp not supported, header size is 4 + 4
        p_iso_sdu = p = p_data_buf - (ISO_LOAD_HEADER_SIZE_WITHOUT_TS + ISO_DATA_HEADER_SIZE);
        data_load_length = data_buf_len + ISO_LOAD_HEADER_SIZE_WITHOUT_TS;
    }

    data_load_length &= ISO_PKT_DATA_LOAD_LENGTH_MASK;
    data_buf_len &= ISO_PKT_SDU_LENGTH_MASK;

    if (is_cis)
        psn = wiced_bt_isoc_central_get_psn_by_cis_handle(conn_handle);
    else
        psn = wiced_bt_isoc_get_psn_by_bis_handle(conn_handle);

    UINT16_TO_STREAM(p, handle_and_flags);
    UINT16_TO_STREAM(p, data_load_length);
    UINT16_TO_STREAM(p, psn);
    UINT16_TO_STREAM(p, data_buf_len);

    wiced_bt_write_iso_data_to_lower(p_iso_sdu, data_load_length + ISO_DATA_HEADER_SIZE);
}
