/*
 * $ Copyright Cypress Semiconductor $
 */

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "wiced_bt_isoc.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "iso_data_handler.h"

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

#define MAX_ISOC_CONNECTION 5

static iso_dhm_num_complete_evt_cb_t g_num_complete_cb;
static iso_dhm_rx_evt_cb_t g_rx_data_cb;

typedef struct
{
    uint8_t num_isoc_conn;
    iso_dhm_packet_record_t pack_rec[MAX_ISOC_CONNECTION];
} isoc_pack_records_t;

isoc_pack_records_t isoc_pack_rec;
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
    uint8_t num_handles, xx;
    uint16_t handle;
    uint16_t num_sent;
    wiced_bool_t complete = WICED_TRUE;
    uint8_t index = 0;

    STREAM_TO_UINT8(num_handles, p_buf);

    for (xx = 0; xx < num_handles; xx++)
    {
        STREAM_TO_UINT16(handle, p_buf);
        STREAM_TO_UINT16(num_sent, p_buf);

        // WICED_BT_TRACE("[%s] handle 0x%x num_sent %d", __FUNCTION__, handle, num_sent);

        //validate handle
        for (index = 0; index < isoc_pack_rec.num_isoc_conn; index++)
        {
            if (isoc_pack_rec.pack_rec[index].isoc_conn_hdl == handle)
            {
                isoc_pack_rec.pack_rec[index].num_sent -= num_sent;
                if (g_num_complete_cb)
                {
                    //callback to app to send more packets
                    g_num_complete_cb(handle, num_sent);
                }
                break;
            }
        }
        if (!isoc_pack_rec.pack_rec[index].isoc_conn_hdl) return WICED_FALSE;
    }
    return complete;
}

void iso_dhm_register_cb(iso_dhm_num_complete_evt_cb_t num_complete_cb,
                  iso_dhm_rx_evt_cb_t rx_data_cb)
{
    wiced_ble_isoc_register_data_cb(iso_dhm_process_rx_data, iso_dhm_process_num_completed_pkts);

    g_num_complete_cb = num_complete_cb;
    g_rx_data_cb = rx_data_cb;
}

uint32_t iso_dhm_get_header_size()
{
    return ISO_LOAD_HEADER_SIZE_WITH_TS + ISO_DATA_HEADER_SIZE;
}

#define MAX_ISOC_SDU_SIZE 240
#define MAX_ISOC_CHANNEL_COUNT 2

uint32_t iso_dhm_get_buffer_size(uint32_t max_sdu_size, uint32_t channel_count)
{
    int buff_size =
        (max_sdu_size * channel_count) + ISO_LOAD_HEADER_SIZE_WITH_TS + ISO_DATA_HEADER_SIZE;
    return buff_size;
}

void iso_dhm_send_packet(uint16_t psn,
                         uint16_t conn_handle,
                         uint8_t ts_flag,
                         uint8_t *p_data_buf,
                         uint32_t data_buf_len)
{
    uint8_t *p = NULL;
    p_data_buf = p_data_buf + iso_dhm_get_header_size();
    uint16_t handle_and_flags = conn_handle;
    uint16_t data_load_length = 0;
    uint8_t *p_iso_sdu = NULL;
    uint8_t index = 0;

    uint16_t max_supported_data_len = wiced_ble_isoc_get_max_data_pkt_len();
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

    UINT16_TO_STREAM(p, handle_and_flags);
    UINT16_TO_STREAM(p, data_load_length);
    UINT16_TO_STREAM(p, psn);
    UINT16_TO_STREAM(p, data_buf_len);

    wiced_ble_isoc_write_data_to_lower(p_iso_sdu, data_load_length + ISO_DATA_HEADER_SIZE);

    for (index = 0; index < isoc_pack_rec.num_isoc_conn; index++)
    {
        if (isoc_pack_rec.pack_rec[index].isoc_conn_hdl == conn_handle) break;
    }
    if (!isoc_pack_rec.pack_rec[index].isoc_conn_hdl)
    {
        isoc_pack_rec.pack_rec[index].isoc_conn_hdl = conn_handle;
        isoc_pack_rec.num_isoc_conn++;
    }
    isoc_pack_rec.pack_rec[index].num_sent++;
    for (int i = 0; i < isoc_pack_rec.num_isoc_conn; i++)
        WICED_BT_TRACE("[%s] conn hdl %d num_sent %d",
                       __FUNCTION__,
                       isoc_pack_rec.pack_rec[i].isoc_conn_hdl,
                       isoc_pack_rec.pack_rec[i].num_sent);
}
