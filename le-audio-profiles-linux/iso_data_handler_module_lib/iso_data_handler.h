/*
 * $ Copyright Cypress Semiconductor $
 */

#include "wiced_bt_cfg.h"

typedef struct
{
    uint16_t isoc_conn_hdl;
    uint32_t num_sent;
}iso_dhm_packet_record_t;

typedef void (*iso_dhm_num_complete_evt_cb_t)(uint16_t cis_handle, uint16_t num_sent);
typedef void (*iso_dhm_rx_evt_cb_t)(uint16_t cis_handle, uint8_t *p_data, uint32_t length);

void iso_dhm_register_cb(iso_dhm_num_complete_evt_cb_t num_complete_cb, iso_dhm_rx_evt_cb_t rx_data_cb);
uint32_t iso_dhm_get_buffer_size(uint32_t max_sdu_size, uint32_t channel_count);

uint8_t *iso_dhm_get_data_buffer(void);
uint32_t iso_dhm_get_header_size();
void iso_dhm_free_data_buffer(uint8_t *p_buf);

void iso_dhm_send_packet(uint16_t psn, uint16_t conn_handle, uint8_t ts_flag, uint8_t *p_data_buf, uint32_t data_buf_len);
wiced_bool_t iso_dhm_process_num_completed_pkts(uint8_t *p_buf);
void iso_dhm_process_rx_data(uint8_t *p_data, uint32_t length);
