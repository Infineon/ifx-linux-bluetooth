/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __LEPL_BIS_H__
#define __LEPL_BIS_H__

#include "wiced_bt_types.h"

#include "le_audio_bap_broadcast.h"

typedef struct
{
    // app info
    wiced_bool_t in_use;
    wiced_bt_device_address_t bd_addr;
    uint8_t big_handle;
    uint8_t adv_handle;
    wiced_bool_t b_base_updated;
    wiced_bool_t b_biginfo_updated;

    // controller info
    wiced_ble_padv_sync_handle_t sync_handle;
    uint8_t number_of_subevents; // for sink only (received in BIGInfo)
    uint8_t bis_conn_id_count;
    uint16_t bis_conn_id_list[BROADCAST_MAX_BIS_PER_SUB_GROUP * BROADCAST_MAX_SUB_GROUP];
    uint8_t bis_index_list[BROADCAST_MAX_BIS_PER_SUB_GROUP * BROADCAST_MAX_SUB_GROUP];
    wiced_bool_t b_encryption;
    wiced_bt_bap_broadcast_code_t broadcast_code;

    // profile info
    le_audio_bap_broadcast_base_t base;
} lepl_broadcast_source_cb_t;

lepl_broadcast_source_cb_t *lehs_get_broadcast_source_cb(void);
void lepl_bis_start_stream(wiced_bt_ga_bap_stream_config_t *p_stream_config);
wiced_result_t lepl_bis_disable_stream(void);
wiced_result_t lepl_bis_release_stream(void);
wiced_result_t lepl_bis_configure_stream(uint32_t broadcast_id,
                                                     uint8_t *broadcast_code,
                                                     uint8_t bis_count,
                                                     uint32_t channel_counts,
                                                     uint32_t sampling_freq,
                                                     uint32_t frame_duration,
                                                     uint16_t octets_per_codec_frame,
                                                     wiced_bool_t enable_encryption);
#endif //__LEPL_BIS_H__
