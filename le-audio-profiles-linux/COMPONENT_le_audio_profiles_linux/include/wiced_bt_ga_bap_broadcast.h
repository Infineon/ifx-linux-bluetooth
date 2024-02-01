/*
 * $ Copyright Cypress Semiconductor $
 */

#pragma once

#include "wiced_bt_ga_bap.h"
#include "wiced_data_types.h"
#include "wiced_bt_ga_pbp.h"


#define BROADCAST_MAX_SUB_GROUP 2
#define BROADCAST_MAX_BIS_PER_SUB_GROUP 2

#define BROADCAST_AUDIO_ANNOUNCEMENT_SIZE 6
#define SOLICITATION_REQ_SIZE 3

#define BAP_BROADCAST_TRACE(...)
#define BAP_BROADCAST_TRACE_CRIT(...)

typedef enum
{
    BAP_BROADCAST_STATE_IDLE,
    BAP_BROADCAST_STATE_CONFIGURED,
    BAP_BROADCAST_STATE_STREAMING,
} wiced_bt_ga_bap_broadcast_state_t;

typedef struct
{
    uint8_t bis_idx;
    wiced_bool_t b_bis_data_path_setup;
    wiced_bt_ga_bap_csc_t bis_csc;
} wiced_bt_ga_bap_broadcast_bis_group_t;

typedef struct
{
    wiced_bt_ga_bap_codec_id_t codec_id;
    wiced_bt_ga_bap_csc_t csc;
    wiced_bt_ga_bap_metadata_t metadata;
    uint8_t bis_cnt;
    wiced_bt_ga_bap_broadcast_bis_group_t bis_config[BROADCAST_MAX_BIS_PER_SUB_GROUP];
} wiced_bt_ga_bap_broadcast_sub_group_t;

typedef struct
{
    uint32_t broadcast_id;
    uint32_t presentation_delay;
    uint8_t sub_group_cnt;
    wiced_bt_ga_bap_broadcast_sub_group_t sub_group[BROADCAST_MAX_SUB_GROUP];
    wiced_bt_ga_bap_broadcast_state_t state;
} wiced_bt_ga_bap_broadcast_base_t;

wiced_bool_t wiced_bt_ga_bap_broadcast_is_broadcast_announcement(uint8_t *p_adv_data, uint32_t *p_br_id);

wiced_result_t wiced_bt_ga_bap_broadcast_configure(uint8_t adv_handle, wiced_bt_ga_bap_broadcast_base_t *p_base, uint8_t *p_ext_adv, uint8_t adv_len);

wiced_bool_t wiced_bt_ga_bap_broadcast_is_basic_announcement(uint8_t *p_adv_data, uint8_t *p_base_len);

wiced_result_t wiced_bt_ga_bap_broadcast_parse_base_info(uint8_t *p_adv_data,
                                                         uint8_t adv_data_len,
                                                         wiced_bt_ga_bap_broadcast_base_t *p_base,
                                                         uint8_t max_group_supported,
                                                         uint8_t max_bis_per_group);

wiced_result_t wiced_bt_ga_bap_broadcast_start_stream_discovery(uint8_t duration);

wiced_result_t wiced_bt_ga_bap_broadcast_stop_stream_discovery(void);

wiced_result_t wiced_bt_ga_bap_broadcast_reconfigure(uint8_t adv_sid, wiced_bt_ga_bap_broadcast_base_t *p_base);

/**
 * @brief Only Metadata can be updated in the Streaming state (Upper layer should
 * ensure that only Metadata is modified in the BASE data)
 *
 * @param adv_sid Adv. set ID to indicate the corresponding Periodic Adv train
 * @param p_base BASE information
 * @return wiced_result_t WICED_SUCCESS if successful
 */
wiced_result_t wiced_bt_ga_bap_broadcast_update_metadata(uint8_t adv_sid, wiced_bt_ga_bap_broadcast_base_t *p_base);

wiced_result_t wiced_bt_ga_bap_broadcast_start_solicitation_requests(uint8_t adv_sid,
                                                                     char *dev_name,
                                                                     uint8_t name_length);

uint8_t wiced_bt_ga_bap_broadcast_get_sampling_freq_index(uint32_t samplaing_freq);
uint8_t wiced_bt_ga_bap_broadcast_get_fram_duration_index(uint32_t frame_duration);

wiced_result_t wiced_bt_ga_bap_build_broadcast_annoncement_adv_data(uint8_t *p_data,
                                                                    uint8_t *len,
                                                                    uint32_t broadcast_id);
