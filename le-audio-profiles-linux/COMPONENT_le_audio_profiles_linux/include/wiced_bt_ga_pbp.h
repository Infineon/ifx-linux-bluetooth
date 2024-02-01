/*
 * $ Copyright Cypress Semiconductor $
 */

#pragma once

#include "wiced_result.h"

#define PBP_BROADCAST_TRACE(...)
#define PBP_BROADCAST_TRACE_CRIT(...)

#define WICED_BT_GA_PBP_TYPE_BROADCAST_NAME 0x30    // adv type: BTM_BLE_ADVERT_TYPE_BROADCAST_NAME
#define MAX_BROADCAST_NAME_ADV_SIZE 32
#define PBP_METADATA_PROGRAM_INFO_TYPE 0x03
#define APPEARANCE_VALUE_AD_SIZE 3
#define MAX_PROGRAM_INFO_SIZE 50
#define MAX_METADATA_LEN 100

typedef struct {
    wiced_bool_t encryption;
    uint32_t sampling_frequency;
    uint16_t frame_duration;
    uint8_t metadata_length;
    uint8_t *metadata;
    uint8_t *broadcast_name;
    size_t name_length;
    uint16_t appearance_value;
    uint8_t *program_info;
    uint8_t program_info_size;
} wiced_bt_ga_public_broadcast;

typedef enum
{
    WICED_BT_GA_PBP_GENERIC_CODEC_PUBLIC_BROADCAST_AUDIO = 0,
    WICED_BT_GA_PBP_STANDARD_QUALITY_PUBLIC_BROADCAST_AUDIO = 2,
    WICED_BT_GA_PBP_HIGH_QUALITY_PUBLIC_BROADCAST_AUDIO = 4,
} wiced_bt_ga_pbp_audio_config;


typedef struct
{
    wiced_bool_t encryption;
    wiced_bt_ga_pbp_audio_config audio_config;
    uint8_t public_broadcast_feature;
    uint8_t metadata_length;
    uint8_t metadata[MAX_PROGRAM_INFO_SIZE + MAX_METADATA_LEN + 2];
    uint8_t broadcast_name[MAX_BROADCAST_NAME_ADV_SIZE];
    uint16_t source_appearance_value;
} wiced_bt_ga_rcv_public_broadcast;

 wiced_bool_t wiced_bt_ga_pbp_is_public_broadcast(uint8_t *p_adv_data,
                                                 wiced_bt_ga_rcv_public_broadcast *p_public_rcv_br);

 wiced_result_t wiced_bt_ga_pbp_build_adv_data(uint8_t *p_data,
                                               uint32_t offset,
                                               wiced_bt_ga_public_broadcast *public_broadcast,
                                               uint32_t *len);
