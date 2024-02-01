/*
 * $ Copyright Cypress Semiconductor $
 */

#pragma once

#include "wiced_bt_ga.h"

#define BAP_TRACE(...)
#define BAP_TRACE_CRIT(...)

// Macros for NULL checking
#define CHECK_FOR_NULL_AND_RETURN_VALUE(x, error_return_value)      \
    if (!x)                                                         \
    {                                                               \
        WICED_BT_TRACE_CRIT("[%s] %s is NULL\n", __FUNCTION__, #x); \
        return error_return_value;                                  \
    }

#define CHECK_FOR_NULL_AND_RETURN(x)                                \
    if (!x)                                                         \
    {                                                               \
        WICED_BT_TRACE_CRIT("[%s] %s is NULL\n", __FUNCTION__, #x); \
        return;                                                     \
    }

#define BAP_BROADCAST_CODE_SIZE                  16
#define BAP_BROADCAST_ID_SIZE                    3

// Context Types
/**! Prohibited */
#define BAP_CONTEXT_TYPE_PROHIBITED 0x0000
/**! Unspecified. Matches any audio context */
#define BAP_CONTEXT_TYPE_UNSPECIFIED 0x0001
/**! Conversation between humans, for example, in telephony or video calls, including traditional cellular as well as VoIP and Push-to-Talk */
#define BAP_CONTEXT_TYPE_CONVERSATIONAL 0x0002
/**! Media, for example, music playback, radio, podcast or movie soundtrack, or tv audio */
#define BAP_CONTEXT_TYPE_MEDIA 0x0004
/**! Audio associated with video gaming, for example gaming media; gaming effects; music and in-game voice chat between participants; or a mix of all the above */
#define BAP_CONTEXT_TYPE_GAME 0x0008
/**! Instructional audio, for example, in navigation, announcements, or user guidance */
#define BAP_CONTEXT_TYPE_INSTRUCTIONAL 0x0010
/**! Man-machine communication, for example, with voice recognition or virtual assistants */
#define BAP_CONTEXT_TYPE_VOICE_ASSISTANTS 0x0020
/**! Live audio, for example, from a microphone where audio is perceived both through a direct acoustic path and through an LE Audio Stream */
#define BAP_CONTEXT_TYPE_LIVE 0x0040
/**! Sound effects including keyboard and touch feedback; menu and user interface sounds; and other system sounds */
#define BAP_CONTEXT_TYPE_SOUND_EFFECT 0x0080
/**! Notification and reminder sounds; attention-seeking audio, for example, in beeps signaling the arrival of a message */
#define BAP_CONTEXT_TYPE_NOTIFICATIONS 0x100
/**! Alerts the user to an incoming call, for example, an incoming telephony or video call, including traditional cellular as well as VoIP and Push-to-Talk */
#define BAP_CONTEXT_TYPE_RINGTONE 0x0200
/**! Alarms and timers; immediate alerts, for example, in a critical battery alarm, timer expiry or alarm clock, toaster, cooker, kettle, microwave, etc */
#define BAP_CONTEXT_TYPE_ALERTS 0x0400
/**! Emergency sounds, for example, fire alarms or other urgent alerts */
#define BAP_CONTEXT_TYPE_EMERGENCY_ALARM 0x0800
/**! Metadata conforming to the Bluetooth Broadcast TV profile */
#define BAP_CONTEXT_TYPE_TV 0x0200

#define VALID_CONTENT_TYPE_MASK 0x0FFF
#define INVALID_CONTENT_TYPE_MASK 0xF000

// audio locations
#define BAP_AUDIO_LOCATION_NOT_ALLOWED 0X00000000
#define BAP_AUDIO_LOCATION_FRONT_LEFT 0X00000001
#define BAP_AUDIO_LOCATION_FRONT_RIGHT 0X00000002
#define BAP_AUDIO_LOCATION_FRONT_CENTER 0X00000004
#define BAP_AUDIO_LOCATION_LOW_FREQUENCY_EFFECTS_1 0X00000008
#define BAP_AUDIO_LOCATION_BACK_LEFT 0X00000010
#define BAP_AUDIO_LOCATION_BACK_RIGHT 0X00000020
#define BAP_AUDIO_LOCATION_FRONT_LEFT_OF CENTER 0X00000040
#define BAP_AUDIO_LOCATION_FRONT_RIGHT_OF CENTER 0X00000080
#define BAP_AUDIO_LOCATION_BACK_CENTER 0X00000100
#define BAP_AUDIO_LOCATION_LOW_FREQUENCY_EFFECTS_2 0X00000200
#define BAP_AUDIO_LOCATION_SIDE_LEFT 0X00000400
#define BAP_AUDIO_LOCATION_SIDE_RIGHT 0X00000800
#define BAP_AUDIO_LOCATION_TOP_FRONT_LEFT 0X00001000
#define BAP_AUDIO_LOCATION_TOP_FRONT_RIGHT 0X00002000
#define BAP_AUDIO_LOCATION_TOP_FRONT_CENTER 0X00004000
#define BAP_AUDIO_LOCATION_TOP_CENTER 0X00008000
#define BAP_AUDIO_LOCATION_TOP_BACK_LEFT 0X00010000
#define BAP_AUDIO_LOCATION_TOP_BACK_RIGHT 0X00020000
#define BAP_AUDIO_LOCATION_TOP_SIDE_LEFT 0X00040000
#define BAP_AUDIO_LOCATION_TOP_SIDE_RIGHT 0X00080000
#define BAP_AUDIO_LOCATION_TOP_BACK_CENTER 0X00100000
#define BAP_AUDIO_LOCATION_BOTTOM_FRONT_CENTER 0X00200000
#define BAP_AUDIO_LOCATION_BOTTOM_FRONT_LEFT 0X00400000
#define BAP_AUDIO_LOCATION_BOTTOM_FRONT_RIGHT 0X00800000
#define BAP_AUDIO_LOCATION_FRONT_LEFT_WIDE 0X01000000
#define BAP_AUDIO_LOCATION_FRONT_RIGHT_WIDE 0X02000000
#define BAP_AUDIO_LOCATION_LEFT_SURROUND 0X04000000
#define BAP_AUDIO_LOCATION_RIGHT_SURROUND 0X08000000
#define BAP_AUDIO_LOCATION_INVALID_MASK 0xF0000000

// sampling freq
#define BAP_SUPPORTED_SAMPLING_FREQ_8_KHZ (1 << 0)        /*< Bit 0: 8,000 Hz */
#define BAP_SUPPORTED_SAMPLING_FREQ_11_025_KHZ (1 << 1)   /*< Bit 1: 11,025 Hz */
#define BAP_SUPPORTED_SAMPLING_FREQ_16_KHZ (1 << 2)       /*< Bit 2: 16,000 Hz */
#define BAP_SUPPORTED_SAMPLING_FREQ_22_050_KHZ (1 << 3)   /*< Bit 3: 22,050 Hz */
#define BAP_SUPPORTED_SAMPLING_FREQ_24_KHZ (1 << 4)       /*< Bit 4: 24,000 Hz */
#define BAP_SUPPORTED_SAMPLING_FREQ_32_KHZ (1 << 5)       /*< Bit 5: 32,000 Hz */
#define BAP_SUPPORTED_SAMPLING_FREQ_44_1_KHZ (1 << 6)     /*< Bit 6: 44,100 Hz */
#define BAP_SUPPORTED_SAMPLING_FREQ_48_KHZ (1 << 7)       /*< Bit 7: 48,000 Hz */
#define BAP_SUPPORTED_SAMPLING_FREQ_88_200_KHZ (1 << 8)   /*< Bit 8: 88,200 Hz */
#define BAP_SUPPORTED_SAMPLING_FREQ_96_KHZ (1 << 9)       /*< Bit 9: 96,000 Hz */
#define BAP_SUPPORTED_SAMPLING_FREQ_176_400_KHZ (1 << 10) /*< Bit 10: 176,400 Hz */
#define BAP_SUPPORTED_SAMPLING_FREQ_192_KHZ (1 << 11)     /*< Bit 11: 192,000 Hz */
#define BAP_SUPPORTED_SAMPLING_FREQ_384_KHZ (1 << 12)     /*< Bit 12: 384,000 Hz */

// frame durations
#define BAP_SUPPORTED_FRAME_DURATION_7_5MS (1 << 0) // Bit 0: 7.5 ms frame duration supported.
#define BAP_SUPPORTED_FRAME_DURATION_10MS (1 << 1)  // Bit 1: 10 ms frame duration supported.

// Valid only when 7.5 ms is supported and 10 ms is supported. Shall not be set to 0b1 any of (bit 5 or bit 6 or bit 7) is set to 0b1.
#define BAP_SUPPORTED_FRAME_DURATION_7_5MS_PREFERRED 0x10 // Bit 4: 7.5 ms preferred.
// Valid only when 7.5 ms is supported and 10 ms is supported. Shall not be set to 0b1 if any of (bit 4 or bit 5 or bit 7) is set to 0b1.
#define BAP_SUPPORTED_FRAME_DURATION_10MS_PREFERRED 0x20 // Bit 6: 10 ms preferred.

#define BAP_AUDIO_CHANNEL_COUNTS_NOT_SUPPORTED 0
#define BAP_AUDIO_CHANNEL_COUNTS_1 1
#define BAP_AUDIO_CHANNEL_COUNTS_2 2

// sampling freq
#define BAP_SAMPLING_FREQ_8_KHz 0x01
#define BAP_SAMPLING_FREQ_11_025_KHz 0x02
#define BAP_SAMPLING_FREQ_16_KHz 0x03
#define BAP_SAMPLING_FREQ_22_050_KHz 0x04
#define BAP_SAMPLING_FREQ_24_KHz 0x05
#define BAP_SAMPLING_FREQ_32_KHz 0x06
#define BAP_SAMPLING_FREQ_44_1_KHz 0x07
#define BAP_SAMPLING_FREQ_48_KHz 0x08
#define BAP_SAMPLING_FREQ_88_2_KHz 0x09
#define BAP_SAMPLING_FREQ_96_KHz 0x0A
#define BAP_SAMPLING_FREQ_176_4_KHz 0x0B
#define BAP_SAMPLING_FREQ_192_KHz 0x0C
#define BAP_SAMPLING_FREQ_384_KHz 0x0D

// frame durations
#define BAP_FRAME_DURATION_7_5 0x00
#define BAP_FRAME_DURATION_10 0x01
#define BAP_FRAME_DURATION_8163 0x02
#define BAP_FRAME_DURATION_10884 0x03

    // Codec Specific Capabilities
    typedef enum
{
    BAP_CODEC_CAPABILITIES_INVALID_TYPE,
    BAP_CODEC_CAPABILITIES_SUPPORTED_SAMPLING_FREQUENCIES_TYPE,
    BAP_CODEC_CAPABILITIES_SUPPORTED_FRAME_DURATIONS_TYPE,
    BAP_CODEC_CAPABILITIES_SUPPORTED_AUDIO_CHANNEL_COUNTS_TYPE,
    BAP_CODEC_CAPABILITIES_SUPPORTED_OCTETS_PER_CODEC_FRAME_TYPE,
    BAP_CODEC_CAPABILITIES_SUPPORTED_MAX_CODEC_FRAMES_PER_SDU_TYPE,
    BAP_CODEC_CAPABILITIES_MAX_TYPE = BAP_CODEC_CAPABILITIES_SUPPORTED_MAX_CODEC_FRAMES_PER_SDU_TYPE
} wiced_bt_ga_bap_codec_capabilities_type_t;

// Codec Specific Configuration
typedef enum
{
    BAP_CODEC_CONFIG_INVALID_TYPE,
    BAP_CODEC_CONFIG_SAMPLING_FREQUENCY_TYPE,
    BAP_CODEC_CONFIG_FRAME_DURATION_TYPE,
    BAP_CODEC_CONFIG_AUDIO_CHANNEL_ALLOCATION_TYPE,
    BAP_CODEC_CONFIG_OCTETS_PER_CODEC_FRAME_TYPE,
    BAP_CODEC_CONFIG_LC3_BLOCKS_PER_SDU_TYPE,
    BAP_CODEC_CONFIG_MAX_TYPE = BAP_CODEC_CONFIG_LC3_BLOCKS_PER_SDU_TYPE
} wiced_bt_ga_bap_codec_configuration_type_t;

//metadata
typedef enum
{
    BAP_METADATA_INVALID_TYPE,
    BAP_METADATA_PREFERRED_AUDIO_CONTEXTS_TYPE,
    BAP_METADATA_STREAMING_AUDIO_CONTEXTS_TYPE,
    BAP_METADATA_PROGRAM_INFO,
    BAP_METADATA_LANGUAGE,
    BAP_METADATA_CCID_LIST,
    BAP_METADATA_PARENTAL_RATING,
    BAP_METADATA_PROGRAM_INFO_URI,
    BAP_METADATA_MAX_TYPE = BAP_METADATA_PROGRAM_INFO_URI,
    BAP_METADATA_EXTENDED_METADATA_TYPE = 0xFE,
    BAP_METADATA_VENDOR_SPECIFIC_TYPE = 0xFF,
} wiced_bt_ga_bap_metadata_type_t;

typedef struct
{
    uint8_t coding_format;
    uint16_t company_id;
    uint16_t vendor_specific_codec_id;
} wiced_bt_ga_bap_codec_id_t;

typedef struct
{
    uint32_t sampling_frequency;
    uint32_t frame_duration;
    uint32_t audio_channel_allocation;
    uint16_t octets_per_codec_frame;
    uint8_t lc3_blocks_per_sdu;
} wiced_bt_ga_bap_csc_t;

typedef struct
{
    wiced_bool_t metadata_present;
    uint16_t preferred_audio_ctx;
    uint16_t streaming_audio_ctx;
    uint8_t *p_vendor_specific_data;
    uint8_t vendor_specific_data_length;
    uint8_t *p_upper_layer_data;
    uint8_t upper_layer_data_length;
    uint8_t *program_info;
    uint8_t program_info_len;
} wiced_bt_ga_bap_metadata_t;

typedef struct
{
    uint8_t audio_channels;
    uint32_t sampling_frequency;
    uint32_t frame_duration;
    uint32_t sdu_interval;
    uint16_t octets_per_codec_frame;
    wiced_bt_isoc_framing_t framing;
    uint8_t retransmission_number;
    uint8_t max_transport_latency;
} wiced_bt_ga_bap_stream_config_t;

typedef enum
{
    BAP_CODEC_CONFIG_8_1_1,         //Config setting for low latency audio data
    BAP_CODEC_CONFIG_8_1_2,         //Config setting for high quality audio data
    BAP_CODEC_CONFIG_8_2_1,         //Config setting for low latency audio data
    BAP_CODEC_CONFIG_8_2_2,         //Config setting for high quality audio data
    BAP_CODEC_CONFIG_16_1_1,        //Config setting for low latency audio data
    BAP_CODEC_CONFIG_16_1_2,        //Config setting for high quality audio data
    BAP_CODEC_CONFIG_16_2_1,        //Config setting for low latency audio data
    BAP_CODEC_CONFIG_16_2_2,        //Config setting for high quality audio data
    BAP_CODEC_CONFIG_24_1_1,        //Config setting for low latency audio data
    BAP_CODEC_CONFIG_24_1_2,        //Config setting for high quality audio data
    BAP_CODEC_CONFIG_24_2_1,        //Config setting for low latency audio data
    BAP_CODEC_CONFIG_24_2_2,        //Config setting for high quality audio data
    BAP_CODEC_CONFIG_32_1_1,        //Config setting for low latency audio data
    BAP_CODEC_CONFIG_32_1_2,        //Config setting for high quality audio data
    BAP_CODEC_CONFIG_32_2_1,        //Config setting for low latency audio data
    BAP_CODEC_CONFIG_32_2_2,        //Config setting for high quality audio data
    BAP_CODEC_CONFIG_441_1_1,       //Config setting for low latency audio data
    BAP_CODEC_CONFIG_441_1_2,       //Config setting for high quality audio data
    BAP_CODEC_CONFIG_441_2_1,       //Config setting for low latency audio data
    BAP_CODEC_CONFIG_441_2_2,       //Config setting for high quality audio data
    BAP_CODEC_CONFIG_48_1_1,        //Config setting for low latency audio data
    BAP_CODEC_CONFIG_48_1_2,        //Config setting for high quality audio data
    BAP_CODEC_CONFIG_48_2_1,        //Config setting for low latency audio data
    BAP_CODEC_CONFIG_48_2_2,        //Config setting for high quality audio data
    BAP_CODEC_CONFIG_48_3_1,        //Config setting for low latency audio data
    BAP_CODEC_CONFIG_48_3_2,        //Config setting for high quality audio data
    BAP_CODEC_CONFIG_48_4_1,        //Config setting for low latency audio data
    BAP_CODEC_CONFIG_48_4_2,        //Config setting for high quality audio data
    BAP_CODEC_CONFIG_48_5_1,        //Config setting for low latency audio data
    BAP_CODEC_CONFIG_48_5_2,        //Config setting for high quality audio data
    BAP_CODEC_CONFIG_48_6_1,        //Config setting for low latency audio data
    BAP_CODEC_CONFIG_48_6_2,        //Config setting for high quality audio data
}wiced_bt_ga_bap_codec_config_t;

typedef uint8_t wiced_bt_bap_broadcast_code_t[BAP_BROADCAST_CODE_SIZE]; /**< BLE database hash */

wiced_bool_t wiced_bt_ga_bap_is_state_transition_valid(ascs_characteristics_t char_type,
                                                       uint8_t ase_state,
                                                       uint8_t opcode,
                                                       uint8_t *next_state);

/**
 * @brief Read ASE characteristics discovered on the ASCS server and enable notifications
 *
 * @param p_service
 * @param conn_id
 */

wiced_bool_t wiced_bt_ga_bap_ase_id_discovery_in_progress(gatt_intf_service_object_t *p_service,
                                                          uint16_t conn_id);

void wiced_bt_ga_bap_start_audio_capability_discovery(gatt_intf_service_object_t *p_service, uint16_t conn_id);

uint8_t wiced_bt_ga_bap_update_ltv(uint8_t *dest, uint8_t length, uint8_t type, const uint8_t *value);

uint8_t *wiced_bt_ga_bap_fill_csc(uint8_t *p_dst, const wiced_bt_ga_bap_csc_t *args_ptr);

wiced_result_t wiced_bt_ga_bap_parse_csc(uint8_t *stream, int stream_len, wiced_bt_ga_bap_csc_t *p_csc);

wiced_result_t wiced_bt_ga_bap_get_metadata(uint8_t *stream,
                                            int stream_len,
                                            wiced_bt_ga_bap_metadata_t *p_metadata,
                                            uint32_t *response,
                                            uint32_t *reason);

uint8_t *wiced_bt_ga_bap_fill_metadata(uint8_t *p_dst, wiced_bt_ga_bap_metadata_t *p_metadata);

uint8_t wiced_bt_ga_bap_get_sampling_freq_index(uint32_t samplaing_freq);

uint8_t wiced_bt_ga_bap_get_fram_duration_index(uint32_t frame_duration);

void wiced_bt_ga_bap_get_stream_config(uint32_t codec_config, wiced_bt_ga_bap_stream_config_t *p_stream_config);

void wiced_bt_ga_bap_broadcast_get_stream_config(uint32_t codec_config,
                                                 wiced_bt_ga_bap_stream_config_t *p_stream_config);

/** Calculates output sample buffer size
 *
 * @param[in]  sampling_frequency    : Supported sampling rate
 *
 * @param[in]  frame_duration        : Supported frame duration
 *
 * @return  frame size : No. of samples in decoded data
 */
uint32_t wiced_bt_ga_bap_get_decoded_data_size(uint16_t sampling_frequency, uint16_t frame_duration);
