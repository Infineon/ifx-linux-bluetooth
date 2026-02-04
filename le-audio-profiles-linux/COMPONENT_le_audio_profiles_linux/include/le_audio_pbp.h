/*
 * $ Copyright Cypress Semiconductor $
 */

#pragma once
#ifndef __WICED_BT_GA_PBP_H__
#define __WICED_BT_GA_PBP_H__

#include "wiced_result.h"

/**
 * @addtogroup Public_Broadcast_Profile
 * @{
 * @brief Public Broadcast Profile (PBP) defines a Public Broadcast Announcement that a Broadcast Source can include in an extended advertisement to indicate that a Broadcast Source is transmitting at least one of the following:
		- A broadcast Audio Stream with a configuration that every BAP Broadcast Sink can receive and decode (Standard Quality Public Broadcast Audio)
		- A broadcast Audio Stream that uses a High Quality Public Broadcast Audio Stream configuration
	    - The Public Broadcast Announcement also indicates whether the broadcast Audio Streams in the Broadcast Isochronous Group (BIG) are encrypted or not. If the Public Broadcast Announcement indicates that the broadcast Audio Streams are encrypted, a Broadcast Sink must use a Broadcast_Code to decrypt them
 */

#define PBP_BROADCAST_TRACE(...) /**< Enable this for getting PBP library traces */
#define PBP_BROADCAST_TRACE_CRIT(...) /**< Enable this for getting PBP library traces */

#define WICED_BT_GA_PBP_TYPE_BROADCAST_NAME 0x30    /**< adv type: BTM_BLE_ADVERT_TYPE_BROADCAST_NAME */
#define MAX_BROADCAST_NAME_ADV_SIZE 32              /**< MAX size of Broadcast name */
#define PBP_METADATA_PROGRAM_INFO_TYPE 0x03         /**< Program info metadata type */
#define APPEARANCE_VALUE_AD_SIZE 3                  /**< Size of appearance value */
#define MAX_PROGRAM_INFO_SIZE 50                    /**< Max size of Program info */
#define MAX_METADATA_LEN 100                        /**< Max len of metadata */


/**
 * @brief Public broadcast structure
 */
typedef struct {
    wiced_bool_t encryption;     /**< true If BIG is encrypted */
    uint32_t sampling_frequency; /**< Sampling frequency of the broadcast stream */
    uint16_t frame_duration;     /**< Frame duration of the broadcast stream */
    uint8_t metadata_length;     /**< Length of the Metadata field */
    uint8_t *metadata;           /**< Metadata in LTV format */
    uint8_t *broadcast_name;     /**< Userfriendly name of the Broadcast stream */
    size_t name_length;          /**< Length of the name */
    uint16_t appearance_value;   /**< appearance value from the assigned number spec */
    uint8_t program_info_size;   /**< Program Info Size */
    uint8_t *program_info;       /**< Program_Info length-type-value (LTV) structure metadata*/
} le_audio_public_broadcast_t;

/**
 * @brief Public broadcast Audio Stream Config
 */
typedef enum
{
    WICED_BT_GA_PBP_GENERIC_CODEC_PUBLIC_BROADCAST_AUDIO = 0, /**< Generic Codec Broadcast Audio config */
    WICED_BT_GA_PBP_STANDARD_QUALITY_PUBLIC_BROADCAST_AUDIO = 2, /**< Standard Quality Broadcast Audio config*/
    WICED_BT_GA_PBP_HIGH_QUALITY_PUBLIC_BROADCAST_AUDIO = 4, /**< High Quality Broadcast Audio config */
} le_audio_pbp_audio_config_t;

/**
 * @brief Public broadcast Structure from adv
 */
typedef struct
{
    wiced_bool_t encryption;                                          /**< true If BIG is encrypted */
    le_audio_pbp_audio_config_t audio_config;                        /**< Broadcast audio configuration */
    uint8_t public_broadcast_feature;                                 /**< Public Broadcast feature */
    uint8_t metadata_length;                                          /**< Length of metadata */
    uint8_t metadata[MAX_PROGRAM_INFO_SIZE + MAX_METADATA_LEN + 2];   /**< Broadcast Metadata */
    uint8_t broadcast_name[MAX_BROADCAST_NAME_ADV_SIZE];              /**< Broadcast name */
    uint16_t source_appearance_value;                                 /**< Appearance value from the assigend number spec */
} le_audio_rcv_public_broadcast_t;


 /**
  *
  * @brief			Check if Peer adv data has public broadcast info
  *
  * @param[in]		 p_adv_data : Peer device adv data
  * @param[out]		 p_public_rcv_br: Broadcast details parsed from adv data
  * @return 		 wiced_bool_t
  *
  */
wiced_bool_t le_audio_pbp_is_public_broadcast(uint16_t adv_len,
                                              uint8_t *p_adv_data,
                                              le_audio_rcv_public_broadcast_t *p_public_rcv_br);

 /**
  *
  * @brief			Build public broadcast adv data from the configuration
  *
  * @param[out]		 p_data : Adv data to be filled after building. Caller has to allocate enough memory
  * @param[in]		 offset: Offset for extended adv to be considered before building
  * @param[in]       public_broadcast: Broadcast data to be considered for filling adv data
  * @param[out]      len : length of the adv data after building
  * @return 		 wiced_bool_t
  *
  */
 wiced_result_t le_audio_pbp_build_adv_data(uint8_t *p_data,
                                               uint32_t offset,
                                               le_audio_public_broadcast_t *public_broadcast,
                                               uint32_t *len);
 /**@} Public_Broadcast_Profile */

#endif //__WICED_BT_GA_PBP_H__
