/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __WICED_BT_GA_PACS_H__
#define __WICED_BT_GA_PACS_H__

#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_bap.h"

/**
 * @addtogroup Published_Audio_Capability_Service_APIs
 * @{
 * @brief  PACS can be instantiated on devices that can accept the establishment of unicast Audio Streams or devices that can receive broadcast Audio Streams. Examples of such devices are speakers, headsets, hearing aids, and microphones.
 - Servers expose one or more sets of audio capabilities and audio availability. Sets of audio capabilities,known as Published Audio Capability (PAC) records, are exposed by using either the Sink PAC characteristic or Source PAC characteristic. Clients can discover and read these characteristics, and servers can notify these characteristics.
 */


#define MAX_CODEC_SPECIFIC_CAPABILITIES_LENGTH 128 /**< Length of codec specific capabilities */
#define MAX_METADATA_LENGTH 128 /**< Maximum length for metadata */


/**
 * @brief PACS codec id information
 *
 */
typedef struct
{
    uint8_t coding_format; /**< Coding format  */
    uint16_t company_id;  /**< Company ID from the assigned value document */
    uint16_t vendor_specific_codec_id; /**< Vendor specific codec id value */
} wiced_bt_ga_pacs_codec_id_t;

/**
 * @brief Defines PAC record
 *
 */
typedef struct
{
    wiced_bt_ga_pacs_codec_id_t codec_id; /**< codec id information for the pac record */
    uint8_t codec_specific_capabilities_length; /**< Length of the codec specific capabilities */
    uint8_t codec_specific_capabilities[MAX_CODEC_SPECIFIC_CAPABILITIES_LENGTH]; /**< Codec specific capabilities */
    uint8_t metadata_length; /**< Length of the metadata information */
    uint8_t metadata[MAX_METADATA_LENGTH]; /**< Metadata for the pacs record */
} wiced_bt_ga_pacs_record_t;

/**
 * @brief PACS characteristic data
 *
 */
typedef struct
{
    uint8_t num_of_records; /**< Number of PACS records */
    wiced_bt_ga_pacs_record_t *record_list; /**< List of PACS records */
} wiced_bt_ga_pacs_char_data_t;

/**
 * @brief PACS audio context information
 *
 */
typedef struct
{
    wiced_bt_ga_bap_context_type_t source_contexts; /**< Source audio contexts */
    wiced_bt_ga_bap_context_type_t sink_contexts; /**< Sink audio contexts */
} wiced_bt_ga_pacs_audio_contexts_t;


typedef uint32_t wiced_bt_ga_pacs_audio_location_t; /**<Device-wide bitmap of supported Audio Location values for all PAC records */

/**
 * @brief Defines all the fields required for Published Audio Capability Service
 *
 */
typedef struct
{
    uint8_t char_instance; /**< Characteristic instance of the pacs data */
    wiced_bt_ga_pacs_char_data_t source_pac_list; /**< List of source pac records */
    wiced_bt_ga_pacs_char_data_t sink_pac_list; /**< List of Sink pac records */
    wiced_bt_ga_pacs_audio_location_t source_audio_location; /**< Audio location for the source ase from assigned doc */
    wiced_bt_ga_pacs_audio_location_t sink_audio_location;/**< Audio location for the sink ase from assigned doc */
    wiced_bt_ga_pacs_audio_contexts_t supported; /**< Supported audio contexts */
    wiced_bt_ga_pacs_audio_contexts_t available; /**< Available audio contexts */
} wiced_bt_ga_pacs_data_t;

/** PACS initialization data */
typedef struct
{
    uint8_t max_snk_caps_spt;   /**< MAX Number of Sink Capabilities */
    uint8_t max_src_caps_spt; /**< MAX Number of Source Capabilities */
} wiced_bt_ga_pacs_init_data_t;

/**
 * @brief Initialize the PACS service/profile
 *
 * @param[in] num_instances : Number of instances
 * @param[in] p_ini : see #wiced_bt_ga_pacs_init_data_t
 * @result result of the init operation
 */
wiced_result_t wiced_bt_ga_pacs_init(uint8_t num_instances, wiced_bt_ga_pacs_init_data_t *p_ini);

/** Enable PACS server module */
void wiced_bt_ga_pacs_enable_server(void);

/** Enable PACS client module */
void wiced_bt_ga_pacs_enable_client(void);

/**
 * @brief Notify all the PACS characteristic values.
 *
 * @param[in] conn_id : GATT Connection ID
 * @param[in] p_service : PACS service object
 * @param[in] p_notification : Notification data
 */

void wiced_bt_ga_pacs_notify_all_characteristics(uint16_t conn_id, gatt_intf_service_object_t *p_service,
                                                 wiced_bt_ga_pacs_data_t *p_notification);

#endif // __WICED_BT_GA_PACS_H__
/**@} Published_Audio_Capability_Service_APIs */
