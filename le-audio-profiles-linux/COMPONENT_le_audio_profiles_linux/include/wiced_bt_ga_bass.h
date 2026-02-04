/*
 * $ Copyright Cypress Semiconductor $
 */


/**
 * @addtogroup Stream_Control_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_bass
 * @{

@brief - BASS can be instantiated on servers to solicit for clients to scan on behalf of the server for broadcast Audio Streams and associated data that are transmitted by Broadcast Sources.
 - Clients scanning on behalf of the server can help reduce the need to scan by the server and reduce power consumption on the server.
 - Servers can receive information from clients that is associated with broadcast Audio Streams, including decryption keys known as Broadcast_Codes necessary to decrypt encrypted BISes.
 */

#ifndef __WICED_BT_GA_BASS_H__
#define __WICED_BT_GA_BASS_H__

#include "gatt_interface.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ga_bap.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_gatt.h"

#define WICED_BT_GA_BASS_MAX_SUBGROUP_COUNT 4 /**< Max subgroups possible */
#define WICED_BT_GA_BASS_MEDATA_MAX_LEN 255 /**< Max len of metadata allowed */
#define WICED_BT_GA_BASS_ADV_DATA_SIZE 6 /**< BASS adv data length */
#define WICED_BT_GA_BASS_SYNC_EST_MASK (1) /**< BASS sync establishment mask */
#define WICED_BT_GA_BASS_SYNC_LOST_MASK (~1) /**< BASS sync lost mask */
#define WICED_BT_GA_BASS_MAX_ADV_SID  0x0F /**< MAX adv sid value */
#define WICED_BT_GA_BASS_BIS_INDEX_MASK 0x7FFF /**< BIS index mask */

/** @brief PA Sync State. Refer BASS SPEC Broadcast Receive State Characteristic Section 3.2 */
enum wiced_bt_ga_bass_pa_sync_state_e
{
    WICED_BT_GA_BASS_PA_NOT_SYNC,         /**< Not synchronized to PA */
    WICED_BT_GA_BASS_PA_SYNC_INFO_REQUEST, /**< SyncInfo Request */
    WICED_BT_GA_BASS_PA_SYNC,             /**< Synchronized to PA */
    WICED_BT_GA_BASS_PA_FAILED_SYNC,      /**< Failed to synchronize to PA */
    WICED_BT_GA_BASS_PA_NO_PAST,          /**< No PAST */
};
typedef uint8_t wiced_bt_ga_bass_pa_sync_state_t; /**< PA Sync states (see #wiced_bt_ga_bass_pa_sync_state_e) */

/** PA Sync PARAM. Refer BASS SPEC Section 3.1.1.4 Add Source operation */
enum wiced_bt_ga_bass_pa_sync_param_e
{
    WICED_BT_GA_BASS_PA_NO_SYNC,         /**< Do not synchronize to PA  */
    WICED_BT_GA_BASS_PA_SYNC_USING_PAST, /**< Synchronize to PA, PAST available */
    WICED_BT_GA_BASS_PA_SYNC_NO_PAST,    /**< Synchronize to PA,  PAST not available */
};
typedef uint8_t wiced_bt_ga_bass_pa_sync_param_t;  /**< PA Sync param (see #wiced_bt_ga_bass_pa_sync_param_e) */

/** BIG Encryption state. Refer BASS SPEC Broadcast Receive State Characteristic Section 3.2 */
enum wiced_bt_ga_bass_big_encryption_state_e
{
    WICED_BT_GA_BASS_BIG_NOT_ENCRYPTED,           /**< Not encrypted */
    WICED_BT_GA_BASS_BIG_BROADCAST_CODE_REQUIRED, /**< Broadcast_Code required */
    WICED_BT_GA_BASS_BIG_DECRPTING,               /**< Decrypting */
    WICED_BT_GA_BASS_BIG_BAD_BROADCAST_CODE,      /**< Bad_Code (incorrect encryption key) */
};
typedef uint8_t wiced_bt_ga_bass_big_encryption_state_t;  /**<  BIG          encryption states  (see #wiced_bt_ga_bass_pa_sync_param_e) */

/** BASS Control Point Opcode. Refer BASS SPEC Broadcast Audio Scan Control Point Characteristic Section 3.1*/
enum wiced_bt_ga_bass_opcode_e
{
    WICED_BT_GA_BASS_OP_REMOTE_SCAN_STOPPED, /**< Informs the server that the client is not scanning for Broadcast Sources on behalf of the server */
    WICED_BT_GA_BASS_OP_REMOTE_SCAN_STARTED, /**< Informs the server that the client is scanning for Broadcast Sources on behalf of the server. */
    WICED_BT_GA_BASS_OP_ADD_SOURCE, /**< Requests the server to add information including Metadata for a Broadcast Source, and requests the server to synchronize to a PA and/or BIS transmitted by the Broadcast Source */
    WICED_BT_GA_BASS_OP_MODIFY_SOURCE, /**< Requests the server to update Metadata, to synchronize to, or to stop synchronizing to a PA and/or BIS transmitted by the Broadcast Source identified by the Source_ID */
    WICED_BT_GA_BASS_OP_SET_BROADCAST_CODE, /**< Provides the server with the Broadcast_Code to decrypt a BIS transmitted by a Broadcast Source identified by the Source_ID. */
    WICED_BT_GA_BASS_OP_REMOVE_SOURCE, /**< Requests the server to remove all information for a Broadcast Source identified by the Source_ID */
};
typedef uint8_t wiced_bt_ga_bass_opcode_t;  /**< BASS Opcodes (see #wiced_bt_ga_bass_opcode_e) */

/** @brief Broadcast Subgroup data. BASS Control Point Opcode. Refer BASS SPEC Broadcast Audio Scan Control Point Characteristic Section 3.1 */
typedef struct
{
    uint32_t bis_sync_state;              /**< BIG Sync State. Bit 0-30 = BIS_index[1-31]
                                                                 0b0 = Not synchronized to BIS
                                                                 0b1 = Synchronized to BIS */
    wiced_bt_ga_bap_metadata_t meta_data; /**< LTV-formatted Metadata  */
} wiced_bt_ga_bass_sub_group_data_t;

/** @brief BASS Common Source Data */
typedef struct
{
    wiced_bt_ga_bass_pa_sync_param_t pa_sync_param; /**< PA Sync Parameter */
    uint16_t pa_interval; /**< SyncInfo field Interval parameter value. 0xFFFF: PA_Interval unknown*/
    uint8_t num_subgroup; /**< Number of subgroups  */
    wiced_bt_ga_bass_sub_group_data_t sub_group_data[WICED_BT_GA_BASS_MAX_SUBGROUP_COUNT]; /**< Subgroup data array */
} wiced_bt_ga_bass_common_source_data_t;

/** @brief BASS Add Source Operation parameter */
typedef struct
{
    wiced_bt_ble_address_t source_addr; /**< Broadcast Source address */
    uint8_t adv_sid;                    /**< Advertising_SID subfield of the ADI field of the AUX_ADV_IND PDU or
    the LL_PERIODIC_SYNC_IND containing the SyncInfo that points to the PA transmitted by the Broadcast Source */
    uint32_t broadcast_id;              /**< Broadcast ID */
    wiced_bt_ga_bass_common_source_data_t src_data; /**< Broadcast Source Data */
} wiced_bt_ga_bass_add_source_t;

/** @brief BASS Modify Source Operation parameter */
typedef struct
{
    uint8_t source_id; /**< Source_ID assigned by the server to a Broadcast Receive State characteristic */
    wiced_bt_ga_bass_common_source_data_t src_data; /**< Broadcast Source Data */
} wiced_bt_ga_bass_modify_source_t;

/** @brief BASS Set Broadcast Code Operation parameter */
typedef struct
{
    uint8_t source_id; /**< Source_ID assigned by the server to a Broadcast Receive State characteristic */
    wiced_bt_bap_broadcast_code_t broadcast_code; /**< Broadcast_Code for the Source_ID assigned to
        a Broadcast Receive State characteristic */
} wiced_bt_ga_bass_set_broadcast_code_t;

/** @brief BASS Operation data */
typedef union
{
    wiced_bt_ga_bass_add_source_t add_source_param;            /**< Add Source Parameter */
    wiced_bt_ga_bass_modify_source_t modify_source_param;      /**< Modifu Source Parameter */
    wiced_bt_ga_bass_set_broadcast_code_t set_broadcast_param; /**< Set Broadcast Code Parameter */
    uint8_t remove_source_id; /**< Source_ID assigned by the server to a Broadcast Receive State characteristic */
} wiced_bt_ga_bass_operation_data_t;

/** @brief BASS Receive State Data */
typedef struct
{
    uint8_t source_id;                  /**< Assigned by the server */
    wiced_bt_ble_address_t source_addr; /**< Source Address */
    uint8_t adv_sid;                    /**< Advertising_SID subfield of the ADI field of the AUX_ADV_IND PDU or
    the LL_PERIODIC_SYNC_IND containing the SyncInfo that points to the PA transmitted by the Broadcast Source */
    uint32_t broadcast_id;              /**< Broadcast_ID of the Broadcast Source */
    wiced_bt_ga_bass_pa_sync_state_t pa_sync_state;               /**< PA Sync State */
    wiced_bt_ga_bass_big_encryption_state_t big_encryption_state; /**< BIG Encryption State */
    wiced_bt_bap_broadcast_code_t broadcast_code;                 /**< Broadcast_Code that fails to decrypt the BIG.
    Valid Only if big_encryption_state=WICED_BT_GA_BASS_BIG_BAD_BROADCAST_CODE */
    uint8_t num_subgroup;                                         /**< Number of subgroups */
    wiced_bt_ga_bass_sub_group_data_t *sub_group_data;            /**< Subgroup Data */
} wiced_bt_ga_bass_receive_state_t;

/** @brief BASS Operation Data */
typedef struct
{
    wiced_bt_ga_bass_opcode_t opcode;       /**< BASS Control Opcode */
    wiced_bt_ga_bass_operation_data_t data; /**< BASS Control Operation data */
} wiced_bt_ga_bass_operation_t;

/** BASS init data for service object */
typedef struct
{
    uint8_t max_broadcast_receive_state; /**< Max number of Broadcast Receive States per BASS service instance */
} wiced_bt_ga_bass_init_data_t;

/**
 * @brief Initialize the BASS service_type/profile
 *
 * @param[in] num_instances: Number of BASS instances to be created.
 * @param[in] pv_ini : see \ref wiced_bt_ga_bass_init_data_t
 */
wiced_result_t wiced_bt_ga_bass_init(uint8_t num_instances, wiced_bt_ga_bass_init_data_t *pv_ini);

/** Enable BASS server module */
void wiced_bt_ga_bass_enable_server(void);

/** Enable BASS client module */
void wiced_bt_ga_bass_enable_client(void);

/**
 * @brief Subscribe to all characteristics of bass server
 *
 * @param[in]   conn_id : GATT Connection ID
 * @param[in]   p_service :      instance of the bass
 */
wiced_bt_gatt_status_t wiced_bt_ga_bass_enable_notifications_all(uint16_t conn_id,
                                                                 gatt_intf_service_object_t *p_service);

/**
 * @brief Check if BASS discovery is in progress
 * @return      status true or false
 */
wiced_bool_t wiced_bt_ga_bass_discovery_in_progress(void);


/**
 * @brief Get bass receive state values from the stream
 *
 * @param[in]   p_data : Stream received from the peer device
 * @param[out]  p_recv_state : values of the receive state characteristic after parsing
 */
uint8_t wiced_bt_ga_bass_parse_receive_state_char_header(uint8_t *p_data,
                                                         wiced_bt_ga_bass_receive_state_t *p_recv_state);
/**
 * @brief Get subgroup data from the stream
 *
 * @param[in]   p_data : Stream received from the peer device
 * @param[out]  sub_group_data : values of the subgroup after parsing
 */
uint8_t wiced_bt_ga_bass_parse_sub_group_data(uint8_t *p_data, wiced_bt_ga_bass_sub_group_data_t *sub_group_data);

/**
 * @brief Get control point data from the stream
 *
 * @param[in]   p_data : Stream received from the peer device
 * @param[in]  total_len : lenght of the stream
 * @param[out] p_operation_data : control point data after parsing
 * @return status  status of parsing
 */
wiced_bt_gatt_status_t wiced_bt_ga_bass_parse_control_point_data(uint8_t *p_data,
                                                                 uint16_t total_len,
                                                                 wiced_bt_ga_bass_operation_t *p_operation_data);

/**
 * @brief Check if peer device is scan delegator
 *
 * @param[in]  p_adv_data : adv received from the peer device
 * @return true if peer device is scan delegator false otherwise
 */

wiced_bool_t wiced_bt_ga_bass_broadcast_is_solicitation_request(uint16_t adv_len, uint8_t *p_adv_data);
#endif // __WICED_BT_GA_BASS_H__
/**@} wiced_bt_ga_bass */
/**@} Stream_Control_APIs */
