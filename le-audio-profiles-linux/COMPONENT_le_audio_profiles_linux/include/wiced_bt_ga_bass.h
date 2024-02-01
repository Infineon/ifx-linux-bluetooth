/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __WICED_BT_GA_BASS_H__
#define __WICED_BT_GA_BASS_H__

#include "gatt_interface.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ga_bap.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_gatt.h"

#define WICED_BT_GA_BASS_MAX_SUBGROUP_COUNT 4
#define WICED_BT_GA_BASS_MEDATA_MAX_LEN 255

#define WICED_BT_GA_BASS_ADV_DATA_SIZE 6
#define WICED_BT_GA_BASS_SYNC_EST_MASK (1)
#define WICED_BT_GA_BASS_SYNC_LOST_MASK (~1)
#define WICED_BT_GA_BASS_MAX_ADV_SID  0x0F
#define WICED_BT_GA_BASS_BIS_INDEX_MASK 0x7FFF

#define BASS_TRACE(...)
#define BASS_TRACE_CRIT(...)

/** PA Sync State. Refer BASS SPEC Broadcast Receive State Characteristic Section 3.2 */
typedef enum
{
    WICED_BT_GA_BASS_PA_NOT_SYNC,         /**< Not synchronized to PA */
    WICED_BT_GA_BASS_PA_SYNC_INFO_REQUST, /**< SyncInfo Request */
    WICED_BT_GA_BASS_PA_SYNC,             /**< Synchronized to PA */
    WICED_BT_GA_BASS_PA_FAILED_SYNC,      /**< Failed to synchronize to PA */
    WICED_BT_GA_BASS_PA_NO_PAST,          /**< No PAST */
} wiced_bt_ga_bass_pa_sync_state_t;

/** PA Sync PARAM. Refer BASS SPEC Section 3.1.1.4 Add Source operation */
typedef enum
{
    WICED_BT_GA_BASS_PA_NO_SYNC,         /**< Do not synchronize to PA  */
    WICED_BT_GA_BASS_PA_SYNC_USING_PAST, /**< Synchronize to PA � PAST available */
    WICED_BT_GA_BASS_PA_SYNC_NO_PAST,    /**< Synchronize to PA � PAST not available */
} wiced_bt_ga_bass_pa_sync_param_t;

/** BIG Encryption state. Refer BASS SPEC Broadcast Receive State Characteristic Section 3.2 */
typedef enum
{
    WICED_BT_GA_BASS_BIG_NOT_ENCRYPTED,           /**< Not encrypted */
    WICED_BT_GA_BASS_BIG_BROADCAST_CODE_REQUIRED, /**< Broadcast_Code required */
    WICED_BT_GA_BASS_BIG_DECRPTING,               /**< Decrypting */
    WICED_BT_GA_BASS_BIG_BAD_BROADCAST_CODE,      /**< Bad_Code (incorrect encryption key) */
} wiced_bt_ga_bass_big_encryption_state_t;

/** BASS Control Point Opcode. Refer BASS SPEC Broadcast Audio Scan Control Point Characteristic Section 3.1*/
typedef enum
{
    WICED_BT_GA_BASS_OP_REMOTE_SCAN_STOPPED, /**< Informs the server that the client is not scanning for Broadcast Sources on behalf of the server */
    WICED_BT_GA_BASS_OP_REMOTE_SCAN_STARTED, /**< Informs the server that the client is scanning for Broadcast Sources on behalf of the server. */
    WICED_BT_GA_BASS_OP_ADD_SOURCE, /**< Requests the server to add information including Metadata for a Broadcast Source, and requests the server to synchronize to a PA and/or BIS transmitted by the Broadcast Source */
    WICED_BT_GA_BASS_OP_MODIFY_SOURCE, /**< Requests the server to update Metadata, to synchronize to, or to stop synchronizing to a PA and/or BIS transmitted by the Broadcast Source identified by the Source_ID */
    WICED_BT_GA_BASS_OP_SET_BROADCAST_CODE, /**< Provides the server with the Broadcast_Code to decrypt a BIS transmitted by a Broadcast Source identified by the Source_ID. */
    WICED_BT_GA_BASS_OP_REMOVE_SOURCE, /**< Requests the server to remove all information for a Broadcast Source identified by the Source_ID */
} wiced_bt_ga_bass_opcode_t;

/** Broadcast Subgroup data. BASS Control Point Opcode. Refer BASS SPEC Broadcast Audio Scan Control Point Characteristic Section 3.1 */
typedef struct
{
    uint32_t bis_sync_state;              /**< BIG Sync State. Bit 0-30 = BIS_index[1-31]
                                                                 0b0 = Not synchronized to BIS
                                                                 0b1 = Synchronized to BIS */
    wiced_bt_ga_bap_metadata_t meta_data; /**< LTV-formatted Metadata  */
} wiced_bt_ga_bass_sub_group_data_t;

/** BASS Common Source Data */
typedef struct
{
    wiced_bt_ga_bass_pa_sync_param_t pa_sync_param; /**< PA Sync Parameter */
    uint16_t pa_interval; /**< SyncInfo field Interval parameter value. 0xFFFF: PA_Interval unknown*/
    uint8_t num_subgroup; /**< Number of subgroups  */
    wiced_bt_ga_bass_sub_group_data_t sub_group_data[WICED_BT_GA_BASS_MAX_SUBGROUP_COUNT]; /**< Subgroup data array */
} wiced_bt_ga_bass_common_source_data_t;

/** BASS Add Source Operation parameter */
typedef struct
{
    wiced_bt_ble_address_t source_addr; /**< Broadcast Source address */
    uint8_t adv_sid;                    /**< Advertising_SID subfield of the ADI field of the AUX_ADV_IND PDU or
    the LL_PERIODIC_SYNC_IND containing the SyncInfo that points to the PA transmitted by the Broadcast Source */
    uint32_t broadcast_id;              /**< Broadcast ID */
    wiced_bt_ga_bass_common_source_data_t src_data; /**< Broadcast Source Data */
} wiced_bt_ga_bass_add_source_data_t;

/** BASS Modify Source Operation parameter */
typedef struct
{
    uint8_t source_id; /**< Source_ID assigned by the server to a Broadcast Receive State characteristic */
    wiced_bt_ga_bass_common_source_data_t src_data; /**< Broadcast Source Data */
} wiced_bt_ga_bass_modify_source_data_t;

/** BASS Set Broadcast Code Operation parameter */
typedef struct
{
    uint8_t source_id; /**< Source_ID assigned by the server to a Broadcast Receive State characteristic */
    wiced_bt_bap_broadcast_code_t broadcast_code; /**< Broadcast_Code for the Source_ID assigned to
        a Broadcast Receive State characteristic */
} wiced_bt_ga_bass_set_broadcast_code_data_t;

/** BASS Operation data */
typedef union
{
    wiced_bt_ga_bass_add_source_data_t add_source_param;            /**< Add Source Parameter */
    wiced_bt_ga_bass_modify_source_data_t modify_source_param;      /**< Modifu Source Parameter */
    wiced_bt_ga_bass_set_broadcast_code_data_t set_broadcast_param; /**< Set Broadcast Code Parameter */
    uint8_t remove_source_id; /**< Source_ID assigned by the server to a Broadcast Receive State characteristic */
} wiced_bt_ga_bass_operation_data_t;

/** BASS Receive State Data */
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
} wiced_bt_ga_bass_receive_state_data_t;

/** BASS Operation Data */
typedef struct
{
    wiced_bt_ga_bass_opcode_t opcode;       /**< BASS Control Opcode */
    wiced_bt_ga_bass_operation_data_t data; /**< BASS Control Operation data */
} wiced_bt_ga_bass_operation_t;

/**
* Initialize the BASS service_type/profile
*/
wiced_result_t wiced_bt_ga_bass_init(ga_cfg_t *p_cfg);

wiced_bt_gatt_status_t wiced_bt_ga_bass_enable_notifications_all(uint16_t conn_id,
                                                                 gatt_intf_service_object_t *p_service);

wiced_bool_t wiced_bt_ga_bass_discovery_in_progress(uint16_t conn_id, gatt_intf_service_object_t *p_service);

uint8_t wiced_bt_ga_bass_parse_receive_state_char_header(uint8_t *p_data,
                                                         wiced_bt_ga_bass_receive_state_data_t *p_recv_state);

uint8_t wiced_bt_ga_bass_parse_sub_group_data(uint8_t *p_data, wiced_bt_ga_bass_sub_group_data_t *sub_group_data);

wiced_bt_gatt_status_t wiced_bt_ga_bass_parse_control_point_data(uint8_t *p_data,
                                                                 uint16_t total_len,
                                                                 wiced_bt_ga_bass_operation_t *p_operation_data);

wiced_bool_t wiced_bt_ga_bass_broadcast_is_solicitation_request(uint8_t *p_adv_data);
#endif // __WICED_BT_GA_BASS_H__
