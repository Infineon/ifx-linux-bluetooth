/*
 * $ Copyright Cypress Semiconductor $
 */


/**************************************************************************//**
 * \file <wiced_bt_ga_common.h>
 *
 * Definitions for common functionalities for all the profiles
 *
 */

#ifndef __WICED_BT_GA_COMMON_H__
#define __WICED_BT_GA_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include <string.h>
#include <stddef.h>
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_result.h"
#include "wiced_bt_trace.h"

#include "gatt_interface.h"
#include "wiced_bt_ga.h"

typedef struct {
    const wiced_bt_cfg_settings_t *p_stack_cfg;
    struct vcs_cfg {
        uint8_t max_vcs_vocs;
        uint8_t max_vcs_aics;
    }vcs;

    struct mics_cfg {
        uint8_t max_mics_aics;
    }mics;

    uint8_t max_mcs;
    uint8_t max_tbs;

    uint8_t vcs_step_size;

    uint8_t ascs_max_sink_ase_supported;
    uint8_t ascs_max_source_ase_supported;
    uint8_t pacs_max_sink_capabilities_supported;
    uint8_t pacs_max_source_capabilities_supported;
    uint8_t broadcast_max_big_supported;
    uint8_t broadcast_max_bis_supported_per_big;

    uint8_t bass_max_receive_state_supported;
    uint8_t max_tmas;
} ga_cfg_t;


typedef gatt_intf_string_t wiced_bt_ga_string_t;

#define VOLUME_OFFSET_MIN_VALUE             -255
#define VOLUME_OFFSET_MAX_VALUE             255

//Volume control profile
typedef enum
{
    WICED_BT_MUTE_STATE_MUTED          = 1, /**< Set mute state to mute */
    WICED_BT_MUTE_STATE_NOT_MUTED      = 0, /**< Set mute state to unmute */
    WICED_BT_MUTE_STATE_DISABLED       = 2 /**< Set mute state to unmute */
} wiced_bt_ga_mute_val_t;


typedef enum
{
    WICED_BT_VOLUME_FLAG_VOLUME_SETTING_PERSISTED     = 1, /**< Set volume flag state to volume setting persisted */
    WICED_BT_VOLUME_FLAG_VOLUME_SETTING_NOT_PERSISTED = 0, /**< Set volume flag state to volume setting not persisted */
} wiced_bt_ga_volume_flag_val_t;

typedef enum
{
    WICED_BT_GA_MICS_ERROR_MUTE_DISABLED = 0x80,
} wiced_bt_ga_mics_error_code_t;

typedef enum
{
    WICED_BT_GA_VOCS_ERROR_INVALID_CHANGE_COUNTER = 0x80, /**< Change counter value is invalid*/
    WICED_BT_GA_VOCS_ERROR_OPCODE_NOT_SUPPORTED = 0x81, /**< Opcode is invalid*/
    WICED_BT_GA_VOCS_ERROR_VALUE_OUT_OF_RANGE = 0x82, /**< Change counter value is invalid*/
} wiced_bt_ga_vocs_error_code_t;

typedef enum
{
    WICED_BT_GA_AICS_ERROR_INVALID_CHANGE_COUNTER = 0x80, /**< Change counter value is invalid */
    WICED_BT_GA_AICS_ERROR_OPCODE_NOT_SUPPORTED   = 0x81, /**< Opcode is invalid*/
    WICED_BT_GA_AICS_ERROR_MUTE_DISABLED          = 0x82, /**< Mute Disabled */
    WICED_BT_GA_AICS_ERROR_VALUE_OUT_OF_RANGE     = 0x83, /**< Gain setting valus is out of range */
    WICED_BT_GA_AICS_GAIN_MODE_CHANGE_NOT_ALLOWED = 0x84, /**< Gain Mode Change Not allowed */
} wiced_bt_ga_aics_error_code_t;

typedef enum
{
    WICED_BT_GA_TBS_ERROR_VALUE_CHANGED_READ_LONG = 0x80, /**< Value Changed during read long */
} wiced_bt_ga_tbs_error_code_t;

typedef enum
{
    WICED_BT_GA_MCS_ERROR_VALUE_CHANGED_READ_LONG = 0x80, /**< Value Changed during read long */
} wiced_bt_ga_mcs_error_code_t;

typedef enum
{
    WICED_BT_GA_BASS_ERROR_OPCODE_NOT_SUPPORTED = 0x80, /**< Opcode is invalid*/
    WICED_BT_GA_BASS_ERROR_INVALID_SOURCE_ID    = 0x81, /**< Invalid Source ID*/
} wiced_bt_ga_bass_error_code_t;

typedef enum
{
    WICED_BT_GA_VCS_ERROR_INVALID_CHANGE_COUNTER = 0x80, /**< Change counter value is invalid*/
    WICED_BT_GA_VCS_ERROR_OPCODE_NOT_SUPPORTED = 0x81, /**< Opcode is invalid*/
} wiced_bt_ga_vcs_error_code_t;

/** Definition for application error codes which can be sent by CSIS profile */
typedef enum
{
    WICED_BT_GA_CSIS_ERROR_LOCK_DENIED = 0x80,       /**< error Lock denied */
    WICED_BT_GA_CSIS_ERROR_LOCK_RELEASE_NOT_ALLOWED, /**< error Lock release not allowed*/
    WICED_BT_GA_CSIS_ERROR_INVALID_LOCK_VALUE,       /**< error invalid lock value*/
    WICED_BT_GA_CSIS_ERROR_OOB_SIRK_ONLY,            /**< error when server supports SIRK only through OOB */
    WICED_BT_GA_CSIS_ERROR_LOCK_ALREADY_GRANTED      /**< error when the requester is already owning the lock */
} wiced_bt_ga_csis_error_codes_t;

/* Volume Audio Location */
typedef enum
{
    WICED_BT_VOCS_LEFT_AUDIO,                       /**< Left Audio Location */
    WICED_BT_VOCS_RIGHT_AUDIO,                      /**< Right Audio Location */
} wiced_bt_ga_volume_audio_location_t;

/** @} GA_CLIENT_EVENT */

#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_GA_COMMON_H__ */
