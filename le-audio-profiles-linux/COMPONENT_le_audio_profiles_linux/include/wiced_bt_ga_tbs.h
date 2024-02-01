/*
 * $ Copyright Cypress Semiconductor $
 */

/** @file
 *
 * Telephone Bearer Service (TBS) Application Programming Interface
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "gatt_interface.h"
#include "wiced_bt_ga_tbs_common.h"
#include "wiced_bt_ga_common.h"

#define TBS_TRACE(...) WICED_BT_TRACE(__VA_ARGS__)
#define TBS_TRACE_CRIT(...) WICED_BT_TRACE(__VA_ARGS__)

/**
 * @addtogroup Telephone_Bearer_Service_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_tbs
 * @{
 */
 
/**
 * @anchor TBS_FEATURE_BIT
 * @name TBS feature bit values
 * @{ */
#define WICED_BT_GA_TBS_FEATURE_BIT_INBAND_RINGTONE         1   /**< inband ringtone: 0 = disabled, 1 = enabled */
#define WICED_BT_GA_TBS_FEATURE_BIT_SILENT_MODE             2   /**< silent_mode:     0 = not in silent mode, 1 = in silent mode */
/** @} TBS_FEATURE_BIT */

/**
 * @anchor TBS_CONTROL_POINT_OPTIONAL_OPCODES
 * @name TBS controlpoint optional opcodes
 * @{ */
#define WICED_BT_GA_TBS_FEATURE_BIT_JOIN                    2   /**< Join Call Control Point Opcode:  0 = not supported, 1 = supported*/
#define WICED_BT_GA_TBS_FEATURE_BIT_LOCAL_HOLD              1   /**< Local Hold and Local Retrieve: 0 = not supported 1 = supported*/
/** @} TBS_CONTROL_POINT_OPTIONAL_OPCODES */

/** Telephone bearer service control point data */
typedef struct
{
    wiced_bt_ga_tbs_call_action_t                   opcode; /**< TBS control point opcode*/
    union {
        uint8_t                                     call_id;  /**< Call ID in case of WICED_BT_GA_CCP_ACTION_ACCEPT_CALL, WICED_BT_GA_CCP_ACTION_TERMINATE_CALL, WICED_BT_GA_CCP_ACTION_HOLD_CALL, WICED_BT_GA_CCP_ACTION_RETRIEVE_CALL*/
        wiced_bt_ga_tbs_join_call_data_t            join_call_data; /**< Join call data in case the operation is join calls in case of WICED_BT_GA_CCP_ACTION_JOIN_CALL*/
        wiced_bt_ga_string_t                        place_call_uri; /**< Call URI string in case of WICED_BT_GA_CCP_ACTION_ORIGINATE */
        wiced_bt_ga_tbs_call_operation_response_t   call_op_resp; /**< Result of the call control operation */
        wiced_bt_ga_tbs_call_termination_reason_data_t termination_data; /**< Reason for termination of the call */
    };
} wiced_bt_ga_tbs_call_control_point_t;

/** telephone bearer service data */
typedef union
{
    wiced_bt_ga_string_t                              bearer_provider_name; /**< Bearer Provider Name */
    wiced_bt_ga_string_t                              bearer_UCI; /**< Bearer UCI */
    wiced_bt_ga_tbs_bearer_technology_t               bearer_technology; /**< Bearer Technology */
    wiced_bt_ga_string_t                              bearer_URI_supported_schemes_list;/**< Bearer URI */
    uint8_t                                           bearer_signal_strength;/**< Bearer Signal Strength */
    uint8_t                                           bearer_signal_strength_reporting_interval;/**< Bearer Signal Strength Reporting Interval */
    uint16_t                                          status_flag; /**< supported flags */
    wiced_bt_ga_tbs_call_state_list_t                 call_state_list; /**< Call State list */
    uint8_t                                           content_control_id; /**< Content control ID */
    wiced_bt_ga_tbs_call_termination_reason_data_t    call_termination_reason;/**< Call termination reason */
    wiced_bt_ga_tbs_incoming_call_t                   incoming_call; /**< Incoming_remote_caller_id */
    wiced_bt_ga_tbs_incoming_tg_bearer_uri_t           incoming_tg_caller_id; /**< Incoming target caller ID */
    wiced_bt_ga_tbs_current_call_list_t               current_call_list; /**< Current call list */
    wiced_bt_ga_tbs_call_control_point_t              call_action; /**< Call control point data */
    uint16_t                                          ccp_supported_opcode; /**< Supported opcode flags*/
    wiced_bt_ga_tbs_user_friendly_call_data_t         call_friendly_name; /**< Call friendly name */
} wiced_bt_ga_tbs_data_t;

/**
* Initialize the GTBS service_type/profile
* @param[in] p_cfg : Generic Audio configuration
*/
wiced_result_t wiced_bt_ga_gtbs_init(ga_cfg_t *p_cfg);

/**
* Initialize the TBS service_type/profile
* @param[in] p_cfg : Generic Audio configuration
*/
wiced_result_t wiced_bt_ga_tbs_init(ga_cfg_t *p_cfg);

/**@} wiced_bt_ga_tbs */
/**@} Telephone_Bearer_Service_APIs */
#ifdef __cplusplus
}
#endif

