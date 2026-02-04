/*
 * $ Copyright Cypress Semiconductor $
 */

/** @file
 *
 * Telephone Bearer Service (TBS) Application Programming Interface
 */

#ifndef __WICED_BT_GA_TBS_H__
#define __WICED_BT_GA_TBS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "gatt_interface.h"
#include "wiced_bt_ga_common.h"

/**
 * @addtogroup Telephone_Bearer_Service_APIs
 * @{
 * @brief TBS and GTBS both expose characteristics that provide status and control of the telephone bearers. A device can have multiple, logically separate, telephone bearers. A TBS instance provides status and control for a specific telephone bearer within the device. A device implements TBS instances when the
     device wants to allow clients to directly access and control the characteristics of separate internal telephone bearer entities.
    - GTBS provides a single point of access and exposes a representation of its internal telephone bearers into a single telephone bearer.
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

 /**
  * @anchor TBS_MAX_SUPPORTED_LEN
  * @name TBS max supported len
  * @{ */
#define WICED_BT_GA_TBS_BEARER_NAME_MAX_SIZE          50 /**< Max Bearer name size supported*/
#define WICED_BT_GA_TBS_BEARER_UCI_MAX_SIZE           50 /**< Max Uniform Caller Identifier size supported*/
#define WICED_BT_GA_TBS_BEARER_URI_MAX_SIZE           50 /**< Max Uniform Resource Intentifier size supported*/
#define MAX_DATA_LEN                                  100/**< Max data length supported */
#define WICED_BT_GA_TBS_RM_CALLERID_MAX_SIZE          50 /**< Max size of uri prefix and remote caller id*/
#define WICED_BT_GA_TBS_TG_CALLERID_MAX_SIZE          50 /**< Max size of target caller id*/
#define WICED_BT_GA_TBS_FRIENDLY_NAME_MAX_SIZE        50 /**< Max size of call friendly name*/
#define WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT         10  /**< Max call count suppoorted */
 /** @} TBS_MAX_SUPPORTED_LEN */

 /**
  * @anchor TBS_PARAM_LEN
  * @name TBS param length
  * @{ */
#define WICED_BT_GA_TBS_BEARER_TECHNOLOGY_LEN                  1 /**< Bearer technology length*/
#define WICED_BT_GA_TBS_SIGNAL_STRENGTH_LEN                    1 /**< Signal strength length*/
#define WICED_BT_GA_TBS_SIGNAL_STRENGTH_REPORTING_INTERVAL_LEN 1 /**< Signal strength reporting interval length*/
#define WICED_BT_GA_TBS_CONTENT_CONTROL_ID_LEN                 1 /**< Content control ID length */
#define WICED_BT_GA_TBS_CCP_SUPPORTED_OPCODE                   2 /**< CCP Supported opcode length */
#define WICED_BT_GA_TBS_BEARER_STATUS_FLAG_LEN                 2 /**< Status flag length */
#define WICED_BT_GA_TBS_OPTIONAL_OPCODE_LEN                    2 /**< TBS optional opcode length */
#define WICED_BT_GA_TBS_CALL_STATE_LEN                         2 /**< Call state length */
#define WICED_BT_GA_TBS_CALL_ACTION_RESP_LEN                   3 /**< Call action response length */
 /** @} TBS_PARAM_LEN */

 /**
  * @anchor CALL_FLAG
  * @name Call flag values
  * @{ */
#define WICED_BT_GA_TBS_INCOMING_CALL               0 /**< Call flag value for incoming call */
#define WICED_BT_GA_TBS_OUTGOING_CALL               1 /**< Call flag value for outgoing call */
#define WICED_BT_GA_TBS_INFO_PROVIDED_BY_SERVER     0 /**< Call flag value information provided by server */
#define WICED_BT_GA_TBS_INFO_WITHHELD_BY_SERVER     2 /**< Call flag value information withheld by server */
#define WICED_BT_GA_TBS_INFO_PROVIDED_BY_NETWORK    0 /**< Call flag value information provided by network */
#define WICED_BT_GA_TBS_INFO_WITHHELD_BY_NETWORK    4 /**< Call flag value information withheld by network */
 /** @} CALL_FLAG */

 /** All posible termination reason */
 enum wiced_bt_ga_tbs_call_termination_reason_e
 {
	 WICED_BT_GA_TBS_IMPROPER_REMOTE_CALLER_ID	 = 0x00, /**< remote Caller ID value used to place a call was formed improperly. */
	 WICED_BT_GA_TBS_CALL_FAIL				 = 0x01, /**< unable to make the call */
	 WICED_BT_GA_TBS_REMOTE_CALL_END		 = 0x02, /**< remote party ended the call */
	 WICED_BT_GA_TBS_SERVER_CALL_END		 = 0x03, /**< call ended from the server */
	 WICED_BT_GA_TBS_LINE_BUSY				 = 0x04, /**< line busy */
	 WICED_BT_GA_TBS_NETWORK_CONGESTION 	 = 0x05, /**< network congestion */
	 WICED_BT_GA_TBS_CLIENT_TERMINATED		 = 0x06, /**< client terminated the call */
	 WICED_BT_GA_TBS_NO_SERVICE 			 = 0x07, /**< No service */
	 WICED_BT_GA_TBS_NO_ANSWER				 = 0x08, /**< No answer */
	 WICED_BT_GA_TBS_UNSPECIFIED			 = 0x09, /**< Reason is not specified */
 };
typedef uint8_t wiced_bt_ga_tbs_call_termination_reason_t;  /**< Termination reason (see #wiced_bt_ga_tbs_call_termination_reason_e) */

 /** Bearer technology values */
 enum wiced_bt_ga_tbs_bearer_technology_e
 {
	 WICED_BT_GA_TBS_3G_TECHNOLOGY	  = 0x01, /**< bearer supports 3G technology .*/
	 WICED_BT_GA_TBS_4G_TECHNOLOGY	  = 0x02, /**< bearer supports 4G technology. */
	 WICED_BT_GA_TBS_LTE_TECHNOLOGY   = 0x03, /**< bearer supports LTE technology. */
	 WICED_BT_GA_TBS_WIFI_TECHNOLOGY  = 0x04, /**< bearer supports WIFI technology. */
	 WICED_BT_GA_TBS_5G_TECHNOLOGY	  = 0x05, /**< bearer supports 5G technology. */
	 WICED_BT_GA_TBS_GSM_TECHNOLOGY   = 0x06, /**< bearer supports GSM technology. */
	 WICED_BT_GA_TBS_CDMA_TECHNOLOGY  = 0x07, /**< bearer supports CDMA technology. */
	 WICED_BT_GA_TBS_2G_TECHNOLOGY	  = 0x08, /**< bearer supports 2G technology. */
	 WICED_BT_GA_TBS_WCDMA_TECHNOLOGY = 0x09, /**< bearer supports WCDMA technology. */
	 WICED_BT_GA_TBS_IP_TECHNOLOGY	  = 0x0A, /**< bearer supports IP technology. */
 };
typedef uint8_t wiced_bt_ga_tbs_bearer_technology_t; /**< TBS bearer technology (see #wiced_bt_ga_tbs_bearer_technology_e) */

 /** All possible call state */
enum wiced_bt_ga_tbs_call_state_e
 {
	 WICED_BT_GA_TBS_CALL_STATE_INCOMING				  = 0x00,  /**< A remote party is calling (incoming call). */
	 WICED_BT_GA_TBS_CALL_STATE_DIALING 				  = 0x01,  /**< The process to call the remote party has started but the remote party is not being alerted (outgoing call).*/
	 WICED_BT_GA_TBS_CALL_STATE_ALERTING				  = 0X02,  /**< A remote party is being alerted (outgoing call). */
	 WICED_BT_GA_TBS_CALL_STATE_ACTIVE					  = 0x03,  /**< The call is in an active conversation */
	 WICED_BT_GA_TBS_CALL_STATE_LOCALLY_HELD			  = 0x04,  /**< The call is connected but held locally with no audio communicated in either direction. Either server/client can control the state */
	 WICED_BT_GA_TBS_CALL_STATE_REMOTELY_HELD			  = 0x05,  /**< The call is connected but held remotely with no audio communicated in either direction. Call state can be controlled by the remote party of the call. */
	 WICED_BT_GA_TBS_CALL_STATE_LOCALLY_AND_REMOTELY_HELD = 0x06,  /**< The call is connected but held both locally and remotely with no audio communicated in either direction */
 };

 typedef uint8_t wiced_bt_ga_tbs_call_state_t; /**< TBS call state (see #wiced_bt_ga_tbs_call_state_e) */

 /** call action notification event after a call action is performed */
 enum wiced_bt_ga_tbs_call_operation_result_e
 {
	 WICED_BT_CALL_SUCCESS								   = 0x00, /**< Opcode write was successful.*/
	 WICED_BT_CALL_OPCODE_NOT_SUPPORTED 				   = 0x01, /**< An invalid opcode was used for the Call Control Point write.*/
	 WICED_BT_CALL_OPERATION_NOT_POSSIBLE				   = 0x02, /**< The tbs server does not currently support joining of calls.*/
	 WICED_BT_CALL_INVALID_CALL_ID						   = 0x03, /**< The Call Control Point write is invalid.*/
	 WICED_BT_CALL_STATE_MISMATCH						   = 0x04, /**< Opcode written in an unexpected call state */
	 WICED_BT_CALL_LACK_OF_RESOURCES					   = 0x05, /**< resources to complete the requested action */
	 WICED_BT_CALL_INVALID_URI							   = 0x06  /**< outgoing caller id is incorrect */
 } ;
 typedef uint8_t wiced_bt_ga_tbs_call_operation_result_t; /**< see #wiced_bt_ga_tbs_call_operation_result_e */

 /** Call action values */
 enum wiced_bt_ga_tbs_call_action_e
 {
	 WICED_BT_GA_CCP_ACTION_ACCEPT_CALL 	= 0x00, /**< Answer the incoming call. */
	 WICED_BT_GA_CCP_ACTION_TERMINATE_CALL	= 0x01, /**< End the currently active/outgoing/held call. */
	 WICED_BT_GA_CCP_ACTION_HOLD_CALL		= 0x02, /**< Place the currently active or alerting call on local hold. */
	 WICED_BT_GA_CCP_ACTION_RETRIEVE_CALL	= 0x03, /**< Move a locally held call to an active call. Move a locally and remotely held call to a remotely held call */
	 WICED_BT_GA_CCP_ACTION_ORIGINATE		= 0x04, /**< Place a call */
	 WICED_BT_GA_CCP_ACTION_JOIN_CALL		= 0x05, /**< Put a call to the active state and join all calls that are in the active state. */
 };

typedef uint8_t wiced_bt_ga_tbs_call_action_t; /**< TBS call action (see #wiced_bt_ga_tbs_call_action_e) */

 /** incoming remote caller ID */
 typedef struct
 {
	 uint8_t				 call_id; /**< Call id of the call*/
	 wiced_bt_ga_string_t	 name;	  /**< remote caller id consisting of uri prefix and caller id*/
 } wiced_bt_ga_tbs_user_friendly_call_data_t;

 /** incoming call data */
 typedef struct
 {
     uint8_t call_id;          /**< Call id of the call*/
     wiced_bt_ga_string_t URI; /**< URI consists of URI scheme followed by Caller ID*/
 } wiced_bt_ga_tbs_incoming_call_t;

 /** incoming  call target caller ID at the bearer */
 typedef struct
 {
     uint8_t call_id;                       /**< Call id of the call*/
     wiced_bt_ga_string_t target_caller_id; /**< remote caller id consisting of uri prefix and caller id*/
 } wiced_bt_ga_tbs_incoming_tg_bearer_uri_t;

 /** call state data */
 typedef struct
 {
     uint8_t call_id;                         /**< call id of the call */
     wiced_bt_ga_tbs_call_state_t call_state; /**< call state of the call */
     uint8_t call_flags;                      /**< call flag bit field of the call */
 } wiced_bt_ga_tbs_call_state_data_t;

 /** call state list*/
 typedef struct
 {
	 uint8_t						   num_calls;				  /**< number of calls in the call list*/
	 wiced_bt_ga_tbs_call_state_data_t call_state[WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT]; /**< call state of each call */
 } wiced_bt_ga_tbs_call_state_list_t;

 /** termination reason */
 typedef struct
 {
	 uint8_t							call_id;					/**< call id to be terminated */
	 wiced_bt_ga_tbs_call_termination_reason_t termination_reason;		   /**< call termination reason */
 } wiced_bt_ga_tbs_call_termination_reason_data_t;

 /** call operation response data after call action is performed */
 typedef struct
 {
	 uint8_t call_id;												  /**< call id of the call*/
	 uint8_t requested_opcode;										  /**< callopertaion to be performed(accept,join...) */
	 wiced_bt_ga_tbs_call_operation_result_t call_operation_result; 	 /**< result of the call operation performde */
 } wiced_bt_ga_tbs_call_operation_response_t;

 /** current call data */
 typedef struct
 {
	 uint8_t					   call_id;    /**< call id of the call */
	 wiced_bt_ga_tbs_call_state_t  call_state; /**< call state of the call */
	 uint8_t					   call_flags;	  /**< call flags */
	 wiced_bt_ga_string_t		   remote_caller_id; /**< caller id of the remote user */
 } wiced_bt_ga_tbs_current_call_t;

 /** list of current call at the bearer */
 typedef struct
 {
	 uint8_t						num_calls;	   /**< number of calls in the list */
	 wiced_bt_ga_tbs_current_call_t call_data[WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT]; /**< current call list */
 } wiced_bt_ga_tbs_current_call_list_t;

 /** Join call param */
 typedef struct
 {
	 uint8_t				num_call_ids;	 /**< call id of the call */
	 uint8_t				call_ids[WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT]; /**< call ids*/
 } wiced_bt_ga_tbs_join_call_data_t;

 /** list of bearer properties that is read from the app */
typedef enum
{
    TBS_BEARER_PROVIDER_NAME,                         /**< telephone bearer provider name value */
    TBS_BEARER_UCI,                                   /**< telephone bearer supported uci */
    TBS_BEARER_TECHNOLOGY,                            /**< telephone bearer supported technology */
    BEARER_URI_PREFIX_SUPPORTED_LIST,                 /**< telephone bearer supported uri*/
    TBS_BEARER_SIGNAL_STRENGTH,                       /**< telephone bearer signal strength*/
    TBS_BEARER_SIGNAL_STRENGTH_REPORTING_INTERVAL,    /**< telephone bearer signal strength reporting interval*/
} wiced_bt_ga_ccp_bearer_property_t;


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
 * @brief Initialize the GTBS service/profile
 *
 * @param[in] num_instances : Number of instances
 * @param[in] pv_ini : expected to be NULL
 * @return result of the init operation
 */
wiced_result_t wiced_bt_ga_gtbs_init(uint8_t num_instances, void *pv_ini);

/**
 * @brief Initialize the TBS service/profile
 *
 * @param[in] num_instances : Number of instances
 * @param[in] pv_ini : expected to be NULL
 * @return result of the init operation
 */
wiced_result_t wiced_bt_ga_tbs_init(uint8_t num_instances, void *pv_ini);

/** Enable TBS server module */
void wiced_bt_ga_tbs_enable_server(void);

/** Enable TBS client module */
void wiced_bt_ga_tbs_enable_client(void);

/** Enable GTBS server module */
void wiced_bt_ga_gtbs_enable_server(void);

/** Enable GTBS client module */
void wiced_bt_ga_gtbs_enable_client(void);

/**@} Telephone_Bearer_Service_APIs */
#ifdef __cplusplus
}
#endif

#endif //__WICED_BT_GA_TBS_H__
