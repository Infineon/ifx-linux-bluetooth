/*
 * $ Copyright Cypress Semiconductor $
 */

/** @file
 *
 * Audio Stream Control Service (ASCS) Application Programming Interface
 */


#ifndef __WICED_BT_GA_ASCS_H__
#define __WICED_BT_GA_ASCS_H__

#include "wiced_bt_ga_bap.h"
#include "wiced_bt_ga_common.h"

/**
 * @addtogroup Stream_Control_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_ascs
 * @{
 * @brief
 ASCS can be instantiated on devices that can accept the establishment of unicast Audio Streams. Examples of such devices are speakers, headsets, hearing aids, earbuds, and wireless microphones.
	 - Two types of ASEs :
		  - Sink ASE characteristics represent Sink ASEs, to which audio data can flow. The server is said to act as Audio Sink for that ASE. There can be more than one Sink ASE characteristic on the server.
		  - Source ASE characteristics represent Source ASEs, from which audio data can flow. The server is said to act as Audio Source for that ASE. There can be more than one Source ASE characteristic on the server.
 */


#define SDU_INTERVAL_LENGTH 3 /**< Length in bytes for SDU interval */
#define PRESENTATION_DELAY_LENGTH 3 /**< Length in bytes for Presentation Delay */

/**
 * @brief defines the response code that shall be used when the server rejects or cannot successfully complete
 * a client-initiated ASE Control operation.
 */
typedef enum
{
    WICED_BT_GA_ASCS_RESPONSE_SUCCESS, /**< The server has successfully completed the client-initiated ASE Control operation */
    WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_OPCODE, /**< The server does not support the client-initiated ASE Control operation defined by the opcode written by the client */
    WICED_BT_GA_ASCS_RESPONSE_INVALID_LENGTH, /**< The server has detected a truncated operation written by the client */
    WICED_BT_GA_ASCS_RESPONSE_INVALID_ASE_ID, /**< The server has detected that the ASE_ID written by the client does not match an ASE_ID in an exposed ASE characteristic value for that client */
    WICED_BT_GA_ASCS_RESPONSE_INVALID_ASE_STATE_MACHINE_TRANSITION, /**< The server has detected that the client-initiated ASE Control operation would cause an invalid ASE state machine transition */
    WICED_BT_GA_ASCS_RESPONSE_INVALID_ASE_DIRECTION, /**< The server has detected that the client-initiated ASE Control operation would cause an invalid ASE state machine transition */
    WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_AUDIO_CAPABILITIES, /**< The server has detected that the audio capabilities requested during a Config Codec operation are not supported (i.e., the server has not exposed the requested configuration in any PAC record) */
    WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE, /**< The server has detected it does not support one or more parameter values written by the client */
    WICED_BT_GA_ASCS_RESPONSE_REJECTED_CONFIGURATION_PARAMETER_VALUE, /**< The server has rejected one or more parameter values written by the client */
    WICED_BT_GA_ASCS_RESPONSE_INVALID_CONFIGURATION_PARAMETER_VALUE, /**< The server has detected one or more invalid parameter values written by the client */
    WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_METADATA, /**< The server has detected an unsupported Metadata Type written by the profile. */
    WICED_BT_GA_ASCS_RESPONSE_REJECTED_METADATA, /**< The server has rejected an unsupported Metadata Type written by the client */
    WICED_BT_GA_ASCS_RESPONSE_INVALID_METADATA, /**< This Response_Code is used to inform the client that the Metadata Value is incorrectly formatted. */
    WICED_BT_GA_ASCS_RESPONSE_INSUFFICIENT_RESOURCES, /**< The server is unable to successfully complete the client-initiated ASE Control operation because of insufficient resources */
    WICED_BT_GA_ASCS_RESPONSE_UNSPECIFIED_ERROR,      /**< The server has encountered an unspecified error */
} wiced_bt_ga_ascs_cp_response_code_t;

/**
 * @brief defines the reason values that shall be used when the server rejects or cannot successfully complete a
 * client-initiated ASE Control operation.
 */
typedef enum
{
    WICED_BT_GA_ASCS_REASON_NOT_APPLICABLE, /**< Reason value is not applicable */
    WICED_BT_GA_ASCS_REASON_CODEC_ID, /**< The server has detected that the codec ID value used in configuration parameters by the client is invalid */
    WICED_BT_GA_ASCS_REASON_CODEC_SPECIFIC_CONFIGURATION, /**< The server has detected that the codec specific configuration value used by the client is invalid */
    WICED_BT_GA_ASCS_REASON_SDU_INTERVAL,/**< The server has detected that the SDU interval used in configuration parameters by the client is out of range */
    WICED_BT_GA_ASCS_REASON_FRAMING,/**< The server has detected that the framing value used in configuration parameters by the client is invalid */
    WICED_BT_GA_ASCS_REASON_PHY,/**< The server has detected that the phy value used in configuration parameters by the client is invalid */
    WICED_BT_GA_ASCS_REASON_MAXIMUM_SDU_SIZE,/**< The server has detected that the maximum sdu value used in configuration parameters by the client is invalid */
    WICED_BT_GA_ASCS_REASON_RETRANSMISSION_NUMBER,/**< The server has detected that the retransmission number used in configuration parameters by the client is invalid */
    WICED_BT_GA_ASCS_REASON_MAX_TRANSPORT_LATENCY,/**< The server has detected that the transport latency used in configuration parameters by the client is invalid */
    WICED_BT_GA_ASCS_REASON_PRESENTATION_DELAY,/**< The server has detected that the presentation delay used in configuration parameters by the client is invalid */
    WICED_BT_GA_ASCS_REASON_INVALID_ASE_CIS_MAPPING,/**< The server has detected that the cis value used in configuration parameters by the client is invalid */
} wiced_bt_ga_ascs_cp_err_reason_t;

/**
 * @brief list of opcodes supported by ASCS
 */
enum wiced_bt_ga_ascs_opcode_e
{
    WICED_BT_GA_ASCS_OPCODE_INVALID = 0, /**< Invalid opcode */
    WICED_BT_GA_ASCS_OPCODE_CONFIG_CODEC = 1, /**< Configures codec parameters for one or more ASEs.*/
    WICED_BT_GA_ASCS_OPCODE_CONFIG_QOS, /**< Configures preferred CIS parameters for one or more ASEs.*/
    WICED_BT_GA_ASCS_OPCODE_ENABLE, /**< Applies codec parameters and preferred CIS parameters, applies any Metadata, and starts coupling an ASE to a CIS for one or more ASEs.*/
    WICED_BT_GA_ASCS_OPCODE_RECEIVER_START_READY, /**< Signals that the Audio Sink is ready to receive audio data transmitted by the Audio Source, and completes coupling an ASE to a CIS*/
    WICED_BT_GA_ASCS_OPCODE_DISABLE, /**< Starts decoupling a Source ASE from a CIS for one or more Source ASEs.*/
    WICED_BT_GA_ASCS_OPCODE_RECEIVER_STOP_READY, /**< Signals that the Audio Sink is ready to stop receiving audio data transmitted by the Audio Source, and completes decoupling a Source ASE from a CIS.*/
    WICED_BT_GA_ASCS_OPCODE_UPDATE_METADATA, /**< Updates Metadata for one or more ASEs */
    WICED_BT_GA_ASCS_OPCODE_RELEASE,/**< Releases resources associated with an ASE */
    WICED_BT_GA_ASCS_OPCODE_RELEASED,/**< Transitions an ASE from Releasing state to the Idle state or the Codec Configured state*/
    WICED_BT_GA_ASCS_OPCODE_MAX, /**< Max opcode, not applicable*/
};

typedef uint8_t wiced_bt_ga_ascs_opcode_t;  /**< ASCS Opcode (see #wiced_bt_ga_ascs_opcode_e) */

/**
 * @brief list of states supported by the ASE
 */
enum wiced_bt_ga_ascs_state_e
{
    WICED_BT_GA_ASCS_STATE_IDLE, /**< ASE state is idle  */
    WICED_BT_GA_ASCS_STATE_CODEC_CONFIGURED, /**< ASE state is codec configured  */
    WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED,/**< ASE state is qos configured  */
    WICED_BT_GA_ASCS_STATE_ENABLING, /**< ASE state is in enabling */
    WICED_BT_GA_ASCS_STATE_STREAMING,/**< ASE state is in streaming state */
    WICED_BT_GA_ASCS_STATE_DISABLING,/**< ASE state is in disabling */
    WICED_BT_GA_ASCS_STATE_RELEASING,/**< ASE state is in releasing */
    WICED_BT_GA_ASCS_STATE_MAX,/**< Invalid ASE state */
} ;

typedef uint8_t wiced_bt_ga_ascs_state_t;  /**< ASCS Opcode (see #wiced_bt_ga_ascs_state_e) */


/**
 * @brief list of framing values used by ASCS
 */
enum wiced_bt_ga_ascs_framing_e
{
    WICED_BT_ASCS_UNFRAMED, /**< Unframed ISOAL PDUs preferred */
    WICED_BT_ASCS_FRAMED,   /**< Framed ISOAL PDUs preferred */
    WICED_BT_ASCS_INVALID_FRAMING /**< Invalid value for Framing */
};

typedef uint8_t wiced_bt_ga_ascs_framing_t; /**< ASCS Framing values (see #wiced_bt_ga_ascs_framing_e) */


/**
 * @brief list of PHY values used by ASCS
 */
enum wiced_bt_ga_ascs_phy_e
{
    WICED_BT_ASCS_PHY_1M = 1, /**< LE 1M PHY preferred */
    WICED_BT_ASCS_PHY_2M,     /**< LE 2M PHY preferred */
    WICED_BT_ASCS_PHY_CODED,  /**< LE Coded PHY preferred */
    WICED_BT_ASCS_INVALID_PHY /**< Invalid PHY value */
};

typedef uint8_t wiced_bt_ga_ascs_phy_t; /**< ASCS Phy values (see #wiced_bt_ga_ascs_phy_e) */

/**
 * @brief list of announcement type values used by ASCS
 */

enum wiced_bt_ga_ascs_announcement_type_e
{
    WICED_BT_ASCS_GENERAL_ANNOUNCEMENT, /**< General Announcement Type */
    WICED_BT_ASCS_TARGETED_ANNOUNCEMENT, /**< Targetted Announcement Type */
};

typedef uint8_t wiced_bt_ga_ascs_announcement_type_t; /**< ASCS Announcement values (see #wiced_bt_ga_ascs_announcement_type_e) */

#define WICED_BT_ASCS_VALID_PHY_MASK ~(WICED_BT_ASCS_PHY_1M | WICED_BT_ASCS_PHY_2M | WICED_BT_ASCS_PHY_CODED) /**< Valid Phy Mask */

/**
 * @brief defines the data received on config codec event (when peer initiates config codec operation)
 */
typedef struct
{
    wiced_bt_ga_bap_codec_id_t codec_id; /**< Codec ID to be used for condif codec  */
    uint8_t target_latency; /**< Provides context for the server to return meaningful values for QoS preferences in Codec Configured state*/
    uint8_t target_phy; /**< PHY parameter target to achieve the Target_Latency value  */
    wiced_bt_ga_bap_csc_t csc; /**< Codec specific configuration value */
} wiced_bt_ga_ascs_config_codec_args_t;

/**
 * @brief defines the QoS info
 *      1) received through config QoS command
 *      2) to be exposed to peer upon accepting config qos from client
 */
typedef struct
{
    uint8_t cig_id;                 /**< CIG ID set by the central */
    uint8_t cis_id;                 /**< CIS ID set by the central */
    uint32_t sdu_interval;          /**< SDU_Interval written by the client*/
    uint8_t framing;                /**< Framing written by the client*/
    uint8_t phy;                    /**< PHY written by the client*/
    uint16_t max_sdu;               /**< Max_SDU written by the client*/
    uint8_t retransmission_number;  /**< Retransmission_Number written by the client*/
    uint16_t max_transport_latency; /**< Max_Transport_Latency written by the client*/
    uint32_t presentation_delay;    /**< Presentation_Delay written by the client*/
} wiced_bt_ga_ascs_config_qos_args_t;

/**
 * @brief ASCS CP parameters
 */

typedef struct
{
    uint8_t ase_id; /**< ASE Id to be used for this control point operation*/
    union
    {
        wiced_bt_ga_ascs_config_codec_args_t config_codec_params; /**< To be used incase of Config Codec Operations */
        wiced_bt_ga_ascs_config_qos_args_t config_qos_params;/**< To be used incase of Qos Config Operations */
        wiced_bt_ga_bap_metadata_t metadata; /**< To be used incase of Metadata related Operations */
    };
} wiced_bt_ga_ascs_cp_params_t;

/**
 * @brief ASCS CP command
 */
typedef struct
{
    wiced_bt_ga_ascs_opcode_t opcode; /**< Opcode for the CP command */
    uint8_t num_of_ase;               /**< Total number of ASE to be considered for the operation */
    wiced_bt_ga_ascs_cp_params_t *p_cp_params; /**< Control point operation parameters */
} wiced_bt_ga_ascs_cp_cmd_t;

/**
 * @brief defines the information to be provided by the application when rejecting a request from client
 */
typedef struct
{
    uint8_t ase_id; /**< ASE id of the status */
    wiced_bt_ga_ascs_cp_response_code_t response_code; /**< Response code of the status */
    wiced_bt_ga_ascs_cp_err_reason_t reason; /**< Error reason if any */
} wiced_bt_ga_ascs_cp_cmd_sts_t;

/**
 * @brief ASCS CP notification
 */
typedef struct
{
    wiced_bt_ga_ascs_opcode_t opcode; /**< Opcode of control point notification */
    uint8_t num_of_ase; /**< Number of ASE considered in this notification */
    wiced_bt_ga_ascs_cp_cmd_sts_t *p_status; /**< Status of the control point operation */
} wiced_bt_ga_ascs_cp_notif_t;

/**
 * @brief ASE Preference values
 */

typedef struct
{
    wiced_bt_ga_ascs_framing_t framing;              /**< Server preferred value for the Framing */
    wiced_bt_ga_ascs_phy_t preferred_phy;            /**< Server preferred value for the PHY */
    uint8_t  preferred_retransmission_number;        /**< Server preferred value for the Retransmission_Number */
    uint16_t max_transport_latency;                  /**< Server preferred value for the Max_Transport_Latency */
    uint32_t presentation_delay_in_us_min;           /**< Minimum server supported Presentation_Delay */
    uint32_t presentation_delay_in_us_max;           /**< Maximum server supported Presentation_Delay */
    uint32_t preferred_presentation_delay_in_us_min; /**< Minimum server supported Presentation_Delay */
    uint32_t preferred_presentation_delay_in_us_max; /**< Maximum server supported Presentation_Delay */
} wiced_bt_ga_ascs_ase_preferences_t;

/**
 * @brief ASE Info data structure
 */

typedef struct
{
    uint8_t ase_id;                               /**< Unique identifier for the ASE */
    uint8_t ase_type;                             /**< ASE type : Source or Sink */
    wiced_ble_isoc_data_path_direction_t data_path_dir; /**< ASE data direction, Host to Controller(0),  Controller to Host(1) */
    wiced_bt_ga_ascs_ase_preferences_t ascs_data; /**< ASE configuration */
} wiced_bt_ga_ascs_ase_info_t;

/**
 * @brief ASE data structure
 */
typedef struct
{
    wiced_bt_ga_ascs_ase_info_t *p_ase_info;         /**< Server ASE info */
    wiced_bt_ga_ascs_state_t ase_state;                    /**< State of the ase for the client */
    wiced_bt_ga_ascs_config_codec_args_t codec_configured; /**< Client configured Codec params  */
    wiced_bt_ga_ascs_config_qos_args_t qos_configured;     /**< Client configured QOS params    */
    wiced_bt_ga_bap_metadata_t metadata;                   /**< Metadata information for the ASE */
} wiced_bt_ga_ascs_ase_t;

/** ASCS init data for service object */
typedef struct
{
    uint8_t max_snk_ase_spt; /**< MAX Number of Sink ASEs supported for ASCS service instance */
    uint8_t max_src_ase_spt; /**< MAX Number of Source ASEs supported for ASCS service instance */
} wiced_bt_ga_ascs_init_data_t;

/**
 * @brief Initialize the ASCS service/profile
 * @param[in] num_instances: Number of ASCS instances to be created.
 * @param[in] pv_ini: see \ref wiced_bt_ga_ascs_init_data_t
 *
 */
wiced_result_t wiced_bt_ga_ascs_init(uint8_t num_instances, wiced_bt_ga_ascs_init_data_t *pv_ini);

/**
 * @brief Enable ASCS server module
 */
void wiced_bt_ga_ascs_enable_server(void);
/**
 * @brief Enable ASCS client module
 */
void wiced_bt_ga_ascs_enable_client(void);


/**
 * @brief Parse ASCS data on write from peer
 *
 * @param[in] opcode :  opcode for the control point operation
 * @param[in] data_stream :  data stream to be parsed
 * @param[in] length :  length of the data stream
 * @param[out] parsed_data :  parsed data from the data stream
 * @param[out] status : status of the ase data parsing
 */
int wiced_bt_ga_ascs_parse_data(wiced_bt_ga_ascs_opcode_t opcode,
                                uint8_t *data_stream,
                                int length,
                                wiced_bt_ga_ascs_cp_params_t *parsed_data,
                                wiced_bt_ga_ascs_cp_cmd_sts_t *status);


/**
 * @brief Get the ASCS opcode and number of ASE from the stream
 *
 * @param[in] data :  data stream to be parsed
 * @param[out] opcode :  opcode for the control point operation
 * @param[out] num_of_ase :  number of ASE in the control point operation
 */

int wiced_bt_ga_ascs_get_cp_header(uint8_t *data, uint8_t *opcode, uint8_t *num_of_ase);

/**
 * @brief Let the peer know that the sink is ready to start/stop receive the stream
 *
 * @param[in] conn_id : GATT Connection ID
 * @param[in] p_service : ASCS service object
 * @param[in] ase_id : ASE ID of the operation
 * @param[in] is_start_ready : start or stop receiving the stream


 */
void wiced_bt_ga_ascs_send_receiver_start_stop_ready(uint16_t conn_id,
                                                     gatt_intf_service_object_t *p_service,
                                                     uint8_t ase_id,
                                                     wiced_bool_t is_start_ready);

extern const char *ascs_opcode_str[]; /**< ASCS Opcode String */
extern const char *ascs_state_str[];  /**< ASCS State String */

/**@} wiced_bt_ga_ascs */
/**@} Stream_Control_APIs */

#endif //__WICED_BT_GA_ASCS_H__
