/*
 * $ Copyright Cypress Semiconductor $
 */

#pragma once

#include "wiced_bt_ga_bap.h"
#include "wiced_bt_ga_common.h"

#define SDU_INTERVAL_LENGTH 3
#define PRESENTATION_DELAY_LENGTH 3

#define ASCS_TRACE(...)
#define ASCS_TRACE_CRIT(...)

/**
 * @brief defines the response code that shall be used when the server rejects or cannot successfully complete
 * a client-initiated ASE Control operation.
 */
typedef enum
{
    WICED_BT_GA_ASCS_RESPONSE_SUCCESS, /**!< The server has successfully completed the client-initiated ASE Control operation */
    WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_OPCODE, /**!< The server does not support the client-initiated ASE Control operation defined by the opcode written by the client */
    WICED_BT_GA_ASCS_RESPONSE_INVALID_LENGTH, /**!< The server has detected a truncated operation written by the client */
    WICED_BT_GA_ASCS_RESPONSE_INVALID_ASE_ID, /**!< The server has detected that the ASE_ID written by the client does not match an ASE_ID in an exposed ASE characteristic value for that client */
    WICED_BT_GA_ASCS_RESPONSE_INVALID_ASE_STATE_MACHINE_TRANSITION, /**!< The server has detected that the client-initiated ASE Control operation would cause an invalid ASE state machine transition */
    WICED_BT_GA_ASCS_RESPONSE_INVALID_ASE_DIRECTION, /**!< The server has detected that the client-initiated ASE Control operation would cause an invalid ASE state machine transition */
    WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_AUDIO_CAPABILITIES, /**!< The server has detected that the audio capabilities requested during a Config Codec operation are not supported (i.e., the server has not exposed the requested configuration in any PAC record) */
    WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE, /**!< The server has detected it does not support one or more parameter values written by the client */
    WICED_BT_GA_ASCS_RESPONSE_REJECTED_CONFIGURATION_PARAMETER_VALUE, /**!< The server has rejected one or more parameter values written by the client */
    WICED_BT_GA_ASCS_RESPONSE_INVALID_CONFIGURATION_PARAMETER_VALUE, /**!< The server has detected one or more invalid parameter values written by the client */
    WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_METADATA, /**!< The server has detected an unsupported Metadata Type written by the profile. */
    WICED_BT_GA_ASCS_RESPONSE_REJECTED_METADATA, /**!< The server has rejected an unsupported Metadata Type written by the client */
    WICED_BT_GA_ASCS_RESPONSE_INVALID_METADATA, /**!< This Response_Code is used to inform the client that the Metadata Value is incorrectly formatted. */
    WICED_BT_GA_ASCS_RESPONSE_INSUFFICIENT_RESOURCES, /**!< The server is unable to successfully complete the client-initiated ASE Control operation because of insufficient resources */
    WICED_BT_GA_ASCS_RESPONSE_UNSPECIFIED_ERROR,      /**!< The server has encountered an unspecified error */
} wiced_bt_ga_ascs_cp_response_code_t;

/**
 * @brief defines the reason values that shall be used when the server rejects or cannot successfully complete a
 * client-initiated ASE Control operation.
 */
typedef enum
{
    WICED_BT_GA_ASCS_REASON_NOT_APPLICABLE,
    WICED_BT_GA_ASCS_REASON_CODEC_ID,
    WICED_BT_GA_ASCS_REASON_CODEC_SPECIFIC_CONFIGURATION,
    WICED_BT_GA_ASCS_REASON_SDU_INTERVAL,
    WICED_BT_GA_ASCS_REASON_FRAMING,
    WICED_BT_GA_ASCS_REASON_PHY,
    WICED_BT_GA_ASCS_REASON_MAXIMUM_SDU_SIZE,
    WICED_BT_GA_ASCS_REASON_RETRANSMISSION_NUMBER,
    WICED_BT_GA_ASCS_REASON_MAX_TRANSPORT_LATENCY,
    WICED_BT_GA_ASCS_REASON_PRESENTATION_DELAY,
    WICED_BT_GA_ASCS_REASON_INVALID_ASE_CIS_MAPPING,
} wiced_bt_ga_ascs_cp_err_reason_t;

/**
 * @brief list of opcodes supported by ASCS
 */
typedef enum
{
    WICED_BT_GA_ASCS_OPCODE_INVALID = 0,
    WICED_BT_GA_ASCS_OPCODE_CONFIG_CODEC = 1,
    WICED_BT_GA_ASCS_OPCODE_CONFIG_QOS,
    WICED_BT_GA_ASCS_OPCODE_ENABLE,
    WICED_BT_GA_ASCS_OPCODE_RECEIVER_START_READY,
    WICED_BT_GA_ASCS_OPCODE_DISABLE,
    WICED_BT_GA_ASCS_OPCODE_RECEIVER_STOP_READY,
    WICED_BT_GA_ASCS_OPCODE_UPDATE_METADATA,
    WICED_BT_GA_ASCS_OPCODE_RELEASE,
    WICED_BT_GA_ASCS_OPCODE_RELEASED,
    WICED_BT_GA_ASCS_OPCODE_MAX,
} wiced_bt_ga_ascs_opcode_t;

/**
 * @brief list of states supported by the ASE
 */
typedef enum
{
    WICED_BT_GA_ASCS_STATE_IDLE,
    WICED_BT_GA_ASCS_STATE_CODEC_CONFIGURED,
    WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED,
    WICED_BT_GA_ASCS_STATE_ENABLING,
    WICED_BT_GA_ASCS_STATE_STREAMING,
    WICED_BT_GA_ASCS_STATE_DISABLING,
    WICED_BT_GA_ASCS_STATE_RELEASING,
    WICED_BT_GA_ASCS_STATE_MAX,
} wiced_bt_ga_ascs_state_t;

typedef enum
{
    WICED_BT_ASCS_UNFRAMED, /**< Unframed ISOAL PDUs preferred */
    WICED_BT_ASCS_FRAMED,   /**< Framed ISOAL PDUs preferred */
    WICED_BT_ASCS_INVALID_FRAMING
} wiced_bt_ga_ascs_framing_t;

typedef enum
{
    WICED_BT_ASCS_PHY_1M = 1,
    WICED_BT_ASCS_PHY_2M,
    WICED_BT_ASCS_PHY_CODED,
    WICED_BT_ASCS_INVALID_PHY
} wiced_bt_ga_ascs_phy_t;

typedef enum
{
    WICED_BT_ASCS_GENERAL_ANNOUNCEMENT,
    WICED_BT_ASCS_TARGETED_ANNOUNCEMENT,
} wiced_bt_ga_ascs_announcement_type_t;

#define WICED_BT_ASCS_PREFEREED_PHY_1M 0x01
#define WICED_BT_ASCS_PREFEREED_PHY_2M 0x02
#define WICED_BT_ASCS_PREFEREED_PHY_CODED 0x04
#define WICED_BT_ASCS_VALID_PHY_MASK ~(WICED_BT_ASCS_PHY_1M | WICED_BT_ASCS_PHY_2M | WICED_BT_ASCS_PHY_CODED)

/**
 * @brief defines the data received on config codec event (when peer initiates config codec operation)
 */
typedef struct
{
    wiced_bt_ga_bap_codec_id_t codec_id;
    uint8_t target_latency;
    uint8_t target_phy;
    wiced_bt_ga_bap_csc_t csc;
} wiced_bt_ga_ascs_config_codec_args_t;

/**
 * @brief defines the QoS info
 *      1) received through config QoS command
 *      2) to be exposed to peer upon accepting config qos from client
 */
typedef struct
{
    uint8_t cig_id;                 /*!< CIG_ID written by the client*/
    uint8_t cis_id;                 /*!< CIS_ID written by the client*/
    uint32_t sdu_interval;          /*!< SDU_Interval written by the client*/
    uint8_t framing;                /*!< Framing written by the client*/
    uint8_t phy;                    /*!< PHY written by the client*/
    uint16_t max_sdu;               /*!< Max_SDU written by the client*/
    uint8_t retransmission_number;  /*!< Retransmission_Number written by the client*/
    uint16_t max_transport_latency; /*!< Max_Transport_Latency written by the client*/
    uint32_t presentation_delay;    /*!< Presentation_Delay written by the client*/
    uint16_t gatt_conn_id;
} wiced_bt_ga_ascs_config_qos_args_t;

typedef struct
{
    uint8_t ase_id;
    union
    {
        wiced_bt_ga_ascs_config_codec_args_t config_codec_params;
        wiced_bt_ga_ascs_config_qos_args_t config_qos_params;
        wiced_bt_ga_bap_metadata_t metadata;
    };
} wiced_bt_ga_ascs_cp_params_t;

/**
 * @brief ASCS CP command
 */
typedef struct
{
    wiced_bt_ga_ascs_opcode_t opcode;
    uint8_t num_of_ase;
    wiced_bt_ga_ascs_cp_params_t *p_cp_params;
} wiced_bt_ga_ascs_cp_cmd_t;

/**
 * @brief defines the information to be provided by the application when rejecting a request from client
 */
typedef struct
{
    uint8_t ase_id;
    wiced_bt_ga_ascs_cp_response_code_t response_code;
    wiced_bt_ga_ascs_cp_err_reason_t reason;
} wiced_bt_ga_ascs_cp_cmd_sts_t;

/**
 * @brief ASCS CP notification
 */
typedef struct
{
    wiced_bt_ga_ascs_opcode_t opcode;
    uint8_t num_of_ase;
    wiced_bt_ga_ascs_cp_cmd_sts_t *p_status;
} wiced_bt_ga_ascs_cp_notif_t;

typedef struct
{
    wiced_bt_ga_ascs_framing_t framing;              /*!< Server preferred value for the Framing */
    wiced_bt_ga_ascs_phy_t preferred_phy;            /*!< Server preferred value for the PHY */
    uint8_t  preferred_retransmission_number;        /*!< Server preferred value for the Retransmission_Number */
    uint16_t max_transport_latency;                  /*!< Server preferred value for the Max_Transport_Latency */
    uint32_t presentation_delay_in_us_min;           /*!< Minimum server supported Presentation_Delay */
    uint32_t presentation_delay_in_us_max;           /*!< Maximum server supported Presentation_Delay */
    uint32_t preferred_presentation_delay_in_us_min; /*!< Minimum server supported Presentation_Delay */
    uint32_t preferred_presentation_delay_in_us_max; /*!< Maximum server supported Presentation_Delay */
} wiced_bt_ga_ascs_ase_preferences_t;

typedef struct
{
    uint8_t ase_id;                               /*!< Unique identifier for the ASE */
    uint8_t ase_type;                             /*!< ASE type : Source or Sink */
    wiced_bt_ga_ascs_ase_preferences_t ascs_data; /*!< ASE configuration */
} wiced_bt_ga_ascs_ase_info_t;

typedef struct
{
    const wiced_bt_ga_ascs_ase_info_t *p_ase_info;         /*!< Server ASE info */
    uint8_t ase_state;                                     /*!< State of the ase for the client */
    wiced_bt_ga_ascs_config_codec_args_t codec_configured; /*!< Client configured Codec params  */
    wiced_bt_ga_ascs_config_qos_args_t qos_configured;     /*!< Client configured QOS params    */
    wiced_bt_ga_bap_metadata_t metadata;
} wiced_bt_ga_ascs_ase_t;

// Generic ASCS API (API provided by all the GA profiles)
wiced_result_t wiced_bt_ga_ascs_init(ga_cfg_t *p_cfg);



// Non Generic ASCS API (API specific to ASCS)
int wiced_bt_ga_ascs_parse_data(wiced_bt_ga_ascs_opcode_t opcode,
                                uint8_t *data_stream,
                                int length,
                                wiced_bt_ga_ascs_cp_params_t *parsed_data,
                                wiced_bt_ga_ascs_cp_cmd_sts_t *status);

int wiced_bt_ga_ascs_get_cp_header(uint8_t *data, uint8_t *opcode, uint8_t *num_of_ase);

void wiced_bt_ga_ascs_send_receiver_start_stop_ready(uint16_t conn_id,
                                                     gatt_intf_service_object_t *p_service,
                                                     uint8_t ase_id,
                                                     wiced_bool_t is_start_ready);

extern const char *ascs_opcode_str[];
extern const char *ascs_state_str[];
