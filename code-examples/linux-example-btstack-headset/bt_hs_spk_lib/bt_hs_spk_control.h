/*
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*/

/** @file
 *
 * This file provides the private interface definitions for bt_hs_spk_control
 *
 */
#ifndef BT_HS_SPK_CONTROL_H
#define BT_HS_SPK_CONTROL_H

/*****************************************************************************
**  Constants that define the capabilities and configuration
*****************************************************************************/
#define EIR_DATA_LENGTH     240 /* max EIR size */
#define BSG_RFCOMM_SCN                  1
#define BSG_DEVICE_MTU                  800
#define WICED_BUFF_MAX_SIZE             264
#define BSG_TRANS_MAX_BUFFERS           10
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI)
#define TRANS_SPI_BUFFER_SIZE           1024
#endif
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_UART)
#define TRANS_UART_BUFFER_SIZE          1024
#endif

#include "wiced_bt_rfcomm.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_avrc_ct.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_dev.h"
#include "hci_control_api.h"
#include "wiced_bt_cfg.h"
#include "bt_hs_spk_button.h"
#include "wiced_audio_manager.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_hfp_hf.h"

////// TEMP for compiling

#ifndef BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS
#define BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS    1
#endif

/* Volume levels passed from the application to Audio Manager should be in the range 0 to 10
 * calculating from 0 to 127 levels to 0 to 10 levels.
 * The mapping between application (absolute) volume level ( 0 ~ 127 ) and Audio Manager ( 0 ~ 10 ) :
 * | Application Volume | 0 | 1 - 11 | 12 - 23 | 24 - 35 | 36 - 47 | 48 - 59 | 60 - 71 |
 * |     Audio Manager  | 0 |    1   |    2    |    3    |    4    |    5    |    6    |
 *
 * | Application Volume | 72 - 83 | 84 - 95 | 96 - 107 | 108 - 127 |
 * |     Audio Manager  |    7    |    8    |    9     |     10    |
 * */
#define APP_VOLUME_HIGH 126
#define APP_VOLUME_LOW 0
#define MIN_LEVEL 1
#define AM_VOL_LEVEL_HIGH 10
#define HFP_VOLUME_HIGH 15
#define HFP_VOLUME_LOW 1
#define HFP_VOLUME_AVERAGE 8
#define APP_LEVEL_DIV_FACTOR  ((APP_VOLUME_HIGH-APP_VOLUME_LOW)/AM_VOL_LEVEL_HIGH)
#define HFP_VOLUME_LEVEL_DIV_FACTOR ((HFP_VOLUME_HIGH-HFP_VOLUME_LOW)/AM_VOL_LEVEL_HIGH)

/* VSE */
#ifndef HCI_VSE_JITTER_BUFFER_EVENT
#define HCI_VSE_JITTER_BUFFER_EVENT     0x1A
#endif // HCI_VSE_JITTER_BUFFER_EVENT

#ifndef BT_EVT_BTU_IPC_BTM_EVT
#define BT_EVT_BTU_IPC_BTM_EVT          0x9005
#endif // BT_EVT_BTU_IPC_BTM_EVT

#ifndef AV_SINK_PLAY_STATUS_IND
#define AV_SINK_PLAY_STATUS_IND         68
#endif // AV_SINK_PLAY_STATUS_IND

#define JITTER_IDLE_STATE               0x1
#define JITTER_NORMAL_STATE             0x2
#define JITTER_UNDERRUN_STATE           0x4
#define JITTER_OVERRUN_STATE            0x8

typedef void (BT_HS_SPK_BLE_DISCOVERABILITY_CHANGE_CB)(wiced_bool_t discoverable);

/* BT connection status change callback */
typedef wiced_bool_t (*BT_HS_SPK_CONTROL_CONN_STATUS_CHANGE_CB)(wiced_bt_device_address_t bd_addr, uint8_t *p_features, wiced_bool_t is_connected, uint16_t handle, wiced_bt_transport_t transport, uint8_t reason);

/* BT visibility (discoverability and connectivity change callback */
typedef void BT_HS_SPK_CONTROL_BT_VISIBILITY_CHANGE_CB(wiced_bool_t discoverable, wiced_bool_t connectable);

/* BT power mode change callback. */
typedef void (BT_HS_SPK_CONTROL_POWER_MODE_CHANGE_CB)(wiced_bt_device_address_t bdaddr, wiced_bt_dev_power_mgmt_status_t power_mode);

/* Handsfree moudle HFP event pre-handler. */
typedef wiced_bool_t (BT_HS_SPK_CONTROL_HFP_CONTROL_CB_PRE_HANDLER)(wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data);

/* Audio module A2DP event pre-handler. */
typedef wiced_bool_t (BT_HS_SPK_CONTROL_A2DP_SINK_CONTROL_CB_PRE_HANDLER)(wiced_bt_a2dp_sink_event_t event, wiced_bt_a2dp_sink_event_data_t* p_data);

/* Audio module AVRC CT connection state pre-handler. */
typedef wiced_bool_t (BT_HS_SPK_CONTROL_AVRC_CT_CONNECTION_STATE_PRE_HANDLER)(uint8_t handle, wiced_bt_device_address_t remote_addr,
        wiced_result_t status, wiced_bt_avrc_ct_connection_state_t connection_state, uint32_t peer_features);

/* VSE callback */
typedef wiced_bool_t (BT_HS_SPK_CONTROL_VSE_CB)(uint8_t len, uint8_t *p);

/* NVRAM space for link keys has been updated callback. */
typedef void (BT_HS_SPK_CONTROL_NVRAM_LINK_KEYS_UPDATE_CB)(void);

/* Local volume change callback. */
typedef void (BT_HS_SPK_CONTROL_LOCAL_VOLUME_CHANGE_CB)(int32_t am_vol_level, uint8_t am_vol_effect_event);

/* Audio streaming source change callback. */
typedef void (BT_HS_SPK_CONTROL_AUDIO_STREAMING_SRC_CHG_CB(void));

/*****************************************************************************
**  Data types
*****************************************************************************/

/* The main application control block */
typedef struct
{
    uint8_t pairing_allowed;
} hci_control_cb_t;

typedef struct
{
    wiced_bool_t test_executing;
    uint16_t     opcode;
} hci_control_test_command_t;

typedef enum {
    SERVICE_NONE        =   (0),
    SERVICE_BT_A2DP     =   (1),         //!< SERVICE_BT_A2DP
    SERVICE_BT_HFP      =   (2),         //!< SERVICE_BT_HFP
} service_type_t;

typedef wiced_result_t (*WICED_APP_BUTTON_HANDLER)( app_service_action_t action );

typedef  struct {

    service_type_t              active_service;
    WICED_APP_BUTTON_HANDLER    button_handler;
} wiced_app_service_t;

typedef struct bt_hs_spk_control_config_audio
{
    struct
    {
        wiced_bt_a2dp_config_data_t                         *p_audio_config;

        /*
         * A2DP Sink event pre-handler
         * If the p_pre_handler is assigned by the user application,
         * the p_pre_handler will be executed in prior to the default A2DP event handler and the
         * default A2DP event handler will be executed only if the return value of the
         * p_pre_handler is set to WICED_TRUE;
         */
        BT_HS_SPK_CONTROL_A2DP_SINK_CONTROL_CB_PRE_HANDLER  *p_pre_handler;
        wiced_bt_a2dp_sink_control_cb_t                     post_handler;
        BT_HS_SPK_CONTROL_AUDIO_STREAMING_SRC_CHG_CB        *p_streaming_source_chg_cb;
    } a2dp;

    struct
    {
        uint8_t *p_supported_events;

        struct
        {
            /*
             * AVRC CT connection state pre-handler
             * If the p_pre_handler is assigned by the user application,
             * the p_pre_handler will be executed in prior to the default AVRC CT connection state
             * handler and the default AVRC CT connection state handler will be executed only if
             * the return value of the p_pre_handler is set to WICED_TRUE;
             */
            BT_HS_SPK_CONTROL_AVRC_CT_CONNECTION_STATE_PRE_HANDLER  *p_pre_handler;
            wiced_bt_avrc_ct_connection_state_cback_t               post_handler;
        } connection_state_cb;

        struct
        {
            wiced_bt_avrc_ct_cmd_cback_t                pre_handler;
            wiced_bt_avrc_ct_cmd_cback_t                post_handler;
        } command_cb;

        struct
        {
            wiced_bt_avrc_ct_rsp_cback_t                pre_handler;
            wiced_bt_avrc_ct_rsp_cback_t                post_handler;
        } rsp_cb;

        struct
        {
            wiced_bt_avrc_ct_pt_rsp_cback_t             pre_handler;
            wiced_bt_avrc_ct_pt_rsp_cback_t             post_handler;
        } passthrough_rsp_cb;
    } avrc_ct;
} bt_hs_spk_control_config_audio_t;

typedef struct bt_hs_spk_control_config_hfp
{
    struct
    {
        uint16_t                buffer_size;
        uint16_t                buffer_count;
    } rfcomm;

    uint32_t                                        feature_mask;

    /*
     * HFP event pre-handler
     * If the p_pre_handler is assigned by the user application,
     * the p_pre_handler will be executed in prior to the default HFP event handler and the
     * default HFP event handler will be executed only if the return value of the
     * p_pre_handler is set to WICED_TRUE;
     */
    BT_HS_SPK_CONTROL_HFP_CONTROL_CB_PRE_HANDLER    *p_pre_handler;
    wiced_bt_hfp_hf_event_cb_t                      post_handler;
} bt_hs_spk_control_config_hfp_t;

typedef struct bt_hs_spk_control_config_sleep
{
    wiced_bool_t            enable;
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
    /**< Enable the sleep mode. */
    wiced_sleep_mode_type_t sleep_mode;             /**< Requested sleep mode */
    wiced_sleep_wake_type_t host_wake_mode;         /**< Active level for host wake */
    wiced_sleep_wake_type_t device_wake_mode;       /**< Active level for device wake */
    uint8_t                 device_wake_source;     /**< Device wake source(s). GPIO mandatory for
                                                         WICED_SLEEP_MODE_TRANSPORT */
    uint32_t                device_wake_gpio_num;   /**< GPIO# for host to wake, mandatory for
                                                         ICED_SLEEP_MODE_TRANSPORT */
#endif
} bt_hs_spk_control_config_sleep_t;

typedef struct bt_hs_spk_control_config_nvram
{
    struct
    {
        uint16_t                                    id;             /**< NVRAM ID used for storing
                                                                       the paired devices' link
                                                                       keys. */
        BT_HS_SPK_CONTROL_NVRAM_LINK_KEYS_UPDATE_CB *p_callback;
    } link_key;
} bt_hs_spk_control_config_nvram_t;

typedef struct bt_hs_spk_control_config
{
    /*
     * BT connection status change callback.
     * If the p_conn_status_change_cb is assigned by the user application,
     * the p_conn_status_change_cb will be executed first and the default BT connection status
     * change callback will be executed if the return value of the p_conn_status_change_cb is set
     * to WICED_TRUE;
     */
    BT_HS_SPK_CONTROL_CONN_STATUS_CHANGE_CB     conn_status_change_cb;

    /*
     * BT (BR/EDR) Visibility (discoverability and connectivity) change callback.
     * When the BT discoverability or connectivity has been changed, this
     * callback assigned by the user application will be called for the
     * visibility change event.
     */
    BT_HS_SPK_CONTROL_BT_VISIBILITY_CHANGE_CB   *p_bt_visibility_chg_cb;
    uint16_t                                    discoverable_timeout;   // in seconds
    wiced_bool_t                                acl3mbpsPacketSupport;
    bt_hs_spk_control_config_audio_t            audio;
    bt_hs_spk_control_config_hfp_t              hfp;
    bt_hs_spk_control_config_sleep_t            sleep_config;
    bt_hs_spk_control_config_nvram_t            nvram;
    BT_HS_SPK_CONTROL_LOCAL_VOLUME_CHANGE_CB    *p_local_vol_chg_cb;
} bt_hs_spk_control_config_t;

typedef struct bt_hs_spk_eir_config
{
    char            *p_dev_name;   /* device name */
    wiced_bool_t    default_uuid_included;

    struct
    {
        wiced_bool_t    included;
        uint8_t         *p_content;
        uint8_t         len;        /* length of content*/
    } app_specific;
} bt_hs_spk_eir_config_t;

typedef struct bt_hs_spk_control_connection_info
{
    struct
    {
        wiced_bt_device_address_t           bdaddr;
        wiced_bool_t                        connected;
        uint8_t                             reason;     // disconnection reason
        uint8_t                             last_disconnection_reason;
        wiced_bt_dev_power_mgmt_status_t    power_mode;
        uint16_t                            sniff_interval;
        uint16_t                            link_policy;
    } acl[BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS];
} bt_hs_spk_control_connection_info_t;

/*****************************************************************************
**  Global data
*****************************************************************************/

/* control block declaration */
#if BTA_DYNAMIC_MEMORY == FALSE
extern hci_control_cb_t hci_control_cb;
#else
extern hci_control_cb_t *hci_control_cb_ptr;
#define hci_control_cb( *hci_control_cb_ptr )
#endif

extern wiced_bool_t avrcp_profile_role;

/*****************************************************************************
**  Function prototypes
*****************************************************************************/
/* main functions */
extern void     bt_hs_spk_control_reconnect(void);
wiced_bool_t    bt_hs_spk_control_reconnect_peer_bdaddr_get(wiced_bt_device_address_t peer_bdaddr);
wiced_bool_t    bt_hs_spk_control_reconnect_state_get(void);
void            bt_hs_spk_control_reconnect_info_reset(void);

extern void     bt_hs_spk_control_switch_avrcp_role(uint8_t new_role);

void     bt_hs_spk_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability, wiced_bt_transport_t transport );
void     bt_hs_spk_control_handle_set_pairability ( uint8_t pairing_allowed );
wiced_app_service_t* get_app_current_service( void );
wiced_result_t app_set_current_service(wiced_app_service_t *app_service);
wiced_result_t bt_hs_spk_post_stack_init(bt_hs_spk_control_config_t *p_config);
wiced_result_t bt_hs_spk_write_eir(bt_hs_spk_eir_config_t *p_config);
extern wiced_result_t bt_hs_spk_audio_insert_start(uint32_t *p_sample_rate, uint32_t duration_sec);
extern wiced_result_t bt_hs_spk_audio_insert_stop(void);
/* common functions */

am_audio_io_device_t bt_hs_spk_get_audio_sink(void);
void bt_hs_spk_set_audio_sink(am_audio_io_device_t sink);

uint8_t bt_hs_spk_control_br_edr_last_disconnection_reason_get(wiced_bt_device_address_t bd_addr);

void bt_hs_spk_ble_discoverability_change_callback_register(BT_HS_SPK_BLE_DISCOVERABILITY_CHANGE_CB *p_cb);

wiced_bool_t bt_hs_spk_control_connection_status_check_be_edr(wiced_bool_t all);

/**
 * bt_hs_spk_control_bt_power_mode_set
 *
 * Set the BT power mode
 *
 * @param[in]   active: WICED_TRUE - set to active mode
 *                      WICED_FALSE - set to sniff mode
 *
 * @param[in]   bdaddr: peer device's BT address
 *                      Set to NULL for all existent ACL links.
 *
 * @param[in]   p_cb: callback function when the power mode has been changed
 *
 * @return      WICED_BT_SUCCESS - the link(s) is already in the target power mode
 *              WICED_BT_PENDING - the target link(s) is waiting to enter the target mode
 *                                 The user application is expected to receive the corresponding
 *                                 BTM_POWER_MANAGEMENT_STATUS_EVT event.
 *              WICED_BT_BADARG - Corresponding ACL link cannot be found or the target ACL link is not connected
 *              WICED_BT_BUSY - the ACL link is already set to enter/leave sniff mode and waiting for power mode being changed
 *              others - fail
 */
wiced_result_t bt_hs_spk_control_bt_power_mode_set(wiced_bool_t active, wiced_bt_device_address_t bdaddr, BT_HS_SPK_CONTROL_POWER_MODE_CHANGE_CB *p_cb);

/**
 * bt_hs_spk_control_bt_power_mode_set_exclusive
 *
 * Set the BT power mode except for the target device's link
 *
 * @param[in]   active: WICED_TRUE - set to active mode
 *                      WICED_FALSE - set to sniff mode
 *
 * @param[in]   bdaddr: the exclusive peer device's BT address
 *
 * @param[in]   p_cb: callback function when the power mode has been changed
 *
 */
void bt_hs_spk_control_bt_power_mode_set_exclusive(wiced_bool_t active, wiced_bt_device_address_t bdaddr, BT_HS_SPK_CONTROL_POWER_MODE_CHANGE_CB *p_cb);

/*
 * bt_hs_spk_control_discoverable_timeout_get
 *
 * Acquire the discoverability timeout setting
 *
 * @return  uint16_t the timeout in second
 */
uint16_t bt_hs_spk_control_discoverable_timeout_get(void);

/**
 * @brief       Update the Bluetooth Link Key to NVRAM if NVRAM writing is pending
 *
 * @note        The operation for writing NVRAM will disable thread preemption
 *
 */
void bt_hs_spk_control_link_key_nvram_update(void);

/**
 * bt_hs_spk_control_link_keys_get
 *
 * Get the stored link keys.
 *
 * @return wiced_bt_device_link_keys_t * - pointer to the link keys database
 *                                         the array number of link keys is defined in
 *                                         BT_HS_SPK_CONTROL_LINK_KEY_COUNT
 */
wiced_bt_device_link_keys_t *bt_hs_spk_control_link_keys_get(void);

/**
 * Set link keys to the database and update the NVRAM if required.
 *
 * @param p_link_keys - pointer to the link keys to be set.
 *                      The array number of link keys is defined in
 *                      BT_HS_SPK_CONTROL_LINK_KEY_COUNT.
 *
 * @return  WICED_BT_SUCCESS - success
 *          WICED_BT_ERROR - fail
 */
wiced_result_t bt_hs_spk_control_link_keys_set(wiced_bt_device_link_keys_t *p_link_keys);

/**
 * bt_hs_spk_control_btm_event_handler_encryption_status
 *
 * Handle the BTM event, BTM_ENCRYPTION_STATUS_EVT
 *
 * @param p_event_data
 */
void bt_hs_spk_control_btm_event_handler_encryption_status(wiced_bt_dev_encryption_status_t *p_event_data);

/**
 * bt_hs_spk_control_btm_event_handler_encryption_status
 *
 * Handle the BTM event, BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT and BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT
 *
 * @param p_link_key
 *
 * @return  WICED_TRUE - success
 *          WICED_FALSE - fail
 */
wiced_bool_t bt_hs_spk_control_btm_event_handler_link_key(wiced_bt_management_evt_t event, wiced_bt_device_link_keys_t *p_link_key);

/**
 * bt_hs_spk_control_btm_event_handler_power_management_status
 *
 * Handle the BTM event, BTM_POWER_MANAGEMENT_STATUS_EVT
 *
 * @param p_event: event data
 */
void bt_hs_spk_control_btm_event_handler_power_management_status(wiced_bt_power_mgmt_notification_t *p_event_data);

/**
 * bt_hs_spk_control_acl_link_policy_sniff_mode_set
 *
 * Set the sniff mode enable/disable for specific/all acl connection(s)
 *
 * @param bdaddr - target peer device's BT address
 *                 NULL for all ACL connections
 * @param enable - WICED_TRUE: enable sniff mode
 *                 WICED_FALSE: disable sniff mode
 */
void bt_hs_spk_control_acl_link_policy_sniff_mode_set(wiced_bt_device_address_t bdaddr, wiced_bool_t enable);

/**
 * bt_hs_spk_control_acl_link_policy_sniff_mode_set_exclusive
 *
 * Set the sniff mode enable/disable except for the target acl connection
 *
 * @param bdaddr - the exclusive peer device's BT address
 *
 * @param enable - WICED_TRUE: enable sniff mode
 *                 WICED_FALSE: disable sniff mode
 */
void bt_hs_spk_control_acl_link_policy_sniff_mode_set_exclusive(wiced_bt_device_address_t bdaddr, wiced_bool_t enable);

/**
 * bt_hs_spk_control_acl_link_policy_set
 *
 * Set the BT ACL link policy.
 *
 * @param bdaddr - connection with peer device
 * @param link_policy - HCI_DISABLE_ALL_LM_MODES
 *                      HCI_ENABLE_ROLE_SWITCH
 *                      HCI_ENABLE_HOLD_MODE
 *                      HCI_ENABLE_SNIFF_MODE
 *                      HCI_ENABLE_PARK_MODE
 */
void bt_hs_spk_control_acl_link_policy_set(wiced_bt_device_address_t bdaddr, uint16_t link_policy);

/**
 * bt_hs_spk_control_disconnect
 *
 * Disconnect target peer device.
 *
 * @param bdaddr - target device's BT address
 *                 If this is set to NULL, all the connected devices will be disconnected
 */
void bt_hs_spk_control_disconnect(wiced_bt_device_address_t bdaddr);

/**
 * bt_hs_spk_control_pairability_get
 *
 * Acquire current pairability.
 *
 * @return  WICED_TRUE: device is pairable now
 */
wiced_bool_t bt_hs_spk_control_pairability_get(void);

/**
 * bt_hs_spk_control_register_vse_callback
 *
 * Register the VSE callback.
 *
 * @param[in] p_cb - callback
 */
void bt_hs_spk_control_register_vse_callback(BT_HS_SPK_CONTROL_VSE_CB *p_cb);

/**
 * bt_hs_spk_control_connection_info_set
 *
 * Set the connection information
 *
 * Note: Do NOT use this utility unless you certainly understand what you are doing.
 *       Using this utility MAY cause unexpected behavior and crash.
 *
 * @param[in] p_info
 */
void bt_hs_spk_control_connection_info_set(bt_hs_spk_control_connection_info_t *p_info);

/**
 * bt_hs_spk_control_connection_info_get
 *
 * Get the content of connection(s)
 *
 * @param[out] p_info
 */
void bt_hs_spk_control_connection_info_get(bt_hs_spk_control_connection_info_t *p_info);

/**
 * bt_hs_spk_control_misc_data_content_check
 *
 * helper function to check if the data content is valid
 *
 * @param[in] p_data - data to be verified
 * @param[in] len - length of data in bytes
 *
 * @return  WICED_TRUE - data is valid
 *          WICED_FALSE - data is invalid (all 0s)
 */
wiced_bool_t bt_hs_spk_control_misc_data_content_check(uint8_t *p_data, uint32_t len);

/**
 * bt_hs_spk_control_bt_role_set
 *
 * Set the IUT role with the target connection
 *
 * @param[in]   bdaddr - peer device's address
 * @param[in]   target_role - HCI_ROLE_CENTRAL
 *                            HCI_ROLE_PERIPHERAL
 *
 * @return      WICED_BT_BADARG
 *              WICED_BT_ERROR
 *              WICED_BT_SUCCESS
 */
wiced_result_t bt_hs_spk_control_bt_role_set(wiced_bt_device_address_t bdaddr, uint8_t target_role);


/**
 * set_skip_find_pairing_key
 *
 * set skip_pair_key = true to skip find pairing key to avoid pairing key cache fail when connect peer device manually
 *
 * @param void - none
 *
 */
void set_skip_find_pairing_key(void);
#endif /* BT_HS_SPK_CONTROL_H */
