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
 * This file implement BTLE controls.
 * The GATT database is defined in this file.
 *
 */
#include "wiced_bt_dev.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_timer.h"
#include "bt_hs_spk_control.h"
#include "wiced_memory.h"
#include "wiced_app_cfg.h"
#include "headset_nvram.h"
#include "headset_control_le.h"
#ifdef FASTPAIR_ENABLE
#include "wiced_bt_gfps.h"
#endif
#include "wiced_hal_memory.h"

#include "app_bt_utils.h"
#include "wiced_bt_dev_utils.h"
#include "log.h"
/******************************************************
 *                     Constants
 ******************************************************/
#define LE_CONTROL_MAX_CONNECTIONS          20
#define LE_CONTROL_CONNECT_TIMEOUT          10

/* UUID value of the Hello Sensor Service */
#define UUID_HELLO_SERVICE                    0x23, 0x20, 0x56, 0x7c, 0x05, 0xcf, 0x6e, 0xb4, 0xc3, 0x41, 0x77, 0x28, 0x51, 0x82, 0x7e, 0x1b
/* UUID value of the Hello Sensor Characteristic, Value Notification */
#define UUID_HELLO_CHARACTERISTIC_NOTIFY      0x26, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8a
/* UUID value of the Hello Sensor Characteristic, Configuration */
#define UUID_HELLO_CHARACTERISTIC_CONFIG      0x1a, 0x89, 0x07, 0x4a, 0x2f, 0x3b, 0x7e, 0xa6, 0x81, 0x44, 0x3f, 0xf9, 0xa8, 0xf2, 0x9b, 0x5e
/* UUID value of the Hello Sensor Characteristic, Configuration */
#define UUID_HELLO_CHARACTERISTIC_LONG_MSG    0x2a, 0x99, 0x17, 0x5a, 0x3f, 0x4b, 0x8e, 0xb6, 0x91, 0x54, 0x2f, 0x09, 0xb8, 0x02, 0xab, 0x6e

#ifdef FASTPAIR_ENABLE
/* MODEL-specific definitions */
#ifdef CYW20721B2
#define FASTPAIR_MODEL_ID                   0x82DA6E
#else
#define FASTPAIR_MODEL_ID                   0xCE948F //0xB49236 //0x000107 //0x140A02 // 0xCE948F
#endif

#if (FASTPAIR_MODEL_ID == 0x82DA6E)
const uint8_t anti_spoofing_public_key[] =  { 0x95, 0xcf, 0xdb, 0xae, 0xc0, 0xef, 0xc5, 0x1f, 0x39, 0x0f, 0x2a, 0xe0, 0x16, 0x5a, 0x2b, 0x59,\
		                                      0x62, 0xb2, 0xfe, 0x82, 0xfa, 0xf0, 0xd4, 0x1e, 0xa3, 0x4f, 0x07, 0x7e, 0xf7, 0x3d, 0xc0, 0x44,\
		                                      0x3d, 0xd0, 0x38, 0xb2, 0x31, 0x5d, 0xc6, 0x45, 0x72, 0x8a, 0x08, 0x0e, 0xc7, 0x4f, 0xc7, 0x76,\
		                                      0xd1, 0x19, 0xed, 0x8b, 0x17, 0x50, 0xb3, 0xa6, 0x94, 0x2e, 0xc8, 0x6b, 0xbb, 0x02, 0xc7, 0x4d };

const uint8_t anti_spoofing_private_key[] = { 0x84, 0xee, 0x67, 0xc3, 0x67, 0xea, 0x57, 0x38, 0xa7, 0x7e, 0xe2, 0x4d, 0x68, 0xaa, 0x9c, 0xf0,\
                                              0xc7, 0x9f, 0xc8, 0x07, 0x7e, 0x4e, 0x20, 0x35, 0x4c, 0x15, 0x43, 0x4d, 0xb5, 0xd2, 0xd1, 0xc3 };

#elif (FASTPAIR_MODEL_ID == 0xCE948F)
const uint8_t anti_spoofing_public_key[] =  { 0x0e, 0xe2, 0xbf, 0xe7, 0x96, 0xc6, 0xe1, 0x13, 0xf6, 0x57, 0x4a, 0xa8, 0x8c, 0x3a, 0x1b, 0x9c,\
                                              0x67, 0x1e, 0x36, 0xdf, 0x62, 0x69, 0xd8, 0xe5, 0x07, 0xe6, 0x8a, 0x72, 0x66, 0x4c, 0x9c, 0x90,\
                                              0xfc, 0xff, 0x00, 0x4f, 0x0f, 0x95, 0xde, 0x63, 0xe1, 0xc0, 0xbb, 0xa0, 0x75, 0xb1, 0xd2, 0x76,\
                                              0xfd, 0xe9, 0x66, 0x25, 0x0d, 0x45, 0x43, 0x7d, 0x5b, 0xf9, 0xce, 0xc0, 0xeb, 0x11, 0x03, 0xbe };

const uint8_t anti_spoofing_private_key[] = { 0x71, 0x11, 0x42, 0xb5, 0xe4, 0xa0, 0x6c, 0xa2, 0x8b, 0x74, 0xd4, 0x87, 0x7d, 0xac, 0x15, 0xc5,\
                                              0x42, 0x38, 0x1d, 0xb7, 0xba, 0x21, 0x19, 0x60, 0x17, 0x67, 0xfc, 0xba, 0x67, 0x47, 0x44, 0xc6 };

#else
const uint8_t anti_spoofing_public_key[] =  "";
const uint8_t anti_spoofing_private_key[] = "";
#warning "No Anti-Spooging key"

#endif
#endif //FASTPAIR_ENABLE

/******************************************************
 *                     Structures
 ******************************************************/
typedef struct
{
#define LE_CONTROL_STATE_IDLE                       0
#define LE_CONTROL_STATE_DISCOVER_PRIMARY_SERVICES  1
#define LE_CONTROL_STATE_DISCOVER_CHARACTERISTICS   2
#define LE_CONTROL_STATE_DISCOVER_DESCRIPTORS       3
#define LE_CONTROL_STATE_READ_VALUE                 4
#define LE_CONTROL_STATE_WRITE_VALUE                5
#define LE_CONTROL_STATE_WRITE_NO_RESPONSE_VALUE    6
#define LE_CONTROL_STATE_NOTIFY_VALUE               7
#define LE_CONTROL_STATE_INDICATE_VALUE             8
#define LE_CONTROL_STATE_WRITE_DESCRIPTOR_VALUE     9
#define LE_CONTROL_STATE_DISCONNECTING              10

    uint8_t           state;                // Application discovery state
    wiced_bool_t      indication_sent;      // TRUE if indication sent and not acked
    wiced_bt_device_address_t           bd_addr;
    uint16_t          conn_id;              // Connection ID used for exchange with the stack
    uint16_t          peer_mtu;             // MTU received in the MTU request (or 23 if peer did not send MTU request)

    uint8_t           role;                 // HCI_ROLE_CENTRAL or HCI_ROLE_PERIPHERAL
} hci_control_le_conn_state_t;

typedef struct
{
    hci_control_le_conn_state_t conn[LE_CONTROL_MAX_CONNECTIONS + 1];
} hci_control_le_cb_t;

hci_control_le_cb_t le_control_cb;

typedef struct t_hci_control_le_pending_tx_buffer_t
{
    wiced_bool_t        tx_buf_saved;
    uint16_t            tx_buf_conn_id;
    uint16_t            tx_buf_type;
    uint16_t            tx_buf_len;
    uint16_t            tx_buf_handle;
    uint8_t             tx_buf_data[HCI_CONTROL_GATT_COMMAND_MAX_TX_BUFFER];
} hci_control_le_pending_tx_buffer_t;

wiced_timer_t hci_control_le_connect_timer;


/******************************************************
 *               Variables Definitions
 ******************************************************/

//uint8_t  hci_control_le_data_xfer_buf[20] = {0xff, 0xfe};
wiced_bt_device_address_t  hci_control_le_remote_bdaddr;

hci_control_le_pending_tx_buffer_t hci_control_le_pending_tx_buffer;

/******************************************************************************
 *                                GATT DATABASE
 ******************************************************************************/
/*
 * This is the GATT database for the Hello Sensor application.  It defines
 * services, characteristics and descriptors supported by the sensor.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if and how peer is allowed to read or write
 * into it.  All handles do not need to be sequential, but need to be in
 * ascending order.
 */
const uint8_t gatt_server_db[]=
{
    /* Declare mandatory GATT service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GATT_SERVICE, UUID_SERVICE_GATT ),

    /* Declare mandatory GAP service. Device Name and Appearance are mandatory
     * characteristics of GAP service                                        */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GAP_SERVICE, UUID_SERVICE_GAP ),

        /* Declare mandatory GAP service characteristic: Dev Name */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,
                GATT_UUID_GAP_DEVICE_NAME, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

        /* Declare mandatory GAP service characteristic: Appearance */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
                GATT_UUID_GAP_ICON, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    /* Declare proprietary Hello Service with 128 byte UUID */
    PRIMARY_SERVICE_UUID128( HANDLE_HSENS_SERVICE, UUID_HELLO_SERVICE ),

        /* Declare characteristic used to notify/indicate change */
        CHARACTERISTIC_UUID128( HANDLE_HSENS_SERVICE_CHAR_NOTIFY, HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,
            UUID_HELLO_CHARACTERISTIC_NOTIFY, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY | GATTDB_CHAR_PROP_INDICATE, GATTDB_PERM_READABLE ),

            /* Declare client characteristic configuration descriptor
             * Value of the descriptor can be modified by the client
             * Value modified shall be retained during connection and across connection
             * for bonded devices.  Setting value to 1 tells this application to send notification
             * when value of the characteristic changes.  Value 2 is to allow indications. */
            CHAR_DESCRIPTOR_UUID16_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_CFG_DESC, GATT_UUID_CHAR_CLIENT_CONFIG,
                GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

        /* Declare characteristic Hello Configuration */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_BLINK, HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,
            UUID_HELLO_CHARACTERISTIC_CONFIG, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE,
            GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ ),

    /* Declare Device info service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE, UUID_SERVCLASS_DEVICE_INFO ),

        /* Handle 0x4e: characteristic Manufacturer Name, handle 0x4f characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
                GATT_UUID_MANU_NAME, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

        /* Handle 0x50: characteristic Model Number, handle 0x51 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
                GATT_UUID_MODEL_NUMBER_STR, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

        /* Handle 0x52: characteristic System ID, handle 0x53 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
                GATT_UUID_SYSTEM_ID, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    /* Declare Battery service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_BATTERY_SERVICE, UUID_SERVCLASS_BATTERY ),

        /* Handle 0x62: characteristic Battery Level, handle 0x63 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL, HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,
                GATT_UUID_BATTERY_LEVEL, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

#ifdef FASTPAIR_ENABLE
    // Declare Fast Pair service
    PRIMARY_SERVICE_UUID16 (HANDLE_FASTPAIR_SERVICE, WICED_BT_GFPS_UUID16),

    CHARACTERISTIC_UUID16_WRITABLE (HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING,
                                    HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_VAL,
                                    WICED_BT_GFPS_UUID_CHARACTERISTIC_KEY_PAIRING,
                                    GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITE_REQ),

    CHARACTERISTIC_UUID16_WRITABLE (HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY,
                                    HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY_VAL,
                                    WICED_BT_GFPS_UUID_CHARACTERISTIC_PASSKEY,
                                    GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITE_REQ),

    CHARACTERISTIC_UUID16_WRITABLE (HANDLE_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY,
                                    HANDLE_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_VAL,
                                    WICED_BT_GFPS_UUID_CHARACTERISTIC_ACCOUNT_KEY,
                                    GATTDB_CHAR_PROP_WRITE,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITE_REQ),
#endif
};

typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void     *p_attr;
} attribute_t;

uint8_t btheadset_sensor_device_name[]          = "HeadsetPro";
uint8_t btheadset_sensor_appearance_name[2]     = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };
char    btheadset_sensor_char_notify_value[]    = { 'H', 'e', 'l', 'l', 'o', ' ', '0', };
char    btheadset_sensor_char_mfr_name_value[]  = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0, };
char    btheadset_sensor_char_model_num_value[] = { '1', '2', '3', '4',   0,   0,   0,   0 };
uint8_t btheadset_sensor_char_system_id_value[] = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71};

static uint8_t btheadset_battery_level;
static char blink_value;
uint8_t        conn_idx = 0;
static char *p_headset_control_le_dev_name = NULL;
static wiced_bt_ble_advert_elem_t headset_control_le_adv_elem = {0};
wiced_bt_db_hash_t headset_db_hash;

attribute_t gauAttributes[] =
{
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,       sizeof( btheadset_sensor_device_name ),         btheadset_sensor_device_name },
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL, sizeof(btheadset_sensor_appearance_name),       btheadset_sensor_appearance_name },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,  sizeof(btheadset_sensor_char_mfr_name_value),   btheadset_sensor_char_mfr_name_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL, sizeof(btheadset_sensor_char_model_num_value),  btheadset_sensor_char_model_num_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, sizeof(btheadset_sensor_char_system_id_value),  btheadset_sensor_char_system_id_value },
    { HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,      1,                                            &btheadset_battery_level },
    //{ HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,      1,                                            &blink_value },
};

extern wiced_bt_cfg_ble_t cy_bt_cfg_ble;
/******************************************************
 *               Function Definitions
 ******************************************************/

static wiced_result_t         hci_control_le_connection_up( wiced_bt_gatt_connection_status_t *p_status );
static wiced_result_t         hci_control_le_connection_down( wiced_bt_gatt_connection_status_t *p_status );
static wiced_result_t         hci_control_le_write_handler( uint16_t conn_id, wiced_bt_gatt_opcode_t opcode , wiced_bt_gatt_write_req_t * p_data );
static void                   hci_control_le_notification_handler( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len );
static void                   hci_control_le_indication_handler( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len );

static wiced_bt_gatt_status_t hci_control_le_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );
static void                   hci_control_le_indication_confirmation_handler( uint16_t conn_id, uint16_t handle );
static void                   hci_control_le_num_complete_callback( void );
static void                   hci_control_le_handle_gatt_db_init(uint8_t* data, uint32_t length,  wiced_bt_db_hash_t hash);
static void                   hci_control_le_connect_timeout( TIMER_PARAM_TYPE arg );
/*
 * Initialize LE Control
 */
void hci_control_le_init( void )
{
    memset( &le_control_cb, 0, sizeof( le_control_cb ) );
    memset( &hci_control_le_pending_tx_buffer, 0, sizeof( hci_control_le_pending_tx_buffer ) );
}
static void headset_control_le_discoverabilty_change_callback(wiced_bool_t discoverable)
{
    TRACE_LOG("Start");
#ifdef FASTPAIR_ENABLE
    wiced_bt_gfps_provider_discoverablility_set(discoverable);
#endif
}
/*
 * Enable LE Control
 */
void hci_control_le_enable( void )
{
    wiced_bt_gatt_status_t     gatt_status;
#ifdef FASTPAIR_ENABLE
    wiced_bt_gfps_provider_conf_t fastpair_conf = {0};
#endif
    char appended_ble_dev_name[] = "ifx";
    uint8_t *p_index;
    uint16_t dev_name_len;

    /*  GATT DB Initialization */
    gatt_status = wiced_bt_gatt_db_init(gatt_server_db, sizeof(gatt_server_db),headset_db_hash);
#ifdef FASTPAIR_ENABLE
    wiced_bt_dev_set_no_smp_on_br(WICED_TRUE);
#endif

    TRACE_LOG("wiced_bt_gatt_db_init %d", gatt_status);

#ifdef FASTPAIR_ENABLE
    // set Tx power level data type in LE advertisement
#if defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1) || defined (CYW20820A1)
    fastpair_conf.ble_tx_pwr_level = cy_bt_cfg_ble.default_ble_power_level; //use Fast Pair Validator test ?
#else
    fastpair_conf.ble_tx_pwr_level = 0;
#endif

    // set GATT event callback
    fastpair_conf.p_gatt_cb = hci_control_le_gatt_callback;

    // set assigned handles for GATT attributes
    fastpair_conf.gatt_db_handle.key_pairing_val        = HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_VAL;
    fastpair_conf.gatt_db_handle.key_pairing_cfg_desc   = HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_CFG_DESC;
    fastpair_conf.gatt_db_handle.passkey_val            = HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY_VAL;
    fastpair_conf.gatt_db_handle.passkey_cfg_desc       = HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY_CFG_DESC;
    fastpair_conf.gatt_db_handle.account_key_val        = HANDLE_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_VAL;

    // model id
    fastpair_conf.model_id = FASTPAIR_MODEL_ID;

    // anti-spoofing public key
    memcpy((void *) &fastpair_conf.anti_spoofing_key.public[0],
           (void *) &anti_spoofing_public_key[0],
           WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PUBLIC);

    // anti-spoofing private key
    memcpy((void *) &fastpair_conf.anti_spoofing_key.private[0],
           (void *) &anti_spoofing_private_key[0],
           WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PRIVATE);

    // Account Key Filter generate format
    fastpair_conf.account_key_filter_generate_random = WICED_TRUE;;

    // Account Key list size
    fastpair_conf.account_key_list_size = FASTPAIR_ACCOUNT_KEY_NUM;

    // NVRAM id for Account Key list
    fastpair_conf.account_key_list_nvram_id = HEADSET_NVRAM_ID_GFPS_ACCOUNT_KEY;
    // LE advertisement appended to fast pair advertisement data
    dev_name_len = strlen((char *) wiced_bt_cfg_settings.device_name) +
                   strlen(appended_ble_dev_name);

    p_headset_control_le_dev_name = (char *) wiced_memory_allocate(dev_name_len);

    if (p_headset_control_le_dev_name)
    {
        p_index = (uint8_t *) p_headset_control_le_dev_name;
        memcpy((void *) p_index,
               (void *) wiced_bt_cfg_settings.device_name,
               strlen((char *) wiced_bt_cfg_settings.device_name));

        p_index += strlen((char *) wiced_bt_cfg_settings.device_name);

        memcpy((void *) p_index,
               (void *) appended_ble_dev_name,
               strlen(appended_ble_dev_name));
    }
    else
    {
        dev_name_len = 0;
    }

    headset_control_le_adv_elem.advert_type    = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    headset_control_le_adv_elem.len            = dev_name_len;
    headset_control_le_adv_elem.p_data         = (uint8_t *) p_headset_control_le_dev_name;

    fastpair_conf.appended_adv_data.p_elem      = &headset_control_le_adv_elem;
    fastpair_conf.appended_adv_data.elem_num    = 1;

    /* Initialize Google Fast Pair Service. */

    if (wiced_bt_gfps_provider_init(&fastpair_conf) == WICED_FALSE)
    {
        TRACE_ERR("wiced_bt_gfps_provider_init fail");
    }

#else
    /* GATT registration */
    gatt_status = wiced_bt_gatt_register( hci_control_le_gatt_callback );
    TRACE_LOG( "wiced_bt_gatt_register status %d", gatt_status );

#endif

    /* Register the BLE discoverability change callback. */
    bt_hs_spk_ble_discoverability_change_callback_register(&headset_control_le_discoverabilty_change_callback);

    /* Initialize connection timer */
    wiced_init_timer( &hci_control_le_connect_timer, &hci_control_le_connect_timeout, 0, WICED_SECONDS_TIMER );
}

/*
 * Process advertisement packet received
 */
void hci_control_le_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    if ( p_scan_result )
    {
        TRACE_LOG("Device:");
        print_bd_address(p_scan_result->remote_bd_addr);
        //hci_control_le_send_advertisement_report( p_scan_result, p_adv_data );
    }
    else
    {
        TRACE_LOG( " Scan completed" );
    }
}

/*
 * Process connection up event
 */
wiced_result_t hci_control_le_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    uint16_t       conn_id = p_status->conn_id;
    uint8_t        role;
    TRACE_LOG( "hci_control_le_connection_up: transport:%d", p_status->transport);
    print_bd_address(p_status->bd_addr); 
    wiced_bt_dev_get_role( p_status->bd_addr, &role, p_status->transport);
    le_control_cb.conn[conn_idx].role = role;
    print_bd_address(p_status->bd_addr);
    TRACE_LOG( "hci_control_le_connection_up, id:%d role:%d:", p_status->conn_id, role );

    /* Stop le connection timer*/
    wiced_stop_timer( &hci_control_le_connect_timer );

    memcpy( le_control_cb.conn[conn_idx].bd_addr, p_status->bd_addr, BD_ADDR_LEN );
    le_control_cb.conn[conn_idx].state      = LE_CONTROL_STATE_IDLE;
    le_control_cb.conn[conn_idx].conn_id    = p_status->conn_id;
    le_control_cb.conn[conn_idx].peer_mtu   = GATT_BLE_DEFAULT_MTU_SIZE;

    //hci_control_le_send_connect_event(0 /* TBD should come from p_status */,
    //        p_status->bd_addr, conn_id, role);
    return ( WICED_SUCCESS );
}

/*
* Process connection down event
*/
wiced_result_t hci_control_le_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    uint16_t          conn_id = p_status->conn_id;

    TRACE_LOG( "le_connection_down conn_id:%d Disc_Reason: %02x", conn_id, p_status->reason );

    /* Stop le connection timer*/
    wiced_stop_timer( &hci_control_le_connect_timer );

    le_control_cb.conn[conn_idx].state   = LE_CONTROL_STATE_IDLE;
    le_control_cb.conn[conn_idx].conn_id = 0;

    if ( ( conn_id == hci_control_le_pending_tx_buffer.tx_buf_conn_id ) &&
                    ( hci_control_le_pending_tx_buffer.tx_buf_saved ) )
    {
        hci_control_le_pending_tx_buffer.tx_buf_saved = WICED_FALSE;
    }

    //hci_control_le_send_disconnect_evt( p_status->reason, conn_id );

    return ( WICED_SUCCESS );
}


/*
* Process connection status callback
*/
wiced_result_t hci_control_le_conn_status_callback( wiced_bt_gatt_connection_status_t *p_status )
{
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
    wiced_ota_fw_upgrade_connection_status_event(p_status);
#endif

    if ( p_status->connected )
    {
        return hci_control_le_connection_up( p_status );
    }
    else
    {
        return hci_control_le_connection_down( p_status );
    }
}

/*
 * Operation complete received from the GATT server
 */
wiced_result_t hci_control_le_gatt_operation_comp_cb( wiced_bt_gatt_operation_complete_t *p_complete )
{
    uint16_t conn_id = p_complete->conn_id;
    TRACE_LOG("hci_control_le_gatt_operation_comp_cb");
    switch ( p_complete->op )
    {
    case GATTC_OPTYPE_DISCOVERY:
        TRACE_LOG( "!!! Disc compl conn_id:%d state:%d", conn_id, le_control_cb.conn[conn_idx].state );
        break;

    case GATTC_OPTYPE_READ_HANDLE:
        // read response received, pass it up and set state to idle
        TRACE_LOG( "Read response conn_id:%d state:%d", conn_id, le_control_cb.conn[conn_idx].state );
        if ( le_control_cb.conn[conn_idx].state == LE_CONTROL_STATE_READ_VALUE )
        {
            le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_IDLE;
        }
        break;

    case GATTC_OPTYPE_WRITE_WITH_RSP:
    case GATTC_OPTYPE_WRITE_NO_RSP:
    case GATTC_OPTYPE_EXECUTE_WRITE:
        // write response received, pass it up and set state to idle
        TRACE_LOG( "Write response conn_id:%d state:%d", conn_id, le_control_cb.conn[conn_idx].state );
        if ( le_control_cb.conn[conn_idx].state == LE_CONTROL_STATE_WRITE_VALUE )
        {
            le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_IDLE;
            //hci_control_le_send_write_completed( conn_id, p_complete->status );
        }
        break;

    case GATTC_OPTYPE_CONFIG_MTU:
        TRACE_LOG( "Config rcvd conn_id:%d state:%d", conn_id, le_control_cb.conn[conn_idx].state );
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        TRACE_LOG( "Notification rcvd conn_id:%d state:%d", conn_id, le_control_cb.conn[conn_idx].state );
        hci_control_le_notification_handler( conn_id,
                p_complete->response_data.att_value.handle,
                p_complete->response_data.att_value.p_data,
                p_complete->response_data.att_value.len );
        break;

    case GATTC_OPTYPE_INDICATION:
        TRACE_LOG( "Indication rcvd conn_id:%d state:%d", conn_id, le_control_cb.conn[conn_idx].state );
        hci_control_le_indication_handler( conn_id,
                p_complete->response_data.att_value.handle,
                p_complete->response_data.att_value.p_data,
                p_complete->response_data.att_value.len );
        break;
    }
    return ( WICED_SUCCESS );
}

/*
 * Discovery result received from the GATT server
 */
wiced_result_t hci_control_le_gatt_disc_result_cb( wiced_bt_gatt_discovery_result_t *p_result )
{
    uint16_t conn_id = p_result->conn_id;

    TRACE_LOG( "Discovery result conn_id:%d state:%d", conn_id, le_control_cb.conn[conn_idx].state );

    switch ( le_control_cb.conn[conn_idx].state )
    {
    case LE_CONTROL_STATE_DISCOVER_PRIMARY_SERVICES:
        if ( ( p_result->discovery_type == GATT_DISCOVER_SERVICES_ALL ) ||
            ( p_result->discovery_type == GATT_DISCOVER_SERVICES_BY_UUID ) )
        {
            TRACE_LOG( "Service s:%04x e:%04x uuid:%04x", p_result->discovery_data.group_value.s_handle,
                    p_result->discovery_data.group_value.e_handle, p_result->discovery_data.group_value.service_type.uu.uuid16 );
        }
        break;

    case LE_CONTROL_STATE_DISCOVER_CHARACTERISTICS:
        if ( p_result->discovery_type == GATT_DISCOVER_CHARACTERISTICS )
        {
            TRACE_LOG( "Found LE characteristics - uuid:%04x, hdl:%04x", p_result->discovery_data.characteristic_declaration.char_uuid.uu.uuid16, p_result->discovery_data.characteristic_declaration.handle );
        }
        break;

    case LE_CONTROL_STATE_DISCOVER_DESCRIPTORS:
        if ( p_result->discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS )
        {
            TRACE_LOG( "Found LE descriptors - uuid:%04x handle:%04x", p_result->discovery_data.char_descr_info.type.uu.uuid16, p_result->discovery_data.char_descr_info.handle );
        }
        break;

    default:
        TRACE_WNG( "ignored" );
        break;
    }
    return ( WICED_SUCCESS );
}

/*
 * process discovery complete notification from the stack
 */
wiced_result_t hci_control_le_gatt_disc_comp_cb( wiced_bt_gatt_discovery_complete_t *p_data )
{
    // if we got here peer returned no more services, or we read up to the handle asked by client, report complete
    TRACE_LOG("Start");
    //le_control_cb.conn[p_data->conn_id].state = LE_CONTROL_STATE_IDLE;
    le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_IDLE;
    return ( WICED_SUCCESS );
}

/*
 * Find attribute description by handle
 */
attribute_t * hci_control_get_attribute( uint16_t handle )
{
    TRACE_LOG("Start");
    int i;
    for ( i = 0; i <  sizeof( gauAttributes ) / sizeof( gauAttributes[0] ); i++ )
    {
        if ( gauAttributes[i].handle == handle )
        {
            return ( &gauAttributes[i] );
            }
    }
    TRACE_WNG( "attr not found:%x", handle );
    return NULL;
}

/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t hci_control_le_get_value( uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_t * p_read_data, uint16_t len_requested )
{
    TRACE_LOG("Start");
    attribute_t *puAttribute;
    int          attr_len_to_copy;

    if ( ( puAttribute = hci_control_get_attribute(p_read_data->handle) ) == NULL)
    {
        TRACE_ERR("read_hndlr attr not found hdl:%x\n", p_read_data->handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Dummy battery value read increment */
    if( p_read_data->handle == HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL)
    {
        if ( btheadset_battery_level++ > 5)
        {
            btheadset_battery_level = 0;
        }
    }

    attr_len_to_copy = puAttribute->attr_len;

    TRACE_LOG("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy );

    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }

    if ( attr_len_to_copy != 0 )
    {
        uint8_t *from;
        int      to_copy = MIN(len_requested, attr_len_to_copy - p_read_data->offset);

        from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
        if (to_copy < 0)
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_data->handle, WICED_BT_GATT_INVALID_OFFSET);
            return WICED_BT_GATT_INVALID_OFFSET;
        }

        wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_copy, from, NULL);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is called when peer issues a Read Request to access characteristics values
 * in the GATT database.  Application can fill the provided buffer and return SUCCESS,
 * return error if something not appropriate, or return PENDING and send Read Response
 * when data is ready.
 */

wiced_bt_gatt_status_t hci_control_le_read_handler( uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_t *p_req, uint16_t len_requested)
{
#if defined(STACK_INSIDE_FREE_RTOS) && (STACK_INSIDE_FREE_RTOS == TRUE)
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_req->handle))
    {
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_req);
    }
#endif
    TRACE_LOG("hci_control_le_read_handler\n");
    TRACE_LOG("[handle:%d] [conn_id:%d] [offset:%d] [length:%d] \n",
            p_req->handle, conn_id, p_req->offset, len_requested);

    return  hci_control_le_get_value(conn_id, opcode, p_req, len_requested);
}
/*
 * The function invoked on timeout of hci_control_le_connect_timer
 */
void hci_control_le_connect_timeout( TIMER_PARAM_TYPE arg )
{
    TRACE_LOG("hci_control_le_connect_timeout");
    /* Stop le connection timer*/
    wiced_stop_timer( &hci_control_le_connect_timer );

    /* Cancel connection request */
    wiced_bt_gatt_cancel_connect( hci_control_le_remote_bdaddr, WICED_TRUE );
}
/*
 * This function is called when peer issues a Write request to access characteristics values
 * in the GATT database
 */
wiced_result_t hci_control_le_write_handler( uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_write_req_t *p_req)
{
    TRACE_LOG("hci_control_le_write_handle");
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_req->handle))
    {
        return wiced_ota_fw_upgrade_write_handler(conn_id, p_req);
    }
#endif
    return ( WICED_BT_GATT_SUCCESS );
}

wiced_result_t hci_control_le_write_exec_handler( uint16_t conn_id, wiced_bt_gatt_exec_flag_t flag )
{
    TRACE_LOG("hci_control_le_write_exec_handler:(conn_id:%d)", conn_id);
    return ( WICED_SUCCESS );
}

wiced_result_t hci_control_le_mtu_handler( uint16_t conn_id, uint16_t mtu )
{
    TRACE_LOG("hci_control_le_mtu_handler mtu:%d", mtu);
    le_control_cb.conn[conn_idx].peer_mtu   = mtu;
    wiced_bt_gatt_server_send_mtu_rsp(conn_id, mtu,
            wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);

    return ( WICED_SUCCESS );
}

/*
 * Process indication confirm.
 */
wiced_result_t  hci_control_le_conf_handler( uint16_t conn_id, uint16_t handle )
{
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
    if (wiced_ota_fw_upgrade_is_gatt_handle(handle))
    {
        return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
    }
#endif

    TRACE_LOG( "hci_control_le_conf_handler conn_id:%d state:%d handle:%x", conn_id, le_control_cb.conn[conn_idx].state, handle );

    return WICED_SUCCESS;
}

/*
 * This is a GATT request callback
 */
wiced_bt_gatt_status_t hci_control_le_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_req )
{
    wiced_bt_gatt_status_t result  = WICED_BT_GATT_SUCCESS;
    uint16_t               conn_id = p_req->conn_id;

    TRACE_LOG("hci_control_le_gatt_req_cb opcode:0x%x", p_req->opcode);
    switch ( p_req->opcode)
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
            result = hci_control_le_read_handler( p_req->conn_id, p_req->opcode, &p_req->data.read_req, p_req->len_requested );
            break;

        case GATT_REQ_WRITE:
        case GATT_REQ_PREPARE_WRITE:
            result = hci_control_le_write_handler( p_req->conn_id, p_req->opcode, &(p_req->data.write_req) );
             break;

        case GATT_REQ_EXECUTE_WRITE:
            result = hci_control_le_write_exec_handler( p_req->conn_id, p_req->data.exec_write_req.exec_write );
            break;

        case GATT_REQ_MTU:
            result = hci_control_le_mtu_handler( p_req->conn_id, p_req->data.remote_mtu );
            break;

        case GATT_HANDLE_VALUE_CONF:
            result = hci_control_le_conf_handler( p_req->conn_id, p_req->data.confirm.handle);
            break;

       default:
            TRACE_LOG("Invalid GATT request conn_id:%d type:%d", conn_id, p_req->opcode);
            break;
    }

    return result;
}

wiced_bt_gatt_status_t hci_control_le_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_SUCCESS;

    TRACE_LOG("hci_control_le_gatt_callback event:%d, conn_id:%d", event, p_data->connection_status.conn_id);
    switch( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = hci_control_le_conn_status_callback( &p_data->connection_status );
        break;

    case GATT_OPERATION_CPLT_EVT:
            result = hci_control_le_gatt_operation_comp_cb( &p_data->operation_complete );
        break;

    case GATT_DISCOVERY_RESULT_EVT:
            result = hci_control_le_gatt_disc_result_cb( &p_data->discovery_result );
        break;

    case GATT_DISCOVERY_CPLT_EVT:
            result = hci_control_le_gatt_disc_comp_cb( &p_data->discovery_complete );
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = hci_control_le_gatt_req_cb( &p_data->attribute_request );
        break;

    case GATT_GET_RESPONSE_BUFFER_EVT:
        p_data->buffer_request.buffer.p_app_rsp_buffer = wiced_bt_get_buffer(p_data->buffer_request.len_requested);
        p_data->buffer_request.buffer.p_app_ctxt = wiced_bt_free_buffer;
        result = WICED_BT_GATT_SUCCESS;
        break;

    case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            void (*pfn_free)(uint8_t *) =
                (void (*)(uint8_t *))p_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
                pfn_free(p_data->buffer_xmitted.p_app_data);

            result = WICED_BT_GATT_SUCCESS;
        }
        break;

    default:
        break;
    }

    return result;
}

/*
 * This function sends write to the peer GATT server
 * */
wiced_bt_gatt_status_t hci_control_le_send_write( uint8_t conn_id, uint16_t attr_handle, uint8_t *p_data, uint16_t len, wiced_bt_gatt_opcode_t type )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INSUF_RESOURCE;
    // Allocating a buffer to send the write request
    wiced_bt_gatt_write_hdr_t write;

    {
        write.handle   = attr_handle;
        write.offset   = 0;
        write.len      = len;
        write.auth_req = GATT_AUTH_REQ_NONE;


        // Register with the server to receive notification
        status = wiced_bt_gatt_client_send_write( conn_id, type, &write, p_data, NULL );

        TRACE_LOG( "status:%d", status );
    }

    return ( status );
}

#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
/*
 * Num complete callback
 */
void hci_control_le_num_complete_callback( void )
{
    uint16_t             handle;
    uint16_t             conn_id;
    static int           last_connection_serviced = 0;
    int                  i;

    //TRACE_LOG( "hci_control_le_num_complete_callback available buffs:%d\n", wiced_bt_ble_get_available_tx_buffers( ) );

    if ( hci_control_le_pending_tx_buffer.tx_buf_saved && ( wiced_bt_ble_get_available_tx_buffers( ) > 1 ) )
    {
        TRACE_LOG( "service conn_id:%d", hci_control_le_pending_tx_buffer.tx_buf_conn_id );

        // saved tx buffer can be write command, or notification.
        if ( hci_control_le_pending_tx_buffer.tx_buf_type == HCI_CONTROL_GATT_COMMAND_WRITE_COMMAND )
        {
            if ( hci_control_le_send_write( hci_control_le_pending_tx_buffer.tx_buf_conn_id, hci_control_le_pending_tx_buffer.tx_buf_handle,
                            hci_control_le_pending_tx_buffer.tx_buf_data, hci_control_le_pending_tx_buffer.tx_buf_len, GATT_WRITE_NO_RSP ) != WICED_BT_GATT_SUCCESS )
            {
                    return;
            }
        }
        else if ( hci_control_le_pending_tx_buffer.tx_buf_type == HCI_CONTROL_GATT_COMMAND_NOTIFY )
        {
            if ( wiced_bt_gatt_send_long_notification( hci_control_le_pending_tx_buffer.tx_buf_conn_id, hci_control_le_pending_tx_buffer.tx_buf_handle,
                            hci_control_le_pending_tx_buffer.tx_buf_len, hci_control_le_pending_tx_buffer.tx_buf_data ) != WICED_BT_GATT_SUCCESS )
            {
                    return;
            }
        }
        else
        {
            TRACE_LOG( "bad packet queued:%d", hci_control_le_pending_tx_buffer.tx_buf_type );
        }
        // tx_buffer sent successfully, clear tx_buf_saved flag
        hci_control_le_pending_tx_buffer.tx_buf_saved = WICED_FALSE;
    }
}
#endif
/*
 * Stack runs the advertisement state machine switching between high duty, low
 * duty, no advertisements, based on the wiced_cfg.  All changes are notified
 * through this callback.
 */
void hci_control_le_advert_state_changed( wiced_bt_ble_advert_mode_t mode )
{
    uint8_t hci_control_le_event;

    TRACE_LOG( "Advertisement State Change:%d", mode );

    switch ( mode )
    {
    case BTM_BLE_ADVERT_OFF:
        hci_control_le_event = LE_ADV_STATE_NO_DISCOVERABLE;
        break;
    case BTM_BLE_ADVERT_DIRECTED_HIGH:
    case BTM_BLE_ADVERT_UNDIRECTED_HIGH:
    case BTM_BLE_ADVERT_NONCONN_HIGH:
    case BTM_BLE_ADVERT_DISCOVERABLE_HIGH:
        hci_control_le_event = LE_ADV_STATE_HIGH_DISCOVERABLE;
        break;
    case BTM_BLE_ADVERT_DIRECTED_LOW:
    case BTM_BLE_ADVERT_UNDIRECTED_LOW:
    case BTM_BLE_ADVERT_NONCONN_LOW:
    case BTM_BLE_ADVERT_DISCOVERABLE_LOW:
        hci_control_le_event = LE_ADV_STATE_LOW_DISCOVERABLE;
        break;
    }
    //hci_control_le_send_advertisement_state_event( hci_control_le_event );
}

/*
 * Stack runs the scan state machine switching between high duty, low
 * duty, no scan, based on the wiced_cfg.  All changes are notified
 * through this callback.
 */
void hci_control_le_scan_state_changed( wiced_bt_ble_scan_type_t state )
{
}

/*
 * This function is called when notification is received from the connected GATT Server
 */
void hci_control_le_notification_handler( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len )
{
    TRACE_LOG( "Notification conn_id:%d handle:%04x len:%d", conn_id, handle, len );
}

/*
 * This function is called when indication is received from the connected GATT Server
 */
void hci_control_le_indication_handler( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len )
{
    TRACE_LOG( "Indication conn_id:%d handle:%04x len:%d", conn_id, handle, len );

        uint8_t *p_n_data = wiced_bt_get_buffer(len);
        if (p_n_data) {
            WICED_MEMCPY(p_n_data, p_data, len);
            wiced_bt_gatt_server_send_indication(conn_id, handle, len, p_n_data, NULL);
        }

}
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
void hci_control_le_indication_confirmation_handler( uint16_t conn_id, uint16_t handle )
{
    TRACE_LOG("INDICATION CONFIRMATION");
    wiced_bt_gatt_client_send_indication_confirm( conn_id, handle );
}

static void hci_control_le_handle_gatt_db_init(uint8_t* data, uint32_t length,  wiced_bt_db_hash_t hash)
{
    wiced_bt_gatt_status_t gatt_status;

    TRACE_LOG("Initializing GATT database...");

    gatt_status = wiced_bt_gatt_db_init(data, length, hash);
    if( gatt_status != WICED_BT_GATT_SUCCESS )
    {
        TRACE_ERR("Error initializing GATT callback...error: %d", gatt_status);
        return;
    }
}
#endif
