/******************************************************************************
* $ Copyright 2022-YEAR Cypress Semiconductor $
******************************************************************************/
/******************************************************************************
 * File Name: hello_sensor_gatt_cfg.c
 *
 * Description: Hello Sensor GATT DB CFG.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif

#include "hello_sensor.h"
#include "hello_uuid.h"
#include "hello_sensor_gatt_cfg.h"


/******************************************************************************
 *                                GLOBAL VARIABLES
 ******************************************************************************/

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define HELLO_SENSOR_GATTS_MAX_CONN     1

#define APP_ADV_NAME wiced_bt_cfg_settings.device_name

#ifndef DEV_NAME
#define DEV_NAME "Hello"
#endif

#define WICED_BT_TRACE printf

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************************************
 *                                Variables Definitions
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

const uint8_t hello_sensor_gatt_database[]=
{
    /* Declare mandatory GATT service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GATT_SERVICE, UUID_SERVICE_GATT ),

    /* Declare mandatory GAP service. Device Name and Appearance are mandatory */
    /* characteristics of GAP service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GAP_SERVICE, UUID_SERVICE_GAP ),

    /* Declare mandatory GAP service characteristic: Dev Name */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,
            UUID_CHARACTERISTIC_DEVICE_NAME, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    /* Declare mandatory GAP service characteristic: Appearance */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
            UUID_CHARACTERISTIC_APPEARANCE, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    /* Declare proprietary Hello Service with 128 byte UUID */
    PRIMARY_SERVICE_UUID128( HANDLE_HSENS_SERVICE, UUID_HELLO_SERVICE ),

    /* Declare characteristic used to notify/indicate change  */
        CHARACTERISTIC_UUID128( HANDLE_HSENS_SERVICE_CHAR_NOTIFY, HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,
            UUID_HELLO_CHARACTERISTIC_NOTIFY, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY | GATTDB_CHAR_PROP_INDICATE, GATTDB_PERM_READABLE ),

    /* Declare client characteristic configuration descriptor
       Value of the descriptor can be modified by the client
       Value modified shall be retained during connection and across connection
       for bonded devices.  Setting value to 1 tells this application to send notification
       when value of the characteristic changes.  Value 2 is to allow indications. */
            CHAR_DESCRIPTOR_UUID16_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_CFG_DESC, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
            GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_AUTH_WRITABLE),

        /* Declare characteristic Hello Configuration
         The configuration consists of 1 bytes which indicates how many times to
          Print the Value when user presses a key. */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_BLINK, HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,
            UUID_HELLO_CHARACTERISTIC_CONFIG, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE,
            GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ ),

    /* Declare Device info service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE, UUID_SERVICE_DEVICE_INFORMATION ),

    /* Handle 0x4e: characteristic Manufacturer Name, handle 0x4f characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
            UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    /* Handle 0x50: characteristic Model Number, handle 0x51 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
            UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    /* Handle 0x52: characteristic System ID, handle 0x53 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
            UUID_CHARACTERISTIC_SYSTEM_ID, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    /* Declare Battery service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_BATTERY_SERVICE, 0x180F ),

    /* Handle 0x62: characteristic Battery Level, handle 0x63 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL, HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,
            UUID_CHARACTERISTIC_BATTERY_LEVEL, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),
};

uint16_t hello_sensor_gatt_database_size = sizeof(hello_sensor_gatt_database);

uint8_t hello_sensor_device_name[]          = DEV_NAME;  /* GAP Service characteristic Device Name */
uint8_t hello_sensor_appearance_name[2]     = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };
char    hello_sensor_char_notify_value[]    = { 'H', 'e', 'l', 'l', 'o', ' ', '0' }; /* Notification Name */
char    hello_sensor_char_mfr_name_value[]  = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0, };
char    hello_sensor_char_model_num_value[] = { '1', '2', '3', '4',   0,   0,   0,   0 };
uint8_t hello_sensor_char_system_id_value[] = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71};
wiced_bt_device_address_t bt_device_address = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };

uint8_t hello_sensor_char_notify_value_len = sizeof(hello_sensor_char_notify_value);
uint8_t hello_sensor_char_mfr_name_value_len  = sizeof(hello_sensor_char_mfr_name_value);

/* Holds the global state of the hello sensor application */
hello_sensor_state_t hello_sensor_state;

/* Holds the host info saved in the NVRAM */
host_info_t hello_sensor_hostinfo;

//static conn_state_info_t conn_state_info;

/* Attribute list of the hello sensor */
attribute_t gauAttributes[] =
{
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,       sizeof(hello_sensor_device_name),           hello_sensor_device_name },
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL, sizeof(hello_sensor_appearance_name),       hello_sensor_appearance_name },
    { HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,             sizeof(hello_sensor_char_notify_value),     hello_sensor_char_notify_value },
    { HANDLE_HSENS_SERVICE_CHAR_CFG_DESC,               2,                                          (void*)&hello_sensor_hostinfo.characteristic_client_configuration },
    { HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,              1,                                          &hello_sensor_hostinfo.number },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,  sizeof(hello_sensor_char_mfr_name_value),   hello_sensor_char_mfr_name_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL, sizeof(hello_sensor_char_model_num_value),  hello_sensor_char_model_num_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, sizeof(hello_sensor_char_system_id_value),  hello_sensor_char_system_id_value },
    { HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,      1,                                          &hello_sensor_state.battery_level },
};
uint16_t gauAttributes_size = sizeof(gauAttributes) / sizeof(gauAttributes[0]);

/* [] END OF FILE */
