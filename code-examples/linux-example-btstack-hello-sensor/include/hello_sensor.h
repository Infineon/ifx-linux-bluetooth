/******************************************************************************
* $ Copyright 2022-YEAR Cypress Semiconductor $
******************************************************************************/
/******************************************************************************
 * File Name: hello_sensor.h
 *
 * Description: Definitions for constants used in the device's GATT server
 * application and function prototypes.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/
/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#ifndef _HELLO_SENSOR_H_
#define _HELLO_SENSOR_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <pthread.h>
#include <ctype.h>
#include <string.h>
#include <sys/time.h>
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "wiced_bt_dev.h"
#include "app_bt_utils.h"
#include "wiced_bt_types.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_ble.h"

#include "wiced_hal_nvram.h"

#include "wiced_bt_types.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "wiced_result.h"
#include "wiced_timer.h"



/******************************************************************************
 *                                MACROS
 *****************************************************************************/
#define HELLO_SENSOR_TIMEOUT    (10u)
#define DONT_FREE_BUFFER (3u)
#define CONN_INTERVAL_MULTIPLIER (1.25f)

#define CY_BT_ADV_PACKET_DATA_SIZE (3)

/* Connection configuration */
#define CY_BT_CONN_LATENCY WICED_BT_CFG_DEFAULT_CONN_LATENCY
#define CY_BT_CONN_SUPERVISION_TIMEOUT WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT
#define CY_BT_MTU_SIZE 247
/* Application buffer Size */
#define APP_BUFFER_SIZE (1024)
/* Advertisement packet length */
#define ADV_ELEM_1_SIZE (1)
#define ADV_ELEM_2_SIZE (4)
#define ADV_ELEM_3_SIZE (16)
/* Default buffer value */
#define APP_BUFFER_INIT_VALUE (0x01)

#define wiced_bt_gatt_send_notification(id, type, len, ptr) wiced_bt_gatt_server_send_notification(id, type, len, ptr, NULL)
#define wiced_bt_gatt_send_indication(id, type, len, ptr)   wiced_bt_gatt_server_send_indication(id, type, len, ptr, NULL)


#ifndef PACKED
#define PACKED
#endif


/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define HELLO_SENSOR_MAX_NUM_CLIENTS 1

/* Hello Sensor App Timer Timeout in seconds  */
#define HELLO_SENSOR_APP_TIMEOUT_IN_SECONDS                 1

/* Hello Sensor Connection Idle  Timeout in milli seconds  */
#define HELLO_SENSOR_CONN_IDLE_TIMEOUT_IN_SECONDS           3

#define HELLO_SENSOR_VS_ID                      WICED_NVRAM_VSID_START
#define HELLO_SENSOR_LOCAL_KEYS_VS_ID          ( WICED_NVRAM_VSID_START + 1 )
#define HELLO_SENSOR_PAIRED_KEYS_VS_ID          ( WICED_NVRAM_VSID_START + 2 )

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/
typedef enum
{
    HANDLE_HSENS_GATT_SERVICE = 0x1, /* service handle */

    HANDLE_HSENS_GAP_SERVICE = 0x14, /* service handle */
        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME, /* characteristic handl */
        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL, /* char value handle */

        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE, /* characteristic handl */
        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,/* char value handle */


    HANDLE_HSENS_SERVICE = 0x28,
        HANDLE_HSENS_SERVICE_CHAR_NOTIFY, /* characteristic handl */
        HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL, /* char value handle */
        HANDLE_HSENS_SERVICE_CHAR_CFG_DESC, /* charconfig desc handl */

        HANDLE_HSENS_SERVICE_CHAR_BLINK, /* characteristic handl */
        HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL, /* char value handle */

        HANDLE_HSENS_SERVICE_CHAR_LONG_MSG, /* characteristic handl */
        HANDLE_HSENS_SERVICE_CHAR_LONG_MSG_VAL, /*long  char value handle */

    HANDLE_HSENS_DEV_INFO_SERVICE = 0x40,
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME, /* characteristic handle */
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,/* char value handle */

        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM, /* characteristic handl */
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,/* char value handle */

        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, /* characteristic handl */
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,/* char value handle */

    HANDLE_HSENS_BATTERY_SERVICE = 0x60, /* service handle */
        HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL, /* characteristic handl */
        HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL, /* char value andle */

    /* Client Configuration */
    HDLD_CURRENT_TIME_SERVICE_CURRENT_TIME_CLIENT_CONFIGURATION,
}hello_sensor_db_tags;



/******************************************************************************
 *                      STRUCTURES AND ENUMERATIONS
 ******************************************************************************/
/* This enumeration combines the advertising, connection states from two different
 * callbacks to maintain the status in a single state variable */
typedef enum
{
    APP_BT_ADV_OFF_CONN_OFF,
    APP_BT_ADV_ON_CONN_OFF,
    APP_BT_ADV_OFF_CONN_ON
} app_bt_adv_conn_mode_t;

typedef struct
{
    /* remote peer device address */
    wiced_bt_device_address_t remote_addr;
    /* connection ID referenced by the stack */
    uint16_t conn_id;
    /* MTU exchanged after connection */
    uint16_t mtu;
    /* connection interval negotiated */
    float conn_interval;
    /* RX PHY selected */
    wiced_bt_ble_host_phy_preferences_t rx_phy;
    /* TX PHY selected */
    wiced_bt_ble_host_phy_preferences_t tx_phy;

} conn_state_info_t;

/******************************************************************************
 *                                Structures
 ******************************************************************************/
typedef struct
{
    wiced_bt_device_address_t remote_addr;              /* remote peer device address */
    uint32_t  timer_count;              /* timer count */
    uint16_t  conn_id;                  /* connection ID referenced by the stack */
    uint16_t  peer_mtu;                 /* peer MTU */
    uint8_t   num_to_write;             /* num msgs to send, incr on each button intr */
    uint8_t   flag_indication_sent;     /* indicates waiting for ack/cfm */
    uint8_t   flag_stay_connected;      /* stay connected or disconnect after all messages are sent */
    uint8_t   battery_level;            /* dummy battery level */

} hello_sensor_state_t;

#pragma pack(1)
/* Host information saved in  NVRAM */
typedef PACKED struct
{
    wiced_bt_device_address_t  bdaddr;                                /* BD address of the bonded host */
    uint16_t  characteristic_client_configuration;  /* Current value of the client configuration descriptor */
    uint8_t   number;                               /* Sensor config, number: this is printed on console */
} host_info_t;
#pragma pack()

typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void     *p_attr;
} attribute_t;


typedef void (*tpt_free_buffer_t)(uint8_t *);

extern wiced_bt_ble_advert_elem_t cy_bt_adv_packet_data[];



/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/
/* Callback function for Bluetooth stack management type events */
wiced_bt_dev_status_t hello_sensor_management_callback(
    wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t
                                         *p_event_data);

typedef void (*pfn_free_buffer_t)(uint8_t *);
void hello_sensor_gatt_init( void );
void hello_sensor_interrupt_handler( void );

#endif
