/*
 * $ Copyright 2023-YEAR Infineon Semiconductor $
 */

/** @file
 *
 * Runtime Bluetooth stack configuration parameters
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_sdp.h"

/* SDP Record for Serial Port */
#define HDLR_SERIAL_BASE     0x010000
#define SDP_PORT_DEF_LEN     72

/* SDP Record for Device ID */
#define HDLR_DEVICE_ID       0x010003

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/

const wiced_bt_cfg_ble_scan_settings_t ble_scan_cfg = {
    .scan_mode = BTM_BLE_SCAN_MODE_ACTIVE,                                                               /**< BLE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE) */

    /* Advertisement scan configuration */
    .high_duty_scan_interval = 96,                                                                       /**< High duty scan interval */
    .high_duty_scan_window = 48,                                                                         /**< High duty scan window */
    .high_duty_scan_duration = 30,                                                                       /**< High duty scan duration in seconds (0 for infinite) */

    .low_duty_scan_interval = 2048,                                                                      /**< Low duty scan interval  */
    .low_duty_scan_window = 48,                                                                          /**< Low duty scan window */
    .low_duty_scan_duration = 30,                                                                        /**< Low duty scan duration in seconds (0 for infinite) */

    /* Connection scan configuration */
    .high_duty_conn_scan_interval = 96,                                                                  /**< High duty cycle connection scan interval */
    .high_duty_conn_scan_window = 48,                                                                    /**< High duty cycle connection scan window */
    .high_duty_conn_duration = 30,                                                                       /**< High duty cycle connection duration in seconds (0 for infinite) */

    .low_duty_conn_scan_interval = 2048,                                                                 /**< Low duty cycle connection scan interval */
    .low_duty_conn_scan_window = 48,                                                                     /**< Low duty cycle connection scan window */
    .low_duty_conn_duration = 30,                                                                        /**< Low duty cycle connection duration in seconds (0 for infinite) */

    /* Connection configuration */
    .conn_min_interval = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,                                         /**< Minimum connection interval */
    .conn_max_interval = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,                                         /**< Maximum connection interval */
    .conn_latency = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                                                   /**< Connection latency */
    .conn_supervision_timeout = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,                           /**< Connection link supervision timeout */
};

const wiced_bt_cfg_ble_advert_settings_t ble_advert_cfg = {
    .channel_map = BTM_BLE_ADVERT_CHNL_37 |                                                              /**< Advertising channel map (mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39) */
                   BTM_BLE_ADVERT_CHNL_38 |
                   BTM_BLE_ADVERT_CHNL_39,

    .high_duty_min_interval = 160,                                                                       /**< High duty undirected connectable minimum advertising interval */
    .high_duty_max_interval = 160,                                                                       /**< High duty undirected connectable maximum advertising interval */
    .high_duty_duration = 0,                                                                             /**< High duty undirected connectable advertising duration in seconds (0 for infinite) */

    .low_duty_min_interval = 400,                                                                        /**< Low duty undirected connectable minimum advertising interval */
    .low_duty_max_interval = 400,                                                                        /**< Low duty undirected connectable maximum advertising interval */
    .low_duty_duration = 0,                                                                              /**< Low duty undirected connectable advertising duration in seconds (0 for infinite) */

    .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,         /**< High duty directed connectable minimum advertising interval */
    .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,         /**< High duty directed connectable maximum advertising interval */

    .low_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,           /**< Low duty directed connectable minimum advertising interval */
    .low_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,           /**< Low duty directed connectable maximum advertising interval */
    .low_duty_directed_duration = 30,                                                                    /**< Low duty directed connectable advertising duration in seconds (0 for infinite) */

    .high_duty_nonconn_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,           /**< High duty non-connectable minimum advertising interval */
    .high_duty_nonconn_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,           /**< High duty non-connectable maximum advertising interval */
    .high_duty_nonconn_duration = 30,                                                                    /**< High duty non-connectable advertising duration in seconds (0 for infinite) */

    .low_duty_nonconn_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,             /**< Low duty non-connectable minimum advertising interval */
    .low_duty_nonconn_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,             /**< Low duty non-connectable maximum advertising interval */
    .low_duty_nonconn_duration = 0                                                                       /**< Low duty non-connectable advertising duration in seconds (0 for infinite) */
};

const wiced_bt_cfg_ble_t ble_cfg = {
    .ble_max_simultaneous_links = 5,                                                                     /**< Max number for simultaneous connections for a layer, profile, protocol */
    .ble_max_rx_pdu_size = 80,                                                                           /**< Max pdu size of the connections (size of data to be received/transmitted) */
    .rpa_refresh_timeout = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,                             /**< Interval of  random address refreshing - secs  @note BLE Privacy is disabled if the value is 0. */
    .host_addr_resolution_db_size = 5,                                                                   /**< addr resolution db size */

    .p_ble_scan_cfg = &ble_scan_cfg,                                                                     /**< BLE scan settings */
    .p_ble_advert_cfg = &ble_advert_cfg,                                                                 /**< BLE advertisement settings */
};

const wiced_bt_cfg_br_t br_cfg = {
    .br_max_simultaneous_links = 5,                                                                      /**< Max number for simultaneous connections for a layer, profile, protocol */
    .br_max_rx_pdu_size = 1024,                                                                          /**< Max pdu size of the connections (size of data to be received/transmitted) */
    .device_class = {0x10, 0x01, 0x0C},                                                                  /**< Local device class */
    .rfcomm_cfg =                                                                                        /**< RFCOMM configuration */
    {
        .max_links = 7,                                                                                  /**< Maximum number of simultaneous connected remote devices*/
        .max_ports = 7                                                                                   /**< Maximum number of simultaneous RFCOMM ports */
    },
};

const wiced_bt_cfg_gatt_t gatt_cfg =                                                                     /**< GATT configuration */
{
    .max_db_service_modules = 0,                                                                         /**< Maximum number of service modules in the DB*/
    .max_eatt_bearers = 0,                                                                               /**< Maximum number of allowed gatt bearers */
};

const wiced_bt_cfg_l2cap_application_t l2cap_application =                                               /* Application managed l2cap protocol configuration */
{
    /* BR EDR l2cap configuration */
    .max_app_l2cap_psms = 0,                                                                             /**< Maximum number of application-managed BR/EDR PSMs */
    .max_app_l2cap_channels = 12,                                                                        /**< Maximum number of application-managed BR/EDR channels  */

    .max_app_l2cap_br_edr_ertm_chnls = 2,                                                                /**< Maximum ERTM channels  */
    .max_app_l2cap_br_edr_ertm_tx_win = 1,                                                               /**< Maximum TX Window      */

    .max_app_l2cap_le_fixed_channels = 0,                                                                /**< Maximum RX MTU allowed */
};

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
#define WICED_DEVICE_NAME   "SPP Server"
const wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    .device_name = (uint8_t*)WICED_DEVICE_NAME,                                                          /**< Local device name (NULL terminated) */
    .security_required = (BTM_SEC_BEST_EFFORT),                                                          /**< Security requirements mask (see #wiced_bt_sec_level_e)) */
    .p_br_cfg = &br_cfg,
    .p_ble_cfg = &ble_cfg,
    .p_gatt_cfg = &gatt_cfg,
    .p_l2cap_app_cfg = &l2cap_application,
};

/*****************************************************************************
* SDP Database
 ****************************************************************************/
static uint8_t* sdp_database = NULL;
static uint16_t sdp_database_size = 0;

static uint8_t sdp_device[] = /* 71 */
{
    SDP_ATTR_SEQUENCE_1(69),
        SDP_ATTR_ID(ATTR_ID_SERVICE_RECORD_HDL),
        SDP_ATTR_VALUE_UINT4(HDLR_DEVICE_ID),                                   /* Record Handle */

        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
        SDP_ATTR_SEQUENCE_1(3),                                                 /* Class ID List */
        SDP_ATTR_UUID16(UUID_SERVCLASS_PNP_INFORMATION),                        /* Class ID */

        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST),
        SDP_ATTR_SEQUENCE_1(13),                                                /* Protocol Descriptor List */
        SDP_ATTR_SEQUENCE_1(6),                                                 /* Protocol Descriptor 0 */
        SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),                                   /* Protocol ID */
        SDP_ATTR_VALUE_UINT2(0x01),                                             /* Parameter 0 */
        SDP_ATTR_SEQUENCE_1(3),                                                 /* Protocol Descriptor 1 */
        SDP_ATTR_UUID16(0x01),                                                  /* L2CAP Channel */

        SDP_ATTR_ID(ATTR_ID_SPECIFICATION_ID), SDP_ATTR_VALUE_UINT2(0x0103),    /* Specification ID */
        SDP_ATTR_ID(ATTR_ID_VENDOR_ID), SDP_ATTR_VALUE_UINT2(0x0F),             /* Vendor ID */
        SDP_ATTR_ID(ATTR_ID_PRODUCT_ID), SDP_ATTR_VALUE_UINT2(0x0401),          /* Product ID */
        SDP_ATTR_ID(ATTR_ID_PRODUCT_VERSION), SDP_ATTR_VALUE_UINT2(0x01),       /* Version */
        SDP_ATTR_ID(ATTR_ID_PRIMARY_RECORD), SDP_ATTR_VALUE_BOOLEAN(0x01),      /* Primary Record */
        SDP_ATTR_ID(ATTR_ID_VENDOR_ID_SOURCE), SDP_ATTR_VALUE_UINT2(0x00),      /* Vendor ID Source */
};

/*
 * wiced_app_cfg_sdp_record_get_size
 */
uint16_t wiced_app_cfg_sdp_record_get_size(void)
{
    return sdp_database_size;
}

/*
 * wiced_get_sdp_database: given number of ports, get sdp database
 */
uint8_t* wiced_get_sdp_database(uint8_t num_ports)
{
    if(sdp_database)
    {
        free(sdp_database);
    }

    sdp_database = calloc(2 + num_ports * SDP_PORT_DEF_LEN + sizeof(sdp_device), 1);
    if(!sdp_database)
    {
        return NULL;
    }

    sdp_database_size = 2 + num_ports * SDP_PORT_DEF_LEN + sizeof(sdp_device);

    sdp_database[0] = ((DATA_ELE_SEQ_DESC_TYPE << 3) | SIZE_IN_NEXT_BYTE);
    sdp_database[1] = num_ports * SDP_PORT_DEF_LEN + sizeof(sdp_device);

    for(int i = 0; i < num_ports; i++)
    {
        /* 2 ~ 61 RFCOMM */
        uint8_t sdp_port_temp[] = /* 72 */
        {
            SDP_ATTR_SEQUENCE_1(70),
                SDP_ATTR_ID(ATTR_ID_SERVICE_RECORD_HDL),
                SDP_ATTR_VALUE_UINT4(HDLR_SERIAL_BASE + i + 1),                         /* Record Handle */

                SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
                SDP_ATTR_SEQUENCE_1(3),                                                 /* Class ID List */
                SDP_ATTR_UUID16(UUID_SERVCLASS_SERIAL_PORT),                            /* Service Class 0 */

                SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST),
                SDP_ATTR_SEQUENCE_1(12),                                                /* Protocol Descriptor List */
                SDP_ATTR_SEQUENCE_1(3),                                                 /* Protocol Descriptor 0 */
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),                                   /* Protocol ID */
                SDP_ATTR_SEQUENCE_1(5),                                                 /* Protocol Descriptor 1 */
                SDP_ATTR_UUID16(UUID_PROTOCOL_RFCOMM),                                  /* Protocol ID */
                SDP_ATTR_VALUE_UINT1(2 + i),                                            /* Parameter 0 */

                SDP_ATTR_ID(ATTR_ID_BROWSE_GROUP_LIST),
                SDP_ATTR_SEQUENCE_1(3),                                                 /* Browse Group List */
                SDP_ATTR_UUID16(UUID_SERVCLASS_PUBLIC_BROWSE_GROUP),                    /* Public Browse Root */

                SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST),
                SDP_ATTR_SEQUENCE_1(8),                                                 /* Bluetooth Profile Descriptor List */
                SDP_ATTR_SEQUENCE_1(6),                                                 /* Profile Descriptor 0 */
                SDP_ATTR_UUID16(UUID_SERVCLASS_SERIAL_PORT),                            /* Profile */
                SDP_ATTR_VALUE_UINT2(0x0102),                                           /* Parameter for Profile: Version */

                SDP_ATTR_ID(ATTR_ID_SERVICE_NAME), SDP_ATTR_VALUE_TEXT_1(11), 'S', 'P', 'P', ' ', 'S', 'E', 'R', 'V', 'E', 'R', '0' + (i % 10) + 1 ,
        };
        memcpy(sdp_database + 2 + i * SDP_PORT_DEF_LEN, sdp_port_temp, SDP_PORT_DEF_LEN);
    }
    memcpy(sdp_database + 2 + num_ports * SDP_PORT_DEF_LEN, sdp_device, sizeof(sdp_device));
    sdp_database[2 + num_ports * SDP_PORT_DEF_LEN + 9] = HDLR_SERIAL_BASE + num_ports + 1;

    return sdp_database;
}
