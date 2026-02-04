/******************************************************************************
* $ Copyright 2022-YEAR Cypress Semiconductor $
******************************************************************************/
/******************************************************************************
 * File Name: wiced_bt_cfg.c
 *
 * Description: Runtime Bluetooth stack configuration parameters.
 *
 * Related Document: See README.md
 *
 *******************************************************************************/
/******************************************************************************
 *                                INCLUDES
 ******************************************************************************/
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"

#ifndef DEV_NAME
#define DEV_NAME "Hello"
#endif

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/

/* BLE SCAN Setting */
const wiced_bt_cfg_ble_scan_settings_t hello_sensor_cfg_scan_settings =
{
    .scan_mode                  = BTM_BLE_SCAN_MODE_ACTIVE,                                  /**< BLE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE) */

    /* Advertisement scan configuration */
    .high_duty_scan_interval    = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL, /**< High duty scan interval */
    .high_duty_scan_window      = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,     /**< High duty scan window */
    .high_duty_scan_duration    = 5,                                            /**< High duty scan duration in seconds ( 0 for infinite ) */

    .low_duty_scan_interval     = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_INTERVAL, /**< Low duty scan interval  */
    .low_duty_scan_window       = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_WINDOW,     /**< Low duty scan window */
    .low_duty_scan_duration     = 5,                                           /**< Low duty scan duration in seconds ( 0 for infinite ) */

    /* Connection scan configuration */
    .high_duty_conn_scan_interval   = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL, /**< High duty cycle connection scan interval */
    .high_duty_conn_scan_window     = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_WINDOW,   /**< High duty cycle connection scan window */
    .high_duty_conn_duration        = 30,                                                   /**< High duty cycle connection duration in seconds ( 0 for infinite ) */

    .low_duty_conn_scan_interval    = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_INTERVAL, /**< Low duty cycle connection scan interval */
    .low_duty_conn_scan_window      = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_WINDOW,     /**< Low duty cycle connection scan window */
    .low_duty_conn_duration         = 30,                                                    /**< Low duty cycle connection duration in seconds ( 0 for infinite ) */

    /* Connection configuration */
    .conn_min_interval          = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,               /**< Minimum connection interval */
    .conn_max_interval          = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,               /**< Maximum connection interval */
    .conn_latency               = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                         /**< Connection latency */
    .conn_supervision_timeout   = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT, /**< Connection link supervision timeout */
};

const wiced_bt_cfg_ble_advert_settings_t hello_sensor_cfg_adv_settings =
{
    .channel_map = BTM_BLE_ADVERT_CHNL_37 | /**< Advertising channel map ( mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39 ) */
                   BTM_BLE_ADVERT_CHNL_38 |
                   BTM_BLE_ADVERT_CHNL_39,

    .high_duty_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,  /**< High duty undirected connectable minimum advertising interval */
    .high_duty_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,  /**< High duty undirected connectable maximum advertising interval */
    .high_duty_duration     = 0,                                                /**< High duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

    .low_duty_min_interval  = 1024,     /**< Low duty undirected connectable minimum advertising interval */
    .low_duty_max_interval  = 1024,     /**< Low duty undirected connectable maximum advertising interval */
    .low_duty_duration      = 0,        /**< Low duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

    .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL, /**< High duty directed connectable minimum advertising interval */
    .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL, /**< High duty directed connectable maximum advertising interval */

    .low_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL, /**< Low duty directed connectable minimum advertising interval */
    .low_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL, /**< Low duty directed connectable maximum advertising interval */
    .low_duty_directed_duration     = 0,                                                       /**< Low duty directed connectable advertising duration in seconds ( 0 for infinite ) */

    .high_duty_nonconn_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL, /**< High duty non-connectable minimum advertising interval */
    .high_duty_nonconn_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL, /**< High duty non-connectable maximum advertising interval */
    .high_duty_nonconn_duration     = 0,                                                          /**< High duty non-connectable advertising duration in seconds ( 0 for infinite ) */

    .low_duty_nonconn_min_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL, /**< Low duty non-connectable minimum advertising interval */
    .low_duty_nonconn_max_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL, /**< Low duty non-connectable maximum advertising interval */
    .low_duty_nonconn_duration      = 0                                                           /**< Low duty non-connectable advertising duration in seconds ( 0 for infinite ) */
};

wiced_bt_cfg_ble_t hello_sensor_cfg_ble = {
    .ble_max_simultaneous_links     = 1,
    .ble_max_rx_pdu_size            = 65,

    .p_ble_scan_cfg                 = &hello_sensor_cfg_scan_settings, /**< */
    .p_ble_advert_cfg               = &hello_sensor_cfg_adv_settings,
    .appearance                     = APPEARANCE_GENERIC_TAG, /**< GATT appearance (see gatt_appearance_e) */

    .host_addr_resolution_db_size   = 5, /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/
    .rpa_refresh_timeout            = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE, /**< Interval of  random address refreshing - secs */
};

wiced_bt_cfg_gatt_t hello_sensor_cfg_gatt = {
    .max_db_service_modules = 0, /**< Maximum number of service modules in the DB*/
    .max_eatt_bearers       = 0,       /**< Maximum number of allowed gatt bearers */
};

wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    .device_name    = (uint8_t*)DEV_NAME,            /**< Local device name (NULL terminated). Use same as configurator generated string.*/
    .p_ble_cfg      = &hello_sensor_cfg_ble,
    .p_gatt_cfg     = &hello_sensor_cfg_gatt
};

