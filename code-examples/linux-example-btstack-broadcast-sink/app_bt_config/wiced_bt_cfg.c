/*
 * $ Copyright Cypress Semiconductor $
 */

/* Application includes */
// #include "unicast_source_bt_manager.h"
// #include "unicast_source_gatt.h"
// #include "unicast_source_rpc.h"
// #include "unicast_source_mcs.h"

/* App Library includes */
//#include "le_audio_rpc.h"

/* BT Stack includes */
#include "wiced_bt_ble.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_stack.h"


/*******************************************************************************
*                                   MACROS
*******************************************************************************/
#define WICED_DEVICE_NAME       "Broadcast Sink"

wiced_bt_cfg_ble_scan_settings_t broadcast_sink_scan_settings = {
    .scan_mode = BTM_BLE_SCAN_MODE_PASSIVE,

    /* Advertisement scan configuration */
    .high_duty_scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,
    .high_duty_scan_window = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,
    .high_duty_scan_duration = 5,

    .low_duty_scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,
    .low_duty_scan_window = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,
    .low_duty_scan_duration = 5,

    /* Connection scan configuration */
    .high_duty_conn_scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL,
    .high_duty_conn_scan_window = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_WINDOW,
    .high_duty_conn_duration = 30,

    .low_duty_conn_scan_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_INTERVAL,
    .low_duty_conn_scan_window = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_WINDOW,
    .low_duty_conn_duration = 30,

    /* Connection configuration */
    .conn_min_interval = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,
    .conn_max_interval = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,
    .conn_latency = WICED_BT_CFG_DEFAULT_CONN_LATENCY,
    .conn_supervision_timeout = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,
};

const wiced_bt_cfg_ble_advert_settings_t broadcast_sink_adv_settings = {
    .channel_map = BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39,

    .high_duty_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,
    .high_duty_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,
    .high_duty_duration = 30,

    .low_duty_min_interval = 1024,
    .low_duty_max_interval = 1024,
    .low_duty_duration = 60,

    .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,
    .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,

    .low_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,
    .low_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,
    .low_duty_directed_duration = 30,

    .high_duty_nonconn_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,
    .high_duty_nonconn_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,
    .high_duty_nonconn_duration = 30,

    .low_duty_nonconn_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,
    .low_duty_nonconn_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,
    .low_duty_nonconn_duration = 0};

wiced_bt_cfg_ble_t broadcast_sink_ble_cfg = {
    .ble_max_simultaneous_links = 4,
    .ble_max_rx_pdu_size = 65,

    .p_ble_scan_cfg = &broadcast_sink_scan_settings,
    .p_ble_advert_cfg = &broadcast_sink_adv_settings,
    .appearance = APPEARANCE_GENERIC_TAG,

    .host_addr_resolution_db_size = 5,
    .rpa_refresh_timeout = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,
};

wiced_bt_cfg_isoc_t broadcast_sink_isoc_cfg = {
    .max_sdu_size = 155*2, /* for 48kz */
    .channel_count = 2,

    .max_cig_count = 0,
    .max_cis_conn = 0,

    .max_big_count = 1,

    .max_buffers_per_cis = 4,
};

wiced_bt_cfg_gatt_t broadcast_sink_gatt_cfg = {
    .max_db_service_modules = 0,
    .max_eatt_bearers = 0,
};

wiced_bt_cfg_settings_t broadcast_sink_cfg_settings = {.device_name = (uint8_t *) WICED_DEVICE_NAME,
                                                       .p_ble_cfg = &broadcast_sink_ble_cfg,
                                                       .p_gatt_cfg = &broadcast_sink_gatt_cfg,
                                                       .p_isoc_cfg = &broadcast_sink_isoc_cfg};
