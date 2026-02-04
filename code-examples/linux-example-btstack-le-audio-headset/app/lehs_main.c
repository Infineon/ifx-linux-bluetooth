/*
 * $ Copyright Cypress Semiconductor $
 */
#ifndef LINUX_PLATFORM
/* Application includes */
#include "lehs.h"
#if defined(WIN32) && defined(CLI_SUPPORT)
#include "windows.h"
static DWORD __stdcall cmdLineThread(VOID* p);
#endif
extern int get_spy_instance(void);

wiced_bt_heap_t *p_lea_default_heap = NULL;
wiced_bt_device_address_t local_bda;

wiced_bt_cfg_ble_scan_settings_t lehs_scan_settings = {
    .scan_mode = BTM_BLE_SCAN_MODE_ACTIVE,

    /* Advertisement scan configuration */
    .high_duty_scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,
    .high_duty_scan_window = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,
    .high_duty_scan_duration = 5,

    .low_duty_scan_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_INTERVAL,
    .low_duty_scan_window = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_WINDOW,
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

const wiced_bt_cfg_ble_advert_settings_t lehs_adv_settings = {
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

wiced_bt_cfg_ble_t lehs_ble_cfg = {
    .ble_max_simultaneous_links = 4,
    .ble_max_rx_pdu_size = 512,

    .p_ble_scan_cfg = &lehs_scan_settings,
    .p_ble_advert_cfg = &lehs_adv_settings,
    .appearance = APPEARANCE_GENERIC_TAG,

    .host_addr_resolution_db_size = 5,
    .rpa_refresh_timeout = 0,
};

wiced_bt_cfg_gatt_t lehs_gatt_cfg = {
    .max_db_service_modules = 0,
    .max_eatt_bearers = 0,
};

wiced_bt_cfg_settings_t lehs_cfg_settings = {.device_name = (uint8_t *)DEVICE_NAME,
                                                     .p_ble_cfg = &lehs_ble_cfg,
                                                     .p_gatt_cfg = &lehs_gatt_cfg};

#define BT_STACK_HEAP_SIZE (12 * 1024)

void APPLICATION_START(void)
{
    /* RPC to work with LE Audio Client Control */
    le_audio_rpc_init(get_spy_instance(), lehs_rpc_rx_callback, 1);

    /* Register call back and configuration with stack */
    wiced_bt_stack_init(lehs_btm_cback, &lehs_cfg_settings);

    /* Create a buffer heap, make it the default heap.  */
    p_lea_default_heap = wiced_bt_create_heap(DEVICE_NAME, NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
#if defined(WIN32) && defined(CLI_SUPPORT)
    {
        DWORD thread_address;
        CreateThread(0, 0, cmdLineThread, 0, 0, &thread_address);
    }
#endif
}

wiced_bt_cfg_settings_t *app_get_cfg_settings(void)
{
    return &lehs_cfg_settings;
}

void app_set_connection_options(wiced_ble_ext_adv_phy_mask_t mask, wiced_ble_ext_conn_cfg_phy_options_t *p_out)
{
    const wiced_bt_cfg_ble_scan_settings_t *p_bsc = app_get_cfg_settings()->p_ble_cfg->p_ble_scan_cfg;

    {
        p_out->scan_int = p_bsc->high_duty_scan_interval;
        p_out->scan_window = p_bsc->high_duty_conn_scan_window;
        p_out->min_conn_int = p_bsc->conn_min_interval;
        p_out->max_conn_int = p_bsc->conn_max_interval;
        p_out->conn_latency = p_bsc->conn_latency;
        p_out->supervision_to = p_bsc->conn_supervision_timeout;
        p_out->min_ce_len = 0;
        p_out->max_ce_len = 0;
    }
}


wiced_result_t app_create_connection(uint8_t addr_type, wiced_bt_device_address_t bdaddr)
{
    wiced_result_t status;
    wiced_ble_ext_conn_cfg_t conn_cfg = {
        .adv_handle = 0xff,
        .sub_event = 0xff,
        .init_filter_policy = 0,
        .own_addr_type = BLE_ADDR_PUBLIC,  /**< initiator address type */
        .peer_addr_type = BLE_ADDR_PUBLIC, /**< peer address type */
        .peer_addr = {0},
        .initiating_phys = WICED_BLE_EXT_ADV_PHY_1M_BIT,
        .timeout_secs = 30,
    };

    /* write address */
    conn_cfg.peer_addr_type = addr_type;
    WICED_MEMCPY(conn_cfg.peer_addr, bdaddr, BD_ADDR_LEN);

    /* write phy options */
    app_set_connection_options(WICED_BLE_EXT_ADV_PHY_1M_BIT, &conn_cfg.phy_options[0]);

    status = wiced_ble_ext_create_connection(&conn_cfg);
    return status;
}

#if defined(WIN32) && defined(CLI_SUPPORT)
static DWORD __stdcall cmdLineThread(VOID *p)
{
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    while (1)
    {
        extern int le_hs_cli_routine(void);
        if (le_hs_cli_routine() == 0)
        {
            break;
        }
    }
    return (0);
}
#endif
#endif
