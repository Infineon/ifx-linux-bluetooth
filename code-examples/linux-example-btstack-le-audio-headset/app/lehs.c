/*
 * $ Copyright Cypress Semiconductor $
 */

/* Application includes */
#include "lehs.h"

#define BT_STACK_HEAP_SIZE (12 * 1024)

extern int                          btspy_inst;
extern wiced_bt_cfg_settings_t      lehs_cfg_settings;

extern int get_spy_instance(void);

wiced_bt_heap_t *p_lea_default_heap = NULL;
wiced_bt_device_address_t local_bda;

void APPLICATION_START(void)
{
    /* RPC to work with LE Audio Client Control */
    le_audio_rpc_init(get_spy_instance(), lehs_rpc_rx_callback, 1);

    /* Register call back and configuration with stack */
    wiced_bt_stack_init(lehs_btm_cback, &lehs_cfg_settings);

    /* Create a buffer heap, make it the default heap.  */
    p_lea_default_heap = wiced_bt_create_heap(DEVICE_NAME, NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);

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

#if 0
uint8_t app_init_csis_by_cfg_file( char *file_name )
{
    uint32_t read_len       = 0;
    unsigned char buf[50]   = {0};
    uint8_t size            = 0;
    uint8_t rank            = 0;
    uint8_t location        = 0;
    FILE * file_fd          = NULL;
    uint8_t i               = 0;
    int fileseek            = 0;

    wiced_bt_ga_csis_sirk_data_t sirk = {0};

    sirk.is_oob     = 0;
    sirk.sirk_type  = WICED_BT_GA_CSIS_SIRK_PLAIN;
    //TODO: sirk read from cfg file
    memcpy(&sirk.sirk, default_sirk, sizeof(wiced_bt_ga_csis_sirk_t));

    file_fd = fopen(file_name, "r");
    if (!file_fd)
    {
        return 0;
    }
    while(WICED_TRUE)
    {
        read_len = fread(buf, 1, sizeof(buf), file_fd);

        if(read_len == -1)
        {
            printf("File read error!\n");
            return 0;
        }
        else if(read_len == 0)
        {
            break;
        }
        else
        {
            //TODO: read sirk
            for (uint8_t csis_cfg_idx = 0; csis_cfg_idx < sizeof(csis_cfg)/sizeof(csis_cfg[0]); csis_cfg_idx++)
            {
                char *csis_str = strstr(buf, csis_cfg[csis_cfg_idx].csis_str);
                //printf("csis_str:%s\n", csis_str);
                char *tmp = strstr(csis_str, "=");
                //char *tmp = strtok(csis_str, "=");
                csis_cfg[csis_cfg_idx].csis_value.number = atoi(tmp+1);
                //printf("csis_cfg[csis_cfg_idx].csis_value.number:%d\n", csis_cfg[csis_cfg_idx].csis_value.number);
            }
        }
    }
    fclose(file_fd);
    
    TRACE_LOG("size:%d, rank:%d, location:%d", csis_cfg[CSIS_SIZE].csis_value.number, csis_cfg[CSIS_RANK].csis_value.number, csis_cfg[CSIS_LOCATION].csis_value.number);

    lehs_csis_set_sirk(&sirk);
    lehs_csis_set_size(csis_cfg[CSIS_SIZE].csis_value.number);
    lehs_csis_set_rank(csis_cfg[CSIS_RANK].csis_value.number);
    lehs_set_audio_location(csis_cfg[CSIS_LOCATION].csis_value.number);
}
#endif
