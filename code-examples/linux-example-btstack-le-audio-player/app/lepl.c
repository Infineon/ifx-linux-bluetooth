/*
 * $ Copyright Cypress Semiconductor $
 */

/* Application includes */
#include "lepl.h"
#ifdef HS_SPK_ENABLED
#include "bt_hs_spk_handsfree.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_sdp_defs.h"
#endif // HS_SPK_ENABLED

#include "log.h"


/*******************************************************************************
*                               MACROS
*******************************************************************************/

/******************************************************************************
 *                              EXTERNS
 *****************************************************************************/
extern wiced_bt_device_address_t    bt_device_address;
extern int                          spy_inst;
extern wiced_bt_cfg_isoc_t          lepl_isoc_cfg;
extern wiced_bt_cfg_ble_t           lepl_ble_cfg;
extern wiced_bt_cfg_gatt_t          lepl_gatt_cfg;

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
//uint8_t zero_bda[6]                 = {0};
wiced_bt_heap_t *p_unicast_heap     = NULL;

#if 0
wiced_bt_cfg_settings_t bt_cfg_settings = {.device_name = (uint8_t *)"Airoc Player",
                                                       .p_ble_cfg = &lepl_ble_cfg,
                                                       .p_gatt_cfg = &lepl_gatt_cfg,
#if defined(HFP_ENABLED) || defined(HS_SPK_ENABLED)
                                                       .security_required = BTM_SEC_BEST_EFFORT, /**< Security requirements mask */
                                                       .p_br_cfg = &lehs_br,
                                                       .p_l2cap_app_cfg = &lehs_l2cap_app
#endif
};
#endif

static uint8_t local_bda[BD_ADDR_LEN] = {0};

int get_spy_instance(void);
void set_local_bd_addr(void);

#define BT_STACK_HEAP_SIZE (12 * 1024)

#if defined(HFP_ENABLED) || defined(HS_SPK_ENABLED)
uint16_t wiced_app_cfg_sdp_record_get_size(void)
{
    return (uint16_t)sizeof(btheadset_sdp_db);
}
#endif

void APPLICATION_START(void)
{
    /* RPC to work with LE Audio Client Control */
    lepl_rpc_init(get_spy_instance());

    /* Register call back and configuration with stack */
    wiced_bt_stack_init(lepl_btm_cback, &lepl_cfg_settings);

    /* Create a buffer heap, make it the default heap.  */
    p_unicast_heap = wiced_bt_create_heap((char *)lepl_cfg_settings.device_name, NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
    //iso_audio_init(&lepl_isoc_cfg);
}

int get_spy_instance(void)
{
    return spy_inst;
}


wiced_bt_cfg_settings_t *app_get_cfg_settings(void)
{
    return &lepl_cfg_settings;
}

void app_set_connection_options(wiced_ble_ext_adv_phy_mask_t mask, wiced_ble_ext_conn_cfg_phy_options_t *p_out)
{
    const wiced_bt_cfg_ble_scan_settings_t *p_bsc = app_get_cfg_settings()->p_ble_cfg->p_ble_scan_cfg;

    {
        p_out->scan_int     = p_bsc->high_duty_scan_interval;
        p_out->scan_window  = p_bsc->high_duty_conn_scan_window;
        p_out->min_conn_int = p_bsc->conn_min_interval;
        p_out->max_conn_int = p_bsc->conn_max_interval;
        p_out->conn_latency = p_bsc->conn_latency;
        p_out->supervision_to = p_bsc->conn_supervision_timeout;
        p_out->min_ce_len   = 0;
        p_out->max_ce_len   = 0;
    }
}

wiced_result_t app_create_connection(uint8_t addr_type, wiced_bt_device_address_t bd_addr)
{
    wiced_result_t status;
    wiced_ble_ext_conn_cfg_t conn_cfg = {
        .adv_handle         = 0xff,
        .sub_event          = 0xff,
        .init_filter_policy = 0,
        .own_addr_type      = BLE_ADDR_PUBLIC,  /**< initiator address type */
        .peer_addr_type     = BLE_ADDR_PUBLIC, /**< peer address type */
        .peer_addr          = {0},
        .initiating_phys    = WICED_BLE_EXT_ADV_PHY_1M_BIT,
        .timeout_secs       = 30,
    };

    /* write address */
    conn_cfg.peer_addr_type = addr_type;
    WICED_MEMCPY(conn_cfg.peer_addr, bd_addr, BD_ADDR_LEN);

    /* write phy options */
    app_set_connection_options(WICED_BLE_EXT_ADV_PHY_1M_BIT, &conn_cfg.phy_options[0]);

    status = wiced_ble_ext_create_connection(&conn_cfg);
    return status;
}

/******************************************************************************
* Function Name: unicast_source_handle_connect
*******************************************************************************
* Summary: connect to Sink Device by bda
*
* Parameters:
*   uint8_t *p_data:
*       addr_type
*       bd_addr
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
//void lepl_handle_connect(uint8_t *p_data, uint32_t data_len, uint8_t buffersize, int cancel)
//{
//    wiced_bt_device_address_t   bd_addr     = {0};
//    uint8_t                     addr_type   = 0;
//    wiced_result_t              status      = 0;
//
//    if (p_data == NULL)
//    {
//        WICED_BT_TRACE("[%s][ERROR] p_data is NULL\n", __FUNCTION__);
//        return;
//    }
//
//    if (data_len > buffersize)
//    {
//        TRACE_ERR("!! data length wrong:%d, buffersize:%d\n", data_len, buffersize);
//        return;
//    }
//
//    //TBD ?
//    //if (unicast_source_get_connected_device_num() == UNICAST_SINK_CONNECT_DEVICE_MAX)
//    //{
//    //    TRACE_ERR("!! Connection Limit Exceed: %d\n", UNICAST_SINK_CONNECT_DEVICE_MAX);
//    //    return;
//    //}
//
//    if (*p_data > BLE_ADDR_RANDOM_ID)
//    {
//        TRACE_ERR("!! addr_type out of range: %d\n", *p_data);
//        return;
//    }
//
//    STREAM_TO_UINT8(addr_type, p_data);
//    STREAM_TO_BDADDR(bd_addr, p_data);
//
//    if (cancel == 0)
//    {
//        status = app_create_connection(addr_type, bd_addr);
//    }
//    else
//    {
//        status = wiced_bt_gatt_cancel_connect(bd_addr, 1);
//    }
//
//    //WICED_BT_TRACE("[%s] %sconnect type %d address %B len %d status %d\n", __FUNCTION__,
//    //               cancel ? "cancel" : "", addr_type, bd_addr, data_len, status);
//    //WICED_BT_TRACE("[%s] type %d address %B\n", __FUNCTION__, addr_type, bd_addr);
//    //status = unicast_source_gatt_connect(bd_addr, addr_type);
//    if(status == FALSE)
//    {
//        WICED_BT_TRACE("CONNECTION FAILED: %d",status);
//    }
//    else
//    {
//        WICED_BT_TRACE("CONNECTION CREATED: %d", status);
//    }
//}

/******************************************************************************
* Function Name: lepl_show_connected_device
*******************************************************************************
* Summary: shoe current connected sink device
*
* Parameters:
*   None
*
* Return:
*   None
*
******************************************************************************/
//void lepl_show_connected_device( void )
//{
//    uint8_t idx = 0;
//    lepl_clcb_t *p_clcb = NULL;
//    for (idx = 0; idx < MAX_CONNECTION_INSTANCE; idx++)
//    {
//        p_clcb = &g_lepl_gatt_cb.unicast_clcb[idx];
//        if (p_clcb->in_use == TRUE)
//        {
//            TRACE_LOG("Connected Remote BD ADDR:%B idx:%d\n", p_clcb->bda, idx);
//        }
//    }
//}

/******************************************************************************
* Function Name: unicast_source_get_connected_device_by_handle
*******************************************************************************
* Summary: get the connected sink device by handle
*
* Parameters:
*   uint8_t idx: handle of the connected device
*
* Return:
*      None
*
******************************************************************************/
//lepl_clcb_t* lepl_get_connected_device_by_idx( uint8_t idx )
//{
//    lepl_clcb_t *p_clcb = NULL;
//    if (idx >= MAX_CONNECTION_INSTANCE)
//    {
//        TRACE_ERR("index out of range\n");
//        return NULL;
//    }
//    p_clcb = &g_lepl_gatt_cb.unicast_clcb[idx];
//    if (p_clcb == NULL || p_clcb->in_use == FALSE)
//    {
//        TRACE_ERR("Input Handle Not connected\n");
//        return NULL;
//    }
//    return p_clcb;
//}


