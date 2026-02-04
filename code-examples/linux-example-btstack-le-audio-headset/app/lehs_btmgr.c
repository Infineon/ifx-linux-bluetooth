/*
 * $ Copyright Cypress Semiconductor $
 */

/* Application includes */
#include "lehs.h"
#include "app_bt_utils.h"

extern wiced_bt_cfg_ble_t lehs_ble_cfg;

wiced_result_t lehs_btm_handle_encryption_sts(wiced_bt_dev_encryption_status_t *p_encryption_sts)
{
    /* Start GATT Discovery */
    if (WICED_SUCCESS != p_encryption_sts->result) return WICED_ERROR;

    lehs_gatt_start_discovery(p_encryption_sts->bd_addr);

    return WICED_SUCCESS;
}

void lehs_isoc_init(void)
{
    wiced_ble_isoc_cfg_t cfg = {.max_cis = 2, .max_bis = 2};
    wiced_ble_isoc_init(&cfg, lehs_isoc_event_handler);

    lehs_isoc_dhm_init();
}

wiced_result_t lehs_app_init(wiced_bt_dev_enabled_t *p_btm_enabled)
{
    wiced_bt_gatt_status_t sts = WICED_BT_ERROR;
    /* Initialize SPI */
    lehs_isoc_init();

    lehs_ga_init_data_t lehs_ga_init_data = {
        .p_pacs_init_data = &(wiced_bt_ga_pacs_init_data_t){ .max_snk_caps_spt = 1, .max_src_caps_spt = 1 },
        .p_ascs_init_data = &(wiced_bt_ga_ascs_init_data_t){ .max_snk_ase_spt = 2,  .max_src_ase_spt = 1 },
        .p_vcs_init_data  = &(wiced_bt_ga_vcs_init_data_t){ .step_size = VCS_STEP_SIZE }, // max_aics/vocs default to 0
        .p_mics_init_data = &(wiced_bt_ga_mics_init_data_t){ .max_aics = 1 },
        .p_bass_init_data = &(wiced_bt_ga_bass_init_data_t){ .max_broadcast_receive_state = 2 }
    };

    /* Initialize GATT */
    sts = lehs_gatt_init(lehs_ble_cfg.ble_max_simultaneous_links,
                                 lehs_ble_cfg.ble_max_rx_pdu_size,
                                 lehs_ga_init_data);
    if (sts) WICED_BT_TRACE("[%s] GATT init sts %d\n", __FUNCTION__, sts);

    lehs_bis_init();

    wiced_bt_set_pairable_mode(1, 0);

    return WICED_SUCCESS;
}

wiced_result_t lehs_btm_handle_key_update_event(wiced_bt_device_link_keys_t *p_event_data)
{
    {
        wiced_bt_device_link_keys_t *keys = p_event_data;

        WICED_BT_TRACE("[%s] linkkey update %B Type: %d  Conn: %B   Key_Mask: 0x%x",
                       __FUNCTION__,
                       keys->bd_addr,
                       keys->key_data.ble_addr_type,
                       keys->conn_addr,
                       keys->key_data.le_keys_available_mask);
        WICED_BT_TRACE("[%s] br_edr %A type %d",
                       __FUNCTION__,
                       keys->key_data.br_edr_key,
                       sizeof(keys->key_data.br_edr_key),
                       keys->key_data.br_edr_key_type);
        WICED_BT_TRACE("[%s] lltk %A sec_level %d %d %d",
                       __FUNCTION__,
                       keys->key_data.le_keys.lltk,
                       sizeof(keys->key_data.le_keys.lltk),
                       keys->key_data.le_keys.sec_level,
                       keys->key_data.le_keys.local_csrk_sec_level,
                       keys->key_data.le_keys.srk_sec_level);
        WICED_BT_TRACE("[%s] pltk %A sec_level %d %d %d",
                       __FUNCTION__,
                       keys->key_data.le_keys.pltk,
                       sizeof(keys->key_data.le_keys.pltk),
                       keys->key_data.le_keys.sec_level,
                       keys->key_data.le_keys.local_csrk_sec_level,
                       keys->key_data.le_keys.srk_sec_level);
    }

    lehs_nvram_write_keys(p_event_data);

    return WICED_BT_SUCCESS;
}

wiced_result_t lehs_btm_handle_key_request_event(wiced_bt_device_link_keys_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
#if SIMULATED_NVRAM
    if (lehs_nvram_read_keys(p_event_data) == -1)
#else
    if (!lehs_nvram_read_keys(p_event_data))
#endif
    {
        WICED_BT_TRACE("[%s] no key for BDA: %B", __FUNCTION__, p_event_data->bd_addr);
        result = WICED_ERROR;
    }
    else
    {
        WICED_BT_TRACE("[%s] found key for BDA: %B", __FUNCTION__, p_event_data->bd_addr);
    }

    return result;
}

wiced_result_t lehs_btm_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t res = WICED_SUCCESS;
    wiced_bt_dev_ble_io_caps_req_t *p_ble_io_caps = &p_event_data->pairing_io_capabilities_ble_request;

    extern const char *wiced_bt_get_btm_management_event_name(wiced_bt_management_evt_t event);
    WICED_BT_TRACE("[%s] Event [%d] %s \n", __FUNCTION__, event, wiced_bt_get_btm_management_event_name(event));

    switch (event)
    {
        case BTM_ENABLED_EVT:
            lehs_app_init(&p_event_data->enabled);
            //if (lehs_cfg_settings.p_ble_cfg->rpa_refresh_timeout)
            //{
            set_local_bd_addr();
                //wiced_bt_set_local_bdaddr(local_bda, BLE_ADDR_RANDOM);
            //}
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: {
            p_ble_io_caps->local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_ble_io_caps->oob_data = BTM_OOB_NONE;
            p_ble_io_caps->auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
            p_ble_io_caps->max_key_size = 16;
            p_ble_io_caps->init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
            p_ble_io_caps->resp_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        }
        break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            WICED_BT_TRACE("[%s] status \n",__FUNCTION__);
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            res = lehs_btm_handle_encryption_sts(&p_event_data->encryption_status);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        {
            wiced_bt_device_link_keys_t *p_update = &p_event_data->paired_device_link_keys_update;
            res = lehs_btm_handle_key_update_event(p_update);
            if (res == WICED_BT_SUCCESS)
            {
                WICED_BT_TRACE_ARRAY(p_update->key_data.le_keys.lltk, 16, "***LLTK***");
            }
        }
        break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        {
            res = lehs_btm_handle_key_request_event(&p_event_data->paired_device_link_keys_request);
        }
        break;
        default:
            res = WICED_ERROR;
            break;
    }

    return res;
}
