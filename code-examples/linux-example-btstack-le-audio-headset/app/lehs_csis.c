/*
 * $ Copyright Cypress Semiconductor $
 */

#include "lehs.h"

#define LOCK_TIMER_TIMEOUT 30

lehs_csis_data_t csis_app_data;
char* csis_cfg_str[]  = {
                            CSIS_SIZE_STR,
                            CSIS_RANK_STR,
                            CSIS_LOCATION_STR
                        };
csis_cfg_t csis_cfg[] = {
                            {CSIS_SIZE, CSIS_SIZE_STR, 1},
                            {CSIS_RANK, CSIS_RANK_STR, 1},
                            {CSIS_LOCATION, CSIS_LOCATION_STR, 1}
                        };

void lehs_csis_set_size(uint8_t size)
{
    csis_app_data.size = size;
}

void lehs_csis_set_rank(uint8_t rank)
{
    csis_app_data.rank = rank;
}

void lehs_csis_set_sirk(wiced_bt_ga_csis_sirk_data_t *p_sirk)
{
    csis_app_data.sirk_data = *p_sirk;
}

wiced_bt_ga_csis_sirk_data_t *lehs_csis_get_sirk(void)
{
    return &csis_app_data.sirk_data;
}

wiced_result_t lehs_csis_initialize_data(void)
{
    gatt_intf_service_object_t *csis_context = g_lehs_gatt_cb.local_profiles.p_csis;
    uint8_t lehs_sirk[] =
        {0x45, 0x7d, 0x7d, 0x09, 0x21, 0xa1, 0xfd, 0x22, 0xce, 0xcd, 0x8c, 0x86, 0xdd, 0x72, 0xcc, 0xcd};

    //set initial default values
    csis_app_data.lock = WICED_BT_GA_CSIS_UNLOCKED;
    csis_app_data.size = 1;
    csis_app_data.rank = 1;
    csis_app_data.sirk_data.is_oob = WICED_FALSE;
    csis_app_data.sirk_data.sirk_type =
        WICED_BT_GA_CSIS_SIRK_PLAIN;
    memcpy(csis_app_data.sirk_data.sirk, lehs_sirk, WICED_BT_GA_CSIS_SET_IDENTITY_RESOLVING_KEY_LEN);

    if (csis_context == NULL) return WICED_ERROR;

    wiced_bt_ga_csis_set_lock_timeout_value(csis_context, LOCK_TIMER_TIMEOUT);
    return WICED_SUCCESS;
}

void encr_sirk(wiced_bt_device_address_t *p_bdaddr, wiced_bt_device_link_keys_t *keys)
{
    if (csis_app_data.sirk_data.sirk_type == WICED_BT_GA_CSIS_SIRK_ENCR)
    {
        ga_csis_sirk_encyption_func(&csis_app_data.sirk_data.sirk, keys, &csis_app_data.encr_sirk);
    }
}

wiced_result_t lehs_csis_handle_write_req_evt(uint16_t conn_id,
                                              const gatt_intf_service_object_t *p_service,
                                              gatt_intf_attribute_t *p_char,
                                              wiced_bt_ga_csis_data_t *p_evt_data)
{
    wiced_result_t result = WICED_SUCCESS;

    switch (p_char->characteristic_type)
    {
        //write requests
    case CSIS_LOCK_CHARACTERISTIC:
        WICED_BT_TRACE("[%s] lock %d \n", __FUNCTION__, p_evt_data->lock_val);
        csis_app_data.lock = p_evt_data->lock_val;
        csis_app_data.conn_id_of_lock_owner = conn_id;
        break;
    }
    return result;
}

wiced_result_t lehs_csis_handle_read_req_evt(uint16_t conn_id, gatt_intf_attribute_t *p_char, wiced_bt_ga_csis_data_t * p_evt_data)
{
    wiced_result_t result = WICED_SUCCESS;

    switch (p_char->characteristic_type)
    {
        //read requests
    case CSIS_LOCK_CHARACTERISTIC:
        p_evt_data->lock_val = csis_app_data.lock;
        break;
    case CSIS_RANK_CHARACTERISTIC:
        p_evt_data->rank = csis_app_data.rank;
        break;
    case CSIS_SIRK_CHARACTERISTIC:
        p_evt_data->sirk_data.sirk_type = csis_app_data.sirk_data.sirk_type;
        p_evt_data->sirk_data.is_oob = csis_app_data.sirk_data.is_oob;
        if (csis_app_data.sirk_data.sirk_type == WICED_BT_GA_CSIS_SIRK_ENCR)
        {
            wiced_bt_device_address_t addr;
            wiced_bt_device_link_keys_t keys;
            wiced_bt_gatt_get_device_address(conn_id, &addr, NULL, NULL);

           /* if (!script_app_read_paired_keys_from_nvram(addr, &keys))
            {
                WICED_BT_TRACE("[%s] no key for BDA: %B", __FUNCTION__, addr);
                result = WICED_ERROR;
            }
            else
            {
                WICED_BT_TRACE("[%s] found key for BDA: %B", __FUNCTION__, addr);
            }
            WICED_BT_TRACE_ARRAY(keys.key_data.le_keys.lltk, 16, "___LLTK___");
            */
            encr_sirk(&addr, &keys);

            memcpy(p_evt_data->sirk_data.sirk, csis_app_data.encr_sirk, sizeof(SIRK));
        }
        else
        {
            memcpy(p_evt_data->sirk_data.sirk, csis_app_data.sirk_data.sirk, sizeof(SIRK));
        }

        break;
    case CSIS_SIZE_CHARACTERISTIC:
        p_evt_data->size = csis_app_data.size;
        break;
    }
    return result;
}
wiced_result_t lehs_csis_callback(uint16_t conn_id,
                                  void *p_app_ctx,
                                  gatt_intf_service_object_t *p_service,
                                  wiced_bt_gatt_status_t status,
                                  uint32_t evt_type,
                                  gatt_intf_attribute_t *p_char,
                                  void *p_data,
                                  int len)
{
    wiced_result_t result = WICED_SUCCESS;

    switch (evt_type)
    {
    case WRITE_REQ_EVT:
        result = lehs_csis_handle_write_req_evt(conn_id, p_service, p_char, (wiced_bt_ga_csis_data_t *)p_data);
        break;
    case READ_REQ_EVT:
        result = lehs_csis_handle_read_req_evt(conn_id, p_char, (wiced_bt_ga_csis_data_t *)p_data);
        break;
    default:
        WICED_BT_TRACE("[%s] event %d \n", __FUNCTION__, p_char);
        break;
    }

    return result;
}
