/*
* $ Copyright Cypress Semiconductor $
*/

#include "lepl.h"

#define MAX_DEVICES 2

typedef struct
{
    wiced_bt_device_address_t address;
    uint8_t rank;
    uint8_t is_used;
    uint16_t conn_id;
} device_data_t;

typedef struct
{
    SIRK sirk_encr;
    SIRK sirk_plain;
    uint8_t sirk_type;
    uint8_t set_size;
    device_data_t device_list[MAX_DEVICES];
    uint8_t num_devices;
} csip_app_data_t;

csip_app_data_t csip_app_data;
wiced_timer_t scan_timer;
#define LEPL_CSIP_SET_MEMBER_DISCOVERY_TIMEOUT_IN_SEC 100

device_data_t *get_device(wiced_bt_device_address_t address)
{
    for (int i = 0; i < MAX_DEVICES; i++)
    {
        if (memcmp(address, csip_app_data.device_list[i].address, BD_ADDR_LEN) == 0)
        {
            return &csip_app_data.device_list[i];
        }
    }
    return NULL;
}

static void csis_scan_timeout(WICED_TIMER_PARAM_TYPE p_inst)
{

    //scan timeout stop scan
    lepl_start_stop_set_member_discovery(NULL, 0);
}

void set_current_device(uint16_t conn_id, wiced_bt_device_address_t address)
{
    device_data_t *p_dev;

    //if device is not present add it
    if ((p_dev = get_device(address)) == NULL)
    {
        if (csip_app_data.num_devices < MAX_DEVICES)
        {
            int index;
            for (index = 0; index < MAX_DEVICES; index++)
            {
                if (!csip_app_data.device_list[index].is_used)
                {
                    WICED_BT_TRACE("[%s] index %d",__FUNCTION__, index);
                    memcpy(csip_app_data.device_list[index].address, address, BD_ADDR_LEN);
                    csip_app_data.device_list[index].is_used = 1;
                    csip_app_data.device_list[index].conn_id = conn_id;
                    csip_app_data.num_devices++;
                    break;
                }
            }

        }
        else
        {
            //error
            WICED_BT_TRACE("Device cannot be added!");
        }
    }
    else
    {
        p_dev->conn_id = conn_id;
    }
}

static void csis_scan_result_cback(wiced_ble_ext_scan_results_t *p_scan_result, uint16_t adv_len, uint8_t *p_adv_data)
{
    if (p_scan_result)
    {
        if (csip_app_data.num_devices == csip_app_data.set_size)
        {
            //all devices_found stop scan
            lepl_start_stop_set_member_discovery(NULL, 0);
            return;
        }

        if (wiced_bt_ga_csip_check_if_belongs_to_coordinated_set(p_scan_result,
                                                                 adv_len,
                                                                 p_adv_data,
                                                                 &csip_app_data.sirk_plain))
        {
            WICED_BT_TRACE("Found CSIS Device!: %B \n", p_scan_result->remote_bd_addr);

            if (get_device(p_scan_result->remote_bd_addr) == NULL)
            {
                int index;
                for (index = 0; index < MAX_DEVICES; index++)
                {
                    if (!csip_app_data.device_list[index].is_used)
                    {
                        WICED_BT_TRACE("[%s] index %d", __FUNCTION__, index);
                        memcpy(csip_app_data.device_list[index].address, p_scan_result->remote_bd_addr, BD_ADDR_LEN);
                        csip_app_data.device_list[index].is_used = 1;
                        csip_app_data.num_devices++;

                        app_create_connection(p_scan_result->ble_addr_type, csip_app_data.device_list[index].address);
                        break;
                    }
                }
            }
            else
            {
                WICED_BT_TRACE("Already Found! %B", p_scan_result->remote_bd_addr);
            }
        }
        else
        {
            WICED_BT_TRACE(" Some other device found! : %B \n", p_scan_result->remote_bd_addr);
        }
    }
}

uint8_t lepl_if_sirk_zero(uint8_t *sirk)
{
    uint8_t res = 0;
    for (int i = 0; i < WICED_BT_GA_CSIS_SET_IDENTITY_RESOLVING_KEY_LEN; i++)
        res |= sirk[i];
    return (res == 0) ? 1 : 0;
}


wiced_result_t lepl_start_stop_set_member_discovery(uint8_t *sirk, uint8_t start_scan)
{
    wiced_result_t status;

    WICED_BT_TRACE("[%s] num device %d", __FUNCTION__, csip_app_data.num_devices);
    //start scan send results to the client profile
    if (start_scan && wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE)
    {
        if (csip_app_data.num_devices == csip_app_data.set_size) return WICED_SUCCESS;
        if (scan_timer.p_cback == NULL)
            wiced_init_timer(&scan_timer, csis_scan_timeout, 0, WICED_SECONDS_PERIODIC_TIMER);
        wiced_start_timer(&scan_timer, LEPL_CSIP_SET_MEMBER_DISCOVERY_TIMEOUT_IN_SEC);
        status = lepl_start_stop_scan(1, csis_scan_result_cback);
        WICED_BT_TRACE("wiced_bt_ble_scan: %d\n", status);
    }
    else
    {
        //stopping scan
        if (wiced_is_timer_in_use(&scan_timer))
        {
            status = lepl_start_stop_scan(0, csis_scan_result_cback);
            wiced_stop_timer(&scan_timer);
        }
    }
    return WICED_SUCCESS;
}

void lepl_csis_handle_disconnection(wiced_bt_device_address_t address)
{
    device_data_t *p_dev = get_device(address);
    if (p_dev!= NULL)
    {
        WICED_MEMSET(p_dev, 0, sizeof(device_data_t));
        csip_app_data.num_devices--;
    }
}

uint8_t lepl_csis_device_belongs_to_coordinated_set(uint16_t conn_id, wiced_bt_ga_csis_sirk_t sirk)
{
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);

    if (!p_clcb) 
    {
        return WICED_FALSE;
    }
    if (!p_clcb->peer_profiles.p_csis)
    {
        WICED_BT_TRACE_CRIT("%s Doesn't support CSIS %x", __FUNCTION__, conn_id);
    }
    if (p_clcb->in_use)
    {
        if (WICED_MEMCMP(p_clcb->csis_data.sirk_data.sirk, sirk, WICED_BT_GA_CSIS_SET_IDENTITY_RESOLVING_KEY_LEN) == 0)
            return WICED_TRUE;
    }
    return WICED_FALSE;
}

wiced_result_t lepl_csis_callback(uint16_t conn_id,
                                  void *p_app_ctx,
                                  gatt_intf_service_object_t *p_service,
                                  wiced_bt_gatt_status_t status,
                                  uint32_t evt_type,
                                  gatt_intf_attribute_t *p_char,
                                  void *p_data,
                                  int len)
{
    wiced_bt_ga_csis_data_t *p_event_data = (wiced_bt_ga_csis_data_t *)p_data;
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    WICED_BT_TRACE("[%s] event %x char %d status %x \n", __FUNCTION__, evt_type, p_char->characteristic_type, status);

    switch (evt_type)
    {
    case WRITE_CMPL_EVT:
        return WICED_BT_SUCCESS;
        break;

    case READ_CMPL_EVT:
    case NOTIFICATION_EVT:

        if (status == WICED_BT_GATT_SUCCESS)
        {
            switch (p_char->characteristic_type)
            {
            case CSIS_LOCK_CHARACTERISTIC: {
                WICED_BT_TRACE("[%s] lock value:%x \n", __FUNCTION__, p_event_data->lock_val);
            }
            break;
            case CSIS_RANK_CHARACTERISTIC: {
                p_clcb->csis_data.rank = p_event_data->rank;
                WICED_BT_TRACE("[%s] rank value:%x \n", __FUNCTION__, p_event_data->rank);
            }
            break;
            case CSIS_SIRK_CHARACTERISTIC: {
                //wiced_bt_device_address_t client_address;
                WICED_BT_TRACE("[%s] sirk_type %d\n", __FUNCTION__, p_event_data->sirk_data.sirk_type);
                WICED_BT_TRACE_ARRAY(p_event_data->sirk_data.sirk, sizeof(SIRK), "sirk val:");
                p_clcb->csis_data.sirk_data.sirk_type = p_event_data->sirk_data.sirk_type;
                memcpy(p_clcb->csis_data.sirk_data.sirk, p_event_data->sirk_data.sirk, sizeof(SIRK));
                if (p_clcb->csis_data.sirk_data.sirk_type == WICED_BT_GA_CSIS_SIRK_ENCR)
                {
                    memcpy(csip_app_data.sirk_encr, p_event_data->sirk_data.sirk, sizeof(SIRK));
                    wiced_bt_device_address_t client_address;
                    wiced_bt_gatt_get_device_address(conn_id, &client_address, NULL, NULL);
                    ga_csis_sirk_decryption_func(&csip_app_data.sirk_encr,
                              &p_clcb->ltk,
                              &p_clcb->csis_data.sirk_data.sirk);
                    memcpy(csip_app_data.sirk_plain, p_clcb->csis_data.sirk_data.sirk, sizeof(SIRK));
                }
                else
                {
                    memcpy(csip_app_data.sirk_plain, p_event_data->sirk_data.sirk, sizeof(SIRK));
                }
            }
            break;
            case CSIS_SIZE_CHARACTERISTIC: {
                WICED_BT_TRACE("[%s] size value:%x \n", __FUNCTION__, p_event_data->size);
                p_clcb->csis_data.size = p_event_data->size;
                csip_app_data.set_size = p_event_data->size;
            }
            break;
            }
        }
    }
    return WICED_BT_SUCCESS;
}
