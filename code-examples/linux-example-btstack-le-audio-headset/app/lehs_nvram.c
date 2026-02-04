/*
 * $ Copyright Cypress Semiconductor $
 */

/* Application includes */
#include "lehs_nvram.h"

/* BT Stack includes */
#include "wiced_hal_nvram.h"

#if SIMULATED_NVRAM
wiced_bt_device_link_keys_t local_nvram_paired_key_store[MAX_NUM_DEVICES_IN_NVRAM];
#endif

static int get_next_nvram_index(int current_nvram_index)
{
    if (current_nvram_index >= UNICAST_APP_NVRAM_ID_PAIRED_KEYS &&
        (current_nvram_index + UNICAST_APP_MAX_IDS_PER_DEVICE_ALIGNED) < UNICAST_APP_NVRAM_ID_PAIRED_KEYS_MAX)
    {
        return current_nvram_index + UNICAST_APP_MAX_IDS_PER_DEVICE_ALIGNED;
    }

    if (current_nvram_index == 0)
    {
        return UNICAST_APP_NVRAM_ID_PAIRED_KEYS;
    }

    return 0;
}

int lehs_nvram_write(int nvram_id,
                                wiced_bt_device_address_t bdaddr,
                                uint8_t *p_data,
                                uint32_t len)
{
    uint32_t write_len;
    wiced_result_t result;

    WICED_BT_TRACE_CRIT("[%s] nvram id 0x%x", __FUNCTION__, nvram_id);

    if ((nvram_id <= UNICAST_APP_NVRAM_ID_START) || (nvram_id >= UNICAST_APP_NVRAM_ID_END))
    {
        WICED_BT_TRACE_CRIT("[%s] bad id 0x%x", __FUNCTION__, nvram_id);
        return 0;
    }
    write_len = wiced_hal_write_nvram(nvram_id, len, (uint8_t *)p_data, &result);

    WICED_BT_TRACE_CRIT("[%s] write nvram result 0x%x", __FUNCTION__, result);

    if (write_len != len)
    {
        return 0;
    }

    WICED_BT_TRACE_CRIT("[%s] bytes written 0x%x", __FUNCTION__, write_len);
    return write_len;
}

int lehs_nvram_read(int nvram_id, wiced_bt_device_address_t bdaddr, uint8_t *p_data, uint32_t len)
{
    uint32_t read_len;
    wiced_result_t result;

    if ((nvram_id <= UNICAST_APP_NVRAM_ID_START) || (nvram_id >= UNICAST_APP_NVRAM_ID_END))
    {
        WICED_BT_TRACE_CRIT("[%s] bad id 0x%x", __FUNCTION__, nvram_id);
        return 0;
    }

    read_len = (uint32_t)wiced_hal_read_nvram(nvram_id, len, (uint8_t *)p_data, &result);

    if (read_len != len)
    {
        return 0;
    }
    WICED_BT_TRACE("[%s] %B id %d len (%d of %d) %A", __FUNCTION__, bdaddr, nvram_id, read_len, len, p_data, read_len);

    return read_len;
}

/* Return the nvram id where the device was found */
uint32_t lehs_nvram_read_peer_device(wiced_bt_device_address_t p_bdaddr,
                                                  lehs_nvram_data_t *p_device)
{
    int nvram_id = get_next_nvram_index(0);

    WICED_BT_TRACE("[%s] bda %B nvram_id  0x%x", __FUNCTION__, p_bdaddr, nvram_id);

    for (; nvram_id; nvram_id = get_next_nvram_index(nvram_id))
    {
        lehs_nvram_data_t bonded_dev_info;

        WICED_BT_TRACE("[%s] nvram_id 0x%x", __FUNCTION__, nvram_id);
        if (!lehs_nvram_read(nvram_id,
                                          p_bdaddr,
                                          (uint8_t *)&bonded_dev_info, sizeof(lehs_nvram_data_t)))
            continue;

        if (!memcmp(bonded_dev_info.link_keys.bd_addr, p_bdaddr, sizeof(wiced_bt_device_address_t)) ||
            !memcmp(bonded_dev_info.link_keys.conn_addr, p_bdaddr, sizeof(wiced_bt_device_address_t)))
        {
            WICED_BT_TRACE("[%s] %d bda %B con bda %B",
                           __FUNCTION__,
                           nvram_id,
                           bonded_dev_info.link_keys.bd_addr,
                           bonded_dev_info.link_keys.conn_addr);

            *p_device = bonded_dev_info;
            return nvram_id;
        }
    }
    return 0;
}

#if SIMULATED_NVRAM
int lehs_nvram_write_keys(wiced_bt_device_link_keys_t *p_linkkeys)
{
    int index = lehs_nvram_read_keys(p_linkkeys);
    if (index == -1)
    {
        int i = 0;
        wiced_bt_device_address_t null_addr = {0};

        for (i = 0; i < MAX_NUM_DEVICES_IN_NVRAM; i++)
        {
            if (!WICED_MEMCMP(local_nvram_paired_key_store[i].bd_addr, null_addr, BD_ADDR_LEN))
            {
                index = i;
                break;
            }
        }
        if (i == MAX_NUM_DEVICES_IN_NVRAM)
        {
            WICED_BT_TRACE_CRIT("[%s] No Storage", __FUNCTION__);
            return -1;
        }
    }

    WICED_MEMCPY(&local_nvram_paired_key_store[index], p_linkkeys, sizeof(wiced_bt_device_link_keys_t));
    return index;
}

#else
int lehs_nvram_write_keys( wiced_bt_device_link_keys_t *p_linkkeys)
{
    int nvram_id = 0;
    lehs_nvram_data_t bonded_dev_info = {0};
    uint32_t last_used_nvram_id = 0;
    uint32_t write_last_used_nvram_id = 0;

    if (p_linkkeys == NULL)
    {
        WICED_BT_TRACE("[%s] p_linkkeys is null", __FUNCTION__, p_linkkeys);
        return 0;
    }

    WICED_BT_TRACE("[%s] bda %B %A",
                   __FUNCTION__,
                   p_linkkeys->bd_addr,
                   p_linkkeys,
                   sizeof(wiced_bt_device_link_keys_t));

    nvram_id = lehs_nvram_read_peer_device(p_linkkeys->bd_addr, &bonded_dev_info);
    if (nvram_id == 0)
    {
        WICED_BT_TRACE("[%s] nvram_id 0x%x", __FUNCTION__, nvram_id);
        if (!lehs_nvram_read(UNICAST_APP_NVRAM_ID_LAST_PAIRED_KEY,
                                          p_linkkeys->bd_addr,
                                        (uint8_t *)&last_used_nvram_id,
                                          sizeof(last_used_nvram_id)))
        {
            // no last used nvram entry
            last_used_nvram_id = UNICAST_APP_NVRAM_ID_PAIRED_KEYS;
            WICED_BT_TRACE("[%s] no last used nvram id 0x%x", __FUNCTION__, last_used_nvram_id);
        }
        else
        {
            // found last used nvram entry, increment it
            WICED_BT_TRACE("[%s] last_used_nvram_id 0x%x", __FUNCTION__, last_used_nvram_id);
            last_used_nvram_id += (last_used_nvram_id + 1) % MAX_NUM_DEVICES_IN_NVRAM;
        }

        //create entry
        nvram_id = last_used_nvram_id;
        write_last_used_nvram_id = 1;
    }

    bonded_dev_info.link_keys = *p_linkkeys;
    lehs_nvram_write(nvram_id, p_linkkeys->bd_addr, (uint8_t *)&bonded_dev_info, sizeof(bonded_dev_info));

    if (write_last_used_nvram_id)
    {
        lehs_nvram_write(UNICAST_APP_NVRAM_ID_LAST_PAIRED_KEY,
                                    p_linkkeys->bd_addr,
                                    (uint8_t *)&last_used_nvram_id,
                                    sizeof(last_used_nvram_id));
    }

    return nvram_id;
}
#endif

#if SIMULATED_NVRAM
int lehs_nvram_read_keys(wiced_bt_device_link_keys_t* p_linkkeys)
{
    for (int i = 0; i < MAX_NUM_DEVICES_IN_NVRAM; i++)
    {
        if ((!WICED_MEMCMP(local_nvram_paired_key_store[i].bd_addr,
                                                     p_linkkeys->bd_addr,
                                                     BD_ADDR_LEN) ||
             !WICED_MEMCMP(local_nvram_paired_key_store[i].conn_addr,
                           p_linkkeys->conn_addr,
                           BD_ADDR_LEN)))
        {
            WICED_MEMCPY(p_linkkeys, &local_nvram_paired_key_store[i],
                         sizeof(wiced_bt_device_link_keys_t));
            return i;
        }
    }
    return -1;
}

#else
int lehs_nvram_read_keys(wiced_bt_device_link_keys_t *p_linkkeys)
{
    int nvram_id = 0;
    lehs_nvram_data_t bonded_dev_info;

    if (p_linkkeys == NULL)
    {
        WICED_BT_TRACE("[%s] p_linkkeys is null", __FUNCTION__, p_linkkeys);
        return 0;
    }

    nvram_id = lehs_nvram_read_peer_device(p_linkkeys->bd_addr, &bonded_dev_info);
    if (nvram_id == 0)
    {
        return nvram_id;
    }

    *p_linkkeys = bonded_dev_info.link_keys;

    WICED_BT_TRACE("[%s] %d %A", __FUNCTION__, nvram_id, p_linkkeys, sizeof(wiced_bt_device_link_keys_t));

    return nvram_id;
}
#endif

void lehs_nvram_delete(int nvram_id, wiced_bt_device_address_t bdaddr)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_hal_delete_nvram(nvram_id, &result);
    WICED_BT_TRACE("[%s] result %d", __FUNCTION__, result);
}
