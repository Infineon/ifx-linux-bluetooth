/*
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*/

/* Application includes */
#include "unicast_sink_nvram.h"

/* BT Stack includes */
#include "wiced_bt_gatt.h"
#include "wiced_hal_nvram.h"

/******************************************************************************
 * Function Name: get_next_nvram_index
 ******************************************************************************
 * Summary: get next index of saved nvram 
 *
 * Parameters:
 *  int current_nvram_index
 *
 * Return:
 *  int
 *
******************************************************************************/
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

/******************************************************************************
 * Function Name: unicast_sink_nvram_write
 ******************************************************************************
 * Summary: write the data into nvram by id 
 *
 * Parameters:
 *  int nvram_id,
 *  wiced_bt_device_address_t bdaddr,
 *  uint8_t *p_data,
 *  uint32_t len
 *  None
 *
 * Return:
 *  int
 *
******************************************************************************/
int unicast_sink_nvram_write(int nvram_id,
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

/******************************************************************************
 * Function Name: unicast_sink_nvram_read
 ******************************************************************************
 * Summary: read data from nvram by id
 *
 * Parameters:
 *  int nvram_id
 *  wiced_bt_device_address_t bdaddr
 *  uint8_t *p_data
 *  uint32_t len
 *
 * Return:
 * int
 *
******************************************************************************/
int unicast_sink_nvram_read(int nvram_id, wiced_bt_device_address_t bdaddr, uint8_t *p_data, uint32_t len)
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

/******************************************************************************
 * Function Name: unicast_sink_nvram_read_peer_device
 ******************************************************************************
 * Summary: read peer device from nvram by bdaddr, Return the nvram id where the device was found
 *
 * 
 *
 * Parameters:
 *  wiced_bt_device_address_t p_bdaddr
 *  unicast_sink_nvram_data_t *p_device
 *
 * Return:
 *  uint32_t
 *
******************************************************************************/
uint32_t unicast_sink_nvram_read_peer_device(wiced_bt_device_address_t p_bdaddr,
                                                  unicast_sink_nvram_data_t *p_device)
{
    int nvram_id = get_next_nvram_index(0);

    WICED_BT_TRACE("[%s] bda %B nvram_id  0x%x", __FUNCTION__, p_bdaddr, nvram_id);

    for (; nvram_id; nvram_id = get_next_nvram_index(nvram_id))
    {
        unicast_sink_nvram_data_t bonded_dev_info;

        WICED_BT_TRACE("[%s] nvram_id 0x%x", __FUNCTION__, nvram_id);
        if (!unicast_sink_nvram_read(nvram_id,
                                          p_bdaddr,
                                          (uint8_t *)&bonded_dev_info, sizeof(unicast_sink_nvram_data_t)))
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

/******************************************************************************
 * Function Name: unicast_sink_nvram_write_keys
 ******************************************************************************
 * Summary: write the link key to nvram
 *
 * Parameters:
 *  wiced_bt_device_link_keys_t *p_linkkeys
 *
 * Return:
 *  int 
 *
******************************************************************************/
int unicast_sink_nvram_write_keys( wiced_bt_device_link_keys_t *p_linkkeys)
{
    int nvram_id = 0;
    unicast_sink_nvram_data_t bonded_dev_info = {0};
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

    nvram_id = unicast_sink_nvram_read_peer_device(p_linkkeys->bd_addr, &bonded_dev_info);
    if (nvram_id == 0)
    {
        WICED_BT_TRACE("[%s] nvram_id 0x%x", __FUNCTION__, nvram_id);
        if (!unicast_sink_nvram_read(UNICAST_APP_NVRAM_ID_LAST_PAIRED_KEY,
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
    unicast_sink_nvram_write(nvram_id, p_linkkeys->bd_addr, (uint8_t *)&bonded_dev_info, sizeof(bonded_dev_info));

    if (write_last_used_nvram_id)
    {
        unicast_sink_nvram_write(UNICAST_APP_NVRAM_ID_LAST_PAIRED_KEY,
                                    p_linkkeys->bd_addr,
                                    (uint8_t *)&last_used_nvram_id,
                                    sizeof(last_used_nvram_id));
    }

    return nvram_id;
}

/******************************************************************************
 * Function Name: unicast_sink_nvram_read_keys
 ******************************************************************************
 * Summary: read the saved link key
 *
 * Parameters:
 *  wiced_bt_device_link_keys_t *p_linkkeys
 *
 * Return:
 *  int
 *
******************************************************************************/
int unicast_sink_nvram_read_keys(wiced_bt_device_link_keys_t *p_linkkeys)
{
    int nvram_id = 0;
    unicast_sink_nvram_data_t bonded_dev_info;
    wiced_result_t result;

    if (p_linkkeys == NULL)
    {
        WICED_BT_TRACE("[%s] p_linkkeys is null", __FUNCTION__, p_linkkeys);
        return 0;
    }

    nvram_id = unicast_sink_nvram_read_peer_device(p_linkkeys->bd_addr, &bonded_dev_info);
    if (nvram_id == 0)
    {
        return nvram_id;
    }

    *p_linkkeys = bonded_dev_info.link_keys;

    WICED_BT_TRACE("[%s] %d %A", __FUNCTION__, nvram_id, p_linkkeys, sizeof(wiced_bt_device_link_keys_t));

    return nvram_id;
}

/******************************************************************************
 * Function Name: unicast_sink_nvram_delete
 ******************************************************************************
 * Summary: delete saved data by nvram_id
 *
 * Parameters:
 *  int nvram_id
 *  wiced_bt_device_address_t bdaddr
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_nvram_delete(int nvram_id, wiced_bt_device_address_t bdaddr)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_hal_delete_nvram(nvram_id, &result);
    WICED_BT_TRACE("[%s] result %d", __FUNCTION__, result);
}
