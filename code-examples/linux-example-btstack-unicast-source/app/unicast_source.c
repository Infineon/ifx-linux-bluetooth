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

/*******************************************************************************
*                           INCLUDES
*******************************************************************************/
/* Application includes */
#include "unicast_source_bt_manager.h"
#include "unicast_source_gatt.h"
#include "unicast_source_vcs.h"
#include "unicast_source_rpc.h"
#include "unicast_source_mcs.h"
#include "hci_control_api.h"
#include "unicast_source.h"

/* App library includes */

/* BT Stack includes */
#include "wiced_bt_cfg.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"

#include "log.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define BT_STACK_HEAP_SIZE (12 * 1024)


/******************************************************************************
 *                              EXTERNS
 *****************************************************************************/
extern wiced_bt_cfg_settings_t  unicast_source_cfg_settings;
extern wiced_bt_cfg_isoc_t      unicast_source_isoc_cfg;
extern int                      btspy_inst;
extern unicast_source_gatt_cb_t g_unicast_source_gatt_cb;
extern uint16_t curr_conn_id;
extern int8_t   curr_handle;

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
uint8_t zero_bda[6]                 = {0};
wiced_bt_heap_t *p_default_heap     = NULL;
static wiced_bt_ble_scan_results_t unicast_scan_sink_list[UNICAST_SINK_SCAN_DEVICE_MAX]         = {0};
static BOOL32 audio_mute   = FALSE;

/******************************************************************************
* Function Name: APPLICATION_START()
*******************************************************************************
* Summary:
*   BT stack initialization.
*
* Parameters:
*   None
*
* Return:
*      None
*
******************************************************************************/
void APPLICATION_START(void)
{
    /* RPC to work with LE Audio Client Control */
    unicast_source_rpc_init(btspy_inst);

    /* Register call back and configuration with stack */
    wiced_bt_stack_init(unicast_source_btm_cback, &unicast_source_cfg_settings);

    /* Create a buffer heap, make it the default heap.  */
    p_default_heap = wiced_bt_create_heap("Unicast Source", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);

    iso_audio_init(&unicast_source_isoc_cfg);

}

/******************************************************************************
* Function Name: unicast_source_handle_scan
*******************************************************************************
* Summary: start or stop scan Sink Device
*
* Parameters:
*   uint8_t *p_data:    
*       one byte for start scan or stop scan
*       1: start
*       0: stop
*       
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_scan(uint8_t *p_data)
{
    uint8_t scan = 0;

    if (p_data == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_data is NULL\n", __FUNCTION__);
        return;
    }
    if (*p_data != TRUE && *p_data != FALSE)
    {
        WICED_BT_TRACE("[%s][ERROR] input data not 1 or 0\n", __FUNCTION__);
        return;
    }

    STREAM_TO_UINT8(scan, p_data);

    WICED_BT_TRACE("[%s] adv %d\n", __FUNCTION__, scan);

    unicast_source_gatt_start_stop_scan_menu(scan);
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
void unicast_source_handle_connect(uint8_t *p_data, uint32_t data_len, uint8_t buffersize)
{
    wiced_bt_device_address_t bd_addr = {0};
    uint8_t addr_type = 0;
    wiced_result_t status;

    if (p_data == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_data is NULL\n", __FUNCTION__);
        return;
    }

    if (data_len > buffersize)
    {
        TRACE_ERR("!! data length wrong:%d, buffersize:%d\n", data_len, buffersize);
        return;
    }

    if (unicast_source_get_connected_device_num() == UNICAST_SINK_CONNECT_DEVICE_MAX)
    {
        TRACE_ERR("!! Connection Limit Exceed: %d\n", UNICAST_SINK_CONNECT_DEVICE_MAX);
        return;
    }

    if (*p_data > BLE_ADDR_RANDOM_ID)
    {
        TRACE_ERR("!! addr_type out of range: %d\n", *p_data);
        return;
    }

    STREAM_TO_UINT8(addr_type, p_data);
    STREAM_TO_BDADDR(bd_addr, p_data);

    WICED_BT_TRACE("[%s] type %d address %B\n", __FUNCTION__, addr_type, bd_addr);
    status = unicast_source_gatt_connect(bd_addr, addr_type);
    if(status == FALSE)
    {
        WICED_BT_TRACE("CONNECTION FAILED: %d",status);
    }
    else
    {
        WICED_BT_TRACE("CONNECTION CREATED: %d", status);
    }
}

/******************************************************************************
* Function Name: unicast_source_handle_disconnect
*******************************************************************************
* Summary: disconnect to the Sink Device by bda
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
void unicast_source_handle_disconnect(uint8_t *p_data, uint32_t data_len, uint8_t buffersize)
{
    wiced_bt_device_address_t bd_addr;
    uint8_t addr_type;

    if (p_data == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_data is NULL\n", __FUNCTION__);
        return;
    }

    if (data_len > buffersize)
    {
        TRACE_ERR("!! data length wrong:%d, buffersize:%d\n", data_len, buffersize);
        return;
    }

    if (*p_data > BLE_ADDR_RANDOM_ID)
    {
        TRACE_ERR("!! addr_type out of range: %d\n", *p_data);
        return;
    }

    STREAM_TO_UINT8(addr_type, p_data);
    STREAM_TO_BDADDR(bd_addr, p_data);

    WICED_BT_TRACE("[%s] BD addr:%B addr_type:%d\n", __FUNCTION__, bd_addr, addr_type);
    unicast_source_gatt_disconnect(bd_addr);
}

/******************************************************************************
* Function Name: unicast_source_handle_play
*******************************************************************************
* Summary: Source device start play audio by conn_id 
*
* Parameters:
*   uint8_t *p_data:    
*       conn_id: the current connection id of the Sink device
*       codec_config: BAP support codec config enum
*
*   uint32_t data_len
*   uint8_t buffersize
*       
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_play(uint8_t *p_data, uint32_t data_len, uint8_t buffersize)
{
    uint16_t conn_id;
    uint32_t codec_config;

    if (p_data == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_data is NULL\n", __FUNCTION__);
        return;
    }

    if (data_len > buffersize)
    {
        TRACE_ERR("!! data length wrong:%d, buffersize:%d\n", data_len, buffersize);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT32(codec_config, p_data);

    WICED_BT_TRACE("[%s] conn_id %d codec_config %d\n", __FUNCTION__, conn_id, codec_config);

    unicast_source_mcs_play(conn_id, codec_config);
}

/******************************************************************************
* Function Name: unicast_source_handle_pause
*******************************************************************************
* Summary: pause the current play audio stream
*
* Parameters:
*   uint8_t *p_data:    
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_pause(uint8_t *p_data, uint32_t data_len, uint8_t buffersize)
{
    uint16_t conn_id;

    if (p_data == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_data is NULL\n", __FUNCTION__);
        return;
    }

    if (data_len > buffersize)
    {
        TRACE_ERR("!! data length wrong:%d, buffersize:%d\n", data_len, buffersize);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d\n", __FUNCTION__, conn_id);

    unicast_source_mcs_pause(conn_id);
}

/******************************************************************************
* Function Name: unicast_source_handle_vol_up
*******************************************************************************
* Summary: volme up
*
* Parameters:
*   uint8_t *p_data:    
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_vol_up(uint8_t *p_data, uint32_t data_len, uint8_t buffersize)
{
    uint16_t conn_id;

    if (p_data == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_data is NULL\n", __FUNCTION__);
        return;
    }

    if (data_len > buffersize)
    {
        TRACE_ERR("!! data length wrong:%d, buffersize:%d\n", data_len, buffersize);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d\n", __FUNCTION__, conn_id);

    unicast_source_vcs_vol_up(conn_id);
}

/******************************************************************************
* Function Name: unicast_source_handle_vol_down
*******************************************************************************
* Summary: volume down
*
* Parameters:
*   uint8_t *p_data:    
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_vol_down(uint8_t *p_data, uint32_t data_len, uint8_t buffersize)
{
    uint16_t conn_id;

    if (p_data == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_data is NULL\n", __FUNCTION__);
        return;
    }

    if (data_len > buffersize)
    {
        TRACE_ERR("!! data length wrong:%d, buffersize:%d\n", data_len, buffersize);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d\n", __FUNCTION__, conn_id);

    unicast_source_vcs_vol_down(conn_id);
}

/******************************************************************************
* Function Name: unicast_source_handle_abs_vol
*******************************************************************************
* Summary: set abs vol
*
* Parameters:
*   uint8_t *p_data:    
*       conn_id: the current connection id of the Sink device
*       volume of the abs value 0-255
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_abs_vol(uint8_t *p_data, uint32_t data_len, uint8_t buffersize)
{
    uint16_t conn_id;
    uint8_t vol;

    if (p_data == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_data is NULL\n", __FUNCTION__);
        return;
    }

    if (data_len > buffersize)
    {
        TRACE_ERR("!! data length wrong:%d, buffersize:%d\n", data_len, buffersize);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(vol, p_data);

    WICED_BT_TRACE("[%s] conn_id %d vol %d\n", __FUNCTION__, conn_id, vol);

    unicast_source_vcs_volume_set(conn_id, vol);
}

/******************************************************************************
* Function Name: unicast_source_handle_mute
*******************************************************************************
* Summary: mute audio streaming
*
* Parameters:
*   uint8_t *p_data
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_mute(uint8_t *p_data, uint32_t data_len, uint8_t buffersize)
{
    uint16_t conn_id;

    if (p_data == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_data is NULL\n", __FUNCTION__);
        return;
    }

    if (data_len > buffersize)
    {
        TRACE_ERR("!! data length wrong:%d, buffersize:%d\n", data_len, buffersize);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d\n", __FUNCTION__, conn_id);

    unicast_source_vcs_mute(conn_id);

    audio_mute = TRUE;
}

/******************************************************************************
* Function Name: unicast_source_handle_unmute
*******************************************************************************
* Summary: unumte audio streaming
*
* Parameters:
*   uint8_t *p_data:    
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*       
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_unmute(uint8_t *p_data, uint32_t data_len, uint8_t buffersize)
{
    uint16_t conn_id;

    if (p_data == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_data is NULL\n", __FUNCTION__);
        return;
    }

    if (data_len > buffersize)
    {
        TRACE_ERR("!! data length wrong:%d, buffersize:%d\n", data_len, buffersize);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d\n", __FUNCTION__, conn_id);

    unicast_source_vcs_unmute(conn_id);

    audio_mute = FALSE;
}

/******************************************************************************
* Function Name: unicast_source_handle_unmute_vol_down
*******************************************************************************
* Summary: unmute and volume down
*
* Parameters:
*   uint8_t *p_data:
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_unmute_vol_down(uint8_t *p_data, uint32_t data_len, uint8_t buffersize)
{
    uint16_t conn_id;

    if (p_data == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_data is NULL\n", __FUNCTION__);
        return;
    }

    if (data_len > buffersize)
    {
        TRACE_ERR("!! data length wrong:%d, buffersize:%d\n", data_len, buffersize);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d\n", __FUNCTION__, conn_id);

    unicast_source_vcs_unmute_relative_volume_down(conn_id);
}

/******************************************************************************
* Function Name: unicast_source_handle_unmute_vol_up
*******************************************************************************
* Summary: unmute and volume up
*
* Parameters:
*   uint8_t *p_data:
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_unmute_vol_up(uint8_t *p_data, uint32_t data_len, uint8_t buffersize)
{
    uint16_t conn_id;

    if (p_data == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_data is NULL\n", __FUNCTION__);
        return;
    }

    if (data_len > buffersize)
    {
        TRACE_ERR("!! data length wrong:%d, buffersize:%d\n", data_len, buffersize);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d\n", __FUNCTION__, conn_id);

    unicast_source_vcs_unmute_relative_volume_up(conn_id);
}

/******************************************************************************
* Function Name: unicast_source_add_sink_dev
*******************************************************************************
* Summary: add found sink device to loacl array when scan, the index of the array
*          is the handle of device, need reset the array when start scan
*
* Parameters:
*   wiced_bt_ble_scan_results_t *p_scan_result
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_add_sink_dev(wiced_bt_ble_scan_results_t *p_scan_result)
{
    uint8_t idx = 0;

    if (p_scan_result == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_scan_result is NULL\n", __FUNCTION__);
        return;
    }

    for (idx = 0; idx < UNICAST_SINK_SCAN_DEVICE_MAX; idx++)
    {
        if (memcmp(unicast_scan_sink_list[idx].remote_bd_addr, p_scan_result->remote_bd_addr, sizeof(zero_bda)) == 0)
        {
            return; 
        }
        if (memcmp(unicast_scan_sink_list[idx].remote_bd_addr, zero_bda, sizeof(zero_bda)) == 0)
        {
            break;
        }
    }
    memcpy(&unicast_scan_sink_list[idx], p_scan_result, sizeof(wiced_bt_ble_scan_results_t));
    TRACE_LOG("Remote_bd_addr:%B Handle:%d\n",unicast_scan_sink_list[idx].remote_bd_addr, idx);
}

/******************************************************************************
* Function Name: unicast_source_clear_sink_dev
*******************************************************************************
* Summary: clear the saving sink device
*
* Parameters:
*   None
*
* Return:
*   None
*
******************************************************************************/
void unicast_source_clear_sink_dev( void )
{
    memset(unicast_scan_sink_list, 0, sizeof(unicast_scan_sink_list));
}

/******************************************************************************
* Function Name: unicast_source_show_sink_dev
*******************************************************************************
* Summary: show current saving sink device
*
* Parameters:
*   None
*
* Return:
*   uint8_t: number of Unicast Sink device found 
*
******************************************************************************/
uint8_t unicast_source_show_sink_dev( void )
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < UNICAST_SINK_SCAN_DEVICE_MAX; i++)
    {
        if (memcmp(unicast_scan_sink_list[i].remote_bd_addr, zero_bda, sizeof(zero_bda)) != 0)
        {
            count++;
            TRACE_LOG("%B, Handle:%d\n", unicast_scan_sink_list[i].remote_bd_addr, i);
        }
    }
    return count;
}

/******************************************************************************
* Function Name: unicast_source_get_scan_result
*******************************************************************************
* Summary: get the saving sink device info
*
* Parameters:
*   uint8_t idx: handle of the sink device
*
* Return:
*   None
*
******************************************************************************/
wiced_bt_ble_scan_results_t* unicast_source_get_scan_result(uint8_t idx)
{
    if (idx >= UNICAST_SINK_SCAN_DEVICE_MAX)
    {
        TRACE_ERR("index out of range:%d, MAX:%d\n", idx, UNICAST_SINK_SCAN_DEVICE_MAX-1);
        return NULL;
    }
    if (memcmp(unicast_scan_sink_list[idx].remote_bd_addr, zero_bda, sizeof(zero_bda)) == 0)
    {
        TRACE_ERR("input Handle wrong, no sink device");
        return NULL;
    }

    return &unicast_scan_sink_list[idx]; 
}

/******************************************************************************
* Function Name: unicast_source_show_connected_device
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
void unicast_source_show_connected_device( void )
{
    uint8_t idx = 0;
    unicast_source_clcb_t *p_clcb = NULL;
    for (idx = 0; idx < MAX_CONNECTION_INSTANCE; idx++)
    {
        p_clcb = &g_unicast_source_gatt_cb.unicast_clcb[idx];
        if (p_clcb->in_use == TRUE)
        {
            TRACE_LOG("Connected Remote BD ADDR:%B Handle:%d\n", p_clcb->bda, idx);
        }
    }
}

/******************************************************************************
* Function Name: unicast_source_get_connected_device_num 
*******************************************************************************
* Summary: get the number of connected sink device
*
* Parameters:
*   None
*
* Return:
*   None
*
******************************************************************************/
uint8_t unicast_source_get_connected_device_num( void )
{
    uint8_t idx = 0;
    uint8_t count = 0;
    unicast_source_clcb_t *p_clcb = NULL;
    for (idx = 0; idx < MAX_CONNECTION_INSTANCE; idx++)
    {
        p_clcb = &g_unicast_source_gatt_cb.unicast_clcb[idx];
        if (p_clcb->in_use == TRUE)
        {
            count++;
        }
    }
    return count;
}

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
unicast_source_clcb_t* unicast_source_get_connected_device_by_handle( uint8_t idx )
{
    unicast_source_clcb_t *p_clcb = NULL;
    if (idx >= MAX_CONNECTION_INSTANCE)
    {
        TRACE_ERR("index out of range\n");
        return NULL;
    }
    p_clcb = &g_unicast_source_gatt_cb.unicast_clcb[idx];
    if (p_clcb == NULL || p_clcb->in_use == FALSE)
    {
        TRACE_ERR("Input Handle Not connected\n");
        return NULL;
    }
    
    return p_clcb;
}

/******************************************************************************
* Function Name: unicast_source_get_audio_mute_status
*******************************************************************************
* Summary: get the status of mute
*
* Parameters:
*   None
*
* Return:
*   BOOL32: 
*       true 
*       false
*
******************************************************************************/
BOOL32 unicast_source_get_audio_mute_status( void )
{
    return audio_mute;
}

/******************************************************************************
* Function Name: unicast_source_menu_clear_info
*******************************************************************************
* Summary: clear the saving variable when disconnect
*
* Parameters:
*   None
*
* Return:
*   None
*
******************************************************************************/
void unicast_source_menu_clear_info(void)
{
    curr_conn_id = 0;
    curr_handle = -1;
}
