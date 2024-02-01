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
#include "unicast_sink_bt_manager.h"
#include "unicast_sink_gatt.h"
#include "unicast_sink_rpc.h"
#include "unicast_sink_gatt.h"
#include "unicast_sink_mcs.h"
#include "unicast_sink_vcs.h"


/* App library includes */
#include "le_audio_rpc.h"

/* BT Stack includes */
#include "wiced_bt_cfg.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "data_types.h"

/* Porting Layer*/
#include "log.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define BT_STACK_HEAP_SIZE  (12 * 1024)
#define DATA_LEN_1_BYTE     ( 1u )
#define DATA_LEN_2_BYTE     ( 2u )
#define DATA_LEN_3_BYTE     ( 3u )

void set_local_bd_addr(void);

/******************************************************************************
 *                              EXTERNS
 *****************************************************************************/
extern wiced_bt_cfg_settings_t unicast_sink_cfg_settings;
extern int btspy_inst;
extern unicast_sink_gatt_cb_t g_unicast_sink_gatt_cb;
extern uint16_t curr_conn_id;
extern int8_t   curr_handle;

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
wiced_bt_heap_t *p_default_heap = NULL;
BOOL32 is_adv = WICED_FALSE;

/******************************************************************************
 * Function Name: APPLICATION_START
 ******************************************************************************
 * Summary: the application start point, and init stack here
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
******************************************************************************/
void APPLICATION_START(void)
{
    /* RPC to work with LE Audio Client Control */
    le_audio_rpc_init(btspy_inst, unicast_sink_rpc_rx_callback, 1);

    /* Register call back and configuration with stack */
    wiced_bt_stack_init(unicast_sink_btm_cback, &unicast_sink_cfg_settings);

    /* Create a buffer heap, make it the default heap.  */
    p_default_heap = wiced_bt_create_heap("Unicast Sink", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);

}

/******************************************************************************
 * Function Name: unicast_sink_handle_adv
 ******************************************************************************
 * Summary: sink start or stop advertisment
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_adv(uint8_t *p_data, uint8_t data_len)
{
    uint8_t adv;
    if (p_data == NULL)
    {
        TRACE_ERR("p_data is NULL\n");
        return;
    }
    if (data_len != DATA_LEN_1_BYTE)
    {
        TRACE_ERR("data length wrong:%d not equal to:%d\n", data_len, DATA_LEN_1_BYTE);
        return; 
    }

    STREAM_TO_UINT8(adv, p_data);

    WICED_BT_TRACE("[%s] adv %d len %d\n", __FUNCTION__, adv, data_len);

    unicast_sink_gatt_start_stop_adv(adv);

    if (adv == WICED_TRUE)
    {
        is_adv = WICED_TRUE;
    }
    else
    {
        is_adv = WICED_FALSE;
    }
}

/******************************************************************************
 * Function Name: unicast_sink_handle_play
 ******************************************************************************
 * Summary: play audio from source
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_play(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    if (p_data == NULL)
    {
        TRACE_ERR("p_data is NULL\n");
        return;
    }

    if (data_len != DATA_LEN_2_BYTE)
    {
        TRACE_ERR("data length wrong:%d not equal to:%d\n", data_len, DATA_LEN_2_BYTE);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    unicast_sink_mcs_play_pause(conn_id, WICED_TRUE);
}

/******************************************************************************
 * Function Name: unicast_sink_handle_pause
 ******************************************************************************
 * Summary: pause audio from source
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_pause(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    if (p_data == NULL)
    {
        TRACE_ERR("p_data is NULL\n");
        return;
    }

    if (data_len != DATA_LEN_2_BYTE)
    {
        TRACE_ERR("data length wrong:%d not equal to:%d\n", data_len, DATA_LEN_2_BYTE);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    unicast_sink_mcs_play_pause(conn_id, WICED_FALSE);
}

/******************************************************************************
 * Function Name: unicast_sink_handle_vol_up
 ******************************************************************************
 * Summary: volume up
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_vol_up(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    if (p_data == NULL)
    {
        TRACE_ERR("p_data is NULL\n");
        return;
    }

    if (data_len != DATA_LEN_2_BYTE)
    {
        TRACE_ERR("data length wrong:%d not equal to:%d\n", data_len, DATA_LEN_2_BYTE);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    unicast_sink_vcs_vol_up(conn_id);
}

/******************************************************************************
 * Function Name: unicast_sink_handle_vol_down
 ******************************************************************************
 * Summary: volume down
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_vol_down(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    if (p_data == NULL)
    {
        TRACE_ERR("p_data is NULL\n");
        return;
    }

    if (data_len != DATA_LEN_2_BYTE)
    {
        TRACE_ERR("data length wrong:%d not equal to:%d\n", data_len, DATA_LEN_2_BYTE);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    unicast_sink_vcs_vol_down(conn_id);
}

/******************************************************************************
 * Function Name: unicast_sink_handle_unmute_vol_down
 ******************************************************************************
 * Summary: unmute first and volume down
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_unmute_vol_down(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    if (p_data == NULL)
    {
        TRACE_ERR("p_data is NULL\n");
        return;
    }

    if (data_len != DATA_LEN_2_BYTE)
    {
        TRACE_ERR("data length wrong:%d not equal to:%d\n", data_len, DATA_LEN_2_BYTE);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    unicast_sink_vcs_unmute_relative_volume_down(conn_id);
}

/******************************************************************************
 * Function Name: unicast_sink_handle_unmute_vol_up
 ******************************************************************************
 * Summary: unmute first and volume down
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_unmute_vol_up(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    if (p_data == NULL)
    {
        TRACE_ERR("p_data is NULL\n");
        return;
    }

    if (data_len != DATA_LEN_2_BYTE)
    {
        TRACE_ERR("data length wrong:%d not equal to:%d\n", data_len, DATA_LEN_2_BYTE);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    unicast_sink_vcs_unmute_relative_volume_up(conn_id);
}

/******************************************************************************
 * Function Name: unicast_sink_handle_abs_vol
 ******************************************************************************
 * Summary: handle set absolute volume
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_abs_vol(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    uint8_t vol;
    if (p_data == NULL)
    {
        TRACE_ERR("p_data is NULL\n");
        return;
    }

    if (data_len != DATA_LEN_3_BYTE)
    {
        TRACE_ERR("data length wrong:%d not equal to:%d\n", data_len, DATA_LEN_3_BYTE);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(vol, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d vol %d\n", __FUNCTION__, conn_id, data_len, vol);

    unicast_sink_vcs_volume_set(conn_id, vol);
}

/******************************************************************************
 * Function Name: unicast_sink_handle_unmute
 ******************************************************************************
 * Summary: mute sink device
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_mute(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    if (p_data == NULL)
    {
        TRACE_ERR("p_data is NULL\n");
        return;
    }

    if (data_len != DATA_LEN_2_BYTE)
    {
        TRACE_ERR("data length wrong:%d not equal to:%d\n", data_len, DATA_LEN_2_BYTE);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    unicast_sink_vcs_mute(conn_id);
}

/******************************************************************************
 * Function Name: unicast_sink_handle_unmute
 ******************************************************************************
 * Summary: unmute sink device
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_unmute(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    if (p_data == NULL)
    {
        TRACE_ERR("p_data is NULL\n");
        return;
    }

    if (data_len != DATA_LEN_2_BYTE)
    {
        TRACE_ERR("data length wrong:%d not equal to:%d\n", data_len, DATA_LEN_2_BYTE);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    unicast_sink_vcs_unmute(conn_id);
}

/******************************************************************************
 * Function Name: unicast_sink_show_connected_device
 ******************************************************************************
 * Summary: show current connected device
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_show_connected_device( void )
{
    uint8_t idx = 0;
    unicast_sink_clcb_t *p_clcb = NULL;
    for(idx = 0; idx < MAX_CONNECTION_INSTANCE; idx++)
    {
        p_clcb = &g_unicast_sink_gatt_cb.unicast_clcb[idx];
        if (p_clcb->in_use == WICED_TRUE)
        {
            TRACE_LOG("Connected Remote BD ADDR:%B Handle:%d\n", p_clcb->bda, idx);
        }
    }
}

/******************************************************************************
 * Function Name: unicast_sink_get_connected_device_by_handle
 ******************************************************************************
 * Summary: get the current connected device by handle (index)
 *
 * Parameters:
 *  uint8_t idx
 *
 * Return:
 *  unicast_sink_clcb_t* 
 *
******************************************************************************/
unicast_sink_clcb_t* unicast_sink_get_connected_device_by_handle( uint8_t idx )
{
    unicast_sink_clcb_t *p_clcb = NULL;
    if (idx >= MAX_CONNECTION_INSTANCE)
    {
        TRACE_ERR("index out of raneg\n");
        return NULL;
    }

    p_clcb = &g_unicast_sink_gatt_cb.unicast_clcb[idx];
    if (p_clcb == NULL || p_clcb->in_use == WICED_FALSE)
    {
        TRACE_ERR("Input Handle Not Connected Source\n");
        return NULL;
    }

    return p_clcb;
}

/******************************************************************************
 * Function Name: unicast_sink_get_connected_device_by_connid
 ******************************************************************************
 * Summary: get the current connected device by conn id
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  unicast_sink_clcb_t* 
 *
******************************************************************************/
unicast_sink_clcb_t* unicast_sink_get_connected_device_by_connid( uint16_t conn_id )
{
    unicast_sink_clcb_t *p_clcb = NULL;
    uint8_t idx = 0;
    for (idx = 0; idx < MAX_CONNECTION_INSTANCE; idx++)
    {
        p_clcb = &g_unicast_sink_gatt_cb.unicast_clcb[idx];
        if (p_clcb->in_use == WICED_TRUE && p_clcb->conn_id == conn_id)
        {
            return p_clcb;
        }
    }

    return NULL;
}

/******************************************************************************
 * Function Name: unicast_sink_is_streaming
 ******************************************************************************
 * Summary: check the streaming status. 
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  BOOL32 
 *
******************************************************************************/
BOOL32 unicast_sink_is_streaming( uint16_t conn_id )
{
    unicast_sink_clcb_t* p_clcb = unicast_sink_get_connected_device_by_connid(conn_id);
    if (p_clcb == NULL)
    {
        TRACE_ERR("conn_id:0x%x not use", conn_id);
        return WICED_FALSE;
    }

    if (p_clcb->mcs_data.media_state == WICED_BT_GA_MCS_MEDIA_PLAYING)
    {
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

/******************************************************************************
 * Function Name: unicast_sink_menu_clear_info
 ******************************************************************************
 * Summary: clear the variable use by menu
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_menu_clear_info( void )
{
    curr_conn_id = 0;
    curr_handle = -1;
    is_adv = WICED_FALSE;
}
