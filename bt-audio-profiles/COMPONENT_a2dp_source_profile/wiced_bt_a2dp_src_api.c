/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company)
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @file
 *
 * This file contains implementation of the audio source interface.
 */

#include <string.h>
#include "wiced_memory.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_a2dp_src_int.h"
#include "wiced_bt_a2dp_source.h"

#if APP_CHIP == 20703
#define WICED_ALREADY_INITIALIZED 1000
#endif

#define GKI_send_msg(a,b,p_msg) wiced_bt_a2dp_source_hdl_event((wiced_bt_avdt_evt_hdr_t *)p_msg)

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_utils_bdcpy
**
** Description      Copy bd addr b to a.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_a2dp_source_utils_bdcpy(wiced_bt_device_address_t a,
    const wiced_bt_device_address_t b)
{
    int i;

    for(i = BD_ADDR_LEN; i!=0; i--)
    {
        *a++ = *b++;
    }
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_utils_bdcmp
**
** Description          Compare bd addr b to a.
**
** Returns              Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
int wiced_bt_a2dp_source_utils_bdcmp(const wiced_bt_device_address_t a,
    const wiced_bt_device_address_t b)
{
    int i;

    for(i = BD_ADDR_LEN; i!=0; i--)
    {
        if( *a++ != *b++ )
        {
            return -1;
        }
    }
    return 0;
}

wiced_result_t wiced_bt_a2dp_source_init(wiced_bt_a2dp_source_config_data_t *p_config_data,
    wiced_bt_a2dp_source_control_cb_t control_cb, wiced_bt_heap_t* heap)
{
    wiced_result_t ret = WICED_SUCCESS;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_source_cb.is_init == WICED_TRUE)
    {
        WICED_BTA2DP_SRC_ERROR("%s: Already initialized \n", __FUNCTION__);
        return WICED_ALREADY_INITIALIZED;
    }

    if(p_config_data == NULL)
    {
        WICED_BTA2DP_SRC_ERROR("%s: Config data is not present \n", __FUNCTION__);
        return WICED_BADARG;
    }

    if(p_config_data->codec_capabilities.info == NULL)
    {
        WICED_BTA2DP_SRC_ERROR("%s: codec_capabilities info is not present \n", __FUNCTION__);
        return WICED_BADARG;
    }

    if (heap == NULL)
    {
        WICED_BTA2DP_SRC_ERROR("%s: heap ptr is NULL \n", __FUNCTION__);
        return WICED_BADARG;
    }

    /* Check if Init of the state machine can be done. If not, then bail-out */
    if(wiced_bt_a2dp_source_init_state_machine() != WICED_SUCCESS)
    {
        return WICED_BADARG;
    }

    /* Initialize main control block */
    memset(&wiced_bt_a2dp_source_cb, 0, sizeof(wiced_bt_a2dp_source_cb_t));

    /* Copy the configuration information */
    wiced_bt_a2dp_source_cb.p_config_data = p_config_data;

    /* Copy the callback information */
    wiced_bt_a2dp_source_cb.control_cb = control_cb;
    //wiced_bt_a2dp_source_cb.data_cb = data_cb;

    wiced_bt_a2dp_source_cb.heap = heap;
    /* Register with AVDT */
    if( (ret = wiced_bt_a2dp_source_register()) != WICED_SUCCESS)
    {
        WICED_BTA2DP_SRC_ERROR("%s: Register with AVDT failed \n", __FUNCTION__);
        return ret;
    }

    wiced_bt_a2dp_source_cb.is_init = WICED_TRUE;
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_source_deinit(void)
{
    wiced_bt_avdt_evt_hdr_t *p_buf = NULL;
    uint8_t event = WICED_BT_A2DP_SOURCE_API_DEINIT_EVT;
    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_source_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SRC_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }

    if((p_buf = (wiced_bt_avdt_evt_hdr_t *)wiced_bt_get_buffer_from_heap(wiced_bt_a2dp_source_cb.heap, sizeof(wiced_bt_avdt_evt_hdr_t))) == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    wiced_bt_a2dp_source_hdl_event(event, (wiced_bt_avdt_evt_hdr_t *)p_buf);
    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_source_connect(wiced_bt_device_address_t bd_address)
{
    wiced_bt_a2dp_source_api_data_t *p_buf = NULL;
    uint8_t event;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_source_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SRC_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }

    if((p_buf = (wiced_bt_a2dp_source_api_data_t *)
        wiced_bt_get_buffer_from_heap(wiced_bt_a2dp_source_cb.heap, sizeof(wiced_bt_a2dp_source_api_data_t))) == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    event = WICED_BT_A2DP_SOURCE_API_CONNECT_EVT;
    wiced_bt_a2dp_source_utils_bdcpy(p_buf->bd_address, bd_address);
    wiced_bt_a2dp_source_hdl_event(event, (wiced_bt_avdt_evt_hdr_t *)&p_buf->hdr);
    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_source_disconnect(uint16_t handle)
{
    wiced_bt_a2dp_source_api_data_t *p_buf = NULL;
    uint8_t event;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_source_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SRC_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }

    if((p_buf = (wiced_bt_a2dp_source_api_data_t *)
        wiced_bt_get_buffer_from_heap(wiced_bt_a2dp_source_cb.heap, sizeof(wiced_bt_a2dp_source_api_data_t))) == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    event = WICED_BT_A2DP_SOURCE_API_DISCONNECT_EVT;
    p_buf->handle    = handle;

    wiced_bt_a2dp_source_hdl_event(event, (wiced_bt_avdt_evt_hdr_t *)&p_buf->hdr);

    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_source_start(uint16_t handle, wiced_bt_a2dp_codec_info_t *codec_info)
{
    wiced_bt_a2dp_source_api_start_t *p_buf = NULL;
    uint8_t event;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_source_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SRC_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }

    if((p_buf = (wiced_bt_a2dp_source_api_start_t *)
        wiced_bt_get_buffer_from_heap(wiced_bt_a2dp_source_cb.heap, sizeof(wiced_bt_a2dp_source_api_start_t))) == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    event = WICED_BT_A2DP_SOURCE_API_START_EVT;
    p_buf->handle    = handle;
    memcpy(&p_buf->codec_params, codec_info, sizeof(wiced_bt_a2dp_codec_info_t));

    wiced_bt_a2dp_source_hdl_event(event, (wiced_bt_avdt_evt_hdr_t *)&p_buf->hdr);
    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_source_send_start_response( uint16_t handle, uint8_t label, uint8_t status )
{
    wiced_bt_a2dp_source_api_data_t *p_buf = NULL;
    uint8_t event;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_source_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SRC_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }

    if((p_buf = (wiced_bt_a2dp_source_api_data_t *)
        wiced_bt_get_buffer_from_heap(wiced_bt_a2dp_source_cb.heap, sizeof(wiced_bt_a2dp_source_api_data_t))) == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    event               = WICED_BT_A2DP_SOURCE_API_START_RESP_EVT;
    p_buf->handle       = handle;
    p_buf->label        = label;
    p_buf->status       = status;

    wiced_bt_a2dp_source_hdl_event(event, (wiced_bt_avdt_evt_hdr_t *)&p_buf->hdr);
    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_source_suspend(uint16_t handle)
{
    wiced_bt_a2dp_source_api_data_t *p_buf = NULL;
    uint8_t event;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_source_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SRC_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }

    if((p_buf = (wiced_bt_a2dp_source_api_data_t *)
        wiced_bt_get_buffer_from_heap(wiced_bt_a2dp_source_cb.heap, sizeof(wiced_bt_a2dp_source_api_data_t))) == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    event = WICED_BT_A2DP_SOURCE_API_SUSPEND_EVT;
    p_buf->handle    = handle;

    wiced_bt_a2dp_source_hdl_event(event, (wiced_bt_avdt_evt_hdr_t *)&p_buf->hdr);
    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_source_start_streaming(uint8_t handle, uint8_t *p_media_buf, uint16_t buf_len, uint32_t time_stamp, uint8_t m_pt )
{
    /* Check if already initialized, then just return with error */
    if(wiced_bt_a2dp_source_cb.is_init != WICED_TRUE)
    {
        WICED_BTA2DP_SRC_ERROR("%s: Not initialized \n", __FUNCTION__);
        return WICED_NOTUP;
    }
    return wiced_bt_avdt_write_req(handle, p_media_buf, buf_len, time_stamp, m_pt, AVDT_DATA_OPT_NONE);
}
