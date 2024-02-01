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
 * This file contains implementation of the handsfree interface.
 */

#include "wiced_bt_hfp_hf_int.h"
#include "string.h"
#include "wiced_memory.h"

#ifdef CYW20706A2
#define WICED_ALREADY_INITIALIZED	1000
#endif

#define GKI_send_msg(a,b,p_msg) wiced_bt_hfp_hf_hdl_event((wiced_bt_hfp_hf_data_t *)p_msg)

//extern void wiced_bt_free_buffer( void *p_buf );
//extern void* wiced_bt_get_buffer( uint16_t size );

static wiced_result_t wiced_bt_hfp_hf_init_check_and_alloc_buffer ( uint32_t buf_size, void** p_buf )
{
   /* Check if already initialized, then just return with error */
   if( wiced_bt_hfp_hf_cb.is_init != WICED_TRUE )
   {
       WICED_BTHFP_ERROR("%s: Not initialized\n", __FUNCTION__);
       return WICED_NOTUP;
   }

   /* If initialized, allocate bffer*/
   if( ( *p_buf = (void*)wiced_bt_get_buffer(buf_size) ) == NULL )
   {
       return WICED_OUT_OF_HEAP_SPACE;
   }

   return WICED_SUCCESS;
}
wiced_result_t wiced_bt_hfp_hf_init(wiced_bt_hfp_hf_config_data_t *p_config_data,
    wiced_bt_hfp_hf_event_cb_t event_cb)
{
    uint8_t        i = 0;
    wiced_result_t ret = WICED_SUCCESS;

    /* Check if already initialized, then just return with error */
    if(wiced_bt_hfp_hf_cb.is_init == TRUE)
    {
        WICED_BTHFP_ERROR("%s: Already initialized\n", __FUNCTION__);
        return WICED_ALREADY_INITIALIZED;
    }

    if(p_config_data == NULL)
    {
        WICED_BTHFP_ERROR("%s: Config data is not present\n", __FUNCTION__);
        return WICED_BADARG;
    }

    if(p_config_data->num_server > WICED_BT_HFP_HF_MAX_CONN)
    {
        WICED_BTHFP_ERROR("%s: Max number of servers exceeded\n", __FUNCTION__);
        return WICED_BADARG;
    }

    /* Check if Init of the state machine can be done. If not, then bail-out */
    if(wiced_bt_hfp_hf_init_state_machine() != WICED_SUCCESS)
    {
        return WICED_BADARG;
    }

    /* Initialize main control block */
    memset(&wiced_bt_hfp_hf_cb, 0, sizeof(wiced_bt_hfp_hf_cb_t));

    /* Copy the configuration information */
    memcpy(&wiced_bt_hfp_hf_cb.config_data, p_config_data,
        sizeof(wiced_bt_hfp_hf_config_data_t));

    /* Copy the callback information */
    wiced_bt_hfp_hf_cb.p_event_cback = event_cb;

    /* Register with RFCOMM */
    for(i=0; i<p_config_data->num_server; i++) {
        if( (ret = wiced_bt_hfp_hf_register(p_config_data->scn[i],p_config_data->uuid[i])) != WICED_SUCCESS)
        {
            /*TODO: Stop the already started server in case any of the server registration fails */
            WICED_BTHFP_ERROR("%s: Register with RFCOMM failed\n", __FUNCTION__);
            return ret;
        }
    }

    wiced_bt_hfp_hf_cb.is_init = TRUE;
    return WICED_SUCCESS;

}

wiced_result_t wiced_bt_update_hfp_hf_supported_features(uint32_t feature_mask)
{
      memcpy(&wiced_bt_hfp_hf_cb.config_data.feature_mask, &feature_mask, sizeof(feature_mask));
      return WICED_SUCCESS;
}

wiced_result_t wiced_bt_hfp_hf_deinit(void)
{
    wiced_bt_hfp_hf_api_data_t *p_buf = NULL;
    wiced_result_t result = wiced_bt_hfp_hf_init_check_and_alloc_buffer( sizeof(wiced_bt_hfp_hf_api_data_t), (void**)&p_buf );

    if ( result != WICED_SUCCESS )
    {
         return result;
    }

    p_buf->hf_evt = WICED_BT_HFP_HF_API_DEINIT_EVT;

    GKI_send_msg(WICED_BT_HFP_HF_TASK_ID, WICED_BT_HFP_HF_TASK_MBOX, p_buf);

    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_hfp_hf_connect(wiced_bt_device_address_t bd_address)
{
    wiced_bt_hfp_hf_api_data_t *p_buf = NULL;
    wiced_result_t result = wiced_bt_hfp_hf_init_check_and_alloc_buffer( sizeof(wiced_bt_hfp_hf_api_data_t), (void**)&p_buf );

    if ( result != WICED_SUCCESS )
    {
         return result;
    }

    wiced_bt_hfp_hf_cb.ag_profile_uuid = UUID_SERVCLASS_AG_HANDSFREE;
    p_buf->hf_evt = WICED_BT_HFP_HF_API_CONNECT_EVT;
    wiced_bt_hfp_hf_utils_bdcpy(p_buf->bd_address, bd_address);
    GKI_send_msg(WICED_BT_HFP_HF_TASK_ID, WICED_BT_HFP_HF_TASK_MBOX, p_buf);

    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_hfp_hf_disconnect(uint16_t handle)
{
    wiced_bt_hfp_hf_api_data_t *p_buf = NULL;
    wiced_result_t result = wiced_bt_hfp_hf_init_check_and_alloc_buffer( sizeof(wiced_bt_hfp_hf_api_data_t), (void**)&p_buf );

    if ( result != WICED_SUCCESS )
    {
         return result;
    }

    p_buf->hf_evt = WICED_BT_HFP_HF_API_DISCONNECT_EVT;
    p_buf->handle = handle;
    GKI_send_msg(WICED_BT_HFP_HF_TASK_ID, WICED_BT_HFP_HF_TASK_MBOX, p_buf);

    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_hfp_hf_perform_call_action(uint16_t handle,
    wiced_bt_hfp_hf_call_action_t action, char* number)
{
    wiced_bt_hfp_hf_api_call_action_t *p_buf = NULL;
    wiced_result_t result = wiced_bt_hfp_hf_init_check_and_alloc_buffer( sizeof(wiced_bt_hfp_hf_api_call_action_t), (void**)&p_buf );

    if ( result != WICED_SUCCESS )
    {
         return result;
    }

    p_buf->hf_evt = WICED_BT_HFP_HF_API_CALL_ACTION_EVT;
    p_buf->handle = handle;
    p_buf->action = action;
    if(number != NULL)
    {
        if(strlen(number) >= WICED_BT_HFP_HF_MAX_PHONE_NUMBER_LEN)
        {
            WICED_BTHFP_ERROR("%s: phone number exceeds max\n", __FUNCTION__);
            wiced_bt_free_buffer(p_buf);
            return WICED_BADARG;
        }
        wiced_bt_hfp_hf_utils_strcpy(p_buf->number, number);
    }
    else
    {
        memset(p_buf->number, 0, sizeof(p_buf->number));
    }
    GKI_send_msg(WICED_BT_HFP_HF_TASK_ID, WICED_BT_HFP_HF_TASK_MBOX, p_buf);

    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_hfp_hf_notify_volume( uint16_t handle,
    wiced_bt_hfp_hf_volume_type_t volume_type, uint8_t volume_level)
{
    wiced_bt_hfp_hf_api_notify_vol_t *p_buf = NULL;
    wiced_result_t result = wiced_bt_hfp_hf_init_check_and_alloc_buffer( sizeof(wiced_bt_hfp_hf_api_notify_vol_t), (void**)&p_buf );

    if ( result != WICED_SUCCESS )
    {
         return result;
    }

    p_buf->hf_evt = WICED_BT_HFP_HF_API_NOTIFY_VOLUME_EVT;
    p_buf->handle = handle;
    p_buf->volume_type = volume_type;
    p_buf->volume_level = volume_level;
    GKI_send_msg(WICED_BT_HFP_HF_TASK_ID, WICED_BT_HFP_HF_TASK_MBOX, p_buf);

    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_hfp_hf_send_at_cmd(uint16_t handle, char* at_cmd)
{
    wiced_bt_hfp_hf_api_send_at_cmd_t *p_buf = NULL;
    wiced_result_t result = wiced_bt_hfp_hf_init_check_and_alloc_buffer( sizeof(wiced_bt_hfp_hf_api_send_at_cmd_t), (void**)&p_buf );

    if ( result != WICED_SUCCESS )
    {
         return result;
    }

    p_buf->hf_evt = WICED_BT_HFP_HF_API_SEND_AT_CMD_EVT;
    p_buf->handle = handle;
    if(at_cmd != NULL)
    {
        wiced_bt_hfp_hf_utils_strcpy(p_buf->at_cmd, at_cmd);
    }
    GKI_send_msg(WICED_BT_HFP_HF_TASK_ID, WICED_BT_HFP_HF_TASK_MBOX, p_buf);

    wiced_bt_free_buffer(p_buf);
    return WICED_SUCCESS;
}

/** API To get LRAC Switch data
 *
 *  Called by the application to get the LRAC Switch Data
 *
 *  @param p_opaque     Pointer to a buffer which will be filled with LRAC Switch data (current
 *                      HFP Sink State)
 *  @param p_opaque     Size of the buffer (IN), size filled (OUT)
 *
 *  @return none
 */
wiced_result_t wiced_bt_hfp_hf_lrac_switch_get(void *p_opaque, uint16_t *p_sync_data_len)
{
    if (p_opaque == NULL)
    {
        WICED_BT_TRACE("%s Err: p_opaque is NULL\n", __FUNCTION__);
        return WICED_BT_BADARG;
    }

    if (p_sync_data_len == NULL)
    {
        WICED_BT_TRACE("%s Err: p_sync_data_len is NULL\n", __FUNCTION__);
        return WICED_BT_BADARG;
    }

    if (*p_sync_data_len < (sizeof(wiced_bt_hfp_hf_cb) + sizeof(wiced_rfcomm_server_data)))
    {
        WICED_BT_TRACE("%s Err: buffer too small (%d/%d)\n", __FUNCTION__,
                *p_sync_data_len, sizeof(wiced_bt_hfp_hf_cb) + sizeof(wiced_rfcomm_server_data));
        return WICED_BT_BADARG;
    }

    /* copy the wiced_bt_hfp_hf_cb followed by wiced_rfcomm_server_data */
    memcpy(p_opaque, &wiced_bt_hfp_hf_cb, sizeof(wiced_bt_hfp_hf_cb));
    memcpy(((uint8_t *)p_opaque) + sizeof(wiced_bt_hfp_hf_cb), &wiced_rfcomm_server_data,
            sizeof(wiced_rfcomm_server_data));

    *p_sync_data_len = sizeof(wiced_bt_hfp_hf_cb) + sizeof(wiced_rfcomm_server_data);

    return WICED_BT_SUCCESS;
}

/** API To set LRAC Switch data
 *
 *  Called by the application to set the LRAC Switch Data
 *
 *  @param p_opaque     Pointer to a buffer which contains LRAC Switch data (new
 *                      HFP Sink State)
 *  @param p_opaque     Size of the buffer (IN)
 *
 *  @return none
 */
wiced_result_t wiced_bt_hfp_hf_lrac_switch_set(void *p_opaque, uint16_t sync_data_len)
{
    if (p_opaque == NULL)
    {
        WICED_BT_TRACE("%s Err: p_opaque is NULL\n", __FUNCTION__);
        return WICED_BT_BADARG;
    }

    if (sync_data_len != (sizeof(wiced_bt_hfp_hf_cb) + sizeof(wiced_rfcomm_server_data)))
    {
        WICED_BT_TRACE("%s Err: bad buffer size (%d/%d)\n", __FUNCTION__,
                sync_data_len, sizeof(wiced_bt_hfp_hf_cb) + sizeof(wiced_rfcomm_server_data));
        return WICED_BT_BADARG;
    }

    /* Copy the wiced_bt_hfp_hf_cb followed by wiced_rfcomm_server_data */
    memcpy(&wiced_bt_hfp_hf_cb, p_opaque, sizeof(wiced_bt_hfp_hf_cb));
    memcpy(&wiced_rfcomm_server_data, ((uint8_t *)p_opaque) + sizeof(wiced_bt_hfp_hf_cb),
            sizeof(wiced_rfcomm_server_data));

    return WICED_BT_SUCCESS;
}
