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

/******************************************************************************/

/* Application includes */
#include "broadcast_sink_bass.h"
#include "broadcast_sink_bis.h"
#include "broadcast_sink_gatt.h"
#include "broadcast_sink_rpc.h"

/* BT Stack includes */
#include "wiced_bt_ga_bass.h"
#include "wiced_bt_trace.h"
#define MAX_BIG 2

broadcast_sink_bass_t g_broadcast_sink_bass_data;

broadcast_sink_bass_t *broadcast_sink_bass_get_bass_data()
{
    return &g_broadcast_sink_bass_data;
}

broadcast_sink_bass_data_t *broadcast_sink_bass_find_source_by_bda(wiced_bt_device_address_t bda)
{
    broadcast_sink_bass_t *p_bass_data = &g_broadcast_sink_bass_data;
    int index;
    for (index = 0; index < MAX_BASS; index++)
    {
        if ((p_bass_data->bass_data[index].is_used) &&
            !WICED_MEMCMP(p_bass_data->bass_data[index].recv_state_data.source_addr.bda, bda, BD_ADDR_LEN))
        {
            return &p_bass_data->bass_data[index];
        }
    }
    return NULL;
}

broadcast_sink_bass_data_t *broadcast_sink_find_source_by_sync_handle(
    wiced_bt_ble_periodic_adv_sync_handle_t sync_handle)
{
    broadcast_sink_bass_t *p_bass_data = &g_broadcast_sink_bass_data;
    int index;
    for (index = 0; index < MAX_BASS; index++)
    {
        if ((p_bass_data->bass_data[index].is_used) && (sync_handle == p_bass_data->bass_data[index].sync_handle))
        {
            return &p_bass_data->bass_data[index];
        }
    }
    return NULL;
}

static void broadcast_sink_fill_common_char_data(wiced_bt_ga_bass_common_source_data_t *p_source_common_param,
                                                 broadcast_sink_bass_data_t *p_bass_data)
{
    p_bass_data->recv_state_data.num_subgroup = p_source_common_param->num_subgroup;
    if (p_source_common_param->num_subgroup > 0)
    {
        int index = 0;
        for (index = 0; (index < p_source_common_param->num_subgroup) && (index < WICED_BT_GA_BASS_MAX_SUBGROUP_COUNT);
             index++)
        {
            WICED_MEMCPY(&p_bass_data->sub_group_data[index],
                         &p_source_common_param->sub_group_data[index],
                         sizeof(wiced_bt_ga_bass_sub_group_data_t));
        }
    }
}

static void broadcast_sink_fill_char_data(wiced_bt_ga_bass_add_source_data_t *p_source_param,
                                          broadcast_sink_bass_data_t *p_bass_data)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);

    p_bass_data->recv_state_data.adv_sid = p_source_param->adv_sid;

    WICED_BT_TRACE("[%s] sync state \n", __FUNCTION__);
    WICED_BT_TRACE("[%s] subgroup %d", __FUNCTION__, p_source_param->src_data.num_subgroup);

    p_bass_data->recv_state_data.sub_group_data = p_bass_data->sub_group_data;

    WICED_BT_TRACE("[%s] bis sync state %d", __FUNCTION__, p_source_param->src_data.sub_group_data[0].bis_sync_state);
    p_bass_data->recv_state_data.sub_group_data[0].bis_sync_state =
        p_source_param->src_data.sub_group_data[0].bis_sync_state;

    p_bass_data->recv_state_data.broadcast_id = p_source_param->broadcast_id;
    WICED_MEMCPY(&p_bass_data->recv_state_data.source_addr,
                 &p_source_param->source_addr,
                 sizeof(wiced_bt_ble_address_t));
    broadcast_sink_fill_common_char_data(&p_source_param->src_data, p_bass_data);
}

broadcast_sink_bass_data_t *broadcast_sink_allocate_source(wiced_bt_ga_bass_add_source_data_t *p_source_param,
                                                           uint8_t *char_instance)
{
    broadcast_sink_bass_t *p_bass_data = broadcast_sink_bass_get_bass_data();
    int index;
    WICED_BT_TRACE("[%s] \n", __FUNCTION__);
    for (index = 0; index < MAX_BASS; index++)
    {
        if (!p_bass_data->bass_data[index].is_used)
        {
            p_bass_data->bass_data[index].is_used = WICED_TRUE;
            p_bass_data->bass_data[index].recv_state_data.source_id = index;
            broadcast_sink_fill_char_data(p_source_param, &p_bass_data->bass_data[index]);
            *char_instance = index;
            return &p_bass_data->bass_data[index];
        }
    }
    return NULL;
}

broadcast_sink_bass_data_t *broadcast_sink_find_source(uint8_t source_id, uint8_t *char_instance)
{
    broadcast_sink_bass_t *p_bass_data = broadcast_sink_bass_get_bass_data();
    int index;
    for (index = 0; index < MAX_BASS; index++)
    {
        if ((p_bass_data->bass_data[index].is_used) &&
            (p_bass_data->bass_data[index].recv_state_data.source_id == source_id))
        {
            *char_instance = index;
            return &p_bass_data->bass_data[index];
        }
    }
    return NULL;
}

static void broadcast_sink_notify_recv_state_char(uint16_t conn_id,
                                                  const gatt_intf_service_object_t *p_service,
                                                  broadcast_sink_bass_data_t *p_bass_data,
                                                  uint8_t char_instance)
{
    gatt_intf_attribute_t characteristic = {0};
    wiced_bt_ga_bass_receive_state_data_t data;
    WICED_MEMSET(&data, 0, sizeof(wiced_bt_ga_bass_receive_state_data_t));

    characteristic.characteristic_type = BASS_BROADCAST_RECEIVE_STATE_CHARACTERISTIC;
    characteristic.characteristic_instance = char_instance;

    WICED_MEMCPY(&data, &p_bass_data->recv_state_data, sizeof(wiced_bt_ga_bass_receive_state_data_t));
    data.sub_group_data = p_bass_data->sub_group_data;

    if (p_service) gatt_interface_notify_characteristic(conn_id, (gatt_intf_service_object_t*)p_service, &characteristic, &data);
}

void broadcast_sink_sync_to_source(broadcast_sink_cb_t *p_big, uint8_t *p_brdcst_code)
{
    wiced_bt_isoc_big_create_sync_t create_sync = {0};
    uint8_t bis_idx = 1;
    if (!p_big) return;

    WICED_BT_TRACE("[%s] event sync handle[%d]\n", __FUNCTION__, p_big->sync_handle);

    create_sync.big_handle = p_big->big_handle;
    create_sync.sync_handle = p_big->sync_handle;
    create_sync.max_sub_events = p_big->number_of_subevents;
    create_sync.big_sync_timeout = 0;

    create_sync.num_bis = 1;
    create_sync.bis_idx_list = &bis_idx;

    create_sync.encrypt = p_big->b_encryption;
    create_sync.broadcast_code = (p_big->b_encryption) ? p_brdcst_code : NULL;
    WICED_BT_TRACE("[%s] calling wiced_bt_isoc_peripheral_big_create_sync\n", __FUNCTION__);
    wiced_bt_isoc_peripheral_big_create_sync(&create_sync);
}

wiced_result_t broadcast_sink_bass_handle_write_req_evt(uint16_t conn_id,
                                                        const gatt_intf_service_object_t *p_service,
                                                        wiced_bt_gatt_status_t status,
                                                        uint32_t evt_type,
                                                        gatt_intf_attribute_t *p_char,
                                                        uint8_t *p_evt_data,
                                                        uint16_t len)
{
    wiced_result_t result = WICED_SUCCESS;
    broadcast_sink_bass_t *p_data = broadcast_sink_bass_get_bass_data();
    wiced_bt_ga_bass_operation_t *p_operational_data = &p_data->operation_data;
    WICED_BT_TRACE("[%s] event [%d]\n", __FUNCTION__, evt_type);

    result = wiced_bt_ga_bass_parse_control_point_data(p_evt_data, len, p_operational_data);

    if (result != WICED_BT_GATT_SUCCESS) return result;

    switch (p_operational_data->opcode)
    {
        case WICED_BT_GA_BASS_OP_ADD_SOURCE: {
            uint8_t char_instance = 0;

            WICED_BT_TRACE("[%s] conn id %x boradcast id %x source addr %B adv sid %d pa_sync_param %d\n",
                           __FUNCTION__,
                           conn_id,
                           p_operational_data->data.add_source_param.broadcast_id,
                           p_operational_data->data.add_source_param.source_addr.bda,
                           p_operational_data->data.add_source_param.adv_sid,
                           p_operational_data->data.add_source_param.src_data.pa_sync_param);
            broadcast_sink_bass_data_t *p_bass_data =
                broadcast_sink_allocate_source(&p_operational_data->data.add_source_param, &char_instance);
            broadcast_sink_bis_alloc_big(p_operational_data->data.add_source_param.broadcast_id,
                                         p_operational_data->data.add_source_param.source_addr.bda,
                                         p_operational_data->data.add_source_param.adv_sid);

            uint8_t *br_name = NULL;
            broadcast_sink_rpc_send_new_stream_info(p_operational_data->data.add_source_param.broadcast_id, br_name);

            if (p_bass_data)
            {
                if (p_operational_data->data.add_source_param.src_data.pa_sync_param == WICED_BT_GA_BASS_PA_SYNC_USING_PAST)
                {
                    p_bass_data->recv_state_data.pa_sync_state = WICED_BT_GA_BASS_PA_SYNC_INFO_REQUST;
                }
                else if (p_operational_data->data.add_source_param.src_data.pa_sync_param == WICED_BT_GA_BASS_PA_SYNC_NO_PAST)
                {
                    broadcast_source_t br_source;
                    WICED_MEMSET((uint8_t *) & br_source, 0, sizeof(broadcast_source_t));
                    br_source.broadcast_id = p_operational_data->data.add_source_param.broadcast_id;
                    broadcast_sink_bis_sync_to_source(BTM_BLE_SCAN_TYPE_HIGH_DUTY, br_source);
                }
                p_bass_data->recv_state_data.big_encryption_state = WICED_BT_GA_BASS_BIG_NOT_ENCRYPTED;

                // Notify
                broadcast_sink_notify_recv_state_char(conn_id, p_service, p_bass_data, char_instance);
            }
        }
        break;
        case WICED_BT_GA_BASS_OP_MODIFY_SOURCE: {

            uint8_t char_instance = 0;
            broadcast_sink_bass_data_t *p_bass_data =
                broadcast_sink_find_source(p_operational_data->data.modify_source_param.source_id, &char_instance);
            if (p_bass_data)
            {
                broadcast_sink_fill_common_char_data(&p_operational_data->data.modify_source_param.src_data,
                                                     p_bass_data);
                broadcast_sink_notify_recv_state_char(conn_id, p_service, p_bass_data, char_instance);
            }
            else
            {
                result = WICED_BT_GA_BASS_ERROR_INVALID_SOURCE_ID;
            }
        }
        break;
        case WICED_BT_GA_BASS_OP_REMOVE_SOURCE: {
            uint8_t char_instance = 0;
            broadcast_sink_bass_data_t *p_bass_data =
                broadcast_sink_find_source(p_operational_data->data.remove_source_id, &char_instance);
            /*
            * The server shall not accept a Remove Source operation for a Source_ID value that matches the Source_ID written by the client
            *  in the Remove Source operation if the server is synchronized to the PA and/or any BIS as defined by the values of the PA_Sync_State
            *  and BIS_Sync_State fields in the Broadcast Receive State characteristic containing that Source_ID value.
            */
            if (p_bass_data && (p_bass_data->recv_state_data.pa_sync_state != WICED_BT_GA_BASS_PA_SYNC))
            {
                WICED_MEMSET(p_bass_data, 0, sizeof(broadcast_sink_bass_data_t));
                p_bass_data->is_used = WICED_FALSE;
            }
            else
            {
                result = WICED_BT_GA_BASS_ERROR_INVALID_SOURCE_ID;
            }
        }
        break;
        case WICED_BT_GA_BASS_OP_SET_BROADCAST_CODE: {
            uint8_t char_instance = 0;
            broadcast_sink_bass_data_t *p_bass_data =
                broadcast_sink_find_source(p_operational_data->data.set_broadcast_param.source_id, &char_instance);
            if (p_bass_data)
            {
                WICED_MEMCPY(p_bass_data->recv_state_data.broadcast_code,
                             p_operational_data->data.set_broadcast_param.broadcast_code,
                             BAP_BROADCAST_CODE_SIZE);
                p_bass_data->waiting_broadcast_code = WICED_FALSE;
                p_bass_data->recv_state_data.big_encryption_state = WICED_BT_GA_BASS_BIG_DECRPTING;
                broadcast_sink_notify_recv_state_char(conn_id, p_service, p_bass_data, char_instance);

                //sync to stream
                broadcast_sink_cb_t *p_big =
                    broadcast_sink_bis_get_big_by_broadcast_id(p_bass_data->recv_state_data.broadcast_id);
                broadcast_sink_sync_to_source(p_big, p_bass_data->recv_state_data.broadcast_code);
            }
        }
        break;
    }
    return result;
}

wiced_result_t broadcast_sink_bass_handle_read_req_evt(uint16_t conn_id,
                                                       const gatt_intf_service_object_t *p_service,
                                                       wiced_bt_gatt_status_t status,
                                                       uint32_t evt_type,
                                                       gatt_intf_attribute_t *p_char,
                                                       wiced_bt_ga_bass_receive_state_data_t *p_recv_data)
{
    WICED_BT_TRACE("[%s] event \n", __FUNCTION__);
    wiced_result_t result = WICED_ERROR;
    broadcast_sink_bass_t *p_bass = broadcast_sink_bass_get_bass_data();
    if (p_char->characteristic_type == BASS_BROADCAST_RECEIVE_STATE_CHARACTERISTIC)
    {
        if (p_char->characteristic_instance < MAX_BASS)
        {
             broadcast_sink_bass_data_t *p_app_data = &(p_bass->bass_data[p_char->characteristic_instance]);
             if (p_app_data && p_app_data->is_used)
             {
                // Provide data to profile
                WICED_MEMCPY(p_recv_data, &p_app_data->recv_state_data, sizeof(wiced_bt_ga_bass_receive_state_data_t));
                p_recv_data->sub_group_data = p_app_data->sub_group_data;
                 result = WICED_SUCCESS;
             }
        }
    }
    return result;
}

wiced_result_t broadcast_sink_bass_callback(uint16_t conn_id,
                                            void *p_app_ctx,
                                            const gatt_intf_service_object_t *p_service,
                                            wiced_bt_gatt_status_t status,
                                            uint32_t evt_type,
                                            gatt_intf_attribute_t *p_char,
                                            void *p_data,
                                            int len)
{
    wiced_result_t result = WICED_SUCCESS;

    WICED_BT_TRACE("[%s] event 0x%x \n", __FUNCTION__, p_char);

    switch (evt_type)
    {
        case WRITE_REQ_EVT:
            result = broadcast_sink_bass_handle_write_req_evt(conn_id,
                                                              p_service,
                                                              status,
                                                              evt_type,
                                                              p_char,
                                                              (uint8_t *)p_data,
                                                              len);
            break;
        case READ_REQ_EVT:
            result = broadcast_sink_bass_handle_read_req_evt(conn_id,
                                                             p_service,
                                                             status,
                                                             evt_type,
                                                             p_char,
                                                             (wiced_bt_ga_bass_receive_state_data_t *)p_data);
            break;
    }

    return result;
}

void broadcast_sink_bass_notify_pa_sync_state(wiced_bt_ble_periodic_adv_sync_established_event_data_t *p_sync)
{
    gatt_intf_service_object_t *p_service = broadcast_sink_gatt_get_bass_service_instance();

    if (p_sync->status == 0)
    {
        broadcast_sink_bass_data_t *p_bass_data = broadcast_sink_bass_find_source_by_bda(p_sync->adv_addr);
        if (p_bass_data)
        {
            p_bass_data->recv_state_data.pa_sync_state = WICED_BT_GA_BASS_PA_SYNC;
            p_bass_data->sync_handle = p_sync->sync_handle;
            // Notify PA Sync State
            broadcast_sink_notify_recv_state_char(p_bass_data->conn_id, p_service, p_bass_data, 0);
        }
    }
}

void broadcast_sink_bass_broadcast_code_check(uint16_t sync_handle)
{
    wiced_bt_bap_broadcast_code_t null_broadcast_code ={0};

    gatt_intf_service_object_t *p_service = broadcast_sink_gatt_get_bass_service_instance();
    broadcast_sink_bass_data_t *p_bass_data = broadcast_sink_find_source_by_sync_handle(sync_handle);

    WICED_BT_TRACE("[%s]\n", __FUNCTION__);

    if (!p_bass_data->waiting_broadcast_code &&
        !WICED_MEMCMP(p_bass_data->recv_state_data.broadcast_code, null_broadcast_code, BAP_BROADCAST_CODE_SIZE))
    {
        WICED_BT_TRACE("[%s] encryption is enabled broadcast code required!\n", __FUNCTION__);
        p_bass_data->waiting_broadcast_code = WICED_TRUE;
        p_bass_data->recv_state_data.big_encryption_state = WICED_BT_GA_BASS_BIG_BROADCAST_CODE_REQUIRED;

        // Notify Broadcast Code required
        broadcast_sink_notify_recv_state_char(p_bass_data->conn_id, p_service, p_bass_data, 0);
    }
}

void broadcast_sink_bass_notify_sync_established(uint8_t *p_addr)
{
    gatt_intf_service_object_t *p_service = broadcast_sink_gatt_get_bass_service_instance();

    broadcast_sink_bass_data_t *p_bass_data = broadcast_sink_bass_find_source_by_bda(p_addr);
    if (p_bass_data)
    {
        p_bass_data->recv_state_data.pa_sync_state = WICED_BT_GA_BASS_PA_SYNC;
        p_bass_data->sub_group_data[0].bis_sync_state =
            p_bass_data->sub_group_data[0].bis_sync_state | WICED_BT_GA_BASS_SYNC_EST_MASK;
        // Notify PA Sync State
        broadcast_sink_notify_recv_state_char(p_bass_data->conn_id, p_service, p_bass_data, 0);
    }
}

void broadcast_sink_bass_notify_big_sync_lost(uint8_t *p_addr)
{
    gatt_intf_service_object_t *p_service = broadcast_sink_gatt_get_bass_service_instance();

    broadcast_sink_bass_data_t *p_bass_data = broadcast_sink_bass_find_source_by_bda(p_addr);
    if (p_bass_data)
    {
        p_bass_data->recv_state_data.pa_sync_state = WICED_BT_GA_BASS_PA_SYNC;
        p_bass_data->sub_group_data[0].bis_sync_state =
            p_bass_data->sub_group_data[0].bis_sync_state & WICED_BT_GA_BASS_SYNC_LOST_MASK;
        // Notify PA Sync State
        broadcast_sink_notify_recv_state_char(p_bass_data->conn_id, p_service, p_bass_data, 0);
    }
}

void broadcast_sink_bass_notify_pa_sync_lost(uint16_t sync_handle)
{
    gatt_intf_service_object_t *p_service = broadcast_sink_gatt_get_bass_service_instance();
    broadcast_sink_bass_data_t *p_bass_data = broadcast_sink_find_source_by_sync_handle(sync_handle);

    WICED_BT_TRACE("[%s]\n", __FUNCTION__);

    if (p_bass_data)
    {
        p_bass_data->recv_state_data.pa_sync_state = WICED_BT_GA_BASS_PA_NO_SYNC;
        // Notify PA Sync State
        broadcast_sink_notify_recv_state_char(p_bass_data->conn_id, p_service, p_bass_data, 0);
    }
}

void broadcast_sink_bass_request_pa_sync_info()
{
    broadcast_sink_bass_t *p_bass_data = broadcast_sink_bass_get_bass_data();
    gatt_intf_service_object_t *p_service = broadcast_sink_gatt_get_bass_service_instance();
    p_bass_data->bass_data[0].recv_state_data.pa_sync_state = WICED_BT_GA_BASS_PA_SYNC_INFO_REQUST;
    p_bass_data->bass_data[0].recv_state_data.big_encryption_state = WICED_BT_GA_BASS_BIG_NOT_ENCRYPTED;

    // Notify
    broadcast_sink_notify_recv_state_char(p_bass_data->bass_data[0].conn_id, p_service, &p_bass_data->bass_data[0], 0);
}
