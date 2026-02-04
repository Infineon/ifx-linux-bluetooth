/*
 * $ Copyright Cypress Semiconductor $
 */

#include "lehs.h"

lehs_bass_t *lehs_bass_get_bass_data()
{
    return &g_lehs_gatt_cb.bass_data;
}

lehs_bass_data_t *lehs_bass_find_source_by_bda(wiced_bt_device_address_t bda)
{
    lehs_bass_t *p_bass = lehs_bass_get_bass_data();
    int index;
    for (index = 0; index < MAX_BASS; index++)
    {
        if ((p_bass->bass_data[index].is_used) &&
            !WICED_MEMCMP(p_bass->bass_data[index].recv_state.source_addr.bda, bda, BD_ADDR_LEN))
        {
            return &p_bass->bass_data[index];
        }
    }
    return NULL;
}

lehs_bass_data_t *lehs_bass_find_source_by_sync_handle(
    wiced_ble_padv_sync_handle_t sync_handle)
{
    lehs_bass_t *p_bass = lehs_bass_get_bass_data();
    int index;
    for (index = 0; index < MAX_BASS; index++)
    {
        if ((p_bass->bass_data[index].is_used) && (sync_handle == p_bass->bass_data[index].sync_handle))
        {
            return &p_bass->bass_data[index];
        }
    }
    return NULL;
}

static void lehs_fill_common_char_data(wiced_bt_ga_bass_common_source_data_t *p_source_common_param,
                                                 lehs_bass_data_t *p_bass)
{
    p_bass->recv_state.num_subgroup = p_source_common_param->num_subgroup;
    if (p_source_common_param->sub_group_data)
    {
        int index = 0;
        for (index = 0; (index < p_source_common_param->num_subgroup) && (index < WICED_BT_GA_BASS_MAX_SUBGROUP_COUNT);
             index++)
        {
            WICED_MEMCPY(&p_bass->sub_group[index],
                         &p_source_common_param->sub_group_data[index],
                         sizeof(wiced_bt_ga_bass_sub_group_data_t));
        }
    }
}

static void lehs_fill_char_data(wiced_bt_ga_bass_add_source_t *p_source_param,
                                          lehs_bass_data_t *p_bass)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);

    p_bass->recv_state.adv_sid = p_source_param->adv_sid;

    WICED_BT_TRACE("[%s] sync state \n", __FUNCTION__);
    WICED_BT_TRACE("[%s] subgroup %d", __FUNCTION__, p_source_param->src_data.num_subgroup);

    p_bass->recv_state.sub_group_data = p_bass->sub_group;

    WICED_BT_TRACE("[%s] bis sync state %d", __FUNCTION__, p_source_param->src_data.sub_group_data[0].bis_sync_state);
    p_bass->recv_state.sub_group_data[0].bis_sync_state =
        p_source_param->src_data.sub_group_data[0].bis_sync_state;

    p_bass->recv_state.broadcast_id = p_source_param->broadcast_id;
    WICED_MEMCPY(&p_bass->recv_state.source_addr,
                 &p_source_param->source_addr,
                 sizeof(wiced_bt_ble_address_t));
    lehs_fill_common_char_data(&p_source_param->src_data, p_bass);
}

lehs_bass_data_t *lehs_allocate_source(wiced_bt_ga_bass_add_source_t *p_source_param,
                                                           uint8_t *char_instance)
{
    lehs_bass_t *p_bass = lehs_bass_get_bass_data();
    int index;
    WICED_BT_TRACE("[%s] \n", __FUNCTION__);
    for (index = 0; index < MAX_BASS; index++)
    {
        if (!p_bass->bass_data[index].is_used)
        {
            p_bass->bass_data[index].is_used = WICED_TRUE;
            p_bass->bass_data[index].recv_state.source_id = index;
            lehs_fill_char_data(p_source_param, &p_bass->bass_data[index]);
            *char_instance = index;
            return &p_bass->bass_data[index];
        }
    }
    return NULL;
}

lehs_bass_data_t *lehs_find_source(uint8_t source_id, uint8_t *char_instance)
{
    lehs_bass_t *p_bass = lehs_bass_get_bass_data();
    int index;
    for (index = 0; index < MAX_BASS; index++)
    {
        if ((p_bass->bass_data[index].is_used) &&
            (p_bass->bass_data[index].recv_state.source_id == source_id))
        {
            *char_instance = index;
            return &p_bass->bass_data[index];
        }
    }
    return NULL;
}

static void lehs_notify_recv_state_char(uint16_t conn_id,
                                                  const gatt_intf_service_object_t *p_service,
                                                  lehs_bass_data_t *p_bass,
                                                  uint8_t char_instance)
{
    gatt_intf_attribute_t characteristic = {0};
    wiced_bt_ga_bass_receive_state_t data;
    WICED_MEMSET(&data, 0, sizeof(wiced_bt_ga_bass_receive_state_t));

    characteristic.characteristic_type = BASS_BROADCAST_RECEIVE_STATE_CHARACTERISTIC;
    characteristic.characteristic_instance = char_instance;

    WICED_MEMCPY(&data, &p_bass->recv_state, sizeof(wiced_bt_ga_bass_receive_state_t));
    data.sub_group_data = p_bass->sub_group;

    if (p_service) gatt_interface_notify_characteristic(conn_id, (gatt_intf_service_object_t*)p_service, &characteristic, &data);
}

static void set_pa_sync_state(lehs_bass_data_t* p_bass, uint8_t state)
{
    WICED_BT_TRACE("[%s] pa sync state %d updated state %d",
                   __FUNCTION__,
                   p_bass->recv_state.pa_sync_state,
                   state);
    p_bass->recv_state.pa_sync_state = state;
}

static void set_bis_sync_state(lehs_bass_data_t *p_bass, uint32_t bis_sync_state)
{
    WICED_BT_TRACE("[%s] bis sync state %d updated state %d",
                   __FUNCTION__,
                   p_bass->recv_state.sub_group_data->bis_sync_state,
                   bis_sync_state);
    p_bass->recv_state.sub_group_data->bis_sync_state = bis_sync_state; // BIS bit fields
}

static uint8_t get_pa_sync_state(lehs_bass_data_t *p_bass)
{
    return p_bass->recv_state.pa_sync_state;
}

static uint32_t get_bis_sync_state(lehs_bass_data_t *p_bass)
{
    return p_bass->recv_state.sub_group_data->bis_sync_state;
}

static void set_big_encryption_state(lehs_bass_data_t *p_bass, uint8_t state)
{
    WICED_BT_TRACE("[%s] big encryption state %d updated state %d",
                   __FUNCTION__,
                   p_bass->recv_state.big_encryption_state,
                   state);
    p_bass->recv_state.big_encryption_state = state;
}

wiced_result_t lehs_bass_handle_write_req_evt(uint16_t conn_id,
                                                        const gatt_intf_service_object_t *p_service,
                                                        wiced_bt_gatt_status_t status,
                                                        uint32_t evt_type,
                                                        gatt_intf_attribute_t *p_char,
                                                        uint8_t *p_evt_data,
                                                        uint16_t len)
{
    wiced_result_t result = WICED_SUCCESS;
    lehs_bass_t *p_data = lehs_bass_get_bass_data();
    wiced_bt_ga_bass_operation_t *p_op_data = &p_data->op_data;
    WICED_BT_TRACE("[%s] event [%d]\n", __FUNCTION__, evt_type);

    result = wiced_bt_ga_bass_parse_control_point_data(p_evt_data, len, p_op_data);

    if (result != WICED_BT_SUCCESS) return result;

    switch (p_op_data->opcode)
    {
        case WICED_BT_GA_BASS_OP_ADD_SOURCE: {
            uint8_t char_instance = 0;

            wiced_bt_ga_bass_add_source_t *p_add_src = &p_op_data->data.add_source_param;

            WICED_BT_TRACE("[%s] conn id %x boradcast id %x source addr %B adv sid %d pa_sync_param %d\n",
                           __FUNCTION__,
                           conn_id,
                           p_add_src->broadcast_id,
                           p_add_src->source_addr.bda,
                           p_add_src->adv_sid,
                           p_add_src->src_data.pa_sync_param);
            lehs_bass_data_t *p_bass = lehs_allocate_source(p_add_src, &char_instance);
            lehs_bis_alloc_big(p_add_src->broadcast_id,
                               p_add_src->source_addr.bda, p_add_src->adv_sid);

            uint8_t *br_name = NULL;
            lehs_rpc_send_new_stream_info(p_add_src->broadcast_id, br_name);

            if (p_bass)
            {
                if (p_add_src->src_data.pa_sync_param == WICED_BT_GA_BASS_PA_SYNC_USING_PAST)
                {
                    wiced_ble_padv_sync_transfer_param_t sync_transfer_param = {
                        .mode = WICED_BLE_PERIODIC_ENABLE_PA_SYNC_TRANSFER_ENABLE_PA_REPORT_EVT,
                        .skip = 0,
                        .sync_cte_type = 0,
                        .sync_timeout = 0x1770 };
                    wiced_bt_device_address_t bdaddr;
                    wiced_bt_gatt_get_device_address(conn_id, &bdaddr, NULL, NULL);
                    wiced_bt_dev_status_t status =
                        wiced_ble_padv_set_sync_transfer_params(bdaddr, &sync_transfer_param);
                    WICED_BT_TRACE("[%s] set sync transfer param %d", __FUNCTION__, status );
                    set_pa_sync_state(p_bass, WICED_BT_GA_BASS_PA_SYNC_INFO_REQUEST);
                }
                else if (p_add_src->src_data.pa_sync_param == WICED_BT_GA_BASS_PA_SYNC_NO_PAST)
                {
                    lehs_sync_to_pa(p_add_src->broadcast_id);
                }
                set_big_encryption_state(p_bass, WICED_BT_GA_BASS_BIG_NOT_ENCRYPTED);

                // Notify
                lehs_notify_recv_state_char(conn_id, p_service, p_bass, char_instance);
            }
        }
        break;
        case WICED_BT_GA_BASS_OP_MODIFY_SOURCE: {

            uint8_t char_instance = 0;
            wiced_bt_ga_bass_modify_source_t *p_modify_src = &p_op_data->data.modify_source_param;
            lehs_bass_data_t *p_bass = lehs_find_source(p_modify_src->source_id, &char_instance);
            if (p_bass)
            {
                if (get_bis_sync_state(p_bass) != p_modify_src->src_data.sub_group_data->bis_sync_state)
                {
                    wiced_ble_isoc_peripheral_big_terminate_sync(p_bass->recv_state.adv_sid);
                    set_bis_sync_state(p_bass, 0);
                }
                if (p_modify_src->src_data.pa_sync_param == WICED_BT_GA_BASS_PA_NO_SYNC)
                {
                    if (get_pa_sync_state(p_bass) == WICED_BT_GA_BASS_PA_SYNC)
                        wiced_ble_padv_terminate_sync(p_bass->sync_handle);
                    set_pa_sync_state(p_bass, WICED_BT_GA_BASS_PA_NOT_SYNC);
                }
                lehs_notify_recv_state_char(conn_id, p_service, p_bass, char_instance);
            }
            else
            {
                result = WICED_BT_GA_BASS_ERROR_INVALID_SOURCE_ID;
            }
        }
        break;
        case WICED_BT_GA_BASS_OP_REMOVE_SOURCE: {
            uint8_t char_instance = 0;
            lehs_bass_data_t *p_bass =
                lehs_find_source(p_op_data->data.remove_source_id, &char_instance);
            /*
            * The server shall not accept a Remove Source operation for a Source_ID value that matches the Source_ID written by the client
            *  in the Remove Source operation if the server is synchronized to the PA and/or any BIS as defined by the values of the PA_Sync_State
            *  and BIS_Sync_State fields in the Broadcast Receive State characteristic containing that Source_ID value.
            */
            if (p_bass &&
                (get_pa_sync_state(p_bass) != WICED_BT_GA_BASS_PA_SYNC))
            {
                broadcast_sink_cb_t *p_big = lehs_bis_get_big_by_broadcast_id(p_bass->recv_state.broadcast_id);
                if (p_big)
                {
                    WICED_MEMSET(p_big, 0, sizeof(broadcast_sink_cb_t));
                }
                WICED_MEMSET(p_bass, 0, sizeof(lehs_bass_data_t));
                lehs_notify_recv_state_char(conn_id, p_service, p_bass, char_instance);
            }
            else
            {
                result = WICED_BT_GA_BASS_ERROR_INVALID_SOURCE_ID;
            }
        }
        break;
        case WICED_BT_GA_BASS_OP_SET_BROADCAST_CODE: {
            uint8_t char_instance = 0;
            lehs_bass_data_t *p_bass =
                lehs_find_source(p_op_data->data.set_broadcast_param.source_id, &char_instance);
            if (p_bass)
            {
                WICED_MEMCPY(p_bass->recv_state.broadcast_code,
                             p_op_data->data.set_broadcast_param.broadcast_code,
                             BAP_BROADCAST_CODE_SIZE);
                p_bass->waiting_broadcast_code = WICED_FALSE;
                set_big_encryption_state(p_bass, WICED_BT_GA_BASS_BIG_DECRPTING);
                lehs_notify_recv_state_char(conn_id, p_service, p_bass, char_instance);

                //sync to stream
                broadcast_sink_cb_t *p_big =
                    lehs_bis_get_big_by_broadcast_id(p_bass->recv_state.broadcast_id);
                lehs_sync_to_source(p_big, p_bass->recv_state.sub_group_data->bis_sync_state);
            }
        }
        break;
    }
    return result;
}

wiced_result_t lehs_bass_handle_read_req_evt(uint16_t conn_id,
                                                       const gatt_intf_service_object_t *p_service,
                                                       wiced_bt_gatt_status_t status,
                                                       uint32_t evt_type,
                                                       gatt_intf_attribute_t *p_char,
                                                       wiced_bt_ga_bass_receive_state_t *p_recv_data)
{
    WICED_BT_TRACE("[%s] event \n", __FUNCTION__);
    wiced_result_t result = WICED_ERROR;
    lehs_bass_t *p_bass = lehs_bass_get_bass_data();
    if (p_char->characteristic_type == BASS_BROADCAST_RECEIVE_STATE_CHARACTERISTIC)
    {
        if (p_char->characteristic_instance <= MAX_BASS)
        {
            lehs_bass_data_t *p_app_data = &(p_bass->bass_data[p_char->characteristic_instance]);
            if (p_app_data->is_used)
            {
                // Provide data to profile
                WICED_MEMCPY(p_recv_data, &p_app_data->recv_state, sizeof(wiced_bt_ga_bass_receive_state_t));
                p_recv_data->sub_group_data = p_app_data->sub_group;
                result = WICED_SUCCESS;
            }
        }
    }
    return result;
}

wiced_result_t lehs_bass_callback(uint16_t conn_id,
                                            void *p_app_ctx,
                                            gatt_intf_service_object_t *p_service,
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
            result = lehs_bass_handle_write_req_evt(conn_id,
                                                              p_service,
                                                              status,
                                                              evt_type,
                                                              p_char,
                                                              (uint8_t *)p_data,
                                                              len);
            break;
        case READ_REQ_EVT:
            result = lehs_bass_handle_read_req_evt(conn_id,
                                                             p_service,
                                                             status,
                                                             evt_type,
                                                             p_char,
                                                             (wiced_bt_ga_bass_receive_state_t *)p_data);
            break;
    }

    return result;
}

void lehs_bass_notify_pa_sync_state(wiced_ble_padv_sync_established_event_data_t *p_sync)
{
    gatt_intf_service_object_t *p_service = lehs_gatt_get_bass_service_instance();

    if (p_sync->status == 0)
    {
        lehs_bass_data_t *p_bass = lehs_bass_find_source_by_bda(p_sync->adv_addr);
        if (p_bass)
        {
            set_pa_sync_state(p_bass, WICED_BT_GA_BASS_PA_SYNC);
            p_bass->sync_handle = p_sync->sync_handle;
            // Notify PA Sync State
            lehs_notify_recv_state_char(p_bass->conn_id, p_service, p_bass, 0);
        }
    }
}

void lehs_bass_broadcast_code_check(uint16_t sync_handle)
{
    wiced_bt_bap_broadcast_code_t null_broadcast_code ={0};

    gatt_intf_service_object_t *p_service = lehs_gatt_get_bass_service_instance();
    lehs_bass_data_t *p_bass = lehs_bass_find_source_by_sync_handle(sync_handle);

    WICED_BT_TRACE("[%s]\n", __FUNCTION__);

    if (!p_bass->waiting_broadcast_code &&
        !WICED_MEMCMP(p_bass->recv_state.broadcast_code, null_broadcast_code, BAP_BROADCAST_CODE_SIZE))
    {
        WICED_BT_TRACE("[%s] encryption is enabled broadcast code required!\n", __FUNCTION__);
        p_bass->waiting_broadcast_code = WICED_TRUE;
        set_big_encryption_state(p_bass, WICED_BT_GA_BASS_BIG_BROADCAST_CODE_REQUIRED);

        // Notify Broadcast Code required
        lehs_notify_recv_state_char(p_bass->conn_id, p_service, p_bass, 0);
    }
}

void lehs_bass_notify_sync_established(uint8_t *p_addr)
{
    gatt_intf_service_object_t *p_service = lehs_gatt_get_bass_service_instance();

    lehs_bass_data_t *p_bass = lehs_bass_find_source_by_bda(p_addr);
    if (p_bass)
    {
        set_pa_sync_state(p_bass, WICED_BT_GA_BASS_PA_NOT_SYNC);
        // Notify PA Sync State
        lehs_notify_recv_state_char(p_bass->conn_id, p_service, p_bass, 0);
    }
}

void lehs_bass_notify_big_sync_lost(uint8_t *p_addr)
{
    gatt_intf_service_object_t *p_service = lehs_gatt_get_bass_service_instance();

    lehs_bass_data_t *p_bass = lehs_bass_find_source_by_bda(p_addr);
    if (p_bass)
    {
        // Notify BIS Sync State
        set_bis_sync_state(p_bass, 0);
        lehs_notify_recv_state_char(p_bass->conn_id, p_service, p_bass, 0);

        // Notify BIG Sync State
        WICED_MEMSET(p_bass, 0, sizeof(lehs_bass_data_t));
        lehs_notify_recv_state_char(p_bass->conn_id, p_service, p_bass, 0);
    }
}

void lehs_bass_notify_pa_sync_lost(uint16_t sync_handle)
{
    gatt_intf_service_object_t *p_service = lehs_gatt_get_bass_service_instance();
    lehs_bass_data_t *p_bass = lehs_bass_find_source_by_sync_handle(sync_handle);

    WICED_BT_TRACE("[%s] service %x bass %x\n", __FUNCTION__, p_service, p_bass);

    if (p_bass)
    {
        set_pa_sync_state(p_bass, WICED_BT_GA_BASS_PA_NO_SYNC);
        // Notify PA Sync State
        lehs_notify_recv_state_char(p_bass->conn_id, p_service, p_bass, 0);
    }
}

