/******************************************************************************
* $ Copyright 2022-YEAR Cypress Semiconductor $
******************************************************************************/
/******************************************************************************
 * File Name: hello_sensor_gatt.c
 *
 * Description: Hello Sensor GATT source file.
 *
 * Related Document: See README.md
 *
******************************************************************************/

/* This file is applicable for all devices with BTSTACK version 3.0 and greater, for example 55572 */
#include "hello_sensor.h"
#include "app_bt_utils.h"
#include "wiced_bt_trace.h"
#include "hello_sensor_gatt_cfg.h"
#include "log.h"

#ifdef TAG
#undef TAG
#define TAG "[hello_sensor_gatt]"
#endif


/******************************************************************************
 *                      Extern functions
 *****************************************************************************/
attribute_t * hello_sensor_get_attribute( uint16_t handle );
void hello_sensor_send_message( void );
wiced_bt_gatt_status_t hello_sensor_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status );
wiced_bt_gatt_status_t hello_sensor_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_req );
wiced_bt_gatt_status_t hello_sensor_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
wiced_bt_gatt_status_t hello_sensor_gatt_op_comp_cb( wiced_bt_gatt_operation_complete_t *p_data );

/******************************************************************************
 *                      Extern variables
 *****************************************************************************/
extern wiced_bt_cfg_ble_t hello_sensor_cfg_ble;
extern hello_sensor_state_t hello_sensor_state;
extern host_info_t hello_sensor_hostinfo;
extern wiced_timer_t hello_sensor_conn_idle_timer;

uint8_t *hello_sensor_alloc_buffer(uint16_t len);
void hello_sensor_free_buffer(uint8_t *p_data);

/******************************************************************************
 * Function Name: hello_sensor_gatt_init
 *******************************************************************************
 * Summary:
 *  helper function to init GATT
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 ******************************************************************************/
void hello_sensor_gatt_init()
{
    wiced_bt_gatt_status_t gatt_status;
    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(hello_sensor_gatts_callback);
    TRACE_LOG( "wiced_bt_gatt_register: %d\n", gatt_status );

    /*  Tell stack to use our GATT database */
    gatt_status =  wiced_bt_gatt_db_init( hello_sensor_gatt_database, hello_sensor_gatt_database_size, NULL );

    TRACE_LOG("wiced_bt_gatt_db_init %d\n", gatt_status);
}

/*
 */
/******************************************************************************
 * Function Name: hello_sensor_gatts_callback
 *******************************************************************************
 * Summary:
 *  Callback for various GATT events.  As this application performs only as a GATT server, some of the events are ommitted.
 *
 * Parameters:
 *  wiced_bt_gatt_evt_t event
 *  wiced_bt_gatt_event_data_t *p_data: GATT event notifications
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
 ******************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    TRACE_LOG("event:%s\n", get_bt_le_gatt_event_name(event));
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = hello_sensor_gatts_conn_status_cb( &p_data->connection_status );
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = hello_sensor_gatts_req_cb( &p_data->attribute_request );
        break;

   case GATT_GET_RESPONSE_BUFFER_EVT:
        p_data->buffer_request.buffer.p_app_rsp_buffer = hello_sensor_alloc_buffer (p_data->buffer_request.len_requested);
        p_data->buffer_request.buffer.p_app_ctxt       = hello_sensor_free_buffer;
        result = WICED_BT_GATT_SUCCESS;
        break;

    case GATT_APP_BUFFER_TRANSMITTED_EVT:
    {
        pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_data->buffer_xmitted.p_app_ctxt;

        /* If the buffer is dynamic, the context will point to a function to free it. */
        if (pfn_free)
            pfn_free(p_data->buffer_xmitted.p_app_data);

        result = WICED_BT_GATT_SUCCESS;
    }
    break;

    case GATT_OPERATION_CPLT_EVT:
        result = hello_sensor_gatt_op_comp_cb( &p_data->operation_complete );
        break;
    default:
        break;
    }
    return result;
}

/******************************************************************************
 * Function Name: hello_sensor_gatts_req_read_handler
 *******************************************************************************
 * Summary:
 *  Process Read request or read blob from peer device
 *
 * Parameters:
 *  uint16_t conn_id
 *  wiced_bt_gatt_opcode_t opcode
 *  wiced_bt_gatt_read_t *p_read_req: Attribute read request
 *  uint16_t len_requested
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
 ******************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_read_handler (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_t *p_read_req, uint16_t len_requested)
{
    attribute_t *puAttribute;
    uint16_t    attr_len_to_copy;
    uint8_t     *from;
    TRACE_LOG("\n");

    if ( ( puAttribute = hello_sensor_get_attribute (p_read_req->handle) ) == NULL)
    {
        TRACE_LOG("attr not found handle: 0x%04x\n", p_read_req->handle);
        wiced_bt_gatt_server_send_error_rsp (conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Dummy battery value read increment */
    if (p_read_req->handle == HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL)
    {
        if (hello_sensor_state.battery_level++ > 5)
            hello_sensor_state.battery_level = 0;
    }

    attr_len_to_copy = puAttribute->attr_len;

    TRACE_LOG("conn_id: %d handle:0x%04x offset:%d len:%d\n", conn_id, p_read_req->handle, p_read_req->offset, attr_len_to_copy);

    if (p_read_req->offset >= puAttribute->attr_len)
    {
        TRACE_LOG ("offset:%d larger than attribute length:%d\n", p_read_req->offset, puAttribute->attr_len);

        wiced_bt_gatt_server_send_error_rsp (conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_OFFSET);
        return (WICED_BT_GATT_INVALID_HANDLE);
    }

    int to_send = MIN(len_requested, attr_len_to_copy - p_read_req->offset);

    from = ((uint8_t*)puAttribute->p_attr) + p_read_req->offset;

    wiced_bt_gatt_server_send_read_handle_rsp (conn_id, opcode, to_send, from, NULL);

    return WICED_BT_GATT_SUCCESS;
}

/******************************************************************************
 * Function Name: hello_sensor_gatts_req_read_by_type_handler
 *******************************************************************************
 * Summary:
 *  Process write read-by-type request from peer device
 *
 * Parameters:
 *  uint16_t conn_id
 *  wiced_bt_gatt_opcode_t opcode
 *  wiced_bt_gatt_read_by_type_t *p_read_req
 *  uint16_t len_requested
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
 ******************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_read_by_type_handler (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
    wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested)
{
    attribute_t *puAttribute;
    uint16_t    attr_handle = p_read_req->s_handle;
    uint8_t     *p_rsp = hello_sensor_alloc_buffer (len_requested);
    uint8_t    pair_len = 0;
    int used = 0;
    TRACE_LOG("\n");

    if (p_rsp == NULL)
    {
        TRACE_LOG ("no memory len_requested: %d!!\n", len_requested);

        wiced_bt_gatt_server_send_error_rsp (conn_id, opcode, attr_handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        attr_handle = wiced_bt_gatt_find_handle_by_type (attr_handle, p_read_req->e_handle, &p_read_req->uuid);

        if (attr_handle == 0)
            break;

        if ((puAttribute = hello_sensor_get_attribute (attr_handle)) == NULL)
        {
            TRACE_LOG ("found type but no attribute ??\n");
            wiced_bt_gatt_server_send_error_rsp (conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_ERR_UNLIKELY);
            hello_sensor_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used, len_requested - used, &pair_len,
                attr_handle, puAttribute->attr_len, puAttribute->p_attr);
            if (filled == 0)
            {
                break;
            }
            used += filled;
        }

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (used == 0)
    {
        TRACE_LOG ("attr not found  start_handle: 0x%04x  end_handle: 0x%04x  Type: 0x%04x\n",
                p_read_req->s_handle, p_read_req->e_handle, p_read_req->uuid.uu.uuid16);

        wiced_bt_gatt_server_send_error_rsp (conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_INVALID_HANDLE);
        hello_sensor_free_buffer (p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp (conn_id, opcode, pair_len, used, p_rsp, hello_sensor_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/*
 */
/******************************************************************************
 * Function Name: hello_sensor_gatts_req_read_multi_handler 
 *******************************************************************************
 * Summary:
 *  Process write read multi request from peer device
 *
 * Parameters:
 *  uint16_t conn_id
 *  wiced_bt_gatt_opcode_t opcode
 *  wiced_bt_gatt_read_multiple_req_t *p_read_req:
 *      Response structure, containing the requested handle list for GATT_RSP_READ_MULTI or GATT_RSP_READ_MULTI_VAR_LENGTH
 *      commands

 *  uint16_t len_requested
 *
 * Return:
 *  wiced_bt_gatt_status_t 
 *
 ******************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_read_multi_handler (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
    wiced_bt_gatt_read_multiple_req_t *p_read_req, uint16_t len_requested)
{
    attribute_t *puAttribute;
    uint8_t     *p_rsp = hello_sensor_alloc_buffer(len_requested);
    int         used = 0;
    int         xx;
    uint16_t    handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, 0);
    TRACE_LOG("\n");

    if (p_rsp == NULL)
    {
        TRACE_LOG ("no memory len_requested: %d!!\n", len_requested);

        wiced_bt_gatt_server_send_error_rsp (conn_id, opcode, handle, WICED_BT_GATT_INSUF_RESOURCE);

        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    for (xx = 0; xx < p_read_req->num_handles; xx++)
    {
        handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, xx);
        if ((puAttribute = hello_sensor_get_attribute (handle)) == NULL)
        {
            TRACE_LOG ("no handle 0x%04xn", handle);
            wiced_bt_gatt_server_send_error_rsp (conn_id, opcode, *p_read_req->p_handle_stream, WICED_BT_GATT_ERR_UNLIKELY);
            hello_sensor_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_multi_rsp_in_stream(opcode, p_rsp + used, len_requested - used,
                puAttribute->handle, puAttribute->attr_len, puAttribute->p_attr);
            if (!filled) {
                break;
            }
            used += filled;
        }
    }

    if (used == 0)
    {
        TRACE_LOG ("no attr found\n");

        wiced_bt_gatt_server_send_error_rsp (conn_id, opcode, *p_read_req->p_handle_stream, WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_multiple_rsp (conn_id, opcode, used, p_rsp, hello_sensor_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/******************************************************************************
 * Function Name: hello_sensor_gatts_req_write_handler
 *******************************************************************************
 * Summary:
 *  Process write request or write command from peer device
 *
 * Parameters:
 *  uint16_t conn_id
 *  wiced_bt_gatt_opcode_t opcode
 *  wiced_bt_gatt_write_req_t* p_data: Attribute write request
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
 ******************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_write_req_t* p_data )
{
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
    uint8_t                *p_attr   = p_data->p_val;
    uint8_t                nv_update = WICED_FALSE;

    TRACE_LOG("conn_id:%d hdl:0x%x opcode:%d offset:%d len:%d\n ", conn_id, p_data->handle, opcode, p_data->offset, p_data->val_len );

    switch ( p_data->handle )
    {
    /* By writing into Characteristic Client Configuration descriptor
     * peer can enable or disable notification or indication */
    case HANDLE_HSENS_SERVICE_CHAR_CFG_DESC:
        if ( p_data->val_len != 2 )
        {
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        hello_sensor_hostinfo.characteristic_client_configuration = p_attr[0] | ( p_attr[1] << 8 );

        TRACE_LOG ("CCCD changed to: 0x%04x\n ", hello_sensor_hostinfo.characteristic_client_configuration);

        nv_update = WICED_TRUE;
        break;

    case HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL:
        if ( p_data->val_len != 1 )
        {
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        hello_sensor_hostinfo.number = p_attr[0];
        if ( hello_sensor_hostinfo.number != 0 )
        {
            TRACE_LOG( "number: %d\n", hello_sensor_hostinfo.number );
            nv_update = WICED_TRUE;
        }
        break;

    default:
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
    }

    if ( nv_update )
    {
        wiced_result_t status;
        wiced_hal_write_nvram(HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo),
            (uint8_t*)&hello_sensor_hostinfo, &status);
        TRACE_LOG("NVRAM write: %d", status);
    }

    return result;
}

/******************************************************************************
 * Function Name: hello_sensor_gatts_req_write_exec_handler
 *******************************************************************************
 * Summary:
 *  Write Execute Procedure
 *
 * Parameters:
 *  uint16_t conn_id
 *  wiced_bt_gatt_exec_flag_t exec_falg:
 *      GATT execute flag
 *
 * Return:
 *  wiced_bt_gatt_status_t 
 *
 ******************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_write_exec_handler( uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_falg)
{
    TRACE_LOG("write exec: flag:%d\n", exec_falg);
    return WICED_BT_GATT_SUCCESS;
}

/******************************************************************************
 * Function Name: hello_sensor_gatts_req_mtu_handler
 *******************************************************************************
 * Summary:
 *  Process MTU request from the peer
 *
 * Parameters:
 *  uint16_t conn_id
 *  uint16_t mtu
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
 ******************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_mtu_handler( uint16_t conn_id, uint16_t mtu)
{
    TRACE_LOG("req_mtu: %d\n", mtu);
    wiced_bt_gatt_server_send_mtu_rsp(conn_id, mtu, hello_sensor_cfg_ble.ble_max_rx_pdu_size);
    return WICED_BT_GATT_SUCCESS;
}

/******************************************************************************
 * Function Name: hello_sensor_gatts_req_conf_handler
 *******************************************************************************
 * Summary:
 *  Process indication confirm. If client wanted us to use indication instead of
 *  notifications we have to wait for confirmation after every message sent.
 *  For example if user pushed button twice very fast
 *  we will send first message, then
 *  wait for confirmation, then
 *  send second message, then
 *  wait for confirmation and
 *  if configured start idle timer only after that.
 *
 * Parameters:
 *  uint16_t conn_id
 *  uint16_t handle
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
 ******************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_conf_handler( uint16_t conn_id, uint16_t handle )
{
    TRACE_LOG( "hello_sensor_indication_cfm, conn %d hdl %d\n", conn_id, handle );

    if ( !hello_sensor_state.flag_indication_sent )
    {
        TRACE_LOG("Hello: Wrong Confirmation!");
        return WICED_BT_GATT_SUCCESS;
    }

    hello_sensor_state.flag_indication_sent = 0;

    /* We might need to send more indications */
    if ( hello_sensor_state.num_to_write )
    {
        hello_sensor_state.num_to_write--;
        hello_sensor_send_message();
    }
    /* if we sent all messages, start connection idle timer to disconnect */
    if ( !hello_sensor_state.flag_stay_connected && !hello_sensor_state.flag_indication_sent )
    {
        if (wiced_is_timer_in_use(&hello_sensor_conn_idle_timer) )
        {
            wiced_stop_timer(&hello_sensor_conn_idle_timer);
            wiced_start_timer(&hello_sensor_conn_idle_timer, HELLO_SENSOR_CONN_IDLE_TIMEOUT_IN_SECONDS);
        }
    }

    return WICED_BT_GATT_SUCCESS;
}

/******************************************************************************
 * Function Name: hello_sensor_gatts_req_cb
 *******************************************************************************
 * Summary:
 *  Process GATT request from the peer
 *
 * Parameters:
 *  wiced_bt_gatt_attribute_request_t *p_data:
 *      GATT attribute request (used by GATT_ATTRIBUTE_REQUEST_EVT notification)
 *
 * Return:
 *  wiced_bt_gatt_status_t 
 *
 ******************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    TRACE_LOG( "hello_sensor_gatts_req_cb. conn %d, opcode 0x%x\n", p_data->conn_id, p_data->opcode);

    switch (p_data->opcode)
    {
    case GATT_REQ_READ:
    case GATT_REQ_READ_BLOB:
        result = hello_sensor_gatts_req_read_handler( p_data->conn_id, p_data->opcode, &p_data->data.read_req, p_data->len_requested );
        break;

    case GATT_REQ_READ_BY_TYPE:
        result = hello_sensor_gatts_req_read_by_type_handler (p_data->conn_id, p_data->opcode, &p_data->data.read_by_type, p_data->len_requested);
        break;

    case GATT_REQ_READ_MULTI:
    case GATT_REQ_READ_MULTI_VAR_LENGTH:
        result = hello_sensor_gatts_req_read_multi_handler (p_data->conn_id, p_data->opcode, &p_data->data.read_multiple_req, p_data->len_requested);
        break;

    case GATT_REQ_WRITE:
    case GATT_CMD_WRITE:
    case GATT_CMD_SIGNED_WRITE:
        result = hello_sensor_gatts_req_write_handler( p_data->conn_id, p_data->opcode, &(p_data->data.write_req) );
        if ( (p_data->opcode == GATT_REQ_WRITE) &&  (result == WICED_BT_GATT_SUCCESS))
        {
            wiced_bt_gatt_write_req_t   *p_write_request = &p_data->data.write_req;
            wiced_bt_gatt_server_send_write_rsp(p_data->conn_id, p_data->opcode, p_write_request->handle);
        } 
        else 
        {
            wiced_bt_gatt_write_req_t   *p_write_request = &p_data->data.write_req;
            wiced_bt_gatt_server_send_error_rsp(p_data->conn_id, p_data->opcode, p_write_request->handle, result);
        }
        break;
    case GATT_REQ_EXECUTE_WRITE:
        hello_sensor_gatts_req_write_exec_handler(p_data->conn_id, p_data->data.exec_write_req.exec_write);
        wiced_bt_gatt_server_send_execute_write_rsp(p_data->conn_id, p_data->opcode);
        break;

    case GATT_REQ_MTU:
        result = hello_sensor_gatts_req_mtu_handler( p_data->conn_id, p_data->data.remote_mtu);
        break;

    case GATT_HANDLE_VALUE_CONF:
        result = hello_sensor_gatts_req_conf_handler( p_data->conn_id, p_data->data.confirm.handle );
        break;

   default:
        break;
    }

    return result;
}

/******************************************************************************
 * Function Name: hello_sensor_gatt_op_comp_cb
 *******************************************************************************
 * Summary:
 *  GATT operation started by the sensor has been completed
 *
 * Parameters:
 *  wiced_bt_gatt_operation_complete_t *p_data:
 *      Response to read/write/disc/config operations, (used by GATT_OPERATION_CPLT_EVT notification)
 *
 * Return:
 *  wiced_bt_gatt_status_t 
 *
 ******************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatt_op_comp_cb( wiced_bt_gatt_operation_complete_t *p_data )
{
    wiced_result_t              status;

    TRACE_LOG("==>hello_sensor conn %d op %d st %d\n", p_data->conn_id, p_data->op, p_data->status );

    switch ( p_data->op )
    {
        case GATTC_OPTYPE_NOTIFICATION:
            p_data->response_data.att_value.p_data[p_data->response_data.att_value.len] = 0;
            TRACE_LOG( "Get Notification from central conn_id %d, data[%d] = %s:\n", p_data->conn_id, p_data->response_data.att_value.len, p_data->response_data.att_value.p_data);
            break;

        default:
            TRACE_LOG( "default:\n");
            break;
    }

    UNUSED_VARIABLE(status);
    return WICED_BT_GATT_SUCCESS;
}

/******************************************************************************
 * Function Name: hello_sensor_alloc_buffer 
 *******************************************************************************
 * Summary:
 *  Allocate buffer of requested length
 *
 * Parameters:
 *  uint16_t len
 *
 * Return:
 *  uint8_t *
 *
 ******************************************************************************/
uint8_t *hello_sensor_alloc_buffer(uint16_t len)
{
    uint8_t *p = (uint8_t *)wiced_bt_get_buffer(len);
    TRACE_LOG("len %d alloc 0x%x", len, p);

    return p;
}

/******************************************************************************
 * Function Name: hello_sensor_free_buffer
 *******************************************************************************
 * Summary:
 *  Free allocated buffer
 *
 * Parameters:
 *  uint8_t *p_data
 *
 * Return:
 *  None
 *
 ******************************************************************************/
void hello_sensor_free_buffer(uint8_t *p_data)
{
    wiced_bt_free_buffer(p_data);

    TRACE_LOG("0x%x", p_data);
}
