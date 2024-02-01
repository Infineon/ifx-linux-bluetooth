/******************************************************************************
 * (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 *****************************************************************************/

/******************************************************************************
 * File Name: findme.c
 *
 * Description: This is the source file for Linux Findme CE.
 *
 * Related Document: See README.md
 *
 *****************************************************************************/

/*******************************************************************************
*      INCLUDES
*******************************************************************************/
#include "wiced_bt_stack.h"
#include <string.h>
#include <stdlib.h>
#include "wiced_memory.h"
#include "stdio.h"
#include "GeneratedSource/cycfg_gatt_db.h"
#include "GeneratedSource/cycfg_bt_settings.h"
#include "GeneratedSource/cycfg_gap.h"
#include "wiced_bt_dev.h"
#include "app_bt_utils.h"
#include "findme.h"
#include "wiced_memory.h"
#include "log.h"

/*******************************************************************************
*       MACROS
*******************************************************************************/

#define BT_STACK_HEAP_SIZE              (0xF000)

/*******************************************************************************
*       STRUCTURES AND ENUMERATIONS
*******************************************************************************/

/* This enumeration combines the advertising, connection states from two
 * different callbacks to maintain the status in a single state variable */
typedef enum
{
    APP_BT_ADV_OFF_CONN_OFF,
    APP_BT_ADV_ON_CONN_OFF,
    APP_BT_ADV_OFF_CONN_ON
} tAppBtAdvConnMode;

/*******************************************************************************
*       VARIABLE DEFINITIONS
*******************************************************************************/

wiced_bt_heap_t *p_default_heap   = NULL;

static uint16_t bt_connection_id = 0;
static tAppBtAdvConnMode app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;

/*******************************************************************************
*       FUNCTION DECLARATIONS
*******************************************************************************/
static void     le_app_init                 (void);
static void*    le_app_alloc_buffer         (int len);
static void     le_app_free_buffer          (uint8_t *p_event_data);

typedef void    (*pfn_free_buffer_t)        (uint8_t *);

static gatt_db_lookup_table_t *
le_app_find_by_handle         (uint16_t handle);

/* GATT Event Callback Functions */
static wiced_bt_gatt_status_t
le_app_write_handler          (uint16_t conn_id,
                              wiced_bt_gatt_opcode_t opcode,
                              wiced_bt_gatt_write_req_t *p_write_req,
                              uint16_t len_req);
static wiced_bt_gatt_status_t
le_app_read_handler           (uint16_t conn_id,
                              wiced_bt_gatt_opcode_t opcode,
                              wiced_bt_gatt_read_t *p_read_req,
                              uint16_t len_req);
static wiced_bt_gatt_status_t
le_app_connect_handler        (wiced_bt_gatt_connection_status_t *p_conn_status);

static wiced_bt_gatt_status_t
le_app_server_handler         (wiced_bt_gatt_attribute_request_t *p_attr_req);

static wiced_bt_gatt_status_t
le_app_gatt_event_callback    (wiced_bt_gatt_evt_t  event,
                              wiced_bt_gatt_event_data_t *p_event_data);

/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t
le_app_bt_management_callback (wiced_bt_management_evt_t event,
                              wiced_bt_management_evt_data_t *p_event_data);

static wiced_bt_gatt_status_t
le_app_bt_gatt_req_read_by_type_handler (uint16_t conn_id,
                                        wiced_bt_gatt_opcode_t opcode,
                                        wiced_bt_gatt_read_by_type_t *p_read_req,
                                        uint16_t len_requested);

/*******************************************************************************
*       FUNCTION DEFINITION
*******************************************************************************/

/*******************************************************************************
* Function Name: application_start
********************************************************************************
* Summary:
*  Set device configuration and start BT stack initialization. The actual
*  application initialization will happen when stack reports that BT device
*  is ready.
*
* Parameters: NONE
*
* Return: NONE
*
*******************************************************************************/
void application_start(void)
{
    wiced_result_t wiced_result;

    TRACE_LOG("************* Find Me Profile Application Start ************************\n");

    /* Register call back and configuration with stack */
    wiced_result = wiced_bt_stack_init(le_app_bt_management_callback, &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if( WICED_BT_SUCCESS == wiced_result)
    {
        TRACE_LOG("Bluetooth Stack Initialization Successful \n");
        /* Create default heap */
        p_default_heap = wiced_bt_create_heap("default_heap", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
        if (p_default_heap == NULL)
        {
            TRACE_ERR("create default heap error: size %d\n", BT_STACK_HEAP_SIZE);
            exit(EXIT_FAILURE);
        }
    }
    else
    {
       TRACE_ERR("Bluetooth Stack Initialization failed!! \n");
       exit(EXIT_FAILURE);
    }
}

/*******************************************************************************
* Function Name: le_app_bt_management_callback
********************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management
*   events from the LE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : LE event code of one byte
*                                                 length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to LE management
*                                                 event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*******************************************************************************/
wiced_result_t le_app_bt_management_callback( wiced_bt_management_evt_t event,
                                              wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Bluetooth Controller and Host Stack Enabled */
            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
                wiced_bt_dev_read_local_addr(bda);
                TRACE_LOG("Local Bluetooth Address: ");
                print_bd_address(bda);

                /* Perform application-specific initialization */
                le_app_init();
            }
            else
            {
                TRACE_ERR("Bluetooth Enabling Failed \n" );
            }

            break;

        case BTM_DISABLED_EVT:
            TRACE_LOG("Bluetooth Disabled \n" );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            /* Advertisement State Changed */
            p_adv_mode = &p_event_data->ble_advert_state_changed;
            TRACE_LOG("Advertisement State Change: %s\n", get_bt_advert_mode_name(*p_adv_mode));

            if (BTM_BLE_ADVERT_OFF == *p_adv_mode)
            {
                /* Advertisement Stopped */
                TRACE_LOG("Advertisement stopped\n");

                /* Check connection status after advertisement stops */
                if(0 == bt_connection_id)
                {
                    app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;
                }
                else
                {
                    app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
                }
            }
            else
            {
                /* Advertisement Started */
                TRACE_LOG("Advertisement started\n");
                app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;
            }

            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            TRACE_LOG( "Connection parameter update status:%d, Connection Interval: %d, \
                       Connection Latency: %d, Connection Timeout: %d\n",
                       p_event_data->ble_connection_param_update.status,
                       p_event_data->ble_connection_param_update.conn_interval,
                       p_event_data->ble_connection_param_update.conn_latency,
                       p_event_data->ble_connection_param_update.supervision_timeout);
            break;

        default:
            break;
    }

    return WICED_BT_SUCCESS;
}

/*******************************************************************************
* Function Name: le_app_init
********************************************************************************
* Summary:
*   This function handles application level initialization tasks and is called
*   from the BT management callback once the LE stack enabled event
*   (BTM_ENABLED_EVT) is triggered This function is executed in the
*   BTM_ENABLED_EVT management callback.
*
* Parameters:
*   None
*
* Return:
*  None
*
*******************************************************************************/
static void le_app_init(void)
{
    wiced_result_t wiced_result = WICED_BT_SUCCESS;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    TRACE_LOG("\n***********************************************\n");
    TRACE_LOG("**Discover device with \"Find Me Target\" name*\n");
    TRACE_LOG("***********************************************\n\n");

    wiced_bt_set_pairable_mode(FALSE, FALSE);

    /* Set Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, cy_bt_adv_packet_data);

    /* Register with BT stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(le_app_gatt_event_callback);
    TRACE_LOG("GATT event Handler registration status: %s \n",get_bt_gatt_status_name(gatt_status));

    /* Initialize GATT Database */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    TRACE_LOG("GATT database initialization status: %s \n",get_bt_gatt_status_name(gatt_status));

    /* Start Undirected LE Advertisements on device startup.
     * The corresponding parameters are contained in 'app_bt_cfg.c' */
    wiced_result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

    /* Failed to start advertisement. Stop program execution */
    if (WICED_BT_SUCCESS != wiced_result)
    {
        TRACE_ERR("failed to start advertisement! \n");
        exit(0);
    }
}

/*******************************************************************************
* Function Name: le_app_gatt_event_callback
********************************************************************************
* Summary:
*   This function handles GATT events from the BT stack.
*
* Parameters:
*   wiced_bt_gatt_evt_t event                : LE GATT event code of one
*                                              byte length
*   wiced_bt_gatt_event_data_t *p_event_data : Pointer to LE GATT event
*                                              structures
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
*******************************************************************************/
static wiced_bt_gatt_status_t le_app_gatt_event_callback( wiced_bt_gatt_evt_t event,
                                                          wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_event_data->attribute_request;

    /* Call the appropriate callback function based on the GATT event type,
     * and pass the relevant event
     * parameters to the callback function */
    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            gatt_status = le_app_connect_handler( &p_event_data->connection_status );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            gatt_status = le_app_server_handler( p_attr_req );
            break;

        /* GATT buffer request, typically sized to max of bearer mtu - 1 */
        case GATT_GET_RESPONSE_BUFFER_EVT:
            p_event_data->buffer_request.buffer.p_app_rsp_buffer =
            le_app_alloc_buffer(p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt = (void *)le_app_free_buffer;
            gatt_status = WICED_BT_GATT_SUCCESS;
            break;

        /* GATT buffer transmitted event, check ref wiced_bt_gatt_buffer_transmitted_t */
        case GATT_APP_BUFFER_TRANSMITTED_EVT:
            {
                pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

                /* If the buffer is dynamic, the context will point
                * to a function to free it. */
                if (pfn_free)
                {
                    pfn_free(p_event_data->buffer_xmitted.p_app_data);
                }

                gatt_status = WICED_BT_GATT_SUCCESS;
            }
            break;

        default:
            gatt_status = WICED_BT_GATT_SUCCESS;
            break;
    }

    return gatt_status;
}

/*******************************************************************************
* Function Name: le_app_set_value
********************************************************************************
* Summary:
*   This function handles writing to the attribute handle in the GATT database
*   using the data passed from the BT stack. The value to write is stored in a
*   buffer whose starting address is passed as one of the function parameters
*
* Parameters:
* @param attr_handle  GATT attribute handle
* @param p_val        Pointer to LE GATT write request value
* @param len          length of GATT write request
*
*
* Return:
*   wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*   in wiced_bt_gatt.h
*
*******************************************************************************/
static wiced_bt_gatt_status_t le_app_set_value( uint16_t attr_handle,
                                                uint8_t *p_val,
                                                uint16_t len)
{
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;
    /* Check for a matching handle entry */
    for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in external lookup table */
            isHandleInTable = WICED_TRUE;

            /* Check if the buffer has space to store the data */
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);

            if (validLen)
            {
                /* Value fits within the supplied buffer; copy over the value */
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                gatt_status = WICED_BT_GATT_SUCCESS;

                /* Add code for any action required when this attribute is 
                 * written. In this case, we update the IAS led based on the 
                 * IAS alert level characteristic value */

                switch (attr_handle)
                {
                    case HDLC_IAS_ALERT_LEVEL_VALUE:
                        TRACE_LOG("Alert Level = %d\n", app_ias_alert_level[0]);
                        break;

                    default:
                        TRACE_LOG("No such attr handle\n");
                        break;
                }
            }
            else
            {
                /* Value to write does not meet size constraints */
                gatt_status = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        /* The write operation was not performed for the indicated handle */
        TRACE_ERR("Write Request to Invalid Handle: 0x%x\n", attr_handle);
        gatt_status = WICED_BT_GATT_WRITE_NOT_PERMIT;
    }

    return gatt_status;
}

/*******************************************************************************
* Function Name: le_app_write_handler
********************************************************************************
* Summary:
*   This function handles Write Requests received from the client device
*
* Parameters:
*  @param conn_id       Connection ID
*  @param opcode        LE GATT request type opcode
*  @param p_write_req   Pointer to LE GATT write request
*  @param len_req       length of data to be written
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
*******************************************************************************/
static wiced_bt_gatt_status_t le_app_write_handler( uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_write_req_t *p_write_req,
                                                    uint16_t len_req)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Write Request */
    gatt_status = le_app_set_value( p_write_req->handle,
                                    p_write_req->p_val,
                                    p_write_req->val_len);

    if( WICED_BT_GATT_SUCCESS != gatt_status )
    {
        TRACE_LOG("WARNING: GATT set attr status 0x%x\n", gatt_status);
    }

    return (gatt_status);
}

/*******************************************************************************
* Function Name: le_app_read_handler
********************************************************************************
* Summary:
*   This function handles Read Requests received from the client device
*
* Parameters:
* @param conn_id       Connection ID
* @param opcode        LE GATT request type opcode
* @param p_read_req    Pointer to read request containing the handle to read
* @param len_req       length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
*******************************************************************************/
static wiced_bt_gatt_status_t le_app_read_handler(  uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_read_t *p_read_req,
                                                    uint16_t len_req)
{

    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    /* Pointer to an attribute in the GATT db lookup table */
    gatt_db_lookup_table_t *puAttribute;
    int attr_len_to_copy;
    uint8_t *from;
    int to_send;

    if(NULL == p_read_req)
    {
        gatt_status = WICED_BT_GATT_ERROR;
    }
    else
    {
        puAttribute = le_app_find_by_handle(p_read_req->handle);
        if ( NULL == puAttribute )
        {
            wiced_bt_gatt_server_send_error_rsp( conn_id, opcode, p_read_req->handle,
                                                 WICED_BT_GATT_INVALID_HANDLE);
            gatt_status = WICED_BT_GATT_INVALID_HANDLE;
        }
        else
        {
            attr_len_to_copy = puAttribute->cur_len;
            if (p_read_req->offset >= puAttribute->cur_len)
            {
                wiced_bt_gatt_server_send_error_rsp( conn_id, opcode, p_read_req->handle,
                                                     WICED_BT_GATT_INVALID_OFFSET);
                gatt_status = WICED_BT_GATT_INVALID_OFFSET;
            }
            else
            {
                to_send = MIN(len_req, attr_len_to_copy - p_read_req->offset);
                from = ((uint8_t *)puAttribute->p_data) + p_read_req->offset;
                /* No need for context, as buff not allocated */
                gatt_status =  wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL);
            }
        }
    }
    return gatt_status;
}

/*******************************************************************************
* Function Name: le_app_connect_handler
********************************************************************************
* Summary:
*   This callback function handles connection status changes.
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_conn_status  : Pointer to data that
*   has connection details
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
*******************************************************************************/
static wiced_bt_gatt_status_t le_app_connect_handler( wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS ;

    if ( NULL != p_conn_status )
    {
        if ( p_conn_status->connected )
        {
            /* Device has connected */
            TRACE_LOG("Connected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            TRACE_LOG("Connection ID '%d' \n", p_conn_status->conn_id );

            /* Store the connection ID */
            bt_connection_id = p_conn_status->conn_id;

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
        }
        else
        {
            /* Device has disconnected */
            TRACE_LOG("Disconnected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            TRACE_LOG( "Connection ID '%d', Reason '%s'\n",
                       p_conn_status->conn_id,
                       get_bt_gatt_disconn_reason_name(p_conn_status->reason) );

            /* Set the connection id to zero to indicate disconnected state */
            bt_connection_id = 0;

            /* Restart the advertisements */
            wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;
        }

    }
    else
    {
        gatt_status = WICED_BT_GATT_ERROR;
    }
    return gatt_status;
}

/*******************************************************************************
* Function Name: le_app_server_handler
********************************************************************************
* Summary:
*   This function handles GATT server events from the BT stack.
*
* Parameters:
*  p_attr_req     Pointer to LE GATT connection status
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
*******************************************************************************/
static wiced_bt_gatt_status_t le_app_server_handler ( wiced_bt_gatt_attribute_request_t *p_attr_req)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;
    if(NULL != p_attr_req)
    {
        switch ( p_attr_req->opcode )
        {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
             /* Attribute read request */
             gatt_status = le_app_read_handler( p_attr_req->conn_id,p_attr_req->opcode,
                                                &p_attr_req->data.read_req,
                                                p_attr_req->len_requested);
             break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
             /* Attribute write request */
             gatt_status = le_app_write_handler( p_attr_req->conn_id, p_attr_req->opcode,
                                                 &p_attr_req->data.write_req,
                                                 p_attr_req->len_requested );
             break;

        case GATT_REQ_MTU:
             gatt_status = wiced_bt_gatt_server_send_mtu_rsp( p_attr_req->conn_id,
                                                              p_attr_req->data.remote_mtu,
                                                              CY_BT_MTU_SIZE);
             break;

        case GATT_REQ_READ_BY_TYPE:
             gatt_status = le_app_bt_gatt_req_read_by_type_handler( p_attr_req->conn_id,
                                                                    p_attr_req->opcode,
                                                                    &p_attr_req->data.read_by_type,
                                                                    p_attr_req->len_requested);
             break;

        default:
             TRACE_ERR("ERROR: Unhandled GATT Connection Request case: %d\n", p_attr_req->opcode);
             break;
        }
    }
    return gatt_status;
}

/*******************************************************************************
 * Function Name: le_app_free_buffer
 *******************************************************************************
 * Summary:
 *  This function frees up the memory buffer
 *
 *
 * Parameters:
 *  uint8_t *p_data: Pointer to the buffer to be free
 *
 ******************************************************************************/
static void le_app_free_buffer(uint8_t *p_buf)
{
    wiced_bt_free_buffer(p_buf);
}

/*******************************************************************************
 * Function Name: le_app_alloc_buffer
 *******************************************************************************
 * Summary:
 *  This function allocates a memory buffer.
 *
 *
 * Parameters:
 *  int len: Length to allocate
 *
 ******************************************************************************/
static void* le_app_alloc_buffer(int len)
{
    uint8_t *p = (uint8_t *)wiced_bt_get_buffer(len);
    return p;
}

/*******************************************************************************
 * Function Name : le_app_find_by_handle
 * *****************************************************************************
 * Summary : @brief  Find attribute description by handle
 *
 * @param handle    handle to look up
 *
 * @return gatt_db_lookup_table_t   pointer containing handle data
 ******************************************************************************/
static gatt_db_lookup_table_t *le_app_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (handle == app_gatt_db_ext_attr_tbl[i].handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}

/*******************************************************************************
 * Function Name: le_app_bt_gatt_req_read_by_type_handler
 * *****************************************************************************
 *
 * Summary:
 *  Process read-by-type request from peer device
 *
 * @param conn_id       Connection ID
 * @param opcode        LE GATT request type opcode
 * @param p_read_req    Pointer to read request containing the handle to read
 * @param len_requested length of data requested
 *
 * @return wiced_bt_gatt_status_t  LE GATT status
 ******************************************************************************/
static wiced_bt_gatt_status_t le_app_bt_gatt_req_read_by_type_handler( uint16_t conn_id,
                                                                       wiced_bt_gatt_opcode_t opcode,
                                                                       wiced_bt_gatt_read_by_type_t *p_read_req,
                                                                       uint16_t len_requested)
{
    /* Pointer to an attribute in the GATT db lookup table */
    gatt_db_lookup_table_t *puAttribute;
    uint16_t last_handle = 0;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = le_app_alloc_buffer(len_requested);
    uint8_t pair_len = 0;
    int used_len = 0;

    if (NULL == p_rsp)
    {
        TRACE_ERR("No memory, len_requested: %d!!\r\n",len_requested);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, 
     * between the start and end handles 
     */
    while (WICED_TRUE)
    {
        last_handle = attr_handle;
        attr_handle = wiced_bt_gatt_find_handle_by_type( attr_handle, p_read_req->e_handle,
                                                         &p_read_req->uuid);
        if (0 == attr_handle )
            break;

        if ( NULL == (puAttribute = le_app_find_by_handle(attr_handle)))
        {
            TRACE_ERR("found type but no attribute for %d \r\n",last_handle);
            wiced_bt_gatt_server_send_error_rsp( conn_id, opcode, p_read_req->s_handle,
                                                 WICED_BT_GATT_ERR_UNLIKELY);
            le_app_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream( p_rsp + used_len,
                                                                   len_requested - used_len,
                                                                   &pair_len,
                                                                   attr_handle,
                                                                   puAttribute->cur_len,
                                                                   puAttribute->p_data);
        if (0 == filled)
        {
            break;
        }
        used_len += filled;

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (0 == used_len)
    {
        TRACE_ERR( "attr not found  start_handle: 0x%04x  end_handle: 0x%04x  Type: 0x%04x\r\n",
                   p_read_req->s_handle,
                   p_read_req->e_handle,
                   p_read_req->uuid.uu.uuid16);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_INVALID_HANDLE);
        le_app_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */

    return wiced_bt_gatt_server_send_read_by_type_rsp( conn_id,
                                                       opcode,
                                                       pair_len,
                                                       used_len,
                                                       p_rsp,
                                                       (void *)le_app_free_buffer);
}
/* END OF FILE [] */
