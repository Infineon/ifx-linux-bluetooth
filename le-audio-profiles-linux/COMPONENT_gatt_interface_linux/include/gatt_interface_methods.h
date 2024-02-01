/*
 * $ Copyright Cypress Semiconductor $
 */


 /**************************************************************************//**
  * \file <gatt_interface.h>
  *
  * Definitions for interface between BT stack GATT and service/application. The GATT interface is responsible for
  * routing the incoming GATT commands to the appropriate service/services or application.
  * GATT Interface library helps applications to
  *  1. Discover remote services, included services, characteristics, descriptors and map them to the
  *     registered service implementations
  *  2. Setup local services, included services, characteristics, descriptors and map them to the
  *     registered service implementations
  *  3. Perform basic operations viz, notifying remote clients, setting up notifications, reading characteristics
  *  4. Organise non volatile data for storing and reading back from NVRAM
  *
  */

#ifndef __GATT_INTERFACE_METHODS_H__
#define __GATT_INTERFACE_METHODS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "gatt_interface_types.h"

/**
 * @brief Utility function creates instances of type \ref gatt_intf_service_object_t for the service
 *        Number_of_instances created = \p num_instances (for local server) +
 *              (\p num_instances * max_connections (in \ref gatt_interface_init) (for remote clients)
 *        Total memory required = Number_of_instances * \p object_instance_size  + (internal header)
 *
 * @param p_methods : Service methods for which the instances are to be created
 * @param num_instances : Number of instances
 * @param object_instance_size : Size of each instance to be created
 * @return WICED_SUCCESS in case of successful creation
*/
wiced_result_t gatt_intf_method_create_service_instances(const struct gatt_intf_service_methods_t_ *p_methods,
    int num_instances, int object_instance_size);

/**
 * @brief Utility function gets the index of the included service pointed by \p p_inc_service of service \p p_service for \p service_index_type
 *
 * @param p_service : GATT Profile Instance of the parent/including service
 * @param service_index_type : Service index type, used to get the \ref gatt_intf_included_service_cfg_t from \p p_service
 * @param p_inc_service : Included service instance
 * @return Index of the service, or -1 in case of failure
*/
int gatt_intf_method_get_included_service_index(gatt_intf_service_object_t *p_service, int service_index_type, gatt_intf_service_object_t *p_inc_service);

/**
 * @brief Utility function sets the included service discovery result to the service
 * @param p_service : GATT Profile Instance of the parent/including service
 * @param p_result : Discovery data \ref wiced_bt_gatt_included_service_t for the included service received during the discovery procedure
 * @param p_inc_service : Included service instance
*/
void gatt_intf_method_set_included_service_data(gatt_intf_service_object_t *p_service, wiced_bt_gatt_included_service_t *p_result, gatt_intf_service_object_t *p_inc_service);

/**
 * @brief Utility function to fill the peer GATT DB cache and characteristic data of the service into a buffer
 * @note: The function can be invoked by setting the \p p_data parameter to NULL.
          In this mode the function returns total length which the \p p_service needs to fill into the buffer
 * @param p_service : GATT Profile Instance
 * @param nvram_id  : NVRAM Id to be used to store the service data
 * @param p_data    : Data buffer to store the NVRAM data of the service/service
 * @param len       : Length of the data buffer pointed to by p_data

 * @return size of data filled in \p p_data
*/
int gatt_intf_method_get_nvram_peer_db(gatt_intf_service_object_t *p_service, uint8_t *p_data, int len);


/**
 * @brief Utility functions to free the GATT Profile Instance
 * @param pp_head : Head of the connected GATT Profile Instance list
 * @param p_service: GATT Profile Instance to be freed
*/
void gatt_intf_method_free_instance(gatt_intf_service_object_t **pp_head, gatt_intf_service_object_t *p_service);

/**
 * @brief Utility function to set the service callbacks for the service
 * @param p_service: GATT Profile Instance
 * @param callback: Application callback
 * @param p_app_ctx: Application callback context
*/
void gatt_intf_method_set_service_callback(gatt_intf_service_object_t *p_service, gatt_intf_service_cb_t callback, void *p_app_ctx);

/**
 * @brief Utility function to print the handles of the GATT Profile Instance
 * @param p_service: GATT Profile Instance
*/
void gatt_intf_method_print_handles(gatt_intf_service_object_t *p_service);

/**
 * @brief Profile/Service callback function called to init the service with data stored in NVRAM
 * @param p_service : GATT Profile Instance
 * @param bdaddr: Bluetooth device adddress
 * @param characteristic_type : The characteristic type to write the data to
 * @param p_data : The stored data from NVRAM
 * @param len : The length of the data pointed by p_data
 * @return : The number of bytes read from \p p_data
*/
typedef int (*write_server_characteristic_t)(gatt_intf_service_object_t *p_service, wiced_bt_device_address_t bdaddr, int characteristic_type,
    uint8_t *p_data, int len);

/**
 * @brief Utility function to write the server data to nvram for a bonded device
 * @param p_service : GATT Profile Instance
 * @param bdaddr : Bluetooth device address
 * @param p_data : Data buffer to fill in
 * @param remaining : Size of data available to fill
 * @param p_handle_list : Handle list array
 * @param num_handles : Sizeo of the handle list array
 * @param pfn_write_char : Profile function callback to fill in additional data for a characteristic
 * @return Size of data filled in \p p_data by this function
*/
int gatt_intf_method_write_server_characteristics_to_nvram(gatt_intf_service_object_t *p_service, wiced_bt_device_address_t bdaddr,
    uint8_t *p_data, int remaining,
    gatt_intf_characteristic_handles_t *p_handle_list, int num_handles, write_server_characteristic_t pfn_write_char);

/**
 * @brief Utility function to get the characteristic type from handle
 * @param p_handles : Array of Handles
 * @param max : Size of array pointed by \ref p_pandles
 * @param handle : Handle to search
 * @param p_type_uuid : Returns the UUID, can be either \ref GATT_UUID_CHAR_DECLARE or \ref GATT_UUID_CHAR_CLIENT_CONFIG
 * @return The characteristic_type of the \p handle, -1 on error
*/
int gatt_intf_method_get_characteristic_type_from_handle(const gatt_intf_characteristic_handles_t *p_handles,
    int max, uint16_t handle, int *p_type_uuid);



//////////////////////////////

/**
 * Function       gatt_intf_method_send_write
 *
 *                Write to remote ATT server.
 *                Result is notified using <b> GATT_OPERATION_CPLT_EVT </b> of #wiced_bt_gatt_cback_t.
 *
 *  @param[in]  p_handle    : Pointer to handles
 *  @param[in]  conn_id     : Connection handle
 *  @param[in]  opcode      : The write Opcode to be used for the write
 *  @param[in]  len         : The length of data to be written
 *  @param[in]  p_data      : Pointer to the data
 *  @param[in]  pv_ctx      : Context of \p p_data
 *
 *  @return @link wiced_bt_gatt_status_e wiced_bt_gatt_status_t @endlink
 */
wiced_bt_gatt_status_t gatt_intf_method_send_write_value_handle(gatt_intf_characteristic_handles_t *p_handle, uint16_t conn_id,
    wiced_bt_gatt_opcode_t opcode, uint16_t len, uint8_t *p_data, void *pv_ctx);

/**
 * Function       gatt_intf_method_send_read
 *
 *                Read from remote ATT server.
 *                Result is notified using <b> GATT_OPERATION_CPLT_EVT </b> outstandingof #wiced_bt_gatt_cback_t.
 *
 *  @param[in]  p_service : Pointer to service's context data that will be passed in the callback.
 *  @param[in]  conn_id   : Connection handle
 *  @param[in]  max_read_len: Max len of the variable to be read
 *
 *  @return @link wiced_bt_gatt_status_e wiced_bt_gatt_status_t @endlink
 *
 */
wiced_bt_gatt_status_t gatt_intf_method_send_read(const gatt_intf_characteristic_handles_t *p_handles, uint16_t conn_id, uint16_t max_read_len);

/**
 * Function       gatt_intf_method_send_read_descriptor
 *
 *                Read from remote ATT server.
 *                Result is notified using <b> GATT_OPERATION_CPLT_EVT </b> outstandingof #wiced_bt_gatt_cback_t.
 *
 *  @param[in]  p_service : Pointer to service's context data that will be passed in the callback.
 *  @param[in]  conn_id   : Connection handle
 *  @param[in]  descriptor_type: 16 bit UUID of the descriptor type, for e.g, GATT_UUID_CHAR_CLIENT_CONFIG
 *  @param[in]  max_read_len: Max len of the variable to be read
 *
 *  @return @link wiced_bt_gatt_status_e wiced_bt_gatt_status_t @endlink
 *
 */
wiced_bt_gatt_status_t gatt_intf_method_send_read_descriptor(const gatt_intf_characteristic_handles_t *p_handles, uint16_t conn_id,
    uint16_t descriptor_type, uint16_t max_len);

/**
 * Function       gatt_intf_method_set_descriptor_value
 *
 *                Read from remote ATT server.
 *                Result is notified using <b> GATT_OPERATION_CPLT_EVT </b> outstandingof #wiced_bt_gatt_cback_t.
 *
 *  @param[in]  p_service : Pointer to service's context data that will be passed in the callback.
 *  @param[in]  conn_id   : Connection handle
 *  @param[in]  descriptor_type : 16 bit UUID of the descriptor type, for e.g, GATT_UUID_CHAR_CLIENT_CONFIG
 *  @param[in]  value_len : Length of the value to be written
 *  @param[in]  p_value   : Pointer to the value to be written
 *
 *  @return @link wiced_bt_gatt_status_e wiced_bt_gatt_status_t @endlink
 *
 */
wiced_bt_gatt_status_t gatt_intf_method_set_descriptor_value(const gatt_intf_characteristic_handles_t *p_handles, uint16_t conn_id,
    uint16_t descriptor_type, uint16_t value_len, uint8_t *p_value);

/**
 *
 * Function     gatt_intf_method_send_notification
 *
 *              Send a handle value notification to a client
 *
 *  @param[in]  conn_d          : connection id
 *  @param[in]  char_handle     : Attribute handle of this handle value indication.
 *  @param[in]  data_len        : Length of notification value passed.
 *  @param[in]  p_val           : Notification Value.
 *  @param[in]  pv_ctx          : app buffer ctx
 *
 *  @return @link wiced_bt_gatt_status_e wiced_bt_gatt_status_t @endlink
 */
wiced_bt_gatt_status_t gatt_intf_method_send_notification( uint16_t conn_id,
    const gatt_intf_characteristic_handles_t *p_handle,
    uint16_t data_len,
    uint8_t *p_data,
    void *pv_ctx);

/**
 *
 * Function     gatt_intf_method_send_write_response
 *
 *              Sends back the response to a write req
 *
 *  @param[in]  p_char     : characteristic handle to respond
 *  @param[in]  conn_id    : GATT connection id
 *  @param[in]  opcode     : Opcode received in the request
 *  @param[in]  status     : Notification Value.
 *
 *  @return @link wiced_bt_gatt_status_e wiced_bt_gatt_status_t @endlink
 */
wiced_bt_gatt_status_t gatt_intf_method_send_write_response(const gatt_intf_characteristic_handles_t *p_char, uint16_t conn_id,
    wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_status_t status);

enum
{
    SUBSCRIPTION_NONE,         /**< Does not allow both notifications and indications */
    SUBSCRIPTION_NOTIFICATION, /**< Allows notifications  */
    SUBSCRIPTION_INDICATION    /**< Allows indications  */
};
typedef uint16_t subscription_enums_t;

/**
 *
 * Function     gatt_intf_method_get_subscription_value_conn_id
 *
 *              Sends back the response to a write req
 *
 *  @param[in]  conn_id    : GATT connection id
 *  @param[in]  p_handle   : characteristic handle
 *
 *  @return @link value of the subscription @endlink
 */
int gatt_intf_method_get_subscription_value_conn_id(uint16_t conn_id,
                                                    const gatt_intf_characteristic_handles_t *p_handle);

/**
 * Function     gatt_intf_method_get_subscription_value_conn_id
 *
 *              Sends back the response to a write req
 *
 *  @param[in]  conn_id    : GATT connection id
 *  @param[in]  p_handle   : characteristic handle
 *  @param[in]  value      : subscription value
 *
 *  @return @link value of the subscription @endlink
 */
int gatt_intf_method_set_subscription_value_conn_id(uint16_t conn_id,
                                                    const gatt_intf_characteristic_handles_t *p_handle,
                                                    uint16_t value);

/**
 * Function     gatt_intf_method_get_buffer
 *
 *              Allocate a buffer to send to GATT
 *
 *  @param[in]  len : length of the buffer
 *
 *  @return @link buffer allocated @endlink
 */
uint8_t *gatt_intf_method_get_buffer(int len);


/**
 * Function     gatt_intf_method_alloc_server_notification_packet
 *
 *              Free buffer allocated with \ref gatt_intf_method_get_buffer
 *
 *  @param[in]  conn_id    : GATT connection id
 *  @param[in]  p_handle   : characteristic handle
 *  @param[in]  type       : Notification or Indication
 *  @param[in]  req_len    : Requested buffer len
 *  @return @link buffer allocated @endlink
 */
uint8_t *gatt_intf_method_alloc_server_notification_packet(uint16_t conn_id,
                                                           gatt_intf_characteristic_handles_t *p_handle,
                                                           subscription_enums_t type,
                                                           int req_len);

/**
 * Function     gatt_intf_method_free_buffer
 *
 *              Free buffer allocated with \ref gatt_intf_method_get_buffer or \ref gatt_intf_method_alloc_server_notification_packet
 *
 *
 *  @param[in]  len : length of the buffer
 *
 *  @return @link buffer allocated @endlink
 */
void gatt_intf_method_free_buffer(uint8_t *ptr);

    /**
 * @brief Function to notify a specific char value to all the subscribed clients
 *
 * @param conn_id : GATT connection Id
 * @param p_service : GATT Profile Instance
 * @param p_char : Characteristic to be notified
 * @param p_n : Data to be notified
 * @param len : length of notification data
 * @param pfn_op_cmpl : Callback function registered for receiving status of the operation
 *
 * @return wiced_bt_gatt_status_t
 * */
wiced_bt_gatt_status_t gatt_intf_notify_characteristic_to_all_clients(
    uint16_t conn_id,
    gatt_intf_service_object_t *p_service,
    gatt_intf_attribute_t *p_characteristic,
    void *p_n,
    on_gatt_interface_operation_complete_t pfn_op_cmpl);
#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_GA_GATT_INTERFACE_H__ */
