/*
 * $ Copyright Cypress Semiconductor $
 */

/**************************************************************************/ /**
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

#ifndef __WICED_BT_GATT_INTERFACE_H__
#define __WICED_BT_GATT_INTERFACE_H__

#include "gatt_interface_methods.h"
#include "gatt_interface_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
 * @brief Initialize the gatt interface
 * Sets up the device context for \p max_connections. The device context created here is allocated on connection.
 * The allocated device context is freed by the application on disconnection by calling \ref gatt_interface_free_client_device_ctx
 *
 * @param max_client_connections : Max simultaneous remote clients (max 255, <= \ref wiced_bt_cfg_ble_t.ble_max_simultaneous_links)
 * @param max_mtu: Maximum MTU to be setup on connection, ( <= \ref wiced_bt_cfg_ble_t.ble_max_rx_pdu_size)
 * @param default_auth_req: Default Auth request to be applied for all gatt client requests
 * @return WICED_SUCCESS on successful init
*/
    wiced_result_t gatt_interface_init(int max_connections, int max_mtu, wiced_bt_gatt_auth_req_t default_auth_req);

    /**
 * @brief  Function to invoke the specific gatt handler of the initialized services on server and client
 * @param p_service : GATT service object instance for which the event was received
 * @param event     : \ref wiced_bt_gatt_evt_t received from GATT
 * @param p_event_data : Event data
*/
    wiced_bt_gatt_status_t gatt_interface_invoke_gatt_handler(wiced_bt_gatt_evt_t event,
                                                              wiced_bt_gatt_event_data_t *p_event_data);

    /**
* @brief Setup the server services by reading the GATT DB inited with \ref wiced_bt_gatt_db_init and \ref wiced_bt_gatt_add_services_to_db
* @note The services/profiles to be supported have to be initialized and registered with gatt_interface prior to invoking this API.
*
* @param[in]  pfn_can_save : Application Callback. Invoked during function execution to check if the application is interested in the service UUID.
                             Application returns WICED_TRUE if interested in the service, WICED_FALSE otherwise
* @param[in]  pfn_store    : Application Callback to store the service/service handle of type \ref gatt_intf_service_object_t found during the setup process
* @param[in]  p_app_ctx    : Application Callback context, passed as argument to pfn_can_save and pfn_store
*
* @return WICED_SUCCESS on successful initialization of all profiles/services
*/
    wiced_bt_gatt_status_t gatt_interface_setup_services_from_local_db(fn_on_service_handle_check_to_save pfn_can_save,
                                                                       fn_on_service_handle_store pfn_store,
                                                                       void *p_app_ctx);

    /**
 * @brief Function invoked by application to start the discovery process
 * @param conn_id: GATT Connection Id
 * @param pfn_can_save : Application returns WICED_TRUE to allow initialization of a GATT Profile Instance
 * @param pfn_save     : Application receives the callback after successful allocation of a GATT Profile Instance.
 *                       The application should invoke \ref gatt_interface_set_callback_to_profile to set the callbacks
 *                       in this callback
 * @param pfn_on_complete : Application callback called on completion of the discovery procedure
 * @param p_app_ctx : Callback context when invoking \p pfn_can_save, \p pfn_save and \p pfn_on_complete
 * @return \ref wiced_result_t
*/
    wiced_result_t gatt_interface_start_discovery(uint16_t conn_id,
                                                  fn_on_service_handle_check_to_save pfn_can_save,
                                                  fn_on_service_handle_store pfn_save,
                                                  fn_on_discovery_complete pfn_on_complete,
                                                  void *p_app_ctx);

    /**
 * @brief Function to read the current discovery state. The application can track the discovery process through this function.
 *        Typically this function will be invoked on \ref GATT_DISCOVERY_CPLT_EVT
 *        \ref discovery_state_t of \ref DISCOVERY_STATE_DISCOVER_COMPLETE signals that the discovery process is complete.
 *        At this stage the application can use the read/write APIs to send data to the remote server
 * @param conn_id : GATT connection Id
 * @return \ref discovery_state_t
*/
    discovery_state_t gatt_interface_get_discovery_state(uint16_t conn_id);

    /**
 * @brief Function to set the application callbacks to the \p p_service
 * @param p_service : GATT Profile Instance
 * @param callback : Application callback to set
 * @param p_app_ctx : Application Context for the callback
 * @return wiced_result_t
*/
    wiced_result_t gatt_interface_set_callback_to_profile(gatt_intf_service_object_t *p_service,
                                                          gatt_intf_service_cb_t callback,
                                                          void *p_app_ctx);

    /**
 * @brief Function to request to read the remote server characteristic data
 * @param conn_id : GATT connection Id
 * @param p_service: GATT Profile Instance
 * @param p_char : Characteristic to read
 *
 * @return wiced_bt_gatt_status_t
 */
    wiced_bt_gatt_status_t gatt_interface_read_characteristic(uint16_t conn_id,
                                                              gatt_intf_service_object_t *p_service,
                                                              gatt_intf_attribute_t *p_char);

    /**
 * @brief Function to request to write data to the remote server characteristic
 * @param conn_id : GATT connection Id
 * @param p_service: GATT Profile Instance
 * @param p_char : Characteristic to write
 * @param p_write_data: data specific to the characteristic to be written
 *
 * @return wiced_bt_gatt_status_t
 */
    wiced_bt_gatt_status_t gatt_interface_write_characteristic(uint16_t conn_id,
                                                               gatt_intf_service_object_t *p_service,
                                                               gatt_intf_attribute_t *p_char,
                                                               void *p_write_data);

    /**
 * @brief Function to request to send a notification to the remote server characteristic
 * @param conn_id : GATT connection Id
 * @param p_service: GATT Profile Instance
 * @param p_char : Characteristic to notify
 * @param p_notification: data specific to the characteristic to be notified
 *
 * @return wiced_bt_gatt_status_t
 */
    wiced_bt_gatt_status_t gatt_interface_notify_characteristic(uint16_t conn_id,
                                                                gatt_intf_service_object_t *p_service,
                                                                gatt_intf_attribute_t *p_char,
                                                                void *p_notification);

    /**
 * @brief Function to request to enable notifications to the remote server characteristic
 * @param conn_id : GATT connection Id
 * @param p_service: GATT Profile Instance
 * @param p_char : Characteristic to enable notifications
 * @param value: value of type \ref wiced_bt_gatt_client_char_config_t
 *
 * @return wiced_bt_gatt_status_t
 */
    wiced_bt_gatt_status_t gatt_interface_enable_notifications(uint16_t conn_id,
                                                               gatt_intf_service_object_t *p_service,
                                                               gatt_intf_attribute_t *p_char,
                                                               uint16_t value);

    // Utility functions to read/enable notifications for all

    /**
 * @brief Function to initiate operations on characteristics of a service
 *
 * @param conn_id : GATT connection Id
 * @param p_service : GATT Profile Instance
 * @param operation : Operation type
 * @param pfn_op_cmpl : Callback function registered for receiving status of the operation
 *
 * @return wiced_bt_gatt_status_t
 * */

wiced_bt_gatt_status_t gatt_interface_characteristic_operation(uint16_t conn_id,
                                                               gatt_intf_service_object_t *p_service,
                                                               gatt_intf_operation_t operation,
                                                               on_gatt_interface_operation_complete_t pfn_op_cmpl);

    // nvram
/**
 * @brief Funtion to write characteristic data to nvram. Function writes both client and server
 *        related information for the device with \p bdaddr
 *
 * @param bdaddr : Address of the remote Bluetooth device
 * @param p_nv: Structure holding the methods for accessing nvram
 * @param device_nvram_id: Start of the nvram ids for the device.
 *   NVRAM data for the device is stored between \p device_nvram_id + 1 to (\p device_nvram_id + 2 + 2 * p_nv->max_services)
 *
 * return Total size of data written to nvram.
 * @note A positive return value indicates a successful write. A negative value indicates failure.
 *
 * */
int gatt_interface_nvram_write(wiced_bt_device_address_t bdaddr,
                                   const gatt_interface_nvram_data_t *p_nv,
                                   int device_nvram_id);

/**
 * @brief Funtion to read characteristic data from nvram. Function reads both client and server
 *        related information for the device with \p bdaddr
 *
 * @param bdaddr : Address of the remote Bluetooth device
 * @param p_nv: Structure holding the methods for accessing nvram
 * @param device_nvram_id: Start of the nvram ids for the device.
 *   NVRAM data for the device is stored between \p device_nvram_id + 1 to (\p device_nvram_id + 2 + 2 * p_nv->max_services)
 *
 * return Total size of data read from nvram
 * @note A positive return value indicates a successful read. A negative value indicates failure.
 *
 * */
int gatt_interface_nvram_read(wiced_bt_device_address_t bda,
                                  const gatt_interface_nvram_data_t *p_nv,
                                  int device_nvram_id,
                                  void *p_app_ctx);

    /**
 * @brief Function frees the device data related to the \ref bdaddr in the gatt interface layer
 * @note : This function should be invoked at the end of application cleanup on disconnection
 * @param bdaddr : Bluetooth device address
 *
*/
void gatt_interface_free_client_device_ctx(wiced_bt_device_address_t bdaddr);

// Utility functions
/**
 * @brief Function to print the linked handles
 * @param bdaddr : Bluetooth device address
*/
void gatt_interface_print_linked_handles(wiced_bt_device_address_t bdaddr);

/**
 * @brief Function to get the service object with a bluetooth address, use NULL for local device service
 * @param bdaddr : Bluetooth device address
 * @param p_uuid : UUID of service
 *
 * @return valid service object or NULL
*/
gatt_intf_service_object_t *gatt_interface_get_service_by_uuid(wiced_bt_device_address_t bdaddr,
                                                                   const wiced_bt_uuid_t *p_uuid);

    /**
 * @brief Function to get the service object with a conn_id, use 0 for local device service
 * @param conn_id : GATT connection identifier, use 0 for server
 * @param p_uuid : UUID of service
 *
 * @return valid service object or NULL
*/

gatt_intf_service_object_t *gatt_interface_get_service_by_uuid_and_conn_id(uint16_t conn_id,
                                                                               const wiced_bt_uuid_t *p_uuid);

    /**
 * @brief Function to get the client service from the list attached to the bdadddr at index
 * @param bdaddr : Bluetooth device address
 * @param index  : Index to get
 * @return gatt_intf_service_object_t *
*/
gatt_intf_service_object_t *gatt_interface_get_linked_client_profile_at(wiced_bt_device_address_t bdaddr,
                                                                            int index);

    /**
 * @brief Function to get the service from the list attached to the server at index
 * @param bdaddr : Bluetooth device address
 * @param index  : Index to get
 * @return gatt_intf_service_object_t *
*/
gatt_intf_service_object_t *gatt_interface_get_linked_server_profile_at(int index);

/**
 * @brief Function to get the next service from the list attached \p p_service
 * @param p_service  : Current service object
 * @return a valid (non NULL) gatt_intf_service_object_t * or NULL (no more services)
*/

gatt_intf_service_object_t *gatt_interface_get_next_linked_profile(gatt_intf_service_object_t *p_service);

void gatt_intf_handle_disconnection(uint16_t conn_id);

#define GATT_INTERFACE_TRACE(...) WICED_BT_TRACE(__VA_ARGS__)
#define GATT_INTERFACE_TRACE_CRIT(...) WICED_BT_TRACE_CRIT(__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_GA_GATT_INTERFACE_H__ */
