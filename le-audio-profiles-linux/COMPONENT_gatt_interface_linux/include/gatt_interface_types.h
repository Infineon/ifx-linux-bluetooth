/*
 * $ Copyright Cypress Semiconductor $
 */


 /**************************************************************************//**
  * \file <gatt_interface_types.h>
  *
  * Type definitions used by GATT Interface
  *
  */

#ifndef __GATT_INTERFACE_TYPES_H__
#define __GATT_INTERFACE_TYPES_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_result.h"

#define MAX_NVRAM_AREA 256

/**
 * @brief GATT Interface events.
 */
typedef enum {
    READ_REQ_EVT = 1,  /**< Read request event */
    WRITE_REQ_EVT,     /**< Write request event */
    READ_CMPL_EVT,     /**< Read complete event */
    WRITE_CMPL_EVT,    /**< Write complete event */
    NOTIFICATION_EVT,  /**< Notification event */
    INDICATION_EVT,    /**< Indication event */
    READ_BLOB_REQ_EVT  /**< Read request blob event, indicates long read
                        * using a flag "is_long_read". Check
                        * \ref _gatt_intf_included_service_cfg_t.read_local_attribute
                        * Event is expected to be handled similar to \ref READ_REQ_EVT
                        * Event is expected to return an error in case the handler
                        * detects a change in value during a long read initiated by the client
                        */
}gatt_interface_events_t;

typedef enum {
    DISCOVERY_STATE_IDLE,
    DISCOVERY_STATE_DISCOVER_PRIMARY_SERVICES,
    DISCOVERY_STATE_DISCOVER_INCLUDE_SERVICES,
    DISCOVERY_STATE_DISCOVER_CHARACTERISTICS,
    DISCOVERY_STATE_DISCOVER_CHARACTERISTIC_DESCRIPTORS,
    DISCOVERY_STATE_DISCOVER_COMPLETE
}discovery_state_t;


typedef struct _gatt_intf_service_object_t gatt_intf_service_object_t;
typedef struct gatt_intf_service_methods_t_ gatt_intf_service_methods_t;
typedef struct _gatt_intf_supported_services_t gatt_intf_supported_services_t;
typedef struct gatt_intf_device_ctx_t_ gatt_intf_device_ctx_t;

/**
 * @brief Structure describes an attribute of the GATT Database
*/
typedef struct
{
    uint32_t included_service_type : 3;       /**< Included service type, valid if not 0 */
    uint32_t included_service_instance : 4;   /**< Instance of the included type indicated by the included_service_type member of this structure*/

    uint32_t descriptor_type : 4;             /**< Descriptor type, enumeration of descriptor types, valid if not 0 */
    uint32_t characteristic_type : 5;         /**< Characteristic type, enumeration of characteristic types starting from 0.
                                                   Enumeration defined by the specific service implementations */
    uint32_t characteristic_instance : 4;     /**< Instance of the characteristic type, indicated by the characteristic member of this structure  */
}gatt_intf_attribute_t;

/**
 * @brief String data structure
*/
typedef struct
{
    char *str; /**< pointer to string */
    uint32_t len; /**< string length */
} gatt_intf_string_t;

/**
 * GATT interface event notification callback
 *
 * Callback for GATT database event notifications
 * Registered using \ref gatt_intf_service_methods_t.set_callback
 *
 * @param conn_id        : GATT connection id
 * @param p_app_ctx      : Application context registered using \ref gatt_intf_service_methods_t.set_callback
 * @param p_service      : GATT Service instance
 * @param status         : GATT operation status
 * @param event_type     : Event type, \ref gatt_interface_events_t
 * @param p_char         : Characteristic for which the event has occurred
 * @param p_data         : Event Data
 * @param len            : Event Data len
 *
 * @return Status of event handling
*/
typedef wiced_result_t(*gatt_intf_service_cb_t)(uint16_t conn_id, void *p_app_ctx, gatt_intf_service_object_t *p_service,
    wiced_bt_gatt_status_t status,
    uint32_t event_type,
    gatt_intf_attribute_t *p_char, void *p_data, int len);


/**
* GATT interface included service reference
* Structure used by service implementations which have included services
*/
typedef struct {
    uint16_t handle;  /**< Handle at which the included service is found in the including/parent service */
    gatt_intf_service_object_t *p_inc_service; /**< Included Profile Instance */
}gatt_intf_included_service_t;


/**
 * gatt service structure provided by service
 */
struct _gatt_intf_service_object_t
{
    gatt_intf_supported_services_t  *p_info;         /**< interface methods */
    gatt_intf_service_cb_t           p_app_callback; /**< callback pointer to the application */
    void                            *p_app_ctx;      /**< application callback context */
    uint16_t                         start_handle;   /**< start handle to the services GATT db which is used to filter the events from the peer */
    uint16_t                         end_handle;     /**< end handle to the services GATT db which is used to filter the events from the peer */
    struct _gatt_intf_service_object_t *next;           /**< next interface handler */
};

/**
 * gatt service characteristic output structure provided to service
 */
typedef struct
{
    uint16_t                               char_handle;         /**< Output : Characteristic handle  */
    uint16_t                               char_val_handle;     /**< Output : Characteristic value handle */
    uint16_t                               desc_handle;         /**< Output : Characteristic Descriptor handle value */
} gatt_intf_characteristic_handles_t;

typedef struct {
    wiced_bt_uuid_t uuid;                 /**< UUID of the characteristic */
    uint8_t characteristic_type;          /**< Characteristic type */
    uint8_t num_handles;                  /**< Number of handles for the characteristic */
    uint32_t is_mandatory : 1;            /**< Characteristic mandatory */
    uint32_t is_readable : 1;             /**< is characteristic readable */
    uint32_t is_writable : 1;             /**< is characteristic writable */
    uint32_t is_notifiable : 1;           /**< is characteristic notifiable */
    char *name;                           /**< Name of the characteristic */
} gatt_intf_characteristic_info_t;

/**
 * @brief Helper structure for the included service methods
*/
typedef struct _gatt_intf_included_service_cfg_t {
    /**
     * @brief Reference to the \ref gatt_intf_service_methods_t of the included service
    */
    const gatt_intf_service_methods_t *p_methods;
    /**
     * @brief Gets the included service instance of the parent service \p p_parent at index \p num
     * @param p_parent : Parent service
     * @param num      : Index of the included service of type \ref gatt_intf_included_service_cfg_t.p_methods->uuid
     * @return         : The included service data else NULL if not present
    */
    gatt_intf_included_service_t *(*get_included_instance)(gatt_intf_service_object_t *p_parent, int num);

    /**
     * @brief Set the included service
     * @param p_parent:
     * @param num :
     * @param handle:
     * @param p_inc_service: Included Profile Instance
     * @return
    */
    gatt_intf_service_object_t *(*set_included_instance)(gatt_intf_service_object_t *p_parent, int num, int handle, gatt_intf_service_object_t *p_inc_service);

    /**
     * @brief Returns the max number of included instances of this type
    */
    int (*get_max_to_find)(void);
    /**
     * @brief Set to the parent callback
    */
    gatt_intf_service_cb_t included_callback;

    /**
     * @brief Function sets the parent handle of the parent service to the included service
     * @param p_inc_service : Included Profile Instance
     * @param handle : Parent handle, at which the \p p_inc_service exists in the GATT database
    */
    void (*set_parent_handle)(gatt_intf_service_object_t *p_inc_service, uint16_t handle);

    /**
     * @brief Function returns the parent handle
     * @param p_inc_service: Included service
     * @return Parent handle, at which the \p p_inc_service exists in the GATT database
     *
    */
    uint16_t(*get_parent_handle)(gatt_intf_service_object_t *p_inc_service);
}gatt_intf_included_service_cfg_t;


/**
 * @brief Defines the variables set and function to be implemented by a new Service implementation
 *
 * Each GATT service/service over the GATT interface has to implement the following member functions
*/
struct gatt_intf_service_methods_t_ {
    char *name; /**< Service Name */
    wiced_bt_uuid_t uuid; /**< Service UUID */
    uint16_t service_type_uuid; /**< set to either of #GATT_UUID_PRI_SERVICE or #GATT_UUID_SEC_SERVICE*/

    /**
     * @brief Callback called during service instance creation, for any further initializations to be done by the service
     * @param p_service : GATT Service instance
    */
    void (*on_instance_create)(gatt_intf_service_object_t *p_service);

    /**
     * @brief Function to free the \p p_service instance, typically called on disconnect
     * @param pp_head : Head of the list of the GATT Profile instances associated with a specific bluetooth adddress
     * @param p_service : GATT Profile Instance to be freed
    */
    void (*free_instance)(gatt_intf_service_object_t **pp_head, gatt_intf_service_object_t *p_service);

    /**
     * @brief Function to set the application callback and the application context to be used by the service implementation on events
     * @param p_service : GATT Profile Instance
     * @param callback  : Callback function implemented by the caller
     * @param p_app_ctx : Application context set by the caller
    */
    void (*set_callback)(gatt_intf_service_object_t *p_service, gatt_intf_service_cb_t callback, void *p_app_ctx);

    /**
     * @brief Function to set the included service data, in case the service uses included services
     *        For services which do not use included services this member can be NULL
     * @param p_service : GATT Profile Instance
     * @param p_result : Included service discovery data \ref wiced_bt_gatt_included_service_t for the included service
     *                   received during the discovery procedure
     * @param p_inc_service : Included Profile instance
    */
    void (*set_included_service_data)(gatt_intf_service_object_t *p_service, wiced_bt_gatt_included_service_t *p_result,
        gatt_intf_service_object_t *p_inc_service);

    /**
     * @brief Function to set the characteristic data received during the GATT service discovery
     * @param p_service : GATT Profile Instance
     * @param p_result  : Characteristic discovery data \ref wiced_bt_gatt_char_declaration_t for the characteristic received
     *                    during the discovery procedure
     * @param p_cc      : Characteristic type info of which the \p p_result was received
    */
    void (*set_characteristic_data)(gatt_intf_service_object_t *p_service,
        wiced_bt_gatt_char_declaration_t *p_result, const gatt_intf_characteristic_info_t *p_cc);
    /**
     * @brief Function to set the descriptor data received during the GATT service discovery
     * @note: Currently only GATT_UUID_CHAR_CLIENT_CONFIG type descriptors supported
     *
     * @param p_service : GATT Profile Instance
     * @param char_handle: Current characteristic handle to be associated with descriptor in \p p_result
     * @param p_result  : Descriptor discovery data \ref wiced_bt_gatt_char_descr_info_t for the descriptor
     *                    received during the discovery procedure
    */
    void (*set_descriptor_data)(gatt_intf_service_object_t *p_service,
        uint16_t char_handle, const wiced_bt_gatt_char_descr_info_t *p_result);

    /**
     * @brief Function to handle GATT read events received by the GATT server
     * This callback will be invoked by the GATT Interface on receiving \ref GATT_REQ_READ,
     * \ref GATT_REQ_READ_BLOB, \ref GATT_REQ_READ_MULTI, \ref GATT_REQ_MULTI_VAR_LENGTH from the remote client
     * It may also be invoked when the server initiates sending notifications.
     * In all cases the function implementation is expected to return the entire attribute value in \p p_attr and
     * update the attribute length in \p p_attr_len
     *
     * @param p_service : GATT Profile Instance
     * @param conn_id : GATT connection id
     * @param attr_handle : Attribute handle to read
     * @param is_long_read : In case \p is_long_read is not zero, and the value of attribute being read
     *      was updated then profile is expected to send an error response.
     * @note During long read on an attribute it is possible that the value of the attribute is updated while it is
     *       being read by a client/s. In such cases the profile/application is expected to return an error based on
     *       the profile/service specification.
     * @param len_to_read : Length to read
     * @param p_attr_len : Pointer to hold the actual attribute length read
     * @param p_attr : Pointer which holds the attribute data
     * @return wiced_bt_gatt_status_t
    */
    wiced_bt_gatt_status_t(*read_local_attribute)(gatt_intf_service_object_t *p_service, uint16_t conn_id,
        uint16_t attr_handle, uint16_t is_long_read, uint16_t len_to_read, int *p_attr_len, uint8_t *p_attr);

    /**
     * @brief Function to handle GATT write events received by the GATT server
     * This callback will be invoked by the GATT Interface on receiving \ref GATT_REQ_WRITE,
     * \ref GATT_CMD_WRITE, \ref GATT_CMD_SIGNED_WRITE and on execution of \ref GATT_REQ_EXECUTE_WRITE from the remote client
     * @param p_service : GATT Profile Instance
     * @param conn_id : GATT connection id
     * @param attr_handle : Attribute handle to write
     * @param len_to_write : Length to write
     * @param p_attr : Pointer to the attribute data
     * @return wiced_bt_gatt_status_t
    */
    wiced_bt_gatt_status_t(*write_local_attribute)(gatt_intf_service_object_t *p_service, uint16_t conn_id,
        uint16_t attr_handle, uint16_t len_to_write, uint8_t *p_attr);

    /**
     * @brief Function to handle GATT operation complete for operations started by the local client
     * This callback will be invoked by the GATT Interface on completion of a GATT read, write, discover operation
     *
     * @param p_service : GATT Profile Instance
     * @param conn_id : GATT connection id
     * @param attr_handle : Attribute handle to write
     * @param len_to_write : Length to write
     * @param p_attr : Pointer to the attribute data
     * @return wiced_bt_gatt_status_t
    */
    wiced_bt_gatt_status_t(*operation_complete)(gatt_intf_service_object_t *p_service, wiced_bt_gatt_operation_complete_t *p_cmpl);

    // iteration functions
    /**
     * @brief Funtion returns the \ref gatt_intf_characteristic_info_t instance at index \p num
     * @param index : Index in the list of the \ref gatt_intf_characteristic_info_t implemented in the service
     * @return An instance of \ref gatt_intf_characteristic_info_t or NULL in case no instance found
    */

    const gatt_intf_characteristic_info_t *(*get_gatt_client_characteristics_info)(int index);
    /**
     * @brief Function returns \ref gatt_intf_characteristic_handles_t instance of type \p p_cc at index \p index
     * @param p_service : GATT Profile Instance
     * @param p_cc : Type of the characteristic
     * @param index : Index into list of characteristics of type \p p_cc
     * @return An instance of \ref gatt_intf_characteristic_handles_t, or NULL in case no instance found
    */
    gatt_intf_characteristic_handles_t *(*get_characteristic_handles_at_index)(gatt_intf_service_object_t *p_service,
        const gatt_intf_characteristic_info_t *p_cc, int index);

    /**
     * @brief Function to return the included service configuration at index \p service_type_index
     * @param service_type_index : Index into list of included service types supported by the service/service implementation
     * @return An instance of \ref gatt_intf_included_service_cfg_t, or NULL in case no instance found
    */
    const gatt_intf_included_service_cfg_t *(*get_included_service_list)(int service_type_index);

    /**
     * @brief Function to print the handles used in the service
     * @param p_service: GATT Profile Instance
    */
    void (*print_handles)(gatt_intf_service_object_t *p_service);

    /**
     * @brief Function to read the peer db handles/characteristic data to buffer
     * @note: Calling the function with \p p_data set to NULL, does a dry run of the contents to be written and
     * returns the maximum size of data to be written into \p p_data
     *
     * @param p_service : GATT Profile Instance
     * @param p_data : Data buffer to store the handle/characteristic information.
     * @param len : Length of the buffer.
     * @return: Returns the number of valid bytes in \p p_data
    */
    int (*get_nvram_peer_db)(gatt_intf_service_object_t *p_service, uint8_t *p_data, int len);

    /**
     * @brief Function to read the server side (descriptor/characteristic) data to buffer
     * @note: Calling the function with \p p_data set to NULL, does a dry run of the contents to be written and
     * returns the maximum size of data to be written into \p p_data
     *
     * @param p_service : GATT Profile Instance
     * @param bdaddr : Address of the remote Bluetooth device connected to this server
     * @param p_data : Data buffer to store the handle/characteristic information.
     * @param len : Length of the buffer.
     * @return: Returns the number of valid bytes in \p p_data
     */
    int (*get_nvram_server_data)(gatt_intf_service_object_t *p_service, wiced_bt_device_address_t bdaddr, uint8_t *p_data, int len);

    /**
     * @brief Function to request to read the remote server characteristic data
     * @param conn_id : GATT Connection identifier
     * @param p_service: GATT Profile Instance
     * @param p_char : Attribute to be read
     */
    wiced_bt_gatt_status_t (*read_remote_attribute)(uint16_t conn_id,
                                                  gatt_intf_service_object_t *p_service,
                                                  gatt_intf_attribute_t *p_char);

    /**
     * @brief Function to request to write data to the remote server characteristic
     * @param conn_id : GATT Connection identifier
     * @param p_service: GATT Profile Instance
     * @param p_char : Attribute to be written
     * @param p_write_data: Pointer to data structure defined by the specific profile/service
     */
    wiced_bt_gatt_status_t (*write_remote_attribute)(uint16_t conn_id,
                                                     gatt_intf_service_object_t *p_service,
                                                     gatt_intf_attribute_t *p_char,
                                                     void *p_write_data);

    /**
     * @brief Function to request to send a notification to the remote server characteristic
     * @param conn_id : GATT Connection identifier
     * @param p_service: GATT Profile Instance
     * @param p_char : Attribute to be written
     * @param p_notification: Pointer to data structure defined by the specific profile/service
     */
    wiced_bt_gatt_status_t (*notify_to_remote)(uint16_t conn_id,
                                               gatt_intf_service_object_t *p_service,
                                               gatt_intf_attribute_t *p_char,
                                               void *p_notification);

};


/**
* @brief Callback function to application to get a permission to save service
* @param p_app_ctx : Application context to the callback function
* @param p_uuid    : UUID of the service to be saved
* @return WICED_TRUE if application wants to save the service, WICED_FALSE otherwise
*/
typedef wiced_bool_t(*fn_on_service_handle_check_to_save)(void *p_app_ctx, wiced_bt_uuid_t *p_uuid);

/**
* @brief Callback function to application to allow it to store the allocated service reference
* @param p_app_ctx : Application context to the callback function
* @param p_uuid    : UUID of the service to be stored
* @param p_service : GATT Service instance
*/
typedef void (*fn_on_service_handle_store)(void *p_app_ctx, wiced_bt_uuid_t *p_uuid, gatt_intf_service_object_t *p_service);

/**
* @brief Callback function to application to allow it to store the allocated service reference
* @param p_app_ctx : Application context to the callback function
* @param conn_id : Connection Id
*/
typedef void (*fn_on_discovery_complete)(uint16_t conn_id, wiced_bt_gatt_status_t status);

enum gatt_intf_operation_e
{
    GATT_INTF_OPERATION_NOTIFY_ALL_CHARACTERISTICS, /**< Notify characteristic data to remote client */
    GATT_INTF_OPERATION_ENABLE_NOTIFICATIONS,          /**< Enable notifications on the server */
    GATT_INTF_OPERATION_NOTIFY_CHARACTERISTIC_TO_ALL_CLIENTS,                   /**< Nofity characteristic data to all subscribed clients*/
    GATT_INTF_OPERATION_READ,                          /**< Read from remote server */
    GATT_INTF_OPERATION_MAX                            /**< Max operation */
};
typedef uint8_t gatt_intf_operation_t; /**< \ref gatt_intf_operation_e */

/**
 * @brief Callback function to indicate operation complete
 *
 * @param conn_id : GATT connection Id
 * @param p_service : GATT Profile Instance
 * @param operation : Operation type
 * @param status : Status of the operation
 *
 * @return void
 * */
typedef void (*on_gatt_interface_operation_complete_t)(uint16_t conn_id,
                                                       gatt_intf_service_object_t *p_service,
                                                       gatt_intf_operation_t operation,
                                                       wiced_bt_gatt_status_t status);


/**
 * @brief Function to write data to nvram at nvram_id
 *
 * @param nvram_id : NVRAM Identifier
 * @param bdaddr : Address of the remote Bluetooth device
 * @param p_data : Data to be written to NVRAM
 * @param len : Length to be written to NVRAM
 *
 * @return uint32_t Length written to NVRAM
 * */
typedef uint32_t (*fn_write_to_nvram)(int nvram_id, wiced_bt_device_address_t bdaddr, uint8_t *p_data, uint32_t len);

/**
 * @brief Function to read data from nvram at nvram_id
 *
 * @param nvram_id : NVRAM Identifier
 * @param bdaddr : Address of the remote Bluetooth device
 * @param p_data : Data pointer to read data from NVRAM
 * @param len : Length of buffer pointed by \p p_data
 *
 * @return uint32_t Actual length read from NVRAM
 * */
typedef uint32_t (*fn_read_from_nvram)(int nvram_id, wiced_bt_device_address_t bdaddr, uint8_t *p_data, uint32_t len);

/**
 * @brief Function to delete data from nvram at nvram_id
 *
 * @param nvram_id : NVRAM Identifier
 * @param bdaddr : Address of the remote Bluetooth device
 *
 * @return void
 * */
typedef void (*fn_delete_nvram_id)(int nvram_id, wiced_bt_device_address_t bdaddr);

/**
 * @brief Structure to describe the NVRAM functions
 */
typedef struct
{
    uint16_t max_services;                           /**< Max services for the device */
    fn_read_from_nvram pfn_read_from_nvram;          /**< function to read from nvram at nvram_id */
    fn_write_to_nvram pfn_write_to_nvram;            /**< function to write to nvram at nvram_id */
    fn_delete_nvram_id pfn_delete_nvram_id;          /**< function to delete nvram id */
    fn_on_service_handle_check_to_save pfn_can_save; /**< function to confirm saving the profile handle */
    fn_on_service_handle_store pfn_store;            /**< function to store the allocated profile handle */
} gatt_interface_nvram_data_t;

#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_GA_GATT_INTERFACE_H__ */
