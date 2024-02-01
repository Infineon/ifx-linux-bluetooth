/*
 * $ Copyright Cypress Semiconductor $
 */

/** @file
 *
 * Volume Offset Control Service (VOCS) Application Programming Interface
 */

#pragma once 

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "gatt_interface.h"
#include "wiced_bt_ga_common.h"

#define VOCS_TRACE(...)
#define VOCS_TRACE_CRIT(...)
/**
 * @addtogroup Volume_Control_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_vocs
 * @{
 */
 
/** Volume Offset Control Service opcodes */
typedef enum {
    WICED_BT_GA_VOCS_OPCODE_SET_VOLUME_OFFSET = 1 /**< Set Volume Offset*/
} wiced_bt_ga_vocs_opcode_t;

/** Volume offset control point data */
typedef struct {
    wiced_bt_ga_vocs_opcode_t opcode; /**< VOCS opcode */
    int16_t volume_offset;            /**< VOCS offset value */
} vocs_opcode_t;

/** Volume offset data which is passed between application and profile */
typedef union {
    int16_t volume_offset;             /**< VOCS offset value */
    uint32_t audio_location;           /**< Audio location value */
    wiced_bt_ga_string_t description;  /**< VOCS offset value */
    vocs_opcode_t control_point;       /**< VOCS control point data */
}wiced_bt_ga_vocs_data_t;

/** Volume offset related methods defined in the init file*/
extern const gatt_intf_service_methods_t vocs_methods;

/**
 * @brief VOCS API to notify characteristic data
 *
 * @param[in] conn_id : GATT Connection ID
 * @param[in] p_service : VOCS service object
 * @param[in] p_char : characteristic to be notified
 * @param[in] p_n : characteristic data to be notified 
 */
wiced_bt_gatt_status_t vocs_notify(uint16_t conn_id, gatt_intf_service_object_t* p_service, 
    gatt_intf_attribute_t *p_char, void *p_n);

/**
 * @brief VOCS API to write characteristic data
 *
 * @param[in] conn_id : GATT Connection ID
 * @param[in] p_service : VOCS service object
 * @param[in] p_char : characteristic to be written
 * @param[in] p_n : characteristic data to be written 
 */
wiced_bt_gatt_status_t vocs_write_remote_attribute(uint16_t conn_id, gatt_intf_service_object_t* p_service,
    gatt_intf_attribute_t *p_char, void* p_n);
	
	
/**@} wiced_bt_ga_vocs */
/**@} Volume_Control_APIs */
