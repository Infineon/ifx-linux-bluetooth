/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __WICED_BT_GA_MICS_H__
#define __WICED_BT_GA_MICS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "gatt_interface.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_aics.h"

#define MICS_TRACE(...)
#define MICS_TRACE_CRIT(...)

typedef struct {
    uint8_t index;                           /**< index of the included service viz, vocs or aics */
    union {
        wiced_bt_ga_aics_data_t* p_aics;        /**< aics data */
    };
} mics_included_t;

typedef union {
    wiced_bt_ga_mute_val_t           mute_val;        /**< mute state */
    mics_included_t                  mics_included;   /**< volume included service data */
} wiced_bt_ga_mics_data_t;

/**
* Initialize the MICS service/profile
*/

wiced_result_t wiced_bt_ga_mics_init(ga_cfg_t *p_cfg);

wiced_bt_gatt_status_t mics_notify(uint16_t conn_id, gatt_intf_service_object_t* p_service,
    gatt_intf_attribute_t *p_char, void *p_data);

wiced_bt_gatt_status_t mics_read_remote_attribute(uint16_t conn_id, gatt_intf_service_object_t* p_service,
    gatt_intf_attribute_t *p_char);

wiced_bt_gatt_status_t mics_enable_notifications(uint16_t conn_id, gatt_intf_service_object_t* p_service,
    gatt_intf_attribute_t *p_char, uint16_t value);

wiced_bt_gatt_status_t mics_write_remote_attribute(uint16_t conn_id, gatt_intf_service_object_t* p_service,
                                                 gatt_intf_attribute_t *p_char,
                                                 void *p_data);

#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_GA_MICS_H__ */
