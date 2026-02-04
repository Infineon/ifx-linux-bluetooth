/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __WICED_BT_GA_IAS_H__
#define __WICED_BT_GA_IAS_H__

#include "gatt_interface.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_gatt.h"

/** IAS alert level enumeration */
enum wiced_bt_ga_ias_alert_level_e
{
    WICED_BT_GA_IAS_NO_ALERT, /**< No alert */
    WICED_BT_GA_IAS_MILD_ALERT, /**< Mild alert */
    WICED_BT_GA_IAS_HIGH_ALERT /**< High alert */
};
typedef uint8_t wiced_bt_ga_ias_alert_level_t; /**< see \ref wiced_bt_ga_ias_alert_level_e */


/** IAS data */
typedef struct
{
    wiced_bt_ga_ias_alert_level_t alert_level;
}wiced_bt_ga_ias_data_t;

/**
 * @brief Initialize the IAS service_type/profile
 *
 * @param[in] num_instances: Number of IAS instances to be created.
 * @param[in] pv_ini : expected to be NULL
 */
wiced_result_t wiced_bt_ga_ias_init(uint8_t num_instances, void *pv_ini);

/** Enable IAS server module */
void wiced_bt_ga_ias_enable_server(void);

/** Enable IAS client module */
void wiced_bt_ga_ias_enable_client(void);

#endif // __WICED_BT_GA_IAS_H__
