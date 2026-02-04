/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __WICED_BT_GA_TMAS_H__
#define __WICED_BT_GA_TMAS_H__

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "gatt_interface.h"
#include "wiced_bt_ga_common.h"

#define TMAP_ROLE_CALL_GATEWAY             (1 << 0) /**< Call gateway */
#define TMAP_ROLE_CALL_TERMINAL            (1 << 1) /**< Call  terminal*/
#define TMAP_ROLE_UNICAST_MEDIA_SENDER     (1 << 2) /**< Unicast media sender */
#define TMAP_ROLE_UNICAST_MEDIA_RECEIVER   (1 << 3) /**< Unicast media receiver */
#define TMAP_ROLE_BROADCAST_MEDIA_SENDER   (1 << 4) /**< Broadcast media sender */
#define TMAP_ROLE_BROADCAST_MEDIA_RECEIVER (1 << 5) /**< Broadcast media receiver */

typedef uint16_t tmap_role_t; /**< TMAP role */

/** @brief Audio Input Control Service event data */
typedef union
{
    tmap_role_t tmap_role; /**< Supported TMAP roles defined in tmap_role_t */

} wiced_bt_ga_tmap_data_t;


/**
 * @brief Initialize the TMAP service_type/profile
 *
 * @param[in] num_instances: Number of TMAP instances to be created.
 * @param[in] pv_ini : expected to be NULL
 */
wiced_result_t wiced_bt_ga_tmap_init(uint8_t num_instances, void *pv_ini);

/** Enable TMAP server module */
void wiced_bt_ga_tmap_enable_server(void);

/** Enable TMAP client module */
void wiced_bt_ga_tmap_enable_client(void);

#endif // __WICED_BT_GA_TMAS_H__
