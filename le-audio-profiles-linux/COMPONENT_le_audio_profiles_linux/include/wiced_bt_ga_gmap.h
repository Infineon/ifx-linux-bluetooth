/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __WICED_BT_GA_GMAP_H__
#define __WICED_BT_GA_GMAP_H__

#include "gatt_interface.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_gatt.h"

#define GMAP_ROLE_UNICAST_GAME_GATEWAY (1 << 0)     /**< Unicast Game Gateway */
#define GMAP_ROLE_UNICAST_GAME_TERMINAL (1 << 1)    /**< Unicast Game Terminal*/
#define GMAP_ROLE_BROADCAST_GAME_SENDER (1 << 4)    /**< Broadcast Game Sender */
#define GMAP_ROLE_BROADCAST_GAME_RECEIVER (1 << 5)  /**< Broadcast Game Receiver */

typedef uint8_t gmap_role_t; /**< GMAP role */

/**@brief GMAP data */
typedef union
{
    gmap_role_t gmap_role; /**< Supported GMAP roles */
    uint8_t ugg_features;
    uint8_t ugt_features;
    uint8_t bgs_feature;
    uint8_t bgr_feature;
} wiced_bt_ga_gmap_data_t;

/**
 * @brief Initialize the GMAP service_type/profile
 *
 * @param[in] num_instances: Number of GMAP instances to be created.
 * @param[in] pv_ini : expected to be NULL
 */
wiced_result_t wiced_bt_ga_gmap_init(uint8_t num_instances, void *pv_ini);

/** Enable GMAP server module */
void wiced_bt_ga_gmap_enable_server(void);

/** Enable GMAP client module */
void wiced_bt_ga_gmap_enable_client(void);

#endif // __WICED_BT_GA_GMAP_H__
