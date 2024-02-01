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

#define TMAS_TRACE(...)
#define TMAS_TRACE_CRIT(...)

typedef enum
{
    TMAP_ROLE_CALL_GATEWAY                  =  1,          /**< Call gateway */
    TMAP_ROLE_CALL_TERMINAL                 = (1 << 1),   /**< Call  terminal*/
    TMAP_ROLE_UNICAST_MEDIA_SENDER          = (1 << 2),   /**< Unicast media sender */
    TMAP_ROLE_UNICAST_MEDIA_RECEIVER        = (1 << 3),   /**< Unicast media receiver */
    TMAP_ROLE_BROADCAST_MEDIA_SENDER        = (1 << 4),   /**< Broadcast media sender */
    TMAP_ROLE_BROADCAST_MEDIA_RECEIVER      = (1 << 5),   /**< Broadcast media receiver */
} tmap_role_t;

/* Audio Input Control Service event data */
typedef union
{
    tmap_role_t tmap_role; /**< Supported TMAP roles defined in tmap_role_t */

} wiced_bt_ga_tmas_data_t;

/**
* Initialize the TMAS service_type/profile
*/
wiced_result_t wiced_bt_ga_tmas_init(ga_cfg_t *p_cfg);

#endif // __WICED_BT_GA_TMAS_H__
