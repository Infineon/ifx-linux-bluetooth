/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __WICED_BT_GA_MICP_H__
#define __WICED_BT_GA_MICP_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_aics.h"

#define MICP_TRACE(...)
#define MICP_TRACE_CRIT(...)

    /* Maximum Instance of AICS service supported by client */
#define WICED_BT_MAX_AICS_INSTANCE          2

/* microphone State data */
typedef struct
{
    wiced_bt_ga_mute_val_t  mute_state;                        /**< current mute state value of the peer*/
} wiced_bt_ga_microphone_state_data_t;

/* microphone Control client status Data */
typedef union
{
    wiced_bt_ga_mute_val_t mute_state;                        /**< changed microphone setting value */
    uint8_t error_status;                                  /**< error opcode in case of error event */
    wiced_bt_ga_aics_data_t aics_data;          /**< AICS Status data */
} wiced_bt_ga_microphone_control_client_status_data_t;

extern const gatt_intf_service_methods_t mics_methods;

#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_GA_MICP_H__ */
