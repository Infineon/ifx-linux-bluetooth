/*
 * $ Copyright Cypress Semiconductor $
 */

/** @file
 *
 * Call Control Profile (CCP) Application Programming Interface
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_tbs_common.h"

#define CCP_TRACE(...) WICED_BT_TRACE(__VA_ARGS__)
#define CCP_TRACE_CRIT(...) WICED_BT_TRACE(__VA_ARGS__)

/**
 * @addtogroup Telephone_Bearer_Service_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_tbs
 * @{
 */
 /** list of bearer properties that is read from the app */
typedef enum
{
    TBS_BEARER_PROVIDER_NAME,                         /**< telephone bearer provider name value */
    TBS_BEARER_UCI,                                   /**< telephone bearer supported uci */
    TBS_BEARER_TECHNOLOGY,                            /**< telephone bearer supported technology */
    BEARER_URI_PREFIX_SUPPORTED_LIST,                 /**< telephone bearer supported uri*/
    TBS_BEARER_SIGNAL_STRENGTH,                       /**< telephone bearer signal strength*/
    TBS_BEARER_SIGNAL_STRENGTH_REPORTING_INTERVAL,    /**< telephone bearer signal strength reporting interval*/
} wiced_bt_ga_ccp_bearer_property_t;
/**@} wiced_bt_ga_tbs */
/**@} Telephone_Bearer_Service_APIs */
#ifdef __cplusplus
}
#endif

