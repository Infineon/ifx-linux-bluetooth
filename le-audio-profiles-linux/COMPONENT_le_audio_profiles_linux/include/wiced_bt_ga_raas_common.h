/*
 * $ Copyright Cypress Semiconductor $
 */


 /**************************************************************************//**
  * \file <wiced_bt_ga_raas_common.h>
  *
  * Definitions for common structures for the server and the profile
  *
  */

#ifndef __WICED_BT_GA_RAAS_COMMON_H__
#define __WICED_BT_GA_RAAS_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_result.h"
#include "wiced_bt_ga_common.h"

    /** RAAS maximum num of audio route end points (AREP) available */
#define WICED_BT_GA_RAAS_MAXIMUM_AREP              7
    /** RAAS maximum length of audio route end point's (AREP) friendly name */
#define WICED_BT_GA_RAAS_MAX_FRIENDLY_NAME_LEN     30
    /** RAAS maximum number of output arep for a given route id*/
#define WICED_BT_GA_RAAS_MAX_OUTPUT_AREP           3
    /** RAAS maximum number of input arep for a given route id */
#define WICED_BT_GA_RAAS_MAX_INPUT_AREP            3
    /** RAAS maximum number of ccid for a given route id */
#define WICED_BT_GA_RAAS_MAX_CCID                  10

    /** RAAS maximum num of audio route end points (AREP) available */
#define WICED_BT_GA_RAAS_YOU              0x01
    /** RAAS maximum num of audio route end points (AREP) available */
#define WICED_BT_GA_RAAS_ME               0x02

    /** All possible value for breadcasting state */
typedef enum
{
    WICED_BT_GA_RAAS_STOP_BROADCASTING,
    WICED_BT_GA_RAAS_START_BROADCASTING,
    WICED_BT_GA_RAAS_NO_CHANGE_IN_CURR_BROADCAST_STATE,
} wiced_bt_ga_raas_broadcast_state_t;

/** Selectable audio route end points at the vicinity of the server */
typedef struct
{
    uint8_t             arep_identifier;                                          /**< Value to uniquly identify arep */
    uint8_t             features;                                                 /**< Arep features bit 0 -> o/p , bit 1 ->i/p , bit 2 ->broadcast */
    char                friendly_name[WICED_BT_GA_RAAS_MAX_FRIENDLY_NAME_LEN];    /**< Friendly name of arep */
} wiced_bt_ga_raas_selectable_arep_t;

/** Configured audio routes for the given route ID */
typedef struct
{
    uint8_t                                 route_id;                                               /**< route id Value */
    uint8_t                                 out_arep_num;                                           /**< Number of output arep for the given route ID */
    uint8_t                                 out_arep_id[WICED_BT_GA_RAAS_MAX_OUTPUT_AREP];          /**< Output arep identifiers list */
    uint8_t                                 in_arep_num;                                            /**< Number of input arep for the given route ID */
    uint8_t                                 in_arep_id[WICED_BT_GA_RAAS_MAX_INPUT_AREP];            /**< Output arep identifiers list */
    wiced_bt_ga_raas_broadcast_state_t      broadcast;                                              /**< Server broadcast : 0 -stop, 1- start, 2 -Do not change current setting */
} wiced_bt_ga_raas_cfg_audio_route_t;

/** Configured audio route content type values for the given route ID */
typedef struct
{
    uint8_t        route_id;                           /**< route id Value*/
    uint16_t       content_type;                       /**< Bit mask of Content Types that are associated with the Route ID.  */
    uint8_t        ccid_num;                           /**< Number of CCID (can be 0)   */
    uint8_t        ccid[WICED_BT_GA_RAAS_MAX_CCID];    /**< CCID list */
} wiced_bt_ga_raas_config_content_type_t;

typedef struct
{
    uint8_t                                arep_count;                                        /**< Number of selectable AREP in the list */
    wiced_bt_ga_raas_selectable_arep_t     selectable_arep[WICED_BT_GA_RAAS_MAXIMUM_AREP];    /**< List of Selecatble Audio Route Endpoints (arep) */
} wiced_bt_ga_raas_selectable_arep_data_t;

typedef struct
{
    uint8_t                               audio_route_count;                                 /**< Number of configured audio route in the list */
    wiced_bt_ga_raas_cfg_audio_route_t    cfg_audio_route[WICED_BT_GA_RAAS_MAXIMUM_AREP];    /**< List of configured audio route */
} wiced_bt_ga_raas_cfg_audio_route_data_t;

typedef struct
{
    uint8_t                                   content_type_count;                                 /**< Number of configured audio route content type in the list */
    wiced_bt_ga_raas_config_content_type_t    cfg_content_type[WICED_BT_GA_RAAS_MAXIMUM_AREP];    /**< list of configured audio route content type */
} wiced_bt_ga_raas_cfg_content_type_data_t;

#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_GA_RAAS_COMMON_H__ */

