/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __WICED_BT_GA_RAAP_H__
#define __WICED_BT_GA_RAAP_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_ga_raas_common.h"

#define RAAP_TRACE(...)
#define RAAP_TRACE_CRIT(...)

typedef struct wiced_bt_ga_raap_t_ wiced_bt_ga_raap_t;

/**
* @anchor ROUTING_ACTIVE_AUDIO_CLIENT_EVENT
* @name Definition for messages exchanged from Routing Active Audio service to app
* @{ */
typedef enum
{
    WICED_BT_GA_RAAP_SELECTABLE_AREP_LIST_CHANGED,                 /**< Change in selectable audio route end points */
    WICED_BT_GA_RAAP_CFG_AUDIO_ROUTE_LIST_CHANGED,                 /**< Change in configured audio route list */
    WICED_BT_GA_RAAP_CFG_AUDIO_ROUTE_CONTENT_TYPE_LIST_CHANGED,    /**< change in configured audio route content type list */
    WICED_BT_GA_RAAP_SELECATBLE_AREP_LIST_RESPONSE,                /**< Response to read selectable audio route end points */
    WICED_BT_GA_RAAP_CFG_AUDIO_ROUTE_LIST_RESPONSE,                /**< Response to read configured audio route list */
    WICED_BT_GA_RAAP_CFG_AUDIO_ROUTE_CONTENT_TYPE_LIST_RESPONSE,   /**< Respose to read configured audio route content type list */
    WICED_BT_GA_RAAP_SET_AUDIO_ROUTE_RESPONSE,                     /**< Response on Setting explicit audio outputs and audio inputs for the specified route */
    WICED_BT_GA_RAAP_MAX_EVENT,                                    /**< Maximun number of routing active audio profile events */
} wiced_bt_ga_raap_event_t;

typedef union
{
    wiced_bt_ga_raas_selectable_arep_data_t     arep_list;               /**< List of available AREP */
    wiced_bt_ga_raas_cfg_audio_route_data_t     audio_route_list;        /**< Configured audio route list for a given route ID */
    wiced_bt_ga_raas_cfg_content_type_data_t    content_type_list;       /**< Configured audio route content type list for a given route ID */
} wiced_bt_ga_raap_data_t;

/**
* \brief routing active audio client callback
* \details The routing active audio client callback is executed by the routing active audio client on receiving
* a SET message from the peer. The callback is provided by the application
*
* @param   p_raap       instance of the routing active audio client
* @param   event        the event that the application should process
* @param   p_context    connection id and handle of the received message
* @param   p_data       pointer to data recevied
*
* @return  None
*/
typedef void(wiced_bt_ga_raap_callback_t)(wiced_bt_ga_raap_t* p_raap, wiced_bt_ga_raap_event_t event, wiced_bt_ga_raap_data_t* p_data);

extern const gatt_intf_service_methods_t raas_methods;

/**
* \brief Read RAAS characteristics listed:
* \ 1.selectable audio route endpoint list
* \ 2.configured audio route list
* \ 3.configured audio route content type
*
* \details Reads the above mentioned characteristic of the routing active audio service
*
* @param   p_raap           instance of the routing active audio client
* @param   characteristic   RAAS char to be read optained from @wiced_bt_ga_raas_server_handle_list_t
* @return  wiced_bt_gatt_status_t result of the read operation
*/
wiced_bt_gatt_status_t wiced_bt_ga_raap_read_char_helper(wiced_bt_ga_raap_t* p_raap, uint8_t characteristic);

/**
* \briefmodify audio route control point characteristic
* \details set configured audio route list of the Routing Active  Service
*
* @param   p_raap             instance of the routing active audio profile.
* @param   set_audio_cfg    1.broadcast       0 - stop broadcast
*                                             1 - start broadcast
*                                             2 - Do not modify the cuurect broadcast state.
*                           2.out_arep_id/in_arep_id
*                             YOU(1) - Transfer input /or& output to server.
*                             ME (2) - Transfer input /or& output to profile.
*                             3-254 - server assigned i/o values
* @return  wiced_bt_gatt_status_t result of the read operation
*/
wiced_bt_gatt_status_t wiced_bt_ga_raap_modify_audio_route_control_point(wiced_bt_ga_raap_t* p_raap, wiced_bt_ga_raas_cfg_audio_route_t* set_audio_cfg);

#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_GA_RAAP_H__ */
