/***************************************************************************//**
* \file <wiced_bt_a2dp_sink.h>
*
* @brief
* 	Contains A2DP Sink APIs and definitions.
*
*//*****************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company)
* SPDX-License-Identifier: Apache-2.0
* 
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*     http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_types.h"
#include "wiced_result.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"
#include "wiced_bt_a2d_m12.h"
#include "wiced_bt_a2d_m24.h"
#include "wiced_bt_a2dp_defs.h"
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
#include "wiced_bt_audio_codec.h"
#include "wiced_audio_sink.h"
#endif


/**
 * @defgroup  wicedbt_a2dp_snk        Advanced Audio Distribution Profile (A2DP) Sink
 *
 * This section describes  Advanced Audio Distribution Profile Sink interfaces.
 * @addtogroup wicedbt_a2dp_snk
 * @ingroup wicedbt_av
 * @{
 */

/******************************************************************************
*
* \addtogroup  wicedbt_a2dp        Advanced Audio Distribution Profile (A2DP) Sink
* \ingroup     wicedbt_av
*
* Advanced Audio Distribution Profile Sink interfaces
*
* @{
*
******************************************************************************/

/******************************************************************************
*                      Macros
******************************************************************************/

/******************************************************************************
*                    Constants
******************************************************************************/


/******************************************************************************
*                   Enumerations
******************************************************************************/

/* Events in wiced_bt_a2dp_sink_control_cb_t() callback,
   for payload, see wiced_bt_a2dp_sink_event_data_t */
typedef enum
{
    WICED_BT_A2DP_SINK_CONNECT_EVT,     /* Connected event, received on establishing connection to a peer device */
    WICED_BT_A2DP_SINK_DISCONNECT_EVT,  /* Disconnected event, received on disconnection from a peer device */
    WICED_BT_A2DP_SINK_START_IND_EVT,   /* Start stream indication event, received when start req is received */
    WICED_BT_A2DP_SINK_START_CFM_EVT,   /* Start stream confirm event, received when start req is sent and response is received */
    WICED_BT_A2DP_SINK_SUSPEND_EVT,     /* Suspend stream event, received when audio streaming is suspended */
    WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT /* Codec config event, received when codec config for a streaming session is updated */
} wiced_bt_a2dp_sink_event_t;


/* A2DP Sink features masks */
typedef enum
{
    WICED_BT_A2DP_SINK_FEAT_PROTECT   = 0x0001, /* Streaming media content protection */
    WICED_BT_A2DP_SINK_FEAT_DELAY_RPT = 0x0002, /* Use delay reporting */
} wiced_bt_a2dp_sink_feature_mask_t;

/******************************************************************************
*                 Type Definitions
******************************************************************************/

/******************************************************************************
*                    Structures
******************************************************************************/

/* Codec capability information list structure,
   used to indicate the supported codecs and their capabilities */
typedef struct
{
    uint8_t                     count; /* Number of codecs present in the list */
    wiced_bt_a2dp_codec_info_t* info;  /* Codec information list */
} wiced_bt_a2dp_codec_info_list_t;

/* A2DP sink configuration data structure *
   NOTE The ext_codec field will be ignored for 20706A2. */
typedef struct
{
    wiced_bt_a2dp_sink_feature_mask_t         feature_mask;       /* Supported features */
    wiced_bt_a2dp_codec_info_list_t           codec_capabilities; /* List of supported codecs and their capabilities */
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
    wiced_bt_a2dp_sink_audio_tuning_params_t  p_param;            /* Audio tuning parameters */
    wiced_bt_a2dp_ext_codec_info_t            ext_codec;          /* External Codec information */
#else
    uint32_t                                sink_delay;
#endif
} wiced_bt_a2dp_config_data_t;

/* Generic event status info */
typedef struct
{
    wiced_result_t            result;  /* Whether the event indicates failure or success, WICED_BT_XXX */
    wiced_bt_device_address_t bd_addr; /* Peer bluetooth device address */
    uint16_t                  handle;  /* Peer connection handle */
} wiced_bt_a2dp_sink_status_t;

typedef struct
{
    wiced_result_t              result;  /* Whether the event indicates failure or success, WICED_BT_XXX */
    uint8_t                     label;   /* Transaction label */
    uint16_t                    handle;  /* Peer connection handle */
    wiced_bt_device_address_t   bdaddr;  /* Source device's BT address */
} wiced_bt_a2dp_sink_start_t;

typedef struct
{
    wiced_bt_device_address_t   bd_addr; /* Peer bluetooth device address */
    uint16_t                    handle;  /* Peer connection handle */
    wiced_bt_a2dp_codec_info_t  codec;   /* Configured codec params */
    uint16_t                    cp_type; /* Content Protection Type */
} wiced_bt_a2dp_sink_codec_config_t;

/* Control callback event data */
typedef union
{
    wiced_bt_a2dp_sink_status_t         connect;        /* WICED_BT_A2DP_SINK_CONNECT_EVT payload */
    wiced_bt_a2dp_sink_status_t         disconnect;     /* WICED_BT_A2DP_SINK_DISCONNECT_EVT payload */
    wiced_bt_a2dp_sink_start_t          start_ind;      /* WICED_BT_A2DP_SINK_START_IND_EVT payload */
    wiced_bt_a2dp_sink_status_t         start_cfm;      /* WICED_BT_A2DP_SINK_START_CFM_EVT payload */
    wiced_bt_a2dp_sink_status_t         suspend;        /* WICED_BT_A2DP_SINK_SUSPEND_EVT payload */
    wiced_bt_a2dp_sink_codec_config_t   codec_config;   /* WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT payload */
} wiced_bt_a2dp_sink_event_data_t;

/******************************************************************************
*                 Callback Type Definitions
******************************************************************************/

/**
 * Function Name: The A2DP Control path callback type.
 *
 *  The application implements a callback of this type to receive A2DP control path events.
 *
 * @param[out]       event   : ID of an event being notified to the app.
 * @param[out]       p_data  : The pointer to data associated with the event.
 *
 * @return          None.
 */

typedef void (*wiced_bt_a2dp_sink_control_cb_t)( wiced_bt_a2dp_sink_event_t event,
    wiced_bt_a2dp_sink_event_data_t* p_data );

/******************************************************************************
*               Function Declarations
******************************************************************************/

/**
 * Function Name: wiced_bt_a2dp_sink_init
 *  The API to initialize the A2DP SINK component and register with the stack.
 * Called by the application before any other API is called.
 *
 * @param[in]       p_config_data   : A2DP sink configuration parameters.
 *                          This should remain valid until deinit is called
 *                          because the pointer is stored and used inside the library.
 * @param[in]       control_cb  : Callback function for receiving sink events..
 *
 * @return          wiced_result_t (WICED_BT_XXX)
 */

wiced_result_t wiced_bt_a2dp_sink_init( wiced_bt_a2dp_config_data_t* p_config_data,
    wiced_bt_a2dp_sink_control_cb_t control_cb );


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_sink_deinit
*
* @brief The API to deregister from the stack and to clean up the memory of the A2DP sink component.
*        Called by the application when the A2DP sink component is no longer needed by it.
*
*  \param channel  Media type to be handled by the sink.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_sink_deinit(void);

/******************************************************************************
*
* Function Name: set_preferred_codec_config
*
* @brief The API to configure the preferred codec parameters
*
*  \param wiced_bt_a2dp_codec_info_t
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/

wiced_result_t wiced_bt_a2dp_set_preferred_codec_config(wiced_bt_a2dp_codec_info_t* codec_config);

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_sink_connect
*
* @brief The API to connect to a peer device.
*        Called by the app to establish an A2DP connection with a peer device.
*
*  \param bd_address    The Bluetooth device address of the device to which connection is requested.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_sink_connect( wiced_bt_device_address_t bd_address );

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_sink_disconnect
*
* @brief The API to disconnect the connection from a connected peer device.
*        Called by the application to disconnected from a connected A2DP source.
*
*  \param handle    Connection handle corresponding to the peer device to disconnect from.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_sink_disconnect( uint16_t handle );


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_sink_start
*
* @brief The API to start streaming.
*        Called by the application when it wants to indicate the peer to start streaming.
*
*  \param handle        Connection handle corresponding to the peer device.
*                       to create a streaming connection.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_sink_start( uint16_t handle );

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_sink_send_start_response
*
* @brief The API to send a start response on receiving a start request from the peer.
*        Called by the application when it wants to indicate the peer that it is ready to start streaming.
*
*  \param handle        Connection handle corresponding to peer device.
*  \param label         Transaction label.
*  \param status        Indicates if start request is accepted(AVDT_SUCCESS) or rejected(AVDT Error codes).
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_sink_send_start_response( uint16_t handle, uint8_t label, uint8_t status );


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_sink_suspend
*
* @brief The API to suspend streaming.
*        Called by the application when the streaming is to be suspended.
*
*  \param handle        Connection handle corresponding to the peer device.
*                       for which streaming is suspended.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_sink_suspend( uint16_t handle );

/** API used to register for data callback.
 *
 *  Application provides the SINK configuration data to receive data packets.
 *
 * @param p_cback          Callback function for receiving audio data.
 *
 */
void wiced_bt_a2dp_sink_register_data_cback(wiced_bt_a2dp_sink_data_cb_t p_cback);


/******************************************************************************
**
* API to send sink delay report to the peer.
*        Called by the app if it supports the sink delay report to report the
*        latency of the audio rendering path.
*
* \param handle        Connection handle corresponding to peer device
*                      to which the delay report is to be sent.
*
* \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_sink_send_delay_report(uint16_t handle);

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_sink_mute_audio
*
* @brief To mute/unmute the audio while streaming.
*        Called by the application to mute an audio when playing music.
*        The application sets this function to unmute, to restart playing music.
*        This api is supported only (in Embedded mode) when app and profile are running on an Infineon BT chip such as 20721.
*
*  \param enable    1 to mute, 0 to unmute
*
*  \param ramp_ms    ramp up/down time in milli seconds
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t  wiced_bt_a2dp_sink_mute_audio( wiced_bool_t enable, uint16_t ramp_ms );

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_sink_lrac_switch_get
*
* @brief The API to get LRAC switch data.
*        Called by the application to get LRAC switch data.
*        This api is supported only (in Embedded mode) when app and profile are running on an Infineon BT chip such as 20721.
*
*  \param p_opaque     The pointer to a buffer which to be filled with LRAC Switch data (current
*                      A2DP Sink State).
*  \param p_opaque     The size of the buffer (IN), size filled (OUT).
*
*  \return None.
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_sink_lrac_switch_get(void *p_opaque, uint16_t *p_sync_data_len);

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_sink_lrac_switch_set
*
* @brief The API to set LRAC switch data.
*        Called by the application to set LRAC Switch Data.
*        This api is supported only (in Embedded mode) when app and profile are running on an Infineon BT chip such as 20721.
*
*  \param p_opaque     The pointer to a buffer that contains LRAC switch data (new
*                      A2DP Sink State).
*  \param p_opaque     The size of the buffer (IN).
*
*  \return None.
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_sink_lrac_switch_set(void *p_opaque, uint16_t sync_data_len);


/**
 * @} wicedbt_a2dp_snk
 */
#ifdef __cplusplus
}
/* extern "C" */
#endif /* _WICED_BT_A2DP_SINK_H_ */
