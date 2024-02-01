/***************************************************************************//**
* \file <wiced_bt_a2dp_source.h>
*
* \brief
* 	Contains A2DP Source APIs and definitions.
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
#include "wiced_memory.h"


/**
 * @defgroup  wicedbt_a2dp_src        Advanced Audio Distribution Profile (A2DP) Source
 *
 * This section describes  Advanced Audio Distribution Profile Source interfaces.
 * @addtogroup wicedbt_a2dp_src

 * @ingroup wicedbt_av
 * @{
 */


/******************************************************************************
*                    Constants
******************************************************************************/


/******************************************************************************
*                   Enumerations
******************************************************************************/

/* Events in wiced_bt_a2dp_source_control_cb_t() callback,
   for payload see wiced_bt_a2dp_source_event_data_t */
typedef enum
{
    WICED_BT_A2DP_SOURCE_CONNECT_EVT,     /* Connected event, received on establishing connection to a peer device */
    WICED_BT_A2DP_SOURCE_DISCONNECT_EVT,  /* Disconnected event, received on disconnection from a peer device */
    WICED_BT_A2DP_SOURCE_START_IND_EVT,   /* Start stream indication event, received when start req is received */
    WICED_BT_A2DP_SOURCE_START_CFM_EVT,   /* Start stream confirm event, received when start req is sent and response is received */
    WICED_BT_A2DP_SOURCE_SUSPEND_EVT,     /* Suspend stream event, received when audio streaming is suspended */
    WICED_BT_A2DP_SOURCE_CONFIGURE_EVT,   /* Configure stream event, received when streaming configuration completed*/
    WICED_BT_A2DP_SOURCE_WRITE_CFM_EVT,   /* Write confirm event, received when AVDT has completed writing the given buffer*/
    WICED_BT_A2DP_SOURCE_DELAY_RPT_EVT,   /* Delay report command received from SINK */
} wiced_bt_a2dp_source_event_t;


/* A2DP Source features masks */
typedef enum
{
    WICED_BT_A2DP_SOURCE_FEAT_PROTECT   = 0x0001, /* Streaming media content protection */
    WICED_BT_A2DP_SOURCE_FEAT_DELAY_RPT = 0x0002, /* Use delay reporting */
} wiced_bt_a2dp_source_feature_mask_t;


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
} wiced_bt_a2dp_source_codec_info_list_t;

/* A2DP source configuration data structure */
typedef struct
{
    wiced_bt_a2dp_source_feature_mask_t         feature_mask;       /* Supported features */
    wiced_bt_a2dp_source_codec_info_list_t           codec_capabilities;   /* List of supported codecs and their capabilities */
    wiced_bt_a2dp_codec_info_t                default_codec_config;
} wiced_bt_a2dp_source_config_data_t;

/* Generic event status info */
typedef struct
{
    wiced_result_t            result;       /* Whether the event indicates failure or success, WICED_BT_XXX */
    wiced_bt_device_address_t bd_addr;      /* Peer bluetooth device address */
    uint16_t                  handle;       /* Peer connection handle */
    uint16_t                  lcid;         /* Local identifier */
    uint8_t                   is_accepter;  /* True if device is accepter, otherwise false */
} wiced_bt_a2dp_source_status_t;

/* A2DP Start event data structure */
typedef struct
{
    wiced_result_t            result;  /* Whether the event indicates failure or success, WICED_BT_XXX */
    uint8_t                   label;   /* Transaction label */
    uint16_t                  handle;  /* Peer connection handle */
} wiced_bt_a2dp_source_start_t;

/* A2DP Set config event data structure */
typedef struct
{
    wiced_result_t result; /* Whether the event indicates failure or success, WICED_BT_XXX */
    uint16_t handle;       /* Peer connection handle */
    uint16_t cp_type;      /* Content protection type */
    wiced_bt_a2dp_codec_info_t *codec_config;
    uint16_t stream_mtu; /* MTU of stream */
} wiced_bt_a2dp_source_config_t;


/* Control callback event data */
typedef union
{
    wiced_bt_a2dp_source_status_t         connect;        /* WICED_BT_A2DP_SOURCE_CONNECT_EVT payload */
    wiced_bt_a2dp_source_status_t         disconnect;     /* WICED_BT_A2DP_SOURCE_DISCONNECT_EVT payload */
    wiced_bt_a2dp_source_start_t          start_ind;      /* WICED_BT_A2DP_SOURCE_START_IND_EVT payload */
    wiced_bt_a2dp_source_status_t         start_cfm;      /* WICED_BT_A2DP_SOURCE_START_CFM_EVT payload */
    wiced_bt_a2dp_source_status_t         suspend;        /* WICED_BT_A2DP_SOURCE_SUSPEND_EVT payload */
    wiced_bt_a2dp_source_config_t         set_config;
    uint16_t                              delay_ms;       /* Delay reported from DELAY_RPT_EVT from SINK */
} wiced_bt_a2dp_source_event_data_t;

/******************************************************************************
*                 Callback Type Definitions
******************************************************************************/

/******************************************************************************
*
* \name The A2DP Control path callback type.
*
* \brief The application implements a callback of this type to receive A2DP control path events.
*
*  \param event    ID of event being notified to app.
*  \param p_data   The pointer to data associated with the event.
*
*  \return None.
*
******************************************************************************/
typedef void (*wiced_bt_a2dp_source_control_cb_t)( wiced_bt_a2dp_source_event_t event,
    wiced_bt_a2dp_source_event_data_t* p_data );

/******************************************************************************
*               Function Declarations
******************************************************************************/

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_init
*
* \brief API to initialize the A2DP SOURCE component and register with the stack.
*
* \details Called by the application before any other API is called.
*          The application provides source configuration data and control and data callbacks
*          to receive control events and data packets, respectively.
*
*  \param p_config_data    A2DP source configuration parameters.
*                          This should remain valid until deinit is called
*                          as the pointer is stored and used inside the library.
*  \param control_cb       Callback function for receiving source events.
*  \param heap             wiced_bt_heap_t pointer to be used in A2dp source profile library
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_init( wiced_bt_a2dp_source_config_data_t* p_config_data,
    wiced_bt_a2dp_source_control_cb_t control_cb, wiced_bt_heap_t* heap);


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_deinit
*
* \brief The API to deregister from the stack and to clean up the memory of A2DP source component.
*        Called by the application when A2DP source component is no longer needed by it.
*
*  \param channel  Media type to be handled by the source.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_deinit(void);


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_connect
*
* \brief The API to connect to a peer device.
*        Called by the app to establish A2DP connection with a peer device.
*
*  \param bd_address    The Bluetooth device address of the device to which connection is requested.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_connect( wiced_bt_device_address_t bd_address );


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_disconnect
*
* \brief The API to disconnect the connection from a connected peer device.
*        Called by the application to disconnect from a connected A2DP source.
*
*  \param handle    Connection handle corresponding to the peer device to disconnect from.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_disconnect( uint16_t handle );


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_start
*
* \brief The API to start streaming.
*        Called by the application when it wants to indicate the peer to start streaming.
*
*  \param handle        Connection handle corresponding to peer device
*                       to create a streaming connection.
*  \param codec_info    Codec configuration structure pointer.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_start( uint16_t handle, wiced_bt_a2dp_codec_info_t *codec_info );

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_send_start_response
*
* \brief The API to send a start response on receiving a start request from the peer.
*        Called by the application when it wants to indicate the peer that it is ready to start streaming.
*
*  \param handle        Connection handle corresponding to peer device
*  \param label         Transaction label
*  \param status        Indicates if a start request is accepted(AVDT_SUCCESS) or rejected(AVDT Error codes)
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_send_start_response( uint16_t handle, uint8_t label, uint8_t status );


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_suspend
*
* \brief The API to suspend streaming.
*        Called by the application when the streaming is to be suspended.
*
*  \param handle        Connection handle corresponding to peer device
*                       for which streaming is suspended.
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_suspend( uint16_t handle );

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_source_start_streaming
*
* \brief The API to send media data .
*        Called by the application after streaming is started.
*
*  \param handle        Connection handle corresponding to peer device
*                       for which streaming is suspended.
*
*  \param p_media_buf        Pointer to the media buffer to write
*
*  \param buf_len        Size of the buffer
*
*  \param time_stamp        Time stamp
*
  \param m_pt        Marker and payload byte
*
*
*  \return wiced_result_t (WICED_BT_XXX)
*
******************************************************************************/
wiced_result_t wiced_bt_a2dp_source_start_streaming(uint8_t handle, uint8_t *p_media_buf, uint16_t buf_len, uint32_t time_stamp, uint8_t m_pt);


/* @} */
/* end of wicedbt_a2dp_src */
#ifdef __cplusplus
} /* extern "C" */
#endif /* _WICED_BT_A2DP_SOURCE_H_ */
