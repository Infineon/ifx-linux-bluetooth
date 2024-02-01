/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company)
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
 */

/** @file
 *
 * This file provides definitions of the SPP implementation
 */

#ifndef __SPP_API_H
#define __SPP_API_H

#ifdef __cplusplus
extern "C" {
#endif

// MAX packet size that can be sent over SPP
#define SPP_MAX_PACKET_SIZE    1013
#define PORT_PURGE_TXCLEAR     0x01
#define PORT_PURGE_RXCLEAR     0x02

/**
 * @addtogroup  wiced_bt_spp_api_functions        SPP Library API
 * @ingroup     wicedbt
 *
 * SPP library of the AIROC BTSDK provide a simple method for an application to integrate SPP
 * functionality.  Application just needs to call API to connect/disconnect and send data.
 * Library in turn indicates status of the connection and passes to the application received data.
 *
 * @{
 */

/*****************************************************************************
 *          Function Prototypes
 *****************************************************************************/

/**
 * The application must implement connection up callback to be called by the library.
 * The callback indicates to the application that SPP over RFCOMM session with a
 * specified device has been successfully established.  This can be a result of the
 * wiced_bt_spp_connect requested by the application, or due to a peer device
 * establishing connection from its side.
 *
 * @param[in]       handle   : Handle that identifies newly established session.
 * @param[in]       bd_addr  : Bluetooth Device address of the connected device.
 * @return          none
 */
typedef void (*wiced_bt_spp_connection_up_callback_t)(uint16_t handle, uint8_t* bd_addr);

/**
 * Connection failed callback indicates to the application that the library
 * failed to establish RFCOMM connection with the peer device after connection
 * was requested by the wiced_bt_spp_connect call.
 *
 * @return          none
 */
typedef void (*wiced_bt_spp_connection_failed_callback_t)(void);

/**
 * Service not found callback indicates wiced_bt_spp_connect call requested to establish
 * connection to a device which is currently not present, or which is not running
 * SPP service.
 *
 * @return          none
 */
typedef void (*wiced_bt_spp_service_not_found_callback_t)(void);

/**
 * Connection Down callback indicates that an active session has been terminated.
 *
 * @param[in]       handle   : Handle that identifies the external accessory session
 * @return          none
 */
typedef void (*wiced_bt_spp_connection_down_callback_t)(uint16_t handle);

/**
 * Rx Data callback passed to the application data received over the SPP
 * session.
 *
 * @param[in]       handle   : Handle that identifies the external accessory session
 * @param[in]       p_data  : Pointer to buffer with data.
 * @param[in]       data_len :     Length of the data
 * @return          none
 */
typedef wiced_bool_t (*wiced_bt_spp_rx_data_callback_t)(uint16_t handle, uint8_t* data, uint32_t data_len);

/**
 * Following structure is used to register application with wiced_bt_spp library
 */
typedef struct
{
    uint8_t                                    rfcomm_scn;                     /**< Application selects RFCOMM SCN that
                                                                                     it publishes in the SDP and need to
                                                                                     pass the same value for library to use. */
    uint16_t                                   rfcomm_mtu;                     /**< MTU to be be used by the RFCOMM layer */
    wiced_bt_spp_connection_up_callback_t      p_connection_up_callback;       /**< iAP2 connection established */
    wiced_bt_spp_connection_failed_callback_t  p_connection_failed_callback;   /**< iAP2 connection establishment failed */
    wiced_bt_spp_service_not_found_callback_t  p_service_not_found_callback;   /**< iAP2 service not found */
    wiced_bt_spp_connection_down_callback_t    p_connection_down_callback;     /**< iAP2 connection disconnected */
    wiced_bt_spp_rx_data_callback_t            p_rx_data_callback;             /**< Data packet received */

} wiced_bt_spp_reg_t;

/**
 * Function         wiced_bt_spp_startup
 *
 * Initialize SPP library and starts the RFCOMM service.
 *
 * @param[in]      p_reg  : Registration control block that includes RFCOMM SCN and callbacks
 * @return          wiced_result_t : Result of operation
 */
wiced_result_t wiced_bt_spp_startup(wiced_bt_spp_reg_t *p_reg);

/**
 * Function         wiced_bt_spp_connect
 *
 * Establish External Accessory connection to an iOS device.  Library will perform
 * Service Discovery.  If SPP service is running on the specified device RFCOMM
 * connection is established.  When session is established, library executes
 * spp_connection_up_callback.
 *
 * @param[in]      bd_addr  : Bluetooth Device address to connect to.
 * @return          wiced_result_t : Result of operation
 */
wiced_result_t wiced_bt_spp_connect(wiced_bt_device_address_t bd_addr);

/**
 * Function         wiced_bt_spp_disconnect
 *
 * Disconnect External Accessory Session with the iOS device.  Bluetooth connection
 * is brought down as well.
 *
 * @param[in]      handle  : The handle returned by the application in the wiced_bt_spp_connection_up_callback.
 * @return          wiced_result_t : Result of operation
 */
wiced_result_t wiced_bt_spp_disconnect(uint16_t handle);

/**
 * Send data over the established External Accessory connection.  The session must
 * be SPP_EA_SESSION_ID.  The first 2 octets of the p_data must be the handle
 * passed to the application in the wiced_bt_spp_connection_up_callback in the big
 * endian format.
 *
 * @param[in]      handle : Connection handle indicated in the connection up callback
 * @param[in]      p_data  : Pointer to buffer with data to send.
 * @param[in]      len :     Length of the data + handle
 * @return          WICED_TRUE: if data is scheduled for transmission, otherwise WICED_FALSE
 */
wiced_bool_t wiced_bt_spp_send_session_data(uint16_t handle, uint8_t *p_data, uint32_t len);

/**
 * SPP application may use this call to disable or reenable the RX data flow
 *
 * @param[in]      handle : Connection handle indicated in the connection up callback
 * @param[in]      enable : If true, data flow is enabled
 * @return          none
 */

void wiced_bt_spp_rx_flow_enable(uint16_t handle, wiced_bool_t enable);

/**
 * Function         wiced_bt_spp_can_send_more_data
 *
 * @param[in]      handle  : The handle returned by the application in the wiced_bt_spp_connection_up_callback.
 * Returns TRUE if library can queue forward data, or FALSE if forward data queue
 * is full.
 */
wiced_bool_t wiced_bt_spp_can_send_more_data(uint16_t handle);

/**
 * SPP application may use this function to discard all the data from the
 * output or input queues of the specified connection.
 *
 * @param[in]      handle : Connection handle indicated in the connection up callback
 * @param[in]      purge_flags - specify the action to take with PORT_PURGE_TXCLEAR
 *                 and/or PORT_PURGE_RXCLEAR.
 * @return         rfcomm port return code
 */
uint8_t wiced_bt_spp_port_purge(uint16_t handle, uint8_t purge_flags);

/**
 * SPP application may use this function to get the connection state when rfcomm connection made
 *
 * @return         connection state
 */
uint8_t wiced_bt_spp_get_connection_state(wiced_bt_device_address_t bd_addr);

/**@} wiced_bt_spp_api_functions */

#ifdef __cplusplus
}
#endif

#endif
