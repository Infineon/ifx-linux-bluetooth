/***************************************************************************//**
* \file <wiced_bt_spp.h>
*
* \brief
* 	Contains SPP (Serial Port Profile) APIs and definitions.
*
*//*****************************************************************************
* Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#pragma once

#ifdef __cplusplus
extern "C"
{
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
 * @typedef wiced_bt_spp_connection_up_callback_t
 * @brief Connection up callback function type.
 * @details The application must implement connection up callback to be called by the library.
 *          The callback indicates to the application that SPP over RFCOMM session with a
 *          specified device has been successfully established. This can be a result of the
 *          wiced_bt_spp_connect requested by the application, or due to a peer device
 *          establishing connection from its side.
 * @param[in] handle   Handle that identifies newly established session.
 * @param[in] bd_addr  Bluetooth Device address of the connected device.
 * @return None.
 */
typedef void (*wiced_bt_spp_connection_up_callback_t)(uint16_t handle, uint8_t* bd_addr);

/**
 * @typedef wiced_bt_spp_connection_failed_callback_t
 * @brief Connection failed callback function type.
 * @details Connection failed callback indicates to the application that the library
 *          failed to establish RFCOMM connection with the peer device after connection
 *          was requested by the wiced_bt_spp_connect call.
 * @return None.
 */
typedef void (*wiced_bt_spp_connection_failed_callback_t)(void);

/**
 * @typedef wiced_bt_spp_service_not_found_callback_t
 * @brief Service not found callback function type.
 * @details Service not found callback indicates wiced_bt_spp_connect call requested to establish
 *          connection to a device which is currently not present, or which is not running
 *          SPP service.
 * @return None.
 */
typedef void (*wiced_bt_spp_service_not_found_callback_t)(void);

/**
 * @typedef wiced_bt_spp_connection_down_callback_t
 * @brief Connection down callback function type.
 * @details Connection Down callback indicates that an active session has been terminated.
 * @param[in] handle Handle that identifies the external accessory session.
 * @return None.
 */
typedef void (*wiced_bt_spp_connection_down_callback_t)(uint16_t handle);

/**
 * @typedef wiced_bt_spp_rx_data_callback_t
 * @brief Rx Data callback function type.
 * @details Rx Data callback passed to the application data received over the SPP session.
 * @param[in] handle   Handle that identifies the external accessory session.
 * @param[in] data     Pointer to buffer with data.
 * @param[in] data_len Length of the data.
 * @return wiced_bool_t indicating if data was processed successfully.
 */
typedef wiced_bool_t (*wiced_bt_spp_rx_data_callback_t)(uint16_t handle, uint8_t* data, uint32_t data_len);

/**
 * @brief Registration structure for SPP application.
 * @details Following structure is used to register application with wiced_bt_spp library.
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
 * @brief Initialize SPP library and starts the RFCOMM service.
 * @param[in] p_reg Registration control block that includes RFCOMM SCN and callbacks.
 * @return wiced_result_t Result of operation.
 */
wiced_result_t wiced_bt_spp_startup(wiced_bt_spp_reg_t *p_reg);

/**
 * @brief Establish External Accessory connection to an iOS device.
 * @details Library will perform Service Discovery. If SPP service is running on the specified device RFCOMM
 *          connection is established. When session is established, library executes
 *          spp_connection_up_callback.
 * @param[in] bd_addr Bluetooth Device address to connect to.
 * @return wiced_result_t Result of operation.
 */
wiced_result_t wiced_bt_spp_connect(wiced_bt_device_address_t bd_addr);

/**
 * @brief Disconnect External Accessory Session with the iOS device.
 * @details Bluetooth connection is brought down as well.
 * @param[in] handle The handle returned by the application in the wiced_bt_spp_connection_up_callback.
 * @return wiced_result_t Result of operation.
 */
wiced_result_t wiced_bt_spp_disconnect(uint16_t handle);

/**
 * @brief Send data over the established External Accessory connection.
 * @details The session must be SPP_EA_SESSION_ID. The first 2 octets of the p_data must be the handle
 *          passed to the application in the wiced_bt_spp_connection_up_callback in the big
 *          endian format.
 * @param[in] handle Connection handle indicated in the connection up callback.
 * @param[in] p_data Pointer to buffer with data to send.
 * @param[in] len    Length of the data + handle.
 * @return wiced_bool_t WICED_TRUE if data is scheduled for transmission, otherwise WICED_FALSE.
 */
wiced_bool_t wiced_bt_spp_send_session_data(uint16_t handle, uint8_t *p_data, uint32_t len);

/**
 * @brief SPP application may use this call to disable or reenable the RX data flow.
 * @param[in] handle Connection handle indicated in the connection up callback.
 * @param[in] enable If true, data flow is enabled.
 * @return None.
 */
void wiced_bt_spp_rx_flow_enable(uint16_t handle, wiced_bool_t enable);

/**
 * @brief Check if library can queue more data for transmission.
 * @param[in] handle The handle returned by the application in the wiced_bt_spp_connection_up_callback.
 * @return wiced_bool_t TRUE if library can queue forward data, or FALSE if forward data queue is full.
 */
wiced_bool_t wiced_bt_spp_can_send_more_data(uint16_t handle);

/**
 * @brief SPP application may use this function to discard all the data from the output or input queues.
 * @param[in] handle      Connection handle indicated in the connection up callback.
 * @param[in] purge_flags Specify the action to take with PORT_PURGE_TXCLEAR and/or PORT_PURGE_RXCLEAR.
 * @return uint8_t RFCOMM port return code.
 */
uint8_t wiced_bt_spp_port_purge(uint16_t handle, uint8_t purge_flags);

/**
 * @brief SPP application may use this function to get the connection state when rfcomm connection made.
 * @param[in] bd_addr Bluetooth device address to check connection state for.
 * @return uint8_t Connection state.
 */
uint8_t wiced_bt_spp_get_connection_state(wiced_bt_device_address_t bd_addr);

/**@} wiced_bt_spp_api_functions */

#ifdef __cplusplus
}
#endif
