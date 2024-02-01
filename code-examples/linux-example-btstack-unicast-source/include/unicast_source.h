/*
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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
*/

#ifndef __UNICAST_SOURCE_H
#define __UNICAST_SOURCE_H

#define UNICAST_SINK_SCAN_DEVICE_MAX    20U
#define UNICAST_SINK_CONNECT_DEVICE_MAX  1U

#include "data_types.h"
#include "unicast_source_gatt.h"

/******************************************************************************
* Function Name: unicast_source_handle_scan
*******************************************************************************
* Summary: start or stop scan Sink Device
*
* Parameters:
*   uint8_t *p_data:
*       one byte for start scan or stop scan
*       1: start
*       0: stop
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_scan(uint8_t *p_data);

/******************************************************************************
* Function Name: unicast_source_handle_connect
*******************************************************************************
* Summary: connect to Sink Device by bda
*
* Parameters:
*   uint8_t *p_data:
*       addr_type
*       bd_addr
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_connect(uint8_t *p_data, uint32_t data_len, uint8_t buffersize);

/******************************************************************************
* Function Name: unicast_source_handle_disconnect
*******************************************************************************
* Summary: disconnect to the Sink Device by bda
*
* Parameters:
*   uint8_t *p_data:
*       addr_type
*       bd_addr
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
extern void unicast_source_handle_disconnect(uint8_t *p_data, uint32_t data_len, uint8_t buffersize);

/******************************************************************************
* Function Name: unicast_source_handle_play
*******************************************************************************
* Summary: Source device start play audio by conn_id
*
* Parameters:
*   uint8_t *p_data:
*       conn_id: the current connection id of the Sink device
*       codec_config: BAP support codec config enum
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_play(uint8_t *p_data, uint32_t data_len, uint8_t buffersize);

/******************************************************************************
* Function Name: unicast_source_handle_pause
*******************************************************************************
* Summary: pause the current play audio stream
*
* Parameters:
*   uint8_t *p_data:
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_pause(uint8_t *p_data, uint32_t data_len, uint8_t buffersize);

/******************************************************************************
* Function Name: unicast_source_handle_vol_up
*******************************************************************************
* Summary: volme up
*
* Parameters:
*   uint8_t *p_data:
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_vol_up(uint8_t *p_data, uint32_t data_len, uint8_t buffersize);

/******************************************************************************
* Function Name: unicast_source_handle_vol_down
*******************************************************************************
* Summary: volume down
*
* Parameters:
*   uint8_t *p_data:    
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_vol_down(uint8_t *p_data, uint32_t data_len, uint8_t buffersize);

/******************************************************************************
* Function Name: unicast_source_handle_mute
*******************************************************************************
* Summary: mute audio streaming
*
* Parameters:
*   uint8_t *p_data
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_mute(uint8_t *p_data, uint32_t payload_len, uint8_t buffersize);

/******************************************************************************
* Function Name: unicast_source_handle_unmute
*******************************************************************************
* Summary: unumte audio streaming
*
* Parameters:
*   uint8_t *p_data:
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_unmute(uint8_t *p_data, uint32_t payload_len, uint8_t buffersize);

/******************************************************************************
* Function Name: unicast_source_handle_unmute_vol_down
*******************************************************************************
* Summary: unmute and volume down
*
* Parameters:
*   uint8_t *p_data:
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_unmute_vol_down(uint8_t *p_data, uint32_t data_len, uint8_t buffersize);

/******************************************************************************
* Function Name: unicast_source_handle_unmute_vol_up
*******************************************************************************
* Summary: unmute and volume up
*
* Parameters:
*   uint8_t *p_data:
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_unmute_vol_up(uint8_t *p_data, uint32_t data_len, uint8_t buffersize);

/******************************************************************************
* Function Name: unicast_source_handle_abs_vol
*******************************************************************************
* Summary: set abs vol
*
* Parameters:
*   uint8_t *p_data:
*       conn_id: the current connection id of the Sink device
*       volume of the abs value 0-255
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_abs_vol(uint8_t *p_data, uint32_t data_len, uint8_t buffersize);


/******************************************************************************
* Function Name: unicast_source_add_sink_dev
*******************************************************************************
* Summary: add found sink device to loacl array when scan, the index of the array
*          is the handle of device, need reset the array when start scan
*
* Parameters:
*   wiced_bt_ble_scan_results_t *p_scan_result
*
* Return:
*      None
*
******************************************************************************/
extern void unicast_source_add_sink_dev(wiced_bt_ble_scan_results_t *p_scan_result);

/******************************************************************************
* Function Name: unicast_source_clear_sink_dev
*******************************************************************************
* Summary: clear the saving sink device
*
* Parameters:
*   None
*
* Return:
*   None
*
******************************************************************************/
extern void unicast_source_clear_sink_dev( void );

/******************************************************************************
* Function Name: unicast_source_show_sink_dev
*******************************************************************************
* Summary: show current saving sink device
*
* Parameters:
*   None
*
* Return:
*   uint8_t: number of Unicast Sink device found
*
******************************************************************************/
extern uint8_t unicast_source_show_sink_dev( void );

/******************************************************************************
* Function Name: unicast_source_get_scan_result
*******************************************************************************
* Summary: get the saving sink device info
*
* Parameters:
*   uint8_t idx: handle of the sink device
*
* Return:
*   None
*
******************************************************************************/
extern wiced_bt_ble_scan_results_t* unicast_source_get_scan_result(uint8_t idx);

/******************************************************************************
* Function Name: unicast_source_show_connected_device
*******************************************************************************
* Summary: shoe current connected sink device
*
* Parameters:
*   None
*
* Return:
*   None
*
******************************************************************************/
void unicast_source_show_connected_device( void );

/******************************************************************************
* Function Name: unicast_source_get_connected_device_num
*******************************************************************************
* Summary: get the number of connected sink device
*
* Parameters:
*   None
*
* Return:
*   None
*
******************************************************************************/
uint8_t unicast_source_get_connected_device_num( void );

/******************************************************************************
* Function Name: unicast_source_get_connected_device_by_handle
*******************************************************************************
* Summary: get the connected sink device by handle
*
* Parameters:
*   uint8_t idx: handle of the connected device
*
* Return:
*      None
*
******************************************************************************/
unicast_source_clcb_t* unicast_source_get_connected_device_by_handle( uint8_t idx );

/******************************************************************************
* Function Name: unicast_source_get_audio_mute_status
*******************************************************************************
* Summary: get the status of mute
*
* Parameters:
*   None
*
* Return:
*   BOOL32:
*       true
*       false
*
******************************************************************************/
BOOL32 unicast_source_get_audio_mute_status( void );



/******************************************************************************
* Function Name: unicast_source_handle_unmute_vol_down
*******************************************************************************
* Summary: unmute and volume down
*
* Parameters:
*   uint8_t *p_data:
*       conn_id: the current connection id of the Sink device
*
*   uint32_t data_len
*   uint8_t buffersize
*
* Return:
*      None
*
******************************************************************************/
void unicast_source_handle_unmute_vol_down(uint8_t *p_data, uint32_t data_len, uint8_t buffersize);

#endif
