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

#ifndef __UNICAST_SINK_H
#define __UNICAST_SINK_H

#include "unicast_sink_gatt.h"

/******************************************************************************
 * Function Name: unicast_sink_handle_adv
 ******************************************************************************
 * Summary: sink start or stop advertisment
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_adv(uint8_t *p_data, uint8_t payload_len);

/******************************************************************************
 * Function Name: unicast_sink_handle_play
 ******************************************************************************
 * Summary: play audio from source
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_play(uint8_t *p_data, uint8_t payload_len);

/******************************************************************************
 * Function Name: unicast_sink_handle_pause
 ******************************************************************************
 * Summary: pause audio from source
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_pause(uint8_t *p_data, uint8_t payload_len);

/******************************************************************************
 * Function Name: unicast_sink_handle_vol_up
 ******************************************************************************
 * Summary: volume up
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_vol_up(uint8_t *p_data, uint8_t payload_len);

/******************************************************************************
 * Function Name: unicast_sink_handle_vol_down
 ******************************************************************************
 * Summary: volume down
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_vol_down(uint8_t *p_data, uint8_t payload_len);

/******************************************************************************
 * Function Name: unicast_sink_handle_unmute_vol_down
 ******************************************************************************
 * Summary: unmute and volume down
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_unmute_vol_down(uint8_t *p_data, uint8_t payload_len);

/******************************************************************************
 * Function Name: unicast_sink_handle_unmute_vol_up
 ******************************************************************************
 * Summary:
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_unmute_vol_up(uint8_t *p_data, uint8_t payload_len);

/******************************************************************************
 * Function Name: unicast_sink_handle_abs_vol
 ******************************************************************************
 * Summary: handle set absolute volume
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_abs_vol(uint8_t *p_data, uint8_t payload_len);

/******************************************************************************
 * Function Name: unicast_sink_handle_unmute
 ******************************************************************************
 * Summary: mute sink device
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_mute(uint8_t *p_data, uint8_t payload_len);

/******************************************************************************
 * Function Name: unicast_sink_handle_unmute
 ******************************************************************************
 * Summary: unmute sink device
 *
 * Parameters:
 *  uint8_t *p_data
 *  uint8_t data_len
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_handle_unmute(uint8_t *p_data, uint8_t payload_len);

/******************************************************************************
 * Function Name: unicast_sink_show_connected_device
 ******************************************************************************
 * Summary: show current connected device
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_show_connected_device( void );

/******************************************************************************
 * Function Name: unicast_sink_get_connected_device_by_handle
 ******************************************************************************
 * Summary: get the current connected device by handle (index)
 *
 * Parameters:
 *  uint8_t idx
 *
 * Return:
 *  unicast_sink_clcb_t* 
 *
******************************************************************************/
unicast_sink_clcb_t* unicast_sink_get_connected_device_by_handle( uint8_t idx );

/******************************************************************************
 * Function Name: unicast_sink_get_connected_device_by_connid
 ******************************************************************************
 * Summary: get the current connected device by conn id
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  unicast_sink_clcb_t*
 *
******************************************************************************/
void unicast_sink_show_connected_device( void );

/******************************************************************************
 * Function Name: unicast_sink_is_streaming
 ******************************************************************************
 * Summary: start streaming by sink
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  BOOL32
 *
******************************************************************************/
BOOL32 unicast_sink_is_streaming( uint16_t conn_id );

#endif
