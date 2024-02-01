/*
 * $ Copyright Cypress Semiconductor $
 */

#pragma once

#include "wiced_bt_isoc.h"
#include <stdio.h>

/******************************************************************************
* Function Name: audio_driver_init
*******************************************************************************
* Summary:
*   init audio driver and parameters
*
* Parameters:
*   wiced_bt_isoc_data_path_bit_t: dir: setup SOC_DPD_bit
*   uint8_t num_of_channels: setting audio channel number (1 or 2)
*   uint32_t sample_rate: setting audio sample rate 
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_init(wiced_bt_isoc_data_path_bit_t dir, uint8_t num_of_channels, uint32_t sample_rate, uint16_t required_latency_ms);

/******************************************************************************
* Function Name: audio_driver_deinit
*******************************************************************************
* Summary:
*   deinit audio driver
*
* Parameters:
*   int8_t direction: wiced_bt_isoc_data_path_bit_t
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_deinit(uint8_t direction);

/******************************************************************************
* Function Name: audio_driver_write_non_interleaved_data
*******************************************************************************
* Summary:
*   write non interleaved data to audio driver
*
* Parameters:
*   uint8_t *p_left_data:  left channel data
*   uint8_t *p_right_data: right channel data
*   uint8_t bit_width_in_bytes:  bit_width
*   uint32_t data_size:  data size
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_write_non_interleaved_data(uint8_t *p_left_data,
                                             uint8_t *p_right_data,
                                             uint8_t bit_width_in_bytes,
                                             uint32_t data_size);

/******************************************************************************
* Function Name: audio_driver_set_volume
*******************************************************************************
* Summary:
*   setting volume 
*
* Parameters:
*   uint8_t volume:  0~100 percentage
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_set_volume(uint8_t volume);

/******************************************************************************
* Function Name: audio_driver_set_mute_state
*******************************************************************************
* Summary:
*   setting volume mute or not
*
* Parameters:
*   uint8_t mute_enabled: 0 muted, others: not muted
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_set_mute_state(uint8_t mute_enabled);

/******************************************************************************
* Function Name: audio_driver_load_wave_file
*******************************************************************************
* Summary:
*   load wave file
*
* Parameters:
*    wiced_bt_isoc_data_path_bit_t dir: isoc direction
*
*    uint32_t sample_rate: smaple rate of wav file
*                          use this parameter to select test wav file
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_load_wave_file(wiced_bt_isoc_data_path_bit_t dir, uint32_t sample_rate);

/******************************************************************************
* Function Name: audio_driver_mic_init
*******************************************************************************
* Summary:
*   interface of alsa api to init microphone
*
* Parameters:
*   uint8_t num_of_channels
*
*   uint32_t sample_rate
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_mic_init(uint8_t num_of_channels, uint32_t sample_rate);

/******************************************************************************
* Function Name: wiced_get_audio_data_form_mic
*******************************************************************************
* Summary:
*   get audio data from mic
*
* Parameters:
*   uint8_t *l_data: data from left channel
*   uint8_t *r_data: data from right cnannel
*
* Return:
*   int: data size
*
****************************************************************************/
int wiced_get_audio_data_form_mic(uint8_t *l_data, uint8_t *r_data);
