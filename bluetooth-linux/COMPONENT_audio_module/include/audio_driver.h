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
*   wiced_ble_isoc_data_path_bit_t: dir: setup SOC_DPD_bit
*   uint8_t num_of_channels: setting audio channel number (1 or 2)
*   uint32_t sample_rate: setting audio sample rate 
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_init(wiced_ble_isoc_data_path_bit_t dir, uint8_t num_of_channels, uint32_t sample_rate, uint16_t required_latency_ms);

/******************************************************************************
* Function Name: audio_driver_deinit
*******************************************************************************
* Summary:
*   deinit audio driver
*
* Parameters:
*   int8_t direction: wiced_ble_isoc_data_path_bit_t
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
* Function Name: audio_driver_write_mic_data
*******************************************************************************
* Summary:
*   to capture mic data and write to file
*
* Parameters:
*   uint8_t *p_data: mic channel data
*   uint8_t bit_width_in_bytes:  bit_width
*   uint32_t data_size:  data size
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_write_mic_data(uint8_t *p_data,
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
*    wiced_ble_isoc_data_path_bit_t dir: isoc direction
*
*    uint32_t sample_rate: smaple rate of wav file
*                          use this parameter to select test wav file
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_load_wave_file(wiced_ble_isoc_data_path_bit_t dir, uint32_t sample_rate);

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
int wiced_get_audio_data_form_mic(uint8_t *l_data);

/******************************************************************************
* Function Name: wiced_get_audio_data_form_mic_noninterleave
*******************************************************************************
* Summary:
*   get non-interleave audio data from mic
*
* Parameters:
*   uint8_t **data: audio data from mic channel
*
* Return:
*   int: data size
*
****************************************************************************/
int wiced_get_audio_data_form_mic_noninterleave(uint8_t **data);

/******************************************************************************
* Function Name: audio_driver_set_mic_gain
*******************************************************************************
* Summary:
*   get gain of mic
*
* Parameters:
*   int32_t gain: gain of mic
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_set_mic_gain(int32_t gain);

/******************************************************************************
* Function Name: audio_driver_set_mic_mute_state
*******************************************************************************
* Summary:
*   set gain of mute state
*
* Parameters:
*   uint8_t mute: mute state
*   int32_t gain: gain of mic
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_set_mic_mute_state(uint8_t mute, int32_t gain);

/******************************************************************************
* Function Name: audio_driver_init_dump_file_by_conn_hdl
*******************************************************************************
* Summary:
*   Initialize the dump file with connection handle
*
* Parameters:
*   uint32_t sample_rate: the audio sample rate
*   uint16_t conn_hdl: connection handle
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_init_dump_file_by_conn_hdl(uint32_t sample_rate, uint16_t conn_hdl);

/******************************************************************************
* Function Name: audio_driver_dump_file_by_conn_hdl
*******************************************************************************
* Summary:
*   Write audio data to file by connection handle
*
* Parameters:
*   uint16_t conn_hdl: connection handle
*   uint8_t *p_data: audio data
*   uint32_t size: size of data
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_dump_file_by_conn_hdl(uint16_t conn_hdl, uint8_t *p_data, uint32_t size);

/******************************************************************************
* Function Name: audio_driver_init_pcm
*******************************************************************************
* Summary:
*   Initialize the pcm data path
*
* Parameters:
*   wiced_ble_isoc_data_path_bit_t dir: the direction of pcm data path
*   uint8_t num_of_channels: number of channels
*   uint32_t sample_rate: the audio sample rate
*   uint16_t required_latency_ms: the required audio latency for alsa driver
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_init_pcm(wiced_ble_isoc_data_path_bit_t dir, uint8_t num_of_channels, uint32_t sample_rate, uint16_t required_latency_ms);

/******************************************************************************
* Function Name: audio_driver_pcm_deinit
*******************************************************************************
* Summary:
*   De-initialize the pcm data path
*
* Parameters:
*   uint8_t direction: the direction of pcm data path
*
* Return:
*   void: none
*
****************************************************************************/
void audio_driver_pcm_deinit(uint8_t direction);
