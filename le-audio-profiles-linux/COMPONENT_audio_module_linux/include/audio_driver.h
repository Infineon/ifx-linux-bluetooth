/*
 * $ Copyright Cypress Semiconductor $
 */

#pragma once

#include "wiced_bt_isoc.h"
#include <stdio.h>

void audio_driver_init(wiced_bt_isoc_data_path_bit_t dir, uint8_t num_of_channels, uint32_t sample_rate);

void audio_driver_deinit(uint8_t direction);

void audio_driver_write_non_interleaved_data(uint8_t *p_left_data,
                                             uint8_t *p_right_data,
                                             uint8_t bit_width_in_bytes,
                                             uint32_t data_size);

void audio_driver_set_volume(uint8_t volume);
void audio_driver_set_mute_state(uint8_t mute_enabled);
void audio_driver_load_wave_file(wiced_bt_isoc_data_path_bit_t dir, uint32_t sample_rate);
void audio_driver_mic_init(uint8_t num_of_channels, uint32_t sample_rate);
