#ifndef AUDIO_PARSE_WAVE_H
#define AUDIO_PARSE_WAVE_H
#include "wiced_data_types.h"

int audio_module_get_wave_data(uint8_t *l_data, uint8_t *r_data, uint16_t frame_duration);
uint8_t audio_module_load_wave_file(char *file_name);
#endif
