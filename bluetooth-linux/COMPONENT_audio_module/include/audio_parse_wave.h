#ifndef AUDIO_PARSE_WAVE_H
#define AUDIO_PARSE_WAVE_H
#include "wiced_data_types.h"

#define CURR_FILE_NAME_MAX_LEN  60U

/******************************************************************************
* Function Name: audio_module_get_wave_data
*******************************************************************************
* Summary:
*   parse and get wave data
*
* Parameters:
*   uint8_t *l_data:  left channel data from wave
*   uint8_t *r_data: right channel data from wave
*   uint16_t frame_duration:  set frame duration
*
* Return:
*   int: get wave data size
*
****************************************************************************/
int audio_module_get_wave_data(uint8_t *l_data, uint8_t *r_data, uint16_t frame_duration);

/******************************************************************************
* Function Name: audio_module_load_wave_file
*******************************************************************************
* Summary:
*   audio module load wave from file
*
* Parameters:
*   char *file_name: wave file path
*
* Return:
*   uint8_t: file size
*
****************************************************************************/
uint8_t audio_module_load_wave_file(char *file_name);
#endif
