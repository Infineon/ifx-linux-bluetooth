#ifndef WICED_SINE_WAVE_H
#define WICED_SINE_WAVE_H

#include <stdint.h>
/******************************************************************************
* Function Name: wiced_load_sine_wave_data
*******************************************************************************
* Summary:
*   audio module load sine wave
*
* Parameters:
*   uint32_t sampling_frequency: sampling frequency
*
* Return:
*   void: none
*
****************************************************************************/
void wiced_load_sine_wave_data(uint32_t sampling_frequency);

/******************************************************************************
* Function Name: wiced_get_sine_wave_data
*******************************************************************************
* Summary:  audio get sine wave data

*
* Parameters:
*   uint8_t *l_data: get left channl data
*   uint8_t *r_data: get right channel data 
*   uint16_t frame_duration: frame_duration 
*
* Return:
*   uint8_t: file size
*
****************************************************************************/
int wiced_get_sine_wave_data(uint8_t *l_data, uint8_t *r_data, uint16_t frame_duration);
#endif
