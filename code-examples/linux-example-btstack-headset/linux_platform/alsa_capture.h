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

#ifndef _ALSA_CAPTURE_H_
#define _ALSA_CAPTURE_H_

/*******************************************************************************
*      INCLUDES
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <alsa/asoundlib.h>

/*******************************************************************************
*       MACROS
********************************************************************************/
#define ALSA_BUFFER_SIZE_MAX 65536
#define ALSA_CAPTURE_ERROR      -1
/*******************************************************************************
*       STRUCTURES AND ENUMERATIONS
*******************************************************************************/
typedef enum {
  ALSA_INT16 = SND_PCM_FORMAT_S16_LE,
  ALSA_FLOAT = SND_PCM_FORMAT_FLOAT_LE
} alsa_format_t;

typedef struct {
  alsa_format_t format;
  uint sample_rate;
  uint channels;
} alsa_config_t;

typedef struct {
  snd_pcm_t *pcm;
  snd_pcm_hw_params_t *hw_params;
  alsa_config_t config;
  snd_pcm_uframes_t period_size;
  snd_pcm_uframes_t buffer_size;
  size_t frame_size;
}alsa_capture_t;


/******************************************************************************
* Function Name: alsa_capture_open
*******************************************************************************
* Summary:
*   init alsa capture setting
*
* Parameters:
*   char *device: alsa capture card name
*
* Return:
*   alsa_capture_t * : pointer to the alsa_capture_t
*
****************************************************************************/
alsa_capture_t *alsa_capture_open(char *device);

/******************************************************************************
* Function Name: alsa_capture_config_set
*******************************************************************************
* Summary:
*   alsa capture configure setting
*
* Parameters:
*   alsa_capture_t *capture: pointer to the alsa_capture_t 
*   alsa_config_t *config: pointer to the alsa_config_t
* Return:
*   bool : true: successful, false: failed
*
****************************************************************************/
bool alsa_capture_config_set(alsa_capture_t *capture, alsa_config_t *config);


/******************************************************************************
* Function Name: alsa_capture_config_get
*******************************************************************************
* Summary:
*   get alsa capture configure
*
* Parameters:
*   alsa_capture_t *capture: pointer to the alsa_capture_t 
*   alsa_config_t *config: output, the pointer to the alsa_config_t
* Return:
*   bool : true: successful, false: failed
*
****************************************************************************/
bool alsa_capture_config_get(alsa_capture_t *capture, alsa_config_t *config);

/******************************************************************************
* Function Name: alsa_capture_start
*******************************************************************************
* Summary:
*    start alsa capture
*
* Parameters:
*   alsa_capture_t *capture: pointer to the alsa_capture_t 
* Return:
*   bool : true: successful, false: failed
*
****************************************************************************/
bool alsa_capture_start(alsa_capture_t *capture);

/******************************************************************************
* Function Name: alsa_capture_pcm_read
*******************************************************************************
* Summary:
*    read capture data from alsa
*
* Parameters:
*   alsa_capture_t *capture: pointer to the alsa_capture_t
*   uint8_t *data: reference to the output data buffer
*   size_t size: reference to the output data buffer size
* Return:
*   ssize_t : positive value: the capture size from mic data, 0 or negative value: error
*
****************************************************************************/
ssize_t alsa_capture_pcm_read(alsa_capture_t *capture, uint8_t *data, size_t size);

/******************************************************************************
* Function Name: alsa_capture_close
*******************************************************************************
* Summary:
*   alsa capture close and free related structure
*
* Parameters:
*   alsa_capture_t* alsa_capture:   pointer to the alsa_capture_t
*
* Return:
*   bool : true: successful, false: failed
*
****************************************************************************/
bool alsa_capture_close(alsa_capture_t** alsa_capture);

#endif

