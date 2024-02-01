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

/*******************************************************************************
*      INCLUDES
*******************************************************************************/
#include "alsa_audio_capture.h"

#include <stdio.h>
#include <stdlib.h>

#include "log.h"


/*******************************************************************************
*       MACROS
********************************************************************************/
#define S16_BYTE_SIZE           2
#define ALSA_BUFFER_RATIO       4
#define BITS_BYTE_RATIO         8 

/*******************************************************************************
*       Variable
********************************************************************************/
static char *alsa_capture_device = "default";

static alsa_config_t  ac_config;
static alsa_capture_t *ac_device;

/*****************************************************************************
* Function Name: start_alsa_capture
*******************************************************************************
* Summary:
*   start to capture audio from alsa 
*
* Parameters:
*   void: none
*
* Return:
*   bool : true: successful, false: failed
*
****************************************************************************/
bool start_alsa_capture(void){
    bool ret;
    TRACE_LOG(" Start");
    memset(&ac_config, 0, sizeof(ac_config));
    ac_device = alsa_capture_open(alsa_capture_device);
    if (!ac_device){
        TRACE_ERR("alsa_capture_open fail");
        return false;
    }
    
    ac_config.sample_rate = MIC_CAPTURE_SAMPLE;
    ac_config.channels = MIC_CAPTURE_CHANNEL;
    ac_config.format = ALSA_INT16;

    ret = alsa_capture_config_set(ac_device, &ac_config);
    if (ret != true){
        TRACE_ERR("alsa_capture_config_set fail");
        return false;
    }
    alsa_capture_start(ac_device);
    return true;
}

/******************************************************************************
* Function Name: stop_alsa_capture
*******************************************************************************
* Summary:
*   Stop to capture audio from alsa 
*
* Parameters:
*   void: none
*
* Return:
*   bool : true: successful, false: failed
*
****************************************************************************/
bool stop_alsa_capture(void){
    TRACE_LOG(" Stop");
    return alsa_capture_close(&ac_device);
}

/*****************************************************************************
* Function Name: alsa_capture_mic_data
******************************************************************************
* Summary: Alsa capture mic data from pcm 
*          
*     
*
* Parameters: 
*           uint8_t *mic_data: data from alsa capture pcm (output)
*           snd_pcm_sframes_t _frames: capture frame size
*   
* 
* Return:  uint32_t: 
*               the real data size get from alsa capture pcm    
*   
*
****************************************************************************/
uint32_t alsa_capture_mic_data(uint8_t *mic_data, snd_pcm_sframes_t _frames)
{
    int rc = 0;
    rc = alsa_capture_pcm_read(ac_device, mic_data, _frames);
    if (rc < 0) {
        TRACE_ERR("alsa_capture_pcm_read error %d\n", rc);
        return 0;
    }
    else{
        return rc;
    }
}

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
alsa_capture_t *alsa_capture_open(char *device) {
  int ret;
  snd_pcm_t *pcm;
  alsa_capture_t * alsa_capture = NULL;

  alsa_capture = calloc(1, sizeof(*alsa_capture));
  if (!alsa_capture){
    return NULL;
  }
  ret = snd_pcm_open(&pcm, device, SND_PCM_STREAM_CAPTURE, 0);
  if (ret != 0) {
    free(alsa_capture);
    TRACE_ERR("error opening device: %s", snd_strerror(ret));
    return NULL;
  }
  alsa_capture->pcm = pcm;
  alsa_capture->hw_params = calloc(1, snd_pcm_hw_params_sizeof());
  if (!alsa_capture->hw_params){
    free(alsa_capture);
    TRACE_ERR("error init hw_params");
    return NULL;
  }
  ret = snd_pcm_hw_params_any(pcm, alsa_capture->hw_params);
  if (ret < 0) {
    free(alsa_capture);
    TRACE_ERR("%s", snd_strerror(ret));
    return NULL;
  }
  ret = snd_pcm_hw_params_set_access(pcm, alsa_capture->hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
  if (ret < 0) {
    free(alsa_capture);
    TRACE_ERR("%s", snd_strerror(ret));
    return NULL;
  }
  return alsa_capture;
}

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
bool alsa_capture_config_set(alsa_capture_t *capture, alsa_config_t *config) {
  int ret;
  ret = snd_pcm_hw_params_set_format(capture->pcm, capture->hw_params, (snd_pcm_format_t)config->format);
  if (ret < 0) {
    TRACE_ERR("%s", snd_strerror(ret));
    return false;
  }
  ret = snd_pcm_hw_params_set_rate_near(capture->pcm, capture->hw_params, &config->sample_rate, 0);
  if (ret < 0) {
    TRACE_ERR("%s", snd_strerror(ret));
    return false;
  }
  ret = snd_pcm_hw_params_set_channels(capture->pcm, capture->hw_params, config->channels);
  if (ret < 0) {
    TRACE_ERR("%s", snd_strerror(ret));
    return false;
  }
  capture->frame_size = S16_BYTE_SIZE * config->channels;
  snd_pcm_hw_params_get_buffer_size_max(capture->hw_params, &capture->buffer_size);
  capture->buffer_size = (capture->buffer_size <= ALSA_BUFFER_SIZE_MAX) ? capture->buffer_size : ALSA_BUFFER_SIZE_MAX;
  ret = snd_pcm_hw_params_set_buffer_size_near(capture->pcm, capture->hw_params, &capture->buffer_size);
  if (ret < 0) {
    TRACE_ERR("%s", snd_strerror(ret));
    return false;
  }
  snd_pcm_hw_params_get_period_size_min(capture->hw_params, &capture->period_size, 0);
  if (!capture->period_size){
    capture->period_size = capture->buffer_size / ALSA_BUFFER_RATIO;
  }
  ret = snd_pcm_hw_params_set_period_size_near(capture->pcm, capture->hw_params, &capture->period_size, 0);
  if (ret < 0) {
    TRACE_ERR("%s", snd_strerror(ret));
    return false;
  }
  capture->config = *config;
  return true;
}

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
bool alsa_capture_config_get(alsa_capture_t *capture, alsa_config_t *config) {
  if (!capture || !config){
    return false;
  }
  *config = capture->config;
  return true;
}

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
bool alsa_capture_start(alsa_capture_t *capture) {
  int ret;
  ret = snd_pcm_hw_params(capture->pcm, capture->hw_params);
  if (ret < 0) {
    TRACE_ERR("%s", snd_strerror(ret));
    return false;
  }
  return true;
}

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
ssize_t alsa_capture_pcm_read(alsa_capture_t *capture, uint8_t *data, size_t size) {
  int ret;
  size_t nframes;
  size_t format_width;
  size_t channels;
  format_width = snd_pcm_format_width(capture->config.format) / BITS_BYTE_RATIO;
  channels = capture->config.channels;
  nframes = size / (format_width * channels);
  ret = snd_pcm_readi(capture->pcm, data, nframes);
  if (ret == -EPIPE) {
    /* overrun, try to recover */
    ret = snd_pcm_prepare(capture->pcm);
    if (ret < 0) {
      TRACE_ERR("%s", snd_strerror(ret));
      return ALSA_CAPTURE_ERROR;
    }
  } else if (ret < 0) {
    TRACE_ERR("%s", snd_strerror(ret));
    return ALSA_CAPTURE_ERROR;
  }
  return ret * (format_width * channels);
}

/******************************************************************************
* Function Name: alsa_capture_close
*******************************************************************************
* Summary:
*   alsa capture close and free related structure
*
* Parameters:
*   void: alsa_capture_t* alsa_capture:   pointer to the alsa_capture_t
*
* Return:
*   bool : true: successful, false: failed
*
****************************************************************************/
bool alsa_capture_close(alsa_capture_t** alsa_capture) {
  int ret;
  if ((*alsa_capture) == NULL){
    return false;
  }
  if ((*alsa_capture)->pcm) {
    ret = snd_pcm_close((*alsa_capture)->pcm);
    if (ret != 0) {
      TRACE_ERR("error closing device: %s", snd_strerror(ret));
      return false;
    }
    (*alsa_capture)->pcm = NULL;
  }
  if ((*alsa_capture)->hw_params) {
    free((*alsa_capture)->hw_params);
    (*alsa_capture)->hw_params = NULL;
  }
  free((*alsa_capture));
  (*alsa_capture) = NULL;
  return true;
}
