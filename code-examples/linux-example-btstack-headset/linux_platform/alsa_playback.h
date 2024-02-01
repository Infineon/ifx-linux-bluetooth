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

/******************************************************************************
 * File Name: alsa_playback.h
 *
 * Description: This is the header file for Linux alsa setting, play and record.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

#ifndef __ALSA_PLAYBACK__
#define __ALSA_PLAYBACK__

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include <stdbool.h>

#include "alsa/asoundlib.h"

#include "wiced_bt_a2dp_defs.h"


/*******************************************************************************
*                           MACROS
*******************************************************************************/
#define ALSA_SUCCESS              0
#define ALSA_FAIL                -1
#define ALSA_VOLUME_HFP_RATIO     7
#define MIC_CAPTURE_SAMPLE     8000
#define MIC_CAPTURE_CHANNEL       2
#define MIC_CAPTURE_CHANNEL_NBS   1
#define ALSA_MIC_FRAMES_SIZE     60
#define HFP_NUM_BLOCKS           15
#define HFP_NUM_CHANNEL           1
#define HFP_NUM_SUBBANDS          8


typedef struct PLAYBACK_CONFIG_PARAMS_TAG
{
    int16_t samplingFreq;        /*16k, 32k, 44.1k or 48k*/
    int16_t channelMode;         /*mono, dual, streo or joint streo*/
    int16_t numOfSubBands;       /*4 or 8*/
    int16_t numOfChannels;
    int16_t numOfBlocks;         /*4, 8, 12 or 16*/
    int16_t allocationMethod;    /*loudness or SNR*/
    int16_t bitPool;
} PLAYBACK_CONFIG_PARAMS;

/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/

/*****************************************************************************
* Function Name: init_alsa_sink
******************************************************************************
* Summary: Init alsa pcm for sink
*          
*     
*
* Parameters: wiced_bt_a2dp_codec_info_t* codec_info:
*               codec related setting, please check wiced_bt_a2dp_defs.h
*   
* 
* Return:  int:
*               0 on success otherwise a negative error code    
*   
*
****************************************************************************/
int init_alsa_sink(wiced_bt_a2dp_codec_info_t *codec_info);

/*****************************************************************************
* Function Name: deinit_alsa_sink
******************************************************************************
* Summary: deinit alsa pcm for sink
*          
*     
*
* Parameters: None
*   
* 
* Return:  None  
*   
*
****************************************************************************/
void deinit_alsa_sink();


/*****************************************************************************
* Function Name: alsa_volume_init
******************************************************************************
* Summary: init alsa volume from amixer
*          
* 
*
* Parameters: None
*   
* 
* Return:  bool: true:success  false: fail  
*   
*
****************************************************************************/
bool alsa_volume_init(void);

/*****************************************************************************
* Function Name: alsa_set_volume
******************************************************************************
* Summary: alsa set volume 
*           
*
* Parameters: uint8_t volume:  range from 0 ~ 100
*   
* 
* Return:  None
*   
*
****************************************************************************/
void alsa_set_volume(uint8_t volume);

/*****************************************************************************
* Function Name: a2dp_sink_data_cback
******************************************************************************
* Summary: a2dp sink data cback function, get the data from bt stack and send 
*          to alsa pcm  
*           
*
* Parameters: uint8_t* p_rx_media:  a2dp data from cback function 
*             uint32_t media_len: a2dp data length from cback function  
*   
* 
* Return:  none: 
*   
*
****************************************************************************/
void a2dp_sink_data_cback(uint8_t* p_rx_media, uint32_t media_len);


/*****************************************************************************
* Function Name: init_alsa_hfp
******************************************************************************
* Summary: init alsa pcm for handsfree playback  
*           
*
* Parameters: PLAYBACK_CONFIG_PARAMS pb_config_params:
*                   alsa playback config parameters  
*   
* 
* Return:  int:
*               0 on success otherwise a negative error code 
*   
*
****************************************************************************/
int init_alsa_hfp(uint16_t set_sample_rate);

/*****************************************************************************
* Function Name: deinit_alsa_hfp
******************************************************************************
* Summary: deinit alsa pcm for handsfree playback
*          
*     
*
* Parameters: None
*   
* 
* Return:  None  
*   
*
****************************************************************************/
void deinit_alsa_hfp();

/*****************************************************************************
* Function Name: alsa_write_hfp_data
******************************************************************************
* Summary: write HFP RX SCO data into ALSA playback buffer
*
*
* Parameters: uint8_t *p_rx_media: RX_SCO data array
*             uint32_t data_size:  RX_SCO data size
*   
* 
* Return:  snd_pcm_sframes_t: the size of writing into ALSA
*   
*
****************************************************************************/
snd_pcm_sframes_t alsa_write_hfp_data(uint8_t *p_rx_media, uint32_t data_size);

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
uint32_t alsa_capture_mic_data(uint8_t *mic_data, snd_pcm_sframes_t _frames);

/*****************************************************************************
* Function Name: a2dp_sink_queue_init
******************************************************************************
* Summary: Init sink decode queue  
*          
*     
* Parameters: None
*   
* 
* Return:  None
*   
*
****************************************************************************/
void a2dp_sink_queue_init();

/*****************************************************************************
* Function Name: a2dp_sink_dequeue
******************************************************************************
* Summary: a2dp sink decode queue dequeue 
*          
*     
* Parameters: None
*   
* 
* Return:  bool: 
*             True: success dequeue
*             False: queue is empty or mutex lock
*   
*
****************************************************************************/
bool a2dp_sink_dequeue();

/*****************************************************************************
* Function Name: clearQueue
******************************************************************************
* Summary: clear A2DP playback queue
*          
*     
* Parameters: None
*   
* 
* Return:  void
*   
*
****************************************************************************/
void clearQueue();

/******************************************************************************
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
bool start_alsa_capture(void);

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
bool stop_alsa_capture(void);

#endif /*__LINUX_PLATFORM__ */

/* [] END OF FILE */
