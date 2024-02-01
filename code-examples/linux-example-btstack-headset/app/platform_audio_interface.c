/*
/*
* Copyright 2019-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 * File Name: platform_audio_interface.c
 *
 * Description: This is the source file for platform audio interface
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include "platform_audio_interface.h"

#include "alsa/asoundlib.h"
#include "sbc_decoder.h"
#include "sbc_dec_func_declare.h"
#include "sbc_dct.h"
#include "sbc_types.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"

#include "alsa_playback.h"
#include "log.h"
/*******************************************************************************
*                                 MACROS
*******************************************************************************/
#define MSBC_STATIC_MEM_SIZE      1920  // BYTES
#define MSBC_SCRATCH_MEM_SIZE     2048  // BYTES
#define WBS_SAMPLE_RATE          16000
#define NBS_SAMPLE_RATE           8000

#if defined(WICED_BT_HFP_HF_WBS_INCLUDED) && (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
#define SAMPLE_RATE  WBS_SAMPLE_RATE
#else
#define SAMPLE_RATE  NBS_SAMPLE_RATE
#endif
/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/


/****************************************************************************
 *                              LOCAL VARIABLES
 ***************************************************************************/
static int  PcmBytesPerFrame;
static SINT32 staticMem[MSBC_STATIC_MEM_SIZE / sizeof (SINT32)];
static SINT32 scratchMem[MSBC_SCRATCH_MEM_SIZE / sizeof (SINT32)];
static snd_mixer_elem_t* snd_mixer_elem = NULL;
static snd_mixer_t *snd_mixer_handle = NULL;
static snd_mixer_selem_id_t *snd_sid = NULL;
static long vol_max;

/******************************************************************************
*                               FUNCTION DEFINITIONS
******************************************************************************/

wiced_result_t a2dp_audio_device_configure(wiced_bt_a2dp_codec_info_t *codec_info)
{
    if (codec_info->codec_id == WICED_BT_A2DP_CODEC_SBC)
    {
        return init_alsa_sink(codec_info);
    }
    else{
        TRACE_ERR("Not configuring ALSA , since codec is other than SBC");
        return WICED_ERROR;
    }
}

wiced_result_t a2dp_set_audio_streaming(wiced_bool_t start_audio)
{
    return WICED_SUCCESS;
}

void a2dp_audio_data_cback(uint8_t *p_rx_media, uint32_t media_len)
{
	a2dp_sink_data_cback(p_rx_media, media_len);
}


//HFP sound playback through alsa 
wiced_result_t hf_audio_process_sco_data(uint16_t sco_channel, uint16_t length, uint8_t* p_sco_data)
{
    uint8_t *pOut = p_sco_data;

    snd_pcm_sframes_t alsa_frames = 0;
    snd_pcm_sframes_t alsa_frames_to_send = 0;

    alsa_frames_to_send = length / HFP_NUM_CHANNEL;
    alsa_frames_to_send = alsa_frames_to_send / SND_PCM_FORMAT_S16_LE; /*Bits per sample is 16 */

    while(1)
    {
        alsa_frames = alsa_write_hfp_data(pOut, alsa_frames_to_send);

        if (alsa_frames > 0 && alsa_frames < alsa_frames_to_send)
        {
            TRACE_WNG("app_avk_uipc_cback Short write (expected %li, wrote %li)",
                (long) alsa_frames_to_send, alsa_frames);
        }
        if(alsa_frames == alsa_frames_to_send)
        {
            break;
        }
        pOut += (alsa_frames * SND_PCM_FORMAT_S16_LE * HFP_NUM_CHANNEL);
        alsa_frames_to_send = alsa_frames_to_send - alsa_frames;
    }
    return WICED_SUCCESS;
}


void audio_decode_init(void){
    a2dp_sink_queue_init();
    TRACE_LOG("Create Decode Queue");
}


wiced_result_t audio_device_init(audio_device_type_t device_type)
{
    if (device_type == PLAYER)  //A2DP SINK
    {
        TRACE_LOG("PLAYER");
    }
    else if (device_type == RECORDER)  //HFP
    {
    #if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
         TRACE_LOG("RECORD WBS enabled");
    #else
         TRACE_LOG("RECORD");
    #endif
        init_alsa_hfp(SAMPLE_RATE);  //16000 SAMPLE_RATE
    }
    return WICED_SUCCESS;
}

wiced_result_t audio_device_deinit(audio_device_type_t device_type)
{
    if (device_type == PLAYER)  //A2DP SINK
    {
        TRACE_LOG("PLAYER");
        deinit_alsa_sink(); 
    }
    else if (device_type == RECORDER)  //HFP
    {
        TRACE_LOG("RECORD");
        deinit_alsa_hfp();
    }
    return WICED_SUCCESS;
}


uint32_t audio_capture_mic_data(uint8_t *mic_data, snd_pcm_sframes_t _frames){
    return alsa_capture_mic_data(mic_data, _frames);
}

bool a2dp_audio_decode(){
    return a2dp_sink_dequeue();
}
