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
/******************************************************************************/
/******************************************************************************
 * File Name: alsa_playback.c
 *
 * Description: This is the source file for Linux alsa setting, play and record.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/

#include "alsa_playback.h"

#include <pthread.h>

#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_stack.h"
#include "wiced_app_cfg.h"
#include "wiced_memory.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_dev.h"
#include "bt_hs_spk_control.h"
#include "wiced_bt_a2dp_sink.h"
#include "sbc_decoder.h"
#include "sbc_dec_func_declare.h"
#include "sbc_types.h"
#include "alsa/asoundlib.h"

#include "bt_hs_spk_handsfree.h"
#include "platform_audio_interface.h"
#include "app_bt_utils.h"
#include "bt_decode_queue.h"
#include "alsa_capture.h"
#include "log.h"
#include "bt_hs_spk_audio.h"

/*******************************************************************************
*                                   MACROS
*******************************************************************************/
#define MSBC_STATIC_MEM_SIZE      1920 //BYTES
#define MSBC_SCRATCH_MEM_SIZE     2048 // BYTES
#define ALSA_LATENCY             80000 // value based on audio playback
#define VOLUME_MAX                 100
#define ALSA_MIXER_NAME_SIZE         3
#define ALSA_A2DP_LATENCY       500000

/*******************************************************************************
*                               STRUCTURES AND ENUMERATIONS
*******************************************************************************/

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/

extern wiced_result_t wiced_transport_send_data(uint16_t type, uint8_t* p_data, uint16_t data_size);
extern wiced_bool_t bMediaRxEvt;

/****************************************************************************
 *                              LOCAL VARIABLES
 ***************************************************************************/
static char *alsa_device = "default";
static char *alsa_playback_device = "default";
static char *alsa_capture_device = "default";

static char *alsa_amixer_device = "default";
static char* alsa_amixer_name[ALSA_MIXER_NAME_SIZE] = {
    "Speaker",
    "Master",
    "Headphone"
};

static SINT32 staticMem[MSBC_STATIC_MEM_SIZE / sizeof (SINT32)];
static SINT32 scratchMem[MSBC_SCRATCH_MEM_SIZE / sizeof (SINT32)];

static int  PcmBytesPerFrame = 0;
static snd_mixer_elem_t* snd_mixer_elem = NULL;
static snd_mixer_t *snd_mixer_handle = NULL;
static snd_mixer_selem_id_t *snd_sid = NULL;
static long vol_min = 0;
static long vol_max = 0;
static long vol_range = 0;

static queue_t* sinkQueue = NULL;

//capture use
static alsa_config_t  ac_config;
static alsa_capture_t *ac_device;

pthread_mutex_t cond_sink_decode_lock = PTHREAD_MUTEX_INITIALIZER;

SBC_DEC_PARAMS  strDecParamsSink = { 0 }; /*for Sink use*/
SBC_DEC_PARAMS  strDecParams = { 0 }; /*for HFP use*/
snd_pcm_t *p_alsa_handle = NULL; /* Handle for a2dp playback */
snd_pcm_t *p_alsa_hfp_handle = NULL; /* Handle for HFP playback */
snd_pcm_t *p_alsa_capture_handle = NULL; /* Handle for audio recorder */
snd_pcm_hw_params_t *params;
snd_pcm_uframes_t frames;
snd_pcm_format_t format;
uint16_t sample_rate = 0;
int period_time = 0;
snd_pcm_sframes_t alsa_frames_to_send = 0;
uint16_t *pOut = NULL;


/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/
static uint16_t a2dp_sink_get_frame_len (uint8_t *pHeader);
static void alsa_volume_driver_deinit(void);

/******************************************************************************
*                               FUNCTION DEFINITIONS
******************************************************************************/

/*****************************************************************************
* Function Name: a2dp_sink_get_frame_len
******************************************************************************
* Summary: a2dp calculate frame len from setting
*          
*     
*
* Parameters: uint8_t *pHeader
*               a2dp package header
*   
* 
* Return:  uint16_t: 
*              frame length
*   
*
****************************************************************************/
static uint16_t a2dp_sink_get_frame_len (uint8_t *pHeader)
{
    uint8_t *pframe, *pframe1, *pframe2;
    uint8_t  Blocks, Channel_mode, Subbands, Bitpool, nrof_channels, Join;
    uint32_t  FrameLen, tmp;

    pframe = pHeader;
    FrameLen = 0;

    if (*pframe != 0x9C)
    {
        TRACE_ERR("Bad Sync Byte: 0x%02x!!!", *pframe);
        return (0);
    }

    pframe1 = pframe + 1;
    pframe2 = pframe1 + 1;

    Blocks = (*(unsigned char *)pframe1 & 0x30) >> 4;

    switch (Blocks)
    {
    case 0x0:Blocks = 4; break;
    case 0x1:Blocks = 8; break;
    case 0x2:Blocks = 12; break;
    case 0x3:Blocks = 16; break;
    default: break;
    }
    Channel_mode	= (*(unsigned char *)pframe1 & 0x0C) >> 2;
    nrof_channels = (Channel_mode == 0) ? 1 : 2;

    Subbands	= (*(unsigned char *)pframe1 & 0x01);
    Subbands = (Subbands == 0) ? 4 : 8;

    Bitpool	= (*(unsigned char *)pframe2 & 0xff);

    FrameLen = 4 + (4 * Subbands * nrof_channels) / 8;

    Join = (Channel_mode == 3) ? 1 : 0;

    if (Channel_mode < 2)
        tmp =  (Blocks * nrof_channels * Bitpool);
    else
        tmp =  (Join * Subbands + Blocks * Bitpool);

    FrameLen += tmp / 8 + ((tmp % 8) ? 1 : 0);

    return ((uint16_t)FrameLen);
}

/*****************************************************************************
* Function Name: alsa_volume_init
******************************************************************************
* Summary: alsa volume driver init
*           
*
* Parameters: none
*   
* 
* Return:  bool: true: success
*                false: fail
*   
*
****************************************************************************/
bool alsa_volume_init(void)
{
    alsa_volume_driver_deinit();

    snd_mixer_open (&snd_mixer_handle, 0);
    if (snd_mixer_handle == NULL)
    {
        TRACE_ERR ("snd_mixer_open Failed");
        return false;
    }
    else
    {
        snd_mixer_attach (snd_mixer_handle, alsa_amixer_device);
        snd_mixer_selem_register (snd_mixer_handle, NULL, NULL);
        snd_mixer_load (snd_mixer_handle);
        snd_mixer_selem_id_malloc (& snd_sid);
        if (snd_sid == NULL)
        {
            alsa_volume_driver_deinit ();
            TRACE_ERR ("snd_sid malloc Failed");
            return false;
        }
        else
        {
            snd_mixer_selem_id_set_index(snd_sid, 0);
            for (int i = 0; i < ALSA_MIXER_NAME_SIZE; i++){
                snd_mixer_selem_id_set_name(snd_sid, alsa_amixer_name[i]);
                snd_mixer_elem = snd_mixer_find_selem(snd_mixer_handle, snd_sid);
                if (snd_mixer_elem != NULL){
                    snd_mixer_selem_get_playback_volume_range(snd_mixer_elem, &vol_min, &vol_max);
                    vol_range = vol_max - vol_min;
                    TRACE_LOG("[Done] Get Alsa Mixer Vol min: %ld, max: %ld range:%ld", vol_min, vol_max, vol_range);
                    return true;
                }
            }
        }
    }
    TRACE_ERR ("snd_mixer_find_selem Failed");
    return false;
}

/*****************************************************************************
* Function Name: alsa_volume_driver_deinit
******************************************************************************
* Summary: alsa volume driver deinit
*           
*
* Parameters: none
*   
* 
* Return:  none: 
*   
*
****************************************************************************/
static void alsa_volume_driver_deinit(void)
{

    if (snd_mixer_handle != NULL)
    {
        snd_mixer_close(snd_mixer_handle);
        snd_mixer_handle = NULL;
    }
    if (snd_sid != NULL)
    {
        snd_mixer_selem_id_free(snd_sid);
        snd_sid = NULL;
    }
    snd_mixer_elem = NULL;
}

/*****************************************************************************
* Function Name: alsa_set_volume
******************************************************************************
* Summary: alsa set volume 
*           
*
* Parameters: uint8_t volume:  range from 0 ~ 100
*   
* 
* Return:  none: 
*   
*
****************************************************************************/
void alsa_set_volume(uint8_t volume)
{
    TRACE_LOG("alsa_set_volume volume %d",volume);
    if (snd_mixer_elem == NULL)
    {
        alsa_volume_init();
    }

    if (volume > VOLUME_MAX){
        volume = VOLUME_MAX;
    }
    if (snd_mixer_elem)
    {
        snd_mixer_selem_set_playback_volume_all(snd_mixer_elem,  volume * vol_range / VOLUME_MAX);
    }
    else{
        TRACE_ERR("alsa_set_volume Fail!!!!");
    }
}

/*****************************************************************************
* Function Name: init_alsa_sink
******************************************************************************
* Summary: Init alsa pcm for sink
*          
*     
*
* Parameters: wiced_bt_a2dp_codec_info_t* codec_info
*               codec related setting, please check wiced_bt_a2dp_defs.h
*   
* 
* Return:  int: 
*               0 on success otherwise a negative error code    
*   
*
****************************************************************************/
int init_alsa_sink(wiced_bt_a2dp_codec_info_t *codec_info)
{
    int status = ALSA_SUCCESS;
    snd_pcm_format_t format = 0;

    TRACE_LOG("Init Start");

    memset (staticMem, 0, sizeof (staticMem));
    memset (scratchMem, 0, sizeof (scratchMem));
    memset (&strDecParamsSink, 0, sizeof (strDecParamsSink));

    strDecParamsSink.s32StaticMem  = staticMem;
    strDecParamsSink.s32ScratchMem = scratchMem;

    strDecParamsSink.numOfBlocks      = codec_info->cie.sbc.block_len;

    if (codec_info->cie.sbc.ch_mode == A2D_SBC_IE_CH_MD_STEREO ||
            codec_info->cie.sbc.ch_mode == A2D_SBC_IE_CH_MD_JOINT)
        strDecParamsSink.numOfChannels    = 2;
    else
        strDecParamsSink.numOfChannels    = 1;

    if (codec_info->cie.sbc.num_subbands == A2D_SBC_IE_SUBBAND_4)
        strDecParamsSink.numOfSubBands = 4;
    else if (codec_info->cie.sbc.num_subbands == A2D_SBC_IE_SUBBAND_8)
        strDecParamsSink.numOfSubBands = 8;

    if (codec_info->cie.sbc.alloc_mthd == A2D_SBC_IE_ALLOC_MD_S)
        strDecParamsSink.allocationMethod = SBC_SNR;
    else if (codec_info->cie.sbc.alloc_mthd == A2D_SBC_IE_ALLOC_MD_L)
        strDecParamsSink.allocationMethod = SBC_LOUDNESS;


    if (codec_info->cie.sbc.samp_freq == A2D_SBC_IE_SAMP_FREQ_44)
        sample_rate = 44100;// "44.1KHz";
    else if (codec_info->cie.sbc.samp_freq == A2D_SBC_IE_SAMP_FREQ_48)
        sample_rate = 48000;// "48KHz";


    SBC_Decoder_decode_Init(&strDecParamsSink);

    TRACE_LOG("nblocks %d nchannels %d nsubbands %d ameth %d freq %d", strDecParamsSink.numOfBlocks , strDecParamsSink.numOfChannels,
                         strDecParamsSink.numOfSubBands, strDecParamsSink.allocationMethod, sample_rate);


    PcmBytesPerFrame = strDecParamsSink.numOfBlocks * strDecParamsSink.numOfChannels * strDecParamsSink.numOfSubBands * 2;


    /* If ALSA PCM driver was already open => close it */
    if (p_alsa_handle != NULL)
    {
        snd_pcm_close(p_alsa_handle);
        p_alsa_handle = NULL;
    }

    status = snd_pcm_open(&(p_alsa_handle), alsa_device, SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK);

    if (status < 0)
    {
        TRACE_LOG("snd_pcm_open failed: %s", snd_strerror(status));
        return status;
    }
    else
    {
        snd_pcm_uframes_t buffer_size = 0;
        snd_pcm_uframes_t period_size = 0;

        format = SND_PCM_FORMAT_S16_LE;
        /* Configure ALSA driver with PCM parameters */
        status = snd_pcm_set_params(p_alsa_handle,
                                    format,
                                    SND_PCM_ACCESS_RW_INTERLEAVED,
                                    strDecParamsSink.numOfChannels,
                                    sample_rate,
                                    1,
                                    ALSA_A2DP_LATENCY);
        if (status < 0)
        {
            TRACE_LOG("snd_pcm_set_params failed: %s", snd_strerror(status));
            return status;
        }
        snd_pcm_get_params(p_alsa_handle, &buffer_size, &period_size);
        TRACE_LOG("snd_pcm_get_params150ms bs %ld ps %ld", buffer_size, period_size);
    }

    alsa_frames_to_send = PcmBytesPerFrame / strDecParamsSink.numOfChannels;
    alsa_frames_to_send /= 2; // Bits per sample is 16 

    if (pOut == NULL){
        pOut = (uint16_t *) malloc(PcmBytesPerFrame);
        if (pOut == NULL){
            TRACE_ERR("Init decode buffer failed!! %s", strerror(errno));
            status = ALSA_FAIL;
        }
    }

    TRACE_LOG("Init Done");
    return status;
}

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
void deinit_alsa_sink()
{
    TRACE_LOG("Flow");
    if (p_alsa_handle != NULL)
    {
        snd_pcm_close(p_alsa_handle);
        p_alsa_handle = NULL;
    }
    alsa_volume_driver_deinit();
    free(pOut);
    pOut = NULL;
}


/*****************************************************************************
* Function Name: init_alsa_hfp
******************************************************************************
* Summary: init alsa pcm for handsfree profile  
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

int init_alsa_hfp(uint16_t set_sample_rate)
{
    int status;
    int latency = ALSA_LATENCY;
    
    snd_pcm_uframes_t buffer_size = 0;
    snd_pcm_uframes_t period_size = 0;

    TRACE_LOG("Sample Rate:%d", set_sample_rate);

    format = SND_PCM_FORMAT_S16_LE; /* SND_PCM_FORMAT_U8; */
    PcmBytesPerFrame = HFP_NUM_BLOCKS * HFP_NUM_CHANNEL * HFP_NUM_SUBBANDS * 2;

    TRACE_LOG("PcmBytesPerFrame = %d\n",PcmBytesPerFrame);

    /* If ALSA PCM driver was already open => close it */
    if (p_alsa_hfp_handle != NULL)
    {
        TRACE_ERR("p_alsa_hfp_handle already one");
        snd_pcm_close(p_alsa_hfp_handle);
        p_alsa_hfp_handle = NULL;
        return ALSA_SUCCESS;
    }
    status = snd_pcm_open(&(p_alsa_hfp_handle), alsa_playback_device, SND_PCM_STREAM_PLAYBACK, SND_PCM_ASYNC);  //SND_PCM_ASYNC SND_PCM_NONBLOCK

    if (status < 0)
    {
        TRACE_ERR("snd_pcm_open failed: %s", snd_strerror(status));
        return status;
    }
    else
    {
        /* Configure ALSA driver with PCM parameters */
        status = snd_pcm_set_params(p_alsa_hfp_handle,
                                    format,
                                    SND_PCM_ACCESS_RW_INTERLEAVED,
                                    HFP_NUM_CHANNEL,
                                    set_sample_rate,
                                    1,
                                    latency);

        if (status < 0)
        {
            TRACE_ERR("snd_pcm_set_params failed: %s", snd_strerror(status));
            return status;
        }
        /* In snd_pcm_get_params:
         * buffer_size : PCM ring buffer size in frames
         * period_size: PCM period size in frames
         */
        snd_pcm_get_params(p_alsa_hfp_handle, &buffer_size, &period_size);
        TRACE_LOG("snd_pcm_get_params %d ms bs %ld ps %ld",(latency / 1000), buffer_size, period_size);
    }
    return ALSA_SUCCESS;
}

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
void deinit_alsa_hfp()
{
    TRACE_LOG("deinit_alsa_hfp");
    if (p_alsa_hfp_handle != NULL)
    {
        snd_pcm_close(p_alsa_hfp_handle);
        p_alsa_hfp_handle = NULL;
    }
}


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
snd_pcm_sframes_t alsa_write_hfp_data(uint8_t *p_rx_media, uint32_t data_size)
{
    int ret;

    if(!p_alsa_hfp_handle)
    {
        TRACE_ERR("p_alsa_handle is NULL");
        return 0;
    }

    ret = snd_pcm_writei(p_alsa_hfp_handle, (uint16_t *)p_rx_media, data_size);

    if (ret < 0)
    {
        if (ret == -EPIPE)
        {
            ret = snd_pcm_recover(p_alsa_hfp_handle, ret, 0);
        }
        if (ret < 0)
        {
            TRACE_ERR("snd_pcm_writei failed\n");
        }
    }
}

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
void a2dp_sink_data_cback( uint8_t* p_rx_media, uint32_t media_len )
{
    uint8_t         xx = 0, num_frames = 0;
    uint16_t        frame_len = 0;
    uint8_t *pInsert = NULL;

    //TRACE_LOG("Recd a2dp data len %d Num frames %d", media_len, (*p_rx_media & 0x0F));

    num_frames =  *p_rx_media & 0x0F;
    p_rx_media++;
    media_len--;

    if (p_alsa_handle == NULL)
    {
        TRACE_ERR("ALSA is not configured, dropping the data pkt!!!!");
        return;
    }
  
    for (xx = 0; xx < num_frames; xx++)
    {
        // Get frame length. stop if any error
        if ((frame_len = a2dp_sink_get_frame_len (p_rx_media)) == 0)
            break;
        if (frame_len > media_len)
        {
            TRACE_LOG("a2dp_sink_proc_stream_data(): media too short %d %d %d %d", num_frames, xx, frame_len, media_len);
            break;
        }

        //add data to queue
        pInsert = malloc(frame_len);
        memcpy(pInsert, p_rx_media, frame_len);

        pthread_mutex_lock(&cond_sink_decode_lock);
        node_t* newNode = node_create(frame_len, pInsert);
        queue_enqueue(sinkQueue, newNode);
        pthread_mutex_unlock(&cond_sink_decode_lock);
        bt_hs_spk_audio_activate_decode_thread();

        p_rx_media += frame_len;
        media_len  -= frame_len;
    }

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
        WICED_BT_TRACE("alsa_capture_pcm_read error %d\n", rc);
        return 0;
    }
    else{
        return rc;
    }
}

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
void a2dp_sink_queue_init(){
    if (sinkQueue == NULL){
        TRACE_LOG("a2dp_sink_queue_init queue_create()");
        sinkQueue = queue_create();

    }
}

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
bool a2dp_sink_dequeue(){
    snd_pcm_sframes_t alsa_frames = 0;
    if (pthread_mutex_trylock(&cond_sink_decode_lock) == 0){
        if (!queue_is_empty(sinkQueue)){
            node_t* nodeOut = queue_dequeue(sinkQueue);
            pthread_mutex_unlock(&cond_sink_decode_lock);
            if (pOut == NULL){
                TRACE_MSG("a2dp_sink_dequeue(): NO BUFFER!");
                clearQueue();
                return false;
            }
            else{
                SBC_Decoder_decoder (&strDecParamsSink, nodeOut->node_data, nodeOut->node_len, (SINT16 *)pOut);

                if (p_alsa_handle){
                    alsa_frames = snd_pcm_writei(p_alsa_handle, pOut, alsa_frames_to_send);
                
                    if (alsa_frames < 0)
                    {
                        alsa_frames = snd_pcm_recover(p_alsa_handle, alsa_frames, 0);
                    }

                    if (alsa_frames > 0 && alsa_frames < alsa_frames_to_send){
                    }
                }
            }
            free(nodeOut->node_data);
            free(nodeOut);
            return true;
        }
        else{
            pthread_mutex_unlock(&cond_sink_decode_lock);
        }
    }
    return false;
}

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
void clearQueue(){
    pthread_mutex_lock(&cond_sink_decode_lock);
    while (!queue_is_empty(sinkQueue)){
        queue_dequeue(sinkQueue);
    }
    pthread_mutex_unlock(&cond_sink_decode_lock);
}


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
#if defined(WICED_BT_HFP_HF_WBS_INCLUDED) && (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
    ac_config.channels = MIC_CAPTURE_CHANNEL; 
#else
    ac_config.channels = MIC_CAPTURE_CHANNEL_NBS;
#endif
    ac_config.format = ALSA_INT16;
    TRACE_LOG(" channel: %d", ac_config.channels);
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