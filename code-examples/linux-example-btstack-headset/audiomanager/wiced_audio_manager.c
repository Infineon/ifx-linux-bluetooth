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

/** @file
 * Audio manager is a high level interface for applications to use platform audio capabilities.
 * It supports different streams like A2DP_PLAYBACK, HFP etc.
 * Audio manager also has interface for specific stream Start, Stop and also stream specific volume control.
 *
 */

#include <wiced_audio_manager.h>
#include <wiced_bt_trace.h>
#include <platform_audio_device.h>
#include "audio_effects.h"

/**
 *  @brief Audio Manager stream information
 */
typedef struct
{
    uint32_t                    no_of_codec_devices;
    stream_type_t               stream_type;
    param_type_t                config_type;
    platform_audio_device_id_t  codec_device_id;
    audio_config_t              audio_config;
    codec_config_t              codec_config;
    nrec_config_t               nerc_config;
    platform_audio_io_device_t  platform_iodevice;
} audio_manager_stream_t;

typedef struct
{
    wiced_bool_t            initialized;
    int32_t                 playback_stream_opened;
    int32_t                 hfp_stream_opened;
    uint32_t                nrec_effect_id;
    audio_manager_stream_t  stream[MAX_NO_OF_STREAMS];
} wiced_audio_manager_info_t;

wiced_audio_manager_info_t wiced_am_info = {0};

static uint32_t wiced_am_stream_set_default_param(uint32_t stream_id, uint32_t stream_type);

/**
 * The application should call this function to Initialize Audio Manager
 *
 * @param           stream_type  : Type of the stream to register any.
 * @return          none
 */
void wiced_am_init(void)
{
    WICED_BT_TRACE("Audio Manager init \n");
    int32_t i;

    if (wiced_am_info.initialized == WICED_TRUE)
        return;

    for (i=0; i<MAX_NO_OF_STREAMS; ++i)
    {
        wiced_am_info.stream[i].stream_type = (stream_type_t) STREAM_TYPE_INVALD;
    }

    wiced_am_info.initialized = WICED_TRUE;
}
/**
 * Application should call this function to open a stream
 * This will construct the graph of various components in the chain based on the stream type.
 *
 * @param           stream_type  : Type of the stream to register (stream_type_t)
 * @param           args : Arguments to be passed if any.
 * @return          stream id
 *                  invalid if WICED_AUDIO_MANAGER_STREAM_ID_INVALID
 */
int32_t wiced_am_stream_open(uint32_t stream_type)
{
    WICED_BT_TRACE("Audio Manager Stream Open \n");
    int32_t i;

    /* Find available space or check if the stream is already opened. */
    for (i= 0; i< MAX_NO_OF_STREAMS; ++i)
    {
        if (wiced_am_info.stream[i].stream_type == (stream_type_t) stream_type)
        {   // stream already opened
            return i;
        }
        else
        {
            if (wiced_am_info.stream[i].stream_type == STREAM_TYPE_INVALD)
            {
                wiced_am_info.stream[i].stream_type = (stream_type_t) stream_type;
                break;
            }
        }
    }

    if (i >= MAX_NO_OF_STREAMS)
    {
        return WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
    }

    if (stream_type == A2DP_PLAYBACK)
    {
        /** Initialize stream type     */
        WICED_BT_TRACE("am_stream_open A2DP_PLAYBACK \n");
        wiced_am_info.stream[i].no_of_codec_devices = 1;
        wiced_am_info.stream[i].codec_device_id     = PLATFORM_DEVICE_PLAY;
        if (!wiced_am_info.playback_stream_opened)
        {
            if (WICED_SUCCESS != \
                platform_audio_device_init(wiced_am_info.stream[i].codec_device_id))
            {
                WICED_BT_TRACE("platform_audio_device_init failed\n");
            }
        }
        wiced_am_info.playback_stream_opened++;
        WICED_BT_TRACE("am_stream_open A2DP stream_id: %d \n", i);
    }
    else if (stream_type == HFP)
    {
        WICED_BT_TRACE("am_stream_open HFP \n");
        wiced_am_info.stream[i].no_of_codec_devices = 2;
        wiced_am_info.stream[i].codec_device_id     = PLATFORM_DEVICE_PLAY_RECORD;
        if (!wiced_am_info.hfp_stream_opened)
        {
            if(WICED_SUCCESS != \
               platform_audio_device_init(wiced_am_info.stream[i].codec_device_id))
            {
                WICED_BT_TRACE("platform_audio_device_init failed\n");
            }
        }
        wiced_am_info.hfp_stream_opened++;
        WICED_BT_TRACE("am_stream_open HFP stream_id: %d \n", i);
    }

    return i;
}

/**
 * The application should call this function to start the stream
 *
 * @param           stream_id  : Id of the stream.
 * @return          WICED_NOT_FOUND
 *                  WICED_SUCCESS
 */
wiced_result_t wiced_am_stream_start(int32_t stream_id)
{
    WICED_BT_TRACE("Audio Manager stream start stream_id: %d \n",stream_id);

    if ((0<=stream_id) && (stream_id < MAX_NO_OF_STREAMS))
    {
        switch (wiced_am_info.stream[stream_id].stream_type)
        {
        case HFP:
            if (WICED_SUCCESS != platform_audio_device_start(wiced_am_info.stream[stream_id].codec_device_id))
            {
                WICED_BT_TRACE("platform_audio_device_start failed\n");
                return WICED_NOT_FOUND;
            }
            break;
        case A2DP_PLAYBACK:
            if (WICED_SUCCESS != platform_audio_device_start(wiced_am_info.stream[stream_id].codec_device_id))
            {
                WICED_BT_TRACE("platform_audio_device_start failed\n");
                return WICED_NOT_FOUND;
            }
            break;
        default:
            WICED_BT_TRACE("Nothing to be done at this point of time \n");
            break;
        }

        return WICED_SUCCESS;
    }
    else
    {
        return WICED_NOT_FOUND;
    }

}

/**
 * The application should call this function to stop the stream
 *
 * @param           stream_id  : Id of the stream
 * @return          WICED_NOT_FOUND
 *                  WICED_SUCCESS
 */
wiced_result_t wiced_am_stream_stop(int32_t stream_id)
{
    WICED_BT_TRACE("Audio Manager stream stop Stream_ID: %d\n",stream_id);

    if ((0 <= stream_id) && (stream_id < MAX_NO_OF_STREAMS))
    {
        WICED_BT_TRACE("Audio Manager Stream Stop \n");
        switch (wiced_am_info.stream[stream_id].stream_type)
        {
        case HFP:
            if (WICED_SUCCESS != platform_audio_device_stop(wiced_am_info.stream[stream_id].codec_device_id))
            {
                WICED_BT_TRACE("platform_audio_device_stop failed\n");
                return WICED_NOT_FOUND;
            }
            break;
        case A2DP_PLAYBACK:
            if (WICED_SUCCESS != platform_audio_device_stop(wiced_am_info.stream[stream_id].codec_device_id))
            {
                WICED_BT_TRACE("platform_audio_device_stop failed\n");
                return WICED_NOT_FOUND;
            }
            break;
        default:
            WICED_BT_TRACE("Nothing to be done at this point of time \n");
            break;
        }

        return WICED_SUCCESS;
    }
    else
    {
        return WICED_NOT_FOUND;
    }
}

/**
 * The application should call this function to close the stream
 *
 * @param           stream_id  : Id of the stream
 * @return          WICED_NOT_FOUND
 *                  WICED_SUCCESS
 */
wiced_result_t wiced_am_stream_close(int32_t stream_id)
{
    WICED_BT_TRACE("Audio Manager stream close \n");

    if ((0 <= stream_id) && (stream_id < MAX_NO_OF_STREAMS))
    {
        WICED_BT_TRACE("Audio Manager Stream Close stream_id: %d\n",stream_id);

        switch (wiced_am_info.stream[stream_id].stream_type)
        {
        case A2DP_PLAYBACK:
            wiced_am_info.playback_stream_opened--;
            WICED_BT_TRACE("Audio Manager Stream Close A2DP stream_id: %d\n",stream_id);
            if (!wiced_am_info.playback_stream_opened)
            {
                if (WICED_SUCCESS != platform_audio_device_deinit(PLATFORM_DEVICE_PLAY))
                {
                    WICED_BT_TRACE("platform_audio_device_deinit failed\n");
                    return WICED_NOT_FOUND;
                }
            }
            break;
        case HFP:
            wiced_am_info.hfp_stream_opened--;
            WICED_BT_TRACE("Audio Manager Stream Close HFP stream_id: %d\n",stream_id);
            if (!wiced_am_info.hfp_stream_opened)
            {
                if (WICED_SUCCESS != platform_audio_device_deinit(PLATFORM_DEVICE_PLAY_RECORD))
                {
                    WICED_BT_TRACE("platform_audio_device_deinit failed\n");
                    return WICED_NOT_FOUND;
                }
            }
            break;
        default:
            WICED_BT_TRACE("Nothing to be done at this point of time \n");
            break;
        }

        wiced_am_info.stream[stream_id].stream_type = STREAM_TYPE_INVALD;
        return WICED_SUCCESS;
    }
    else
    {
        return WICED_NOT_FOUND;
    }
}

/**
 * The application should call this function to set the stream parameters
 *
 * @param           stream_id  : ID of the stream
 * @param           param_type : Type of parameters
 * @param           param_config: parameters configuration
 * @return          WICED_NOT_FOUND
 *                  WICED_ERROR
 *                  WICED_SUCCESS
 */
wiced_result_t wiced_am_stream_set_param(int32_t stream_id, uint32_t param_type, void *param_config)
{
    int32_t sink;
    platform_audio_config_t codec_config;
    audio_effect_config_t effect_conf;

    WICED_BT_TRACE("Audio Manager stream set param: %d, %d\n", stream_id, param_type);

    if ((0 <= stream_id) && (stream_id < MAX_NO_OF_STREAMS))
    {
        switch (param_type)
        {
        case AM_AUDIO_CONFIG:
            memcpy((void *) &wiced_am_info.stream[stream_id].audio_config, \
                   param_config, \
                   sizeof(audio_config_t));

            codec_config.sample_rate = wiced_am_info.stream[stream_id].audio_config.sr;
            codec_config.channels =  wiced_am_info.stream[stream_id].audio_config.channels;
            codec_config.bits_per_sample = wiced_am_info.stream[stream_id].audio_config.bits_per_sample;
            codec_config.volume = wiced_am_info.stream[stream_id].audio_config.volume;

            switch (wiced_am_info.stream[stream_id].stream_type)
            {
            case HFP:
                WICED_BT_TRACE("AM_AUDIO_CONFIG - HFP\n");
                codec_config.io_device = ANALOGMIC;
                if( WICED_SUCCESS != platform_audio_device_configure(wiced_am_info.stream[stream_id].codec_device_id, &codec_config))
                {
                    WICED_BT_TRACE("platform_audio_device_configure HFP failed\n");
                    return WICED_NOT_FOUND;
                }

                if (wiced_am_info.stream[stream_id].audio_config.sink == AM_SPEAKERS)
                    sink = LINEOUT;
                else
                    sink = HEADPHONES;

                if (WICED_SUCCESS != platform_audio_device_set_output_device(wiced_am_info.stream[stream_id].codec_device_id,sink))
                {
                    WICED_BT_TRACE("platform_audio_device_set_sink failed\n");
                    return WICED_NOT_FOUND;
                }
                break;
            case A2DP_PLAYBACK:
                WICED_BT_TRACE("AM_AUDIO_CONFIG - A2DP_PLAYBACK\n");
                switch (wiced_am_info.stream[stream_id].audio_config.sink)
                {
                case AM_SPEAKERS:
                    codec_config.io_device = LINEOUT;
                    break;
                case AM_HEADPHONES:
                default:
                    codec_config.io_device = HEADPHONES;
                    break;
                }

                if( WICED_SUCCESS != platform_audio_device_configure(wiced_am_info.stream[stream_id].codec_device_id, &codec_config))
                {
                    WICED_BT_TRACE("platform_audio_device_configure A2DP failed\n");
                    return WICED_NOT_FOUND;
                }
                break;
            default:
                WICED_BT_TRACE("Nothing to be done at this point of time \n");
                break;
            }

            break;
        case AM_CODEC_CONFIG:
            break;
        case AM_SPEAKER_VOL_LEVEL:
            wiced_am_info.stream[stream_id].audio_config.volume = *((int32_t*)(param_config));
            if (WICED_SUCCESS != platform_audio_device_set_volume(wiced_am_info.stream[stream_id].codec_device_id, wiced_am_info.stream[stream_id].audio_config.volume))
            {
                WICED_BT_TRACE("platform_audio_device_set_volume failed\n");
                return WICED_NOT_FOUND;
            }
            break;
        case AM_MIC_GAIN_LEVEL:
            wiced_am_info.stream[stream_id].audio_config.mic_gain = *((int32_t*)(param_config));
            WICED_BT_TRACE("am_stream_set_param mic_gain:%d\n",
                    wiced_am_info.stream[stream_id].audio_config.mic_gain);
            if (wiced_am_info.stream[stream_id].codec_device_id == PLATFORM_DEVICE_PLAY_RECORD)
            {
                if (WICED_SUCCESS != platform_audio_device_set_mic_gain(wiced_am_info.stream[stream_id].codec_device_id,wiced_am_info.stream[stream_id].audio_config.mic_gain))
                {
                    WICED_BT_TRACE("platform_audio_device_set_mic_gain failed\n");
                    return WICED_NOT_FOUND;
                }
            }
            break;
        case AM_SAMPLE_RATE:
            wiced_am_info.stream[stream_id].audio_config.sr = *((int32_t*)(param_config));
            if (WICED_SUCCESS != platform_audio_device_set_sr(wiced_am_info.stream[stream_id].codec_device_id,wiced_am_info.stream[stream_id].audio_config.sr))
            {
                WICED_BT_TRACE("platform_audio_device_set_sr failed\n");
                return WICED_NOT_FOUND;
            }
            break;
        case AM_IO_DEVICE:
            wiced_am_info.stream[stream_id].audio_config.sink = *((int32_t*)(param_config));
            if (wiced_am_info.stream[stream_id].audio_config.sink == AM_SPEAKERS)
                sink = LINEOUT;
            else
                sink = HEADPHONES;

            if (WICED_SUCCESS != platform_audio_device_set_output_device(wiced_am_info.stream[stream_id].codec_device_id,sink))
            {
                WICED_BT_TRACE("platform_audio_device_set_sink failed\n");
                return WICED_NOT_FOUND;
            }
            break;
        default:
            WICED_BT_TRACE("Unknown param_type:%d\n", param_type);
            break;
        }

        return WICED_SUCCESS;
    }
    else
    {
        return WICED_NOT_FOUND;
    }
}

/**
 * Called to set default_parameters
 *
 * @param           stream_type  : Type of the stream
 * @param           stream_id  : ID of the stream
 * @return          result
 */
uint32_t wiced_am_stream_set_default_param(uint32_t stream_id, uint32_t stream_type)
{
    static platform_audio_config_t codec_config;
    uint32_t i;
    WICED_BT_TRACE("Audio Manager Stream set default param Called \n");
    for(i=0; i<wiced_am_info.stream[stream_id].no_of_codec_devices; ++i) {
        WICED_BT_TRACE("codec device id : %d \n", wiced_am_info.stream[stream_id].codec_device_id);
        if(wiced_am_info.stream[stream_id].codec_device_id == PLATFORM_DEVICE_PLAY) {
            codec_config.sample_rate = DEFAULT_PLAYBACK_SR;
            codec_config.channels =  DEFAULT_CH;
            codec_config.bits_per_sample = DEFAULT_BITSPSAM;
            codec_config.volume = DEFAULT_VOLUME;
            codec_config.io_device = HEADPHONES;
            WICED_BT_TRACE("playback default param being set \n");
            if( WICED_SUCCESS != platform_audio_device_configure(PLATFORM_DEVICE_PLAY, &codec_config)) {
                WICED_BT_TRACE("platform_audio_device_configure failed\n");
                return WICED_NOT_FOUND;
                }
        }

        else if(wiced_am_info.stream[stream_id].codec_device_id == PLATFORM_DEVICE_PLAY_RECORD) {
            codec_config.sample_rate = DEFAULT_RECORD_SR;
            codec_config.channels =  1;
            codec_config.bits_per_sample = DEFAULT_BITSPSAM;
            codec_config.volume = DEFAULT_VOLUME;
            codec_config.io_device = ANALOGMIC;
            WICED_BT_TRACE("Record default param being set \n");
            if( WICED_SUCCESS != platform_audio_device_configure(PLATFORM_DEVICE_PLAY_RECORD, &codec_config)) {
                WICED_BT_TRACE("platform_audio_device_configure failed\n");
                return WICED_NOT_FOUND;
                }

        }
    }

    return WICED_SUCCESS;
}

/**
 * The application should call this function to get parameters of the stream
 *
 * @param           stream_id  : ID of the stream
 * @param           param_type : Type of parameters
 * @param           param_config: parameters configuration
 * @return          result
 */
int32_t wiced_am_stream_get_param(uint32_t stream_id, uint32_t param_type, void* param_config)
{
    /** TO Do */
    return WICED_SUCCESS;
}

/**
 * The application should call this function to get interface
 *
 * @param           stream_id  : Id of the stream
 * @param           inf_type : interface type
 * @return          stream_interface_t : pointer to stream interface
 */
stream_interface_t* wiced_am_stream_get_intf(uint32_t stream_id, uint32_t inf_type)
{
    /** TO DO */
    /** static to avoid warning */
    static stream_interface_t interface;
    return &interface;
}

/**
 * wiced_am_stream_id_get
 *
 * Get the assigned stream id.
 *
 * @param stream_type
 *
 * @return assigned stream id or WICED_AUDIO_MANAGER_STREAM_ID_INVALID
 */
int32_t wiced_am_stream_id_get(stream_type_t stream_type)
{
    int32_t i;

    for (i = 0 ; i < MAX_NO_OF_STREAMS ; i++)
    {
        if (wiced_am_info.stream[i].stream_type == stream_type)
        {   // stream already opened
            return i;
        }
    }

    return WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
}
