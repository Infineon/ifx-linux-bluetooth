/*
 * $ Copyright Cypress Semiconductor $
 */

#include "wiced_bt_trace.h"
#include "wiced_data_types.h"
#ifdef COMPONENT_audiomanager
#include "wiced_audio_manager.h"
#endif // COMPONENT_audiomanager
#ifdef COMPONENT_cyw9bt_audio5
#include "platform_audio_device.h"
#endif // COMPONENT_cyw9bt_audio5

#ifndef DEFAULT_VOLUME
#define DEFAULT_VOLUME 80
#endif

static uint8_t prev_volume = DEFAULT_VOLUME;
static uint8_t prev_mute_enabled = 0;

#if 0
void audio_driver_set_volume(uint8_t volume)
{
    WICED_BT_TRACE("[%s] volume %d\n", __FUNCTION__, volume / 10);
    WICED_BT_TRACE("[%s] mute state %d\n", __FUNCTION__, prev_mute_enabled);
#ifdef COMPONENT_cyw9bt_audio5
    if(!prev_mute_enabled)
        platform_audio_device_set_volume(PLATFORM_DEVICE_PLAY, volume / 10);
#endif // COMPONENT_cyw9bt_audio5
    prev_volume = volume;
}

void audio_driver_set_mute_state(uint8_t mute_enabled)
{
    int volume = mute_enabled ? 0 : prev_volume;
    prev_mute_enabled = mute_enabled;
    WICED_BT_TRACE("[%s] mute state %d\n", __FUNCTION__, mute_enabled);
    WICED_BT_TRACE("[%s] volume %d\n", __FUNCTION__, volume / 10);
#ifdef COMPONENT_cyw9bt_audio5
    platform_audio_device_set_volume(PLATFORM_DEVICE_PLAY, volume / 10);
#endif
}
#endif

#ifdef COMPONENT_audiomanager
int32_t audio_driver_config_frequency(int32_t sampling_rate,
                                      int32_t no_of_channels,
                                      int32_t bits_per_sample,
                                      am_audio_io_device_t sink,
                                      uint32_t stream_type)
{
    audio_config_t audio_config;
    int32_t stream_id;

    audio_config.sr = sampling_rate;
    audio_config.channels = no_of_channels;
    audio_config.bits_per_sample = bits_per_sample;
    audio_config.volume = prev_mute_enabled ? 0 : prev_volume;
    audio_config.sink = sink;

    stream_id = wiced_am_stream_open(stream_type);
    WICED_BT_TRACE("[%s] volume %d\n", __FUNCTION__,  audio_config.volume / 10);

    if (WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_AUDIO_CONFIG, &audio_config))
    {
        WICED_BT_TRACE("wiced_am_set_param set audio config failed\n");
    }

    /* Set MIC gain. */
    if (WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_MIC_GAIN_LEVEL, &audio_config.volume))
    {
        WICED_BT_TRACE("wiced_am_set_param: AM_MIC_GAIN_LEVEL set audio config failed\n");
    }

    return stream_id;
}
#endif // COMPONENT_audiomanager

void audio_driver_init_vol()
{
    int volume = prev_mute_enabled ? 0 : prev_volume/10;
#ifdef COMPONENT_cyw9bt_audio5
    platform_audio_device_set_volume(PLATFORM_DEVICE_PLAY, volume);
    platform_audio_device_set_mic_gain(PLATFORM_DEVICE_PLAY_RECORD, volume);
#endif
}
