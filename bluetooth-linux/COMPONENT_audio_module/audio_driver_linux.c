/*
 * $ Copyright Cypress Semiconductor $
 */

#include "asoundlib.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "audio_driver.h"
#include "audio_parse_wave.h"

#include "log.h"

#undef WICED_BT_TRACE
#undef WICED_BT_TRACE_CRIT
#define WICED_BT_TRACE printf
#define WICED_BT_TRACE_CRIT printf

static char *alsa_device = "default";
static char* alsa_amixer_name[] = {
    "Speaker",
    "Master",
    "Headphone"
};
snd_pcm_t *p_alsa_handle = NULL;
snd_pcm_t *p_alsa_mic_handle = NULL; //Microphone
snd_pcm_hw_params_t *mic_params;
snd_pcm_uframes_t mic_frames;

/* file to write the received audio data along with playing over ALSA for
 debugging purposes */
static FILE *rx_audio_file_ptr;
static snd_pcm_uframes_t period_size = 0;

static snd_mixer_elem_t *snd_mixer_elem = NULL;
static snd_mixer_t *snd_mixer_handle = NULL;
static snd_mixer_selem_id_t *snd_sid = NULL;
static long vol_max;

// ========================== ALSA Volume ==========================

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

static void alsa_volume_driver_init(void)
{
    long vol_min;
    WICED_BT_TRACE("alsa_volume_driver_init\n");

    alsa_volume_driver_deinit();

    snd_mixer_open(&snd_mixer_handle, 0);
    if (snd_mixer_handle == NULL)
    {
        WICED_BT_TRACE("alsa_volume_driver_init snd_mixer_open Failed\n");
        return;
    }
    snd_mixer_attach(snd_mixer_handle, "default");
    snd_mixer_selem_register(snd_mixer_handle, NULL, NULL);
    snd_mixer_load(snd_mixer_handle);

    snd_mixer_selem_id_malloc(&snd_sid);
    if (snd_sid == NULL)
    {
        alsa_volume_driver_deinit();
        WICED_BT_TRACE("alsa_volume_driver_init snd_mixer_selem_id_alloca Failed\n");
        return;
    }
    else
    {
        snd_mixer_selem_id_set_index(snd_sid, 0);
        for (int i = 0; i < sizeof(alsa_amixer_name)/sizeof(alsa_amixer_name[0]); i++)
        {
            snd_mixer_selem_id_set_name(snd_sid, alsa_amixer_name[i]);
            snd_mixer_elem = snd_mixer_find_selem(snd_mixer_handle, snd_sid);
            if (snd_mixer_elem != NULL)
            {
                snd_mixer_selem_get_playback_volume_range(snd_mixer_elem, &vol_min, &vol_max);
                TRACE_LOG("[Done] Get Alsa Mixer Vol min: %ld, max: %ld", vol_min, vol_max);
                return;
            }
        }
    }

    alsa_volume_driver_deinit();
    TRACE_ERR("alsa_volume_driver_init snd_mixer_find_selem Failed\n");
}

void audio_driver_set_volume(uint8_t volume)
{
    WICED_BT_TRACE("audio_driver_set_volume volume %d \n", volume);
    if (snd_mixer_elem == NULL)
    {
        alsa_volume_driver_init();
    }
    if (snd_mixer_elem)
    {
        snd_mixer_selem_set_playback_volume_all(snd_mixer_elem, volume * vol_max / 100);
    }
}

void audio_driver_set_mute_state(uint8_t mute_enabled)
{
    WICED_BT_TRACE("audio_driver_mute_state %d \n", mute_enabled);
    if (snd_mixer_elem == NULL)
    {
        alsa_volume_driver_init();
    }
    if (snd_mixer_elem)
    {
        snd_mixer_selem_set_playback_switch_all(snd_mixer_elem, mute_enabled ? 0 : 1);
    }
}

// ========================== ALSA Audio ==========================
static void alsa_init_microphone(uint8_t num_of_channels, uint32_t sample_rate)
{
    int rc, dir;

    /* Open PCM device for recording (capture). */
    rc = snd_pcm_open(&p_alsa_mic_handle, alsa_device, SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0)
    {
        fprintf(stderr, "unable to open pcm device: %s\n", snd_strerror(rc));
        exit(1);
    }

    /* Allocate a hardware parameters object. */
    snd_pcm_hw_params_alloca(&mic_params);

    /* Fill it in with default values. */
    snd_pcm_hw_params_any(p_alsa_mic_handle, mic_params);

    /* Set the desired hardware parameters. */
    snd_pcm_hw_params_set_access(p_alsa_mic_handle, mic_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(p_alsa_mic_handle, mic_params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(p_alsa_mic_handle, mic_params, num_of_channels);
    snd_pcm_hw_params_set_rate_near(p_alsa_mic_handle, mic_params, &sample_rate, &dir);
    mic_frames = (sample_rate / 100);
    snd_pcm_hw_params_set_period_size_near(p_alsa_mic_handle, mic_params, &mic_frames, &dir);

    /* Write the parameters to the driver */
    rc = snd_pcm_hw_params(p_alsa_mic_handle, mic_params);
    if (rc < 0)
    {
        fprintf(stderr, "unable to set hw parameters: %s\n", snd_strerror(rc));
    }

    /* Use a buffer large enough to hold one period */
    snd_pcm_hw_params_get_period_size(mic_params, &mic_frames, &dir);
}

int wiced_get_audio_data_form_mic(uint8_t *l_data, uint8_t *r_data)
{
    int rc = snd_pcm_readi(p_alsa_mic_handle, l_data, mic_frames);
    if (rc == -EPIPE)
    {
        /* EPIPE means overrun */
        fprintf(stderr, "overrun occurred\n");
        snd_pcm_prepare(p_alsa_mic_handle);
    }
    else if (rc < 0)
    {
        fprintf(stderr, "error from read: %s\n", snd_strerror(rc));
    }
    else if (rc != (int)mic_frames)
    {
        fprintf(stderr, "short read, read %d mic_frames\n", rc);
    }

    return rc;
}

static void alsa_audio_init(uint8_t num_of_channels, uint32_t sample_rate, uint16_t alsa_cfg_latency_ms)
{
    snd_pcm_uframes_t buffer_size = 0;

    TRACE_LOG("num_of_channels:%d, sample_rate:%d, alsa_cfg_latency_ms:%d\n", num_of_channels, sample_rate, alsa_cfg_latency_ms);

    int status =
        snd_pcm_open(&(p_alsa_handle), alsa_device, SND_PCM_STREAM_PLAYBACK, 0); /* Mode = 0 for blocking mode*/
    WICED_BT_TRACE("snd_pcm_open\n");

    if (status < 0)
    {
        WICED_BT_TRACE_CRIT("snd_pcm_open failed: %s\n", snd_strerror(status));
        return;
    }

    /* Configure ALSA driver with PCM parameters */
    status = snd_pcm_set_params(p_alsa_handle,
                                SND_PCM_FORMAT_S16_LE,
                                SND_PCM_ACCESS_RW_INTERLEAVED,
                                num_of_channels,
                                sample_rate,
                                1,
                                alsa_cfg_latency_ms * 1000);

    if (status < 0)
    {
        WICED_BT_TRACE("snd_pcm_set_params failed: %s\n", snd_strerror(status));
    }

    snd_pcm_get_params(p_alsa_handle, &buffer_size, &period_size);
    WICED_BT_TRACE("snd_pcm_get_params buffersize %lu frames, periodsize %lu frames\n", buffer_size, period_size);
    WICED_BT_TRACE("snd_pcm_get_params periodsize %lu bytes\n", period_size * 4 /*NUM_CHANNELS*SAMPLE_SIZE*/);
}

static void alsa_write_data(uint8_t *p_rx_media, uint32_t data_size)
{
    int ret;

    if(!p_alsa_handle)
    {
        WICED_BT_TRACE_CRIT("p_alsa_handle is NULL");
        return;
    }

    ret = snd_pcm_writei(p_alsa_handle, (uint16_t *)p_rx_media, data_size);
    if (ret < 0)
    {
        if (ret == -EPIPE)
        {
            ret = snd_pcm_recover(p_alsa_handle, ret, 0);
        }
        if (ret < 0)
        {
            WICED_BT_TRACE_CRIT("snd_pcm_writei failed\n");
        }
    }
}

static wiced_bool_t read_test_wav_file(uint32_t sample_rate)
{
    uint8_t ret = FALSE;

    if (48000 == sample_rate)
    {
        ret = audio_module_load_wave_file("./test_audio_files/test_48k.wav");
    }
	else if (32000 == sample_rate)
    {
        ret = audio_module_load_wave_file("./test_audio_files/test_32k.wav");
    }
    else if (24000 == sample_rate)
    {
        ret = audio_module_load_wave_file("./test_audio_files/test_24k.wav");
    }
    else if (16000 == sample_rate)
    {
        ret = audio_module_load_wave_file("./test_audio_files/test_16k.wav");
    }
    else if (8000 == sample_rate)
    {
        ret = audio_module_load_wave_file("./test_audio_files/test_8k.wav");
    }
    else
    {
        ret = 0xFF;
        TRACE_ERR("Unknown sampling frequency\n");
    }

    return ret;
}

static FILE *init_audio_dump_file(uint32_t sample_rate)
{
#ifndef ENABLE_DUMP_AUDIO_FILE
    TRACE_LOG("DISABLE dump audio file\n");
#else
    TRACE_LOG("Enable dump audio file:%d\n", sample_rate);
    switch (sample_rate)
    {
    case 48000:
        return fopen("out_48k.wav", "wb");
        break;

    case 24000:
        return fopen("out_24k.wav", "wb");
        break;

    case 32000:
        return fopen("out_32k.wav", "wb");
        break;

    case 16000:
        return fopen("out_16k.wav", "wb");
        break;
    case 8000:
        return fopen("out_8k.wav", "wb");
        break;

    default:
        WICED_BT_TRACE_CRIT("Unknown sampling frequency\n");
        break;
    }
#endif
    return NULL;
}

static void write_to_audio_dump_file(FILE *audio_dump, uint8_t *audio_data, uint32_t size)
{
#ifdef ENABLE_DUMP_AUDIO_FILE
    if(!rx_audio_file_ptr)
    {
        WICED_BT_TRACE_CRIT("rx_audio_file_ptr is NULL\n");
        return;
    }

    fwrite(audio_data, size, 1, rx_audio_file_ptr);
#endif
}

void audio_driver_init(wiced_bt_isoc_data_path_bit_t dir, uint8_t num_of_channels, uint32_t sample_rate, uint16_t required_latency_ms)
{
    int status;
    WICED_BT_TRACE("[%s] direction %d num_of_channels %d sample_rate %d \n",
                   __FUNCTION__,
                   dir,
                   num_of_channels,
                   sample_rate);

    /* receiving ISO audio data from controller,configure ALSA to play */
    if (dir & WICED_BLE_ISOC_DPD_OUTPUT_BIT)
    {
        // Init ALSA audio
        alsa_audio_init(num_of_channels, sample_rate, required_latency_ms);

        // Init volume driver
        alsa_volume_driver_init();

        // file to dump received audio data
        rx_audio_file_ptr = init_audio_dump_file(sample_rate);
    }

}

void audio_driver_load_wave_file(wiced_bt_isoc_data_path_bit_t dir, uint32_t sample_rate)
{
    /* sending ISO data, open the file to read the audio data to be transmitted */
    if (dir & WICED_BLE_ISOC_DPD_INPUT_BIT)
    {
        if (!read_test_wav_file(sample_rate))
        {
            WICED_BT_TRACE("Unable to read test file\n");
            assert(0);
        }
    }
}

void audio_driver_mic_init(uint8_t num_of_channels, uint32_t sample_rate)
{
    alsa_init_microphone(num_of_channels, sample_rate);
}

void audio_driver_deinit(uint8_t direction)
{
    WICED_BT_TRACE("audio_driver_deinit\n");

    if (p_alsa_handle != NULL)
    {
        WICED_BT_TRACE("snd_pcm_close\n");
        snd_pcm_drain(p_alsa_handle);
        snd_pcm_close(p_alsa_handle);
        p_alsa_handle = NULL;
    }

    if(p_alsa_mic_handle)
    {
        WICED_BT_TRACE("snd_pcm_close p_alsa_mic_handle\n");
        snd_pcm_drain(p_alsa_mic_handle);
        snd_pcm_close(p_alsa_mic_handle);
        p_alsa_mic_handle = NULL;
    }

    WICED_BT_TRACE("rx_audio_file_ptr 0x%lx \n", (uintptr_t)rx_audio_file_ptr);
    if (rx_audio_file_ptr)
    {
        fclose(rx_audio_file_ptr);
        rx_audio_file_ptr = 0;
    }

    alsa_volume_driver_deinit();
}

void audio_driver_write_non_interleaved_data(uint8_t *p_left_data,
                                             uint8_t *p_right_data,
                                             uint8_t bit_width_in_bytes,
                                             uint32_t data_size)
{
#define MAX_SAMPLE_SIZE_SUPPORTED 1920 //480*2*2 48k sampling 2 channels 16 bit sample width

    static uint8_t buff[MAX_SAMPLE_SIZE_SUPPORTED];
    uint8_t *p_interleaved_data = buff;

    if (p_right_data)
    {
        for (uint32_t i = 0; i < data_size; i += bit_width_in_bytes)
        {
            memcpy(p_interleaved_data, p_left_data, bit_width_in_bytes);
            p_interleaved_data += bit_width_in_bytes;

            memcpy(p_interleaved_data, p_right_data, bit_width_in_bytes);
            p_interleaved_data += bit_width_in_bytes;

            p_left_data += bit_width_in_bytes;
            p_right_data += bit_width_in_bytes;
        }

        alsa_write_data(buff, data_size/2); // provide frame size
        write_to_audio_dump_file(rx_audio_file_ptr, buff, (p_interleaved_data - buff));
    }
    else
    {
        alsa_write_data(p_left_data, data_size);
        write_to_audio_dump_file(rx_audio_file_ptr, p_left_data, data_size * bit_width_in_bytes);
    }
}
