/*
 * $ Copyright Cypress Semiconductor $
 */
#include <pthread.h>
#include "asoundlib.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdatomic.h>

#include "audio_driver.h"
#include "audio_parse_wave.h"

#include "log.h"

#undef WICED_BT_TRACE
#undef WICED_BT_TRACE_CRIT
#define WICED_BT_TRACE printf
#define WICED_BT_TRACE_CRIT printf
#define AUDIO_FILE_DUMP_CONNECTION    3
#define ALSA_DEVICE_STR_LENGTH        64 // make it is 2^N
#define PLUG_PREFIX                   4 //means "plug"

typedef struct
{
    uint16_t conn_hdl;
    FILE *audio_file_ptr;
}audio_dump_cb_t;

typedef struct
{
    uint8_t pcm_channels;
    uint8_t pcm_sample_size;
    uint16_t buffer_cap_frames;
    wiced_ble_isoc_data_path_bit_t dir;
}audio_pcm_config_t;


static char *alsa_device_capture_name_i2s   = "I2S-Slave-Device";
static char *alsa_driver_simple_card        = "simple-card";
static char *alsa_driver_usb                = "USB-Audio"; //suppose only one usb headset
static char *alsa_device                    = "default";
static char* alsa_amixer_name[] =   {
                                        "Speaker",
                                        "Master",
                                        "Headphone"
                                    };

audio_pcm_config_t pcm_config;

snd_pcm_t *p_alsa_handle        = NULL;
snd_pcm_t *p_alsa_mic_handle    = NULL; //Microphone
snd_pcm_hw_params_t *mic_params;
snd_pcm_uframes_t mic_frames;
static unsigned int req_mic_channels;

/* file to write the received audio data along with playing over ALSA for
 debugging purposes */
static FILE *rx_audio_file_ptr;
static audio_dump_cb_t audio_dump_cb[AUDIO_FILE_DUMP_CONNECTION];
static FILE *mic_audio_file_ptr      = NULL;
static snd_pcm_uframes_t period_size = 0;

static snd_mixer_elem_t *snd_mixer_elem = NULL;
static snd_mixer_t *snd_mixer_handle = NULL;
static snd_mixer_selem_id_t *snd_sid = NULL;
static long vol_max;

pthread_t pcm_bridge_thread;
atomic_bool pcm_bridge_start = 0;

static int alsa_set_hwparams(snd_pcm_t           *handle,
                             snd_pcm_hw_params_t *params,
                             uint8_t             num_of_channels,
                             uint32_t            sample_rate, 
                             uint8_t             frame_duration,
                             uint16_t            alsa_cfg_latency_ms);

static int alsa_set_mic_hwparams(snd_pcm_t           *handle,
                                 snd_pcm_hw_params_t *params,
                                 uint8_t             num_of_channels,
                                 uint32_t            sample_rate, 
                                 uint8_t             frame_duration,
                                 uint8_t             interleaved_type);

static bool alsa_check_mic_support_mono(snd_pcm_t *handle);
static bool alsa_check_support_channel(snd_pcm_t *handle, uint8_t num_of_channel);


extern int audio_module_get_num_of_samples(void);
extern int audio_module_get_curr_count(void);


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

    if (snd_mixer_handle != NULL)
    {
        TRACE_LOG("[Done] already init, snd_mixer_handle:%p", snd_mixer_handle);
        return;
    }

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
void alsa_list_cards(void)
{
    int card_index = -1;
    bool have_name = false;
    snd_ctl_t *ctl;
    snd_ctl_card_info_t *info;
    snd_ctl_card_info_alloca(&info);

    //iterate all sound cards
    while (snd_card_next(&card_index) >= 0 && card_index >= 0)
    {
        char card_name[ALSA_DEVICE_STR_LENGTH] = {0};
        snprintf(card_name, sizeof(card_name), "hw:%d", card_index);

        //open control interface
        if (snd_ctl_open(&ctl, card_name, 0) < 0)
        {
            continue;
        }

        //get sound card info
        if (snd_ctl_card_info(ctl, info) < 0)
        {
            snd_ctl_close(ctl);
            continue;
        }

        //print card name
        printf("Card %d: %s %s\n", card_index, snd_ctl_card_info_get_driver(info), snd_ctl_card_info_get_name(info));
        snd_ctl_close(ctl);
    }
}

static void alsa_get_device_str_by_driver(char *alsa_driver_target, char *device_str, uint8_t str_len, snd_pcm_stream_t stream_type)
{
    int card_index  = -1;
    int index       = -1;
    int device_num  = -1;
    bool have_name  = false;
    snd_ctl_t *ctl;
    snd_ctl_card_info_t *info;
    snd_pcm_info_t *pcm_info;

    if (alsa_driver_target == NULL)
    {
        TRACE_ERR("alsa_driver_target is NULL");
        return;
    }
    if (device_str == NULL)
    {
        TRACE_ERR("device_str is NULL");
        return;
    }
    snd_ctl_card_info_alloca(&info);
    snd_pcm_info_alloca(&pcm_info);

    //iterate all sound cards
    while (snd_card_next(&card_index) >= 0 && card_index >= 0)
    {
        char card_name[ALSA_DEVICE_STR_LENGTH] = {0};
        snprintf(card_name, sizeof(card_name), "hw:%d", card_index);

        //open control interface
        if (snd_ctl_open(&ctl, card_name, 0) < 0)
        {
            continue;
        }

        //get sound card info
        if (snd_ctl_card_info(ctl, info) < 0)
        {
            snd_ctl_close(ctl);
            continue;
        }

        //print card name
        printf("Card %d: %s %s\n", card_index, snd_ctl_card_info_get_driver(info), snd_ctl_card_info_get_name(info));
        if (strncmp(alsa_driver_target, snd_ctl_card_info_get_driver(info), strlen(alsa_driver_target)) == 0)
        {
            //list all device in sound card
            while (snd_ctl_pcm_next_device(ctl, &device_num) >= 0 && device_num >= 0)
            {
                snd_pcm_info_set_device(pcm_info, device_num);
                snd_pcm_info_set_subdevice(pcm_info, 0);

                //check is placback
                snd_pcm_info_set_stream(pcm_info, SND_PCM_STREAM_PLAYBACK);
                if (snd_ctl_pcm_info(ctl, pcm_info) == 0)
                {
                    printf("  Playback Device %d: %s\n", device_num, snd_pcm_info_get_name(pcm_info));
                    if (stream_type == SND_PCM_STREAM_PLAYBACK)
                    {
                        printf("find %s, device_num:%d\n", alsa_driver_target, device_num);
                        break;
                    }
                }

                //check is capture
                snd_pcm_info_set_stream(pcm_info, SND_PCM_STREAM_CAPTURE);
                if (snd_ctl_pcm_info(ctl, pcm_info) == 0)
                {
                    printf("  Capture Device %d: %s\n", device_num, snd_pcm_info_get_name(pcm_info));
                    if (stream_type == SND_PCM_STREAM_CAPTURE)
                    {
                        printf("find %s, device_num:%d\n", alsa_driver_target, device_num);
                        break;
                    }
                }
            }
            index = card_index;
        }
        snd_ctl_close(ctl);
    }

    if (index < 0)
    {
        TRACE_ERR("No find driver:%s sound card", alsa_driver_target);
        return;
    }
    printf("card_index:%d\n", index);
    if (device_num < 0)
    {
        printf("device_num error:%d\n", device_num);
    }
    snprintf(device_str, str_len, "hw:%d,%d", index, device_num);
    printf("device_str:%s\n", device_str);
}

/*
 * sound card for input
 */
static void alsa_init_microphone(uint8_t num_of_channels, uint32_t sample_rate, char *alsa_driver_target)
{
    int rc  = 0;
    int dir = 0;
    int err = 0;
    int mode = 0;   //0:blocking mode, SND_PCM_NONBLOCK
    char device_str[10] = {0};

    TRACE_LOG("num_of_channels:%d, sample_rate:%d", num_of_channels, sample_rate);

    alsa_get_device_str_by_driver(alsa_driver_target, device_str, sizeof(device_str), SND_PCM_STREAM_CAPTURE);

    TRACE_LOG("device_str:%s", device_str);
    /* Open PCM device for recording (capture). */
    rc = snd_pcm_open(&p_alsa_mic_handle, device_str, SND_PCM_STREAM_CAPTURE, mode);
    if (rc < 0)
    {
        fprintf(stderr, "unable to open pcm device: %s\n", snd_strerror(rc));
        exit(1);
    }

    snd_pcm_hw_params_t *hwparams = {0};
    snd_pcm_hw_params_alloca(&hwparams);

    if (alsa_check_support_channel(p_alsa_mic_handle, num_of_channels) == false)
    {
        TRACE_LOG("enable alsa plug route mic num_of_channels:%d", num_of_channels);
        snd_pcm_close(p_alsa_mic_handle);
        p_alsa_mic_handle = NULL;
        char plug_hw_card_name[ALSA_DEVICE_STR_LENGTH] = {0};

        snprintf(plug_hw_card_name, sizeof(plug_hw_card_name), "plug%s", device_str);

        TRACE_LOG("%s", plug_hw_card_name);

        rc = snd_pcm_open(&p_alsa_mic_handle, plug_hw_card_name, SND_PCM_STREAM_CAPTURE, mode);
    }
    //if MIC support 2 channel, use noninterleave to get mic data
    uint8_t access_type = (num_of_channels == 1) ? SND_PCM_ACCESS_RW_INTERLEAVED : SND_PCM_ACCESS_RW_NONINTERLEAVED;

    int status = alsa_set_mic_hwparams(p_alsa_mic_handle, hwparams, num_of_channels, sample_rate, 10, access_type);
    if (status < 0)
    {
        WICED_BT_TRACE("snd_pcm_set_params failed: %s\n", snd_strerror(status));
    }
}

int wiced_get_audio_data_form_mic(uint8_t *l_data)
{
    int rc = snd_pcm_readi(p_alsa_mic_handle, l_data, mic_frames);
    if (rc == -EPIPE)
    {
        /* EPIPE means overrun */
        fprintf(stderr, "overrun occurred\n");
        snd_pcm_recover(p_alsa_mic_handle, rc, 0);
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

int wiced_get_audio_data_form_mic_noninterleave(uint8_t **data)
{
    int rc = snd_pcm_readn(p_alsa_mic_handle, (void**) data, mic_frames);
    if (rc == -EPIPE)
    {
        /* EPIPE means overrun */
        fprintf(stderr, "overrun occurred\n");
        snd_pcm_recover(p_alsa_mic_handle, rc, 0);
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

/*
 * sound card for ouput
 */
static void alsa_audio_init(uint8_t num_of_channels, uint32_t sample_rate, char *alsa_driver_target, uint16_t alsa_cfg_latency_ms)
{
    snd_pcm_uframes_t buffer_size                         = 0;
    char device_str[ALSA_DEVICE_STR_LENGTH - PLUG_PREFIX] = {0};
    char plug_hw_device_name[ALSA_DEVICE_STR_LENGTH]      = {0};
    uint8_t mode                                          = SND_PCM_NONBLOCK; //0 is BLOCKING MODE
    int rc                                                = 0;

    TRACE_LOG("num_of_channels:%d, sample_rate:%d, alsa_cfg_latency_ms:%d\n", num_of_channels, sample_rate, alsa_cfg_latency_ms);

    if (p_alsa_handle != NULL)
    {
        TRACE_LOG("p_alsa_handle not NULL, already init");
        snd_pcm_get_params(p_alsa_handle, &buffer_size, &period_size);
        WICED_BT_TRACE("snd_pcm_get_params buffersize %lu frames, periodsize %lu frames\n", buffer_size, period_size);
        WICED_BT_TRACE("snd_pcm_get_params periodsize %lu bytes\n", period_size * 4 /*NUM_CHANNELS*SAMPLE_SIZE*/);
        return;
    }

    alsa_get_device_str_by_driver(alsa_driver_target, device_str, sizeof(device_str), SND_PCM_STREAM_PLAYBACK);

    WICED_BT_TRACE("snd_pcm_open:%s\n", mode == SND_PCM_NONBLOCK ? "SND_PCM_NONBLOCK":"BLOCK MODE");

    int len = snprintf(plug_hw_device_name, ALSA_DEVICE_STR_LENGTH, "plug%s", device_str);
    if (len < 0) 
    {
        WICED_BT_TRACE("snprintf error\n");
    }
    else if (len >= ALSA_DEVICE_STR_LENGTH) 
    {
        WICED_BT_TRACE("WARNING: plug_hw_device_name truncated (len=%d)\n", len);
    }
    TRACE_LOG("Using ALSA plug device: %s", plug_hw_device_name);

    int status =
        snd_pcm_open(&p_alsa_handle, plug_hw_device_name, SND_PCM_STREAM_PLAYBACK, mode);

    if (status < 0)
    {
        WICED_BT_TRACE_CRIT("snd_pcm_open failed: %s\n", snd_strerror(status));
        return;
    }

    /* Configure ALSA driver with PCM parameters */
#ifndef ALSA_SETUP_PERIODSIZE   //setup param auto
    status = snd_pcm_set_params(p_alsa_handle,
                                SND_PCM_FORMAT_S16_LE,
                                SND_PCM_ACCESS_RW_INTERLEAVED,
                                num_of_channels,
                                sample_rate,
                                1,
                                alsa_cfg_latency_ms * 1000);

#else   //setup param for period size
    snd_pcm_hw_params_t *hwparams = {0};
    snd_pcm_hw_params_alloca(&hwparams);
    status = alsa_set_hwparams(p_alsa_handle, hwparams, num_of_channels, sample_rate, 10, alsa_cfg_latency_ms);
#endif

    if (status < 0)
    {
        WICED_BT_TRACE("snd_pcm_set_params failed: %s\n", snd_strerror(status));
    }

    snd_pcm_prepare(p_alsa_handle);

    snd_pcm_get_params(p_alsa_handle, &buffer_size, &period_size);
    WICED_BT_TRACE("snd_pcm_get_params buffersize %lu frames, periodsize %lu frames\n", buffer_size, period_size);
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
        //NONBLOCK MODE ONLY
        if (ret == -EAGAIN) {
            // Buffer is full; handle this case (e.g., wait or skip)
            TRACE_ERR("buffer is full");
            return;
        }

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

static FILE *init_audio_dump_file_by_conn_hdl(uint32_t sample_rate, uint16_t conn_hdl)
{
#ifndef ENABLE_DUMP_AUDIO_FILE_CONN_HDL
    TRACE_LOG("DISABLE dump audio file\n");
#else
    char output_file[50] = "out_";
    char str_conn_hdl[5] = {0};
    TRACE_LOG("Enable dump audio file:%d\n", sample_rate);
    switch (sample_rate)
    {
    case 48000:
        strcat(output_file, "48k");
        break;

    case 24000:
        strcat(output_file, "24k");
        break;

    case 32000:
        strcat(output_file, "32k");
        break;

    case 16000:
        strcat(output_file, "16k");
        break;
    case 8000:
        strcat(output_file, "8k");
        break;

    default:
        WICED_BT_TRACE_CRIT("Unknown sampling frequency, no dump file\n");
        return NULL;
    }

    strcat(output_file, "_");

    snprintf(str_conn_hdl, sizeof(str_conn_hdl), "%x", conn_hdl);

    strcat(output_file, str_conn_hdl);

    strcat(output_file, ".wav");

    TRACE_LOG("output_file:%s", output_file);
    return fopen(output_file, "wb");
#endif
    return NULL;
}

static FILE *init_cap_audio_dump_file(uint32_t sample_rate)
{
#ifndef ENABLE_DUMP_CAP_AUDIO_FILE
    TRACE_LOG("DISABLE dump cap audio file\n");
#else
    TRACE_LOG("Enable dump cap audio file:%d\n", sample_rate);
    switch (sample_rate)
    {
    case 48000:
        return fopen("mic_48k.wav", "wb");
        break;

    case 24000:
        return fopen("mic_24k.wav", "wb");
        break;

    case 32000:
        return fopen("mic_32k.wav", "wb");
        break;

    case 16000:
        return fopen("mic_16k.wav", "wb");
        break;
    case 8000:
        return fopen("mic_8k.wav", "wb");
        break;

    default:
        WICED_BT_TRACE_CRIT("Unknown sampling frequency\n");
        break;
    }
#endif
    return NULL;
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

static void write_to_mic_dump_file(FILE *audio_dump, uint8_t *audio_data, uint32_t size)
{
#ifdef ENABLE_DUMP_CAP_AUDIO_FILE
    if(!mic_audio_file_ptr)
    {
        WICED_BT_TRACE_CRIT("mic_audio_file_ptr is NULL\n");
        return;
    }

    fwrite(audio_data, size, 1, mic_audio_file_ptr);
#endif
}

static void write_to_audio_dump_file(FILE *audio_dump_ptr, uint8_t *audio_data, uint32_t size)
{
#ifdef ENABLE_DUMP_AUDIO_FILE
    if(!audio_dump_ptr)
    {
        WICED_BT_TRACE_CRIT("rx_audio_file_ptr is NULL\n");
        return;
    }

    fwrite(audio_data, size, 1, audio_dump_ptr);
#endif
}

void audio_driver_init(wiced_ble_isoc_data_path_bit_t dir, uint8_t num_of_channels, uint32_t sample_rate, uint16_t required_latency_ms)
{
    int status;
    WICED_BT_TRACE("[%s] direction %d num_of_channels %d sample_rate %d \n",
                   __FUNCTION__,
                   dir,
                   num_of_channels,
                   sample_rate);

    TRACE_LOG("snd_mixer_handle:%p %p", snd_mixer_handle, &snd_mixer_handle);

    /* receiving ISO audio data from controller,configure ALSA to play */
    // Init ALSA audio
    alsa_audio_init(num_of_channels, sample_rate, alsa_driver_usb, required_latency_ms);

    // Init volume driver
    alsa_volume_driver_init();

    // file to dump received audio data
    rx_audio_file_ptr = init_audio_dump_file(sample_rate);
}

void audio_driver_load_wave_file(wiced_ble_isoc_data_path_bit_t dir, uint32_t sample_rate)
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

/*
 * test use, currently
 */
void audio_driver_mic_init(uint8_t num_of_channels, uint32_t sample_rate)
{
    alsa_init_microphone(num_of_channels, sample_rate, alsa_driver_usb);

    mic_audio_file_ptr = init_cap_audio_dump_file(sample_rate);
}

void audio_driver_deinit(uint8_t direction)
{
    WICED_BT_TRACE("audio_driver_deinit\n");

    if (p_alsa_handle != NULL)
    {
        WICED_BT_TRACE("snd_pcm_close: p_alsa_handle\n");
        snd_pcm_drop(p_alsa_handle);
        snd_pcm_close(p_alsa_handle);
        p_alsa_handle = NULL;
    }

    if(p_alsa_mic_handle)
    {
        WICED_BT_TRACE("snd_pcm_close: p_alsa_mic_handle\n");
        //int ret = snd_pcm_drain(p_alsa_mic_handle); //kernel 6.1 blocking, for capture no need to use drain
        snd_pcm_close(p_alsa_mic_handle);
        p_alsa_mic_handle = NULL;
    }

    if (rx_audio_file_ptr)
    {
        fclose(rx_audio_file_ptr);
        rx_audio_file_ptr = 0;
    }
    alsa_volume_driver_deinit();
}

/*
 * TBD, not complete yet
 */
void audio_driver_write_data(uint8_t *p_left_data,
                             uint8_t *p_right_data,
                             uint8_t bit_width_in_bytes,
                             uint32_t data_size,
                             wiced_bool_t interleaved)
{
#define MAX_SAMPLE_SIZE_SUPPORTED 1920 //480*2*2 48k sampling 2 channels 16 bit sample width

    static uint8_t buff[MAX_SAMPLE_SIZE_SUPPORTED] = {0};
    uint8_t *p_interleaved_data = buff;

    if (interleaved)
    {
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
    else
    {
        alsa_write_data(p_left_data, data_size/2);
        alsa_write_data(p_right_data, data_size/2);
    }
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

static unsigned int buffer_time = 500000;       /* ring buffer length in us */
static unsigned int period_time = 100000;       /* period time in us */
static int alsa_set_hwparams(snd_pcm_t           *handle,
                             snd_pcm_hw_params_t *params,
                             uint8_t             num_of_channels,
                             uint32_t            sample_rate, 
                             uint8_t             frame_duration,
                             uint16_t            alsa_cfg_latency_ms)
{
    unsigned int    rrate;
    snd_pcm_uframes_t size;
    int             err = 0, dir = 0;

    buffer_time = alsa_cfg_latency_ms * 1000;
    period_time = buffer_time / 4; //TBD: small period_time is better for low latency 

    TRACE_LOG("\n");
    /* choose all parameters */
    err = snd_pcm_hw_params_any(handle, params);
    if (err < 0) {
        printf("Broken configuration for playback: no configurations available: %s\n", snd_strerror(err));
        return err;
    }
    /* set hardware resampling */
    err = snd_pcm_hw_params_set_rate_resample(handle, params, 1);
    if (err < 0) {
        printf("Resampling setup failed for playback: %s\n", snd_strerror(err));
        return err;
    }
    /* set the interleaved read/write format */
    err = snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    if (err < 0) {
        printf("Access type not available for playback: %s\n", snd_strerror(err));
        return err;
    }
    /* set the sample format */
	err = snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
    if (err < 0) {
        printf("Sample format not available for playback: %s\n", snd_strerror(err));
        return err;
    }
    /* set the count of channels */
    err = snd_pcm_hw_params_set_channels(handle, params, num_of_channels);
    if (err < 0) {
        printf("Channels count (%u) not available for playbacks: %s\n", num_of_channels, snd_strerror(err));
        return err;
    }
    /* set the stream rate */
    rrate = sample_rate;
    err = snd_pcm_hw_params_set_rate_near(handle, params, &rrate, 0);
    if (err < 0) {
        printf("Rate %uHz not available for playback: %s\n", sample_rate, snd_strerror(err));
        return err;
    }
    if (rrate != sample_rate) {
        printf("Rate doesn't match (requested %uHz, get %dHz)\n", sample_rate, rrate);
        return -EINVAL;
    }
    /* set the buffer time */
    err = snd_pcm_hw_params_set_buffer_time_near(handle, params, &buffer_time, &dir);
    if (err < 0) {
        printf("Unable to set buffer time %u for playback: %s\n", buffer_time, snd_strerror(err));
        return err;
    }
    err = snd_pcm_hw_params_get_buffer_size(params, &size);
    if (err < 0) {
        printf("Unable to get buffer size for playback: %s\n", snd_strerror(err));
        return err;
    }
    /* set the period time */
    err = snd_pcm_hw_params_set_period_time_near(handle, params, &period_time, &dir);
    if (err < 0) {
        printf("Unable to set period time %u for playback: %s\n", period_time, snd_strerror(err));
        return err;
    }
    err = snd_pcm_hw_params_get_period_size(params, &size, &dir);
    if (err < 0) {
        printf("Unable to get period size for playback: %s\n", snd_strerror(err));
        return err;
    }
    period_size = size;
    TRACE_LOG("size:%lu, period_size:%lu\n", size, period_size);

    /* write the parameters to device */
    err = snd_pcm_hw_params(handle, params);
    if (err < 0) {
        printf("Unable to set hw params for playback: %s\n", snd_strerror(err));
        return err;
    }

    TRACE_LOG("done\n");
    return 0;
}

static bool alsa_check_mic_support_mono(snd_pcm_t *handle)
{
    int err                     = 0;
    unsigned int min_channels   = 0;
    snd_pcm_hw_params_t *hwparams = {0};

    snd_pcm_hw_params_alloca(&hwparams);

    err = snd_pcm_hw_params_any(handle, hwparams);

    //check hw support channel range
    snd_pcm_hw_params_get_channels_min(hwparams, &min_channels);

    return min_channels == 1 ? true : false;
}

static bool alsa_check_support_channel(snd_pcm_t *handle, uint8_t num_of_channel)
{
    int err                     = 0;
    unsigned int min_channels   = 0;
    unsigned int max_channels   = 0;
    snd_pcm_hw_params_t *hwparams = {0};

    snd_pcm_hw_params_alloca(&hwparams);

    err = snd_pcm_hw_params_any(handle, hwparams);

    //check hw support channel min
    snd_pcm_hw_params_get_channels_min(hwparams, &min_channels);
    //check hw support channel max
    snd_pcm_hw_params_get_channels_max(hwparams, &max_channels);

    TRACE_LOG("min_channels:%d, max_channels:%d, num_of_channel:%d", min_channels, max_channels, num_of_channel);

    return snd_pcm_hw_params_test_channels(handle, hwparams, num_of_channel) == 0 ? true : false;
}

/*
 * headset mic hw param init  
 * use period_size and buffer_size to setup, period_size calculate by frame_duration 
 */
static int alsa_set_mic_hwparams(snd_pcm_t           *handle,
                                 snd_pcm_hw_params_t *params,
                                 uint8_t             num_of_channels,
                                 uint32_t            sample_rate, 
                                 uint8_t             frame_duration,
                                 uint8_t             interleaved_type)
{
    unsigned int    rrate = (unsigned int)sample_rate;
    int err                     = 0;
    int dir                     = 0;
    snd_pcm_uframes_t size      = 0;
    unsigned int buffer_time    = 500000;       /* ring buffer length in us */
    unsigned int period_time    = 100000;       /* period time in us */
    unsigned long buffer_size   = 500000;       /* ring buffer length in us */
    snd_pcm_uframes_t mic_period_size = 0;

    req_mic_channels            = (unsigned int)num_of_channels;

    mic_period_size = (sample_rate / (1000 / frame_duration)); //or use get_frame_size api, frame_duration in ms
    buffer_size = mic_period_size * 4;

    TRACE_LOG("mic_period_size:%u, buffer_size:%u", mic_period_size, buffer_size);
    /* choose all parameters */
    err = snd_pcm_hw_params_any(handle, params);
    if (err < 0) {
        printf("Broken configuration for capture: no configurations available: %s\n", snd_strerror(err));
        return err;
    }

    /* set hardware resampling */
    err = snd_pcm_hw_params_set_rate_resample(handle, params, 1);
    if (err < 0) {
        printf("Resampling setup failed for capture: %s\n", snd_strerror(err));
        return err;
    }
    /* set the interleaved read/write format */
    err = snd_pcm_hw_params_set_access(handle, params, interleaved_type);
    if (err < 0) {
        printf("Access type not available for capture: %s\n", snd_strerror(err));
        return err;
    }
    /* set the sample format */
	err = snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
    if (err < 0) {
        printf("Sample format not available for playback: %s\n", snd_strerror(err));
        return err;
    }

    /* set the count of channels */
    err = snd_pcm_hw_params_set_channels(handle, params, req_mic_channels);
    if (err < 0) {
        printf("Channels count (%u) not available for capture: %s\n", req_mic_channels, snd_strerror(err));
        return err;
    }
    /* set the stream rate */
    err = snd_pcm_hw_params_set_rate_near(handle, params, &rrate, 0);
    if (err < 0) {
        printf("Rate %uHz not available for playback: %s\n", sample_rate, snd_strerror(err));
        return err;
    }
    if (rrate != sample_rate) {
        printf("Rate doesn't match (requested %uHz, get %iHz)\n", sample_rate, err);
        return -EINVAL;
    }

    /* set the buffer size */
    err = snd_pcm_hw_params_set_buffer_size_near(handle, params, &buffer_size);
    if (err < 0) {
        printf("Unable to set buffer size %lu for capture: %s\n", buffer_size, snd_strerror(err));
        return err;
    }
    err = snd_pcm_hw_params_get_buffer_size(params, &size);
    if (err < 0) {
        printf("Unable to get buffer size for playback: %s\n", snd_strerror(err));
        return err;
    }
    TRACE_LOG("buffer size:%ld", size);

    /* set the mic period size */
    err = snd_pcm_hw_params_set_period_size_near(handle, params, &mic_period_size, &dir);
    if (err < 0) {
        printf("Unable to set period size %lu for capture: %s\n", mic_period_size, snd_strerror(err));
        return err;
    }
    err = snd_pcm_hw_params_get_period_size(params, &size, &dir);
    if (err < 0) {
        printf("Unable to get period size for capture: %s\n", snd_strerror(err));
        return err;
    }

    mic_period_size = size;
    mic_frames = mic_period_size;
    TRACE_LOG("size:%lu, mic_period_size:%lu, mic_frames:%lu\n", size, mic_period_size, mic_frames);

    /* write the parameters to device */
    err = snd_pcm_hw_params(handle, params);
    if (err < 0) {
        printf("Unable to set hw params for playback: %s\n", snd_strerror(err));
        return err;
    }

    TRACE_LOG("done\n");
    return 0;
}

void audio_driver_write_mic_data(uint8_t *p_data,
                                 uint8_t bit_width_in_bytes,
                                 uint32_t data_size)
{
    write_to_mic_dump_file(mic_audio_file_ptr, p_data, data_size);
}

void audio_driver_set_mic_gain(int32_t gain)
{

}

void audio_driver_set_mic_mute_state(uint8_t mute, int32_t gain)
{

}

/*
 * pcm audio bridge to usb headset
 */
void *audio_driver_pcm_bridge_loop(void *arg)
{
    //pcm frames
    //Buffer Size = frames × channel s× sample size in bytes
    uint16_t buffer_size = 0;
    if (arg == NULL)
    {
        TRACE_ERR("arg is NULL");
        return NULL;
    }
    audio_pcm_config_t *p_pcm_config = (audio_pcm_config_t*)arg;

    TRACE_LOG("p_pcm_config->buffer_cap_frames:%d, p_pcm_config->pcm_channels:%d, p_pcm_config->pcm_sample_size:%d", p_pcm_config->buffer_cap_frames, p_pcm_config->pcm_channels, p_pcm_config->pcm_sample_size);

    uint16_t read_buffer_size = p_pcm_config->buffer_cap_frames * p_pcm_config->pcm_channels * p_pcm_config->pcm_sample_size;
    uint16_t write_buffer_size = p_pcm_config->buffer_cap_frames * 2 * p_pcm_config->pcm_sample_size;
    TRACE_LOG("read buffer_size:%d", read_buffer_size);
    TRACE_LOG("write buffer_size:%d", write_buffer_size);

    uint16_t *p_buf          = calloc(read_buffer_size/2, sizeof(uint16_t));
    uint16_t *p_stero_buf    = calloc(write_buffer_size/2, sizeof(uint16_t));

    int err                 = 0;

    if (p_buf == NULL)
    {
        TRACE_ERR("allocate buffer_size fail");
        return NULL;
    }

    pcm_bridge_start = 1;

    while (pcm_bridge_start)
    {
        //Controller I2S out
        //if (p_pcm_config->dir == WICED_BLE_ISOC_DPD_OUTPUT_BIT)
        {
            //read data from input sound card device pcm/i2s or usb
            err = snd_pcm_readi(p_alsa_mic_handle, p_buf, p_pcm_config->buffer_cap_frames);
            if (err < 0)
            {
                if (err == -EPIPE)
                {
                    snd_pcm_recover(p_alsa_mic_handle, err, 0);
                }
                else if (err == -EAGAIN)
                {
                    continue;
                }
            }
            else //success read frames
            {
#ifdef ENABLE_DUMP_CAP_AUDIO_FILE
                audio_driver_write_mic_data((uint8_t *)p_buf, p_pcm_config->pcm_sample_size, buffer_size);
#endif
                //write to output device
                if (p_alsa_handle == NULL) break;

                //TODO: check sound card support 1 channel or not
                if (p_pcm_config->pcm_channels == 1)
                {
                    for (int i = 0; i < p_pcm_config->buffer_cap_frames; i++)
                    {
                        //interleve
                        p_stero_buf[2 *i] = p_buf[i];
                        p_stero_buf[2* i+1] = p_buf[i];
                    }
                    err = snd_pcm_writei(p_alsa_handle, p_stero_buf, err);
                }
                else
                {
                    err = snd_pcm_writei(p_alsa_handle, p_buf, err);
                }
                if (err < 0)
                {
                    //fprintf(stderr, "write err: %s\n", snd_strerror(err));
                    snd_pcm_recover(p_alsa_handle, err, 0);
                }
            }
        }
    }
    TRACE_LOG("end");
    free(p_buf);
    free(p_stero_buf);

    return NULL;
}

/*
 * init pcm interface as capture device, usb headset as playback
 */
void audio_driver_init_pcm(wiced_ble_isoc_data_path_bit_t dir, uint8_t num_of_channels, uint32_t sample_rate, uint16_t required_latency_ms)
{
    int status;
    TRACE_LOG("[%s] direction %d num_of_channels %d sample_rate %d \n",
                   __FUNCTION__,
                   dir,
                   num_of_channels,
                   sample_rate);
    pcm_config.pcm_channels         = num_of_channels;
    pcm_config.pcm_sample_size      = 2;     //sample width in byte
    pcm_config.buffer_cap_frames    = 1024;  //pcm frames
    pcm_config.dir                  = dir;  //pcm frames

    if (pcm_bridge_start == 1)
    {
        TRACE_ERR("pcm_bridge_thread is running");
        return;
    }

#ifdef ENABLE_DUMP_CAP_AUDIO_FILE
    mic_audio_file_ptr = init_cap_audio_dump_file(sample_rate);
#endif

    /* receiving ISO audio data from controller,configure ALSA to play */
    if (dir & WICED_BLE_ISOC_DPD_OUTPUT_BIT)
    {
        // Init ALSA audio in
        //I2S in
        alsa_init_microphone(num_of_channels, sample_rate, alsa_driver_simple_card);
        pcm_config.buffer_cap_frames = mic_frames;

        //usb out
        alsa_audio_init(num_of_channels, sample_rate, alsa_driver_usb, required_latency_ms);

        // file to dump received audio data
        rx_audio_file_ptr = init_audio_dump_file(sample_rate);
        pthread_create(&pcm_bridge_thread, NULL, audio_driver_pcm_bridge_loop, (void *)&pcm_config);
    }
    else if (dir & WICED_BLE_ISOC_DPD_INPUT_BIT)
    {
        //usb in, in Host view
        alsa_init_microphone(num_of_channels, sample_rate, alsa_driver_usb);

        //I2S out, in Host view
        alsa_audio_init(num_of_channels, sample_rate, alsa_driver_simple_card, required_latency_ms);

        // file to dump received audio data
        //rx_audio_file_ptr = init_audio_dump_file(sample_rate);
        pthread_create(&pcm_bridge_thread, NULL, audio_driver_pcm_bridge_loop, (void *)&pcm_config);
    }

}

void audio_driver_pcm_deinit(uint8_t direction)
{
    TRACE_LOG("");

    pcm_bridge_start = 0;

    audio_driver_deinit(direction);

    TRACE_LOG("wait pcm_bridge_thread end");

    pthread_join(pcm_bridge_thread, NULL);

    TRACE_LOG("pcm_bridge_thread end");


}

#define MAX_INPUT_SAMPLE_SIZE_IN_BYTES 480 * 2 //48khz @ 10ms interval, sampel_width 2,
/*
 * test headset plackback, play local file
 */
void audio_driver_playback_test(uint32_t sample_rate, uint16_t frame_duration)
{
                                               //
    uint8_t wav_data_l[MAX_INPUT_SAMPLE_SIZE_IN_BYTES] = {0};
    uint8_t wav_data_r[MAX_INPUT_SAMPLE_SIZE_IN_BYTES] = {0};
    int req_num_samples     = 0;
    uint32_t count          = 2960;
    uint8_t num_of_channels = 2;
    int total_num_samples   = 0;
    int curr_count          = 0;
    double wait_time        = 0;

    audio_driver_load_wave_file(WICED_BLE_ISOC_DPD_INPUT_BIT, sample_rate); 

    total_num_samples = audio_module_get_num_of_samples();
    curr_count  = audio_module_get_curr_count();

    audio_driver_set_volume(30);

    while(count--)
    {
        memset(wav_data_l, 0, MAX_INPUT_SAMPLE_SIZE_IN_BYTES);
        memset(wav_data_r, 0, MAX_INPUT_SAMPLE_SIZE_IN_BYTES);
        req_num_samples = audio_module_get_wave_data(wav_data_l, wav_data_r, frame_duration);

        audio_driver_write_data(wav_data_l, wav_data_r, 2, num_of_channels * req_num_samples, 1);
        wait_time = (req_num_samples / (double)sample_rate);
        TRACE_LOG("wait_time:%f", wait_time);
        usleep(wait_time * 1000000);
        curr_count  = audio_module_get_curr_count();
        TRACE_LOG("curr_count:%d, num_samples:%d", curr_count, total_num_samples);
    }
}

/*
 * test headset mic input, dump to file
 */
void audio_driver_mic_test()
{
    int num_of_samples        = 0;
    int sample_width_in_bytes = 2;
    int rc                    = 0;
    uint32_t cap_data_size    = mic_frames * sample_width_in_bytes * req_mic_channels;

    TRACE_LOG("req_mic_channel:%d, mic_frames:%ld", req_mic_channels, mic_frames);

    uint8_t *wav_data = calloc(cap_data_size, sizeof(uint8_t));
    if (wav_data == NULL)
    {
        TRACE_ERR("calloc fail");
        return;
    }

    while (1)
    {
        rc = snd_pcm_readi(p_alsa_mic_handle, wav_data, mic_frames);
        if (rc == -EPIPE)
        {
            /* EPIPE means overrun */
            fprintf(stderr, "overrun occurred\n");
            snd_pcm_recover(p_alsa_mic_handle, rc, 0);
        }
        else if (rc < 0)
        {
            fprintf(stderr, "error from read: %s\n", snd_strerror(rc));
        }
        else if (rc != (int)mic_frames)
        {
            fprintf(stderr, "short read, read %d mic_frames\n", rc);
        }
        else
        {
            TRACE_LOG("cap num_of_samples:%d", rc);
        }

#ifdef ENABLE_DUMP_CAP_AUDIO_FILE
        audio_driver_write_mic_data(wav_data, sample_width_in_bytes, cap_data_size);
#else
        printf("ENABLE_DUMP_AUDIO_FILE to dump cap mic data\n");
#endif
    }

    free(wav_data);
    wav_data = NULL;
}

/*
 * for test alsa headset mic input bridge to itselt playback
 */
void audio_driver_mic_bridge_usb_headset(void)
{
    int sample_width_in_bytes   = 2;
    int rc                      = 0;
    uint8_t *wav_data           = NULL;
    uint32_t cap_data_size      = mic_frames * sample_width_in_bytes * req_mic_channels;
    uint32_t left_channel_size  = mic_frames * sample_width_in_bytes;
    uint8_t headset_playback_channels = 2;

    TRACE_LOG("req_mic_channels:%d, mic_frames:%ld, cap_data_size:%ld", req_mic_channels, mic_frames, cap_data_size);

    wav_data    = calloc(cap_data_size, sizeof(uint8_t));

    if (wav_data == NULL)
    {
        TRACE_ERR("calloc fail");
        return;
    }

    while (1)
    {
        rc = snd_pcm_readi(p_alsa_mic_handle, wav_data, mic_frames);
        if (rc == -EPIPE)
        {
            /* EPIPE means overrun */
            fprintf(stderr, "overrun occurred\n");
            snd_pcm_recover(p_alsa_mic_handle, rc, 0);
        }
        else if (rc < 0)
        {
            fprintf(stderr, "error from read: %s\n", snd_strerror(rc));
        }
        else if (rc != (int)mic_frames)
        {
            fprintf(stderr, "short read, read %d mic_frames\n", rc);
            continue;
        }
        else
        {
            TRACE_LOG("cap num of samples:%d", rc);
        }

#ifdef ENABLE_DUMP_CAP_AUDIO_FILE
        audio_driver_write_mic_data(wav_data, sample_width_in_bytes, cap_data_size);
#else
        printf("ENABLE_DUMP_AUDIO_FILE to save capture mic data\n");
#endif
        if (req_mic_channels == 1)
        {
            //headset playback is 2 channel
            audio_driver_write_data(wav_data, wav_data, sample_width_in_bytes, headset_playback_channels * mic_frames, 1);
        }
        else
        {
            alsa_write_data(wav_data, cap_data_size/2); // provide frame size
        }
        //write_to_audio_dump_file(rx_audio_file_ptr, buff, (p_interleaved_data - buff));
    }

    free(wav_data);
    wav_data = NULL;
}

void audio_driver_init_dump_file_by_conn_hdl(uint32_t sample_rate, uint16_t conn_hdl)
{
#ifdef ENABLE_DUMP_AUDIO_FILE_CONN_HDL
    uint8_t index = 0;

    for (index = 0; index < AUDIO_FILE_DUMP_CONNECTION; index++)
    {
        if (audio_dump_cb[index].conn_hdl == 0 && audio_dump_cb[index].audio_file_ptr == NULL)
        {
            TRACE_LOG("index:%d", index);
            break;
        }
    }
    if (index == AUDIO_FILE_DUMP_CONNECTION)
    {
        TRACE_ERR("init dump file fail, index limit or file_ptr not NULL, index:%d", index);
        return;
    }

    audio_dump_cb[index].audio_file_ptr = init_audio_dump_file_by_conn_hdl(sample_rate, conn_hdl);

    if (audio_dump_cb[index].audio_file_ptr != NULL)
    {
        audio_dump_cb[index].conn_hdl = conn_hdl;
    }
#endif
}

void audio_driver_dump_file_by_conn_hdl(uint16_t conn_hdl, uint8_t *p_data, uint32_t size)
{
    uint8_t index = 0;
    for (index = 0; index < AUDIO_FILE_DUMP_CONNECTION; index++)
    {
        if (audio_dump_cb[index].conn_hdl == conn_hdl && audio_dump_cb[index].audio_file_ptr != NULL)
        {
            TRACE_LOG("index:%d", index);
            break;
        }
    }
    if (index == AUDIO_FILE_DUMP_CONNECTION)
    {
        TRACE_ERR("no find match conn_hdl or file_ptr is NULL, conn_hdl:0x%x", conn_hdl);
        return;
    }

    write_to_audio_dump_file(audio_dump_cb[index].audio_file_ptr, p_data, size);
}


