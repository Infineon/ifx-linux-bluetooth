/*
* $ Copyright Cypress Semiconductor $
*/
#ifndef ISOC_OFFLOAD
#include "lepl.h"
#include "iso_data_handler.h"
#include "lc3_codec.h"
#include "audio_driver.h"
#include "audio_parse_wave.h"
#ifdef WIN32
#include "windows.h"
#else
#include <pthread.h>
#include "log.h"
#endif
#include "le_isoc.h"

extern void BTU_stack_lock(void);
extern void BTU_stack_unlock(void);

#define MAX_INPUT_SAMPLE_SIZE_IN_BYTES 480 * 2 //48khz @ 10ms interval
#define MAX_STREAMS_SUPPORTED 2
#define QUEUE_BUFFER_SIZE 50
#define MAX_BUFFER_PER_CIS 20
#define SDU_SIZE 240
#define CHANNEL_COUNT 2
wiced_bt_pool_t *iso_audio_pool = NULL;
#ifdef WIN32
HANDLE ghEvents[1];
HANDLE mRxThread = NULL;
#else
pthread_t ptid;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t lock1 = PTHREAD_MUTEX_INITIALIZER;
#endif
wiced_bt_lock_t queue_lock;

typedef struct
{
    uint16_t conn_hdl;
    uint16_t octets_per_frame;
    uint8_t num_of_channels;
    wiced_bool_t b_stream_active;
    uint32_t frame_duration;
    uint32_t sampling_frequency;
    uint16_t psn;
    uint8_t audio_location;
    wiced_ble_isoc_data_path_bit_t dpd_bits;
} ga_iso_audio_stream_info_t;

typedef enum
{
    GA_ISOC_STATE_IDLE,
    GA_ISOC_STATE_MEDIA_STREAMING,
    GA_ISOC_STATE_MIC
} ga_isoc_state_t;

typedef struct
{
    uint8_t stream_count;
    uint8_t is_multiplexed;
    uint16_t conn_hdl_list[MAX_STREAMS_SUPPORTED];
    ga_iso_audio_stream_info_t *p_stream_info;
} ga_iso_stream_data_t;

typedef struct
{
    ga_isoc_state_t state;
    wiced_bt_buffer_q_t isoc_data_q;
} ga_iso_q_data_t;

static ga_iso_q_data_t q_data;
static ga_iso_audio_stream_info_t g_stream_info[MAX_STREAMS_SUPPORTED] = {0};
static ga_iso_stream_data_t stream_data;

extern int wiced_get_audio_data_form_mic(uint8_t *l_data);
static void rx_handler_voice_assist(uint16_t conn_hdl, uint8_t *p_data, uint32_t length);

extern lepl_ase_data_t *lepl_get_remote_ase_data_by_ase_id(
    lepl_clcb_t *p_clcb,
    uint8_t ase_id);

static void iso_audio_set_state(ga_isoc_state_t state)
{
    q_data.state = state;
}

static ga_isoc_state_t iso_audio_get_state()
{
    return q_data.state;
}

static void iso_audio_lock(void *p_lock_context)
{
    BTU_stack_lock();
}

static void iso_audio_unlock(void *p_lock_context)
{
    BTU_stack_unlock();
}

static uint32_t count_set_bits(uint32_t n)
{
    uint32_t count = 0;
    while (n)
    {
        n &= (n - 1);
        count++;
    }
    return count;
}

static void init_stream_info(uint16_t conn_hdl, wiced_bt_ga_bap_csc_t *p_csc, wiced_ble_isoc_data_path_bit_t dir)
{
    uint32_t numOfChannels = 1;
    /* determine num of channels required*/
    if (p_csc->audio_channel_allocation)
    {
        numOfChannels = count_set_bits(p_csc->audio_channel_allocation);
        if (numOfChannels > ISO_AUDIO_MAX_PARAM_COUNT)
        {
            return;
        }
    }
    WICED_BT_TRACE("[%s] [SF %d] [OPF %d] [FD %d] [num_of_channels %d]\n",
                   __FUNCTION__,
                   p_csc->sampling_frequency,
                   p_csc->octets_per_codec_frame,
                   p_csc->frame_duration,
                   numOfChannels);

    for (size_t i = 0; i < MAX_STREAMS_SUPPORTED; i++)
    {
        if (!g_stream_info[i].conn_hdl)
        {
            g_stream_info[i].conn_hdl = conn_hdl;
            g_stream_info[i].octets_per_frame = p_csc->octets_per_codec_frame;
            g_stream_info[i].num_of_channels = numOfChannels;
            g_stream_info[i].sampling_frequency = p_csc->sampling_frequency;
            g_stream_info[i].frame_duration = p_csc->frame_duration;
            g_stream_info[i].b_stream_active = 1;
            g_stream_info[i].audio_location = p_csc->audio_channel_allocation;
            g_stream_info[i].dpd_bits |=
                (dir == WICED_BLE_ISOC_DPD_INPUT) ? WICED_BLE_ISOC_DPD_INPUT_BIT : WICED_BLE_ISOC_DPD_OUTPUT_BIT;

            WICED_BT_TRACE("[%s] conn_hdl 0x%x \n", __FUNCTION__, conn_hdl);
            return;
        }
    }
}

static ga_iso_audio_stream_info_t *get_stream_info(uint16_t conn_hdl)
{
    for (size_t i = 0; i < MAX_STREAMS_SUPPORTED; i++)
    {
        if (conn_hdl == g_stream_info[i].conn_hdl)
        {
            return &g_stream_info[i];
        }
    }

    return NULL;
}

static void deinit_stream_info(uint16_t conn_hdl)
{
    WICED_BT_TRACE("[%s] [conn_hdl %d]\n", __FUNCTION__, conn_hdl);

    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);
    if (!p_stream_info) return;

    memset(p_stream_info, 0, sizeof(ga_iso_audio_stream_info_t));
    WICED_BT_TRACE("[%s] deinit successful\n", __FUNCTION__);
}

static uint8_t *enqueue_isoc_data_to_send(uint16_t octets_per_frame, uint8_t num_of_channels, uint16_t frame_duration)
{
    uint8_t *p_buf = NULL;
    uint8_t wav_data_l[MAX_INPUT_SAMPLE_SIZE_IN_BYTES];
    uint8_t wav_data_r[MAX_INPUT_SAMPLE_SIZE_IN_BYTES];
    int num_of_samples = 0;
    int sample_width_in_bytes = 2; //FIXME: remove hardcoding to 16 bit

    uint8_t *p_iso_data = NULL;

    num_of_samples = (q_data.state == GA_ISOC_STATE_MEDIA_STREAMING)?
        audio_module_get_wave_data(wav_data_l, wav_data_r, frame_duration) : wiced_get_audio_data_form_mic(wav_data_l);

    if (stream_data.is_multiplexed)
    {
        p_iso_data = (uint8_t *)wiced_bt_get_buffer_from_pool(iso_audio_pool);
        if (!p_iso_data)
        {
            WICED_BT_TRACE_CRIT("[%s] p_buf is NULL", __FUNCTION__);
            return NULL;
        }
        p_buf = p_iso_data;
        UINT16_TO_STREAM(p_buf, stream_data.conn_hdl_list[0]);
        p_buf+=iso_dhm_get_header_size();
        lc3_codec_Encode(0, wav_data_l, num_of_samples * sample_width_in_bytes, p_buf, octets_per_frame);
        lc3_codec_Encode(1,
                         wav_data_r,
                         num_of_samples * sample_width_in_bytes,
                         p_buf + octets_per_frame,
                         octets_per_frame);
        wiced_bt_enqueue(&q_data.isoc_data_q, p_iso_data);
    }
    else
    {
        ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(stream_data.conn_hdl_list[0]);
        if (p_stream_info && p_stream_info->b_stream_active)
        {
            p_iso_data = (uint8_t *)wiced_bt_get_buffer_from_pool(iso_audio_pool);
            if (!p_iso_data)
            {
                WICED_BT_TRACE_CRIT("[%s] p_buf is NULL", __FUNCTION__);
                return NULL;
            }
            p_buf = p_iso_data;
            UINT16_TO_STREAM(p_buf, stream_data.conn_hdl_list[0]);
            p_buf += iso_dhm_get_header_size();
            lc3_codec_Encode(0, wav_data_l, num_of_samples * sample_width_in_bytes, p_buf, octets_per_frame);
            wiced_bt_enqueue(&q_data.isoc_data_q, p_iso_data);
        }
        p_stream_info = get_stream_info(stream_data.conn_hdl_list[1]);
        if (p_stream_info && p_stream_info->b_stream_active)
        {
            p_iso_data = (uint8_t *)wiced_bt_get_buffer_from_pool(iso_audio_pool);
            if (!p_iso_data)
            {
                WICED_BT_TRACE_CRIT("[%s] p_buf is NULL", __FUNCTION__);
                return NULL;
            }
            p_buf = p_iso_data;
            UINT16_TO_STREAM(p_buf, stream_data.conn_hdl_list[1]);
            p_buf += iso_dhm_get_header_size();
            lc3_codec_Encode(1, wav_data_r, num_of_samples * sample_width_in_bytes, p_buf, octets_per_frame);
            wiced_bt_enqueue(&q_data.isoc_data_q, p_iso_data);
        }
    }

    return p_iso_data;
}

static void fill_isoc_data_queue(void)
{
    if (q_data.state == GA_ISOC_STATE_IDLE) return;

    while (wiced_bt_get_pool_free_count(iso_audio_pool) > (uint32_t)(stream_data.stream_count))
    {
        if (stream_data.p_stream_info == NULL)
        {
            WICED_BT_TRACE_CRIT("[%s] stream is null", __FUNCTION__);
            return;
        }
        uint8_t *p_data = enqueue_isoc_data_to_send(stream_data.p_stream_info->octets_per_frame,
                                                    stream_data.p_stream_info->num_of_channels,
                                                    stream_data.p_stream_info->frame_duration);
        if (p_data == NULL) return;
    }

}

#ifdef WIN32
static DWORD WINAPI isocWin32TxThread(LPVOID lpParam)
{
    DWORD dwWaitResult;
    int len = 0;

    while (TRUE)
    {
        dwWaitResult = WaitForMultipleObjects(1,         // number of objects in array
                                              ghEvents,  // array of objects
                                              FALSE,     // wait for any object
                                              INFINITE); // wait time

        switch (dwWaitResult)
        {
        case WAIT_OBJECT_0 + 0: //Stream started event
            fill_isoc_data_queue();
        break;
        }

    }
    return TRUE;
}
#else
static void *isocLinuxTxThread(void *args)
{
    while (TRUE)
    {
        pthread_mutex_lock(&lock1);
        pthread_cond_wait(&cond, &lock1);
        pthread_mutex_unlock(&lock1);
        fill_isoc_data_queue();
    }
}
#endif

static void iso_create_thread(void)
{
#ifdef WIN32
    DWORD thread_address;
    ghEvents[0] = CreateEvent(NULL, FALSE, FALSE, "StreamStartedEvent");
    mRxThread = CreateThread(NULL, 0, isocWin32TxThread, 0, 0, &thread_address);
#else
    pthread_create(&ptid, NULL, &isocLinuxTxThread, NULL);
#endif
}

static void set_event(void)
{
#ifdef WIN32
    SetEvent(ghEvents[0]);
#else
    pthread_mutex_lock(&lock1);
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&lock1);
#endif
}

wiced_result_t lepl_isoc_dhm_setup_bis_datapath(uint16_t conn_hdl, wiced_bt_ga_bap_csc_t *p_csc)
{
    wiced_bool_t is_bis = FALSE;

    WICED_BT_TRACE("[%s] p_csc [0x%x]\n", __FUNCTION__, p_csc);
    if (!p_csc) return WICED_BADARG;

    WICED_BT_TRACE("[%s] dir %s sampleRate %d sduInterval %d octetsPerFrame %d \n",
                   __FUNCTION__,
                   "Source",
                   p_csc->sampling_frequency,
                   p_csc->frame_duration,
                   p_csc->octets_per_codec_frame);

    if (!p_csc->sampling_frequency || !p_csc->frame_duration || !p_csc->octets_per_codec_frame) return WICED_ERROR;

    wiced_ble_isoc_setup_data_path_info_t iso_bis_audio_param_data = {0};
    uint8_t codec_id[5] = {0x03, 0x00, 0x00, 0x00, 0x00};

    iso_bis_audio_param_data.data_path_dir = WICED_BLE_ISOC_DPD_INPUT;
    iso_bis_audio_param_data.data_path_id = WICED_BLE_ISOC_DPID_HCI;
    memcpy(iso_bis_audio_param_data.codec_id, codec_id, sizeof(iso_bis_audio_param_data.codec_id));

    // setup ISO data path and LC3 codec for INPUT from controller or OUTPUT to controller
    iso_bis_audio_param_data.isoc_conn_hdl = conn_hdl;
    wiced_result_t res = wiced_ble_isoc_setup_data_path(&iso_bis_audio_param_data);
    WICED_BT_TRACE_CRIT("[%s] Datapath setup sts 0x%x", __FUNCTION__, res);

    if (res != WICED_BT_SUCCESS)
    {
        return res;
    }

    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);
    if (!p_stream_info)
    {
        init_stream_info(conn_hdl, p_csc, WICED_BLE_ISOC_DPD_INPUT);
    }

    return WICED_SUCCESS;
}

wiced_result_t lepl_isoc_dhm_setup_cis_datapath(lepl_ase_data_t *p_ase)
{

    wiced_ble_isoc_data_path_direction_t data_path_dir = p_ase->data.p_ase_info->data_path_dir;
     wiced_bt_ga_bap_csc_t * p_csc = &p_ase->data.codec_configured.csc;
    uint16_t cis_conn_hdl = p_ase->cis_conn_handle;

    WICED_BT_TRACE("[%s] conn_hdl [0x%x] dir [%d] p_csc [0x%x] p_ase 0x%x\n", __FUNCTION__,
        cis_conn_hdl, data_path_dir, p_csc, p_ase);

    if (!p_csc) return WICED_BADARG;

    WICED_BT_TRACE("[%s] dir %s sampleRate %d sduInterval %d octetsPerFrame %d conn_hdl 0x%x\n",
                   __FUNCTION__,
                   (data_path_dir == WICED_BLE_ISOC_DPD_INPUT) ? "Source" : "Sink",
                   p_csc->sampling_frequency,
                   p_csc->frame_duration,
                   p_csc->octets_per_codec_frame,
                   cis_conn_hdl);

    if (!p_csc->sampling_frequency || !p_csc->frame_duration || !p_csc->octets_per_codec_frame) return WICED_ERROR;

    wiced_ble_isoc_setup_data_path_info_t iso_audio_param_data = {0};
    uint8_t codec_id[5] = {0x03, 0x00, 0x00, 0x00, 0x00};

    iso_audio_param_data.isoc_conn_hdl = cis_conn_hdl;
    iso_audio_param_data.p_app_ctx = p_ase;
    iso_audio_param_data.data_path_dir = data_path_dir;
    iso_audio_param_data.data_path_id = WICED_BLE_ISOC_DPID_HCI;
    memcpy(iso_audio_param_data.codec_id, codec_id, sizeof(iso_audio_param_data.codec_id));

    // setup ISO data path and LC3 codec for INPUT from controller or OUTPUT to controller
    wiced_result_t res = wiced_ble_isoc_setup_data_path(&iso_audio_param_data);
    WICED_BT_TRACE("[%s] Datapath setup res 0x%x", __FUNCTION__, res);
    if (res != WICED_BT_SUCCESS)
    {
        return res;
    }

    if (data_path_dir == WICED_BLE_ISOC_DPD_INPUT)
    {
        switch (p_csc->audio_channel_allocation)
        {
        case BAP_AUDIO_LOCATION_FRONT_LEFT:
            stream_data.conn_hdl_list[0] = cis_conn_hdl;
            stream_data.is_multiplexed = WICED_FALSE;
            break;

        case BAP_AUDIO_LOCATION_FRONT_RIGHT:
            stream_data.conn_hdl_list[1] = cis_conn_hdl;
            stream_data.is_multiplexed = WICED_FALSE;
            break;

        case BAP_AUDIO_LOCATION_FRONT_LEFT | BAP_AUDIO_LOCATION_FRONT_RIGHT:
            stream_data.conn_hdl_list[0] = cis_conn_hdl;
            stream_data.is_multiplexed = WICED_TRUE;
            break;

        default:
            WICED_BT_TRACE("Unsupported Audio Location");
            break;
        }
        stream_data.stream_count++;
    }

    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(cis_conn_hdl);
    if (!p_stream_info)
    {
        init_stream_info(cis_conn_hdl, p_csc, data_path_dir);
    }
    else
    {
        p_stream_info->dpd_bits |=
            (data_path_dir == WICED_BLE_ISOC_DPD_INPUT) ? WICED_BLE_ISOC_DPD_INPUT_BIT : WICED_BLE_ISOC_DPD_OUTPUT_BIT;
    }

    return WICED_SUCCESS;
}

void lepl_isoc_dhm_remove_cis_datapath(uint16_t conn_hdl, wiced_ble_isoc_data_path_bit_t dir)
{
    wiced_bool_t is_cis = FALSE;
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);

    if (p_stream_info == NULL) return;

    WICED_BT_TRACE("[%s] [dir %d] [conn_hdl %x] [num of channel %x]",
                   __FUNCTION__,
                   dir,
                   conn_hdl);
    if (!(p_stream_info->dpd_bits & dir))
    {
        WICED_BT_TRACE_CRIT("[%s]  No active data path", __FUNCTION__);
        return;
    }

    p_stream_info->dpd_bits &= (~dir);

    if (!p_stream_info->dpd_bits)
    {
        deinit_stream_info(conn_hdl);
    }

    if (dir == WICED_BLE_ISOC_DPD_INPUT_BIT)
        stream_data.stream_count--;

    /* check if CIS is established before setting up data path */
    is_cis = wiced_ble_isoc_is_cis_connected_with_conn_hdl(conn_hdl);
    if (!is_cis) return;

    if (!wiced_ble_isoc_remove_data_path(conn_hdl, dir, NULL))
    {
        WICED_BT_TRACE_CRIT("[%s] No active data path\n", __FUNCTION__);
    }
}

//Send ISO data
static void tx_iso_data(void)
{
    uint8_t *p_buf = NULL;
    uint16_t isoc_conn_hdl;
    uint8_t *p_isoc_data = (uint8_t *)wiced_bt_dequeue(&q_data.isoc_data_q);
    p_buf = p_isoc_data;

    if (q_data.state == GA_ISOC_STATE_IDLE) return;

    if (p_isoc_data)
    {
        STREAM_TO_UINT16(isoc_conn_hdl, p_buf);
        ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(isoc_conn_hdl);
        if (p_stream_info)
        {
            iso_dhm_send_packet(p_stream_info->psn++,
                                p_stream_info->conn_hdl,
                                0,
                                p_buf,
                                p_stream_info->octets_per_frame * p_stream_info->num_of_channels);
        }

        wiced_bt_free_buffer(p_isoc_data);
    }
    else
        WICED_BT_TRACE_CRIT("[%s] queue is empty!!",__FUNCTION__);
}

void lepl_isoc_dhm_remove_bis_datapath(uint16_t *conn_hdl_list, uint8_t bis_count)
{
    wiced_bool_t is_bis = FALSE;
    uint8_t numOfchannel = 1;
    for (uint8_t i = 0; i < bis_count; i++)
    {
        ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl_list[i]);

        if (p_stream_info == NULL) return;

        WICED_BT_TRACE("[%s] [conn_hdl %x] [num of channel %x]",
                       __FUNCTION__,
                       conn_hdl_list[i],
                       p_stream_info->num_of_channels);

        numOfchannel = p_stream_info->num_of_channels;
        deinit_stream_info(conn_hdl_list[i]);

        stream_data.stream_count--;
        /* check if BIS is established before removing data path */
        is_bis = wiced_ble_isoc_is_bis_created(conn_hdl_list[i]);
        if (!is_bis) return;

        if (!wiced_ble_isoc_remove_data_path(conn_hdl_list[i], WICED_BLE_ISOC_DPD_INPUT_BIT, NULL))
            WICED_BT_TRACE_CRIT("[%s] No active data path\n", __FUNCTION__);
    }
}

static void lepl_isoc_dhm_start_stream(ga_iso_audio_stream_info_t *p_stream_info, wiced_ble_isoc_data_path_direction_t dir)
{
    lepl_app_state_t app_state = lepl_cap_get_application_state();
    if (app_state == LEPL_APP_STATE_IN_TRANSIT)
        app_state = lepl_cap_get_application_final_state();

    if (q_data.state != GA_ISOC_STATE_IDLE) return;

    lc3_config_t codec_config = {.sampleRate = p_stream_info->sampling_frequency,
                                 .sduInterval = p_stream_info->frame_duration,
                                 .octetsPerFrame = p_stream_info->octets_per_frame,
                                 .sampleWidthInBits = 16};
    wiced_bool_t codec_config_sts;

    for (int i = 0; i < CHANNEL_COUNT; i++)
    {
        codec_config_sts = (dir == WICED_BLE_ISOC_DPD_INPUT) ? lc3_codec_initializeEncoder(i, &codec_config)
                                                             : lc3_codec_initializeDecoder(i, &codec_config);
        if (FALSE == codec_config_sts)
        {
            WICED_BT_TRACE_CRIT("[%s] lc3_codec_initialize NOT SUCCESSFUL", __FUNCTION__);
            return;
        }
    }

    if (dir == WICED_BLE_ISOC_DPD_INPUT)
    {
        //read from wav file
        audio_driver_init(WICED_BLE_ISOC_DPD_INPUT_BIT, p_stream_info->num_of_channels, codec_config.sampleRate, LINUX_ALSA_LATENCY);
        audio_driver_load_wave_file(WICED_BLE_ISOC_DPD_INPUT_BIT, codec_config.sampleRate);

        if (app_state == LEPL_APP_STATE_MEDIA ||
            lepl_get_call_control_server_state() == CALL_CONTROL_SERVER_STATE_INBAND_RINGTONE)
        {
            iso_audio_set_state(GA_ISOC_STATE_MEDIA_STREAMING);
        }
        else if (app_state == LEPL_APP_STATE_IDLE)
        {
            TRACE_ERR("app_state is LEPL_APP_STATE_IDLE");
            return;
        }
        else
        {
            iso_audio_set_state(GA_ISOC_STATE_MIC);
        }

        for (int i = 0; i < 10 * p_stream_info->num_of_channels; i++)
        {
            uint8_t *p_data = enqueue_isoc_data_to_send(p_stream_info->octets_per_frame,
                                                        p_stream_info->num_of_channels,
                                                        p_stream_info->frame_duration);
        }
        p_stream_info->psn = 0;
        for (int i = 0; i < 10; i++)
        {
            tx_iso_data();
        }
        set_event();
    }
    else if (dir == WICED_BLE_ISOC_DPD_OUTPUT)
    {
        //no good, but in currently code structure, need to check channel number by cis count or csis num_devices 
        uint8_t num_devices = g_lepl_gatt_cb.cap_profile_data.num_devices; 
        audio_driver_init(WICED_BLE_ISOC_DPD_OUTPUT_BIT, (num_devices == 0 ? p_stream_info->num_of_channels:num_devices), codec_config.sampleRate, LINUX_ALSA_LATENCY);
        audio_driver_init_dump_file_by_conn_hdl(codec_config.sampleRate, p_stream_info->conn_hdl);
        audio_driver_set_volume((130 * 100) / 255);
    }

}

void lepl_isoc_dhm_start_cis_stream(uint16_t conn_hdl, wiced_ble_isoc_data_path_direction_t dir)
{
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);
    stream_data.p_stream_info = p_stream_info; //update q stream info

    WICED_BT_TRACE("[%s] p_stream_info 0x%x conn_hdl 0x%x\n", __FUNCTION__, p_stream_info, conn_hdl);
    if (!p_stream_info) return;


    lepl_isoc_dhm_start_stream(p_stream_info, dir);
}

void lepl_isoc_dhm_start_bis_stream(uint16_t conn_hdl)
{
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);

    stream_data.p_stream_info = p_stream_info; //update q stream info

    WICED_BT_TRACE("[%s] p_stream_info 0x%x conn_hdl 0x%x\n", __FUNCTION__, p_stream_info, conn_hdl);
    if (!p_stream_info) return;

    switch (p_stream_info->audio_location)
    {
    case BAP_AUDIO_LOCATION_FRONT_LEFT:
        stream_data.conn_hdl_list[0] = conn_hdl;
        stream_data.is_multiplexed = WICED_FALSE;
        break;

    case BAP_AUDIO_LOCATION_FRONT_RIGHT:
        stream_data.conn_hdl_list[1] = conn_hdl;
        stream_data.is_multiplexed = WICED_FALSE;
        break;

    case BAP_AUDIO_LOCATION_FRONT_LEFT | BAP_AUDIO_LOCATION_FRONT_RIGHT:
        stream_data.conn_hdl_list[0] = conn_hdl;
        stream_data.is_multiplexed = WICED_TRUE;
        break;

    default:
        WICED_BT_TRACE("Unsupported Audio Location");
        break;
    }
    stream_data.stream_count++;

    lepl_isoc_dhm_start_stream(p_stream_info, WICED_BLE_ISOC_DPD_INPUT);
}

void lepl_isoc_dhm_stop_stream(uint16_t isoc_conn_hdl)
{

}

static void num_complete_handler(uint16_t conn_hdl, uint16_t num_sent)
{
    if (q_data.state == GA_ISOC_STATE_IDLE) return;

    for (uint8_t i = 0; i < num_sent; i++)
    {
        tx_iso_data();
    }
    set_event();
}

static void rx_handler(uint16_t conn_hdl, uint8_t *p_data, uint32_t length)
{

    uint8_t lc3_data_l[480 * 2] = {0};
    uint8_t lc3_data_r[480 * 2] = {0};
    uint32_t decoded_data_size = 0;
    int sample_width_in_bytes = 2; //FIXME: remove hardcoding to 16 bit
    wiced_bt_ga_ascs_config_codec_args_t *codec_params_ptr = NULL;
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);

    if (!p_stream_info) return;
    
    //debug log
    //TRACE_LOG("conn_hdl:0x%x, num_of_channels:%d, length:%d", conn_hdl, p_stream_info->num_of_channels, length);

    decoded_data_size =
        wiced_bt_ga_bap_get_decoded_data_size(p_stream_info->sampling_frequency, p_stream_info->frame_duration);

    // Validate received length against expected octets per frame
    if (length != (p_stream_info->octets_per_frame * p_stream_info->num_of_channels))
    {
        return;
    }

    //Decode
    lc3_codec_Decode(0,
                     0,
                     p_data,
                     p_stream_info->octets_per_frame,
                     lc3_data_l,
                     decoded_data_size * sample_width_in_bytes);

    if (2 == p_stream_info->num_of_channels)
    {
        lc3_codec_Decode(1,
                         0,
                         p_data + p_stream_info->octets_per_frame,
                         p_stream_info->octets_per_frame,
                         lc3_data_r,
                         decoded_data_size * sample_width_in_bytes);
    }

#ifdef ENABLE_DUMP_AUDIO_FILE_CONN_HDL
    audio_driver_dump_file_by_conn_hdl(conn_hdl, lc3_data_l, sample_width_in_bytes * decoded_data_size * p_stream_info->num_of_channels);
#endif
    audio_driver_write_non_interleaved_data(lc3_data_l,
                                            (2 == p_stream_info->num_of_channels) ? lc3_data_r : NULL,
                                            sample_width_in_bytes,
                                            decoded_data_size * p_stream_info->num_of_channels);
}


static void iso_create_pool(void)
{
    //create a iso pool
    int buff_size = iso_dhm_get_buffer_size(SDU_SIZE, CHANNEL_COUNT);

    queue_lock.p_lock_context = NULL;
    queue_lock.pf_lock_func = iso_audio_lock;
    queue_lock.pf_unlock_func = iso_audio_unlock;

    // Allocate only once, allowing multiple calls to update callbacks
    if (!iso_audio_pool)
        iso_audio_pool =
            wiced_bt_create_pool("ISO SDU", buff_size + QUEUE_BUFFER_SIZE, MAX_BUFFER_PER_CIS, &queue_lock);

    if (!iso_audio_pool)
    {
        WICED_BT_TRACE_CRIT("[%s] iso_audio_pool is NULL\n", __FUNCTION__);
        return;
    }

    WICED_BT_TRACE("[%s] g_cis_iso_pool 0x%x size %d count %d",
                   __FUNCTION__,
                   iso_audio_pool,
                   buff_size,
                   MAX_BUFFER_PER_CIS);
}

void lepl_isoc_dhm_init(void)
{
    iso_create_pool();
    iso_dhm_register_cb(num_complete_handler, rx_handler);
    lc3_codec_reset();
    iso_create_thread();
    wiced_bt_init_q(&q_data.isoc_data_q, &queue_lock);
}

void lepl_isoc_dhm_disable_audio(void)
{
    if (stream_data.stream_count) return;

    iso_audio_set_state(GA_ISOC_STATE_IDLE);

    WICED_MEMSET(&stream_data, 0, sizeof(ga_iso_stream_data_t));
    if (wiced_bt_queue_is_empty(&q_data.isoc_data_q) == FALSE)
    {
        uint8_t *p_isoc_data = NULL;
        while ((p_isoc_data = (uint8_t *)wiced_bt_dequeue(&q_data.isoc_data_q)) != NULL)
        {
            wiced_bt_free_buffer(p_isoc_data);
        }
    }

    audio_driver_deinit(WICED_BLE_ISOC_DPD_INPUT_BIT);
    audio_driver_deinit(WICED_BLE_ISOC_DPD_OUTPUT_BIT);

    for (int i = 0; i < CHANNEL_COUNT; i++)
    {
        lc3_codec_releaseEncoder(i);
        lc3_codec_releaseDecoder(i);
    }
}

void lepl_ccs_isoc_handle_ringtone_to_convo(lepl_ase_data_t *p_ase)
{
    wiced_bt_ga_bap_csc_t *p_csc = &p_ase->data.codec_configured.csc;
    p_ase->data.metadata.streaming_audio_ctx = BAP_CONTEXT_TYPE_CONVERSATIONAL;

    audio_driver_mic_init(1, p_csc->sampling_frequency);
    iso_audio_set_state(GA_ISOC_STATE_MIC);

    lc3_config_t codec_config;
    wiced_bool_t codec_config_sts;
    codec_config.sampleRate = p_csc->sampling_frequency;
    codec_config.sduInterval = p_csc->frame_duration;
    codec_config.octetsPerFrame = p_csc->octets_per_codec_frame;
    codec_config.sampleWidthInBits = 16;

    codec_config_sts = lc3_codec_initializeDecoder(0, &codec_config);
    if (FALSE == codec_config_sts)
    {
        WICED_BT_TRACE_CRIT("[%s] lc3_codec_initialize NOT SUCCESSFUL", __FUNCTION__);
    }
    audio_driver_init(WICED_BLE_ISOC_DPD_OUTPUT_BIT, p_csc->audio_channel_allocation, p_csc->sampling_frequency, LINUX_ALSA_LATENCY);
}

/*
 *  rx_handler_voice_assist
 *
 *  This function use for voice_assist, 2 CIS in 1 CIG, each CIS have 1 channel
 *  use USB headset as playback device, 2 use case, 2 CIS, or 1 CIS
 *  2 CIS: collect left and right channel data and write to playback
 *  1 CIS: write to playback 
 */
static void rx_handler_voice_assist(uint16_t conn_hdl, uint8_t *p_data, uint32_t length)
{
    static uint8_t wav_data_l[480 * 2]  = {0};
    static uint8_t wav_data_r[480 * 2]  = {0};
    uint8_t *p_wav_data                 = NULL;
    static uint8_t left_right_isoc      = 0;
    uint32_t decoded_data_size          = 0;
    uint8_t lc3_decode_index            = 0;
    int sample_width_in_bytes           = 2; //FIXME: remove hardcoding to 16 bit

    wiced_bt_ga_ascs_config_codec_args_t *codec_params_ptr = NULL;

    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);

    if (!p_stream_info) return;

    //debug log
    //TRACE_LOG("conn_hdl:0x%x, num_of_channels:%d, length:%d, num_devices:%d", conn_hdl, p_stream_info->num_of_channels, length, g_lepl_gatt_cb.cap_profile_data.num_devices);

    decoded_data_size =
        wiced_bt_ga_bap_get_decoded_data_size(p_stream_info->sampling_frequency, p_stream_info->frame_duration);

    // Validate received length against expected octets per frame
    if (length != (p_stream_info->octets_per_frame * p_stream_info->num_of_channels))
    {
        //TRACE_LOG("error length wrong:%d", length);
        return;
    }

    if (p_stream_info->audio_location == BAP_AUDIO_LOCATION_FRONT_LEFT)
    {
        memset(wav_data_l, 0, sizeof(wav_data_l));
        lc3_decode_index = 0;
        p_wav_data = wav_data_l;
    }

    if (p_stream_info->audio_location == BAP_AUDIO_LOCATION_FRONT_RIGHT) //front right
    {
        memset(wav_data_r, 0, sizeof(wav_data_r));
        lc3_decode_index = 1;
        p_wav_data = wav_data_r;
    }

    lc3_codec_Decode(lc3_decode_index,
                    0,
                    p_data,
                    p_stream_info->octets_per_frame,
                    p_wav_data,
                    decoded_data_size * sample_width_in_bytes);

    left_right_isoc |= p_stream_info->audio_location;

#ifdef ENABLE_DUMP_AUDIO_FILE_CONN_HDL
    audio_driver_dump_file_by_conn_hdl(conn_hdl, p_wav_data, sample_width_in_bytes * decoded_data_size * p_stream_info->num_of_channels);
#endif

    if (g_lepl_gatt_cb.cap_profile_data.num_devices == 1)
    {
        audio_driver_write_non_interleaved_data(wav_data_l, wav_data_r, sample_width_in_bytes, decoded_data_size * 2);
        left_right_isoc = 0;
        return;
    }
    else
    {
        if (!(left_right_isoc & BAP_AUDIO_LOCATION_FRONT_LEFT) || !(left_right_isoc & BAP_AUDIO_LOCATION_FRONT_RIGHT)) 
        {
            return;
        }
        left_right_isoc = 0;
    }

    audio_driver_write_non_interleaved_data(wav_data_l, wav_data_r, sample_width_in_bytes, decoded_data_size * 2);

}

void lepl_isoc_dhm_reinit_default(void)
{
    iso_dhm_register_cb(num_complete_handler, rx_handler);
}

void lepl_isoc_dhm_reinit_voice_assist(void)
{
    iso_dhm_register_cb(num_complete_handler, rx_handler_voice_assist);
}

#endif
