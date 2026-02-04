/*
* $ Copyright Cypress Semiconductor $
*/
#ifndef ISOC_OFFLOAD
#ifndef ISOC_OFFLOAD_LINUX
#include "lehs.h"

#include "wiced_memory.h"
#include "audio_driver.h"
#include "iso_data_handler.h"
#include "lc3_codec.h"
#include "lehs_isoc.h"

static ga_iso_audio_stream_info_t g_stream_info[MAX_STREAMS_SUPPORTED] = {0};

wiced_bt_pool_t *ga_iso_audio_pool = NULL;
extern int wiced_get_audio_data_form_mic(uint8_t *l_data);
uint8_t *p_buf = NULL;

#define LINUX_ALSA_LATENCY 500U
#define MAX_BUFFER_PER_CIS 20
#define MAX_SDU_SIZE 240
#define MAX_CHANNEL_COUNT 2

void iso_create_pool(void)
{
    //create a iso pool
    int buff_size = iso_dhm_get_buffer_size(MAX_SDU_SIZE, MAX_CHANNEL_COUNT);

    // Allocate only once, allowing multiple calls to update callbacks
    if (!ga_iso_audio_pool) ga_iso_audio_pool = wiced_bt_create_pool("ISO SDU", buff_size, MAX_BUFFER_PER_CIS, NULL);

    if (!ga_iso_audio_pool)
    {
        WICED_BT_TRACE_CRIT("[%s] ga_iso_audio_pool is NULL\n", __FUNCTION__);
        return;
    }

    WICED_BT_TRACE("[%s] g_cis_iso_pool 0x%x size %d count %d",
                   __FUNCTION__,
                   ga_iso_audio_pool,
                   buff_size,
                   MAX_BUFFER_PER_CIS);
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

void init_stream_info(uint16_t conn_hdl,
                      wiced_bt_ga_bap_csc_t *p_csc,
                      wiced_ble_isoc_data_path_bit_t dir_bit)
{
    WICED_BT_TRACE("[%s] [SF %d] [OPF %d] [FD %d] [audio location %d]\n",
                   __FUNCTION__,
                   p_csc->sampling_frequency,
                   p_csc->octets_per_codec_frame,
                   p_csc->frame_duration,
                   p_csc->audio_channel_allocation);

    for (size_t i = 0; i < MAX_STREAMS_SUPPORTED; i++)
    {
        if (!g_stream_info[i].conn_hdl)
        {
            g_stream_info[i].conn_hdl = conn_hdl;
            g_stream_info[i].octets_per_frame = p_csc->octets_per_codec_frame;
            g_stream_info[i].audio_location = p_csc->audio_channel_allocation;
            g_stream_info[i].sampling_frequency = p_csc->sampling_frequency;
            g_stream_info[i].frame_duration = p_csc->frame_duration;
            g_stream_info[i].num_of_channels = count_set_bits(p_csc->audio_channel_allocation);
            g_stream_info[i].b_stream_active = 1;

            WICED_BT_TRACE("[%s] conn_hdl 0x%x \n", __FUNCTION__, conn_hdl);
            return;
        }
    }
}

ga_iso_audio_stream_info_t *get_stream_info(uint16_t conn_hdl)
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

void lehs_isoc_audio_stop_stream(uint16_t conn_hdl)
{
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);
    p_stream_info->b_stream_active = 0;
}

static void deinit_stream_info(uint16_t conn_hdl)
{
    WICED_BT_TRACE("[%s] [conn_hdl %d]\n", __FUNCTION__, conn_hdl);

    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);
    if (!p_stream_info) return;

    memset(p_stream_info, 0, sizeof(ga_iso_audio_stream_info_t));
    WICED_BT_TRACE("[%s] deinit successful\n", __FUNCTION__);
}

#define LEHS_MAX_ISO_BUFF_SIZE_PER_CIS                                                                    \
    10 //this should be less than Max available controller buffers for isoc per cis

/*
 *  tx_iso_data
 *
 *  get audio data from mic and send to lepl, different usb headset have different channel support.
 *  , do some preprocess in linux porting layer, so always get mono 
 *
 */
static void tx_iso_data(uint16_t conn_handle)
{
    uint8_t wav_data[MAX_INPUT_SAMPLE_SIZE_IN_BYTES]    = {0}; //TODO: by channel size
    int num_of_samples                                  = 0;
    int sample_width_in_bytes                           = 2; //FIXME: remove hardcoding to 16 bit
    wiced_bt_ga_bap_csc_t *p_csc                        = NULL;
    uint8_t *p_data                                     = NULL;

    //TODO: chec if LC3 codec is initialized

    if (!p_buf)
    {
        WICED_BT_TRACE_CRIT("[%s] p_buf is NULL\n", __FUNCTION__);
        return;
    }

    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_handle);

    if (!p_stream_info) return;

    p_data = p_buf + iso_dhm_get_header_size(); //leave space for header

    num_of_samples = wiced_get_audio_data_form_mic(wav_data);
    if (num_of_samples < 0)
    {
        num_of_samples = 0;
        return;
    }
    
    //dump mic data to file
    audio_driver_write_mic_data(wav_data, sample_width_in_bytes, num_of_samples * sample_width_in_bytes * p_stream_info->num_of_channels);

    WICED_BT_TRACE("[%s] MIC sample = %d, audio_location:%d, num_of_channels:%d, bytes_per_codec_frame:%d", __FUNCTION__, num_of_samples, p_stream_info->audio_location, p_stream_info->num_of_channels, p_stream_info->octets_per_frame);

    lc3_codec_Encode(0, wav_data, num_of_samples * sample_width_in_bytes * p_stream_info->num_of_channels, p_data, p_stream_info->octets_per_frame);

    iso_dhm_send_packet(p_stream_info->psn++, conn_handle, 0, p_buf, p_stream_info->octets_per_frame);
}

static void num_complete_handler(uint16_t conn_hdl, uint16_t num_sent)
{
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);
    WICED_BT_TRACE("[%s] [conn_hdl %d] [num_sent %d] \n",
                   __FUNCTION__,
                    conn_hdl,
                    num_sent);

    for (int i = 0; i < num_sent; i++)
    {
        if (!p_stream_info || !p_stream_info->b_stream_active) return;
        tx_iso_data(conn_hdl);
    }
}

static void rx_handler(uint16_t conn_hdl, uint8_t *p_data, uint32_t length)
{
    uint8_t lc3_data_l[480 * 2] = {0};
    uint8_t lc3_data_r[480 * 2] = {0};
    uint32_t decoded_data_size = 0;
    int sample_width_in_bytes = 2; //FIXME: remove hardcoding to 16 bit
    wiced_bt_ga_ascs_config_codec_args_t *codec_params_ptr = NULL;
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);

    if (!p_stream_info)
    {
        WICED_BT_TRACE_CRIT("[%s] stream is null", __FUNCTION__);
        return;
    }


    //TODO: check if LC3 codec is initialized

    // Get the following from application data
    // num_of_channels
    // sample_rate
    // sdu_interval
    // sample_width_in_bytes
    decoded_data_size = wiced_bt_ga_bap_get_decoded_data_size(p_stream_info->sampling_frequency, p_stream_info->frame_duration);

    // Validate received length against expected octets per frame
    if (length != (p_stream_info->octets_per_frame * p_stream_info->num_of_channels))
    {
        WICED_BT_TRACE_CRIT("[%s] Expected %d bytes, received %d bytes (channel cnt - %d)",
                            __FUNCTION__,
                            (p_stream_info->octets_per_frame * p_stream_info->num_of_channels),
                            length,
                            p_stream_info->num_of_channels);
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

    audio_driver_write_non_interleaved_data(lc3_data_l,
                                            (2 == p_stream_info->num_of_channels) ? lc3_data_r : NULL,
                                            sample_width_in_bytes,
                                            decoded_data_size * p_stream_info->num_of_channels);
}

void lehs_isoc_dhm_init(void)
{
    iso_create_pool();
    iso_dhm_register_cb(num_complete_handler, rx_handler);
    lc3_codec_reset();
}

static wiced_result_t iso_audio_start_stream(uint16_t conn_hdl,
                                             uint32_t channel_count,
                                             wiced_ble_isoc_data_path_bit_t dir_bit,
                                             wiced_bt_ga_bap_csc_t *p_csc)
{
    lc3_config_t codec_config;
    wiced_bool_t codec_config_sts;

    codec_config.sampleRate = p_csc->sampling_frequency;
    codec_config.sduInterval = p_csc->frame_duration;
    codec_config.octetsPerFrame = p_csc->octets_per_codec_frame;
    codec_config.sampleWidthInBits = 16;

    for (uint32_t i = 0; i < channel_count; i++)
    {
        codec_config_sts = (dir_bit == WICED_BLE_ISOC_DPD_INPUT_BIT) ? lc3_codec_initializeEncoder(i, &codec_config)
                                                                       : lc3_codec_initializeDecoder(i, &codec_config);
        if (FALSE == codec_config_sts)
        {
            WICED_BT_TRACE_CRIT("[%s] lc3_codec_initialize NOT SUCCESSFUL", __FUNCTION__);
            return WICED_ERROR;
        }
    }

    init_stream_info(conn_hdl, p_csc, dir_bit);

    audio_driver_init(dir_bit, channel_count, codec_config.sampleRate, LINUX_ALSA_LATENCY);
    if (dir_bit == WICED_BLE_ISOC_DPD_INPUT_BIT)
    {
        WICED_BT_TRACE("[%s]:WICED_BLE_ISOC_DPD_INPUT_BIT", __FUNCTION__);
        audio_driver_mic_init(channel_count, codec_config.sampleRate);
    }
    audio_driver_set_volume((130 * 100) / 255);
    return WICED_SUCCESS;
}

wiced_result_t lehs_isoc_dhm_setup_cis_stream(lehs_ase_data_t *p_ase)
{
    int channel_count = 1;
    wiced_ble_isoc_data_path_direction_t data_path_dir = p_ase->data.p_ase_info->data_path_dir;
    wiced_bt_ga_bap_csc_t *p_csc = &p_ase->data.codec_configured.csc;
    wiced_result_t ret;
    uint16_t cis_conn_handle = p_ase->cis_conn_handle;

    WICED_BT_TRACE("[%s] conn_hdl [0x%x] dir [%d] p_csc [0x%x] p_ase 0x%x\n",
                   __FUNCTION__,
                   cis_conn_handle,
                   data_path_dir,
                   p_csc,
                   p_ase);
    if (!p_csc) return WICED_BADARG;

    if (!p_csc->sampling_frequency || !p_csc->frame_duration || !p_csc->octets_per_codec_frame)
    {
        return WICED_ERROR;
    }

    /* determine num of channels required and configure codec accordingly
    (if not configured default to 1) */
    if (p_csc->audio_channel_allocation)
    {
        channel_count = count_set_bits(p_csc->audio_channel_allocation);
    }

    wiced_ble_isoc_setup_data_path_info_t iso_audio_param_data;
    uint8_t codec_id[5] = {0x03, 0x00, 0x00, 0x00, 0x00};

    iso_audio_param_data.isoc_conn_hdl = cis_conn_handle;
    iso_audio_param_data.p_app_ctx = p_ase;
    iso_audio_param_data.data_path_dir = data_path_dir;
    iso_audio_param_data.data_path_id = WICED_BLE_ISOC_DPID_HCI;
    iso_audio_param_data.p_csc = NULL;
    iso_audio_param_data.csc_length = 0;
    memcpy(iso_audio_param_data.codec_id, codec_id, sizeof(iso_audio_param_data.codec_id));

    // setup ISO data path and LC3 codec for INPUT from controller or OUTPUT to controller
    ret = wiced_ble_isoc_setup_data_path(&iso_audio_param_data);
    WICED_BT_TRACE("[%s] Datapath setup res 0x%x", __FUNCTION__, ret);
    if (ret != WICED_SUCCESS)
    {
        return ret;
    }

    wiced_ble_isoc_data_path_bit_t dir_bit =
        (data_path_dir == WICED_BLE_ISOC_DPD_INPUT) ? WICED_BLE_ISOC_DPD_INPUT_BIT : WICED_BLE_ISOC_DPD_OUTPUT_BIT;
    iso_audio_start_stream(cis_conn_handle,channel_count, dir_bit, p_csc);

    return WICED_SUCCESS;
}

wiced_result_t lehs_isoc_dhm_setup_bis_stream(broadcast_sink_cb_t *p_big, uint8_t bis_count)
{
    int channel_count = 1;
    for (int i = 0; i < bis_count; i++)
    {
        wiced_bt_ga_bap_csc_t *p_csc = &p_big->base.sub_group[0].csc;
        WICED_BT_TRACE("[%s] SR %d sduInterval %d OPF %d conn_hdl %x \n",
                       __FUNCTION__,
                       p_csc->sampling_frequency,
                       p_csc->frame_duration,
                       p_csc->octets_per_codec_frame,
                       p_big->bis_conn_id_list[i]);

        if (!p_csc) return WICED_BADARG;

        if (!p_csc->sampling_frequency || !p_csc->frame_duration || !p_csc->octets_per_codec_frame) return WICED_ERROR;

        /* determine num of channels required and configure codec accordingly
        (if not configured default to 1) */
        if (p_csc->audio_channel_allocation)
        {
            channel_count = count_set_bits(p_csc->audio_channel_allocation);
            if (channel_count > 2) WICED_BT_TRACE("[%s] Unsupported Audio Locations", __FUNCTION__);
        }
        wiced_ble_isoc_setup_data_path_info_t iso_bis_audio_param_data = {0};
        uint8_t codec_id[5] = {0x03, 0x00, 0x00, 0x00, 0x00};

        iso_bis_audio_param_data.isoc_conn_hdl = p_big->bis_conn_id_list[i];
        iso_bis_audio_param_data.data_path_dir = WICED_BLE_ISOC_DPD_OUTPUT;
        iso_bis_audio_param_data.data_path_id = WICED_BLE_ISOC_DPID_HCI;
        memcpy(iso_bis_audio_param_data.codec_id, codec_id, sizeof(iso_bis_audio_param_data.codec_id));

        // setup ISO data path and LC3 codec for INPUT from controller or OUTPUT to controller

        wiced_result_t res = wiced_ble_isoc_setup_data_path(&iso_bis_audio_param_data);
        WICED_BT_TRACE_CRIT("[%s] Datapath setup sts 0x%x", __FUNCTION__, res);

        if (res != WICED_BT_SUCCESS)
        {
            return res;
        }
        iso_audio_start_stream(p_big->bis_conn_id_list[i], channel_count, WICED_BLE_ISOC_DPD_OUTPUT_BIT, p_csc);
    }
    return WICED_SUCCESS;
}

void lehs_isoc_free_stream(uint16_t conn_hdl, wiced_ble_isoc_data_path_bit_t dir)
{
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);

    if (p_stream_info == NULL) return;

    int num_channels = p_stream_info->num_of_channels;

    WICED_BT_TRACE("[%s] [dir %d] [conn_hdl %x] [num of channel %x]",
                   __FUNCTION__,
                   dir,
                   conn_hdl,
                   p_stream_info->num_of_channels);
    audio_driver_deinit(dir);
    deinit_stream_info(conn_hdl);
    // if removing paths for both directions release both decoder instances,
    // otherwise only release the one corresponding to the removed path

    if (WICED_BLE_ISOC_DPD_INPUT_BIT & dir)
    {
        for (int i = 0; i < num_channels; i++)
        {
            lc3_codec_releaseEncoder(i);
        }
    }
    else if (WICED_BLE_ISOC_DPD_OUTPUT_BIT & dir)
    {
        for (int i = 0; i < num_channels; i++)
        {
            lc3_codec_releaseDecoder(i);
        }
    }
    if (p_buf != NULL)
    {
        wiced_bt_free_buffer(p_buf);
        p_buf = NULL;
    }

}

void lehs_isoc_dhm_free_cis_stream(uint16_t conn_hdl,
                                       wiced_ble_isoc_data_path_bit_t dir)
{
    if (!wiced_ble_isoc_remove_data_path(conn_hdl, dir, NULL))
        WICED_BT_TRACE_CRIT("[%s] No active data path\n", __FUNCTION__);
    lehs_isoc_free_stream(conn_hdl, dir);
}

void lehs_isoc_dhm_free_bis_stream(uint16_t *conn_hdl_list, uint8_t bis_count)
{
    for (int i = 0; i < bis_count; i++)
    {
        if (!wiced_ble_isoc_remove_data_path(conn_hdl_list[i], WICED_BLE_ISOC_DPD_OUTPUT_BIT, NULL))
            WICED_BT_TRACE_CRIT("[%s] No active data path\n", __FUNCTION__);
        lehs_isoc_free_stream(conn_hdl_list[i], WICED_BLE_ISOC_DPD_OUTPUT_BIT);
    }
}

void lehs_isoc_dhm_start_stream(uint16_t conn_hdl, uint8_t ase_type)
{
    if (ase_type == ASCS_SOURCE_ASE_CHARACTERISTIC)
    {
        ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);
        p_buf = (uint8_t *)wiced_bt_get_buffer_from_pool(ga_iso_audio_pool);
        WICED_BT_TRACE("[%s] p_stream_info 0x%x conn_hdl 0x%x\n", __FUNCTION__, p_stream_info, conn_hdl);

        if (!p_stream_info)
        {
            WICED_BT_TRACE("p_stream_info is NULL");
            return;
        }

        p_stream_info->psn = 0;
        for (int i = 0; i < LEHS_MAX_ISO_BUFF_SIZE_PER_CIS; i++)
        {
            tx_iso_data(conn_hdl);
        }
    }
}
#endif
#endif
