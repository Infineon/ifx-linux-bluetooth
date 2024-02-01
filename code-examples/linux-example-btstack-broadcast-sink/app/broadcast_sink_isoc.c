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

#include "iso_data_handler.h"
#include "lc3_codec.h"
#include "wiced_bt_cfg.h"

#include "audio_driver.h"
#include "wiced_bt_ga_ascs.h"
#include "wiced_bt_trace.h"

#define LINUX_ALSA_LATENCY              500U
#define MAX_STREAMS_SUPPORTED 2
#define MAX_INPUT_SAMPLE_SIZE_IN_BYTES 480 * 2 //48khz @ 10ms interval

typedef struct
{
    wiced_bool_t is_cis;
    uint16_t conn_hdl;
    uint16_t octets_per_frame;
    uint8_t num_of_channels;
    wiced_bool_t b_stream_active;
    uint32_t frame_duration;
    uint32_t sampling_frequency;
    uint32_t num_bis;
} ga_iso_audio_stream_info_t;

static ga_iso_audio_stream_info_t g_stream_info[MAX_STREAMS_SUPPORTED] = { 0 };

static void init_stream_info(uint16_t conn_hdl,
    wiced_bool_t is_cis,
    uint32_t sampling_frequency,
    uint16_t octets_per_frame,
    uint32_t frame_duration,
    uint8_t num_of_channels, int8_t num_bis)
{
    WICED_BT_TRACE("[%s] [is_cis %d] [SF %d] [OPF %d] [FD %d] [num_of_channels %d]\n",
        __FUNCTION__,
        is_cis,
        sampling_frequency,
        octets_per_frame,
        frame_duration,
        num_of_channels);

    for (size_t i = 0; i < MAX_STREAMS_SUPPORTED; i++)
    {
        if (!g_stream_info[i].conn_hdl)
        {
            g_stream_info[i].conn_hdl = conn_hdl;
            g_stream_info[i].is_cis = is_cis;
            g_stream_info[i].octets_per_frame = octets_per_frame;
            g_stream_info[i].num_of_channels = num_of_channels;
            g_stream_info[i].sampling_frequency = sampling_frequency;
            g_stream_info[i].frame_duration = frame_duration;
            g_stream_info[i].b_stream_active = 1;
            g_stream_info[i].num_bis = num_bis;
            WICED_BT_TRACE("[%s] conn_hdl 0x%x \n", __FUNCTION__, conn_hdl);
            return;
        }
    }
}

static ga_iso_audio_stream_info_t* get_stream_info(uint16_t conn_hdl)
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

    ga_iso_audio_stream_info_t* p_stream_info = get_stream_info(conn_hdl);
    if (!p_stream_info) return;

    memset(p_stream_info, 0, sizeof(ga_iso_audio_stream_info_t));
    WICED_BT_TRACE("[%s] deinit successful\n", __FUNCTION__);
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

wiced_result_t iso_audio_setup_data_path(uint16_t conn_hdl, uint16_t direction, wiced_bt_ga_bap_csc_t* p_csc, uint8_t bis_count)
{
    lc3_config_t codec_config;
    wiced_bool_t codec_config_sts;
    wiced_bt_isoc_data_path_direction_t data_path_dir = WICED_BLE_ISOC_DPD_MAX_DIRECTIONS;
    int numOfChannels = 1;
    wiced_bool_t is_bis = FALSE;

    WICED_BT_TRACE("[%s] conn_hdl [0x%x] dir [%d] p_csc [0x%x]\n", __FUNCTION__, conn_hdl, direction, p_csc);
    if (!p_csc) return WICED_BADARG;

    /* check if CIS is established or BIS is created before setting up data path */
    is_bis = wiced_bt_isoc_is_bis_created(conn_hdl);
    if (!is_bis)
    {
        return WICED_BADARG;
    }

    WICED_BT_TRACE("[%s] is_bis %d \n", __FUNCTION__, is_bis);

    /* determine num of channels required and configure codec accordingly
    (if not configured default to 1) */
    if (p_csc->audio_channel_allocation)
    {
        numOfChannels = count_set_bits(p_csc->audio_channel_allocation);
        if (numOfChannels > ISO_AUDIO_MAX_PARAM_COUNT)
        {
            return WICED_UNSUPPORTED;
        }
    }

//     p_csc->sampling_frequency = 48000;
//     p_csc->frame_duration = 10000;
//     p_csc->octets_per_codec_frame = 100;

    data_path_dir = (direction == WICED_BLE_ISOC_DPD_INPUT_BIT) ? WICED_BLE_ISOC_DPD_INPUT : WICED_BLE_ISOC_DPD_OUTPUT;

    WICED_BT_TRACE("[%s] dir %s sampleRate %d sduInterval %d octetsPerFrame %d numOfChannels %d conn_hdl 0x%x\n",
        __FUNCTION__,
        (data_path_dir == WICED_BLE_ISOC_DPD_INPUT) ? "Source" : "Sink",
        p_csc->sampling_frequency,
        p_csc->frame_duration,
        p_csc->octets_per_codec_frame,
        numOfChannels,
        conn_hdl);

    if (!p_csc->sampling_frequency || !p_csc->frame_duration || !p_csc->octets_per_codec_frame) return WICED_ERROR;

    codec_config.sampleRate = p_csc->sampling_frequency;
    codec_config.sduInterval = p_csc->frame_duration;
    codec_config.octetsPerFrame = p_csc->octets_per_codec_frame;
    codec_config.sampleWidthInBits = 16;

    // setup ISO data path and LC3 codec for INPUT from controller or OUTPUT to controller
    if (!wiced_bt_isoc_setup_data_path(conn_hdl, 0, data_path_dir, WICED_BLE_ISOC_DPID_HCI, 0))
    {
        return WICED_ERROR;
    }

    for (int i = 0; i < numOfChannels; i++)
    {
        codec_config_sts = (direction == WICED_BLE_ISOC_DPD_INPUT_BIT) ? lc3_codec_initializeEncoder(i, &codec_config)
            : lc3_codec_initializeDecoder(i, &codec_config);
        if (FALSE == codec_config_sts)
        {
            WICED_BT_TRACE_CRIT("[%s] lc3_codec_initialize NOT SUCCESSFUL", __FUNCTION__);
            return WICED_ERROR;
        }
    }

    init_stream_info(conn_hdl,
        0,
        p_csc->sampling_frequency,
        p_csc->octets_per_codec_frame,
        p_csc->frame_duration,
        numOfChannels, bis_count);

    audio_driver_init(direction, numOfChannels, codec_config.sampleRate, LINUX_ALSA_LATENCY);

    return WICED_SUCCESS;
}

static void rx_handler(uint16_t conn_hdl, uint8_t* p_data, uint32_t length)
{
    uint8_t lc3_data_l[MAX_INPUT_SAMPLE_SIZE_IN_BYTES] = { 0 };
    uint8_t lc3_data_r[MAX_INPUT_SAMPLE_SIZE_IN_BYTES] = { 0 };
    uint32_t decoded_data_size = 0;
    int sample_width_in_bytes = 2; //FIXME: remove hardcoding to 16 bit
    wiced_bt_ga_ascs_config_codec_args_t* codec_params_ptr = NULL;
    ga_iso_audio_stream_info_t* p_stream_info = get_stream_info(conn_hdl);
    uint8_t num_channels = 0;
    if (!p_stream_info) return;

    //TODO: chec if LC3 codec is initialized

    // Get the following from application data
    // num_of_channels
    // sample_rate
    // sdu_interval
    // sample_width_in_bytes
    decoded_data_size =
        wiced_bt_ga_bap_get_decoded_data_size(p_stream_info->sampling_frequency, p_stream_info->frame_duration);

    if (p_stream_info->num_bis > 1)
    {
        num_channels = 1;
    }
    else
    {
        num_channels = p_stream_info->num_of_channels;
    }

    // Validate received length against expected octets per frame
    if (length != (p_stream_info->octets_per_frame * num_channels))
    {
        WICED_BT_TRACE_CRIT("[%s] Expected %d bytes, received %d bytes (channel cnt - %d)",
            __FUNCTION__,
            (p_stream_info->octets_per_frame * num_channels),
            length,
            num_channels);
        return;
    }

    //Decode
    lc3_codec_Decode(0,
        0,
        p_data,
        p_stream_info->octets_per_frame,
        lc3_data_l,
        decoded_data_size * sample_width_in_bytes);

    if (p_stream_info->num_bis == 1)
    {
        if (2 == num_channels)
        {
            lc3_codec_Decode(1,
                             0,
                             p_data + p_stream_info->octets_per_frame,
                             p_stream_info->octets_per_frame,
                             lc3_data_r,
                             decoded_data_size * sample_width_in_bytes);
        }
    }

    audio_driver_write_non_interleaved_data(lc3_data_l,
        (2 == num_channels) ? lc3_data_r : NULL,
        sample_width_in_bytes,
        decoded_data_size * num_channels);
}

void iso_audio_init(const wiced_bt_cfg_isoc_t* p_iso_cfg)
{
    iso_dhm_init(NULL, rx_handler);
    lc3_codec_reset();
}

void iso_audio_remove_data_path(uint16_t conn_hdl, wiced_bt_isoc_data_path_bit_t dir, uint8_t* idx_list)
{
    wiced_bool_t is_bis = FALSE;
    ga_iso_audio_stream_info_t* p_stream_info = get_stream_info(conn_hdl);

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

    /* check if BIS is established before removing data path */
    is_bis = wiced_bt_isoc_is_bis_created(conn_hdl);
    if (!is_bis)
        return;

    if (!wiced_bt_isoc_remove_data_path(conn_hdl, FALSE, dir))
        WICED_BT_TRACE_CRIT("[%s] No active data path\n", __FUNCTION__);
}

void iso_audio_stop_stream(uint16_t conn_hdl)
{
    ga_iso_audio_stream_info_t* p_stream_info = get_stream_info(conn_hdl);
    p_stream_info->b_stream_active = 0;
}
