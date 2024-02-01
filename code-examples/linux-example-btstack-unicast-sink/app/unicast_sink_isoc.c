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

#include "unicast_sink_isoc.h"

#include "audio_driver.h"
#include "iso_data_handler.h"
#include "lc3_codec.h"
#include "data_types.h"
#include "app_bt_utils.h"

#include "wiced_bt_ga_ascs.h"

#include "log.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define LINUX_ALSA_LATENCY              500U
#define LC3_DATA_BUFFER_SIZE            (480*2)

/******************************************************************************
 *                              EXTERNS
 *****************************************************************************/
extern BOOL32 isStreaming;

/******************************************************************************
 * Function Name: num_complete_handler 
 ******************************************************************************
 * Summary: no use now, Future use
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
******************************************************************************/ 
static void num_complete_handler(uint16_t conn_hdl, uint16_t num_sent)
{
}

/******************************************************************************
 * Function Name: rx_handler
 ******************************************************************************
 * Summary: isoc callback function, use for process receive cis data
 *
 * Parameters:
 *  uint16_t conn_hdl
 *  uint8_t *p_data
 *  uint32_t length
 *
 * Return:
 *  None
 *
******************************************************************************/
static void rx_handler(uint16_t conn_hdl, uint8_t *p_data, uint32_t length)
{
    uint8_t lc3_data_l[LC3_DATA_BUFFER_SIZE] = {0};
    uint8_t lc3_data_r[LC3_DATA_BUFFER_SIZE] = {0};
    uint32_t decoded_data_size = 0;
    int sample_width_in_bytes = 2; 
    wiced_bt_ga_ascs_config_codec_args_t *codec_params_ptr = NULL;
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);

    if (!p_stream_info)
    {
        WICED_BT_TRACE_CRIT("[%s] stream is null", __FUNCTION__);
        return;
    }


    // Get the following from application data
    // num_of_channels
    // sample_rate
    // sdu_interval
    // sample_width_in_bytes
    decoded_data_size = wiced_bt_ga_bap_get_decoded_data_size(p_stream_info->sampling_frequency, p_stream_info->frame_duration);

    // Validate received length against expected octets per frame
    if (length != (p_stream_info->octets_per_frame * p_stream_info->num_of_channels))
    {
#ifndef SUPPORT_PLC 
        TRACE_ERR("Expected %d bytes, received %d bytes (channel cnt - %d)",
                            (p_stream_info->octets_per_frame * p_stream_info->num_of_channels),
                            length,
                            p_stream_info->num_of_channels);

        return;
#else
        if (!isStreaming) {
            return;
        }
        lc3_codec_Decode(0,
                         0,
                         NULL,
                         p_stream_info->octets_per_frame,
                         lc3_data_l,
                         decoded_data_size * sample_width_in_bytes);

        if (2 == p_stream_info->num_of_channels)
        {
            lc3_codec_Decode(1,
                             0,
                             NULL,
                             p_stream_info->octets_per_frame,
                             lc3_data_r,
                             decoded_data_size * sample_width_in_bytes);
        }
        bt_rx_fps_show(SHOW_FPS_SEC);

        audio_driver_write_non_interleaved_data(lc3_data_l,
                                                (2 == p_stream_info->num_of_channels) ? lc3_data_r : NULL,
                                                sample_width_in_bytes,
                                                decoded_data_size * p_stream_info->num_of_channels);
        return;
#endif
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
    isStreaming = TRUE;
    bt_rx_fps_show(SHOW_FPS_SEC);

    audio_driver_write_non_interleaved_data(lc3_data_l,
                                            (2 == p_stream_info->num_of_channels) ? lc3_data_r : NULL,
                                            sample_width_in_bytes,
                                            decoded_data_size * p_stream_info->num_of_channels);
}

/******************************************************************************
 * Function Name: unicast_sink_isoc_dhm_init
 ******************************************************************************
 * Summary: init isoc and init lc3 codec, iso_dhm_init api in iso_data_handler_module_lib
 *
 * Parameters:
 *  const wiced_bt_cfg_isoc_t *p_iso_cfg
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_isoc_dhm_init(const wiced_bt_cfg_isoc_t *p_iso_cfg)
{
    iso_dhm_init(num_complete_handler, rx_handler);
    lc3_codec_reset();
}

/******************************************************************************
 * Function Name: count_set_bits
 ******************************************************************************
 * Summary: static funciton, count the bits in set
 *
 * Parameters:
 *  uint32_t n 
 *
 * Return:
 *  uint32_t
 *
******************************************************************************/
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

/******************************************************************************
 * Function Name: unicast_sink_isoc_dhm_setup_stream
 ******************************************************************************
 * Summary: sink setup stream, after cis connection.
 *      setup codec configure
 *      init stream
 *      init audio driver
 *
 * Parameters:
 *  uint16_t conn_hdl
 *  wiced_bool_t is_cis
 *  uint16_t direction
 *  wiced_bt_ga_bap_csc_t *p_csc
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_isoc_dhm_setup_stream(uint16_t conn_hdl,
                                        wiced_bool_t is_cis,
                                        uint16_t direction,
                                        wiced_bt_ga_bap_csc_t *p_csc)
{
    lc3_config_t codec_config;
    wiced_bool_t codec_config_sts;
    int channel_count = 1;
    wiced_bt_isoc_data_path_direction_t data_path_dir = WICED_BLE_ISOC_DPD_MAX_DIRECTIONS;

    /* determine num of channels required and configure codec accordingly
    (if not configured default to 1) */
    if (p_csc->audio_channel_allocation)
    {
        channel_count = count_set_bits(p_csc->audio_channel_allocation);
    }

    data_path_dir = (direction == WICED_BLE_ISOC_DPD_INPUT_BIT) ? WICED_BLE_ISOC_DPD_INPUT : WICED_BLE_ISOC_DPD_OUTPUT;

    // setup ISO data path and LC3 codec for INPUT from controller or OUTPUT to controller
    if (!wiced_bt_isoc_setup_data_path(conn_hdl, is_cis, data_path_dir, WICED_BLE_ISOC_DPID_HCI, 0))
    {
        return;
    }

    codec_config.sampleRate = p_csc->sampling_frequency;
    codec_config.sduInterval = p_csc->frame_duration;
    codec_config.octetsPerFrame = p_csc->octets_per_codec_frame;
    codec_config.sampleWidthInBits = 16;

    for (int i = 0; i < channel_count; i++)
    {
        codec_config_sts = (direction == WICED_BLE_ISOC_DPD_INPUT_BIT) ? lc3_codec_initializeEncoder(i, &codec_config)
                                                                       : lc3_codec_initializeDecoder(i, &codec_config);
        if (FALSE == codec_config_sts)
        {
            WICED_BT_TRACE_CRIT("[%s] lc3_codec_initialize NOT SUCCESSFUL", __FUNCTION__);
            return;
        }
    }

    init_stream_info(conn_hdl,
                     is_cis,
                     p_csc->sampling_frequency,
                     p_csc->octets_per_codec_frame,
                     p_csc->frame_duration,
                     channel_count);

    audio_driver_init(direction, channel_count, codec_config.sampleRate, LINUX_ALSA_LATENCY);
}

/******************************************************************************
 * Function Name: unicast_sink_isoc_dhm_free_stream
 ******************************************************************************
 * Summary: use for release lc3 Encoder (if source ASE enable) or Decode
 *      when iso_audio_remove_data_path
 *
 * Parameters:
 *  uint16_t conn_hdl
 *  wiced_bt_isoc_data_path_bit_t dir
 *  uint8_t *idx_list
 *  int num_channels
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_isoc_dhm_free_stream(uint16_t conn_hdl,
                                       wiced_bt_isoc_data_path_bit_t dir,
                                       uint8_t *idx_list,
                                       int num_channels)
{
    audio_driver_deinit(dir);

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
}

/******************************************************************************
 * Function Name:  unicast_sink_isoc_dhm_start_stream
 ******************************************************************************
 * Summary: no use, future use
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_isoc_dhm_start_stream(void)
{
}
