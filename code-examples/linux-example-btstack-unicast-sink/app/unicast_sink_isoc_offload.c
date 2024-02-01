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

#if ISOC_OFFLOAD
/* Application includes */
#include "unicast_sink_gatt.h"
#include "unicast_sink_isoc.h"

/* App Library includes */
#include "wiced_audio_manager.h"
#include "wiced_bt_codec_cs47l35.h"

/* BT Stack includes */
#include "wiced_bt_audio.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_ga_ascs.h"
#include "wiced_bt_ga_bap.h"
#include "wiced_bt_trace.h"

/*

Data_Path_ID' => (Interface << 5) + slot

Bits 5:7 = hardware interface
Bits 0:6 = slot, for TDM bus slot = slot, for I2S slot 0 = left and slot 1 = right.

Hardware interfaces available are:
    0 = HCI  (illegal value for Configure_Data_Path)
    1 = ARIP_I2S master only
    2 = PCM2, I2S master only (H2 only, PCM is removed in H1)
    3 = MXTDM_0 in I2S mode (H1 only)
    4 = MXTDM_0 in TDM mode(H1 only)
    5 = MXTDM_1 in I2S mode (H1 only)
    6 = MXTDM_1 in TDM mode (H1 only)

*/
#define ARIP_I2S_DATA_PATH_ID (1 << 5)

int32_t stream_id;
extern int32_t audio_driver_config_frequency(int32_t sampling_rate,
                                             int32_t no_of_channels,
                                             int32_t bits_per_sample,
                                             am_audio_io_device_t sink);
extern void audio_driver_init_vol();

const wiced_bt_audio_config_buffer_t unicast_sink_audio_buf_config = {.role = WICED_AUDIO_SINK_ROLE,
                                                                      .audio_tx_buffer_size = 0,
                                                                      .audio_codec_buffer_size = 0x10000};

void unicast_sink_isoc_dhm_init(const wiced_bt_cfg_isoc_t *p_iso_cfg)
{
    /* required to call this for ARIP/I2S configuration */
    wiced_audio_buffer_initialize(unicast_sink_audio_buf_config);
    wiced_am_init();

    /* Open and Close the Codec now (Boot time) to prevent DSP download delay later */
    /* Pre-initialization also can prevent pop sound for playing audio first time */
    stream_id = wiced_am_stream_open(A2DP_PLAYBACK);

    if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        WICED_BT_TRACE("wiced_am_stream_open failed\n");
        return;
    }

    if (wiced_am_stream_close(stream_id) != WICED_SUCCESS)
        WICED_BT_TRACE("Err: wiced_am_stream_close\n");
}

void unicast_sink_isoc_dhm_setup_stream(uint16_t conn_hdl,
                                        wiced_bool_t is_cis,
                                        uint16_t direction,
                                        wiced_bt_ga_bap_csc_t *p_csc)
{
    uint32_t am_playback_freq = 0;
    uint8_t csc_val = 0;
    uint8_t csc[] = {3,                                        // Length
                     BAP_CODEC_CONFIG_SAMPLING_FREQUENCY_TYPE, // Type
                     (1 << 5),                                 // (bit pos + 1) * 8khz ((5+1)*8 = 48k)
                     0,
                     2,                                    // Length
                     BAP_CODEC_CONFIG_FRAME_DURATION_TYPE, // Type
                     2,
                     3,                                            // Length
                     BAP_CODEC_CONFIG_OCTETS_PER_CODEC_FRAME_TYPE, // Type
                     120,
                     0};

    // return if the configures frequency is not supported
    /* reference for sampling freq calc in the controller

        do
        {
            sampleRate += 8000;
            pConfig->configData.sampleFrequency >>= 1;
        } while (pConfig->configData.sampleFrequency);

    */
    switch (p_csc->sampling_frequency)
    {
    case 48000:
        am_playback_freq = AM_PLAYBACK_SR_48K;
        csc_val = 1 << 5;
        break;

    case 32000:
        am_playback_freq = AM_PLAYBACK_SR_32K;
        csc_val = 1 << 3;
        break;

    case 16000:
        am_playback_freq = AM_PLAYBACK_SR_16K;
        csc_val = 1 << 1;
        break;

    default:
        WICED_BT_TRACE("[%s] Unsupported sampling frequency selected : %d\n", __FUNCTION__, p_csc->sampling_frequency);
        return;
        break;
    }

    // TODO: allow only sink directions (arg direction is not used)
    stream_id = audio_driver_config_frequency(am_playback_freq, DEFAULT_CH, DEFAULT_BITSPSAM, AM_HEADPHONES);
    wiced_bt_isoc_configure_data_path(WICED_BLE_ISOC_DPD_OUTPUT, ARIP_I2S_DATA_PATH_ID);
    csc[2] = csc_val; // update the sampling freq to the one received thru  args
    wiced_bt_isoc_setup_data_path(conn_hdl,
                                  TRUE,
                                  WICED_BLE_ISOC_DPD_OUTPUT,
                                  ARIP_I2S_DATA_PATH_ID,
                                  15000, /* controller delay */
                                  sizeof(csc),
                                  csc);
}

void unicast_sink_isoc_dhm_start_stream(void)
{
    if (WICED_SUCCESS != wiced_am_stream_start(stream_id))
    {
        WICED_BT_TRACE("wiced_am_stream_start failed\n");
    }
    wiced_bt_codec_cs47l35_set_sink_mono2stereo();

    audio_driver_init_vol();
}

void unicast_sink_isoc_dhm_free_stream(uint16_t conn_hdl,
                                       wiced_bt_isoc_data_path_bit_t dir,
                                       uint8_t *idx_list,
                                       int num_channels)
{
    if (wiced_am_stream_stop(stream_id) != WICED_SUCCESS)
    {
        WICED_BT_TRACE("Err: wiced_am_stream_stop\n");
    }
    if (wiced_am_stream_close(stream_id) != WICED_SUCCESS)
    {
        WICED_BT_TRACE("Err: wiced_am_stream_close\n");
    }
}
#endif
