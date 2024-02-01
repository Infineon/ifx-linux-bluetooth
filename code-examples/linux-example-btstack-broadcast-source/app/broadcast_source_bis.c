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

/******************************************************************************
 * File Name: broadcast_source_bis.c
 *
 * Description: This is the source file for Broadcast_source CE application.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

/* BT Stack includes */

#include "wiced_bt_trace.h"

/* App Library includes */
#include "broadcast_source_iso_audio.h"

/* Application includes */
#include "broadcast_source_bis.h"
#include "wiced_bt_ga_pbp.h"

#define SUB_GROUP_CNT 1
#define BIS_CNT 1
#define CONTROLLER_BUSY 0x3A
#define BROADCASTING_DEVICE 0x0885
//#define PBP_AUDIO_ACTIVE_STATE_TYPE 0x08

extern wiced_bt_cfg_settings_t broadcast_source_cfg_settings;

broadcast_source_cb_t g_broadcast_source_cb = {
    .big_handle = 1,
    .adv_handle = 1,
    .b_encryption = TRUE,
    .base =
        {
            .broadcast_id = 0x123456,
            .sub_group_cnt = 1,
            .sub_group[0] = {.codec_id =
                                 {
                                     .coding_format = LC3_CODEC_ID,
                                 },
                             .csc =
                                 {
                                     .sampling_frequency = 48000,
                                     .frame_duration = 10000,
                                     .octets_per_codec_frame = 100,
                                     .audio_channel_allocation = 1,
                                 },
                             .metadata =
                                 {
                                     .streaming_audio_ctx = 2,
                                 },
                             .bis_cnt = BIS_CNT,
                             .bis_config[0] =
                                 {
                                     .bis_idx = 1,
                                 }},
        },
};

void broadcast_source_bis_isoc_cb(wiced_bt_isoc_event_t event, wiced_bt_isoc_event_data_t *p_ed)
{
    wiced_bt_isoc_terminated_data_t *p_big_terminated = NULL;
    wiced_bt_isoc_create_big_complete_t *p_create_big_sts = NULL;
    broadcast_source_cb_t *p_big = &g_broadcast_source_cb;
    wiced_result_t res = WICED_ERROR;

    WICED_BT_TRACE("[%s] event %d ", __FUNCTION__, event);

    switch (event)
    {
        case WICED_BLE_ISOC_BIG_CREATED: {
            p_create_big_sts = &p_ed->create_big;

            if (p_create_big_sts->sync_data.status)
            {
                WICED_BT_TRACE_CRIT("[%s] BIG Creation unsuccessful\n", __FUNCTION__);
                return;
            }

            /* Map BIS index and bis_conn_handle */
            p_big->bis_conn_id_count = p_create_big_sts->sync_data.num_bis;
            memcpy(p_big->bis_conn_id_list,
                   p_create_big_sts->sync_data.bis_conn_hdl_list,
                   p_big->bis_conn_id_count * sizeof(uint16_t));

            /* start setting up data paths for all the BIS streams */
            res = iso_audio_setup_data_path(p_big->bis_conn_id_list[0],
                                            WICED_BLE_ISOC_DPD_INPUT_BIT,
                                            &p_big->base.sub_group[0].csc);
            if (res)
            {
                WICED_BT_TRACE_CRIT("[%s] data path setup unsuccessful...(err:%d)\n", __FUNCTION__, res);
            }
        }
        break;

        case WICED_BLE_ISOC_BIG_TERMINATED: {
            p_big_terminated = &p_ed->terminate_big;

            // p_big->base.state = BAP_BROADCAST_STATE_CONFIGURED;
            // FIXME: Disabling state check to allow this fn. call immediately after
            // disabling stream. Should have a mechanism to queue release req to handle
            // from data path removed

            WICED_BT_TRACE("[%s] BASE State [%d] \n", __FUNCTION__, p_big->base.state);
        }
        break;

        case WICED_BLE_ISOC_DATA_PATH_SETUP: {
            if (p_ed->datapath.status)
            {
                WICED_BT_TRACE_CRIT("[%s] Data path setup not successful\n", __FUNCTION__);
                return;
            }

            p_big->base.state = BAP_BROADCAST_STATE_STREAMING;
            WICED_BT_TRACE("[%s] BASE State [%d] \n", __FUNCTION__, p_big->base.state);

            // start streaming as broadcast source
            iso_audio_start_stream(p_ed->datapath.conn_hdl);
        }
        break;

        case WICED_BLE_ISOC_DATA_PATH_REMOVED: {
            if (p_ed->datapath.status)
            {
                WICED_BT_TRACE_CRIT("[%s] Data path removal not successful\n", __FUNCTION__);
                return;
            }

            res = wiced_bt_isoc_central_terminate_big(p_big->big_handle, 0);
            iso_audio_stop_stream(p_ed->datapath.conn_hdl);
        }
        break;

        default:
            WICED_BT_TRACE("[%s] Unknown event %d ", __FUNCTION__, event);
            break;
    }
}

void broadcast_source_bis_init(wiced_bt_cfg_isoc_t *p_isoc_cfg)
{
    wiced_bt_isoc_register_cb(&broadcast_source_bis_isoc_cb);
    iso_audio_init(p_isoc_cfg);
}

void broadcast_source_bis_start_stream(wiced_bt_ga_bap_stream_config_t *p_stream_config)
{
    wiced_bt_isoc_create_big_param_t create_big_params = {0};
    broadcast_source_cb_t *p_big = &g_broadcast_source_cb;

    /* validate BASE state */
    if (BAP_BROADCAST_STATE_CONFIGURED != p_big->base.state) return;

    // create BIG
    create_big_params.big_handle = p_big->big_handle;
    create_big_params.adv_handle = p_big->adv_handle;

    for (int i = 0; i < p_big->base.sub_group_cnt; ++i)
    {
        create_big_params.num_bis += p_big->base.sub_group[i].bis_cnt;
    }

    create_big_params.sdu_interval = p_stream_config->sdu_interval;
    if (p_big->base.sub_group[0].csc.audio_channel_allocation ==
        (BAP_AUDIO_LOCATION_FRONT_LEFT | BAP_AUDIO_LOCATION_FRONT_RIGHT))
        create_big_params.max_sdu = p_big->base.sub_group[0].csc.octets_per_codec_frame * 2;
    else
        create_big_params.max_sdu = p_big->base.sub_group[0].csc.octets_per_codec_frame;
    create_big_params.max_trans_latency = p_stream_config->max_transport_latency;
    create_big_params.rtn = p_stream_config->retransmission_number;

    create_big_params.phy = WICED_BLE_ISOC_LE_2M_PHY;
    create_big_params.packing = WICED_BLE_ISOC_SEQUENTIAL_PACKING;
    create_big_params.framing = p_stream_config->framing;

    create_big_params.encrypt = p_big->b_encryption;

    if (p_big->b_encryption)
        memcpy(create_big_params.broadcast_code, p_big->broadcast_code, BAP_BROADCAST_CODE_SIZE);
    else
        memset(create_big_params.broadcast_code, 0, BAP_BROADCAST_CODE_SIZE);

    WICED_BT_TRACE("[%s] broadcast_code %A \n", __FUNCTION__, create_big_params.broadcast_code, 16);

    wiced_result_t res = wiced_bt_isoc_central_create_big(&create_big_params);
    WICED_BT_TRACE("[%s] res %x \n", __FUNCTION__, res);

    if (res == CONTROLLER_BUSY) //controller is busy
    {
        //controller is busy, create big again
        wiced_result_t res = wiced_bt_isoc_central_create_big(&create_big_params);
        WICED_BT_TRACE("[%s] create big retry %x \n", __FUNCTION__, res);
    }
}

wiced_bt_ga_bap_broadcast_base_t *broadcast_source_bis_update_config(uint32_t broadcast_id,
                                                                     uint8_t *broadcast_code,
                                                                     uint8_t bis_cnt,
                                                                     uint32_t channel_counts,
                                                                     uint32_t sampling_freq,
                                                                     uint32_t frame_duration,
                                                                     uint16_t octets_per_codec_frame,
                                                                     wiced_bool_t enable_encryption)
{
    uint8_t bis_idx = 0;
    broadcast_source_cb_t *p_big = &g_broadcast_source_cb;

    p_big->base.presentation_delay = 0;
    p_big->base.broadcast_id = broadcast_id;

    p_big->b_encryption = enable_encryption;
    if (p_big->b_encryption) memcpy(p_big->broadcast_code, broadcast_code, BAP_BROADCAST_CODE_SIZE);

    p_big->base.sub_group[0].csc.sampling_frequency = sampling_freq;
    p_big->base.sub_group[0].csc.frame_duration = frame_duration;
    p_big->base.sub_group[0].csc.octets_per_codec_frame = octets_per_codec_frame;
    p_big->base.sub_group[0].bis_cnt = bis_cnt;

    p_big->base.sub_group[0].csc.audio_channel_allocation = BAP_AUDIO_LOCATION_FRONT_LEFT;
    if (2 == channel_counts) p_big->base.sub_group[0].csc.audio_channel_allocation |= BAP_AUDIO_LOCATION_FRONT_RIGHT;

    WICED_BT_TRACE("[%s] broadcast_code %A \n", __FUNCTION__, p_big->broadcast_code, BAP_BROADCAST_CODE_SIZE);

    return &p_big->base;
}

void broadcast_source_get_metadata(wiced_bt_ga_public_broadcast *p_pub_br)
{
    uint8_t program_info[] = "IFX_BROADCAST_DEMO";
    p_pub_br->name_length = strlen(broadcast_source_cfg_settings.device_name);
    p_pub_br->broadcast_name = broadcast_source_cfg_settings.device_name;
    p_pub_br->appearance_value = BROADCASTING_DEVICE;
    p_pub_br->program_info_size = sizeof(program_info);
    p_pub_br->program_info = program_info;
    /* uint8_t *p_data = p_pub_br->metadata;

    //LTV format
    UINT8_TO_STREAM(p_data, 2);
    UINT8_TO_STREAM(p_data, PBP_AUDIO_ACTIVE_STATE_TYPE);
    UINT8_TO_STREAM(p_data, 1);

    p_pub_br->metadata_length = p_data - p_pub_br->metadata;*/
    p_pub_br->metadata_length = 0;
}

wiced_result_t broadcast_source_build_broadcast_adv_data(uint8_t adv_sid,
                                                        uint32_t broadcast_id,
                                                        uint32_t sampling_frequency,
                                                        uint16_t frame_duration,
                                                        wiced_bool_t encryption,
                                                        uint8_t *p_adv_len,
                                                        uint8_t *p_ext_adv)
{
    WICED_BT_TRACE("%s", __FUNCTION__);
    uint8_t len = 0;
    uint32_t pub_br_len = 0;
    wiced_bt_ga_public_broadcast pub_br;
    wiced_result_t ret_sts =
        wiced_bt_ga_bap_build_broadcast_annoncement_adv_data(p_ext_adv, &len, broadcast_id);
    if (ret_sts != WICED_SUCCESS) return ret_sts;
    pub_br.encryption = encryption;
    pub_br.sampling_frequency = sampling_frequency;
    pub_br.frame_duration = frame_duration;
    broadcast_source_get_metadata(&pub_br);
    ret_sts = wiced_bt_ga_pbp_build_adv_data(p_ext_adv, len, &pub_br, &pub_br_len);
    *p_adv_len = len + pub_br_len;

    return WICED_SUCCESS;
}

wiced_result_t broadcast_source_bis_configure_stream(uint32_t broadcast_id,
                                                     uint8_t *broadcast_code,
                                                     uint8_t bis_cnt,
                                                     uint32_t channel_counts,
                                                     uint32_t sampling_freq,
                                                     uint32_t frame_duration,
                                                     uint16_t octets_per_codec_frame,
                                                     wiced_bool_t enable_encryption)
{
    wiced_bt_device_address_t addr = {0};
    wiced_result_t ret_sts = WICED_ERROR;
    wiced_bt_ga_bap_broadcast_base_t *p_base = NULL;
    uint8_t adv_sid = 1;

    p_base = broadcast_source_bis_update_config(broadcast_id,
                                                broadcast_code,
                                                bis_cnt,
                                                channel_counts,
                                                sampling_freq,
                                                frame_duration,
                                                octets_per_codec_frame,
                                                enable_encryption);
    if (!p_base) return ret_sts;

    uint8_t adv_len = 0;
    uint8_t p_ext_adv[250] = {0};
    ret_sts = broadcast_source_build_broadcast_adv_data(adv_sid,
                                                       broadcast_id,
                                                       sampling_freq,
                                                       frame_duration,
                                                       enable_encryption,
                                                       &adv_len,
                                                       p_ext_adv);
    if (ret_sts == WICED_SUCCESS)
    {
        ret_sts = wiced_bt_ga_bap_broadcast_configure(adv_sid, p_base, p_ext_adv, adv_len);
        if (WICED_SUCCESS == ret_sts)
        {
            p_base->state = BAP_BROADCAST_STATE_CONFIGURED;
            WICED_BT_TRACE("[%s] BASE State [%d] \n", __FUNCTION__, p_base->state);
        }
    }
    return ret_sts;
}

wiced_result_t broadcast_source_bis_disable_stream(void)
{
    uint32_t big_handle = 1;
    broadcast_source_cb_t *p_big = &g_broadcast_source_cb;
    uint8_t lc3_index = 0;

    /* validate BASE state */
    if (BAP_BROADCAST_STATE_STREAMING != p_big->base.state) return WICED_ERROR;

    iso_audio_remove_data_path(p_big->bis_conn_id_list[0], WICED_BLE_ISOC_DPD_INPUT_BIT, &lc3_index);

    return WICED_SUCCESS;
}

wiced_result_t broadcast_source_bis_release_stream(void)
{
    wiced_bt_ble_ext_adv_duration_config_t duration_cfg;
    broadcast_source_cb_t *p_big = &g_broadcast_source_cb;
    wiced_result_t ret_sts = WICED_ERROR;

    /* validate BASE state */
    // if (BAP_BROADCAST_STATE_CONFIGURED != p_big->base.state) return WICED_ERROR;
    // FIXME: Disabling state check to allow this fn. call immediately after
    // disabling stream. Should have a mechanism to queue release req to handle
    // from data path removed

    wiced_bt_isoc_central_terminate_big(p_big->adv_handle, 0);

    duration_cfg.adv_handle = p_big->adv_handle;
    duration_cfg.adv_duration = 0;
    duration_cfg.max_ext_adv_events = 0;

    ret_sts = wiced_bt_ble_start_ext_adv(FALSE, 1, &duration_cfg);
    ret_sts = wiced_bt_ble_start_periodic_adv(p_big->adv_handle, FALSE);

    p_big->base.state = BAP_BROADCAST_STATE_IDLE;
    WICED_BT_TRACE("[%s] BASE State [%d] \n", __FUNCTION__, p_big->base.state);

    return ret_sts;
}
