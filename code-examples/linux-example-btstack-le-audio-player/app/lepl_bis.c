/*
 * $ Copyright Cypress Semiconductor $
 */

#include "lepl.h"
#include "lepl_bis.h"

#define SUB_GROUP_CNT 1
#define BIS_CNT 1
#define CONTROLLER_BUSY 0x3A
#define BROADCASTING_DEVICE 0x0885
#define MONO_BIS_AUDIO_STREAM 1
#define MULTI_BIS_AUDIO_STREAM 2
#define MAX_SUPPORTED_BASE_SIZE 252

lepl_broadcast_source_cb_t g_broadcast_source_cb = {
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

lepl_broadcast_source_cb_t *lehs_get_broadcast_source_cb(void)
{
    return &g_broadcast_source_cb;
}

void lepl_bis_start_stream(wiced_bt_ga_bap_stream_config_t *p_stream_config)
{
    wiced_ble_isoc_create_big_param_t create_big_params = {0};
    lepl_broadcast_source_cb_t *p_big = &g_broadcast_source_cb;

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

    wiced_result_t res = wiced_ble_isoc_central_create_big(&create_big_params);
    WICED_BT_TRACE("[%s] res %x \n", __FUNCTION__, res);

    if (res == CONTROLLER_BUSY) //controller is busy
    {
        //controller is busy, create big again
        wiced_result_t res = wiced_ble_isoc_central_create_big(&create_big_params);
        WICED_BT_TRACE("[%s] create big retry %x \n", __FUNCTION__, res);
    }
}

le_audio_bap_broadcast_base_t *broadcast_source_bis_update_config(uint32_t broadcast_id,
                                                                     uint8_t *broadcast_code,
                                                                     uint8_t bis_cnt,
                                                                     uint32_t channel_counts,
                                                                     uint32_t sampling_freq,
                                                                     uint32_t frame_duration,
                                                                     uint16_t octets_per_codec_frame,
                                                                     wiced_bool_t enable_encryption)
{
    lepl_broadcast_source_cb_t *p_big = &g_broadcast_source_cb;

    WICED_MEMSET(p_big, 0, sizeof(lepl_broadcast_source_cb_t));

    p_big->adv_handle = 1;
    p_big->big_handle = 1;
    p_big->base.sub_group_cnt = 1;
    p_big->base.presentation_delay = 0x9c40;
    p_big->base.broadcast_id = broadcast_id;

    p_big->b_encryption = enable_encryption;
    if (p_big->b_encryption) memcpy(p_big->broadcast_code, broadcast_code, BAP_BROADCAST_CODE_SIZE);

    p_big->base.sub_group[0].codec_id.coding_format = LC3_CODEC_ID;
    p_big->base.sub_group[0].csc.sampling_frequency = sampling_freq;
    p_big->base.sub_group[0].csc.frame_duration = frame_duration;
    p_big->base.sub_group[0].csc.octets_per_codec_frame = octets_per_codec_frame;
    p_big->base.sub_group[0].bis_cnt = bis_cnt;

    if (bis_cnt == 1)
    {
        p_big->base.sub_group[0].csc.audio_channel_allocation = BAP_AUDIO_LOCATION_FRONT_LEFT;
        if (2 == channel_counts)
            p_big->base.sub_group[0].csc.audio_channel_allocation |= BAP_AUDIO_LOCATION_FRONT_RIGHT;
        p_big->base.sub_group[0].bis_config->bis_csc.audio_channel_allocation =
            p_big->base.sub_group[0].csc.audio_channel_allocation;
        p_big->base.sub_group[0].bis_config[0].bis_idx = 1;
    }
    else
    {
        for (int i = 0; i < bis_cnt; i++)
        {
            p_big->base.sub_group[0].bis_config[i].bis_idx = i + 1;
            p_big->base.sub_group[0].bis_config[i].bis_csc.audio_channel_allocation = (BAP_AUDIO_LOCATION_FRONT_LEFT
                                                                                      << i);
        }
    }

    p_big->base.sub_group[0].metadata.streaming_audio_ctx = 2;

    WICED_BT_TRACE("[%s] broadcast_code %A \n", __FUNCTION__, p_big->broadcast_code, BAP_BROADCAST_CODE_SIZE);

    return &p_big->base;
}

void broadcast_source_get_metadata(le_audio_public_broadcast_t *p_pub_br)
{
    uint8_t program_info[] = "IFX_BROADCAST_DEMO";
    p_pub_br->name_length = (uint8_t)strlen((char *)lepl_cfg_settings.device_name);
    p_pub_br->broadcast_name = lepl_cfg_settings.device_name;
    p_pub_br->appearance_value = BROADCASTING_DEVICE;
    p_pub_br->program_info_size = (uint8_t)sizeof(program_info);
    p_pub_br->program_info = program_info;
    /* uint8_t *p_data = p_pub_br->metadata;

    //LTV format
    UINT8_TO_STREAM(p_data, 2);
    UINT8_TO_STREAM(p_data, PBP_AUDIO_ACTIVE_STATE_TYPE);
    UINT8_TO_STREAM(p_data, 1);

    p_pub_br->metadata_length = p_data - p_pub_br->metadata;*/
    p_pub_br->metadata_length = 0;
}


wiced_result_t broadcast_source_start_broadcast_audio_announcements(uint8_t adv_sid,
                                                            uint32_t broadcast_id,
                                                            uint8_t *p_ext_adv_data,
                                                            uint8_t adv_size)
{
    wiced_ble_ext_adv_duration_config_t duration_cfg;

    if (!adv_sid || !broadcast_id) return WICED_BADARG;

    uint8_t addr_type = (lepl_cfg_settings.p_ble_cfg->rpa_refresh_timeout) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;

    wiced_ble_ext_adv_params_t params = {
        .event_properties = WICED_BLE_EXT_ADV_EVENT_PROPERTY_INCLUDE_TX_POWER,
        .primary_adv_int_min = 48,
        .primary_adv_int_max = 48,
        .primary_adv_channel_map = (BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39),
        .own_addr_type = addr_type,
        .peer_addr_type = BLE_ADDR_PUBLIC,
        .peer_addr = {0, 0, 0, 0, 0, 0},
        .adv_filter_policy = BTM_BLE_ADV_POLICY_ACCEPT_CONN_AND_SCAN,
        .adv_tx_power = 0x7f,
        .primary_adv_phy = WICED_BLE_EXT_ADV_PHY_1M,
        .secondary_adv_max_skip = 0,
        .secondary_adv_phy = WICED_BLE_EXT_ADV_PHY_1M,
        .adv_sid = 1,
        .scan_request_not = WICED_BLE_EXT_ADV_SCAN_REQ_NOTIFY_ENABLE,
        .primary_phy_opts = 0,
        .secondary_phy_opts = 0};

    wiced_ble_ext_adv_set_params(adv_sid, &params);

    if (addr_type == BLE_ADDR_RANDOM)
    {
        wiced_ble_ext_adv_set_random_address(adv_sid, g_lepl_gatt_cb.own_addr);
    }

    // Set adv data in LTV format
    wiced_ble_ext_adv_set_adv_data(adv_sid, adv_size, p_ext_adv_data);

    duration_cfg.adv_handle = adv_sid;
    duration_cfg.adv_duration = 0;
    duration_cfg.max_ext_adv_events = 0;

    // Start adv
    wiced_ble_ext_adv_enable(TRUE, 1, &duration_cfg);

    return WICED_SUCCESS;
}

wiced_result_t broadcast_source_start_basic_audio_announcements(uint8_t adv_sid, le_audio_bap_broadcast_base_t *p_base)
{
    uint8_t *p_basic_audio_ancmnt = NULL;
    uint8_t base_data_size = 0;

    if (!p_base) return WICED_BT_BADARG;

    p_basic_audio_ancmnt = wiced_bt_get_buffer(MAX_SUPPORTED_BASE_SIZE);
    if (!p_basic_audio_ancmnt) return WICED_BT_OUT_OF_HEAP_SPACE;

    memset(p_basic_audio_ancmnt, 0, MAX_SUPPORTED_BASE_SIZE);
    base_data_size = le_audio_bap_build_base_data(p_base, p_basic_audio_ancmnt, MAX_SUPPORTED_BASE_SIZE);
    if (!base_data_size) return WICED_BT_BADARG;

    {
        wiced_ble_padv_params_t params = {.adv_int_min = 200, .adv_int_max = 200, .adv_properties = 0};
        wiced_ble_padv_set_adv_params(adv_sid, &params);
        wiced_ble_padv_set_adv_data(adv_sid, base_data_size, p_basic_audio_ancmnt);
    }

    wiced_bt_free_buffer(p_basic_audio_ancmnt);

    wiced_ble_padv_enable_adv(adv_sid, TRUE);

    return WICED_BT_SUCCESS;
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
    le_audio_public_broadcast_t pub_br;
    wiced_result_t ret_sts = le_audio_bap_build_broadcast_annoncement_adv_data(p_ext_adv,
                                                                                  &len,
                                                                                  broadcast_id);
    if (ret_sts != WICED_SUCCESS) return ret_sts;
    pub_br.encryption = encryption;
    pub_br.sampling_frequency = sampling_frequency;
    pub_br.frame_duration = frame_duration;
    broadcast_source_get_metadata(&pub_br);
    ret_sts = le_audio_pbp_build_adv_data(p_ext_adv, len, &pub_br, &pub_br_len);
    *p_adv_len = len + pub_br_len;

    return WICED_SUCCESS;
}

wiced_result_t lepl_bis_configure_stream(uint32_t broadcast_id,
                                                     uint8_t *broadcast_code,
                                                     uint8_t bis_cnt,
                                                     uint32_t channel_counts,
                                                     uint32_t sampling_freq,
                                                     uint32_t frame_duration,
                                                     uint16_t octets_per_codec_frame,
                                                     wiced_bool_t enable_encryption)
{
    wiced_result_t ret_sts = WICED_ERROR;
    le_audio_bap_broadcast_base_t *p_base = NULL;
    uint8_t adv_sid = 1;

    p_base = broadcast_source_bis_update_config(broadcast_id,
                                                broadcast_code,
                                                bis_cnt,
                                                channel_counts,
                                                sampling_freq,
                                                frame_duration,
                                                octets_per_codec_frame,
                                                enable_encryption);
    if (!p_base)
    {
	    return ret_sts;
    }

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
        ret_sts = broadcast_source_start_broadcast_audio_announcements(adv_sid, p_base->broadcast_id, p_ext_adv, adv_len);
        if (WICED_SUCCESS != ret_sts) return ret_sts;
        ret_sts = broadcast_source_start_basic_audio_announcements(adv_sid, p_base);
        if (WICED_SUCCESS == ret_sts)
        {
            p_base->state = BAP_BROADCAST_STATE_CONFIGURED;
            WICED_BT_TRACE("[%s] BASE State [%d] \n", __FUNCTION__, p_base->state);
        }
    }
    return ret_sts;
}

wiced_result_t lepl_bis_disable_stream(void)
{
    lepl_broadcast_source_cb_t *p_big = &g_broadcast_source_cb;

    /* validate BASE state */
    if (BAP_BROADCAST_STATE_STREAMING != p_big->base.state)
    {
	    return WICED_ERROR;
    }
    lepl_isoc_dhm_remove_bis_datapath(p_big->bis_conn_id_list, p_big->bis_conn_id_count);

    return WICED_SUCCESS;
}

wiced_result_t lepl_bis_release_stream(void)
{
    wiced_ble_ext_adv_duration_config_t duration_cfg;
    lepl_broadcast_source_cb_t *p_big = &g_broadcast_source_cb;
    wiced_result_t ret_sts = WICED_ERROR;

    /* validate BASE state */
    if (BAP_BROADCAST_STATE_IDLE == p_big->base.state) return WICED_ERROR;

    wiced_ble_isoc_central_terminate_big(p_big->adv_handle, 0);

    duration_cfg.adv_handle = p_big->adv_handle;
    duration_cfg.adv_duration = 0;
    duration_cfg.max_ext_adv_events = 0;

    ret_sts = wiced_ble_ext_adv_enable(FALSE, 1, &duration_cfg);
    ret_sts = wiced_ble_padv_enable_adv(p_big->adv_handle, FALSE);

    p_big->base.state = BAP_BROADCAST_STATE_IDLE;
    WICED_BT_TRACE("[%s] BASE State [%d] \n", __FUNCTION__, p_big->base.state);

    return ret_sts;
}
