/*
 * $ Copyright Cypress Semiconductor $
 */

#include "lepl.h"

wiced_bt_ga_ascs_config_qos_args_t cap_qos_config = {.cig_id = 1,
                                                     .cis_id = 1,
                                                     .phy = WICED_BLE_ISOC_LE_2M_PHY,
                                                     .max_sdu = 240,
                                                     .presentation_delay = 0x9c40};
le_audio_cap_start_unicast_param_t unicast_param;

wiced_bool_t le_audio_cap_utils_fill_cis_config_data(wiced_ble_isoc_cis_config_t *data,
                                                     uint8_t cis_id,
                                                     uint16_t max_sdu_c_to_p,
                                                     uint16_t max_sdu_p_to_c,
                                                     wiced_ble_isoc_phy_t phy_c_to_p,
                                                     wiced_ble_isoc_phy_t phy_p_to_c,
                                                     uint8_t rtn_c_to_p,
                                                     uint8_t rtn_p_to_c)
{
    if (data) {
        data->cis_id = cis_id;
        data->max_sdu_c_to_p = max_sdu_c_to_p;
        data->max_sdu_p_to_c = max_sdu_p_to_c;
        data->phy_c_to_p = phy_c_to_p;
        data->phy_p_to_c = phy_p_to_c;
        data->rtn_c_to_p = rtn_c_to_p;
        data->rtn_p_to_c = rtn_p_to_c;
        return WICED_TRUE;
    }
    return WICED_FALSE;
}

wiced_result_t le_audio_cap_utils_create_cig(wiced_bt_ga_ascs_config_qos_args_t *p_qos_params,
                                             uint8_t cig_id,
                                             uint8_t cis_count)
{
    int index;
    uint8_t cis_id;
    wiced_ble_isoc_cis_config_t cis_config_list[2];
    wiced_ble_isoc_cig_param_t cig_param = {0};

    cig_param.cig_id = cig_id; // Assign CIG ID
    cig_param.cis_count = cis_count;
    cig_param.sdu_interval_c_to_p = p_qos_params->sdu_interval;
    cig_param.sdu_interval_p_to_c = p_qos_params->sdu_interval;
    cig_param.max_trans_latency_c_to_p = p_qos_params->max_transport_latency;
    cig_param.max_trans_latency_p_to_c = p_qos_params->max_transport_latency;
    cig_param.packing = WICED_BLE_ISOC_SEQUENTIAL_PACKING;
    cig_param.framing = p_qos_params->framing;
    cig_param.p_cis_config_list = cis_config_list;
    for (index = 0; index < cig_param.cis_count; index++) {
        cis_id = (p_qos_params->cis_id) + index;
        WICED_BT_TRACE("[%s] cis_id %d", __FUNCTION__, cis_id);
        le_audio_cap_utils_fill_cis_config_data(&cig_param.p_cis_config_list[index],
                                                cis_id,
                                                p_qos_params->max_sdu,
                                                p_qos_params->max_sdu,
                                                p_qos_params->phy,
                                                p_qos_params->phy,
                                                p_qos_params->retransmission_number,
                                                p_qos_params->retransmission_number);
    }

    p_qos_params->cig_id = cig_param.cig_id;

    WICED_BT_TRACE("CIG ID %d SDU Interval (M->S/S->M) (0x%x/0x%x) SCA %d Packing %d Framing %d ",
                   cig_param.cig_id,
                   cig_param.sdu_interval_c_to_p,
                   cig_param.sdu_interval_p_to_c,
                   cig_param.worst_case_sca,
                   cig_param.packing,
                   cig_param.framing);

    WICED_BT_TRACE("Trans Latency (M->S/S->M) (%d/%d) CIS count %d",
                   cig_param.max_trans_latency_c_to_p,
                   cig_param.max_trans_latency_p_to_c,
                   cig_param.cis_count);

    WICED_BT_TRACE("CIS ID %d SDU (M->S/S->M) (%d/%d) PHY (M->S/S->M) (%d/%d) RTN (M->S/S->M) (%d/%d)",
                   cig_param.p_cis_config_list[0].cis_id,
                   cig_param.p_cis_config_list[0].max_sdu_c_to_p,
                   cig_param.p_cis_config_list[0].max_sdu_p_to_c,
                   cig_param.p_cis_config_list[0].phy_c_to_p,
                   cig_param.p_cis_config_list[0].phy_p_to_c,
                   cig_param.p_cis_config_list[0].rtn_c_to_p,
                   cig_param.p_cis_config_list[0].rtn_p_to_c);

    return wiced_ble_isoc_central_set_cig_param(&cig_param);
}

void lepl_cap_update_context_type(le_audio_cap_start_unicast_param_t *param, wiced_bt_ga_bap_context_type_t context_type);

    wiced_bt_ga_ascs_config_codec_args_t cap_codec_config = {
    .codec_id =
        {
            .coding_format = LC3_CODEC_ID,
            .company_id = 0,
            .vendor_specific_codec_id = 0,
        },
    .target_latency = 1,
    .target_phy = WICED_BT_ASCS_PHY_2M,
};

static void lepl_get_config(lepl_stream_config_t *p_unicast_stream_config,
                               le_audio_cap_start_unicast_param_t *param)
{
    param->dir = WICED_BT_CAP_DIRECTION_SOURCE;
    param->context_type = p_unicast_stream_config->ctx_type;

    cap_qos_config.framing = p_unicast_stream_config->stream_config->framing;
    cap_qos_config.retransmission_number = p_unicast_stream_config->stream_config->retransmission_number;
    cap_qos_config.max_transport_latency = p_unicast_stream_config->stream_config->max_transport_latency;
    cap_qos_config.sdu_interval = p_unicast_stream_config->stream_config->sdu_interval;
    if (p_unicast_stream_config->num_devices > 1)
        cap_qos_config.max_sdu = p_unicast_stream_config->stream_config->octets_per_codec_frame;
    else
        cap_qos_config.max_sdu = p_unicast_stream_config->stream_config->octets_per_codec_frame * 2;
    param->p_codec_configuration = &cap_codec_config;
    param->p_qos_configuration = &cap_qos_config;

    param->p_codec_configuration->csc.sampling_frequency = p_unicast_stream_config->stream_config->sampling_frequency;
    param->p_codec_configuration->csc.frame_duration = p_unicast_stream_config->stream_config->frame_duration;
    param->p_codec_configuration->csc.audio_channel_allocation =
        BAP_AUDIO_LOCATION_FRONT_LEFT | BAP_AUDIO_LOCATION_FRONT_RIGHT;
    param->p_codec_configuration->csc.octets_per_codec_frame = p_unicast_stream_config->stream_config->octets_per_codec_frame;
    param->p_codec_configuration->csc.lc3_blocks_per_sdu = 1;

   // param->metadata.p_upper_layer_data = cap_temp_ccid_buffer;
   // param->metadata.upper_layer_data_length = sizeof(cap_temp_ccid_buffer);
   // le_audio_cap_fill_metadata(1, cap_ccid_list, BAP_CONTEXT_TYPE_MEDIA, &param->metadata);
}

void lepl_cap_set_next_application_state(lepl_app_state_t state, uint32_t codec_config)
{
    lepl_app_state_info_t *p_app_state = &g_lepl_gatt_cb.app_state;
    lepl_app_state_t current_state = p_app_state->current_state;

    if (state != LEPL_APP_STATE_IDLE)
    {
        if ((current_state > LEPL_APP_STATE_IDLE) && (current_state < LEPL_APP_STATE_IN_TRANSIT))
        {
            p_app_state->paused_state = current_state;
            p_app_state->paused_strm_codec = g_lepl_gatt_cb.app_state.current_strm_codec;
        }
    }

    p_app_state->current_strm_codec = codec_config;
    p_app_state->transit_info.initial_state = current_state;
    p_app_state->transit_info.final_state = state;
    p_app_state->current_state = LEPL_APP_STATE_IN_TRANSIT;
}

lepl_app_state_t lepl_cap_get_application_state(void)
{
    return g_lepl_gatt_cb.app_state.current_state;
}

lepl_app_state_t lepl_cap_get_application_final_state(void)
{
    return g_lepl_gatt_cb.app_state.transit_info.final_state;
}

void lepl_cap_reset_application_state(void)
{
    lepl_app_state_info_t *p_app_state = &g_lepl_gatt_cb.app_state;
    p_app_state->current_state = LEPL_APP_STATE_IDLE;
    p_app_state->paused_state = LEPL_APP_STATE_IDLE;
    p_app_state->transit_info.initial_state = LEPL_APP_STATE_IDLE;
    p_app_state->transit_info.final_state = LEPL_APP_STATE_IDLE;
    p_app_state->current_strm_codec = 0xFF;
    p_app_state->paused_strm_codec = 0XFF;
}

static void lepl_cap_update_application_state(uint16_t conn_id, le_audio_cap_event_data_t *p_event_data)
{
    lepl_app_state_info_t *p_app_state = &g_lepl_gatt_cb.app_state;
    if (p_app_state->current_state != LEPL_APP_STATE_IN_TRANSIT)
        return;

    if ((p_event_data->group_state == WICED_BT_GA_ASCS_STATE_CODEC_CONFIGURED) &&
        !p_event_data->p_app_data->is_disconnecting)
        return;

    if ((p_event_data->group_state == WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED) && !p_event_data->p_app_data->is_disabling)
        return;

    switch (p_event_data->group_state)
    {
    case WICED_BT_GA_ASCS_STATE_IDLE:
    case WICED_BT_GA_ASCS_STATE_CODEC_CONFIGURED:
    case WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED:
        for (int i = 0; (i < p_event_data->p_app_data->num_devices) && (i < MAX_CONNECTION_INSTANCE); i++)
        {
            uint8_t num_ase = p_event_data->p_app_data->device_info_list[i].num_ase;
            for (int index = 0; index < num_ase; index++)
            {
                wiced_bt_ga_ascs_config_qos_args_t *p_qos =
                    &p_event_data->p_app_data->device_info_list[i].ascs_data[index]->qos_configured;
                uint16_t cis_conn_handle = wiced_ble_isoc_central_get_cis_conn_handle(p_qos->cig_id, p_qos->cis_id);
                if (!wiced_ble_isoc_is_cis_connected_with_conn_hdl(cis_conn_handle))
                {
                    wiced_result_t res = wiced_ble_isoc_disconnect_cis(cis_conn_handle);
                    WICED_BT_TRACE("[%s]  Disconnecting cis status %d", __FUNCTION__, res);
                }
            }
        }

        if ((lepl_get_call_control_server_state() == CALL_CONTROL_SERVER_STATE_INBAND_RINGTONE) &&
            (p_app_state->transit_info.final_state == LEPL_APP_STATE_CALL))
        {
            lepl_ccs_start_streaming_convo(conn_id);
            return;
        }

        if (p_app_state->transit_info.final_state != LEPL_APP_STATE_IDLE)
        {
            switch (p_app_state->transit_info.final_state)
            {
            case LEPL_APP_STATE_MEDIA:
                lepl_mcs_play(conn_id, p_app_state->current_strm_codec);
                break;
            case LEPL_APP_STATE_CALL:
                lepl_ccs_set_incoming_remote_call(conn_id);
                break;
            case LEPL_APP_STATE_MIC:
                lepl_start_voice_capture(conn_id, p_app_state->current_strm_codec);
                break;
            default:
                WICED_BT_TRACE_CRIT("[%s] Not a valid state %d",
                                    __FUNCTION__, p_app_state->transit_info.final_state);
                break;
            }
            return;
        }
        else if (p_app_state->paused_state)
        {
            switch (p_app_state->paused_state)
            {
            case LEPL_APP_STATE_MEDIA:
                lepl_mcs_play(conn_id, p_app_state->paused_strm_codec);
                break;
            case LEPL_APP_STATE_MIC:
                lepl_start_voice_capture(conn_id, p_app_state->paused_strm_codec);
                break;
            default:
                break;
            }
            p_app_state->current_state = LEPL_APP_STATE_IDLE;
            lepl_cap_set_next_application_state(p_app_state->paused_state, p_app_state->paused_strm_codec);
            p_app_state->paused_state = LEPL_APP_STATE_IDLE;
            return;
        }
        break;
    case WICED_BT_GA_ASCS_STATE_STREAMING:
        if (lepl_get_call_control_server_state() == CALL_CONTROL_SERVER_STATE_INBAND_RINGTONE)
        {
            return;
        }
        break;
    default:
        return;
    }
    p_app_state->current_state = p_app_state->transit_info.final_state;
}

void lepl_cap_get_coordinated_set_members(uint16_t conn_id)
{
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);

    if (!p_clcb)
    {
        WICED_BT_TRACE("[%s] No clcb! conn id 0x%x",__FUNCTION__, conn_id);
        return;
    }

    g_lepl_gatt_cb.cap_profile_data.is_bonded = 1;
    if (p_clcb->peer_profiles.p_csis &&
        !lepl_if_sirk_zero(p_clcb->csis_data.sirk_data.sirk))
    {
        uint8_t num_device = 0;
        for (int i = 0; i < MAX_CONNECTION_INSTANCE; i++)
        {
            if (lepl_csis_device_belongs_to_coordinated_set(g_lepl_gatt_cb.unicast_clcb[i].conn_id,
                                                            p_clcb->csis_data.sirk_data.sirk))
            {
                g_lepl_gatt_cb.cap_profile_data.device_info_list[num_device].conn_id =
                    g_lepl_gatt_cb.unicast_clcb[i].conn_id;
                num_device++;
            }
        }
        g_lepl_gatt_cb.cap_profile_data.num_devices = num_device;
    }
    else
    {
        g_lepl_gatt_cb.cap_profile_data.num_devices = 1;
        g_lepl_gatt_cb.cap_profile_data.device_info_list[0].conn_id = conn_id;

    }
}

#ifdef HS_SPK_ENABLED
void lepl_cap_update_cig_sync(uint16_t delay_limit)
{
    uint8_t param_buf[3] ={0x09,0,0};
    param_buf[1] = (delay_limit)&0xFF;
    param_buf[2] = (delay_limit>>8)&0xFF;
    // Send VSC to update CIG_Delay_Sync
    wiced_bt_dev_vendor_specific_command(0xFDDF,sizeof(param_buf),param_buf,NULL);
}
#endif

wiced_result_t lepl_cap_start_media_streaming(lepl_stream_config_t *p_stream_config)
{
    WICED_BT_TRACE("[%s]", __FUNCTION__);
    lepl_get_config(p_stream_config, &unicast_param);
    g_lepl_gatt_cb.cap_profile_data.is_bonded = 1;
    if (p_stream_config->num_devices == 1)
    {
        int ase_count = 0;

        lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(p_stream_config->config_list[0].conn_id);
        if (p_clcb == NULL)
        {
            WICED_BT_TRACE_CRIT("[%s] conn id %x NO p_clcb!", __FUNCTION__, p_stream_config->config_list->conn_id);
            return WICED_ERROR;
        }

        g_lepl_gatt_cb.cap_profile_data.device_info_list->conn_id = p_clcb->conn_id;
        if (le_audio_cap_mutiplex_audio_supported(unicast_param.p_codec_configuration, p_clcb->p_cap->p_pacs_data))
        {
            lepl_ase_data_t *p_ase = lepl_get_remote_ase(p_clcb, ASCS_SINK_ASE_CHARACTERISTIC, 0);
            if (!p_ase)
            {
                WICED_BT_TRACE_CRIT("[%s] conn id %x p_ase is NULL",
                                    __FUNCTION__, p_clcb->conn_id);
                return WICED_ERROR;
            }
            g_lepl_gatt_cb.cap_profile_data.device_info_list->ascs_data[ase_count++] = &p_ase->data;
            p_ase->data.qos_configured.cis_id = 1;
            p_ase->data.codec_configured.csc.audio_channel_allocation =
            p_clcb->p_cap->p_pacs_data->sink_audio_location &
                (BAP_AUDIO_LOCATION_FRONT_LEFT | BAP_AUDIO_LOCATION_FRONT_RIGHT);
        }
        else
        {
            uint8_t index = 0;
            cap_qos_config.max_sdu = p_stream_config->stream_config->octets_per_codec_frame;
            unicast_param.p_qos_configuration->max_sdu = p_stream_config->stream_config->octets_per_codec_frame;

            while (ase_count < 2)
            {
                lepl_ase_data_t *p_ase = lepl_get_remote_ase(p_clcb, ASCS_SINK_ASE_CHARACTERISTIC, &index);
                if (!p_ase)
                {
                    break;
                }
                p_ase->data.codec_configured.csc.audio_channel_allocation =
                    p_clcb->p_cap->p_pacs_data->sink_audio_location & (BAP_AUDIO_LOCATION_FRONT_LEFT << ase_count);
                g_lepl_gatt_cb.cap_profile_data.device_info_list->ascs_data[ase_count++] = &p_ase->data;
                p_ase->data.qos_configured.cis_id = ase_count;
            }
        }
        g_lepl_gatt_cb.cap_profile_data.device_info_list->num_ase = ase_count;
        unicast_param.num_of_cis = ase_count;
    }

    else
    {
        for (int i = 0; i < p_stream_config->num_devices; i++)
        {
            int ase_count = 0;
            lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(p_stream_config->config_list[i].conn_id);
            if (p_clcb == NULL)
            {
                WICED_BT_TRACE_CRIT("[%s] conn id %x NO p_clcb!", __FUNCTION__, p_stream_config->config_list->conn_id);
                return WICED_ERROR;
            }

            lepl_ase_data_t *p_ase = lepl_get_remote_ase(p_clcb, ASCS_SINK_ASE_CHARACTERISTIC, 0);
            if (!p_ase)
            {
                WICED_BT_TRACE_CRIT("[%s] conn id %x p_ase is NULL",
                                    __FUNCTION__,
                                    p_stream_config->config_list->conn_id);
                return WICED_ERROR;
            }
            le_audio_cap_device_data_t *p_dil = &g_lepl_gatt_cb.cap_profile_data.device_info_list[i];
            p_dil->conn_id =
                p_stream_config->config_list[i].conn_id;
            p_dil->ascs_data[ase_count++] = &p_ase->data;
            p_ase->data.qos_configured.cis_id = i + 1;
            p_ase->data.codec_configured.csc.audio_channel_allocation =
                (p_clcb->p_cap->p_pacs_data->sink_audio_location & BAP_AUDIO_LOCATION_FRONT_LEFT)
                    ? BAP_AUDIO_LOCATION_FRONT_LEFT
                    : BAP_AUDIO_LOCATION_FRONT_RIGHT;
            p_dil->num_ase = ase_count;
        }
        unicast_param.num_of_cis = p_stream_config->num_devices;
    }
    g_lepl_gatt_cb.cap_profile_data.num_devices = p_stream_config->num_devices;
#ifdef HS_SPK_ENABLED
    lepl_cap_update_cig_sync(LE_CIG_SYNC_DELAY);
#endif
    return le_audio_cap_start_unicast_streaming(&g_lepl_gatt_cb.cap_profile_data, &unicast_param);
}

//Application can decode only one incoming stream
wiced_result_t lepl_cap_start_conv_streaming(lepl_stream_config_t *p_stream_config)
{
    WICED_BT_TRACE("[%s]", __FUNCTION__);
    lepl_get_config(p_stream_config, &unicast_param);
    g_lepl_gatt_cb.cap_profile_data.is_bonded = 1;
    unicast_param.dir = WICED_BT_CAP_DIRECTION_SINK | WICED_BT_CAP_DIRECTION_SOURCE;
    for (int i = 0; i < p_stream_config->num_devices; i++)
    {
        int ase_count = 0;
        le_audio_cap_device_data_t *p_dil = &g_lepl_gatt_cb.cap_profile_data.device_info_list[i];
        lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(p_stream_config->config_list[i].conn_id);
        if (p_clcb == NULL)
        {
            WICED_BT_TRACE_CRIT("[%s] conn id %x No p_clcb", __FUNCTION__, p_stream_config->config_list[i].conn_id);
            return WICED_ERROR;
        }
        lepl_ase_data_t *p_ase = lepl_get_remote_ase(p_clcb, ASCS_SOURCE_ASE_CHARACTERISTIC, 0);
        if (p_ase && (p_clcb->p_cap->p_pacs_data->sink_audio_location & BAP_AUDIO_LOCATION_FRONT_LEFT))
        {
            p_dil->conn_id = p_clcb->conn_id;
            p_dil->ascs_data[ase_count++] = &p_ase->data;
            p_ase->data.qos_configured.cis_id = i + 1;
            p_ase->data.codec_configured.csc.audio_channel_allocation = BAP_AUDIO_LOCATION_FRONT_LEFT;
        }
        p_ase = lepl_get_remote_ase(p_clcb, ASCS_SINK_ASE_CHARACTERISTIC, 0);
        if (!p_ase)
        {
            WICED_BT_TRACE_CRIT("[%s] conn id %x p_ase is NULL", __FUNCTION__, p_clcb->conn_id);
            return WICED_ERROR;
        }
        p_dil->conn_id = p_stream_config->config_list[i].conn_id;
        p_dil->ascs_data[ase_count++] = &p_ase->data;
        p_ase->data.qos_configured.cis_id = i + 1;
        p_ase->data.codec_configured.csc.audio_channel_allocation =
        p_clcb->p_cap->p_pacs_data->sink_audio_location &
                (BAP_AUDIO_LOCATION_FRONT_LEFT | BAP_AUDIO_LOCATION_FRONT_RIGHT);
        p_dil->num_ase = ase_count;
    }
    unicast_param.num_of_cis = p_stream_config->num_devices;
    g_lepl_gatt_cb.cap_profile_data.num_devices = p_stream_config->num_devices;
    return le_audio_cap_start_unicast_streaming(&g_lepl_gatt_cb.cap_profile_data, &unicast_param);
}

wiced_result_t lepl_cap_start_mic_streaming(lepl_stream_config_t *p_stream_config)
{
    WICED_BT_TRACE("[%s]", __FUNCTION__);
    lepl_get_config(p_stream_config, &unicast_param);
    g_lepl_gatt_cb.cap_profile_data.is_bonded = 1;
    unicast_param.dir = WICED_BT_CAP_DIRECTION_SINK;
    for (int i = 0; i < p_stream_config->num_devices; i++)
    {
        le_audio_cap_device_data_t *p_dil = &g_lepl_gatt_cb.cap_profile_data.device_info_list[i];
        lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(p_stream_config->config_list[i].conn_id);
        if (p_clcb == NULL)
        {
            WICED_BT_TRACE_CRIT("[%s] conn id %x No p_clcb", __FUNCTION__, p_stream_config->config_list[i].conn_id);
            return WICED_ERROR;
        }

        p_dil->conn_id = p_clcb->conn_id;
        lepl_ase_data_t *p_ase = lepl_get_remote_ase(p_clcb, ASCS_SOURCE_ASE_CHARACTERISTIC, 0);
        if (p_ase)
        {
            p_dil->ascs_data[0] = &p_ase->data;
            p_ase->data.qos_configured.cis_id = i + 1;
            p_ase->data.codec_configured.csc.audio_channel_allocation =
                (p_clcb->p_cap->p_pacs_data->sink_audio_location & BAP_AUDIO_LOCATION_FRONT_LEFT)
                    ? BAP_AUDIO_LOCATION_FRONT_LEFT
                    : BAP_AUDIO_LOCATION_FRONT_RIGHT;
            p_dil->num_ase = 1;
        }
    }
    unicast_param.num_of_cis = p_stream_config->num_devices;
    g_lepl_gatt_cb.cap_profile_data.num_devices = p_stream_config->num_devices;
    return le_audio_cap_start_unicast_streaming(&g_lepl_gatt_cb.cap_profile_data, &unicast_param);
}

void lepl_cap_stop_media_streaming(uint16_t conn_id)
{
    lepl_cap_get_coordinated_set_members(conn_id);

    for (int i = 0; i < g_lepl_gatt_cb.cap_profile_data.num_devices; i++)
    {
        le_audio_cap_device_data_t *p_dil = &g_lepl_gatt_cb.cap_profile_data.device_info_list[i];
        lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(p_dil->conn_id);
        int ase_count = 0;

        uint8_t offset = 0;
        lepl_ase_data_t *p_ase;
        while (ase_count < MAX_NUM_ASE_ID)
        {
            p_ase = lepl_get_remote_ase(p_clcb, ASCS_SINK_ASE_CHARACTERISTIC, &offset);
            if (!p_ase)
            {
                break;
            }
            if (p_ase->data.ase_state != WICED_BT_GA_ASCS_STATE_IDLE)
            {
                p_dil->ascs_data[ase_count++] = &p_ase->data;
            }
        }

        p_dil->num_ase = ase_count;
    }

    le_audio_cap_release_stream(&g_lepl_gatt_cb.cap_profile_data);
}

void lepl_cap_stop_conv_streaming(uint16_t conn_id)
{
    lepl_cap_get_coordinated_set_members(conn_id);

    for (int i = 0; i < g_lepl_gatt_cb.cap_profile_data.num_devices; i++)
    {
        le_audio_cap_device_data_t *p_dil = &g_lepl_gatt_cb.cap_profile_data.device_info_list[i];
        lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(p_dil->conn_id);
        int ase_count = 0;

        uint8_t offset = 0;
        lepl_ase_data_t *p_ase;
        while (ase_count < MAX_NUM_ASE_ID)
        {
            p_ase = lepl_get_remote_ase(p_clcb, ASCS_SINK_ASE_CHARACTERISTIC, &offset);
            if (!p_ase)
            {
                break;
            }
            if (p_ase->data.ase_state != WICED_BT_GA_ASCS_STATE_IDLE)
            {
                p_dil->ascs_data[ase_count++] = &p_ase->data;
            }
        }
        p_ase = lepl_get_remote_ase(p_clcb, ASCS_SOURCE_ASE_CHARACTERISTIC, 0);
        if (p_ase->data.ase_state > WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED)
            p_dil->ascs_data[ase_count++] = &p_ase->data;

        p_dil->num_ase = ase_count;
    }
    le_audio_cap_release_stream(&g_lepl_gatt_cb.cap_profile_data);
}

void lepl_cap_stop_mic_streaming(uint16_t conn_id)
{
    lepl_cap_get_coordinated_set_members(conn_id);

    for (int i = 0; i < g_lepl_gatt_cb.cap_profile_data.num_devices; i++)
    {
        le_audio_cap_device_data_t *p_dil = &g_lepl_gatt_cb.cap_profile_data.device_info_list[i];
        lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(p_dil->conn_id);
        int ase_count = 0;

        lepl_ase_data_t *p_ase = lepl_get_remote_ase(p_clcb, ASCS_SOURCE_ASE_CHARACTERISTIC, 0);
        p_dil->ascs_data[ase_count++] = &p_ase->data;

        p_dil->num_ase = ase_count;
    }

    le_audio_cap_release_stream(&g_lepl_gatt_cb.cap_profile_data);
    lepl_rpc_send_mic_state(0);
}

static void lepl_cap_select_qos_config_param(void)
{
    le_audio_cap_app_data_t *p_cap_app_data = &g_lepl_gatt_cb.cap_profile_data;
    wiced_bt_ga_ascs_config_qos_args_t *p_qos_param = unicast_param.p_qos_configuration;

    for (int i = 0; i < p_cap_app_data->num_devices; i++)
    {
        le_audio_cap_device_data_t *p_device_info = &p_cap_app_data->device_info_list[i];
        for (int index = 0; index < p_device_info->num_ase; index++)
        {
            wiced_bt_ga_ascs_ase_t *p_ase = p_device_info->ascs_data[index];
            if ((p_ase->p_ase_info->ascs_data.preferred_presentation_delay_in_us_max != 0) &&
                (p_qos_param->presentation_delay > p_ase->p_ase_info->ascs_data.presentation_delay_in_us_max))
            {
                p_qos_param->presentation_delay = p_ase->p_ase_info->ascs_data.presentation_delay_in_us_max;
            }
            if ((p_ase->p_ase_info->ascs_data.preferred_presentation_delay_in_us_min != 0) &&
                (p_qos_param->presentation_delay < p_ase->p_ase_info->ascs_data.presentation_delay_in_us_min))
            {
                p_qos_param->presentation_delay = p_ase->p_ase_info->ascs_data.presentation_delay_in_us_min;
            }
            if (p_ase->p_ase_info->ascs_data.max_transport_latency >= 5 && (p_ase->p_ase_info->ascs_data.max_transport_latency <
                p_qos_param->max_transport_latency))
                p_qos_param->max_transport_latency = p_ase->p_ase_info->ascs_data.max_transport_latency;
            p_qos_param->retransmission_number = p_ase->p_ase_info->ascs_data.preferred_retransmission_number;
            p_qos_param->phy = p_ase->p_ase_info->ascs_data.preferred_phy;
        }
    }
}

#ifdef HS_SPK_ENABLED
wiced_result_t lepl_cap_utils_create_cig(wiced_bt_ga_ascs_config_qos_args_t *p_qos_params, uint8_t cig_id, uint8_t cis_count)
{
    int index;
    uint8_t cis_id;
    wiced_ble_isoc_cis_config_t cis_config_list[2];
    wiced_ble_isoc_cig_param_t cig_param = {0};

    cig_param.cig_id = cig_id; // Assign CIG ID
    cig_param.cis_count = cis_count;
    cig_param.sdu_interval_c_to_p = p_qos_params->sdu_interval;
    cig_param.sdu_interval_p_to_c = p_qos_params->sdu_interval;

    cig_param.max_trans_latency_c_to_p = p_qos_params->max_transport_latency;
    cig_param.max_trans_latency_p_to_c = p_qos_params->max_transport_latency;
    //cig_param.max_trans_latency_c_to_p = 95;
    //cig_param.max_trans_latency_p_to_c = 95;
    cig_param.packing = WICED_BLE_ISOC_SEQUENTIAL_PACKING;
    cig_param.framing = p_qos_params->framing;
    cig_param.p_cis_config_list = cis_config_list;
    for (index = 0; index < cig_param.cis_count; index++)
    {
        cis_id = (p_qos_params->cis_id) + index;
        WICED_BT_TRACE("[%s] cis_id %d", __FUNCTION__, cis_id);
        le_audio_cap_utils_fill_cis_config_data(&cig_param.p_cis_config_list[index],
                                                cis_id,
                                                p_qos_params->max_sdu,
                                                (g_lepl_gatt_cb.app_state.current_state == LEPL_APP_STATE_MEDIA) ? 0 : p_qos_params->max_sdu,
                                                p_qos_params->phy,
                                                p_qos_params->phy,
                                                p_qos_params->retransmission_number,
                                                p_qos_params->retransmission_number);
    }

    p_qos_params->cig_id = cig_param.cig_id;

    WICED_BT_TRACE("CIG ID %d SDU Interval (M->S/S->M) (0x%x/0x%x) SCA %d Packing %d Framing %d ",
                   cig_param.cig_id,
                   cig_param.sdu_interval_c_to_p,
                   cig_param.sdu_interval_p_to_c,
                   cig_param.worst_case_sca,
                   cig_param.packing,
                   cig_param.framing);

    WICED_BT_TRACE("Trans Latency (M->S/S->M) (%d/%d) CIS count %d",
                   cig_param.max_trans_latency_c_to_p,
                   cig_param.max_trans_latency_p_to_c,
                   cig_param.cis_count);

    WICED_BT_TRACE("CIS ID %d SDU (M->S/S->M) (%d/%d) PHY (M->S/S->M) (%d/%d) RTN (M->S/S->M) (%d/%d)",
                   cig_param.p_cis_config_list[0].cis_id,
                   cig_param.p_cis_config_list[0].max_sdu_c_to_p,
                   cig_param.p_cis_config_list[0].max_sdu_p_to_c,
                   cig_param.p_cis_config_list[0].phy_c_to_p,
                   cig_param.p_cis_config_list[0].phy_p_to_c,
                   cig_param.p_cis_config_list[0].rtn_c_to_p,
                   cig_param.p_cis_config_list[0].rtn_p_to_c);

    return wiced_ble_isoc_central_set_cig_param(&cig_param);
}
#endif // HS_SPK_ENABLED

void lepl_cap_event_cb(uint16_t conn_id, le_audio_cap_event_t event, le_audio_cap_event_data_t *p_event_data)
{
    switch (event)
    {
    case WICED_BT_GA_CAP_STATE_CHANGED_EVENT:
    {
        WICED_BT_TRACE("[%s] state update %d", __FUNCTION__, p_event_data->group_state);
        switch (p_event_data->group_state)
        {
        case WICED_BT_GA_ASCS_STATE_CODEC_CONFIGURED:
            if (!g_lepl_gatt_cb.cap_profile_data.is_disconnecting)
            {
                lepl_cap_select_qos_config_param();
#ifdef HS_SPK_ENABLED
                lepl_cap_update_cig_sync(LE_CIG_SYNC_DELAY);
                lepl_cap_utils_create_cig(unicast_param.p_qos_configuration,
                                              1,
                                              unicast_param.num_of_cis);
#else
                le_audio_cap_utils_create_cig(unicast_param.p_qos_configuration,
                                            1, unicast_param.num_of_cis);
#endif
            }
            break;
        case WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED:
            if (p_event_data->p_app_data->is_disconnecting || p_event_data->p_app_data->is_disabling)
                break;

            WICED_BT_TRACE("[%s] num of device %d", __FUNCTION__, p_event_data->p_app_data->num_devices);

            for (int i = 0; (i < p_event_data->p_app_data->num_devices) && (i < MAX_CONNECTION_INSTANCE); i++)
            {
                uint8_t num_ase = p_event_data->p_app_data->device_info_list[i].num_ase;
                wiced_ble_isoc_cis_acl_t conn_hdl[MAX_CIS_CONN] = {0};
                uint8_t num_of_cis = 0;
                for(int index = 0; index < num_ase; index++)
                {
                    wiced_bt_ga_ascs_config_qos_args_t *p_qos =
                        &p_event_data->p_app_data->device_info_list[i].ascs_data[index]->qos_configured;
                    uint16_t cis_conn_hdl = wiced_ble_isoc_central_get_cis_conn_handle(p_qos->cig_id, p_qos->cis_id);

                    //Assign cis handle to p_ase if it's already connected
                    if (wiced_ble_isoc_is_cis_connected_with_conn_hdl(cis_conn_hdl))
                    {
                        lepl_ase_data_t *p_ase = lepl_get_remote_ase_data_by_ase_id(
                            lepl_gatt_get_clcb_by_conn_id(p_event_data->p_app_data->device_info_list[i].conn_id),
                            p_event_data->p_app_data->device_info_list[i].ascs_data[index]->p_ase_info->ase_id);
                        p_ase->cis_conn_handle = cis_conn_hdl;
                        continue;
                    }

                    //Check if it's already stored
                    for (int i = 0; i < MAX_CIS_CONN; i++)
                    {
                        if (conn_hdl[i].cis_conn_handle == cis_conn_hdl)
                        {
                            cis_conn_hdl = 0;
                            break;
                        }
                    }

                    //Store conn handle to create cis
                    if (cis_conn_hdl && (num_of_cis < MAX_CIS_CONN))
                    {
                        conn_hdl[num_of_cis].cis_conn_handle = cis_conn_hdl;
                        conn_hdl[num_of_cis].acl_conn_handle =
                            wiced_bt_gatt_get_acl_conn_handle(p_event_data->p_app_data->device_info_list[i].conn_id);
                        num_of_cis++;
                    }
                }
                wiced_result_t res = wiced_ble_isoc_central_create_cis(num_of_cis, conn_hdl);
                WICED_BT_TRACE("[%s] create cis res %d",__FUNCTION__, res);
            }
            break;
        }
    }
        lepl_cap_update_application_state(conn_id, p_event_data);
    break;

    default:
        break;
    }
}

void lepl_cap_update_context_type(le_audio_cap_start_unicast_param_t *param, wiced_bt_ga_bap_context_type_t context_type)
{
    uint8_t temp_buffer[10] = {0};

    // point to global buffer or allocate number of ccid+2 bytes
    param->metadata.p_upper_layer_data = temp_buffer;
    param->metadata.upper_layer_data_length = sizeof(temp_buffer);
    le_audio_cap_fill_metadata(0, NULL, context_type, &param->metadata);
}
