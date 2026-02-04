/*
 * $ Copyright Cypress Semiconductor $
 */
#include "lepl.h"

extern lepl_gatt_cb_t g_lepl_gatt_cb;
lepl_mcs_data_t *media_app_data;
static wiced_bt_ga_mcp_media_control_operation_t remaining_op_handle;
const gatt_intf_service_object_t *remaining_op_profile_ptr = NULL;
int current_track_number = 0;
int current_codec_config = BAP_CODEC_CONFIG_16_2_2;

#define MEDIA_PLAYER_NAME "CYPRESS_PLAYER"
#define MEDIA_TRACK_TITLE "DEFAULT_TRACK"

static wiced_bool_t notify_mcs_data(uint16_t conn_id,
                             const gatt_intf_service_object_t *p_service,
                             mcs_characteristics_t type,
                             void *p_data)
{
    wiced_bt_gatt_status_t status;
    gatt_intf_attribute_t characteristic = {.characteristic_type = type};

    status =
        gatt_interface_notify_characteristic(conn_id, (gatt_intf_service_object_t *)p_service, &characteristic, p_data);
    return (status == WICED_BT_GATT_SUCCESS) ? WICED_TRUE : WICED_FALSE;
}

static wiced_bool_t notify_media_state(const gatt_intf_service_object_t *p_profile)
{
    WICED_BT_TRACE("[%s] state %d ", __FUNCTION__, media_app_data->media_state);
    wiced_bool_t res = WICED_FALSE;

    for (int i = 0; i < MAX_CONNECTION_INSTANCE; i++)
    {
        if (g_lepl_gatt_cb.unicast_clcb[i].in_use)
        {
            le_audio_rpc_send_mcs_state_update(g_lepl_gatt_cb.unicast_clcb[i].conn_id,
                                               media_app_data->media_state);

            notify_mcs_data(g_lepl_gatt_cb.unicast_clcb[i].conn_id,
                            p_profile,
                            MCS_MEDIA_STATE_CHARACTERISTIC,
                            &media_app_data->media_state);
        }
    }
    return res;
}

static wiced_bool_t media_control_service_update_state(const gatt_intf_service_object_t *p_profile,
                                                wiced_bt_ga_media_control_state_t state)
{
    if (media_app_data->media_state != state)
    {
        media_app_data->media_state = state;
        return notify_media_state(p_profile);
    }
    return WICED_TRUE;
}
static wiced_bt_ga_mcp_result_t media_control_service_handle_play(const gatt_intf_service_object_t *p_profile)
{
    media_control_service_update_state(p_profile, WICED_BT_GA_MCS_MEDIA_PLAYING);
    WICED_BT_TRACE("Media State WICED_BT_GA_MCS_MEDIA_PLAYING \n");
    return WICED_BT_GA_MCS_SUCCESS;
}

static wiced_bt_ga_mcp_result_t media_control_service_handle_pause(const gatt_intf_service_object_t *p_profile)
{
    media_control_service_update_state(p_profile, WICED_BT_GA_MCS_MEDIA_PAUSED);
    WICED_BT_TRACE("Media State WICED_BT_MCS_MEDIA_PAUSED \n");
    return WICED_BT_GA_MCS_SUCCESS;
}

static wiced_bt_ga_mcp_result_t media_control_service_handle_stop(const gatt_intf_service_object_t *p_profile)
{
    if (p_profile)
        media_control_service_update_state(p_profile, WICED_BT_GA_MCS_MEDIA_PAUSED);
    else
        media_app_data->media_state = WICED_BT_GA_MCS_MEDIA_PAUSED;

    WICED_BT_TRACE("Media State WICED_BT_GA_MCS_MEDIA_STOPPED \n");
    return WICED_BT_GA_MCS_SUCCESS;
}

wiced_bool_t lepl_mcs_is_streaming()
{
    return (media_app_data->media_state == WICED_BT_GA_MCS_MEDIA_PLAYING) ? WICED_TRUE : WICED_FALSE;
}

static wiced_result_t lepl_start_streaming(uint16_t conn_id, uint32_t codec_config)
{
    lepl_stream_config_t unicast_stream_config;
    wiced_bt_ga_bap_stream_config_t stream_config1;
    lepl_device_config_t config1[2];
    wiced_result_t res;
    unicast_stream_config.num_devices = 0;

    res = wiced_bt_ga_bap_get_unicast_stream_config(codec_config,
                                      &stream_config1);
    if (res)
    {
        WICED_BT_TRACE_CRIT("[%s] res %d", __FUNCTION__, res);
        return res;
    }

    WICED_BT_TRACE("lepl_start_streaming %x\n", conn_id);

    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);

    if (p_clcb->peer_profiles.p_csis &&
        !lepl_if_sirk_zero(p_clcb->csis_data.sirk_data.sirk))
    {
        for (int i = 0; i < MAX_CONNECTION_INSTANCE; i++)
        {
            if (lepl_csis_device_belongs_to_coordinated_set(g_lepl_gatt_cb.unicast_clcb[i].conn_id,
                                                            p_clcb->csis_data.sirk_data.sirk))
            {
                config1[unicast_stream_config.num_devices].audio_location =
                    g_lepl_gatt_cb.unicast_clcb[i].p_cap->p_pacs_data->sink_audio_location &
                    (BAP_AUDIO_LOCATION_FRONT_LEFT | BAP_AUDIO_LOCATION_FRONT_RIGHT);
                config1[unicast_stream_config.num_devices].conn_id = g_lepl_gatt_cb.unicast_clcb[i].conn_id;
                unicast_stream_config.num_devices++;
            }
        }
    }
    else
    {
        config1[unicast_stream_config.num_devices].audio_location =
            g_lepl_gatt_cb.unicast_clcb[0].p_cap->p_pacs_data->sink_audio_location &
            (BAP_AUDIO_LOCATION_FRONT_LEFT | BAP_AUDIO_LOCATION_FRONT_RIGHT);
        config1[unicast_stream_config.num_devices].conn_id = conn_id;
        unicast_stream_config.num_devices = 1;
    }

    unicast_stream_config.ctx_type = BAP_CONTEXT_TYPE_MEDIA;
    unicast_stream_config.stream_config = &stream_config1;
    unicast_stream_config.config_list = config1;

    res = lepl_cap_start_media_streaming(&unicast_stream_config);
    if (res == WICED_SUCCESS)
    {
        current_codec_config = codec_config;
    }
    return res;
}


static wiced_bt_ga_mcp_result_t lepl_handle_streaming_operation(const gatt_intf_service_object_t *p_profile,
                                                                   wiced_bt_ga_mcp_media_control_operation_t opcode,
                                                                   uint16_t conn_id,
                                                                   uint32_t codec_config)
{
    // Start/Stop actual streaming
    if (opcode == WICED_BT_GA_MCS_PLAY)
    {
        WICED_BT_TRACE("[%s] media_app_data->media_state %d \n", __FUNCTION__, media_app_data->media_state);

        // start streaming
        if (media_app_data->media_state != WICED_BT_GA_MCS_MEDIA_PLAYING)
        {
            remaining_op_handle = opcode;
            remaining_op_profile_ptr = p_profile;

            if (lepl_start_streaming(conn_id, codec_config))
                return WICED_BT_GA_MCS_COMMAND_CANNOT_BE_COMPLETED;
        }
        else
        {
            return media_control_service_handle_play(p_profile);
        }
    }
    else if ((opcode == WICED_BT_GA_MCS_PAUSE) || (opcode == WICED_BT_GA_MCS_STOP))
    {
        // stop streaming
        if (media_app_data->media_state != WICED_BT_GA_MCS_MEDIA_PAUSED)
        {
            remaining_op_handle = opcode;
            remaining_op_profile_ptr = p_profile;
            lepl_cap_stop_media_streaming(conn_id);
        }
        else
        {
            if (opcode == WICED_BT_GA_MCS_PAUSE)
                return media_control_service_handle_pause(p_profile);
            else
                return media_control_service_handle_stop(p_profile);
        }
    }
    return WICED_BT_GA_MCS_SUCCESS;
}

static void media_control_service_clear_operation(void)
{
    remaining_op_handle = WICED_BT_GA_MCS_INVALID;
    remaining_op_profile_ptr = NULL;
}

// Once actual media start/stops call this API to send notification
void lepl_mcs_handle_post_operation(void)
{
    if (remaining_op_profile_ptr == NULL) return;

    switch (remaining_op_handle)
    {
        case WICED_BT_GA_MCS_PLAY:
            media_control_service_handle_play(remaining_op_profile_ptr);
            break;
        case WICED_BT_GA_MCS_PAUSE:
            media_control_service_handle_pause(remaining_op_profile_ptr);
            lepl_gatt_handle_disconnecting_state();
            break;
        case WICED_BT_GA_MCS_STOP:
            media_control_service_handle_stop(remaining_op_profile_ptr);
            break;
        default:
            break;
    }
    media_control_service_clear_operation();
}

static wiced_bt_ga_mcp_result_t media_control_service_handle_event(const gatt_intf_service_object_t *p_profile,
                                                            wiced_bt_ga_mcs_data_t *p_data)
{
    wiced_bt_ga_mcp_result_t result = WICED_BT_GA_MCS_OPCODE_NOT_SUPPORTED;
    wiced_bt_ga_mcp_media_control_operation_t opcode = p_data->control_point_operation.opcode;

    WICED_BT_TRACE("[%s] opcode %d state %d", __FUNCTION__, opcode, media_app_data->media_state);

    switch (opcode)
    {
        case WICED_BT_GA_MCS_PLAY:
            result = WICED_BT_GA_MCS_SUCCESS;
            break;
        case WICED_BT_GA_MCS_PAUSE:
            result = WICED_BT_GA_MCS_SUCCESS;
            break;
        case WICED_BT_GA_MCS_STOP:
            result = WICED_BT_GA_MCS_SUCCESS;
            break;
    }

    WICED_BT_TRACE("end -- [%s] opcode %d state %d result %d",
                   __FUNCTION__,
                   opcode,
                   media_app_data->media_state,
                   result);

    return result;
}

void lepl_mcs_initialize_data(void)
{
    gatt_intf_service_object_t *g_profile = g_lepl_gatt_cb.local_profiles.p_gmcs;
    gatt_intf_service_object_t *m_profile = g_lepl_gatt_cb.local_profiles.p_mcs;

    WICED_BT_TRACE("[%s] ", __FUNCTION__);

    media_app_data = &g_lepl_gatt_cb.mcs_data;

    // Set default playing order single repeat
    media_app_data->playing_order = WICED_BT_GA_MCS_SINGLE_REPEAT;
    // Playing order supported
    media_app_data->playing_order_supported =
        (MCS_SINGLE_ONCE_PLAYING_ORDER_MASK | MCS_SINGLE_REPEAT_PLAYING_ORDER_MASK |
         MCS_IN_ORDER_ONCE_PLAYING_ORDER_MASK | MCS_IN_ORDER_REPEAT_PLAYING_ORDER_MASK);

    media_app_data->media_state = WICED_BT_GA_MCS_MEDIA_INACTIVE;

    memcpy(media_app_data->media_player_name, MEDIA_PLAYER_NAME, strlen(MEDIA_PLAYER_NAME));
    media_app_data->media_player_name[strlen(MEDIA_PLAYER_NAME)] = '\0';

    memcpy(media_app_data->track_title, MEDIA_TRACK_TITLE, strlen(MEDIA_TRACK_TITLE));
    media_app_data->track_title[strlen(MEDIA_TRACK_TITLE)] = '\0';

    media_app_data->track_position = 0;
    current_track_number = 0;

    media_app_data->media_control_supported_opcodes = (MCS_BASIC_OPCODES_SUPPORTED | MCS_TRACK_OPCODES_SUPPORTED);

    // select the track and update media state
    media_app_data->media_state = WICED_BT_GA_MCS_MEDIA_PAUSED;

    if (g_profile)
    {
        WICED_BT_TRACE("[%s] generic media context %x \n", __FUNCTION__, g_profile);
        wiced_bt_ga_mcs_media_track_selected(g_profile, WICED_TRUE);
    }
    if (m_profile)
    {
        WICED_BT_TRACE("[%s] media context %x \n", __FUNCTION__, m_profile);
        wiced_bt_ga_mcs_media_track_selected(m_profile, WICED_TRUE);
    }
}

static wiced_bt_ga_mcp_result_t lepl_handle_mcs_control_point_operation(uint16_t conn_id,
    const gatt_intf_service_object_t *p_profile,
    wiced_bt_ga_mcp_media_control_operation_t opcode)
{
    lepl_app_state_t state = lepl_cap_get_application_state();

    wiced_bt_ga_mcp_result_t mcp_result =
        lepl_handle_streaming_operation(p_profile, opcode, conn_id, current_codec_config);

    if (opcode == WICED_BT_GA_MCS_PLAY)
    {
        if (state != LEPL_APP_STATE_IDLE)
        {
            WICED_BT_TRACE_CRIT("[%s] Not ready to play! state: %d", __FUNCTION__, state);
            return WICED_BT_GA_MCS_COMMAND_CANNOT_BE_COMPLETED;
        }
        else
        {
            lepl_cap_set_next_application_state(LEPL_APP_STATE_MEDIA, current_codec_config);
        }
    }
    else if ((opcode == WICED_BT_GA_MCS_PAUSE) || (opcode == WICED_BT_GA_MCS_STOP))
    {
        if (state == LEPL_APP_STATE_IN_TRANSIT)
            state = lepl_cap_get_application_final_state();

        if (state != LEPL_APP_STATE_MEDIA)
        {
            WICED_BT_TRACE_CRIT("[%s] state: %d", __FUNCTION__, state);
            return WICED_BT_GA_MCS_COMMAND_CANNOT_BE_COMPLETED;
        }

        lepl_cap_set_next_application_state(LEPL_APP_STATE_IDLE, 0XFF);
    }
    return mcp_result;
}

static wiced_result_t lepl_handle_mcs_data(uint16_t conn_id,
                                              void *p_app_ctx,
                                              const gatt_intf_service_object_t *p_profile,
                                              wiced_bt_gatt_status_t status,
                                              uint32_t evt_type,
                                              gatt_intf_attribute_t *p_char,
                                              wiced_bt_ga_mcs_data_t *p_evt_data,
                                              int len)
{
    wiced_result_t result = WICED_SUCCESS;

    WICED_BT_TRACE("[%s] event_id 0x%x char %d \n", __FUNCTION__, evt_type, p_char->characteristic_type);

    if (evt_type == READ_REQ_EVT)
    {
        switch (p_char->characteristic_type)
        {
                //read requests
            case MCS_MEDIA_PLAYER_NAME_CHARACTERISTIC:
                p_evt_data->media_player_name.len = strlen(media_app_data->media_player_name);
                p_evt_data->media_player_name.str = media_app_data->media_player_name;
                break;
            case MCS_MEDIA_TRACK_TITLE_CHARACTERISTIC:
                p_evt_data->media_player_name.len = strlen(media_app_data->track_title);
                p_evt_data->media_player_name.str = media_app_data->track_title;
                break;
            case MCS_MEDIA_TRACK_DURATION_CHARACTERISTIC:
                p_evt_data->track_duration = media_app_data->track_duration;
                break;
            case MCS_MEDIA_TRACK_POSITION_CHARACTERISTIC:
                p_evt_data->track_position = media_app_data->track_position;
                break;
            case MCS_MEDIA_PLAYBACK_SPEED_CHARACTERISTIC:
                p_evt_data->playback_speed = media_app_data->playback_speed;
                break;
            case MCS_MEDIA_SEEKING_SPEED_CHARACTERISTIC:
                p_evt_data->seeking_speed = media_app_data->seeking_speed;
                break;
            case MCS_MEDIA_PLAYING_ORDER_CHARACTERISTIC:
                p_evt_data->playing_order = media_app_data->playing_order;
                break;
            case MCS_MEDIA_PLAYING_ORDER_SUPPORTED_CHARACTERISTIC:
                p_evt_data->playing_order_supported = media_app_data->playing_order_supported;
                break;
            case MCS_MEDIA_STATE_CHARACTERISTIC:
                p_evt_data->media_state = media_app_data->media_state;
                break;
            case MCS_MEDIA_OPCODE_SUPPORTED_CHARACTERISTIC:
                p_evt_data->media_control_supported_opcodes = media_app_data->media_control_supported_opcodes;
                break;
            case MCS_CONTENT_CONTROL_ID_CHARACTERISTIC:
                p_evt_data->content_control_id = media_app_data->content_control_id;
                break;
            default:
                return WICED_ERROR;
                break;
        }
    }
    else if (evt_type == WRITE_REQ_EVT)
    {
        switch (p_char->characteristic_type)
        {
                //write requests

            case MCS_MEDIA_CONTROL_POINT_CHARACTERISTIC:
            {
                wiced_bt_ga_mcp_result_t mcp_result = media_control_service_handle_event(p_profile, p_evt_data);

                if (mcp_result == WICED_BT_GA_MCS_SUCCESS)
                    mcp_result = lepl_handle_mcs_control_point_operation(conn_id, p_profile, p_evt_data->control_point_operation.opcode);

                p_evt_data->control_point_operation.result = mcp_result;

                if (mcp_result == WICED_BT_GA_MCS_SUCCESS)
                {
                    lepl_mcs_handle_post_operation();
                }
           }break;
            case MCS_MEDIA_OPCODE_SUPPORTED_CHARACTERISTIC:
                p_evt_data->media_control_supported_opcodes = media_app_data->media_control_supported_opcodes;
                WICED_BT_TRACE("[%s]  0x%x \n", __FUNCTION__, p_evt_data->media_control_supported_opcodes);
                break;
            case MCS_MEDIA_STATE_CHARACTERISTIC:
                p_evt_data->media_state = media_app_data->media_state;
                break;
            default:
                return WICED_ERROR;
                break;
        }
    }
    return result;
}

wiced_result_t lepl_mcs_callback(uint16_t conn_id,
                                           void *p_app_ctx,
                                           gatt_intf_service_object_t *p_service,
                                           wiced_bt_gatt_status_t status,
                                           uint32_t evt_type,
                                           gatt_intf_attribute_t *p_char,
                                           void *p_data,
                                           int len)
{

    WICED_BT_TRACE("[%s] event 0x%x \n", __FUNCTION__, p_char);
    switch (evt_type)
    {
        case READ_REQ_EVT:
        case WRITE_REQ_EVT:
            return lepl_handle_mcs_data(conn_id,
                                                  p_app_ctx,
                                                  p_service,
                                                  status,
                                                  evt_type,
                                                  p_char,
                                                  (wiced_bt_ga_mcs_data_t *)p_data,
                                                  len);
            break;
    }
    return WICED_BT_SUCCESS;
}

wiced_result_t lepl_mcs_play(uint16_t conn_id,
                                       uint32_t codec_config)
{
    wiced_bt_ga_mcs_data_t data;
    data.control_point_operation.opcode = WICED_BT_GA_MCS_PLAY;
    WICED_BT_TRACE("[%s] conn_id 0x%x  \n", __FUNCTION__, conn_id);
    wiced_bt_ga_mcp_result_t status =
        media_control_service_handle_event(g_lepl_gatt_cb.local_profiles.p_gmcs, &data);


    if (status == WICED_BT_GA_MCS_SUCCESS)
    {
        lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
        if (p_clcb && p_clcb->in_use)
        {
		const wiced_bt_cfg_ble_scan_settings_t *p_scan_cfg = lepl_cfg_settings.p_ble_cfg->p_ble_scan_cfg;

		WICED_BT_TRACE("[%s]",__FUNCTION__, p_clcb->conn_interval, p_scan_cfg->conn_min_interval);

		if(p_clcb->conn_interval && p_clcb->conn_interval < p_scan_cfg->conn_min_interval){
			lepl_update_conn_param(p_clcb->bda);
		}
            status = lepl_handle_streaming_operation(g_lepl_gatt_cb.local_profiles.p_gmcs,
                                                     WICED_BT_GA_MCS_PLAY,
                                                     conn_id,
                                                     codec_config);
            if (status == WICED_BT_GA_MCS_SUCCESS)
            {
                lepl_mcs_handle_post_operation();
            }
            else
            {
                WICED_BT_TRACE_CRIT("[%s] MCS play failed %d",__FUNCTION__, status);
            }
        }
    }
    return (status == WICED_BT_GA_MCS_SUCCESS) ? WICED_SUCCESS : WICED_ERROR;
}

wiced_result_t lepl_mcs_pause(uint16_t conn_id)
{
    wiced_bt_ga_mcs_data_t data;
    data.control_point_operation.opcode = WICED_BT_GA_MCS_PAUSE;
    WICED_BT_TRACE("[%s] conn_id 0x%x \n", __FUNCTION__, conn_id);
    wiced_bt_ga_mcp_result_t status =
        media_control_service_handle_event(g_lepl_gatt_cb.local_profiles.p_gmcs, &data);

    if (status == WICED_BT_GA_MCS_SUCCESS)
    {
        lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
        if (p_clcb && p_clcb->in_use)
        {
            status = lepl_handle_streaming_operation(g_lepl_gatt_cb.local_profiles.p_gmcs,
                                                     WICED_BT_GA_MCS_PAUSE,
                                                     conn_id,
                                                     -1);
            if (status == WICED_BT_GA_MCS_SUCCESS)
            {
                lepl_mcs_handle_post_operation();
            }
            else
            {
                WICED_BT_TRACE_CRIT("[%s] MCS pause failed %d", __FUNCTION__, status);
            }
        }
    }
    return (status == WICED_BT_GA_MCS_SUCCESS) ? WICED_SUCCESS : WICED_ERROR;
}
