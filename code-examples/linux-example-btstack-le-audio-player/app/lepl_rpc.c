/*
 * $ Copyright Cypress Semiconductor $
 */

/* Application includes */
#include "lepl.h"
#include "lepl_bis.h"
#ifdef HS_SPK_ENABLED
#include "bt_hs_spk_control.h"
#endif
#include "log.h"

lepl_app_state_info_t *p_app_state_info = &g_lepl_gatt_cb.app_state;

static void lepl_rpc_handle_scan(uint8_t *p_data, uint32_t data_len)
{
    uint8_t scan;
    uint8_t enable_uuid_filter;

    STREAM_TO_UINT8(scan, p_data);
    STREAM_TO_UINT8(enable_uuid_filter, p_data);

    WICED_BT_TRACE("[%s] adv %d len %d\n", __FUNCTION__, scan, data_len);

    lepl_gatt_start_stop_scan(scan, enable_uuid_filter);
}

static void lepl_rpc_handle_adv(uint8_t *p_data, uint8_t payload_len)
{
    uint8_t adv;

    STREAM_TO_UINT8(adv, p_data);

    WICED_BT_TRACE("[%s] adv %d len %d\n", __FUNCTION__, adv, payload_len);

    lepl_gatt_start_stop_adv(adv);
}

static void lepl_rpc_handle_connect(uint8_t *p_data, uint32_t data_len, int cancel)
{
    wiced_bt_device_address_t bd_addr;
    uint8_t addr_type;
    wiced_result_t status;

    STREAM_TO_UINT8(addr_type, p_data);
    STREAM_TO_BDADDR(bd_addr, p_data);

    if (cancel == 0)
    {
        status = app_create_connection(addr_type, bd_addr);
    }
    else
    {
        status = wiced_bt_gatt_cancel_connect(bd_addr, 1);
    }

    WICED_BT_TRACE("[%s] %sconnect type %d address %B len %d status %d\n", __FUNCTION__,
                   cancel ? "cancel" : "", addr_type, bd_addr, data_len, status);
}

static void lepl_rpc_handle_disconnect(uint8_t *p_data, uint32_t data_len)
{
    uint16_t conn_handle;
    wiced_result_t status;
    STREAM_TO_UINT16(conn_handle, p_data);

    status = lepl_gatt_disconnect(conn_handle);
    WICED_BT_TRACE("[%s] conn %d status 0x%x\n", __FUNCTION__, conn_handle, status);
}

static void lepl_rpc_handle_play(uint8_t *p_data, uint32_t data_len)
{
    uint16_t conn_id;
    uint32_t codec_config;

    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT32(codec_config, p_data);

    WICED_BT_TRACE("[%s] conn_id %d codec_config %d len %d\n", __FUNCTION__, conn_id, codec_config, data_len);

    lepl_app_state_t state = lepl_cap_get_application_state();
    if (state != LEPL_APP_STATE_IDLE)
    {
        WICED_BT_TRACE_CRIT("[%s] Not ready to play! state: %d", __FUNCTION__, state);
        return;
    }
    if (lepl_mcs_play(conn_id, codec_config) == WICED_SUCCESS)
    {
        lepl_cap_set_next_application_state(LEPL_APP_STATE_MEDIA, codec_config);
    }
}

static void lepl_rpc_handle_pause(uint8_t *p_data, uint32_t data_len)
{
    uint16_t conn_id;

    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d len %d\n", __FUNCTION__, conn_id, data_len);

    lepl_app_state_t state = lepl_cap_get_application_state();
    if (state == LEPL_APP_STATE_IN_TRANSIT)
        state = lepl_cap_get_application_final_state();

    if (state != LEPL_APP_STATE_MEDIA)
    {
        WICED_BT_TRACE_CRIT("[%s] state: %d", __FUNCTION__, state);
        return;
    }

    if (lepl_mcs_pause(conn_id) == WICED_SUCCESS)
    {
        lepl_cap_set_next_application_state(LEPL_APP_STATE_IDLE, 0XFF);
    }
}

static void lepl_rpc_vcs_set_vol(uint8_t *p_data, uint32_t data_len, uint16_t opcode)
{
    uint16_t conn_id;
    uint8_t abs_vol = 0;
    volume_control_opcodes_t vcs_opcode;
    STREAM_TO_UINT16(conn_id, p_data);

    WICED_BT_TRACE("[%s] conn_id %d opcode %d len %d\n", __FUNCTION__, conn_id, opcode, data_len);

    switch (opcode)
    {
    case HCI_CONTROL_LE_AUDIO_COMMAND_VOL_UP:
        vcs_opcode = VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_UP;
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_VOL_DOWN:
        vcs_opcode = VOLUME_CONTROL_OPCODE_RELATIVE_VOLUME_DOWN;
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_UP:
        vcs_opcode = VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_UP;
       break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_DOWN:
        vcs_opcode = VOLUME_CONTROL_OPCODE_UNMUTE_RELATIVE_VOLUME_DOWN;
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_ABS_VOL:
        vcs_opcode = VOLUME_CONTROL_OPCODE_SET_ABSOLUTE_VOLUME;
        STREAM_TO_UINT8(abs_vol, p_data);
        break;
    default:
        return;
    }
    lepl_vcs_set_volume(conn_id, vcs_opcode, abs_vol);
}

static void lepl_rpc_handle_set_mute_state(uint8_t *p_data, uint32_t payload_len, uint16_t opcode)
{
    uint16_t conn_id;
    uint8_t mute_state = 0;
    STREAM_TO_UINT16(conn_id, p_data);
    if (opcode == HCI_CONTROL_LE_AUDIO_COMMAND_MUTE) mute_state = 1;

    WICED_BT_TRACE("[%s] conn_id %d mute state %d\n", __FUNCTION__, conn_id, mute_state);
    lepl_vcs_set_mute_state(conn_id, mute_state);
}

#define MAX_BROADCAST_CODE_LEN 16

static void lepl_rpc_broadcast_src_handle_start_streaming(uint8_t *p_data, uint32_t data_len)
{
    wiced_bt_ga_bap_stream_config_t stream_config;
    uint8_t start;
    uint32_t codec_config;
    uint8_t enable_encryption;
    uint32_t channel_counts;
    uint32_t broadcast_id;
    uint8_t broadcast_code[MAX_BROADCAST_CODE_LEN];
    uint8_t bis_count;

    wiced_result_t ret_sts = WICED_ERROR;
    STREAM_TO_UINT8(start, p_data);
    STREAM_TO_UINT32(codec_config, p_data);
    STREAM_TO_UINT8(enable_encryption, p_data);
    STREAM_TO_UINT32(channel_counts, p_data);
    STREAM_TO_UINT32(broadcast_id, p_data);
    STREAM_TO_ARRAY(broadcast_code, p_data, MAX_BROADCAST_CODE_LEN);
    STREAM_TO_UINT8(bis_count, p_data);

    WICED_BT_TRACE("[%s] Broadcast ID: %x", __FUNCTION__, broadcast_id);

    lepl_app_state_t state = lepl_cap_get_application_state();
    //if (state != LEPL_APP_STATE_IDLE)
    //{
    //    WICED_BT_TRACE_CRIT("[%s] Not ready to play! state: %d", __FUNCTION__, state);
    //    return;
    //}
    if (start)
    {
        
        lepl_broadcast_source_cb_t *p_big = lehs_get_broadcast_source_cb();
        if (p_big->base.state == BAP_BROADCAST_STATE_IDLE)
        {
            wiced_bt_ga_bap_get_broadcast_stream_config(codec_config, &stream_config);

            ret_sts = lepl_bis_configure_stream(broadcast_id,
                                                broadcast_code,
                                                bis_count,
                                                channel_counts,
                                                stream_config.sampling_frequency,
                                                stream_config.frame_duration,
                                                stream_config.octets_per_codec_frame,
                                                enable_encryption);
            WICED_BT_TRACE("[%s] Cfg stream SF:%d FD:%d OPF:%d res:%d\n",
                           __FUNCTION__,
                           stream_config.sampling_frequency,
                           stream_config.frame_duration,
                           stream_config.octets_per_codec_frame,
                           ret_sts);
        }
        lepl_cap_set_next_application_state(LEPL_APP_STATE_MEDIA, codec_config);
        lepl_bis_start_stream(&stream_config);
    }
    else
    {
        ret_sts = lepl_bis_disable_stream();
        WICED_BT_TRACE_CRIT("[%s] disable_stream 0x%x\n", __FUNCTION__, ret_sts);

        ret_sts = lepl_bis_release_stream();
        WICED_BT_TRACE_CRIT("[%s] release_stream 0x%x\n", __FUNCTION__, ret_sts);
    }
}

static void lepl_rpc_ccs_handle_generate_call_uri(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    uint8_t uri_len, f_name_len;
    char call_uri[MAX_URI_LEN] = {'\0'};
    char friendly_name[MAX_FRIENDLY_NAME_LEN] = {'\0'};

    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(uri_len, p_data);
    if (uri_len >= WICED_BT_GA_TBS_BEARER_URI_MAX_SIZE) uri_len = WICED_BT_GA_TBS_BEARER_URI_MAX_SIZE - 1;
    STREAM_TO_ARRAY(call_uri, p_data, uri_len);

    STREAM_TO_UINT8(f_name_len, p_data);
    if (f_name_len >= WICED_BT_GA_TBS_FRIENDLY_NAME_MAX_SIZE) f_name_len = WICED_BT_GA_TBS_FRIENDLY_NAME_MAX_SIZE - 1;
    STREAM_TO_ARRAY(friendly_name, p_data, f_name_len);

    WICED_BT_TRACE("[%s] conn_id %d call URI %s friendly_name %s  \n", __FUNCTION__, conn_id, call_uri, friendly_name);

    lepl_app_state_t state = lepl_cap_get_application_state();

    switch (state)
    {
    case LEPL_APP_STATE_IDLE:
        TRACE_LOG("LEPL_APP_STATE_IDLE");
        lepl_tbs_set_incoming_remote_call(&g_lepl_gatt_cb.tbs_data, call_uri, friendly_name);
        if (lepl_ccs_set_incoming_remote_call(conn_id) != WICED_SUCCESS)
        {
            return;
        }
        break;
    case LEPL_APP_STATE_MEDIA:
        TRACE_LOG("LEPL_APP_STATE_MEDIA");
        lepl_mcs_pause(conn_id);
        lepl_tbs_set_incoming_remote_call(&g_lepl_gatt_cb.tbs_data, call_uri, friendly_name);
        WICED_BT_TRACE("[%s] Not ready! current state: %d", __FUNCTION__, state);
        break;
    case LEPL_APP_STATE_MIC:
        TRACE_LOG("LEPL_APP_STATE_MIC");
        lepl_cap_stop_mic_streaming(conn_id);
        lepl_tbs_set_incoming_remote_call(&g_lepl_gatt_cb.tbs_data, call_uri, friendly_name);
        WICED_BT_TRACE("[%s] Not ready! current state: %d", __FUNCTION__, state);
        break;
    case LEPL_APP_STATE_IN_TRANSIT:
        TRACE_LOG("LEPL_APP_STATE_TRANSIT");
        state = lepl_cap_get_application_final_state();
        if (state == LEPL_APP_STATE_MEDIA)
            lepl_mcs_pause(conn_id);
        else if (state == LEPL_APP_STATE_MIC)
            lepl_cap_stop_mic_streaming(conn_id);
        else
            return;
        WICED_BT_TRACE("[%s] Not ready! current state: %d", __FUNCTION__, LEPL_APP_STATE_IN_TRANSIT);
        lepl_tbs_set_incoming_remote_call(&g_lepl_gatt_cb.tbs_data, call_uri, friendly_name);
        break;
    default:
        WICED_BT_TRACE_CRIT("[%s] Can't proceed state:%d", __FUNCTION__, state);
        return;
    }

    lepl_cap_set_next_application_state(LEPL_APP_STATE_CALL, BAP_CODEC_CONFIG_32_1_2);
}

static void lepl_rpc_ccs_simulate_remote_hold_retrieve_call(uint8_t *p_data, uint8_t data_len, wiced_bool_t hold)
{
    uint8_t call_id;
    STREAM_TO_UINT8(call_id, p_data);
    WICED_BT_TRACE("[%s] call_id %d \n", __FUNCTION__, call_id);
    if (hold)
        lepl_ccs_set_remote_hold_call(call_id);
    else
        lepl_ccs_set_retrieve_remote_hold_call(call_id);
}

static void lepl_rpc_ccs_handle_terminate_call(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    uint8_t call_id, termination_reason;

    lepl_app_state_t state = lepl_cap_get_application_state();
    if (state == LEPL_APP_STATE_IN_TRANSIT)
        state = lepl_cap_get_application_final_state();

    if (state != LEPL_APP_STATE_CALL)
    {
        WICED_BT_TRACE_CRIT("[%s] state: %d", __FUNCTION__, state);
        return;
    }

    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(call_id, p_data);
    STREAM_TO_UINT8(termination_reason, p_data);
    if (lepl_ccs_terminate_call(conn_id, call_id, termination_reason) == WICED_SUCCESS)
    {
        lepl_cap_set_next_application_state(LEPL_APP_STATE_IDLE, 0XFF);
    }
}

void lepl_rpc_send_mic_state(uint8_t state)
{
    for (int i = 0; i < MAX_CONNECTION_INSTANCE; i++)
    {
        if (g_lepl_gatt_cb.unicast_clcb[i].in_use)
        {
            le_audio_rpc_send_mic_state_update(g_lepl_gatt_cb.unicast_clcb[i].conn_id, state);
        }
    }
}

void lepl_start_voice_capture(uint16_t conn_id, uint32_t codec_config)
{
    lepl_stream_config_t mic_stream_config = {0};
    lepl_device_config_t config1[2] = {0};
    wiced_bt_ga_bap_stream_config_t stream_config = {0};

    wiced_bt_ga_bap_get_unicast_stream_config(codec_config, &stream_config);

    WICED_BT_TRACE("[%s] start voice assistant %x\n",__FUNCTION__, conn_id);

    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    if (!p_clcb)
    {
        WICED_BT_TRACE_CRIT("No clcb");
        return;
    }

    if (p_clcb->peer_profiles.p_csis &&
        !lepl_if_sirk_zero(p_clcb->csis_data.sirk_data.sirk))
    {
        for (int i = 0; i < MAX_CONNECTION_INSTANCE; i++)
        {
            if (lepl_csis_device_belongs_to_coordinated_set(g_lepl_gatt_cb.unicast_clcb[i].conn_id,
                                                            p_clcb->csis_data.sirk_data.sirk))
            {
                config1[mic_stream_config.num_devices].audio_location =
                    g_lepl_gatt_cb.unicast_clcb[i].p_cap->p_pacs_data->sink_audio_location &
                    (BAP_AUDIO_LOCATION_FRONT_LEFT | BAP_AUDIO_LOCATION_FRONT_RIGHT);
                config1[mic_stream_config.num_devices].conn_id = g_lepl_gatt_cb.unicast_clcb[i].conn_id;
                mic_stream_config.num_devices++;
            }
        }
    }
    else
    {
        config1[mic_stream_config.num_devices].audio_location =
            g_lepl_gatt_cb.unicast_clcb[0].p_cap->p_pacs_data->sink_audio_location &
            (BAP_AUDIO_LOCATION_FRONT_LEFT | BAP_AUDIO_LOCATION_FRONT_RIGHT);
        config1[mic_stream_config.num_devices].conn_id = conn_id;
        mic_stream_config.num_devices = 1;
    }

    mic_stream_config.config_list = config1;
    mic_stream_config.stream_config = &stream_config;
    mic_stream_config.ctx_type = BAP_CONTEXT_TYPE_UNSPECIFIED | BAP_CONTEXT_TYPE_CONVERSATIONAL;

    lepl_cap_start_mic_streaming(&mic_stream_config);
    lepl_rpc_send_mic_state(1);
}

static void lepl_rpc_handle_start_stop_capture_voice(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    uint32_t codec_config;
    uint8_t start;
    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(start, p_data);
    STREAM_TO_UINT16(codec_config, p_data);

    lepl_app_state_t state = lepl_cap_get_application_state();

    if (start)
    {
        switch (state)
        {
        case LEPL_APP_STATE_IDLE:
            lepl_start_voice_capture(conn_id, codec_config);
            break;
        case LEPL_APP_STATE_MEDIA:
            lepl_mcs_pause(conn_id);
            WICED_BT_TRACE("[%s] Not ready! current state: %d", __FUNCTION__, state);
            break;
        case LEPL_APP_STATE_IN_TRANSIT:
            if (lepl_cap_get_application_final_state() == LEPL_APP_STATE_MEDIA)
            {
                lepl_mcs_pause(conn_id);
                WICED_BT_TRACE("[%s] Not ready! current state: %d", __FUNCTION__, state);
            }
            break;
         default:
            WICED_BT_TRACE_CRIT("[%s] Not ready! current state: %d", __FUNCTION__, state);
            return;
            break;
        }
        lepl_cap_set_next_application_state(LEPL_APP_STATE_MIC, codec_config);
    }
    else
    {
        if (state == LEPL_APP_STATE_IN_TRANSIT)
            state = lepl_cap_get_application_final_state();

        if (state != LEPL_APP_STATE_MIC)
        {
            WICED_BT_TRACE_CRIT("[%s] state: %d", __FUNCTION__, state);
            return;
        }
        lepl_cap_stop_mic_streaming(conn_id);
        lepl_cap_set_next_application_state(LEPL_APP_STATE_IDLE, 0XFF);
    }
}

static void lepl_rpc_mics_mute(uint8_t* p_data, uint8_t data_len)
{
    uint16_t conn_id;
    uint8_t mute;
    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(mute, p_data);

    lepl_micp_mute(conn_id, mute);
}

static void lepl_rpc_mics_aics_mute(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    uint8_t mute;
    uint32_t instance;
    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(mute, p_data);
    STREAM_TO_UINT32(instance, p_data);

    lepl_micp_aics_mute(conn_id, instance, mute);
}

static void lepl_rpc_mics_aics_set_gain(uint8_t* p_data, uint8_t data_len)
{
    uint16_t conn_id;
    uint8_t opcode;
    uint32_t instance;
    int8_t gain = 0;
    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(opcode, p_data);
    STREAM_TO_UINT32(instance, p_data);
    if (data_len - 7) STREAM_TO_INT8(gain, p_data);

    lepl_micp_aics_set_gain(conn_id, instance, opcode, gain);
}

static void lepl_rpc_hap_read_preset(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    STREAM_TO_UINT16(conn_id, p_data);
    lepl_hap_read_preset_records(conn_id);
}

static void lepl_rpc_hap_write_preset_name(uint8_t *p_data, uint8_t data_len)
{
    uint16_t conn_id;
    uint8_t preset_index;
    gatt_intf_string_t name;
    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(preset_index, p_data);
    name.len = data_len - 3;
    name.str = (char *)p_data;
    lepl_hap_set_preset_name(conn_id, preset_index, &name);
}

static void lepl_rpc_hap_set_active_preset(uint8_t* p_data, uint8_t data_len)
{
    uint16_t conn_id;
    uint8_t opcode;
    uint8_t preset_index;
    STREAM_TO_UINT16(conn_id, p_data);
    STREAM_TO_UINT8(opcode, p_data);
    STREAM_TO_UINT8(preset_index, p_data);
    lepl_hap_set_active_preset(conn_id, opcode, preset_index);
}

#ifdef HS_SPK_ENABLED
static wiced_timer_t hci_control_update_role_timer;
static wiced_bt_device_address_t update_role_bda;
void hci_control_update_role_timer_cb(wiced_timer_callback_arg_t data)
{
    wiced_result_t status;
    wiced_bt_dev_role_t curr_role;
    status = wiced_bt_dev_get_role(update_role_bda, &curr_role, BT_TRANSPORT_BR_EDR);
    if ( (status == WICED_BT_SUCCESS) && (curr_role == HCI_ROLE_PERIPHERAL))
    {
        // Switch role to master and disable role switch so we can be in master role.
        uint16_t new_link_policy = HCI_ENABLE_SNIFF_MODE;
        // Comment below two lines to avoid the role switch
        wiced_bt_dev_switch_role(update_role_bda, HCI_ROLE_CENTRAL, NULL);
        wiced_bt_dev_set_link_policy(update_role_bda, &new_link_policy);
#ifdef DISABLE_3M_PKT
        wiced_bt_dev_setAclPacketTypes(update_role_bda,
                HCI_PKT_TYPES_MASK_DM5 | HCI_PKT_TYPES_MASK_DH5 | /* Use 1 mbps 5 slot packets */
                HCI_PKT_TYPES_MASK_DH3 | HCI_PKT_TYPES_MASK_DM3 | /* Use 1 mbps 3 slot packets */
                HCI_PKT_TYPES_MASK_DH1 | HCI_PKT_TYPES_MASK_DM1 | /* Use 1 mbps 1 slot packets */
                HCI_PKT_TYPES_MASK_NO_3_DH1 |               /* Don't use 3 mbps 1 slot packets */
                HCI_PKT_TYPES_MASK_NO_3_DH3 |               /* Don't use 3 mbps 3 slot packets */
                HCI_PKT_TYPES_MASK_NO_3_DH5);               /* Don't use 3 mbps 5 slot packets */
#endif // DISABLE_3M_PKT
    }
}

void hci_control_connection_status_callback (wiced_bt_device_address_t bd_addr, uint8_t *p_features, wiced_bool_t is_connected, uint16_t handle, wiced_bt_transport_t transport, uint8_t reason)
{
    uint8_t event_data[2];

    //Build event payload
    event_data[0] = is_connected;
    event_data[1] = reason;
#ifdef HS_SPK_ENABLED
    if ( (transport == BT_TRANSPORT_BR_EDR) && (is_connected == WICED_TRUE))
    {
        wiced_init_timer(&hci_control_update_role_timer, hci_control_update_role_timer_cb, NULL, WICED_MILLI_SECONDS_TIMER);
        memcpy(update_role_bda, bd_addr, BD_ADDR_LEN);
        wiced_start_timer(&hci_control_update_role_timer, 50);
    }
#endif

    le_audio_rpc_send_data( HCI_CONTROL_EVENT_CONNECTION_STATUS, event_data, 2 );

    WICED_BT_TRACE("%s  is_connected:%d reason:%x\n", __FUNCTION__, is_connected, reason );
}

void hci_control_audio_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint32_t handle )
{
    int i;
    uint8_t event_data[BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t)];

    WICED_BT_TRACE( "[%s] %B status %x handle %x\n", __FUNCTION__, bd_addr, status, handle );

    //Build event payload
    if ( status == WICED_SUCCESS )
    {
        for ( i = 0; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

        event_data[i++] = handle & 0xff;                        //handle
        event_data[i++]   = ( handle >> 8 ) & 0xff;

        //event_data[i] = wiced_bt_rc_target_is_peer_absolute_volume_capable( );
        le_audio_rpc_send_data( HCI_CONTROL_AUDIO_SINK_EVENT_CONNECTED, event_data, sizeof(event_data));
    }
    else
    {
        le_audio_rpc_send_data( HCI_CONTROL_AUDIO_SINK_EVENT_CONNECTION_FAILED, NULL, 0 );
    }
}

/*
 *  send audio disconnect complete event to UART
 */
void hci_control_audio_send_disconnect_complete( uint32_t handle, uint8_t status, uint8_t reason )
{
    uint8_t event_data[4];

    WICED_BT_TRACE( "[%s] %04x status %d reason %d\n", __FUNCTION__, handle, status, reason );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;
    event_data[2] = status;                                 // status
    event_data[3] = reason;                                 // reason(1 byte)

    le_audio_rpc_send_data( HCI_CONTROL_AUDIO_SINK_EVENT_DISCONNECTED, event_data, 4 );
}

/*
 *  send audio connect complete event to UART
 */
void hci_control_audio_send_started_stopped( uint32_t handle, wiced_bool_t started )
{
    uint8_t event_data[2];

    WICED_BT_TRACE( "[%s] handle %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    le_audio_rpc_send_data(started ? HCI_CONTROL_AUDIO_SINK_EVENT_STARTED : HCI_CONTROL_AUDIO_SINK_EVENT_STOPPED, event_data, 2);
}

void hci_control_avrc_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle )
{
    int i = 0;
    uint8_t event_data[BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t)];

    WICED_BT_TRACE( "[%s] %B status %x handle %x\n", __FUNCTION__, bd_addr, status, handle );

    //Build event payload
    if ( status == WICED_SUCCESS )
    {
        for ( ; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

        event_data[i++] = status;

        event_data[i++] = handle & 0xff;                        //handle
        event_data[i++] = ( handle >> 8 ) & 0xff;

    }
    else
    {
        event_data[i++] = status;
    }

    le_audio_rpc_send_data( HCI_CONTROL_AVRC_CONTROLLER_EVENT_CONNECTED, event_data, i );
}

/*
 *  send avrcp controller disconnect complete event to UART
 */
void hci_control_avrc_send_disconnect_complete( uint16_t handle )
{
    uint8_t event_data[4];

    WICED_BT_TRACE( "[%s] handle: %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    le_audio_rpc_send_data( HCI_CONTROL_AVRC_CONTROLLER_EVENT_DISCONNECTED, event_data, 2 );
}

void hci_control_send_encryption_changed_evt( uint8_t encrypted ,  wiced_bt_device_address_t bdaddr )
{
    int i;
    uint8_t event_data[BD_ADDR_LEN + sizeof(uint8_t)];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = encrypted;

    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    le_audio_rpc_send_data( HCI_CONTROL_EVENT_ENCRYPTION_CHANGED, event_data, cmd_bytes );
}

void hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr )
{
    int i;

    uint8_t event_data[BD_ADDR_LEN + sizeof(uint8_t)];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = status;

    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    WICED_BT_TRACE( "pairing complete evt: %B as %B status %d\n", bdaddr, &event_data[1], status );

    le_audio_rpc_send_data( HCI_CONTROL_EVENT_PAIRING_COMPLETE, event_data, sizeof(event_data));
}

void _le_audio_rpc_handle_get_version(void)
{
    uint8_t tx_buf[15];
    uint8_t idx = 0;
    uint32_t chip = 55571;

    tx_buf[idx++] = 0;
    tx_buf[idx++] = 0;
    tx_buf[idx++] = 0;
    tx_buf[idx++] = 0;
    tx_buf[idx++] = 0;
    tx_buf[idx++] = chip & 0xFF;
    tx_buf[idx++] = (chip >> 8) & 0xFF;
    tx_buf[idx++] = (chip >> 24) & 0xFF;
    tx_buf[idx++] = 0; // not used

    /* Send MCU app the supported features */
    tx_buf[idx++] = HCI_CONTROL_GROUP_AVRC_CONTROLLER;
    tx_buf[idx++] = HCI_CONTROL_GROUP_HF;
    tx_buf[idx++] = HCI_CONTROL_GROUP_LE_AUDIO;

    le_audio_rpc_send_data(HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, idx);
}
#endif // HS_SPK_ENABLED

wiced_bool_t lepl_rpc_rx_cback(uint16_t opcode, uint8_t *p_data, uint32_t payload_len)
{
    wiced_bool_t b_response_sent = TRUE;

    WICED_BT_TRACE("[%s] [opcode 0x%04x] (%d bytes)\n", __FUNCTION__, opcode, payload_len);

    switch (opcode)
    {
    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
#ifdef HS_SPK_ENABLED
    {
        uint8_t tx_buf[8];
        uint8_t *p = tx_buf;
        uint32_t dev_role = HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE |
                HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SOURCE |
                HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_SERVER;
        UINT32_TO_STREAM(p, dev_role);
        _le_audio_rpc_handle_get_version();
        le_audio_rpc_send_data(HCI_CONTROL_LE_AUDIO_EVENT_DEVICE_ROLE, tx_buf, p - tx_buf);
    }
#else

        le_audio_rpc_send_dev_role(HCI_CONTROL_LE_AUDIO_DEV_ROLE_UNICAST_SOURCE |
                                   HCI_CONTROL_LE_AUDIO_DEV_ROLE_BROADCAST_SOURCE |
                                   HCI_CONTROL_LE_AUDIO_DEV_ROLE_CALL_CONTROL_SERVER
                                   );
#endif
        break;

    case HCI_CONTROL_LE_COMMAND_ADVERTISE:
        lepl_rpc_handle_adv(p_data, payload_len);
        break;
    case HCI_CONTROL_LE_COMMAND_SCAN:
        lepl_rpc_handle_scan(p_data, payload_len);
        break;
    case HCI_CONTROL_LE_COMMAND_CONNECT:
        lepl_rpc_handle_connect(p_data, payload_len, 0);
        break;
    case HCI_CONTROL_LE_COMMAND_CANCEL_CONNECT:
        lepl_rpc_handle_connect(p_data, payload_len, 1);
        break;
    case HCI_CONTROL_LE_COMMAND_DISCONNECT:
        lepl_rpc_handle_disconnect(p_data, payload_len);
        break;
    case HCI_CONTROL_MISC_COMMAND_PING:
        le_audio_rpc_send_data(HCI_CONTROL_MISC_EVENT_PING_REPLY, p_data, payload_len);
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_PLAY:
        lepl_rpc_handle_play(p_data, payload_len);
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_PAUSE:
        lepl_rpc_handle_pause(p_data, payload_len);
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_VOL_UP:
    case HCI_CONTROL_LE_AUDIO_COMMAND_VOL_DOWN:
    case HCI_CONTROL_LE_AUDIO_COMMAND_ABS_VOL:
    case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_UP:
    case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE_VOL_DOWN:
        lepl_rpc_vcs_set_vol(p_data, payload_len, opcode);
        break;

    case HCI_CONTROL_LE_AUDIO_COMMAND_MUTE:
    case HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE:
        lepl_rpc_handle_set_mute_state(p_data, payload_len, opcode);
        break;

    case HCI_CONTROL_LE_AUDIO_COMMAND_BROADCAST_SOURCE_START_STREAMIMG:
        lepl_rpc_broadcast_src_handle_start_streaming(p_data, payload_len);
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_PLACE_CALL:
        lepl_rpc_ccs_handle_generate_call_uri(p_data, payload_len);
        break;

    case HCI_CONTROL_LE_AUDIO_COMMAND_REM_HOLD_CALL:
        lepl_rpc_ccs_simulate_remote_hold_retrieve_call(p_data, payload_len, TRUE);
        break;

    case HCI_CONTROL_LE_AUDIO_COMMAND_REM_HOLD_RETRIEVE:
        lepl_rpc_ccs_simulate_remote_hold_retrieve_call(p_data, payload_len, FALSE);
        break;

    case HCI_CONTROL_LE_AUDIO_COMMAND_TERMINATE_CALL:
        lepl_rpc_ccs_handle_terminate_call(p_data, payload_len);
        break;

    case HCI_CONTROL_LE_AUDIO_COMMAND_START_STOP_MIC:
        lepl_rpc_handle_start_stop_capture_voice(p_data, payload_len);
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_MICP_MUTE:
        lepl_rpc_mics_mute(p_data, payload_len);
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_MICP_AICS_MUTE:
        lepl_rpc_mics_aics_mute(p_data, payload_len);
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_MICP_AICS_SET_GAIN:
        lepl_rpc_mics_aics_set_gain(p_data, payload_len);
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_HAS_READ_PRESET:
        lepl_rpc_hap_read_preset(p_data, payload_len);
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_HAS_WRITE_PRESET_NAME:
        lepl_rpc_hap_write_preset_name(p_data, payload_len);
        break;
    case HCI_CONTROL_LE_AUDIO_COMMAND_HAS_SET_PRESET:
        lepl_rpc_hap_set_active_preset(p_data, payload_len);
        break;
#ifdef HS_SPK_ENABLED
   case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_UP:
       WICED_BT_TRACE("[%s] VOLUME_UP_NEXT_TRACK_BUTTON - BUTTON_CLICK_EVENT - BUTTON_STATE_RELEASED\n", __FUNCTION__);
       bt_hs_spk_app_service_action_run(ACTION_VOLUME_UP);
       break;
   case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_DOWN:
       WICED_BT_TRACE("[%s] VOLUME_DOWN_PREVIOUS_TRACK_BUTTON - BUTTON_CLICK_EVENT - BUTTON_STATE_RELEASED\n", __FUNCTION__);
       bt_hs_spk_app_service_action_run(ACTION_VOLUME_UP);
       break;
   case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PLAY:
   case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PAUSE:
       extern wiced_result_t service_pause_play(void);
       WICED_BT_TRACE("[%s] PLAY_PAUSE_BUTTON - BUTTON_CLICK_EVENT - BUTTON_STATE_RELEASED\n", __FUNCTION__);
       bt_hs_spk_app_service_action_run(ACTION_PAUSE_PLAY);
       break;
#endif // HS_SPK_ENABLED
    default:
        b_response_sent = FALSE;
        WICED_BT_TRACE("[%s] Unknown Function code [%d] \n", __FUNCTION__, opcode);
        break;
    }

    return b_response_sent;
}

void lepl_rpc_init(uint8_t app_instance)
{
    /* RPC to work with LE Audio Client Control */
    le_audio_rpc_init(app_instance, lepl_rpc_rx_cback, TRUE);
}
