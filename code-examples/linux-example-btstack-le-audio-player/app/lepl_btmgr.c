/*
 * $ Copyright Cypress Semiconductor $
 */

/* Application includes */
#include "lepl.h"
#ifdef HS_SPK_ENABLED
#include "headset_nvram.h"
#include "bt_hs_spk_control.h"
#include "bt_hs_spk_handsfree.h"
#include "wiced_bt_a2dp_sink.h"
#include "lepl.h"
#include "wiced_lite_host.h"
#endif // HS_SPK_ENABLED
#include "wiced_bt_l2c.h"

#ifdef HS_SPK_ENABLED
extern const uint8_t                btheadset_sdp_db[];
extern wiced_bt_a2dp_config_data_t  bt_audio_config;
extern uint8_t                      bt_avrc_ct_supported_events[];
extern uint16_t wiced_app_cfg_sdp_record_get_size(void);
extern void hci_control_connection_status_callback (wiced_bt_device_address_t bd_addr, uint8_t *p_features, wiced_bool_t is_connected, uint16_t handle, wiced_bt_transport_t transport, uint8_t reason);
extern void hci_control_avrc_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle );
extern void hci_control_avrc_send_disconnect_complete( uint16_t handle );
extern void hci_control_send_encryption_changed_evt( uint8_t encrypted ,  wiced_bt_device_address_t bdaddr );
extern void hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr );
extern void hci_control_audio_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint32_t handle );
extern void hci_control_audio_send_disconnect_complete( uint32_t handle, uint8_t status, uint8_t reason );
extern void hci_control_audio_send_started_stopped( uint32_t handle, wiced_bool_t started );
#endif // HS_SPK_ENABLED

#define LE_ISOCHRONOUS_CHANNELS_META_COMMAND 0xFDDF

extern wiced_bt_cfg_ble_t lepl_ble_cfg;
extern void set_local_bd_addr(void);
static wiced_bt_device_link_keys_t br_edr_link_key;

// ga_cfg_t lepl_ga_cfg = {.pacs_max_sink_capabilities_supported = MAX_PACS_SINK_CAP_SUPPORTED,
//                         .pacs_max_source_capabilities_supported = MAX_PACS_SOURCE_CAP_SUPPORTED,

//                         .ascs_max_sink_ase_supported = MAX_SINK_ASE_SUPPORTED,
//                         .ascs_max_source_ase_supported = MAX_SOURCE_ASE_SUPPORTED,

//                         .vcs.max_vcs_vocs = 5,
//                         .vcs.max_vcs_aics = 5,
//                         .max_tbs = 2,

//                         .max_mcs = 1,
//                         .mics.max_mics_aics = 5};

static void set_host_enforced_BIG_offset(void)
{
    uint8_t param[] = {
        0x08,// Sub_Code (8 - Host Enforced Min BIG Offset of 5 ms)
        0x01 // hostEnforcedMinBigOffset (1 - Enable Enforced Min BIG Offset)
    };

    wiced_bt_dev_vendor_specific_command(LE_ISOCHRONOUS_CHANNELS_META_COMMAND, sizeof(param), param, NULL);
}
#ifdef HS_SPK_ENABLED
#include "wiced_hal_pcm.h"
#define HCI_OPCODE_WRITE_SCO_TS                             (0xFC22)
#define HCI_OPCODE_WRITE_SCO_PCM_INT_PARAMS                 (0xFC1C)
#define WICED_HAL_MXTDM0_I2S_MODE        (2)
#define WICED_HAL_MXTDM0_PCM_MODE        (3)
#define WICED_HAL_MXTDM1_I2S_MODE        (4)
#define WICED_HAL_MXTDM1_PCM_MODE        (5)
void lepl_set_hfp_tdm_params(void)
{
	uint8_t channel_num = ARIP_MXTDM_HFP_CH;
	uint8_t pcm_int_params[] = {ARIP_MXTDM_PAIR + 4, 0, 0, 0, 0};

	wiced_hal_pcm_config_t wiced_hal_pcm_params =
	{
		WICED_HAL_MXTDM1_I2S_MODE,
		WICED_HAL_PCM_MASTER,
		// Below settings are not applicable for I2S MODE
		{
			WICED_HAL_PCM_MSB_FIRST,
			0,
			WICED_HAL_PCM_FILL_0S,
			3,
			WICED_HAL_PCM_DISABLE_RIGHT_JUSTIFY,
			WICED_HAL_PCM_FRAME_TYPE_SHORT
		}
	};
	if (ARIP_MXTDM_MODE == MXTDM_MODE_TDM)
	{
		if (ARIP_MXTDM_PAIR == MXTDM_PAIR_0)
			wiced_hal_pcm_params.mode = WICED_HAL_MXTDM0_PCM_MODE;
		else
			wiced_hal_pcm_params.mode = WICED_HAL_MXTDM1_PCM_MODE;
	}
	else
	{
		if (ARIP_MXTDM_PAIR == MXTDM_PAIR_0)
			wiced_hal_pcm_params.mode = WICED_HAL_MXTDM0_I2S_MODE;
		else
			wiced_hal_pcm_params.mode = WICED_HAL_MXTDM1_I2S_MODE;
	}
	if (ARIP_MXTDM_ROLE == MXTDM_BUS_SLAVE)
		wiced_hal_pcm_params.role = WICED_HAL_PCM_SLAVE;
	else
		wiced_hal_pcm_params.role = WICED_HAL_PCM_MASTER;

	wiced_hal_set_pcm_config(&wiced_hal_pcm_params);
	// Set channel number for SCO
	wiced_bt_dev_vendor_specific_command(HCI_OPCODE_WRITE_SCO_TS, sizeof(channel_num),&channel_num,NULL);
	wiced_bt_dev_vendor_specific_command(HCI_OPCODE_WRITE_SCO_PCM_INT_PARAMS, sizeof(pcm_int_params),pcm_int_params,NULL);
}
#ifdef AUDIO_TRANSCODING
void lepl_enable_transcoding(wiced_bool_t enable)
{
	uint8_t vsc_params[] = { 0xFC,  // routing, select Transcoding
			1,                      // MXTDM pair
			0,                      // Left Channel
			1,                      // Right Channel
			0                       // Enable
			};
	vsc_params[4] = enable;
	wiced_bt_dev_vendor_specific_command(HCI_OPCODE_WRITE_SCO_PCM_INT_PARAMS, sizeof(vsc_params),vsc_params,NULL);

}
#else
void lepl_set_tdm_params(void)
{
    wiced_bt_tdm_composite_setting_t inParam;
    wiced_bt_tdm_composite_channel_setting_t inParam2;

    inParam.mxtdmPair = ARIP_MXTDM_PAIR;
    inParam.chNumber = ARIP_MXTDM_TDM_CH_NUM;
    inParam.mode = ARIP_MXTDM_MODE;
    inParam.busRole = ARIP_MXTDM_ROLE;
    inParam.enable = TRUE;
    inParam.a2dpEnable = TRUE;
    inParam.a2dpLeftCh = ARIP_MXTDM_A2DP_LEFT_CH;
    inParam.a2dpRightCh = ARIP_MXTDM_A2DP_RIGHT_CH;
    wiced_lite_host_set_compositeMxtdm(&inParam);

    inParam2.mxtdmPair = ARIP_MXTDM_PAIR;
    inParam2.mode = ARIP_MXTDM_MODE;
    inParam2.mxtdmTxHwChMap = ARIP_MXTDM_TDM_TX_CH_MAP;
    inParam2.mxtdmRxHwChMap = ARIP_MXTDM_TDM_RX_CH_MAP;

    wiced_lite_host_set_compositeMxtdmChannel(&inParam2);
}
#endif // AUDIO_TRANSCODING
#endif // HS_SPK_ENABLED
wiced_result_t lepl_handle_btm_enabled(wiced_bt_dev_enabled_t *p_btm_enabled)
{
    wiced_bt_gatt_status_t sts = WICED_BT_ERROR;
#ifdef HS_SPK_ENABLED
    uint8_t param_buf[2] ={0x0a,0};
    uint8_t param_buf2[7] ={0x0b,0x01, 0x00, 0x01, 0x00, 0x40, 0x04};
    uint8_t param_44_to_48[] ={0xfb, 0x01, 0x01, 0x00, 0x00};
#endif

    /* Initialize and Register ISOC callabck */
    /* Initialize audio interfaces and register callbacks for data handling */
    lepl_isoc_init();

    set_local_bd_addr();

    /* Initialize GATT */
    lepl_ga_init_data_t lepl_init_data = {
        .p_ascs_init_data = &(wiced_bt_ga_ascs_init_data_t) {
            .max_snk_ase_spt=MAX_SINK_ASE_SUPPORTED,
            .max_src_ase_spt=MAX_SOURCE_ASE_SUPPORTED
        },
        .p_pacs_init_data = &(wiced_bt_ga_pacs_init_data_t) {
            .max_snk_caps_spt=MAX_PACS_SINK_CAP_SUPPORTED,
            .max_src_caps_spt=MAX_PACS_SOURCE_CAP_SUPPORTED
        },
        .p_vcs_init_data = &(wiced_bt_ga_vcs_init_data_t) {
            .max_aics=5,
            .max_vocs=5,
            .step_size=15
        },
        .p_mics_init_data = &(wiced_bt_ga_mics_init_data_t) {
            .max_aics=5
        }
    };
    sts = lepl_gatt_init(lepl_ble_cfg.ble_max_simultaneous_links,
                                   lepl_ble_cfg.ble_max_rx_pdu_size,
                                   &lepl_init_data);
    if (sts) WICED_BT_TRACE("[%s] GATT init sts %d\n", __FUNCTION__, sts);

    // Sets offset of 5ms to ISOC data
    set_host_enforced_BIG_offset();

#ifdef HS_SPK_ENABLED

#ifdef AUDIO_TRANSCODING
    lepl_enable_transcoding(WICED_TRUE);
#else
    lepl_set_tdm_params();
#endif // AUDIO_TRANSCODING
    // disable LE audio source timestamp
    wiced_bt_dev_vendor_specific_command(0xFDDF,sizeof(param_buf),param_buf,NULL);

    // jitter buffer as  1 *  transport_latency + 0x04 * sdu_interval
    wiced_bt_dev_vendor_specific_command(0xFDDF,sizeof(param_buf2),param_buf2,NULL);

    // Enable 44.1 to 48K sample rate
    wiced_bt_dev_vendor_specific_command(HCI_OPCODE_WRITE_SCO_PCM_INT_PARAMS,sizeof(param_44_to_48),param_44_to_48,NULL);
#endif

    /* Initialize profiles */
    lepl_mcs_initialize_data();
    lepl_tbs_initialize_data();

    return WICED_SUCCESS;
}

wiced_result_t lepl_btm_handle_encryption_sts(wiced_bt_dev_encryption_status_t *p_encryption_sts)
{
    /* Start GATT Discovery */
    if (WICED_SUCCESS != p_encryption_sts->result) return WICED_ERROR;

    lepl_gatt_start_discovery(p_encryption_sts->bd_addr);

    return WICED_SUCCESS;
}


extern void handsfree_post_bt_init(wiced_bt_management_evt_data_t *p_event_data);

#ifdef HS_SPK_ENABLED

void btheadset_connection_state_cb( uint8_t handle, wiced_bt_device_address_t remote_addr,
                            wiced_result_t status, wiced_bt_avrc_ct_connection_state_t connection_state,
                            uint32_t peer_features)
{

    switch( connection_state )
    {
    case REMOTE_CONTROL_DISCONNECTED:
        hci_control_avrc_send_disconnect_complete(handle);
        break;
    case REMOTE_CONTROL_CONNECTED:
        hci_control_avrc_send_connect_complete(remote_addr, WICED_SUCCESS, handle);
        break;
    default:
        break;
    }
}

void btheadset_update_flow_params(wiced_bt_device_address_t bd_addr)
{
    extern wiced_result_t wiced_bt_dev_qos_setup_by_bda(wiced_bt_device_address_t remote_bda, wiced_bt_flow_spec_t *p_flow, wiced_bt_dev_cmpl_cback_t *p_cb);
    wiced_bt_flow_spec_t flow;
    flow.qos_flags = 0;
    flow.service_type = 1;
    //flow.token_rate = 22400; // Tpool 16
    flow.token_rate = 58300; // Tpool 6
    flow.peak_bandwidth =  0;
    flow.latency = 0xFFFFFFFF;
    flow.delay_variation = 0xFFFFFFFF;
    wiced_bt_dev_qos_setup_by_bda(bd_addr, &flow, NULL);
}

void btheadset_a2dp_post_handler( wiced_bt_a2dp_sink_event_t event, wiced_bt_a2dp_sink_event_data_t* p_data )
{
    WICED_BT_TRACE( "[%s] Event: (%d)\n", __FUNCTION__, event );

    switch(event)
    {
        case WICED_BT_A2DP_SINK_CONNECT_EVT:
            if (p_data->connect.result == WICED_SUCCESS)
            {
                /* Notify MCU of connection state change */
                hci_control_audio_send_connect_complete( p_data->connect.bd_addr, WICED_SUCCESS, p_data->connect.handle );

                WICED_BT_TRACE( "[%s] A2DP sink connected to addr: <%B> Handle:%d\n", __FUNCTION__, p_data->connect.bd_addr,
                    p_data->connect.handle );
            }
            else
            {
                /* Notify MCU of connection state change */
                hci_control_audio_send_connect_complete( NULL, (uint8_t)WICED_BT_ERROR, 0 );

                WICED_BT_TRACE(" a2dp sink connection failed %d \n", p_data->connect.result );
            }
            break;

        case WICED_BT_A2DP_SINK_DISCONNECT_EVT:
            WICED_BT_TRACE(" a2dp sink disconnected  handle:%d\n", p_data->disconnect.handle);
            hci_control_audio_send_disconnect_complete( p_data->disconnect.handle, 0, 0 );
            break;

        case WICED_BT_A2DP_SINK_START_IND_EVT:
            WICED_BT_TRACE(" WICED_BT_A2DP_SINK_START_IND_EVT handle: %x \n", p_data->start_ind.handle );
            btheadset_update_flow_params(p_data->start_ind.bdaddr);
            break;

        case WICED_BT_A2DP_SINK_START_CFM_EVT:
            WICED_BT_TRACE(" WICED_BT_A2DP_SINK_START_CFM_EVT handle: %x \n", p_data->start_cfm.handle );
            btheadset_update_flow_params(p_data->start_cfm.bd_addr);
            hci_control_audio_send_started_stopped( p_data->start_cfm.handle, TRUE );
            break;

        case WICED_BT_A2DP_SINK_SUSPEND_EVT:
            WICED_BT_TRACE(" a2dp sink streaming suspended Handle:%d\n", p_data->suspend.handle);
            hci_control_audio_send_started_stopped( p_data->suspend.handle, FALSE );
            break;

        default:
            break;
    }
}


void bthfp_post_handler( wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data)
{
}

static wiced_result_t btheadset_post_bt_init(void)
{
    wiced_bool_t ret = WICED_FALSE;
    bt_hs_spk_control_config_t config = {0};
    bt_hs_spk_eir_config_t eir = {0};

    eir.p_dev_name              = (char *) lepl_cfg_settings.device_name;
    eir.default_uuid_included   = WICED_TRUE;

    WICED_BT_TRACE("%s\n",__FUNCTION__);

    if (WICED_SUCCESS != bt_hs_spk_write_eir(&eir))
    {
        WICED_BT_TRACE("Write EIR Failed\n");
    }

    ret = wiced_bt_sdp_db_init((uint8_t *) btheadset_sdp_db, wiced_app_cfg_sdp_record_get_size());
    if (ret != TRUE)
    {
        WICED_BT_TRACE("%s Failed to Initialize SDP databse\n", __FUNCTION__);
        return WICED_BT_ERROR;
    }

    config.conn_status_change_cb            = NULL;
#ifdef LOW_POWER_MEASURE_MODE
    config.discoverable_timeout             = 60;   /* 60 Sec */
#else
    config.discoverable_timeout             = 240;  /* 240 Sec */
#endif
    config.acl3mbpsPacketSupport            = WICED_TRUE;
    config.audio.a2dp.p_audio_config        = &bt_audio_config;
    config.audio.a2dp.p_pre_handler         = NULL;
    config.audio.a2dp.post_handler          = btheadset_a2dp_post_handler;
    config.audio.avrc_ct.p_supported_events = bt_avrc_ct_supported_events;
    config.audio.avrc_ct.connection_state_cb.post_handler = btheadset_connection_state_cb;
    config.conn_status_change_cb = hci_control_connection_status_callback;
    config.hfp.rfcomm.buffer_size           = 700;
    config.hfp.rfcomm.buffer_count          = 4;
    config.hfp.post_handler = bthfp_post_handler;
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
    config.hfp.feature_mask                 = WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                              WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                              WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                              WICED_BT_HFP_HF_FEATURE_HF_INDICATORS | \
                                              WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION | \
                                              WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION | \
                                              WICED_BT_HFP_HF_FEATURE_ESCO_S4_SETTINGS_SUPPORT;
#else
    config.hfp.feature_mask                 = WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                              WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                              WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                              WICED_BT_HFP_HF_FEATURE_HF_INDICATORS | \
                                              WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION | \
                                              WICED_BT_HFP_HF_FEATURE_ESCO_S4_SETTINGS_SUPPORT;
#endif

    config.nvram.link_key.id            = HEADSET_NVRAM_ID_LINK_KEYS;
    config.nvram.link_key.p_callback    = NULL;

    if(WICED_SUCCESS != bt_hs_spk_post_stack_init(&config))
    {
        WICED_BT_TRACE("bt_audio_post_stack_init failed\n");
        return WICED_BT_ERROR;
    }

    /*Set audio sink*/
#ifdef SPEAKER
    bt_hs_spk_set_audio_sink(AM_SPEAKERS);
    WICED_BT_TRACE("Default Application: Speaker\n");
#else
    bt_hs_spk_set_audio_sink(AM_HEADPHONES);
    WICED_BT_TRACE("Default Application: Headset\n");
#endif

#if (WICED_APP_LE_INCLUDED == TRUE)
    hci_control_le_enable();
#endif

    /*we will use the channel map provided by the phone*/
    ret = wiced_bt_dev_set_afh_channel_assessment(WICED_TRUE);
    WICED_BT_TRACE("wiced_bt_dev_set_afh_channel_assessment status:%d\n",ret);
    if (ret != WICED_BT_SUCCESS)
    {
        return WICED_BT_ERROR;
    }

#ifdef OTA_FW_UPGRADE
    if (!wiced_ota_fw_upgrade_init(NULL, NULL, NULL))
    {
        WICED_BT_TRACE("wiced_ota_fw_upgrade_init failed\n");
    }

    if (WICED_SUCCESS != ofu_spp_init())
    {
        WICED_BT_TRACE("ofu_spp_init failed\n");
        return WICED_ERROR;
    }
#endif

#ifdef AUTO_ELNA_SWITCH
    wiced_hal_rfm_auto_elna_enable(1, RX_PU);
#endif
#ifdef AUTO_EPA_SWITCH
    wiced_hal_rfm_auto_epa_enable(1, TX_PU);
#endif

    return WICED_SUCCESS;
}
#endif // HS_SPK_ENABLED

wiced_result_t lepl_btm_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t res = WICED_SUCCESS;
    wiced_bt_dev_ble_io_caps_req_t *p_ble_io_caps = &p_event_data->pairing_io_capabilities_ble_request;

    WICED_BT_TRACE("[%s] Received Event [%d] \n", __FUNCTION__, event);

    switch (event) {
        case BTM_ENABLED_EVT:
            lepl_handle_btm_enabled(&p_event_data->enabled);
#ifdef HS_SPK_ENABLED
#ifdef DISABLE_3M_PKT
            {
                uint8_t param_buf[1] = { 0x03 };
                wiced_bt_dev_vendor_specific_command(0xFDF9, sizeof(param_buf), param_buf, NULL);
            }
#endif // DISABLE_3M_PKT
            btheadset_post_bt_init();
            /*if (WICED_SUCCESS != btheadset_init_button_interface())
                WICED_BT_TRACE("btheadset button init failed\n");*/
            bt_hs_spk_button_set_discovery(WICED_TRUE);
            lepl_set_hfp_tdm_params();
            break;
        case BTM_SCO_CONNECTED_EVT:
        case BTM_SCO_CONNECTION_REQUEST_EVT:
        case BTM_SCO_CONNECTION_CHANGE_EVT:
        case BTM_SCO_DISCONNECTED_EVT:
            {
                hf_sco_management_callback(event, p_event_data);
            }
            break;
        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %B\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
            //            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_ALL_PROFILES_NO;
            break;
#endif // HS_SPK_ENABLED
        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: {
            p_ble_io_caps->local_io_cap = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            p_ble_io_caps->oob_data = BTM_OOB_NONE;
            p_ble_io_caps->auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
            p_ble_io_caps->max_key_size = 16;
            p_ble_io_caps->init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
            p_ble_io_caps->resp_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        } break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            res = lepl_btm_handle_encryption_sts(&p_event_data->encryption_status);
#ifdef HS_SPK_ENABLED
            {
                wiced_bt_dev_encryption_status_t *p_encryption_status = &p_event_data->encryption_status;

                WICED_BT_TRACE( "Encryption Status:(%B) res:%d\n", p_encryption_status->bd_addr, p_encryption_status->result );

                hci_control_send_encryption_changed_evt( p_encryption_status->result, p_encryption_status->bd_addr );
            }
#endif // HS_SPK_ENABLED
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            WICED_BT_TRACE("[%s] status %d\n",
                           __FUNCTION__,
                           p_event_data->pairing_complete.pairing_complete_info.ble.status);
#ifdef HS_SPK_ENABLED
            {
                wiced_bt_dev_pairing_cplt_t *p_pairing_cmpl = &p_event_data->pairing_complete;
                uint8_t pairing_result;
                if(p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
                {
                    pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
                }
                else
                {
                    pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
                }
                WICED_BT_TRACE( "Pairing Result: %d\n", pairing_result );
                hci_control_send_pairing_completed_evt( pairing_result, p_event_data->pairing_complete.bd_addr );
            }
#endif
            break;
        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        {
            lepl_clcb_t *p_clcb = lepl_gatt_get_clcb(p_event_data->paired_device_link_keys_request.bd_addr);
            if (p_clcb)
            {
                WICED_MEMCPY(&p_event_data->paired_device_link_keys_request.key_data,
                        &p_clcb->ltk.key_data,
                             sizeof(wiced_bt_device_sec_keys_t));
            }
            else if (!WICED_MEMCMP(br_edr_link_key.bd_addr,p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN))
            {
                WICED_MEMCPY(&p_event_data->paired_device_link_keys_request.key_data,
                                        &br_edr_link_key.key_data,
                                             sizeof(wiced_bt_device_sec_keys_t));
                res = WICED_SUCCESS;
                WICED_BT_TRACE("[%s] ltk %A",
                                           __FUNCTION__,
                                           &p_event_data->paired_device_link_keys_request,
                                           sizeof(wiced_bt_device_link_keys_t));
            }
        }
        break;
        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: {
            lepl_clcb_t *p_clcb = lepl_gatt_get_clcb(p_event_data->paired_device_link_keys_update.bd_addr);
            if (!p_clcb)
            {
                p_clcb = lepl_gatt_get_clcb(p_event_data->paired_device_link_keys_update.conn_addr);
            }
            WICED_BT_TRACE("[%s] p_clcb 0x%x ltk %A",
                           __FUNCTION__,
                           p_clcb,
                           &p_event_data->paired_device_link_keys_update,
                           sizeof(wiced_bt_device_link_keys_t));
            if (p_clcb)
            {
                WICED_MEMCPY(&p_clcb->ltk,
                             &p_event_data->paired_device_link_keys_update,
                             sizeof(wiced_bt_device_link_keys_t));
            }
            else
            {
                WICED_MEMCPY(&br_edr_link_key,
                             &p_event_data->paired_device_link_keys_update,
                             sizeof(wiced_bt_device_link_keys_t));
            }
        }
            break;
        case BTM_BLE_CONNECTION_PARAM_UPDATE:
        {
            wiced_bt_ble_connection_param_update_t *p_conn_param = &p_event_data->ble_connection_param_update;
            WICED_BT_TRACE("[%s] status %d bd_addr %B conn_interval %d conn_latency %d timeout %d",
                           __FUNCTION__,
                           p_conn_param->status,
                           p_conn_param->bd_addr,
                           p_conn_param->conn_interval,
                           p_conn_param->conn_latency,
                           p_conn_param->supervision_timeout);

            lepl_clcb_t *p_clcb = lepl_gatt_get_clcb(p_conn_param->bd_addr);

            if(p_clcb){
		p_clcb->conn_interval = p_conn_param->conn_interval;
            }

#if 0
			uint8_t security_flags = 0;
			uint8_t le_key_size = 0;
			wiced_bool_t bsec =
					wiced_bt_ble_get_security_state(p_conn_param->bd_addr, &security_flags, &le_key_size);

			WICED_BT_TRACE("[%s] %B %d 0x%x %d",__FUNCTION__, p_conn_param->bd_addr,
					bsec, security_flags, le_key_size);

			if(le_key_size)
		{
                if ((p_conn_param->conn_interval < lepl_cfg_settings.p_ble_cfg->p_ble_scan_cfg->conn_min_interval) ||
					p_conn_param->conn_latency != 0)
			{
                    lepl_update_conn_param(p_conn_param->bd_addr);
			}
		}
#endif
        }break;

        case BTM_BLE_DEVICE_ADDRESS_UPDATE_EVENT:
            if (p_event_data->ble_addr_update_event.status == WICED_SUCCESS)
            {
                WICED_MEMCPY(g_lepl_gatt_cb.own_addr, p_event_data->ble_addr_update_event.bdaddr, BD_ADDR_LEN);
            }
            break;


        default:
            res = WICED_ERROR;
            break;
    }

    return res;
}

void lepl_update_conn_param(wiced_bt_device_address_t bd_addr)
{
    wiced_bt_ble_conn_params_t conn_param;
    const wiced_bt_cfg_ble_scan_settings_t *p_scan_cfg = lepl_cfg_settings.p_ble_cfg->p_ble_scan_cfg;
    wiced_bt_ble_get_connection_parameters(bd_addr, &conn_param);
    if ((conn_param.conn_interval < p_scan_cfg->conn_min_interval) ||
        (conn_param.conn_interval > p_scan_cfg->conn_max_interval))
    {
        wiced_bool_t ret;
#if 0
        ret = wiced_bt_l2cap_enable_update_ble_conn_params(bd_addr, WICED_TRUE);
        WICED_BT_TRACE("[%s] enable_update_ble_conn_params ret %B %d", __FUNCTION__, bd_addr, ret);
#endif
        wiced_bt_ble_pref_conn_params_t pref_conn_param = {.conn_interval_max = p_scan_cfg->conn_max_interval,
                                                           .conn_interval_min = p_scan_cfg->conn_min_interval,
                                                           .conn_latency = p_scan_cfg->conn_latency,
                                                           .conn_supervision_timeout =
                                                               p_scan_cfg->conn_supervision_timeout,
                                                           .max_ce_length = 0,
                                                           .min_ce_length = 0};

        ret = wiced_bt_l2cap_update_ble_conn_params(bd_addr, &pref_conn_param);
        WICED_BT_TRACE("[%s] update_ble_conn_params ret %d", __FUNCTION__, ret);
    }
}
