/*
 * $ Copyright Cypress Semiconductor $
 */

/* Application includes */
#include "lepl.h"
#ifdef HS_SPK_ENABLED
#include "bt_hs_spk_handsfree.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_sdp_defs.h"
#endif // HS_SPK_ENABLED

#ifndef LINUX_PLATFORM
#if defined(WIN32) && defined(CLI_SUPPORT)
#include "windows.h"
static DWORD __stdcall cmdLineThread(VOID* p);
#endif

wiced_bt_heap_t *p_unicast_heap = NULL;

wiced_bt_cfg_ble_scan_settings_t lepl_scan_settings = {
    .scan_mode = BTM_BLE_SCAN_MODE_ACTIVE,

    /* Advertisement scan configuration */
    .high_duty_scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,
    .high_duty_scan_window = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,
    .high_duty_scan_duration = 5,

    .low_duty_scan_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_INTERVAL,
    .low_duty_scan_window = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_WINDOW,
    .low_duty_scan_duration = 5,

    /* Connection scan configuration */
    .high_duty_conn_scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL,
    .high_duty_conn_scan_window = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_WINDOW,
    .high_duty_conn_duration = 30,

    .low_duty_conn_scan_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_INTERVAL,
    .low_duty_conn_scan_window = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_WINDOW,
    .low_duty_conn_duration = 30,

    /* Connection configuration */
#ifdef HS_SPK_ENABLED
    // Set Connection interval multiple of 7.5 & 10msec ISOC interval to align the timing.
    .conn_min_interval = 192, // 240 msec
    .conn_max_interval = 192, // 240 msec
#else
    .conn_min_interval = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,
    .conn_max_interval = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,
#endif // HS_SPK_ENABLED
    .conn_latency = WICED_BT_CFG_DEFAULT_CONN_LATENCY,
    .conn_supervision_timeout = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,
};

const wiced_bt_cfg_ble_advert_settings_t lepl_adv_settings = {
    .channel_map = BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39,

    .high_duty_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,
    .high_duty_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,
    .high_duty_duration = 30,

    .low_duty_min_interval = 1024,
    .low_duty_max_interval = 1024,
    .low_duty_duration = 60,

    .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,
    .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,

    .low_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,
    .low_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,
    .low_duty_directed_duration = 30,

    .high_duty_nonconn_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,
    .high_duty_nonconn_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,
    .high_duty_nonconn_duration = 30,

    .low_duty_nonconn_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,
    .low_duty_nonconn_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,
    .low_duty_nonconn_duration = 0};

wiced_bt_cfg_ble_t lepl_ble_cfg = {
    .ble_max_simultaneous_links = 4,
    .ble_max_rx_pdu_size = 512,

    .p_ble_scan_cfg = &lepl_scan_settings,
    .p_ble_advert_cfg = &lepl_adv_settings,
    .appearance = APPEARANCE_GENERIC_TAG,

    .host_addr_resolution_db_size = 5,
    .rpa_refresh_timeout = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,
};

wiced_bt_cfg_gatt_t lepl_gatt_cfg = {
    .max_db_service_modules = 0,
    .max_eatt_bearers = 0,
};

#ifdef HS_SPK_ENABLED
/* L2CAP Setting */
const wiced_bt_cfg_l2cap_application_t lehs_l2cap_app = /* Application managed l2cap protocol configuration */
{
    /* BR EDR l2cap configuration */
    .max_app_l2cap_psms = 7,      /**< Maximum number of application-managed BR/EDR PSMs */
    .max_app_l2cap_channels = 7, /**< Maximum number of application-managed BR/EDR channels  */

    .max_app_l2cap_br_edr_ertm_chnls = 0,  /**< Maximum ERTM channels allowed */
    .max_app_l2cap_br_edr_ertm_tx_win = 0, /**< Maximum ERTM TX Window allowed */
                            /* LE L2cap connection-oriented channels configuration */
    .max_app_l2cap_le_fixed_channels = 0,
};
/* BR Setting */
const wiced_bt_cfg_br_t lehs_br =
{
    .br_max_simultaneous_links = 3,
    .br_max_rx_pdu_size = 1024,
    .device_class = {0x24, 0x04, 0x18},                     /**< Local device class */

    .rfcomm_cfg = /* RFCOMM configuration */
    {
        .max_links = WICED_BT_HFP_HF_MAX_CONN, /**< Maximum number of simultaneous connected remote devices. Should be less than or equal to l2cap_application_max_links */
        .max_ports = WICED_BT_HFP_HF_MAX_CONN, /**< Maximum number of simultaneous RFCOMM ports */
    },
    .avdt_cfg = /* Audio/Video Distribution configuration */
    {
        .max_links = WICED_BT_A2DP_SINK_MAX_NUM_CONN, /**< Maximum simultaneous audio/video links */
        .max_seps = WICED_BT_A2DP_SINK_MAX_NUM_CONN * WICED_BT_A2DP_SINK_MAX_NUM_CODECS,  /**< Maximum number of stream end points */
    },

    .avrc_cfg = /* Audio/Video Remote Control configuration */
    {
        .max_links = MAX_CONNECTED_RCC_DEVICES, /**< Maximum simultaneous remote control links */
    },
};

#define BT_AUDIO_A2DP_SBC_MAX_BITPOOL   35

/* Array of decoder capabilities information. */
wiced_bt_a2dp_codec_info_t bt_audio_codec_capabilities[] =
{
    {
        .codec_id = WICED_BT_A2DP_CODEC_SBC,
        .cie =
        {
            .sbc =
            {
#ifdef ENABLE_PTS_TESTING
                (A2D_SBC_IE_SAMP_FREQ_44 | A2D_SBC_IE_SAMP_FREQ_48 | A2D_SBC_IE_SAMP_FREQ_16 | A2D_SBC_IE_SAMP_FREQ_32),    /* samp_freq */
#else
                (A2D_SBC_IE_SAMP_FREQ_44 | A2D_SBC_IE_SAMP_FREQ_48),    /* samp_freq */
#endif
                (A2D_SBC_IE_CH_MD_MONO   | A2D_SBC_IE_CH_MD_STEREO |
                 A2D_SBC_IE_CH_MD_JOINT  | A2D_SBC_IE_CH_MD_DUAL),      /* ch_mode */
                (A2D_SBC_IE_BLOCKS_16    | A2D_SBC_IE_BLOCKS_12 |
                 A2D_SBC_IE_BLOCKS_8     | A2D_SBC_IE_BLOCKS_4),        /* block_len */
                (A2D_SBC_IE_SUBBAND_4    | A2D_SBC_IE_SUBBAND_8),       /* num_subbands */
                (A2D_SBC_IE_ALLOC_MD_L   | A2D_SBC_IE_ALLOC_MD_S),      /* alloc_mthd */
                BT_AUDIO_A2DP_SBC_MAX_BITPOOL,                          /* max_bitpool for high quality audio */
                A2D_SBC_IE_MIN_BITPOOL                                  /* min_bitpool */
            }
        }
#ifdef A2DP_SINK_AAC_ENABLED
    },

    {
        .codec_id = WICED_BT_A2DP_CODEC_M24,
        .cie =
        {
                .m24 =
                {
                    (A2D_M24_IE_OBJ_MSK),                                   /* obj_type */
                    (A2D_M24_IE_SAMP_FREQ_44 | A2D_M24_IE_SAMP_FREQ_48),    /*samp_freq */
                    (A2D_M24_IE_CHNL_MSK),                                  /* chnl */
                    (A2D_M24_IE_VBR_MSK),                                   /* b7: VBR */
                    (A2D_M24_IE_BITRATE_MSK)                                /* bitrate - b7-b0 of octect 3, all of octect4, 5*/
                }
        }
#endif
    }
};

/** A2DP sink configuration data */
wiced_bt_a2dp_config_data_t bt_audio_config =
{
    .feature_mask = WICED_BT_A2DP_SINK_FEAT_DELAY_RPT,                                  /* feature mask */
    .codec_capabilities =
    {
        .count = sizeof_array(bt_audio_codec_capabilities),
        .info  = bt_audio_codec_capabilities,                   /* codec configuration */
    },
    .p_param =
    {
        .buf_depth_ms                   = 300,                                          /* in msec */
        .start_buf_depth                = 50,                                           /* start playback percentage of the buffer depth */
        .target_buf_depth               = 50,                                           /* target level percentage of the buffer depth */
        .overrun_control                = WICED_BT_A2DP_SINK_OVERRUN_CONTROL_FLUSH_DATA,/* overrun flow control flag */
        .adj_ppm_max                    = +300,                                         /* Max PPM adjustment value */
        .adj_ppm_min                    = -300,                                         /* Min PPM adjustment value */
        .adj_ppb_per_msec               = 200,                                          /* PPM adjustment per milli second */
        .lvl_correction_threshold_high  = +2000,                                        /* Level correction threshold high value */
        .lvl_correction_threshold_low   = -2000,                                        /* Level correction threshold low value */
        .adj_proportional_gain          = 20,                                           /* Proportional component of total PPM adjustment */
        .adj_integral_gain              = 2,                                            /* Integral component of total PPM adjustment */
    },
    .ext_codec =
    {
#ifdef A2DP_SINK_AAC_ENABLED
        .codec_id        = WICED_AUDIO_CODEC_AAC_DEC,
#else
        .codec_id        = WICED_AUDIO_CODEC_NONE,
        .codec_functions = NULL,
#endif
    }
};
/** AVRC CT supported events. */
uint8_t bt_avrc_ct_supported_events[] =
{
    FALSE,
    TRUE,            /* AVRC_EVT_PLAY_STATUS_CHANGE             0x01    Playback Status Changed */
    TRUE,            /* AVRC_EVT_TRACK_CHANGE                   0x02    Track Changed */
    TRUE,            /* AVRC_EVT_TRACK_REACHED_END              0x03    Track End Reached */
    TRUE,            /* AVRC_EVT_TRACK_REACHED_START            0x04    Track Reached Start */
    TRUE,            /* AVRC_EVT_PLAY_POS_CHANGED               0x05    Playback position changed */
    FALSE,           /* AVRC_EVT_BATTERY_STATUS_CHANGE          0x06    Battery status changed */
    FALSE,           /* AVRC_EVT_SYSTEM_STATUS_CHANGE           0x07    System status changed */
    TRUE,            /* AVRC_EVT_APP_SETTING_CHANGE             0x08    Player application settings changed */
    FALSE,           /* AVRC_EVT_NOW_PLAYING_CHANGE             0x09    Now Playing Content Changed (AVRCP 1.4) */
    FALSE,           /* AVRC_EVT_AVAL_PLAYERS_CHANGE            0x0a    Available Players Changed Notification (AVRCP 1.4) */
    FALSE,           /* AVRC_EVT_ADDR_PLAYER_CHANGE             0x0b    Addressed Player Changed Notification (AVRCP 1.4) */
    FALSE,           /* AVRC_EVT_UIDS_CHANGE                    0x0c    UIDs Changed Notification (AVRCP 1.4) */
    TRUE             /* AVRC_EVT_VOLUME_CHANGE                  0x0d    Notify Volume Change (AVRCP 1.4) */
};
/*****************************************************************************
 * SDP database for the hci_control application
 ****************************************************************************/
// SDP Record handle for AVDT Sink
#define HANDLE_AVDT_SINK                        0x10001
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_TARGET                      0x10002
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_CONTROLLER                  0x10003
// SDP Record handle for SPP OFU
#define HANDLE_OFU_SPP                          0x10005

#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
#define WICED_APP_CFG_SDP_HFP_FEATURE   (WICED_BT_HFP_HF_SDP_FEATURE_3WAY_CALLING | \
                                         WICED_BT_HFP_HF_SDP_FEATURE_CLIP | \
                                         WICED_BT_HFP_HF_SDP_FEATURE_REMOTE_VOL_CTRL | \
                                         WICED_BT_HFP_HF_SDP_FEATURE_WIDEBAND_SPEECH)
#else
#define WICED_APP_CFG_SDP_HFP_FEATURE   (WICED_BT_HFP_HF_SDP_FEATURE_3WAY_CALLING | \
                                         WICED_BT_HFP_HF_SDP_FEATURE_CLIP | \
                                         WICED_BT_HFP_HF_SDP_FEATURE_REMOTE_VOL_CTRL)
#endif

const uint8_t btheadset_sdp_db[] =
{
    SDP_ATTR_SEQUENCE_2(
                            77 + 2                // A2DP Sink       ==> 77 + 2
                          + 56 + 2                // AVRC Target     ==> 56 + 2
                          + 59 + 2                // AVRC Controller ==> 59 + 2
                          + 75 + 2                // Handsfree       ==> 75 + 2
                          + 69 + 2                // SPP OFU         ==> 69 + 2
                       ),

    // SDP Record for A2DP Sink
    SDP_ATTR_SEQUENCE_1(77),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVDT_SINK),
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_AUDIO_SINK),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST),
            SDP_ATTR_SEQUENCE_1(16),
                SDP_ATTR_SEQUENCE_1(6),
                    SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                    SDP_ATTR_VALUE_UINT2(BT_PSM_AVDTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVDTP),
                SDP_ATTR_VALUE_UINT2(AVDT_VERSION_1_3),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST),
            SDP_ATTR_SEQUENCE_1(8),
                SDP_ATTR_SEQUENCE_1(6),
                    SDP_ATTR_UUID16(UUID_SERVCLASS_ADV_AUDIO_DISTRIBUTION),
                    SDP_ATTR_VALUE_UINT2(AVDT_VERSION_1_3),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x000B),
        SDP_ATTR_SERVICE_NAME(16),
        'W', 'I', 'C', 'E', 'D', ' ', 'A', 'u', 'd', 'i', 'o', ' ', 'S', 'i', 'n', 'k',

    // SDP Record for AVRC Target
    SDP_ATTR_SEQUENCE_1(56),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_TARGET),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(3),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_TARGET),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x0104),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_5),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_TG_CAT2),

    // SDP Record for AVRC Controller
    SDP_ATTR_SEQUENCE_1(59),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_CONTROLLER),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_CONTROL),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x104),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_3),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_CT_CAT1),

        // SDP Record for Hands-Free Unit
            SDP_ATTR_SEQUENCE_1(75),
                SDP_ATTR_RECORD_HANDLE(WICED_HANDSFREE_HDLR_UNIT),
                SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST), SDP_ATTR_SEQUENCE_1(6),
                    SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
                    SDP_ATTR_UUID16(UUID_SERVCLASS_GENERIC_AUDIO),
                SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(WICED_HANDSFREE_SCN),
                SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
                    SDP_ATTR_SEQUENCE_1(6),
                        SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
                        SDP_ATTR_VALUE_UINT2(0x0107),
                SDP_ATTR_SERVICE_NAME(15),
                    'W', 'I', 'C', 'E', 'D', ' ', 'H', 'F', ' ', 'D', 'E', 'V', 'I', 'C', 'E',
                SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, WICED_APP_CFG_SDP_HFP_FEATURE),

    // SDP Record for SPP OFU
    SDP_ATTR_SEQUENCE_1(69),                                            // 2 bytes
        SDP_ATTR_RECORD_HANDLE(HANDLE_OFU_SPP),                         // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_SERIAL_PORT),                  // 8
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(OFU_SPP_RFCOMM_SCN),         // 17 bytes
        SDP_ATTR_BROWSE_LIST,                                           // 8
        SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_SERIAL_PORT, 0x0102), // 13 byte
        SDP_ATTR_SERVICE_NAME(10),                                      // 15
        'S', 'P', 'P', ' ', 'S', 'E', 'R', 'V', 'E', 'R',
};
#endif // HS_SPK_ENABLED
wiced_bt_cfg_settings_t lepl_cfg_settings = {.device_name = (uint8_t *)"Airoc Player",
                                                       .p_ble_cfg = &lepl_ble_cfg,
                                                       .p_gatt_cfg = &lepl_gatt_cfg,
#ifdef HS_SPK_ENABLED
                                                       .security_required = BTM_SEC_BEST_EFFORT, /**< Security requirements mask */
                                                       .p_br_cfg = &lehs_br,
                                                       .p_l2cap_app_cfg = &lehs_l2cap_app
#endif
};

int spy_inst = 0;
static uint8_t local_bda[BD_ADDR_LEN] = {0};

int get_spy_instance(void);
void set_local_bd_addr(void);

#define BT_STACK_HEAP_SIZE (12 * 1024)

#ifdef HS_SPK_ENABLED
uint16_t wiced_app_cfg_sdp_record_get_size(void)
{
    return (uint16_t)sizeof(btheadset_sdp_db);
}
#endif
void APPLICATION_START(void)
{
    /* RPC to work with LE Audio Client Control */
    lepl_rpc_init(get_spy_instance());

    /* Register call back and configuration with stack */
    wiced_bt_stack_init(lepl_btm_cback, &lepl_cfg_settings);

    /* Create a buffer heap, make it the default heap.  */
    p_unicast_heap = wiced_bt_create_heap((char *)lepl_cfg_settings.device_name, NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
    //iso_audio_init(&lepl_isoc_cfg);
#if defined(WIN32) && defined(CLI_SUPPORT)
    {
        DWORD thread_address;
        CreateThread(0, 0, cmdLineThread, 0, 0, &thread_address);
    }
#endif
}

void set_local_bd_addr(void)
{
    if ((local_bda[0] | local_bda[1] | local_bda[2] | local_bda[3] | local_bda[4] | local_bda[5]) != 0)
        wiced_bt_set_local_bdaddr(local_bda, BLE_ADDR_PUBLIC);
}


#ifdef __linux__
#include "platform_linux.h"
/* App Library includes */
#include "arg_parser.h"

int main(int argc, char *argv[])
{

    arg_parser_arguments_t parsed_args = { 0 };
    cybt_controller_autobaud_config_t parse_autobaud_cfg;

    if (-1 == arg_parser_get_args(argc, argv, &parsed_args))
        return -1;

    spy_inst = parsed_args.spy_inst;
    memcpy(local_bda, parsed_args.local_bda, sizeof(local_bda));

    /* Initialize Spy TCP/UDP Sockets */
    cy_bt_spy_comm_init(parsed_args.is_socket_tcp, parsed_args.spy_inst, NULL);

    /* Initialize BT HCI */
    cy_bt_hci_init(parsed_args.com_port, parsed_args.baud_rate, NULL);


    WICED_BT_TRACE("[%s] COM [%s] Baud [%d] Instance [%d]\n", __FUNCTION__, parsed_args.com_port, parsed_args.baud_rate, parsed_args.spy_inst);

    APPLICATION_START();

    for (;;) 
    {
#ifdef CLI_SUPPORT
        extern int le_pl_cli_routine(void);
        if (le_pl_cli_routine() == 0)
        {
            break;
        }
#endif // CLI_SUPPORT
    }

    return 0;
}

int get_spy_instance(void)
{
    return spy_inst;
}

#elif defined WIN32
#ifdef CLI_SUPPORT
static DWORD __stdcall cmdLineThread(VOID *p)
{
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    while (1)
    {
        extern int le_pl_cli_routine(void);
        if (le_pl_cli_routine() == 0)
        {
            break;
        }
    }
    return (0);
}
#endif // CLI_SUPPORT
extern int wicedx_emulator_instance;

int get_spy_instance(void)
{
    return wicedx_emulator_instance;
}

#else
int get_spy_instance(void)
{
    return spy_inst;
}

int main(void)
{
    APPLICATION_START();

    return 0;
}

#endif

wiced_bt_cfg_settings_t *app_get_cfg_settings(void)
{
    return &lepl_cfg_settings;
}

void app_set_connection_options(wiced_ble_ext_adv_phy_mask_t mask, wiced_ble_ext_conn_cfg_phy_options_t *p_out)
{
    const wiced_bt_cfg_ble_scan_settings_t *p_bsc = app_get_cfg_settings()->p_ble_cfg->p_ble_scan_cfg;

    {
        p_out->scan_int = p_bsc->high_duty_scan_interval;
        p_out->scan_window = p_bsc->high_duty_conn_scan_window;
        p_out->min_conn_int = p_bsc->conn_min_interval;
        p_out->max_conn_int = p_bsc->conn_max_interval;
        p_out->conn_latency = p_bsc->conn_latency;
        p_out->supervision_to = p_bsc->conn_supervision_timeout;
        p_out->min_ce_len = 0;
        p_out->max_ce_len = 0;
    }
}

wiced_result_t app_create_connection(uint8_t addr_type, wiced_bt_device_address_t bd_addr)
{
    wiced_result_t status;
    wiced_ble_ext_conn_cfg_t conn_cfg = {
        .adv_handle = 0xff,
        .sub_event = 0xff,
        .init_filter_policy = 0,
        .own_addr_type = BLE_ADDR_PUBLIC,  /**< initiator address type */
        .peer_addr_type = BLE_ADDR_PUBLIC, /**< peer address type */
        .peer_addr = {0},
        .initiating_phys = WICED_BLE_EXT_ADV_PHY_1M_BIT,
        .timeout_secs = 30,
    };

    /* write address */
    conn_cfg.peer_addr_type = addr_type;
    WICED_MEMCPY(conn_cfg.peer_addr, bd_addr, BD_ADDR_LEN);

    /* write phy options */
    app_set_connection_options(WICED_BLE_EXT_ADV_PHY_1M_BIT, &conn_cfg.phy_options[0]);

    status = wiced_ble_ext_create_connection(&conn_cfg);
    return status;
}

#endif
