/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __LEPL_H__
#define __LEPL_H__

#include "wiced_bt_types.h"
#include "wiced_memory.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_timer.h"

/* App Library includes */
#include "le_audio_cap.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_mcs.h"
#include "wiced_bt_ga_csis.h"
#include "wiced_bt_ga_csip.h"
#include "wiced_bt_ga_tbs.h"
#include "wiced_bt_ga_mics.h"
#include "wiced_bt_ga_has.h"
#include "csis_psri_key.h"
#include "le_audio_rpc.h"
#include "audio_driver.h"
#include <stdlib.h>
#include "gatt_interface.h"

#define ARIP_MXTDM_PAIR                       MXTDM_PAIR_1 // 0 - MXTDM0, 1 - MXTDM1
#define ARIP_MXTDM_MODE                       MXTDM_MODE_TDM // 0 - MXTDM_TDM, 1 - MXTDM_I2S
#define ARIP_MXTDM_ROLE                       MXTDM_BUS_SLAVE // 0 - Slave, 1 - Master
#define ARIP_MXTDM_TDM_CH_NUM                 4 // Total number of slots/channels, minimum - 2
#define ARIP_MXTDM_A2DP_LEFT_CH               0
#define ARIP_MXTDM_A2DP_RIGHT_CH              1
#define ARIP_MXTDM_HFP_CH                     2 // Bi-direction on same channel
#define ARIP_MXTDM_LE_LEFT_CH                 0
#define ARIP_MXTDM_LE_RIGHT_CH                1

// For A2DP + LE Case set to 10
// For A2DP + HFP Case set to 6
#define LE_CIG_SYNC_DELAY                     10

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
#define ARIP_MXTDM_TDM_1                      0xC0 //  6 << 5 TDM1 peripheral in TDM mode, encoded as shown above

#define ARIP_MXTDM_LE_AUDIO_CH_L_DATA_PATH_ID (ARIP_MXTDM_TDM_1 | ARIP_MXTDM_LE_LEFT_CH) //Interface -6, channel - 0 1100 0000
#define ARIP_MXTDM_LE_AUDIO_CH_R_DATA_PATH_ID (ARIP_MXTDM_TDM_1 | ARIP_MXTDM_LE_RIGHT_CH) //Interface -6, channel - 1 1100 0001

#define ARIP_MXTDM_TDM_TX_CH_MAP              0x0F
#define ARIP_MXTDM_TDM_RX_CH_MAP              0x0F

#define HAP_ENABLED 1

#define MAX_CONNECTION_INSTANCE 2
#define MAX_PACS_SOURCE_CAP_SUPPORTED 5
#define MAX_PACS_SINK_CAP_SUPPORTED 5
#define MAX_SINK_ASE_SUPPORTED 5
#define MAX_SOURCE_ASE_SUPPORTED 5
#define INVALID_ASE_ID 55

#define MAX_URI_LEN 50
#define MAX_FRIENDLY_NAME_LEN 50

#define IS_JOIN_SUPPORTED(x) x &WICED_BT_GA_TBS_FEATURE_BIT_JOIN
#define IS_HOLD_SUPPORTED(x) x &WICED_BT_GA_TBS_FEATURE_BIT_LOCAL_HOLD
#define IS_INBAND_RINGTONE_SUPPORTED(x) &WICED_BT_GA_TBS_FEATURE_BIT_INBAND_RINGTONE
#define IS_SILENT_MODE_SUPPORTED(x) x &WICED_BT_GA_TBS_FEATURE_BIT_SILENT_MODE

#define MAX_CIS_CONN 2
#define LEPL_MAX_SDU_SIZE 240
#define MAX_MICS_AICS 2
#define MAX_DESCRIPTION 20
#define MAX_PRESET_RECORDS 5


#define HFP_CONNECTED       (0)
#define A2DP_CONNECTED      (1)
#define AVRCP_CONNECTED     (2)
#define ALL_CONNECTED       ((1 << HFP_CONNECTED) | (1 << A2DP_CONNECTED) | (1 << AVRCP_CONNECTED))

typedef enum
{
    LEPL_GATT_STATE_CONNECTED,
    LEPL_GATT_STATE_MTU_CONFIGURED,
    LEPL_GATT_STATE_DISCOVERY_COMPLETE,
    LEPL_GATT_STATE_INITING,
    LEPL_GATT_STATE_READY,
    LEPL_GATT_STATE_DISCONNECTING,
    LEPL_GATT_STATE_DISCONNECTED
} lepl_gatt_state_t;

typedef enum
{
    LEPL_APP_STATE_IDLE,
    LEPL_APP_STATE_MEDIA,
    LEPL_APP_STATE_CALL,
    LEPL_APP_STATE_MIC,
    LEPL_APP_STATE_IN_TRANSIT
} lepl_app_state_t;

typedef enum
{
    INCOMING_CALL = 0,
    OUTGOING_CALL = 1
} lepl_tbs_call_type_t;

typedef enum
{
    TBS_ACTION_CALL_ACCEPTED = 0x00,          /**< Accept the call locally. */
    TBS_ACTION_CALL_TERMINATED = 0x01,        /**< Terminate the call */
    TBS_ACTION_CALL_HELD = 0x02,              /**< Locally hold the call */
    TBS_ACTION_CALL_RETRIEVED = 0x03,         /**< Retrieve hold the call */
    TBS_ACTION_CALL_PLACED = 0x04,            /**< Place the call */
    TBS_ACTION_CALL_JOINED = 0x05,            /**< Join multiple calls */
    TBS_ACTION_CALL_REMOTELY_HELD = 0x06,     /**< Remotely hold the call */
    TBS_ACTION_CALL_REM_REMOTELY_HELD = 0x07, /**< Remove Remotely hold the call */
    TBS_ACTION_CALL_ALERT = 0x08,             /**< Set the call to Alerting state */
} lepl_tbs_action_t;

typedef enum
{
    CALL_CONTROL_SERVER_STATE_IDLE,
    CALL_CONTROL_SERVER_STATE_INBAND_RINGTONE,
    CALL_CONTROL_SERVER_STATE_INBAND_RINGTONE_CONVO,
    CALL_CONTROL_SERVER_STATE_CONVO
} lepl_ccs_states_t;

enum
{
    OFU_SPP_RFCOMM_PORT_COUNT = 1,
    OFU_SPP_RFCOMM_SCN = 2,
};

typedef struct
{
    lepl_app_state_t initial_state;
    lepl_app_state_t final_state;
} lepl_app_state_transit_info_t;

typedef struct
{
    lepl_app_state_t current_state;
    lepl_app_state_t paused_state;
    lepl_app_state_transit_info_t transit_info;
    uint32_t paused_strm_codec;
    uint32_t current_strm_codec;
} lepl_app_state_info_t;


typedef struct
{
    uint16_t acl_conn_handle; /**< ACL connection handle */
    uint16_t cis_conn_handle; /**< CIS connection handle */

    uint8_t lc3_index;                  // lc3 codec index ?  What is this one exactly ?
    uint32_t data_path_established : 1; // data path established ?
    wiced_bt_ga_ascs_ase_t data;
} lepl_ase_data_t;

typedef struct
{
    wiced_bt_ga_vcs_volume_state_t volume_state; /**< volume information */
    wiced_bt_ga_volume_flag_val_t volume_flag;   /**< volume persistence flag */
} lepl_vcs_data_t;

typedef struct
{
    char media_player_name[MAX_MEDIA_PLAYER_NAME_LEN];       /**< Media Player Name */
    char track_title[MAX_MEDIA_TRACK_TITLE_LEN];             /**< Track Title */
    int32_t track_duration;                                  /**< Track Duration */
    int32_t track_position;                                  /**< Track Position */
    int8_t playback_speed;                                   /**< Playback Speed */
    int8_t seeking_speed;                                    /**< Seeking Speed */
    wiced_bt_ga_media_control_playing_order_t playing_order; /**< Playing Order */
    uint16_t playing_order_supported;                        /**< Playing Order Supported bit field */
    wiced_bt_ga_media_control_state_t media_state;           /**< Media State */
    uint32_t media_control_supported_opcodes;                /**< Media Control Supported Opcodes */
    uint8_t content_control_id;                              /**< Content Control ID */
} lepl_mcs_data_t;

typedef struct {
    wiced_bt_ga_csis_sirk_data_t sirk_data; /**< set identity resolving key */
    uint8_t size;                           /**< number of devices in the coordinated set */
    uint8_t rank;                           /**< rank of the device in the coordinated set */
} lepl_csis_data_t;

typedef struct
{
    wiced_bool_t in_use;
    lepl_tbs_call_type_t call_type;
    uint8_t call_id;
    wiced_bt_ga_tbs_call_state_t call_state; // enum
    uint8_t call_flags;
    char URI[WICED_BT_GA_TBS_RM_CALLERID_MAX_SIZE]; // variable length
} tbs_call_state_data_t;

typedef struct
{
    uint8_t call_id;                                   /**< Call id of the call*/
    char name[WICED_BT_GA_TBS_FRIENDLY_NAME_MAX_SIZE]; /**< remote caller id consisting of uri prefix and caller id*/
} tbs_call_friendly_name_data_t;

/* telephone bearer service data */
typedef struct
{
    uint8_t current_call_id;
    uint8_t num_calls;
    uint8_t is_caller_id_invalid_enabled;
    uint8_t content_control_id;
    char bearer_provider_name[WICED_BT_GA_TBS_BEARER_NAME_MAX_SIZE];
    char bearer_UCI[WICED_BT_GA_TBS_BEARER_UCI_MAX_SIZE];
    char bearer_URI[WICED_BT_GA_TBS_BEARER_URI_MAX_SIZE];
    uint8_t bearer_technology;
    uint8_t bearer_signal_strength;
    uint8_t prev_bearer_signal_strength;
    uint8_t bearer_signal_strength_reporting_interval;
    uint16_t bearer_status_flag;
    uint16_t ccp_supported_opcode;
    tbs_call_friendly_name_data_t call_friendly_name;
    uint8_t latest_incoming_remote_call_id;
    tbs_call_state_data_t call_state_data[WICED_BT_GA_TBS_BEARER_MAX_CALL_COUNT];
    char incoming_tg_caller_id[WICED_BT_GA_TBS_TG_CALLERID_MAX_SIZE];
} lepl_tbs_data_t;

typedef struct
{
    wiced_bt_ga_aics_input_state_t input_state;           /**< Audio Input State */
    wiced_bt_ga_aics_gain_settings_params_t gain_setting; /**< Audio Input Gain setting */
    wiced_bt_ga_aics_input_type_t input_type;             /**< Audio Input Type */
    wiced_bt_ga_aics_input_status_t input_status;         /**< Audio Input Status */
    char description[MAX_DESCRIPTION];                    /**< Audio Input descritpion */
    uint8_t description_len;
} lepl_aics_data_t;

typedef struct
{
    uint8_t mute_state;
    lepl_aics_data_t aics[MAX_MICS_AICS];
} lepl_mics_data_t;

typedef struct
{
    uint8_t preset_index;
    uint8_t properties;
    uint8_t name_len;
    char name[HAS_MAX_PRESET_RECORD_NAME_LENGTH + 1];
}lepl_has_preset_rec_t;

typedef struct
{
    uint8_t num_rec;
    uint8_t hearing_aid_features;
    uint8_t active_preset_index;
    lepl_has_preset_rec_t preset_rec_list[MAX_PRESET_RECORDS];
} lepl_has_data_t;

typedef struct
{
    gatt_intf_service_object_t *p_mcs;
    gatt_intf_service_object_t *p_gmcs;
    gatt_intf_service_object_t *p_tbs;
    gatt_intf_service_object_t *p_gtbs;
} lepl_local_profiles_t;

typedef struct
{
    gatt_intf_service_object_t *p_pacs;
    gatt_intf_service_object_t *p_ascs;
    gatt_intf_service_object_t *p_vcs;
    gatt_intf_service_object_t *p_csis;
    gatt_intf_service_object_t *p_mics;
    gatt_intf_service_object_t *p_has;
    gatt_intf_service_object_t *p_mics_aics[MAX_MICS_AICS];
    uint8_t num_mics_aics;
} lepl_peer_profiles_t;

typedef struct
{
    uint8_t in_use;
    uint16_t conn_id;
    uint32_t addr_type;
    wiced_bt_device_address_t bda;
    wiced_bt_device_link_keys_t ltk;
    wiced_bool_t b_is_central;
    lepl_gatt_state_t app_state;
    lepl_peer_profiles_t peer_profiles;
    le_audio_cap_device_data_t *p_cap;
    wiced_bt_ga_pacs_record_t pacs_src_record[MAX_PACS_SOURCE_CAP_SUPPORTED];
    wiced_bt_ga_pacs_record_t pacs_sink_record[MAX_PACS_SINK_CAP_SUPPORTED];
    wiced_bt_ga_pacs_data_t pacs_data;
    wiced_bt_ga_vcs_volume_state_t vcs_data;
    lepl_csis_data_t csis_data;
    lepl_mics_data_t mics_data;
    lepl_has_data_t has_data;
    lepl_ase_data_t *p_remote_ase_data;
    int num_remote_ases;
    uint16_t conn_interval;
} lepl_clcb_t;

typedef struct
{
    wiced_bt_device_address_t own_addr;
    lepl_local_profiles_t local_profiles;
    lepl_clcb_t unicast_clcb[MAX_CONNECTION_INSTANCE];
    le_audio_cap_device_data_t cap_data[MAX_CONNECTION_INSTANCE];
    le_audio_cap_app_data_t cap_profile_data;
    lepl_mcs_data_t mcs_data;
    lepl_tbs_data_t tbs_data;
    uint8_t enable_uuid_filter;
    lepl_app_state_info_t app_state;
} lepl_gatt_cb_t;

typedef struct
{
    uint16_t conn_id;                                 /** Peer Connection Id */
    wiced_bt_ga_pacs_audio_location_t audio_location; /** Audio Location */
} lepl_device_config_t;

typedef struct
{
    wiced_bt_ga_bap_stream_config_t *stream_config;
    lepl_device_config_t *config_list;
    uint16_t ctx_type;
    uint8_t num_devices;
} lepl_stream_config_t;

typedef struct {
    wiced_bt_ga_pacs_init_data_t *p_pacs_init_data;
    wiced_bt_ga_ascs_init_data_t *p_ascs_init_data;
    wiced_bt_ga_vcs_init_data_t  *p_vcs_init_data;
    wiced_bt_ga_mics_init_data_t *p_mics_init_data;
} lepl_ga_init_data_t;

extern wiced_bt_cfg_settings_t lepl_cfg_settings;
extern lepl_gatt_cb_t g_lepl_gatt_cb;

wiced_bt_gatt_status_t lepl_gatt_init(int max_connections, int max_mtu, lepl_ga_init_data_t *p_ga_init_data);

void lepl_gatt_start_discovery(uint8_t *p_bd_addr);

wiced_result_t lepl_gatt_start_stop_scan(uint32_t start, uint8_t enable_uuid_filter);

wiced_result_t lepl_gatt_disconnect(uint16_t conn_id);

wiced_result_t lepl_gatt_handle_disconnecting_state();

void lepl_gatt_start_stop_adv(uint32_t b_start);

extern wiced_result_t lepl_mcs_callback(uint16_t conn_id,
                                                  void *p_app_ctx,
                                                  gatt_intf_service_object_t *p_service,
                                                  wiced_bt_gatt_status_t status,
                                                  uint32_t evt_type,
                                                  gatt_intf_attribute_t *p_char,
                                                  void *p_data,
                                                  int len);

extern wiced_result_t lepl_ascs_callback(uint16_t conn_id,
                                                   void *p_app_ctx,
                                                   gatt_intf_service_object_t *p_service,
                                                   wiced_bt_gatt_status_t status,
                                                   uint32_t evt_type,
                                                   gatt_intf_attribute_t *p_char,
                                                   void *p_data,
                                                   int len);

extern wiced_result_t lepl_pacs_callback(uint16_t conn_id,
                                                   void *p_app_ctx,
                                                   gatt_intf_service_object_t *p_service,
                                                   wiced_bt_gatt_status_t status,
                                                   uint32_t evt_type,
                                                   gatt_intf_attribute_t *p_char,
                                                   void *p_data,
                                                   int len);

extern wiced_result_t lepl_vcs_callback(uint16_t conn_id,
                                                  void *p_app_ctx,
                                                  gatt_intf_service_object_t *p_service,
                                                  wiced_bt_gatt_status_t status,
                                                  uint32_t evt_type,
                                                  gatt_intf_attribute_t *p_char,
                                                  void *p_data,
                                                  int len);

wiced_result_t lepl_tbs_callback(uint16_t conn_id,
                                 void *p_app_ctx,
                                 gatt_intf_service_object_t *p_service,
                                 wiced_bt_gatt_status_t status,
                                 uint32_t evt_type,
                                 gatt_intf_attribute_t *p_char,
                                 void *p_data,
                                 int len);

extern wiced_result_t lepl_csis_callback(uint16_t conn_id,
                                  void *p_app_ctx,
                                  gatt_intf_service_object_t *p_service,
                                  wiced_bt_gatt_status_t status,
                                  uint32_t evt_type,
                                  gatt_intf_attribute_t *p_char,
                                  void *p_data,
                                  int len);

extern wiced_result_t lepl_mics_callback(uint16_t conn_id,
                                  void *p_app_ctx,
                                  gatt_intf_service_object_t *p_volume,
                                  wiced_bt_gatt_status_t status,
                                  uint32_t evt_type,
                                  gatt_intf_attribute_t *p_char,
                                  void *p_data_ptr,
                                  int len);

extern wiced_result_t lepl_mics_aics_callback(uint16_t conn_id,
                                              void *p_app_ctx,
                                              gatt_intf_service_object_t *p_volume,
                                              wiced_bt_gatt_status_t status,
                                              uint32_t evt_type,
                                              gatt_intf_attribute_t *p_char,
                                              void *p_data_ptr,
                                              int len);

wiced_result_t lepl_has_callback(uint16_t conn_id,
                                 void *p_app_ctx,
                                 gatt_intf_service_object_t *p_service,
                                 wiced_bt_gatt_status_t status,
                                 uint32_t evt_type,
                                 gatt_intf_attribute_t *p_char,
                                 void *p_data,
                                 int len);

void lepl_ascs_alloc_memory();
lepl_clcb_t *lepl_gatt_get_clcb(uint8_t *p_bd_addr);
lepl_clcb_t *lepl_gatt_get_clcb_by_conn_id(uint16_t conn_id);
void lepl_cap_event_cb(uint16_t conn_id, le_audio_cap_event_t event, le_audio_cap_event_data_t* p_event_data);
void lepl_init_remote_ases(lepl_clcb_t* p_clcb);
gatt_intf_attribute_t* ascs_init_characteristic(gatt_intf_attribute_t* p_char, lepl_ase_data_t* p_ase);
void iso_audio_init(void);

lepl_ase_data_t *lepl_get_remote_ase_data_by_ase_id(lepl_clcb_t *p_clcb, uint8_t ase_id);

lepl_ase_data_t *lepl_get_remote_ase(lepl_clcb_t *p_clcb,
                                     ascs_characteristics_t type,
                                     uint8_t *p_start_index);

wiced_result_t lepl_btm_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

// Cap

void lepl_cap_set_next_application_state(lepl_app_state_t state, uint32_t conn_id);
lepl_app_state_t lepl_cap_get_application_state(void);
lepl_app_state_t lepl_cap_get_application_final_state(void);
void lepl_cap_reset_application_state(void);
void lepl_cap_get_coordinated_set_members(uint16_t conn_id);
wiced_result_t lepl_cap_start_media_streaming(lepl_stream_config_t *p_stream_config);
wiced_result_t lepl_cap_start_conv_streaming(lepl_stream_config_t *p_stream_config);
wiced_result_t lepl_cap_start_mic_streaming(lepl_stream_config_t *p_stream_config);
void lepl_cap_stop_media_streaming(uint16_t conn_id);
void lepl_cap_stop_conv_streaming(uint16_t conn_id);
void lepl_cap_stop_mic_streaming(uint16_t conn_id);

// Iso

wiced_result_t lepl_isoc_dhm_setup_bis_datapath(uint16_t conn_hdl, wiced_bt_ga_bap_csc_t *p_csc);

wiced_result_t lepl_isoc_dhm_setup_cis_datapath(lepl_ase_data_t *p_ase);

void lepl_isoc_dhm_remove_cis_datapath(uint16_t conn_hdl, wiced_ble_isoc_data_path_bit_t dir);
void lepl_isoc_dhm_remove_bis_datapath(uint16_t *conn_hdl_list, uint8_t bis_count);
void lepl_isoc_dhm_start_cis_stream(uint16_t conn_hdl, wiced_ble_isoc_data_path_direction_t dir);
void lepl_isoc_dhm_start_bis_stream(uint16_t conn_hdl);
void lepl_isoc_dhm_stop_stream(uint16_t isoc_conn_hdl);
void lepl_isoc_init();
void lepl_isoc_dhm_init(void);
void lepl_isoc_dhm_disable_audio(void);


// MCS

wiced_result_t lepl_mcs_play(uint16_t conn_id, uint32_t codec_config);
wiced_result_t lepl_mcs_pause(uint16_t conn_id);
wiced_bool_t lepl_mcs_is_streaming();
void lepl_mcs_initialize_data();
void lepl_mcs_handle_post_operation();

// VCS
wiced_result_t lepl_vcs_set_volume(uint16_t conn_id, volume_control_opcodes_t opcode, uint8_t abs_vol);
wiced_result_t lepl_vcs_set_mute_state(uint16_t conn_id, wiced_bt_ga_mute_val_t mute_state);

//CSIS
wiced_result_t lepl_start_stop_set_member_discovery(uint8_t *sirk, uint8_t start_scan);
uint8_t lepl_if_sirk_zero(uint8_t *sirk);
uint8_t lepl_csis_device_belongs_to_coordinated_set(uint16_t conn_id, wiced_bt_ga_csis_sirk_t sirk);
void lepl_csis_handle_disconnection(wiced_bt_device_address_t address);

//TBS
void lepl_tbs_set_incoming_remote_call(lepl_tbs_data_t *p_tbs, char *uri_scheme, char *friendly_name);
wiced_result_t lepl_ccs_set_incoming_remote_call(uint16_t conn_id);
wiced_result_t lepl_ccs_terminate_call(uint16_t conn_id, uint8_t call_id, uint8_t termination_reason);
void lepl_ccs_set_remote_hold_call(uint8_t conn_id);
void lepl_ccs_set_retrieve_remote_hold_call(uint8_t conn_id);
void lepl_tbs_initialize_data();
lepl_ccs_states_t lepl_get_call_control_server_state();
void lepl_ccs_isoc_handle_ringtone_to_convo(lepl_ase_data_t *p_ase);
uint8_t lepl_ccs_get_active_call_id(void);
void lepl_ccs_start_streaming(uint16_t conn_id);
void lepl_ccs_start_streaming_convo(uint16_t conn_id);
void lepl_ccs_start_inband_ringtone(uint16_t conn_id);

//MICS
void lepl_micp_mute(uint16_t conn_id, uint8_t mute);
void lepl_micp_aics_mute(uint16_t conn_id, uint32_t instance, uint8_t mute);
void lepl_micp_aics_set_gain(uint16_t conn_id, uint32_t instance, uint8_t opcode, int8_t input_gain);

//HAP
void lepl_hap_read_preset_records(uint16_t conn_id);
void lepl_hap_set_active_preset(uint16_t conn_id, uint8_t opcode, uint8_t preset_index);
void lepl_hap_set_preset_name(uint16_t conn_id, uint8_t preset_index, gatt_intf_string_t *p_name);

// rpc
void lepl_rpc_init(uint8_t app_instance);
wiced_result_t lepl_start_stop_scan(uint32_t start, wiced_ble_ext_scan_result_cback_t *p_cback);

void lepl_start_voice_capture(uint16_t conn_id, uint32_t codec_config);
void lepl_rpc_send_mic_state(uint8_t state);


void app_set_connection_options(wiced_ble_ext_adv_phy_mask_t mask, wiced_ble_ext_conn_cfg_phy_options_t *p_out);
wiced_result_t app_create_connection(uint8_t addr_type, wiced_bt_device_address_t bd_addr);
void lepl_update_conn_param(wiced_bt_device_address_t bd_addr);

extern wiced_bt_cfg_settings_t lepl_cfg_settings;

/******************************************************************************
* Function Name: le_pl_cli_add_sink_dev
*******************************************************************************
* Summary: add found sink device to loacl array when scan, the index of the array
*          is the handle of device, need reset the array when start scan
*
* Parameters:
*   wiced_ble_ext_scan_results_t *p_scan_result
*
* Return:
*      None
*
******************************************************************************/
extern void le_pl_cli_add_sink_dev(wiced_ble_ext_scan_results_t *p_scan_result);


#endif
