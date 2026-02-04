/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __LEHS_GATT_H__
#define __LEHS_GATT_H__

typedef enum {
    ADV_STATE_IDLE = 0,
    ADV_STATE_SWIFT_PAIR_HIGH_DUTY_CYCLE,
    ADV_STATE_SWIFT_PAIR_LOW_DUTY_CYCLE,
    ADV_STATE_REGULAR_ADV,
    ADV_STATE_MAX
} adv_state_t;

typedef struct
{
    uint16_t acl_conn_handle; /**< ACL connection handle */
    uint16_t cis_conn_handle; /**< CIS connection handle */

    uint8_t lc3_index;                  // lc3 codec index ?  What is this one exactly ?
    uint32_t data_path_established : 1; // data path established ?
    wiced_bt_ga_ascs_ase_t data;
} lehs_ase_data_t;

typedef struct
{
    gatt_intf_service_object_t *p_pacs;
    gatt_intf_service_object_t *p_ascs;
    gatt_intf_service_object_t *p_vcs;
    gatt_intf_service_object_t *p_bass;
    gatt_intf_service_object_t *p_csis;
    gatt_intf_service_object_t *p_mics;
    gatt_intf_service_object_t *p_has;
    gatt_intf_service_object_t *p_mics_aics[MAX_MICS_AICS];
    uint8_t num_mics_aics;
} lehs_local_profiles_t;

typedef struct
{
    gatt_intf_service_object_t *p_mcs;
    gatt_intf_service_object_t *p_gmcs;
    gatt_intf_service_object_t *p_gtbs;
} lehs_peer_profiles_t;

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
} lehs_mcs_data_t;

typedef struct
{
    uint8_t in_use;
    uint16_t conn_id;
    uint32_t addr_type;
    wiced_bt_device_address_t bda;
    wiced_bool_t b_is_central;
    uint32_t num_local_ases;
    lehs_peer_profiles_t peer_profiles;
    //wiced_bt_ga_pacs_data_t pacs_data;
    lehs_ase_data_t *p_local_ase_data;
    lehs_mcs_data_t mcs_data;
} lehs_clcb_t;

typedef struct
{
    wiced_bt_ga_vcs_volume_state_t state; /* current volume state */
    uint8_t flag;                      /* current volume flag */
} lehs_volume_t;

typedef struct
{
    wiced_bt_ga_aics_input_state_t input_state;           /**< Audio Input State */
    wiced_bt_ga_aics_gain_settings_params_t gain_setting; /**< Audio Input Gain setting */
    wiced_bt_ga_aics_input_type_t input_type;             /**< Audio Input Type */
    wiced_bt_ga_aics_input_status_t input_status;         /**< Audio Input Status */
    char description[MAX_DESCRIPTION];                    /**< Audio Input descritpion */
    uint8_t description_len;
} lehs_aics_t;

typedef struct
{
    uint8_t mute_state;
    lehs_aics_t aics[MAX_MICS_AICS];
} lehs_mics_t;

typedef struct
{
    lehs_local_profiles_t local_profiles;      /* list of local supported profiles */
    lehs_volume_t vcs_data;                    /* local volume data */
    lehs_mics_t mics_data;
    wiced_bt_ga_pacs_data_t *p_pacs_data;
    lehs_clcb_t clcb[MAX_CONNECTION_INSTANCE]; /* Number of simultaneous connections */
    broadcast_sink_cb_t broadcast_sink_cb[MAX_BIG];
    wiced_bool_t broadcast_sink_periodic_sync_in_progress;
    broadcast_source_t broadcast_source;
    lehs_bass_t bass_data;
    adv_state_t adv_state;
    uint16_t adv_data_options;
    uint8_t adv_tx_power;
    uint8_t do_swift_pair;
} lehs_gatt_cb_t;

typedef struct {
    wiced_bt_ga_pacs_init_data_t *p_pacs_init_data;
    wiced_bt_ga_ascs_init_data_t *p_ascs_init_data;
    wiced_bt_ga_vcs_init_data_t  *p_vcs_init_data;
    wiced_bt_ga_mics_init_data_t *p_mics_init_data;
    wiced_bt_ga_bass_init_data_t *p_bass_init_data;
} lehs_ga_init_data_t;

extern lehs_gatt_cb_t g_lehs_gatt_cb;

wiced_bt_gatt_status_t lehs_gatt_init(int max_connections, int max_mtu, lehs_ga_init_data_t p_ga_data);

void lehs_gatt_start_discovery(uint8_t *p_bd_addr);

void lehs_gatt_start_stop_adv(uint32_t b_start, adv_state_t adv_state);

adv_state_t lehs_move_to_next_adv_state(adv_state_t adv_state, char * from);

extern wiced_result_t lehs_mcs_callback(uint16_t conn_id,
                                                void *p_app_ctx,
                                                gatt_intf_service_object_t *p_service,
                                                wiced_bt_gatt_status_t status,
                                                uint32_t evt_type,
                                                gatt_intf_attribute_t *p_char,
                                                void *p_data,
                                                int len);

extern wiced_result_t lehs_ascs_callback(uint16_t conn_id,
                                                 void *p_app_ctx,
                                                 gatt_intf_service_object_t *p_service,
                                                 wiced_bt_gatt_status_t status,
                                                 uint32_t evt_type,
                                                 gatt_intf_attribute_t *p_char,
                                                 void *p_data,
                                                 int len);

extern wiced_result_t lehs_pacs_callback(uint16_t conn_id,
                                                 void *p_app_ctx,
                                                 gatt_intf_service_object_t *p_service,
                                                 wiced_bt_gatt_status_t status,
                                                 uint32_t evt_type,
                                                 gatt_intf_attribute_t *p_char,
                                                 void *p_data,
                                                 int len);

extern wiced_result_t lehs_vcs_callback(uint16_t conn_id,
                                                void *p_app_ctx,
                                                gatt_intf_service_object_t *p_service,
                                                wiced_bt_gatt_status_t status,
                                                uint32_t evt_type,
                                                gatt_intf_attribute_t *p_char,
                                                void *p_data,
                                                int len);

extern wiced_result_t lehs_bass_callback(uint16_t conn_id,
                                             void *p_app_ctx,
                                             gatt_intf_service_object_t *p_service,
                                             wiced_bt_gatt_status_t status,
                                             uint32_t evt_type,
                                             gatt_intf_attribute_t *p_char,
                                             void *p_data,
                                             int len);

extern wiced_result_t lehs_ccp_callback(uint16_t conn_id,
                                        void *p_app_ctx,
                                        gatt_intf_service_object_t *p_service,
                                        wiced_bt_gatt_status_t status,
                                        uint32_t evt_type,
                                        gatt_intf_attribute_t *p_char,
                                        void *p_data,
                                        int len);

wiced_result_t lehs_mics_aics_callback(uint16_t conn_id,
                                       void *p_app_ctx,
                                       gatt_intf_service_object_t *p_service,
                                       wiced_bt_gatt_status_t status,
                                       uint32_t evt_type,
                                       gatt_intf_attribute_t *p_char,
                                       void *p_evt_data,
                                       int len);


void lehs_ascs_alloc_memory();
void lehs_pacs_alloc_memory();
void lehs_ascs_init_data(lehs_clcb_t *p_clcb);
void lehs_pacs_init_data(void);
void lehs_vcs_initialize_data(void);

extern lehs_clcb_t *lehs_gatt_get_clcb_by_conn_id(uint16_t conn_id);
gatt_intf_attribute_t *ascs_init_characteristic(gatt_intf_attribute_t *p_char,
                                                     lehs_ase_data_t *p_ase);
void lehs_isoc_event_handler(wiced_ble_isoc_event_t event, wiced_ble_isoc_event_data_t *p_event_data);
void iso_audio_init(void);

gatt_intf_service_object_t *lehs_gatt_get_bass_service_instance(void);
void lehs_ext_adv_cback(wiced_ble_ext_adv_event_t event, wiced_ble_ext_adv_event_data_t *p_ed);
void lehs_bis_isoc_cb(wiced_ble_isoc_event_t event, wiced_ble_isoc_event_data_t *p_ed);
#endif
