/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __LEHS_H__
#define __LEHS_H__

/* BT Stack includes */
#include "wiced_bt_dev.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "wiced_timer.h"

/* App Library includes */
#include "wiced_bt_ga_bass.h"
#include "wiced_bt_ga_ascs.h"
#include "wiced_bt_ga_pacs.h"
#include "wiced_bt_ga_vcs.h"
#include "wiced_bt_ga_mcs.h"
#include "wiced_bt_ga_tbs.h"
#include "wiced_bt_ga_mics.h"
#include "wiced_bt_ga_has.h"
#include "wiced_bt_ga_common.h"
#include "le_audio_bap_broadcast.h"
#include "wiced_bt_ga_csis.h"
#include "wiced_bt_ga_csis_common.h"
#include "le_audio_rpc.h"
#include "wiced_bt_ga_csis_common.h"
#include "csis_psri_key.h"


#define HAS_ENABLED 1

#define UNICAST_SINK_EXT_ADV_HANDLE 1
#define MAX_CONNECTION_INSTANCE 2
#define MAX_BIG 20
#define DEFAULT_VOL 100
#define MAX_BROADCAST_NAME_SIZE 32
#define VCS_STEP_SIZE 15
#define MAX_BASS 2 /**/
#define INVALID_ASE_ID 55
#define MAX_MICS_AICS 2
#define MAX_DESCRIPTION 20

/* Application includes */
#include "lehs_rpc.h"
#include "lehs_nvram.h"
#include "lehs_bass.h"
#include "lehs_bis.h"
#include "lehs_gatt.h"
#include "lehs_isoc.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define DEVICE_NAME             "LeAudioHS"
#define SUPPORT_INBAND_RINGTONE 1
#define CSIS_CFG_MIC_LEFT_NAME  "csis_cfg_mic_l.bin"
#define CSIS_CFG_MIC_RIGHT_NAME "csis_cfg_mic_r.bin"
#define CSIS_SIZE_STR           "size"
#define CSIS_RANK_STR           "rank"
#define CSIS_LOCATION_STR       "location"
#define CSIS_CFG_STR_SIZE       (3)

//Enable definition of MAX_SUPPORTED_OCTETS_PER_CODEC_FRAME in makefile for audio config higher than 48_2_2
#ifndef MAX_SUPPORTED_OCTETS_PER_CODEC_FRAME
#define MAX_SUPPORTED_OCTETS_PER_CODEC_FRAME 0x64
#endif // !MAX_SUPPORTED_OCTETS_PER_CODEC_FRAME
#define LEHS_MAX_SDU_SIZE (MAX_SUPPORTED_OCTETS_PER_CODEC_FRAME * 2)

/*******************************************************************************
*                               STRUCTURES AND ENUMERATIONS
*******************************************************************************/
typedef enum {
     CSIS_SIZE,
     CSIS_RANK,
     CSIS_LOCATION
}csis_enum_t;

typedef struct
{
    wiced_bt_ga_csis_sirk_data_t sirk_data;
    SIRK encr_sirk; /**< encrypted SIRK key */
    uint8_t size;
    uint8_t rank;
    uint16_t conn_id_of_lock_owner;
    wiced_bt_ga_csis_lock_val_t lock;
} lehs_csis_data_t;

typedef struct {
    csis_enum_t csis_idx;
    char *csis_str;
    union
    {
        int number;
    }csis_value;
}csis_cfg_t;

wiced_result_t lehs_btm_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
wiced_result_t lehs_mcs_play_pause(uint16_t conn_id, wiced_bool_t play);
void lehs_set_audio_location(uint32_t audio_location);

//VCS
wiced_result_t lehs_vcs_set_volume(uint16_t conn_id, volume_control_opcodes_t vcs_opcode, uint8_t abs_vol);
void lehs_set_vol(volume_control_opcodes_t vcs_opcode, uint8_t abs_vol);

//CSIS
void lehs_csis_set_size(uint8_t size);
void lehs_csis_set_rank(uint8_t rank);
void lehs_csis_set_sirk(wiced_bt_ga_csis_sirk_data_t *p_sirk);
wiced_result_t lehs_csis_callback(uint16_t conn_id,
                                  void *p_app_ctx,
                                  gatt_intf_service_object_t *p_service,
                                  wiced_bt_gatt_status_t status,
                                  uint32_t evt_type,
                                  gatt_intf_attribute_t *p_char,
                                  void *p_evt_data,
                                  int len);

wiced_bt_ga_csis_sirk_data_t *lehs_csis_get_sirk(void);

wiced_result_t lehs_csis_initialize_data(void);

//MICS
wiced_result_t lehs_mics_callback(uint16_t conn_id,
                                  void *p_app_ctx,
                                  gatt_intf_service_object_t *p_service,
                                  wiced_bt_gatt_status_t status,
                                  uint32_t evt_type,
                                  gatt_intf_attribute_t *p_char,
                                  void *p_evt_data,
                                  int len);
void lehs_mics_initialize_data(void);
wiced_bt_gatt_status_t lehs_mics_mute(uint16_t conn_id, uint8_t mute);
wiced_bt_gatt_status_t lehs_mics_aics_mute(uint16_t conn_id, uint32_t instance, uint8_t mute);
wiced_bt_gatt_status_t lehs_mics_aics_set_gain(uint16_t conn_id, uint32_t instance, uint8_t opcode, int8_t input_gain);

//HAS
wiced_result_t lehs_has_callback(uint16_t conn_id,
                                  void *p_app_ctx,
                                  gatt_intf_service_object_t *p_service,
                                  wiced_bt_gatt_status_t status,
                                  uint32_t evt_type,
                                  gatt_intf_attribute_t *p_char,
                                  void *p_evt_data,
                                  int len);
void lehs_has_initialize_data(void);

extern wiced_bt_cfg_settings_t *app_get_cfg_settings(void);
wiced_result_t app_create_connection(uint8_t addr_type, wiced_bt_device_address_t bd_addr);
uint8_t app_init_csis_by_cfg_file( char *file_name );
#endif
