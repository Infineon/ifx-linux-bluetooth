/*
 * $ Copyright Cypress Semiconductor $
 */


 /**************************************************************************//**
  * \file <wiced_bt_ga.h>
  *
  * Definitions for interface between WICED BT GA libraries and applications
  *
  */

#ifndef __WICED_BT_GA__
#define __WICED_BT_GA__

#include "gatt_interface.h"

typedef enum {
    AICS_INPUT_STATE_CHARACTERISTIC,                    //0 RWN
    AICS_GAIN_SETTING_PROPERTIES_CHARACTERISTIC,        //1
    AICS_INPUT_TYPE_CHARACTERISTIC,                     //2
    AICS_INPUT_STATUS_CHARACTERISTIC,                   //3
    AICS_INPUT_CONTROL_POINT_CHARACTERISTIC,            //4
    AICS_INPUT_DESCRIPTION_CHARACTERISTIC,              //5
    AICS_MAX_CHARACTERISTIC,                            //6
}aics_characteristics_t;

// VOCS Characteristic index
typedef enum
{
    VOCS_OFFSET_STATE_CHARACTERISTIC,                   //0
    VOCS_AUDIO_LOCATION_CHARACTERISTIC,                 //1
    VOCS_VOLUME_OFFSET_CONTROL_POINT_CHARACTERISTIC,    //2
    VOCS_AUDIO_DESCRIPTION_CHARACTERISTIC,              //3
    VOCS_MAX_CHARACTERISTIC,                            //4
} vocs_characteristics_t;

#define INCLUDED_SERVICE_NONE 0
#define INCLUDED_SERVICE_VCS_AICS  0x1
#define INCLUDED_SERVICE_VCS_VOCS  0x2
#define INCLUDED_SERVICE_MICS_AICS 0x1

typedef enum {
    VCS_VOLUME_STATE_CHARACTERISTIC,      //0
    VCS_CONTROL_POINT_CHARACTERISTIC,     //1
    VCS_VOLUME_FLAG_CHARACTERISTIC,       //2
    VCS_MAX_CHARACTERISTIC,               //3
} vcs_characteristics_t;

typedef enum
{
    MICS_MUTE_STATE_CHARACTERISTIC,                          //0
    MICS_MAX_CHARACTERISTIC,                                 //1
} mics_characteristics_t;

typedef enum {
    MCS_MEDIA_PLAYER_NAME_CHARACTERISTIC,               //0
    MCS_MEDIA_TRACK_CHANGED_CHARACTERISTIC,             //1
    MCS_MEDIA_TRACK_TITLE_CHARACTERISTIC,               //2
    MCS_MEDIA_TRACK_DURATION_CHARACTERISTIC,            //3
    MCS_MEDIA_TRACK_POSITION_CHARACTERISTIC,            //4
    MCS_MEDIA_PLAYBACK_SPEED_CHARACTERISTIC,            //5
    MCS_MEDIA_SEEKING_SPEED_CHARACTERISTIC,             //6
    MCS_MEDIA_PLAYING_ORDER_CHARACTERISTIC,             //7
    MCS_MEDIA_PLAYING_ORDER_SUPPORTED_CHARACTERISTIC,   //8
    MCS_MEDIA_STATE_CHARACTERISTIC,                     //9
    MCS_MEDIA_CONTROL_POINT_CHARACTERISTIC,             //10
    MCS_MEDIA_OPCODE_SUPPORTED_CHARACTERISTIC,          //11
    MCS_CONTENT_CONTROL_ID_CHARACTERISTIC,              //12
    MCS_MEDIA_ICON_OBJECT_CHARACTERISTIC,               //13
    MCS_MEDIA_ICON_URI_CHARACTERISTIC,                  //14
    MCS_MEDIA_TRACK_SEGMENT_CHARACTERISTIC,             //15
    MCS_MEDIA_CURRENT_TRACK_OBJ_CHARACTERISTIC,         //16
    MCS_MEDIA_NEXT_TRACK_OBJ_CHARACTERISTIC,            //17
    MCS_MEDIA_PARENT_GROUP_OBJ_CHARACTERISTIC,          //18
    MCS_MEDIA_CURRENT_GROUP_OBJ_CHARACTERISTIC,         //19
    MCS_MEDIA_SEARCH_RESULTS_OBJ_CHARACTERISTIC,        //20
    MCS_MEDIA_SEARCH_CONTROL_POINT_CHARACTERISTIC,      //21
    MCS_MEDIA_CONTENT_CONTROL_ID,                       //22
    MCS_MAX_CHARACTERISTIC,                             //23
} mcs_characteristics_t;

typedef enum
{
    TBS_BEARER_PROVIDER_NAME_CHARACTERISTIC,                      //0
    TBS_BEARER_UCI_CHARACTERISTIC,                                //1
    TBS_BEARER_TECHNOLOGY_CHARACTERISTIC,                         //2
    TBS_BEARER_URI_SUPPORTED_SCHEMES_CHARACTERISTIC,              //3
    TBS_BEARER_SIGNAL_STRENGTH_CHARACTERISTIC,                    //4
    TBS_BEARER_SIGNAL_STRENGTH_REPORTING_INTERVAL_CHARACTERISTIC, //5
    TBS_BEARER_LIST_CURRENT_CALLS_CHARACTERISTIC,                 //6
    TBS_CONTENT_CONTROL_ID_CHARACTERISTIC,                        //7
    TBS_INCOMING_CALL_TG_BEARER_URI_CHARACTERISTIC,               //8
    TBS_STATUS_FLAGS_CHARACTERISTIC,                              //9
    TBS_CALL_STATE_CHARACTERISTIC,                                //10
    TBS_CALL_CONTROL_POINT_CHARACTERISTIC,                        //11
    TBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE_CHARACTERISTIC,        //12
    TBS_CALL_TERMINATION_REASON_CHARACTERISTIC,                   //13
    TBS_INCOMING_CALL_CHARACTERISTIC,                             //14
    TBS_CALL_FRIENDLY_NAME_CHARACTERISTIC,                        //15
    TBS_MAX_CHARACTERISTIC,                                       //16
} tbs_characteristics_t;

typedef enum
{
    CSIS_SIRK_CHARACTERISTIC,   //0
    CSIS_SIZE_CHARACTERISTIC,   //1
    CSIS_LOCK_CHARACTERISTIC,   //2
    CSIS_RANK_CHARACTERISTIC,   //3
    CSIS_MAX_CHARACTERISTIC,    //4
} csis_characteristics_t;

typedef enum
{
    WICED_BT_GA_RAAS_SELECTABLE_AREP_CHARACTERISTIC,               //0
    WICED_BT_GA_RAAS_CFG_AUDIO_ROUTE_CHARACTERISTIC,              //1
    WICED_BT_GA_RAAS_CFG_AUDIO_ROUTE_CONTENT_TYPE_CHARACTERISTIC, //2
    WICED_BT_GA_RAAS_MODIFY_AUDIO_ROUTE_CHARACTERISTIC,           //3
    WICED_BT_GA_RAAS_MAX_CHARACTERISTIC,                          //4
} wiced_bt_ga_raas_server_handle_list_t;

typedef enum
{
    ASCS_ASE_CONTROL_POINT_CHARACTERISTIC,                         // 0
    ASCS_MAX_UNIQUE_CHARACTERISTIC,                                // 1
    ASCS_SINK_ASE_CHARACTERISTIC = ASCS_MAX_UNIQUE_CHARACTERISTIC, // 1
    ASCS_SOURCE_ASE_CHARACTERISTIC,                                // 2
    ASCS_MAX_CHARACTERISTIC                                        // 3
} ascs_characteristics_t;

typedef enum
{
    PACS_AVAILABILE_AUDIO_CONTEXTS_CHARACTERISTIC,                        // 0
    PACS_SUPPORTED_AUDIO_CONTEXTS_CHARACTERISTIC,                         // 1
    PACS_SINK_AUDIO_LOCATIONS_CHARACTERISTIC,                             // 2
    PACS_SOURCE_AUDIO_LOCATIONS_CHARACTERISTIC,                           // 3
    PACS_MAX_UNIQUE_CHARACTERISTIC,                                       // 4
    PACS_SINK_CAPABILITY_CHARACTERISTIC = PACS_MAX_UNIQUE_CHARACTERISTIC, // 4
    PACS_SOURCE_CAPABILITY_CHARACTERISTIC,                                // 5
    PACS_MAX_CHARACTERISTIC                                               // 6
} pacs_characteristics_t;

typedef enum
{
    BASS_BROADCAST_AUDIO_SCAN_CONTROL_POINT_CHARACTERISTIC,               // 0
    BASS_MAX_UNIQUE_CHARACTERISTIC,                                       // 1
    BASS_BROADCAST_RECEIVE_STATE_CHARACTERISTIC = BASS_MAX_UNIQUE_CHARACTERISTIC,                          // 1
    BASS_MAX_CHARACTERISTIC,                                              // 2
} bass_characteristics_t;

typedef enum
{
    TMAP_ROLE_CHARACTERISTIC, //0
    TMAS_MAX_CHARACTERISTIC,  //1
} tmas_characteristics_t;

extern const wiced_bt_uuid_t ga_service_uuid_vcs;
extern const wiced_bt_uuid_t ga_service_uuid_mcs;
extern const wiced_bt_uuid_t ga_service_uuid_mics;
extern const wiced_bt_uuid_t ga_service_uuid_tbs;
extern const wiced_bt_uuid_t ga_service_uuid_gtbs;
extern const wiced_bt_uuid_t ga_service_uuid_mcs;
extern const wiced_bt_uuid_t ga_service_uuid_gmcs;
extern const wiced_bt_uuid_t ga_service_uuid_pacs;
extern const wiced_bt_uuid_t ga_service_uuid_ascs;
extern const wiced_bt_uuid_t ga_service_uuid_bass;
extern const wiced_bt_uuid_t ga_service_uuid_csis;
extern const wiced_bt_uuid_t ga_service_uuid_tmas;

#endif
