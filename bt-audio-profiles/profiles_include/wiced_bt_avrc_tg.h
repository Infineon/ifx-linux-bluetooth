/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company)
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/** @file
 *
 * Bluetooth AVRC Remote control Target API
 *
 */

#ifndef WICED_BT_RC_TG_H_
#define WICED_BT_RC_TG_H_

#include "wiced_bt_cfg.h"
#include "wiced_bt_avrc_defs.h"
/****************************************************************************/



/**
 * @defgroup  wicedbt_avrc_tg        Audio/Video Remote Control Profile  (AVRCP) Target
 *
 * WICED Bluetooth AVRC Remote Control Functions.
 *
 * @addtogroup wicedbt_avrc_tg Audio/Video Remote control Profile
 * @ingroup wicedbt_av
 * @{
 */


/****************************************************************************/

/******************************************************************************
 *                         Supported features
 ******************************************************************************/

/* APP_AVRCP10 is defined in make target when AVRCP version needs to be 1.0, default is AVRCP 1.3.
 * AVRCP 1.3 adds features to display of track (song) info, player control (for repeat, shuffle, etc.)
 * and display of play state (pause, play, song position)
 */

#ifndef APP_AVRCP10
/* Whether track info is supported */
#define APP_AVRC_TRACK_INFO_SUPPORTED

/* Whether player state change (pause/play/stop, play position) is supported */
#define APP_AVRC_PLAY_STATUS_SUPPORTED

/* Whether player settings (repeat/shuffle/etc.) is supported */
#define APP_AVRC_SETTING_CHANGE_SUPPORTED

#ifdef APP_AVRC_PLAY_STATUS_SUPPORTED
/* Whether player position updates are supported */
#define APP_AVRC_TRACK_PLAY_POS_CHANGE_SUPPORTED
#endif

/* Additional capabilities could be supported by application, compiled out */
/*
#define APP_AVRC_TRACK_REACHED_END_SUPPORTED
#define APP_AVRC_TRACK_REACHED_START_SUPPORTED
#define APP_AVRC_BATTERY_STATUS_SUPPORTED
#define APP_AVRC_SYSTEM_STATUS_SUPPORTED
*/
#endif /* APP_AVRCP10 */

#define MAX_AVRCP_VOLUME_LEVEL  0x7f


/*****************Track info data structures********************/
/* If track info is supported ..*/
#ifdef APP_AVRC_TRACK_INFO_SUPPORTED

/* Maximum number of attributes types supported in track info */
/* Track info attributes include Track title, Artist, Album, Genre, Total tracks, Current track number, Playing time*/
#define APP_AVRC_MAX_ATTR   AVRC_MAX_NUM_MEDIA_ATTR_ID

/* Maximum string length of track info attribute */
#define APP_AVRC_MAX_ATTR_LEN   200          /**< Maximum length of the stored media attribute fields. */

/* Track info attributes */
typedef struct {

    /* Type of track attribute - Track title, Artist, Album, Genre, Total tracks, Current track number, Playing time*/
    uint8_t    attr_id;
    /* String length */
    uint8_t    str_len;
    /* String name of attribute in UTF8 */
    uint8_t    p_str[APP_AVRC_MAX_ATTR_LEN+1];
} wiced_bt_avrc_tg_track_attr_t;

/* Track info array */
typedef struct {
    /* total number of track info attributes supported by app */
    uint8_t             app_attr_count;
    /* array of track info attributes */
    wiced_bt_avrc_tg_track_attr_t app_track_attr[APP_AVRC_MAX_ATTR+1];
} wiced_bt_avrc_tg_track_info_t;

#endif
/***************************************************************/

/********************Player settings data structures************/
/* If player settings (repeat/shuffle/etc.) is supported ... */
#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED

/* Maximum number of attribute values supported, see below */
#define APP_AVRC_MAX_APP_ATTR_SIZE 4

/* Player settings attribute value definition struct */
typedef struct {
    /* Attribute type (repeat, shuffle, etc.)*/
    uint8_t                 attr_id;
    /* Number of supported values for each attribute, example : Repeat Off, Repeat single track, Repeat all tracks, etc. */
    uint8_t                 num_val;
    /* Array of supported values for each attribute type */
    uint8_t                 vals[APP_AVRC_MAX_APP_ATTR_SIZE];
    /* current attribute value set by user */
    uint8_t                 curr_value;
} wiced_bt_avrc_tg_player_attr_t;

#endif
/***************************************************************/

/********************Player status data structures**************/
/* If player state change (pause/play/stop, play position) is supported ... */
#ifdef APP_AVRC_PLAY_STATUS_SUPPORTED
typedef struct {
    /* Song length */
    uint32_t                song_len;
    /* Song position */
    uint32_t                song_pos;
    /* Play state (Play, pause, stop */
    uint8_t                 play_state;
} wiced_bt_avrc_tg_play_status_t;
#endif
/***************************************************************/
#define AVRC_NUM_PLAYER_SUPPORTED   0x02 /* Number of player supported */
#define AVRC_MAX_FOLDER_DEPTH       0x03 /* Maximum folder depth for getfolderitems */


/* Event handling, events sent to the app */

/* Event IDs */
#define APP_AVRC_EVENT_DEVICE_CONNECTED             1  /* peer device connected */
#define APP_AVRC_EVENT_REPEAT_SETTINGS_CHANGED      2  /* peer changed repeat settings */
#define APP_AVRC_EVENT_SHUFFLE_SETTINGS_CHANGED     3  /* peer changed shuffle settings */
#define APP_AVRC_EVENT_DEVICE_DISCONNECTED          4  /* peer device disconnected */
#define APP_AVRC_EVENT_PASSTHROUGH_RESPONSE      5  /* passthrough command response from peer */
#define APP_AVRC_EVENT_PASSTHROUGH_CMD           6  /* passthrough command response from peer */
#define APP_AVRC_EVENT_ABS_VOL_CHANGED           7  /* peer changed Absolute Volume */

/* Paththrough Cmd sub event */
#define APP_AVRC_EVENT_PASSTHROUGH_CMD_PLAY             1  /* Passthrough Command Play */
#define APP_AVRC_EVENT_PASSTHROUGH_CMD_PAUSE            2  /* Passthrough Command Pause */
#define APP_AVRC_EVENT_PASSTHROUGH_CMD_STOP             3  /* Passthrough Command Stop */
#define APP_AVRC_EVENT_PASSTHROUGH_CMD_NEXT_TRACK       4  /* Passthrough Command Next Track */
#define APP_AVRC_EVENT_PASSTHROUGH_CMD_PREVIOUS_TRACK   5  /* Passthrough Command Previous Track */
#define APP_AVRC_EVENT_PASSTHROUGH_CMD_VOLUME_UP        6  /* Passthrough Command Volume Up */
#define APP_AVRC_EVENT_PASSTHROUGH_CMD_VOLUME_DOWN      7  /* Passthrough Command Volume Down */


/* Passthrough Command event data */
typedef struct {
    uint16_t                    handle;
    uint8_t                     command;
} wiced_bt_avrc_tg_passthrough_cmd_t;

/* Absolute Volume event data */
typedef struct {
    uint16_t                    handle;
    uint8_t                     volume;
} wiced_bt_avrc_tg_absolute_volume_t;

/* Event data */
typedef struct {
    /* Peer connected device info, sent to MCU app (used with APP_AVRC_EVENT_DEVICE_CONNECTED) */
    wiced_bt_device_address_t   bd_addr;
    uint16_t                    handle;
    /* Repeat or shuffle setting, senf to MCU app (used with APP_AVRC_EVENT_REPEAT_SETTINGS_CHANGED or APP_AVRC_EVENT_SHUFFLE_SETTINGS_CHANGED) */
    uint8_t                     setting_val;
    /* AVRCP Supported Features Attribute */
    wiced_bool_t                attribute_search_completed;
    uint16_t                    supported_features;
    /* Passthrough command response */
    uint8_t                     passthrough_response;
    /* Passthrough Command received (used with APP_AVRC_EVENT_PASSTHROUGH_CMD event) */
    wiced_bt_avrc_tg_passthrough_cmd_t passthrough_command;
    /* Absolute Volume [0..100] */
    wiced_bt_avrc_tg_absolute_volume_t absolute_volume;

} wiced_bt_rc_event_t;

/*
 * AVRC event callback to receive events from AVRC profile. Application must implement this api
 */
typedef void ( wiced_bt_avrc_tg_event_cback_t) (uint8_t event_id,  wiced_bt_rc_event_t *_event);


/******************************************************
 *               Function Declarations
 ******************************************************/

/*******************************************************************************
* Function        wiced_bt_avrc_tg_init

** Description    Called to initialize AV RC profile
*******************************************************************************/
void wiced_bt_avrc_tg_init( wiced_bt_avrc_tg_event_cback_t *p_cb );


/*******************************************************************************
* Function        wiced_bt_avrc_tg_deinit

** Description    Called to deinitialize AV RC profile
*******************************************************************************/

void wiced_bt_avrc_tg_deinit(void);


/*******************************************************************************
* Function        wiced_bt_avrc_tg_register

** Description    Called to register AVRC profile with the BT stack
*******************************************************************************/
void wiced_bt_avrc_tg_register(void);

/*******************************************************************************
* Function        wiced_bt_avrc_tg_initiate_open

** Description    Called to initiate connection to given BDA
*******************************************************************************/
void wiced_bt_avrc_tg_initiate_open(wiced_bt_device_address_t peer_addr);

/*******************************************************************************
* Function        wiced_bt_avrc_tg_initiate_close

** Description    Called to disconnect AVRC connection
*******************************************************************************/
void wiced_bt_avrc_tg_initiate_close( void );

#ifdef APP_AVRC_TRACK_INFO_SUPPORTED
/*******************************************************************************
* Function        wiced_bt_rc_set_track_info

** Description    Called to set current playing track information
*******************************************************************************/
void wiced_bt_rc_set_track_info(wiced_bt_avrc_tg_track_attr_t *p_track_attr);
#endif

#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
/*******************************************************************************
* Function        wiced_bt_rc_set_player_settings

** Description    Called to set player settings (repeat, shuffle, etc).
*******************************************************************************/
void wiced_bt_rc_set_player_settings(wiced_bt_avrc_tg_player_attr_t *p_info);

/*******************************************************************************
* Function        wiced_bt_rc_player_setting_changed

** Description    Called when player setting (repeat, shuffle) is changed
** Paramaters
    uint8_t attr_id : Atrribute type (repeat, shuffle, etc.)
    uint8_t value   : Attrbute value (off, single, all, etc.)
*******************************************************************************/
void wiced_bt_rc_player_setting_changed(uint8_t attr_id, uint8_t value);

#endif

#ifdef APP_AVRC_PLAY_STATUS_SUPPORTED
/*******************************************************************************
* Function        wiced_bt_rc_set_player_status

** Description    Called to set player status (pause/play/stop, song position)
*******************************************************************************/
void wiced_bt_rc_set_player_status(wiced_bt_avrc_tg_play_status_t *p_info);

#endif

#ifdef APP_AVRC_TRACK_INFO_SUPPORTED
/**
 * Function         wiced_bt_rc_track_changed
 *
 *                  Called when current track is changed
 *
 * @return          None
 *
 */
void wiced_bt_rc_track_changed(void);
#endif

/**
 * Function         wiced_bt_avrc_tg_register_absolute_volume_change
 *
 *                  Register for absolute volume change notifications
 *
 * @return          None
 *
 */
void wiced_bt_avrc_tg_register_absolute_volume_change(void);

/**
 * Function         wiced_bt_avrc_tg_absolute_volume_change
 *
 *                  Called when volume is changed, send Absolute volume request to peer
 *
 * @param[in]       handle : handle of connection
 * @param[in]       volume : volume set by application as a percentage
 *
 * @return          wiced_result_t
 *
 */

wiced_result_t wiced_bt_avrc_tg_absolute_volume_changed(uint16_t handle, uint8_t volume );

/**
 * Function         wiced_bt_avrc_tg_is_peer_absolute_volume_capable
 *
 *                  Return non zero if peer is absolute volume capable
 *
 * @return          None
 *
 */
uint8_t wiced_bt_avrc_tg_is_peer_absolute_volume_capable( void );

/**
 * Function         wiced_bt_avrc_tg_volume_button_press
 *
 *                  Called when volume up/down button is pressed
 *
 * @param[in]       op_id : operation ID  AVRC_ID_VOL_UP or AVRC_ID_VOL_DOWN
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_avrc_tg_volume_button_press(uint8_t op_id);

/**
 * @} wicedbt_avrc_tg
 */
/* end of avrc_tg */
#ifdef __cplusplus
}
/* extern "C" */
#endif
#endif /* WICED_BT_RC_TG_H_ */
