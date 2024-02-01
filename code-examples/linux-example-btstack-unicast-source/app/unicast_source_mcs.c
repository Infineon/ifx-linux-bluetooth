/*
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*/

#include "unicast_source_mcs.h"
#include "unicast_source_cap.h"
#include "unicast_source_gatt.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_mcp.h"
#include "wiced_timer.h"
#include "le_audio_rpc.h"

/******************************************************************************
 *                              EXTERNS VARIABLE
 *****************************************************************************/
extern unicast_source_gatt_cb_t g_unicast_source_gatt_cb;
unicast_source_mcs_data_t *media_app_data;
static wiced_bt_ga_mcp_media_control_operation_t remaining_op_handle;
const gatt_intf_service_object_t *remaining_op_profile_ptr = NULL;

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
int current_track_number = 0;
int current_codec_config = BAP_CODEC_CONFIG_16_2_1;

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define MEDIA_PLAYER_NAME "CYPRESS_PLAYER"
#define MEDIA_TRACK_TITLE "DEFAULT_TRACK"

/******************************************************************************
 * Function Name: notify_mcs_data
 *
 * Summary: send media control state to remote device through gatt notify
 *
 * Parameters:
 *  uint16_t    conn_id
 *  const gatt_intf_service_object_t *p_service
 *  mcs_characteristics_t type
 *  void *p_data
 *
 * Return:
 *  wiced_bool_t 
 *
******************************************************************************/
wiced_bool_t notify_mcs_data(uint16_t conn_id,
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

/******************************************************************************
 * Function Name: notify_media_state
 *
 * Summary: send media control state to remote device through notify_mcs_data api
 *
 * Parameters:
 *  const gatt_intf_service_object_t *p_profile
 *
 * Return:
 *  wiced_bool_t 
 *
******************************************************************************/
wiced_bool_t notify_media_state(const gatt_intf_service_object_t *p_profile)
{
    WICED_BT_TRACE("[%s] state %d ", __FUNCTION__, media_app_data->media_state);
    wiced_bool_t res = WICED_FALSE;

    for (int i = 0; i < MAX_CONNECTION_INSTANCE; i++)
    {
        if (g_unicast_source_gatt_cb.unicast_clcb[i].in_use)
        {
            le_audio_rpc_send_mcs_state_update(g_unicast_source_gatt_cb.unicast_clcb[i].conn_id,
                                               WICED_SUCCESS,
                                               media_app_data->media_state);

            notify_mcs_data(g_unicast_source_gatt_cb.unicast_clcb[i].conn_id,
                            p_profile,
                            MCS_MEDIA_STATE_CHARACTERISTIC,
                            &media_app_data->media_state);
        }
    }
    return res;
}

/******************************************************************************
 * Function Name: media_control_service_update_state
 *
 * Summary: update media control service state to remote device
 *
 * Parameters:
 *  const gatt_intf_service_object_t *p_profile
 *  wiced_bt_ga_media_control_state_t state
 *
 * Return:
 *  wiced_bool_t 
 *
******************************************************************************/
wiced_bool_t media_control_service_update_state(const gatt_intf_service_object_t *p_profile,
                                                wiced_bt_ga_media_control_state_t state)
{
    if (media_app_data->media_state != state)
    {
        media_app_data->media_state = state;
        return notify_media_state(p_profile);
    }
    return WICED_TRUE;
}

/******************************************************************************
 * Function Name: media_control_service_handle_play
 *
 * Summary: update media control service state
 *
 * Parameters:
 *  const gatt_intf_service_object_t *p_profile
 *
 * Return:
 *  wiced_bt_ga_mcp_result_t
 *
******************************************************************************/
wiced_bt_ga_mcp_result_t media_control_service_handle_play(const gatt_intf_service_object_t *p_profile)
{
    media_control_service_update_state(p_profile, WICED_BT_GA_MCS_MEDIA_PLAYING);
    WICED_BT_TRACE("Media State WICED_BT_GA_MCS_MEDIA_PLAYING \n");
    return WICED_BT_GA_MCS_SUCCESS;
}

/******************************************************************************
 * Function Name: media_control_service_handle_pause
 *
 * Summary: update media control service state
 *
 * Parameters:
 *  const gatt_intf_service_object_t *p_profile
 *
 * Return:
 *  wiced_bt_ga_mcp_result_t 
 *
******************************************************************************/
wiced_bt_ga_mcp_result_t media_control_service_handle_pause(const gatt_intf_service_object_t *p_profile)
{
    media_control_service_update_state(p_profile, WICED_BT_GA_MCS_MEDIA_PAUSED);
    WICED_BT_TRACE("Media State WICED_BT_MCS_MEDIA_PAUSED \n");
    return WICED_BT_GA_MCS_SUCCESS;
}

/******************************************************************************
 * Function Name: media_control_service_handle_stop
 *
 * Summary: update media control service state
 *
 * Parameters:
 *  const gatt_intf_service_object_t *p_profile
 *
 * Return:
 *  wiced_bt_ga_mcp_result_t 
 *
******************************************************************************/
wiced_bt_ga_mcp_result_t media_control_service_handle_stop(const gatt_intf_service_object_t *p_profile)
{
    if (p_profile)
        media_control_service_update_state(p_profile, WICED_BT_GA_MCS_MEDIA_PAUSED);
    else
        media_app_data->media_state = WICED_BT_GA_MCS_MEDIA_PAUSED;

    WICED_BT_TRACE("Media State WICED_BT_GA_MCS_MEDIA_STOPPED \n");
    return WICED_BT_GA_MCS_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_source_is_streaming
 *
 * Summary: check is streaming
 *
 * Parameters:
 *  None
 *
 * Return:
 *  wiced_bool_t 
 *
******************************************************************************/
wiced_bool_t unicast_source_is_streaming(void)
{
    return media_app_data->media_state == WICED_BT_GA_MCS_MEDIA_PLAYING ? WICED_TRUE : WICED_FALSE;
}

/******************************************************************************
 * Function Name: unicast_source_start_streaming
 *
 * Summary: prepare streaming config and use cap api to start streaming
 *
 * Parameters:
 *  uint16_t conn_id
 *  uint32_t codec_config
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_start_streaming(uint16_t conn_id, uint32_t codec_config)
{
    unicast_source_stream_config_t unicast_stream_config;
    wiced_bt_ga_bap_stream_config_t stream_config1;
    unicast_source_device_config_t config1;

    wiced_bt_ga_bap_get_stream_config(codec_config,
                                      &stream_config1);

    WICED_BT_TRACE("unicast_source_start_streaming conn_id:%x octets_per_codec_frame:%d\n", conn_id, stream_config1.octets_per_codec_frame);

    config1.audio_location = BAP_AUDIO_LOCATION_FRONT_CENTER;
    config1.conn_id = conn_id;
    stream_config1.audio_channels = BAP_AUDIO_LOCATION_FRONT_LEFT | BAP_AUDIO_LOCATION_FRONT_RIGHT;
    unicast_stream_config.stream_config = &stream_config1;
    unicast_stream_config.num_devices = 1;
    unicast_stream_config.config_list = &config1;

    unicast_source_cap_start_streaming(&unicast_stream_config);
    current_codec_config = codec_config;
}



/******************************************************************************
 * Function Name: unicast_source_handle_streaming_operation
 *
 * Summary: media control service handle operation
 *
 * Parameters:
 *  const gatt_intf_service_object_t *p_profile
 *  wiced_bt_ga_mcp_media_control_operation_t opcode
 *  uint16_t conn_id
 *  uint32_t codec_config
 *
 * Return:
 *  wiced_bt_ga_mcp_result_t 
 *
******************************************************************************/
wiced_bt_ga_mcp_result_t unicast_source_handle_streaming_operation(const gatt_intf_service_object_t *p_profile,
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

            unicast_source_start_streaming(conn_id, codec_config);
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
            unicast_source_cap_stop_streaming(conn_id);
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

/******************************************************************************
 * Function Name: media_control_service_clear_operation
 *
 * Summary: media control service clean operation state
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
******************************************************************************/
void media_control_service_clear_operation(void)
{
    remaining_op_handle = WICED_BT_GA_MCS_INVALID;
    remaining_op_profile_ptr = NULL;
}

/******************************************************************************
 * Function Name: unicast_source_mcs_is_streaming
 *
 * Summary: check if source is streaming
 *
 * Parameters:
 *  None
 *
 * Return:
 *  wiced_bool_t 
 *
******************************************************************************/
wiced_bool_t unicast_source_mcs_is_streaming(void)
{
    if (media_app_data->media_state == WICED_BT_GA_MCS_MEDIA_PLAYING)
        return WICED_TRUE;
    else
        return WICED_FALSE;
}

// Once actual media start/stops call this API to send notification
/******************************************************************************
 * Function Name: unicast_source_mcs_handle_post_operation
 *
 * Summary: handle event of media control service
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_mcs_handle_post_operation(void)
{
    if (remaining_op_profile_ptr == NULL) return;

    switch (remaining_op_handle)
    {
        case WICED_BT_GA_MCS_PLAY:
            media_control_service_handle_play(remaining_op_profile_ptr);
            break;
        case WICED_BT_GA_MCS_PAUSE:
            media_control_service_handle_pause(remaining_op_profile_ptr);
            unicast_source_gatt_handle_disconnecting_state();
            break;
        case WICED_BT_GA_MCS_STOP:
            media_control_service_handle_stop(remaining_op_profile_ptr);
            break;
        default:
            break;
    }
    media_control_service_clear_operation();
}

/******************************************************************************
 * Function Name: media_control_service_handle_event
 *
 * Summary: handle event of media control service
 *
 * Parameters:
 *  const gatt_intf_service_object_t *p_profile
 *  wiced_bt_ga_mcs_data_t *p_data
 *
 * Return:
 *  wiced_bt_ga_mcp_result_t 
 *
******************************************************************************/
wiced_bt_ga_mcp_result_t media_control_service_handle_event(const gatt_intf_service_object_t *p_profile,
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

/******************************************************************************
 * Function Name: unicast_source_mcs_initialize_data
 *
 * Summary: media control service initialize
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_mcs_initialize_data(void)
{
    gatt_intf_service_object_t *g_profile = g_unicast_source_gatt_cb.local_profiles.p_gmcs;
    gatt_intf_service_object_t *m_profile = g_unicast_source_gatt_cb.local_profiles.p_mcs;

    WICED_BT_TRACE("[%s] ", __FUNCTION__);

    media_app_data = &g_unicast_source_gatt_cb.mcs_data;

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

/******************************************************************************
 * Function Name: unicast_source_handle_mcs_data
 *
 * Summary: handle data of media control service
 *
 * Parameters:
 *  uint16_t conn_id
 *  void *p_app_ctx
 *  const gatt_intf_service_object_t *p_profile
 *  wiced_bt_gatt_status_t status
 *  uint32_t evt_type
 *  gatt_intf_attribute_t *p_char
 *  wiced_bt_ga_mcs_data_t *p_evt_data
 *  int     len
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_source_handle_mcs_data(uint16_t conn_id,
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
                result = media_control_service_handle_event(p_profile, p_evt_data);

                if (result == WICED_BT_GA_MCS_SUCCESS)
                    result = unicast_source_handle_streaming_operation(p_profile,
                                                                       p_evt_data->control_point_operation.opcode,
                                                                       conn_id,
                                                                       current_codec_config);

                p_evt_data->control_point_operation.result = result;
                break;
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

/******************************************************************************
 * Function Name: unicast_source_mcs_callback
 *
 * Summary: media control service callback function, handle READ/WRITE REQUEST
 *
 * Parameters:
 *  uint16_t conn_id
 *  void *p_app_ctx
 *  const gatt_intf_service_object_t *p_service
 *  wiced_bt_gatt_status_t status
 *  uint32_t evt_type
 *  gatt_intf_attribute_t *p_char
 *  void *p_data
 *  int len
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_source_mcs_callback(uint16_t conn_id,
                                           void *p_app_ctx,
                                           const gatt_intf_service_object_t *p_service,
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
            return unicast_source_handle_mcs_data(conn_id,
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

/******************************************************************************
 * Function Name: unicast_source_mcs_play
 *
 * Summary: media control service play function
 *
 * Parameters:
 *  uint16_t conn_id
 *  uint32_t codec_config
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_source_mcs_play(uint16_t conn_id,
                                       uint32_t codec_config)
{
    wiced_bt_ga_mcs_data_t data;
    data.control_point_operation.opcode = WICED_BT_GA_MCS_PLAY;
    WICED_BT_TRACE("[%s] conn_id 0x%x  \n", __FUNCTION__, conn_id);
    wiced_bt_ga_mcp_result_t status =
        media_control_service_handle_event(g_unicast_source_gatt_cb.local_profiles.p_gmcs, &data);


    if (status == WICED_BT_GA_MCS_SUCCESS)
    {
        for (int i = 0; i < MAX_CONNECTION_INSTANCE; i++)
        {
            if (g_unicast_source_gatt_cb.unicast_clcb[i].in_use)
            {
                unicast_source_handle_streaming_operation(g_unicast_source_gatt_cb.local_profiles.p_gmcs,
                                                          WICED_BT_GA_MCS_PLAY,
                                                          g_unicast_source_gatt_cb.unicast_clcb[i].conn_id,
                                                          codec_config);
            }
        }
    }
    return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_source_mcs_pause
 *
 * Summary: media control service pause function
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_source_mcs_pause(uint16_t conn_id)
{
    wiced_bt_ga_mcs_data_t data;
    data.control_point_operation.opcode = WICED_BT_GA_MCS_PAUSE;
    WICED_BT_TRACE("[%s] conn_id 0x%x \n", __FUNCTION__, conn_id);
    wiced_bt_ga_mcp_result_t status =
        media_control_service_handle_event(g_unicast_source_gatt_cb.local_profiles.p_gmcs, &data);

    if (status == WICED_BT_GA_MCS_SUCCESS)
    {
        if (status == WICED_BT_GA_MCS_SUCCESS)
        {
            for (int i = 0; i < MAX_CONNECTION_INSTANCE; i++)
            {
                if (g_unicast_source_gatt_cb.unicast_clcb[i].in_use)
                {
                    unicast_source_handle_streaming_operation(g_unicast_source_gatt_cb.local_profiles.p_gmcs,
                                                              WICED_BT_GA_MCS_PAUSE,
                                                              g_unicast_source_gatt_cb.unicast_clcb[i].conn_id,
                                                              -1);
                }
            }
        }
    }
    return WICED_SUCCESS;
}
