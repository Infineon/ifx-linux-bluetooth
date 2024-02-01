/**
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company)
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
 *
 * Bluetooth Wiced AVRC Remote Control Target interface
 *
 */

#include "wiced_bt_avrc_tg.h"
#include "wiced_bt_avrc_tg_int.h"
#include "string.h"
#include "wiced_memory.h"
#include "wiced_bt_avrc_defs.h"

/******************************************************************************
 *                           Constants
 ******************************************************************************/
#define AVCT_MIN_CONTROL_MTU        48      /* Per the AVRC spec, minimum MTU for the control channel */
#define AVCT_CONTROL_MTU            256     /* MTU for the control channel */
#define AVCT_MIN_BROWSE_MTU         335     /* Per the AVRC spec, minimum MTU for the browsing channel */

#define SDP_DB_LEN                  400

#define APP_AVRC_TEMP_BUF           128
#define GET_ELMENT_ATTR_HDR         8 /* Attr_id(4)+ charsetID(2) +attr_len(2) */
#define BTAVRCP_TRACE_DEBUG
#define AVRC_TG_PLAYER_ID           1
#define AVRC_TG_PLAYER_NAME "Player"

#ifdef BTAVRCP_TRACE_DEBUG
#define WICED_BTAVRCP_TRACE WICED_BT_TRACE
#else
#define WICED_BTAVRCP_TRACE(...)
#endif

/******************************************************************************
 *                         Variable Definitions
 ******************************************************************************/

static wiced_bt_avrc_tg_cb_t wiced_bt_avrc_tg_cb;
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern wiced_bt_avrc_sts_t wiced_avrc_build_and_send_metadata_cmd(void* p_cmd, uint8_t ctype, uint8_t handle, uint8_t label);
extern wiced_bt_avrc_sts_t wiced_bt_avrc_build_browse_rsp (void *p_rsp, wiced_bt_avrc_xmit_buf_t **p_rspbuf);
extern void AVCT_Register(uint16_t mtu, uint16_t mtu_br, uint8_t sec_mask);
wiced_result_t wiced_bt_avrc_tg_open(wiced_bt_device_address_t peer_addr);
wiced_bt_device_address_t null_bda = {0,0,0,0,0,0};
void wiced_bt_avrc_tg_handle_get_total_num_of_items(uint8_t handle, uint8_t label, wiced_bt_avrc_browse_get_num_of_items_cmd_t * p_command, wiced_bt_avrc_xmit_buf_t **pp_rsp );
extern wiced_bt_avrc_sts_t wiced_avrc_build_metadata_rsp(void* p_rsp, wiced_bt_avrc_xmit_buf_t** p_rspbuf);
extern wiced_bt_avrc_sts_t wiced_avrc_build_and_send_metadata_rsp(void* p_rsp, uint8_t ctype, uint8_t handle, uint8_t label);
extern  wiced_bt_avrc_sts_t wiced_avrc_build_and_send_browse_rsp (uint8_t handle, uint8_t label, void *p_rsp, wiced_bt_avrc_xmit_buf_t **p_rspbuf);
extern wiced_bt_avrc_sts_t wiced_avrc_build_and_send_browse_cmd (uint8_t handle, void *p_rsp, wiced_bt_avrc_xmit_buf_t **p_rspbuf);
int wiced_bt_avrc_tg_get_uid_counter(uint8_t scope);
extern wiced_bt_avrc_sts_t *wiced_bt_avrc_build_rejected_rsp(wiced_bt_avrc_xmit_buf_t **p_rspbuf, uint8_t pdu, uint8_t status);

/******************************************************************************
 *  Application data
 ******************************************************************************/
#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED

wiced_bt_avrc_tg_player_attr_t player_settings[APP_AVRC_SETTING_SUPPORTED_MAX];

wiced_bt_avrc_app_setting_text_t app_setting_attr_text[] = {
    {
    .attr_id = AVRC_PLAYER_SETTING_REPEAT,
    .name.charset_id = AVRC_CHARSET_ID_UTF8,
    .name.name.str_len = 0x06,
    .name.name.p_str = (uint8_t *)"REPEAT"
    },
     {
    .attr_id = AVRC_PLAYER_SETTING_SHUFFLE,
    .name.charset_id = AVRC_CHARSET_ID_UTF8,
    .name.name.str_len = 0x07,
    .name.name.p_str = (uint8_t *)"SHUFFLE"
    },
    {
    .attr_id = AVRC_PLAYER_SETTING_EQUALIZER,
    .name.charset_id = AVRC_CHARSET_ID_UTF8,
    .name.name.str_len = 0x09,
    .name.name.p_str = (uint8_t *)"EQUALIZER"
    },
};

wiced_bt_avrc_app_setting_text_t equalizer_value_txt[] = {
     {
    .attr_id = AVRC_PLAYER_VAL_OFF,
    .name.charset_id = AVRC_CHARSET_ID_UTF8,
    .name.name.str_len = 0x03,
    .name.name.p_str = (uint8_t *)"OFF"
    },
    {
    .attr_id = AVRC_PLAYER_VAL_ON,
    .name.charset_id = AVRC_CHARSET_ID_UTF8,
    .name.name.str_len = 0x0b,
    .name.name.p_str = (uint8_t *)"ON"
    },
};

wiced_bt_avrc_app_setting_text_t shuffle_value_txt[] = {
     {
    .attr_id = AVRC_PLAYER_VAL_OFF,
    .name.charset_id = AVRC_CHARSET_ID_UTF8,
    .name.name.str_len = 0x03,
    .name.name.p_str = (uint8_t *)"OFF"
    },
    {
    .attr_id = AVRC_PLAYER_VAL_ALL_SHUFFLE,
    .name.charset_id = AVRC_CHARSET_ID_UTF8,
    .name.name.str_len = 0x0b,
    .name.name.p_str = (uint8_t *)"ALL SHUFFLE"
    },
};

wiced_bt_avrc_app_setting_text_t repeat_value_txt[] = {
     {
    .attr_id = AVRC_PLAYER_VAL_OFF,
    .name.charset_id = AVRC_CHARSET_ID_UTF8,
    .name.name.str_len = 0x03,
    .name.name.p_str =(uint8_t *) "OFF"
    },
    {
    .attr_id = AVRC_PLAYER_VAL_ON,
    .name.charset_id = AVRC_CHARSET_ID_UTF8,
    .name.name.str_len = 0x02,
    .name.name.p_str = (uint8_t *)"ON"
    },
};

#endif

const uint32_t  app_avrc_meta_caps_co_ids[] = {
    AVRC_CO_METADATA,
    AVRC_CO_BROADCOM
};

/* supported events */
const uint8_t  app_avrc_meta_caps_evt_ids[] = {
#ifdef APP_AVRC_PLAY_STATUS_SUPPORTED
    AVRC_EVT_PLAY_STATUS_CHANGE,
#endif
#ifdef APP_AVRC_TRACK_INFO_SUPPORTED
    AVRC_EVT_TRACK_CHANGE,
#endif
#ifdef APP_AVRC_TRACK_REACHED_END_SUPPORTED
    AVRC_EVT_TRACK_REACHED_END,
#endif
#ifdef APP_AVRC_TRACK_REACHED_START_SUPPORTED
    AVRC_EVT_TRACK_REACHED_START,
#endif
#ifdef APP_AVRC_TRACK_PLAY_POS_CHANGE_SUPPORTED
    AVRC_EVT_PLAY_POS_CHANGED,
#endif
#ifdef APP_AVRC_BATTERY_STATUS_SUPPORTED
    AVRC_EVT_BATTERY_STATUS_CHANGE,
#endif
#ifdef APP_AVRC_SYSTEM_STATUS_SUPPORTED
    AVRC_EVT_SYSTEM_STATUS_CHANGE,
#endif
#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
    AVRC_EVT_APP_SETTING_CHANGE,
#endif
    AVRC_EVT_VOLUME_CHANGE,
};

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

#ifdef BTAVRCP_TRACE_DEBUG
/*******************************************************************************
* Function        wiced_bt_avrc_tg_dump_message_type

** Description    trace messages
*******************************************************************************/
static const char *wiced_bt_avrc_tg_dump_message_type(uint8_t ctype)
{
    switch((int)ctype)
    {
    CASE_RETURN_STR(AVRC_CMD_CTRL)
    CASE_RETURN_STR(AVRC_CMD_STATUS)
    CASE_RETURN_STR(AVRC_CMD_SPEC_INQ)
    CASE_RETURN_STR(AVRC_CMD_NOTIF)
    CASE_RETURN_STR(AVRC_CMD_GEN_INQ)
    CASE_RETURN_STR(AVRC_RSP_NOT_IMPL)
    CASE_RETURN_STR(AVRC_RSP_ACCEPT)
    CASE_RETURN_STR(AVRC_RSP_REJ)
    CASE_RETURN_STR(AVRC_RSP_IN_TRANS)
    CASE_RETURN_STR(AVRC_RSP_IMPL_STBL)
    CASE_RETURN_STR(AVRC_RSP_CHANGED)
    CASE_RETURN_STR(AVRC_RSP_INTERIM)
    }

    return NULL;
}
#endif

/*******************************************************************************
* Function        wiced_bt_avrc_tg_is_peer_absolute_volume_capable

** Description    return non zero if peer is absolute volume capable
*******************************************************************************/
uint8_t wiced_bt_avrc_tg_is_peer_absolute_volume_capable( void )
{
    return (uint8_t)wiced_bt_avrc_tg_cb.is_abs_volume_capable;
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_get_capabilities

** Description    getcap message sent to target (if supported).
*******************************************************************************/
static void wiced_bt_avrc_tg_get_capabilities(void)
{
    wiced_bt_avrc_metadata_cmd_t cmd;

    WICED_BTAVRCP_TRACE( "%s: avrc_hdl=0x%x \n\r", __FUNCTION__, wiced_bt_avrc_tg_cb.avrc_handle );

    /* Can't send request on a bad handle */
    if (wiced_bt_avrc_tg_cb.avrc_handle == INVALID_AVRC_HANDLE) return;

    /* Build and send GetCapabilities command if we know  the remote is AVRCP Target 1.4 capable */
    cmd.metadata_hdr.pdu     = AVRC_PDU_GET_CAPABILITIES;
    cmd.u.get_caps.capability_id  = AVRC_CAP_EVENTS_SUPPORTED;


    uint8_t  label = 1; /* TODO: Need to do transaction label accounting. */
    if (wiced_avrc_build_and_send_metadata_cmd(&cmd, AVRC_CMD_STATUS, wiced_bt_avrc_tg_cb.avrc_handle, label) != AVRC_STS_NO_ERROR)
    {
        /* TODO: Message not sent. Need to release the label */
    }


}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_sdp_find_service_callback

** Description    Handler for callback for SDP find services
*******************************************************************************/
#define AVRC_FIND_SERVICE_UUID  UUID_SERVCLASS_AV_REM_CTRL_TARGET
void wiced_bt_avrc_tg_sdp_find_service_callback (uint16_t sdp_result)
{
    wiced_bt_sdp_discovery_record_t *p_rec = NULL;
    wiced_bt_sdp_discovery_attribute_t *p_sdp_discovery_attribute;

    wiced_bt_rc_event_t avrc_event;

    WICED_BTAVRCP_TRACE("%s: sdp_result: %d\n\r", __FUNCTION__, sdp_result);

    memcpy( avrc_event.bd_addr, wiced_bt_avrc_tg_cb.peer_addr, BD_ADDR_LEN );
    avrc_event.handle = wiced_bt_avrc_tg_cb.avrc_handle;
    avrc_event.attribute_search_completed = WICED_FALSE;

    if (sdp_result == WICED_BT_SDP_SUCCESS)
    {
        /* loop through all records we found */
        do
        {
            /* get next record; if none found, we're done */
            if ((p_rec = wiced_bt_sdp_find_service_in_db(wiced_bt_avrc_tg_cb.p_sdp_db_avrc, AVRC_FIND_SERVICE_UUID, p_rec)) == NULL)
            {
                WICED_BTAVRCP_TRACE("%s: No Target Record found", __FUNCTION__);

                if(wiced_bt_avrc_tg_cb.p_event_cb)
                    (wiced_bt_avrc_tg_cb.p_event_cb)(APP_AVRC_EVENT_DEVICE_CONNECTED, &avrc_event);

                break;
            }

            /* Get the version of the AVRCPP profile the peer has implemented. */
            if ( wiced_bt_sdp_find_attribute_in_rec (p_rec, ATTR_ID_BT_PROFILE_DESC_LIST) != NULL )
            {
                wiced_bool_t got_version = wiced_bt_sdp_find_profile_version_in_rec (p_rec, UUID_SERVCLASS_AV_REMOTE_CONTROL, &wiced_bt_avrc_tg_cb.peer_avrcp_version);
                if (got_version)
                {
                    WICED_BTAVRCP_TRACE("%s: AVRCP Version: 0x%x", __FUNCTION__, wiced_bt_avrc_tg_cb.peer_avrcp_version);

                    if (wiced_bt_avrc_tg_cb.peer_avrcp_version >= AVRC_REV_1_4)
                    {
                        /* Call Getcaps to determine if the remote is Abs. Volume capable*/
                        wiced_bt_avrc_tg_get_capabilities();
                    }
                }
            }

            /* Get the Supported Features Attribute */
            if ( (p_sdp_discovery_attribute = wiced_bt_sdp_find_attribute_in_rec (p_rec, ATTR_ID_SUPPORTED_FEATURES)) != NULL )
            {
                if( (SDP_DISC_ATTR_TYPE(p_sdp_discovery_attribute->attr_len_type) == UINT_DESC_TYPE) &&
                                (SDP_DISC_ATTR_LEN(p_sdp_discovery_attribute->attr_len_type) == 2) )
                {
                    avrc_event.supported_features = p_sdp_discovery_attribute->attr_value.v.u16;
                    avrc_event.attribute_search_completed = WICED_TRUE;
                }
            }

            /* SDP complete, send device connected event to app */
            if(wiced_bt_avrc_tg_cb.p_event_cb)
                (wiced_bt_avrc_tg_cb.p_event_cb)(APP_AVRC_EVENT_DEVICE_CONNECTED, &avrc_event);

            /* we've got everything we need. we're done */
            break;

        } while (TRUE);
    }

    /* Record information, if it exists, has been extracted. Free the DB */
    wiced_bt_free_buffer(wiced_bt_avrc_tg_cb.p_sdp_db_avrc);
    wiced_bt_avrc_tg_cb.p_sdp_db_avrc = NULL;
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_sdp_find_service

** Description     AV RC SDP record request
*******************************************************************************/
wiced_bool_t wiced_bt_avrc_tg_sdp_find_service( wiced_bt_device_address_t bd_addr)
{
    wiced_bt_uuid_t uuid_list;
    wiced_bool_t    result = WICED_TRUE;

    uint16_t  avc_attr_list[] = {ATTR_ID_SERVICE_CLASS_ID_LIST,
                               ATTR_ID_BT_PROFILE_DESC_LIST,
                               ATTR_ID_SUPPORTED_FEATURES};

    WICED_BTAVRCP_TRACE("%s: uuid: %x\n\r", __FUNCTION__, AVRC_FIND_SERVICE_UUID);

    if ( wiced_bt_avrc_tg_cb.p_sdp_db_avrc == NULL )
    {
        /* Allocate memory for the discovery database */
        wiced_bt_avrc_tg_cb.p_sdp_db_avrc = ( wiced_bt_sdp_discovery_db_t * ) wiced_bt_get_buffer( SDP_DB_LEN );
        if ( wiced_bt_avrc_tg_cb.p_sdp_db_avrc == NULL )
        {
            WICED_BTAVRCP_TRACE("%s: Failed to allocate SDP request DB\n\r", __FUNCTION__);
            return WICED_FALSE;
        }
    }
    else
    {
        WICED_BTAVRCP_TRACE("%s: SDP request already in progress\n\r", __FUNCTION__);
        return WICED_FALSE;
    }

    /* set up discovery database */
    uuid_list.len       = LEN_UUID_16;
    uuid_list.uu.uuid16 = AVRC_FIND_SERVICE_UUID;

    result = wiced_bt_sdp_init_discovery_db ( wiced_bt_avrc_tg_cb.p_sdp_db_avrc, SDP_DB_LEN,
                                              1, &uuid_list,
                                              sizeof_array(avc_attr_list), avc_attr_list);

    if (result == WICED_TRUE)
    {
        /* perform service search */
        result = wiced_bt_sdp_service_search_attribute_request(bd_addr, wiced_bt_avrc_tg_cb.p_sdp_db_avrc, wiced_bt_avrc_tg_sdp_find_service_callback);
        WICED_BTAVRCP_TRACE("%s: calling wiced_bt_sdp_service_search_attribute_request: result %d\n\r", __FUNCTION__, result);
    }

    return result;
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_register_for_notification

** Description    register for notifications for the specified event ID
*******************************************************************************/
void wiced_bt_avrc_tg_register_for_notification( uint8_t event_id )
{
    wiced_bt_avrc_metadata_cmd_t cmd;

    WICED_BTAVRCP_TRACE("%s: event_id %d handle %d\n\r", __FUNCTION__, event_id, wiced_bt_avrc_tg_cb.avrc_handle);

    if ( wiced_bt_avrc_tg_cb.avrc_handle != INVALID_AVRC_HANDLE )
    {
        /* Build and send notification registration command */
        cmd.metadata_hdr.pdu                = AVRC_PDU_REGISTER_NOTIFICATION;
        cmd.u.reg_notif.event_id = event_id;
        cmd.u.reg_notif.playback_interval    = 0;

        uint8_t label = 5; /* TODO: Need to do transaction label accounting. */
        if (wiced_avrc_build_and_send_metadata_cmd(&cmd, AVRC_CMD_NOTIF, wiced_bt_avrc_tg_cb.avrc_handle, label) != AVRC_STS_NO_ERROR)
        {
            /* TODO: Message not sent. Need to release the label */
        }

    }
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_register_absolute_volume_change

** Description    register for absolute volume change notifications
*******************************************************************************/
void wiced_bt_avrc_tg_register_absolute_volume_change(void)
{
    wiced_bt_avrc_tg_register_for_notification( AVRC_EVT_VOLUME_CHANGE );
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_update_abs_volume

** Description    Peer sent absoute volume, update the MCU app
*******************************************************************************/
static void wiced_bt_avrc_tg_update_abs_volume(uint8_t abs_volume)
{
    wiced_bt_rc_event_t avrc_event;

    // Ignore bit 7 as it is reserved for future use.
    abs_volume &= 0x7F;

    /* If the volume percentage has changed since the last update, send current value up. */
    if (wiced_bt_avrc_tg_cb.last_abs_volume != abs_volume)
    {
        wiced_bt_avrc_tg_cb.last_abs_volume = abs_volume;
        /* Pass the Absolute Volume to the app if there is an application event callback registered */
        if(wiced_bt_avrc_tg_cb.p_event_cb)
        {
            avrc_event.absolute_volume.handle = wiced_bt_avrc_tg_cb.avrc_handle;
            avrc_event.absolute_volume.volume = (uint8_t)( ((abs_volume * 100) + 50) / MAX_AVRCP_VOLUME_LEVEL);
            wiced_bt_avrc_tg_cb.p_event_cb(APP_AVRC_EVENT_ABS_VOL_CHANGED, &avrc_event);
        }
    }
}


wiced_bt_avrc_sts_t wiced_bt_avrc_get_suported_app_setting(uint8_t idx, wiced_bt_avrc_app_setting_t *p_app_setting)
{
    if(idx <APP_AVRC_SETTING_SUPPORTED_MAX)
    {
        p_app_setting->attr_id = player_settings[idx].attr_id;
        p_app_setting->attr_val = player_settings[idx].num_val;
    }
    else
    {
        return AVRC_STS_BAD_PARAM;
    }
    return AVRC_STS_NO_ERROR;
}

wiced_bt_avrc_sts_t wiced_bt_add_app_supported_settings(wiced_bt_avrc_app_setting_t *p_attr, wiced_bt_avrc_xmit_buf_t *p_rspbuf)
{
    uint8_t *p_data, *p_start;
    p_start = p_data  = &p_rspbuf->payload[p_rspbuf->len_used];
    if(avrc_is_valid_player_attrib_value(p_attr->attr_id, p_attr->attr_val))
    {
        UINT8_TO_BE_STREAM(p_data, p_attr->attr_id);
        UINT8_TO_BE_STREAM(p_data, p_attr->attr_val);
    }
    else
        return AVRC_STS_BAD_PARAM;

    p_rspbuf->len_used += (uint16_t)(p_data - p_start);
    return AVRC_STS_NO_ERROR;

}


void wiced_bt_avrc_get_app_attr_txt(uint8_t attr_id, void **p_attr)
{
    switch (attr_id)
    {
    case AVRC_PLAYER_SETTING_REPEAT:
        {
            *p_attr = &app_setting_attr_text[0];
        }
    break;
    case AVRC_PLAYER_SETTING_SHUFFLE:
        {
            *p_attr = &app_setting_attr_text[1];
        }
    break;
    case AVRC_PLAYER_SETTING_EQUALIZER:
        {
            *p_attr = &app_setting_attr_text[2];
        }
    break;
    default :
    break;
    }
}

void wiced_bt_avrc_get_app_attr_value_text(uint8_t attr_id, uint8_t attr_value, void **p_attr)
{
    *p_attr = NULL; /*initialize  to NULL*/
    switch (attr_id)
    {
    case AVRC_PLAYER_SETTING_REPEAT:
    {
        if (attr_value == AVRC_PLAYER_VAL_OFF)
            *p_attr = &repeat_value_txt[0];
        if (attr_value == AVRC_PLAYER_VAL_ON)
            *p_attr = &repeat_value_txt[1];
    }
    break;
    case AVRC_PLAYER_SETTING_SHUFFLE:
    {
        if (attr_value == AVRC_PLAYER_VAL_OFF)
            *p_attr = &shuffle_value_txt[0];
        if (attr_value == AVRC_PLAYER_VAL_ALL_SHUFFLE)
            *p_attr = &shuffle_value_txt[1];

    }
    break;
    case AVRC_PLAYER_SETTING_EQUALIZER:
    {
        if (attr_value == AVRC_PLAYER_VAL_OFF)
            *p_attr = &equalizer_value_txt[0];
        if (attr_value == AVRC_PLAYER_VAL_ON)
            *p_attr = &equalizer_value_txt[1];
    }
    break;
    default:
        break;
    }
}

wiced_bt_avrc_sts_t wiced_bt_add_vendor_get_player_app_attr_text_rsp(wiced_bt_avrc_app_setting_text_t  *p_attr,  wiced_bt_avrc_xmit_buf_t *p_rspbuf)
{
    uint8_t *p_data, *p_start;
    uint16_t  remaining = p_rspbuf->buffer_size;
    p_start = p_data  = &p_rspbuf->payload[p_rspbuf->len_used];

    remaining = p_rspbuf->buffer_size + p_rspbuf->len_used;

    if  (remaining < (p_attr->name.name.str_len + 4))
    {
        WICED_BTAVRCP_TRACE ("avrc_bld_app_setting_text_rsp out of room (str_len:%d, left:%d)",
           p_attr->name.name.str_len, remaining);
        return AVRC_STS_INTERNAL_ERR;
    }
    if ( !p_attr->name.name.str_len || !p_attr->name.name.p_str )
    {
        WICED_BTAVRCP_TRACE ("avrc_bld_app_setting_text_rsp NULL attr text");
        return AVRC_STS_INTERNAL_ERR;
    }
    UINT8_TO_BE_STREAM(p_data, p_attr->attr_id);
    UINT16_TO_BE_STREAM(p_data, p_attr->name.charset_id);
    UINT8_TO_BE_STREAM(p_data, p_attr->name.name.str_len);
    ARRAY_TO_BE_STREAM(p_data, p_attr->name.name.p_str, p_attr->name.name.str_len);
    p_rspbuf->len_used += (uint16_t)(p_data - p_start);
    return AVRC_STS_NO_ERROR;
}

wiced_bt_avrc_sts_t wiced_bt_add_vendor_get_player_app_attr_value_text_rsp(wiced_bt_avrc_app_setting_text_t  *p_attr,  wiced_bt_avrc_xmit_buf_t *p_rspbuf)
{
    uint8_t *p_data, *p_start;
    uint16_t  remaining = p_rspbuf->buffer_size;
    p_start = p_data  = &p_rspbuf->payload[p_rspbuf->len_used];
    remaining = p_rspbuf->buffer_size + p_rspbuf->len_used;

    if  (remaining < (p_attr->name.name.str_len + 4))
    {
        WICED_BTAVRCP_TRACE ("avrc_bld_app_setting_text_rsp out of room (str_len:%d, left:%d)",
           p_attr->name.name.str_len, remaining);
        return AVRC_STS_INTERNAL_ERR;
    }
    if ( !p_attr->name.name.str_len || !p_attr->name.name.p_str )
    {
        WICED_BTAVRCP_TRACE ("avrc_bld_app_setting_text_rsp NULL attr text");
        return AVRC_STS_INTERNAL_ERR;
    }
    UINT8_TO_BE_STREAM(p_data, p_attr->attr_id);
    UINT16_TO_BE_STREAM(p_data, p_attr->name.charset_id);
    UINT8_TO_BE_STREAM(p_data, p_attr->name.name.str_len);
    ARRAY_TO_BE_STREAM(p_data, p_attr->name.name.p_str, p_attr->name.name.str_len);
    p_rspbuf->len_used += (uint16_t)(p_data - p_start);
    return AVRC_STS_NO_ERROR;
}


wiced_bt_avrc_sts_t wiced_bt_add_vendor_get_element_attr_rsp(wiced_bt_avrc_attr_entry_t *p_attr, wiced_bt_avrc_xmit_buf_t *p_rspbuf)
{

    uint8_t    *p_data, *p_start;

    WICED_BTAVRCP_TRACE("[%s]", __FUNCTION__);

    if (!p_attr)
    {
        WICED_BTAVRCP_TRACE("avrc_bld_get_elem_attrs_rsp NULL parameter");
     return AVRC_STS_BAD_PARAM;
    }

    /* get the existing length, if any, and also the num attributes */
    p_start = p_data = &p_rspbuf->payload[p_rspbuf->len_used];

     if (!AVRC_IS_VALID_MEDIA_ATTRIBUTE(p_attr->attr_id))
     {
         WICED_BTAVRCP_TRACE("avrc_bld_get_elem_attrs_rsp invalid attr id: %d",  p_attr->attr_id);
         return AVRC_STS_BAD_PARAM;;
     }

     if ( !p_attr->name.name.p_str )
     {
         p_attr->name.name.str_len = 0;
     }
     UINT32_TO_BE_STREAM(p_data, p_attr->attr_id);
     UINT16_TO_BE_STREAM(p_data, p_attr->name.charset_id);
     UINT16_TO_BE_STREAM(p_data, p_attr->name.name.str_len);
     ARRAY_TO_BE_STREAM(p_data, p_attr->name.name.p_str, p_attr->name.name.str_len);

    p_rspbuf->len_used += (uint16_t)(p_data - p_start);

    return AVRC_STS_NO_ERROR;

}

wiced_bt_avrc_sts_t wiced_bt_add_get_folder_items_rsp(wiced_bt_avrc_item_t *p_item, wiced_bt_avrc_xmit_buf_t *p_rspbuf)
{
    uint8_t               *p_item_start, *p_data;
    uint8_t     *p_item_len;
    uint16_t  item_len;
    uint16_t              len_left;
    wiced_bt_avrc_item_folder_t *p_folder;
    wiced_bt_avrc_item_player_t *p_player;
    //wiced_bt_avrc_item_media_t *p_media;
   // wiced_bt_avrc_attr_entry_t  *p_attr;
    wiced_bt_avrc_sts_t status =  AVRC_STS_NO_ERROR;
    p_item_start = p_data = &p_rspbuf->payload[p_rspbuf->len_used];
    len_left = p_rspbuf->buffer_size - p_rspbuf->len_used;
    UINT8_TO_BE_STREAM (p_data, p_item->item_type);
    /* variable item length - save the location to add length */
    p_item_len = p_data;
    p_data += 2;
    item_len = 0;
    len_left -= 3; /* item_type(1) + item len(2) */
    switch (p_item->item_type)
    {
    case AVRC_ITEM_PLAYER:
        /* min len required: 2 + 1 + 4 + 1 + 16 + 2 + 2 = 30 + str_len */
        p_player = &p_item->u.player;
        item_len = AVRC_FEATURE_MASK_SIZE + p_player->name.name.str_len + 12;
        if ((len_left > item_len) &&
            (p_player->major_type & AVRC_PLAYER_MAJOR_TYPE_INVALID) == 0 &&
            (p_player->sub_type & AVRC_PLAYER_SUB_TYPE_INVALID) == 0 &&
            (p_player->play_status <= AVRC_PLAYSTATE_REV_SEEK || p_player->play_status == AVRC_PLAYSTATE_ERROR))
        {
            UINT16_TO_BE_STREAM (p_data, p_player->player_id);
            UINT8_TO_BE_STREAM (p_data, p_player->major_type);
            UINT32_TO_BE_STREAM (p_data, p_player->sub_type);
            UINT8_TO_BE_STREAM (p_data, p_player->play_status);
            ARRAY_TO_BE_STREAM (p_data, p_player->features, AVRC_FEATURE_MASK_SIZE);
            UINT16_TO_BE_STREAM (p_data, p_player->name.charset_id);
            UINT16_TO_BE_STREAM (p_data, p_player->name.name.str_len);
            ARRAY_TO_BE_STREAM (p_data, p_player->name.name.p_str, p_player->name.name.str_len);
        }
        else
        {
            p_data = p_item_start;
        }
        break;

    case AVRC_ITEM_FOLDER:
        /* min len required: 8 + 1 + 1 + 2 + 2 = 14 + str_len */
        p_folder = &p_item->u.folder;
        item_len = AVRC_UID_SIZE + p_folder->name.name.str_len + 6;
        if ((len_left > item_len) &&
            p_folder->name.name.p_str &&
            p_folder->type <= AVRC_FOLDER_TYPE_YEARS &&
            p_folder->playable <= TRUE)
        {
            ARRAY_TO_BE_STREAM (p_data, p_folder->uid, AVRC_UID_SIZE);
            UINT8_TO_BE_STREAM (p_data, p_folder->type);
            UINT8_TO_BE_STREAM (p_data, p_folder->playable);
            UINT16_TO_BE_STREAM (p_data, p_folder->name.charset_id);
            UINT16_TO_BE_STREAM (p_data, p_folder->name.name.str_len);
            ARRAY_TO_BE_STREAM (p_data, p_folder->name.name.p_str, p_folder->name.name.str_len);
        }
        else
        {
            p_data = p_item_start;
        }
        break;

    case AVRC_ITEM_MEDIA:
#if 0
        /* min len required: 8 + 1 + 2 + 2 + 1 = 14 + str_len */
        p_media = &p_item.u.media;
        item_len = AVRC_UID_SIZE + p_media->name.name.str_len + 6;
        if ((len_left > item_len) &&
            p_media->name.name.p_str &&
            p_media->type <= AVRC_MEDIA_TYPE_VIDEO)
        {
            ARRAY_TO_BE_STREAM (p_data, p_media->uid, AVRC_UID_SIZE);
            UINT8_TO_BE_STREAM (p_data, p_media->type);
            UINT16_TO_BE_STREAM (p_data, p_media->name.charset_id);
            UINT16_TO_BE_STREAM (p_data, p_media->name.name.str_len);
            ARRAY_TO_BE_STREAM (p_data, p_media->name.name.p_str, p_media->name.name.str_len);
            p_attr_count = p_data++;
            count = 0;
            len_left -= item_len;
            if (p_media->attr_count>0)
            {
                p_attr = p_media->p_attr_list;
                for (yy=0; yy<p_media->attr_count && len_left > 8; yy++)
                {
                    if (p_attr[yy].name.name.p_str &&
                        AVRC_IS_VALID_MEDIA_ATTRIBUTE (p_attr[yy].attr_id) &&
                        (len_left >= (p_attr[yy].name.name.str_len + 8)))
                    {
                        count ++;
                        UINT32_TO_BE_STREAM (p_data, p_attr[yy].attr_id);
                        UINT16_TO_BE_STREAM (p_data, p_attr[yy].name.charset_id);
                        UINT16_TO_BE_STREAM (p_data, p_attr[yy].name.name.str_len);
                        ARRAY_TO_BE_STREAM (p_data, p_attr[yy].name.name.p_str, p_attr[yy].name.name.str_len);
                        item_len += (p_attr[yy].name.name.str_len + 8);
                        len_left -= (p_attr[yy].name.name.str_len + 8);
                    }
                }
            }
        }
        else
        {
            p_data = p_item_start;
        }
#endif
        break;

    } /* switch item_type */

    if (p_item_start != p_data)
    {
        /* fill in variable item length */
        UINT16_TO_BE_STREAM (p_item_len, item_len);
    }
    else
    {
        /* some item is not added properly - set an error status */
        if (len_left > item_len)
            status = AVRC_STS_INTERNAL_ERR;
        else
            status = AVRC_STS_BAD_PARAM;
    }
    p_rspbuf->len_used += (uint16_t)(p_data - p_item_start);

    return status;
}

wiced_bt_avrc_sts_t wiced_bt_add_set_browsed_player_rsp(wiced_bt_avrc_name_t *p_folder_name, wiced_bt_avrc_xmit_buf_t *p_rspbuf)
{
    uint8_t *p_start, *p_data;
    wiced_bt_avrc_sts_t sts = AVRC_STS_NO_ERROR;
    uint16_t len_left = p_rspbuf->buffer_size - p_rspbuf->len_used;
    p_start = p_data = &p_rspbuf->payload[p_rspbuf->len_used];
    if(len_left >= p_folder_name->str_len)
    {
        UINT16_TO_BE_STREAM (p_data, p_folder_name->str_len);
        ARRAY_TO_BE_STREAM(p_data, p_folder_name->p_str, p_folder_name->str_len);
        p_rspbuf->len_used += p_data - p_start;
    }
    else
        sts = AVRC_STS_NO_RESOURCES;
    
    return sts;
}

wiced_bt_avrc_sts_t wiced_bt_avrc_add_attr_to_response(uint8_t opcode, void *p_attr,  wiced_bt_avrc_xmit_buf_t *p_rspbuf)
{
    wiced_bt_avrc_sts_t status = AVRC_STS_NO_ERROR;
    switch(opcode)
    {
    case AVRC_PDU_GET_PLAYER_APP_ATTR_TEXT:
        status = wiced_bt_add_vendor_get_player_app_attr_text_rsp(p_attr, p_rspbuf);
        break;
    case AVRC_PDU_GET_PLAYER_APP_VALUE_TEXT:
        status = wiced_bt_add_vendor_get_player_app_attr_value_text_rsp(p_attr, p_rspbuf);
        break;
    case AVRC_PDU_GET_ELEMENT_ATTR:
         status = wiced_bt_add_vendor_get_element_attr_rsp(p_attr, p_rspbuf);
        break;
    case AVRC_PDU_GET_FOLDER_ITEMS:
       status = wiced_bt_add_get_folder_items_rsp(p_attr, p_rspbuf);
       break;
    case AVRC_PDU_SET_BROWSED_PLAYER:
       status = wiced_bt_add_set_browsed_player_rsp(p_attr, p_rspbuf);
       break;       
     default:
        break;
    }

    return status;

}


wiced_bt_avrc_xmit_buf_t * wiced_bt_avrc_build_app_attr_txt_rsp(uint8_t handle, wiced_bt_avrc_metadata_get_app_attr_txt_cmd_t *p_msg, wiced_bt_avrc_metadata_rsp_t *p_response)
{
    wiced_bt_avrc_xmit_buf_t *p_rsp = NULL;
    wiced_bt_avrc_sts_t sts;
    uint8_t xx = 0;
    uint8_t attr_id;
    wiced_bt_avrc_metadata_get_app_attr_txt_rsp_t attr_txt = { 0 };
    wiced_bt_avrc_app_setting_text_t *p_app_attr_txt = NULL;
    wiced_avrc_build_metadata_rsp (p_response, &p_rsp);
    if(p_rsp != NULL)
    {
        p_rsp->len_used = 1;
        attr_txt.p_val_stream = p_rsp->payload;
        for(xx =0; xx < p_msg->num_attr; xx++)
        {
           attr_id = p_msg->p_attrs[xx];
           wiced_bt_avrc_get_app_attr_txt(attr_id, (void **)&p_app_attr_txt);
           if (p_app_attr_txt == NULL)
           {
               break;
           }
           sts = wiced_bt_avrc_add_attr_to_response(AVRC_PDU_GET_PLAYER_APP_ATTR_TEXT, p_app_attr_txt, p_rsp);
            p_app_attr_txt = NULL;
            if(sts != AVRC_STS_NO_ERROR)
            {
                wiced_bt_free_buffer(p_rsp);
                return NULL;
            }
            attr_txt.num_attr++;
        }
        p_rsp->payload[0] = attr_txt.num_attr; /*At last fill num_attr*/
    }
    return p_rsp;

}


wiced_bt_avrc_xmit_buf_t * wiced_bt_avrc_build_app_attr_value_txt_rsp(uint8_t handle, wiced_bt_avrc_metadata_get_app_val_txt_cmd_t *p_msg, wiced_bt_avrc_metadata_rsp_t *p_response)
{
    wiced_bt_avrc_xmit_buf_t *p_rsp = NULL;
    wiced_bt_avrc_sts_t sts;
    uint8_t xx = 0;
    wiced_bt_avrc_metadata_get_app_attr_txt_rsp_t attr_txt = { 0 };
    wiced_bt_avrc_app_setting_text_t *p_app_attr_value_txt = NULL;
    wiced_avrc_build_metadata_rsp (p_response, &p_rsp);
    if(p_rsp != NULL)
    {
        p_rsp->len_used = 1;
        attr_txt.p_val_stream = p_rsp->payload;
        for(xx =0; xx < p_msg->num_val; xx++)
        {
            uint8_t attr_val = p_msg->p_vals[xx];
           wiced_bt_avrc_get_app_attr_value_text(p_msg->attr_id, attr_val, (void **)&p_app_attr_value_txt);
           if (p_app_attr_value_txt == NULL)
           {
               break;
           }

           sts = wiced_bt_avrc_add_attr_to_response(AVRC_PDU_GET_PLAYER_APP_VALUE_TEXT, p_app_attr_value_txt, p_rsp);
           p_app_attr_value_txt = NULL;
            if(sts != AVRC_STS_NO_ERROR)
            {
                wiced_bt_free_buffer(p_rsp);
                return NULL;
            }
            attr_txt.num_attr++;
        }
        p_rsp->payload[0] = attr_txt.num_attr; /*At last fill num_attr*/
    }
    return p_rsp;

}


/*******************************************************************************
* Function        wiced_bt_avrc_tg_handle_abs_volume_response

** Description    Absolute volume response handler
*******************************************************************************/
static void wiced_bt_avrc_tg_handle_abs_volume_response( uint8_t response_type, uint8_t volume)
{
    switch(response_type)
    {
        case AVRC_RSP_INTERIM:
            /* Update the value sent up to the app if changed */
            wiced_bt_avrc_tg_update_abs_volume(volume);
            break;

        case AVRC_RSP_CHANGED:
            /* Update the value sent up to the app */
            wiced_bt_avrc_tg_update_abs_volume(volume);

            /* Resubmit the registration for the next event. */
            wiced_bt_avrc_tg_register_for_notification(AVRC_EVT_VOLUME_CHANGE);
            break;

        case AVRC_RSP_REJ:
            /* Resubmit the registration for the next event. */
            wiced_bt_avrc_tg_register_for_notification(AVRC_EVT_VOLUME_CHANGE);
            break;
    }
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_getcap_response

** Description    Handle notification response from peer.
                  For AV source, only absolute volume is handled
*******************************************************************************/
static void wiced_bt_avrc_tg_handle_notification_response( uint8_t rsp_status, uint8_t ctype, wiced_bt_avrc_metadata_rsp_t *p_vendor_data)
{
    uint8_t event_id = p_vendor_data->u.reg_notif.event_id;

    WICED_BTAVRCP_TRACE( "%s: response_type=%s, id=0x%x\n\r", __FUNCTION__,
                    wiced_bt_avrc_tg_dump_message_type(ctype), event_id);

    /* switch on the event */
    switch(event_id)
    {
        case AVRC_EVT_VOLUME_CHANGE:
            wiced_bt_avrc_tg_handle_abs_volume_response(ctype, p_vendor_data->u.reg_notif.param.volume);
            break;
        default:
            break;
    }
}

/*******************************************************************************
* Function         wiced_bt_avrc_tg_getcap_response

** Description    Get capabilities reponse from peer. For AV source,
                  we only need to know if Absolute volume is supported
*******************************************************************************/
static void wiced_bt_avrc_tg_getcap_response( uint8_t rsp_status, wiced_bt_avrc_metadata_get_caps_rsp_t *get_caps )
{
    if ( AVRC_STS_NO_ERROR == rsp_status )
    {
        uint8_t capability_id = get_caps->capability_id;

        /* if capability id not = AVRC_CAP_EVENTS_SUPPORTED, fail out */
        if ( capability_id == AVRC_CAP_EVENTS_SUPPORTED )
        {
            int i;
            uint8_t cap_count = get_caps->count;
            wiced_bt_avrc_caps_param_t  *p_caps   = &get_caps->param;

            /* Determine if the remote is capable of Abs. Volume. If so, register. */
            for ( i = 0; i < cap_count; i++ )
            {
                switch ( p_caps->event_id[i] )
                {
                case AVRC_EVT_VOLUME_CHANGE:
                    wiced_bt_avrc_tg_cb.is_abs_volume_capable = WICED_TRUE;

                    /* Abs. Volume supported. Register for notification. */
                    wiced_bt_avrc_tg_register_for_notification( AVRC_EVT_VOLUME_CHANGE );
                    break;
                default:
                    break;
                }
            }
        }
    }
}

/*******************************************************************************
* Function         wiced_bt_avrc_tg_handle_abs_volume_set_response

** Description    Get capabilities reponse from peer. For AV source,
                  we only need to know if Absolute volume is supported
*******************************************************************************/
static void wiced_bt_avrc_tg_handle_abs_volume_set_response(uint8_t rsp_status, uint8_t ctype, wiced_bt_avrc_metadata_rsp_t *p_vendor_data)
{
    if (rsp_status == AVRC_STS_NO_ERROR)
    {
        if (ctype == AVRC_RSP_ACCEPT)
        {
            wiced_bt_avrc_tg_update_abs_volume(p_vendor_data->u.volume);
        }
    }
}


#ifdef CATEGORY_2_PASSTROUGH
/*******************************************************************************
* Function        wiced_bt_avrc_tg_button_press

** Description    Send AVRC button press command (pass-thru command)
*******************************************************************************/
wiced_result_t wiced_bt_avrc_tg_button_press ( uint8_t label, uint8_t state, uint8_t op_id )
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_bt_avrc_pass_thru_cmd_t pass_thru;

    memset(&msg, 0, sizeof(wiced_bt_avrc_cmd_t));

    pass_thru.hdr.operation_id = op_id;
    pass_thru.hdr.state = state;

    wiced_bt_avrc_send_passthrough_cmd(wiced_bt_avrc_tg_cb.avrc_handle,&pass_thru);

    return result;
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_button_press

** Description    Send AVRC button press command (pass-thru command)
*******************************************************************************/
wiced_result_t wiced_bt_avrc_tg_volume_button_press(uint8_t op_id)
{
    wiced_result_t result = WICED_SUCCESS;
    if(op_id == AVRC_ID_VOL_UP)
    {
        result = wiced_bt_avrc_tg_button_press ( 10, AVRC_STATE_PRESS, AVRC_ID_VOL_UP );
    }
    else
    {
        result = wiced_bt_avrc_tg_button_press ( 11, AVRC_STATE_PRESS, AVRC_ID_VOL_DOWN );
    }

    return result;
}



/*******************************************************************************
* Function        wiced_bt_avrc_tg_pass_through_response_handler

** Description    pass-thru response handler
*******************************************************************************/
void wiced_bt_avrc_tg_pass_through_response_handler(uint8_t handle, uint8_t label, uint8_t opcode, wiced_bt_avrc_rsp_t *p_msg)
{
    wiced_bt_rc_event_t avrc_event;

    if (p_msg->pass.hdr.ctype == AVRC_RSP_ACCEPT)
    {
        avrc_event.passthrough_response = WICED_SUCCESS;

        if ( p_msg->pass.state == AVRC_STATE_PRESS )
        {
            /* If the keypress was accepted, we need to send the release. */
            wiced_bt_avrc_tg_button_press ( label, AVRC_STATE_RELEASE, p_msg->pass.operation_id);
        }
    }
    else
    {
        avrc_event.passthrough_response = WICED_ERROR;

        WICED_BTAVRCP_TRACE( "%s: Passthrough 0x%x not accepted", __FUNCTION__, p_msg->pass.operation_id);
    }

    /* Pass the status to the app if there is an application event callback registered */
    if(wiced_bt_avrc_tg_cb.p_event_cb)
        (wiced_bt_avrc_tg_cb.p_event_cb)(APP_AVRC_EVENT_PASSTHROUGH_RESPONSE, &avrc_event);
}
#endif

/*******************************************************************************
* Function        wiced_bt_avrc_tg_vendor_response_handler

** Description    AVRC vendor response handler
*******************************************************************************/
void wiced_bt_avrc_tg_vendor_response_handler(uint8_t handle, uint8_t label, uint8_t opcode, wiced_bt_avrc_rsp_t *p_msg)
{
  //  uint8_t   *p    = p_msg->vendor.p_vendor_data;
    uint8_t   pdu   = p_msg->type.metadata.metadata_hdr.pdu;
    uint16_t  vendor_len = 0;
    uint8_t   rsp_status = AVRC_STS_NO_ERROR;

    WICED_BTAVRCP_TRACE( "%s: avrc_hdl=0x%x, label=0x%x, opcode=%x\n\r", __FUNCTION__, handle, label, opcode );

    /* TODO: Need to validate the transaction label as an outstanding request */

    WICED_BTAVRCP_TRACE( "%s: parsed response pdu=%x vendor_len=%d\n\r", __FUNCTION__, pdu, p_msg->type.metadata.metadata_hdr.param_len);

    switch ( pdu )
    {
    case AVRC_PDU_GET_CAPABILITIES:
        wiced_bt_avrc_tg_getcap_response( rsp_status, &p_msg->type.metadata.u.get_caps);
        break;

    case AVRC_PDU_REGISTER_NOTIFICATION:
        wiced_bt_avrc_tg_handle_notification_response( rsp_status, p_msg->hdr.ctype, &p_msg->type.metadata);
        break;

    case AVRC_PDU_SET_ABSOLUTE_VOLUME:
        /* Assume this is the accept message from the Abs Volume set request. */
        wiced_bt_avrc_tg_handle_abs_volume_set_response(rsp_status, p_msg->hdr.ctype, &p_msg->type.metadata);
        break;

    default:
        break;
    }
    UNUSED_VARIABLE(vendor_len);
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_response_handler

** Description    AVRC response handler
*******************************************************************************/
void wiced_bt_avrc_tg_response_handler(uint8_t handle, uint8_t label, uint8_t opcode, wiced_bt_avrc_rsp_t *p_msg)
{
    switch ( opcode )
    {

#ifdef CATEGORY_2_PASSTROUGH
    case AVRC_OP_PASS_THRU:
        wiced_bt_avrc_tg_pass_through_response_handler( handle, label, opcode, p_msg );
        break;
#endif

    case AVRC_OP_VENDOR:        /**< Vendor-dependent commands  */
        wiced_bt_avrc_tg_vendor_response_handler( handle, label, opcode, p_msg );
        break;

    default:
        WICED_BTAVRCP_TRACE( "%s: unhandled response opcode: 0x%x\n\r", __FUNCTION__, opcode );
        break;
    }
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_get_capabilities_handler

** Description    Handle get capabilities command from peer
*******************************************************************************/
wiced_bt_avrc_xmit_buf_t * wiced_bt_avrc_tg_get_capabilities_handler(uint8_t handle, wiced_bt_avrc_metadata_get_caps_cmd_t *get_cap, wiced_bt_avrc_metadata_rsp_t *p_response)
{
    wiced_bt_avrc_xmit_buf_t *p_rsp = NULL;
    wiced_bt_avrc_sts_t avrc_status;
    uint8_t *p_cap_list = NULL;
    uint32_t byte_count = 0;

    p_response->u.get_caps.capability_id = get_cap->capability_id;

    if (get_cap->capability_id == AVRC_CAP_EVENTS_SUPPORTED )
    {
        p_response->u.get_caps.count = sizeof_array(app_avrc_meta_caps_evt_ids);
        p_cap_list = (uint8_t *)app_avrc_meta_caps_evt_ids;
        byte_count = sizeof(app_avrc_meta_caps_evt_ids);
    }
    else if (get_cap->capability_id == AVRC_CAP_COMPANY_ID)
    {
        p_response->u.get_caps.count = sizeof_array(app_avrc_meta_caps_co_ids);
        p_cap_list = (uint8_t *)app_avrc_meta_caps_co_ids;
        byte_count = sizeof(app_avrc_meta_caps_co_ids);
    }
    else
    {
        WICED_BTAVRCP_TRACE( "%s bad cap id: %d\n\r", __FUNCTION__, get_cap->capability_id);
        wiced_bt_avrc_build_rejected_rsp(&p_rsp, p_response->metadata_hdr.pdu, AVRC_STS_BAD_PARAM);
        p_response->u.status = AVRC_STS_BAD_PARAM;
       return p_rsp;
    }

    WICED_BTAVRCP_TRACE( "%s num caps: %d\n\r", __FUNCTION__, p_response->u.get_caps.count);
 
        /* reply with all event ids that are supported */
        if (byte_count)
            memcpy(&p_response->u.get_caps.param.event_id,
                   p_cap_list,
                   byte_count);
 
    avrc_status = wiced_avrc_build_metadata_rsp (p_response, &p_rsp);
    if (AVRC_STS_NO_ERROR != avrc_status)
    {
        WICED_BTAVRCP_TRACE( "%s failed to create response: %d\n\r", __FUNCTION__, avrc_status);
    }
    p_response->u.status = AVRC_STS_NO_ERROR;
    return p_rsp;
}


#ifdef APP_AVRC_TRACK_INFO_SUPPORTED

wiced_bt_avrc_sts_t wiced_bt_avrc_find_get_element_attr_entry(uint8_t attr_id, wiced_bt_avrc_attr_entry_t *p_att_entry)
{
    int attr;
     /* Go through the list of requested attributes */
    for (attr = 0 ; attr < APP_AVRC_MAX_ATTR ; attr++)
    {
        /* If this attribute contains valid information */
        if (wiced_bt_avrc_tg_cb.app_track_attr[attr].str_len)
        {
            if(attr_id == wiced_bt_avrc_tg_cb.app_track_attr[attr].attr_id)
            {
                p_att_entry->name.charset_id    = AVRC_CHARSET_ID_UTF8;
                p_att_entry->name.name.str_len    = wiced_bt_avrc_tg_cb.app_track_attr[attr].str_len;
                p_att_entry->attr_id         = wiced_bt_avrc_tg_cb.app_track_attr[attr].attr_id;
                p_att_entry->name.name.p_str      = wiced_bt_avrc_tg_cb.app_track_attr[attr].p_str;
                return AVRC_STS_NO_ERROR;
            }
        }
    }

    return AVRC_STS_NOT_FOUND;

}

uint16_t wiced_bt_get_track_info_size()
{
    uint16_t length = 0;
    uint8_t attr ;
    for (attr = 0 ; attr < APP_AVRC_MAX_ATTR ; attr++)
    {
        /* If this attribute contains valid information */
        if (wiced_bt_avrc_tg_cb.app_track_attr[attr].str_len)
        {
            length += GET_ELMENT_ATTR_HDR + wiced_bt_avrc_tg_cb.app_track_attr[attr].str_len;
        }
    }
    WICED_BTAVRCP_TRACE( "%s Total GetElementAttribut Size: %d\n\r", __FUNCTION__, length);

    return length;


}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_get_element_attr_handler

** Description    Handle get element attribute command from peer
*******************************************************************************/
wiced_bt_avrc_xmit_buf_t * wiced_bt_avrc_tg_get_element_attr_handler(uint8_t handle,
    wiced_bt_avrc_metadata_get_elem_attrs_cmd_t *p_get_elem_cmd,
    wiced_bt_avrc_metadata_rsp_t * p_response)
{
    wiced_bt_avrc_xmit_buf_t *p_rsp = NULL;
    uint8_t xx;
    uint32_t attr;
    wiced_bt_avrc_attr_entry_t att_entry;
    wiced_bt_avrc_sts_t avrc_status = AVRC_STS_NO_ERROR;
    uint16_t len = 0;
    memset(&att_entry, 0, sizeof(wiced_bt_avrc_attr_entry_t));

    WICED_BTAVRCP_TRACE("[%s] ", __FUNCTION__);

    len = wiced_bt_get_track_info_size();
    if (len)
    {
        /*len +1 for num_attr which is 1 byte*/
        if ((p_rsp = (wiced_bt_avrc_xmit_buf_t *)wiced_bt_get_buffer(len+1+ WICED_AVRC_XMIT_BUF_OVERHEAD)) == NULL)
            return NULL;
        p_rsp->is_cmd = FALSE;
        p_rsp->meta_pdu_id = p_response->metadata_hdr.pdu;
    }


    if(p_rsp != NULL)
    {
        p_rsp->len_used = 1;
        /* If no attribute list provided, send every attribute available */
        if (p_get_elem_cmd->num_attr == 0)
        {

                for (uint8_t attr_id = AVRC_MEDIA_ATTR_ID_TITLE ; attr_id <= APP_AVRC_MAX_ATTR ; attr_id++)
                {
                    avrc_status = wiced_bt_avrc_find_get_element_attr_entry(attr_id, &att_entry);
                    if(avrc_status == AVRC_STS_NO_ERROR)
                    {
                        avrc_status = wiced_bt_avrc_add_attr_to_response(AVRC_PDU_GET_ELEMENT_ATTR, &att_entry, p_rsp);
                        if(avrc_status == AVRC_STS_NO_ERROR) 
                            p_response->u.get_elem_attrs.num_attr++;
                    }
                    
                }
                p_rsp->payload[0] = p_response->u.get_elem_attrs.num_attr;


        }
        else
        {
            /* Go through the list of requested attributes */
            for (xx=0 ; xx < p_get_elem_cmd->num_attr; xx++)
            {
                attr = p_get_elem_cmd->attrs[xx];

                /* If the attribute is supported */
                if ((attr >= AVRC_MEDIA_ATTR_ID_TITLE) &&
                    (attr <= APP_AVRC_MAX_ATTR))
                {
                    WICED_BTAVRCP_TRACE("checking attribute %d ", attr);
                    avrc_status = wiced_bt_avrc_find_get_element_attr_entry(attr, &att_entry);
                    if(avrc_status == AVRC_STS_NO_ERROR)
                    {
                        avrc_status = wiced_bt_avrc_add_attr_to_response(AVRC_PDU_GET_ELEMENT_ATTR, &att_entry, p_rsp);
                        if(avrc_status == AVRC_STS_NO_ERROR)
                            p_response->u.get_elem_attrs.num_attr++;
                    }
                }
                else
                {
                    WICED_BTAVRCP_TRACE("unsupported attribute requested %d ", attr);
                }
            }
             p_rsp->payload[0] = p_response->u.get_elem_attrs.num_attr;
        }
    }
    return p_rsp;
}

#endif

#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
/*******************************************************************************
* Function        wiced_bt_avrc_tg_list_player_attr_handler

** Description    Handle list player attribute command from peer
*******************************************************************************/
wiced_bt_avrc_xmit_buf_t * wiced_bt_avrc_tg_list_player_attr_handler(uint8_t handle, wiced_bt_avrc_metadata_rsp_t* p_response)
{
    wiced_bt_avrc_xmit_buf_t *p_rsp = NULL;
    uint8_t i;
    WICED_BTAVRCP_TRACE( "%s\n\r", __FUNCTION__);
    p_response->u.list_app_attr.num_attr = APP_AVRC_SETTING_SUPPORTED_MAX;
    uint8_t attrs[APP_AVRC_SETTING_SUPPORTED_MAX];
    p_response->u.list_app_attr.p_attrs = attrs;

    /* send all the player attributes supported to the peer*/
    for(i = 0; i < APP_AVRC_SETTING_SUPPORTED_MAX; i++)
    {
        p_response->u.list_app_attr.p_attrs[i] = player_settings[i].attr_id;
    }

    wiced_avrc_build_metadata_rsp (p_response, &p_rsp);

    return p_rsp;
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_list_player_attr_values_handler

** Description    Handle list player attribute values command from peer
*******************************************************************************/
wiced_bt_avrc_xmit_buf_t * wiced_bt_avrc_tg_list_player_attr_values_handler(uint8_t handle, uint8_t attr_id, wiced_bt_avrc_metadata_rsp_t *p_response)
{
    wiced_bt_avrc_xmit_buf_t *p_rsp = NULL;
    uint8_t i;

    WICED_BTAVRCP_TRACE( "%s\n\r", __FUNCTION__);

    /* send attribute values for the requested attribute id to the peer*/
    for(i = 0; i < APP_AVRC_SETTING_SUPPORTED_MAX; i++)
    {
        if(attr_id == player_settings[i].attr_id)
        {
            p_response->u.list_app_values.num_val =  player_settings[i].num_val;
            p_response->u.list_app_values.p_vals = player_settings[i].vals;
            wiced_avrc_build_metadata_rsp (p_response, &p_rsp);
            return p_rsp;

        }
    }
    return p_rsp;
}


/*******************************************************************************
* Function        wiced_bt_avrc_tg_get_cur_player_value_handler

** Description    Handle get current player attribute values command from peer
*******************************************************************************/
wiced_bt_avrc_xmit_buf_t * wiced_bt_avrc_tg_get_cur_player_value_handler(uint8_t handle, wiced_bt_avrc_metadata_get_cur_app_value_cmd_t *cur_value, wiced_bt_avrc_metadata_rsp_t * p_response)
{
    wiced_bt_avrc_xmit_buf_t *p_rsp = NULL;
    uint8_t xx = 0, yy = 0;
    uint8_t idx = 0;
    uint8_t app_setting[2*APP_AVRC_SETTING_SUPPORTED_MAX]; /*one pair(attr_d + attr_val)*/

    WICED_BTAVRCP_TRACE( "%s\n\r", __FUNCTION__);

    p_response->u.get_cur_app_val.p_vals = (uint8_t *)&app_setting;

    /* for each of the attribute ID requested, respond with the value to the peer */
    for(xx = 0; xx < APP_AVRC_SETTING_SUPPORTED_MAX; xx++)
    {
        for(yy = 0; yy < cur_value->num_attr; yy++)
        {
            if(cur_value->p_vals[yy] == player_settings[xx].attr_id)
            {
                app_setting[idx++] = player_settings[xx].attr_id;
                app_setting[idx++] = player_settings[xx].curr_value;
                p_response->u.get_cur_app_val.num_val++;
                
            }
        }
    }
    
    WICED_BTAVRCP_TRACE( "[%s]: get_cur_app_val.num_val =  %d\n\r", __FUNCTION__, p_response->u.get_cur_app_val.num_val);
    wiced_avrc_build_metadata_rsp( p_response, &p_rsp);
    return p_rsp;
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_set_player_value_handler

** Description    Handle set player attribute values command from peer
*******************************************************************************/
wiced_bt_avrc_xmit_buf_t * wiced_bt_avrc_tg_set_player_value_handler(uint8_t handle, wiced_bt_avrc_metadata_set_app_value_cmd_t *set_value, wiced_bt_avrc_metadata_rsp_t * p_response)
{
    wiced_bt_avrc_xmit_buf_t *p_rsp = NULL;
    uint8_t xx = 0, i;
    wiced_bt_rc_event_t avrc_event;

    WICED_BTAVRCP_TRACE( "%s - set vals %d\n\r", __FUNCTION__, set_value->num_val);

    /* Peer set the player settings. For each of the setting supported, provide a response*/
    for(i = 0; i < APP_AVRC_SETTING_SUPPORTED_MAX; i++)
    {
        for(xx = 0; xx < set_value->num_val; xx++)
        {
            if(set_value->p_vals[xx].attr_id == player_settings[i].attr_id)
            {
                /* update the current player value */
                player_settings[i].curr_value = set_value->p_vals[xx].attr_val;

                if(set_value->p_vals[xx].attr_id == AVRC_PLAYER_SETTING_REPEAT)
                {
                    /* Update the MCU app with new value of repeat*/
                    avrc_event.setting_val = set_value->p_vals[xx].attr_val;
                    if(wiced_bt_avrc_tg_cb.p_event_cb)
                        (wiced_bt_avrc_tg_cb.p_event_cb)(APP_AVRC_EVENT_REPEAT_SETTINGS_CHANGED, &avrc_event);
                }
                else if(set_value->p_vals[xx].attr_id == AVRC_PLAYER_SETTING_SHUFFLE)
                {
                    /* Update the MCU app with new value of shuffle*/
                    avrc_event.setting_val = set_value->p_vals[xx].attr_val;
                    if(wiced_bt_avrc_tg_cb.p_event_cb)
                        (wiced_bt_avrc_tg_cb.p_event_cb)(APP_AVRC_EVENT_SHUFFLE_SETTINGS_CHANGED, &avrc_event);
                }
                /* send response to peer */
                wiced_avrc_build_metadata_rsp (p_response, &p_rsp);
            }
        }
    }

    return p_rsp;
}
#endif

#ifdef APP_AVRC_PLAY_STATUS_SUPPORTED
/*******************************************************************************
* Function        wiced_bt_avrc_tg_get_play_status_handler

** Description    Handle get play status command from peer, get info from MCU application
*******************************************************************************/
wiced_bt_avrc_xmit_buf_t * wiced_bt_avrc_tg_get_play_status_handler(uint8_t handle, uint8_t label, uint8_t opcode)
{
    wiced_bt_avrc_xmit_buf_t *p_rsp = NULL;
    wiced_bt_avrc_metadata_rsp_t avrc_rsp;

    wiced_bt_avrc_sts_t avrc_status;

    WICED_BTAVRCP_TRACE( "%s\n\r", __FUNCTION__);

    memset (&avrc_rsp, 0, sizeof(wiced_bt_avrc_metadata_rsp_t));
    avrc_rsp.u.get_play_status.play_status = wiced_bt_avrc_tg_cb.player_status.play_state;
    avrc_rsp.u.get_play_status.song_len    = wiced_bt_avrc_tg_cb.player_status.song_len;
    avrc_rsp.u.get_play_status.song_pos    = wiced_bt_avrc_tg_cb.player_status.song_pos;

    avrc_rsp.metadata_hdr.pdu    = AVRC_PDU_GET_PLAY_STATUS;

    avrc_status = wiced_avrc_build_metadata_rsp (&avrc_rsp, &p_rsp);

    /* TODO: Ensure that the response was built correctly! */
    if (avrc_status != AVRC_STS_NO_ERROR)
    {
        WICED_BTAVRCP_TRACE( "%s ERROR: Could not build response: %d\n\r", __FUNCTION__, avrc_status);
    }

    return p_rsp;
}

#endif

/*******************************************************************************
* Function        wiced_bt_avrc_tg_register_notifications_handler

** Description    Handle register notification command from peer
*******************************************************************************/
wiced_bt_avrc_xmit_buf_t * wiced_bt_avrc_tg_register_notifications_handler(uint8_t handle, wiced_bt_avrc_metadata_cmd_t *p_cmd, wiced_bt_avrc_metadata_rsp_t * p_response)
{
    wiced_bt_avrc_xmit_buf_t *p_rsp = NULL;

    WICED_BTAVRCP_TRACE( "%s event: %d\n\r", __FUNCTION__, p_cmd->u.reg_notif.event_id);

    /* Check if AVRC connection is still open, if not return */
    if ( INVALID_AVRC_HANDLE == handle )
    {
        return p_rsp;
    }

    /* send current application info with the notification */
    p_response->u.reg_notif.event_id = p_cmd->u.reg_notif.event_id;

    switch(p_cmd->u.reg_notif.event_id)
    {
#ifdef APP_AVRC_PLAY_STATUS_SUPPORTED
        case AVRC_EVT_PLAY_STATUS_CHANGE:   /* 0x01 */
        {
            p_response->u.reg_notif.param.play_status = wiced_bt_avrc_tg_cb.player_status.play_state;
        }
        break;
#endif

#ifdef APP_AVRC_TRACK_INFO_SUPPORTED
        case AVRC_EVT_TRACK_CHANGE:   /* 0x02 */
        {
            if (wiced_bt_avrc_tg_cb.app_track_attr[AVRC_MEDIA_ATTR_ID_TRACK_NUM].str_len)
            {
            }
            else
            {
                memset(p_response->u.reg_notif.param.track, 0xff, sizeof(p_response->u.reg_notif.param.track));
            }
        }
        break;
#endif

#ifdef APP_AVRC_TRACK_REACHED_END_SUPPORTED
        case AVRC_EVT_TRACK_REACHED_END:   /* 0x03 */
        {
            /* Currently not supported */
        }
        break;

#endif

#ifdef APP_AVRC_TRACK_REACHED_START_SUPPORTED

        case AVRC_EVT_TRACK_REACHED_START:   /* 0x04 */
        {
            /* Currently not supported */
        }
        break;

#endif

#ifdef APP_AVRC_TRACK_PLAY_POS_CHANGE_SUPPORTED
        case AVRC_EVT_PLAY_POS_CHANGED:   /* 0x05 */
        {
            p_response->u.reg_notif.param.play_pos = wiced_bt_avrc_tg_cb.player_status.song_pos;
        }
        break;
#endif

#ifdef APP_AVRC_BATTERY_STATUS_SUPPORTED

        case AVRC_EVT_BATTERY_STATUS_CHANGE:   /* 0x06 */
        {
            /* Currently not supported */
        }
        break;

#endif

#ifdef APP_AVRC_SYSTEM_STATUS_SUPPORTED

        case AVRC_EVT_SYSTEM_STATUS_CHANGE:   /* 0x07 */
        {
            /* Currently not supported */
        }
        break;

#endif

#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
        case AVRC_EVT_APP_SETTING_CHANGE:   /* 0x08 */
        {
            uint8_t i;
            uint8_t status;
            wiced_bt_avrc_app_setting_t attr;
            uint8_t  num_attr = 0;
            uint8_t *p_num_attr;
             wiced_avrc_build_metadata_rsp (p_response, &p_rsp);
             p_num_attr = &p_rsp->payload[p_rsp->len_used];
            p_rsp->len_used +=1; 
            for(i =0; i<APP_AVRC_SETTING_SUPPORTED_MAX; i++)
            {
                status = wiced_bt_avrc_get_suported_app_setting(i, &attr);
                if(status != AVRC_STS_NO_ERROR)
                    break;
                status = wiced_bt_add_app_supported_settings(&attr, p_rsp);
                if(status == AVRC_STS_NO_ERROR)
                    num_attr ++;
            }
            *p_num_attr = num_attr;


        }
        break;
#endif
        case AVRC_EVT_VOLUME_CHANGE:
        {
            // Reply back with the current volume
            p_response->u.reg_notif.param.volume = wiced_bt_avrc_tg_cb.last_abs_volume;
        }break;
    }

#if (defined(APP_AVRC_TRACK_INFO_SUPPORTED)            || \
     defined(APP_AVRC_PLAY_STATUS_SUPPORTED)           || \
     defined(APP_AVRC_SETTING_CHANGE_SUPPORTED)        || \
     defined(APP_AVRC_TRACK_REACHED_END_SUPPORTED)     || \
     defined(APP_AVRC_TRACK_REACHED_START_SUPPORTED)   || \
     defined(APP_AVRC_TRACK_PLAY_POS_CHANGE_SUPPORTED) || \
     defined(APP_AVRC_BATTERY_STATUS_SUPPORTED)        || \
     defined(APP_AVRC_SYSTEM_STATUS_SUPPORTED))

if(p_cmd->u.reg_notif.event_id != AVRC_EVT_APP_SETTING_CHANGE)
    wiced_avrc_build_metadata_rsp (p_response, &p_rsp);
#endif

    return p_rsp;
}

void wiced_bt_avrc_tg_send_rsp(uint8_t handle, uint8_t label, uint8_t ctype, uint8_t pdu_id, wiced_bt_avrc_xmit_buf_t * p_rsp, wiced_bt_avrc_metadata_rsp_t * p_avrc_rsp)
{
    uint8_t status = AVRC_STS_NOT_FOUND;

    if((ctype != AVRC_RSP_NOT_IMPL) && (p_rsp == NULL))
    {
        WICED_BTAVRCP_TRACE("%s: getting info from MCU app :%d. pdu:%d", __FUNCTION__, handle, pdu_id);
    }
    else
    {
        if(p_rsp == NULL)
        {
            ctype = AVRC_RSP_NOT_IMPL;
            status = wiced_bt_avrc_bld_metadata_response(p_avrc_rsp, p_rsp);
        }

        if(p_rsp)
        {
            if(ctype == AVRC_RSP_NOT_IMPL)
            {
                 WICED_BTAVRCP_TRACE("%s: Sending not implemented response to handle:%d. pdu:%d",
                __FUNCTION__, handle, pdu_id);
            }
            else
            {
                 WICED_BTAVRCP_TRACE("%s: Sending response to handle:%d. pdu:%d",
                __FUNCTION__, handle, pdu_id);
            }

            if (wiced_bt_avrc_send_metadata_msg(wiced_bt_avrc_tg_cb.avrc_handle, label, ctype, p_rsp) != AVRC_SUCCESS)
            {
                WICED_BTAVRCP_TRACE( "failed to send response\n\r");
            }
        }
    }
    UNUSED_VARIABLE(status);
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_vendor_command_handler

** Description    handle avrc target commands
*******************************************************************************/
void wiced_bt_avrc_tg_vendor_command_handler( uint8_t handle, uint8_t label, uint8_t opcode, wiced_bt_avrc_metadata_cmd_t *p_msg )
{
    wiced_bt_avrc_xmit_buf_t *p_rsp = NULL;
    uint8_t pdu_id = 0;
    uint8_t     ctype = AVRC_RSP_ACCEPT;
    wiced_bt_avrc_metadata_rsp_t avrc_rsp;
    pdu_id = p_msg->metadata_hdr.pdu;
    WICED_BTAVRCP_TRACE("%s: pdu:%d", __FUNCTION__, pdu_id);

    memset (&avrc_rsp, 0, sizeof(wiced_bt_avrc_metadata_rsp_t));

    avrc_rsp.metadata_hdr.pdu    = pdu_id;

    switch(pdu_id)
    {
    case AVRC_PDU_GET_CAPABILITIES:
        {
            ctype = AVRC_RSP_IMPL_STBL;
             p_rsp = wiced_bt_avrc_tg_get_capabilities_handler(handle, &p_msg->u.get_caps, &avrc_rsp);
            if ((p_rsp != NULL) && (avrc_rsp.u.status != AVRC_STS_NO_ERROR))
                ctype = AVRC_RSP_REJ;
        }
        break;

#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
    case AVRC_PDU_LIST_PLAYER_APP_ATTR:
        {
            ctype = AVRC_RSP_IMPL_STBL;
            p_rsp = wiced_bt_avrc_tg_list_player_attr_handler(handle, &avrc_rsp);
        }
        break;
    case AVRC_PDU_LIST_PLAYER_APP_VALUES:
        {
            ctype = AVRC_RSP_IMPL_STBL;
            p_rsp = wiced_bt_avrc_tg_list_player_attr_values_handler(handle, p_msg->u.list_app_values.attr_id, &avrc_rsp);
        }
        break;
    case AVRC_PDU_GET_CUR_PLAYER_APP_VALUE:
        {
            ctype = AVRC_RSP_IMPL_STBL;
            p_rsp = wiced_bt_avrc_tg_get_cur_player_value_handler(handle, &p_msg->u.get_cur_app_val, &avrc_rsp);
        }
        break;
    case AVRC_PDU_SET_PLAYER_APP_VALUE:
        {
            ctype = AVRC_RSP_ACCEPT;
            p_rsp = wiced_bt_avrc_tg_set_player_value_handler(handle, &p_msg->u.set_app_val, &avrc_rsp);
        }
        break;
#endif

#ifdef APP_AVRC_TRACK_INFO_SUPPORTED
    case AVRC_PDU_GET_ELEMENT_ATTR:
        {
            ctype = AVRC_RSP_IMPL_STBL;
            p_rsp = wiced_bt_avrc_tg_get_element_attr_handler(handle,
                    &p_msg->u.get_elem_attrs, &avrc_rsp);
        }
        break;
#endif

#ifdef APP_AVRC_PLAY_STATUS_SUPPORTED
    case AVRC_PDU_GET_PLAY_STATUS:
        {
            ctype = AVRC_RSP_IMPL_STBL;
            p_rsp = wiced_bt_avrc_tg_get_play_status_handler(handle, label, opcode);
        }
        break;
#endif

    case AVRC_PDU_REGISTER_NOTIFICATION:
        /* TODO: Need to only accept registration from events we support. */
        
        if (p_msg->u.reg_notif.event_id <= AVRC_NUM_NOTIF_EVENTS)
        {
            /* for registered notification, send AVRC_RSP_INTERIM as response */
            ctype = AVRC_RSP_INTERIM;

            /* save the requested notification in an event mask */
            wiced_bt_avrc_tg_cb.registered_event_mask |= (1 << (p_msg->u.reg_notif.event_id - 1));
            wiced_bt_avrc_tg_cb.registered_event_label[p_msg->u.reg_notif.event_id] = label;

#ifdef APP_AVRC_PLAY_STATUS_SUPPORTED
            if (p_msg->u.reg_notif.event_id == AVRC_EVT_PLAY_POS_CHANGED)
            {
                /* Save the update interval if valid */
                wiced_bt_avrc_tg_cb.position_update_interval_sec = p_msg->u.reg_notif.playback_interval;
            }
#endif

            p_rsp = wiced_bt_avrc_tg_register_notifications_handler(handle, p_msg, &avrc_rsp);
            WICED_BTAVRCP_TRACE( "AVRC_PDU_REGISTER_NOTIFICATION, event id %d \n\r", p_msg->u.reg_notif.event_id);
        }
        else
        {
            ctype = AVRC_RSP_REJ;
            wiced_bt_avrc_build_rejected_rsp(&p_rsp, avrc_rsp.metadata_hdr.pdu, AVRC_STS_BAD_PARAM);
        }

        break;
    case AVRC_PDU_SET_ADDRESSED_PLAYER:
        if (p_msg->u.player_id != AVRC_TG_PLAYER_ID) {
            ctype = AVRC_RSP_REJ;
            avrc_rsp.u.status = AVRC_STS_BAD_PLAYER_ID;
        }
        else {
            ctype = AVRC_RSP_ACCEPT;
            avrc_rsp.u.status = AVRC_STS_NO_ERROR;
        }
        wiced_avrc_build_metadata_rsp (&avrc_rsp, &p_rsp);
        break;
    case AVRC_PDU_SET_ABSOLUTE_VOLUME:
    {
        ctype = AVRC_RSP_ACCEPT;
        /*
        * Update the volume as received. <TBD>
        */
        wiced_bt_avrc_tg_update_abs_volume(p_msg->u.volume);
        wiced_avrc_build_metadata_rsp (&avrc_rsp, &p_rsp);
    }break;
    case AVRC_PDU_GET_PLAYER_APP_ATTR_TEXT:
    {
        ctype = AVRC_RSP_ACCEPT;

        p_rsp = wiced_bt_avrc_build_app_attr_txt_rsp(handle, &p_msg->u.get_app_attr_txt, &avrc_rsp);

    }break;
    case AVRC_PDU_GET_PLAYER_APP_VALUE_TEXT:
         ctype = AVRC_RSP_ACCEPT;
        p_rsp = wiced_bt_avrc_build_app_attr_value_txt_rsp(handle, &p_msg->u.get_app_val_txt, &avrc_rsp);
        break;
    case AVRC_PDU_INFORM_DISPLAY_CHARSET :
        avrc_rsp.u.status = AVRC_STS_NO_ERROR;
        wiced_avrc_build_metadata_rsp (&avrc_rsp, &p_rsp);
        break;
    case AVRC_PDU_INFORM_BATTERY_STAT_OF_CT:
        avrc_rsp.u.status = AVRC_STS_NO_ERROR;
        wiced_avrc_build_metadata_rsp (&avrc_rsp, &p_rsp);
        break;
    default:
        ctype = AVRC_RSP_NOT_IMPL;
    }

    if (p_rsp) {
        wiced_bt_avrc_tg_send_rsp(handle, label, ctype, pdu_id, p_rsp, &avrc_rsp);
    }
    else if((ctype != AVRC_RSP_NOT_IMPL) && (p_rsp == NULL))
    {
        WICED_BTAVRCP_TRACE("%s: getting info from MCU app :%d. pdu:%d", __FUNCTION__, handle, pdu_id);
    }


}



wiced_bt_avrc_sts_t wiced_bt_get_browsed_player_folder(uint16_t uid_counter, uint16_t idx,  wiced_bt_avrc_name_t *p_name )
{
    if(idx < 1)
    {
        p_name->str_len = 0x04;
        p_name->p_str = (uint8_t*)"root";
        return AVRC_STS_NO_ERROR;
    }
    else
        return AVRC_STS_NOT_EXIST;
}

void wiced_bt_avrc_tg_handle_set_browsed_player(uint8_t handle, uint8_t label, uint16_t player_id, wiced_bt_avrc_xmit_buf_t **p_rsp )
{
    wiced_bt_avrc_browse_rsp_t avrc_rsp;
    wiced_bt_avrc_browse_set_br_player_rsp_t *p_rsp_data = &avrc_rsp.u.set_browse_player;
    wiced_bt_avrc_sts_t status = AVRC_STS_PLAYER_N_ADDR;
    wiced_bt_avrc_name_t name;
    uint8_t *p_data, *p_len, *p_folder_depth;
    uint8_t *p_start;
    int xx = 0, count = 0;
    avrc_rsp.status = AVRC_STS_NO_ERROR;
    avrc_rsp.pdu_id = AVRC_PDU_SET_BROWSED_PLAYER;
    wiced_bt_avrc_build_browse_rsp(&avrc_rsp, p_rsp);
    if (p_rsp)
    {
        p_data = p_start = (*p_rsp)->payload;
        UINT8_TO_STREAM(p_data, AVRC_PDU_SET_BROWSED_PLAYER);      /* PDU ID      */
        p_len = p_data;                             /* Will fill in length later  */
        p_data += 2;

        UINT8_TO_BE_STREAM(p_data, AVRC_STS_NO_ERROR); /* status */
        UINT16_TO_BE_STREAM(p_data, wiced_bt_avrc_tg_get_uid_counter(AVRC_SCOPE_PLAYER_LIST));
        UINT32_TO_BE_STREAM(p_data, 1); /* default num_item 1 in a folder*/
        UINT16_TO_BE_STREAM(p_data, AVRC_CHARSET_ID_UTF8);
        p_folder_depth = p_data++;
        (*p_rsp)->len_used = p_data - p_start;
        for (xx = 0; xx < AVRC_MAX_FOLDER_DEPTH; xx++)
        {
            status = wiced_bt_get_browsed_player_folder(p_rsp_data->uid_counter, xx, &name);
            if (status == AVRC_STS_NO_ERROR)
            {
                status = wiced_bt_avrc_add_attr_to_response(AVRC_PDU_SET_BROWSED_PLAYER, &name, *p_rsp);
                if (status == AVRC_STS_NO_ERROR)
                    count++;
            }
            else
                break;
        }
        UINT8_TO_BE_STREAM(p_folder_depth, count);
        UINT16_TO_BE_STREAM(p_len, ((*p_rsp)->len_used - 3));
    }

}

int wiced_bt_avrc_tg_get_uid_counter(uint8_t scope)
{
    int uid_counter = -1;
    switch(scope)
    {
     case AVRC_SCOPE_PLAYER_LIST:
       uid_counter = 1;
       break;
     case AVRC_SCOPE_FILE_SYSTEM :
        uid_counter = 2;
        break;
     case AVRC_SCOPE_NOW_PLAYING :
        uid_counter =3 ;
        break;
        default :
            break;
    }

    return uid_counter;
}

wiced_bt_avrc_item_t tg_players[] = {
        {
        .item_type = AVRC_ITEM_PLAYER,
        .item_length = 34 ,
        .u.player.player_id = AVRC_TG_PLAYER_ID,
        .u.player.major_type = AVRC_PLAYER_MAJOR_TYPE_AUDIO,
        .u.player.sub_type = AVRC_PLAYER_SUB_TYPE_NONE,
        .u.player.features = {
                0x00, /**/
                0x00, /**/
                0x00, /**/
                0x00, /**/
                0xE0, /* vol up , vol dn, mute */
                0xB3, /* Play, Stop, Pause, Rewind, Fast Forward, forward*/
                0x01, /* Backward */
                0x06, /* Avrc 1.4, Browsing */
                0x08,
        },
        .u.player.name.charset_id = AVRC_CHARSET_ID_UTF8,
        .u.player.name.name.str_len = sizeof(AVRC_TG_PLAYER_NAME),
        .u.player.name.name.p_str = (uint8_t *)AVRC_TG_PLAYER_NAME
    }
};

wiced_bt_avrc_item_t tg_folders[] = {
        {
        .item_type = AVRC_ITEM_FOLDER,
        .item_length = 19 ,
        .u.folder.uid = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05 },
        .u.folder.type = AVRC_FOLDER_TYPE_PLAYLISTS,
        .u.folder.playable = FALSE,
        .u.folder.name.charset_id = AVRC_CHARSET_ID_UTF8,
        .u.folder.name.name.str_len = 0x03,
        .u.folder.name.name.p_str = (uint8_t*)"VFS"
    }
};

wiced_bt_avrc_attr_entry_t attr_list[2] = {
{
    .attr_id = AVRC_MEDIA_ATTR_ID_ARTIST,
    .name.charset_id = AVRC_CHARSET_ID_UTF8,
    .name.name.str_len = 0x05,
    .name.name.p_str = (uint8_t*)"artist",

},

{
    .attr_id = AVRC_MEDIA_ATTR_ID_ALBUM,
    .name.charset_id = AVRC_CHARSET_ID_UTF8,
    .name.name.str_len = 0x05,
    .name.name.p_str = (uint8_t*)"album",

},

};

wiced_bt_avrc_item_t tg_media[] = {
        {
        .item_type = AVRC_ITEM_MEDIA,
        .item_length = 51 ,
        .u.media.uid = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05 },
        .u.media.type = AVRC_PLAYER_MAJOR_TYPE_AUDIO,
        .u.media.name.charset_id = AVRC_CHARSET_ID_UTF8,
        .u.media.name.name.str_len = 0x05,
        .u.media.name.name.p_str = (uint8_t*)"songs",
        .u.media.attr_count  = 2,
        .u.media.p_attr_list = (uint8_t *)&attr_list
    },
};

wiced_bt_avrc_sts_t wiced_bt_get_folder_item_from_app(uint8_t scope, uint8_t idx, wiced_bt_avrc_item_t **p_folder_item)
{
    wiced_bt_avrc_sts_t  status =  AVRC_STS_NOT_FOUND;
    if(scope == AVRC_SCOPE_PLAYER_LIST )
    {
        uint8_t item_count = sizeof(tg_players) / sizeof(tg_players[0]);
        if(idx < item_count)
        {
            *p_folder_item = &tg_players[idx];
            status  = AVRC_STS_NO_ERROR;
        }

    }

    if(scope == AVRC_SCOPE_FILE_SYSTEM)
    {
        uint8_t item_count = sizeof(tg_folders) / sizeof(tg_folders[0]);
        if(idx < item_count)
        {
            *p_folder_item = &tg_folders[idx];
            status  = AVRC_STS_NO_ERROR;
        }

    }

    if(scope == AVRC_SCOPE_NOW_PLAYING)
    {
        uint8_t item_count = sizeof(tg_media[0]) / sizeof(tg_media[0]);
        if(idx < item_count)
        {
            *p_folder_item = &tg_media[idx];
            status  = AVRC_STS_NO_ERROR;
        }

    }
    return status;
    
}


void wiced_bt_avrc_tg_handle_get_folder_items(uint8_t handle, uint8_t label, 
                            wiced_bt_avrc_browse_get_items_cmd_t * p_command,
                            wiced_bt_avrc_xmit_buf_t **p_rsp )
    {
    wiced_bt_avrc_browse_rsp_t avrc_rsp;
    uint8_t status = AVRC_STS_NOT_FOUND;
    wiced_bt_avrc_item_t *p_folder_item;
    uint8_t *p_data, *p_len, *p_num;
    uint32_t xx = 0;
    uint16_t item_count = 0;;

    avrc_rsp.pdu_id = AVRC_PDU_GET_FOLDER_ITEMS;
    avrc_rsp.status = AVRC_STS_NO_ERROR;
    wiced_bt_avrc_build_browse_rsp(&avrc_rsp, p_rsp);
    if(p_rsp)
    {
        p_data = (*p_rsp)->payload;
        UINT8_TO_STREAM (p_data, AVRC_PDU_GET_FOLDER_ITEMS);      /* PDU ID      */
        p_len = p_data;                             /* Will fill in length later  */
        p_data += 2;

        UINT8_TO_BE_STREAM (p_data, AVRC_STS_NO_ERROR); /* status */

        avrc_rsp.u.get_folder_items.uid_counter = wiced_bt_avrc_tg_get_uid_counter(p_command->scope);
        UINT16_TO_BE_STREAM (p_data, avrc_rsp.u.get_folder_items.uid_counter); /* uid counter */
        p_num = p_data;                             /* Will fill in item count later */
        p_data += 2;
        (*p_rsp)->len_used = 8; /* pduid + len + status + uid_counter + item_count */
        if(p_command->scope == AVRC_SCOPE_PLAYER_LIST &&
            (p_command->attr_count== 0))
        {
           /* we have asked for provide all the supported players */
            for(xx = 0; xx < (AVRC_NUM_PLAYER_SUPPORTED); xx++)
            {
                status = wiced_bt_get_folder_item_from_app(p_command->scope, xx, &p_folder_item);
                if(status != AVRC_STS_NO_ERROR)
                {
                    break;
                }
                wiced_bt_avrc_add_attr_to_response(AVRC_PDU_GET_FOLDER_ITEMS, p_folder_item, *p_rsp);
                item_count++;
            }

            
        }else if((p_command->scope == AVRC_SCOPE_FILE_SYSTEM) &&
            (p_command->start_item <= p_command->end_item ))
        {
                for(xx = p_command->start_item; xx <= p_command->end_item ; xx++)
                {
                    status = wiced_bt_get_folder_item_from_app(p_command->scope, xx, &p_folder_item);
                    if(status != AVRC_STS_NO_ERROR)
                    {
                        break;
                    }
                    wiced_bt_avrc_add_attr_to_response(AVRC_PDU_GET_FOLDER_ITEMS, p_folder_item, *p_rsp);
                    item_count++;

                    
                }

        }
        UINT16_TO_BE_STREAM (p_num, item_count);
        UINT16_TO_BE_STREAM (p_len, ((*p_rsp)->len_used -3)); /* we can not used pdu_id + pdu_len size in pdu_len[] field*/

    }
    //wiced_bt_avrc_send_browse_data(handle, label, 2, *p_rsp);
    UNUSED_VARIABLE(status);
    return;
}


void wiced_bt_avrc_tg_handle_change_path(uint8_t handle, uint8_t label, wiced_bt_avrc_browse_chg_path_cmd_t *p_change_path, wiced_bt_avrc_xmit_buf_t **pp_rsp)
{
     WICED_BTAVRCP_TRACE("[%s] wiced_bt_avrc_tg_handle_change_path : response not handled", __FUNCTION__);
}

void wiced_bt_avrc_tg_handle_get_item_attributes(uint8_t handle, uint8_t label, wiced_bt_avrc_browse_get_item_attrs_cmd_t * p_command, wiced_bt_avrc_xmit_buf_t **p_rsp )
    {

    wiced_bt_avrc_browse_rsp_t avrc_rsp;
    uint8_t xx;
    uint32_t attr;
    wiced_bt_avrc_attr_entry_t att_entry;
    wiced_bt_avrc_sts_t avrc_status = AVRC_STS_NO_ERROR;
    uint8_t num_attr = 0;
    uint8_t *p_data, *p_start,*p_num, *p_len;
    memset(&att_entry, 0, sizeof(wiced_bt_avrc_attr_entry_t));
    
    WICED_BTAVRCP_TRACE("[%s] ", __FUNCTION__);
    if(p_command->scope == AVRC_SCOPE_NOW_PLAYING)
    {
        avrc_rsp.pdu_id = AVRC_PDU_GET_ITEM_ATTRIBUTES;
        avrc_rsp.status =  AVRC_STS_NO_ERROR;
        avrc_status = wiced_bt_avrc_build_browse_rsp(&avrc_rsp, p_rsp);
        if (avrc_status != AVRC_STS_NO_ERROR)
            return;
        if(p_rsp)
        {
            p_start = p_data = (*p_rsp)->payload;
            UINT8_TO_STREAM(p_data, AVRC_PDU_GET_ITEM_ATTRIBUTES);      /* PDU ID      */
            p_len = p_data;                             /* Will fill in length later  */
            p_data += 2;
            UINT8_TO_BE_STREAM(p_data, avrc_rsp.status);
            p_num = p_data++;
            num_attr = 0;                         /* Will fill in the number later */
            (*p_rsp)->len_used = p_data - p_start;
            if (p_command->attr_count)
            {
                for (xx = 0; xx < p_command->attr_count; xx++)
                {
                    attr = p_command->p_attr_list[xx];

                    /* If the attribute is supported */
                    if ((attr >= AVRC_MEDIA_ATTR_ID_TITLE) &&
                        (attr <= APP_AVRC_MAX_ATTR))
                    {
                        WICED_BTAVRCP_TRACE("checking attribute %d ", attr);
                        avrc_status = wiced_bt_avrc_find_get_element_attr_entry(attr, &att_entry);
                        if (avrc_status == AVRC_STS_NO_ERROR)
                        {
                            avrc_status = wiced_bt_avrc_add_attr_to_response(AVRC_PDU_GET_ELEMENT_ATTR, &att_entry, *p_rsp);
                            if (avrc_status == AVRC_STS_NO_ERROR)
                                num_attr++;
                        }

                    }
                }
            }
            else
            {/* In case all attributes are requested */
                for (xx = AVRC_MEDIA_ATTR_ID_TITLE; xx < AVRC_MAX_NUM_MEDIA_ATTR_ID; xx++)
                {

                        WICED_BTAVRCP_TRACE("checking attribute %d ", xx);
                        avrc_status = wiced_bt_avrc_find_get_element_attr_entry(xx, &att_entry);
                        if (avrc_status == AVRC_STS_NO_ERROR)
                        {
                            avrc_status = wiced_bt_avrc_add_attr_to_response(AVRC_PDU_GET_ELEMENT_ATTR, &att_entry, *p_rsp);
                            if (avrc_status == AVRC_STS_NO_ERROR)
                                num_attr++;
                        }
                }
            }
            UINT8_TO_BE_STREAM(p_num, num_attr);
            UINT16_TO_BE_STREAM(p_len, (*p_rsp)->len_used - 3);
        }
    }

   }

void wiced_bt_avrc_tg_handle_get_total_num_of_items(uint8_t handle, uint8_t label, wiced_bt_avrc_browse_get_num_of_items_cmd_t * p_command, wiced_bt_avrc_xmit_buf_t **pp_rsp )
        {
    wiced_bt_avrc_browse_rsp_t avrc_rsp;
    wiced_bt_avrc_browse_num_of_items_rsp_t *p_rsp_data = &avrc_rsp.u.get_num_of_items;
    uint8_t status = AVRC_STS_BAD_SCOPE;

    if(p_command->scope != AVRC_SCOPE_PLAYER_LIST)
            {
        return ;
    }

    avrc_rsp.pdu_id    = AVRC_PDU_GET_TOTAL_NUM_OF_ITEMS;
    avrc_rsp.status = AVRC_STS_NO_ERROR;
    p_rsp_data->num_items = sizeof(tg_players) / sizeof(tg_players[0]);
    p_rsp_data->uid_counter = wiced_bt_avrc_tg_get_uid_counter(p_command->scope);

	wiced_bt_avrc_build_browse_rsp (&avrc_rsp, pp_rsp);
    UNUSED_VARIABLE(status);
    return;
            }

/*******************************************************************************
* Function        wiced_bt_avrc_tg_browse_command_handler

** Description    handle avrc target commands
*******************************************************************************/
void wiced_bt_avrc_tg_browse_command_handler( uint8_t handle, uint8_t label, uint8_t opcode, wiced_bt_avrc_browse_cmd_t *p_browse_cmd)
{
    wiced_bt_avrc_xmit_buf_t *p_rsp = NULL;
    uint8_t pdu_id = p_browse_cmd->pdu;

    WICED_BTAVRCP_TRACE("[%s]: pdu:%d", __FUNCTION__, pdu_id);
    switch(p_browse_cmd->pdu)
    {
    case AVRC_PDU_SET_BROWSED_PLAYER:
        wiced_bt_avrc_tg_handle_set_browsed_player(handle , label, p_browse_cmd->browse_cmd.player_id, &p_rsp);
        break;
    case AVRC_PDU_GET_FOLDER_ITEMS:
        wiced_bt_avrc_tg_handle_get_folder_items(handle , label, &p_browse_cmd->browse_cmd.get_folder_items, &p_rsp );
        break;
    case AVRC_PDU_CHANGE_PATH:
        wiced_bt_avrc_tg_handle_change_path(handle , label, &p_browse_cmd->browse_cmd.chg_path, &p_rsp );
        break;
    case AVRC_PDU_GET_ITEM_ATTRIBUTES:
        wiced_bt_avrc_tg_handle_get_item_attributes(handle , label, &p_browse_cmd->browse_cmd.get_item_attrs, &p_rsp );
        break;
    case AVRC_PDU_GET_TOTAL_NUM_OF_ITEMS:
        wiced_bt_avrc_tg_handle_get_total_num_of_items(handle , label, &p_browse_cmd->browse_cmd.get_num_of_items, &p_rsp );
        break;
    case AVRC_PDU_SEARCH:
            WICED_BTAVRCP_TRACE("[%s]: pdu:%d : Not Implemented", __FUNCTION__, p_browse_cmd->pdu);
        break;
    case AVRC_PDU_GENERAL_REJECT:
        break;
    default:
        {
            WICED_BTAVRCP_TRACE("[%s] : Invalid command %d", __FUNCTION__,p_browse_cmd->pdu);
            wiced_bt_avrc_browse_rsp_t avrc_rsp;
            avrc_rsp.status = AVRC_STS_BAD_CMD;
            avrc_rsp.pdu_id = p_browse_cmd->pdu;
            wiced_bt_avrc_build_browse_rsp(&avrc_rsp, &p_rsp);
        }

        break;
}

  wiced_bt_avrc_send_browse_data(handle, label, 2, p_rsp);
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_complete_notification

** Description    send registered notification response for specified event ID
*******************************************************************************/
void wiced_bt_avrc_tg_complete_notification(uint8_t event_id)
{
    wiced_bt_avrc_metadata_rsp_t avrc_rsp;
    wiced_bt_avrc_xmit_buf_t *p_rsp = NULL;
    uint8_t ctype = AVRC_RSP_CHANGED;

    wiced_bt_avrc_metadata_cmd_t command;

    uint16_t  evt_mask = 1 << (event_id - 1);

    command.u.reg_notif.event_id = event_id;

    /* first check if the event id is registered for notification */
    if (wiced_bt_avrc_tg_cb.registered_event_mask & evt_mask)
    {
        avrc_rsp.metadata_hdr.pdu    = AVRC_PDU_REGISTER_NOTIFICATION;
        p_rsp = wiced_bt_avrc_tg_register_notifications_handler(wiced_bt_avrc_tg_cb.avrc_handle, &command, &avrc_rsp);
        if(p_rsp)
        {
            wiced_bt_avrc_send_metadata_msg (wiced_bt_avrc_tg_cb.avrc_handle, wiced_bt_avrc_tg_cb.registered_event_label[event_id], ctype, p_rsp);

            /* clear the registered event bit once we've completed the action */
            wiced_bt_avrc_tg_cb.registered_event_mask &= ~(1 << (event_id - 1));
        }
    }
}

/*
 * wiced_bt_avrc_tg_send_passthrough_cmd
 * Send Passthrough Command received froim peer device.
 * Note that this function does not handles passthrough parameters, so it cannot be used to send
 * Passthrough Absolute Volume Command.
 */
static void wiced_bt_avrc_tg_send_passthrough_cmd(uint8_t passthrough_cmd)
{
    wiced_bt_rc_event_t avrc_event;

    if(wiced_bt_avrc_tg_cb.p_event_cb == NULL)
    {
        return;
    }

    avrc_event.passthrough_command.command = passthrough_cmd;
    avrc_event.passthrough_command.handle = wiced_bt_avrc_tg_cb.avrc_handle;
    wiced_bt_avrc_tg_cb.p_event_cb(APP_AVRC_EVENT_PASSTHROUGH_CMD, &avrc_event);
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_cmd_pass_through_command_handler

** Description    Pass-thru cmd received from peer
*******************************************************************************/
void wiced_bt_avrc_tg_cmd_pass_through_command_handler( uint8_t handle, uint8_t label, uint8_t opcode, wiced_bt_avrc_cmd_t *p_msg )
{
    if ( p_msg->type.pass_thru.hdr.state == AVRC_STATE_PRESS )
    {
        p_msg->hdr.ctype = AVRC_RSP_ACCEPT;

        /* NOTE: Not all pass through IDs are supported. Simply pass the supported IDs up to the application. */
        switch ( p_msg->type.pass_thru.hdr.operation_id )
        {
        case AVRC_ID_PLAY:
            wiced_bt_avrc_tg_send_passthrough_cmd(APP_AVRC_EVENT_PASSTHROUGH_CMD_PLAY);
            break;
        case AVRC_ID_PAUSE:
            wiced_bt_avrc_tg_send_passthrough_cmd(APP_AVRC_EVENT_PASSTHROUGH_CMD_PAUSE);
            break;
        case AVRC_ID_STOP:
            wiced_bt_avrc_tg_send_passthrough_cmd(APP_AVRC_EVENT_PASSTHROUGH_CMD_STOP);
            break;
        case AVRC_ID_FORWARD:
            wiced_bt_avrc_tg_send_passthrough_cmd(APP_AVRC_EVENT_PASSTHROUGH_CMD_NEXT_TRACK);
            break;
        case AVRC_ID_BACKWARD:
            wiced_bt_avrc_tg_send_passthrough_cmd(APP_AVRC_EVENT_PASSTHROUGH_CMD_PREVIOUS_TRACK);
            break;
        case AVRC_ID_VOL_UP:
            wiced_bt_avrc_tg_send_passthrough_cmd(APP_AVRC_EVENT_PASSTHROUGH_CMD_VOLUME_UP);
            break;
        case AVRC_ID_VOL_DOWN:
            wiced_bt_avrc_tg_send_passthrough_cmd(APP_AVRC_EVENT_PASSTHROUGH_CMD_VOLUME_DOWN);
            break;
        default:
            WICED_BTAVRCP_TRACE("\n\rWARNING: pass through op_id %d not supported\n\r", p_msg->type.pass_thru.hdr.operation_id);
            break;
        }
    }
    else if (p_msg->type.pass_thru.hdr.state == AVRC_STATE_RELEASE )
    {
        /* No action on the key release. just accept it. */
        /* TODO: ensure that the release matches the press */
        p_msg->hdr.ctype = AVRC_RSP_ACCEPT;
    }

    WICED_BTAVRCP_TRACE( "\n\r Sending the response to peer\n\r" );

    /* send the response */
    wiced_bt_avrc_send_passthrough_rsp( handle, label, p_msg->hdr.ctype, &p_msg->type.pass_thru);
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_command_handler

** Description    AVRC command handler
*******************************************************************************/
void wiced_bt_avrc_tg_command_handler( uint8_t handle, uint8_t label, uint8_t opcode, wiced_bt_avrc_cmd_t *p_msg )
{
    WICED_BTAVRCP_TRACE( "%s: avrc_hdl=0x%x, label=0x%x, opcode=%x\n\r", __FUNCTION__, handle, label, opcode );

    switch ( opcode )
    {
    case AVRC_OP_PASS_THRU:
        wiced_bt_avrc_tg_cmd_pass_through_command_handler( handle, label, opcode, p_msg );
        break;

    case AVRC_OP_VENDOR:        /**< Vendor-dependent commands  */
        wiced_bt_avrc_tg_vendor_command_handler( handle, label, opcode, &p_msg->type.metadata );
        break;

    /* Unhandled */
    case AVRC_OP_BROWSE:    /**< Browsing                   */
    {
       wiced_bt_avrc_tg_browse_command_handler( handle, label, opcode, &p_msg->type.browse_cmd);
    }break;
    case AVRC_OP_UNIT_INFO: /**< Report unit information    */
    case AVRC_OP_SUB_INFO:  /**< Report subunit information */
    default:
        /* TODO: Need to use proper response call instead of passthrough? */

        /* The command default response is AVRC_RSP_NOT_IMPL (NOT IMPLEMENTED) */
        p_msg->hdr.ctype = AVRC_RSP_NOT_IMPL;

        wiced_bt_avrc_send_passthrough_rsp( handle, label,p_msg->hdr.ctype, &p_msg->type.pass_thru);
        break;
    }
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_msg_cback

** Description    message callback handler from stack
*******************************************************************************/
void wiced_bt_avrc_tg_msg_cback( wiced_bt_avrc_msg_t *p_msg )
{

    wiced_bt_avrc_hdr_t *hdr = NULL;
    uint8_t handle = p_msg->handle;
    uint8_t label = p_msg->label;
    uint8_t opcode = p_msg->opcode;
    if(opcode == AVRC_OP_BROWSE)
    { 
        if(p_msg->msg_type == AVRC_CMD)
        {
            wiced_bt_avrc_tg_browse_command_handler(handle, label, opcode, &p_msg->type.command.type.browse_cmd);
        }
        return;
    }
    if (p_msg->msg_type == AVRC_CMD)
    {
        hdr = &p_msg->type.command.hdr;
    }
    else if (p_msg->msg_type == AVRC_RSP)
    {
        hdr = &p_msg->type.response.hdr;
    }
    WICED_BTAVRCP_TRACE("%s: ctype: %s (0x%02x)\n\r", __FUNCTION__,
     wiced_bt_avrc_tg_dump_message_type(hdr->ctype), hdr->ctype);
    /* check if command or response */
    switch ( hdr->ctype )
    {
    case AVRC_CMD_CTRL:
    case AVRC_CMD_STATUS:
    case AVRC_CMD_SPEC_INQ:
    case AVRC_CMD_NOTIF:
    case AVRC_CMD_GEN_INQ:
        wiced_bt_avrc_tg_command_handler( handle, label, opcode, &p_msg->type.command );
        break;

    case AVRC_RSP_NOT_IMPL:
    case AVRC_RSP_ACCEPT:
    case AVRC_RSP_REJ:
    case AVRC_RSP_IN_TRANS:
    case AVRC_RSP_IMPL_STBL:
    case AVRC_RSP_CHANGED:
    case AVRC_RSP_INTERIM:
        wiced_bt_avrc_tg_response_handler( handle, label, opcode, &p_msg->type.response );
        break;

    default:
        WICED_BTAVRCP_TRACE( "%s: UNKNOWN ctype=%x ignored\n\r", __FUNCTION__, hdr->ctype );
        break;
    }
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_ctrl_cback

** Description    Control callback handler from stack
*******************************************************************************/
void wiced_bt_avrc_tg_ctrl_cback( uint8_t avrc_handle, wiced_bt_avrc_ctrl_evt_t event, uint8_t result, wiced_bt_avrc_xmit_buf_t* p_buf, wiced_bt_device_address_t peer_addr )
{
    wiced_bt_rc_event_t avrc_event;
    WICED_BTAVRCP_TRACE( "%s: avrc_hdl=0x%x, event=0x%x, result=%x", __FUNCTION__, avrc_handle, event, result );


    switch ( event )
    {
    case AVRC_OPEN_IND_EVT:
        WICED_BTAVRCP_TRACE( "AVRC TG connection opened to <%B>", peer_addr );
        wiced_bt_avrc_tg_cb.avrc_handle = avrc_handle;

        /* Copy peer_addr so we can send connected event to app once sdp complete */
        memcpy(wiced_bt_avrc_tg_cb.peer_addr, peer_addr, BD_ADDR_LEN);

        /* Get the AVRC sdp record for the remote */
        wiced_bt_avrc_tg_sdp_find_service( peer_addr );
        break;

    case AVRC_CLOSE_IND_EVT:
        WICED_BTAVRCP_TRACE( "AVRC TG connection closed" );
        avrc_event.handle = wiced_bt_avrc_tg_cb.avrc_handle;

        /* Reset last volume setting */
        wiced_bt_avrc_tg_cb.is_abs_volume_capable = WICED_FALSE;
        wiced_bt_avrc_tg_cb.last_abs_volume = -1;
#ifdef APP_AVRC_TRACK_INFO_SUPPORTED
        /* reset the track information*/
        memset(&wiced_bt_avrc_tg_cb.app_track_attr,0,sizeof(wiced_bt_avrc_tg_cb.app_track_attr));
#endif
        if(wiced_bt_avrc_tg_cb.p_event_cb)
            (wiced_bt_avrc_tg_cb.p_event_cb)(APP_AVRC_EVENT_DEVICE_DISCONNECTED, &avrc_event);

        break;

    case AVRC_CMD_TIMEOUT_EVT:
        WICED_BTAVRCP_TRACE( "AVRC TG connection timeout" );
        break;

    case AVRC_CONG_IND_EVT:
        WICED_BTAVRCP_TRACE( "AVRC TG congested" );
        break;

    case AVRC_UNCONG_IND_EVT:
        WICED_BTAVRCP_TRACE( "AVRC TG uncongested" );
        break;
    case AVRC_APP_BUFFER_TX_EVT:
         WICED_BTAVRCP_TRACE("%s handle[%d] buf: %p  sent_ok: %d\n", __FUNCTION__, avrc_handle, p_buf, result);
            wiced_bt_free_buffer(p_buf);
    break;

        /* AVRC 1.4 browsing events currently not supported by BTA_RC */
    case AVRC_BROWSE_OPEN_IND_EVT:
         WICED_BTAVRCP_TRACE( "AVRC TG- AVRC_BROWSE_OPEN_IND_EVT" );
         wiced_bt_avrc_set_browse_drb(avrc_handle , wiced_bt_avrc_tg_cb.p_browse_drb, wiced_bt_avrc_tg_cb.avrc_br_mtu, NULL);
        
    break;
    case AVRC_BROWSE_CLOSE_IND_EVT:
    case AVRC_BROWSE_CONG_IND_EVT:
    case AVRC_BROWSE_UNCONG_IND_EVT:
    default:
        WICED_BTAVRCP_TRACE("unhandled avrc event (0x%x)\n\r", event );
        break;
    }
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_init_app_data

** Description    Initialize application data (cached data from MCU application)
*******************************************************************************/
void wiced_bt_avrc_tg_init_app_data(void)
{
#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
    memset(player_settings, 0, sizeof(wiced_bt_avrc_tg_player_attr_t) * APP_AVRC_SETTING_SUPPORTED_MAX);
#endif

#ifdef APP_AVRC_TRACK_INFO_SUPPORTED
    /* Initialize the track index */
{
    int i;
    for (i=0; i<8; i++)
    {
        wiced_bt_avrc_tg_cb.app_track_attr[AVRC_MEDIA_ATTR_ID_TRACK_NUM].p_str[0] = 'F';
    }
}
#endif

}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_init

** Description    Called to initialize AV RC profile
*******************************************************************************/
void wiced_bt_avrc_tg_init( wiced_bt_avrc_tg_event_cback_t *pcb )
{

    if(wiced_bt_avrc_tg_cb.p_avct_buf)
        wiced_bt_free_buffer(wiced_bt_avrc_tg_cb.p_avct_buf);
    if(wiced_bt_avrc_tg_cb.p_avrc_buf)
        wiced_bt_free_buffer(wiced_bt_avrc_tg_cb.p_avrc_buf);
    if(wiced_bt_avrc_tg_cb.p_browse_drb)
        wiced_bt_free_buffer(wiced_bt_avrc_tg_cb.p_browse_drb);
    /* Init the AVCT specific values */
    memset(&wiced_bt_avrc_tg_cb, 0, sizeof(wiced_bt_avrc_tg_cb));
    wiced_bt_avrc_tg_cb.avrc_handle = INVALID_AVRC_HANDLE;

    /* parameters which are set at registration. */
    wiced_bt_avrc_tg_cb.avrc_mtu    = AVCT_CONTROL_MTU;
    wiced_bt_avrc_tg_cb.avrc_br_mtu = AVCT_MIN_BROWSE_MTU;
    wiced_bt_avrc_tg_cb.features    = AV_FEAT_INT | AV_FEAT_ACP | AV_FEAT_TARGET | AV_FEAT_CONTROL;
    /* event callback */
    wiced_bt_avrc_tg_cb.p_event_cb  = pcb;

    /* Initialize application data*/
    wiced_bt_avrc_tg_init_app_data();
}

void wiced_bt_avrc_tg_deinit()
{
    WICED_BTAVRCP_TRACE("[%s]", __FUNCTION__ );

    /* Init the AVCT specific values */
    if(wiced_bt_avrc_tg_cb.p_avct_buf != NULL)
    {
        wiced_bt_free_buffer(wiced_bt_avrc_tg_cb.p_avct_buf);
        wiced_bt_avrc_tg_cb.p_avct_buf = NULL;
    }
    if(wiced_bt_avrc_tg_cb.p_avrc_buf != NULL)
    {
        wiced_bt_free_buffer(wiced_bt_avrc_tg_cb.p_avrc_buf);
        wiced_bt_avrc_tg_cb.p_avrc_buf = NULL;
    }
    if(wiced_bt_avrc_tg_cb.p_browse_drb != NULL)
    {
        wiced_bt_free_buffer(wiced_bt_avrc_tg_cb.p_browse_drb);
        wiced_bt_avrc_tg_cb.p_browse_drb =  NULL;
    }
    if(wiced_bt_avrc_tg_cb.p_sdp_db_avrc != NULL)
    {
        wiced_bt_free_buffer(wiced_bt_avrc_tg_cb.p_sdp_db_avrc);
        wiced_bt_avrc_tg_cb.p_sdp_db_avrc =  NULL;
    }
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_open

** Description    Called to initialize AVRCP connection for given role
*******************************************************************************/
wiced_result_t wiced_bt_avrc_tg_open(wiced_bt_device_address_t peer_addr)
{
    wiced_bt_avrc_config_t avrc_conn_cb;
    uint16_t avrc_status;
    uint8_t  role;
    uint32_t feature_flag;

    if ( memcmp(null_bda, peer_addr, BD_ADDR_LEN) )
    {
        role = AVRC_CONN_INITIATOR;
        feature_flag = AV_FEAT_INT;
    }
    else
    {
        role = AVRC_CONN_ACCEPTOR;
        feature_flag = AV_FEAT_ACP;
    }

    if (wiced_bt_avrc_tg_cb.avrc_handle != INVALID_AVRC_HANDLE)
    {
        if (wiced_bt_avrc_tg_cb.conn_role == role)
        {
            WICED_BTAVRCP_TRACE (" %s already connected as role %d \n",__FUNCTION__, role);
            return WICED_ALREADY_CONNECTED;
        }
        else
        {
            if ( ( avrc_status = wiced_bt_avrc_close( wiced_bt_avrc_tg_cb.avrc_handle ) ) == AVRC_SUCCESS )
            {
                WICED_BTAVRCP_TRACE (" %s close acceptor channel success status %d \n",__FUNCTION__, avrc_status);
            }
            else
            {
                WICED_BTAVRCP_TRACE (" %s close acceptor channel failed status %d \n",__FUNCTION__, avrc_status);
                return WICED_ERROR;
            }
        }
    }

    /* Register with AVRC as acceptor (Here, target is acceptor) */
    if ( wiced_bt_avrc_tg_cb.features & feature_flag )
    {
        avrc_conn_cb.connection_role        = role;
        avrc_conn_cb.control      = ( ( wiced_bt_avrc_tg_cb.features & AV_FEAT_CONTROL ) ? AVRC_CT_CONTROL : 0 );
        avrc_conn_cb.control     |= ( ( wiced_bt_avrc_tg_cb.features & AV_FEAT_TARGET ) ? AVRC_CT_TARGET : 0 );
        avrc_conn_cb.p_ctrl_cback = ( wiced_bt_avrc_ctrl_cback_t * ) wiced_bt_avrc_tg_ctrl_cback;
        avrc_conn_cb.p_msg_cback  = ( wiced_bt_avrc_msg_cback_t * ) wiced_bt_avrc_tg_msg_cback;
        if (wiced_bt_avrc_tg_cb.p_avct_buf == NULL)
            wiced_bt_avrc_tg_cb.p_avct_buf = wiced_bt_get_buffer(AVRC_MAX_AVCT_RCV_PKT_SIZE);
        avrc_conn_cb.p_avct_buff = wiced_bt_avrc_tg_cb.p_avct_buf;
        avrc_conn_cb.avct_seg_buf_len = AVRC_MAX_AVCT_RCV_PKT_SIZE;

        if (wiced_bt_avrc_tg_cb.p_avrc_buf == NULL)
            wiced_bt_avrc_tg_cb.p_avrc_buf = wiced_bt_get_buffer(AVRC_MAX_METADATA_RCV_MSG_SIZE);
        avrc_conn_cb.p_avrc_buff = wiced_bt_avrc_tg_cb.p_avrc_buf;
        avrc_conn_cb.avrc_seg_buf_len = AVRC_MAX_METADATA_RCV_MSG_SIZE;
         if (wiced_bt_avrc_tg_cb.p_browse_drb == NULL)
                wiced_bt_avrc_tg_cb.p_browse_drb = (tDRB*)wiced_bt_get_buffer(wiced_bt_avrc_tg_cb.avrc_br_mtu + DRB_OVERHEAD_SIZE);

        WICED_BTAVRCP_TRACE("conn (ACP = 1/Init = 0) =%d, role(target=1 / controller=2) =%d\n\r",
                       role, avrc_conn_cb.control);
        avrc_status  = wiced_bt_avrc_init(wiced_bt_avrc_tg_cb.avrc_mtu, wiced_bt_avrc_tg_cb.avrc_br_mtu);
        if(avrc_status != AVRC_SUCCESS)
        {
            WICED_BTAVRCP_TRACE("%s:wiced_bt_avrc_init[TG] : FAILED (%d) \n", __FUNCTION__, avrc_status);
        }

        if ( ( avrc_status = wiced_bt_avrc_open( &wiced_bt_avrc_tg_cb.avrc_handle, &avrc_conn_cb, peer_addr ) ) == AVRC_SUCCESS )
        {
            wiced_bt_avrc_tg_cb.conn_role = role;
            WICED_BTAVRCP_TRACE( "avrc open success for %d role (avrc status 0x%x)\n\r", role, avrc_status );

            wiced_bt_avrc_open_browse(wiced_bt_avrc_tg_cb.avrc_handle, AVRC_CONN_ACCEPTOR);
        }
        else
        {
            WICED_BTAVRCP_TRACE( "avrc open failed for %d role (avrc error 0x%x)\n\r", role, avrc_status );
        }
        return WICED_SUCCESS;
    }
    return WICED_ERROR;
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_register

** Description    Called to register AVRC profile
*******************************************************************************/
void wiced_bt_avrc_tg_register(void)
{
    WICED_BTAVRCP_TRACE( "%s Enter... features: 0x%x\n\r", __FUNCTION__, wiced_bt_avrc_tg_cb.features );
    wiced_bt_avrc_tg_open(null_bda);
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_initiate_open

** Description    Called to initiate connection to given BDA
*******************************************************************************/
void wiced_bt_avrc_tg_initiate_open( wiced_bt_device_address_t peer_addr )
{
    WICED_BTAVRCP_TRACE( "%s Enter... Peer: <%B> features: 0x%x\n\r", __FUNCTION__, peer_addr, wiced_bt_avrc_tg_cb.features );

    wiced_bt_avrc_tg_open(peer_addr);
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_initiate_close

** Description    Called to disconnect AVRC connection
*******************************************************************************/
void wiced_bt_avrc_tg_initiate_close( void )
{
    WICED_BTAVRCP_TRACE( "%s handle: %d\n\r", __FUNCTION__, wiced_bt_avrc_tg_cb.avrc_handle );

    if (wiced_bt_avrc_tg_cb.avrc_handle != INVALID_AVRC_HANDLE)
    {
        wiced_bt_avrc_close( wiced_bt_avrc_tg_cb.avrc_handle );
    }
    wiced_bt_avrc_tg_cb.avrc_handle = INVALID_AVRC_HANDLE;
}

/*******************************************************************************
* Function        wiced_bt_avrc_tg_control_volume

** Description    Called when volume is changed, send Absolute volume request to peer
*******************************************************************************/
wiced_result_t wiced_bt_avrc_tg_absolute_volume_changed(uint16_t handle,   uint8_t  volume )
{
    wiced_bt_avrc_metadata_cmd_t cmd;

    wiced_result_t status = WICED_SUCCESS;

    WICED_BTAVRCP_TRACE( "%s handle:%x volume value: 0x%x\n\r", __FUNCTION__, handle, volume );

    WICED_BTAVRCP_TRACE("avrc_handle:%x", wiced_bt_avrc_tg_cb.avrc_handle);

    if (wiced_bt_avrc_tg_cb.avrc_handle == INVALID_AVRC_HANDLE)
    {
        return WICED_ERROR;
    }

        if (volume > MAX_AVRCP_VOLUME_LEVEL)
        {
            volume = MAX_AVRCP_VOLUME_LEVEL;
        }

    /* If remote has registered for volume notifications, then notify else send absolute volume command */
    if (wiced_bt_avrc_tg_cb.registered_event_mask & (1 << (AVRC_EVT_VOLUME_CHANGE - 1)))
    {
        wiced_bt_avrc_tg_complete_notification(AVRC_EVT_VOLUME_CHANGE);
    }
    else
    {
        cmd.metadata_hdr.pdu      = AVRC_PDU_SET_ABSOLUTE_VOLUME;  /**< PDU ID AVRC_PDU_SET_ABSOLUTE_VOLUME */
        cmd.u.volume   = volume;                        /**< Absolute Volume */

        uint8_t label = 3; /* TODO: Need to do transaction label accounting. */
        if (wiced_avrc_build_and_send_metadata_cmd(&cmd, AVRC_CMD_CTRL, wiced_bt_avrc_tg_cb.avrc_handle, label) != AVRC_STS_NO_ERROR)
        {
            status = WICED_ERROR;
            /* TODO: Message not sent. Need to release the label */
        }

    }

    return status;
}

#ifdef APP_AVRC_TRACK_INFO_SUPPORTED
/*******************************************************************************
* Function        wiced_bt_rc_set_track_info

** Description    Called to set current playing track information
*******************************************************************************/
void wiced_bt_rc_set_track_info(wiced_bt_avrc_tg_track_attr_t *p_track_attr)
{
    if(p_track_attr->attr_id > APP_AVRC_MAX_ATTR)
        return;

    memcpy(&wiced_bt_avrc_tg_cb.app_track_attr[p_track_attr->attr_id], p_track_attr, sizeof(wiced_bt_avrc_tg_track_attr_t));

    WICED_BTAVRCP_TRACE( "%s : attr_id %d, len %d\n\r", __FUNCTION__, p_track_attr->attr_id,
                    wiced_bt_avrc_tg_cb.app_track_attr[p_track_attr->attr_id].str_len);
}
#endif

#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
/*******************************************************************************
* Function        wiced_bt_rc_set_player_settings

** Description    Called to set player settings (repeat, shuffle, etc).
*******************************************************************************/
void wiced_bt_rc_set_player_settings(wiced_bt_avrc_tg_player_attr_t *p_info)
{
    int i, index = 0;

    for(i = 0; i < APP_AVRC_SETTING_SUPPORTED_MAX; i++)
    {
        if(player_settings[i].attr_id == p_info->attr_id)
        {
            index = i;
            break;
        }
        else if(player_settings[i].attr_id != 0)
            index++;
    }

    if(index < APP_AVRC_SETTING_SUPPORTED_MAX)
    {
        memcpy(&player_settings[index], p_info, sizeof(wiced_bt_avrc_tg_player_attr_t));

        WICED_BTAVRCP_TRACE( "%s : index %d, id %d\n\r", __FUNCTION__, index,
                player_settings[index].attr_id);
    }
}
#endif

#ifdef APP_AVRC_PLAY_STATUS_SUPPORTED

/*******************************************************************************
* Function        wiced_bt_rc_set_player_status

** Description    Called to set player status (pause/play, song position)
*******************************************************************************/
void wiced_bt_rc_set_player_status(wiced_bt_avrc_tg_play_status_t *p_info)
{
    uint8_t old_state     = wiced_bt_avrc_tg_cb.player_status.play_state;
    uint32_t old_position = wiced_bt_avrc_tg_cb.player_status.song_pos;

    memcpy(&wiced_bt_avrc_tg_cb.player_status, p_info, sizeof(wiced_bt_avrc_tg_play_status_t));

    WICED_BTAVRCP_TRACE( "%s: new play_state %d song_len: %d\n\r", __FUNCTION__,
                    p_info->play_state, p_info->song_len);

    /* If there was a change in state let the remote know if it has registered for the event */
    if ( (old_state != p_info->play_state) || (old_position != p_info->song_pos) )
    {
        wiced_bt_avrc_tg_complete_notification(AVRC_EVT_PLAY_STATUS_CHANGE);
    }
}
#endif

#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
/*******************************************************************************
* Function        wiced_bt_rc_player_setting_changed

** Description    Called with player setting (repeat, shuffle) is changed
*******************************************************************************/
void wiced_bt_rc_player_setting_changed(uint8_t attr_id, uint8_t value)
{
    uint8_t i = 0;
    for(i = 0; i < APP_AVRC_SETTING_SUPPORTED_MAX; i++)
    {
        if(player_settings[i].attr_id == attr_id)
        {
            player_settings[i].curr_value = value;
            break;
        }
    }

    WICED_BTAVRCP_TRACE( "%s : id %d, val %d\n\r", __FUNCTION__, attr_id, value);

    wiced_bt_avrc_tg_complete_notification(AVRC_EVT_APP_SETTING_CHANGE);
}
#endif

#ifdef APP_AVRC_TRACK_INFO_SUPPORTED
/*******************************************************************************
* Function        wiced_bt_rc_track_changed

** Description    Called when current track is changed
*******************************************************************************/
void wiced_bt_rc_track_changed(void)
{
    WICED_BTAVRCP_TRACE( "%s\n\r", __FUNCTION__);
    wiced_bt_avrc_tg_complete_notification(AVRC_EVT_TRACK_CHANGE);
}
#endif
