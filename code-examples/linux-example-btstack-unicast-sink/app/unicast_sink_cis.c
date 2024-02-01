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

#include "unicast_sink_gatt.h"
#include "unicast_sink_isoc.h"

#include "wiced_bt_cfg.h"
#include "wiced_bt_ga_ascs.h"
#include "wiced_bt_isoc.h"
#include "wiced_bt_trace.h"

#include "log.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define DEFAULT_CIS_ID 0xFF
#define DEFAULT_CIG_ID 0xFF

static ga_iso_audio_stream_info_t g_stream_info[MAX_STREAMS_SUPPORTED] = {0};

/******************************************************************************
 * Function Name: init_stream_info
 ******************************************************************************
 * Summary: init the stream info
 *
 * Parameters:
 *  uint16_t conn_hdl
 *  wiced_bool_t is_cis
 *  uint32_t sampling_frequency
 *  uint16_t octets_per_frame
 *  uint32_t frame_duration
 *  uint8_t num_of_channels
 *
 * Return:
 *  None
 *
******************************************************************************/
void init_stream_info(uint16_t conn_hdl,
                      wiced_bool_t is_cis,
                      uint32_t sampling_frequency,
                      uint16_t octets_per_frame,
                      uint32_t frame_duration,
                      uint8_t num_of_channels)
{
    WICED_BT_TRACE("[%s] [is_cis %d] [SF %d] [OPF %d] [FD %d] [num_of_channels %d]\n",
                   __FUNCTION__,
                   is_cis,
                   sampling_frequency,
                   octets_per_frame,
                   frame_duration,
                   num_of_channels);

    for (size_t i = 0; i < MAX_STREAMS_SUPPORTED; i++)
    {
        if (!g_stream_info[i].conn_hdl)
        {
            g_stream_info[i].conn_hdl = conn_hdl;
            g_stream_info[i].is_cis = is_cis;
            g_stream_info[i].octets_per_frame = octets_per_frame;
            g_stream_info[i].num_of_channels = num_of_channels;
            g_stream_info[i].sampling_frequency = sampling_frequency;
            g_stream_info[i].frame_duration = frame_duration;
            g_stream_info[i].b_stream_active = 1;

            WICED_BT_TRACE("[%s] conn_hdl 0x%x \n", __FUNCTION__, conn_hdl);
            return;
        }
    }
}

/******************************************************************************
 * Function Name: get_stream_info
 ******************************************************************************
 * Summary: get the stream info by conn_hdl
 *
 * Parameters:
 *  uint16_t conn_hdl
 *
 * Return:
 *  ga_iso_audio_stream_info_t *
 *
******************************************************************************/
ga_iso_audio_stream_info_t *get_stream_info(uint16_t conn_hdl)
{
    for (size_t i = 0; i < MAX_STREAMS_SUPPORTED; i++)
    {
        if (conn_hdl == g_stream_info[i].conn_hdl)
        {
            return &g_stream_info[i];
        }
    }

    return NULL;
}

/******************************************************************************
 * Function Name: iso_audio_stop_stream
 ******************************************************************************
 * Summary: set stream info to stop, only use in update ase, for stop ready
 *
 * Parameters:
 *  uint16_t conn_hdl
 *
 * Return:
 *  None
 *
******************************************************************************/
void iso_audio_stop_stream(uint16_t conn_hdl)
{
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);
    p_stream_info->b_stream_active = 0;
}

/******************************************************************************
 * Function Name: deinit_stream_info(
 ******************************************************************************
 * Summary: deinit the stream info
 *
 * Parameters:
 *  uint16_t conn_hdl
 *
 * Return:
 *  None
 *
******************************************************************************/
static void deinit_stream_info(uint16_t conn_hdl)
{
    WICED_BT_TRACE("[%s] [conn_hdl %d]\n", __FUNCTION__, conn_hdl);

    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);
    if (!p_stream_info) return;

    memset(p_stream_info, 0, sizeof(ga_iso_audio_stream_info_t));
    WICED_BT_TRACE("[%s] deinit successful\n", __FUNCTION__);
}

/******************************************************************************
 * Function Name: iso_audio_setup_data_path
 ******************************************************************************
 * Summary: setup data path when cis connection
 *
 * Parameters:
 *  uint16_t conn_hdl
 *  uint16_t direction
 *  wiced_bt_ga_bap_csc_t *p_csc
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t iso_audio_setup_data_path(uint16_t conn_hdl, uint16_t direction, wiced_bt_ga_bap_csc_t *p_csc)
{
    wiced_bool_t is_cis = FALSE;

    WICED_BT_TRACE("[%s] conn_hdl [0x%x] dir [%d] p_csc [0x%x]\n", __FUNCTION__, conn_hdl, direction, p_csc);
    if (!p_csc) return WICED_BADARG;

    /* check if CIS is established or BIS is created before setting up data path */
    is_cis = wiced_bt_isoc_is_cis_connected_by_conn_id(conn_hdl);
    if (!is_cis)
    {
        return WICED_BADARG;
    }

    WICED_BT_TRACE("[%s] dir %s SR %d sduInterval %d OPF %d conn_hdl 0x%x is_cis %d\n",
                   __FUNCTION__,
                   (direction == WICED_BLE_ISOC_DPD_INPUT_BIT) ? "Source" : "Sink",
                   p_csc->sampling_frequency,
                   p_csc->frame_duration,
                   p_csc->octets_per_codec_frame,
                   conn_hdl,
                   is_cis);

    if (!p_csc->sampling_frequency || !p_csc->frame_duration || !p_csc->octets_per_codec_frame) return WICED_ERROR;

    unicast_sink_isoc_dhm_setup_stream(conn_hdl, is_cis, direction, p_csc);

    return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: iso_audio_remove_data_path
 ******************************************************************************
 * Summary: remove data path when cis disconnection
 *
 * Parameters:
 *  uint16_t conn_hdl
 *  wiced_bt_isoc_data_path_bit_t dir
 *  uint8_t *idx_list
 *
 * Return:
 *  None
 *
******************************************************************************/
void iso_audio_remove_data_path(uint16_t conn_hdl, wiced_bt_isoc_data_path_bit_t dir, uint8_t *idx_list)
{
    uint8_t channel_count = 1;

    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);
    if (p_stream_info == NULL)
    {
        TRACE_ERR("p_stream_info is NULL\n");
        return;
    }

    channel_count = p_stream_info->num_of_channels;

    WICED_BT_TRACE("[%s] [dir %d] [conn_hdl 0x%x] [num of channel %d]", __FUNCTION__, dir, conn_hdl, channel_count);

    if (!wiced_bt_isoc_remove_data_path(conn_hdl, TRUE, dir))
        WICED_BT_TRACE_CRIT("[%s] No active data path\n", __FUNCTION__);

    deinit_stream_info(conn_hdl);
    unicast_sink_isoc_dhm_free_stream(conn_hdl, dir, idx_list, channel_count);
}

/******************************************************************************
 * Function Name: unicast_sink_find_ase_with_cis_id
 ******************************************************************************
 * Summary: find the ase by cis_id
 *
 * Parameters:
 *  uint8_t cig_id
 *  uint8_t cis_id
 *  uint8_t char_type
 *  uint16_t *p_conn_id
 *
 * Return:
 *  unicast_sink_ase_data_t*
 *
******************************************************************************/
unicast_sink_ase_data_t *unicast_sink_find_ase_with_cis_id(uint8_t cig_id,
                                                           uint8_t cis_id,
                                                           uint8_t char_type,
                                                           uint16_t *p_conn_id)
{
    unicast_sink_clcb_t *p_clcb = g_unicast_sink_gatt_cb.unicast_clcb;
    int limit = sizeof(g_unicast_sink_gatt_cb.unicast_clcb) / sizeof(g_unicast_sink_gatt_cb.unicast_clcb[0]);
    unicast_sink_ase_data_t *p_ase = NULL;
    int num_ase = 0;

    while (limit--)
    {
        p_ase = p_clcb->p_local_ase_data;
        num_ase = p_clcb->num_local_ases;
        if (char_type == ASCS_SINK_ASE_CHARACTERISTIC)
        {
            for (; num_ase--; p_ase++)
            {
                if (p_ase->data.ase_state < WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED)
                {
                    continue;
                }
                if ((p_ase->data.qos_configured.cig_id == cig_id) && (p_ase->data.qos_configured.cis_id == cis_id))
                {
                    if (p_conn_id)
                    {
                        *p_conn_id = p_clcb->conn_id;
                    }
                    return p_ase;
                }
            }
        }
        p_clcb++;
    }

    return NULL;
}

/******************************************************************************
 * Function Name: unicast_sink_get_ase_app_data_ptr_by_cis_conn_hdl
 ******************************************************************************
 * Summary: get the ase by cis_conn_handle
 *
 * Parameters:
 *  uint16_t cis_conn_hdl
 *  uint8_t char_type
 *  uint16_t *p_conn_id
 *
 * Return:
 *  unicast_sink_ase_data_t*
 *
******************************************************************************/
unicast_sink_ase_data_t *unicast_sink_get_ase_app_data_ptr_by_cis_conn_hdl(uint16_t cis_conn_hdl,
                                                                           uint8_t char_type,
                                                                           uint16_t *p_conn_id)
{
    unicast_sink_clcb_t *p_clcb = g_unicast_sink_gatt_cb.unicast_clcb;
    int limit = sizeof(g_unicast_sink_gatt_cb.unicast_clcb) / sizeof(g_unicast_sink_gatt_cb.unicast_clcb[0]);
    unicast_sink_ase_data_t *p_ase = NULL;
    int num_ase = 0;
    while (limit--)
    {
        p_ase = p_clcb->p_local_ase_data;
        num_ase = p_clcb->num_local_ases;
        for (; num_ase--; p_ase++)
        {
            if (p_ase->data.ase_state < WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED)
            {
                continue;
            }
            if (p_ase->cis_conn_hdl == cis_conn_hdl)
            {
                if (p_conn_id)
                {
                    *p_conn_id = p_clcb->conn_id;
                }
                return p_ase;
            }
        }
        p_clcb++;
    }

    return NULL;
}

/******************************************************************************
 * Function Name: unicast_sink_cis_handle_connection 
 ******************************************************************************
 * Summary: cis handle connection
 *
 * Parameters:
 *  uint8_t cig_id
 *  uint8_t cis_id
 *  uint8_t char_type
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_sink_cis_handle_connection(uint8_t cig_id, uint8_t cis_id, uint8_t char_type)
{
    wiced_result_t res = WICED_ERROR;
    uint16_t conn_id;
    unicast_sink_ase_data_t *p_ase_data = unicast_sink_find_ase_with_cis_id(cig_id, cis_id, char_type, &conn_id);
    wiced_bt_isoc_data_path_direction_t data_path_dir = WICED_BLE_ISOC_DPD_MAX_DIRECTIONS;
    gatt_intf_service_object_t *p_service = NULL;

    CHECK_FOR_NULL_AND_RETURN(p_ase_data);

    WICED_BT_TRACE("[%s] ase_id %d, state : %d characteristic_type %d",
                   __FUNCTION__,
                   p_ase_data->data.p_ase_info->ase_id,
                   p_ase_data->data.ase_state,
                   p_ase_data->data.p_ase_info->ase_type);

    //assign CIS to ASE
    p_ase_data->cis_conn_hdl = wiced_bt_isoc_get_cis_conn_handle(cig_id, cis_id);

    // if (p_ase_data->data_path_established) {
    //     p_service = gatt_interface_get_service_by_uuid_and_conn_id(0, &ga_service_uuid_ascs);
    //     CHECK_FOR_NULL_AND_RETURN(p_service);

    //     unicast_sink_isoc_dhm_start_stream();

    //     // if server + source and in streaming state, start audio streaming
    //     // if server + sink and in enabling state, transition to streaming state
    //     if (ASCS_SINK_ASE_CHARACTERISTIC == char_type && WICED_BT_GA_ASCS_STATE_ENABLING == p_ase_data->data.ase_state)
    //     {
    //         gatt_intf_attribute_t ase_char;

    //         p_ase_data->data.ase_state = WICED_BT_GA_ASCS_STATE_STREAMING;
    //         gatt_interface_notify_characteristic(conn_id,
    //                                              p_service,
    //                                              ascs_init_characteristic(&ase_char, p_ase_data),
    //                                              &p_ase_data->data);
    //     }
    //     else {
    //         WICED_BT_TRACE_CRIT("[%s] sr unexpected state %d char_type %d\n",
    //                             __FUNCTION__,
    //                             p_ase_data->data.ase_state,
    //                             char_type);
    //     }

    //     return;
    // }

    if (ASCS_SINK_ASE_CHARACTERISTIC == p_ase_data->data.p_ase_info->ase_type)
        res = iso_audio_setup_data_path(p_ase_data->cis_conn_hdl,
                                        WICED_BLE_ISOC_DPD_OUTPUT_BIT,
                                        &p_ase_data->data.codec_configured.csc);
    else
        res = iso_audio_setup_data_path(p_ase_data->cis_conn_hdl,
                                        WICED_BLE_ISOC_DPD_INPUT_BIT,
                                        &p_ase_data->data.codec_configured.csc);

    if (res)
    {
        WICED_BT_TRACE_CRIT("[%s] data path setup unsuccessful...(err:%d)\n", __FUNCTION__, res);
    }
}

/******************************************************************************
 * Function Name: unicast_sink_cis_handle_data_path_setup
 ******************************************************************************
 * Summary: cis handle data path setup for WICED_BLE_ISOC_DATA_PATH_SETUP case
 *
 * Parameters:
 *  uint16_t cis_conn_hdl
 *  wiced_bt_isoc_data_path_direction_t dpd
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_sink_cis_handle_data_path_setup(uint16_t cis_conn_hdl, wiced_bt_isoc_data_path_direction_t dpd)
{
    wiced_bool_t is_inp = (WICED_BLE_ISOC_DPD_INPUT == dpd);
    uint8_t char_type = (is_inp) ? ASCS_SOURCE_ASE_CHARACTERISTIC : ASCS_SINK_ASE_CHARACTERISTIC;
    uint16_t conn_id = 0;
    unicast_sink_ase_data_t *p_ase_data =
        unicast_sink_get_ase_app_data_ptr_by_cis_conn_hdl(cis_conn_hdl, char_type, &conn_id);
    gatt_intf_service_object_t *p_service = NULL;

    CHECK_FOR_NULL_AND_RETURN(p_ase_data);

    WICED_BT_TRACE("[%s] is_inp %d \n", __FUNCTION__, is_inp);

    p_service = gatt_interface_get_service_by_uuid_and_conn_id(0, &ga_service_uuid_ascs);
    CHECK_FOR_NULL_AND_RETURN(p_service);

    // Update ASE data to indicate data path is setup successfully
    p_ase_data->data_path_established = 1;

    unicast_sink_isoc_dhm_start_stream();

    // if server + source and in streaming state, start audio streaming
    // if server + sink and in enabling state, transition to streaming state
    if (ASCS_SINK_ASE_CHARACTERISTIC == char_type && WICED_BT_GA_ASCS_STATE_ENABLING == p_ase_data->data.ase_state)
    {
        gatt_intf_attribute_t ase_char = {0};

        p_ase_data->data.ase_state = WICED_BT_GA_ASCS_STATE_STREAMING;
        gatt_interface_notify_characteristic(conn_id,
                                             p_service,
                                             ascs_init_characteristic(&ase_char, p_ase_data),
                                             &p_ase_data->data);
    }
    else
    {
        WICED_BT_TRACE_CRIT("[%s] sr unexpected state %d char_type %d\n",
                            __FUNCTION__,
                            p_ase_data->data.ase_state,
                            char_type);
    }
}

/******************************************************************************
 * Function Name: unicast_sink_cis_handle_disconnection
 ******************************************************************************
 * Summary: cis handle disconnection
 *
 * Parameters:
 *  uint8_t cig_id
 *  uint8_t cis_id
 *  uint8_t char_type
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_sink_cis_handle_disconnection(uint8_t cig_id, uint8_t cis_id, uint8_t char_type)
{
    unicast_sink_ase_data_t *p_ase_data = NULL;
    gatt_intf_service_object_t *p_service = NULL;
    uint16_t conn_id = 0;
    gatt_intf_attribute_t ase_char = {0};

    p_ase_data = unicast_sink_find_ase_with_cis_id(cig_id, cis_id, char_type, &conn_id);
    CHECK_FOR_NULL_AND_RETURN(p_ase_data);

    iso_audio_remove_data_path(p_ase_data->cis_conn_hdl, p_ase_data->data.p_ase_info->ase_type, &p_ase_data->lc3_index);

    if (!conn_id)
    {
        WICED_BT_TRACE_CRIT("[%s] conn_id is 0", __FUNCTION__);
        conn_id = 0x8000;
        //return;
    }

    p_service = gatt_interface_get_service_by_uuid_and_conn_id(0, &ga_service_uuid_ascs);

    CHECK_FOR_NULL_AND_RETURN(p_service);
    p_ase_data->data_path_established = 0;

    if (WICED_BT_GA_ASCS_STATE_RELEASING == p_ase_data->data.ase_state)
    {
        p_ase_data->data.ase_state = WICED_BT_GA_ASCS_STATE_IDLE;

        wiced_bt_isoc_peripheral_remove_cig(p_ase_data->data.qos_configured.cig_id);
        p_ase_data->data.qos_configured.cis_id = DEFAULT_CIS_ID;
        p_ase_data->data.qos_configured.cig_id = DEFAULT_CIG_ID;
    }
    else if (WICED_BT_GA_ASCS_STATE_STREAMING == p_ase_data->data.ase_state)
    {
        if (ASCS_SOURCE_ASE_CHARACTERISTIC != p_ase_data->data.p_ase_info->ase_type)
        {
            p_ase_data->data.ase_state = WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED;
        }
        else
        {
            p_ase_data->data.ase_state = WICED_BT_GA_ASCS_STATE_DISABLING;
        }
    }
    else if (WICED_BT_GA_ASCS_STATE_DISABLING == p_ase_data->data.ase_state)
    {
        p_ase_data->data.ase_state = WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED;
    }
    else if (WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED == p_ase_data->data.ase_state)
    {
        wiced_bt_isoc_peripheral_remove_cig(p_ase_data->data.qos_configured.cig_id);
    }
    else
    {
        return;
    }
    gatt_interface_notify_characteristic(conn_id,
                                         p_service,
                                         ascs_init_characteristic(&ase_char, p_ase_data),
                                         &p_ase_data->data);
}

/******************************************************************************
 * Function Name: set_conn_on_ase_id
 ******************************************************************************
 * Summary: when CIS REQUEST, get the ase by cig_id, cis_id and save the cis_conn_handle of request
 *
 * Parameters:
 *  wiced_bt_isoc_cis_request_data_t *p_cis_req
 *  int type
 *
 * Return:
 *  int
 *
******************************************************************************/
int set_conn_on_ase_id(wiced_bt_isoc_cis_request_data_t *p_cis_req, int type)
{
    uint16_t conn_id;
    unicast_sink_ase_data_t *p_ase =
        unicast_sink_find_ase_with_cis_id(p_cis_req->cig_id, p_cis_req->cis_id, type, &conn_id);
    int ase_id = (p_ase) ? p_ase->data.p_ase_info->ase_id : 0xFF;

    WICED_BT_TRACE("[%s] cis_id %d cig_id %d cis_conn_handle %d acl_handle %d p_ase %x ase_id %d\n",
                   __FUNCTION__,
                   p_cis_req->cis_id,
                   p_cis_req->cig_id,
                   p_cis_req->cis_conn_handle,
                   p_cis_req->acl_handle,
                   p_ase,
                   ase_id);

    if (p_ase)
    {
        p_ase->cis_conn_hdl = p_cis_req->cis_conn_handle;
    }

    return ase_id;
}

/******************************************************************************
 * Function Name: unicast_sink_isoc_event_handler
 ******************************************************************************
 * Summary: callback of isoc event, register at wiced_bt_isoc_register_cb 
 *
 * Parameters:
 *  wiced_bt_isoc_event_t event
 *  wiced_bt_isoc_event_data_t *p_event_data
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_isoc_event_handler(wiced_bt_isoc_event_t event, wiced_bt_isoc_event_data_t *p_event_data)
{
    unicast_sink_ase_data_t *p_ase_data = NULL;

    WICED_BT_TRACE("[%s] event %d ", __FUNCTION__, event);

    switch (event)
    {
        case WICED_BLE_ISOC_CIS_REQUEST: {
            wiced_bt_isoc_cis_request_data_t *p_cis_req = &p_event_data->cis_request;
            int snk_ase_id = set_conn_on_ase_id(p_cis_req, ASCS_SINK_ASE_CHARACTERISTIC);

            WICED_BT_TRACE("cis_id %d cig_id %d cis_conn_handle %d acl_handle %d \n",
                           p_cis_req->cis_id,
                           p_cis_req->cig_id,
                           p_cis_req->cis_conn_handle,
                           p_cis_req->acl_handle);

            wiced_bt_isoc_peripheral_accept_cis(p_cis_req->cig_id,
                                                p_cis_req->cis_id,
                                                p_cis_req->cis_conn_handle,
                                                0xFF,
                                                snk_ase_id);
        }
        break;

        case WICED_BLE_ISOC_CIS_ESTABLISHED:
            if (p_event_data->cis_established_data.status)
            {
                WICED_BT_TRACE_CRIT("[%s] status %d \n", __FUNCTION__, p_event_data->cis_established_data.status);
                return;
            }

            // Stup data path after CIS establishment for sink role as server/client,
            // Data path is setup upon receiving streaming notification / receiver start ready
            // as client and server
            unicast_sink_cis_handle_connection(p_event_data->cis_established_data.cig_id,
                                               p_event_data->cis_established_data.cis_id,
                                               ASCS_SINK_ASE_CHARACTERISTIC);
            break;

        case WICED_BLE_ISOC_CIS_DISCONNECTED:
            // in case of bi-directional CIS handle for both the ASE's attached to the CIS
            unicast_sink_cis_handle_disconnection(p_event_data->cis_disconnect.cig_id,
                                                  p_event_data->cis_disconnect.cis_id,
                                                  ASCS_SINK_ASE_CHARACTERISTIC);
            break;

        case WICED_BLE_ISOC_DATA_PATH_SETUP:
            if (p_event_data->datapath.status)
            {
                WICED_BT_TRACE_CRIT("[%s] Data path setup not successful\n", __FUNCTION__);
                return;
            }

            if (wiced_bt_isoc_is_cis_connected_by_conn_id(p_event_data->datapath.conn_hdl))
            {
                unicast_sink_cis_handle_data_path_setup(p_event_data->datapath.conn_hdl,
                                                        p_event_data->datapath.data_path_dir);
            }
            break;

        case WICED_BLE_ISOC_DATA_PATH_REMOVED:
            if (p_event_data->datapath.status)
            {
                WICED_BT_TRACE_CRIT("[%s] Data path removal not successful\n", __FUNCTION__);
            }
            break;

        default:
            break;
    }
}
