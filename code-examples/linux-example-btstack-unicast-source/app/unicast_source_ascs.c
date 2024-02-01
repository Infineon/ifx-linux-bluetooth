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

#include "unicast_source_gatt.h"
#include "unicast_source_iso.h"

#include "wiced_bt_ga_common.h"

#include "wiced_memory.h"

#include "log.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define DEFAULT_CIG_ID 0xFF
#define DEFAULT_CIS_ID 0xFF

/******************************************************************************
 *                              EXTERNS VARIABLE
 *****************************************************************************/
extern wiced_bt_ga_cap_start_unicast_param_t unicast_param;

// create a pool to store app ASE info
/******************************************************************************
 * Function Name: unicast_source_ascs_alloc_memory
 *
 * Summary: alloc memory for ascs 
 *
 * Parameters:
 *  ga_cfg_t *p_cfg
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_ascs_alloc_memory(ga_cfg_t *p_cfg)
{
    if (p_cfg == NULL)
    {
        return;
    }
    for (int index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        unicast_source_clcb_t *p_clcb = &g_unicast_source_gatt_cb.unicast_clcb[index];

        /** size of remote ases is the configuration + space for ase data */
        int num_remote_ases = p_cfg->ascs_max_sink_ase_supported + p_cfg->ascs_max_source_ase_supported;
        int size_remote_ases =
            num_remote_ases * (sizeof(unicast_source_ase_data_t) + sizeof(wiced_bt_ga_ascs_ase_info_t));

        p_clcb->p_remote_ase_data = wiced_memory_alloc_long_term_mem_block(size_remote_ases, "ase_src");
        if (num_remote_ases && !p_clcb->p_remote_ase_data)
        {
            WICED_BT_TRACE("[%s] %d remote 0x%x %d", __FUNCTION__, index, p_clcb->p_remote_ase_data, size_remote_ases);
            return;
        }

        p_clcb->num_remote_ases = num_remote_ases;
    }
}


/******************************************************************************
 * Function Name: unicast_source_init_remote_ases
 *
 * Summary: init remote ases when gatt discovery complete
 *
 * Parameters:
 *  unicast_source_clcb_t *p_clcb
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_init_remote_ases(unicast_source_clcb_t *p_clcb)
{
    if (p_clcb == NULL) 
    {
        return;
    }
    unicast_source_ase_data_t *p_ase = p_clcb->p_remote_ase_data;
    const wiced_bt_ga_ascs_ase_info_t *p_ase_info = NULL;
    int num_ases = p_clcb->num_remote_ases;

    memset(p_ase, 0, sizeof(unicast_source_ase_data_t) * num_ases);
    p_ase_info = (const wiced_bt_ga_ascs_ase_info_t *)(p_ase + num_ases);

    for (int index = 0; index < num_ases; index++, p_ase++, p_ase_info++)
    {
        p_ase->data.p_ase_info = p_ase_info;
    }
}

/******************************************************************************
 * Function Name: unicast_source_assign_remote_ase_id
 *
 * Summary: find the first free slot and assign ASE ID
 *
 * Parameters:
 *  unicast_source_clcb_t *p_clcb
 *  uint8_t ase_id
 *  uint8_t char_type
 *  uint8_t char_instance
 *
 * Return:
 *  unicast_source_ase_data_t*
 *
******************************************************************************/
unicast_source_ase_data_t *unicast_source_assign_remote_ase_id(unicast_source_clcb_t *p_clcb,
                                                               uint8_t ase_id,
                                                               uint8_t char_type,
                                                               uint8_t char_instance)
{
    int num_ase = p_clcb->num_remote_ases;
    wiced_bt_ga_ascs_ase_t *p_ase = NULL;
    wiced_bt_ga_ascs_ase_info_t *p_ase_info = NULL;
    int index = 0;

    while (index < num_ase) {
        p_ase = &p_clcb->p_remote_ase_data[index].data;
        p_ase_info = (wiced_bt_ga_ascs_ase_info_t *)p_ase->p_ase_info;

        WICED_BT_TRACE("[%s] index %d ase_id %d type %d inst %d slot ase_id %d slot ase_state %d",
                       __FUNCTION__,
                       index,
                       ase_id,
                       char_type,
                       char_instance,
                       p_ase->p_ase_info->ase_id,
                       p_ase->ase_state);

        if (p_ase->p_ase_info->ase_id == 0) {
            p_ase_info->ase_id = ase_id;
            p_ase_info->ase_type = char_type;
            return &p_clcb->p_remote_ase_data[index];
        }
        index++;
    }

    return NULL;
}


/******************************************************************************
 * Function Name: unicast_source_get_first_remote_ase
 *
 * Summary: get the pointer of ase by type
 *
 * Parameters:
 *  unicast_source_clcb_t *p_clcb
 *  ascs_characteristics_t type
 *
 * Return:
 *  unicast_source_ase_data_t*
 *
******************************************************************************/
unicast_source_ase_data_t *unicast_source_get_first_remote_ase(unicast_source_clcb_t *p_clcb, ascs_characteristics_t type)
{
    unicast_source_ase_data_t *p_ase = p_clcb->p_remote_ase_data;
    int num_ase = p_clcb->num_remote_ases;

    while (num_ase--)
    {
        if (p_ase->data.p_ase_info->ase_type == type)
        {
            return p_ase;
        }
        p_ase++;
    }
    return NULL;
}


/******************************************************************************
 * Function Name: unicast_source_get_remote_ase_data_by_ase_id
 *
 * Summary: get the saved ase data by ase id
 *
 * Parameters:
 *  unicast_source_clcb_t *p_clcb
 *  uint8_t ase_id
 *
 * Return:
 *  unicast_source_ase_data_t*
 *
******************************************************************************/
unicast_source_ase_data_t *unicast_source_get_remote_ase_data_by_ase_id(unicast_source_clcb_t *p_clcb, uint8_t ase_id)
{
    unicast_source_ase_data_t *p_ase = p_clcb->p_remote_ase_data;
    int num_ase = p_clcb->num_remote_ases;

    while (num_ase--) {
        if (p_ase->data.p_ase_info->ase_id == ase_id) {
            return p_ase;
        }
        p_ase++;
    }

    return NULL;
}


/******************************************************************************
 * Function Name: ascs_init_characteristic
 *
 * Summary: init ascs data
 *
 * Parameters:
 *  gatt_intf_attribute_t *p_char,
 *  unicast_source_ase_data_t *p_ase
 *
 * Return:
 *  gatt_intf_attribute_t*
 *
******************************************************************************/
gatt_intf_attribute_t *ascs_init_characteristic(gatt_intf_attribute_t *p_char,
                                                     unicast_source_ase_data_t *p_ase)
{
    memset(p_char, 0, sizeof(gatt_intf_attribute_t));

    p_char->characteristic_type = p_ase->data.p_ase_info->ase_type;
    p_char->characteristic_instance = 1;

    return p_char;
}


/******************************************************************************
 * Function Name: unicast_source_handle_ase_notification 
 *
 * Summary: handle ASE notification event
 *
 * Parameters:
 *  uint16_t conn_id
 *  unicast_source_ase_data_t *p_ase_data
 *
 * Return:
 *  wiced_bool_t
 *
******************************************************************************/
wiced_bool_t unicast_source_handle_ase_notification(uint16_t conn_id, unicast_source_ase_data_t *p_ase_data)
{
    wiced_result_t data_path_setup_sts = WICED_ERROR;
    gatt_intf_service_object_t *p_service =
        gatt_interface_get_service_by_uuid_and_conn_id(conn_id, &ga_service_uuid_ascs);

    uint16_t cis_conn_handle = wiced_bt_isoc_get_cis_conn_handle(p_ase_data->data.qos_configured.cig_id,
                                                                 p_ase_data->data.qos_configured.cis_id);

    switch (p_ase_data->data.ase_state) {
        case WICED_BT_GA_ASCS_STATE_ENABLING:
            //if client is sink setup datapath and send rcr start ready
            if (ASCS_SOURCE_ASE_CHARACTERISTIC == p_ase_data->data.p_ase_info->ase_type) {
                if (!p_ase_data->data_path_established) {
                    data_path_setup_sts = iso_audio_setup_data_path(cis_conn_handle,
                                                                    p_ase_data->data.p_ase_info->ase_type,
                                                                    &p_ase_data->data.codec_configured.csc);
                    if (data_path_setup_sts) {
                        WICED_BT_TRACE_CRIT("[%s] data path setup unsuccessful..(err:%d)\n",
                                            __FUNCTION__,
                                            data_path_setup_sts);
                        return FALSE;
                    }
                }
                wiced_bt_ga_ascs_send_receiver_start_stop_ready(conn_id,
                                                                p_service,
                                                                p_ase_data->data.p_ase_info->ase_id,
                                                                TRUE);
            }
            break;

        case WICED_BT_GA_ASCS_STATE_STREAMING:
            if (!p_ase_data->data_path_established) {
                data_path_setup_sts = iso_audio_setup_data_path(cis_conn_handle,
                                                                p_ase_data->data.p_ase_info->ase_type,
                                                                &p_ase_data->data.codec_configured.csc);

                if (data_path_setup_sts) {
                    WICED_BT_TRACE_CRIT("[%s] data path setup unsuccessful..(err:%d)\n",
                                        __FUNCTION__,
                                        data_path_setup_sts);
                    return FALSE;
                }
            }
            else if (WICED_BLE_ISOC_DPD_INPUT_BIT == p_ase_data->data.p_ase_info->ase_type) // revisit: tbd
            {
                iso_audio_start_stream(cis_conn_handle);
            }
            break;

        case WICED_BT_GA_ASCS_STATE_DISABLING:
            // TODO: prepare for not receiving audio (ALSA config if any)
            // (source will not send data, datapath and CIS connection will be active..)
            wiced_bt_ga_ascs_send_receiver_start_stop_ready(conn_id,
                                                            p_service,
                                                            p_ase_data->data.p_ase_info->ase_id,
                                                            FALSE);
            break;

        case WICED_BT_GA_ASCS_STATE_RELEASING:
            // TODO: disconnect CIS after data path removal is successful
            iso_audio_remove_data_path(wiced_bt_isoc_get_cis_conn_handle(p_ase_data->data.qos_configured.cig_id,
                                                                         p_ase_data->data.qos_configured.cis_id),
                                       p_ase_data->data.p_ase_info->ase_type, // revisit: tbd
                                       &p_ase_data->lc3_index);
            break;

        default:
            break;
    }
    return TRUE;
}


/******************************************************************************
 * Function Name: unicast_source_set_default_ase_params
 *
 * Summary: set ASE to default when Acceptor ASE state change to IDLE
 *
 * Parameters:
 *  unicast_source_ase_data_t *p_ase_data
 *  wiced_bool_t is_client
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_set_default_ase_params(unicast_source_ase_data_t *p_ase_data, wiced_bool_t is_client)
{
    wiced_bt_ga_ascs_config_codec_args_t *p_codec_configured_params = NULL;

    p_codec_configured_params = &p_ase_data->data.codec_configured;
    if (!p_ase_data) {
        return;
    }

    p_ase_data->data.ase_state = WICED_BT_GA_ASCS_STATE_IDLE;

    p_ase_data->data.qos_configured.cig_id = DEFAULT_CIG_ID;
    p_ase_data->data.qos_configured.cis_id = DEFAULT_CIS_ID;

    p_ase_data->lc3_index = 0;

    WICED_BT_TRACE("[%s] ase_id %d, ase_state 0x%x type %s cis_handle %d",
                   __FUNCTION__,
                   p_ase_data->data.p_ase_info->ase_id,
                   p_ase_data->data.ase_state,
                   (p_ase_data->data.p_ase_info->ase_type == ASCS_SOURCE_ASE_CHARACTERISTIC) ? "src" : "sink",
                   p_ase_data->cis_conn_hdl);
}

/******************************************************************************
 * Function Name: wiced_bt_cap_ascs_handle_ase_read_notification
 *
 * Summary: handle notification of ASE from Sink 
 *
 * Parameters:
 *  uint16_t conn_id
 *  uint32_t state
 *  const gatt_intf_service_object_t *p_service
 *  gatt_intf_attribute_t *p_char
 *  wiced_bt_ga_ascs_ase_t *p_notif_data
 *
 * Return:
 *  None
 *
******************************************************************************/
void wiced_bt_cap_ascs_handle_ase_read_notification(uint16_t conn_id,
                                                    uint32_t state,
                                                    const gatt_intf_service_object_t *p_service,
                                                    gatt_intf_attribute_t *p_char,
                                                    wiced_bt_ga_ascs_ase_t *p_notif_data)
{
    uint8_t *p_buf = NULL;
    unicast_source_ase_data_t *p_ase_data = NULL;

    WICED_BT_TRACE("[%s] state %d", __FUNCTION__, state);

    p_ase_data = unicast_source_get_remote_ase_data_by_ase_id(unicast_source_gatt_get_clcb_by_conn_id(conn_id),
                                                              p_notif_data->p_ase_info->ase_id);
    if (!p_ase_data) {
        return;
    }
    p_ase_data->data.ase_state = state;

    switch (state) {
        case WICED_BT_GA_ASCS_STATE_CODEC_CONFIGURED:
            memcpy(&p_ase_data->data.codec_configured,
                   &p_notif_data->codec_configured,
                   sizeof(wiced_bt_ga_ascs_config_codec_args_t));
            break;

        case WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED:
            memcpy(&p_ase_data->data.qos_configured,
                   &p_notif_data->qos_configured,
                   sizeof(wiced_bt_ga_ascs_config_qos_args_t));
            break;

        case WICED_BT_GA_ASCS_STATE_IDLE: {
            unicast_source_set_default_ase_params(p_ase_data, TRUE); // revisit: tbd
        } break;

        case WICED_BT_GA_ASCS_STATE_ENABLING:
        case WICED_BT_GA_ASCS_STATE_DISABLING:
        case WICED_BT_GA_ASCS_STATE_RELEASING:
        case WICED_BT_GA_ASCS_STATE_STREAMING:
            unicast_source_handle_ase_notification(conn_id, p_ase_data);
            break;

        default:
            break;
    }
}

/******************************************************************************
 * Function Name: unicast_source_ascs_callback
 *
 * Summary: ASCS callback function
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
wiced_result_t unicast_source_ascs_callback(uint16_t conn_id,
                                            void *p_app_ctx,
                                            const gatt_intf_service_object_t *p_service,
                                            wiced_bt_gatt_status_t status,
                                            uint32_t evt_type,
                                            gatt_intf_attribute_t *p_char,
                                            void *p_data,
                                            int len)
{
    unicast_source_clcb_t *p_clcb = unicast_source_gatt_get_clcb_by_conn_id(conn_id);

    switch (evt_type) {
        case WRITE_CMPL_EVT:
            WICED_BT_TRACE("[%s] WRITE_CMPL_EVT characteristic %d\n", __FUNCTION__, p_char);
            break;

        case NOTIFICATION_EVT:
        case READ_CMPL_EVT:
            WICED_BT_TRACE("[%s] NOTIFICATION_EVT/READ_CMPL_EVT | characteristic type %d instance %d\n",
                           __FUNCTION__,
                           p_char->characteristic_type,
                           p_char->characteristic_instance);

            if (p_char->characteristic_type == ASCS_SINK_ASE_CHARACTERISTIC ||
                p_char->characteristic_type == ASCS_SOURCE_ASE_CHARACTERISTIC)
            {
                wiced_bt_ga_ascs_ase_t *p_received_data = (wiced_bt_ga_ascs_ase_t *)p_data;
                int ase_id = p_received_data->p_ase_info->ase_id;

                /* get ASE data of the peer if available */
                unicast_source_ase_data_t *p_ase_data = unicast_source_get_remote_ase_data_by_ase_id(p_clcb, ase_id);
                if (NULL == p_ase_data) {

                    /* If no record found, create an entry now */
                    p_ase_data = unicast_source_assign_remote_ase_id(p_clcb,
                                                                     ase_id,
                                                                     p_char->characteristic_type,
                                                                     p_char->characteristic_instance);
                    if (NULL == p_ase_data) return WICED_ERROR;
                }

                p_ase_data->data.ase_state = p_received_data->ase_state;
                WICED_BT_TRACE("[%s] [%d => %s]\n", __FUNCTION__, ase_id, ascs_state_str[p_received_data->ase_state]);

                if (evt_type == NOTIFICATION_EVT) {
                    wiced_ga_cap_ascs_update_event(&g_unicast_source_gatt_cb.cap_profile_data,
                                                   &unicast_param, conn_id,
                                                   status,
                                                   evt_type,
                                                   p_char,
                                                   p_data,
                                                   len);
                    //this api will running at wiced_bt_cap_ascs_handle_ase_read_notification
                    //should get the buffer only when WICED_BT_GA_ASCS_STATE_STREAMING
                    //review more here, if occur another way play but, not start play audio.
                    //unicast_source_handle_ase_notification(conn_id, p_ase_data);
                }

                wiced_bt_cap_ascs_handle_ase_read_notification(conn_id,
                                                               p_received_data->ase_state,
                                                               p_service,
                                                               p_char,
                                                               p_received_data);
            }
            else if (p_char->characteristic_type == ASCS_ASE_CONTROL_POINT_CHARACTERISTIC) {
                wiced_bt_ga_ascs_cp_notif_t *p_cp_notif_data = (wiced_bt_ga_ascs_cp_notif_t *)p_data;

                WICED_BT_TRACE("[%s] opcode %d response_code : %d reason %d",
                               __FUNCTION__,
                               p_cp_notif_data->opcode,
                               p_cp_notif_data->p_status->response_code,
                               p_cp_notif_data->p_status->reason);
            }
            break;

        default:
            WICED_BT_TRACE("[%s] Unknown event characteristic %d\n", __FUNCTION__, p_char);
            break;
    }

    return WICED_SUCCESS;
}
