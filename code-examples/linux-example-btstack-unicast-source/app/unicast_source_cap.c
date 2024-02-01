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

#include "unicast_source_cap.h"
#include "unicast_source_ascs.h"
#include "unicast_source_gatt.h"

#include "wiced_bt_ga_cap.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define DEFAULT_CIS_COUNT   1
#define CIG_ID  1

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
wiced_bt_ga_ascs_config_codec_args_t cap_codec_config = {
    .codec_id =
        {
            .coding_format = LC3_CODEC_ID,
            .company_id = 0,
            .vendor_specific_codec_id = 0,
        },
    .target_latency = 1,
    .target_phy = WICED_BT_ASCS_PHY_2M,
};

wiced_bt_ga_ascs_config_qos_args_t cap_qos_config = {.cig_id = 1,
                                                     .cis_id = 1,
                                                     .phy = WICED_BLE_ISOC_LE_2M_PHY,
                                                     .max_sdu = 240, //MAX supported config for xavier + H2
                                                     .presentation_delay = 0x9c40};
wiced_bt_ga_cap_start_unicast_param_t unicast_param;

/******************************************************************************
 * Function Name: unicast_source_get_config
 *
 * Summary:
 *  when start stream, use this funciton to get the codec_config include qos config
 *
 * Parameters:
 *  unicast_source_stream_config_t *p_unicast_stream_config
 *      :stream configs setting
 *
 *  wiced_bt_ga_cap_start_unicast_param_t *param:
 *      cap config
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_get_config(unicast_source_stream_config_t *p_unicast_stream_config,
                               wiced_bt_ga_cap_start_unicast_param_t *param)
{
    static uint8_t cap_ccid_list[5] = {1, 2, 3, 4, 5};
    static uint8_t cap_temp_ccid_buffer[7] = {0};
    uint8_t csc_count = 0;

    param->dir = WICED_BT_CAP_DIRECTION_SOURCE;
    param->context_type = BAP_CONTEXT_TYPE_MEDIA;

    cap_qos_config.framing = p_unicast_stream_config->stream_config->framing;
    cap_qos_config.retransmission_number = p_unicast_stream_config->stream_config->retransmission_number;
    cap_qos_config.max_transport_latency = p_unicast_stream_config->stream_config->max_transport_latency;
    cap_qos_config.sdu_interval = p_unicast_stream_config->stream_config->sdu_interval;
    cap_qos_config.max_sdu = p_unicast_stream_config->stream_config->octets_per_codec_frame * 2;
    param->p_codec_configuration = &cap_codec_config;
    param->p_qos_configuration = &cap_qos_config;

    param->p_codec_configuration->csc.sampling_frequency = p_unicast_stream_config->stream_config->sampling_frequency;
    param->p_codec_configuration->csc.frame_duration = p_unicast_stream_config->stream_config->frame_duration;
    param->p_codec_configuration->csc.audio_channel_allocation = p_unicast_stream_config->stream_config->audio_channels;
    param->p_codec_configuration->csc.octets_per_codec_frame = p_unicast_stream_config->stream_config->octets_per_codec_frame;
    param->p_codec_configuration->csc.lc3_blocks_per_sdu = 1;

}


/******************************************************************************
 * Function Name: unicast_source_cap_start_streaming
 *
 * Summary: CAP level start stream API
 *
 * Parameters:
 *  unicast_source_stream_config_t *p_unicast_stream_config
 *
 * Return:
 *  None
******************************************************************************/
void unicast_source_cap_start_streaming(unicast_source_stream_config_t *p_unicast_stream_config)
{
    int index;

    WICED_BT_TRACE("[%s]", __FUNCTION__);

    unicast_source_get_config(p_unicast_stream_config, &unicast_param);

    g_unicast_source_gatt_cb.cap_profile_data.is_bonded = 1; // ykmh: revisit
    g_unicast_source_gatt_cb.cap_profile_data.num_devices = (uint8_t)p_unicast_stream_config->num_devices;

    for (index = 0; index < p_unicast_stream_config->num_devices; index++)
    {
        int ase_count = 0;
        unicast_source_clcb_t *p_clcb =
            unicast_source_gatt_get_clcb_by_conn_id(p_unicast_stream_config->config_list[index].conn_id);

        if (p_clcb == NULL) return;

        g_unicast_source_gatt_cb.cap_profile_data.device_info_list[index].num_ase = 1;

        unicast_source_ase_data_t *p_ase =
            unicast_source_get_first_remote_ase(p_clcb, ASCS_SINK_ASE_CHARACTERISTIC);
        g_unicast_source_gatt_cb.cap_profile_data.device_info_list[index].ascs_data[ase_count] = &p_ase->data;
        p_ase->data.qos_configured.cis_id = index + 1;
    }

    wiced_bt_cap_utils_create_cig(unicast_param.p_qos_configuration, CIG_ID, DEFAULT_CIS_COUNT);
    wiced_bt_ga_cap_start_unicast_streaming(&g_unicast_source_gatt_cb.cap_profile_data, &unicast_param);
}


/******************************************************************************
 * Function Name: unicast_source_cap_stop_streaming
 *
 * Summary:
 *  CAP Level stop streaming API
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_cap_stop_streaming(uint16_t conn_id)
{
    unicast_source_clcb_t *p_clcb = unicast_source_gatt_get_clcb_by_conn_id(conn_id);

    if (p_clcb == NULL) return;

    wiced_bt_ga_cap_release_stream(&g_unicast_source_gatt_cb.cap_profile_data);
}


/******************************************************************************
 * Function Name: unicast_source_cap_event_cb
 *
 * Summary: CAP Level State event callback for wiced_bt_ga_cap_register_cb use
 *
 * Parameters:
 *  wiced_bt_ga_cap_event_t event   
 *  
 *  wiced_bt_ga_cap_event_data_t *p_event_data
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_cap_event_cb(wiced_bt_ga_cap_event_t event, wiced_bt_ga_cap_event_data_t *p_event_data)
{
    switch (event)
    {
    case WICED_BT_GA_CAP_STATE_CHANGED_EVENT: {
        WICED_BT_TRACE("[%s] state update %d", __FUNCTION__, p_event_data->group_state.ase_state);
        if ((p_event_data->group_state.ase_state == WICED_BT_GA_ASCS_STATE_IDLE) ||
            (p_event_data->group_state.ase_state == WICED_BT_GA_ASCS_STATE_STREAMING))
        {
            extern void unicast_source_mcs_handle_post_operation(void);
            unicast_source_mcs_handle_post_operation();
        }

        if (p_event_data->group_state.ase_state == WICED_BT_GA_ASCS_OPCODE_CONFIG_QOS)
        {
            wiced_bt_ga_ascs_ase_t *ase_list[1];
            ase_list[0] = &p_event_data->group_state;
            wiced_bt_cap_utils_create_cis(p_event_data->group_state.qos_configured.gatt_conn_id, ase_list, 1);
        }
    }
    break;

    default:
        break;
    }
}
