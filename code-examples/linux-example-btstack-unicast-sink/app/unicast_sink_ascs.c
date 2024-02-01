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

#include "wiced_bt_ga_ascs.h"
#include "wiced_bt_ga_common.h"

#include "wiced_bt_isoc.h"
#include "wiced_memory.h"

#include "unicast_sink_gatt.h"
#include "unicast_sink_isoc.h"
#include "data_types.h"
#include "log.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define MAX_CODEC_TRANSPORT_LATENCY 0x0D
#define MAX_PRESENTATION_DELAY  40000 //0x9c40
#define NEGATIVE    -1

/******************************************************************************
 *                              EXTERNS
 *****************************************************************************/
extern wiced_bt_cfg_isoc_t unicast_sink_isoc_cfg;

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
BOOL32 isStreaming = FALSE;

const wiced_bt_ga_ascs_ase_info_t unicast_sink_ases[] = {
    {
        .ase_id = 1,
        .ase_type = ASCS_SINK_ASE_CHARACTERISTIC,
        .ascs_data =
            {
                .framing = WICED_BT_ASCS_UNFRAMED,
                .preferred_phy = WICED_BT_ASCS_PHY_2M,
                .preferred_retransmission_number = 2,
                .max_transport_latency = 0x64,
                .presentation_delay_in_us_min = 20000, //0x4e20
                .presentation_delay_in_us_max = 40000, //0x9c40
                .presentation_delay_in_us_min = 40000,     // if set, set a value >= presentation_delay_in_us_min
                .presentation_delay_in_us_max = 40000,     // if set, set a value <= presentation_delay_in_us_max
            },
    },
};

/******************************************************************************
 * Function Name: unicast_sink_ascs_alloc_memory
 ******************************************************************************
 * Summary: allocate memory for ascs
 *
 * Parameters:
 *  ga_cfg_t *p_cfg
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_ascs_alloc_memory(ga_cfg_t *p_cfg)
{
    for (int index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        unicast_sink_clcb_t *p_clcb = &g_unicast_sink_gatt_cb.unicast_clcb[index];
        int num_local_ases = sizeof(unicast_sink_ases) / sizeof(unicast_sink_ases[0]);
        /* size of local ases is just the configuration */
        int size_local_ases = num_local_ases * sizeof(unicast_sink_ase_data_t);
        unicast_sink_ase_data_t *p_ase = NULL;
        int idx = 0;

        p_clcb->p_local_ase_data = wiced_memory_alloc_long_term_mem_block(size_local_ases, "ase_snk");
        if ((num_local_ases && !p_clcb->p_local_ase_data))
        {
            WICED_BT_TRACE_CRIT("[%s] %d. local 0x%x %d",
                                __FUNCTION__,
                                index,
                                p_clcb->p_local_ase_data,
                                size_local_ases);
            return;
        }

        p_clcb->num_local_ases = num_local_ases;

        /** init local ases with the app information */
        {
            for (idx = 0, p_ase = p_clcb->p_local_ase_data; idx < num_local_ases; idx++, p_ase++)
            {
                p_ase->data.p_ase_info = unicast_sink_ases + idx;
            }
        }
    }
}

/******************************************************************************
 * Function Name: unicast_sink_get_ase_instance_internal
 ******************************************************************************
 * Summary: get ase instance
 *
 * Parameters:
 *  unicast_sink_ase_data_t *p_ases
 *  int num_ases
 *  int type
 *  int instance
 *
 * Return:
 *  unicast_sink_ase_data_t *
 *
******************************************************************************/
static unicast_sink_ase_data_t *unicast_sink_get_ase_instance_internal(unicast_sink_ase_data_t *p_ases,
                                                                       int num_ases,
                                                                       int type,
                                                                       int instance)
{
    int num_of_instance_type = 0;
    if (p_ases == NULL)
    {
        TRACE_ERR("p_ases is NULL");
        return NULL;
    }

    for (int i = 0; i < num_ases; i++, p_ases++)
    {
        if (p_ases->data.p_ase_info->ase_type == type)
        {
            // TODO: Access using index
            if (instance == num_of_instance_type)
            {
                return p_ases;
            }
            num_of_instance_type++;
        }
    }

    return NULL;
}

/******************************************************************************
 * Function Name: unicast_sink_get_local_ase_instance_ptr
 ******************************************************************************
 * Summary: get ase instance pointer by conn_id
 *
 * Parameters:
 *  uint16_t conn_id
 *  int type
 *  int instance
 *
 * Return:
 *  unicast_sink_ase_data_t *
 *
******************************************************************************/
unicast_sink_ase_data_t *unicast_sink_get_local_ase_instance_ptr(uint16_t conn_id, int type, int instance)
{
    unicast_sink_clcb_t *p_clcb = unicast_sink_gatt_get_clcb_by_conn_id(conn_id);

    if (!p_clcb) return NULL;

    return unicast_sink_get_ase_instance_internal(p_clcb->p_local_ase_data, p_clcb->num_local_ases, type, instance);
}

/******************************************************************************
 * Function Name: unicast_sink_get_local_ase_app_data_ptr_by_ase_id
 ******************************************************************************
 * Summary: get ase data by ase_id
 *
 * Parameters:
 *  unicast_sink_clcb_t *p_clcb
 *  uint8_t ase_id
 *
 * Return:
 *  unicast_sink_ase_data_t *
 *
******************************************************************************/
unicast_sink_ase_data_t *unicast_sink_get_local_ase_app_data_ptr_by_ase_id(unicast_sink_clcb_t *p_clcb, uint8_t ase_id)
{
    unicast_sink_ase_data_t *p_ase = p_clcb->p_local_ase_data;
    int num_ase = p_clcb->num_local_ases;

    while (num_ase--)
    {
        if (p_ase->data.p_ase_info->ase_id == ase_id) return p_ase;

        p_ase++;
    }

    return NULL;
}

/******************************************************************************
 * Function Name: unicast_sink_ascs_validate_cp_parameters 
 ******************************************************************************
 * Summary: validate cp paramters for opcode
 *
 * Parameters:
 *  wiced_bt_ga_ascs_opcode_t opcode
 *  wiced_bt_ga_ascs_cp_params_t *p_cp_params
 *  wiced_bt_ga_ascs_cp_cmd_sts_t *p_sts
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_sink_ascs_validate_cp_parameters(wiced_bt_ga_ascs_opcode_t opcode,
                                                        wiced_bt_ga_ascs_cp_params_t *p_cp_params,
                                                        wiced_bt_ga_ascs_cp_cmd_sts_t *p_sts)
{
    switch (opcode)
    {
        case WICED_BT_GA_ASCS_OPCODE_CONFIG_QOS:

            // validate retransmission_number
            if (p_cp_params->config_qos_params.retransmission_number > MAX_CODEC_TRANSPORT_LATENCY)
             {
                 p_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE;
                 p_sts->reason = WICED_BT_GA_ASCS_REASON_RETRANSMISSION_NUMBER;
               break;
             }

            // validate presentation_delay
            if (p_cp_params->config_qos_params.presentation_delay != MAX_PRESENTATION_DELAY)
            {
                p_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE;
                p_sts->reason = WICED_BT_GA_ASCS_REASON_PRESENTATION_DELAY;
                break;
            }
            if (p_cp_params->config_qos_params.max_sdu > unicast_sink_isoc_cfg.max_sdu_size)
            {
                p_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE;
                p_sts->reason = WICED_BT_GA_ASCS_REASON_MAXIMUM_SDU_SIZE;
                break;
            }
            if ((p_cp_params->config_qos_params.max_transport_latency >
                    unicast_sink_ases[0].ascs_data.max_transport_latency) ||
                (p_cp_params->config_qos_params.max_transport_latency < 0x5))
            {
                p_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE;
                p_sts->reason = WICED_BT_GA_ASCS_REASON_MAX_TRANSPORT_LATENCY;
                TRACE_ERR("WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE\n");
                break;
            }
            if (p_cp_params->config_qos_params.framing > WICED_BT_ASCS_FRAMED)
            {
                p_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE;
                p_sts->reason = WICED_BT_GA_ASCS_REASON_FRAMING;
                break;
            }
            break;

        case WICED_BT_GA_ASCS_OPCODE_ENABLE:
        case WICED_BT_GA_ASCS_OPCODE_UPDATE_METADATA:

            // if (!p_evt_data->app_data.common.metadata.data[1] || !p_evt_data->app_data.common.metadata.data[5] || p_evt_data->app_data.common.metadata.data[1] > 2 ||
            //    p_evt_data->app_data.common.metadata.data[5] > 2)
            //{
            //    p_evt_data->result.response_code = WICED_BT_GA_ASCS_RESPONSE_INVALID_METADATA;
            //    p_evt_data->result.reason = WICED_BT_GA_ASCS_REASON_METADATA_TYPE_IN_ERROR;
            //}
            break;

        case WICED_BT_GA_ASCS_OPCODE_DISABLE:

            break;

        default:
            break;
    }

    return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_sink_ascs_update_ase_data
 ******************************************************************************
 * Summary: update ase data
 *
 * Parameters:
 *  wiced_bt_ga_ascs_opcode_t opcode
 *  wiced_bt_ga_ascs_cp_params_t *p_cp_params
 *  unicast_sink_ase_data_t *p_ase_data
 *  uint8_t next_state
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_ascs_update_ase_data(wiced_bt_ga_ascs_opcode_t opcode,
                                       wiced_bt_ga_ascs_cp_params_t *p_cp_params,
                                       unicast_sink_ase_data_t *p_ase_data,
                                       uint8_t next_state)
{
    TRACE_LOG("opcode:%x\n", opcode);
    wiced_result_t data_path_setup_sts = WICED_ERROR;

    p_ase_data->data.ase_state = next_state;

    if (WICED_BT_GA_ASCS_OPCODE_CONFIG_CODEC == opcode)
    {
        memcpy(&p_ase_data->data.codec_configured,
               &p_cp_params->config_codec_params,
               sizeof(wiced_bt_ga_ascs_config_codec_args_t));
    }
    else if (WICED_BT_GA_ASCS_OPCODE_CONFIG_QOS == opcode)
    {
        memcpy(&p_ase_data->data.qos_configured,
               &p_cp_params->config_qos_params,
               sizeof(wiced_bt_ga_ascs_config_qos_args_t));
    }
    else if (WICED_BT_GA_ASCS_OPCODE_ENABLE == opcode || WICED_BT_GA_ASCS_OPCODE_UPDATE_METADATA == opcode)
    {
        WICED_BT_TRACE("pre ctx val %d", p_cp_params->metadata.preferred_audio_ctx);
        WICED_BT_TRACE("str ctx val %d", p_cp_params->metadata.streaming_audio_ctx);
        WICED_BT_TRACE("vsc ctx val %d", p_cp_params->metadata.p_vendor_specific_data);

        memcpy(&p_ase_data->data.metadata, &p_cp_params->metadata, sizeof(wiced_bt_ga_bap_metadata_t));

        if (WICED_BT_GA_ASCS_OPCODE_ENABLE == opcode &&
            wiced_bt_isoc_is_cis_connected(p_ase_data->data.qos_configured.cig_id,
                                           p_ase_data->data.qos_configured.cis_id))
        {
            data_path_setup_sts = iso_audio_setup_data_path(p_ase_data->cis_conn_hdl,
                                                            p_ase_data->data.p_ase_info->ase_type,
                                                            &p_ase_data->data.codec_configured.csc);
            if (data_path_setup_sts)
            {
                WICED_BT_TRACE_CRIT("[%s] data path setup unsuccessful..(err:%d)\n", __FUNCTION__, data_path_setup_sts);
                return;
            }
            p_ase_data->data.ase_state = WICED_BT_GA_ASCS_STATE_STREAMING;
        }
    }
    else if (WICED_BT_GA_ASCS_OPCODE_RECEIVER_START_READY == opcode)
    {
        data_path_setup_sts = iso_audio_setup_data_path(p_ase_data->cis_conn_hdl,
                                                        p_ase_data->data.p_ase_info->ase_type,
                                                        &p_ase_data->data.codec_configured.csc);
        TRACE_LOG("WICED_BT_GA_ASCS_OPCODE_RECEIVER_START_READY\n");
        if (data_path_setup_sts)
        {
            WICED_BT_TRACE_CRIT("[%s] data path setup unsuccessful..(err:%d)\n", __FUNCTION__, data_path_setup_sts);
            return;
        }
    }
    else if (WICED_BT_GA_ASCS_OPCODE_RECEIVER_STOP_READY == opcode)
    {
        // If server is source stop sending audio data
        // The Unicast Server in the Audio Source role should not stop transmitting audio
        // data for a Source ASE in the Disabling state until the Unicast Server transitions
        // the ASE to the QoS Configured state.
        if (WICED_BLE_ISOC_DPD_INPUT_BIT == p_ase_data->data.p_ase_info->ase_type)
        {
            iso_audio_stop_stream(p_ase_data->cis_conn_hdl);
        }
    }
    else if (WICED_BT_GA_ASCS_OPCODE_RELEASE == opcode)
    {
        iso_audio_remove_data_path(p_ase_data->cis_conn_hdl, WICED_BLE_ISOC_DPD_OUTPUT_BIT, &p_ase_data->lc3_index);
        isStreaming = FALSE;
    }
}

/******************************************************************************
 * Function Name: unicast_sink_ascs_handle_read_req_evt
 ******************************************************************************
 * Summary: ascs callback for handle read request
 *
 * Parameters:
 *  uint16_t conn_id
 *  gatt_intf_attribute_t *p_char
 *  wiced_bt_ga_ascs_ase_t *p_evt_data
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
static wiced_result_t unicast_sink_ascs_handle_read_req_evt(uint16_t conn_id,
                                                            gatt_intf_attribute_t *p_char,
                                                            wiced_bt_ga_ascs_ase_t *p_evt_data)
{
    unicast_sink_ase_data_t *p_ase_data;

    WICED_BT_TRACE("[%s] characteristic type %d instance %d",
                   __FUNCTION__,
                   p_char->characteristic_type,
                   p_char->characteristic_instance);

    p_ase_data =
        unicast_sink_get_local_ase_instance_ptr(conn_id, p_char->characteristic_type, p_char->characteristic_instance);

    // validate ASE ID
    CHECK_FOR_NULL_AND_RETURN_VALUE(p_ase_data, WICED_ERROR);

    WICED_BT_TRACE("[%s] ase_id %d", __FUNCTION__, p_ase_data->data.p_ase_info->ase_id);

    if (ASCS_SINK_ASE_CHARACTERISTIC == p_char->characteristic_type)
    {
        memcpy(p_evt_data, &p_ase_data->data, sizeof(wiced_bt_ga_ascs_ase_t));
        WICED_BT_TRACE("[%s] p_evt_data->ase_state %s \n", __FUNCTION__, ascs_state_str[p_evt_data->ase_state]);
    }
    else
    {
        WICED_BT_TRACE("[%s] invalid characteristic_type %d \n", __FUNCTION__, p_char->characteristic_type);
    }

    return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_sink_get_local_ase_instance
 ******************************************************************************
 * Summary: get number of local ase instance by p_ase
 *
 * Parameters:
 *  wiced_bt_ga_ascs_ase_t *p_ase
 *
 * Return:
 *  int
 *
******************************************************************************/
int unicast_sink_get_local_ase_instance(wiced_bt_ga_ascs_ase_t *p_ase)
{
    int type = p_ase->p_ase_info->ase_type;
    int instance = 0;
    const wiced_bt_ga_ascs_ase_info_t *p_info = unicast_sink_ases;
    int limit = sizeof(unicast_sink_ases) / sizeof(unicast_sink_ases[0]);

    while (limit--)
    {
        if (type == p_info->ase_type)
        {
            if (p_ase->p_ase_info == p_info)
            {
                return instance;
            }
            instance++;
        }
        p_info++;
    }

    return NEGATIVE;
}

/******************************************************************************
 * Function Name: ascs_init_characteristic
 ******************************************************************************
 * Summary: init characteristic, callback of gatt_interface_notify_characteristic
 *
 * Parameters:
 *  gatt_intf_attribute_t *p_char
 *  unicast_sink_ase_data_t *p_ase
 *
 * Return:
 *  gatt_intf_attribute_t *
 *
******************************************************************************/
gatt_intf_attribute_t *ascs_init_characteristic(gatt_intf_attribute_t *p_char, unicast_sink_ase_data_t *p_ase)
{
    memset(p_char, 0, sizeof(gatt_intf_attribute_t));

    p_char->characteristic_type = p_ase->data.p_ase_info->ase_type;
    p_char->characteristic_instance = unicast_sink_get_local_ase_instance(&p_ase->data);

    return p_char;
}

/******************************************************************************
 * Function Name: unicast_sink_ascs_notify_ase
 ******************************************************************************
 * Summary: after reveice write request from source, notify to source device
 *
 * Parameters:
 *  uint16_t conn_id
 *  gatt_intf_service_object_t *p_service,
 *  wiced_bt_ga_ascs_cp_notif_t *p_cp_notif_data)
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_ascs_notify_ase(uint16_t conn_id,
                                  gatt_intf_service_object_t *p_service,
                                  wiced_bt_ga_ascs_cp_notif_t *p_cp_notif_data)
{
    gatt_intf_attribute_t ase_char;

    for (int index = 0; index < p_cp_notif_data->num_of_ase; index++)
    {
        wiced_bt_ga_ascs_cp_cmd_sts_t *p_cp_cmd_sts = &p_cp_notif_data->p_status[index];

        if (WICED_BT_GA_ASCS_RESPONSE_SUCCESS != p_cp_cmd_sts->response_code)
        {
            continue;
        }

        unicast_sink_ase_data_t *p_ase_data =
            unicast_sink_get_local_ase_app_data_ptr_by_ase_id(unicast_sink_gatt_get_clcb_by_conn_id(conn_id),
                                                              p_cp_cmd_sts->ase_id);
        CHECK_FOR_NULL_AND_RETURN(p_ase_data);

        gatt_interface_notify_characteristic(conn_id,
                                             p_service,
                                             ascs_init_characteristic(&ase_char, p_ase_data),
                                             (void *)&p_ase_data->data);

        WICED_BT_TRACE("[%s] characteristic type %d inst %d ase_id %d",
                       __FUNCTION__,
                       ase_char.characteristic_type,
                       ase_char.characteristic_instance,
                       p_cp_cmd_sts->ase_id);

        if (p_ase_data->data.ase_state == WICED_BT_GA_ASCS_STATE_RELEASING &&
            !wiced_bt_isoc_is_cis_connected(p_ase_data->data.qos_configured.cig_id,
                                            p_ase_data->data.qos_configured.cis_id))
        {
            p_ase_data->data.ase_state = WICED_BT_GA_ASCS_STATE_IDLE;
            gatt_interface_notify_characteristic(conn_id, p_service, &ase_char, &p_ase_data->data);
        }
    }
}

/******************************************************************************
 * Function Name: unicast_sink_ascs_handle_write_req_evt
 ******************************************************************************
 * Summary: ascs handle write request
 *
 * Parameters:
 *  uint16_t conn_id
 *  const gatt_intf_service_object_t *p_service
 *  gatt_intf_attribute_t *p_char
 *  uint8_t *p_data_stream
 *  int length
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_sink_ascs_handle_write_req_evt(uint16_t conn_id,
                                                      const gatt_intf_service_object_t *p_service,
                                                      gatt_intf_attribute_t *p_char,
                                                      uint8_t *p_data_stream,
                                                      int length)
{
    static wiced_bt_ga_ascs_cp_params_t cp_params = {0};
    static wiced_bt_ga_ascs_cp_notif_t cp_notif_data = {0xFF};
    int bytes_consumed = 0;
    uint8_t index = 0;
    wiced_bt_ga_ascs_opcode_t opcode = 0xFF;
    uint8_t num_of_ase = 0;
    uint8_t next_state = WICED_BT_GA_ASCS_STATE_MAX;

    memset(&cp_params, 0, sizeof(wiced_bt_ga_ascs_cp_params_t));

    p_data_stream += wiced_bt_ga_ascs_get_cp_header(p_data_stream, (uint8_t *)&opcode, &num_of_ase);
    length -= 2;

    WICED_BT_TRACE("[%s] opcode %d num_of_ase %d", __FUNCTION__, opcode, num_of_ase);

    if (!num_of_ase)
    {
        WICED_BT_TRACE_CRIT("[%s] Number of ASE's 0\n", __FUNCTION__);
        return WICED_ERROR;
    }

    cp_notif_data.opcode = opcode;
    cp_notif_data.num_of_ase = num_of_ase;

    cp_notif_data.p_status =
        (wiced_bt_ga_ascs_cp_cmd_sts_t *)wiced_bt_get_buffer(sizeof(wiced_bt_ga_ascs_cp_cmd_sts_t) * num_of_ase);
    CHECK_FOR_NULL_AND_RETURN_VALUE(cp_notif_data.p_status, WICED_ERROR);

    memset(cp_notif_data.p_status, 0, sizeof(wiced_bt_ga_ascs_cp_cmd_sts_t) * num_of_ase);

    if (!opcode || opcode >= WICED_BT_GA_ASCS_OPCODE_MAX)
    {
        cp_notif_data.num_of_ase = 1;
        cp_notif_data.p_status[0].response_code = WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_OPCODE;
        cp_notif_data.p_status[0].reason = WICED_BT_GA_ASCS_REASON_NOT_APPLICABLE;
        gatt_interface_notify_characteristic(conn_id, (gatt_intf_service_object_t *)p_service, p_char, &cp_notif_data);
        wiced_bt_free_buffer(cp_notif_data.p_status);
        return WICED_ERROR;
    }

    //TODO: Validate opcode and length before proceeding instead of checking in the loop below.
    // this way, when we receive a partial multi ASE operation packet we can avoid updating the
    // state of the first ASE before realizing the data for second ASE is partial

    while (length > 0)
    {
        wiced_bt_ga_ascs_cp_cmd_sts_t *p_cp_cmd_sts = &cp_notif_data.p_status[index];
        unicast_sink_ase_data_t *p_ase_data = NULL;

        WICED_BT_TRACE("[%s] bytes remaining %d index %d", __FUNCTION__, length, index);

        bytes_consumed = wiced_bt_ga_ascs_parse_data(opcode, p_data_stream, length, &cp_params, p_cp_cmd_sts);

        WICED_BT_TRACE("[%s] ASE ID %d bytes_consumed %d reason %d response %d",
                       __FUNCTION__,
                       cp_params.ase_id,
                       bytes_consumed,
                       p_cp_cmd_sts->reason,
                       p_cp_cmd_sts->response_code);

        // validate length
        if (-1 == bytes_consumed)
        {
            cp_notif_data.num_of_ase = 1;
            cp_notif_data.p_status[0].response_code = WICED_BT_GA_ASCS_RESPONSE_INVALID_LENGTH;
            cp_notif_data.p_status[0].reason = WICED_BT_GA_ASCS_REASON_NOT_APPLICABLE;
            gatt_interface_notify_characteristic(conn_id,
                                                 (gatt_intf_service_object_t *)p_service,
                                                 p_char,
                                                 &cp_notif_data);
            wiced_bt_free_buffer(cp_notif_data.p_status);
            return WICED_ERROR;
        }

        p_data_stream += bytes_consumed;
        length -= bytes_consumed;

        // skip updating ASE if received data is not supported
        if (p_cp_cmd_sts->response_code != WICED_BT_GA_ASCS_RESPONSE_SUCCESS)
        {
            continue;
        }

        // validate ASE ID
        p_ase_data = unicast_sink_get_local_ase_app_data_ptr_by_ase_id(unicast_sink_gatt_get_clcb_by_conn_id(conn_id),
                                                                       cp_params.ase_id);
        if (NULL == p_ase_data)
        {
            p_cp_cmd_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_INVALID_ASE_ID;
            p_cp_cmd_sts->reason = WICED_BT_GA_ASCS_REASON_NOT_APPLICABLE;
            continue;
        }

        // validate state transition
        if (!wiced_bt_ga_bap_is_state_transition_valid(p_ase_data->data.p_ase_info->ase_type,
                                                       p_ase_data->data.ase_state,
                                                       opcode,
                                                       &next_state))
        {
            p_cp_cmd_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_INVALID_ASE_STATE_MACHINE_TRANSITION;
            p_cp_cmd_sts->reason = WICED_BT_GA_ASCS_REASON_NOT_APPLICABLE;
            continue;
        }

        //validate parameters
        unicast_sink_ascs_validate_cp_parameters(opcode, &cp_params, p_cp_cmd_sts);
        unicast_sink_ascs_update_ase_data(opcode, &cp_params, p_ase_data, next_state);
        index++;
    }

    // send CP notification indicating status of the command
    gatt_interface_notify_characteristic(conn_id, (gatt_intf_service_object_t *)p_service, p_char, &cp_notif_data);

    // send ASE notifications for all ASE's updated by this command
    unicast_sink_ascs_notify_ase(conn_id, (gatt_intf_service_object_t *)p_service, &cp_notif_data);

    wiced_bt_free_buffer(cp_notif_data.p_status);

    return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_sink_ascs_callback 
 ******************************************************************************
 * Summary: ascs callback, use in unicast_sink_store_service_ref_local, as callback of gatt_interface_set_callback_to_profile
 *          handle write or read request
 *
 * Parameters:
 *  uint16_t conn_id
 *  void *p_app_ctx,
 *  const gatt_intf_service_object_t *p_service,
 *  wiced_bt_gatt_status_t status,
 *  uint32_t evt_type,
 *  gatt_intf_attribute_t *p_char,
 *  void *p_data,
 *  int len)
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_sink_ascs_callback(uint16_t conn_id,
                                          void *p_app_ctx,
                                          const gatt_intf_service_object_t *p_service,
                                          wiced_bt_gatt_status_t status,
                                          uint32_t evt_type,
                                          gatt_intf_attribute_t *p_char,
                                          void *p_data,
                                          int len)
{
    wiced_result_t result = WICED_SUCCESS;

    switch (evt_type)
    {
        case WRITE_REQ_EVT:
            result = unicast_sink_ascs_handle_write_req_evt(conn_id, p_service, p_char, (uint8_t *)p_data, len);
            break;
        case READ_REQ_EVT:
            result = unicast_sink_ascs_handle_read_req_evt(conn_id, p_char, (wiced_bt_ga_ascs_ase_t *)p_data);
            break;
        default:
            WICED_BT_TRACE("[%s] event %d \n", __FUNCTION__, p_char);
            break;
    }

    return result;
}

/******************************************************************************
 * Function Name: unicast_sink_set_default_ase_params
 ******************************************************************************
 * Summary: set ase data to default
 *
 * Parameters:
 *  unicast_sink_ase_data_t *p_ase_data
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_set_default_ase_params(unicast_sink_ase_data_t *p_ase_data)
{
    CHECK_FOR_NULL_AND_RETURN(p_ase_data);

    wiced_bt_ga_ascs_config_codec_args_t *p_codec_configured_params = &p_ase_data->data.codec_configured;

    p_ase_data->data.ase_state = WICED_BT_GA_ASCS_STATE_IDLE;

    p_ase_data->data.qos_configured.cig_id = 0xFF;
    p_ase_data->data.qos_configured.cis_id = 0xFF;

    p_ase_data->lc3_index = 0;

    WICED_BT_TRACE("[%s] ase_id %d, ase_state 0x%x type %s cis_handle %d",
                   __FUNCTION__,
                   p_ase_data->data.p_ase_info->ase_id,
                   p_ase_data->data.ase_state,
                   (p_ase_data->data.p_ase_info->ase_type == ASCS_SOURCE_ASE_CHARACTERISTIC) ? "src" : "sink",
                   p_ase_data->cis_conn_hdl);
}

/******************************************************************************
 * Function Name: unicast_sink_ascs_init_data
 ******************************************************************************
 * Summary: init ascs data
 *
 * Parameters:
 *  unicast_sink_clcb_t *p_clcb
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_ascs_init_data(unicast_sink_clcb_t *p_clcb)
{
    int num_ase = p_clcb->num_local_ases;
    unicast_sink_ase_data_t *p_ase = p_clcb->p_local_ase_data;
    WICED_BT_TRACE("[%s] %B", __FUNCTION__, p_clcb->bda);

    for (; num_ase--; p_ase++)
    {
        unicast_sink_set_default_ase_params(p_ase);
    }
}
