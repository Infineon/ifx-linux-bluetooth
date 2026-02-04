/*
 * $ Copyright Cypress Semiconductor $
 */

#include "lehs.h"
#include "le_audio_cap.h"

#define MAX_CODEC_RETRANSMISSION_NUMBER 0x0D

#define MIN_SUPPORTED_PRESENTATION_DELAY 0
#define MAX_SUPPORTED_PRESENTATION_DELAY 40000 //0x9c40

wiced_bt_ga_ascs_ase_info_t lehs_ases[] = {
    {
        .ase_id = 1,
        .ase_type = ASCS_SINK_ASE_CHARACTERISTIC,
        .data_path_dir = WICED_BLE_ISOC_DPD_OUTPUT,
        .ascs_data =
            {
                .framing = 0,  // Unframed ISOAL PDUs supported
                .preferred_phy = WICED_BT_ASCS_PHY_2M,
                .preferred_retransmission_number = 2,
                .max_transport_latency = 0x64,
                .presentation_delay_in_us_min =
                    MIN_SUPPORTED_PRESENTATION_DELAY,  // if set, set a value >= presentation_delay_in_us_min
                .presentation_delay_in_us_max =
                    MAX_SUPPORTED_PRESENTATION_DELAY, // if set, set a value <= presentation_delay_in_us_max
                .preferred_presentation_delay_in_us_min = 0, // No preference
                .preferred_presentation_delay_in_us_max = 0, // No preference
            },
    },
    {
        .ase_id = 2,
        .ase_type = ASCS_SOURCE_ASE_CHARACTERISTIC,
        .data_path_dir = WICED_BLE_ISOC_DPD_INPUT,
        .ascs_data =
            {
                .framing = 0, // Unframed ISOAL PDUs supported
                .preferred_phy = WICED_BT_ASCS_PHY_2M,
                .preferred_retransmission_number = 2,
                .max_transport_latency = 0x64,
                .presentation_delay_in_us_min =
                    MIN_SUPPORTED_PRESENTATION_DELAY, // if set, set a value >= presentation_delay_in_us_min
                .presentation_delay_in_us_max =
                    MAX_SUPPORTED_PRESENTATION_DELAY, // if set, set a value <= presentation_delay_in_us_max
                 .preferred_presentation_delay_in_us_min = 0, // No preference
                 .preferred_presentation_delay_in_us_max = 0, // No preference
            },
    },
    {
        .ase_id = 3,
        .ase_type = ASCS_SINK_ASE_CHARACTERISTIC,
        .data_path_dir = WICED_BLE_ISOC_DPD_OUTPUT,
        .ascs_data =
            {
                .framing = 0, // Unframed ISOAL PDUs supported
                .preferred_phy = WICED_BT_ASCS_PHY_2M,
                .preferred_retransmission_number = 2,
                .max_transport_latency = 0x64,
                .presentation_delay_in_us_min =
                    MIN_SUPPORTED_PRESENTATION_DELAY, // if set, set a value >= presentation_delay_in_us_min
                .presentation_delay_in_us_max =
                    MAX_SUPPORTED_PRESENTATION_DELAY,        // if set, set a value <= presentation_delay_in_us_max
                .preferred_presentation_delay_in_us_min = 0, // No preference
                .preferred_presentation_delay_in_us_max = 0, // No preference
            },
    },
};

void lehs_ascs_alloc_memory()
{
    for (int index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        lehs_clcb_t *p_clcb = &g_lehs_gatt_cb.clcb[index];
        int num_local_ases = sizeof(lehs_ases) / sizeof(lehs_ases[0]);
        /* size of local ases is just the configuration */
        int size_local_ases = num_local_ases * sizeof(lehs_ase_data_t);
        lehs_ase_data_t *p_ase = NULL;
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
                p_ase->data.p_ase_info = lehs_ases + idx;
            }
        }
    }
}

static lehs_ase_data_t *lehs_get_ase_instance_internal(lehs_ase_data_t *p_ases,
                                                                       int num_ases,
                                                                       int type,
                                                                       int instance)
{
    int num_of_instance_type = 0;

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

lehs_ase_data_t *lehs_get_local_ase_instance_ptr(uint16_t conn_id, int type, int instance)
{
    lehs_clcb_t *p_clcb = lehs_gatt_get_clcb_by_conn_id(conn_id);

    if (!p_clcb) return NULL;

    return lehs_get_ase_instance_internal(p_clcb->p_local_ase_data, p_clcb->num_local_ases, type, instance);
}

lehs_ase_data_t *lehs_get_local_ase_app_data_ptr_by_ase_id(lehs_clcb_t *p_clcb, uint8_t ase_id)
{
    lehs_ase_data_t *p_ase = p_clcb->p_local_ase_data;
    int num_ase = p_clcb->num_local_ases;

    while (num_ase--)
    {
        if (p_ase->data.p_ase_info->ase_id == ase_id) return p_ase;

        p_ase++;
    }

    return NULL;
}

wiced_result_t lehs_ascs_validate_cp_parameters(uint16_t conn_id, wiced_bt_ga_ascs_opcode_t opcode,
                                                        wiced_bt_ga_ascs_cp_params_t *p_cp_params,
                                                        wiced_bt_ga_ascs_cp_cmd_sts_t *p_sts)
{
    lehs_clcb_t *p_clcb = lehs_gatt_get_clcb_by_conn_id(conn_id);
    lehs_ase_data_t *p_ase = lehs_get_local_ase_app_data_ptr_by_ase_id(p_clcb, p_cp_params->ase_id);
    switch (opcode)
    {
    case WICED_BT_GA_ASCS_OPCODE_CONFIG_CODEC: {
        wiced_bool_t res = le_audio_cap_verify_codec(
            (p_ase->data.p_ase_info->ase_type == ASCS_SOURCE_ASE_CHARACTERISTIC) ? WICED_BT_CAP_DIRECTION_SOURCE
                                                                                 : WICED_BT_CAP_DIRECTION_SINK,
                                  &p_cp_params->config_codec_params,
                                  g_lehs_gatt_cb.p_pacs_data,
                                  WICED_FALSE);
        if (res == WICED_FALSE)
        {
            p_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_AUDIO_CAPABILITIES;
            p_sts->reason = WICED_BT_GA_ASCS_REASON_NOT_APPLICABLE;
        }
    }
        break;
    case WICED_BT_GA_ASCS_OPCODE_CONFIG_QOS: {
        wiced_bt_ga_ascs_ase_preferences_t *p_ase_pref = &p_ase->data.p_ase_info->ascs_data;
        // validate retransmission_number
        if (p_cp_params->config_qos_params.retransmission_number > MAX_CODEC_RETRANSMISSION_NUMBER)
        {
            p_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE;
            p_sts->reason = WICED_BT_GA_ASCS_REASON_RETRANSMISSION_NUMBER;
            break;
        }

        // validate presentation_delay
        if ((p_ase_pref->preferred_presentation_delay_in_us_max && (p_cp_params->config_qos_params.presentation_delay > p_ase_pref->presentation_delay_in_us_max)) ||
            (p_ase_pref->preferred_presentation_delay_in_us_min &&
                (p_cp_params->config_qos_params.presentation_delay < p_ase_pref->presentation_delay_in_us_min)))
        {
            p_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE;
            p_sts->reason = WICED_BT_GA_ASCS_REASON_PRESENTATION_DELAY;
            break;
        }
        if (p_cp_params->config_qos_params.max_sdu > LEHS_MAX_SDU_SIZE)
        {
            p_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE;
            p_sts->reason = WICED_BT_GA_ASCS_REASON_MAXIMUM_SDU_SIZE;
            break;
        }
        if ((p_cp_params->config_qos_params.max_transport_latency > 0x0FA0) ||
            (p_cp_params->config_qos_params.max_transport_latency < 0x5))
        {
            p_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE;
            p_sts->reason = WICED_BT_GA_ASCS_REASON_MAX_TRANSPORT_LATENCY;
            break;
        }
        if (p_ase_pref->framing && (p_cp_params->config_qos_params.framing == WICED_BT_ASCS_FRAMED))
        {
            p_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE;
            p_sts->reason = WICED_BT_GA_ASCS_REASON_FRAMING;
            break;
        }
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



void lehs_ascs_update_ase_data(uint16_t conn_id, wiced_bt_ga_ascs_opcode_t opcode,
                                       wiced_bt_ga_ascs_cp_params_t *p_cp_params,
                                       lehs_ase_data_t *p_ase,
                                       uint8_t next_state)
{
    wiced_result_t data_path_setup_sts = WICED_ERROR;

    WICED_BT_TRACE("[%s] current %d next %d", __FUNCTION__, p_ase->data.ase_state, next_state);

    p_ase->data.ase_state = next_state;

    if (WICED_BT_GA_ASCS_OPCODE_CONFIG_CODEC == opcode)
    {
        memcpy(&p_ase->data.codec_configured,
               &p_cp_params->config_codec_params,
               sizeof(wiced_bt_ga_ascs_config_codec_args_t));
    }
    else if (WICED_BT_GA_ASCS_OPCODE_CONFIG_QOS == opcode)
    {
        wiced_bt_ga_ascs_config_qos_args_t *p_qos = &p_ase->data.qos_configured;

        memcpy(&p_ase->data.qos_configured,
               &p_cp_params->config_qos_params,
               sizeof(wiced_bt_ga_ascs_config_qos_args_t));
        p_ase->acl_conn_handle = wiced_bt_gatt_get_acl_conn_handle(conn_id);
        p_ase->cis_conn_handle =
            wiced_ble_isoc_get_cis_conn_handle(p_qos->cig_id, p_qos->cis_id, p_ase->acl_conn_handle);

    }
    else if (WICED_BT_GA_ASCS_OPCODE_ENABLE == opcode || WICED_BT_GA_ASCS_OPCODE_UPDATE_METADATA == opcode)
    {
        WICED_BT_TRACE("pre ctx val %d", p_cp_params->metadata.preferred_audio_ctx);
        WICED_BT_TRACE("str ctx val %d", p_cp_params->metadata.streaming_audio_ctx);
        WICED_BT_TRACE("vsc ctx val %d", p_cp_params->metadata.p_vendor_specific_data);

        memcpy(&p_ase->data.metadata, &p_cp_params->metadata, sizeof(wiced_bt_ga_bap_metadata_t));

        if (WICED_BT_GA_ASCS_OPCODE_ENABLE == opcode &&
            wiced_ble_isoc_is_cis_connected_with_conn_hdl(p_ase->cis_conn_handle) &&
            p_ase->data.p_ase_info->ase_type == ASCS_SINK_ASE_CHARACTERISTIC)
        {
            data_path_setup_sts = lehs_isoc_dhm_setup_cis_stream(p_ase);
            if (data_path_setup_sts)
            {
                WICED_BT_TRACE_CRIT("[%s] data path setup unsuccessful..(err:%d)\n", __FUNCTION__, data_path_setup_sts);
                return;
            }
            {
                gatt_intf_attribute_t ase_char = {0};
                gatt_intf_service_object_t *p_service =
                    gatt_interface_get_service_by_uuid_and_conn_id(0, &ga_service_uuid_ascs);

                p_ase->data.ase_state = WICED_BT_GA_ASCS_STATE_ENABLING;
                gatt_interface_notify_characteristic(conn_id,
                                                     p_service,
                                                     ascs_init_characteristic(&ase_char, p_ase),
                                                     &p_ase->data);
            }
            p_ase->data.ase_state = WICED_BT_GA_ASCS_STATE_STREAMING;
        }
    }
    else if (WICED_BT_GA_ASCS_OPCODE_RECEIVER_START_READY == opcode)
    {
        data_path_setup_sts = lehs_isoc_dhm_setup_cis_stream(p_ase);
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
        if (WICED_BLE_ISOC_DPD_INPUT_BIT == p_ase->data.p_ase_info->ase_type)
        {
            lehs_isoc_audio_stop_stream(p_ase->cis_conn_handle);
        }
    }
    else if ((WICED_BT_GA_ASCS_OPCODE_RELEASE == opcode) || (WICED_BT_GA_ASCS_OPCODE_DISABLE == opcode))
    {
        lehs_isoc_dhm_free_cis_stream(p_ase->cis_conn_handle, WICED_BLE_ISOC_DPD_OUTPUT_BIT);
    }
}

static wiced_result_t lehs_ascs_handle_read_req_evt(uint16_t conn_id,
                                                            gatt_intf_attribute_t *p_char,
                                                            wiced_bt_ga_ascs_ase_t *p_evt_data)
{
    lehs_ase_data_t *p_ase;

    WICED_BT_TRACE("[%s] characteristic type %d instance %d",
                   __FUNCTION__,
                   p_char->characteristic_type,
                   p_char->characteristic_instance);

    p_ase =
        lehs_get_local_ase_instance_ptr(conn_id, p_char->characteristic_type, p_char->characteristic_instance);

    // validate ASE ID
    CHECK_FOR_NULL_AND_RETURN_VALUE(p_ase, WICED_ERROR);

    WICED_BT_TRACE("[%s] ase_id %d", __FUNCTION__, p_ase->data.p_ase_info->ase_id);

    {
        memcpy(p_evt_data, &p_ase->data, sizeof(wiced_bt_ga_ascs_ase_t));
        WICED_BT_TRACE("[%s] p_evt_data->ase_state %s \n", __FUNCTION__, ascs_state_str[p_evt_data->ase_state]);
    }

    return WICED_SUCCESS;
}

int lehs_get_local_ase_instance(wiced_bt_ga_ascs_ase_t *p_ase)
{
    int type = p_ase->p_ase_info->ase_type;
    int instance = 0;
    const wiced_bt_ga_ascs_ase_info_t *p_info = lehs_ases;
    int limit = sizeof(lehs_ases) / sizeof(lehs_ases[0]);

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

    return -1;
}

gatt_intf_attribute_t *ascs_init_characteristic(gatt_intf_attribute_t *p_char, lehs_ase_data_t *p_ase)
{
    memset(p_char, 0, sizeof(gatt_intf_attribute_t));

    p_char->characteristic_type = p_ase->data.p_ase_info->ase_type;
    p_char->characteristic_instance = lehs_get_local_ase_instance(&p_ase->data);

    return p_char;
}

void lehs_ascs_notify_ase(uint16_t conn_id,
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

        lehs_ase_data_t *p_ase =
            lehs_get_local_ase_app_data_ptr_by_ase_id(lehs_gatt_get_clcb_by_conn_id(conn_id),
                                                              p_cp_cmd_sts->ase_id);
        CHECK_FOR_NULL_AND_RETURN(p_ase);

        gatt_interface_notify_characteristic(conn_id,
                                             p_service,
                                             ascs_init_characteristic(&ase_char, p_ase),
                                             (void *)&p_ase->data);

        WICED_BT_TRACE("[%s] characteristic type %d inst %d ase_id %d",
                       __FUNCTION__,
                       ase_char.characteristic_type,
                       ase_char.characteristic_instance,
                       p_cp_cmd_sts->ase_id);

        if (p_ase->data.ase_state == WICED_BT_GA_ASCS_STATE_RELEASING &&
            !wiced_ble_isoc_is_cis_connected_with_conn_hdl(p_ase->cis_conn_handle))
        {
            p_ase->data.ase_state = WICED_BT_GA_ASCS_STATE_IDLE;
            gatt_interface_notify_characteristic(conn_id, p_service, &ase_char, &p_ase->data);
        }
    }
}

wiced_result_t lehs_ascs_handle_write_req_evt(uint16_t conn_id,
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
        lehs_ase_data_t *p_ase = NULL;

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
        p_ase = lehs_get_local_ase_app_data_ptr_by_ase_id(lehs_gatt_get_clcb_by_conn_id(conn_id),
                                                                       cp_params.ase_id);
        if (NULL == p_ase)
        {
            p_cp_cmd_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_INVALID_ASE_ID;
            p_cp_cmd_sts->reason = WICED_BT_GA_ASCS_REASON_NOT_APPLICABLE;
            index++;
            continue;
        }

        // validate state transition
        if (!wiced_bt_ga_bap_is_state_transition_valid(p_ase->data.p_ase_info->ase_type,
                                                       p_ase->data.ase_state,
                                                       opcode,
                                                       &next_state))
        {
            p_cp_cmd_sts->response_code = WICED_BT_GA_ASCS_RESPONSE_INVALID_ASE_STATE_MACHINE_TRANSITION;
            p_cp_cmd_sts->reason = WICED_BT_GA_ASCS_REASON_NOT_APPLICABLE;
            index++;
            continue;
        }

        //validate parameters
        lehs_ascs_validate_cp_parameters(conn_id, opcode, &cp_params, p_cp_cmd_sts);
        lehs_ascs_update_ase_data(conn_id, opcode, &cp_params, p_ase, next_state);
        index++;
    }

    // send CP notification indicating status of the command
    gatt_interface_notify_characteristic(conn_id, (gatt_intf_service_object_t *)p_service, p_char, &cp_notif_data);

    // send ASE notifications for all ASE's updated by this command
    lehs_ascs_notify_ase(conn_id, (gatt_intf_service_object_t *)p_service, &cp_notif_data);

    wiced_bt_free_buffer(cp_notif_data.p_status);

    return WICED_SUCCESS;
}

wiced_result_t lehs_ascs_callback(uint16_t conn_id,
                                          void *p_app_ctx,
                                          gatt_intf_service_object_t *p_service,
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
            result = lehs_ascs_handle_write_req_evt(conn_id, p_service, p_char, (uint8_t *)p_data, len);
            break;
        case READ_REQ_EVT:
            result = lehs_ascs_handle_read_req_evt(conn_id, p_char, (wiced_bt_ga_ascs_ase_t *)p_data);
            break;
        default:
            WICED_BT_TRACE("[%s] event %d \n", __FUNCTION__, p_char);
            break;
    }

    return result;
}

void lehs_set_default_ase_params(lehs_ase_data_t *p_ase)
{
    CHECK_FOR_NULL_AND_RETURN(p_ase);

    p_ase->data.ase_state = WICED_BT_GA_ASCS_STATE_IDLE;

    p_ase->data.qos_configured.cig_id = 0xFF;
    p_ase->data.qos_configured.cis_id = 0xFF;

    p_ase->lc3_index = 0;

    WICED_BT_TRACE("[%s] ase_id %d, ase_state 0x%x type %s cis_handle %d",
                   __FUNCTION__,
                   p_ase->data.p_ase_info->ase_id,
                   p_ase->data.ase_state,
                   (p_ase->data.p_ase_info->ase_type == ASCS_SOURCE_ASE_CHARACTERISTIC) ? "src" : "sink",
                   p_ase->cis_conn_handle);
}

void lehs_ascs_init_data(lehs_clcb_t *p_clcb)
{
    int num_ase = p_clcb->num_local_ases;
    lehs_ase_data_t *p_ase = p_clcb->p_local_ase_data;
    WICED_BT_TRACE("[%s] %B", __FUNCTION__, p_clcb->bda);

    for (; num_ase--; p_ase++)
    {
        lehs_set_default_ase_params(p_ase);
    }
}
