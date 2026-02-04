/*
 * $ Copyright Cypress Semiconductor $
 */

#include "lepl.h"

extern le_audio_cap_start_unicast_param_t unicast_param;

// create a pool to store app ASE info
void lepl_ascs_alloc_memory(void)
{
    for (int index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        lepl_clcb_t *p_clcb = &g_lepl_gatt_cb.unicast_clcb[index];

        /** size of remote ases is the configuration + space for ase data */
        int num_remote_ases = MAX_SINK_ASE_SUPPORTED + MAX_SOURCE_ASE_SUPPORTED;
        int size_remote_ases =
            num_remote_ases * (sizeof(lepl_ase_data_t) + sizeof(wiced_bt_ga_ascs_ase_info_t));

        p_clcb->p_remote_ase_data = wiced_memory_alloc_long_term_mem_block(size_remote_ases, "ase_src");
        if (num_remote_ases && !p_clcb->p_remote_ase_data)
        {
            WICED_BT_TRACE("[%s] %d remote 0x%x %d", __FUNCTION__, index, p_clcb->p_remote_ase_data, size_remote_ases);
            return;
        }

        p_clcb->num_remote_ases = num_remote_ases;
    }
}

void lepl_init_remote_ases(lepl_clcb_t *p_clcb)
{
    lepl_ase_data_t *p_ase = p_clcb->p_remote_ase_data;
    wiced_bt_ga_ascs_ase_info_t *p_ase_info = NULL;
    int num_ases = p_clcb->num_remote_ases;

    memset(p_ase, 0, sizeof(lepl_ase_data_t) * num_ases);
    p_ase_info = (wiced_bt_ga_ascs_ase_info_t *)(p_ase + num_ases);

    for (int index = 0; index < num_ases; index++, p_ase++, p_ase_info++)
    {
        p_ase->data.p_ase_info = p_ase_info;
    }
}

// find the first free slot and assign ASE ID
lepl_ase_data_t *lepl_assign_remote_ase_id(lepl_clcb_t *p_clcb,
                                           const wiced_bt_ga_ascs_ase_info_t *p_recv,
                                                               uint8_t char_type,
                                                               uint8_t char_instance)
{
    int num_ase = p_clcb->num_remote_ases;
    lepl_ase_data_t *p_ase = NULL;
    wiced_bt_ga_ascs_ase_info_t *p_ase_info = NULL;
    int index = 0;

    while (index < num_ase) {
        p_ase = &p_clcb->p_remote_ase_data[index];
        p_ase_info = (wiced_bt_ga_ascs_ase_info_t *)p_ase->data.p_ase_info;

        WICED_BT_TRACE("[%s] index %d ase_id %d type %d inst %d slot ase_id %d slot ase_state %d",
                       __FUNCTION__,
                       index,
                       p_recv->ase_id,
                       char_type,
                       char_instance,
                       p_ase_info->ase_id,
                       p_ase->data.ase_state);

        if (p_ase_info->ase_id == 0) {
            p_ase_info->ase_id = p_recv->ase_id;
            p_ase_info->ase_type = char_type;
            p_ase_info->data_path_dir = p_recv->data_path_dir;
            p_ase->acl_conn_handle = wiced_bt_gatt_get_acl_conn_handle(p_clcb->conn_id);

            return &p_clcb->p_remote_ase_data[index];
        }
        index++;
    }

    return NULL;
}

lepl_ase_data_t *lepl_get_remote_ase(lepl_clcb_t *p_clcb, ascs_characteristics_t type, uint8_t *p_index)
{
    int index = 0;
    lepl_ase_data_t *p_ase;

    if (!p_clcb->p_remote_ase_data)
    {
        return NULL;
    }

    if (p_index)
    {
        index = *p_index;
    }
    p_ase = (p_clcb->p_remote_ase_data + index);

    while ((index < p_clcb->num_remote_ases) && p_ase->data.p_ase_info)
    {
        index++;
        if (p_ase->data.p_ase_info->ase_type == type)
        {
            if (p_index)
            {
                *p_index = index;
            }

            return p_ase;
        }
        p_ase++;
    }
    return NULL;
}

lepl_ase_data_t *lepl_get_remote_ase_data_by_ase_id(lepl_clcb_t *p_clcb, uint8_t ase_id)
{
    lepl_ase_data_t *p_ase = p_clcb->p_remote_ase_data;
    int num_ase = p_clcb->num_remote_ases;

    while (num_ase--) {
        if (p_ase->data.p_ase_info->ase_id == ase_id) {
            return p_ase;
        }
        p_ase++;
    }

    return NULL;
}

gatt_intf_attribute_t *ascs_init_characteristic(gatt_intf_attribute_t *p_char,
                                                     lepl_ase_data_t *p_ase)
{
    memset(p_char, 0, sizeof(gatt_intf_attribute_t));

    p_char->characteristic_type = p_ase->data.p_ase_info->ase_type;
    p_char->characteristic_instance = 1;

    return p_char;
}

wiced_bool_t lepl_handle_ase_notification(uint16_t conn_id, lepl_ase_data_t *p_ase)
{
    wiced_result_t data_path_setup_sts = WICED_ERROR;

    if (!p_ase)
    {
        WICED_BT_TRACE_CRIT("[%s] p_ase is null", __FUNCTION__);
        return TRUE;
    }
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    gatt_intf_service_object_t *p_service = p_clcb->peer_profiles.p_ascs;

    //TODO: validate state before using CIS/CIG id.. create separate fn for common use..
    uint16_t cis_conn_handle = p_ase->cis_conn_handle;

    WICED_BT_TRACE("[%s] p_ase 0x%x 0x%x, 0x%x", __FUNCTION__, p_ase, p_ase->data.ase_state, cis_conn_handle);

    switch (p_ase->data.ase_state) {
        case WICED_BT_GA_ASCS_STATE_ENABLING:
            //if client is sink setup datapath and send rcr start ready
            if (ASCS_SOURCE_ASE_CHARACTERISTIC == p_ase->data.p_ase_info->ase_type) {
                if (!p_ase->data_path_established && cis_conn_handle)
                {
                    data_path_setup_sts = lepl_isoc_dhm_setup_cis_datapath(p_ase);

                    if (data_path_setup_sts) {
                        WICED_BT_TRACE_CRIT("[%s] data path setup unsuccessful..(err:%d)\n",
                                            __FUNCTION__,
                                            data_path_setup_sts);
                        return FALSE;
                    }
                }
            }
            break;

        case WICED_BT_GA_ASCS_STATE_STREAMING:
		WICED_BT_TRACE("[%s] type %d",__FUNCTION__, p_ase->data.p_ase_info->ase_type);
            if (!p_ase->data_path_established) {
                data_path_setup_sts = lepl_isoc_dhm_setup_cis_datapath(p_ase);

                if (data_path_setup_sts) {
                    WICED_BT_TRACE_CRIT("[%s] data path setup unsuccessful..(err:%d)\n",
                                        __FUNCTION__,
                                        data_path_setup_sts);
                    return FALSE;
                }
            }
            else
            {
                lepl_isoc_dhm_start_cis_stream(cis_conn_handle, p_ase->data.p_ase_info->data_path_dir);
            }
            break;

        case WICED_BT_GA_ASCS_STATE_DISABLING:
            // TODO: prepare for not receiving audio (ALSA config if any)
            // (source will not send data, datapath and CIS connection will be active..)
            wiced_bt_ga_ascs_send_receiver_start_stop_ready(conn_id,
                                                            p_service,
                                                            p_ase->data.p_ase_info->ase_id,
                                                            FALSE);
            break;

        case WICED_BT_GA_ASCS_STATE_RELEASING:
            // TODO: disconnect CIS after data path removal is successful
            lepl_isoc_dhm_remove_cis_datapath(cis_conn_handle, p_ase->data.p_ase_info->ase_type);
            break;

        default:
            break;
    }
    return TRUE;
}

void lepl_set_default_ase_params(lepl_ase_data_t *p_ase, wiced_bool_t is_client)
{
    if (!p_ase) {
        return;
    }

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

void wiced_bt_cap_ascs_handle_ase_read_notification(uint16_t conn_id,
                                                    uint32_t state,
                                                    const gatt_intf_service_object_t *p_service,
                                                    gatt_intf_attribute_t *p_char,
                                                    wiced_bt_ga_ascs_ase_t *p_notif_data)
{
    lepl_ase_data_t *p_ase = NULL;

    WICED_BT_TRACE("[%s] state %d", __FUNCTION__, state);

    p_ase = lepl_get_remote_ase_data_by_ase_id(lepl_gatt_get_clcb_by_conn_id(conn_id),
                                                              p_notif_data->p_ase_info->ase_id);
    if (!p_ase) {
        return;
    }
    p_ase->data.ase_state = state;

    switch (state) {
        case WICED_BT_GA_ASCS_STATE_CODEC_CONFIGURED:
            memcpy(&p_ase->data.codec_configured,
                   &p_notif_data->codec_configured,
                   sizeof(wiced_bt_ga_ascs_config_codec_args_t));
            memcpy(&p_ase->data.p_ase_info->ascs_data,
                   &p_notif_data->p_ase_info->ascs_data,
                   sizeof(wiced_bt_ga_ascs_ase_preferences_t));

            break;

        case WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED:
            memcpy(&p_ase->data.qos_configured,
                   &p_notif_data->qos_configured,
                   sizeof(wiced_bt_ga_ascs_config_qos_args_t));
            break;

        case WICED_BT_GA_ASCS_STATE_IDLE: {
            lepl_set_default_ase_params(p_ase, TRUE); // revisit: tbd
        } break;

        case WICED_BT_GA_ASCS_STATE_ENABLING:
        case WICED_BT_GA_ASCS_STATE_DISABLING:
        case WICED_BT_GA_ASCS_STATE_RELEASING:
        case WICED_BT_GA_ASCS_STATE_STREAMING:
            lepl_handle_ase_notification(conn_id, p_ase);
            break;

        default:
            break;
    }
}

wiced_result_t lepl_ascs_callback(uint16_t conn_id,
                                            void *p_app_ctx,
                                            gatt_intf_service_object_t *p_service,
                                            wiced_bt_gatt_status_t status,
                                            uint32_t evt_type,
                                            gatt_intf_attribute_t *p_char,
                                            void *p_data,
                                            int len)
{
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);

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
                lepl_ase_data_t *p_ase = lepl_get_remote_ase_data_by_ase_id(p_clcb, ase_id);
                if (NULL == p_ase) {

                    /* If no record found, create an entry now */
                    p_ase = lepl_assign_remote_ase_id(p_clcb,
                                                           p_received_data->p_ase_info,
                                                           p_char->characteristic_type,
                                                           p_char->characteristic_instance);
                    if (NULL == p_ase) return WICED_ERROR;
                }

                p_ase->data.ase_state = p_received_data->ase_state;
                WICED_BT_TRACE("[%s] [%d => %s]\n", __FUNCTION__, ase_id, ascs_state_str[p_received_data->ase_state]);

                wiced_bt_cap_ascs_handle_ase_read_notification(conn_id,
                                                               p_received_data->ase_state,
                                                               p_service,
                                                               p_char,
                                                               p_received_data);
                if (evt_type == NOTIFICATION_EVT)
                {
                    le_audio_cap_ascs_update_event(&g_lepl_gatt_cb.cap_profile_data,
                                                   &unicast_param,
                                                   conn_id,
                                                   status,
                                                   evt_type,
                                                   p_char,
                                                   p_data,
                                                   len);
                }
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
