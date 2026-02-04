/*
 * $ Copyright Cypress Semiconductor $
 */

#include "lepl.h"
#include "lepl_bis.h"

extern lepl_broadcast_source_cb_t g_broadcast_source_cb;

static lepl_ase_data_t *lepl_find_ase_with_cis_id(uint8_t cig_id,
                                                  uint8_t cis_id,
                                                  uint8_t char_type,
                                                  uint16_t *p_conn_id)
{
    lepl_clcb_t *p_clcb = g_lepl_gatt_cb.unicast_clcb;
    int limit = sizeof(g_lepl_gatt_cb.unicast_clcb) / sizeof(g_lepl_gatt_cb.unicast_clcb[0]);
    lepl_ase_data_t *p_ase = NULL;

    while (limit--)
    {
        p_ase = p_clcb->p_remote_ase_data;
        int num_ase = p_clcb->num_remote_ases;
        for (; num_ase--; p_ase++)
        {
            wiced_bt_ga_ascs_config_qos_args_t *p_qos = &p_ase->data.qos_configured;
            if (p_ase->data.ase_state < WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED)
            {
                continue;
            }
            if ((p_qos->cig_id == cig_id) && (p_qos->cis_id == cis_id) &&
                (p_ase->data.p_ase_info->ase_type == char_type))
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

static lepl_ase_data_t* lepl_cis_handle_connection(wiced_ble_isoc_cis_t *p_cis, uint8_t char_type)
{
    wiced_result_t res = WICED_ERROR;
    uint16_t conn_id;
    lepl_ase_data_t *p_ase = lepl_find_ase_with_cis_id(p_cis->cig_id, p_cis->cis_id, char_type, &conn_id);

    CHECK_FOR_NULL_AND_RETURN_VALUE(p_ase, NULL);

    WICED_BT_TRACE("[%s] ase_id %d, state : %d characteristic_type %d",
                   __FUNCTION__,
                   p_ase->data.p_ase_info->ase_id,
                   p_ase->data.ase_state,
                   p_ase->data.p_ase_info->ase_type);

    //assign CIS to ASE
    p_ase->cis_conn_handle = p_cis->cis_conn_handle;
    if (p_ase->data.ase_state < WICED_BT_GA_ASCS_STATE_ENABLING)
    {
        return p_ase;
    }
    else
    {
        res = lepl_isoc_dhm_setup_cis_datapath(p_ase);

        if (res)
        {
            WICED_BT_TRACE_CRIT("[%s] data path setup unsuccessful...(err:%d)\n", __FUNCTION__, res);
        }
    }
    return p_ase;
}

static void lepl_cis_handle_disconnection(wiced_ble_isoc_cis_t *p_cis)
{
    lepl_ase_data_t *p_ase = NULL;
    uint16_t conn_id = 0;

    // in case of bi-directional CIS handle for both the ASE's attached to the CIS
    p_ase = lepl_find_ase_with_cis_id(p_cis->cig_id, p_cis->cis_id, ASCS_SINK_ASE_CHARACTERISTIC, &conn_id);
    if (p_ase)
    {
        lepl_isoc_dhm_remove_cis_datapath(p_ase->cis_conn_handle, p_ase->data.p_ase_info->ase_type);
        p_ase->data_path_established = 0;
        p_ase->cis_conn_handle = 0;
    }

    p_ase = lepl_find_ase_with_cis_id(p_cis->cig_id, p_cis->cis_id, ASCS_SOURCE_ASE_CHARACTERISTIC, &conn_id);
    if (p_ase)
    {
        lepl_isoc_dhm_remove_cis_datapath(p_ase->cis_conn_handle, p_ase->data.p_ase_info->ase_type);
        p_ase->data_path_established = 0;
        p_ase->cis_conn_handle = 0;
    }

    wiced_result_t res = wiced_ble_isoc_central_remove_cig(p_cis->cig_id);
    if (res != WICED_BT_ILLEGAL_ACTION)
    {
        lepl_isoc_dhm_stop_stream(p_cis->cis_conn_handle);
        lepl_isoc_dhm_disable_audio();
    }
}

static lepl_ase_data_t *lepl_get_ase_app_data_ptr_by_cis_conn_hdl(uint16_t cis_conn_hdl,uint16_t *p_conn_id)
{
    lepl_clcb_t *p_clcb = g_lepl_gatt_cb.unicast_clcb;
    int limit = sizeof(g_lepl_gatt_cb.unicast_clcb) / sizeof(g_lepl_gatt_cb.unicast_clcb[0]);
    lepl_ase_data_t *p_ase = NULL;
    int num_ase = 0;
    while (limit--)
    {
        p_ase = p_clcb->p_remote_ase_data;
        num_ase = p_clcb->num_remote_ases;
        for (; num_ase--; p_ase++)
        {
            if (p_ase->data.ase_state < WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED)
            {
                continue;
            }
            if (p_ase->cis_conn_handle == cis_conn_hdl)
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

#if 0
static uint8_t lepl_get_ase_type_form_ase_id(lepl_ase_data_t *p_ase, uint8_t ase_id)
{
    lepl_clcb_t *p_clcb = g_lepl_gatt_cb.unicast_clcb;
    int num_ase = p_clcb->num_remote_ases;

    while (num_ase--)
    {
        if (p_ase->data.p_ase_info->ase_id == ase_id) return p_ase->data.p_ase_info->ase_type;
        p_ase++;
    }
    return INVALID_ASE_ID;
}
#endif

static void lepl_cis_handle_data_path_setup(uint16_t cis_conn_hdl,
                                            wiced_bool_t is_cl,
                                            lepl_ase_data_t * p_ase)
{
    uint16_t conn_id = 0;
    uint8_t ase_type, is_inp;
    gatt_intf_service_object_t *p_service = NULL;

    CHECK_FOR_NULL_AND_RETURN(p_ase);

    ase_type = p_ase->data.p_ase_info->ase_type;
    is_inp = (ase_type == ASCS_SINK_ASE_CHARACTERISTIC) ? 1 : 0; // for char_type : char_type_peer : is_cl-TRUE
    ase_type = p_ase->data.p_ase_info->ase_type;

    WICED_BT_TRACE("[%s] ase_type %s \n", __FUNCTION__,(ase_type == ASCS_SOURCE_ASE_CHARACTERISTIC) ? "ASCS_SOURCE" : "ASCS_SINK" );

    uint8_t char_type_local = (is_inp) ? ASCS_SOURCE_ASE_CHARACTERISTIC : ASCS_SINK_ASE_CHARACTERISTIC;
    uint8_t char_type_peer = (is_inp) ? ASCS_SINK_ASE_CHARACTERISTIC : ASCS_SOURCE_ASE_CHARACTERISTIC;
    uint8_t char_type = (is_cl) ? char_type_peer : char_type_local;

    WICED_BT_TRACE("[%s] is_inp %d is_cl %d\n", __FUNCTION__, is_inp, is_cl);

    lepl_get_ase_app_data_ptr_by_cis_conn_hdl(cis_conn_hdl, &conn_id);

    if (is_cl) {
        lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
        p_service = p_clcb->peer_profiles.p_ascs;
    }
    CHECK_FOR_NULL_AND_RETURN(p_service);

    // Update ASE data to indicate data path is setup successfully
    p_ase->data_path_established = 1;

    if (is_cl)
    {
        // if client + source and in streaming state, start audio streaming
        // if client + sink and in enabling state, send receiver start ready
        if (WICED_BT_GA_ASCS_STATE_STREAMING == p_ase->data.ase_state)
        {
            lepl_isoc_dhm_start_cis_stream(cis_conn_hdl, p_ase->data.p_ase_info->data_path_dir);
        }
        else if (ASCS_SOURCE_ASE_CHARACTERISTIC == char_type &&
                 WICED_BT_GA_ASCS_STATE_ENABLING == p_ase->data.ase_state)
        {
            wiced_bt_ga_ascs_send_receiver_start_stop_ready(conn_id,
                                                            p_service,
                                                            p_ase->data.p_ase_info->ase_id,
                                                            TRUE);
        }
        else
        {
            WICED_BT_TRACE_CRIT("[%s] cl unexpected state %d char_type %d\n",
                                __FUNCTION__,
                                p_ase->data.ase_state,
                                char_type);
        }
    }
    else
    {
        // if server + source and in streaming state, start audio streaming
        // if server + sink and in enabling state, transition to streaming state

        if (WICED_BT_GA_ASCS_STATE_STREAMING == p_ase->data.ase_state)
        {
            lepl_isoc_dhm_start_cis_stream(cis_conn_hdl, p_ase->data.p_ase_info->data_path_dir);
        }
        else if (ASCS_SINK_ASE_CHARACTERISTIC == char_type &&
                 WICED_BT_GA_ASCS_STATE_ENABLING == p_ase->data.ase_state)
        {
            gatt_intf_attribute_t ase_char;

            p_ase->data.ase_state = WICED_BT_GA_ASCS_STATE_STREAMING;
            gatt_interface_notify_characteristic(conn_id,
                                                 p_service,
                                                 ascs_init_characteristic(&ase_char, p_ase),
                                                 &p_ase->data);
        }
        else
        {
            WICED_BT_TRACE_CRIT("[%s] sr unexpected state %d char_type %d\n",
                                __FUNCTION__,
                                p_ase->data.ase_state,
                                char_type);
        }
    }
}

void lepl_isoc_event_handler(wiced_ble_isoc_event_t event, wiced_ble_isoc_event_data_t *p_event_data)
{
    static wiced_bool_t is_client = TRUE;
    wiced_ble_isoc_set_cig_cmd_status_evt_t *p_cig_status_data = NULL;
    wiced_ble_isoc_create_big_cmpl_evt_t *p_create_big_sts = NULL;
    lepl_broadcast_source_cb_t *p_big = &g_broadcast_source_cb;
    wiced_result_t res = WICED_ERROR;

    WICED_BT_TRACE("[%s] event %d ", __FUNCTION__, event);

    switch (event)
    {
    case WICED_BLE_ISOC_SET_CIG_CMD_COMPLETE_EVT:

        p_cig_status_data = &p_event_data->cig_status_data;

        if (WICED_BT_SUCCESS != p_cig_status_data->status) return;

        WICED_BT_TRACE("status %d cig_id %d cis_count %d CIS Handle %d\n",
                       p_cig_status_data->status,
                       p_cig_status_data->cig_id,
                       p_cig_status_data->cis_count,
                       p_cig_status_data->cis_connection_handle_list[0]);

        break;

    case WICED_BLE_ISOC_CIS_ESTABLISHED_EVT:
        if (p_event_data->cis_established_data.status)
        {
            WICED_BT_TRACE_CRIT("[%s] status %d \n", __FUNCTION__, p_event_data->cis_established_data.status);
            wiced_ble_isoc_central_remove_cig(p_event_data->cis_established_data.cis.cig_id);
            return;
        }

        // Setup data path after CIS establishment for sink role as server/client,
        // Data path is setup upon receiving streaming notification / receiver start ready
        // as client and server
        if (is_client)
        {

            lepl_ase_data_t * p_source_ase =
                lepl_cis_handle_connection(&p_event_data->cis_established_data.cis,
                                                        ASCS_SOURCE_ASE_CHARACTERISTIC);

            lepl_ase_data_t *p_sink_ase =
                lepl_cis_handle_connection(&p_event_data->cis_established_data.cis,
                                       ASCS_SINK_ASE_CHARACTERISTIC);

            if (!p_source_ase && !p_sink_ase)
            {
                WICED_BT_TRACE("[%s] disconnecting CIS..\n", __FUNCTION__);
                wiced_ble_isoc_disconnect_cis(p_event_data->datapath.conn_hdl);
            }
        }
        break;

    case WICED_BLE_ISOC_CIS_DISCONNECTED_EVT:
        lepl_cis_handle_disconnection(&p_event_data->cis_disconnect.cis);
        break;

    case WICED_BLE_ISOC_DATA_PATH_SETUP_EVT:
        if (p_event_data->datapath.status)
        {
            WICED_BT_TRACE_CRIT("[%s] Data path setup not successful\n", __FUNCTION__);
            return;
        }

        if (wiced_ble_isoc_is_cis_connected_with_conn_hdl(p_event_data->datapath.conn_hdl))
        {

		    WICED_BT_TRACE("[%s] p_ase 0x%x",__FUNCTION__, p_event_data->datapath.p_app_ctx);
            lepl_cis_handle_data_path_setup(p_event_data->datapath.conn_hdl,
                                            is_client,
                                            p_event_data->datapath.p_app_ctx);
        }
        else if (wiced_ble_isoc_is_bis_created(p_event_data->datapath.conn_hdl))
        {
             p_big->base.state = BAP_BROADCAST_STATE_STREAMING;
             WICED_BT_TRACE("[%s] BASE State [%d] \n", __FUNCTION__, p_big->base.state);

            // start streaming as broadcast source
            lepl_isoc_dhm_start_bis_stream(p_event_data->datapath.conn_hdl);
        }
        break;

    case WICED_BLE_ISOC_DATA_PATH_REMOVED_EVT:
        if (p_event_data->datapath.status)
        {
            WICED_BT_TRACE_CRIT("[%s] Data path removal not successful\n", __FUNCTION__);
        }
        if (wiced_ble_isoc_is_bis_created(p_event_data->datapath.conn_hdl))
        {
            lepl_isoc_dhm_stop_stream(p_event_data->datapath.conn_hdl);
        }
        else
        {
            if (is_client)
            { // TODO: check id data path is removed in both directions
                WICED_BT_TRACE("[%s] disconnecting CIS..\n", __FUNCTION__);
                wiced_ble_isoc_disconnect_cis(p_event_data->datapath.conn_hdl);
            }
        }
        break;

    case WICED_BLE_ISOC_BIG_CREATED_EVT:
    {
        p_create_big_sts = &p_event_data->create_big;

         if (p_create_big_sts->sync_data.status)
         {
             WICED_BT_TRACE_CRIT("[%s] BIG Creation unsuccessful\n", __FUNCTION__);
             return;
         }

         /* Map BIS index and bis_conn_handle */
         p_big->bis_conn_id_count = p_create_big_sts->sync_data.num_bis;
         memcpy(p_big->bis_conn_id_list,
                p_create_big_sts->sync_data.bis_conn_hdl_list,
                p_big->bis_conn_id_count * sizeof(uint16_t));

         /* start setting up data paths for all the BIS streams */
         for (int i = 0; i < p_big->bis_conn_id_count; i++)
         {
             wiced_bt_ga_bap_csc_t *p_csc = &p_big->base.sub_group[0].csc;
             if (p_big->base.sub_group[0].bis_config[i].bis_csc.audio_channel_allocation)
                 p_csc->audio_channel_allocation =
                     p_big->base.sub_group[0].bis_config[i].bis_csc.audio_channel_allocation;

             res = lepl_isoc_dhm_setup_bis_datapath(p_big->bis_conn_id_list[i], p_csc);
             if (res)
             {
                 WICED_BT_TRACE_CRIT("[%s] data path setup unsuccessful...(err:%d)\n", __FUNCTION__, res);
             }

         }
        }
        break;

    case WICED_BLE_ISOC_BIG_TERMINATED_EVT:
    {
         // p_big->base.state = BAP_BROADCAST_STATE_CONFIGURED;
         // FIXME: Disabling state check to allow this fn. call immediately after
         // disabling stream. Should have a mechanism to queue release req to handle
         // from data path removed
        lepl_isoc_dhm_disable_audio();
         WICED_BT_TRACE("[%s] BASE State [%d] \n", __FUNCTION__, p_big->base.state);
    }break;

    default:
        break;
    }
}
void lepl_isoc_init(void)
{
    wiced_ble_isoc_cfg_t cfg = {.max_cis = 2, .max_bis = 2};

    wiced_ble_isoc_init(&cfg, lepl_isoc_event_handler);

    /* Initialize audio interfaces and register callbacks for data handling */
    lepl_isoc_dhm_init();
}
