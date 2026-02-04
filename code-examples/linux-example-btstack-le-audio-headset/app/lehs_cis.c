/*
* $ Copyright Cypress Semiconductor $
*/

#include "lehs.h"

lehs_ase_data_t *lehs_find_ase_with_cis_id(uint8_t cig_id,
                                                           uint8_t cis_id,
                                                           uint8_t char_type,
                                                           uint16_t *p_conn_id)
{
    lehs_clcb_t *p_clcb = g_lehs_gatt_cb.clcb;
    int limit = sizeof(g_lehs_gatt_cb.clcb) / sizeof(g_lehs_gatt_cb.clcb[0]);
    lehs_ase_data_t *p_ase = NULL;
    int num_ase = 0;

    while (limit--)
    {
        p_ase = p_clcb->p_local_ase_data;
        num_ase = p_clcb->num_local_ases;
        for (; num_ase--; p_ase++)
        {
            wiced_bt_ga_ascs_config_qos_args_t *p_qos = &p_ase->data.qos_configured;
            if (p_ase->data.ase_state < WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED)
            {
                continue;
            }
            if ((p_qos->cig_id == cig_id) && (p_qos->cis_id == cis_id) &&
                (char_type == p_ase->data.p_ase_info->ase_type))
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

lehs_ase_data_t *lehs_get_ase_app_data_ptr_by_cis_conn_hdl(uint16_t cis_conn_hdl, uint16_t *p_conn_id)
{
    lehs_clcb_t *p_clcb = g_lehs_gatt_cb.clcb;
    int limit = sizeof(g_lehs_gatt_cb.clcb) / sizeof(g_lehs_gatt_cb.clcb[0]);
    lehs_ase_data_t *p_ase = NULL;
    int num_ase = 0;
    while (limit--)
    {
        p_ase = p_clcb->p_local_ase_data;
        num_ase = p_clcb->num_local_ases;
        for (; num_ase--; p_ase++)
        {
            wiced_bt_ga_ascs_config_qos_args_t *p_qos = &p_ase->data.qos_configured;
            uint16_t cis_conn_hdl =
                wiced_ble_isoc_get_cis_conn_handle(p_qos->cig_id, p_qos->cis_id, p_ase->acl_conn_handle);

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

static void lehs_cis_handle_connection(wiced_ble_isoc_cis_t *p_cis, uint8_t char_type)
{
    wiced_result_t res = WICED_ERROR;
    uint16_t conn_id;
    lehs_ase_data_t *p_ase = lehs_find_ase_with_cis_id(p_cis->cig_id, p_cis->cis_id, char_type, &conn_id);

    CHECK_FOR_NULL_AND_RETURN(p_ase);

    WICED_BT_TRACE("[%s] ase_id %d, state : %d char_type %d",
                   __FUNCTION__,
                   p_ase->data.p_ase_info->ase_id,
                   p_ase->data.ase_state,
                   p_ase->data.p_ase_info->ase_type);

    //assign CIS to ASE
    p_ase->cis_conn_handle = p_cis->cis_conn_handle;

    if (char_type == ASCS_SINK_ASE_CHARACTERISTIC && p_ase->data.ase_state == WICED_BT_GA_ASCS_STATE_ENABLING)
        res = lehs_isoc_dhm_setup_cis_stream(p_ase);
    else if (char_type == ASCS_SOURCE_ASE_CHARACTERISTIC && p_ase->data.ase_state == WICED_BT_GA_ASCS_STATE_STREAMING)
        res = lehs_isoc_dhm_setup_cis_stream(p_ase);

    if (res)
    {
        WICED_BT_TRACE_CRIT("[%s] data path setup unsuccessful...(err:%d)\n", __FUNCTION__, res);
    }
}

 uint8_t lehs_get_ase_type_form_ase_id(lehs_ase_data_t *p_ase, uint8_t ase_id)
{
     lehs_clcb_t *p_clcb = g_lehs_gatt_cb.clcb;
    int num_ase = p_clcb->num_local_ases;

    while (num_ase--)
    {
        if (p_ase->data.p_ase_info->ase_id == ase_id) return p_ase->data.p_ase_info->ase_type;
        p_ase++;
    }
    return INVALID_ASE_ID;
}

static void lehs_cis_handle_data_path_setup(uint16_t cis_conn_hdl, lehs_ase_data_t *p_ase)
{
    uint16_t conn_id = 0;
    gatt_intf_service_object_t *p_service = NULL;
    uint8_t ase_type;

    if (!lehs_get_ase_app_data_ptr_by_cis_conn_hdl(cis_conn_hdl, &conn_id))
    {
        WICED_BT_TRACE("[%s] did not get conn_id for 0x%x ", __FUNCTION__, cis_conn_hdl);
        return;
    }

    ase_type = p_ase->data.p_ase_info->ase_type;
    WICED_BT_TRACE("[%s] ase_type %s \n", __FUNCTION__, (ase_type == 1)? "ASCS_SINK_ASE_CHARACTERISTIC":"ASCS_SOURCE_ASE_CHARACTERISTIC");

    p_service = gatt_interface_get_service_by_uuid_and_conn_id(0, &ga_service_uuid_ascs);
    CHECK_FOR_NULL_AND_RETURN(p_service);

    // Update ASE data to indicate data path is setup successfully
    p_ase->data_path_established = 1;

    lehs_isoc_dhm_start_stream(cis_conn_hdl, p_ase->data.p_ase_info->ase_type);
    // if server + source and in streaming state, start audio streaming
    // if server + sink and in enabling state, transition to streaming state
    if (ASCS_SINK_ASE_CHARACTERISTIC == ase_type && WICED_BT_GA_ASCS_STATE_ENABLING == p_ase->data.ase_state)
    {
        gatt_intf_attribute_t ase_char = {0};

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
                            ase_type);
    }
}

static void lehs_cis_handle_disconnection(uint8_t cig_id, uint8_t cis_id, uint8_t char_type)
{
    lehs_ase_data_t *p_ase = NULL;
    gatt_intf_service_object_t *p_service = NULL;
    uint16_t conn_id = 0;
    gatt_intf_attribute_t ase_char = {0};

    p_ase = lehs_find_ase_with_cis_id(cig_id, cis_id, char_type, &conn_id);
    CHECK_FOR_NULL_AND_RETURN(p_ase);

    lehs_isoc_dhm_free_cis_stream(p_ase->cis_conn_handle,
                                      p_ase->data.p_ase_info->ase_type);

    if (!conn_id)
    {
        WICED_BT_TRACE_CRIT("[%s] conn_id is 0", __FUNCTION__);
        conn_id = 0x8000;
        //return;
    }

    p_service = gatt_interface_get_service_by_uuid_and_conn_id(0, &ga_service_uuid_ascs);

    CHECK_FOR_NULL_AND_RETURN(p_service);
    p_ase->data_path_established = 0;

    if (WICED_BT_GA_ASCS_STATE_RELEASING == p_ase->data.ase_state)
    {
        p_ase->data.ase_state = WICED_BT_GA_ASCS_STATE_IDLE;

        p_ase->data.qos_configured.cis_id = 0xFF;
        p_ase->data.qos_configured.cig_id = 0xFF;
    }
    else if (WICED_BT_GA_ASCS_STATE_STREAMING == p_ase->data.ase_state)
    {
        if (ASCS_SOURCE_ASE_CHARACTERISTIC != p_ase->data.p_ase_info->ase_type)
        {
            p_ase->data.ase_state = WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED;
        }
        else
        {
            p_ase->data.ase_state = WICED_BT_GA_ASCS_STATE_DISABLING;
        }
    }
    else if (WICED_BT_GA_ASCS_STATE_DISABLING == p_ase->data.ase_state)
    {
        p_ase->data.ase_state = WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED;
    }
    else if (WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED == p_ase->data.ase_state)
    {
        // wiced_ble_isoc_peripheral_remove_cig(p_ase->data.qos_configured.cig_id);
    }
    else
    {
        return;
    }
    gatt_interface_notify_characteristic(conn_id,
                                         p_service,
                                         ascs_init_characteristic(&ase_char, p_ase),
                                         &p_ase->data);
}

static int set_conn_on_ase_id(wiced_ble_isoc_cis_t *p_cis_req, int type)
{
    uint16_t conn_id;
    lehs_ase_data_t *p_ase =
        lehs_find_ase_with_cis_id(p_cis_req->cig_id, p_cis_req->cis_id, type, &conn_id);
    int ase_id = (p_ase) ? p_ase->data.p_ase_info->ase_id : 0xFF;

    WICED_BT_TRACE("[%s] cis_id %d cig_id %d cis 0x%x acl 0x%x p_ase %x ase_id %d\n",
                   __FUNCTION__,
                   p_cis_req->cis_id,
                   p_cis_req->cig_id,
                   p_cis_req->cis_conn_handle,
                   p_cis_req->acl_conn_handle,
                   p_ase,
                   ase_id);

    if (p_ase)
    {
        p_ase->cis_conn_handle = p_cis_req->cis_conn_handle;
    }

    return ase_id;
}

void lehs_isoc_event_handler(wiced_ble_isoc_event_t event, wiced_ble_isoc_event_data_t *p_event_data)
{
    WICED_BT_TRACE("[%s] event %d ", __FUNCTION__, event);

    switch (event)
    {
        case WICED_BLE_ISOC_CIS_REQUEST_EVT:
        {
            wiced_ble_isoc_cis_t *p_cis_req = &p_event_data->cis_request;
            int snk_ase_id = set_conn_on_ase_id(p_cis_req, ASCS_SINK_ASE_CHARACTERISTIC);

            WICED_BT_TRACE("ase id %d cis_id %d cig_id %d cis_conn_handle %d acl_handle %d \n",
                           snk_ase_id,
                           p_cis_req->cis_id,
                           p_cis_req->cig_id,
                           p_cis_req->cis_conn_handle,
                           p_cis_req->acl_conn_handle);

            wiced_ble_isoc_peripheral_accept_cis(p_cis_req);
        }
        break;

        case WICED_BLE_ISOC_CIS_ESTABLISHED_EVT:
            if (p_event_data->cis_established_data.status)
            {
                WICED_BT_TRACE_CRIT("[%s] status %d \n", __FUNCTION__, p_event_data->cis_established_data.status);
                return;
            }

            // Stup data path after CIS establishment for sink role as server/client,
            // Data path is setup upon receiving streaming notification / receiver start ready
            // as client and server
            lehs_cis_handle_connection(&p_event_data->cis_established_data.cis,
                                               ASCS_SINK_ASE_CHARACTERISTIC);
            lehs_cis_handle_connection(&p_event_data->cis_established_data.cis,
                                                ASCS_SOURCE_ASE_CHARACTERISTIC);
            break;

        case WICED_BLE_ISOC_CIS_DISCONNECTED_EVT:
            // in case of bi-directional CIS handle for both the ASE's attached to the CIS
            lehs_cis_handle_disconnection(p_event_data->cis_disconnect.cis.cig_id,
                                                  p_event_data->cis_disconnect.cis.cis_id,
                                                  ASCS_SINK_ASE_CHARACTERISTIC);

            lehs_cis_handle_disconnection(p_event_data->cis_disconnect.cis.cig_id,
                                          p_event_data->cis_disconnect.cis.cis_id,
                                          ASCS_SOURCE_ASE_CHARACTERISTIC);

            break;
        case WICED_BLE_ISOC_BIG_SYNC_ESTABLISHED_EVT:
        case WICED_BLE_ISOC_BIG_SYNC_LOST_EVT:
            lehs_bis_isoc_cb(event, p_event_data);
            break;

        case WICED_BLE_ISOC_DATA_PATH_SETUP_EVT:
            if (p_event_data->datapath.status)
            {
                WICED_BT_TRACE_CRIT("[%s] Data path setup not successful\n", __FUNCTION__);
                return;
            }

            if (wiced_ble_isoc_is_cis_connected_with_conn_hdl(p_event_data->datapath.conn_hdl))
            {
                lehs_cis_handle_data_path_setup(p_event_data->datapath.conn_hdl, p_event_data->datapath.p_app_ctx);
            }
            else if (wiced_ble_isoc_is_bis_created(p_event_data->datapath.conn_hdl))
            {
                lehs_isoc_dhm_start_stream(p_event_data->datapath.conn_hdl, ASCS_SINK_ASE_CHARACTERISTIC);
            }
            break;

        case WICED_BLE_ISOC_DATA_PATH_REMOVED_EVT:
            if (p_event_data->datapath.status)
            {
                WICED_BT_TRACE_CRIT("[%s] Data path removal not successful\n", __FUNCTION__);
            }
            break;

        default:
            break;
    }
}
