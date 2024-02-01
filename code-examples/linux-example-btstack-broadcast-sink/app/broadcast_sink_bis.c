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

/******************************************************************************/

/* Application includes */
#include "broadcast_sink_bis.h"
#include "broadcast_sink_bass.h"
#include "broadcast_sink_rpc.h"
#include "le_audio_rpc.h"

/* App Library includes */
#include "audio_driver.h"
#include "broadcast_sink_iso_audio.h"

/* BT Stack includes */
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "wiced_bt_ga_pbp.h"
#include "log.h"


/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define MAX_BIG 20


/******************************************************************************
 *                              EXTERNS VARIABLE
 *****************************************************************************/


/******************************************************************************
 *                              EXTERNS FUNCTION
 *****************************************************************************/


/*******************************************************************************
*                               STRUCTURES AND ENUMERATIONS
*******************************************************************************/


/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
broadcast_sink_cb_t g_broadcast_sink_cb[MAX_BIG] = {0};
static wiced_bool_t broadcast_sink_periodic_sync_in_progress = FALSE;
broadcast_source_t broadcast_source = {0};

void broadcast_sink_create_periodic_sync(wiced_bt_ble_scan_results_t *p_scan_result);

/******************************************************************************
*                               FUNCTION PROTOTYPE
******************************************************************************/
void broadcast_sink_bis_menu_discover_sources(bool start);
void broadcast_sink_bis_menu_sync_to_source(uint32_t broadcast_id, bool listen, uint8_t* broadcast_code);
void broadcast_sink_bis_clear_data(void);
void broadcast_sink_bis_sync_to_source(wiced_bt_ble_scan_type_t scan_type, broadcast_source_t source);
void broadcast_sink_bis_discover_sources(wiced_bt_ble_scan_type_t scan_type);
void broadcast_sink_bis_free_big(broadcast_sink_cb_t *p_big);
void broadcast_sink_bis_init(wiced_bt_cfg_isoc_t *p_isoc_cfg);
broadcast_sink_cb_t *broadcast_sink_bis_alloc_big(uint32_t broadcast_id, wiced_bt_device_address_t bd_addr, uint8_t adv_sid);
broadcast_sink_cb_t *broadcast_sink_bis_get_big_by_sync_handle(wiced_bt_ble_periodic_adv_sync_handle_t sync_handle);
broadcast_sink_cb_t *broadcast_sink_bis_get_big_by_broadcast_id(uint32_t br_id);

static void priv_periodic_adv_report_handler(wiced_bt_ble_adv_ext_event_t event, wiced_bt_ble_adv_ext_event_data_t *p_ed);
static void broadcast_sink_bis_menu_ext_adv_scan_cback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);
static void broadcast_sink_bis_ext_adv_scan_cback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);
static void broadcast_sink_bis_ext_adv_scan_to_sync_cback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);
static void broadcast_sink_bis_create_periodic_sync(wiced_bt_ble_scan_results_t *p_ext_adv_report);
static void broadcast_sink_bis_isoc_cb(wiced_bt_isoc_event_t event, wiced_bt_isoc_event_data_t *p_ed);
static void broadcast_sink_bis_ext_adv_cback(wiced_bt_ble_adv_ext_event_t event, wiced_bt_ble_adv_ext_event_data_t *p_ed);
static void broadcast_sink_bis_update_sync_handle(wiced_bt_device_address_t bd_addr, wiced_bt_ble_ext_adv_sid_t adv_sid, wiced_bt_ble_periodic_adv_sync_handle_t sync_handle);
static broadcast_sink_cb_t *broadcast_sink_bis_get_big_by_adv_sid(wiced_bt_device_address_t bd_addr, wiced_bt_ble_ext_adv_sid_t adv_sid);
static broadcast_sink_cb_t *broadcast_sink_bis_get_big_by_handle(uint8_t big_handle);


/******************************************************************************
 * Function Name: broadcast_sink_bis_get_big_by_handle
 *******************************************************************************
 * Summary:
 *   Get big by handle
 *
 * Parameters:
 *   uint8_t big_handle :   big handle
 *
 * Return:
 *   broadcast_sink_cb_t : information structure
 *
 ******************************************************************************/
broadcast_sink_cb_t *broadcast_sink_bis_get_big_by_handle(uint8_t big_handle)
{
    for (size_t i = 0; i < MAX_BIG; i++)
    {
        if (TRUE == g_broadcast_sink_cb[i].in_use && big_handle == g_broadcast_sink_cb[i].big_handle)
        {
            return &g_broadcast_sink_cb[i];
        }
    }

    return NULL;
}

/******************************************************************************
 * Function Name: broadcast_sink_bis_get_big_by_broadcast_id
 *******************************************************************************
 * Summary:
 *   Get big by broadcast ID
 *
 * Parameters:
 *   uint32_t br_id :   braodcast ID
 *
 * Return:
 *   broadcast_sink_cb_t : information structure
 *
 ******************************************************************************/
broadcast_sink_cb_t *broadcast_sink_bis_get_big_by_broadcast_id(uint32_t br_id)
{
    for (size_t i = 0; i < MAX_BIG; i++)
    {
        if (TRUE == g_broadcast_sink_cb[i].in_use && br_id == g_broadcast_sink_cb[i].base.broadcast_id)
        {
            return &g_broadcast_sink_cb[i];
        }
    }

    return NULL;
}

broadcast_sink_cb_t *broadcast_sink_bis_get_big_by_adv_sid(wiced_bt_device_address_t bd_addr,
                                                           wiced_bt_ble_ext_adv_sid_t adv_sid)
{
    WICED_BT_TRACE("[%s] Searching for [addr:%B] [adv_sid:%d] \n", __FUNCTION__, bd_addr, adv_sid);

    for (size_t i = 0; i < MAX_BIG; i++)
    {
        // WICED_BT_TRACE("[%s] Entry %d => [addr:%B] [adv_sid:%d] \n",
        //                __FUNCTION__,
        //                g_broadcast_sink_cb[i].bd_addr,
        //                g_broadcast_sink_cb[i].adv_handle);

        if (TRUE == g_broadcast_sink_cb[i].in_use &&
            0 == memcmp(bd_addr, g_broadcast_sink_cb[i].bd_addr, BD_ADDR_LEN) &&
            adv_sid == g_broadcast_sink_cb[i].adv_handle)
        {
            WICED_BT_TRACE("[%s] Entry %d is matching\n", __FUNCTION__, i);
            return &g_broadcast_sink_cb[i];
        }
    }

    return NULL;
}

/******************************************************************************
 * Function Name: broadcast_sink_bis_menu_sync_to_source
 *******************************************************************************
 * Summary:
 *   Manually sync to source
 *
 * Parameters:
 *   uint32_t broadcast_id  : broadcast ID
 *   bool listen            : listen to source or not
 *   uint8_t broadcast_code : broadcast code
 *
 * Return:
 *
 ******************************************************************************/
void broadcast_sink_bis_menu_sync_to_source(uint32_t broadcast_id, bool listen, uint8_t* broadcast_code)
{
    broadcast_sink_cb_t *p_big = NULL;
    broadcast_source_t source = {0};

    uint8_t lc3_index[2] = {0};

    source.broadcast_id = broadcast_id;
    TRACE_LOG("Broadcast ID : 0x%x", source.broadcast_id);

    memcpy(&source.broadcast_code, broadcast_code, 16);
    WICED_BT_TRACE_ARRAY(&source.broadcast_code, 16, "Broadcast code :\n");

    if (!listen)
    {
        p_big = broadcast_sink_bis_get_big_by_broadcast_id(source.broadcast_id);
        if (p_big == NULL)
        {
            TRACE_ERR("[%s] p_big is null", __FUNCTION__);
            return;
        }

        wiced_bool_t res = wiced_bt_isoc_peripheral_big_terminate_sync(p_big->big_handle);
        wiced_bt_ble_terminate_sync_to_periodic_adv(p_big->sync_handle);
        iso_audio_remove_data_path(p_big->bis_conn_id_list[0], WICED_BLE_ISOC_DPD_OUTPUT_BIT, lc3_index);
        broadcast_sink_bis_free_big(p_big);
    }

    broadcast_sink_bis_sync_to_source(listen ? BTM_BLE_SCAN_TYPE_HIGH_DUTY : BTM_BLE_SCAN_TYPE_NONE, source);
}

/******************************************************************************
 * Function Name: broadcast_sink_bis_get_big_by_sync_handle
 *******************************************************************************
 * Summary:
 *   Get big by sync handle
 *
 * Parameters:
 *   wiced_bt_ble_periodic_adv_sync_handle_t sync_handle    : sync handle
 *
 * Return:
 *  broadcast_sink_cb_t : information structure
 *
 ******************************************************************************/
broadcast_sink_cb_t *broadcast_sink_bis_get_big_by_sync_handle(wiced_bt_ble_periodic_adv_sync_handle_t sync_handle)
{
    for (size_t i = 0; i < MAX_BIG; i++)
    {
        if (TRUE == g_broadcast_sink_cb[i].in_use && sync_handle == g_broadcast_sink_cb[i].sync_handle)
        {
            return &g_broadcast_sink_cb[i];
        }
    }

    return NULL;
}

void broadcast_sink_bis_update_sync_handle(wiced_bt_device_address_t bd_addr,
                                           wiced_bt_ble_ext_adv_sid_t adv_sid,
                                           wiced_bt_ble_periodic_adv_sync_handle_t sync_handle)
{
    broadcast_sink_cb_t *p_big = broadcast_sink_bis_get_big_by_adv_sid(bd_addr, adv_sid);
    if (!p_big) return;

    WICED_BT_TRACE("[%s] Updating sync_handle to %d\n", __FUNCTION__, sync_handle);

    p_big->sync_handle = sync_handle;
}

void priv_periodic_adv_report_handler(wiced_bt_ble_adv_ext_event_t event, wiced_bt_ble_adv_ext_event_data_t *p_ed)
{
    broadcast_sink_cb_t *p_big = NULL;
    wiced_bt_ble_periodic_adv_sync_handle_t sync_handle = (WICED_BT_BLE_PERIODIC_ADV_REPORT_EVENT == event)
                                                              ? p_ed->periodic_adv_report.sync_handle
                                                              : p_ed->biginfo_adv_report.sync_handle;
    wiced_bt_isoc_big_create_sync_t create_sync = {0};
    p_big = broadcast_sink_bis_get_big_by_sync_handle(sync_handle);
    if (!p_big)
    {
        WICED_BT_TRACE("[%s] p_big is NULL\n", __FUNCTION__, sync_handle);
        return;
    }

    // check if the report is processed already
    if (WICED_BT_BLE_PERIODIC_ADV_REPORT_EVENT == event && !p_big->b_base_updated)
    {
        /* received truncated data, discard the report */
        if (2 == p_ed->periodic_adv_report.data_status)
        {
            WICED_BT_TRACE("[%s] Dropping truncated PA report\n", __FUNCTION__);
            if (p_big->p_periodic_adv_data)
            {
                wiced_bt_free_buffer(p_big->p_periodic_adv_data);
                p_big->p_periodic_adv_data = NULL;
            }
            return;
        }

        /* Allocate memory to store the periodic adv data */
        if (!p_big->p_periodic_adv_data)
        {
            /* Discard if the report is not Basic Announcement  */
            if (!wiced_bt_ga_bap_broadcast_is_basic_announcement(p_ed->periodic_adv_report.adv_data, &p_big->base_len))
            {
                return;
            }

            p_big->p_periodic_adv_data = wiced_bt_get_buffer(p_big->base_len);
            WICED_BT_TRACE("[%s] p_periodic_adv_data 0x%x [%d bytes]\n",
                           __FUNCTION__,
                           p_big->p_periodic_adv_data,
                           p_big->base_len);
            if (!p_big->p_periodic_adv_data) return;

            p_big->periodic_adv_data_offset = 0;
        }

        /* Store data for future if partial data is received */
        if (p_ed->periodic_adv_report.data_status)
        {
            memcpy(p_big->p_periodic_adv_data + p_big->periodic_adv_data_offset,
                   p_ed->periodic_adv_report.adv_data,
                   p_ed->periodic_adv_report.data_len);

            p_big->periodic_adv_data_offset += p_ed->periodic_adv_report.data_len;
            WICED_BT_TRACE("[%s] cached partial PA report\n", __FUNCTION__);

            return;
        }

        WICED_BT_TRACE("[%s] Received a new PA report [SyncHandle:%d] len %d base len %d\n",
                       __FUNCTION__,
                       sync_handle,
                       p_ed->periodic_adv_report.data_len,
                       p_big->base_len);

        memcpy(p_big->p_periodic_adv_data + p_big->periodic_adv_data_offset,
               p_ed->periodic_adv_report.adv_data,
               p_ed->periodic_adv_report.data_len);

        // parse the received BASE info and save (adding 4 bytes to skip LTV which is already parsed)
        wiced_bt_ga_bap_broadcast_parse_base_info(p_big->p_periodic_adv_data + 4,
                                                  p_big->base_len - 3,
                                                  &p_big->base,
                                                  BROADCAST_MAX_SUB_GROUP,
                                                  BROADCAST_MAX_BIS_PER_SUB_GROUP);
        wiced_bt_free_buffer(p_big->p_periodic_adv_data);
        p_big->p_periodic_adv_data = NULL;

        p_big->b_base_updated = TRUE;
        p_big->base.state = BAP_BROADCAST_STATE_CONFIGURED;
    }
    else if (WICED_BT_BLE_BIGINFO_ADV_REPORT_EVENT == event)
    {
        /* Discard if BASE is not parsed yet or BIGInfo is already updated*/
        if (!p_big->b_base_updated || p_big->b_biginfo_updated) return;

        /* Enable encryption if stream is encrypted */
        p_big->b_encryption = p_ed->biginfo_adv_report.encryption;
        p_big->b_biginfo_updated = TRUE;
        p_big->base.state = BAP_BROADCAST_STATE_STREAMING;

        if (broadcast_sink_rpc_is_dev_role_sink())
        {
            create_sync.big_handle = p_big->big_handle;
            create_sync.sync_handle = p_big->sync_handle;
            create_sync.max_sub_events = p_big->number_of_subevents;
            create_sync.big_sync_timeout = 10;

            create_sync.num_bis = p_big->base.sub_group[0].bis_cnt;
            create_sync.bis_idx_list = p_big->bis_index_list;
            uint8_t idx = 0;
            for (uint8_t i = 0; i < p_big->base.sub_group_cnt; i++)
            {
                for (uint8_t j = 0; j < p_big->base.sub_group[i].bis_cnt; j++)
                {
                    create_sync.bis_idx_list[idx] =
                        p_big->base.sub_group[i].bis_config[j].bis_idx;
                    WICED_BT_TRACE("[%s] bis idx %d", __FUNCTION__, create_sync.bis_idx_list[idx]);
                    idx++;
                }
            }
            create_sync.encrypt = p_big->b_encryption;
            create_sync.broadcast_code = (p_big->b_encryption) ? broadcast_source.broadcast_code : NULL;

            wiced_bool_t res = wiced_bt_isoc_peripheral_big_create_sync(&create_sync);
            WICED_BT_TRACE("[%s] result %x\n", __FUNCTION__, res);
        }
        else
        {
            if (p_ed->biginfo_adv_report.encryption == WICED_BLE_ISOC_ENCRYPTED)
            {
                broadcast_sink_bass_broadcast_code_check(p_ed->biginfo_adv_report.sync_handle);
            }
            else
            {
                //no broadcast code required, sync to source stream
                wiced_bt_bap_broadcast_code_t null_broadcast_code = {0};
                broadcast_sink_sync_to_source(p_big, null_broadcast_code);
            }
        }
    }

    return;
}

void broadcast_sink_ext_adv_cback(wiced_bt_ble_adv_ext_event_t event, wiced_bt_ble_adv_ext_event_data_t *p_ed)
{
    broadcast_sink_cb_t *p_big = NULL;
    wiced_bt_ble_scan_results_t ext_adv_report = {0};

    // WICED_BT_TRACE("[%s] event [%d]\n", __FUNCTION__, event);

    switch (event)
    {
    case WICED_BT_BLE_PERIODIC_ADV_SYNC_TRANSFER_EVENT:

        WICED_BT_TRACE("[%s] WICED_BT_BLE_PERIODIC_ADV_SYNC_TRANSFER_EVENT adv sid %d sync_handle %d BDA %B",
                       __FUNCTION__,
                       p_ed->sync_transfer.sync_data.adv_sid,
                       p_ed->sync_transfer.sync_data.sync_handle,
                       p_ed->sync_transfer.sync_data.adv_addr);

        broadcast_sink_bis_update_sync_handle(p_ed->sync_transfer.sync_data.adv_addr,
                                              p_ed->sync_transfer.sync_data.adv_sid,
                                              p_ed->sync_transfer.sync_data.sync_handle);

        broadcast_sink_bass_notify_pa_sync_state(&p_ed->sync_transfer.sync_data);
        break;

    case WICED_BT_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT:

        WICED_BT_TRACE("[%s] PERIODIC_ADV_SYNC_ESTABLISHED status 0x%x sync_handle %d BDA %B",
                       __FUNCTION__,
                       p_ed->sync_establish.status,
                       p_ed->sync_establish.sync_handle,
                       p_ed->sync_establish.adv_addr);
        p_big = broadcast_sink_bis_get_big_by_adv_sid(p_ed->sync_establish.adv_addr, p_ed->sync_establish.adv_sid);
        if (p_big)
        {
            p_big->sync_in_progress = WICED_FALSE;
        }
        // Stop any scan operations
        wiced_bt_ble_observe(WICED_FALSE, 0, NULL);

        /* Handle error status */
        if (p_ed->sync_establish.status)
        {
            broadcast_sink_periodic_sync_in_progress = FALSE;
            ext_adv_report.adv_sid = p_ed->sync_establish.adv_sid;
            ext_adv_report.ble_addr_type = p_ed->sync_establish.adv_addr_type;
            memcpy(ext_adv_report.remote_bd_addr, p_ed->sync_establish.adv_addr, WICED_BT_ADDRESS_BYTE_SIZE);
            ext_adv_report.periodic_adv_interval = p_ed->sync_establish.periodic_adv_int;
            broadcast_sink_create_periodic_sync(&ext_adv_report);
            return;
        }

        le_audio_rpc_send_status_update(PA_SYNC_ESTABLISHED);

        /* map sync_handle => (addr, adv_sid) */
        broadcast_sink_bis_update_sync_handle(p_ed->sync_establish.adv_addr,
                                              p_ed->sync_establish.adv_sid,
                                              p_ed->sync_establish.sync_handle);

        broadcast_sink_bass_notify_pa_sync_state(&p_ed->sync_establish);

        /* Issue further sync commands if pending */
        broadcast_sink_create_periodic_sync(NULL);
        break;

    case WICED_BT_BLE_PERIODIC_ADV_REPORT_EVENT:
    case WICED_BT_BLE_BIGINFO_ADV_REPORT_EVENT:
        priv_periodic_adv_report_handler(event, p_ed);
        break;

    case WICED_BT_BLE_PERIODIC_ADV_SYNC_LOST_EVENT:
        le_audio_rpc_send_status_update(PA_SYNC_LOST);
        broadcast_sink_bass_notify_pa_sync_lost(p_ed->sync_handle);
    default:
        break;
    }
}

void broadcast_sink_bis_isoc_cb(wiced_bt_isoc_event_t event, wiced_bt_isoc_event_data_t *p_ed)
{
    broadcast_sink_cb_t *p_big = NULL;
    wiced_bt_isoc_terminated_data_t *p_big_sync_lost = NULL;
    wiced_bt_ga_bap_csc_t *p_csc = NULL;
    wiced_bt_isoc_big_sync_established_t *p_big_sync_established = &p_ed->big_sync_established;
    uint8_t lc3_index[2] = {0};

    WICED_BT_TRACE("[%s] event %d ", __FUNCTION__, event);

    switch (event)
    {
    case WICED_BLE_ISOC_BIG_SYNC_ESTABLISHED: {
        if (p_big_sync_established->status) return;

        p_big = broadcast_sink_bis_get_big_by_handle(p_big_sync_established->big_handle);
        if (!p_big) return;

        le_audio_rpc_send_status_update(BIG_SYNC_ESTABLISHED);
        // TODO: support selecting  a particular BIS instead of hardcoding to 0
        p_csc = &p_big->base.sub_group[0].csc;

        // Stop any scan operations
        wiced_bt_ble_observe(WICED_FALSE, 0, NULL);

        wiced_bt_ble_terminate_sync_to_periodic_adv(p_big->sync_handle);

        memcpy(p_big->bis_conn_id_list,
               p_big_sync_established->bis_conn_hdl_list,
               p_big_sync_established->num_bis * sizeof(uint16_t));

        iso_audio_setup_data_path(p_big->bis_conn_id_list[0],
                                  WICED_BLE_ISOC_DPD_OUTPUT_BIT,
                                  p_csc,
                                  p_big->base.sub_group[0].bis_cnt);
        audio_driver_set_volume((DEFAULT_VOL * 100) / 255);

        broadcast_sink_bass_notify_sync_established(p_big->bd_addr);
    }
    break;

    case WICED_BLE_ISOC_BIG_SYNC_LOST: {
        p_big_sync_lost = &p_ed->big_sync_lost;

        p_big = broadcast_sink_bis_get_big_by_handle(p_big_sync_lost->big_handle);
        if (!p_big) return;

        broadcast_sink_bass_notify_big_sync_lost(p_big->bd_addr);

        iso_audio_remove_data_path(p_big->bis_conn_id_list[0], WICED_BLE_ISOC_DPD_OUTPUT_BIT, lc3_index);
        broadcast_sink_bis_free_big(p_big);
        le_audio_rpc_send_status_update(BIG_SYNC_LOST);
        WICED_BT_TRACE("[%s] BASE State [%d] \n", __FUNCTION__, p_big->base.state);
    }
    break;

    default:
        break;
    }
}

/******************************************************************************
 * Function Name: broadcast_sink_bis_init
 *******************************************************************************
 * Summary:
 *   Initiate bis
 *
 * Parameters:
 *   wiced_bt_cfg_isoc_t *p_isoc_cfg    : isoc config
 *
 * Return:
 *
 ******************************************************************************/
void broadcast_sink_bis_init(wiced_bt_cfg_isoc_t *p_isoc_cfg)
{
    WICED_BT_TRACE("[%s] \n", __FUNCTION__);

    iso_audio_init(p_isoc_cfg);
    wiced_bt_isoc_register_cb(broadcast_sink_bis_isoc_cb);
    wiced_bt_ble_register_adv_ext_cback(broadcast_sink_ext_adv_cback);
}

/******************************************************************************
 * Function Name: broadcast_sink_bis_alloc_big
 *******************************************************************************
 * Summary:
 *   Allocate big
 *
 * Parameters:
 *   uint32_t broadcast_id              : broadcast ID
 *   wiced_bt_device_address_t bd_addr  : bd address
 *   uint8_t adv_sid                    : Advertising set identifier(SID)
 *
 * Return:
 *  broadcast_sink_cb_t : information structure
 *
 ******************************************************************************/
broadcast_sink_cb_t *broadcast_sink_bis_alloc_big(uint32_t broadcast_id,
                                                  wiced_bt_device_address_t bd_addr,
                                                  uint8_t adv_sid)
{
    broadcast_sink_cb_t *p_big = NULL;

    /* If BIG already exists do not allocate a new one */
    p_big = broadcast_sink_bis_get_big_by_broadcast_id(broadcast_id);
    if (p_big) return p_big;

    /* find a free slot */
    for (size_t i = 0; i < MAX_BIG; i++)
    {
        if (TRUE != g_broadcast_sink_cb[i].in_use)
        {
            p_big = &g_broadcast_sink_cb[i];
            break;
        }
    }

    if (!p_big) return p_big;

    p_big->in_use = TRUE;
    p_big->base.broadcast_id = broadcast_id;
    p_big->adv_handle = adv_sid;
    /* Adv. set ID will be unique per BASE, so it should be ok to use the same as BIG handle */
    p_big->big_handle = adv_sid;
    memcpy(p_big->bd_addr, bd_addr, BD_ADDR_LEN);

    p_big->sync_handle = 0xFF;
    p_big->base.state = BAP_BROADCAST_STATE_IDLE;
    p_big->b_encryption = FALSE;

    WICED_BT_TRACE("[%s] Initializing [State:%d] [br_id:0x%x] [adv_sid:%d] [sync_handle:0xFF]\n",
                   __FUNCTION__,
                   p_big->base.state,
                   p_big->base.broadcast_id,
                   adv_sid);

    return p_big;
}

/******************************************************************************
 * Function Name: broadcast_sink_bis_free_big
 *******************************************************************************
 * Summary:
 *   Free big
 *
 * Parameters:
 *   broadcast_sink_cb_t *p_big : big to be freed
 *
 * Return:
 *
 ******************************************************************************/
void broadcast_sink_bis_free_big(broadcast_sink_cb_t *p_big)
{
    memset(p_big, 0, sizeof(broadcast_sink_cb_t));
}

void broadcast_sink_create_periodic_sync(wiced_bt_ble_scan_results_t *p_ext_adv_report)
{
    /* Queue to hold "sync to periodic adv" requests (since controller rejects more than one request at a time) */
    static wiced_bt_buffer_q_t broadcast_sink_pending_scan_q = {0};
    wiced_bt_ble_scan_results_t *p_cached_scan_result = NULL;
    uint16_t sync_timeout = 0;

    /* If previous sync is pending still add this req to queue else controller will reject the command */
    if (broadcast_sink_periodic_sync_in_progress && p_ext_adv_report)
    {
        wiced_bt_ble_scan_results_t *p_ext_adv_report_cpy =
            (wiced_bt_ble_scan_results_t *)wiced_bt_get_buffer(sizeof(wiced_bt_ble_scan_results_t));

        *p_ext_adv_report_cpy = *p_ext_adv_report;

        /* add the request to queue */
        wiced_bt_enqueue(&broadcast_sink_pending_scan_q, p_ext_adv_report_cpy);
        WICED_BT_TRACE("[%s] Queuing periodic adv sync request\n", __FUNCTION__);
        return;
    }

    if (!p_ext_adv_report)
    {
        /* get pending sync request from queue */
        p_cached_scan_result = (wiced_bt_ble_scan_results_t *)wiced_bt_dequeue(&broadcast_sink_pending_scan_q);
        if (!p_cached_scan_result)
        {
            broadcast_sink_periodic_sync_in_progress = FALSE;
            return;
        }

        p_ext_adv_report = p_cached_scan_result;
    }

    WICED_BT_TRACE("[%s] Trying to sync to periodic adv\n", __FUNCTION__);
    /* The Sync timeout should at least be 6 times the interval to accommodate for 6 opportunities to catch the peer. */
    sync_timeout = (p_ext_adv_report->periodic_adv_interval * 3) / 4;
    wiced_bt_ble_create_sync_to_periodic_adv(WICED_BT_BLE_IGNORE_SYNC_TO_PERIODIC_ADV_LIST,
                                             p_ext_adv_report->adv_sid,
                                             p_ext_adv_report->ble_addr_type,
                                             p_ext_adv_report->remote_bd_addr,
                                             0,
                                             sync_timeout,
                                             0);

    broadcast_sink_periodic_sync_in_progress = TRUE;

    if (p_cached_scan_result) wiced_bt_free_buffer(p_cached_scan_result);
}

void broadcast_sink_bis_ext_adv_scan_cback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
    uint32_t br_id = 0;

    if (!p_scan_result || !p_scan_result->is_extended || p_scan_result->periodic_adv_interval == 0) return;

    if (!wiced_bt_ga_bap_broadcast_is_broadcast_announcement(p_adv_data, &br_id)) return;
    wiced_bt_ga_rcv_public_broadcast p_rcv_br;
    if (wiced_bt_ga_pbp_is_public_broadcast(p_adv_data, &p_rcv_br))
    {
        WICED_BT_TRACE("%s", p_rcv_br.broadcast_name);
        WICED_BT_TRACE("%d", p_rcv_br.audio_config);
        WICED_BT_TRACE("%d", p_rcv_br.encryption);
        WICED_BT_TRACE("%d", p_rcv_br.metadata_length);
        WICED_BT_TRACE("%d", p_rcv_br.source_appearance_value);
    }

    WICED_BT_TRACE("[%s] broadcast id found %x\n", __FUNCTION__, br_id);

    broadcast_sink_rpc_send_new_stream_info(br_id, p_rcv_br.broadcast_name);
}


void broadcast_sink_bis_ext_adv_scan_to_sync_cback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
    uint32_t br_id = 0;
    broadcast_sink_cb_t *p_big = NULL;

    if (!p_scan_result || !p_scan_result->is_extended || p_scan_result->periodic_adv_interval == 0) return;

    if (!wiced_bt_ga_bap_broadcast_is_broadcast_announcement(p_adv_data, &br_id)) return;
    wiced_bt_ga_rcv_public_broadcast p_rcv_br;

    if (!wiced_bt_ga_pbp_is_public_broadcast(p_adv_data, &p_rcv_br))
    {

        WICED_BT_TRACE("%s", p_rcv_br.broadcast_name);
        WICED_BT_TRACE("%d", p_rcv_br.audio_config);
        WICED_BT_TRACE("%d", p_rcv_br.encryption);
        WICED_BT_TRACE("%d", p_rcv_br.metadata_length);
        WICED_BT_TRACE("%d", p_rcv_br.source_appearance_value);
    }
    if (br_id != broadcast_source.broadcast_id)
    {
        WICED_BT_TRACE("[%s] broadcast id found %x looking for %x \n",
                       __FUNCTION__,
                       br_id,
                       broadcast_source.broadcast_id);
        return;
    }

    // check for duplicate reports
    //TODO: Check for addr and br_id ?
    p_big = broadcast_sink_bis_get_big_by_broadcast_id(br_id);
    if (p_big == NULL)
    {
        // Alloc a slot to store BASE, do not sync to PA if unsuccessful
        p_big = broadcast_sink_bis_alloc_big(br_id, p_scan_result->remote_bd_addr, p_scan_result->adv_sid);
    }
    if (!p_big)
        return;

    WICED_BT_TRACE("[%s] sync_handle %d\n", __FUNCTION__, p_big->sync_handle);

    if ( (0xFF == p_big->sync_handle)  && (p_big->sync_in_progress == WICED_FALSE))
    {
        p_big->sync_in_progress = WICED_TRUE;
        broadcast_sink_create_periodic_sync(p_scan_result);
    }
}

/******************************************************************************
 * Function Name: broadcast_sink_bis_menu_ext_adv_scan_cback
 *******************************************************************************
 * Summary:
 *   Callback function for manually adv scan
 *
 * Parameters:
 *   wiced_bt_ble_scan_results_t *p_scan_result : pointer to scan result
 *   uint8_t *p_adv_data                        : pointer to adv data
 *
 * Return:
 *
 ******************************************************************************/
static void broadcast_sink_bis_menu_ext_adv_scan_cback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
    uint32_t br_id = 0;

    if (!p_scan_result || !p_scan_result->is_extended || p_scan_result->periodic_adv_interval == 0)
    {
        return;
    }

    if (!wiced_bt_ga_bap_broadcast_is_broadcast_announcement(p_adv_data, &br_id))
    {
        return;
    }
    wiced_bt_ga_rcv_public_broadcast p_rcv_br;
    if (wiced_bt_ga_pbp_is_public_broadcast(p_adv_data, &p_rcv_br))
    {
        WICED_BT_TRACE("       broadcast ID[%x]         %4d           %4d             %4d              %4d                     %20s", br_id, p_rcv_br.audio_config, p_rcv_br.encryption, p_rcv_br.metadata_length, p_rcv_br.source_appearance_value, p_rcv_br.broadcast_name);
    }
}

/******************************************************************************
 * Function Name: broadcast_sink_bis_menu_discover_sources
 *******************************************************************************
 * Summary:
 *   Manually discover sources
 *
 * Parameters:
 *   bool start : start to discover sources
 *
 * Return:
 *
 ******************************************************************************/
void broadcast_sink_bis_menu_discover_sources(bool start)
{
    if (start)
    {
        broadcast_sink_clear_data();
    }

    wiced_bt_ble_observe(start ? BTM_BLE_SCAN_TYPE_HIGH_DUTY : BTM_BLE_SCAN_TYPE_NONE, 255, broadcast_sink_bis_menu_ext_adv_scan_cback);
}

/******************************************************************************
 * Function Name: broadcast_sink_bis_discover_sources
 *******************************************************************************
 * Summary:
 *   Discover sources
 *
 * Parameters:
 *   wiced_bt_ble_scan_type_t scan_type : scan type
 *
 * Return:
 *
 ******************************************************************************/
void broadcast_sink_bis_discover_sources(wiced_bt_ble_scan_type_t scan_type)
{
    wiced_bt_ble_observe(scan_type, 255, broadcast_sink_bis_ext_adv_scan_cback);
}

/******************************************************************************
 * Function Name: broadcast_sink_bis_sync_to_source
 *******************************************************************************
 * Summary:
 *   Sync to source
 *
 * Parameters:
 *   wiced_bt_ble_scan_type_t scan_type : scan type
 *   broadcast_source_t source          : source
 *
 * Return:
 *
 ******************************************************************************/
void broadcast_sink_bis_sync_to_source(wiced_bt_ble_scan_type_t scan_type, broadcast_source_t source)
{
    memcpy(&broadcast_source, &source, sizeof(broadcast_source_t));
    // Make sure to stop previous scan
    wiced_bt_ble_observe(WICED_FALSE, 0, NULL);
    wiced_bt_ble_observe(scan_type, 255, broadcast_sink_bis_ext_adv_scan_to_sync_cback);
}

/******************************************************************************
 * Function Name: broadcast_sink_clear_data
 *******************************************************************************
 * Summary:
 *   Clear bis data
 *
 * Parameters:
 *
 * Return:
 *
 ******************************************************************************/
void broadcast_sink_clear_data()
{
    WICED_BT_TRACE("[%s] \n", __FUNCTION__);
    for (size_t i = 0; i < MAX_BIG; i++)
    {
        if (g_broadcast_sink_cb[i].in_use == TRUE)
        {
            WICED_BT_TRACE("[%s]terminating sync to  %B big handle %d\n",
                           __FUNCTION__,
                           g_broadcast_sink_cb[i].bd_addr,
                           g_broadcast_sink_cb[i].big_handle);
            wiced_result_t ret = wiced_bt_ble_terminate_sync_to_periodic_adv(g_broadcast_sink_cb[i].sync_handle);
            WICED_BT_TRACE("[%s] terminate res %x \n", __FUNCTION__, ret);
        }
        memset(g_broadcast_sink_cb, 0, sizeof(g_broadcast_sink_cb));
    }

    broadcast_sink_periodic_sync_in_progress = FALSE;
}
