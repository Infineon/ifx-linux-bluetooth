/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __LEHS_BIS_H__
#define __LEHS_BIS_H__


typedef struct
{
    // app info
    wiced_bool_t in_use;                     // True if it has been allocated for any BIG
    uint8_t big_handle;                      // big handle of BIG
    uint8_t adv_handle;                      // adv handle of BIG
    wiced_bt_device_address_t bd_addr;       // device address of broadcast source
    wiced_bool_t b_base_updated;             // True if received periodic adv data and updated base information correctly
    wiced_bool_t b_biginfo_updated;          // True if received big_info(information about BIG) adv
    uint8_t sync_state;                      // Synchronization state

    // controller info
    wiced_ble_padv_sync_handle_t sync_handle;                                              // Sync handle of periodic adv data
    wiced_bool_t big_sync_in_progress;                                                     // True if synchronization is in progress
    uint8_t number_of_subevents;                                                           // for sink only (received in BIGInfo)
    uint8_t bis_conn_id_count;                                                             // Total BISes present in BIG (Sum of number of bis in all subgroups)
    uint16_t bis_conn_id_list[BROADCAST_MAX_BIS_PER_SUB_GROUP * BROADCAST_MAX_SUB_GROUP];  // Stores conn_hdl of all BISes
    uint8_t bis_index_list[BROADCAST_MAX_BIS_PER_SUB_GROUP * BROADCAST_MAX_SUB_GROUP];     // Stores bis index of all BISes
    wiced_bool_t b_encryption;                                                             // True for encrypted broadcast audio stream
    wiced_bt_bap_broadcast_code_t broadcast_code;                                          // Stores Broadcast code if encrypted streaming

    // profile info
    le_audio_bap_broadcast_base_t base;    // Stores base information
} broadcast_sink_cb_t;

typedef struct
{
    uint32_t broadcast_id;
    wiced_bt_bap_broadcast_code_t broadcast_code;
    uint32_t bis_index_bits;
} broadcast_source_t;

void lehs_bis_init(void);
void lehs_bis_discover_sources(uint8_t start);
void broadcast_sink_clear_data();
void lehs_sync_to_pa(uint32_t broadcast_id);
wiced_bool_t lehs_broadcast_get_synk_progress(void);
void lehs_bis_sync_to_source(broadcast_source_t source);
void lehs_bis_terminate_sync(uint32_t broadcast_id);

broadcast_sink_cb_t *lehs_bis_get_big_by_broadcast_id(uint32_t br_id);

broadcast_sink_cb_t *lehs_bis_alloc_big(uint32_t broadcast_id,
                                                  wiced_bt_device_address_t bd_addr,
                                                  uint8_t adv_sid);

void lehs_bis_free_big(broadcast_sink_cb_t *p_big);

broadcast_sink_cb_t *lehs_bis_get_big_by_sync_handle(wiced_ble_padv_sync_handle_t sync_handle);
void lehs_sync_to_source(broadcast_sink_cb_t *p_big, uint32_t bis_index_bits);

#endif
