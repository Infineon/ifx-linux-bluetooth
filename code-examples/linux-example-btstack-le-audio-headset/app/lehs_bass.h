/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __LEHS_BASS_H__
#define __LEHS_BASS_H__


typedef struct
{
    wiced_bool_t is_used;
    uint16_t conn_id;
    wiced_bool_t waiting_broadcast_code;
    uint16_t sync_handle;
    wiced_bt_ga_bass_receive_state_t recv_state;
    wiced_bt_ga_bass_sub_group_data_t sub_group[WICED_BT_GA_BASS_MAX_SUBGROUP_COUNT]; /**< Subgroup Data */
} lehs_bass_data_t;

typedef struct
{
    lehs_bass_data_t bass_data[MAX_BASS];
    wiced_bt_ga_bass_operation_t op_data;
} lehs_bass_t;

wiced_result_t lehs_bass_callback(uint16_t conn_id,
                                            void *p_app_ctx,
                                            gatt_intf_service_object_t *p_service,
                                            wiced_bt_gatt_status_t status,
                                            uint32_t evt_type,
                                            gatt_intf_attribute_t *p_char,
                                            void *p_data,
                                            int len);

lehs_bass_data_t *lehs_bass_find_source_by_sync_handle(wiced_ble_padv_sync_handle_t sync_handle);
void lehs_bass_notify_pa_sync_state(wiced_ble_padv_sync_established_event_data_t *p_sync);
void lehs_bass_broadcast_code_check(uint16_t sync_handle);
void lehs_bass_notify_sync_established(uint8_t *p_addr);
void lehs_bass_notify_big_sync_lost(uint8_t *p_addr);
void lehs_bass_notify_pa_sync_lost(uint16_t sync_handle);
lehs_bass_t *lehs_bass_get_bass_data();

#endif
