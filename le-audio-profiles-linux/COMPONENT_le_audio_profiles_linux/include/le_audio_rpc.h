/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __LE_AUDIO_RPC_H__
#define __LE_AUDIO_RPC_H__

#include "wiced_data_types.h"
#include "hci_control_api.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "gatt_interface.h"
#include "wiced_bt_ga_aics.h"

typedef wiced_bool_t (*le_audio_rpc_cback_t)(uint16_t opcode, uint8_t *p_data, uint32_t data_len);
void le_audio_rpc_init(int host_instance, le_audio_rpc_cback_t le_audio_rpc_cback, wiced_bool_t b_route_traces_to_CC);

typedef void (*route_data_to_client_control_t)(uint8_t type, uint8_t *buffer, uint16_t length, uint8_t spy_instance);

void le_audio_rpc_send_data(int type, uint8_t *p_data, uint16_t data_size);

void le_audio_rpc_send_misc_event(uint32_t chip, uint8_t group);
void le_audio_rpc_send_dev_role(uint32_t dev_role);

void le_audio_rpc_send_broadcast_status_update(uint32_t status);

void le_audio_rpc_send_vcs_state_update(uint16_t conn_id,
                                        uint8_t volume_setting,
                                        uint8_t mute_state,
                                        uint8_t which_vcs_data);
void le_audio_rpc_update_call_state(uint16_t conn_id,
                                    uint8_t call_id,
                                    wiced_bt_ga_string_t *p_call_URI,
                                    uint8_t call_state);
void le_audio_rpc_send_call_terminated_event(uint16_t conn_id, uint8_t call_id, uint8_t termination_reason);

void le_audio_rpc_send_micp_aics_description(uint16_t conn_id, uint32_t instance, gatt_intf_string_t *p_desc);
void le_audio_rpc_send_micp_mute_state(uint16_t conn_id, uint8_t mute_state);
void le_audio_rpc_send_micp_aics_input_state(uint16_t conn_id,
                                             uint32_t instance,
                                             wiced_bt_ga_aics_input_state_t *p_input_state);
void le_audio_rpc_send_preset_record(uint16_t conn_id, uint8_t preset_index, wiced_bt_ga_string_t *p_name);
void le_audio_rpc_update_active_preset(uint16_t conn_id, uint8_t preset_index);

void le_audio_rpc_send_mcs_state_update(uint16_t conn_id, uint8_t state);
void le_audio_rpc_send_mic_state_update(uint16_t conn_id, uint8_t state);
void le_audio_rpc_send_convo_stream_state_update(uint16_t conn_id, uint8_t state);
void le_audio_rpc_send_connect_event(wiced_bt_gatt_connection_status_t *p_status);
void le_audio_rpc_send_disconnect_evt(wiced_bt_gatt_connection_status_t *p_status);
void le_audio_rpc_send_scan_res_event(wiced_ble_ext_scan_results_t *p_scr, uint8_t *p_adv_data);
void le_audio_rpc_send_app_status(uint16_t conn_id, uint32_t init_state, uint32_t sub_state);
void le_audio_rpc_send_app_sub_status(uint16_t conn_id, uint32_t init_state, uint32_t sub_state, const wiced_bt_uuid_t *p_uuid);

void le_audio_rpc_send_app_operation_status(uint16_t conn_id,
                                            gatt_intf_service_object_t *p_service,
                                            gatt_intf_operation_t operation);

void hci_control_le_send_advertisement_state_event(uint8_t state);
void le_audio_rpc_send_advertisement_state(uint32_t state);

uint16_t nvram_memory_read(uint16_t id, uint16_t buf_len, uint8_t *p_buf);

uint16_t nvram_memory_write(uint16_t id, uint16_t data_len, const uint8_t *p_data);

#endif
