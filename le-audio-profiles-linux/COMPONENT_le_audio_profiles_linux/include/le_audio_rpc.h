/*
 * $ Copyright Cypress Semiconductor $
 */

#pragma once

#include "wiced_data_types.h"

#include "hci_control_api.h"
#include "wiced_bt_ble.h"

typedef enum
{
    TEST_MSG = 0, // not used
    PA_SYNC_ESTABLISHED = 1,
    PA_SYNC_LOST = 2,
    BIG_SYNC_ESTABLISHED = 3,
    BIG_SYNC_LOST = 4,
    STATUS_ANY_EVT = 20
} le_audio_rpc_sts_t;

typedef enum
{
    MUTE_STATUS = 0,
    VOLUME_STATUS = 1,
    MUTE_AND_VOLUME_STATUS = 2,
} le_audio_rpc_vcs_sts_t;

typedef wiced_bool_t (*le_audio_rpc_cback_t)(uint16_t opcode, uint8_t *p_data, uint32_t data_len);
void le_audio_rpc_init(int host_instance, le_audio_rpc_cback_t le_audio_rpc_cback, wiced_bool_t b_route_traces_to_CC);

typedef void (*route_data_to_client_control_t)(uint8_t type, uint8_t *buffer, uint16_t length, uint8_t spy_instance);

void le_audio_rpc_send_data(int type, uint8_t *p_data, uint16_t data_size);

void le_audio_rpc_send_dev_role(uint8_t dev_role);

void le_audio_rpc_send_status_update(le_audio_rpc_sts_t msg);

void le_audio_rpc_send_vcs_state_update(uint16_t conn_id,
                                        uint8_t volume_setting,
                                        uint8_t mute_state,
                                        le_audio_rpc_vcs_sts_t which_vcs_data);

void le_audio_rpc_send_mcs_state_update(uint16_t conn_id, uint8_t operation_status, uint8_t state);
void le_audio_rpc_send_connect_event(uint8_t addr_type, uint8_t *addr, uint16_t con_handle, wiced_bool_t role);
void le_audio_rpc_send_disconnect_evt(uint16_t reason, uint16_t conn_idx);
void le_audio_rpc_send_scan_res_event(uint8_t *addr, uint8_t addr_type, wiced_bt_dev_ble_evt_type_t evt_type);
