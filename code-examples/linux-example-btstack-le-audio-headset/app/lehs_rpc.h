/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __LEHS_RPC_H__
#define __LEHS_RPC_H__


#define MAX_PLAYER_NAME_LEN 25

struct player
{
    char player_name[MAX_PLAYER_NAME_LEN];
    uint8_t len;
};

typedef struct player lehs_player_t;

wiced_bool_t lehs_rpc_rx_callback(uint16_t opcode, uint8_t *p_data, uint32_t payload_len);

void lehs_rpc_send_play_status(uint16_t conn_id, uint8_t play_status);
void lehs_rpc_send_device_role_event(uint8_t dev_role);
void lehs_rpc_send_get_players_event(uint16_t conn_id, lehs_player_t *players, int num);
wiced_bool_t lehs_rpc_is_dev_role_sink(void);
void lehs_rpc_send_new_stream_info(uint32_t broadcast_id, const uint8_t *br_name);

void lehs_terminate_incoming_call(uint16_t conn_id,
                                  uint8_t call_id,
                                  uint8_t reason,
                                  gatt_intf_service_object_t *p_service,
                                  gatt_intf_attribute_t *p_char);

void lehs_handle_call_control_point_action(uint16_t conn_id,
                                           uint8_t call_id,
                                           gatt_intf_service_object_t *p_service,
                                           gatt_intf_attribute_t *p_char,
                                           uint8_t opcode);
#endif
