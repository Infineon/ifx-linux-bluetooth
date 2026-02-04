/******************************************************************************
* $ Copyright 2022-YEAR Cypress Semiconductor $
******************************************************************************/
/******************************************************************************
 * File Name: hello_sensor_gatt_cfg.h
 *
 * Description: Hello Sensor GATT DB CFG.
 *
 * Related Document: See README.md
 *
******************************************************************************/

#ifndef _HELLO_SENSOR_GATT_CFG_H_
#define _HELLO_SENSOR_GATT_CFG_H_

extern const uint8_t hello_sensor_gatt_database[];
extern uint16_t hello_sensor_gatt_database_size;

extern attribute_t gauAttributes[];
extern uint16_t gauAttributes_size;

extern host_info_t hello_sensor_hostinfo;
extern hello_sensor_state_t hello_sensor_state;

extern uint8_t hello_sensor_device_name[]          ;  /* GAP Service characteristic Device Name */
extern uint8_t hello_sensor_appearance_name[2]     ;
extern char    hello_sensor_char_notify_value[]    ; /* Notification Name */ 
extern char    hello_sensor_char_mfr_name_value[]  ;
extern char    hello_sensor_char_model_num_value[] ;
extern uint8_t hello_sensor_char_system_id_value[] ;
extern wiced_bt_device_address_t bt_device_address ;

extern uint8_t hello_sensor_char_notify_value_len;

#endif
