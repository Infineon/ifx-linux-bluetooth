/******************************************************************************
 * File Name: app_bt_utils.h 
 *
 * Description: Header file for the utility functions that will help
 *          debugging and developing the applications easier with much more
 *          meaningful information.
 *
 * Related Document: See README.md
 *
 ******************************************************************************
* $ Copyright 2022-YEAR Cypress Semiconductor $
 *****************************************************************************/
#ifndef __APP_BT_UTILS_H__
#define __APP_BT_UTILS_H__

/******************************************************************************
 *      INCLUDES
 ******************************************************************************/
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include <stdio.h>

/******************************************************************************
 *      MACROS
 ******************************************************************************/
#define CASE_RETURN_STR(const)          case const: return #const;

#define FROM_BIT16_TO_8(val)            ((uint8_t)(((val) >> 8 )& 0xff))

/****************************************************************************
 *      FUNCTION DECLARATIONS
 ***************************************************************************/
void print_bd_address(wiced_bt_device_address_t bdadr);

void print_array(void * to_print, uint16_t len);

const char *get_bt_event_name(wiced_bt_management_evt_t event);

const char *get_bt_advert_mode_name(wiced_bt_ble_advert_mode_t mode);

const char *get_bt_gatt_disconn_reason_name(wiced_bt_gatt_disconn_reason_t reason);

const char *get_bt_gatt_status_name(wiced_bt_gatt_status_t status);

const char *get_bt_smp_status_name(wiced_bt_smp_status_t status);

const char *get_bt_avrc_event_name(wiced_bt_management_evt_t event);

const char *get_bt_a2dp_src_event_name(uint8_t event);

const char *get_bt_le_gatt_event_name(uint8_t event);

const char *get_bt_gatt_optype_name(uint8_t op);

void device_name_add_bdaddr(uint8_t **p);

#endif /*__APP_BT_UTILS_H__ */
