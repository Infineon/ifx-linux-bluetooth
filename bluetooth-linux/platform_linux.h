/*
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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

/******************************************************************************
 * File Name: platform_linux.h
 *
 * Description: This is the header file for platform_linux.
 *
 ******************************************************************************/

#ifndef PLTFRM_LINUX_H
#define PLTFRM_LINUX_H

#include "wiced_bt_types.h"
#include "data_types.h"

#define DEFAULT_PATCH_BAUD_RATE 115200
#define DEFAULT_APP_BAUD_RATE 115200

#define BLUETOOTH_LINUX_VERSION_MAJOR	"0"
#define BLUETOOTH_LINUX_VERSION_MINOR 	"1"
#define BLUETOOTH_LINUX_VERSION_PATCH 	"1"
#define BLUETOOTH_LINUX_VER  (BLUETOOTH_LINUX_VERSION_MAJOR "." BLUETOOTH_LINUX_VERSION_MINOR "."  BLUETOOTH_LINUX_VERSION_PATCH)

/** GPIO information */

#define CYHAL_GPIO_BUFFER_SIZE      100
#define CYHAL_GPIO_FIXED_STR_SIZE   41
#define CYHAL_GPIOCHIP_NAME_SIZE    30
#define CYHAL_GPIO_LINENUM_SIZE     10

typedef void (*gpio_event_cb_t)();

typedef BOOL32 (*pf_le_local_support_func_t)(uint8_t *, uint16_t);

typedef struct
{
    char p_gpiochip[CYHAL_GPIOCHIP_NAME_SIZE];  /**< gpio chip name i.e. gpiochip0 */
    char line_num[CYHAL_GPIO_LINENUM_SIZE];     /**< line number in gpio bank */
}cyhal_gpio_t;

/** GPIO configuration to put the device in autobaud mode */
typedef struct
{
    cyhal_gpio_t                    bt_reg_on_off;      /**< BT reg_on_off pin */
    uint8_t                         use_ioctl;          /* To use ioctl */
}cybt_controller_autobaud_config_t;

/*
*   GPIO EVENT and EVENT CALLBACK
*/
typedef struct gpio_event_args
{
    cyhal_gpio_t 	gpio_event;       /* which gpio we want monitor */
    gpio_event_cb_t 	gpio_event_cb;    /* the callback function when gpio event occure  */
    uint32_t		gpio_event_flag;  /* what event we want, there are 3 types: GPIOEVENT_REQUEST_RISING_EDGE, GPIOEVENT_REQUEST_FALLING_EDGE, GPIOEVENT_REQUEST_BOTH_EDGES   */
}cybt_gpio_event_t;

/*
*   FOR WAKE ON BLE USE
*   DEV-WAKE:  host to Controller
*   HOST-WAKE: controller event for Host
*/
typedef struct gpio_wake_on_ble
{
    cyhal_gpio_t 	dev_wake;
    cybt_gpio_event_t 	host_wake_args;
}cybt_gpio_wake_on_ble_t;

/*
*   PLATFORM GPIO CONFIG
*/
typedef struct
{
    cybt_controller_autobaud_config_t autobaud_cfg;
    cybt_gpio_wake_on_ble_t wake_on_ble_cfg;
}cybt_controller_gpio_config_t;


/*******************************************************************************
**
** Function         cy_platform_bluetooth_init
**
** Description      This function will initialize and configures the UART port by which Bluetooth
**                  which Bluetooth stack and app can communicate with Bluetooth controller.
** Input Parameters:
**     patchFile: Handle of this connection.
**
**     device: Transaction label.
**
**     baudrate: Uart device Baudrate
**
**     fw_download_baudrate: Firmware download BaudRate
**
**
** Returns          TRUE if success otherwise FALSE.
**
*******************************************************************************/
wiced_bool_t cy_platform_bluetooth_init(char *patchFile,
                                        char *device,
                                        uint32_t baudrate,
                                        uint32_t fw_download_baudrate, const cybt_controller_autobaud_config_t *pconfig);

void cy_bt_spy_comm_init(wiced_bool_t b_tcp, int emu_instance, char *peer_ip_addr);

void configure_spy(int emu_instance, char* peer_ip_addr);
void configure_spy_tcp(char *peer_ip_addr);
void wiced_bt_platform_interface_init(void);

void wait_controller_reset_ready(void);

BOOL32 platform_gpio_write(const char *dev_name, const char* offset, uint8_t value, char *str);
BOOL32 platform_gpio_poll(cybt_gpio_event_t *args);
void linux_stack_lock(void *);
void linux_stack_unlock(void *);

void wiced_exp_set_local_support_cback(pf_le_local_support_func_t p_func);

#endif //PLTFRM_LINUX_H
