/******************************************************************************
 * (c) 2023, Infineon Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Infineon Semiconductor Corporation or one of its
 * subsidiaries ("Infineon") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Infineon hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Infineon's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Infineon.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Infineon
 * reserves the right to make changes to the Software without notice. Infineon
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Infineon does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Infineon product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Infineon's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Infineon against all liability.
 *****************************************************************************/


/******************************************************************************
 * File Name: spp.h
 *
 * Description: This is the include file for Linux SPP CE.
 *
 * Related Document: See README.md
 *
 *****************************************************************************/
#ifndef __APP_SPP_H__
#define __APP_SPP_H__


/*******************************************************************************
*                               INCLUDES
*******************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include "wiced_bt_cfg.h"
#include "wiced_bt_trace.h"


/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define HDLR_SPP                                (0x10001)
#define SPP_RFCOMM_SCN                          (2)
#define SPP_MAX_PAYLOAD                         (1007)
#define LOCAL_BDA_LEN                           (6)
#define SUPPORT_PORT_NUM                        (6)


/*******************************************************************************
*                               VARIABLE DEFINITIONS
*******************************************************************************/
extern uint8_t spp_bd_address[];


/*******************************************************************************
*                               FUNCTION DECLARATIONS
*******************************************************************************/
void spp_bt_send_data(uint8_t server_idx, uint8_t* p_data, uint32_t length);
wiced_bool_t spp_init(uint8_t num_ports);
uint32_t spp_get_rx_bytes(uint8_t server_idx);
uint16_t spp_port_handle(uint8_t server_idx);
uint8_t* spp_port_tx_buffer(uint8_t server_idx);
uint8_t spp_get_current_port_num(void);
bool spp_enable_terminal(bool value);
void spp_application_start(void);
void spp_send_sample_data(uint8_t server_idx);
void spp_mode_loopback_switch(void);
void spp_bt_enabled_lock(void);
void spp_rx_log_switch(void);
void spp_server_status_string(char* buffer, uint8_t len);
void spp_start_throughput(void);
void spp_end_throughput(struct timespec* user_start_time, struct timespec* user_end_time);
void spp_send_serial_data(uint8_t server_idx, uint8_t* p_data, uint32_t length);
void* spp_close_terminal(void* idx);

#endif /* __APP_SPP_H__ */