/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company)
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Bluetooth WICED Utility functions
 *
 */

#ifndef __WICED_BT_UTILS_H
#define __WICED_BT_UTILS_H

#include "wiced_bt_types.h"
#include "wiced_bt_dev.h"

/*******************************************************************
** WICED BT related definitions and declarations
*******************************************************************/
extern void GKI_freebuf (void *memPtr);
typedef wiced_bt_device_address_t BD_ADDR;
extern BD_ADDR bd_addr_any;
extern BD_ADDR bd_addr_null;

void utl_freebuf(void **p);
void utl_bdcpy(BD_ADDR a, const BD_ADDR b);
int utl_bdcmp(const BD_ADDR a, const BD_ADDR b);
int utl_bdcmpany(const BD_ADDR a);
void utl_bdsetany(BD_ADDR a);



/*******************************************************************
** string and character related definitions and declarations
*******************************************************************/
int utl_strncmp(const char *s1, const char *s2, int n);
char *utl_strcat(char *s1, const char *s2);
char *utl_strcpy(char *p_dst, const char *p_src);
char *utl_strrchr(char *s, int c);
int16_t utl_str2int(const char *p_s);
int utl_strucmp(const char *p_s, const char *p_t);
unsigned long utl_strtoul(const char *nptr, char **endptr, int base);
char utl_toupper(char c);
uint8_t utl_itoa(uint16_t i, char *p_s);
void utl_ignore_spaces(char* str);



/*******************************************************************
 * Function         wiced_bt_read_raw_rssi
 *
 *                  returns the raw or actual RSSI
 *
 * @param[in]       connection_handle   : peer connection handle
 * @param[in]       p_callback_in       : application callback to receive RSSI
 *
 * @return          void
*******************************************************************/
typedef void (wiced_bt_read_raw_rssi_command_complete_cback_t) (wiced_bt_dev_vendor_specific_command_complete_params_t *p_command_complete_params);
wiced_bt_dev_status_t wiced_bt_read_raw_rssi(uint16_t connection_handle, wiced_bt_read_raw_rssi_command_complete_cback_t *p_callback_in);

/*******************************************************************
 * Function         wiced_hal_get_pseudo_rand_number_array
 *
 *                  Fills a given array with pseudo random 32-bit integers.
 *                  Uses the function rbg_get_psrng() from ROM.
 *
 * @param[in]       randNumberArrayPtr  : Pointer to an array to be populated with
 *                                        the random numbers.
 * @param[in]       length              : Length of the array pointed to by
 *                                        randNumberArrayPtr.
 *
 * @return          void
*******************************************************************/
void wiced_hal_get_pseudo_rand_number_array(uint32_t* randNumberArrayPtr, uint32_t length);

/*******************************************************************
 * Function         wiced_transport_send_hci_trace
 *
 * Send the hci trace data over the transport.
 * Note: This API is provided for BTSTACK backward compatibility
 *
 * @param[in]   hci_trans_pool  :Pass the pointer to the pool created by the application
 *                               incase application  has created a dedicated trans pool for
 *                               communicating to host. Pass NULL if the application wants the stack to
 *                               take care of allocating the buffer for sending the data to host.
 *                               Application should be able to use transport buffer pool that it allocates and trace the whole HCI packets.
 *                               In case of stack allocation, the size of trace compromised according to buffer availability.
 *
 * @param[in]   type            :HCI trace type
 * @param[in]   p_data          :Pointer to the data payload
 * @param[in]   length          :Data payload length
 *
 * @return  wiced_result_t      WICED_SUCCESS, if success,
 *                              WICED_NO_MEMORY if buffers not available to send,
 *                              WICED_ERROR otherwise
 ******************************************************************/
#if BTSTACK_VER >= 0x03000001
 #define wiced_bt_transport_send_hci_trace(hci_trans_pool, type, p_data, length )   wiced_transport_send_hci_trace(type, p_data, length)
#else
 #define wiced_bt_transport_send_hci_trace(hci_trans_pool, type, p_data, length )   wiced_transport_send_hci_trace(NULL, type, p_data, length)
#endif


#if defined (CYW20706A2) || defined (CYW43012C0)
// strchr defined in ROM for most chips, define here for the missing ones
#define strchr(x,y) utl_strchr(x,y)
char *utl_strchr(const char *s, int c);
#endif
#endif // __WICED_BT_UTILS_H
