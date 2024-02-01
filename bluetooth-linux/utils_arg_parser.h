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
 * File Name: arg_parser.h
 *
 * Description: This is the header file for argument parsing module.
 *
 ******************************************************************************/

#ifndef __ARG_PARSER_H__
#define __ARG_PARSER_H__

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include <stdint.h>
#include "platform_linux.h"


/*******************************************************************************
*                           MACROS
*******************************************************************************/
#define PARSE_ERROR         ( -1 )
#define PARSE_SUCCESS       ( 0 )

#define DEFAULT_BAUDRATE         115200
#define BAUDRATE_HCI_UART       3000000
#define BAUDRATE_PATCH_DOWNLOAD  921600
/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/

/******************************************************************************
 * Function Name: AsciiToHex()
 ******************************************************************************
 * Summary:
 *   Function convert bdaddr ASCII to Hex
 *
 * Parameters:
 *   uint8_t *bda           : converted Hex char array 
 *
 *   size_t len             : size of bda
 *
 *   char *pBdAddr           : input hex char array
 *
 * Return:
 *  Converted digit value
 *
 *****************************************************************************/
void AsciiToHex(uint8_t* bda, size_t len, char* pBdAddr);

/******************************************************************************
 * Function Name: HexDigits()
 ******************************************************************************
 * Summary:
 *   Function convert Hex to Digit.
 *
 * Parameters:
 *   uint8_t *p             : Hex Number
 *
 * Return:
 *  Converted digit value
 *
 *****************************************************************************/
uint8_t HexDigits( char *p );


/*******************************************************************************
**
** Function         arg_parser_get_args
**
** Description      This function will parse the linux CE user input argments for
*                   setting baudrate setting / patch downloading 
** Input Parameters:
**     argc: main function's argc.
**
**     argv: main function's argv.
**
**     device: HCI Uart interface. ex: /dev/ttyUSB0
**
**     local_bda: bluetooth device address. ex: 112233445566 12 digits
**
**     baud: HCI Uart BaudRate.
**
**     spy_inst: BTSPY instance.
**     
**     peer_ip_addr: BTSPY communication peer ip address.
**
**     is_socket_tcp: BTSPY communication socket.
**
**     patchFile: Chip patch firmware file name & location.
**
**     patch_baud: Chip patch firmware download BaudRate.
**
**     p_gpio_cfg: For GPIO control: ex: hatchet2 autobaud mode should trigger RegON GPIO
**
** Returns          TRUE if success otherwise FALSE.
**
*******************************************************************************/
int arg_parser_get_args( int argc,
                        char **argv,
                        char *device,
                        uint8_t *local_bda,
                        uint32_t *baud,
                        int *spy_inst,
                        char *peer_ip_addr,
                        uint8_t *is_socket_tcp,
                        char* patchFile,
                        uint32_t *patch_baud,
                        cybt_controller_gpio_config_t *p_gpio_cfg);

uint32_t checkAppBaudRate(uint32_t uBaudRate);

uint32_t checkPatchBaudRate(uint32_t uBaudRate);


#endif /*__ARG_PARSER_H__ */

/* [] END OF FILE */
