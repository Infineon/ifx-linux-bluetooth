/*
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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
* File Name: main_linux.c
*
* Description: This is the source code for Linux CE LE Audio Player.
*
* Related Document: See README.md
*
*******************************************************************************/

#ifdef LINUX_PLATFORM
/*******************************************************************************
*                           INCLUDES
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "wiced_bt_ble.h"
#include "wiced_bt_types.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "platform_linux.h"
#include "wiced_bt_cfg.h"
#include "utils_arg_parser.h"
#include "app_bt_utils.h"
#include "lepl.h"
#include "log.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define DATA_BUFFER_SIZE                ( 20u )

/******************************************************************************
 *                              EXTERNS
 *****************************************************************************/
extern wiced_bt_device_address_t bt_device_address;
int spy_inst = 0;

/*******************************************************************************
*                               STRUCTURES AND ENUMERATIONS
*******************************************************************************/

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
static uint8_t local_bda[BD_ADDR_LEN] = {0};
uint16_t        curr_conn_id            = 0;
int8_t          curr_handle             = 0;
static char     g_app_name[MAX_PATH]    = {0};      /* Application name */

/****************************************************************************
 *                              FUNCTION DECLARATION
 ***************************************************************************/
extern void lepl_isoc_dhm_reinit_voice_assist(void);
static BOOL32 check_curr_conn_id( void );
extern void audio_driver_mic_test(uint8_t mic_channel);

/****************************************************************************
 *                              FUNCTION DEFINITIONS
 ***************************************************************************/

/******************************************************************************
 * Function Name: hci_control_proc_rx_cmd()
 *******************************************************************************
 * Summary:
 *   Function to handle HCI receive
 *
 * Parameters:
 *   uint8_t* p_buffer  : rx buffer
 *  uint32_t length     : rx buffer length
 *
 * Return:
 *  status code
 *
 ******************************************************************************/
uint32_t hci_control_proc_rx_cmd(uint8_t *p_buffer, uint32_t length)
{
    return 0;
}

/******************************************************************************
* Function Name: main()
*******************************************************************************
* Summary:
*   Application entry function
*
* Parameters:
*   int argc            : argument count
*   char *argv[]        : list of arguments
*
* Return:
*     int : main function exit success 
*
******************************************************************************/
int main( int argc, char* argv[] )
{
    int         ret                     = 0;
    int         input                   = 0;
    char        inputstr[3]             = {0};
    BOOL32      exit                    = WICED_FALSE;
    uint8_t     p_data[DATA_BUFFER_SIZE]= {0};
    uint8_t     handle                  = 0;
    int         abs_vol                 = 0;
    uint8_t     *p                      = p_data;
    wiced_result_t status = WICED_BT_SUCCESS;
    
    arg_parser_arguments_t parsed_args = { 0 };

    if (-1 == arg_parser_get_args(argc, argv, &parsed_args))
    {
        return -1;
    }

    spy_inst = parsed_args.spy_inst;

    memcpy(bt_device_address, parsed_args.local_bda, BD_ADDR_LEN);

    
    /* Initialize Spy TCP/UDP Sockets */
    cy_bt_spy_comm_init(parsed_args.is_socket_tcp, parsed_args.spy_inst, NULL);

    cy_platform_bluetooth_init( parsed_args.patchFile, parsed_args.com_port, parsed_args.baud_rate, parsed_args.patch_baud, &parsed_args.p_gpio_cfg.autobaud_cfg);
    if (parsed_args.p_gpio_cfg.autobaud_cfg.bt_reg_on_off.p_gpiochip[0])
    {
        TRACE_LOG("Do autobaud, pull BT-Regon:%s", parsed_args.p_gpio_cfg.autobaud_cfg.bt_reg_on_off.p_gpiochip); 
    }
    else
    {
        TRACE_LOG("No pull BT-Regon");
    }
    WICED_BT_TRACE("[%s] COM [%s] Baud [%d] Instance [%d]\n", __FUNCTION__, parsed_args.com_port, parsed_args.baud_rate, parsed_args.spy_inst);


    /* Extract the application name */
    memset( g_app_name, 0, sizeof( g_app_name ) );
    strncpy(g_app_name, argv[0], MAX_PATH - 1);


    if (parsed_args.patchFile[0])
    {
        TRACE_LOG ("Waiting for downloading patch...");

        wait_controller_reset_ready();

    } else {
        TRACE_LOG ("No patch FW");
    }
     
    TRACE_LOG(" Linux CE LE-Audio Unicast Sink initialization complete...\n" );

    for (;;)
    {
#ifdef CLI_SUPPORT
         extern int le_pl_cli_routine(void);      
         if (le_pl_cli_routine() == 0)
         {
             break;
         }
#endif // CLI_SUPPORT
    }
 
    return EXIT_SUCCESS;
}

/******************************************************************************
 * Function Name: check_curr_conn_id
 *******************************************************************************
 * Summary: check global variabl curr_conn_id, if 0 means no select conn_id
 *
 * Parameters:
 *  None
 *
 * Return:
 *  BOOL32
 *
 ******************************************************************************/
static BOOL32 check_curr_conn_id( void )
{
    if (curr_conn_id == 0)
    {
        TRACE_ERR("Select Connected Source First or Not Connect to Source Device\n");
        return WICED_FALSE;
    }
    return WICED_TRUE;
}
#endif
