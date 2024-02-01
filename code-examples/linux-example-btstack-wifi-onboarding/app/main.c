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
 * File Name: main.c
 *
 * Description: This is the main source file for Linux wifi_onboarding CE.
 *
 * Related Document: See README.md
 *
*******************************************************************************/


/******************************************************************************
*       INCLUDES
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "utils_arg_parser.h"
#include "wifi_onboarding.h"
#include "wiced_memory.h"
#include "platform_linux.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_trace.h"

#include <pthread.h>
#include "log.h"
/*******************************************************************************
*       MACROS
*******************************************************************************/
#define MAX_PATH        (256)
#define IP_ADDR_LEN     (16)


/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
cybt_controller_gpio_config_t gpio_cfg = {0};

/*******************************************************************************
*       FUNCTION DECLARATIONS
*******************************************************************************/
uint32_t hci_control_proc_rx_cmd(uint8_t* p_buffer, uint32_t length);
void APPLICATION_START(void);

/*******************************************************************************
*       FUNCTION DEFINITION
*******************************************************************************/

/*******************************************************************************
* Function Name: hci_control_proc_rx_cmd()
********************************************************************************
* Summary:
*   Function to handle HCI receive
*
* Parameters:
*   uint8_t* p_buffer   : rx buffer
*   uint32_t length     : rx buffer length
*
* Return:
*  status code
*
*******************************************************************************/
uint32_t hci_control_proc_rx_cmd(uint8_t* p_buffer, uint32_t length)
{
    return 0;
}

/*******************************************************************************
* Function Name: APPLICATION_START()
********************************************************************************
* Summary:
*   BT stack initialization function wrapper
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void APPLICATION_START(void)
{
    application_start();
}

/*******************************************************************************
* Function Name: main()
********************************************************************************
* Summary:
*   Application entry function
*
* Parameters:
*   int argc            : argument count
*   char *argv[]        : list of arguments
*
* Return:
*   None
*
*******************************************************************************/
int main(int argc, char* argv[])
{
    char fw_patch_file[MAX_PATH] = {0};
    char hci_port[MAX_PATH] = {0};
    char local_bda[BD_ADDR_LEN] = {0};  
    char peer_ip_addr[IP_ADDR_LEN] = "000.000.000.000";
    uint32_t hci_baudrate = 0;
    uint32_t patch_baudrate = 0;
    int btspy_inst = 0;
    uint8_t btspy_is_tcp_socket = 0;
    pthread_t thid = 0;

        /* Audobaud configuration GPIO bank and pin */
    cybt_controller_autobaud_config_t autobaud = {0};

    if ( PARSE_ERROR == arg_parser_get_args( argc,
                                           argv,
                                           hci_port,
                                           local_bda,
                                           &hci_baudrate,
                                           &btspy_inst,
                                           peer_ip_addr,
                                           &btspy_is_tcp_socket,
                                           fw_patch_file,
                                           &patch_baudrate,
                                           &gpio_cfg) )
    {
        return EXIT_FAILURE;
    }

    cy_platform_bluetooth_init( fw_patch_file, hci_port, hci_baudrate,patch_baudrate, &gpio_cfg.autobaud_cfg );

    if (pthread_create(&thid, NULL, wifi_tracker, NULL) != 0) {
        TRACE_ERR("pthread_create() error");
        return EXIT_SUCCESS;
    }

    TRACE_LOG("Linux CE WiFi-Onboarding project initialization complete....\n");

    for(;;);

    return EXIT_SUCCESS;
}
