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
* Description: This is the source code for Linux CE LE audio braodcast sink.
*
* Related Document: See README.md
*
*******************************************************************************/


/*******************************************************************************
*                           INCLUDES
*******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include "broadcast_sink_bt_manager.h"
#include "broadcast_sink_rpc.h"
#include "broadcast_sink_bis.h"
#include "utils_arg_parser.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "log.h"
#include "platform_linux.h"
#include "pthread.h"
#include <unistd.h>


/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define LOCAL_BDA_LEN       (6)
#define MAX_PATH            (256)
#define BT_STACK_HEAP_SIZE  (12 * 1024)
#define BIS_START_SCAN      (2)
#define BIS_STOP_SCAN       (3)
#define BIS_SYNC_SOURCE     (4)
#define BIS_STOP_STREAM     (5)
#define BIS_LISTEN          (1)
#define BIS_STOP_LISTEN     (0)


/******************************************************************************
 *                              EXTERNS VARIABLE
 *****************************************************************************/
extern wiced_bt_cfg_settings_t broadcast_sink_cfg_settings;


/******************************************************************************
 *                              EXTERNS FUNCTION
 *****************************************************************************/


/*******************************************************************************
*                               STRUCTURES AND ENUMERATIONS
*******************************************************************************/


/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
pthread_mutex_t console_mutex;
static void (*bt_ready_callback)(void) = NULL;
static uint8_t broadcast_code[16] = {0};
static int spy_inst = 0;
static char local_bda[BD_ADDR_LEN] = {0};
static uint8_t bis_bd_address[LOCAL_BDA_LEN] = {0x11, 0x12, 0x13, 0x21, 0x22, 0x23};
static const char app_menu[] = "\n\
---------------------BIS SNK CONTROL MENU-----------------------\n\n\
    0.  Exit \n\
    1.  Print Menu \n\
    2.  Scan BIS source \n\
    3.  Stop scan \n\
    4.  Sync to source \n\
    5.  Stop sync \n\
Choose option: \n";
static const char enter_braodcast_id[] = "\n\
Please enter broadcast ID (hex): \n";


/******************************************************************************
*                               FUNCTION PROTOTYPE
******************************************************************************/
uint32_t hci_control_proc_rx_cmd(uint8_t *p_buffer, uint32_t length);
void APPLICATION_START(void);

static int get_spy_instance(void);
static void set_local_bd_address(void);


/******************************************************************************
 * Function Name: get_spy_instance
 *******************************************************************************
 * Summary:
 *   Get spy instance
 *
 * Parameters:
 *
 * Return:
 *   int : spy instance
 *
 ******************************************************************************/
static int get_spy_instance(void)
{
    return spy_inst;
}

/******************************************************************************
 * Function Name: set_local_bd_addr
 *******************************************************************************
 * Summary:
 *   Set local bd address
 *
 * Parameters:
 *
 * Return:
 *
 ******************************************************************************/
void set_local_bd_addr(void)
{
    if ((local_bda[0] | local_bda[1] | local_bda[2] | local_bda[3] | local_bda[4] | local_bda[5]) != 0)
        wiced_bt_set_local_bdaddr(local_bda, BLE_ADDR_PUBLIC);
}

/******************************************************************************
 * Function Name: APPLICATION_START
 *******************************************************************************
 * Summary:
 *   Start application 
 *
 * Parameters:
 *
 * Return:
 *
 ******************************************************************************/
void APPLICATION_START(void)
{
    /* RPC to work with LE Audio Client Control */
    broadcast_sink_rpc_init(get_spy_instance());

    /* Register call back and configuration with stack */
    wiced_bt_stack_init(broadcast_sink_btm_cback, &broadcast_sink_cfg_settings);

    /* Create a buffer heap, make it the default heap.  */
    wiced_bt_create_heap("broadcast_sink", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
}

/******************************************************************************
 * Function Name: hci_control_proc_rx_cmd
 *******************************************************************************
 * Summary:
 *   Function to handle HCI receive (This will be called from
 *   porting layer - linux_tcp_server.c)
 *
 * Parameters:
 *   uint8_t* p_buffer    : rx buffer
 *   uint32_t length      : rx buffer length
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
 * Function Name: input_hex
 *******************************************************************************
 * Summary:
 *   Get user input hex number
 *
 * Parameters:
 *   int* p_number : pointer to int buffer
 *
 * Return:
 *   bool : get any error from input or not
 *
 ******************************************************************************/
static bool input_hex(int* p_number)
{
    fflush(stdin);
    if(0 == scanf("%x", p_number))
    {
        fprintf(stdout, "Invalid input hex\n");
        return FALSE;
    }
    fflush(stdin);
    while(getchar() != '\n');
    return TRUE; 
}

/******************************************************************************
 * Function Name: input_chars
 *******************************************************************************
 * Summary:
 *   Get user input characteristics, enter again if input is invalid
 *
 * Parameters:
 *   char* buf  : pointer to cahr buffer
 *   int size   : buffer size
 * 
 * Return:
 *
 ******************************************************************************/
static void input_chars(uint8_t* buf, int size){
    for(int i = 0, c = 0;;)
    {
        fflush(stdin);
        memset(buf, 0, size);
        fprintf(stdout, "Enter broadcast code (16 bytes): ");
        for(; (c = getchar()) != EOF && c != '\n'; i++) 
        {
            if(i < size)
            {
                buf[i] = c;
            }
        }
        if(i <= size)
        {
            return;
        }
        else
        {
            fprintf(stdout, "Invalid input\n");
            i = 0;
        }
    }

}

void unlock_menu(void)
{
    pthread_mutex_unlock(&console_mutex);
}

int main(int argc, char *argv[])
{
    cybt_controller_gpio_config_t autobaud = {0};
    char fw_patch_file[MAX_PATH] = {0};
    char hci_port[MAX_PATH] = {0};
    char peer_ip_addr[16] = "000.000.000.000";
    uint32_t hci_baudrate = 0;
    uint32_t patch_baudrate = 0;   
    int btspy_inst = 0; 
    int number = 0;   
    uint8_t btspy_is_tcp_socket = 0;
    bool skip_menu = FALSE;

    pthread_mutex_init(&console_mutex, NULL);
    broadcast_sink_reg_bt_ready_callback(unlock_menu);

    if (PARSE_ERROR ==
        arg_parser_get_args(argc, argv, hci_port, bis_bd_address, &hci_baudrate,
                            &btspy_inst, peer_ip_addr, &btspy_is_tcp_socket,
                            fw_patch_file, &patch_baudrate, &autobaud))
    {
        return EXIT_FAILURE;
    }

    spy_inst = btspy_inst;
    memcpy(local_bda, bis_bd_address, sizeof(local_bda));

    cy_platform_bluetooth_init(fw_patch_file, hci_port, hci_baudrate,
                               patch_baudrate, &autobaud.autobaud_cfg);

    pthread_mutex_lock(&console_mutex);

    /* Initialize Spy TCP/UDP Sockets */
    cy_bt_spy_comm_init(btspy_is_tcp_socket, btspy_inst, NULL);

    TRACE_LOG("APPLICATION START:\r\n");

    for (;;)
    {
        if(!skip_menu)
        {
            fprintf(stdout, "%s", app_menu);          
        }
        else
        {
            skip_menu = FALSE;
        }       

        if(input_hex(&number) == FALSE)
        {
            continue;
        }
        switch (number)
        {
            case 0:
                exit(EXIT_SUCCESS);
            case 1:
                break;     
            case BIS_START_SCAN:  
                TRACE_LOG("Broadcast ID[ID]  |  Audio Config  |  Encryption  |  Metadata Length  |  Source appearance value  |  Name\n");     
                broadcast_sink_bis_menu_discover_sources(TRUE);
                skip_menu = TRUE;
                break;    
            case BIS_STOP_SCAN:
                broadcast_sink_bis_menu_discover_sources(FALSE);
                break;  
            case BIS_SYNC_SOURCE:
                fprintf(stdout, "%s", enter_braodcast_id);    
                if(input_hex(&number) == FALSE)
                {
                    continue;
                }
                input_chars(broadcast_code, 16);
                broadcast_sink_bis_menu_sync_to_source((uint32_t)number, BIS_LISTEN, broadcast_code);
                break;
            case BIS_STOP_STREAM:
                fprintf(stdout, "%s", enter_braodcast_id);    
                if(input_hex(&number) == FALSE)
                {
                    continue;
                }
                memset(broadcast_code, 0, 16);
                broadcast_sink_bis_menu_sync_to_source((uint32_t)number, BIS_STOP_LISTEN, broadcast_code);            
                break;                
            default:
                fprintf(stdout, "Invalid input received, Try again\n");
                break;
        }
    }
    return 0;    
}
