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
 * File Name: main.c
 *
 * Description: This is the source code for Linux SPP code example.
 *
 * Related Document: See README.md
 *
 *******************************************************************************/


/*******************************************************************************
*                               INCLUDES
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>
#include <time.h>
#include "wiced_bt_trace.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_types.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_spp.h"
#include "platform_linux.h"
#include "utils_arg_parser.h"
#include "log.h"
#include "spp.h"


/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define MAX_PATH                    (256)
#define LOCAL_IP_LEN                (16)
#define EXIT                        (0)

#define MENU_PRINT_MENU             (1)
#define MENU_SEND_SAMPLE_DATA       (2)
#define MENU_TYPE_AND_SEND_DATA     (3)
#define MENU_LOOPBACK_SWITCH        (4)
#define MENU_RX_LOG_SWITCH          (5)
#define MENU_THROUGHPUT_START_TIME  (6)
#define MENU_THROUGHPUT_END_TIME    (7)
#define MENU_BUFFER_LENGTH          (1000)

#define SCAN_ERROR                  (0)
#define SUPPORT_PORTS_MIN           (1)
#define SUPPORT_PORTS_MAX           (6)
#define INPUT_NUMBER_LEN            (10)


/******************************************************************************
*                               VARIABLE DEFINITIONS
******************************************************************************/
uint8_t spp_bd_address[LOCAL_BDA_LEN] = {0x11, 0x12, 0x13, 0x21, 0x22, 0x23};
static struct timespec start_time = {0};
static struct timespec end_time = {0};
static char spp_menu[] = "\n\
------------------------------------SPP MENU------------------------------------\n\n";
static char menu_options[] = "\n\
    0.  Exit\n\
    1.  Print Menu \n\
    2.  Send Large Sample Data\n\
    3.  Type And Send String\n\
    4.  Enable RX Loopback\n\
    5.  Disable RX Log\n\
    6.  Start Throughput Calculation\n\
    7.  End Throughput Calculation \n\n\
Choose option -> ";
static char app_init_ports_prompt[] = "\n\n\
------------------------------------SPP MENU------------------------------------\n\n\
    Please enter how many spp ports that you want to generate. [1] to [6] \n\n\
Kindly pair again in case you changed it.\n\
Enter-> ";
static char app_spp_terminal_prompt[] = "\n\n\
------------------------------------SPP MENU------------------------------------\n\n\
    Please enter number:\n\
    [1]: Enable display board (Raspberry Pi)   [0]: No display board.\n\n\
Kindly close all other terminals before you enalbe display board.\n\
Enter-> ";


/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/
uint32_t hci_control_proc_rx_cmd(uint8_t *p_buffer, uint32_t length);
void APPLICATION_START(void);

static void clean_stdin(void);
static void send_port_data(uint8_t port_idx);
static bool user_create_spp_ports(void);
static bool user_enable_spp_terminal(void);
static int input_a_num(char* prompt);


/******************************************************************************
 * Function Name: hci_control_proc_rx_cmd
 *******************************************************************************
 * Summary:
 *   Function to handle HCI receive (This will be called from
 *   porting layer -> linux_tcp_server.c)
 *
 * Parameters:
 *   uint8_t* p_buffer : rx buffer
 *   uint32_t length : rx buffer length
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
 * Function Name: APPLICATION_START
 *******************************************************************************
 * Summary:
 *   BT stack initialization function wrapper
 *
 ******************************************************************************/
void APPLICATION_START(void)
{
    spp_application_start();
}

/******************************************************************************
 * Function Name: clean_stdin
 *******************************************************************************
 * Summary:
 *   Clean up stdin
 *
 ******************************************************************************/
static void clean_stdin(void)
{
    for(int c = 0; (c = getchar()) != EOF && c != '\n';);
}

/******************************************************************************
 * Function Name: input_a_num
 *******************************************************************************
 * Summary:
 *   Get user input number
 * 
 * Parameters:
 *   char* prompt : prompt string
 * 
 * Return:
 *   int : input number, in this code example -1 as invalid input
 *
 ******************************************************************************/
static int input_a_num(char* prompt)
{
    char buf[INPUT_NUMBER_LEN + 1] = {0};
    uint8_t i = 0;
    int num = 0;
    int c = 0;
    fprintf(stdout, "%s\n", prompt);
    for(;; i++)
    {
        c = getchar();
        if ((uint8_t)c == '\n' || c == EOF)
        {
            break;
        }
        else if (i < INPUT_NUMBER_LEN && isdigit(c))
        {
            buf[i] = (char)c;
        }
        else
        {
            clean_stdin();
            fprintf(stdout, "Invalid input received, try again.\n");
            return -1;
        }
    }
    if(i)
    {
        num = (int)strtol(buf, NULL, 10);
    }
    else
    {
        fprintf(stdout, "Invalid input received, try again.\n");
        return -1;
    }
    return num;
}

/******************************************************************************
 * Function Name: user_create_spp_ports
 *******************************************************************************
 * Summary:
 *   Create (init) spp ports according user's input number
 *
 * Return:
 *   bool : result status
 *
 ******************************************************************************/
static bool user_create_spp_ports(void)
{
    int num = 0;
    while(true)
    {
        num = input_a_num(app_init_ports_prompt);
        if(num < SUPPORT_PORTS_MIN || num > SUPPORT_PORTS_MAX)
        {
            fprintf(stdout, "Please enter number within %d ~ %d, try again.\n", SUPPORT_PORTS_MIN, SUPPORT_PORTS_MAX);
        }
        else
        {
            return spp_init(num) == WICED_TRUE;
        }
    }
    /* Will not be here */
    return false;
}

/******************************************************************************
 * Function Name: user_enable_spp_terminal
 *******************************************************************************
 * Summary:
 *   Enalbe spp display terminal according user's input number
 *
 * Return:
 *   bool : result value
 *
 ******************************************************************************/
static bool user_enable_spp_terminal(void)
{
    int enable = 0;
    while(true)
    {
        enable = input_a_num(app_spp_terminal_prompt);
        if(enable < 0 || enable > 1)
        {
            fprintf(stdout, "Please enter number 1 or 0, try again.\n");
        }
        else
        {
            return spp_enable_terminal((bool)enable);
        }
    }
}

/******************************************************************************
 * Function Name: send_port_data
 *******************************************************************************
 * Summary:
 *   Send data to specific spp port
 *
 * Parameters:
 *   uint8_t server_idx : server index
 *
 ******************************************************************************/
static void send_port_data(uint8_t server_idx)
{
    int c = 0;
    uint8_t* p_buf = spp_port_tx_buffer(server_idx);

    if (spp_port_handle(server_idx))
    {
        fprintf(stdout, "Enter string to be sent:\n");
        for(uint16_t i = 0;; i++)
        {
            c = getchar();
            if ((uint8_t)c == '\n')
            {
                p_buf[i] = '\0';
                spp_send_serial_data(server_idx, p_buf, i + 1);
                return;
            }
            if(c == EOF)
            {
                p_buf[i] = '\0';
                return;
            }

            p_buf[i] = (uint8_t)c;
            if (i >= SPP_MAX_PAYLOAD - 1)
            {
                i = 0;
                spp_send_serial_data(server_idx, p_buf, SPP_MAX_PAYLOAD);
                continue;
            }
        }
    }
    else
    {
        fprintf(stdout, "SPP server %d is not connected.\n", server_idx + 1);
    }
}

/******************************************************************************
 * Function Name: main
 *******************************************************************************
 * Summary:
 *   Application entry function
 *
 * Parameters:
 *   int argc : argument count
 *   char *argv[] : list of arguments
 *
 ******************************************************************************/
int main(int argc, char *argv[])
{
    cybt_controller_gpio_config_t autobaud = {0};
    pthread_t throughput_calc_thread_handle = {0};      /* Throughput calculation thread handler */
    uint32_t hci_baudrate = 0;
    uint32_t patch_baudrate = 0;
    int btspy_inst = 0;
    int choice = 0;
    char fw_patch_file[MAX_PATH] = {0};
    char hci_port[MAX_PATH] = {0};
    char peer_ip_addr[16] = "000.000.000.000";
    uint8_t btspy_is_tcp_socket = 0;
    bool error = false;

    if (PARSE_ERROR ==
        arg_parser_get_args(argc, argv, hci_port, spp_bd_address, &hci_baudrate,
                            &btspy_inst, peer_ip_addr, &btspy_is_tcp_socket,
                            fw_patch_file, &patch_baudrate, &autobaud))
    {
        return EXIT_FAILURE;
    }

    cy_bt_spy_comm_init(btspy_is_tcp_socket, btspy_inst, peer_ip_addr);

    cy_platform_bluetooth_init(fw_patch_file, hci_port, hci_baudrate,
                               patch_baudrate, &autobaud.autobaud_cfg);

    /* 1 - Wait for BT ready */
    spp_bt_enabled_lock();
    sleep(1);

    fprintf(stdout, "Create spp ports status: %s\n", user_create_spp_ports()? "OK" : "Not OK");
    fprintf(stdout, "Enalbe spp terminal: %s\n", user_enable_spp_terminal()? "OK" : "Not OK");

    fprintf(stdout, "\n%s\n", "Configuring...one moment\n");
    sleep(1);

    /* Throughput calculation default start time */
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    for (;;)
    {
        char menu_buffer[MENU_BUFFER_LENGTH] = {0};
        sprintf(menu_buffer, "%s", spp_menu);
        spp_server_status_string(menu_buffer + strlen(menu_buffer), MENU_BUFFER_LENGTH - strlen(menu_buffer));
        sprintf(menu_buffer + strlen(menu_buffer), "%s", menu_options);
        choice = input_a_num(menu_buffer);
        switch (choice)
        {
            case EXIT:
                exit(EXIT_SUCCESS);

            case MENU_PRINT_MENU:
                break;

            case MENU_SEND_SAMPLE_DATA:
                choice = input_a_num("Input port number to which you want to send sample data:");
                if(choice >= SUPPORT_PORTS_MIN && choice <= spp_get_current_port_num())
                {
                    if (spp_port_handle(choice - 1))
                    {
                        spp_send_sample_data(choice - 1);
                        fprintf(stdout, "%s", "one moment.\n");
                        sleep(1);
                    }
                    else
                    {
                        fprintf(stdout, "SPP server %d is not connected.\n", choice);
                    }
                }
                else
                {
                    fprintf(stdout, "Please check your input number.\n");
                }
                break;

            case MENU_TYPE_AND_SEND_DATA:
                choice = input_a_num("Input port number to which you want to send sample data:");
                if(choice >= SUPPORT_PORTS_MIN && choice <= spp_get_current_port_num())
                {
                    send_port_data(choice - 1);
                    fprintf(stdout, "%s", "one moment.\n");
                    sleep(1);
                }
                else
                {
                    fprintf(stdout, "Please check your input number.\n");
                }
                break;

            case MENU_LOOPBACK_SWITCH:
                spp_mode_loopback_switch();
                break;

            case MENU_RX_LOG_SWITCH:
                spp_rx_log_switch();
                break;

            case MENU_THROUGHPUT_START_TIME:
                clock_gettime(CLOCK_MONOTONIC, &start_time);
                spp_start_throughput();
                fprintf(stdout, "Start Time: %ld (s) %ld (ns)\n", start_time.tv_sec, start_time.tv_nsec);
                break;

            case MENU_THROUGHPUT_END_TIME:
                clock_gettime(CLOCK_MONOTONIC, &end_time);
                fprintf(stdout, "\n\n%s\n\n", "--------------------------------Throughtput Stop--------------------------------");
                fprintf(stdout, "Start Time: %ld (s) %ld (ns)\n", start_time.tv_sec, start_time.tv_nsec);
                fprintf(stdout, "End Time:   %ld (s) %ld (ns)\n\n", end_time.tv_sec, end_time.tv_nsec);
                spp_end_throughput(&start_time, &end_time);
                break;

            default:
                fprintf(stdout, "Invalid input received, try again.\n");
                break;
        }
    }
    return EXIT_SUCCESS;
}