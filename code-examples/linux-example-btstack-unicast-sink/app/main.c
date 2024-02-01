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
* Description: This is the source code for Linux CE Audio Watch.
*
* Related Document: See README.md
*
*******************************************************************************/


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
#include "unicast_sink.h"
#include "log.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define MAX_PATH                        ( 256u )
#define DISCOVERABLE                    ( 1u )
#define CONNECTABLE                     ( 1u )
#define DATA_BUFFER_SIZE                ( 20u )
#define NO_HANDLE                       ( -1 )

/******************************************************************************
 *                              EXTERNS
 *****************************************************************************/
extern wiced_bt_device_address_t    bt_device_address;
extern wiced_bool_t                 le_adv_enable;
extern BOOL32                       is_adv;

/*******************************************************************************
*                               STRUCTURES AND ENUMERATIONS
*******************************************************************************/
typedef enum {
    EXIT,
    PRINT_MENU,
    READ_LOCAL_BD_ADDRESS,
    LE_CURRENT_STATUS,
    LE_START_ADV,
    LE_STOP_ADV,
    LE_SELECT_CONN_DEV,
    LE_AUDIO_PLAY,
    LE_AUDIO_PAUSE,
    LE_AUDIO_VOL_UP,
    LE_AUDIO_VOL_DOWN,
    LE_AUDIO_MUTE,
    LE_AUDIO_UNMUTE,
    LE_AUDIO_ABS_VOL,
    END
}eapp_menu_t ;

typedef struct {
    eapp_menu_t eidx;
    char *str;
}app_menu_t;

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
int         btspy_inst              = 0;        /* BTSPY instance */
static char g_app_name[MAX_PATH]    = {0};      /* Application name */
uint16_t    curr_conn_id            = 0;
int8_t      curr_handle             = -1;

app_menu_t app_menu_array[] = {
    {EXIT,                  "Exit"      },
    {PRINT_MENU,            "Print Menu"},
    {READ_LOCAL_BD_ADDRESS, "Read Local BD address"},
    {LE_CURRENT_STATUS,     "SHOW SINK CURRENT STATUS"},
    {LE_START_ADV,          "START ADV"},
    {LE_STOP_ADV,           "STOP  ADV"},
    {LE_SELECT_CONN_DEV,    "SELECT CONNECTED DEVICE"},
    {LE_AUDIO_PLAY,         "PLAY"},
    {LE_AUDIO_PAUSE,        "PAUSE"},
    {LE_AUDIO_VOL_UP,       "VOL UP"},
    {LE_AUDIO_VOL_DOWN,     "VOL DOWN"},
    {LE_AUDIO_MUTE,         "MUTE"},
    {LE_AUDIO_UNMUTE,       "UNMUTE"},
    {LE_AUDIO_ABS_VOL,      "SET ABS VOL"},
};

/****************************************************************************
 *                              FUNCTION DECLARATION
 ***************************************************************************/
static BOOL32 check_curr_conn_id( void );


/******************************************************************************
 * Function Name: print_menu
 *******************************************************************************
 * Summary: print menu
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 ******************************************************************************/
static void print_menu(void)
{
    for (uint16_t i = 0; i < sizeof(app_menu_array) / sizeof(app_menu_t); i++)
    {
        printf("%d.\t", i);
        printf("%s\n", app_menu_array[i].str);
    }
}

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
* Function Name: empty_stdin()
*******************************************************************************
* Summary:
*   clear stdin buffer
*
* Parameters:
*   None;
*
* Return:
*   None;
*
******************************************************************************/
static void empty_stdin(void)
{
    int c = getchar();

    while (c != '\n' && c != EOF)
        c = getchar();
}

/******************************************************************************
* Function Name: error_check()
*******************************************************************************
* Summary:
*   check the scanf result
*       EOF: CTRL-D, user cancel input, use clearerr(stdin), restore stdin
*       result == 0: input not match the scanf type, eg: %d, or %x
*
* Parameters:
*   int result: scanf retrun result
*
* Return:
*   BOOL32: WICED_TRUE: no error
*           WICED_FALSE: error occur
*
******************************************************************************/
static BOOL32 error_check(int result)
{
   if (result == EOF)
   {
        TRACE_ERR("USER CANCEL INPUT\n");
        clearerr(stdin);
        return WICED_FALSE;
   } else if (result == 0) {
        TRACE_ERR("USER INPUT FORMAT WRONG\n");
        empty_stdin();
        return WICED_FALSE;
   }
   return WICED_TRUE;
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
    char        fw_patch_file[MAX_PATH] = {0};      /* Firmware patch file */
    char        hci_port[MAX_PATH]      = {0};      /* Interface Device */ 
    char        peer_ip_addr[16]        = "000.000.000.000"; /* Peer IP Address */
    uint32_t    hci_baudrate            = 0;        /* HCI baud rate */ 
    uint32_t    patch_baudrate          = 0;        /* Patch downloading baud rate */
    uint8_t     btspy_is_tcp_socket     = 0;        /* BTSPY communication socket */
    int         ret                     = 0;
    int         input                   = 0;
    char        inputstr[3]             = {0};
    BOOL32      exit                    = WICED_FALSE;
    uint8_t     p_data[DATA_BUFFER_SIZE]= {0};
    uint8_t     handle                  = 0;
    int         abs_vol                 = 0;
    uint8_t     *p                      = p_data;
    wiced_result_t status = WICED_BT_SUCCESS;
    /* Platform GPIO configuration 
         * Audobaud configuration GPIO bank and pin 
         * Dev-Wake and Host-Wake
    */
    cybt_controller_gpio_config_t gpio_cfg = {0};


    /* Parse the arguments */
    if ( PARSE_ERROR == arg_parser_get_args(argc,
                                            argv,
                                            hci_port,
                                            bt_device_address,
                                            &hci_baudrate,
                                            &btspy_inst,
                                            peer_ip_addr, 
                                            &btspy_is_tcp_socket,
                                            fw_patch_file, 
                                            &patch_baudrate,
                                            &gpio_cfg))
    {
        return EXIT_FAILURE;
    }

    /* Extract the application name */
    memset( g_app_name, 0, sizeof( g_app_name ) );
    strncpy(g_app_name, argv[0], MAX_PATH - 1);

    cy_bt_spy_comm_init(btspy_is_tcp_socket, btspy_inst, peer_ip_addr);
    cy_platform_bluetooth_init( fw_patch_file, hci_port, hci_baudrate, patch_baudrate, &gpio_cfg.autobaud_cfg);

    if (fw_patch_file[0])
    {
        TRACE_LOG ("Waiting for downloading patch...");

        wait_controller_reset_ready();

    } else {
        TRACE_LOG ("no patch FW: %s\n", fw_patch_file);
    }

    TRACE_LOG(" Linux CE LE-Audio Unicast Sink initialization complete...\n" );
    
    do 
    {
        memset(inputstr, 0, sizeof(inputstr));
        input = 0;
        empty_stdin();
        printf("Press 1 and Click Enter to Show Menu Option!!\n");
        ret = scanf ("%[^\n]s", inputstr);
        if (ret == 0)
        {
            continue;
        }

        handle = 0;
        status = WICED_BT_SUCCESS;
        memset(p_data, 0, sizeof(p_data));
        p = p_data;
        if(error_check(ret) == WICED_FALSE)
        {
            goto INPUT_ERROR;
        }
        input = atoi(inputstr);
        printf("---------------------------------------------------\n");
        switch (input)
        {
            case EXIT:
                printf("Enter 1 Again For Exit:");
                ret = scanf("%d", &input);
                if(error_check(ret) == WICED_FALSE)
                {
                    goto INPUT_ERROR;
                }
                if (input == 1)
                {
                    exit = WICED_TRUE;
                    printf("Exit!!\n");
                }
                break;
            case PRINT_MENU:
                print_menu();
                break;
            case READ_LOCAL_BD_ADDRESS:
                read_local_bda();
                break;
            case LE_START_ADV:
                UINT8_TO_STREAM(p, WICED_TRUE);
                unicast_sink_handle_adv(p_data, p-p_data);
                break;
            case LE_STOP_ADV:
                UINT8_TO_STREAM(p, WICED_FALSE);
                unicast_sink_handle_adv(p_data, p-p_data);
                break;
            case LE_SELECT_CONN_DEV:
            {
                printf("Input the Handle of Connected Source Device\n");
                unicast_sink_clcb_t *p_clcb = NULL;
                unicast_sink_show_connected_device();
                ret = scanf("%hhd", &handle);
                if(error_check(ret) == WICED_FALSE)
                {
                    goto INPUT_ERROR;
                }
                if (handle >= MAX_CONNECTION_INSTANCE)
                {
                    TRACE_ERR("Input Handle:%d out of range, 0~%d\n", handle, MAX_CONNECTION_INSTANCE-1);
                    goto INPUT_ERROR;
                }
                p_clcb = unicast_sink_get_connected_device_by_handle(handle);

                if (p_clcb == NULL)
                {
                    TRACE_ERR("p_clcb is NULL, get connected device by handle:%d fail\n", handle);
                    break;
                }
                curr_handle = handle;
                curr_conn_id = p_clcb->conn_id;
            }
                break;
            case LE_AUDIO_PLAY:
            case LE_AUDIO_PAUSE:
            case LE_AUDIO_VOL_UP:
            case LE_AUDIO_VOL_DOWN:
            case LE_AUDIO_MUTE:
            case LE_AUDIO_UNMUTE:
                if (check_curr_conn_id() != WICED_TRUE)
                {
                    break;
                }
                UINT16_TO_STREAM(p, curr_conn_id);
                switch(input)
                {
                    case LE_AUDIO_PLAY:
                        if (unicast_sink_is_streaming(curr_conn_id))
                        {
                            TRACE_ERR("Is Streaming, please pause first\n");
                            break;
                        }
                        unicast_sink_handle_play(p_data, p-p_data);
                        break;
                    case LE_AUDIO_PAUSE:
                        if (unicast_sink_is_streaming(curr_conn_id) == WICED_FALSE)
                        {
                            TRACE_ERR("Not Streaming\n");
                            break;
                        }
                        unicast_sink_handle_pause(p_data, p-p_data);
                        break;
                    case LE_AUDIO_VOL_UP:
                        unicast_sink_handle_vol_up(p_data, p-p_data);
                        break;
                    case LE_AUDIO_VOL_DOWN:
                        unicast_sink_handle_vol_down(p_data, p-p_data);
                        break;
                    case LE_AUDIO_MUTE:
                        unicast_sink_handle_mute(p_data, p-p_data);
                        break;
                    case LE_AUDIO_UNMUTE:
                        unicast_sink_handle_unmute(p_data, p-p_data);
                        break;
                }
                break;
            case LE_AUDIO_ABS_VOL:
                if (check_curr_conn_id() != WICED_TRUE)
                {
                    break;
                }
                abs_vol = 0;

                UINT16_TO_STREAM(p, curr_conn_id);
  
                TRACE_MSG("Enter ABS volume value:0~255\n");
                ret = scanf("%d", &abs_vol);
                if(error_check(ret) == WICED_FALSE)
                {
                    goto INPUT_ERROR;
                }
                if (abs_vol < 0)
                {
                    TRACE_ERR("Input value is Negative\n");
                    goto INPUT_ERROR;
                }
                if (abs_vol > 255)
                {
                    TRACE_ERR("Input value over 255\n");
                    goto INPUT_ERROR;
                }
                UINT8_TO_STREAM(p, abs_vol);
                unicast_sink_handle_abs_vol(p_data, p-p_data);
                break;
            case LE_CURRENT_STATUS:
            {
                //1. show adv status
                printf("ADV Status:%s\n", is_adv ? "Start":"Stop");

                //2. show current connected status
                unicast_sink_show_connected_device();

                //3. show current select handle
                if (curr_handle == NO_HANDLE)
                {
                    printf("No Selected Connected Source\n");
                    break;
                }
                else
                {
                    printf("Current Selected Handle:%d\n", curr_handle);
                }

                //4. show media state
                printf("Media State:%s\n", unicast_sink_is_streaming(curr_conn_id)?"PLAYING":"OTHERS");
            }
                break;
            default:
INPUT_ERROR:
                TRACE_ERR("Input error!!\n");
                break;
        }
        printf("---------------------------------------------------\n");
    } while (!exit);
    
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

