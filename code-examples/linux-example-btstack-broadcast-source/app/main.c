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
* Description: This is the source code for Broadcast Source Application Main function
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
#include "wiced_bt_ga_bap.h"

#include "broadcast_source.h"
#include "app_bt_utils.h"
#include "log.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define MAX_PATH                        ( 256u )
#define DISCOVERABLE                    ( 1u )
#define CONNECTABLE                     ( 1u )


/******************************************************************************
 *                              EXTERNS
 *****************************************************************************/
extern wiced_bt_device_address_t    bt_device_address;
extern wiced_bool_t                 le_adv_enable;
extern broadcast_stream_config      bis_config;
/*******************************************************************************
*                               STRUCTURES AND ENUMERATIONS
*******************************************************************************/

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
/* Platform GPIO configuration 
     * Audobaud configuration GPIO bank and pin 
     * Dev-Wake and Host-Wake
*/
cybt_controller_gpio_config_t gpio_cfg;

int btspy_inst = 0;        /* BTSPY instance */

/* Application name */
static char         g_app_name[MAX_PATH];
uint8_t             avrc_ct_handle[2];

typedef enum {
    EXIT,
    PRINT_MENU,
    READ_LOCAL_BD_ADDRESS,
    READ_BIS_CONFIG,
    SET_BIS_ENCRYPTED,
    SET_BIS_BROADCAST_CODE,
    SET_BIS_BROADCAST_ID,
    SET_PLAYBACK_SAMPLE,
    START_STOP_BROADCAST,
    END
}eapp_menu_t ;

typedef enum {
    BIS_16_2,
    BIS_48_2,
    BIS_48_4,
    BIS_48_6,
    BIS_32_2,
    SAMPLE_END
}esample_menu_t ;

typedef struct {
    eapp_menu_t eidx;
    char *str;
}app_menu_t;

typedef struct {
    esample_menu_t eidx;
    char *str;
}sample_menu_t;

app_menu_t app_menu_array[] = {
    {EXIT,                  "Exit"      },
    {PRINT_MENU,            "Print Menu"},
    {READ_LOCAL_BD_ADDRESS, "Read Local BD address" },
    {READ_BIS_CONFIG, "Read BIS Config" },
    {SET_BIS_ENCRYPTED, "Set BIS Config: Encrypted" },
    {SET_BIS_BROADCAST_CODE, "Set BIS Config: Broadcast_code" },
    {SET_BIS_BROADCAST_ID, "Set BIS Config: Broadcast_ID" },
    {SET_PLAYBACK_SAMPLE, "Set BIS Config: Choose Sample" },
    {START_STOP_BROADCAST, "Set BIS START or STOP" },
};

sample_menu_t sample_menu_array[] = {
    {BIS_16_2, "16_2 (10ms-32kpbs)"},
    {BIS_48_2, "48_2 (10ms-80kpbs)"},
    {BIS_48_4, "48_4 (10ms-96kpbs)" },
    {BIS_48_6, "48_6 (10ms-124kpbs)" },
    {BIS_32_2, "32_2 (10ms-64kpbs)" },
};

static void print_menu(void)
{
    for (uint16_t i = 0; i < sizeof(app_menu_array) / sizeof(app_menu_t); i++)
    {
        printf("%d.\t", i);
        printf("%s\n", app_menu_array[i].str);
    }
}

static void print_sample(void)
{
    for (uint16_t i = 0; i < sizeof(sample_menu_array) / sizeof(sample_menu_t); i++)
    {
        printf("%d.\t", i);
        printf("%s\n", sample_menu_array[i].str);
    }
}

static uint32_t codec_config_mapping(uint32_t input){
    uint32_t res = 0;
    switch (input){
        case BIS_16_2:
            printf("%s\n", sample_menu_array[input].str);
            res = BAP_CODEC_CONFIG_16_2_2;
            break;
        case BIS_48_2:
            printf("%s\n", sample_menu_array[input].str);
            res = BAP_CODEC_CONFIG_48_2_2;
            break;
        case BIS_48_4:
            printf("%s\n", sample_menu_array[input].str);
            res = BAP_CODEC_CONFIG_48_4_2;
            break;
        case BIS_48_6:
            printf("%s\n", sample_menu_array[input].str);
            res = BAP_CODEC_CONFIG_48_6_2;
            break;
        case BIS_32_2:
            printf("%s\n", sample_menu_array[input].str);
            res = BAP_CODEC_CONFIG_32_2_2;
            break;
        default:
            res = BAP_CODEC_CONFIG_16_2_2;
            break;
    }
    return res;
}
/****************************************************************************
 *                              FUNCTION
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
    int         ret = 0;
    int         input = 0;
    BOOL32      exit = FALSE;
    wiced_result_t status = WICED_BT_SUCCESS;

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

    TRACE_LOG(" Linux CE LE-Audio Broadcast Source initialization complete" );
    
    do 
    {
        printf("Press 1 and Click Enter to Show Menu Option!!\n");
        ret = scanf ("%d", &input);
        status = WICED_BT_SUCCESS;
        if(error_check(ret) == WICED_FALSE)
        {
            goto INPUT_ERROR;
        }
        while(getchar()!= '\n');
        switch (input)
        {
            case EXIT:
                exit = TRUE;
                TRACE_MSG("Exit!!\n");
                break;
            case PRINT_MENU:
                print_menu();
                break;
            case READ_LOCAL_BD_ADDRESS:
                read_local_bda();
                break;
            case READ_BIS_CONFIG:
                print_broadcast_stream_config(bis_config);
                break;
            case SET_BIS_ENCRYPTED:
                if (is_broadcast_streaming(bis_config)){
                    TRACE_LOG("Can't change setting until stop BIS streaming");
                }
                else{
                    unsigned int encryption;
                    TRACE_LOG("Enter if set BIS encryption: 0: no, 1: yes");
                    ret = scanf("%u", &encryption);
                    if (error_check(ret) ==  WICED_FALSE){
                        TRACE_LOG("Enter encryption input error!");
                        break;
                    }
                    else{
                        if (encryption == 1){
                            set_broadcast_stream_config_encryption(true, &bis_config);
                        }
                        else{
                            set_broadcast_stream_config_encryption(false, &bis_config);
                        }
                        print_broadcast_stream_config(bis_config);
                    }
                }
                break;
            case SET_BIS_BROADCAST_CODE:
                if (is_broadcast_streaming(bis_config)){
                    TRACE_LOG("Can't change setting until stop BIS streaming");
                }
                else{
                    int code_count = 0;
                    char input;
                    char input_broadcast_code[MAX_BROADCASTCODE_LEN] = {0};
                     TRACE_LOG("Enter the BIS broadcast_code:");
                    while (code_count <= MAX_BROADCASTCODE_LEN - 1){
                        if (scanf("%c", &input) == EOF){
                            TRACE_ERR("Enter allowed input error!");
                            break;
                        }
                        else{
                            if (input != '\n'){
                                input_broadcast_code[code_count] = input;
                            }
                            else{
                                break;
                            }
                            code_count++;
                        }
                    }
                    set_broadcast_stream_config_broadcastcode(input_broadcast_code, &bis_config);
                    print_broadcast_stream_config(bis_config);
                }
                break;
            case SET_BIS_BROADCAST_ID:
                if (is_broadcast_streaming(bis_config)){
                    TRACE_LOG("Can't change setting until stop BIS streaming");
                }
                else{
                    unsigned int broadcast_id;
                    TRACE_LOG("Enter the BIS broadcast_id (hex) from 1 ~ f423f");
                    ret = scanf("%x", &broadcast_id);
                    if (error_check(ret) ==  WICED_FALSE){
                        TRACE_LOG("SET_BIS_BROADCAST_ID input error!");
                        break;
                    }
                    set_broadcast_stream_config_broadcastid(broadcast_id, &bis_config);
                    print_broadcast_stream_config(bis_config);
                }
                break;
            case SET_PLAYBACK_SAMPLE:
                TRACE_LOG("Enter the Sample from 0 ~ %d",sizeof(sample_menu_array) / sizeof(sample_menu_t));
                print_sample();

                unsigned int sample_id;
                ret = scanf("%d", &sample_id);
                if (error_check(ret) ==  WICED_FALSE){
                    TRACE_LOG("SET sample_id input error!");
                    break;
                }
                set_broadcast_stream_config_codec(codec_config_mapping(sample_id), &bis_config);
                print_broadcast_stream_config(bis_config);
                break;
            case START_STOP_BROADCAST:
                if (bis_config.start == 0){
                    TRACE_LOG("START_BROADCAST");
                    bis_config.start = 1;
                    broadcast_source_handle_start_streaming(bis_config);
                }
                else{
                    TRACE_LOG("STOP_BROADCAST");
                    bis_config.start = 0;
                    broadcast_source_handle_start_streaming(bis_config);
                }
                break;
/*
*/
            default:
INPUT_ERROR:
                TRACE_ERR("Input error!!\n");
                break;
        }
    } while (!exit);
    
    return EXIT_SUCCESS;
}
