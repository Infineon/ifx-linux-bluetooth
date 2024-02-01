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
* Description: This is the source code for Linux CE template project.
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

#include "wiced_bt_trace.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_types.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "platform_linux.h"
#include "utils_arg_parser.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_gfps.h"

#include "wiced_bt_cfg.h"
#include "headset.h"
#include "bt_hs_spk_audio.h"
#include "headset_control.h"
#include "app_bt_utils.h"
#include "log.h"

/*******************************************************************************
*                           MACROS
*******************************************************************************/
#define MAX_PATH                         ( 256 )
#define CONN_INTERVAL_MULTIPLIER        ( 1.25f )
#define IP_ADDR_LEN           (16)
#define MAX_PHONE_NUM_LEN     (16)
#define USER_INPUT_TRUE        (1)
#define USER_INPUT_FALSE       (0)
/*******************************************************************************
*                     STRUCTURES AND ENUMERATIONS
*******************************************************************************/
/* This enumeration combines the scan and connection states from two different
 * callbacks to maintain the status in a single state variable */
typedef enum
{
    APP_BT_SCAN_OFF_CONN_OFF,
    APP_BT_SCAN_ON_CONN_OFF,
    APP_BT_SCAN_OFF_CONN_ON
} tAppBtAdvConnMode;

/* Connection state information structure */
typedef struct
{
    /* remote peer device address */
    wiced_bt_device_address_t             remote_addr;
    /* connection ID referenced by the stack */
    uint16_t                              conn_id;
    /* MTU exchanged after connection */
    uint16_t                              mtu;
    /* connection interval negotiated */
    float                                 conn_interval;
    /* RX PHY selected */
    wiced_bt_ble_host_phy_preferences_t   rx_phy;
    /* TX PHY selected */
    wiced_bt_ble_host_phy_preferences_t   tx_phy;
} tAppBtConnStateInfo;

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/

/* device BD addr */
extern uint8_t rm_deviceBDAddr[6];

/* BT BLE configuration settings */
extern const  wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
/* Application name */
static char g_app_name[MAX_PATH];
/* Connection information */
static tAppBtConnStateInfo conn_state_info;
/* Connection state */
static tAppBtAdvConnMode app_bt_scan_conn_state = APP_BT_SCAN_OFF_CONN_OFF;

static const char appMenu[] = "\
------------Headset MENU-----------------------\n\n\
    0.  Exit \n\
    1.  Print Menu \n\
    2.  Set Visibility \n\
    3.  Set Pairing Mode\n\
    4.  Set Inquiry and Connect Device\n\
    5.  BR/EDR ReConnect Paired Device\n\
    6.  BR/EDR Disconnect \n\
    7.  Get Current Statuses\n\
    8.  [HFP] Print Connection Details\n\
    9.  [HFP] Answer Call\n\
    10. [HFP] Hangup Call\n\
    11. [HFP] Dial Number\n\
    12. [HFP] Redial\n\
    13. [HFP] Query Current Calls\n\
    14. [HFP] Set Speaker Volume\n\
    15. [HFP] Set Microphone Volume\n\
    16. [HFP] Get Subscriber Info\n\
    17. [AVRCP] AVRCP_ACTION_VOLUME_UP\n\
    18. [AVRCP] AVRCP_ACTION_VOLUME_DOWN\n\
    19. [AVRCP] ACTION_PAUSE_PLAY\n\
    20. [AVRCP] AVRCP_ACTION_FORWARD\n\
    21. [AVRCP] AVRCP_ACTION_BACKWARD\n\
    22. [AVRCP] AVRCP_ACTION_STOP\n\
    23. [AVRCP] AVRCP_ACTION_FAST_FORWARD\n\
    24. [AVRCP] AVRCP_ACTION_FAST_REWIND\n\
    25. [AVRCP] AVRCP_ACTION_UNIT_INFO\n\
    26. [GFPS] Enable GFPS Discoverable\n\
Choose option -> ";

/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/

/******************************************************************************
*                               FUNCTION DEFINITIONS
******************************************************************************/


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
*      None
*
******************************************************************************/
int main( int argc, char* argv[] )
{
    int len = 0;
    char fw_patch_file[MAX_PATH];
    char hci_port[MAX_PATH];
    char peer_ip_addr[IP_ADDR_LEN] = "000.000.000.000";
    uint32_t baud =0 ,patch_baud = 0;
    int spy_inst = 0;
    uint8_t is_socket_tcp = 0;
    char input_choice[3] = {0,0,0};
    unsigned int choice = 0;
    wiced_result_t result = WICED_BT_SUCCESS;
    unsigned int handle;

    cybt_controller_gpio_config_t gpio_cfg;

    /* Parse the arguments */
    memset( fw_patch_file,0,MAX_PATH );
    memset( hci_port,0,MAX_PATH );
    if (EXIT_FAILURE == arg_parser_get_args(argc,
                                  argv,
                                  hci_port,
                                  rm_deviceBDAddr,
                                  &baud,
                                  &spy_inst,
                                  peer_ip_addr,
                                  &is_socket_tcp,
                                  fw_patch_file,
                                  &patch_baud,
                                  &gpio_cfg))
    {
        return EXIT_FAILURE;
    }

    cy_bt_spy_comm_init(is_socket_tcp, spy_inst, peer_ip_addr);
    cy_platform_bluetooth_init(fw_patch_file, hci_port, baud, patch_baud, &gpio_cfg.autobaud_cfg);

    TRACE_LOG(" Linux CE template project initialization complete..." );

    if (fw_patch_file[0])
    {
        TRACE_LOG ("Waiting for downloading patch...");
        wait_controller_reset_ready();
    }

    for(;;)
    {
        TRACE_LOG("%s", appMenu);
        fflush(stdin);
        if (scanf("%2s", input_choice) == EOF){
            TRACE_WNG( "Enter input cmd fail!!");
            choice = PRINT_MENU;
        }
        else{
            bt_atoi(input_choice, &choice);
        }

        if(headset_is_valid_state(choice))
        {
            switch(choice)
            {
                case EXIT:
                    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
                    exit(EXIT_SUCCESS);
                    break;
                case PRINT_MENU:
                    break;
                case SET_VISIBILITY:
                    {
                        unsigned int discoverable;
                        unsigned int connectable;
                        TRACE_LOG("Enter discoverability: 0:Non Discoverable, 1: Discoverable");
                        if (scanf("%u", &discoverable) == EOF){
                            TRACE_WNG("Enter discoverability input error!");
                            break;
                        }
                        TRACE_LOG("\nEnter connectability: 0:Non Connectable, 1: Connectable");
                        if (scanf("%u", &connectable) == EOF){
                            TRACE_WNG("Enter connectability input error!");
                            break;
                        }
                        headset_handle_set_visibility(discoverable, connectable);
                    }
                    break;
                case SET_PAIRING_MODE:
                    {
                        unsigned int allowed;
                        TRACE_LOG("Enter if pairing is allowed: 0: Not allowed, 1: Allowed");
                        if (scanf("%u", &allowed) == EOF){
                            TRACE_WNG("Enter allowed input error!");
                            break;
                        }
                        handset_handle_set_pairability(allowed);
                    }
                    break;
                case SET_INQUIRY:
                    {
                        unsigned int enable = 0;
                        unsigned int read = 0;
                        TRACE_LOG("Enter if Inquiry has to be enabled/disabled: 0: Disabled, 1: Enabled");
                        if (scanf("%u", &enable) == EOF){
                            TRACE_ERR("Enter Inquiry input error!");
                            break;
                        } 
                        handset_handle_set_inquiry(enable);
                        wait_inquery_done();
                        TRACE_LOG("Enter if want to connect peer device: 0: No, 1: Yes");
                        if (scanf("%u", &enable) == EOF){
                            TRACE_ERR("Enter connect peer device error!");
                            break;
                        }
                        if (enable == true){
                            wiced_bt_device_address_t peer_bd_addr;
                            TRACE_LOG("Enter Peer BD Address:\n");
                            for(int i = 0; i < BDA_LEN; i++)
                            {
                                if (scanf("%x", &read) == EOF){
                                    TRACE_ERR("Enter BD_ADDR input error!\n");
                                    break;
                                }
                                peer_bd_addr[i] = (unsigned char)read;
                            }
                             handset_handle_BREDR_connect(peer_bd_addr);
                        }
                    }
                    break;
                case HF_CONNECT:
                    {
                        TRACE_LOG("Reconnect Paired Device");
                        bt_hs_spk_control_reconnect();
                    }
                    break;
                case HF_DISCONNECT:
                    {
                        TRACE_LOG("BR/EDR Disconnect!");
                        bt_hs_spk_control_disconnect(NULL);
                    }
                    break;
                case GET_HEADSET_STATE:
                    {
                        headset_get_statues();
                    }
                    break;
               case HF_PRINT_CONNECTION_DETAILS:
                    {
                        bt_hs_spk_control_link_key_display();
                        headset_get_br_connect();
                    }
                    break;

                case ANSWER_CALL:
                    {
                        TRACE_LOG("ANSWER_CALL");
                        headset_send_at_command(headset_get_rfcomm_handle_idx(), HF_AT_CMD_ATA, 0, NULL);
                    }
                    break;
                case HANGUP_CALL:
                    {
                        headset_send_at_command(headset_get_rfcomm_handle_idx(), HF_AT_CMD_CHUP, 0, NULL);
                    }
                    break;
                case DIAL_NUM:
                case REDIAL:
                    {
                        uint8_t p_data[MAX_PHONE_NUM_LEN];
                        memset(p_data, 0, strlen(p_data));
                        if(DIAL_NUM == choice)
                        {
                            TRACE_LOG("Enter the Number to be Dialed: ");
                            if (scanf("%15s", p_data) == EOF){
                                TRACE_ERR("Enter Number input error!");
                                break;
                            }
                        }
                        headset_send_at_command(headset_get_rfcomm_handle_idx(), HF_AT_CMD_D, 0, p_data);
                    }
                    break;
                case QUERY_CUR_CALLS:
                    {
                        headset_send_at_command(headset_get_rfcomm_handle_idx(), HF_AT_CMD_CLCC, 0, NULL);
                    }
                    break;
                case SET_SPK_VOL:
                    {
                        int vol = HFP_VOLUME_LOW;
                        TRACE_LOG("Enter Volume: %d ~ %d:",HFP_VOLUME_LOW, HFP_VOLUME_HIGH);
                        if(scanf("%d", &vol) == EOF){                      
                            TRACE_ERR("Enter Volume input error!");
                            break;
                        }
                        else{
                            if ((vol < HFP_VOLUME_LOW) || (vol > HFP_VOLUME_HIGH)){
                                vol = HFP_VOLUME_AVERAGE;
                            }
                            TRACE_LOG("Volume input: %d", vol);
                        }
                        headset_send_at_command(headset_get_rfcomm_handle_idx(), HF_AT_CMD_VGS, vol, NULL);
                    }
                    break;
                case SET_MIC_VOL:
                    {
                        int vol = HFP_VOLUME_LOW;
                        TRACE_LOG("Enter Volume: %d ~ %d:",HFP_VOLUME_LOW, HFP_VOLUME_HIGH);
                        if (scanf("%d", &vol) == EOF){
                            TRACE_ERR("Enter Volume input error!");
                            break;
                        }
                        else{
                            if ((vol < HFP_VOLUME_LOW) || (vol > HFP_VOLUME_HIGH)){
                                vol = HFP_VOLUME_AVERAGE;
                            }
                            TRACE_LOG("Volume input: %d", vol);
                        }
                        headset_send_at_command(headset_get_rfcomm_handle_idx(), HF_AT_CMD_VGM, vol, NULL);
                    }
                    break;
                case SUBSCIBER_NUM_INFO:
                    {
                        headset_send_at_command(headset_get_rfcomm_handle_idx(), HF_AT_CMD_CNUM, 0, NULL);
                    }
                    break;
  
                case ENABLE_GFPS: //cmd: 26
                    {
                        unsigned int discover = USER_INPUT_FALSE;
                        TRACE_LOG("Enter ADV discovery mode : 0: Not discover, 1: discover\n");
                        if (scanf("%u", &discover) == EOF){
                            TRACE_LOG("Enter allowed input error!");
                            break;
                        }
                        if (discover == USER_INPUT_TRUE)
                        {
                            wiced_bt_gfps_provider_advertisement_start(USER_INPUT_TRUE);
                        } else {
                            wiced_bt_gfps_provider_advertisement_start(USER_INPUT_FALSE);
                        }
                    }
                    break;

                case AVRCP_ACTION_VOLUME_UP: //cmd: 17
                    {
                        result = bt_hs_spk_audio_button_handler_volume_up();
                        TRACE_LOG("Cmd: AVRCP_ACTION_VOLUME_UP: %d", result);
                    }
                    break;

                case AVRCP_ACTION_VOLUME_DOWN: //cmd: 18
                    {
                        result = bt_hs_spk_audio_button_handler_volume_down();
                        TRACE_LOG("Cmd: ACTION_VOLUME_DOWN: %d", result);
                    }
                    break;

                case AVRCP_ACTION_PAUSE_PLAY: //cmd: 19
                    {
                        result = bt_hs_spk_audio_button_handler_pause_play();
                        TRACE_LOG("Cmd: ACTION_PAUSE_PLAY: %d", result);
                    }
                    break;

                case AVRCP_ACTION_FORWARD: //cmd: 20
                    {
                        result = bt_hs_spk_audio_handler_skip_stop_fastforward_rewind(AVRC_ID_FORWARD, AVRC_STATE_PRESS);
                        TRACE_LOG("Cmd: AVRCP_ACTION_FORWARD: %d", result);
                    }
                    break;

                case AVRCP_ACTION_BACKWARD: //cmd: 21
                    {
                        result = bt_hs_spk_audio_handler_skip_stop_fastforward_rewind(AVRC_ID_BACKWARD, AVRC_STATE_PRESS);
                        TRACE_LOG("Cmd: AVRCP_ACTION_FORWARD: %d", result);
                    }
                    break;

                case AVRCP_ACTION_STOP: //cmd: 22
                    {
                        result = bt_hs_spk_audio_handler_skip_stop_fastforward_rewind(AVRC_ID_STOP, AVRC_STATE_PRESS);
                        TRACE_LOG("Cmd: AVRCP_ACTION_STOP: %d", result);
                    }
                    break;

                case AVRCP_ACTION_FAST_FORWARD: //cmd: 23
                    {
                        if (headset_get_playstate() == AVRC_PLAYSTATE_FWD_SEEK){
                            TRACE_LOG("Release FastForward");
                            result = bt_hs_spk_audio_handler_skip_stop_fastforward_rewind(AVRC_ID_FAST_FOR, AVRC_STATE_RELEASE);
                        }
                        else{
                            TRACE_LOG("Press FastForward");
                            result = bt_hs_spk_audio_handler_skip_stop_fastforward_rewind(AVRC_ID_FAST_FOR, AVRC_STATE_PRESS);  
                        }
                        TRACE_LOG("Cmd: AVRCP_ACTION_FAST_FORWARD: %d", result);
                    }
                    break;

                case AVRCP_ACTION_FAST_REWIND: //cmd: 24
                    {
                        if (headset_get_playstate() == AVRC_PLAYSTATE_REV_SEEK){
                            TRACE_LOG("Release FastRewind");
                            result = bt_hs_spk_audio_handler_skip_stop_fastforward_rewind(AVRC_ID_REWIND, AVRC_STATE_RELEASE);
                        }
                        else{
                            TRACE_LOG("Press FastRewind");
                            result = bt_hs_spk_audio_handler_skip_stop_fastforward_rewind(AVRC_ID_REWIND, AVRC_STATE_PRESS);
                        }
                        TRACE_LOG("Cmd: AVRCP_ACTION_FAST_REWIND: %d", result);
                    }
                    break;

                case AVRCP_ACTION_UNIT_INFO: //cmd: 25
                    {
                        result = bt_hs_spk_audio_get_attr_info();
                        TRACE_LOG("Cmd: AVRCP_ACTION_UNIT_INFO: %d", result);
                    }
                    break;

                default:
                    TRACE_WNG("Invalid Input: %d", choice);
                    break;
            }
        }
        else
        {
            TRACE_LOG("--------------------------------------------");
            TRACE_LOG("User Input Disallowed in the Current State");
            TRACE_LOG("--------------------------------------------");
        }
    }
    return EXIT_SUCCESS;
}
