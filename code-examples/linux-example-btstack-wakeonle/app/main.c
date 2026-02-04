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
* Description: This is the source code for Linux CE WakeOnBle.
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
#include "wiced_bt_cfg.h"
#include "utils_arg_parser.h"
#include "wakeon_le.h"
#include "wiced_exp.h"
#include "log.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#ifndef MAX_PATH
#define MAX_PATH        (256)
#endif

/******************************************************************************
 *                                EXTERNS
 *****************************************************************************/
extern wiced_bt_device_address_t bt_device_address;
extern BOOL32 inSleep;
extern tBT_UUID uuid;
extern wiced_bt_device_address_t peer_BDAddr;

extern uint8_t pattern[LE_PCF_MANUFACTURE_DATA_PATTERN_LEN_MAX];
extern uint32_t data_len;
extern BOOL32 app_ready;

/*******************************************************************************
*                               STRUCTURES AND ENUMERATIONS
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
/* Application name */
static char g_app_name[MAX_PATH];
static const char app_menu[] = "\n\
------------WakeOnLE MENU-----------------------\n\n\
    0.  Exit \n\
    1.  Print Menu \n\
    2.  Disable WakeOnLE \n\
    3.  Enable WakeOnLE with 16bit UUID \n\
    4.  Enable WakeOnLE with 32bit UUID \n\
    5.  Enable WakeOnLE with 32bit UUID AND MANUFACTURE DATA \n\
    6.  Enable WakeOnLE Connection \n\
    7.  Reset to LE Legacy Only \n\
    8.  Reset to Default \n\
    9.  Is Ext Adv Support \n\
Choose option -> ";

/* Platform GPIO configuration 
     * Audobaud configuration GPIO bank and pin 
     * Dev-Wake and Host-Wake
*/
cybt_controller_gpio_config_t gpio_cfg;

/****************************************************************************
 *                              FUNCTION DECLARATIONS
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
* Function Name: APPLICATION_START()
*******************************************************************************
* Summary:
*   BT stack initialization function wrapper
*
* Parameters:
*   None
*
* Return:
*      None
*
******************************************************************************/
void APPLICATION_START( void )
{
    application_start();
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
   }
   else if (result == 0)
   {
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
    uint32_t uuid32 = 0;
    uint16_t uuid16 = 0;
    int ret = 0;
    int input = 0;
    int i = 0;

    arg_parser_arguments_t parsed_args = {0};
    if (PARSE_ERROR == arg_parser_get_args(argc, argv, &parsed_args))
    {
        return PARSE_ERROR;
    }
    memcpy(bt_device_address, parsed_args.local_bda, BD_ADDR_LEN);
    memcpy(&gpio_cfg, &parsed_args.p_gpio_cfg, sizeof(gpio_cfg));

    /* Extract the application name */
    memset(g_app_name, 0, sizeof( g_app_name ) );
    strncpy(g_app_name, argv[0], MAX_PATH - 1);

    cy_bt_spy_comm_init(parsed_args.is_socket_tcp, parsed_args.spy_inst, parsed_args.peer_ip_addr);
    cy_platform_bluetooth_init(
        parsed_args.patchFile,
        parsed_args.com_port,
        parsed_args.baud_rate,
        parsed_args.patch_baud,
        &parsed_args.p_gpio_cfg.autobaud_cfg);

    wait_controller_reset_ready();
    TRACE_MSG("Linux CE Wake On LE initialization complete...\n" );

    do 
    {
        printf("%s", app_menu);
        memset(&uuid, 0, sizeof(uuid));
        uuid32 = 0;
        uuid16 = 0;
        data_len = 0;
        ret = scanf ("%d", &input);
        if(error_check(ret) == WICED_FALSE)
        {
            goto INPUT_ERROR;
        }
        switch (input)
        {
            case 0:
                TRACE_MSG("Exit!!\n");
                break;
            case 1:
                //print menu
                break;
            case 2:
                app_disable_wake_on_le();
                break;
            case 3:
            {
                if (wiced_exp_isExtAdvSupported())
                {
                    TRACE_LOG("EXT ADV Support, set to Legacy first\n");
                    break;
                }
                unsigned int read = 0;
                if (inSleep == TRUE)
                {
                    TRACE_MSG("In Sleep MODE\n");
                    break;
                }
                TRACE_MSG("Enter 16bit uuid XX XX. eg: AA BB\n");
                for(i = 0; i < LEN_UUID_16; i++)
                {
                    ret = scanf("%x", &read);
                    if(error_check(ret) == WICED_FALSE)
                    {
                        goto INPUT_ERROR;
                    }
                    uuid16 = uuid16 << 8;
                    uuid16 |= read;
                }
                TRACE_MSG("INPUT UUID is:%x\n", uuid16);
                uuid.uu.uuid16 = uuid16;
                uuid.len = LEN_UUID_16;
                app_enable_wake_on_le_uuid();
            }
                break;
            case 4:
            {
                if (wiced_exp_isExtAdvSupported())
                {
                    TRACE_LOG("EXT ADV Support, set to Legacy first\n");
                    break;
                }
                unsigned int read = 0;
                if (inSleep == TRUE)
                {
                    TRACE_MSG("In Sleep MODE\n");
                    break;
                }
                TRACE_MSG("Enter 32bit uuid XX XX XX XX. eg: 11 22 33 44\n");
                for(i = 0; i < LEN_UUID_32; i++)
                {
                    ret = scanf("%x", &read);
                    if(error_check(ret) == WICED_FALSE)
                    {
                        goto INPUT_ERROR;
                    }
                    uuid32 = uuid32 << 8;
                    uuid32 |= read;
                }
                TRACE_MSG("INPUT UUID is:%x\n", uuid32);
                uuid.uu.uuid32 = uuid32;
                uuid.len = LEN_UUID_32;
                app_enable_wake_on_le_uuid();
	        }
	        break;
	    case 5:
            {
                if (wiced_exp_isExtAdvSupported())
                {
                    TRACE_LOG("EXT ADV Support, set to Legacy first\n");
                    break;
                }
                unsigned int read = 0;
                if (inSleep == TRUE)
                {
                    TRACE_MSG("In Sleep MODE\n");
                    break;
                }
                TRACE_MSG("Enter 32bit uuid XX XX XX XX. eg: 11 22 33 44\n");
                for(i = 0; i < LEN_UUID_32; i++)
                {
                    ret = scanf("%x", &read);
                    if(error_check(ret) == WICED_FALSE)
                    {
                        goto INPUT_ERROR;
                    }
                    uuid32 = uuid32 << 8;
                    uuid32 |= read;
                }
                TRACE_MSG("Enter Manufacture Data Pattern length limited 27 bytes:\n");
                ret = scanf("%d", &data_len);
                if(error_check(ret) == WICED_FALSE)
                {
                    goto INPUT_ERROR;
                }
                if (data_len > LE_PCF_MANUFACTURE_DATA_PATTERN_LEN_MAX)
                {
                    TRACE_MSG("ERROR:Data Pattern length Over 27 bytes:\n");
                    break;
                }
                TRACE_MSG("Enter Manufacture Data Pattern in Hex. XX XX ... XX \n");
                for(i = 0; i < data_len; i++)
                {
                    ret = scanf("%x", (uint32_t*)&pattern[i]);
                    if(error_check(ret) == WICED_FALSE)
                    {
                        goto INPUT_ERROR;
                    }
                }

                TRACE_MSG("INPUT UUID is:0x%x\n", uuid32);
                TRACE_MSG("INPUT DATA PATTERN is 0x:");
                for (i = 0; i < data_len; i++)
                {
                    TRACE_MSG("%x", pattern[i]);
                }
                TRACE_MSG("\n");
                uuid.uu.uuid32 = uuid32;
                uuid.len = LEN_UUID_32;
                app_enable_wake_on_le_uuid_manu();
            }
		    break;
        case 6:
        {
            
            TRACE_MSG("Wake ON LE Connection from particular LE ADDR");
            unsigned int read;
            if (inSleep == TRUE)
            {
                TRACE_MSG("In Sleep MODE\n");
                break;
            }
            TRACE_MSG("Enter Address of device to add in Hex: eg: 11 22 33 44 55 66");

            for (int i = 0; i < 6; i++)
            {
                ret = scanf("%x", (uint32_t *)&peer_BDAddr[i]);
                if (error_check(ret) == WICED_FALSE)
                {
                    goto INPUT_ERROR;
                }
            }

            app_enable_wake_on_connection();
            break;
        }
        case 7:
#ifdef WAKEONLE
            if (wiced_exp_isExtAdvSupported())
                wiced_exp_reset_to_le_legacy(NULL);
            else
                TRACE_LOG("Already LE Legacy ADV Only");
            app_ready = WICED_FALSE;
#else
            TRACE_ERR("No define WAKEONLE\n");
#endif
            break;
        case 8:
            wiced_exp_reset(NULL);
            app_ready = WICED_FALSE;
            break;
        case 9:
            TRACE_LOG("is Ext support:%s\n", wiced_exp_isExtAdvSupported() ? "Ext Adv Supported" : "Legacy Adv Only");
            break;
        default:
INPUT_ERROR:
            TRACE_ERR("Input error!!\n");
            break;
        }
    } while (input != 0);
    
    return EXIT_SUCCESS;
}
