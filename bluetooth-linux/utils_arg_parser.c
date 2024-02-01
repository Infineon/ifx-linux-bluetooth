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
 * File Name: arg_parser.c
 *
 * Description: This is the source file for argument parsing module.
 *
 *****************************************************************************/

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wiced_bt_trace.h"
#include "utils_arg_parser.h"
#include "log.h"



/******************************************************************************
 *                                MACROS
 *****************************************************************************/
#define MAX_PATH        (256)
#define BD_ADDR_LENGTH     (6)


/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/
static void stream_to_bdaddr( uint8_t *a, uint8_t *p );
static void print_usage( char *p_app_name );
static int parse_gpio(int *idx, char **argv, char *gpiochip, char *line_num);
static BOOL32 isDigit(char *line);

char app_name[MAX_PATH]; /* Application name */

/****************************************************************************
 *                              FUNCTION DEFINITIONS
 ***************************************************************************/

/******************************************************************************
 * Function Name: stream_to_bdaddr()
 ******************************************************************************
 * Summary:
 *   Function to convert byte stream to bd addresss.
 *
 * Parameters:
 *   uint8_t *a             : bd address
 *   uint8_t *p             : byte stream
 *
 * Return:
 *  None
 *
 *****************************************************************************/
static void stream_to_bdaddr( uint8_t *a, uint8_t *p )
{
    int iCnt; /* Loop counter variable */
    uint8_t *pbda = (uint8_t *)a + BD_ADDR_LENGTH - 1;
    for ( iCnt = 0; iCnt < BD_ADDR_LENGTH; iCnt++ )
    {
        *pbda-- = *p++;
    }
}


/******************************************************************************
 * Function Name: print_usage()
 ******************************************************************************
 * Summary:
 *   Function to print application usage.
 *
 * Parameters:
 *   uint8_t *p_app_name             : Application name
 *
 * Return:
 *  None
 *
 *****************************************************************************/
static void print_usage( char *p_app_name )
{
    printf( "\n" );
    printf( "Usage:\n" );
    printf( "\n" );
    printf( "%s -c <comport> -b <baud rate> -d <bda> -i <spy instance>\n", p_app_name );
    printf( "\nFor example:\n" );
    printf( "   %s -c COM21 -b 3000000 -i 1 -p \"firmware.hcd\")\n", p_app_name );
    printf( "\n" );
    printf( " c  Local device name, for example COM21 for UART.\n" );
    printf( " b  Baud rate for HCI UART device. support 115200 and 3000000 only\n" );
    printf( " d  Bluetooth Device Address to be programmed"
                    " to the device. A pre-determined address"
                    " is used if this is omitted.\n" );
    printf( " i  BT Spy instance number for logging (default is 0)\n" );
    printf( " r  GPIO bank and pin number for BR_REG_ON\n" );
    printf( " n  GPIO control ioctl method\n" );
    printf( " p  Full file path of .hcd patch file to download to device on startup.\n" );
    printf( " f  baud rate to be used for patch download, default value is 115200 if not specified, support 115200 and 921600 only\n" );
    printf( " w  GPIO bank and pin number for HOST-WAKE\n" );
    printf( " h  GPIO bank and pin number for DEV-WAKE\n" );
    printf( " t  Provide the IP address\n" );
    printf( " s  Enable TCP Socket for data transfer\n" );
    printf( "\n");
    printf( " NOTE: Please provide instance before enabling TCP"
                   " socket to get the updated port number display. "
                   "for eg :  -i 1 -t 127.0.0.1\n ");
    printf( "\n");
    return;
}


/******************************************************************************
 * Public Function 
 ******************************************************************************/
void AsciiToHex(uint8_t* bda, size_t len, char* pBdAddr)
{
    if (pBdAddr != NULL)
    {
        int count = strlen(pBdAddr) / 2;
        memset(bda, 0, len);
        for (int i = 0; i < (int)len && i < count; ++i)
        {
            bda[len - i - 1] = HexDigits(pBdAddr);
            pBdAddr += 2;
        }
    }
}

uint8_t HexDigits( char *p )
{
    uint8_t out = 0; /* Output variable */
    for ( int i = 0; i < 2; ++i, ++p )
    {
        if ( p == NULL || *p == '\0' )
        {
            return out;
        }
        out <<= 4;
        if ( '0' <= *p && *p <= '9' )
        {
            out += *p - '0';
        }
        else if ( 'A' <= *p && *p <= 'F' )
        {
            out += *p - 'A' + 10;
        }
        else if ( 'a' <= *p && *p <= 'f' )
        {
            out += *p - 'a' + 10;
        }
    }
    return out;
}

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
                        cybt_controller_gpio_config_t *p_gpio_cfg)
{
    unsigned int len = 0; /* Application name length */
    char local_bda_string[50]; /* Local BD address string */
    unsigned char r_exist = 0, p_exist = 0;
    if (p_gpio_cfg == NULL) 
    {
        TRACE_ERR( "gpio cfg is NULL.\n" );
        return PARSE_ERROR;
    }
    cybt_controller_autobaud_config_t *p_autobaud_cfg = &p_gpio_cfg->autobaud_cfg;
    cybt_gpio_wake_on_ble_t *p_wake_on_ble_cfg = &p_gpio_cfg->wake_on_ble_cfg;

    p_autobaud_cfg->use_ioctl = 0;
    memset( p_autobaud_cfg->bt_reg_on_off.line_num, 0, sizeof( p_autobaud_cfg->bt_reg_on_off.line_num ) );
    memset( p_autobaud_cfg->bt_reg_on_off.p_gpiochip, 0, sizeof( p_autobaud_cfg->bt_reg_on_off.p_gpiochip ) );

    /* get the app name */
    memset( app_name, 0, sizeof( app_name ) );
    len = strlen( argv[0] );
    if ( len >= MAX_PATH )
    {
        len = MAX_PATH - 1;
    }
    strncpy( app_name, argv[0], MAX_PATH - 1 );
    app_name[len] = 0;

    if ( 1 == argc )
    {
        print_usage( app_name );
        return PARSE_ERROR;
    };

    for ( int i = 1; i < argc; i++ )
    {
        if ( ( argv[i][0] != '-' ) )
        {
            continue;
        }
        switch ( toupper( argv[i][1] ) )
        {
        case 'C': /* HCI UART Port */
            if ( argc >= ( i + 2 ) )
            {
                unsigned int hci_len = 0;
                hci_len = strlen( argv[i + 1] );
                if ( hci_len >= MAX_PATH )
                    hci_len = MAX_PATH - 1;

                strncpy( device, argv[++i], hci_len );
                device[hci_len] = '\0';
            }
            else
            {
                TRACE_ERR( "No comport found in Command line options.\n" );
                print_usage( app_name );
                return PARSE_ERROR;
            }
            break;

        case 'B': /* HCI baud rate */
            if ( argc >= ( i + 2 ) )
            {  
                if (EOF == sscanf( argv[++i], "%u", baud ))
                {
                    TRACE_ERR( "set baud rate error in Command line options.\n" );
                    print_usage( app_name );
                    return PARSE_ERROR;
                }
            }
            else
            {
                TRACE_ERR( "No baud rate found in Command line options.\n" );
                print_usage( app_name );
                return PARSE_ERROR;
            }
            break;
        case 'F':     /* FW patch download baud rate */
            if ( argc >= ( i + 2 ) )
            {
                if (EOF == sscanf( argv[++i], "%u", patch_baud ))
                {
                    TRACE_ERR( "set baud rate error in Command line options.\n" );
                    print_usage( app_name );
                    return PARSE_ERROR;
                }
            }
            else
            {
                TRACE_ERR( "No baud rate for patch download found in Command"
                                                        "line options.\n" );
                print_usage( app_name );
                return PARSE_ERROR;
            }
            break;
        case 'P':     /* FW patch file to download */
            if ( argc >= ( i + 2 ) )
            {
                int len = 0;
                len = strlen( argv[i + 1] );
                if ( len >= MAX_PATH )
                    len = MAX_PATH - 1;

                strncpy( patchFile, argv[++i], len );
                patchFile[len] = '\0';
                p_exist = 1;
            }
            else
            {
                TRACE_ERR( "No patch file found in Command line options.\n" );
                print_usage( app_name );
                return PARSE_ERROR;
            }
            break;

        case 'I':
            *spy_inst = atoi( argv[++i] );
            if ( *spy_inst < 0 )
            {
                *spy_inst = 0;
            }
            break;

        case 'D': /* Device BD Address */
            if ( argc >= ( i + 2 ) )
            {
                strncpy(local_bda_string, argv[++i], sizeof(local_bda_string)-1);
                local_bda_string[sizeof(local_bda_string)-1] = '\0';
                uint8_t bda[BD_ADDR_LENGTH];
                AsciiToHex( bda, 6, local_bda_string );
                stream_to_bdaddr( local_bda, bda );
            }
            else
            {
                TRACE_ERR( "No BDA found in Command line options.\n" );
                print_usage( app_name );
                return PARSE_ERROR;
            }
            break;

        case 'T': /* Peer IP address for BYSPY connection */
            if ( argc >= ( i + 2 ) )
            {
                strcpy( peer_ip_addr, argv[++i] );
            }
            else
            {
                /* Set default to localHost */
                strcpy( peer_ip_addr, "127.0.0.1" );
            }
            break;

        case 'S': /* TCP Socket for BTSPY connection */
            *is_socket_tcp = 1;
            break;
        case 'N' :  /* use ioctl */
            p_autobaud_cfg->use_ioctl = 1;
            break;
        case 'R' :  /* BT REG of gpiochip and pin number(offset) */
        case 'H' :  /* BT HOST-WAKE path of gpiochip ie. /dev/gpiochipX and  pin number */
        case 'W' :  /* BT DEV-WAKE path of gpiochip ie. /dev/gpiochipX and  pin number */
            if (argc >= (i + 3))
            {
                if (strstr(argv[i+1], "gpiochip") == NULL || isDigit(argv[i+2]) == WICED_FALSE)
                {
                    TRACE_ERR("BT parsing %s %s %s\n", argv[i], argv[i+1], argv[i+2]);
                    print_usage( app_name );
                    return PARSE_ERROR;
                }
                char *p_gpiochip = NULL;
                char *p_line_num = NULL;
                char gpiochip[CYHAL_GPIOCHIP_NAME_SIZE] = {0};
                char line_num[CYHAL_GPIO_LINENUM_SIZE] = {0};
                switch ( toupper( argv[i][1] ) )
                {
                    case 'R':
                        p_gpiochip = p_autobaud_cfg->bt_reg_on_off.p_gpiochip;
                        p_line_num = p_autobaud_cfg->bt_reg_on_off.line_num;
                        r_exist = 1;
                        break;
                    case 'H':
                        p_gpiochip = p_wake_on_ble_cfg->host_wake_args.gpio_event.p_gpiochip;
                        p_line_num = p_wake_on_ble_cfg->host_wake_args.gpio_event.line_num;
                        break;
                    case 'W':
                        p_gpiochip = p_wake_on_ble_cfg->dev_wake.p_gpiochip;
                        p_line_num = p_wake_on_ble_cfg->dev_wake.line_num;
                        break;
                }
                if (parse_gpio(&i, argv, gpiochip, line_num) == PARSE_ERROR)
                {
                    print_usage( app_name );
                    return PARSE_ERROR;
                }
                if(p_gpiochip != NULL && p_line_num != NULL)
                {
                    memcpy(p_gpiochip, gpiochip, strlen(gpiochip));
                    memcpy(p_line_num, line_num, strlen(line_num));
                } else {
                    print_usage( app_name );
                    return PARSE_ERROR;
                }
            }
            else
            {
                TRACE_ERR("BT GPIO line not found in Command line options.\n");
                print_usage( app_name );
                return PARSE_ERROR;
            }
            break;
	}
    }
    if(r_exist)
    {
        if(!p_exist)
        {   
            return PARSE_ERROR;
        }
    }

    return PARSE_SUCCESS;
}

/******************************************************************************
 * Function Name: parse_gpio()
 ******************************************************************************
 * Summary:
 *   parse the user input gpio name and line number 
 *
 * Parameters:
 *   int *i:         index of argv
 *   char **argv:    argv from main, include user input parameter at command line
 *   char *gpiochip: gpiochip name
 *   char *lin_num:  gpiochip offset
 *
 * Return:
 *   int : 
 *       PARSE_ERROR 
 *       PARSE_SUCCESS 
 *
 *****************************************************************************/
static int parse_gpio(int *idx, char **argv, char *gpiochip, char *line_num)
{
    if(strlen(argv[++(*idx)]) > CYHAL_GPIOCHIP_NAME_SIZE)
    {
        TRACE_ERR("GPIO chip name is too long.\n" );
        return PARSE_ERROR;
    }
    strncpy(gpiochip, argv[*idx], CYHAL_GPIOCHIP_NAME_SIZE - 1);

    if(strlen(argv[++(*idx)]) > CYHAL_GPIO_LINENUM_SIZE)
    {
        TRACE_ERR("GPIO number is too long.\n" );
        return PARSE_ERROR;
    }
    strncpy(line_num, argv[*idx], CYHAL_GPIO_LINENUM_SIZE - 1);

    if(strlen(gpiochip) + strlen(line_num) > CYHAL_GPIO_BUFFER_SIZE - CYHAL_GPIO_FIXED_STR_SIZE)
    {
        TRACE_ERR("GPIO buffer is not enough.\n" );
        return PARSE_ERROR;
    }
    return PARSE_SUCCESS;
}

/*
* Function Name: isDigit()
 ******************************************************************************
 * Summary:
 *   check the user input gpio line numer is digit
 *
 * Parameters:
 *   char *line:  gpiochip offset
 *
 * Return:
 *   BOOL32:
 *         WICED_TRUE
 *         WICED_FALSE
*/
static BOOL32 isDigit(char *line)
{
    for(int i = 0; i < strlen(line); i++)
    {
        if(isdigit(line[i]) == WICED_FALSE)
        {
            return WICED_FALSE;
        }
    }
    return WICED_TRUE;
}
