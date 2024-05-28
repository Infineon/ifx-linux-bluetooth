/******************************************************************************
 * (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 *****************************************************************************/
/******************************************************************************
 * File Name: arg_parser.c
 *
 * Description: This is the source file for argument parsing module.
 *
 * Related Document: See README.md
 *
 *****************************************************************************/
/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arg_parser.h"

/******************************************************************************
 *                                MACROS
 *****************************************************************************/
#define WICED_BT_TRACE  printf
#define MAX_PATH        ( 256 )
#define BD_ADDR_LEN     ( 6 )

/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/
static void stream_to_bdaddr( uint8_t *a, uint8_t *p );
static void print_usage( char *p_app_name );
static uint8_t HexDigits( char *p );
static void AsciiToHex( uint8_t *bda, size_t len, char *pBdAddr );

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
    uint8_t *pbda = (uint8_t *)a + BD_ADDR_LEN - 1;
    for ( iCnt = 0; iCnt < BD_ADDR_LEN; iCnt++ )
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
    WICED_BT_TRACE( " \n" );
    WICED_BT_TRACE( "Usage:\n" );
    WICED_BT_TRACE( " \n" );
    WICED_BT_TRACE( "%s -c <comport> -b <baud rate> -d <bda> -i <spy \
                                                instance>\n",
                   p_app_name );
    WICED_BT_TRACE( "\nFor example:\n" );
    WICED_BT_TRACE( "   %s -c COM21 -b 3000000 -i 1 -p \"firmware.hcd\")\n",
                   p_app_name );
    WICED_BT_TRACE( " \n" );
    WICED_BT_TRACE( " c  Local device name, for example COM21 for UART.\n" );
    WICED_BT_TRACE( " b  Baud rate for UART device.\n" );
    WICED_BT_TRACE( " d  Bluetooth Device Address to be programmed"
                    " to the device. A pre-determined address"
                    " is used if this is omitted.\n" );
    WICED_BT_TRACE( " i  BT Spy instance number "
                    "for logging (default is 0)\n" );
    WICED_BT_TRACE( " p  Full file path of .hcd patch file to download to" 
                                                    "device on startup." );
    WICED_BT_TRACE( " f  baud rate to be used for patch download,"
                        " default value is 115200 if not specified" );
    WICED_BT_TRACE( " t  Provide the IP address\n" );
    WICED_BT_TRACE( " s  Enable TCP Socket for data transfer\n" );
    WICED_BT_TRACE( " \n");
    WICED_BT_TRACE( " NOTE: Please provide instance before enabling TCP"
                   " socket to get the updated port number display. "
                   "for eg :  -i 1 -t 127.0.0.1\n ");
    WICED_BT_TRACE( " \n ");
    return;
}

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
static uint8_t HexDigits( char *p )
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

/******************************************************************************
 * Function Name: AsciiToHex()
 ******************************************************************************
 * Summary:
 *   Function convert Ascii to Hex.
 *
 * Parameters:
 *   uint8_t *bda             : Hex BDA
 *   size_t len               : BDA Length
 *   char *pBdAddr            : Ascii number
 *
 * Return:
 *  None
 *
 *****************************************************************************/
static void AsciiToHex( uint8_t *bda, size_t len, char *pBdAddr )
{
    int count = 0; /* Counter variable for BDA length */

    if ( ( pBdAddr == NULL ) || ( bda == NULL ) )
    {
        return;
    }

    memset( bda, 0, len );
    count = strlen( pBdAddr ) / 2;

    for ( int i = 0; i < (int)len && i < count; ++i )
    {
        bda[len - i - 1] = HexDigits( pBdAddr );
        pBdAddr += 2;
    }
}

/******************************************************************************
 * Function Name: arg_parser_get_args()
 ******************************************************************************
 * Summary:
 *   Function to parse command line arguments.
 *
 * Parameters:
 *   uint8_t argc             : Argument count
 *   size_t *argv             : Arguments
 *   char *device             : Device name
 *   uint8_t *local_bda       : Local BD address
 *   uint32_t *baud           : HCI Baudrate
 *   int *spy_inst            : BT SPY Instance
 *   char *peer_ip_addr       : Peer IP address
 *   uint8_t *is_socket_tcp   : Socket is TCP or not
 *   char *patchFile          : Firmware patch file
 *   uint32_t *patch_baud     : Patch baudrate
 *
 * Return:
 *  Parsing status
 *
 *****************************************************************************/
int arg_parser_get_args( int argc,
                        char **argv,
                        char *device,
                        uint8_t *local_bda,
                        uint32_t *baud,
                        int *spy_inst,
                        char *peer_ip_addr,
                        uint8_t *is_socket_tcp,
                        char* patchFile,
                        uint32_t *patch_baud )
{
    char app_name[MAX_PATH]; /* Application name */
    int len = 0; /* Application name length */
    char local_bda_string[50]; /* Local BD address string */

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
        if ( ( argv[i][0] != '-' ) && ( argv[i][0] != '/' ) )
        {
            continue;
        }

        switch ( toupper( argv[i][1] ) )
        {
        case 'C': /* comm port, COMXX */
            if ( argc >= ( i + 2 ) )
            {
                int len = 0;
                len = strlen( argv[i + 1] );
                if ( len >= MAX_PATH )
                    len = MAX_PATH - 1;

                strncpy( device, argv[++i], len );
                device[len] = '\0';
            }
            else
            {
                printf( "No comport found in Command line options.\n" );
                print_usage( app_name );
                i = argc; /* break from for loop */
            }
            break;

        case 'B': /* baud rate */
            if ( argc >= ( i + 2 ) )
            {
                sscanf( argv[++i], "%u", baud );
            }
            else
            {
                printf( "No baud rate found in Command line options.\n" );
                print_usage( app_name );
                i = argc; /* break from for loop */
            }
            break;
        case 'F':     /* patch_download baudrate */
            if ( argc >= ( i + 2 ) )
            {
                sscanf( argv[++i], "%u", patch_baud );
            }
            else
            {
                printf( "No baud rate for patch download found in Command" 
                                                        "line options.\n" );
                print_usage( app_name );
                i = argc;     /* break from for loop */
            }
            break;
        case 'P':     /* patch file to download */
            if ( argc >= ( i + 2 ) )
            {
                int len = 0;
                len = strlen( argv[i + 1] );
                if ( len >= MAX_PATH )
                    len = MAX_PATH - 1;

                strncpy( patchFile, argv[++i], len );
                patchFile[len] = '\0';
            }
            else
            {
                printf( "No patch file found in Command line options.\n" );
                print_usage( app_name );
                i = argc;     /* break from for loop */
            }
            break;

        case 'I':
            *spy_inst = atoi( argv[++i] );
            if ( *spy_inst < 0 )
            {
                *spy_inst = 0;
            }
            break;

        case 'D': /* device bda */
            if ( argc >= ( i + 2 ) )
            {
                strncpy(local_bda_string, argv[++i], sizeof(local_bda_string)-1);
                local_bda_string[sizeof(local_bda_string)-1] = '\0';
                uint8_t bda[BD_ADDR_LEN];
                AsciiToHex( bda, 6, local_bda_string );
                stream_to_bdaddr( local_bda, bda );
            }
            else
            {
                printf( "No BDA found in Command line options.\n" );
                print_usage( app_name );
                i = argc; /* break from for loop */
            }
            break;

        case 'T':
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

        case 'S':
            *is_socket_tcp = 1;
            break;
        }
    }
    return PARSE_SUCCESS;
}
