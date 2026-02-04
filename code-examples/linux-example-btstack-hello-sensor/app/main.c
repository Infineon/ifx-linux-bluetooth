/******************************************************************************
* $ Copyright 2022-YEAR Cypress Semiconductor $
******************************************************************************/
/******************************************************************************
 * File Name: main.c
 *
 * Description: Entry file for Throughput GATT server application.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/
/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include "wiced_bt_trace.h"
#include "platform_linux.h"
#include "wiced_bt_cfg.h"
#include "hello_sensor.h"
#include "utils_arg_parser.h"


/******************************************************************************
 *                                EXTERNS
 *****************************************************************************/
/* Stack configuration */
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
/* BLE configuration */
extern wiced_bt_cfg_ble_t hello_sensor_cfg_ble;
/* GATT configuration */
extern wiced_bt_cfg_gatt_t hello_sensor_cfg_gatt;
/* BD_ADDR*/
extern wiced_bt_device_address_t bt_device_address;

/******************************************************************************
 *                                MACROS
 *****************************************************************************/
#ifndef MAX_PATH
#define MAX_PATH        (256)
#endif
 #define LOCAL_BDA_LEN          50
#define IP_ADDR_LEN            16
//#define PARSE_ERROR            -1
#define BT_STACK_HEAP_SIZE     0xF000

/* ***************************************************************************
 *                              GLOBAL VARIABLES
 * **************************************************************************/
wiced_bt_heap_t *p_default_heap   = NULL;

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
 * Function Name: application_start()
 *******************************************************************************
 * Summary:
 *   BT stack initialization function
 *
 * Parameters:
 *   None
 *
 * Return:
 *      None
 *
 ******************************************************************************/
void application_start(void)
{
    wiced_result_t wiced_result = WICED_BT_SUCCESS;

#ifdef WICED_BT_TRACE_ENABLE
    /* Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints */
    /* wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE); */

    /* Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart() */
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );
#endif /* WICED_BT_TRACE_ENABLE */

    /* Register call back and configuration with stack */
    wiced_result = wiced_bt_stack_init(hello_sensor_management_callback, &wiced_bt_cfg_settings);

    WICED_BT_TRACE("AIROC BTStack Hello sensor Start");

    /* Check if stack initialization was successful */
    if (WICED_BT_SUCCESS == wiced_result)
    {
        printf("Bluetooth Stack Initialization Successful \n");
        /* Create a buffer heap, make it the default heap.  */
        p_default_heap = wiced_bt_create_heap("app", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
        if (NULL == p_default_heap)
        {
            printf("create default heap error: size %d\n", BT_STACK_HEAP_SIZE);
            return;
        }
    } 
    else
    {
        printf("Bluetooth Stack Initialization failed!! Exiting App...\n");
        exit(0);
    }
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
void APPLICATION_START(void)
{
    application_start();
}

/******************************************************************************
 * Function Name: main()
 *******************************************************************************
 * Summary:
 *   Application entry function
 *
 * Parameters:
 *   int argc           : argument count
 *  char *argv[]        : list of arguments
 *
 * Return:
 *      None
 *
 ******************************************************************************/
int main(int argc, char *argv[])
{
    /* Parse the arguments */
    int ip = 0;
    arg_parser_arguments_t parsed_args = {0};
    if (PARSE_ERROR == arg_parser_get_args(argc, argv, &parsed_args))
    {
        return PARSE_ERROR;
    }

    memcpy(bt_device_address, parsed_args.local_bda, BD_ADDR_LEN);

    cy_bt_spy_comm_init(parsed_args.is_socket_tcp, parsed_args.spy_inst, parsed_args.peer_ip_addr);
    cy_platform_bluetooth_init(
        parsed_args.patchFile,
        parsed_args.com_port,
        parsed_args.baud_rate,
        parsed_args.patch_baud,
        &parsed_args.p_gpio_cfg.autobaud_cfg);

    do 
    {
        printf ("Hello Sensor Application\n");
        printf ("=============================================\n");
        printf (" Press <ENTER> key to send notification \n");
        printf (" Press 0, then <ENTER> to Exit \n");
        printf ("=============================================\n");
        ip = getchar();
        
        switch (ip)
        {
            case '0':
                /* App exits */
                break;
            default:
                hello_sensor_interrupt_handler();
                break;
        }
    }while (ip != '0');
    
   wiced_bt_delete_heap(p_default_heap);
   return 0;
}
