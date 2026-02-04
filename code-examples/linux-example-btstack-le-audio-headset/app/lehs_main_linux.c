/*
 * $ Copyright Cypress Semiconductor $
 */
#ifdef LINUX_PLATFORM

#include "platform_linux.h"
#include "wiced_bt_cfg.h"
#include "utils_arg_parser.h"
#include "app_bt_utils.h"
#include "log.h"

/* App Library includes */

/* BT Stack includes */
#include "wiced_bt_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"

/******************************************************************************
 *                              EXTERNS
 *****************************************************************************/
extern wiced_bt_device_address_t bt_device_address;

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
static int spy_inst = 0;

int get_spy_instance(void);
void set_local_bd_addr(void);

//extern wiced_bt_device_address_t local_bda;

int main(int argc, char *argv[])
{

    arg_parser_arguments_t parsed_args = { 0 };
    cybt_controller_autobaud_config_t parse_autobaud_cfg;

    if (-1 == arg_parser_get_args(argc, argv, &parsed_args))
        return -1;

    spy_inst = parsed_args.spy_inst;
    memcpy(bt_device_address, parsed_args.local_bda, BD_ADDR_LEN);

    /* Initialize Spy TCP/UDP Sockets */
    cy_bt_spy_comm_init(parsed_args.is_socket_tcp, parsed_args.spy_inst, NULL);

    cy_platform_bluetooth_init( parsed_args.patchFile, parsed_args.com_port, parsed_args.baud_rate, parsed_args.patch_baud, &parsed_args.p_gpio_cfg.autobaud_cfg);

    /* Initialize BT HCI */
    //cy_bt_hci_init(parsed_args.com_port, parsed_args.baud_rate, NULL);

    WICED_BT_TRACE("[%s] COM [%s] Baud [%d] Instance [%d]\n", __FUNCTION__, parsed_args.com_port, parsed_args.baud_rate, parsed_args.spy_inst);


    if (parsed_args.patchFile[0])
    {
        TRACE_LOG ("Waiting for downloading patch...");

        wait_controller_reset_ready();

    } else {
        TRACE_LOG ("No patch FW");
    }

    TRACE_LOG(" Linux CE LE-Audio Headset initialization complete...\n" );

    for (;;)
    {
#ifdef CLI_SUPPORT
        extern int le_hs_cli_routine(void);
        if (le_hs_cli_routine() == 0)
        {
            break;
        }
#endif // CLI_SUPPORT
    }

    return 0;
}

int get_spy_instance(void)
{
    return spy_inst;
}

/******************************************************************************
 * Function Name: hci_control_proc_rx_cmd()
 *******************************************************************************
 * Summary:
 *  Function to handle HCI receive
 *
 * Parameters:
 *  uint8_t* p_buffer  : rx buffer
 *  uint32_t length    : rx buffer length
 *
 * Return:
 *  status code
 *
 ******************************************************************************/
uint32_t hci_control_proc_rx_cmd(uint8_t *p_buffer, uint32_t length)
{
    return 0;
}
#endif
