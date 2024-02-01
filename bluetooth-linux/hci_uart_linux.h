#ifndef HCI_UART_LINUX_H
#define HCI_UART_LINUX_H

// HCI Transport packet types. Each packet sent or
// received must be one of these four types, though if running
// on a host we do not send events, and if running on a controller
// we do not send commands.
#define HCIT_TYPE_COMMAND       1
#define HCIT_TYPE_ACL_DATA      2
#define HCIT_TYPE_SCO_DATA      3
#define HCIT_TYPE_EVENT         4
#define HCIT_TYPE_ISOC_DATA     5
#define HCIT_TYPE_DIAG          7

#define HCIT_TYPE_WICED_HCI     25

typedef enum
{
    WAIT_TYPE, WAIT_OPCODE_1, WAIT_HANDLE_1, WAIT_HANDLE_2, WAIT_LEN_1, WAIT_LEN_2, WAIT_DATA
} eRX_STATE;

typedef struct
{
    uint8_t     data[2000];
    eRX_STATE   state;       // State of the receiver
    uint8_t     type;        // Type of the packet being received
    uint16_t    len;
    uint16_t    hci_len;
}tUART_RX;

typedef enum
{
    BTTTYSDIO_DEFAULT,
    BTTTYSDIO_BUS_CLK_DISABLE,
    BTTTYSDIO_BUS_CLK_ENABLE
} eBTTTYSDIO_IOCTL;

/******************************************************************************
 * Function Name: bt_ioctl_interface()
 *******************************************************************************
 * Summary: porting layer interface of ioctl, this api used by our wiced_exp_lib
 *          for doing btsdio driver ioctl
 *
 * Parameters:
 *  eBTTTYSDIO_IOCTL bt_ioctl
 *
 * Return:
 *  BOOL32
 *
 ******************************************************************************/
BOOL32 bt_ioctl_interface(eBTTTYSDIO_IOCTL);

#endif
