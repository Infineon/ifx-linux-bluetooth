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

/*****************************************************************************
 **
 **  Name:          hci_uart_linux.c
 **
 **  Description: hanlde hci uart communication.
 **
 ******************************************************************************/

#include <time.h>
#include <sys/timeb.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <semaphore.h>

#include "wiced_bt_types.h"
#include "wiced_memory.h"
#include "userial.h"
#include <ctype.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <assert.h>

#include "data_types.h"
#include "platform_linux.h"
#include "sco_process.h"
#include "hci_uart_linux.h"
#include "btttysdio_ioctl.h"
#include "log.h"

#define HCI_COMM_PORT_BUFSIZE		4096

#ifdef DEBUG_UART_RX
#define MAX_DEBUG_RX_BUFSIZE (1 << 10)
typedef struct{
    int fd;
    uint8_t buf[MAX_DEBUG_RX_BUFSIZE];
    uint32_t size_in_use;
}debug_rx_t;

debug_rx_t debug_rx;
#endif

#ifndef RX_PKT_NUM
#define RX_PKT_NUM          (10U)   //this number can change.
#endif
#define RX_HEAP_SIZE        sizeof(tUART_RX)*(RX_PKT_NUM)

static tUART_RX rx_pkt;
static void*  uartRecv (void *p);
static BOOL32 rx_queue_ready = FALSE;
static wiced_bt_buffer_q_t rx_queue;
static wiced_bt_lock_t rx_qlock;
static pthread_mutex_t rx_mutex = PTHREAD_MUTEX_INITIALIZER;

static wiced_bt_pool_t* hci_rx_pool;
static wiced_bt_lock_t hci_rx_pool_lock;
static pthread_mutex_t hci_rx_pool_mutex = PTHREAD_MUTEX_INITIALIZER;

static wiced_bool_t ProcessChar (uint8_t in_byte);
static void pktRcvd (void);
static void* hciRcvd (void*);
static void ClosePort(void);

extern void ProcessEventFromHCI (uint8_t *pData, uint32_t length);
extern void ProcessAclFromHCI (uint8_t *pData, uint32_t length);
extern void ProcessScoFromHCI(uint8_t *pData, uint32_t length);
extern void ProcessDiagFromHCI(uint8_t* pData, uint32_t length);
extern void ProcessIsocFromHCI (uint8_t *pData, uint32_t length);

BOOL32 uart_reconfigure(uint32_t uBaudRate);

typedef struct
{
    /* Name of the opened device (for later usage) */
    char devname[20];
    /* File descriptor of the serial device */
    int fd;
    /* Indicate what kind of device is opened */
    uint8_t devicetype;
    /* Thread to use for serial read wait */
    pthread_t read_thread;
    /* Open configuration */
    tUSERIAL_OPEN_CFG cfg;
    /* Serial device callback when data is received */
    /* Thread to use for process HCI pkt */
    pthread_t hci_thread;

    /* semaphores for hci rx_deque and rx_enque */
    sem_t rx_sem;

} tUSERIAL_CB;

static tUSERIAL_CB userial_cb;

int devid = -1;

void wait_for_uart_recv_thread()
{
    pthread_join(userial_cb.read_thread, NULL);
}
BOOL32 uartInit (char *pPortName, uint32_t uBaudRate)
{
    struct sched_param param;
    int policy;
    int flags;

    memset(&userial_cb, 0, sizeof(userial_cb));
    userial_cb.fd = -1;
    userial_cb.cfg.fmt = USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1;
    userial_cb.cfg.fc = 0;
    userial_cb.cfg.buf = USERIAL_BUF_BYTE;

    /* Save the open parameters */
    strncpy(userial_cb.devname, pPortName, sizeof(userial_cb.devname)-1);

    if (strlen(userial_cb.devname) == 0)
    {
        strncpy(userial_cb.devname, "/dev/ttyUSB0", sizeof(userial_cb.devname)-1);
    }

    flags = O_RDWR;

    /* Try to open the device depending on the device type */
    userial_cb.fd = open(userial_cb.devname, flags);
    if (userial_cb.fd < 0)
    {
        TRACE_ERR("open(%s) failed(%d)", userial_cb.devname, userial_cb.fd);
        return FALSE;
    }
    devid = userial_cb.fd;

    if(!uart_reconfigure(uBaudRate))
    {
        TRACE_ERR("uart_reconfigure failed");
        return FALSE;
    }

    sem_init(&userial_cb.rx_sem, 0, 0);

    pthread_attr_t thread_attr;
    pthread_attr_init(&thread_attr);
    pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);

    if (pthread_create(&(userial_cb.read_thread), &thread_attr, uartRecv, NULL) < 0)
    {
        TRACE_ERR("pthread_create failed");
        return FALSE;
    }

    if (pthread_getschedparam(userial_cb.read_thread, &policy, &param) == 0)
    {
        policy = SCHED_FIFO;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        TRACE_LOG("sched priority %d\n", param.sched_priority);
        pthread_setschedparam(userial_cb.read_thread, policy, &param);
    }
    sleep(1);

#ifdef DEBUG_UART_RX
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        char filename[100];

        TRACE_LOG("Seconds since Jan. 1, 1970: %ld\n", tv.tv_sec);

        snprintf(filename, sizeof(filename),"my_debug_rx_%u",tv.tv_sec);

        debug_rx.fd = open(filename, O_WRONLY|O_CREAT);
        TRACE_LOG("debug uart rx %s fd %u \n", filename, debug_rx.fd);
    }
#endif

    return (TRUE);
}


#ifdef DEBUG_UART_RX
void write_to_debug_rx(uint8_t *p_data, int len)
{
    if(debug_rx.fd == -1)
    {
        return;
    }

    while(len)
    {
        int remaining = MAX_DEBUG_RX_BUFSIZE - debug_rx.size_in_use;
        int to_copy = MIN(len, remaining);

        if(to_copy){
            memcpy(debug_rx.buf + debug_rx.size_in_use, p_data, to_copy);
            debug_rx.size_in_use += to_copy;
            len -= to_copy;
        }

        if(debug_rx.size_in_use == MAX_DEBUG_RX_BUFSIZE){
            write(debug_rx.fd, debug_rx.buf, debug_rx.size_in_use);
            debug_rx.size_in_use = 0;
        }
    }
}

void flush_debug_rx(void)
{
    TRACE_LOG("closing %u", debug_rx.fd);
    if(debug_rx.fd != -1)
    {
        write(debug_rx.fd, debug_rx.buf, debug_rx.size_in_use);
        fclose(debug_rx.fd);
    }
    exit(-1);
}

#endif

/*******************************************************************************
 **
 ** Function:   p_mutex_lock
 **
 ** Description:
 **     linux layer mutex lock for any api need use mutex lock
 **
 ** Parametre:
 **     void* p_lock_context
 ** Returns
 **     void
 *******************************************************************************/
static void p_mutex_lock(void* p_lock_context)
{
    pthread_mutex_lock(p_lock_context);
}

/*******************************************************************************
 **
 ** Function:   p_mutex_unlock
 **
 ** Description:
 **     linux layer mutex lock for any api need use mutex unlock
 **
 ** Parametre:
 **     void* p_lock_context
 ** Returns
 **     void
 *******************************************************************************/
static void p_mutex_unlock(void* p_lock_context)
{
    pthread_mutex_unlock(p_lock_context);
}

/*******************************************************************************
 **
 ** Function:   rx_enqueue
 **
 ** Description:
 **     rx_queue enqueue, get buffer from pool, copy data into buffer, put the 
 **     buffer into queue, reset the rx_pkt buffer. 
 **
 ** Parametre:
 **     void
 ** Returns
 **     void
 *******************************************************************************/
static void rx_enqueue(void)
{
    if (wiced_bt_get_pool_free_count(hci_rx_pool) == 0)
    {
        TRACE_ERR("!!! No Free Pool, will LOST HCI, force halt \n");
        assert(FALSE);
    }

    wiced_bt_buffer_t* p_rx_pkt = wiced_bt_get_buffer_from_pool(hci_rx_pool);

    assert(p_rx_pkt != NULL);

    memcpy(p_rx_pkt, &rx_pkt, sizeof(tUART_RX));

    wiced_bt_enqueue(&rx_queue, p_rx_pkt);

    memset (&rx_pkt, 0, sizeof (rx_pkt));

    rx_pkt.state = WAIT_TYPE;

    sem_post(&userial_cb.rx_sem);
}

/*******************************************************************************
 **
 ** Function:   rx_dequeue
 **
 ** Description:
 **     rx_queue dequeue, block when queue empty, get the buffer from queue,
 **     copy data from buffer to hci_pkt, release the buffer. 
 **
 ** Parametre:
 **     wiced_bt_buffer_t* p_hci_pkt
 ** Returns
 **     void
 *******************************************************************************/
static void rx_dequeue(wiced_bt_buffer_t* p_hci_pkt)
{
    sem_wait(&userial_cb.rx_sem);

    wiced_bt_buffer_t* p_rx_pkt = wiced_bt_dequeue(&rx_queue);

    memcpy(p_hci_pkt, p_rx_pkt, sizeof(tUART_RX));

    wiced_bt_free_buffer(p_rx_pkt);
}

/*******************************************************************************
 **
 ** Function:   init_rx_queue
 **
 ** Description:
 **     init the hci_rx_pool_lock for pool use
 **     init rx_qlock for rx_queue use
 **     create a thread for process hci pkt from queue
 **     use macro HCI_RX_QUEUE to enable
 ** Parametre:
 **     void 
 ** Returns
 **     BOOL32 
 *******************************************************************************/
BOOL32 init_rx_queue(void)
{
#ifdef HCI_RX_QUEUE
    TRACE_LOG("init\n");
    if (init_sco_queue() == FALSE)
    {
        TRACE_ERR("init sco queue fail\n");
        return FALSE;
    }

    //init hci_rx_pool
    hci_rx_pool_lock.p_lock_context = (void*)&hci_rx_pool_mutex;
    hci_rx_pool_lock.pf_lock_func = p_mutex_lock;
    hci_rx_pool_lock.pf_unlock_func = p_mutex_unlock;
    hci_rx_pool = wiced_bt_create_pool("hci_rx_pool", sizeof(tUART_RX), RX_PKT_NUM, &hci_rx_pool_lock);

    //init queue
    rx_qlock.p_lock_context = &rx_mutex;
    rx_qlock.pf_lock_func = p_mutex_lock;
    rx_qlock.pf_unlock_func = p_mutex_unlock;
    wiced_bt_init_q(&rx_queue, &rx_qlock);
    rx_queue_ready = TRUE;

    pthread_attr_t thread_attr;
    pthread_attr_init(&thread_attr);
    pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);

    if (pthread_create(&(userial_cb.hci_thread), &thread_attr, hciRcvd, NULL) < 0)
    {
        TRACE_ERR("pthread_create failed\n");
        return FALSE;
    }
#endif
    return TRUE;
}

void*  uartRecv (void *ppp)
{
    int		dwBytesRead = 0;
    uint16_t		i;
    uint8_t     scratchPad[2000];

    /* loop forever, polling for serial data */
    while (userial_cb.fd != -1)
    {
        // read the data (read up to a quarter of a buffer each time)
        dwBytesRead = read(userial_cb.fd, scratchPad, HCI_COMM_PORT_BUFSIZE / 4);
        if (dwBytesRead < 0)
        {
            TRACE_LOG("uartRecv: Readfile failed 0x%x\n", errno);
            break;
        }

        if (userial_cb.fd == -1)
            break;

        // send it on up the stack
        if (dwBytesRead)
        {
#ifdef DEBUG_UART_RX
            write_to_debug_rx(scratchPad, dwBytesRead);
#endif
            for (i = 0; i < dwBytesRead; i++)
            {
                if (ProcessChar(scratchPad[i]) == FALSE){
                    break;
                }
            }
        }
    }

    ClosePort();
    return NULL;

}

/*******************************************************************************
 **
 ** Function            ProcessChar
 **
 ** Description         Parse HCI packets from Controller
 **
 ** Returns             TRUE:   Get HCI packets successful 
 **                     FALSE:  Get HCI packets failed, error in HCI packets 
 **
 *******************************************************************************/
static wiced_bool_t ProcessChar (uint8_t in_byte)
{
    switch (rx_pkt.state)
    {
      case WAIT_TYPE:
        rx_pkt.len = rx_pkt.hci_len = 0;
        switch (in_byte)
        {
          case HCIT_TYPE_ACL_DATA:
            rx_pkt.state = WAIT_HANDLE_1;
            rx_pkt.type  = HCIT_TYPE_ACL_DATA;
            break;

          case HCIT_TYPE_SCO_DATA:
            rx_pkt.state = WAIT_HANDLE_1;
            rx_pkt.type  = HCIT_TYPE_SCO_DATA;
            break;

          case HCIT_TYPE_EVENT:
            rx_pkt.state = WAIT_OPCODE_1;
            rx_pkt.type  = HCIT_TYPE_EVENT;
            break;

          case HCIT_TYPE_WICED_HCI:
              rx_pkt.state = WAIT_HANDLE_1;
              rx_pkt.type  = HCIT_TYPE_WICED_HCI;
              break;

          case HCIT_TYPE_DIAG:
              rx_pkt.state = WAIT_DATA;
              rx_pkt.hci_len = 63;
              rx_pkt.type  = HCIT_TYPE_DIAG;
              break;

          case HCIT_TYPE_ISOC_DATA:
              rx_pkt.state = WAIT_HANDLE_1;
              rx_pkt.type  = HCIT_TYPE_ISOC_DATA;
              break;

          default:
#ifdef DEBUG_UART_RX
            flush_debug_rx();
#endif
            TRACE_ERR("ProcessChar: bad HCI H4 type %d\n", in_byte);
            tcflush(userial_cb.fd, TCIFLUSH);
            return FALSE;
        }
        return TRUE;

      case WAIT_OPCODE_1:
        rx_pkt.data[rx_pkt.len++] = in_byte;
        rx_pkt.state = WAIT_LEN_1;
        return TRUE;

      case WAIT_HANDLE_1:
        rx_pkt.data[rx_pkt.len++] = in_byte;
        rx_pkt.state = WAIT_HANDLE_2;
        return TRUE;

      case WAIT_HANDLE_2:
        rx_pkt.data[rx_pkt.len++] = in_byte;
        rx_pkt.state = WAIT_LEN_1;
        return TRUE;

      case WAIT_LEN_1:
        rx_pkt.data[rx_pkt.len++] = in_byte;
        rx_pkt.hci_len = in_byte;

        if ( (rx_pkt.type != HCIT_TYPE_ACL_DATA) && (rx_pkt.type != HCIT_TYPE_WICED_HCI) && (rx_pkt.type != HCIT_TYPE_ISOC_DATA) )
        {
            if (rx_pkt.hci_len == 0)
                pktRcvd();
            else
                rx_pkt.state = WAIT_DATA;
        }
        else
            rx_pkt.state = WAIT_LEN_2;
        break;

      case WAIT_LEN_2:
        rx_pkt.data[rx_pkt.len++] = in_byte;
        rx_pkt.hci_len += (in_byte << 8);

        if (rx_pkt.type == HCIT_TYPE_ISOC_DATA)
        {
            rx_pkt.hci_len &= 0x3FFF; // 14 bit for data length, 2 bit RFU
        }

        if (rx_pkt.hci_len == 0)
            pktRcvd();
        else
            rx_pkt.state = WAIT_DATA;
        break;

      case WAIT_DATA:
        rx_pkt.data[rx_pkt.len++] = in_byte;

        if (--rx_pkt.hci_len == 0)
            pktRcvd();
        break;
    }
    return TRUE;
}

/*******************************************************************************
 **
 ** Function:   hciRcvd 
 **
 ** Description:
 **     hci Pkt process thread, get the pkt from rx_queue, process it    
 ** Parametre:
 **     void *p: no use
 ** Returns
 **     void *
 *******************************************************************************/
static void* hciRcvd (void *p)
{
    tUART_RX hci_pkt = {0};
    while (userial_cb.fd != -1)
    {
        rx_dequeue(&hci_pkt);
        // We only care about event packets
        if (hci_pkt.type == HCIT_TYPE_EVENT)
        {
            ProcessEventFromHCI (hci_pkt.data, hci_pkt.len);
        }
        else if (hci_pkt.type == HCIT_TYPE_ACL_DATA)
        {
            ProcessAclFromHCI (hci_pkt.data, hci_pkt.len);
        }
        else if (hci_pkt.type == HCIT_TYPE_DIAG)
        {
            ProcessDiagFromHCI(hci_pkt.data, hci_pkt.len);
        }
#ifdef ENABLE_SCO_QUEUE 
        else if (hci_pkt.type == HCIT_TYPE_SCO_DATA || hci_pkt.type == HCIT_TYPE_ISOC_DATA)
        {
            if (sco_get_queue_ready())
            {
                sco_enqueue(&hci_pkt);
            }
            else
            {
                assert(sco_get_queue_ready() == TRUE);
            }
        }
#else
        else if (hci_pkt.type == HCIT_TYPE_SCO_DATA)
        {
            ProcessScoFromHCI(hci_pkt.data, hci_pkt.len);
        }
        else if (hci_pkt.type == HCIT_TYPE_ISOC_DATA)
        {
            ProcessIsocFromHCI(hci_pkt.data, hci_pkt.len);
        }
#endif
        else if (hci_pkt.type == HCIT_TYPE_WICED_HCI)
        {
            TRACE_ERR("!!!!Dropping WICED HCI Packet Length: %u\n", hci_pkt.len);
        }
        else
        {
            TRACE_ERR("!!!!Unexpected HCI H4 Packet Type: %u  Length: %u\n", hci_pkt.type, hci_pkt.len);
        }
        memset (&hci_pkt, 0, sizeof (tUART_RX));
    }
    TRACE_ERR("hci recv thread end\n");
    return NULL;
}

/****************************************************************************
**
**  Function Name:  pktRcvd
*/
static void pktRcvd (void)
{
    if (rx_queue_ready == TRUE)
    {
        rx_enqueue();
        return;
    }
    // Reset state
    rx_pkt.state = WAIT_TYPE;
    // We only care about event packets
    if (rx_pkt.type == HCIT_TYPE_EVENT)
    {
        ProcessEventFromHCI (rx_pkt.data, rx_pkt.len);
    }
    else if (rx_pkt.type == HCIT_TYPE_ACL_DATA)
    {
        ProcessAclFromHCI (rx_pkt.data, rx_pkt.len);
    }
    else if (rx_pkt.type == HCIT_TYPE_DIAG)
    {
        ProcessDiagFromHCI(rx_pkt.data, rx_pkt.len);
    }
    else if (rx_pkt.type == HCIT_TYPE_SCO_DATA)
    {
        ProcessScoFromHCI(rx_pkt.data, rx_pkt.len);
    }
    else if (rx_pkt.type == HCIT_TYPE_ISOC_DATA)
    {
        ProcessIsocFromHCI(rx_pkt.data, rx_pkt.len);
    }
    else if (rx_pkt.type == HCIT_TYPE_WICED_HCI)
    {
        TRACE_LOG("!!!!Dropping WICED HCI Packet Length: %u\n", rx_pkt.len);
    }
    else
    {
        TRACE_ERR("!!!!Unexpected HCI H4 Packet Type: %u  Length: %u\n", rx_pkt.type, rx_pkt.len);
    }
    memset (rx_pkt.data, 0, sizeof (rx_pkt.data));
}

static void ClosePort(void)
{
    int fd = userial_cb.fd;

    TRACE_LOG("ClosePort fd %d", fd);

    /* if not already closed */
    if (fd != -1)
    {
        userial_cb.fd = -1;
        close(fd);
    }

}


void uartSend (uint32_t bIsCommand, uint8_t *pData, uint32_t len)
{
    uint16_t		dwBytesWritten = 0;
    uint16_t		dwBytesToWrite = len + 1;
    ssize_t      result;
    uint8_t     buff[1200];
    uint8_t     *p_buff = buff;

    // Add in H4 packet type
    if (bIsCommand == 0)
        buff[0] = HCIT_TYPE_ACL_DATA;
    else if (bIsCommand == 0xDD)
        buff[0] = HCIT_TYPE_SCO_DATA;
    else if (bIsCommand == 0x7)
        buff[0] = HCIT_TYPE_DIAG;
    else if (bIsCommand == 0x5)
        buff[0] = HCIT_TYPE_ISOC_DATA;
    else
        buff[0] = HCIT_TYPE_COMMAND;

    memcpy (&buff[1], pData, len);

    // Verify there is a valid port to send out on
    if (userial_cb.fd == -1)
    {
        TRACE_ERR("fd is -1\n");
        return;
    }

    // Send the data out the serial port, and verify it all gets sent
    do
    {
        result = write(userial_cb.fd, p_buff, dwBytesToWrite);

        if (result < 0)
        {
            TRACE_ERR("uartSend: Writefile failed: 0x%x\n", errno);

            ClosePort();
            return;
        }
        dwBytesWritten = result;

        p_buff         += dwBytesWritten;
        dwBytesToWrite -= dwBytesWritten;

    } while (dwBytesToWrite != 0);

}

#ifdef PRINT_PACKET
void dumpHex (char *p_title, uint8_t *p, uint32_t len)
{
    uint32_t  xx, yy;
    char    buff1[100], buff2[20];

    if (p_title)
    {
        TRACE_LOG("%s   Len: %u\n", p_title, len);
    }

    memset (buff2, ' ', 16);
    buff2[16] = 0;

    yy = sprintf (buff1, "%04x: ", 0);
    for (xx = 0; xx < len; xx++)
    {
        if ( (xx) && ((xx & 15) == 0) )
        {
            TRACE_LOG("    %s  %s\n", buff1, buff2);
            yy = sprintf(buff1, "%04x: ", xx);
            memset (buff2, ' ', 16);
        }
        yy += sprintf (&buff1[yy], "%02x ", *p);

        if ((*p >= ' ') && (*p <= 'z'))
            buff2[xx & 15] = *p;
        else
            buff2[xx & 15] = '.';

        p++;
    }

    /* Pad out the remainder */
    for ( ; ; xx++)
    {
        if ((xx & 15) == 0)
        {
            TRACE_LOG("    %s  %s\n", buff1, buff2);
            break;
        }
        yy += sprintf (&buff1[yy], "   ");
    }
}
#endif

/******************************************************************************
 * Function Name: uart_reconfigure(uint32_t)
 *******************************************************************************
 * Summary:
 *   check input baud rate and set to available baud rate first then  
 *   reconfigure uart baud rate
 *
 * Parameters:
 *   uint32_t uBaudRate: BaudRate for reconfigure UART 
 *
 * Return BOOL32:
 *   TRUE: Success 
     FALSE: Error 
 *
 ******************************************************************************/
BOOL32 uart_reconfigure(uint32_t uBaudRate)
{
    uint32_t baud;
    uint8_t data_bits;
    uint16_t parity;
    uint8_t stop_bits;
    struct termios termios;

    if(uBaudRate < 921600)
    {
        userial_cb.cfg.baud = USERIAL_BAUD_115200;
        baud = B115200;
        uBaudRate = 115200;
    }
    else if (uBaudRate < 2000000)
    {
        userial_cb.cfg.baud = USERIAL_BAUD_921600;
        baud = B921600;
        uBaudRate = 921600;
    }
    else if (uBaudRate < 3000000)
    {
        userial_cb.cfg.baud = USERIAL_BAUD_2M;
        baud = B2000000;
        uBaudRate = 2000000;
    }
    else if (uBaudRate < 4000000)
    {
        userial_cb.cfg.baud = USERIAL_BAUD_3M;
        baud = B3000000;
        uBaudRate = 3000000;
    }
    else
    {
        userial_cb.cfg.baud = USERIAL_BAUD_4M;
        baud = B4000000;
        uBaudRate = 4000000;
    }

    if(userial_cb.cfg.fmt & USERIAL_DATABITS_8)
        data_bits = CS8;
    else if(userial_cb.cfg.fmt & USERIAL_DATABITS_7)
        data_bits = CS7;
    else if(userial_cb.cfg.fmt & USERIAL_DATABITS_6)
        data_bits = CS6;
    else if(userial_cb.cfg.fmt & USERIAL_DATABITS_5)
        data_bits = CS5;
    else
    {
        printf("ERROR: serial_configure bad size format:0x%x", userial_cb.cfg.fmt);
        return FALSE;
    }

    if(userial_cb.cfg.fmt & USERIAL_PARITY_NONE)
        parity = 0;
    else if(userial_cb.cfg.fmt & USERIAL_PARITY_EVEN)
        parity = PARENB;
    else if(userial_cb.cfg.fmt & USERIAL_PARITY_ODD)
        parity = (PARENB | PARODD);
    else
    {
        printf("ERROR: serial_configure bad parity format:0x%x", userial_cb.cfg.fmt);
        return FALSE;
    }

    if(userial_cb.cfg.fmt & USERIAL_STOPBITS_1)
        stop_bits = 0;
    else if (userial_cb.cfg.fmt & USERIAL_STOPBITS_2)
    {
        stop_bits = CSTOPB;
    }
    else
    {
        printf("ERROR: serial_configure bad stop format:0x%x", userial_cb.cfg.fmt);
        return FALSE;
    }

    tcflush(userial_cb.fd, TCIOFLUSH);
    tcgetattr(userial_cb.fd, &termios);

    /* Configure in default raw mode */
    cfmakeraw(&termios);

    /* Clear out what can be overriden */
    termios.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
    termios.c_cflag |= parity | data_bits | stop_bits;

    /* use hw  flow control */
    userial_cb.cfg.fc = USERIAL_FC_HW;
    if (userial_cb.cfg.fc == USERIAL_FC_HW)
    {
        termios.c_cflag |= CRTSCTS;
    }


    termios.c_cflag &= ~CSTOPB; // 1 stop bit
    tcsetattr(userial_cb.fd, TCSANOW, &termios);

    tcflush(userial_cb.fd, TCIOFLUSH);

    tcsetattr(userial_cb.fd, TCSANOW, &termios);

    tcflush(userial_cb.fd, TCIOFLUSH);
    tcflush(userial_cb.fd, TCIOFLUSH);

    cfsetospeed(&termios, baud);
    cfsetispeed(&termios, baud);
    tcsetattr(userial_cb.fd, TCSANOW, &termios);

    TRACE_LOG("uart_reconfigure: set baudrate:%d\n", uBaudRate);
    return TRUE;
}

/******************************************************************************
 * Function Name: checkAppBaudRate(uint32_t uBaudRate)
 *******************************************************************************
 * Summary:
 *   After firmware download complete, application will reset the uart baud rate if need.
 *   Check user input Application running baud rate, and return the avaliable baud rate.
 *   The App baud rate speed is limited by BT chip.
 *   supported baud rate:
 *      115200, 921600, 2000000, 3000000, 4000000 
 *
 * Parameters:
 *      uint32_t uBaudRate: user input CE running baud rate 
 *
 * Return:
 *      uint32_t: avaliable baud rate
 *
 ******************************************************************************/
uint32_t checkAppBaudRate(uint32_t uBaudRate) 
{
    uint32_t appBaudRate = DEFAULT_APP_BAUD_RATE;    
    TRACE_LOG("input app Baudrate:%d\n", uBaudRate);
    if(uBaudRate < 921600)
    {
        appBaudRate = 115200;
    }
    else if (uBaudRate < 2000000)
    {
        appBaudRate = 921600;
    }
    else if (uBaudRate < 3000000)
    {
        appBaudRate = 2000000;
    }
    else if (uBaudRate < 4000000)
    {
        appBaudRate = 3000000;
    }
    else
    {
        appBaudRate = 4000000;
    }
    TRACE_LOG("appBaudrate:%d\n", appBaudRate);
    return appBaudRate;
}

/******************************************************************************
 * Function Name: checkPatchBaudRate(uint32_t uBaudRate)
 *******************************************************************************
 * Summary:
 *   Check user input patch Firmware baud rate, and return avaliable baud rate.  
 *   Patch baud rate for app download firmware only, when app running download  
 *   firmware to BT chip, use patch firmware baud rate to setup uart speed.
 *   The patch baud rate speed is limited by BT chip. 
 *   This function check the input patch baud rate and adjust to the avaliable
 *   speed.
 *   supported baud rate:
 *       115200, 921600, 3000000  
 *
 * Parameters:
 *   uint32_t uBaudRate: user input patch firmware baud rate 
 *
 * Return:
 *      uint32_t: avaliable baud rate 
 *
 ******************************************************************************/
uint32_t checkPatchBaudRate(uint32_t uBaudRate) 
{
    TRACE_LOG("input patchBaudRate:%d\n", uBaudRate);
    uint32_t patchBaudRate = DEFAULT_PATCH_BAUD_RATE;    
    if(uBaudRate < 921600)
    {
        patchBaudRate = 115200;
    }
    else if(uBaudRate < 3000000)
    {
        patchBaudRate = 921600;
    }
    else if(uBaudRate == 3000000)
    {
        patchBaudRate = 3000000;
    }
    else
    {
        patchBaudRate = DEFAULT_PATCH_BAUD_RATE;
    }

    TRACE_LOG("patchBaudRate:%d\n", patchBaudRate);
    return patchBaudRate;
}

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
BOOL32 bt_ioctl_interface(eBTTTYSDIO_IOCTL bt_ioctl)
{
    if (userial_cb.fd < 0)
    {
        TRACE_ERR("userial_cb.fd:%d\n", userial_cb.fd);
        return FALSE;
    }
    switch(bt_ioctl) 
    {
        case BTTTYSDIO_BUS_CLK_DISABLE:
            TRACE_LOG("SDIO_BUS_CLK_DISBLE\n");
            if (ioctl(userial_cb.fd, BTTTYSDIO_IOC_CLK_DIS) < 0)
            {
                TRACE_ERR("ioctl BTTTYSDIO_IOC_CLK_DIS error\n");
                return FALSE;
            }
            break;

        case BTTTYSDIO_BUS_CLK_ENABLE:
            TRACE_LOG("SDIO_BUS_CLK_ENABLE\n");
            if (ioctl(userial_cb.fd, BTTTYSDIO_IOC_CLK_ENA) < 0)
            {
                TRACE_ERR("ioctl BTTTYSDIO_IOC_CLK_ENA error\n");
                return FALSE;
            }
            break;
        default:
            TRACE_LOG("Do Nothing\n");
            break;
    }
    return TRUE;
}
