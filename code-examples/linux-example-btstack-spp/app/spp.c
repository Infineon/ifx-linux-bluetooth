/******************************************************************************
 * (c) 2023, Infineon Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Infineon Semiconductor Corporation or one of its
 * subsidiaries ("Infineon") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Infineon hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Infineon's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Infineon.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Infineon
 * reserves the right to make changes to the Software without notice. Infineon
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Infineon does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Infineon product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Infineon's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Infineon against all liability.
 *****************************************************************************/


/******************************************************************************
 * File Name: spp.c
 *
 * Description: This is the source file for SPP CE.
 *
 * Related Document: See README.md
 *
 *****************************************************************************/


/*******************************************************************************
*                               INCLUDES
*******************************************************************************/
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <dirent.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "platform_linux.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "wiced_bt_dev.h"
#include "wiced_memory.h"
#include "wiced_hal_nvram.h"
#include "wiced_spp_int.h"
#include "wiced_bt_sdp.h"
#include "wiced_timer.h"
#include "log.h"
#include "spp.h"
#include <time.h>
#include <signal.h>
#include <semaphore.h>


/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define BT_STACK_HEAP_SIZE                          (0xF000)
#define WICED_EIR_BUF_MAX_SIZE                      (264)
#define SPP_TOTAL_DATA_TO_SEND                      (10000)
#define MAX_TX_RETRY                                (5)
#define PINCODE_LENGTH                              (4)
#define SHM_SIZE                                    (1024)
#define NVRAM_FILE_NAME_LEN                         (26)
#define CONNECTED_INFO_LEN                          (26)
#define LARGE_OUTPUT                                (1000)
#define AWAKE_INTERVAL                              (5)
#define RECOMMAND_SPEED                             (8000)
#define MIN_LATENCY                                 (25000)     /* in us */
#define SEND_ERROR_RECOVERY_TIME                    (8)
#define TERMINAL_WAIT                               (3)
#define CLEAN_BUFFER_TIME                           (4)
#define MIN_COMMAND_BUFFER_LEN                      (12)
#define CASE_RETURN_STR(enum_val) \
    case enum_val:                \
        return #enum_val;


/*******************************************************************************
*                               VARIABLE DEFINITIONS
*******************************************************************************/
typedef struct node
{
    struct node*    next;
    uint8_t         server_idx;
    uint8_t*        p_data;
    int             len;
} Node_t;

typedef struct
{
    uint8_t         tx_buffer[SPP_MAX_PAYLOAD];
    char            connection_info[CONNECTED_INFO_LEN];
    FILE*           com_terminal;
    pthread_t       opening;
    pthread_t       closing;
    uint32_t        tx_retry;
    uint32_t        tx_bytes;
    uint32_t        rx_bytes;
    uint8_t         awake;
    int             tx_offset;
    int             pid;
} tPROT;

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

wiced_bt_heap_t *p_default_heap = NULL;
uint8_t pincode[PINCODE_LENGTH] = {0x30, 0x30, 0x30, 0x30};

/* SHM SEM key is index + 1 -> ack key is SEM key + 0xFF*/
static tPROT port[SUPPORT_PORT_NUM] = {0};
static uint16_t spp_handle[SUPPORT_PORT_NUM] = {0};
static char* comm_data[SUPPORT_PORT_NUM] = {0};
static int comm_ack_sem[SUPPORT_PORT_NUM] = {0};
static Node_t* head = {0};
static Node_t* tail = {0};
struct timespec tx_serial_start_time;
struct timespec tx_serial_end_time;
struct timespec tx_average_start_time[SUPPORT_PORT_NUM] = {0};
struct timespec tx_average_end_time[SUPPORT_PORT_NUM] = {0};
static pthread_t terminal_input = {0};
static pthread_t dispatching = {0};
static double queue_bytes = 0;
static bool enable_loopback = false;
static bool enalbe_terminal = false;
static bool enalbe_rx_log = true;
static uint8_t current_port_num = 1;
static wiced_bt_spp_reg_t* spp_regs = NULL;
static pthread_mutex_t main_console_mutex = {0};
static long created_idx[SUPPORT_PORT_NUM] = {0,1,2,3,4,5};
int shm_id = 0;


/*******************************************************************************
*                               FUNCTION DECLARATIONS
*******************************************************************************/
extern uint16_t wiced_app_cfg_sdp_record_get_size(void);
extern uint8_t* wiced_get_sdp_database(uint8_t num_ports);

wiced_bool_t spp_init(uint8_t num_ports);

void* spp_keep_awake(void* idx);
void* spp_init_terminal(void* idx);
void* spp_close_terminal(void* idx);
void spp_sync_state(bool open, uint8_t server_idx);
void spp_bt_enabled_lock(void);
void spp_start_throughput(void);
void spp_end_throughput(struct timespec* user_start_time, struct timespec* user_end_time);
void spp_server_status_string(char* buffer, uint8_t len);
void spp_send_serial_data(uint8_t server_idx, uint8_t* p_data, uint32_t length);
bool spp_enable_terminal(bool value);
bool spp_send_session_data(uint8_t handle, char* p_data, size_t len);
uint8_t spp_get_current_port_num(void);
uint8_t* spp_port_tx_buffer(uint8_t server_idx);
uint16_t spp_port_handle(uint8_t server_idx);

static const char *spp_get_bt_event_name(wiced_bt_management_evt_t event);
static void spp_write_eir(void);
static void spp_print_bd_address(wiced_bt_device_address_t bdadr);
static void spp_connection_handle_reset(uint16_t handle, uint8_t server_idx);
static void spp_connection_handle_register(uint16_t handle, uint8_t *bda, uint8_t server_idx);
static bool spp_execute_command(const char* command, char* buffer, uint16_t buf_size);
static int spp_find_process_pid(uint8_t server_idx);
static int spp_find_process_pts(uint8_t server_idx, char* buf, int len);
static wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t *p_data, uint32_t data_len, int server_idx);
static wiced_result_t spp_management_callback(
    wiced_bt_management_evt_t event,
    wiced_bt_management_evt_data_t *p_event_data);


/*******************************************************************************
 * Function Name: SPP_RX_CALLBACK
 *******************************************************************************
 * Summary:
 *   SPP RX callback handling coming data for each port
 *
 * Parameters:
 *   n: n port
 *
 ******************************************************************************/
#define SPP_RX_CALLBACK(n) wiced_bool_t src##n(uint16_t handle, uint8_t *p_data, uint32_t data_len)\
{\
    return spp_rx_data_callback(handle, p_data, data_len, n - 1);\
}
/*******************************************************************************
 * Function Name: SPP_CONNECTION_UP_CALLBACK
 *******************************************************************************
 * Summary:
 *   SPP connection up callback for n ports
 *
 * Parameters:
 *   n: n port
 *
 ******************************************************************************/
#define SPP_CONNECTION_UP_CALLBACK(n) void scuc##n(uint16_t handle, uint8_t *bda)\
{\
    spp_connection_handle_register(handle, bda, n - 1);\
}
/*******************************************************************************
 * Function Name: SPP_CONNECTION_DOWN_CALLBACK
 *******************************************************************************
 * Summary:
 *   SPP connection down callback for n ports
 *
 * Parameters:
 *   n: n port
 *
 ******************************************************************************/
#define SPP_CONNECTION_DOWN_CALLBACK(n) void scdc##n(uint16_t handle)\
{\
    spp_connection_handle_reset(handle, n - 1);\
}

#define SPP_CONNECTION_PORT(n) SPP_CONNECTION_UP_CALLBACK(n);SPP_CONNECTION_DOWN_CALLBACK(n);SPP_RX_CALLBACK(n);

SPP_CONNECTION_PORT(1);
SPP_CONNECTION_PORT(2);
SPP_CONNECTION_PORT(3);
SPP_CONNECTION_PORT(4);
SPP_CONNECTION_PORT(5);
SPP_CONNECTION_PORT(6);

/* Function Entities */
static void (*spp_connection_up_callback[SUPPORT_PORT_NUM])(uint16_t, uint8_t*) = {scuc1, scuc2, scuc3, scuc4, scuc5, scuc6};
static void (*spp_connection_down_callback[SUPPORT_PORT_NUM])(uint16_t) = {scdc1, scdc2, scdc3, scdc4, scdc5, scdc6};
wiced_bool_t (*spp_rx_callback[SUPPORT_PORT_NUM])(uint16_t , uint8_t*, uint32_t) = {src1, src2, src3, src4, src5, src6};


/*******************************************************************************
 * Function Name: spp_application_start
 *******************************************************************************
 * Summary:
 *   Set device configuration and start BT stack initialization. The actual
 *   application initialization will happen when stack reports that BT device
 *   is ready.
 *
 ******************************************************************************/
void spp_application_start(void)
{
    pthread_mutex_init(&main_console_mutex, NULL);

    /* 1 - Lock console */
    pthread_mutex_lock(&main_console_mutex);

    /* Menu like -> Use fprintf which has no time and function tag */
    fprintf(stdout, "\n%s\n", "------------------------------SPP Application Start-----------------------------");

    /* Register call back and configuration with stack */
    wiced_result_t wiced_result = wiced_bt_stack_init(spp_management_callback, &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if (WICED_BT_SUCCESS == wiced_result)
    {
        TRACE_LOG("Bluetooth stack initialization succeeded.\n");

        /* Create default heap */
        p_default_heap = wiced_bt_create_heap("default_heap", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
        if (p_default_heap == NULL)
        {
            TRACE_ERR("Create default heap failed: size %d\n", BT_STACK_HEAP_SIZE);
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        TRACE_ERR("Bluetooth stack initialization failed!!\n");
        exit(EXIT_FAILURE);
    }
}

/*******************************************************************************
 * Function Name: spp_sync_state
 *******************************************************************************
 * Summary:
 *   Sync port conneted state and communication board
 *
 * Parameters:
 *   bool open : is called by open process
 *   uint8_t server_idx
 *
 ******************************************************************************/
void spp_sync_state(bool open, uint8_t server_idx)
{
    if(open && !spp_handle[server_idx])
    {
        if (pthread_create(&port[server_idx].closing, NULL, spp_close_terminal,(void*)&created_idx[server_idx]))
        {
            TRACE_LOG("Failed to close communication board, please check it manually.\n");
        }
    }
    if(!open && spp_handle[server_idx])
    {
        if (pthread_create(&port[server_idx].opening, NULL, spp_init_terminal, (void*)&created_idx[server_idx]))
        {
            TRACE_LOG("Failed to open Communication Board -> Reconnect to SERVER%d.\n", server_idx + 1);
        }
    }    
}


/*******************************************************************************
 * Function Name: spp_init_terminal
 *******************************************************************************
 * Summary:
 *   Initiate terminal for connection port
 *
 * Parameters:
 *   void* idx : pointer to server index
 *
 ******************************************************************************/
void* spp_init_terminal(void* idx)
{
    int server_idx = *(int*)idx;
    char pts[100] = {0};
    char path[25] = "/dev/";

    /* If not exist */
    if(!(port[server_idx].pid = spp_find_process_pid(server_idx)) && port[server_idx].pid != -1)
    {
        /* Open terminal, please change to terminal on your platform */
        char command[25] = "./SERVER 1";
        command[9] += server_idx;
        sprintf(command + strlen(command), " %d",getpid());
        if (!fork())
        {
            execlp("lxterminal", "lxterminal", "-e", command, NULL);
        }
    }

    /* pid */
    for(int retry = 5; retry-- && !(port[server_idx].pid = spp_find_process_pid(server_idx)) && port[server_idx].pid != -1;)
    {  
        sleep(1);
    }

    /* pts */
    for(int retry = 5; retry-- && !spp_find_process_pts(server_idx, pts, 100);)
    {
        TRACE_ERR("Checking Communication Board... %d\n", retry);
        sleep(1);
    }

    if(!spp_find_process_pts(server_idx, pts, 100))
    {
        /* no pts handle */
        TRACE_ERR("SERVER%d: no control.\n", server_idx + 1);

        port[server_idx].opening = 0;
        spp_sync_state(true, server_idx);
        pthread_exit(NULL);
    }
    
    for(int i = 0; (path[5 + i] = pts[i]) && i < 15; i++);
    if ((port[server_idx].com_terminal = fopen(path, "w")) == NULL)
    {
        /* no pts handle */
        TRACE_ERR("SERVER%d: no control, please check your terminal settings and system.\n", server_idx + 1);

        port[server_idx].opening = 0;
        spp_sync_state(true, server_idx);
        pthread_exit(NULL);
    }

    port[server_idx].opening = 0;
    spp_sync_state(true, server_idx);
    pthread_exit(NULL);
}

/*******************************************************************************
 * Function Name: spp_close_terminal
 *******************************************************************************
 * Summary:
 *   Close process for Communication Board
 *
 * Parameters:
 *   void* server_idx : pointer to server index
 *
 ******************************************************************************/
void* spp_close_terminal(void* idx)
{
    uint8_t server_idx = *(uint8_t*)idx;

    /* Close 0 = successfully close */
    if(port[server_idx].com_terminal && fclose(port[server_idx].com_terminal))
    {
        TRACE_ERR("SERVER%d: Failed to close terminal file.\n", server_idx + 1);
    }

    int pid = port[server_idx].pid;

    if(!pid)
    {
        TRACE_LOG("SERVER%d: No Communication Board.", server_idx + 1);
        memset(&port[server_idx], 0, sizeof(tPROT));
        port[server_idx].closing = 0;
        spp_sync_state(false, server_idx);
        pthread_exit(NULL);
    }

    kill(pid, SIGUSR2);
    TRACE_LOG("SERVER%d: The terminal is in the process of being closed. Kindly wait for a moment.", server_idx + 1);

    sleep(CLEAN_BUFFER_TIME);

    kill(pid, SIGTERM);
    TRACE_LOG("SERVER%d: Bye.", server_idx + 1);

    memset(&port[server_idx], 0, sizeof(tPROT));
    port[server_idx].closing = 0;
    spp_sync_state(false, server_idx);
    pthread_exit(NULL);
}

/*******************************************************************************
 * Function Name: spp_connection_handle_register
 *******************************************************************************
 * Summary:
 *   SPP connection handle register to support needs for a SERVER port being conneted
 *
 * Parameters:
 *   uint16_t handle : spp handle
 *   uint8_t* bda : connected device's BD address
 *   uint8_t server_idx : spp server index
 *
 ******************************************************************************/
static void spp_connection_handle_register(uint16_t handle, uint8_t *bda, uint8_t server_idx)
{
    if (bda)
    {
        TRACE_LOG("SERVER%d address:%02X:%02X:%02X:%02X:%02X:%02X\n",
                server_idx + 1, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        port[server_idx].rx_bytes = 0;
        port[server_idx].tx_bytes = 0;

        sprintf(port[server_idx].connection_info, "address:%02X:%02X:%02X:%02X:%02X:%02X",
                bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

        spp_handle[server_idx] = handle;

        if(enalbe_terminal)
        {
            if(!port[server_idx].closing && !port[server_idx].opening)
            {
                if (pthread_create(&port[server_idx].opening, NULL, spp_init_terminal, (void*)&created_idx[server_idx]))
                {
                    TRACE_LOG("Failed to open Communication Board -> Please reconnect to SERVER%d.\n", server_idx + 1);
                }
            }

            pthread_t awake = {0};
            if (pthread_create(&awake, NULL, spp_keep_awake, (void*)&created_idx[server_idx]))
            {
                TRACE_ERR("Failed to run Keep Awake.\n");
                return;
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &tx_average_start_time[server_idx]);
        clock_gettime(CLOCK_MONOTONIC, &tx_average_end_time[server_idx]);
    }
    else
    {
        TRACE_ERR("The bda is NULL\n");
    }
}

/*******************************************************************************
 * Function Name: spp_connection_handle_reset
 *******************************************************************************
 * Summary:
 *   SPP connection handle reset, to clean up the needs for a SERVER port after disconnected
 *
 * Parameters:
 *   uint16_t handle : spp handle which should be disconnected
 *   uint8_t server_idx : spp server index
 * 
 ******************************************************************************/
static void spp_connection_handle_reset(uint16_t handle, uint8_t server_idx)
{
    TRACE_LOG("SERVER%d: TX bytes:%8d | RX bytes:%8d\n", server_idx + 1, port[server_idx].tx_bytes, port[server_idx].rx_bytes);

    spp_handle[server_idx] = 0;

    if(enalbe_terminal)
    {
        if(!port[server_idx].closing && !port[server_idx].opening)
        {
            if (pthread_create(&port[server_idx].closing, NULL, spp_close_terminal,(void*)&created_idx[server_idx]))
            {
                TRACE_ERR("Failed to close Communication Board.\n");
            }
        }  
    }
    else
    {
        memset(&port[server_idx], 0, sizeof(tPROT));
    }
}
/*******************************************************************************
 * Function Name: enqueue
 *******************************************************************************
 * Summary:
 *   TX enqueue, queue data up for later dispatching
 * 
 * Parameters:
 *   uint8_t server_idx
 *   uint8_t *p_data
 *   int len
 * 
 ******************************************************************************/
static void enqueue(uint8_t server_idx, uint8_t *p_data, int len)
{
    /* Beware of your limitation */
    Node_t* p_n = (Node_t*)calloc(1, sizeof(Node_t));
    if(!p_n)
    {
        TRACE_ERR("No memory. SERVER%d   len: %d\n", server_idx + 1, len);
        return;
    }

    if(!(p_n->p_data = (uint8_t*)calloc(1, len)))
    {
        TRACE_ERR("No memory. SERVER%d   len: %d\n", server_idx + 1, len);
        free(p_n);
        return;
    }

    if(tail)
    {
        tail->next = p_n;
    }
    tail = p_n;

    p_n->len = len;
    p_n->server_idx = server_idx;
    memcpy(p_n->p_data, p_data, len);

    if(!head)
    {
        head = p_n;
    }
}

/*******************************************************************************
 * Function Name: dequeue
 *******************************************************************************
 * Summary:
 *   TX dequeue for serial data input from main console
 *
 * Return:
 *   uint16_t : data len, used as bool here to see if it needs next round
 *
 ******************************************************************************/
static uint16_t dequeue(void)
{
    Node_t* p_n = head;
    bool ret = false;
    uint32_t interval = MIN_LATENCY;
    uint16_t rtn = 0;

    if(!p_n)
    {
        return 0;
    }
    uint8_t server_idx = p_n->server_idx;


    /* Disconnected */
    if(!spp_handle[server_idx])
    {
        p_n->len = 0;
    }

    if(p_n->len)
    {
        /* Max Retry */
        if (port[server_idx].tx_retry >= MAX_TX_RETRY)
        {
            TRACE_ERR("SERVER%d: reached max tx retries! Drop data len:%d, make sure peer device is providing us credits.\n", server_idx + 1, p_n->len);
            p_n->len = 0;
        }
        else
        {

            ret = spp_send_session_data(server_idx, p_n->p_data, p_n->len);

            if(!ret)
            {
                port[server_idx].tx_retry++;
                return 0;
            }
            else
            {
                port[server_idx].tx_retry = 0;
                rtn = p_n->len;
            }
            queue_bytes += p_n->len;
            p_n->len = 0;
        }
    }

    if(tail && p_n != tail)
    {
        head = p_n->next;
        if(p_n->p_data)
        {
            free(p_n->p_data);
        }
        if(p_n)
        {
            free(p_n);
        }
    }
    return rtn;
}

/*******************************************************************************
 * Function Name: spp_send_session_data
 *******************************************************************************
 * Summary:
 *   Spp send function data with auto allocated speed limited by RECOMMAND_SPEED
 * 
 * Parameters:
 *   uint8_t server_idx
 *   uint8_t *p_data
 *   int len
 * 
 ******************************************************************************/
bool spp_send_session_data(uint8_t server_idx, char* p_data, size_t len)
{
    bool rtn = false;
    uint32_t interval = MIN_LATENCY;

    linux_stack_lock(NULL);
    rtn = (bool) wiced_bt_spp_send_session_data(spp_handle[server_idx], p_data, len);
    linux_stack_unlock(NULL);

    if(rtn)
    {
        port[server_idx].tx_bytes += len;
        port[server_idx].awake = 1;
        usleep((interval = len * 1000000 / RECOMMAND_SPEED) < MIN_LATENCY ? MIN_LATENCY : interval);
    }
    else
    {
        TRACE_ERR("SPP transmission of session data was unsuccessful. Retransmission will commence in 8 seconds..\n");
        for(int i = SEND_ERROR_RECOVERY_TIME; i--;)
        {
            sleep(1);
            TRACE_LOG("Retransmission will commence in %d seconds..\n", i);
        }
    }
    return rtn;
}

/*******************************************************************************
 * Function Name: spp_terminal_input
 *******************************************************************************
 * Summary:
 *   Get communication board input
 * 
 * Parameters:
 *   void * idx
 * 
 ******************************************************************************/
void* spp_terminal_input(void *idx)
{
    uint32_t interval = MIN_LATENCY;
    
    struct sembuf sem_ack_op = {0, 1, 0};
    size_t len = 0;
    bool ret[SUPPORT_PORT_NUM] = {0};

    while(true)
    {
        for(int server_idx = 0; server_idx < current_port_num; server_idx++)
        {
            if(spp_handle[server_idx] && comm_data[server_idx][0])
            {
                len = strlen(comm_data[server_idx]);

                if(port[server_idx].tx_retry >= MAX_TX_RETRY)
                {
                    TRACE_ERR("SERVER%d: max tx retry, please disconnect from peer device and try to connect again.\n", server_idx + 1);

                    /* Drop data */
                    memset(comm_data[server_idx], 0, len);
                    semop(comm_ack_sem[server_idx], &sem_ack_op, 1);
                    continue;
                }

                ret[server_idx] = spp_send_session_data(server_idx, comm_data[server_idx], len);

                if(ret[server_idx])
                {
                    memset(comm_data[server_idx], 0, len);
                    semop(comm_ack_sem[server_idx], &sem_ack_op, 1);
                    port[server_idx].tx_retry = 0;
                }
                else
                {
                    port[server_idx].tx_retry++;
                }
            }
        }
    }
    return NULL;
}

/*******************************************************************************
 * Function Name: dispatch
 *******************************************************************************
 * Summary:
 *   Despatching tx queue data which comes from user main console
 *
 * Parameters:
 *   void* idx : pointer to server index
 *
 ******************************************************************************/
void* dispatch(void *idx) 
{
    double secs = 0;
    queue_bytes = 0;
    clock_gettime(CLOCK_MONOTONIC, &tx_serial_start_time);
    for(;(head && head->len) || (tail && tail != head);)
    {
        for(;dequeue(););
    }
    clock_gettime(CLOCK_MONOTONIC, &tx_serial_end_time);
    dispatching = 0;

    /* Large data */
    if(queue_bytes >= LARGE_OUTPUT)
    {
        secs = (tx_serial_end_time.tv_sec - tx_serial_start_time.tv_sec) + ((double)(tx_serial_end_time.tv_nsec - tx_serial_start_time.tv_nsec)) / 1000000000.0;
        TRACE_LOG("Large Serial Output: TX bytes: %8.f, Secs: %8.3f (s), Rate: %8.3f\n", queue_bytes, secs, queue_bytes / secs);
    }

    pthread_exit(NULL);
}

/*******************************************************************************
 * Function Name: spp_bt_enabled_lock
 *******************************************************************************
 * Summary:
 *   Register bt ready callback function 
 *
 * Parameters:
 *   void* callback : function to be called when bt is ready
 *
 ******************************************************************************/
void spp_bt_enabled_lock(void)
{
    pthread_mutex_lock(&main_console_mutex);
}

/*******************************************************************************
 * Function Name: spp_rx_data_callback
 *******************************************************************************
 * Summary:
 *   Function to be called when comm_data data is coming
 *
 * Parameters:
 *   uint16_t handle : spp connection handle
 *   uint8_t* p_data : pointer to data buffer
 *   uint32_t length : data length
 *   int server_idx : server index
 * 
 * Return:
 *   wiced_bool_t
 *
 ******************************************************************************/
wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t *p_data, uint32_t data_len, int server_idx)
{
    if (p_data)
    {
        port[server_idx].rx_bytes += data_len;
        port[server_idx].awake = 1;
        if(enalbe_rx_log)
        {
            if(enalbe_terminal)
            {
                if(port[server_idx].com_terminal)
                {
                    fwrite(p_data, sizeof(char), data_len, port[server_idx].com_terminal);
                    fflush(port[server_idx].com_terminal);
                }
                else
                {
                    TRACE_ERR("Terminal initialization might not complete. Please close terminal connections, disconnect, and then reconnect the port..\n");
                    return WICED_FALSE;
                }
            }
            else
            {
                TRACE_LOG("SERVER%d: Data len:%d   Data:%02x-%02x   Total RX Bytes:%d\n",
                        server_idx + 1, data_len, p_data[0], p_data[data_len - 1], port[server_idx].rx_bytes);
                TRACE_LOG("Data: ");
                for (int i = 0; i < data_len; i++)
                {
                    TRACE_LOG(" %c , %d", p_data[i], p_data[i]);
                }
            }
        }

        if(enable_loopback)
        {
            for(;data_len > SPP_MAX_PAYLOAD;)
            {
                spp_send_serial_data(server_idx, p_data, data_len);
                data_len -= SPP_MAX_PAYLOAD;
                p_data += SPP_MAX_PAYLOAD;
            }
            spp_send_serial_data(server_idx, p_data, data_len);
        }
    }
    else
    {
        TRACE_ERR("RX p_data is NULL.\n");
    }
    return WICED_TRUE;
}

/*******************************************************************************
 * Function Name: spp_rx_log_switch
 *******************************************************************************
 * Summary:
 *   Enable / Disable comm_data log
 *
 ******************************************************************************/
void spp_rx_log_switch(void)
{
    if(enalbe_rx_log ^= 1)
    {
        TRACE_LOG("Enable RX Log.\n");
    }
    else
    {
        TRACE_LOG("Disable RX Log.\n");
    }
}

/*******************************************************************************
 * Function Name: spp_mode_loopback_switch
 *******************************************************************************
 * Summary:
 *   Turn On / Off RX Loopback
 *
 ******************************************************************************/
void spp_mode_loopback_switch(void)
{
    if(enable_loopback ^= 1)
    {
        TRACE_LOG("Enalbe Loopback.\n");
    }
    else
    {
        TRACE_LOG("Disable Loopback.\n");
    }
}

/*******************************************************************************
 * Function Name: spp_enable_terminal
 *******************************************************************************
 * Summary:
 *   Enable terminal as communication board when connection is established
 * 
 * Parameters:
 *   bool value
 * 
 ******************************************************************************/
bool spp_enable_terminal(bool value)
{
    if(enalbe_terminal = value)
    {
        for(int server_idx = 0; server_idx < current_port_num; server_idx++)
        {
            if
            (
                (shm_id = shmget((key_t)(server_idx + 1), SHM_SIZE, IPC_CREAT | 0666)) == -1 ||
                (comm_data[server_idx] = shmat(shm_id, NULL, 0)) == (char *)-1 ||
                (comm_ack_sem[server_idx] = semget((key_t)(server_idx + 1 + 0xFFFF), 1, IPC_CREAT | 0666)) == -1
            )
            {
                TRACE_ERR("SERVER%d: Failed to request system source, kindly restart program.\n", server_idx + 1);
            }
            memset(comm_data[server_idx], 0, SHM_SIZE);
        }

        TRACE_LOG("Terminal input start.\n");
        if (pthread_create(&terminal_input, NULL, spp_terminal_input, NULL))
        {
            TRACE_ERR("Failed to configure terminal input service.\n");
        }
    }

    return enalbe_terminal;
}

/*******************************************************************************
 * Function Name: spp_send_serial_data
 *******************************************************************************
 * Summary:
 *   Add mutex to spp send session data and print log
 *
 * Parameters:
 *   uint8_t server_idx : server index
 *   uint8_t* p_data : pointer to data
 *   uint32_t length : data length
 * 
 * Return:
 *   wiced_bool_t
 *
 ******************************************************************************/
void spp_send_serial_data(uint8_t server_idx, uint8_t* p_data, uint32_t length)
{
    while(length > SPP_MAX_PAYLOAD)
    {
        enqueue(server_idx, p_data, length);
        length -= SPP_MAX_PAYLOAD;
        p_data += SPP_MAX_PAYLOAD;
    }
    enqueue(server_idx, p_data, length);

    if(!dispatching)
    {
        if (pthread_create(&dispatching, NULL, dispatch, NULL))
        {
            TRACE_ERR("Dispatching service create failed. Kindly try to send data again.\n");
        }
    }
}

/*******************************************************************************
 * Function Name: spp_nvram_file_name
 *******************************************************************************
 * Summary:
 *   Generate spp nvram file name which is:
 *   nv + each byte of bd_addr in decimal 3 digits + .bin
 *   e.g.: nv112026184248031250.bin
 *
 * Parameters:
 *   uint8_t server_idx : server index
 *   uint8_t* p_data : pointer to data
 *   uint32_t length : data length
 * 
 * Return: 
 *   wiced_bool_t
 *
 ******************************************************************************/
void spp_nvram_file_name(char* nvram_file_name, wiced_bt_device_address_t bd_addr, uint8_t len_name)
{
    if(len_name != NVRAM_FILE_NAME_LEN)
    {
        TRACE_ERR("NVRAM file name length error.");
        return;
    }
    memset(nvram_file_name, 0, NVRAM_FILE_NAME_LEN);
    sprintf(nvram_file_name, "nv");

    /* Use all six bytes from bd_addr to file name */
    for(int i = 0; i < 6; i++)
    {
        sprintf(nvram_file_name + 2 + (i * 3), "%03d", bd_addr[i]);
    }
    sprintf(nvram_file_name + 20, ".bin");
}

/*******************************************************************************
 * Function Name: spp_management_callback
 *******************************************************************************
 * Summary:
 *   This is a Bluetooth stack event handler function to receive management
 *   events from the BT stack and process as per the application.
 *
 * Parameters:
 *   wiced_bt_management_evt_t event : BT event code of one byte length
 *   wiced_bt_management_evt_data_t *p_event_data : Pointer to BT management
 *                                                  event structures
 *
 * Return:
 *  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 ******************************************************************************/
static wiced_result_t spp_management_callback(wiced_bt_management_evt_t event,
                                              wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_dev_encryption_status_t *p_encryption_status = {0};
    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification  = {0};
    wiced_bt_dev_pairing_info_t *p_pairing_info = {0};
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = {0};
    char nvram_file_name[NVRAM_FILE_NAME_LEN] = {0};
    FILE* p_file = NULL;
    FILE* p_file_read = NULL;
    size_t rw_bytes = 0;

    TRACE_LOG("Event: 0x%x %s\n", event, spp_get_bt_event_name(event));

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Bluetooth Controller and Host Stack Enabled */
            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                wiced_bt_set_local_bdaddr(spp_bd_address,BLE_ADDR_PUBLIC);
                wiced_bt_dev_read_local_addr(bda);
                TRACE_LOG("Local Bluetooth Address: ");
                spp_print_bd_address(bda);
                /* 1 - Release console */
                pthread_mutex_unlock(&main_console_mutex);
            }
            else
            {
                TRACE_ERR("Bluetooth Enable Failed.\n");
                return WICED_BT_ERROR;
            }
            break;

        case BTM_DISABLED_EVT:
            TRACE_LOG("Bluetooth Disabled.\n");
            break;

        case BTM_PIN_REQUEST_EVT:
            TRACE_LOG("Remote address= %B\n", p_event_data->pin_request.bd_addr);
            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr, result, PINCODE_LENGTH, &pincode[0]);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            /* This application always confirms peer's attempt to pair */
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* This application supports only Just Works pairing */
            TRACE_LOG("BTM_PAIRING_IO_CAPABILITIES_REQUEST_EVT bda %B\n",
                        p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_info = &p_event_data->pairing_complete.pairing_complete_info;
            TRACE_LOG("Pairing Complete: %d\n", p_pairing_info->br_edr.status);
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;
            TRACE_LOG("Encryption Status Event: bd (%B) res %d\n",
                        p_encryption_status->bd_addr, p_encryption_status->result);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* Support multi-device link key stored, file name nv"bd_addr".bin, for example:
                nv112026184248031250.bin " */
            TRACE_LOG("Link remote device address= %B\n", p_event_data->paired_device_link_keys_update.bd_addr);

            spp_nvram_file_name(nvram_file_name, p_event_data->paired_device_link_keys_update.bd_addr, NVRAM_FILE_NAME_LEN);
            if (!(p_file = fopen(nvram_file_name, "wb")))
            {
                TRACE_ERR("Unable to open NVRAM file %s with %s\n", nvram_file_name, "wb");
                return WICED_BT_ERROR;
            }
            rw_bytes = fwrite(&p_event_data->paired_device_link_keys_update, 1, sizeof(wiced_bt_device_link_keys_t), p_file);
            fclose(p_file);
            if(rw_bytes)
            {
                TRACE_LOG("NVRAM file name: %s len %zu\n", nvram_file_name, rw_bytes);
                return WICED_BT_SUCCESS;
            }
            else
            {
                TRACE_ERR("NVRAM file name: %s key retrieval failure.\n", nvram_file_name);
                return WICED_BT_ERROR;
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            TRACE_LOG("Link remote device address= %B\n", p_event_data->paired_device_link_keys_request.bd_addr);

            spp_nvram_file_name(nvram_file_name, p_event_data->paired_device_link_keys_request.bd_addr, NVRAM_FILE_NAME_LEN);
            if (!(p_file_read = fopen(nvram_file_name, "rb")))
            {
                TRACE_ERR("Unable to open NVRAM file %s with %s\n", nvram_file_name, "rb");
                return WICED_BT_ERROR;
            }
            rw_bytes = fread(&p_event_data->paired_device_link_keys_request, 1, sizeof(wiced_bt_device_link_keys_t), p_file_read);
            fclose(p_file_read);
            if(rw_bytes)
            {
                TRACE_LOG("NVRAM file name: %s len %zu\n", nvram_file_name, rw_bytes); 
                return WICED_BT_SUCCESS;
            }
            else
            {
                TRACE_ERR("NVRAM file name: %s key retrieval failure.\n", nvram_file_name); 
                return WICED_BT_ERROR;
            }
            break;

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
            TRACE_LOG("Power mgmt status event: bd (%B) status:%d hci_status:%d\n", p_power_mgmt_notification->bd_addr,
                        p_power_mgmt_notification->status, p_power_mgmt_notification->hci_status);
            break;

        default:
            return WICED_BT_USE_DEFAULT_SECURITY;
    }
    return result;
}

/*******************************************************************************
 * Function Name: spp_write_eir
 *******************************************************************************
 * Summary:
 *   Prepare extended inquiry response data.
 *
 ******************************************************************************/
static void spp_write_eir(void)
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;
    uint16_t eir_length;

    pBuf = (uint8_t *)wiced_bt_get_buffer(WICED_EIR_BUF_MAX_SIZE);
    TRACE_LOG("hci_control_write_eir %x\n", pBuf);

    if (!pBuf)
    {
        TRACE_LOG("app_write_eir %x\n", pBuf);
    }
    else
    {
        p = pBuf;

        length = strlen((char *)wiced_bt_cfg_settings.device_name);

        *p++ = length + 1;
        *p++ = BT_EIR_COMPLETE_LOCAL_NAME_TYPE;     /* EIR type full name */
        memcpy(p, wiced_bt_cfg_settings.device_name, length);
        p += length;

        *p++ = 2 + 1;                               /* Length of 16 bit services*/
        *p++ = BT_EIR_COMPLETE_16BITS_UUID_TYPE;    /* 0x03 EIR type full list
                                                    of 16 bit service UUIDs */
        *p++ = UUID_SERVCLASS_SERIAL_PORT & 0xff;
        *p++ = (UUID_SERVCLASS_SERIAL_PORT >> 8) & 0xff;

        /* End of EIR Data is 0 */
        *p++ = 0;

        eir_length = (uint16_t)(p - pBuf);

        /* Print EIR data */
        TRACE_LOG("%s %A","EIR :",pBuf,MIN(p - pBuf, 100));
        wiced_bt_dev_write_eir(pBuf, eir_length);
    }
}

/*******************************************************************************
 * Function Name: spp_reg_ports
 *******************************************************************************
 * Summary:
 *   Register SPP ports
 *
 * Parameters:
 *   uint8_t num_ports : number of ports
 *
 * Return:
 *   wiced_bool_t
 *
 ******************************************************************************/
wiced_bool_t spp_reg_ports(uint8_t num_ports)
{
    spp_regs = calloc(num_ports, sizeof(wiced_bt_spp_reg_t));
    if(!spp_regs)
    {
        TRACE_ERR("SPP register ports failed.\n");
        return WICED_FALSE;
    }
    for(uint8_t p = 0; p < num_ports; p++ )
    {
        wiced_bt_spp_reg_t temp = 
        {
            SPP_RFCOMM_SCN + p,
            SPP_MAX_PAYLOAD,
            spp_connection_up_callback[p],
            NULL,
            NULL,
            spp_connection_down_callback[p],
            spp_rx_callback[p],
        };
        spp_regs[p] = temp;
        wiced_bt_spp_startup(&spp_regs[p]);
    }
    return WICED_TRUE;
}

/*******************************************************************************
 * Function Name: spp_init
 *******************************************************************************
 * Summary:
 *   Inititate SPP
 *
 * Parameters:
 *   uint8_t num_ports : number of ports to be initiated
 *
 * Return:
 *   wiced_bool_t
 *
 ******************************************************************************/
wiced_bool_t spp_init(uint8_t num_ports)
{
    uint8_t* sdp_database = wiced_get_sdp_database(num_ports);

    spp_write_eir();

    current_port_num = num_ports;

    /* create SDP records and register SPP ports */
    if(!(sdp_database && wiced_bt_sdp_db_init(sdp_database, wiced_app_cfg_sdp_record_get_size()) && spp_reg_ports(num_ports)))
    {
        TRACE_ERR("SDP initiation failed.");
        return WICED_FALSE;
    }

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* This application will always configure device connectable
     * and discoverable
     */
    wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE,
                                     WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,
                                     WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW);

    wiced_bt_dev_set_connectability(BTM_CONNECTABLE,
                                    WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,
                                    WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW);

    return WICED_TRUE;
}

/*******************************************************************************
 * Function Name: spp_init
 *******************************************************************************
 * Summary:
 *   Inititate SPP
 *
 * Parameters:
 *   uint8_t num_ports : number of ports to be initiated
 *
 * Return:
 *   uint8_t*
 *
 ******************************************************************************/
uint8_t* spp_port_tx_buffer(uint8_t server_idx)
{
    return port[server_idx].tx_buffer;
}

/*******************************************************************************
 * Function Name: spp_find_process_pts
 *******************************************************************************
 * Summary:
 *   Initiate terminal for connection port
 *
 * Parameters:
 *   void* idx : pointer to server index
 *
 ******************************************************************************/
static int spp_find_process_pts(uint8_t server_idx, char* buf, int len) 
{
    char command[100] = {0};

    memset(buf, 0, len);

    if(server_idx >= SUPPORT_PORT_NUM)
    {
        return 0;
    }

    snprintf(command, 100, "ps aux | grep \"./SERVER %d\" | grep -v grep | grep -v lxterminal | awk '{print $7}'", server_idx + 1);

    if (!spp_execute_command(command, buf, len))
    {
        TRACE_ERR("Failed to execute command: %s\n", command);
        return 0;
    }

    /* Take the first pts */
    buf[strcspn(buf, "\n")] = '\0';

    return strlen(buf);
}

/*******************************************************************************
 * Function Name: spp_server_status_string
 *******************************************************************************
 * Summary:
 *   Get spp ports status string, show address if it is connected by peer device
 *
 * Parameters:
 *   char* buffer
 *   uint8_t len
 *
 ******************************************************************************/
void spp_server_status_string(char* buffer, uint8_t len)
{
    memset(buffer, 0, len);
    for(int server_idx = 0; server_idx < current_port_num; server_idx++)
    {
        if(spp_handle[server_idx])
        {
            sprintf(buffer + strlen(buffer), "                  SERVER%d -> %s\n",
                server_idx + 1, port[server_idx].connection_info);
        }
        else
        {
            sprintf(buffer + strlen(buffer), "                  SERVER%d -> \n", server_idx + 1);
        }
    }
}

/*******************************************************************************
 * Function Name: spp_keep_awake
 *******************************************************************************
 * Summary:
 *   Keep ports awake by sending empty data to have better response from peer device
 *
 * Parameters:
 *   void* idx
 *
 ******************************************************************************/
void* spp_keep_awake(void* idx)
{
    uint8_t server_idx = *(uint8_t*)idx;
    uint8_t hi[1] = {0};
    double secs = 0;
    double bytes = 0.0;
    bool show_average = true;

    while(spp_handle[server_idx])
    {
        if(port[server_idx].awake)
        {
            port[server_idx].awake = 0;
            show_average = true;

            /* If not exist */
            if(!(port[server_idx].pid = spp_find_process_pid(server_idx)) && port[server_idx].pid != -1)
            {
                if(!port[server_idx].closing && !port[server_idx].opening)
                {
                    if (pthread_create(&port[server_idx].opening, NULL, spp_init_terminal, (void*)&created_idx[server_idx]))
                    {
                        TRACE_LOG("Failed to open Communication Board -> Reconnect to SERVER%d.\n", server_idx + 1);
                    }
                    TRACE_LOG("Communication Board: Save me...\n"); 
                }
            }
        }
        else
        {
            linux_stack_lock(NULL);
            if(!wiced_bt_spp_send_session_data(spp_handle[server_idx], hi, sizeof(hi)))
            {
                TRACE_ERR("SERVER%d: The peer device appears to be malfunctioning..\n", server_idx + 1);
            }
            linux_stack_unlock(NULL);
            
            /* If not exist */
            if(!(port[server_idx].pid = spp_find_process_pid(server_idx)) && port[server_idx].pid != -1)
            {
                if(!port[server_idx].closing && !port[server_idx].opening)
                {
                    if (pthread_create(&port[server_idx].opening, NULL, spp_init_terminal, (void*)&created_idx[server_idx]))
                    {
                        TRACE_ERR("Failed to open Communication Board -> Reconnect to SERVER%d.\n", server_idx + 1);
                    }
                }
            }

            /* Average Speed */
            if(show_average)
            {
                show_average = false;
                clock_gettime(CLOCK_MONOTONIC, &tx_average_end_time[server_idx]);
                secs = (tx_average_end_time[server_idx].tv_sec - tx_average_start_time[server_idx].tv_sec) + ((double)(tx_average_end_time[server_idx].tv_nsec - tx_average_start_time[server_idx].tv_nsec)) / 1000000000.0;
                if(secs > 0)
                {
                    bytes = port[server_idx].tx_bytes;
                    TRACE_LOG("SERVER%d: Average speed after being connected.: TX bytes: %8.f, Secs: %8.3f (s), Rate: %8.3f\n", server_idx + 1, bytes, secs, bytes / secs);
                    bytes = port[server_idx].rx_bytes;
                    TRACE_LOG("SERVER%d: Average speed after being connected.: RX bytes: %8.f, Secs: %8.3f (s), Rate: %8.3f\n", server_idx + 1, bytes, secs, bytes / secs);
                }
            }
            sleep(AWAKE_INTERVAL - 1);
        }
        sleep(1);
    }
    pthread_exit(NULL);
}

/*******************************************************************************
 * Function Name: spp_port_handle
 *******************************************************************************
 * Summary:
 *   Get spp handle
 *
 * Parameters:
 *   uint8_t server_idx: server index
 *
 * Return:
 *   uint16_t
 *
 ******************************************************************************/
uint16_t spp_port_handle(uint8_t server_idx)
{
    if(server_idx >= current_port_num)
    {
        TRACE_LOG("Invalid port. SERVER%d", server_idx);
        return 0;
    }
    return spp_handle[server_idx];
}

/*******************************************************************************
 * Function Name: spp_execute_command
 *******************************************************************************
 * Summary:
 *   Function to execute the command and get the output
 *
 * Parameters:
 *   const char* command : command to execute
 *   char* buffer : buffer to take results
 *   uint16_t buf_size : size of buffer
 * 
 * Return:
 *   bool : execute status
 *
 ******************************************************************************/
static bool spp_execute_command(const char* command, char* buffer, uint16_t buf_size)
{
    int c = 0;
    FILE* command_output = popen(command, "r");

    if (!command_output) 
    {
        return false;
    }

    /* 12 as min buffer len here */
    while(fgets(buffer + c, buf_size - c, command_output) && buf_size - c > MIN_COMMAND_BUFFER_LEN)
    {
        c += strlen(buffer + c);
    }

    /* Enough buffer size for this application */
    if(buf_size - c <= MIN_COMMAND_BUFFER_LEN)
    {
        TRACE_ERR("Need more buffer for get command result.");
        pclose(command_output);
        return false;
    }

    pclose(command_output);
    return true;
}

/*******************************************************************************
 * Function Name: spp_find_process_pid
 *******************************************************************************
 * Summary:
 *   Find pid of corresponding terminal number
 *
 * Parameters:
 *   int terminal_number : corresponding terminal number
 *
 * Return:
 *   int : pid, -1 if it is faild to execute command
 *
 ******************************************************************************/
static int spp_find_process_pid(uint8_t server_idx) 
{
    char command[100] = {0};
    char results[512] = {0};

    snprintf(command, 100, "ps aux | grep \"./SERVER %d\" | grep -v grep | grep -v lxterminal | awk '{print $2}'", server_idx + 1);

    if (!spp_execute_command(command, results, sizeof(results))) 
    {
        TRACE_ERR("Failed to execute command: %s\n", command);
        return -1;
    }

    /* Take the first pid */
    results[strcspn(results, "\n")] = '\0';

    return atoi(results);
}

/*******************************************************************************
 * Function Name: spp_get_current_port_num
 *******************************************************************************
 * Summary:
 *   SPP get current port number, the number of ports we have created
 *
 * Return:
 *   uint8_t number of ports
 *
 ******************************************************************************/
uint8_t spp_get_current_port_num(void)
{
    return current_port_num;
}

/*******************************************************************************
 * Function Name: spp_start_throughput
 *******************************************************************************
 * Summary:
 *   SPP start throughput test, keep current time as start time
 *
 ******************************************************************************/
void spp_start_throughput(void)
{
    for(int server_idx = current_port_num; server_idx--;)
    {
        port[server_idx].rx_bytes = 0;
        port[server_idx].tx_bytes = 0;
        if(port[server_idx].com_terminal)
        {
            fprintf(port[server_idx].com_terminal, "\n\
\n\
SEVER%d Throughput Test: \n\
\n\
--------------------------------Throughtput Start-------------------------------\n\
"
            , server_idx + 1);
            fflush(port[server_idx].com_terminal);  
        }
    }
    return;
}

/*******************************************************************************
 * Function Name: spp_end_throughput
 *******************************************************************************
 * Summary:
 *   SPP end throughput test
 * 
 * Parameters:
 *   uint8_t server_idx
 *   uint32_t rx_bytes
 * 
 ******************************************************************************/
void spp_end_throughput(struct timespec* user_start_time, struct timespec* user_end_time)
{
    double secs = 0;
    double bytes_per_sec = 0;
    secs = (user_end_time->tv_sec - user_start_time->tv_sec) + ((double)(user_end_time->tv_nsec - user_start_time->tv_nsec)) / 1000000000.0;
    if(secs <= 0)
    {
        /* Time error */
        TRACE_ERR("Time interval error. please try again.");
        memcpy(user_start_time, user_end_time, sizeof(struct timespec));
        return;
    }

    for(int server_idx = 0; server_idx < current_port_num; server_idx++)
    {
        bytes_per_sec = port[server_idx].rx_bytes / secs;
        /* Menu */
        fprintf(stdout, "SERVER%d: Received %12d bytes in %8.3f (s) -> Data Transfer Rate: %8.3f bytes per second.\n", 
        server_idx + 1, port[server_idx].rx_bytes, secs, bytes_per_sec);

        if(port[server_idx].com_terminal)
        {
            /* Directly displayed on connection terminal */
            fprintf(port[server_idx].com_terminal, "\n\n\n\
--------------------------------Throughtput Stop--------------------------------\n\
\n\
SERVER%d: Received %12d bytes in %8.3f (s) \n\
Data Transfer Rate:   %8.3f bytes per second.\n\
\n\
"
            , server_idx + 1, port[server_idx].rx_bytes, secs, bytes_per_sec);
            fflush(port[server_idx].com_terminal);  
        }
        port[server_idx].rx_bytes = 0;
    }
    fprintf(stdout, "\n");
    for(int server_idx = 0; server_idx < current_port_num; server_idx++)
    {
        bytes_per_sec = port[server_idx].tx_bytes / secs;
        /* Menu */
        fprintf(stdout, "SERVER%d: Transmit %12d bytes in %8.3f (s) -> Data Transfer Rate: %8.3f bytes per second.\n", 
        server_idx + 1, port[server_idx].tx_bytes, secs, bytes_per_sec);

        if(port[server_idx].com_terminal)
        {
            /* Directly displayed on connection terminal */
            fprintf(port[server_idx].com_terminal, "\n\
SERVER%d: Transmit %12d bytes in %8.3f (s) \n\
Data Transfer Rate:   %8.3f bytes per second.\n\
\n\
--------------------------------Throughtput Start-------------------------------\n\
\n\
"
            , server_idx + 1, port[server_idx].tx_bytes, secs, bytes_per_sec);
            fflush(port[server_idx].com_terminal);
        }
        port[server_idx].tx_bytes = 0;
    }
    memcpy(user_start_time, user_end_time, sizeof(struct timespec));
}

/*******************************************************************************
 * Function Name: spp_send_sample_data
 *******************************************************************************
 * Summary:
 *   Test function which sends large data to SPP client
 *
 * Parameters:
 *   uint8_t server_idx : index to corresponding spp server
 *
 ******************************************************************************/
void spp_send_sample_data(uint8_t server_idx)
{
    int i = 0;

    while ((spp_handle[server_idx] != 0) && (port[server_idx].tx_offset != SPP_TOTAL_DATA_TO_SEND))
    {
        int bytes_to_send = port[server_idx].tx_offset + SPP_MAX_PAYLOAD < SPP_TOTAL_DATA_TO_SEND ?
            SPP_MAX_PAYLOAD : SPP_TOTAL_DATA_TO_SEND - port[server_idx].tx_offset;

        for (i = 0; i < bytes_to_send; i++)
        {
            port[server_idx].tx_buffer[i] = port[server_idx].tx_offset + i;
        }
        spp_send_serial_data(server_idx, port[server_idx].tx_buffer, bytes_to_send);
        port[server_idx].tx_offset += bytes_to_send;
    }
    port[server_idx].tx_offset = 0;
}

/******************************************************************************
 * Function Name: spp_print_bd_address
 ******************************************************************************
 * Summary:
 *   Prints the address of Bluetooth device
 *
 * Parameters:
 *   wiced_bt_device_address_t bdadr : Bluetooth address
 *
 ******************************************************************************/
static void spp_print_bd_address(wiced_bt_device_address_t bdadr)
{
    TRACE_LOG("%02X:%02X:%02X:%02X:%02X:%02X\n",
                   bdadr[0], bdadr[1], bdadr[2], bdadr[3], bdadr[4], bdadr[5]);
}

/******************************************************************************
 * Function Name: spp_get_bt_event_name
 ******************************************************************************
 * Summary:
 *   The function converts the wiced_bt_management_evt_t enum value to its
 *   corresponding string literal. This will help the programmer to debug
 *   easily with log traces without navigating through the source code.
 *
 * Parameters:
 *   wiced_bt_management_evt_t event: Bluetooth management event type
 *
 * Return:
 *   const char *: String for wiced_bt_management_evt_t
 *
 ******************************************************************************/
static const char *spp_get_bt_event_name(wiced_bt_management_evt_t event)
{
    switch ((int)event)
    {
        CASE_RETURN_STR(BTM_ENABLED_EVT)
        CASE_RETURN_STR(BTM_DISABLED_EVT)
        CASE_RETURN_STR(BTM_POWER_MANAGEMENT_STATUS_EVT)
        CASE_RETURN_STR(BTM_PIN_REQUEST_EVT)
        CASE_RETURN_STR(BTM_USER_CONFIRMATION_REQUEST_EVT)
        CASE_RETURN_STR(BTM_PASSKEY_NOTIFICATION_EVT)
        CASE_RETURN_STR(BTM_PASSKEY_REQUEST_EVT)
        CASE_RETURN_STR(BTM_KEYPRESS_NOTIFICATION_EVT)
        CASE_RETURN_STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT)
        CASE_RETURN_STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT)
        CASE_RETURN_STR(BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT)
        CASE_RETURN_STR(BTM_PAIRING_COMPLETE_EVT)
        CASE_RETURN_STR(BTM_ENCRYPTION_STATUS_EVT)
        CASE_RETURN_STR(BTM_SECURITY_REQUEST_EVT)
        CASE_RETURN_STR(BTM_SECURITY_FAILED_EVT)
        CASE_RETURN_STR(BTM_SECURITY_ABORTED_EVT)
        CASE_RETURN_STR(BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT)
        CASE_RETURN_STR(BTM_REMOTE_OOB_DATA_REQUEST_EVT)
        CASE_RETURN_STR(BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT)
        CASE_RETURN_STR(BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT)
        CASE_RETURN_STR(BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT)
        CASE_RETURN_STR(BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT)
        CASE_RETURN_STR(BTM_BLE_SCAN_STATE_CHANGED_EVT)
        CASE_RETURN_STR(BTM_BLE_ADVERT_STATE_CHANGED_EVT)
        CASE_RETURN_STR(BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT)
        CASE_RETURN_STR(BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT)
        CASE_RETURN_STR(BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT)
        CASE_RETURN_STR(BTM_SCO_CONNECTED_EVT)
        CASE_RETURN_STR(BTM_SCO_DISCONNECTED_EVT)
        CASE_RETURN_STR(BTM_SCO_CONNECTION_REQUEST_EVT)
        CASE_RETURN_STR(BTM_SCO_CONNECTION_CHANGE_EVT)
        CASE_RETURN_STR(BTM_BLE_CONNECTION_PARAM_UPDATE)
    }
    return NULL;
}

/* END OF FILE [] */
