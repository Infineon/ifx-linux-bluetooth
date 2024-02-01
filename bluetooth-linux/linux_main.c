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
 **  Name:          linux_main.c
 **
 **  Description: entrance of launch linux BT Code Example.
 **
 ******************************************************************************/

#include "stdio.h"
#include <stdlib.h>
#include "string.h"
#include <time.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <sys/time.h>
#include <asm-generic/termbits.h>
#include <asm-generic/ioctls.h>
#include "wiced_bt_types.h"

#include "wiced_timer.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_stack_platform.h"
#include <pthread.h>
#include <ctype.h>
#include "data_types.h"
#include "platform_linux.h"
#include "utils_arg_parser.h"
#include "patch_download.h"

#include <unistd.h>
#include <errno.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/poll.h>
#include "log.h"


extern wiced_bt_stack_platform_t btu_platform_interfaces;

extern BOOL32 uartInit (char *comPortName, uint32_t dwBaudRate);
extern void uartSend (uint32_t bIsCommand, uint8_t* pData, uint32_t len);

extern void APPLICATION_START(void);
extern void patch_download_check(void);
extern BOOL32 uart_reconfigure(uint32_t dwBaudRate);

uint32_t app_baud_rate   = DEFAULT_APP_BAUD_RATE;
uint32_t patch_baud_rate = DEFAULT_PATCH_BAUD_RATE;

#define MAX_PATH 256
#define DEBUG_ERROR_MSG "[ERROR]:"
#define GPIO_MAX_NAME_SIZE 32
#define INFITIME -1

uint8_t PATCH_FILE_NAME[MAX_PATH + 1] = "";

pthread_t timer_thread;
pthread_mutex_t lock  = PTHREAD_MUTEX_INITIALIZER;;
pthread_cond_t schedParamInit = PTHREAD_COND_INITIALIZER;
pthread_t gpioThread;

static uint64_t     absTimeHi = 0;
static uint64_t     TargetTimeToWake = 0;
void*               timerTask(void *t);

static pthread_mutex_t cs_mutex;

#define SPY_TRACE_TYPE_TEXT         0
#define SPY_TRACE_TYPE_ERROR        1
#define SPY_TRACE_TYPE_HCI_COMMAND  3
#define SPY_TRACE_TYPE_HCI_EVENT    4
#define SPY_TRACE_TYPE_ACL_RX       6
#define SPY_TRACE_TYPE_ACL_TX       7
#define SPY_TRACE_TYPE_LMP_RECV     8
#define SPY_TRACE_TYPE_LMP_XMIT     9
#define SPY_TRACE_TYPE_SPY_COMMAND  29
#define SPY_TRACE_TYPE_WARN         30
#define SPY_TRACE_TYPE_API          31
#define SPY_TRACE_TYPE_EVENT        32
#define SPY_TRACE_TYPE_SCO_RX       33
#define SPY_TRACE_TYPE_SCO_TX       34

#define SPY_TRACE_TYPE_ISO_RX       38
#define SPY_TRACE_TYPE_ISO_TX       39

#define GPIO_ACTIVE_HIGH      1
#define GPIO_ACTIVE_LOW       0

pf_le_local_support_func_t pf_le_local_support_func;

static const wiced_bt_trace_type_t spy_trace_types[] = {
    SPY_TRACE_TYPE_TEXT, // WICED_BT_TRACE_DEBUG
    SPY_TRACE_TYPE_ERROR,// WICED_BT_TRACE_ERROR
    SPY_TRACE_TYPE_WARN, // WICED_BT_TRACE_WARN
    SPY_TRACE_TYPE_API,  // WICED_BT_TRACE_API
    SPY_TRACE_TYPE_EVENT // WICED_BT_TRACE_EVENT
};


static void SendHciTraceToSpy (wiced_bt_hci_trace_type_t type, uint16_t len, uint8_t* p_data);
static uint8_t* GetAclBuffer (wiced_bt_transport_t transport, uint32_t size);
static wiced_result_t SendCommandToBtHCI (uint8_t *pCmd, uint16_t cmd_len);
static wiced_result_t SendAclToBtHCI (wiced_bt_transport_t transport, uint8_t *pData, uint16_t len);
BOOL32 init_rx_queue(void);

extern void TraceHciPkt(BYTE type, BYTE *buffer, uint16_t length, int spy_instance);
BOOL32 bUseUART = FALSE;

int wicedx_emulator_instance = 0;
char g_peer_ip_addr[16] = "000.000.000.000";

wiced_bool_t bSupressProtocolTraces = FALSE;

/*
* Description:
*    Only for Support autobaud detect BT chip 
*    if need autobaud detect 3M patch fw. use this VSC let BT chip detect speed
* opcode: 0xFFED (65517, "Enter Download Mode")
*/
static uint8_t enter_download_mode[] = {0xED, 0xFF, 0x14, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xED, 0xFF, 0x01, 0x00};


typedef  struct {
    pthread_mutex_t m_mutex;
    pthread_cond_t m_cond;
    BOOL32 m_set;
} BT_WAIT_EVENT;

BT_WAIT_EVENT g_stop_event;
BOOL32 timer_task_created = FALSE;
void init_stack_lock()
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&cs_mutex, &attr);
}
void CreateEvent()
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&g_stop_event.m_mutex, &attr);
    pthread_cond_init(&g_stop_event.m_cond, 0);
}

void SetEvent()
{
    pthread_mutex_lock(&g_stop_event.m_mutex);
    pthread_cond_signal(&g_stop_event.m_cond);
    pthread_mutex_unlock(&g_stop_event.m_mutex);
}

void WaitEvent(uint32_t wait_time)
{
    struct timespec timewait;
    struct timeval now;

    gettimeofday(&now, NULL);
    timewait.tv_nsec = 0;
    timewait.tv_sec = now.tv_sec+wait_time;

    pthread_mutex_lock(&g_stop_event.m_mutex);
    pthread_cond_timedwait(&g_stop_event.m_cond, &g_stop_event.m_mutex, &timewait);
    pthread_mutex_unlock(&g_stop_event.m_mutex);
}


uint32_t getTick()
{
    struct timespec ts;
    unsigned theTick = 0U;
    clock_gettime( CLOCK_REALTIME, &ts );
    theTick  = ts.tv_nsec / 1000000;
    theTick += ts.tv_sec * 1000;
    return theTick;
}

static BOOL32 cy_platform_open_file_write(const char *file_name, const char* content)
{
    int fd;
    fd = open(file_name, O_WRONLY);
    if(fd == -1)
    {
        TRACE_ERR("opening %s failed.\n", file_name);
        return FALSE;
    }
    else
    {
        if (write(fd, content, strlen(content)) != strlen(content)) 
        {
            TRACE_ERR("writing to %s\n", file_name);
            close(fd);
            return FALSE;
        }
    }
    close(fd);
    return TRUE;
}

/* 
* @brief Function for toggling reg_on pin to enter download mode.
* @param p_autobaud_cfg 

typedef struct
{
    char p_gpiochip[CYHAL_GPIOCHIP_NAME_SIZE];  // < gpio chip name i.e. gpiochip0
    char line_num[CYHAL_GPIO_LINENUM_SIZE];     // < line number in gpio bank
}cyhal_gpio_t;

// GPIO configuration to put the device in autobaud mode
typedef struct{
    cyhal_gpio_t                    bt_reg_on_off;      // < BT reg_on_off pin
    uint8_t                   use_ioctl;          // < To use ioctl
}cybt_controller_autobaud_config_t;

* @usage 
    ioctl:
        ./CE_executable <other parameters with -p patch.hcd> -r gpiochip0 3 -n
            -r <gpiochip> <offset> -n
    export gpio:
        ./CE_executable <other parameters with -p patch.hcd> -r gpiochip0 3
            -r <gpiochip> <offset> 
*/
static BOOL32 cy_platform_gpio_toggle_reg_on(const cybt_controller_autobaud_config_t *p_autobaud_cfg)
{   
    TRACE_LOG("\n"); 

    int fd = 0;  //use for file open
    unsigned int len_to_export = 0;  //the length of "/sys/class/gpio/gpiochip0/subsystem/"
    unsigned int len_to_gpion_end = 0; //the length of "/sys/class/gpio/gpiochip0/subsystem/gpio_n"

    char str_gpio_sub[CYHAL_GPIO_BUFFER_SIZE] = "/sys/class/gpio/";
    strcat(str_gpio_sub,  p_autobaud_cfg->bt_reg_on_off.p_gpiochip);
    strcat(str_gpio_sub, "/subsystem");
    len_to_export = strlen(str_gpio_sub);
    
    //  echo line_num > /sys/class/gpio/gpiochip0/subsystem/export
    strcat(str_gpio_sub, "/export");
    if (cy_platform_open_file_write(str_gpio_sub, p_autobaud_cfg->bt_reg_on_off.line_num) == FALSE){
        return FALSE;
    }
    sleep(1);   // delay waiting for udev chown change
    memset(str_gpio_sub + len_to_export, 0, strlen("/export"));

    // /sys/class/gpio/gpiochip0/subsystem/gpio_n
    strcat(str_gpio_sub, "/gpio");
    strcat(str_gpio_sub, p_autobaud_cfg->bt_reg_on_off.line_num);
    len_to_gpion_end = strlen(str_gpio_sub);

    // echo out > /sys/class/gpio/gpiochip0/subsystem/gpio_n/direction 
    strcat(str_gpio_sub, "/direction");
    if (cy_platform_open_file_write(str_gpio_sub, "out") == FALSE){
        return FALSE;
    }
    memset(str_gpio_sub + len_to_gpion_end, 0, strlen("/direction"));

    // echo 1 > /sys/class/gpio/gpiochip0/subsystem/gpio_n/value
    // echo 0 > /sys/class/gpio/gpiochip0/subsystem/gpio_n/value
    // echo 1 > /sys/class/gpio/gpiochip0/subsystem/gpio_n/value

    strcat(str_gpio_sub, "/value");
    fd = open(str_gpio_sub, O_WRONLY);
    if(fd == -1)
    {
        TRACE_ERR("opening %s failed.\n", str_gpio_sub);
        return FALSE;
    }
    else
    {
        if (write(fd, "1", 1) && write(fd, "0", 1) && write(fd, "1", 1) != 1) 
        {
            TRACE_ERR("writing to %s.\n", str_gpio_sub);
            close(fd);
            return FALSE;
        }
    }
    close(fd);
    memset(str_gpio_sub + len_to_export, 0, strlen("/gpio/value") + strlen(p_autobaud_cfg->bt_reg_on_off.line_num));

    // echo line_num > /sys/class/gpio/gpiochip0/subsystem/unexport
    strncat(str_gpio_sub, "/unexport", CYHAL_GPIO_BUFFER_SIZE - strlen(str_gpio_sub) - 1);

    if (cy_platform_open_file_write(str_gpio_sub, p_autobaud_cfg->bt_reg_on_off.line_num) == FALSE){
        return FALSE;
    }
    memset(str_gpio_sub , 0, sizeof(str_gpio_sub));

    return TRUE;
}

static BOOL32 cy_platform_gpio_write(const char *dev_name, const char* offset, uint8_t value, char *str)
{
    struct gpiohandle_request rq;
    struct gpiohandle_data data;
    int fd, ret;

    // /dev/gpiochip0
    char dev_gpio[CYHAL_GPIO_BUFFER_SIZE] = "/dev/";
    strncat(dev_gpio, dev_name, CYHAL_GPIO_BUFFER_SIZE - strlen(dev_gpio) - 1);

    TRACE_LOG("Write value %d to GPIO at offset %s (OUTPUT mode) on chip %s\n", value, offset, dev_name);
    fd = open(dev_gpio, O_RDONLY);
    if (fd < 0)
    {
        TRACE_ERR("Unabled to open %s: %s\n", dev_name, strerror(errno));
        return FALSE;
    }

    rq.lineoffsets[0] = strtol(offset, NULL, 10);
    rq.flags = GPIOHANDLE_REQUEST_OUTPUT;
    strncpy(rq.consumer_label, str, GPIO_MAX_NAME_SIZE - 1);
    rq.lines = 1;
    ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &rq);
    close(fd);
    if (ret == -1)
    {
        TRACE_ERR("Unable to line handle from ioctl : %s\n", strerror(errno));
        return FALSE;
    }
    data.values[0] = value;
    ret = ioctl(rq.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
    if (ret == -1)
    {
        TRACE_ERR("Unable to set line value using ioctl : %s\n", strerror(errno));
        return FALSE;
    }

    close(rq.fd);
    return TRUE;
}


extern void wait_for_uart_recv_thread();
extern BOOL32 patch_downloaded;


/*******************************************************************************
**
** Function         cy_platform_bluetooth_init
**
** Description      This function will initialize and configures the UART port by which Bluetooth
**                  which Bluetooth stack and app can communicate with Bluetooth controller.
**
*******************************************************************************/
BOOL32 cy_platform_bluetooth_init(char* patchFile, char *device, uint32_t baudrate, uint32_t fw_download_baudrate, const cybt_controller_autobaud_config_t *p_autobaud_cfg)
{
    TRACE_LOG("Bluetooth_linux porting layer version:%s\n", BLUETOOTH_LINUX_VER);
    TRACE_LOG("patchfile %s, device %s, baud %d\n", patchFile, device, baudrate);

    if (device[0] == 'C' || device[0] == 'c' || strncmp("/dev/",device,5) == 0)
    {
        if (fw_download_baudrate)
        {
            patch_baud_rate = checkPatchBaudRate(fw_download_baudrate);
        }

        if (patchFile[0])  //Need FW download and Reset Chip to avoid previous active behevior.
        {
            if(strlen(p_autobaud_cfg->bt_reg_on_off.p_gpiochip) > 0) //GPIO control to reset controller (trigger REGON Pin from High->Low->High) 
            {
                if(p_autobaud_cfg->use_ioctl)
                {
                    //Set REGON GPIO level to High
                    if (cy_platform_gpio_write(p_autobaud_cfg->bt_reg_on_off.p_gpiochip, p_autobaud_cfg->bt_reg_on_off.line_num, GPIO_ACTIVE_HIGH, "consumer") == FALSE)
                    {
                        return FALSE;
                    }
                    //Set REGON GPIO level to Low
                    if (cy_platform_gpio_write(p_autobaud_cfg->bt_reg_on_off.p_gpiochip, p_autobaud_cfg->bt_reg_on_off.line_num, GPIO_ACTIVE_LOW, "consumer") == FALSE)
                    {
                        return FALSE;
                    }
                    //Set REGON GPIO level to High
                    if (cy_platform_gpio_write(p_autobaud_cfg->bt_reg_on_off.p_gpiochip, p_autobaud_cfg->bt_reg_on_off.line_num, GPIO_ACTIVE_HIGH, "consumer") == FALSE)
                    {
                        return FALSE;
                    }
                }
                else
                {
                    if (cy_platform_gpio_toggle_reg_on(p_autobaud_cfg) == FALSE)
                    {
                        return FALSE;
                    }
                }
            }
        }

        if(!uartInit(device, (patchFile[0] == '\0') ? app_baud_rate : patch_baud_rate))
        {
            TRACE_ERR("could not set up serial port %s at %d init_baud_rate - exiting!!!\n", device, (patchFile[0] == '\0') ? app_baud_rate : 115200);
            return FALSE;
        }
        bUseUART = TRUE;
    }

    if (patchFile[0])  //Need FW download
    {
        if(strlen(p_autobaud_cfg->bt_reg_on_off.p_gpiochip) > 0) //GPIO control for Auto Baud feature (1.CTS low then 2.REG ON from low to high) 
        {
            if(p_autobaud_cfg->use_ioctl)
            {
                if (cy_platform_gpio_write(p_autobaud_cfg->bt_reg_on_off.p_gpiochip, p_autobaud_cfg->bt_reg_on_off.line_num, GPIO_ACTIVE_HIGH, "consumer") == FALSE)
                {
                    return FALSE;
                }     
                if (cy_platform_gpio_write(p_autobaud_cfg->bt_reg_on_off.p_gpiochip, p_autobaud_cfg->bt_reg_on_off.line_num, GPIO_ACTIVE_LOW, "consumer") == FALSE)
                {
                    return FALSE;
                }
                if (cy_platform_gpio_write(p_autobaud_cfg->bt_reg_on_off.p_gpiochip, p_autobaud_cfg->bt_reg_on_off.line_num, GPIO_ACTIVE_HIGH, "consumer") == FALSE)
                {
                    return FALSE;
                }                    
            }
            else
            {
                if (cy_platform_gpio_toggle_reg_on(p_autobaud_cfg) == FALSE)
                {
                    return FALSE;
                }
            }
        }
        strncpy ((char *)PATCH_FILE_NAME, patchFile, MAX_PATH);
    }
    else{
        patch_downloaded = TRUE;  //Without enter download FW function: patch_download_check()
    }

    if (baudrate != 0)
    {
        app_baud_rate = checkAppBaudRate(baudrate);
        if (patch_downloaded == TRUE){ //if not download FW, change app baudrate.
            if (app_baud_rate != DEFAULT_APP_BAUD_RATE) //update App baurate
            {
                uart_reconfigure(app_baud_rate);
            }
        }
    }

    if (toupper(device[0]) == 'U')
    {
        bUseUART = FALSE;
        TRACE_ERR("USB Not supported   !!!\n");
        return FALSE;
    }

    if (patch_baud_rate == 3000000)
    {
        uartSend (TRUE, enter_download_mode, sizeof(enter_download_mode));
    }

    init_stack_lock();

    // Loop forever servicing timers
    if (pthread_create(&timer_thread, NULL, timerTask, NULL) < 0)
    {
        TRACE_ERR("pthread_create for timer_task failed\n");
        return FALSE;
    }

    pthread_cond_wait(&schedParamInit, &lock);

    APPLICATION_START();

    if (init_rx_queue() == FALSE)
    {
        TRACE_ERR("init rx queue fail\n");
        return FALSE;
    }

    return TRUE;
}

void cy_bt_spy_comm_init(wiced_bool_t b_tcp, int emu_instance, char *peer_ip_addr)
{
    extern void set_TCP_instance(int val);
    extern void set_TCP_enabled(BOOL32 val);

    if (b_tcp)
    {
        set_TCP_enabled(b_tcp);
    }

    configure_spy(emu_instance, peer_ip_addr);
}

void configure_spy(int emu_instance, char* peer_ip_addr)
{
    wicedx_emulator_instance = emu_instance;
    if (peer_ip_addr)
    {
        strncpy(g_peer_ip_addr, peer_ip_addr, sizeof(g_peer_ip_addr) - 1);
    }
}

void* timerTask(void *t)
{
    uint32_t        last_tc, cur_tc, sleepTime;
    uint64_t          curAbsTime;

    int policy;
    struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy);
    pthread_setschedparam(pthread_self(), policy, &param);

    CreateEvent();
    pthread_cond_signal(&schedParamInit);

    for ( ; ; )
    {
        last_tc         = getTick();
        curAbsTime      = absTimeHi + last_tc;

        linux_stack_lock(NULL);
        if (TargetTimeToWake != 0)
        {
            if (TargetTimeToWake <= curAbsTime)
            {
                sleepTime = 0;
                TargetTimeToWake = 0;
            }
            else
            {
                sleepTime = (uint32_t)(TargetTimeToWake - curAbsTime);
            }
        }
        else
            sleepTime = 60000;        // 1 minute is small enough to detect rollovers
        linux_stack_unlock(NULL);

        sleepTime = (uint32_t) sleepTime / 1000; //in seconds

        if (sleepTime != 0)
        {
            WaitEvent(sleepTime);
        }
        cur_tc = getTick();

        // Check for rollover - this assumes no timer is more than 49 days
        if (cur_tc < last_tc)
            absTimeHi += 0x100000000;

        if (sleepTime == 0)
        {
            linux_stack_lock(NULL);
            wiced_bt_process_timer();
            linux_stack_unlock(NULL);
        }
    }
    return NULL;
}

// returns current tick count in us, 64-bit
static uint64_t GetuSTicks64 (void)
{
    return (uint64_t)(( getTick () + absTimeHi ) * 1000);
}

/* Sets target time in us */
static void SetExpireTargetTime (uint64_t targetTime)
{
    linux_stack_lock(NULL);
    TargetTimeToWake = (targetTime/1000);
    SetEvent ();
    linux_stack_unlock(NULL);
}

wiced_result_t SendScoToBtHCI (uint8_t *pData, uint8_t len)
{

    // Check transport type, USB or UART
    if (bUseUART)
        uartSend (0xDD, pData, len);

    free(pData);
    return WICED_SUCCESS;
}

wiced_result_t SendCommandToBtHCI (uint8_t *pCmd, uint16_t cmd_len)
{
    // Check transport type, USB or UART
    if (bUseUART)
        uartSend (TRUE, pCmd, cmd_len);

    return (WICED_SUCCESS);
}

static uint8_t* GetAclBuffer (wiced_bt_transport_t transport, uint32_t size)
{
    return (malloc(size));
}

static uint8_t* GetScoBuffer (uint32_t size)
{
    return (malloc(size));
}

wiced_result_t SendAclToBtHCI (wiced_bt_transport_t transport, uint8_t*pData, uint16_t len)
{
    // Check transport type, USB or UART
    if (bUseUART)
        uartSend (FALSE, pData, len);

    free (pData);
    return (WICED_SUCCESS);
}

wiced_result_t SendIsoToBtHCI (uint8_t*pData, uint16_t len)
{
    // Check transport type, USB or UART
    if (bUseUART)
        uartSend (5, pData, len);

    return (WICED_SUCCESS);
}

void ProcessEventFromHCI (uint8_t *pData, uint32_t length)
{
    /* Safety check in case data is received before the stack is initialized (USB) */
    if (btu_platform_interfaces.stack_lock.pf_lock_func == NULL)
        return;

#if WAKEONLE
    if (pf_le_local_support_func != NULL) 
    {
        if (pf_le_local_support_func(pData, length))
        {
            pf_le_local_support_func = NULL;
        }
    }
#endif
    linux_stack_lock(NULL);
    wiced_bt_process_hci_events(pData, length);
    linux_stack_unlock(NULL);
}

void ProcessAclFromHCI (uint8_t *pData, uint32_t length)
{
    /* Safety check in case data is received before the stack is initialized (USB) */
    if (btu_platform_interfaces.stack_lock.pf_lock_func == NULL)
        return;

    linux_stack_lock(NULL);
    wiced_bt_process_acl_data(pData, length);
    linux_stack_unlock(NULL);
}

void ProcessIsocFromHCI (uint8_t *pData, uint32_t length)
{
    if (btu_platform_interfaces.stack_lock.pf_lock_func == NULL)
        return;

    linux_stack_lock(NULL);
    wiced_bt_process_isoc_data(pData, length);
    linux_stack_unlock(NULL);
}

void ProcessDiagFromHCI (uint8_t *pData, uint32_t length)
{
#define EDR_LMP_RECV 1
#define EDR_LMP_XMIT 0

    if (btu_platform_interfaces.stack_lock.pf_lock_func == NULL)
        return;

    if (!pData || (0 == length))
        return;

    if (EDR_LMP_RECV == pData[0])
    {
        linux_stack_lock(NULL);
        TraceHciPkt(SPY_TRACE_TYPE_LMP_RECV, &pData[1], length - 1, wicedx_emulator_instance);
        linux_stack_unlock(NULL);
    }
    else if (EDR_LMP_XMIT == pData[0])
    {
        linux_stack_lock(NULL);
        TraceHciPkt(SPY_TRACE_TYPE_LMP_XMIT, &pData[1], length - 1, wicedx_emulator_instance);
        linux_stack_unlock(NULL);
    }
    else
        return;
}

void ProcessScoFromHCI(uint8_t *pData, uint32_t length)
{
    uint16_t handle;
    uint8_t len;

    /* Safety check in case data is received before the stack is initialized (USB) */
    if (btu_platform_interfaces.stack_lock.pf_lock_func == NULL) return;

    linux_stack_lock(NULL);
    wiced_bt_process_sco_data(pData, length);
    linux_stack_unlock(NULL);
}

/////////////////////////////////////////////////
// Debug traces
//
static void SendTextTraceToSpy(char *str, uint8_t spy_instance, BYTE trace_type)
{
    char buff[1000];
    char *p;
    int x, y;
    char *timeline;
    struct timespec tstruct;
    unsigned short milli_sec;

    clock_gettime(CLOCK_REALTIME, &tstruct);
    timeline = ctime(&tstruct.tv_sec);
    milli_sec = tstruct.tv_nsec / 1000000; // covert nano second to milli seconds

#if (BTM_SECURE_CONN_CATB_CONFORMANCE_TESTER == TRUE)
    // '*' the traces so we can tell if the conformance tester code is running
    x = sprintf(buff, "%.8s.%03d * ", &timeline[11], milli_sec);
#else
    x = sprintf(buff, "%.8s.%03d   ", &timeline[11], milli_sec);
#endif

    // remove leading and trailing CR/LF
    for (y = 0, p = str; *p != 0 && (x < sizeof(buff) - 3); p++)
    {
        if (*p >= ' ')
        {
            buff[x++] = *p;
            y++;
        }
    }

    // If an empty line, just return.
    if (y == 0) return;

    buff[x++] = '\n';
    buff[x] = 0;

    printf("%s\n", buff);

    TraceHciPkt(spy_trace_types[trace_type], (BYTE *)&buff[13], sizeof(buff) - 13, spy_instance);
}

void emu_Trace(char *traceBuf, int length, wiced_bt_trace_type_t trace_type)
{
    SendTextTraceToSpy(traceBuf, wicedx_emulator_instance, trace_type);
}

/* Change %B to %p */
static void changeBtoP(const char *inBuf, char *outBuf)
{
    char ch;

    while ((ch = *inBuf++) != '\0')
    {
        if (ch != '%')
            *outBuf++ = ch;
        else
        {
            *outBuf++ = '%';
            ch = *inBuf++;
            if (ch != 'B')
                *outBuf++ = ch;
            else
            {
                outBuf--;
                *outBuf++ = '~';
                *outBuf++ = '!';
                *outBuf++ = '~';
                *outBuf++ = '%';
                *outBuf++ = 'p';
                *outBuf++ = '~';
                *outBuf++ = '!';
                *outBuf++ = '~';
                *outBuf++ = ' ';
                *outBuf++ = ' ';
                *outBuf++ = ' ';
            }
        }
    }
    *outBuf++ = '\0';
}

void debug_Printf(char *format, ...)
{
    va_list args;
    char buff[1000];
    char format_safe[1000];
    uint8_t *p_bda = NULL;
    char *p, bda_str[20];

    changeBtoP(format, format_safe);

    va_start(args, format);
    vsnprintf(buff, 1000, format_safe, args);
    va_end(args);
    buff[999] = '\0';

    if ((p = strstr(buff, "~!~")) != NULL) sscanf(p, "~!~%p~!~", &p_bda);

    if (p_bda != NULL)
    {
        sprintf(bda_str, "%02x-%02x-%02x-%02x-%02x-%02x", p_bda[0], p_bda[1], p_bda[2], p_bda[3], p_bda[4], p_bda[5]);
        memcpy(p, bda_str, 17);
    }

    SendTextTraceToSpy(buff, wicedx_emulator_instance, FALSE);
}

void debug_PrintError(char *format, ...)
{
    va_list args;
    char buff[1000] = DEBUG_ERROR_MSG;
    char format_safe[1000];
    uint8_t *p_bda = NULL;
    char *p, bda_str[20];

    changeBtoP(format, format_safe);

    va_start(args, format);
    vsnprintf(buff + strlen(DEBUG_ERROR_MSG), 1000 - strlen(DEBUG_ERROR_MSG), format_safe, args);
    va_end(args);
    buff[999] = '\0';

    if ((p = strstr(buff, "~!~")) != NULL) sscanf(p, "~!~%p~!~", &p_bda);

    if (p_bda != NULL)
    {
        sprintf(bda_str, "%02x-%02x-%02x-%02x-%02x-%02x", p_bda[0], p_bda[1], p_bda[2], p_bda[3], p_bda[4], p_bda[5]);
        memcpy(p, bda_str, 17);
    }

    SendTextTraceToSpy(buff, wicedx_emulator_instance, TRUE);
}

uint8_t *scru_dump_hex(uint8_t *p, char *p_title, uint32_t len, uint32_t trace_layer, uint32_t trace_type)
{
    uint32_t xx, yy;
    char buff1[100], buff2[20];

    if (p_title) 
    {
        TRACE_LOG("%s\n", p_title);
    }

    memset(buff2, ' ', 16);
    buff2[16] = 0;

    yy = sprintf(buff1, "%04x: ", 0);
    for (xx = 0; xx < len; xx++)
    {
        if ((xx) && ((xx & 15) == 0))
        {
            TRACE_LOG("    %s  %s\n", buff1, buff2);
            yy = sprintf(buff1, "%04x: ", xx);
            memset(buff2, ' ', 16);
        }
        yy += sprintf(&buff1[yy], "%02x ", *p);

        if ((*p >= ' ') && (*p <= 'z'))
            buff2[xx & 15] = *p;
        else
            buff2[xx & 15] = '.';

        p++;
    }

    /* Pad out the remainder */
    for (;; xx++)
    {
        if ((xx & 15) == 0)
        {
            TRACE_LOG("    %s  %s\n", buff1, buff2);
            break;
        }
        yy += sprintf(&buff1[yy], "   ");
    }

    return (p);
}

///////////////////////////////////////////////////////////////////////////////////////////////
static void SendHciTraceToSpy(wiced_bt_hci_trace_type_t type, uint16_t len, uint8_t *p_data)
{
    /* Don't trace while streaming, or if using USB and old Spy (instance == 0) */
    if ((bSupressProtocolTraces) || ((bUseUART == FALSE) && (wicedx_emulator_instance == 0))) return;

    if (type == HCI_TRACE_EVENT)
        TraceHciPkt(SPY_TRACE_TYPE_HCI_EVENT, p_data, len, wicedx_emulator_instance);
    else if (type == HCI_TRACE_COMMAND)
        TraceHciPkt(SPY_TRACE_TYPE_HCI_COMMAND, p_data, len, wicedx_emulator_instance);
    else if (type == HCI_TRACE_INCOMING_ACL_DATA)
        TraceHciPkt(SPY_TRACE_TYPE_ACL_RX, p_data, len, wicedx_emulator_instance);
    else if (type == HCI_TRACE_OUTGOING_ACL_DATA)
        TraceHciPkt(SPY_TRACE_TYPE_ACL_TX, p_data, len, wicedx_emulator_instance);
    else if (type == HCI_TRACE_INCOMING_ISO_DATA)
        TraceHciPkt(SPY_TRACE_TYPE_ISO_RX, p_data, len, wicedx_emulator_instance);
    else if (type == HCI_TRACE_OUTGOING_ISO_DATA)
        TraceHciPkt(SPY_TRACE_TYPE_ISO_TX, p_data, len, wicedx_emulator_instance);
    else if (type == HCI_TRACE_INCOMING_SCO_DATA)
        TraceHciPkt(SPY_TRACE_TYPE_SCO_RX, p_data, len, wicedx_emulator_instance);
    else if (type == HCI_TRACE_OUTGOING_SCO_DATA)
        TraceHciPkt(SPY_TRACE_TYPE_SCO_TX, p_data, len, wicedx_emulator_instance);
    else
        TRACE_ERR("SendHciTraceToSpy - unknown type: %d\n", type);
}

BOOL32 SendLmpLogEnable(BOOL32 enable)
{
    uint8_t lmp_enable_log[] = { 0xf0, 1};
    uint8_t lmp_disable_log[] = { 0xf0, 0};
    uint8_t lmp_nop[] = { 0 };
    static uint8_t first = 1;
    TRACE_LOG("SendLmpLogEnable \n");
    if (enable)
    {
        if(first)
            first = 0;
        else
            uartSend(7, lmp_nop, sizeof(lmp_nop));

        uartSend(7, lmp_enable_log, sizeof(lmp_enable_log));
    }
    else
    {
        uartSend(7, lmp_nop, sizeof(lmp_nop));
        uartSend(7, lmp_disable_log, sizeof(lmp_disable_log));
    }

    return (TRUE);
}

BOOL32 SendSpyLogEnable(BOOL32 enable, uint8_t *file_name, uint8_t file_name_len)
{
    uint8_t spy_log_enable[100] = {0};
    uint8_t spy_log_disable[] = {1};

    if(file_name_len)
        memcpy(&spy_log_enable[1], file_name, file_name_len);

    if (enable)
        TraceHciPkt(SPY_TRACE_TYPE_SPY_COMMAND, spy_log_enable, sizeof(spy_log_enable), wicedx_emulator_instance);
    else
        TraceHciPkt(SPY_TRACE_TYPE_SPY_COMMAND, spy_log_disable, sizeof(spy_log_disable), wicedx_emulator_instance);

    return (TRUE);
}

void SendSpyClearTraces(void)
{
    uint8_t spy_clear_traces[] = {2};

    TraceHciPkt(SPY_TRACE_TYPE_SPY_COMMAND, spy_clear_traces, sizeof(spy_clear_traces), wicedx_emulator_instance);

    return;
}

/*******************************************************************************
** Function     linux_stack_lock
**
** Description  this funciton implement a mutex lock of cs_mutex,
**              cs_mutex is a recursive mutex set in init_stack_lock().
**              this function also as a real mutex lock protection of btstack if btstack enable lock.
**              so in order to avoid race condition and deadlock, use this recursive mutex api for btstack
**              and linux CE, every time when CE call btstack api should lock first, and unlock after api complete.
**
** Param:
**    void *pv_ctx
**          this parameter will send from btstack, inorder to align btstack, need it although we did not use it 
** Return:
**    void
*******************************************************************************/
void linux_stack_lock(void * pv_ctx)
{
    pthread_mutex_lock( &cs_mutex );
}

/*******************************************************************************
** Function     linux_stack_unlock
**
** Description  this funciton implement a mutex unlock of cs_mutex.
**              cs_mutex is a recursive mutex set in init_stack_lock().
**              this function also as a real mutex unlock protection of btstack if btstack enable lock
**              so in order to avoid race condition and deadlock, use this recursive mutex api for btstack
**              and linux CE, every time when CE call btstack api should lock first, and unlock after api complete.
**
** Param:
**    void *pv_ctx
**          this parameter will send from btstack , inorder to align btstack, need it although we did not use it 
**
** Return:
**    void
*******************************************************************************/
void linux_stack_unlock(void* pv_ctx)
{
    pthread_mutex_unlock( &cs_mutex );
}

static void linux_stack_exception(uint16_t code, char* msg, void* ptr)
{
    TRACE_ERR("!!!! EXCEPTION Code: 0x%x  Message: %s ptr:%p\n", code, msg, ptr);
    abort();
}

static void * platform_alloc(uint32_t size)
{
    return malloc((size_t)size);
}

char stack_trace_buffer[150];

void wiced_bt_platform_interface_init(void)
{
    TRACE_LOG("\n");
    wiced_bt_stack_platform_t platform_intf = { 0 };

    platform_intf.pf_exception          = linux_stack_exception;
    platform_intf.pf_os_malloc          = platform_alloc;
    platform_intf.pf_os_free            = free;
    platform_intf.pf_get_tick_count_64  = GetuSTicks64;
    platform_intf.pf_set_next_timeout   = SetExpireTargetTime;
    platform_intf.pf_write_acl_to_lower = SendAclToBtHCI;
    platform_intf.pf_write_sco_to_lower = SendScoToBtHCI;
    platform_intf.pf_write_iso_to_lower = SendIsoToBtHCI;
    platform_intf.pf_write_cmd_to_lower = SendCommandToBtHCI;
    platform_intf.pf_get_acl_to_lower_buffer = GetAclBuffer;
    platform_intf.pf_get_sco_to_lower_buffer = GetScoBuffer;

    platform_intf.pf_hci_trace_cback_t  = SendHciTraceToSpy;
    platform_intf.pf_debug_trace        = emu_Trace;
    platform_intf.pf_patch_download     = patch_download_check;

    platform_intf.stack_lock.pf_lock_func   = linux_stack_lock;
    platform_intf.stack_lock.pf_unlock_func = linux_stack_unlock;

    platform_intf.trace_buffer = stack_trace_buffer;
    platform_intf.trace_buffer_len = sizeof(stack_trace_buffer);

    /* Provide stack with platform interface functions */
    wiced_bt_stack_platform_initialize(&platform_intf);
    TRACE_LOG("Starting Application");
}

/*******************************************************************************
**
** Function         wait_controller_reset_ready
**
** Description      This function is used to wait for reset commnad sent after updating baudrate
**
*******************************************************************************/
void wait_controller_reset_ready(void)
{
    // The reset after update baudrate
    wait_controller_reset();
}

/*******************************************************************************
** Function         platform_gpio_write
**
** Description      This function is the interface of cy_platform_gpio_write
** 
** Param:
**	dev_name:	gpiochipx
**	offset:		gpio num or offset
**	value:		1 or 0
**	str:	        the gpio name for user input	
** Return:
**	BOOL32:		write result
**
*******************************************************************************/
BOOL32 platform_gpio_write(const char *dev_name, const char* offset, uint8_t value, char *str)
{
    return cy_platform_gpio_write(dev_name, offset, value, str); 
}

/*******************************************************************************
**
** Function         cy_platform_gpio_poll
**
** Description      This function will poll the specific gpio event,
** 		    detect the gpio rising event or falling event,
** 		    thread will block in poll
** 		    after event occur, run the callback
** 
** Param:
**	void *info:
**             it is cybt_gpio_event_t *args
**
** Return:
**	void *	
**
*******************************************************************************/
static void* cy_platform_gpio_poll(void *info)
{
    int fd = 0, ret = 0;
    int offset = 0;
    struct gpioevent_request event_req = {0};
    struct gpioevent_data event_data = {0};
    struct pollfd poll_fd;
    cybt_gpio_event_t *args = info;

    if (args == NULL)
    {
        TRACE_ERR("args is NULL\n");
    }
    offset = atoi(args->gpio_event.line_num);

    // /dev/gpiochip0
    char dev_gpio[CYHAL_GPIO_BUFFER_SIZE] = "/dev/";
    strncat(dev_gpio, args->gpio_event.p_gpiochip, CYHAL_GPIO_BUFFER_SIZE - strlen(dev_gpio) - 1);

    fd = open(dev_gpio, O_RDONLY);
    if (fd < 0)
    {
        TRACE_ERR("Unabled to open %s: %s\n", dev_gpio, strerror(errno));
        return NULL;
    }
    /* event request */
    event_req.lineoffset = offset;
    event_req.handleflags = GPIOHANDLE_REQUEST_INPUT;
    event_req.eventflags = args->gpio_event_flag;

    ret = ioctl(fd, GPIO_GET_LINEEVENT_IOCTL, &event_req);
    if (ret == -1)
    {
        TRACE_ERR("Unable to get line event using ioctl : %s\n",  strerror(errno));
        close(fd); 
        return NULL;
    }
    if (close(fd) == -1)
    {
        TRACE_ERR("Failed to close GPIO character device file: %s\n", strerror(errno));
        return NULL;
    }

    poll_fd.fd = event_req.fd;
    poll_fd.events = POLLIN;

    TRACE_LOG("POLL GPIO %d on %s\n", offset, args->gpio_event.p_gpiochip);
    ret = poll(&poll_fd, 1, INFITIME);

    event_data.timestamp = 0;
    event_data.id = 0;

    // must read out event 
    ret = read(event_req.fd, &event_data, sizeof(event_data));
    if (ret == -1)
    {
        TRACE_ERR("read event error:%s\n", strerror(errno));
    }

    // close event req fd
    close(event_req.fd);
    if (args->gpio_event_cb != NULL)
    {
        args->gpio_event_cb();
    } else {
        TRACE_ERR("NO CALLBACK assign\n");
    }

    return NULL;
}

/*******************************************************************************
**
** Function         platform_gpio_poll
**
** Description      This function create a thread to run cy_platform_gpio_poll
**
** Param:
**	cybt_gpio_event_t *gpio_event_args
**
** Return:
**	BOOL32:	    result
**
*******************************************************************************/
BOOL32 platform_gpio_poll(cybt_gpio_event_t *gpio_event_args)
{
    if (pthread_create(&gpioThread, NULL, cy_platform_gpio_poll, (void*)gpio_event_args) < 0)
    {
        TRACE_ERR("pthread_create failed\n");
        return FALSE;
    }
    return TRUE;
}

/*******************************************************************************
**
** Function         wiced_exp_set_local_support_cback
**
** Description      This function set a callback func to check le local support.
                    use in CE:wakeonLe and lib:wiced_exp
**
** Param: pf_le_local_support_func_t p_func
                callback func for le_local_support check 
**
** Return:
**	BOOL32: None
**
*******************************************************************************/
void wiced_exp_set_local_support_cback(pf_le_local_support_func_t p_func)
{
    pf_le_local_support_func = p_func;
}
