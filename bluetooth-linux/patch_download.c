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
 **  Name:          patch_download.c
 **
 **  Description: Patch ram download and Post HCI_RESET procedures
 **
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdint.h>
#include <unistd.h>
#include <wiced_bt_dev.h>
#include <wiced_result.h>
#include "hcidefs.h"
#include "wiced_bt_types.h"
#include <unistd.h>
#include <sys/time.h>
#include <assert.h>
#include "wiced_result.h"
#include "data_types.h"
#include "wiced_bt_stack_platform.h"
#include "platform_linux.h"
#include "patch_download.h"
#include "log.h"


pthread_cond_t cond_controller_reset = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex_controller_reset = PTHREAD_MUTEX_INITIALIZER;

#define tBTM_VSC_CMPL wiced_bt_dev_vendor_specific_command_complete_params_t
#define BTE_PRM_FORMAT_HCD    0x01
#define PATCH_DOWNLOAD_TIMEOUT_SEC 20 //FW donwload timeout

enum
{
    BRCM_PRM_STS_CONTINUE = 0,
    BRCM_PRM_STS_COMPLETE,
    BRCM_PRM_STS_ABORT
};

#define HCI_BRCM_UPDATE_BAUDRATE_CMD        (0x0018 | HCI_GRP_VENDOR_SPECIFIC)  
#define HCI_BRCM_UPDATE_BAUD_RATE_UNENCODED_LENGTH      0x06
#define tBTM_STATUS wiced_result_t
/* Patch filename from application */
extern char PATCH_FILE_NAME[];
extern BOOL32 bUseUART;
typedef void (tBRCM_PRM_CBACK)(tBRCM_PRM_STATUS status);
extern BOOL32 BRCM_PrmInit(tBRCM_PRM_CBACK* p_cb, uint8_t* p_patch_buf, uint32_t patch_buf_len,
    uint32_t address, uint8_t format_type);
typedef void (tBTM_CMPL_CB)(void* p1);
extern void BTM_DeviceReset(tBTM_CMPL_CB* p_cb);
typedef void (tBTM_VSC_CMPL_CB)(tBTM_VSC_CMPL* p1);


extern tBTM_STATUS BTM_VendorSpecificCommand(uint16_t opcode,
    uint8_t param_len,
    uint8_t* p_param_buf,
    tBTM_VSC_CMPL_CB* p_cb);
// Flag to indicate whether patch ram download is done
BOOL32 patch_downloaded = FALSE;
extern unsigned int patch_baud_rate;

/* Patchfile variables used by brcm_prm */
typedef struct
{
    int         btewiced_patchram_len;
    uint8_t     *p_btewiced_patchram_buf;
}tPATCH_RAM_INFO;
tPATCH_RAM_INFO patch_info;

/* Static function decrations
*/
static void patch_download_complete (tBRCM_PRM_STATUS status);

static BOOL32 Send_UpdateBaudRateVSC (unsigned int baudrate);

static void update_baudrate_cback (tBTM_VSC_CMPL* p);

static void signal_controller_reset (void);

// user specified baud rate
extern unsigned int app_baud_rate;
extern BOOL32 uart_reconfigure(uint32_t dwBaudRate);

void GKI_delay(uint32_t timeout)
{
#define MICROSECONDS 1000
    usleep(timeout * MICROSECONDS);
}

// patch_download_check
// This function is called after HCI Reset is complete
// Download the patch ram if needed and update serial port
// baud rate if needed
void patch_download_check (void)
{
    FILE    *fd;
    char    *p_file;
    // If patch ram download has not been done yet and user specified
    // patch ram file, download it now.
    if (!patch_downloaded && (PATCH_FILE_NAME[0] != '\0'))
    {
        TRACE_LOG("Downloading patchfile %s\n", PATCH_FILE_NAME);

        patch_downloaded = TRUE;

        /* open patchfile, read it into a buffer */
        if ((fd = fopen(PATCH_FILE_NAME, "rb")) != NULL)
        {
            /* For tracing - point to start of filename (after path) */
            p_file = strrchr (PATCH_FILE_NAME, '\\');
            if (p_file == NULL)
                p_file = strrchr (PATCH_FILE_NAME, '/');
            if (p_file)
                p_file++;
            else
                p_file = PATCH_FILE_NAME;

            /* Get filelength */
            fseek(fd, 0, SEEK_END);     /* seek to end of file              */
            patch_info.btewiced_patchram_len = ftell(fd);    /* get current file pointer         */
            fseek(fd, 0, SEEK_SET);     /* seek back to beginning of file   */

            /* Allocate buffer to hold patch data */
            if ((patch_info.btewiced_patchram_len > 0) && (patch_info.p_btewiced_patchram_buf = (uint8_t *)malloc(patch_info.btewiced_patchram_len)) != NULL)
            {
                size_t read_len = fread(patch_info.p_btewiced_patchram_buf, patch_info.btewiced_patchram_len, 1, fd);
                if  (read_len > 0)
                {
                    TRACE_LOG("Downloading patchfile %s (size: %i)\n", p_file, patch_info.btewiced_patchram_len);

                    /* Download patch using static memeory mode */
                    BRCM_PrmInit (patch_download_complete, patch_info.p_btewiced_patchram_buf, patch_info.btewiced_patchram_len, 0, BTE_PRM_FORMAT_HCD);
                }
                else
                {
                    TRACE_ERR("reading patch file\n");
                    free(patch_info.p_btewiced_patchram_buf);
                }
            }
            else
            {
                TRACE_ERR("Unable to buffer to hold patchram (%i bytes), or invalid patch file\n", patch_info.btewiced_patchram_len);
            }
            fclose(fd);

			// Return here, but will come back after patchram d/l
			// patch_download_complete() calls HCI reset, which again calls this function.
			return;
        }
        else
        {
            TRACE_ERR("Unable to open patchfile %s\n", PATCH_FILE_NAME);
        }
    }
    else{
        TRACE_LOG("Proceeding to post reset..\n");
        wiced_bt_continue_reset();
    }
}

// Callback on patch ram download completion
static void patch_download_complete(tBRCM_PRM_STATUS status)
{
    TRACE_LOG("%s\n",__FUNCTION__);
    free(patch_info.p_btewiced_patchram_buf);

    if (status == BRCM_PRM_STS_COMPLETE)
    {
        TRACE_LOG("\nPatch has successfully downloaded. Reset chip\n");
        // If serial port baud rate needs to be updated, do it now.
        if(bUseUART)
        {
           if(patch_baud_rate != DEFAULT_PATCH_BAUD_RATE)
           {
                if(!uart_reconfigure(DEFAULT_PATCH_BAUD_RATE))
                {
                        TRACE_ERR("Uart reconfiguring is failed\n");
                        return;
                }
            }
            GKI_delay(500); //For Hatchet2 suggest

    	//although host use 115200 no need send to controller, we still send it to trigger stack enter BTM_ENABLE_STATE and send reset to controller
        TRACE_LOG("Sending UpdateBaudRateVSC \n");
        Send_UpdateBaudRateVSC(app_baud_rate);
        }
        else
        {
            wiced_bt_continue_reset();
        }
        return;
    }
    else
    {
        TRACE_ERR("Patch downloading is failed status=0x%x\n", status);
    }
}

/*******************************************************************************
 **
 ** Function            Send_UpdateBaudRateVSC
 **
 ** Description         Updates the baud rate for this HCI connection
 **
 ** Returns             TRUE:	Command was sent
 **                     FALSE:  Command not supported or transport busy
 **
 *******************************************************************************/
static BOOL32 Send_UpdateBaudRateVSC (unsigned int baudrate)
{
    tBTM_STATUS result;
    uint8_t     hci_data[HCI_BRCM_UPDATE_BAUD_RATE_UNENCODED_LENGTH];

    /* Baudrate is loaded LittleEndian */
    hci_data[0] = 0;
    hci_data[1] = 0;
    hci_data[2] = baudrate & 0xFF;
    hci_data[3] = (baudrate >> 8) & 0xFF;
    hci_data[4] = (baudrate >> 16) & 0xFF;
    hci_data[5] = (baudrate >> 24) & 0xFF;

    /* Send the command to the host controller */
    result = BTM_VendorSpecificCommand(HCI_BRCM_UPDATE_BAUDRATE_CMD, HCI_BRCM_UPDATE_BAUD_RATE_UNENCODED_LENGTH,
                                       hci_data, update_baudrate_cback);

    return ((result == WICED_BT_SUCCESS) || (result == WICED_BT_PENDING)) ? TRUE : FALSE;
}


/* Callback when baudrate has been updated
*/
static void update_baudrate_cback (tBTM_VSC_CMPL* p)
{
    TRACE_LOG("result = %X\n", p->p_param_buf[0]);
    TRACE_LOG("changed baudrate to %d\n", app_baud_rate);
    uart_reconfigure(app_baud_rate);
    wiced_bt_continue_reset();
    signal_controller_reset();
}

/*******************************************************************************
 **
 ** Function            wait_controller_reset, signal_controller_reset
 **
 ** Description         These two functions are a pair for waiting and signaling controller reset 
 **
 *******************************************************************************/
void wait_controller_reset (void)
{
    int err = 0;
    struct timespec timewait = {0};
    struct timeval now = {0};

    gettimeofday(&now, NULL);
    timewait.tv_nsec = 0;
    timewait.tv_sec = now.tv_sec + PATCH_DOWNLOAD_TIMEOUT_SEC;

    pthread_mutex_lock(&mutex_controller_reset);
    err = pthread_cond_timedwait(&cond_controller_reset, &mutex_controller_reset, &timewait);
    if (err != 0){
        TRACE_ERR("FW download Timeout Fail! err:%d\n", err);
        fflush(stdout);
        assert(0);
    }
    pthread_mutex_unlock(&mutex_controller_reset);
}

static void signal_controller_reset (void)
{
    TRACE_LOG("HCI reset has been sent");
    pthread_mutex_lock(&mutex_controller_reset);
    pthread_cond_signal(&cond_controller_reset);
    pthread_mutex_unlock(&mutex_controller_reset);

}
