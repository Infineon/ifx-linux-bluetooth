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
 * File Name: headset.c
 *
 * Description: This is the source file for headset CE application.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

#include "headset.h"

#include <stdio.h>
#include <pthread.h>
#include <sys/time.h>

#include "wiced_bt_hfp_hf.h"
#include "wiced_bt_hfp_hf_int.h"
#include "wiced_bt_utils.h"
#include "wiced_bt_gatt.h"

#include "headset_control.h"
#include "headset_control_le.h"
#include "bt_hs_spk_handsfree.h"
#include "alsa_playback.h"
#include "log.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define MAX_PATH 256
#define HEADSET_APP_HEAP_SIZE      1024*7
#define INQUERY_TIMEOUT_SEC    10 //Inquery Timeout

wiced_bt_heap_t* headset_heap;
wiced_bt_lock_t headset_heap_lock;

static pthread_cond_t cond_inquery_reset = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t mutex_inquery_reset = PTHREAD_MUTEX_INITIALIZER;
/******************************************************
 *                      functions declare
 ******************************************************/
#if (RPC_CONTROL_ENABLE == TRUE)
extern void InitializeRpc(void);
#endif


static void handsfree_build_send_at_cmd(
                                uint16_t handle,
                                char *cmd,
                                uint8_t arg_type,
                                uint8_t arg_format,
                                const char *p_arg,
                                int16_t int_arg
                            );

static void handset_inquiry_result_cback(wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data);

/******************************************************
 *                      functions
 ******************************************************/

/*****************************************************************************
* Function Name: APPLICATION_START
******************************************************************************
* Summary: CE Application Start function 
*          
*     
*
* Parameters: none
*   
* 
* Return: none
*              
*   
*
****************************************************************************/
void APPLICATION_START(void )
{
    btheadset_control_init();
#if (WICED_APP_LE_INCLUDED == TRUE)
    hci_control_le_init();
#endif

#if (RPC_CONTROL_ENABLE == TRUE)
    InitializeRpc();
#endif

    /* Create a buffer heap, make it the default heap.  */
    headset_heap = wiced_bt_create_heap("headset", NULL, HEADSET_APP_HEAP_SIZE, NULL, WICED_TRUE);
    TRACE_LOG("Wiced headset_pro Application Started...");
}


/*****************************************************************************
* Function Name: headset_get_rfcomm_handle_idx
******************************************************************************
* Summary: Get the RF COMM handle index
*          
*     
*
* Parameters: none
*   
* 
* Return:  uint16_t:
*               the index ID, 0 on no handle
*   
*
****************************************************************************/
uint16_t headset_get_rfcomm_handle_idx(void){
    return bt_hs_spk_handsfree_get_rfcomm_handle();
}

/*****************************************************************************
* Function Name: headset_is_valid_state
******************************************************************************
* Summary: Check current state is valid for cmd
*          
*     
*
* Parameters: int user_input
*   
* 
* Return:  bool:
*               true on success and false on invalid    
*   
*
****************************************************************************/
bool headset_is_valid_state (int user_input){
    bool res = true;
    if ((user_input < NO_SUPPORT_CMD) && (user_input >= 0)){
        switch(user_input)
        {
            case ENABLE_GFPS:
            case SET_VISIBILITY:
            case SET_PAIRING_MODE:
            case SET_INQUIRY:
            case HF_CONNECT:
            {
                if (headset_get_br_connect()){
                    TRACE_WNG("Already Connected. not allowed!");
                    res = false;
                }
            }
            break;
            case HF_DISCONNECT:
            {
                if (!headset_get_br_connect()){
                    TRACE_WNG("no Connect. not allowed!");
                    res = false;
                }
            }
            break;

            case DIAL_NUM:
            case REDIAL:
            {
                if (!headset_get_br_connect()){
                    TRACE_WNG("no Connect. not allowed!");
                    res = false;
                }
                else{
                    if (headset_get_call_active())
                    {
                        TRACE_LOG("call active now, not allowed!");
                        res = false;
                    }
                }
            }
            break;
            case ANSWER_CALL:
            case HANGUP_CALL:
            case QUERY_CUR_CALLS:
            case SET_SPK_VOL:
            case SET_MIC_VOL:
            case SUBSCIBER_NUM_INFO:
            {
                if (!headset_get_call_active())
                {
                    TRACE_WNG("No call active not allowed!");
                    res = false;
                }
            }
            break;
  
            
            case AVRCP_ACTION_VOLUME_UP: //cmd: 17
            {
                if (!headset_get_br_connect()){
                    TRACE_WNG("no Connect. not allowed!");
                    res = false;
                }
                else{
                    if(headset_get_playstate() == AVRC_PLAYSTATE_ERROR){
                        TRACE_WNG("No playstate not allowed!");
                        res = false;
                    }
                }

            }
            break;
            case AVRCP_ACTION_VOLUME_DOWN: //cmd: 18
            case AVRCP_ACTION_PAUSE_PLAY: //cmd: 19
            case AVRCP_ACTION_FORWARD: //cmd: 20
            case AVRCP_ACTION_BACKWARD: //cmd: 21
            case AVRCP_ACTION_STOP: //cmd: 22
            case AVRCP_ACTION_FAST_FORWARD: //cmd: 23
            case AVRCP_ACTION_FAST_REWIND: //cmd: 24
            case AVRCP_ACTION_UNIT_INFO: //cmd: 25
            {
                if (!headset_get_br_connect()){
                    TRACE_WNG("no Connect. not allowed!");
                    res = false;
                }
                else{
                    if(headset_get_playstate() == AVRC_PLAYSTATE_ERROR){
                        TRACE_WNG("No playstate not allowed!");
                        res = false;
                    }
                }

            }

            default:
                break;
        }
        return res;
    }
    else{
        return false;
    }
}

/*******************************************************************************
* Function Name: headset_handle_set_visibility
********************************************************************************
* Summary:
*   Sets the device discoverable and connectability mode
*
* Parameters:
*   bool discoverability : true-discoverable, false-Non-Discoverable
*   bool connectability :  true-Connectable,  false-Non-Connectable
*
* Return:
*   NONE
*
*******************************************************************************/
void headset_handle_set_visibility(bool discoverability, bool connectability)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    result = wiced_bt_dev_set_discoverability((discoverability != false) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE ,
                                        BTM_DEFAULT_DISC_WINDOW,
                                        BTM_DEFAULT_DISC_INTERVAL);
    if (WICED_BT_SUCCESS != result)
    {
        TRACE_ERR("wiced_bt_dev_set_discoverability failed. Status = %x", result);
    }

     result = wiced_bt_dev_set_connectability((connectability != false) ? WICED_TRUE : WICED_FALSE ,
                                            BTM_DEFAULT_CONN_WINDOW,
                                            BTM_DEFAULT_CONN_INTERVAL);
    if (WICED_BT_SUCCESS != result)
    {
        TRACE_ERR("wiced_bt_dev_set_connectability failed. Status = %x", result);
    }
    
}

/*******************************************************************************
* Function Name: handset_handle_set_pairability
********************************************************************************
* Summary:
*   Enable or disable pairing
*
* Parameters:
*   bool allowed : true - enable pairing , false - disable pairing
*
* Return:
*   NONE
*
*******************************************************************************/
void handset_handle_set_pairability (bool allowed)
{
    wiced_bt_set_pairable_mode(allowed, 0);
    TRACE_LOG("Set the pairing allowed to %d", allowed);
}

/*******************************************************************************
* Function Name: handsfree_inquiry
********************************************************************************
* Summary:
*   Starts or Stops Device Inquiry
*
* Parameters:
*   bool enable : true - starts Inquiry, false - Stops Inquiry
*
* Return:
*   wiced_result_t: result 
*
*******************************************************************************/
wiced_result_t handset_handle_set_inquiry (bool enable){
    wiced_result_t result;
    wiced_bt_dev_inq_parms_t params;

    if (enable == true)
    {
        memset(&params, 0, sizeof(params));

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = INQUIRY_DURATION;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry(&params, &handset_inquiry_result_cback);
    }
    else if (enable == false)
    {
        result = wiced_bt_cancel_inquiry();
    }
    else
    {
        result = WICED_BADVALUE;
    }
    TRACE_LOG("Inquiry Enable:%d Result:%d",enable, result);
    return result;
}

/*******************************************************************************
* Function Name: handsfree_inquiry_result_cback
********************************************************************************
* Summary:
*   Callback function called from stack for Inquiry Results
*
* Parameters:
*   wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result : Inquiry results
*   uint8_t *p_eir_data : EIR data
*
* Return:
*
*******************************************************************************/
static void handset_inquiry_result_cback(wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data)
{
    if (p_inquiry_result == NULL)
    {
        TRACE_LOG("Inquiry Complete");
        signal_inquery_done();
    }
    else
    {
        TRACE_LOG("--------------------------------------------");
        TRACE_LOG("Inquiry Result: %02X %02X %02X %02X %02X %02X",
                            p_inquiry_result->remote_bd_addr[0], p_inquiry_result->remote_bd_addr[1],
                            p_inquiry_result->remote_bd_addr[2], p_inquiry_result->remote_bd_addr[3],
                            p_inquiry_result->remote_bd_addr[4], p_inquiry_result->remote_bd_addr[5]);
        TRACE_LOG("Clock Offset = 0x%x", p_inquiry_result->clock_offset);
        TRACE_LOG("RSSI = %d", p_inquiry_result->rssi);
        TRACE_LOG("--------------------------------------------");
    }
}

/*******************************************************************************
* Function Name: handsfree_build_send_at_cmd
********************************************************************************
* Summary:
*   Builds the AT command for the HF command
*
* Parameters:
*   uint16_t handle : Connection Handle
*   char *cmd : HF command to be sent
*   uint8_t arg_type : WICED_BT_HFP_HF_AT_SET - command for setting the value
*                      AT+<cmd>=<val>
*                      WICED_BT_HFP_HF_AT_READ - command for getting the value
*                      AT+<cmd>?
*   uint8_t arg_format : Format of the command value/data
*                       (WICED_BT_HFP_HF_AT_FMT_INT/WICED_BT_HFP_HF_AT_FMT_STR)
*   const char *p_arg : command value in case the arg_format is string
*   int16_t int_arg :  command value in case the arg_format is int
*
* Return:
*
*******************************************************************************/
static void handsfree_build_send_at_cmd (
                                            uint16_t handle,
                                            char *cmd,
                                            uint8_t arg_type,
                                            uint8_t arg_format,
                                            const char *p_arg,
                                            int16_t int_arg
                                        )
{
    char buf[WICED_BT_HFP_HF_AT_MAX_LEN + 16];
    char *p = buf;
    uint32_t arglen = 0;

    if (NULL == cmd)
    {
        TRACE_ERR("error cmd to be sent is null");
    }
    else
    {
        if (p_arg != NULL){
            arglen = strlen(p_arg);
        }
        if ((strlen(cmd) + arglen) > WICED_BT_HFP_HF_AT_MAX_LEN){
            TRACE_ERR("Cmd too long!");
            return;
        }

        memset (buf, 0, (WICED_BT_HFP_HF_AT_MAX_LEN+16));

        *p++ = 'A';
        *p++ = 'T';

        /* copy result code string */
        memcpy(p,cmd, strlen(cmd));
        p += strlen(cmd);

        if (arg_type == WICED_BT_HFP_HF_AT_SET)
        {
            *p++ = '=';

        }
        else if (arg_type == WICED_BT_HFP_HF_AT_READ)
        {
            *p++ = '?';

        }
        else if (arg_type == WICED_BT_HFP_HF_AT_TEST)
        {
            *p++ = '=';
            *p++ = '?';

        }

        /* copy argument if any */
        if (arg_format == WICED_BT_HFP_HF_AT_FMT_INT)
        {
            p += utl_itoa((uint16_t) int_arg, p);
        }
        else if (arg_format == WICED_BT_HFP_HF_AT_FMT_STR)
        {
            if (p_arg != NULL){
                utl_strcpy(p, (char *)p_arg);
                p += arglen;
            }
        }

        /* finish with \r*/
        *p++ = '\r';

        TRACE_LOG("SENDING AT CMD << %s\n", buf);

        wiced_bt_hfp_hf_send_at_cmd(handle,buf);
    }
}

/*******************************************************************************
* Function Name: headset_send_at_command  //handsfree_send_at_command
********************************************************************************
* Summary:
*   calls handsfree_build_send_at_cmd to build and sent the AT command
*
* Parameters:
*   uint16_t handle : connection handle
*   uint8_t command : command to be sent
*   int num : AT command arguement / value in case of integer
*   uint8_t* p_data : AT command arguement / value in case of string
*
* Return:
*   NONE
*
*******************************************************************************/
void headset_send_at_command (uint16_t handle, uint8_t command, int num, uint8_t* p_data)
{
    switch (command)
    {
        case HF_AT_CMD_VGS:
            wiced_bt_hfp_hf_notify_volume (handle,
                    WICED_BT_HFP_HF_SPEAKER, num);
#ifdef LINUX_PLATFORM
            alsa_set_volume(num * ALSA_VOLUME_HFP_RATIO);
#endif
            break;

        case HF_AT_CMD_VGM:
            wiced_bt_hfp_hf_notify_volume (handle,
                    WICED_BT_HFP_HF_MIC, num);
            break;

        case HF_AT_CMD_BINP:
            handsfree_build_send_at_cmd(handle, "+BINP",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, 1);
            break;

        case HF_AT_CMD_CHLD:
            wiced_bt_hfp_hf_perform_call_action(handle,
                    WICED_BT_HFP_HF_CALL_ACTION_HOLD_0 + num,(char *)p_data);
            break;

        case HF_AT_CMD_BVRA:
            handsfree_build_send_at_cmd(handle, "+BVRA",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, num);
            break;

        case HF_AT_CMD_CMEE:
            handsfree_build_send_at_cmd(handle, "+CMEE",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, num);
            break;

        case HF_AT_CMD_ATA:
            wiced_bt_hfp_hf_perform_call_action(handle,
                        WICED_BT_HFP_HF_CALL_ACTION_ANSWER,(char *)p_data);
            break;

        case HF_AT_CMD_CHUP:
            wiced_bt_hfp_hf_perform_call_action(handle,
                        WICED_BT_HFP_HF_CALL_ACTION_HANGUP,(char *)p_data);
            break;

        case HF_AT_CMD_CNUM:
            handsfree_build_send_at_cmd(handle, "+CNUM",
                        WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            break;

        case HF_AT_CMD_CLCC:
            handsfree_build_send_at_cmd(handle, "+CLCC",
                    WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            break;

        case HF_AT_CMD_CIND:
            handsfree_build_send_at_cmd(handle, "+CIND",
                    WICED_BT_HFP_HF_AT_READ, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            break;

        case HF_AT_CMD_D:
        case HF_AT_CMD_BLDN:
            wiced_bt_hfp_hf_perform_call_action (handle ,
                                        WICED_BT_HFP_HF_CALL_ACTION_DIAL ,(char *)p_data);
            break;

        case HF_AT_CMD_NREC:
            handsfree_build_send_at_cmd(handle, "+NREC",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, 0);
            break;

        case HF_AT_CMD_VTS:
            handsfree_build_send_at_cmd(handle, "+VTS",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0);
            break;

        case HF_AT_CMD_BTRH:
            handsfree_build_send_at_cmd(handle, "+BTRH",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, num);
            break;

        case HF_AT_CMD_BIEV:
            handsfree_build_send_at_cmd(handle, "+BIEV",
                            WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0);
            break;
        case HF_AT_CMD_BIA:
            handsfree_build_send_at_cmd(handle, "+BIA",
                            WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0);
            break;
        case HF_AT_CMD_BCC:
            handsfree_build_send_at_cmd(handle, "+BCC",
                            WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            break;
        case HF_AT_CMD_BAC:
            handsfree_build_send_at_cmd(handle, "+BAC",
                            WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0);
            break;
    }
}


/*******************************************************************************
* Function Name: handset_handle_BREDR_connect
********************************************************************************
* Summary:
*   Connect to an BREDR device throught Address
*
* Parameters:
*   wiced_bt_device_address_t addr: the peer device address
*
* Return:
*   void: none 
*
*******************************************************************************/
void handset_handle_BREDR_connect (wiced_bt_device_address_t peer_bd_addr){
    set_skip_find_pairing_key();
    //connect GATT
    wiced_bt_gatt_bredr_connect(peer_bd_addr);
    //connect HFP
    wiced_bt_hfp_hf_connect(peer_bd_addr);
    //connect A2DP Sink
    wiced_bt_a2dp_sink_connect(peer_bd_addr);
    //connect AVRCP
    wiced_bt_avrc_ct_connect(peer_bd_addr);
}

/*******************************************************************************
* Function Name: wait_inquery_done
********************************************************************************
* Summary:
*   block function to wait inquery done or timeout
*
* Parameters:
*   void: none
*
* Return:
*   void: none 
*
*******************************************************************************/
void wait_inquery_done(void)
{
    int err = 0;
    struct timespec timewait = {0};
    struct timeval now = {0};

    TRACE_LOG("Wait for Inqury done\n");
    gettimeofday(&now, NULL);
    timewait.tv_nsec = 0;
    timewait.tv_sec = now.tv_sec + INQUERY_TIMEOUT_SEC;

    pthread_mutex_lock(&mutex_inquery_reset);
    err = pthread_cond_timedwait(&cond_inquery_reset, &mutex_inquery_reset, &timewait);
    if (err != 0){
        TRACE_LOG("Inqury Timeout! err:%d\n", err);
    }
    pthread_mutex_unlock(&mutex_inquery_reset);
}

/*******************************************************************************
* Function Name: signal_inquery_done
********************************************************************************
* Summary:
*   signal to unblock wait_inquery_done() function
*
* Parameters:
*   void: none
*
* Return:
*   void: none 
*
*******************************************************************************/
void signal_inquery_done(void)
{
    TRACE_LOG("Inquery Done");
    pthread_mutex_lock(&mutex_inquery_reset);
    pthread_cond_signal(&cond_inquery_reset);
    pthread_mutex_unlock(&mutex_inquery_reset);

}