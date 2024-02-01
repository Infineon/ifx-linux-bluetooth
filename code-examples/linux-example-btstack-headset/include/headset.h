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
 * File Name: headset.h
 *
 * Description: This is the header file for headset CE application.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/
#ifndef HEADSET_H
#define HEADSET_H

#include <stdbool.h>
#include <stdint.h>

#include "wiced_bt_dev.h"

/******************************************************************************
 *                                MACROS
 *****************************************************************************/

/*USER INPUT COMMANDS*/
#define EXIT                            (0)
#define PRINT_MENU                      (1)
#define SET_VISIBILITY                  (2)
#define SET_PAIRING_MODE                (3)
#define SET_INQUIRY                     (4)
#define HF_CONNECT                      (5)
#define HF_DISCONNECT                   (6)
#define GET_HEADSET_STATE               (7)
#define HF_PRINT_CONNECTION_DETAILS     (8)
#define ANSWER_CALL                     (9)
#define HANGUP_CALL                     (10)
#define DIAL_NUM                        (11)
#define REDIAL                          (12)
#define QUERY_CUR_CALLS                 (13)
#define SET_SPK_VOL                     (14)
#define SET_MIC_VOL                     (15)
#define SUBSCIBER_NUM_INFO              (16)
#define AVRCP_ACTION_VOLUME_UP          (17)
#define AVRCP_ACTION_VOLUME_DOWN        (18)
#define AVRCP_ACTION_PAUSE_PLAY         (19)
#define AVRCP_ACTION_FORWARD            (20)
#define AVRCP_ACTION_BACKWARD           (21)
#define AVRCP_ACTION_STOP               (22)
#define AVRCP_ACTION_FAST_FORWARD       (23)
#define AVRCP_ACTION_FAST_REWIND        (24)
#define AVRCP_ACTION_UNIT_INFO          (25)
#define ENABLE_GFPS                     (26)
#define NO_SUPPORT_CMD                  (27)

#define HF_AT_CMD_VGS                       0x00    /* Update speaker volume */
#define HF_AT_CMD_VGM                       0x01    /* Update microphone volume */
#define HF_AT_CMD_ATA                       0x02    /* Answer incoming call */
#define HF_AT_CMD_BINP                      0x03    /* Retrieve number from voice tag */
#define HF_AT_CMD_BVRA                      0x04    /* Enable/Disable voice recognition */
#define HF_AT_CMD_BLDN                      0x05    /* Last Number redial */
#define HF_AT_CMD_CHLD                      0x06    /* Call hold command */
#define HF_AT_CMD_CHUP                      0x07    /* Call hang up command */
#define HF_AT_CMD_CIND                      0x08    /* Read Indicator Status */
#define HF_AT_CMD_CNUM                      0x09    /* Retrieve Subscriber number */
#define HF_AT_CMD_D                         0x0A    /* Place a call using a number or memory dial */
#define HF_AT_CMD_NREC                      0x0B    /* Disable Noise reduction and echo canceling in AG */
#define HF_AT_CMD_VTS                       0x0C    /* Transmit DTMF tone */
#define HF_AT_CMD_BTRH                      0x0D    /* CCAP incoming call hold */
#define HF_AT_CMD_COPS                      0x0E    /* Query operator selection */
#define HF_AT_CMD_CMEE                      0x0F    /* Enable/disable extended AG result codes */
#define HF_AT_CMD_CLCC                      0x10    /* Query list of current calls in AG */
#define HF_AT_CMD_BIA                       0x11    /* Activate/Deactivate indicators */
#define HF_AT_CMD_BIEV                      0x12    /* Send HF indicator value to peer */
#define HF_AT_CMD_BCC                       0x13    /* Initiate Codec Connection */
#define HF_AT_CMD_BCS                       0x14    /* Codec Selection */
#define HF_AT_CMD_BAC                       0x15    /* Updating Available Codec */
#define HF_AT_CMD_MAX                       0x16    /* For Command validation */


/* BR/EDR Profiles/Applications */
#define WICED_APP_AUDIO_SRC_INCLUDED        FALSE
#define WICED_APP_AUDIO_RC_TG_INCLUDED      TRUE
#define WICED_APP_AUDIO_RC_CT_INCLUDED      FALSE
#define WICED_APP_AUDIO_SINK_INCLUDED       TRUE

#ifndef WICED_NVRAM_VSID_START
#define    WICED_NVRAM_VSID_START              0x200
#endif

#define INQUIRY_DURATION      (5)
#define BDA_LEN               (6)

/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/

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
void APPLICATION_START(void );

/*****************************************************************************
* Function Name: headset_is_valid_state
******************************************************************************
* Summary: Check current statues is valid for cmd
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
bool headset_is_valid_state (int user_input);

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
uint16_t headset_get_rfcomm_handle_idx(void);


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
void headset_send_at_command (uint16_t handle, uint8_t command, int num, uint8_t* p_data);

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
void headset_handle_set_visibility (bool discoverability, bool connectability);

/*******************************************************************************
* Function Name: handset_handle_set_pairability
********************************************************************************
* Summary:
*   Enable or disable pairing
*
* Parameters:
*   bool allowed : true - starts Inquiry, false - Stops Inquiry
*
* Return:
*   NONE
*
*******************************************************************************/
void handset_handle_set_pairability (bool allowed);

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
wiced_result_t handset_handle_set_inquiry (bool enable);


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
void handset_handle_BREDR_connect (wiced_bt_device_address_t peer_bd_addr);


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
void wait_inquery_done(void);

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
void signal_inquery_done(void);

#endif //HEADSET_H
