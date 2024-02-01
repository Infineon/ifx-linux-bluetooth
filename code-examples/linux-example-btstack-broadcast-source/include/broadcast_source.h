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
 * File Name: broadcast_source.h
 *
 * Description: This is the header file for Broadcast_source function.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

#ifndef BROADCAST_SOURCE_H
#define BROADCAST_SOURCE_H
/*******************************************************************************
*                           INCLUDES
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define MAX_BROADCASTCODE_LEN 16
#define MAX_BROADCAST_ID  999999
/*******************************************************************************
*                           FUNCTION DECLARATIONS
*******************************************************************************/
typedef struct {
    uint8_t start;
    uint32_t codec_config;
    uint8_t enable_encryption;
    uint32_t channel_counts;
    uint32_t broadcast_id;
    uint8_t broadcast_code[MAX_BROADCASTCODE_LEN];
    uint8_t bis_count;
} broadcast_stream_config;


/*******************************************************************************
* Function Name: print_broadcast_stream_config
********************************************************************************
* Summary:
*   print the setting of broadcast_stream_config
*
* Parameters:
*   broadcast_stream_config config
*
* Return:
*   void: none
*
*******************************************************************************/
void print_broadcast_stream_config(broadcast_stream_config config);

/*******************************************************************************
* Function Name: broadcast_source_handle_start_streaming
********************************************************************************
* Summary:
*   broadcast_source start streaming
*
* Parameters:
*   broadcast_stream_config config
*
* Return:
*   void: none
*
*******************************************************************************/
void broadcast_source_handle_start_streaming(broadcast_stream_config config);

/*******************************************************************************
* Function Name: is_broadcast_streaming
********************************************************************************
* Summary:
*   check the broadcast is streaming or not
*
* Parameters:
*   broadcast_stream_config config
*
* Return:
*   bool: true: yes; false: no
*
*******************************************************************************/
bool is_broadcast_streaming(broadcast_stream_config config);

/*******************************************************************************
* Function Name: set_broadcast_stream_config_codec
********************************************************************************
* Summary:
*   setting codec for broadcast_stream_config
*
* Parameters:
*   uint32_t codec  (enum in wiced_bt_ga_bap_codec_config_t)
*   broadcast_stream_config *config: target broadcast_stream_config pointer
*
* Return:
*   void: none
*
*******************************************************************************/
void set_broadcast_stream_config_codec(uint32_t codec, broadcast_stream_config *config);

/*******************************************************************************
* Function Name: set_broadcast_stream_config_encryption
********************************************************************************
* Summary:
*   setting encryption or not for broadcast_stream_config
*
* Parameters:
*   bool enable
*   broadcast_stream_config *config: target broadcast_stream_config pointer
*
* Return:
*   void: none
*
*******************************************************************************/
void set_broadcast_stream_config_encryption(bool enable, broadcast_stream_config *config);

/*******************************************************************************
* Function Name: set_broadcast_stream_config_channelcounts
********************************************************************************
* Summary:
*   setting channel_counts for broadcast_stream_config
*
* Parameters:
*   uint32_t channel_counts: number
*   broadcast_stream_config *config: target broadcast_stream_config pointer
*
* Return:
*   void: none
*
*******************************************************************************/
void set_broadcast_stream_config_channelcounts(uint32_t channel_counts, broadcast_stream_config *config);

/*******************************************************************************
* Function Name: set_broadcast_stream_config_broadcastid
********************************************************************************
* Summary:
*   setting broadcast id for broadcast_stream_config
*
* Parameters:
*   uint32_t broadcast_id
*   broadcast_stream_config *config: target broadcast_stream_config pointer
*
* Return:
*   void: none
*
*******************************************************************************/
void set_broadcast_stream_config_broadcastid(uint32_t broadcast_id, broadcast_stream_config *config);

/*******************************************************************************
* Function Name: set_broadcast_stream_config_broadcastcode
********************************************************************************
* Summary:
*   setting broadcast code for broadcast_stream_config
*
* Parameters:
*   int8_t* broadcast_code
*   broadcast_stream_config *config: target broadcast_stream_config pointer
*
* Return:
*   void: none
*
*******************************************************************************/
void set_broadcast_stream_config_broadcastcode(int8_t* broadcast_code, broadcast_stream_config *config);

/*******************************************************************************
* Function Name: get_spy_instance
********************************************************************************
* Summary:
*   get the spy instance
*
* Parameters:
*   void: none
*
* Return:
*   int: instance number
*
*******************************************************************************/
int get_spy_instance(void);

/*******************************************************************************
* Function Name: set_local_bd_addr
********************************************************************************
* Summary:
*   set_local_bd_addr
*
* Parameters:
*   void: none
*
* Return:
*   void: none
*
*******************************************************************************/
void set_local_bd_addr(void);

#endif
