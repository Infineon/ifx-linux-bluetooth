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
 * File Name: broadcast_source.c
 *
 * Description: This is the source file for Broadcast_source CE application.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

/*******************************************************************************
*                           INCLUDES
*******************************************************************************/

#include "broadcast_source.h"
/* Application includes */
#include "broadcast_source_bis.h"
#include "broadcast_source_bt_manager.h"
#include "broadcast_source_rpc.h"
#include "app_bt_utils.h"

/* BT Stack includes */
#include "wiced_bt_cfg.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "log.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define BT_STACK_HEAP_SIZE (12 * 1024)

#define DEFAULT_BIS_START             0
#define DEFAULT_BIS_CODEC             7
#define DEFAULT_BIS_ENCRYPTION        0
#define DEFAULT_BIS_CHANNEL_COUNT     1
#define DEFAULT_BIS_BROADCAST_ID     16
#define DEFAULT_BIS_COUNT             1

typedef struct {
    int idx;
    char *str;
}codec_config_t;


/******************************************************************************
 *                              EXTERNS
 *****************************************************************************/
extern wiced_bt_cfg_settings_t broadcast_source_cfg_settings;
extern wiced_bt_cfg_isoc_t broadcast_source_isoc_cfg;
extern int btspy_inst;

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
char * codec_config_array[] = {
"16_2 (10ms-32kpbs)",
"48_2 (10ms-80kpbs)",
"48_4 (10ms-96kpbs)",
"48_6 (10ms-124kpbs)",
"32_2 (10ms-64kpbs)",
};


wiced_bt_heap_t *p_default_heap = NULL;

wiced_bt_device_address_t    bt_device_address = {0};

broadcast_stream_config bis_config;


/*******************************************************************************
* Function Name: set_default_broadcast_stream_config
********************************************************************************
* Summary:
*   assign broadcast_stream_config default setting
*
* Parameters:
*   broadcast_stream_config config
*
* Return:
*   void: none
*
*******************************************************************************/
static void set_default_broadcast_stream_config(broadcast_stream_config* config);

/*******************************************************************************
* Function Name: codec_config_mapping
********************************************************************************
* Summary:
*   get the str from codec number (wiced_bt_ga_bap_codec_config_t)
*
* Parameters:
*   uint32_t input
*
* Return:
*   void: none
*
*******************************************************************************/
static char* codec_config_mapping(uint32_t input);


/*******************************************************************************
* Function Name: print_broadcast_code
********************************************************************************
* Summary:
*   pirnt boradcast code with Hex format
*
* Parameters:
*   uint8_t* code:   uint8_t array
*   int size:   code array size
*
* Return:
*   void: none
*
*******************************************************************************/
static void print_broadcast_code(uint8_t* code, int size);

/******************************************************
 *               Function Definitions
 ******************************************************/
void APPLICATION_START(void)
{
    // RPC to work with LE Audio Client Control
    broadcast_source_rpc_init(get_spy_instance());

    // Register call back and configuration with stack
    wiced_bt_stack_init(broadcast_source_btm_cback, &broadcast_source_cfg_settings);

    /* Initialize BIS */
    broadcast_source_bis_init(&broadcast_source_isoc_cfg);

    /* Create a buffer heap, make it the default heap.  */
    wiced_bt_create_heap("broadcast_source", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);

    set_default_broadcast_stream_config(&bis_config);
}

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
int get_spy_instance(void)
{
    return btspy_inst;
}

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
void set_local_bd_addr(void)
{
    if ((bt_device_address[0] | bt_device_address[1] | bt_device_address[2] | bt_device_address[3] | bt_device_address[4] | bt_device_address[5]) != 0){
        wiced_bt_set_local_bdaddr(bt_device_address, BLE_ADDR_PUBLIC);
    }
    print_bd_address(bt_device_address);
}

/*******************************************************************************
* Function Name: set_default_broadcast_stream_config
********************************************************************************
* Summary:
*   assign broadcast_stream_config default setting
*
* Parameters:
*   broadcast_stream_config config
*
* Return:
*   void: none
*
*******************************************************************************/
static void set_default_broadcast_stream_config(broadcast_stream_config* config){
    config->start = DEFAULT_BIS_START;
    config->codec_config = DEFAULT_BIS_CODEC;
    config->enable_encryption = DEFAULT_BIS_ENCRYPTION;
    config->channel_counts = DEFAULT_BIS_CHANNEL_COUNT;
    config->broadcast_id = DEFAULT_BIS_BROADCAST_ID;
    config->bis_count = DEFAULT_BIS_COUNT;
    memset(config->broadcast_code, '\0', MAX_BROADCASTCODE_LEN);
}

/*******************************************************************************
* Function Name: codec_config_mapping
********************************************************************************
* Summary:
*   get the str from codec number (wiced_bt_ga_bap_codec_config_t)
*
* Parameters:
*   uint32_t input
*
* Return:
*   void: none
*
*******************************************************************************/
static char* codec_config_mapping(uint32_t input){
    switch (input){
        case BAP_CODEC_CONFIG_16_2_2:
            return codec_config_array[0];
        case BAP_CODEC_CONFIG_48_2_2:
            return codec_config_array[1];
        case BAP_CODEC_CONFIG_48_4_2:
            return codec_config_array[2];
        case BAP_CODEC_CONFIG_48_6_2:
            return codec_config_array[3];
        case BAP_CODEC_CONFIG_32_2_2:
            return codec_config_array[4];
        default:
            return "Not Settting";
    }
}

/*******************************************************************************
* Function Name: print_broadcast_code
********************************************************************************
* Summary:
*   pirnt boradcast code with Hex format
*
* Parameters:
*   uint8_t* code:   uint8_t array
*   int size:   code array size
*
* Return:
*   void: none
*
*******************************************************************************/
static void print_broadcast_code(uint8_t* code, int size)
{
    printf("broadcast_code:");
    for (int i = 0; i < size; i++){
        printf("%02X",code[i]);
    }
    printf("\n\n");
}

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
void print_broadcast_stream_config(broadcast_stream_config config){
    uint8_t start = config.start;
    uint32_t codec_config = config.codec_config;
    uint8_t enable_encryption = config.enable_encryption;
    uint32_t channel_counts = config.channel_counts;
    uint32_t broadcast_id = config.broadcast_id;
    uint8_t broadcast_code[MAX_BROADCASTCODE_LEN];
    uint8_t bis_count = config.bis_count;
    memcpy(broadcast_code, config.broadcast_code, MAX_BROADCASTCODE_LEN);

    TRACE_LOG("Start:%d, codec_config:%s, enable_encryption:%d channel_counts:%d Broadcast ID:0x%x ",start, codec_config_mapping(codec_config), enable_encryption, channel_counts, broadcast_id);
    print_broadcast_code(broadcast_code, MAX_BROADCASTCODE_LEN);
}

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
void broadcast_source_handle_start_streaming(broadcast_stream_config config)
{
    wiced_result_t ret_sts = WICED_ERROR;
    wiced_bt_ga_bap_stream_config_t stream_config;
    uint8_t start = config.start;
    uint32_t codec_config = config.codec_config;
    uint8_t enable_encryption = config.enable_encryption;
    uint32_t channel_counts = config.channel_counts;
    uint32_t broadcast_id = config.broadcast_id;
    uint8_t broadcast_code[MAX_BROADCASTCODE_LEN];
    uint8_t bis_count = config.bis_count;

    memcpy(broadcast_code, config.broadcast_code, MAX_BROADCASTCODE_LEN);

    TRACE_LOG("start:%d, codec_config:%s,enable_encryption:%d channel_counts:%d Broadcast ID:%d",start, codec_config_mapping(codec_config), enable_encryption, channel_counts, broadcast_id);
    
    if (start)
    {
        wiced_bt_ga_bap_broadcast_get_stream_config(codec_config, &stream_config);

        TRACE_LOG("Configuring source stream [SF:%d] [FD:%d] [OPF:%d] [Latency:%d]",
                       stream_config.sampling_frequency,
                       stream_config.frame_duration,
                       stream_config.octets_per_codec_frame,
                       stream_config.max_transport_latency);

        ret_sts = broadcast_source_bis_configure_stream(broadcast_id,
                                                        broadcast_code,
                                                        bis_count,
                                                        channel_counts,
                                                        stream_config.sampling_frequency,
                                                        stream_config.frame_duration,
                                                        stream_config.octets_per_codec_frame,
                                                        enable_encryption);
        TRACE_LOG("broadcast_source_bis_configure_stream [ret_sts:0x%x]", ret_sts);

        broadcast_source_bis_start_stream(&stream_config);
    }
    else
    {
        ret_sts = broadcast_source_bis_disable_stream();
        TRACE_LOG("broadcast_source_bis_disable_stream [ret_sts:0x%x]", ret_sts);

        ret_sts = broadcast_source_bis_release_stream();
        TRACE_LOG("broadcast_source_bis_release_stream [ret_sts:0x%x]", ret_sts);
    }
}

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
bool is_broadcast_streaming(broadcast_stream_config config){
    return (config.start == 1) ? true : false;
}

/*******************************************************************************
* Function Name: set_broadcast_stream_config_codec
********************************************************************************
* Summary:
*   setting codec for broadcast_stream_config
*
* Parameters:
*   uint32_t codec
*   broadcast_stream_config *config: target broadcast_stream_config pointer
*
* Return:
*   void: none
*
*******************************************************************************/
void set_broadcast_stream_config_codec(uint32_t codec, broadcast_stream_config *config){
     if (config != NULL){
        config->codec_config = codec;
     }
     else{
        TRACE_ERR("config is NULL");
     }
}

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
void set_broadcast_stream_config_encryption(bool enable, broadcast_stream_config *config){
    if (config != NULL){
        config->enable_encryption = (enable == true) ? 1 : 0;
    }
    else{
        TRACE_ERR("config is NULL");
    }
}


/*******************************************************************************
* Function Name: set_broadcast_stream_config_channelcounts
********************************************************************************
* Summary:
*   setting channel_counts or broadcast_stream_config
*
* Parameters:
*   uint32_t channel_counts: number
*   broadcast_stream_config *config: target broadcast_stream_config pointer
*
* Return:
*   void: none
*
*******************************************************************************/
void set_broadcast_stream_config_channelcounts(uint32_t channel_counts, broadcast_stream_config *config){
    if (config != NULL){
        config->channel_counts = channel_counts;
    }
    else{
        TRACE_ERR("config is NULL");
    }
}

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
void set_broadcast_stream_config_broadcastid(uint32_t broadcast_id, broadcast_stream_config *config){
    if (config != NULL){
        if (broadcast_id == 0 || broadcast_id > MAX_BROADCAST_ID){
            TRACE_ERR("broadcast_id %x out of range from 1 ~ f423f", broadcast_id);
        }
        else{
            config->broadcast_id = broadcast_id;
        }
    }
    else{
        TRACE_ERR("config is NULL");
    }
}

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
void set_broadcast_stream_config_broadcastcode(int8_t* broadcast_code, broadcast_stream_config *config){
    if (config != NULL){
        memcpy(config->broadcast_code, broadcast_code, MAX_BROADCASTCODE_LEN);
        print_broadcast_code(config->broadcast_code, MAX_BROADCASTCODE_LEN);
    }
    else{
        TRACE_ERR("config is NULL");
    }
}
