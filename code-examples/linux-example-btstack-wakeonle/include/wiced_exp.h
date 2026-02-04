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
 * File Name: wiced_exp.h
 * 
 * Description: This is the header file of WICED EXP LIB open to USER
 *
 * This File content need match to wiced_exp_lib, do not modify it.
 *
 *****************************************************************************/

#ifndef __LIB_CY_EXP_H__
#define __LIB_CY_EXP_H__

#include "data_types.h"

/******************************************************************************
*       MACRO
******************************************************************************/
#define LE_PCF_MANUFACTURE_DATA_LEN_MAX           29U
#define LE_PCF_COMANY_ID_LEN                      2U
#define LE_PCF_MANUFACTURE_DATA_PATTERN_LEN_MAX   27U

#define tBT_UUID                wiced_bt_uuid_t
#define tBTM_STATUS             wiced_result_t
#define tBTM_VSC_CMPL           wiced_bt_dev_vendor_specific_command_complete_params_t

#define GPIO_ACTIVE_LOW         0U
#define GPIO_ACTIVE_HIGH        1U
#define GPIO_ASSERT(x)			((x==GPIO_ACTIVE_LOW)?GPIO_ACTIVE_LOW:GPIO_ACTIVE_HIGH)
#define GPIO_DEASSERT(x)		((x==GPIO_ACTIVE_LOW)?GPIO_ACTIVE_HIGH:GPIO_ACTIVE_LOW)

#define WICED_SLEEP_MODE_BT_WAKE_ACT_LOW          GPIO_ACTIVE_LOW 
#define WICED_SLEEP_MODE_BT_WAKE_ACT_HIGH         GPIO_ACTIVE_HIGH         
#define WICED_SLEEP_MODE_HOST_WAKE_ACT_LOW        GPIO_ACTIVE_LOW  
#define WICED_SLEEP_MODE_HOST_WAKE_ACT_HIGH       GPIO_ACTIVE_HIGH         

/* filter selection bit index  */
#define WICED_LE_ADV_PCF_FEA_NONE                    0U
#define WICED_LE_ADV_PCF_FEA_BROADCAST_ADDR          0x01 
#define WICED_LE_ADV_PCF_FEA_SRCV_DATA_CHANGE        0x02 
#define WICED_LE_ADV_PCF_FEA_SRVC_UUID               0x04 
#define WICED_LE_ADV_PCF_FEA_SRVC_SOL_UUID           0x08 
#define WICED_LE_ADV_PCF_FEA_LOCAL_NAME              0x10
#define WICED_LE_ADV_PCF_FEA_MANU_DATA               0x20
#define WICED_LE_ADV_PCF_FEA_SRVC_DATA               0x40
#define WICED_LE_ADV_PCF_FEA_SELECT_ALL              0x7F

typedef uint16_t tWICED_LE_ADV_PCF_FEATURE_SELE;
typedef uint16_t tWICED_LE_ADV_PCF_FEATURE_LOGIC_TYPE;

//SLEEP MODE PARAM SLEEP MODE
#define BTM_SLEEP_MODE_NONE              0U
#define BTM_SLEEP_MODE_UART              0x01
#define BTM_SLEEP_MODE_SDIO              0x06

#define WICED_LE_ADV_PCF_RSSI_HIGH_THRESHOLD    -128
#define WICED_LE_ADV_PCF_FILTER_INDEX_START     0x00
#define WICED_LE_ADV_PCF_FILTER_INDEX_END       0x1F

typedef enum
{
    WICED_LE_ADV_PCF_LOGIC_OR         = 0,
    WICED_LE_ADV_PCF_LOGIC_AND        = 1,
    WICED_LE_ADV_PCF_LOGIC_NONE
}tWICED_LE_ADV_PCF_FILTER_LOGIC_TYPE;


//APCF SUB COMMAND
typedef enum
{
    WICED_LE_ADV_PCF_ENABLE              = 0,
    WICED_LE_ADV_PCF_FILTER_PARM         = 1,
    WICED_LE_ADV_PCF_BROD_ADDR           = 2,
    WICED_LE_ADV_PCF_SRVC_UUID           = 3,
    WICED_LE_ADV_PCF_SRVC_SOL_UUID       = 4, 
    WICED_LE_ADV_PCF_LOCAL_NAME          = 5,
    WICED_LE_ADV_PCF_MANU_DATA           = 6,
    WICED_LE_ADV_PCF_SRVC_DATA           = 7,
    WICED_LE_ADV_PCF_NONE
}tWICED_LE_ADV_PCF_SUB_CMD;

typedef enum
{
    WICED_LE_ADV_PCF_ACT_ADD    = 0,
    WICED_LE_ADV_PCF_ACT_DELETE = 1,
    WICED_LE_ADV_PCF_ACT_CLEAR  = 2,
    WICED_LE_ADV_PCF_ACT_NONE
}tWICED_LE_ADV_PCF_ACT;

typedef enum
{
    WICED_LE_ADV_PCF_DELIVERY_MODE_IMMEDIATE = 0,
    WICED_LE_ADV_PCF_DELIVERY_MODE_ON_FOUND  = 1,
    WICED_LE_ADV_PCF_DELIVERY_MODE_BATCHED   = 2,
    WICED_LE_ADV_PCF_DELIVERY_MODE_NONE
}tWICED_LE_ADV_PCF_DELIVERY_MODE;

typedef enum
{
    WICED_LE_ADV_PCF_BD_ADDR_PUBLIC  = 0,
    WICED_LE_ADV_PCF_BD_ADDR_RANDOM  = 1,
    WICED_LE_ADV_PCF_BD_ADDR_NONE
}tWICED_LE_ADV_PCF_BD_ADDR_TYPE;

typedef wiced_bt_ble_address_t tLE_BD_ADDR;
typedef uint8_t tWICED_LE_ADV_PCF_FILTER_INDEX;
typedef uint16_t tWICED_LE_ADV_PCF_RSSI_HIGH_THRESHOLD;
typedef void (tBTM_VSC_CMPL_CB)(tBTM_VSC_CMPL* );

/******************************************************************************
*       FUNCTION PROTOTYPE
******************************************************************************/
/*******************************************************************************
* Function Name: wiced_set_sleep_mode_with_param
********************************************************************************
* Summary: set the sleep mode param for BT chip and set sleep mode by VSC
*
* Parameters:
*    uint8_t    sleep_mode:
*               set sleep mode, only support UART SLEEP MODE NOW
*    uint8_t    dev_wake_active:
*               set dev_wake gpio LOW ACTIVE or HIGH ACTIVE
*    uint8_t    host_wake_active
*               set host_wake gpio LOW ACTIVE or HIGH ACTIVE
*    uint8_t    combne_lpm
*               set sleep mode combine low power mode
*   tBTM_VSC_CMPL_CB* p_cb
*               VSC command complete callback function
* Return:
*    BOOL32:    result
*               WICED_FALSE: error
*               WICED_TRUE:  success
*******************************************************************************/
BOOL32 wiced_set_sleep_mode_with_param(uint8_t sleep_mode, uint8_t dev_wake_active, uint8_t host_wake_active, uint8_t combine_lpm, tBTM_VSC_CMPL_CB* p_cb);

/*******************************************************************************
* Function Name: wiced_set_apcf_enable
********************************************************************************
* Summary: enable apcf function
*
* Parameters:
*    BOOL32 enable:
*           enable or disable apcf
*
* Return:
*    BOOL32 result
*           WICED_TRUE:  SUCCESS
*           WICED_FALSE: ERROR
*
*******************************************************************************/
BOOL32 wiced_set_apcf_enable(BOOL32 enable);

/*******************************************************************************
* Function Name: wiced_set_apcf_data_uuid
********************************************************************************
* Summary: This Function set the APCF Data uuid, and use VSC send to Controller
*
* Parameters:
*    tBT_UUID uuid:
*             UUID for APCF
*    tWICED_LE_ADV_PCF_ACT act:
*             WICED_LE_ADV_PCF_ACT_ADD,
*             WICED_LE_ADV_PCF_ACT_DELETE,
*             WICED_LE_ADV_PCF_ACT_CLEAR
*
*    tWICED_ADV_PCF_FILTER_INDEX idx:
*             filter index
* Return:
*    BOOL32 result
*           WICED_TRUE:  SUCCESS
*           WICED_FALSE: ERROR
*
*******************************************************************************/
BOOL32 wiced_set_apcf_data_uuid(tBT_UUID uuid, tWICED_LE_ADV_PCF_ACT act, tWICED_LE_ADV_PCF_FILTER_INDEX idx);

/******************************************************************************************
* Function Name: wiced_set_apcf_data_manufacture
*******************************************************************************************
* Summary: This Function set the APCF Data Manufacture data, and use VSC send to Controller
*
* Parameters:
*    uint16_t company_id
*
*    uint32_t data_len
*
*    uint8_t *p_pattern
*
*    uint16_t company_id_mask
*
*    uint16_t *p_pattern_mask
*
*    tWICED_LE_ADV_PCF_ACT act:
*             WICED_LE_ADV_PCF_ACT_ADD,
*             WICED_LE_ADV_PCF_ACT_DELETE,
*             WICED_LE_ADV_PCF_ACT_CLEAR
*
*    tWICED_ADV_PCF_FILTER_INDEX idx:
*             filter index
* Return:
*    BOOL32 result
*           WICED_TRUE:  SUCCESS
*           WICED_FALSE: ERROR
*
*******************************************************************************************/
BOOL32 wiced_set_apcf_data_manufacture(uint16_t company_id, uint32_t data_len, uint8_t *p_pattern, uint16_t company_id_mask, uint8_t *p_pattern_mask, tWICED_LE_ADV_PCF_ACT act, tWICED_LE_ADV_PCF_FILTER_INDEX idx);

/*******************************************************************************
* Function Name: wiced_set_apcf_filter_param
********************************************************************************
* Summary:
*   This function set the APCF Filter Parameter
*
* Parameters:
*    tWICED_LE_ADV_PCF_ACT act:
*
*    tWICED_LE_ADV_PCF_FILTER_INDEX idx:
*
*    tWICED_LE_ADV_PCF_FEATURE_SELE feature_sele:
*
*    tWICED_LE_ADV_PCF_FEATURE_LOGIC_TYPE feature_logic_type:
*
*    tWICED_LE_ADV_PCF_FILTER_LOGIC_TYPE filter_logic_type:
*
*    tWICED_LE_ADV_PCF_RSSI_HIGH_THRESHOLD rssi_high:
*
*    tWICED_LE_ADV_PCF_DELIVERY_MODE delivery_mode:
*
* Return:
*   BOOL32:
*         WICED_TRUE:  api send VSC success
*         WICED_FALSE: api send VSC fail or other reason
*
*******************************************************************************/
BOOL32 wiced_set_apcf_filter_param(tWICED_LE_ADV_PCF_ACT act, tWICED_LE_ADV_PCF_FILTER_INDEX idx, tWICED_LE_ADV_PCF_FEATURE_SELE feature_sele,
                                    tWICED_LE_ADV_PCF_FEATURE_LOGIC_TYPE feature_logic_type, tWICED_LE_ADV_PCF_FILTER_LOGIC_TYPE filter_logic_type,
                                    tWICED_LE_ADV_PCF_RSSI_HIGH_THRESHOLD rssi_high, tWICED_LE_ADV_PCF_DELIVERY_MODE delivery_mode);

/*******************************************************************************
* Function Name: wiced_exp_version
********************************************************************************
* Summary: show wiced_exp version
*
* Parameters:
*   None
*
* Return:
*   None
*******************************************************************************/
void wiced_exp_version();

/*******************************************************************************
* Function Name: wiced_set_sdio_sleep_mode_with_param
********************************************************************************
* Summary: set the sdio sleep mode param for BT chip
*
* Parameters:
*    uint8_t    idle_threshold_hc
*
*    tBTM_VSC_CMPL_CB* p_cback
*               callback for any task need do after set sdio sleep complete
*
* Return:
*    BOOL32:    result
*               WICED_FALSE: error
*               WICED_TRUE:  success
*******************************************************************************/
BOOL32 wiced_set_sdio_sleep_mode_with_param(uint8_t idle_threshold_hc, tBTM_VSC_CMPL_CB* p_cback);

/*******************************************************************************
* Function Name: wiced_set_sdio_sleep_none
********************************************************************************
* Summary: disable sdio sleep 
*
* Parameters:
*   tBTM_VSC_CMPL_CB* p_cback
*       callback for any task need do after set sdio sleep none complete
*
* Return:
*    BOOL32:    result
*               WICED_FALSE: error
*               WICED_TRUE:  success
*******************************************************************************/
BOOL32 wiced_set_sdio_sleep_none(tBTM_VSC_CMPL_CB* p_cback);

/*******************************************************************************
* Function Name: wiced_exp_reset_to_le_legacy
********************************************************************************
* Summary: use this API set BTSTACK and Controller use Leagcy ADV
*           set callback func first and Reset Controller
*
* Parameters:
*   tBTM_VSC_CMPL_CB* p_cback: callback func when Reset
*                               Normall set it as NULL
*
* Return:
*   None
*******************************************************************************/
BOOL32 wiced_exp_reset_to_le_legacy(tBTM_VSC_CMPL_CB* p_cback);

/*******************************************************************************
* Function Name: wiced_exp_reset
********************************************************************************
* Summary: wiced exp api reset to default, if Controller support EXT ADV,
*           but we reset it to legacy only. after the wake of wakeonLE.
*           need reset stack and Controller to default status.
* 
* Parameters:
*   tBTM_VSC_CMPL_CB* p_cback: callback func when Reset
*                               Normall set it as NULL
*
* Return:
*   void    
*******************************************************************************/
void wiced_exp_reset(tBTM_VSC_CMPL_CB* p_cback);

/*******************************************************************************
* Function Name: wiced_exp_isExtAdvSupported
********************************************************************************
* Summary: wiced exp api, check is Ext Adv Support, Controller FW send
*           le_local_feature to stack, stack save it.
*
* Parameters:
*   None
*
* Return:
*   BOOL32
*******************************************************************************/
BOOL32 wiced_exp_isExtAdvSupported(void);

#endif /* __LIB_CY_EXP_H__ */
