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
 * File Name: wakeon_le.c
 *
 * Description: This is the source file for Linux wakeon_le CE.
 *
 * Related Document: See README.md
 * 
 ******************************************************************************
* $ Copyright 2022-YEAR Cypress Semiconductor $
*******************************************************************************
*      INCLUDES
*******************************************************************************/
#include "wiced_bt_stack.h"
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "wiced_memory.h"
#include "stdio.h"
#include "wiced_bt_dev.h"
#include "app_bt_utils.h"
#include "wakeon_le.h"
#include "wiced_hal_nvram.h"
#include "data_types.h"
#include "wiced_exp.h"
#include "platform_linux.h"
#include "linux/gpio.h"
#include "log.h"

#ifdef TAG
#undef TAG
#endif
#define TAG "[WAKEONLE]"

#define debug 0

typedef enum {
    NONE,
    WAKE_ON_UUID,
    NEW_CONNECTION,
    WAKE_ON_CONNECTION
} APP_MODE;

APP_MODE current_mode;

/*******************************************************************************
*       MACROS
*******************************************************************************/
#define BT_STACK_HEAP_SIZE          (0xF000)

#define APP_VS_ID                        WICED_NVRAM_VSID_START
#define APP_LOCAL_KEYS_VS_ID           ( WICED_NVRAM_VSID_START + 1 )
#define APP_PAIRED_KEYS_VS_ID          ( WICED_NVRAM_VSID_START + 2 )

#ifndef PACKED
#define PACKED
#endif

/*******************************************************************************
*       STRUCTURES AND ENUMERATIONS
*******************************************************************************/
wiced_bt_device_address_t bt_device_address = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};

#pragma pack(1)
/* Host information saved in  NVRAM */
typedef PACKED struct
{
    wiced_bt_device_address_t  bdaddr;                                /* BD address of the bonded host */
    wiced_bt_ble_address_type_t addr_type;
    uint8_t paired;
} host_info_t;
#pragma pack()

/******************************************************************************
 *                                Structures
 ******************************************************************************/
typedef struct
{
    wiced_bt_device_address_t remote_addr;  /* remote peer device address */
    wiced_bt_ble_address_type_t addr_type;
    uint16_t  conn_id;                  /* connection ID referenced by the stack */
    uint16_t  peer_mtu;                 /* peer MTU */

} app_state_t;

/*******************************************************************************
*       VARIABLE DEFINITIONS
*******************************************************************************/
wiced_bt_heap_t *p_default_heap   = NULL;
extern cybt_controller_gpio_config_t gpio_cfg;
BOOL32 inSleep = WICED_FALSE;
tBT_UUID uuid = {0};
wiced_bt_device_address_t peer_BDAddr;
uint16_t company_id = COMPANY_ID;
uint16_t company_id_mask = 0xFFFF;
uint32_t data_len = 0;
uint8_t pattern[LE_PCF_MANUFACTURE_DATA_PATTERN_LEN_MAX] = {0};
uint8_t pattern_mask[LE_PCF_MANUFACTURE_DATA_PATTERN_LEN_MAX] = {0};
uint8_t apcf_filter_idx =  WICED_LE_ADV_PCF_FILTER_INDEX_START;
BOOL32 app_ready = WICED_FALSE;
host_info_t app_host_info;
app_state_t app_state;

extern const wiced_bt_cfg_ble_t cy_bt_cfg_ble;

/*******************************************************************************
*       FUNCTION DECLARATIONS
*******************************************************************************/
static void  app_init(void);
static void* app_alloc_buffer(int len);
static void  app_free_buffer(uint8_t *p_event_data);
static void  app_scan_result_cback(wiced_bt_ble_scan_results_t* p_scan_result, uint8_t* p_adv_data);

/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t    app_bt_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void bt_host_wake_assert_cback();
static void bt_sleep_cmpl_cback(tBTM_VSC_CMPL *p_params);

static wiced_bt_dev_status_t app_set_adv_data();
static wiced_bt_gatt_status_t app_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t* p_data);
static wiced_bt_gatt_status_t app_gatts_conn_status_cb(wiced_bt_gatt_connection_status_t* p_status);
static wiced_bt_gatt_status_t app_gatts_connection_up(wiced_bt_gatt_connection_status_t* p_status);
static wiced_bt_gatt_status_t app_gatts_connection_down(wiced_bt_gatt_connection_status_t* p_status);
static wiced_bt_gatt_status_t app_gatts_req_cb(wiced_bt_gatt_attribute_request_t* p_data);
static void app_load_keys_for_address_resolution(void);
static void app_smp_bond_result(uint8_t result);
static void app_clear_bond_info(void);

/*******************************************************************************
*       FUNCTION DEFINITION
*******************************************************************************/
/*******************************************************************************
* Function Name: application_start
********************************************************************************
* Summary:
*   Set device configuration and start BT stack initialization. The actual
*   application initialization will happen when stack reports that BT device
*   is ready.
*
* Parameters: NONE
*
* Return: NONE
*
*******************************************************************************/
void application_start(void)
{
    wiced_result_t wiced_result;

    TRACE_LOG("************* WakeOn_LE Application Start ************************\n");
    wiced_exp_version();
    /* Register call back and configuration with stack */
    wiced_result = wiced_bt_stack_init (app_bt_management_callback, &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if( WICED_BT_SUCCESS == wiced_result)
    {
        TRACE_LOG("Bluetooth Stack Initialization Successful \n");
        /* Create default heap */
        p_default_heap = wiced_bt_create_heap("default_heap", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
        if (p_default_heap == NULL)
        {
            TRACE_ERR("create default heap error: size %d\n", BT_STACK_HEAP_SIZE);
            exit(EXIT_FAILURE);
        }
    }
    else
    {
       TRACE_ERR("Bluetooth Stack Initialization Failed!! \n");
       exit(EXIT_FAILURE);
    }
}

/*******************************************************************************
* Function Name: app_bt_management_callback
********************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management
*   events from the LE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : LE event code of one byte
*                                                 length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to LE management
*                                                 event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*******************************************************************************/
static wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event,
                                          wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_ble_advert_mode_t *p_mode = NULL;
    wiced_bt_dev_ble_pairing_info_t* p_info;
    wiced_bt_device_link_keys_t* p_keys;
    wiced_bt_dev_encryption_status_t* p_status;

    uint16_t  read_bytes = 0;

    const uint8_t *link_key;

    TRACE_LOG( "event 0x%x \n", event );
    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth Controller and Host Stack Enabled */
        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            wiced_bt_set_local_bdaddr((uint8_t *)bt_device_address, BLE_ADDR_PUBLIC);
            /* Bluetooth is enabled */               
            wiced_bt_dev_read_local_addr(bda);
            TRACE_LOG("Local Bluetooth Address:");
            print_bd_address(bda);

            /* Perform application-specific initialization */
            app_init();

        }
        else
        {
            TRACE_ERR( "Bluetooth Enabling Failed \n");
            result = WICED_BT_ERROR;
        }
        break;

    case BTM_DISABLED_EVT:
        TRACE_LOG( "Bluetooth Disabled \n" );
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        TRACE_LOG("Numeric_value: %d \n", p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
        break;

    case BTM_PASSKEY_NOTIFICATION_EVT:
        TRACE_LOG("PassKey Notification. BDA: ");
        print_bd_address(p_event_data->user_passkey_notification.bd_addr);
        TRACE_LOG("PassKey Notification.  Key %d \n", p_event_data->user_passkey_notification.passkey);
        wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr);
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        p_info = &p_event_data->pairing_complete.pairing_complete_info.ble;
        TRACE_LOG("Pairing Complete: 0x%x ", p_info->reason);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* save keys to NVRAM */
        p_keys = &p_event_data->paired_device_link_keys_update;

        wiced_hal_write_nvram(APP_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t*)p_keys, &result);

        TRACE_LOG("Keys saved to NVRAM result: %d\n ", result);
        break;

    case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
    {
        p_keys = &p_event_data->paired_device_link_keys_request;
        read_bytes = wiced_hal_read_nvram(APP_PAIRED_KEYS_VS_ID,
            sizeof(wiced_bt_device_link_keys_t),
            (uint8_t*)p_keys,
            &result);

        /* Break if link key retrieval is failed or link key is not available. */
        if (result != WICED_BT_SUCCESS)
        {
            result = WICED_BT_ERROR;
            app_host_info.paired = 0;
            TRACE_LOG("\n Reading keys from NVRAM failed or link key not available.");
            break;
        }

        TRACE_LOG("keys read from NVRAM result:%d\n", result);

    }
    break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* save keys to NVRAM */
    {
        wiced_bt_local_identity_keys_t* p_ikeys = &p_event_data->local_identity_keys_update;
        wiced_hal_write_nvram(APP_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), (uint8_t*)p_ikeys, &result);
        TRACE_LOG("local keys save to NVRAM result: %d\n ", result);
    }
    break;

    case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
    {
        wiced_bt_local_identity_keys_t* p_ikeys = &p_event_data->local_identity_keys_request;
        wiced_hal_read_nvram(APP_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), (uint8_t*)p_ikeys, &result);
        TRACE_LOG("local keys read from NVRAM result:%d\n", result);
    }
    break;

    case BTM_ENCRYPTION_STATUS_EVT:
        p_status = &p_event_data->encryption_status;
        TRACE_LOG("Encryption Status Event:  res %d\n", p_status->result);
        app_smp_bond_result(p_status->result);
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;
        TRACE_MSG("Advertisement State Change: %d", *p_mode);
        if (*p_mode == BTM_BLE_ADVERT_OFF)
        {
            TRACE_MSG("ADV stopped");  
        }

        break;

    default:
        TRACE_LOG("Unhandled Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));
        break;
    }

    return result;
}

/*******************************************************************************
* Function Name: app_init
********************************************************************************
* Summary:
*   This function handles application level initialization tasks and is called
*   from the BT management callback once the LE stack enabled event
*   (BTM_ENABLED_EVT) is triggered This function is executed in the
*   BTM_ENABLED_EVT management callback.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
static void app_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t wiced_result = WICED_BT_SUCCESS;
    if(platform_gpio_write(gpio_cfg.wake_on_ble_cfg.dev_wake.p_gpiochip, gpio_cfg.wake_on_ble_cfg.dev_wake.line_num, GPIO_ASSERT(WICED_SLEEP_MODE_BT_WAKE_ACT_LOW), "DEV-WAKE") == WICED_FALSE)
    {
        TRACE_ERR("DEV-WAKE ASSERT Failed\n");
    }

    /* Load previous paired keys for address resolution */
    app_load_keys_for_address_resolution();

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);
    app_ready = WICED_TRUE;
    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(app_gatts_callback);
    TRACE_MSG("wiced_bt_gatt_register: %d\n", gatt_status);

    current_mode = NONE;
}

/******************************************************************************
 * Function Name: app_load_keys_for_address_resolution
 *******************************************************************************
 * Summary: This function read the saving link_key, and write to stack for
 *          bd address resolution
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 ******************************************************************************/
static void app_load_keys_for_address_resolution(void)
{
    wiced_bt_device_link_keys_t link_keys;
    wiced_result_t              result = WICED_ERROR;
    uint8_t* p;

    memset(&link_keys, 0, sizeof(wiced_bt_device_link_keys_t));
    p = (uint8_t*)&link_keys;
    wiced_hal_read_nvram(APP_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p, &result);

    if (result == WICED_BT_SUCCESS)
    {
        result = wiced_bt_dev_add_device_to_address_resolution_db(&link_keys);
    }
    TRACE_LOG("app_load_keys_for_address_resolution result:%d\n", result);
}

static wiced_bt_gatt_status_t app_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t* p_data)
{
    wiced_bt_gatt_status_t result;

    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = app_gatts_conn_status_cb(&p_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = app_gatts_req_cb(&p_data->attribute_request);
        break;
    default:
        break;
    }
}

/*
 */
 /******************************************************************************
  * Function Name: app_smp_bond_result
  *******************************************************************************
  * Summary:
  *      Process SMP bonding result. If we successfully paired with the
  *      central device, save its BDADDR in the NVRAM and initialize
  *      associated data
  *
  * Parameters:
  *  uint8_t result
  *
  * Return:
  *  None
  *
  ******************************************************************************/
static void app_smp_bond_result(uint8_t result)
{
    wiced_result_t status;
    uint8_t written_byte = 0;
    TRACE_LOG("\n app_smp_bond_result, bond result: %d\n", result);

    /* Bonding success */
    if (result == WICED_BT_SUCCESS)
    {
        /* Pack the data to be stored into the hostinfo structure */
        memcpy(app_host_info.bdaddr, app_state.remote_addr, sizeof(wiced_bt_device_address_t));
        app_host_info.addr_type = app_state.addr_type;
        app_host_info.paired = 1;

        TRACE_LOG("ADDR saved to NVRAM: %x %x %x %x %x %x ", app_host_info.bdaddr[0], app_host_info.bdaddr[1], \
            app_host_info.bdaddr[2], app_host_info.bdaddr[3], app_host_info.bdaddr[4], app_host_info.bdaddr[5]);

        TRACE_LOG("ADDR Type: %x ", app_host_info.addr_type);
        TRACE_LOG("Paired: %x ", app_host_info.paired);

        /* Write to NVRAM */
        written_byte = wiced_hal_write_nvram(APP_VS_ID, sizeof(app_host_info), (uint8_t*)&app_host_info, &status);
        TRACE_LOG("\n NVRAM write: %d\n", written_byte);
    }
}

static wiced_bt_gatt_status_t app_gatts_conn_status_cb(wiced_bt_gatt_connection_status_t * p_status)
{
    if (p_status->connected)
    {
        return app_gatts_connection_up(p_status);
    }

    return app_gatts_connection_down(p_status);
}

static wiced_bt_gatt_status_t app_gatts_connection_up(wiced_bt_gatt_connection_status_t* p_status)
{
    wiced_result_t result;
    
    TRACE_MSG("Connection UP:  id:%d\n:", p_status->conn_id);
    TRACE_MSG("BD ADDR");
    print_bd_address(p_status->bd_addr);
    TRACE_MSG("ADDR_TYPE: %d", p_status->addr_type);

    app_state.conn_id = p_status->conn_id;
    memcpy(app_state.remote_addr, p_status->bd_addr, sizeof(wiced_bt_device_address_t));
    app_state.addr_type = p_status->addr_type;

    /* Saving host info in NVRAM */
    memcpy(app_host_info.bdaddr, p_status->bd_addr, BD_ADDR_LEN);
    app_host_info.addr_type = p_status->addr_type;

    wiced_hal_write_nvram(APP_VS_ID, sizeof(app_host_info), (uint8_t*)&app_host_info, &result);
    TRACE_LOG("NVRAM write %d\n", result);
  
    current_mode = NONE;
}

static wiced_bt_gatt_status_t app_gatts_connection_down(wiced_bt_gatt_connection_status_t* p_status)
{
    /* Resetting the device info */
    memset(app_state.remote_addr, 0, 6);
    app_state.conn_id = 0;
    //do nothing , wait for user input on what to do next?
    TRACE_MSG("LE Disconnected\n");
}

static wiced_bt_gatt_status_t app_gatts_req_cb(wiced_bt_gatt_attribute_request_t* p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_SUCCESS;
   
    switch (p_data->opcode)
    {
        case GATT_REQ_MTU:
            TRACE_LOG("req_mtu: %d\n", p_data->data.remote_mtu);
            result = wiced_bt_gatt_server_send_mtu_rsp(p_data->conn_id, p_data->data.remote_mtu, cy_bt_cfg_ble.ble_max_rx_pdu_size);
            break;
        default:
            TRACE_LOG("Unhandled Opcode %x", p_data->opcode);
        break;
    }

    return result;
}


/*******************************************************************************
* Function Name: app_set_sleep_mode
********************************************************************************
* Summary: dev-wake assert fisrt, then call set_sleep_mode
* 
* Parameters:
*   None
*
* Return:
*   BOOL32:
*         WICED_TRUE:  SUCCESS 
*         WICED_FALSE: ERROR HAPPENED
*
*******************************************************************************/
static BOOL32 app_set_sleep_mode(void)
{
    TRACE_LOG("DEV-WAKE ASSERT FIRST\n");
    /* dev wake assert first */
    if(platform_gpio_write(gpio_cfg.wake_on_ble_cfg.dev_wake.p_gpiochip, gpio_cfg.wake_on_ble_cfg.dev_wake.line_num, GPIO_ASSERT(WICED_SLEEP_MODE_BT_WAKE_ACT_LOW), "DEV-WAKE") == WICED_FALSE)
    {
        TRACE_ERR("DEV-WAKE ASSERT Failed\n");
        return WICED_FALSE;
    }
    /* set sleep mode with param */
    if(wiced_set_sleep_mode_with_param(BTM_SLEEP_MODE_UART, WICED_SLEEP_MODE_BT_WAKE_ACT_LOW, WICED_SLEEP_MODE_HOST_WAKE_ACT_LOW, WICED_TRUE, bt_sleep_cmpl_cback) == WICED_FALSE)
    {
        TRACE_ERR("set sleep mode with param Failed");
        return WICED_FALSE;
    }
    return WICED_TRUE;
}

/*******************************************************************************
* Function Name: app_clear_apcf_setting
********************************************************************************
* Summary: 
*   This Function clear all apcf filter param setting
* 
* Parameters:
*   None
*
* Return:
*   BOOL32:
*         WICED_TRUE:  SUCCESS 
*         WICED_FALSE: ERROR HAPPENED
*
*******************************************************************************/
static BOOL32 app_clear_apcf_setting(void)
{
    TRACE_LOG("\n");

    /* disable apcf first */
    if (wiced_set_apcf_enable(WICED_FALSE) == WICED_FALSE)
    {
        TRACE_ERR("set apcf disable Failed\n");
        return WICED_FALSE;
    }
    
    /* clear apcf filter setting */
    if (wiced_set_apcf_filter_param(WICED_LE_ADV_PCF_ACT_CLEAR, apcf_filter_idx, WICED_LE_ADV_PCF_FEA_NONE,
                          WICED_LE_ADV_PCF_FEA_NONE, WICED_LE_ADV_PCF_LOGIC_AND, WICED_LE_ADV_PCF_RSSI_HIGH_THRESHOLD, WICED_LE_ADV_PCF_DELIVERY_MODE_IMMEDIATE) == WICED_FALSE)
    {
        TRACE_ERR("set_apcf_filter_param Failed\n");
        return WICED_FALSE;
    }

    TRACE_LOG("success\n");
    return WICED_TRUE;
}

/*******************************************************************************
* Function Name: app_set_apcf_setting
********************************************************************************
* Summary: 
*   This Function set apcf data, apcf filter param and enable apcf
* 
* Parameters:
*   None
*
* Return:
*   BOOL32:
*         WICED_TRUE:  SUCCESS 
*         WICED_FALSE: ERROR HAPPENED
*
*******************************************************************************/
BOOL32 app_set_apcf_setting()
{
    TRACE_LOG("\n");
    /* set apcf data uuid */
    if (wiced_set_apcf_data_uuid(uuid, WICED_LE_ADV_PCF_ACT_ADD, apcf_filter_idx) == WICED_FALSE)
    {
        TRACE_ERR("set_apcf_data Failed\n");
        return WICED_FALSE;
    }

    /* set apcf filter param */
    if (wiced_set_apcf_filter_param(WICED_LE_ADV_PCF_ACT_ADD, apcf_filter_idx, WICED_LE_ADV_PCF_FEA_SRVC_UUID,
                          WICED_LE_ADV_PCF_FEA_SRVC_UUID, WICED_LE_ADV_PCF_LOGIC_AND, WICED_LE_ADV_PCF_RSSI_HIGH_THRESHOLD, WICED_LE_ADV_PCF_DELIVERY_MODE_IMMEDIATE) == WICED_FALSE)
    {
        TRACE_ERR("set_apcf_filter_param Failed\n");
        return WICED_FALSE;
    }

    /* enable apcf */
    if(wiced_set_apcf_enable(WICED_TRUE) == WICED_FALSE)
    {
        TRACE_ERR("set apcf enable Failed\n");
        return WICED_FALSE;
    }

    TRACE_LOG("success\n");
    return WICED_TRUE;
}

/*******************************************************************************
* Function Name: app_disable_wake_on_ble
********************************************************************************
* Summary:
*   This Function disable wake on ble and stop le scan
* 
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void app_disable_wake_on_le()
{
    TRACE_LOG("\n");
    if (inSleep == WICED_FALSE)
    {
        TRACE_LOG("[%s]:Not in Sleep.\n", __FUNCTION__);
        return;
    }

    /* assert gpio DEV-WAKE */
    if (platform_gpio_write(gpio_cfg.wake_on_ble_cfg.dev_wake.p_gpiochip, gpio_cfg.wake_on_ble_cfg.dev_wake.line_num, GPIO_ASSERT(WICED_SLEEP_MODE_BT_WAKE_ACT_LOW), "DEV-WAKE") == WICED_FALSE)
    {
        TRACE_ERR("Assert DEV WAKE Failed\n");
        return;
    }
    TRACE_LOG("Disable Le Scan\n");
    /* disable le scan */
    /* wiced bt stack api */
    if (wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, app_scan_result_cback) != 0)
    {
        TRACE_ERR("Disable Le Scan Failed\n");
        return;
    }
    TRACE_LOG("success\n");
}

/*******************************************************************************
* Function Name: app_enable_wake_on_ble_with_manu
********************************************************************************
* Summary:
*   Enalbe Wake On LE with uuid AND Manufacture Data
* 
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void app_enable_wake_on_le_uuid_manu()
{
    current_mode = WAKE_ON_UUID;
    TRACE_LOG("\n");
    wiced_result_t status = WICED_BT_SUCCESS;
    
    /* clear apcf setting first */
    if(app_clear_apcf_setting() == WICED_FALSE)
    {
        TRACE_ERR("app_clear_apcf_setting Failed\n");
        return;
    }

    /* set apcf data uuid */
    if (wiced_set_apcf_data_uuid(uuid, WICED_LE_ADV_PCF_ACT_ADD, apcf_filter_idx) == WICED_FALSE)
    {
        TRACE_ERR("set_apcf_data Failed\n");
        return;
    }
    memset(pattern_mask, 0xFF, sizeof(pattern_mask));
    /* set apcf data manufacture */
    if (wiced_set_apcf_data_manufacture(company_id, data_len, pattern, company_id_mask, pattern_mask, WICED_LE_ADV_PCF_ACT_ADD, apcf_filter_idx) == WICED_FALSE)
    {
        TRACE_ERR("set_apcf_data Failed\n");
        return;
    }

    /* set apcf filter param */
    if (wiced_set_apcf_filter_param(WICED_LE_ADV_PCF_ACT_ADD, apcf_filter_idx, WICED_LE_ADV_PCF_FEA_SRVC_UUID | WICED_LE_ADV_PCF_FEA_MANU_DATA,
                          WICED_LE_ADV_PCF_FEA_SRVC_UUID | WICED_LE_ADV_PCF_FEA_MANU_DATA, WICED_LE_ADV_PCF_LOGIC_AND, WICED_LE_ADV_PCF_RSSI_HIGH_THRESHOLD, WICED_LE_ADV_PCF_DELIVERY_MODE_IMMEDIATE) == WICED_FALSE)
    {
        TRACE_ERR("set_apcf_filter_param Failed\n");
        return;
    }

    /* enable apcf */
    if(wiced_set_apcf_enable(WICED_TRUE) == WICED_FALSE)
    {
        TRACE_ERR("set apcf enable Failed\n");
        return;
    }

    /* enable ble scan */
    /* wiced bt stack api */
    status = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_LOW_DUTY, WICED_TRUE, app_scan_result_cback);
    if ((WICED_BT_PENDING != status ) && ( WICED_BT_BUSY != status))
    {
        TRACE_ERR("enable ble scan Failed\n");
        return;
    }

    /* set sleep mode */
    if (app_set_sleep_mode() == WICED_FALSE)
    {
        TRACE_ERR("set sleep mode Failed\n");
        return;
    }
    TRACE_LOG("success\n");
}

/*******************************************************************************
* Function Name: app_enable_wake_on_ble_uuid
********************************************************************************
* Summary:
*   Enalbe Wake On LE with uuid
* 
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void app_enable_wake_on_le_uuid()
{
    current_mode = WAKE_ON_UUID;
    TRACE_LOG("\n");
    wiced_result_t status = WICED_BT_SUCCESS;

    /* clear apcf first */
    if(app_clear_apcf_setting() == WICED_FALSE)
    {
        TRACE_ERR("app_clear_apcf_setting Failed\n");
        return;
    }

    /* set apcf data uuid */
    if (wiced_set_apcf_data_uuid(uuid, WICED_LE_ADV_PCF_ACT_ADD, apcf_filter_idx) == WICED_FALSE)
    {
        TRACE_ERR("set_apcf_data Failed\n");
        return;
    }

    /* set apcf filter param */
    if (wiced_set_apcf_filter_param(WICED_LE_ADV_PCF_ACT_ADD, apcf_filter_idx, WICED_LE_ADV_PCF_FEA_SRVC_UUID,
                          WICED_LE_ADV_PCF_FEA_SRVC_UUID, WICED_LE_ADV_PCF_LOGIC_AND, WICED_LE_ADV_PCF_RSSI_HIGH_THRESHOLD, WICED_LE_ADV_PCF_DELIVERY_MODE_IMMEDIATE) == WICED_FALSE)
    {
        TRACE_ERR("set_apcf_filter_param Failed\n");
        return;
    }

    /* enable apcf */
    if (wiced_set_apcf_enable(WICED_TRUE) == WICED_FALSE)
    {
        TRACE_ERR("set apcf enable Failed\n");
        return;
    }

    /* enable ble scan */
    /* wiced bt stack api */
    status = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_LOW_DUTY, WICED_TRUE, app_scan_result_cback);
    if ((WICED_BT_PENDING != status ) && ( WICED_BT_BUSY != status))
    {
        TRACE_ERR("enable ble scan Failed, status:%d\n", status);
        return;
    }

    /* set sleep mode */
    if (app_set_sleep_mode() == WICED_FALSE)
    {
        TRACE_ERR("set sleep mode Failed\n");
        return;
    }
    TRACE_LOG("success\n");
}

void app_start_new_connection(void)
{
    wiced_result_t result;
    wiced_bt_ble_advert_mode_t advert_mode = BTM_BLE_ADVERT_UNDIRECTED_HIGH;
    current_mode = NEW_CONNECTION;

    /* Accept connection request from any peer */
    if((wiced_btm_ble_update_advertisement_filter_policy(BTM_BLE_ADV_POLICY_ACCEPT_CONN_AND_SCAN)) != WICED_TRUE)
        TRACE_MSG("wiced_btm_ble_update_advertisement_filter_policy failed ");

    /* Clear all previously available bond data and devices */
    app_clear_bond_info();

    /* Set Advertisement Data */
    app_set_adv_data();

    /* Start advertisement */
    result = wiced_bt_start_advertisements(advert_mode, BLE_ADDR_PUBLIC, NULL);
    TRACE_MSG("wiced_bt_start_advertisements %d", result);
    TRACE_LOG("success\n");
}

static void app_clear_bond_info(void)
{
    TRACE_MSG("Clear previously connected devices if available");
    wiced_result_t status = WICED_SUCCESS;
    memset(&app_host_info, 0, sizeof(host_info_t));

    wiced_hal_delete_nvram(APP_VS_ID, &status);
    wiced_hal_delete_nvram(APP_PAIRED_KEYS_VS_ID, &status);
}


void app_enable_wake_on_connection()
{
    wiced_result_t result;
    uint8_t peer_addr[BD_ADDR_LEN];
    wiced_bt_ble_address_type_t peer_addr_type;
    wiced_bt_ble_advert_mode_t advert_mode = BTM_BLE_ADVERT_UNDIRECTED_HIGH;
    wiced_bt_device_link_keys_t p_keys;
    host_info_t host_data;
    uint16_t read_bytes = 0;

    current_mode = WAKE_ON_CONNECTION;

   /* Update advertisement filter policy */
    if (wiced_btm_ble_update_advertisement_filter_policy(BTM_BLE_ADV_POLICY_FILTER_CONN_ACCEPT_SCAN) != WICED_TRUE)
        TRACE_MSG("wiced_btm_ble_update_advertisement_filter_policy failed ");

    /* Read host info */
    read_bytes = wiced_hal_read_nvram(APP_VS_ID,
        sizeof(host_info_t),
        (uint8_t*)&host_data,
        &result);

    /* Return if no peer info available */
    if (result != WICED_BT_SUCCESS)
    {
        TRACE_LOG("\n No previous connected device found. Initiate new connection first using option 6");
        return;
    }

    /* Check if device is previously paired */
    if (host_data.paired)
    {
        TRACE_LOG("Device previously paired");
        /* Retrieve public ID address */
        read_bytes = wiced_hal_read_nvram(APP_PAIRED_KEYS_VS_ID,
            sizeof(wiced_bt_device_link_keys_t),
            (uint8_t*)&p_keys,
            &result);

        if (result == WICED_BT_SUCCESS)
        {
            /* Previous paired device found, add Public ID to the accept list */
            memcpy(peer_addr, p_keys.bd_addr, 6);
            peer_addr_type = BLE_ADDR_PUBLIC_ID;
        }
        else
        {
            TRACE_LOG("Paired key missing: Error!");
            return;
        }
    }
    else
    {
        /* If not paired, add previous connection address to whitelist */
        memcpy(peer_addr, host_data.bdaddr, sizeof(wiced_bt_device_address_t));
        peer_addr_type = host_data.addr_type;
    }

    /* Clear existing devices from the accept list */
    wiced_bt_ble_clear_filter_accept_list();

    /* Add peer device to the allow list */
    if (wiced_bt_ble_update_advertising_filter_accept_list(WICED_TRUE, peer_addr_type, peer_addr) == WICED_TRUE)
    {
        TRACE_MSG("Peer device %x %x %x %x %x %x added to Whitelist\n", peer_addr[0], peer_addr[1], \
            peer_addr[2], peer_addr[3], peer_addr[4], peer_addr[5]);
    }
    else
    {
        TRACE_MSG("failed to update adv filter accept list");
    }
 
    /* Set Advertisement Data */
    app_set_adv_data();
    
    /* Start advertisement */ 
    result = wiced_bt_start_advertisements(advert_mode, BLE_ADDR_PUBLIC, NULL);
    TRACE_MSG("wiced_bt_start_advertisements %d", result);

    /* Set sleep mode */
    if (app_set_sleep_mode() == WICED_FALSE)
    {
        TRACE_ERR("set sleep mode Failed\n");
        return;
    }
    TRACE_LOG("success\n");
}

wiced_bt_dev_status_t app_set_adv_data()
{
    wiced_bt_dev_status_t result = WICED_BT_ERROR;
    uint8_t adv_data[BTM_BLE_LEGACY_AD_DATA_LEN];
    int used = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    
    used += wiced_bt_ble_build_raw_advertisement_data(adv_data + used, sizeof(adv_data) - used,
        BTM_BLE_ADVERT_TYPE_FLAG, &flag, sizeof(uint8_t));

    used += wiced_bt_ble_build_raw_advertisement_data(adv_data + used, sizeof(adv_data) - used,
        BTM_BLE_ADVERT_TYPE_NAME_COMPLETE, (uint8_t*)wiced_bt_cfg_settings.device_name,
        (uint8_t)strlen((const char*)wiced_bt_cfg_settings.device_name));

    result = wiced_bt_ble_set_legacy_adv_data(used, adv_data);

    TRACE_MSG("[%s] set adv of len %d result 0x%x", __FUNCTION__, used, result);

    return result;
}

/*******************************************************************************
* Function Name: app_scan_result_cback
********************************************************************************
* Summary:
*   This callback function handles the le scan results
*   if in WakeOnLE mode, will not trigeer it
*   if want to test APCF Function.
*   can use this function and enable APCF with UUID to see the ADV
*
* Parameters:
*   wiced_bt_ble_scan_results_t* p_scan_result:
*            scan result from stack
*   uint8_t* p_adv_data:
*            scan data 
* Return:
*   None
*
*******************************************************************************/
static void app_scan_result_cback(wiced_bt_ble_scan_results_t* p_scan_result, uint8_t* p_adv_data)
{
    if (p_scan_result)
    {
        TRACE_LOG("Got ADV from: %s\n", p_scan_result->remote_bd_addr);
	print_bd_address(p_scan_result->remote_bd_addr);
    } else {
        TRACE_LOG("Scan completed:\n");
    }
}
 
/*******************************************************************************
* Function Name: bt_sleep_cmpl_cback
********************************************************************************
* Summary:
*   Callback function for set sleep mode, when set sleep mode complete,    
*   need use dev-wake let Controll enter sleep mode.
*   and start a thread to monitor Host-Wake
*
* Parameters:
*   tBTM_VSC_CMPL *p_params:
*             param of VSC command data 
*
* Return:
*   None
*
*******************************************************************************/
static void bt_sleep_cmpl_cback(tBTM_VSC_CMPL *p_params) 
{
    if (p_params == NULL)
    {
        TRACE_ERR("p_params is NULL\n");
        return;
    }
    TRACE_LOG("opcode: %x, param_len:%d\n", p_params->opcode, p_params->param_len);
    uint8_t  status = 0;
    uint8_t  *p = p_params->p_param_buf, op_subcode, action = 0xff;
    STREAM_TO_UINT8(status, p);
    
    if (status == HCI_SUCCESS) 
    {
	    TRACE_LOG("set sleep mode success \n");
 	    TRACE_LOG("Ready Enter UART Sleep Mode\n");
        if (platform_gpio_write(gpio_cfg.wake_on_ble_cfg.dev_wake.p_gpiochip, gpio_cfg.wake_on_ble_cfg.dev_wake.line_num, GPIO_DEASSERT(WICED_SLEEP_MODE_BT_WAKE_ACT_LOW), "DEV-WAKE") == WICED_FALSE)
        {
	    TRACE_ERR("Deassert DEV WAKE Failed\n");
            return;
        }
	    gpio_cfg.wake_on_ble_cfg.host_wake_args.gpio_event_cb = &bt_host_wake_assert_cback;
        gpio_cfg.wake_on_ble_cfg.host_wake_args.gpio_event_flag = GPIOEVENT_REQUEST_FALLING_EDGE;
	    if (platform_gpio_poll(&(gpio_cfg.wake_on_ble_cfg.host_wake_args)) == WICED_FALSE)
        {
            TRACE_ERR("Monitor host-wake Failed\n");
            return;
        }
    } 
    else 
    {
        TRACE_ERR("Set Sleep Mode Param Failed, status:%d\n", status);
        return;
    }
    
    inSleep = WICED_TRUE;
}

/*******************************************************************************
* Function Name: bt_host_wake_assert_cback
********************************************************************************
* Summary:
*   Callback function when host-wake assert, assert Dev-Wake, let Controller 
*   leave sleep mode, and stop le-scan
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
static void bt_host_wake_assert_cback()
{
    TRACE_LOG("HOST WAKE ASSERT\n");
    if (platform_gpio_write(gpio_cfg.wake_on_ble_cfg.dev_wake.p_gpiochip, gpio_cfg.wake_on_ble_cfg.dev_wake.line_num, GPIO_ASSERT(WICED_SLEEP_MODE_BT_WAKE_ACT_LOW), "DEV-WAKE") == WICED_FALSE)
    {
	TRACE_ERR("assert DEV WAKE Failed\n");
        return;
    }
    if (current_mode == WAKE_ON_CONNECTION)
    {
        TRACE_LOG("Connection Request Received - Wake Host\n");
    }
    else if (current_mode == WAKE_ON_UUID)
    {
        TRACE_LOG("Disable Le scan\n");
        /* disable le scan */
        /* wiced bt stack api */
        if (wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, app_scan_result_cback) != 0)
        {
            TRACE_ERR("disable ble scan Failed\n");
            return;
        }

        /* disable apcf first */
        if (wiced_set_apcf_enable(WICED_FALSE) == WICED_FALSE)
        {
            TRACE_ERR("set apcf disable Failed\n");
            return;
        }

        /* clear apcf filter setting */
        if (wiced_set_apcf_filter_param(WICED_LE_ADV_PCF_ACT_CLEAR, apcf_filter_idx, WICED_LE_ADV_PCF_FEA_NONE,
            WICED_LE_ADV_PCF_FEA_NONE, WICED_LE_ADV_PCF_LOGIC_AND, WICED_LE_ADV_PCF_RSSI_HIGH_THRESHOLD, WICED_LE_ADV_PCF_DELIVERY_MODE_IMMEDIATE) == WICED_FALSE)
        {
            TRACE_ERR("set_apcf_filter_param Failed\n");
            return;
        }
    }
    
    /* disable sleep mode */
    if(wiced_set_sleep_mode_with_param(BTM_SLEEP_MODE_NONE, WICED_SLEEP_MODE_BT_WAKE_ACT_LOW, WICED_SLEEP_MODE_HOST_WAKE_ACT_LOW, WICED_FALSE, NULL) == WICED_FALSE)
    {
        TRACE_ERR("set sleep mode with param Failed");
        return;
    }

    inSleep = WICED_FALSE;
}



/* END OF FILE [] */
