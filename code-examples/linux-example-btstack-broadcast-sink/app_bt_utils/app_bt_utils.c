/******************************************************************************
 * File Name: app_bt_utils.c 
 *
 * Description: This file consists of the utility functions that will help
 *          debugging and developing the applications easier with much more
 *          meaningful information.
 *
 * Related Document: See README.md
 *
 ******************************************************************************
* $ Copyright 2022-YEAR Cypress Semiconductor $
*/
/******************************************************************************
 *      INCLUDES
 ******************************************************************************/
#include "app_bt_utils.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_avrc_tg.h"
#include "wiced_bt_a2dp_source.h"
#include "wiced_hal_memory.h"
#include "log.h"


/******************************************************************************
* EXTERN Variables 
******************************************************************************/
extern wiced_bt_cfg_settings_t broadcast_sink_cfg_settings;

/******************************************************************************
* Variables Definitions
******************************************************************************/
uint8_t bt_device_address[BD_ADDR_LEN] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/******************************************************************************
*       MACRO DEFINITIONS
******************************************************************************/
#define CASE_DEFAULT_STR default: printf("%s: defalut case\n", __FUNCTION__); break;

/******************************************************************************
 *      FUNCTION DEFINITIONS
 *****************************************************************************/

/******************************************************************************
* Function Name: print_bd_address()
*******************************************************************************
* Summary:
*   This is the utility function that prints the address of the Bluetooth
*   device
*
* Parameters:
*   wiced_bt_device_address_t bdadr                : Bluetooth address
*
* Return:
*  void
*
*******************************************************************************/
void print_bd_address(wiced_bt_device_address_t bdadr)
{
    printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
            bdadr[0], bdadr[1], bdadr[2], bdadr[3], bdadr[4], bdadr[5]);
}

/*******************************************************************************
* Function Name: void print_array( void* to_print, uint16_t len )
********************************************************************************
* Summary:
*   This is a utility function that prints the specified number of values from
*   memory
*
* Parameters:
*   void* to_print                : Pointer to the location to print
*   uint16_t                    : Number of bytes to print
*
* Return:
*  void
*
********************************************************************************/
void print_array(void * to_print, uint16_t len)
{
    uint16_t counter;

    for (counter = 0; counter < len; counter++)
    {
        if (0 == counter % 16)
        {
           printf("\n");
        }
        printf("%02X ", *(((uint8_t *)(to_print)) + counter));
    }
    printf("\n");

}

/*******************************************************************************
* Function Name: get_bt_event_name
********************************************************************************
* Summary:
* The function converts the wiced_bt_management_evt_t enum value to its
* corresponding string literal. This will help the programmer to debug easily
* with log traces without navigating through the source code.
*
* Parameters:
*  wiced_bt_management_evt_t event: Bluetooth management event type
*
* Return:
*  wiced_bt_management_evt_t
*
*******************************************************************************/
const char *get_bt_event_name(wiced_bt_management_evt_t event)
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
    CASE_RETURN_STR(BTM_BLE_PHY_UPDATE_EVT)
    CASE_DEFAULT_STR
    }

    return "UNKNOWN_EVENT";
}

/*******************************************************************************
* Function Name: get_bt_advert_mode_name
********************************************************************************
* Summary:
*  The function converts the wiced_bt_ble_advert_mode_t enum value to its
*  corresponding string literal. This will help the programmer to debug easily
*  with log traces without navigating through the source code.
*
* Parameters:
*  wiced_bt_ble_advert_mode_t mode: Bluetooth advertisement mode type
*
* Return:
*  wiced_bt_ble_advert_mode_t
*
*******************************************************************************/
const char *get_bt_advert_mode_name(wiced_bt_ble_advert_mode_t mode)
{
    switch ((int)mode)
    {
    CASE_RETURN_STR(BTM_BLE_ADVERT_OFF)
    CASE_RETURN_STR(BTM_BLE_ADVERT_DIRECTED_HIGH)
    CASE_RETURN_STR(BTM_BLE_ADVERT_DIRECTED_LOW)
    CASE_RETURN_STR(BTM_BLE_ADVERT_UNDIRECTED_HIGH)
    CASE_RETURN_STR(BTM_BLE_ADVERT_UNDIRECTED_LOW)
    CASE_RETURN_STR(BTM_BLE_ADVERT_NONCONN_HIGH)
    CASE_RETURN_STR(BTM_BLE_ADVERT_NONCONN_LOW)
    CASE_RETURN_STR(BTM_BLE_ADVERT_DISCOVERABLE_HIGH)
    CASE_RETURN_STR(BTM_BLE_ADVERT_DISCOVERABLE_LOW)
    CASE_DEFAULT_STR
    }

    return "UNKNOWN_MODE";
}

/*******************************************************************************
* Function Name: get_bt_gatt_disconn_reason_name
********************************************************************************
* Summary:
*  The function converts the wiced_bt_gatt_disconn_reason_t enum value to its
*  corresponding string literal. This will help the programmer to debug easily
*  with log traces without navigating through the source code.
*
* Parameters:
*  wiced_bt_gatt_disconn_reason_t reason: GATT Disconnection reason
*
* Return:
*  wiced_bt_gatt_disconn_reason_t
*
*******************************************************************************/
const char *get_bt_gatt_disconn_reason_name(wiced_bt_gatt_disconn_reason_t reason)
{
    switch ((int)reason)
    {
    CASE_RETURN_STR(GATT_CONN_UNKNOWN)
    CASE_RETURN_STR(GATT_CONN_L2C_FAILURE)
    CASE_RETURN_STR(GATT_CONN_TIMEOUT)
    CASE_RETURN_STR(GATT_CONN_TERMINATE_PEER_USER)
    CASE_RETURN_STR(GATT_CONN_TERMINATE_LOCAL_HOST)
    CASE_RETURN_STR(GATT_CONN_FAIL_ESTABLISH)
    CASE_RETURN_STR(GATT_CONN_LMP_TIMEOUT)
    CASE_RETURN_STR(GATT_CONN_CANCEL)
    CASE_DEFAULT_STR
    }

    return "UNKNOWN_REASON";
}

/*******************************************************************************
* Function Name: get_bt_gatt_status_name
********************************************************************************
* Summary:
*  The function converts the wiced_bt_gatt_disconn_reason_t enum value to its
*  corresponding string literal. This will help the programmer to debug easily
*  with log traces without navigating through the source code.
*
* Parameters:
*  wiced_bt_gatt_status_t status: GATT status
*
* Return:
*  wiced_bt_gatt_status_t
*
*******************************************************************************/
const char *get_bt_gatt_status_name(wiced_bt_gatt_status_t status)
{
    switch ((int)status)
    {
    CASE_RETURN_STR(WICED_BT_GATT_SUCCESS || WICED_BT_GATT_ENCRYPTED_MITM)
    CASE_RETURN_STR(WICED_BT_GATT_INVALID_HANDLE)
    CASE_RETURN_STR(WICED_BT_GATT_READ_NOT_PERMIT)
    CASE_RETURN_STR(WICED_BT_GATT_WRITE_NOT_PERMIT)
    CASE_RETURN_STR(WICED_BT_GATT_INVALID_PDU)
    CASE_RETURN_STR(WICED_BT_GATT_INSUF_AUTHENTICATION)
    CASE_RETURN_STR(WICED_BT_GATT_REQ_NOT_SUPPORTED)
    CASE_RETURN_STR(WICED_BT_GATT_INVALID_OFFSET)
    CASE_RETURN_STR(WICED_BT_GATT_INSUF_AUTHORIZATION)
    CASE_RETURN_STR(WICED_BT_GATT_PREPARE_Q_FULL)
    CASE_RETURN_STR(WICED_BT_GATT_ATTRIBUTE_NOT_FOUND)
    CASE_RETURN_STR(WICED_BT_GATT_NOT_LONG)
    CASE_RETURN_STR(WICED_BT_GATT_INSUF_KEY_SIZE)
    CASE_RETURN_STR(WICED_BT_GATT_INVALID_ATTR_LEN)
    CASE_RETURN_STR(WICED_BT_GATT_ERR_UNLIKELY)
    CASE_RETURN_STR(WICED_BT_GATT_INSUF_ENCRYPTION)
    CASE_RETURN_STR(WICED_BT_GATT_UNSUPPORT_GRP_TYPE)
    CASE_RETURN_STR(WICED_BT_GATT_INSUF_RESOURCE)
    CASE_RETURN_STR(WICED_BT_GATT_ILLEGAL_PARAMETER)
    CASE_RETURN_STR(WICED_BT_GATT_NO_RESOURCES)
    CASE_RETURN_STR(WICED_BT_GATT_INTERNAL_ERROR)
    CASE_RETURN_STR(WICED_BT_GATT_WRONG_STATE)
    CASE_RETURN_STR(WICED_BT_GATT_DB_FULL)
    CASE_RETURN_STR(WICED_BT_GATT_BUSY)
    CASE_RETURN_STR(WICED_BT_GATT_ERROR)
    CASE_RETURN_STR(WICED_BT_GATT_CMD_STARTED)
    CASE_RETURN_STR(WICED_BT_GATT_PENDING)
    CASE_RETURN_STR(WICED_BT_GATT_AUTH_FAIL)
    CASE_RETURN_STR(WICED_BT_GATT_MORE)
    CASE_RETURN_STR(WICED_BT_GATT_INVALID_CFG)
    CASE_RETURN_STR(WICED_BT_GATT_SERVICE_STARTED)
    CASE_RETURN_STR(WICED_BT_GATT_ENCRYPTED_NO_MITM)
    CASE_RETURN_STR(WICED_BT_GATT_NOT_ENCRYPTED)
    CASE_RETURN_STR(WICED_BT_GATT_CONGESTED)
    CASE_RETURN_STR(WICED_BT_GATT_WRITE_REQ_REJECTED)
    CASE_RETURN_STR(WICED_BT_GATT_CCC_CFG_ERR)
    CASE_RETURN_STR(WICED_BT_GATT_PRC_IN_PROGRESS)
    CASE_RETURN_STR(WICED_BT_GATT_OUT_OF_RANGE)
    CASE_DEFAULT_STR
    }

    return "UNKNOWN_STATUS";
}

/*******************************************************************************
* Function Name: get_bt_smp_status_name
********************************************************************************
* Summary:
*  The function converts the wiced_bt_gatt_disconn_reason_t enum value to its
*  corresponding string literal. This will help the programmer to debug easily
*  with log traces without navigating through the source code.
*
* Parameters:
*  wiced_bt_smp_status_t status: GATT status
*
* Return:
*  wiced_bt_smp_status_t
*
*******************************************************************************/
const char *get_bt_smp_status_name(wiced_bt_smp_status_t status)
{
    switch ((int)status)
    {
    CASE_RETURN_STR(SMP_SUCCESS)                 /**< Success */
    CASE_RETURN_STR(SMP_PASSKEY_ENTRY_FAIL)      /**< Passkey entry failed */
    CASE_RETURN_STR(SMP_OOB_FAIL)                /**< OOB failed */
    CASE_RETURN_STR(SMP_PAIR_AUTH_FAIL)          /**< Authentication failed */
    CASE_RETURN_STR(SMP_CONFIRM_VALUE_ERR)       /**< Value confirmation failed */
    CASE_RETURN_STR(SMP_PAIR_NOT_SUPPORT)        /**< Not supported */
    CASE_RETURN_STR(SMP_ENC_KEY_SIZE)            /**< Encryption key size failure */
    CASE_RETURN_STR(SMP_INVALID_CMD)             /**< Invalid command */
    CASE_RETURN_STR(SMP_PAIR_FAIL_UNKNOWN)       /**< Unknown failure */
    CASE_RETURN_STR(SMP_REPEATED_ATTEMPTS)       /**< Repeated attempts */
    CASE_RETURN_STR(SMP_INVALID_PARAMETERS)      /**< Invalid parameters  */
    CASE_RETURN_STR(SMP_DHKEY_CHK_FAIL)          /**< DH Key check failed */
    CASE_RETURN_STR(SMP_NUMERIC_COMPAR_FAIL)     /**< Numeric comparison failed */
    CASE_RETURN_STR(SMP_BR_PAIRING_IN_PROGR)     /**< BR paIring in progress */
    CASE_RETURN_STR(SMP_XTRANS_DERIVE_NOT_ALLOW) /**< Cross transport key derivation not allowed */
    /* bte smp status codes */
    CASE_RETURN_STR(SMP_PAIR_INTERNAL_ERR) /**< Internal error */
    CASE_RETURN_STR(SMP_UNKNOWN_IO_CAP)    /**< unknown IO capability, unable to decide associatino model */
    CASE_RETURN_STR(SMP_INIT_FAIL)         /**< Initialization failed */
    CASE_RETURN_STR(SMP_CONFIRM_FAIL)      /**< Confirmation failed */
    CASE_RETURN_STR(SMP_BUSY)              /**< Busy */
    CASE_RETURN_STR(SMP_ENC_FAIL)          /**< Encryption failed */
    CASE_RETURN_STR(SMP_STARTED)           /**< Started */
    CASE_RETURN_STR(SMP_RSP_TIMEOUT)       /**< Response timeout */
    CASE_RETURN_STR(SMP_FAIL)              /**< Generic failure */
    CASE_RETURN_STR(SMP_CONN_TOUT)         /**< Connection timeout */
    CASE_DEFAULT_STR
    }

    return "UNKNOWN_STATUS";
}

/*******************************************************************************
* Function Name: get_bt_avrc_event_name
********************************************************************************
* Summary:
*  The function converts the wiced_bt_ble_advert_mode_t enum value to its
*  corresponding string literal. This will help the programmer to debug easily
*  with log traces without navigating through the source code.
*
* Parameters:
*   uint8_t event: avrc event 
*
* Return:
*  wiced_bt_avrc_evt_t
*
*******************************************************************************/
const char *get_bt_avrc_event_name(uint8_t event)
{
    switch ((int)event)
    {
    CASE_RETURN_STR(APP_AVRC_EVENT_DEVICE_CONNECTED)
    CASE_RETURN_STR(APP_AVRC_EVENT_REPEAT_SETTINGS_CHANGED)
    CASE_RETURN_STR(APP_AVRC_EVENT_SHUFFLE_SETTINGS_CHANGED)
    CASE_RETURN_STR(APP_AVRC_EVENT_DEVICE_DISCONNECTED)
    CASE_RETURN_STR(APP_AVRC_EVENT_PASSTHROUGH_RESPONSE)
    CASE_RETURN_STR(APP_AVRC_EVENT_PASSTHROUGH_CMD)
    CASE_RETURN_STR(APP_AVRC_EVENT_ABS_VOL_CHANGED)
    CASE_DEFAULT_STR
    }

    return "UNKNOWN_EVENT";
}

const char *get_bt_a2dp_src_event_name(uint8_t event)
{
    switch ((int)event)
    {
    CASE_RETURN_STR(WICED_BT_A2DP_SOURCE_CONNECT_EVT)
    CASE_RETURN_STR(WICED_BT_A2DP_SOURCE_START_IND_EVT)
    CASE_RETURN_STR(WICED_BT_A2DP_SOURCE_START_CFM_EVT)
    CASE_RETURN_STR(WICED_BT_A2DP_SOURCE_SUSPEND_EVT)
    CASE_RETURN_STR(WICED_BT_A2DP_SOURCE_CONFIGURE_EVT)
    CASE_RETURN_STR(WICED_BT_A2DP_SOURCE_WRITE_CFM_EVT)
    CASE_RETURN_STR(WICED_BT_A2DP_SOURCE_DELAY_RPT_EVT)
    CASE_DEFAULT_STR
    }

    return "UNKNOWN_EVENT";
}

const char *get_bt_le_gatt_event_name(uint8_t event)
{
    switch ((int)event)
    {
    CASE_RETURN_STR(GATT_CONNECTION_STATUS_EVT)
    CASE_RETURN_STR(GATT_OPERATION_CPLT_EVT)
    CASE_RETURN_STR(GATT_DISCOVERY_RESULT_EVT)
    CASE_RETURN_STR(GATT_DISCOVERY_CPLT_EVT)
    CASE_RETURN_STR(GATT_ATTRIBUTE_REQUEST_EVT)
    CASE_RETURN_STR(GATT_CONGESTION_EVT)
    CASE_RETURN_STR(GATT_GET_RESPONSE_BUFFER_EVT)
    CASE_RETURN_STR(GATT_APP_BUFFER_TRANSMITTED_EVT)
    CASE_DEFAULT_STR
    }

    return "UNKNOWN_EVENT";
}

const char *get_bt_gatt_optype_name(uint8_t op)
{
    switch ((int)op)
    {
    CASE_RETURN_STR(GATTC_OPTYPE_NONE)
    CASE_RETURN_STR(GATTC_OPTYPE_DISCOVERY)
    CASE_RETURN_STR(GATTC_OPTYPE_READ_HANDLE)
    CASE_RETURN_STR(GATTC_OPTYPE_READ_BY_TYPE)
    CASE_RETURN_STR(GATTC_OPTYPE_READ_MULTIPLE)
    CASE_RETURN_STR(GATTC_OPTYPE_WRITE_WITH_RSP)
    CASE_RETURN_STR(GATTC_OPTYPE_WRITE_NO_RSP)
    CASE_RETURN_STR(GATTC_OPTYPE_PREPARE_WRITE)
    CASE_RETURN_STR(GATTC_OPTYPE_EXECUTE_WRITE)
    CASE_RETURN_STR(GATTC_OPTYPE_CONFIG_MTU)
    CASE_RETURN_STR(GATTC_OPTYPE_NOTIFICATION)
    CASE_RETURN_STR(GATTC_OPTYPE_INDICATION)
    CASE_DEFAULT_STR
    }

    return "UNKNOWN_EVENT";
}

/*
 *
 */
void configure_bluetooth()
{
    wiced_bt_device_address_t         local_bda;
    uint8_t *p = bt_device_address;

    wiced_bt_dev_read_local_addr(local_bda);
    TRACE_LOG("Local bdaddr:");
    if ((p[0] | p[1] | p[2] | p[3] | p[4] | p[5]) != 0)
    {
        wiced_bt_set_local_bdaddr(p, BLE_ADDR_PUBLIC);
        print_bd_address(p);
    } else {
        print_bd_address(local_bda);
    }

    /* Disable pairing bydefault */
    wiced_bt_set_pairable_mode(FALSE, FALSE );
}

void device_name_add_bdaddr(uint8_t **dev_name_bdaddr)
{
    char tmp[4] = {0};
    uint8_t local_dev_length = strlen( (char *)broadcast_sink_cfg_settings.device_name ) + sizeof(bt_device_address) + 1;
    uint8_t *local_dev_name = (char*)wiced_memory_allocate(local_dev_length); //don't release, le need use

    memcpy(local_dev_name, broadcast_sink_cfg_settings.device_name, strlen(broadcast_sink_cfg_settings.device_name) );
    strncat(local_dev_name, "-", local_dev_length);

    for(int i = 0; i < sizeof(bt_device_address); i++)
    {
        snprintf(tmp, sizeof(tmp), "%02X", bt_device_address[i]);
        strncat(local_dev_name, tmp, strlen(tmp));
    }
    local_dev_name[local_dev_length-1] = 0;
    TRACE_LOG("local_dev_name:%s, strlen:%d\n", local_dev_name, strlen(local_dev_name));
    *dev_name_bdaddr = local_dev_name;
}
/* [] END OF FILE */
