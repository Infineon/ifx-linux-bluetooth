/******************************************************************************
 * File Name: wifi_onboarding.c
 *
 * Description: This is the source file for Linux wifi_onboarding CE.
 *
 * Related Document: See README.md
 * 
 ******************************************************************************
* $ Copyright 2022-YEAR Cypress Semiconductor $
/*******************************************************************************
*      INCLUDES
*******************************************************************************/
#include "wiced_bt_stack.h"
#include <string.h>
#include <stdlib.h>
#include "wiced_memory.h"
#include "stdio.h"
#include "app_bt_config/cycfg_gatt_db.h"
#include "app_bt_config/cycfg_bt_settings.h"
#include "app_bt_config/cycfg_gap.h"
#include "wiced_bt_dev.h"
#include "app_bt_utils.h"
#include "wifi_onboarding.h"
#include "wiced_hal_nvram.h"
#include "wiced_bt_trace.h"

#include <pthread.h>
#include <unistd.h>
#include "log.h"
#include "stdbool.h"
#include <ctype.h>

/*******************************************************************************
*       MACROS
*******************************************************************************/
#ifdef TAG
#undef TAG
#endif
#define TAG "[WiFi]"

#define BT_STACK_HEAP_SIZE              (0xF000)
#define BD_ADDR_LENGTH                  (6)

/* Macros defining the commands for WiFi control point characteristic */
#define WIFI_CONTROL_DISCONNECT         (0u)
#define WIFI_CONTROL_CONNECT            (1u)
#define WIFI_CONTROL_SCAN               (2u)
#define WIFI_CONTROL_CONNECT_SCANNED_INDEX (3u)

/* Macro defining types in TLV format */
#define TYPE_SSID           (1u)
#define TYPE_PASSWORD       (2u)
#define TYPE_SECURITY       (2u)
#define SECURITY_TAG_SIZE   (4u)

#define NOTIFY_BUFFER_LENGTH       (2048u)
#define SCAN_BUFFER_LENGTH         (1035u)
#define LINUX_COMMAND_LENGTH       (100u)
#define IPV4_LENGTH                (28u)
#define IPV4_BUFFER_LENGTH         (100u)
#define SEC_DEFAULT_VALUE          (-1)
#define WPA2_VALUE                 (0x00400000u)
#define DEFAULT_CONNECT_ID         (0xFFFFu)
#define SCAN_RESULTS_PACKET_LENGTH (256u)
#define SEC_VALUE_LENGTH           (4u)

#define WIFI_STATE_DISCONNECTED (0u)
#define WIFI_STATE_CONNECTED    (1u)
#define WIFI_STATE_CONNECTING   (2u)
#define GET_STATUS_PERIOD       (3u)
#define TIMEOUT_PERIODS         (15u)

/*******************************************************************************
*       STRUCTURES AND ENUMERATIONS
*******************************************************************************/

/* This enumeration combines the advertising, connection states from two
 * different callbacks to maintain the status in a single state variable */
typedef enum
{
    APP_BT_ADV_OFF_CONN_OFF,
    APP_BT_ADV_ON_CONN_OFF,
    APP_BT_ADV_OFF_CONN_ON
} tAppBtAdvConnMode;

#pragma pack(1)
/* Host information saved in  NVRAM */
typedef WICED_BT_STRUCT_PACKED
{
    wiced_bt_device_address_t  bdaddr;                    /* BD address of the bonded host */
} host_info_t;
#pragma pack()

typedef struct
{
    wiced_bt_device_address_t   remote_addr;              // remote peer device address
    uint16_t                    conn_id;                  // connection ID referenced by the stack
    uint16_t                    peer_mtu;                 // peer MTU
    tAppBtAdvConnMode           app_bt_adv_conn_state;
} iwos_state_t;

/*******************************************************************************
*       VARIABLE DEFINITIONS
*******************************************************************************/

wiced_bt_heap_t *p_default_heap   = NULL;

static tAppBtAdvConnMode app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;

host_info_t   iwos_hostinfo;

iwos_state_t iwos_state;

static bool has_subscribed = false;
static char buf_notify[NOTIFY_BUFFER_LENGTH] = {0};
static char* cur_buf = buf_notify;
static uint8_t max_scan_idx = 0;
static uint8_t wifi_connected_state = 0;
static uint8_t last_wifi_connected_state = WIFI_STATE_DISCONNECTED;

/*******************************************************************************
*       FUNCTION DECLARATIONS
*******************************************************************************/
static void                     le_app_init(void);
static void*                    le_app_alloc_buffer(int len);
static void                     le_app_free_buffer(uint8_t *p_event_data);
typedef void                    (*pfn_free_buffer_t)(uint8_t *);
static gatt_db_lookup_table_t   *le_app_find_by_handle(uint16_t handle);

/* GATT Event Callback Functions */
static wiced_bt_gatt_status_t   le_app_write_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_write_req_t *p_write_req, uint16_t len_req);
static wiced_bt_gatt_status_t   le_app_read_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_t *p_read_req, uint16_t len_req);
static wiced_bt_gatt_status_t   le_app_connect_handler(wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t   le_app_server_handler(wiced_bt_gatt_attribute_request_t *p_attr_req);
static wiced_bt_gatt_status_t   le_app_gatt_event_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);

/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t    le_app_bt_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static wiced_bt_gatt_status_t   le_app_bt_gatt_req_read_by_type_handler (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested);

static void                     iwos_smp_bond_result( uint8_t wiced_result );
static void                     iwos_load_keys_for_address_resolution( void );

/*******************************************************************************
*       FUNCTION DEFINITION
*******************************************************************************/

/*******************************************************************************
* Function Name: application_start
********************************************************************************
* Summary:
*  Set device configuration and start BT stack initialization. The actual
*  application initialization will happen when stack reports that BT device
*  is ready.
*
* Parameters: NONE
*
* Return: NONE
*
*******************************************************************************/
void application_start(void)
{
    wiced_result_t wiced_result;

    TRACE_LOG("************* WiFi_Onboarding Application Start ************************\n");

    /* Register call back and configuration with stack */
    wiced_result = wiced_bt_stack_init (le_app_bt_management_callback, &wiced_bt_cfg_settings);

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
       TRACE_ERR("Bluetooth Stack Initialization failed!! \n");
       exit(EXIT_FAILURE);
    }
}

/*******************************************************************************
* Function Name: security_parser
********************************************************************************
* Summary:
*    Parsing tags form scan results and showing if the SSID has WPA2 tag
*
* Return:
*    WPA2_VALUE         : If it has WPA2
*    SEC_DEFAULT_VALUE  : If it has no WPA2
*
*******************************************************************************/
static int security_parser(char* str_sec)
{
  int length = strlen(str_sec);
  if(length < SEC_VALUE_LENGTH)
  {
    return SEC_DEFAULT_VALUE;
  }
  for(int i = 0; i < length - (SEC_VALUE_LENGTH - 1); i++)
  {
    if(str_sec[i] == 'W' && str_sec[i+1] == 'P' && str_sec[i+2] == 'A' && str_sec[i+3] == '2')
    {
      return WPA2_VALUE;
    }
  }

  return SEC_DEFAULT_VALUE;
}

/*******************************************************************************
* Function Name: is_process_ok
********************************************************************************
* Summary:
*   Print result of executed process, and check if it is exactly "OK"
*
* Return:
*   True  : If result is exactly "OK"
*   False : If result is not "OK"
*
*******************************************************************************/
static bool is_process_ok(char* buf, int buf_size, FILE* fp)
{
    while (fgets(buf, buf_size, fp) != NULL)
    {
        TRACE_LOG("%s", buf);
    }
    return buf[0] == 'O' && buf[1] == 'K';
}

/*******************************************************************************
* Function Name: sync_wifi_state
********************************************************************************
* Summary:
*   Sync Wi-Fi state and last Wi-Fi state
*
* Return:
*   None
*
*******************************************************************************/
static void sync_wifi_state(uint8_t status)
{
    last_wifi_connected_state = wifi_connected_state = status;
}

/*******************************************************************************
* Thread Name: wifi_tracker
********************************************************************************
* Summary:
*   It sends notifications for updating Wi-Fi status and the scan results
*
* Parameters:
*   arg     : Parameters passed to this thread
*
* Return:
*   None
*
*******************************************************************************/
void *wifi_tracker(void *arg)
{
    FILE *fp;
    uint8_t timeout_counter = 0;
    for(;;)
    {
        uint16_t i = 0, sent_status = 0, packet_idx = 0;
        uint8_t n_send = 0;
        char buffer_notification[30] = {0};
        char ipv4[IPV4_BUFFER_LENGTH] = {0};
        char* have_ip = ipv4;
        char* str_no_ip = "None";

        /* Timeout counter */
        if(timeout_counter >= TIMEOUT_PERIODS)
        {
            timeout_counter = 0;
            wifi_connected_state = WIFI_STATE_DISCONNECTED;
            TRACE_LOG("Wi-Fi: connecting timeout, stop connecting. Please check your Wi-Fi information and send connect command again.\n");
            /* Stop connecting to Wi-Fi access point */
            system("bash wlan_remove_network.sh");
        }
        else if(wifi_connected_state == WIFI_STATE_CONNECTING)
        {
            timeout_counter++;
            TRACE_LOG("Wi-Fi: Connecting: %d..., please wait.\r\n", timeout_counter);
        }

        /* Get Wi-Fi status */
        fp = popen("ifconfig |grep -C2 wlan0|grep \"inet \"", "r");
        if (fp == NULL)
        {
            TRACE_ERR("Failed to get Wi-Fi status.\n" );
            break;
        }

        while (fgets(ipv4, sizeof(ipv4), fp) != NULL) i++;


        if(i)
        {
            /* Get ipv4 address */
            for(int j = 0; j < IPV4_BUFFER_LENGTH; j++)
            {
                if(have_ip == ipv4)
                {
                    if(isdigit(ipv4[j]))
                    {
                        have_ip = ipv4 + j;
                    }
                }
                else
                {
                    if(!isdigit(ipv4[j]) && ipv4[j] != '.')
                    {
                        ipv4[j] = '\0';
                        break;
                    }
                }
            }
            if(strncmp("169.254", have_ip, 7) != 0)
            {
                wifi_connected_state = WIFI_STATE_CONNECTED;
            }
            else
            {
                wifi_connected_state = WIFI_STATE_DISCONNECTED;
            }
            timeout_counter = 0;
        }
        else
        {
            have_ip = str_no_ip;
            if(last_wifi_connected_state == WIFI_STATE_CONNECTED)
            {
                wifi_connected_state = WIFI_STATE_CONNECTING;
            }
            else
            {
                if(wifi_connected_state != WIFI_STATE_CONNECTING)
                {
                    timeout_counter = 0;
                    wifi_connected_state = WIFI_STATE_DISCONNECTED;
                }
            }
        }

        /* LE connected */
        if(iwos_state.conn_id != 0)
        {
            /* Display */
            switch (wifi_connected_state)
            {
            case WIFI_STATE_CONNECTED:
                TRACE_LOG("Wi-Fi: Connected, ipv4: %s\r\n", have_ip);
                break;

            case WIFI_STATE_DISCONNECTED:
                TRACE_LOG("Wi-Fi: Disconnected, ipv4: %s\r\n", have_ip);
                break;

            default:
                break;
            }

            /* Send notification */
            if(has_subscribed)
            {
                if(wifi_connected_state == WIFI_STATE_CONNECTED)
                {
                    snprintf(buffer_notification, 30, "%s:%s", "Connected", have_ip);
                    wiced_bt_gatt_server_send_notification( iwos_state.conn_id, HDLC_CUSTOM_SERVICE_WIFI_NETWORKS_VALUE, strlen(buffer_notification), buffer_notification, NULL);
                }
                else if(wifi_connected_state == WIFI_STATE_CONNECTING)
                {
                    snprintf(buffer_notification, 30, "%s: %d...", "Connecting", timeout_counter);
                    wiced_bt_gatt_server_send_notification( iwos_state.conn_id, HDLC_CUSTOM_SERVICE_WIFI_NETWORKS_VALUE, strlen(buffer_notification), buffer_notification, NULL);
                }
                else
                {
                    wiced_bt_gatt_server_send_notification( iwos_state.conn_id, HDLC_CUSTOM_SERVICE_WIFI_NETWORKS_VALUE, strlen("Disconnected"), "Disconnected", NULL);
                }
                n_send = (cur_buf - buf_notify) / SCAN_RESULTS_PACKET_LENGTH;
                for(; packet_idx < n_send; packet_idx++)
                {
                    sent_status = wiced_bt_gatt_server_send_notification(iwos_state.conn_id, \
                    HDLC_CUSTOM_SERVICE_WIFI_NETWORKS_VALUE, SCAN_RESULTS_PACKET_LENGTH, buf_notify + (packet_idx * SCAN_RESULTS_PACKET_LENGTH), NULL);
                    TRACE_LOG("packet_idx: %d, sent_status: %d\n", packet_idx, sent_status);
                }
                if((cur_buf - buf_notify) % SCAN_RESULTS_PACKET_LENGTH)
                {
                    sent_status = wiced_bt_gatt_server_send_notification(iwos_state.conn_id, \
                    HDLC_CUSTOM_SERVICE_WIFI_NETWORKS_VALUE, (cur_buf - buf_notify) - (packet_idx * SCAN_RESULTS_PACKET_LENGTH), \
                    buf_notify + (packet_idx * SCAN_RESULTS_PACKET_LENGTH), NULL);
                    TRACE_LOG("packet_idx: %d, sent_status: %d\n", packet_idx, sent_status);
                }
                cur_buf = buf_notify;
            }
        }
        pclose(fp);
        last_wifi_connected_state = wifi_connected_state;
        sleep(GET_STATUS_PERIOD);
    }
    return NULL;
}

/*******************************************************************************
* Function Name: scan_results_finder
********************************************************************************
* Summary:
*   Return SSID position of corresponding scanned index
*
* Parameters:
*   idx     : Scanned index from scan results
*
* Return:
*   SSID position
*
*******************************************************************************/
static char* scan_results_finder(uint8_t idx)
{
    if(idx > max_scan_idx)
    {
        return NULL;
    }
    char* position = buf_notify;
    for(;--idx;)
    {
        /* shift to next ssid start position (1(type) + 1(length)) * 2 + 4(tag) */
        position += position[1] + 8;
    }
    return position;
}

/*******************************************************************************
* Function Name: le_app_bt_management_callback
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
wiced_result_t le_app_bt_management_callback(wiced_bt_management_evt_t event,
                                          wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t wiced_result = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_dev_ble_pairing_info_t     *p_info;
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;
    wiced_bt_device_link_keys_t         *p_keys;
    wiced_bt_local_identity_keys_t      *p_ikeys;
    const uint8_t *link_key;

    TRACE_LOG( "le_app_bt_management_callback: event 0x%x \n", event );
    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Bluetooth Controller and Host Stack Enabled */
            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
                wiced_bt_dev_read_local_addr(bda);
                TRACE_LOG("Local Bluetooth Address: ");
                print_bd_address(bda);

                /* Perform application-specific initialization */
                le_app_init();
            }
            else
            {
                TRACE_ERR( "Bluetooth Enabling Failed \n" );
                wiced_result = WICED_BT_ERROR;
            }
            break;

        case BTM_DISABLED_EVT:
            TRACE_LOG( "Bluetooth Disabled \n" );
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            TRACE_LOG( "***********************************************************\r\n");
            TRACE_LOG( "Passkey Notification\r\n");
            TRACE_LOG( "PassKey: %u \r\n",
            p_event_data->user_passkey_notification.passkey );
            TRACE_LOG( "***********************************************************\r\n");
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            TRACE_LOG( "BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT \n" );
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap      = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data          = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req          = BTM_LE_AUTH_REQ_SC_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size      = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys         = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys         = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            TRACE_LOG( "BTM_PAIRING_COMPLETE_EVT \n" );
            p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
            TRACE_LOG( "Pairing Complete: %d ", p_info->reason);
            iwos_smp_bond_result( p_info->reason );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            TRACE_LOG("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT\n");

            /* save keys to NVRAM */
            p_keys = &p_event_data->paired_device_link_keys_update;
            wiced_hal_write_nvram(IWOS_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t),
                                    (uint8_t *)p_keys,&wiced_result);
            TRACE_LOG("keys save to NVRAM wiced_result: %d \n", wiced_result);

            /* read existing key from the NVRAM  */
            p_keys = &p_event_data->paired_device_link_keys_request;
            wiced_hal_read_nvram(IWOS_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t),
                                    (uint8_t *)p_keys, &wiced_result);
            TRACE_LOG("keys read from NVRAM wiced_result: %d \n", wiced_result);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read existing key from the NVRAM  */
            p_keys = &p_event_data->paired_device_link_keys_request;
            wiced_hal_read_nvram(IWOS_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t),
                                    (uint8_t *)p_keys, &wiced_result);
            TRACE_LOG("keys read from NVRAM wiced_result: %d \n", wiced_result);
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Local identity Keys Update */
            /* save keys to NVRAM */
            p_ikeys = &p_event_data->local_identity_keys_update;
            wiced_hal_write_nvram(IWOS_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t),
                    (uint8_t *)p_ikeys, &wiced_result);
            TRACE_LOG("local keys save to NVRAM wiced_result: %d \n", wiced_result);
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* Local identity Keys Request */
            /* read keys from NVRAM */
            p_ikeys = &p_event_data->local_identity_keys_request;
            wiced_hal_read_nvram(IWOS_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t),
                    (uint8_t *)p_ikeys, &wiced_result);
            TRACE_LOG("local keys read from NVRAM wiced_result: %d \n", wiced_result);
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            if (WICED_SUCCESS == p_event_data->encryption_status.result)
            {
                TRACE_LOG("Encryption Status Event: SUCCESS\n");
                wiced_result = WICED_SUCCESS;
            }
            else /* Encryption Failed */
            {
                TRACE_ERR("Encryption Status Event: FAILED\n");
                wiced_result = WICED_BT_ERROR;
            }
            break;

        case BTM_SECURITY_REQUEST_EVT:
            TRACE_LOG( "BTM_SECURITY_REQUEST_EVT \n" );
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                                        WICED_BT_SUCCESS);
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

            /* Advertisement State Changed */
            p_adv_mode = &p_event_data->ble_advert_state_changed;
            TRACE_LOG("Advertisement State Change: %s\n", get_bt_advert_mode_name(*p_adv_mode));

            if (BTM_BLE_ADVERT_OFF == *p_adv_mode)
            {
                /* Advertisement Stopped */
                TRACE_LOG("Advertisement stopped\n");

                /* Check connection status after advertisement stops */
                if(0 == iwos_state.conn_id)
                {
                    iwos_state.app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;
                    wiced_result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

                    /* Failed to start advertisement. Stop program execution */
                    if (WICED_BT_SUCCESS != wiced_result)
                    {
                        TRACE_ERR("failed to start advertisement! \n");
                        exit(EXIT_FAILURE);
                    }
                }
                else
                {
                    iwos_state.app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
                }
            }
            else
            {
                /* Advertisement Started */
                TRACE_LOG("Advertisement started\n");
                iwos_state.app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;
            }
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            TRACE_LOG("Connection parameter update status:%d, Connection Interval: %d, \
                                        Connection Latency: %d, Connection Timeout: %d\n",
                                        p_event_data->ble_connection_param_update.status,
                                        p_event_data->ble_connection_param_update.conn_interval,
                                        p_event_data->ble_connection_param_update.conn_latency,
                                        p_event_data->ble_connection_param_update.supervision_timeout);
            break;

        default:
            TRACE_LOG("Unhandled Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));
            break;
    }

    return wiced_result;
}

/*******************************************************************************
* Function Name: le_app_init
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
static void le_app_init(void)
{
    wiced_result_t wiced_result = WICED_BT_SUCCESS;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    /* Initialize wlan0 */
    system("bash wlan_init.sh");
    sync_wifi_state(WIFI_STATE_DISCONNECTED);

    TRACE_LOG("*********************************************************\n");
    TRACE_LOG("*********************************************************\n");
    TRACE_LOG("***                                                   ***\n");
    TRACE_LOG("***   Welcome Infinon Wi-Fi onboarding Code Example   ***\n");
    TRACE_LOG("***                                                   ***\n");
    TRACE_LOG("*********************************************************\n");
    TRACE_LOG("*********************************************************\n");
    TRACE_LOG("\n");
    TRACE_LOG("=>   Discover device with \"bleProv\" name\n");
    TRACE_LOG("=>   Discover device with \"bleProv\" name\n");
    TRACE_LOG("=>   Discover device with \"bleProv\" name\n");

    /* Set Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, cy_bt_adv_packet_data);

    /* Register with BT stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(le_app_gatt_event_callback);
    TRACE_LOG("GATT event Handler registration status: %s \n",get_bt_gatt_status_name(gatt_status));

    /* Initialize GATT Database */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    TRACE_LOG("GATT database initialization status: %s \n",get_bt_gatt_status_name(gatt_status));

    
    /* Load previous paired keys for address resolution */
    iwos_load_keys_for_address_resolution();

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(TRUE, FALSE);

    /* Start Undirected LE Advertisements on device startup.
     * The corresponding parameters are contained in 'app_bt_cfg.c' */
    wiced_result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

    /* Failed to start advertisement. Stop program execution */
    if (WICED_BT_SUCCESS != wiced_result)
    {
        TRACE_ERR("failed to start advertisement! \n");
        exit(EXIT_FAILURE);
    }
}

/*******************************************************************************
* Function Name: le_app_gatt_event_callback
********************************************************************************
* Summary:
*   This function handles GATT events from the BT stack.
*
* Parameters:
*   wiced_bt_gatt_evt_t event                : LE GATT event code of one
*                                              byte length
*   wiced_bt_gatt_event_data_t *p_event_data : Pointer to LE GATT event
*                                              structures
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
*******************************************************************************/
static wiced_bt_gatt_status_t le_app_gatt_event_callback(wiced_bt_gatt_evt_t event,
                                                         wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;

    /* Call the appropriate callback function based on the GATT event type,
     * and pass the relevant event
     * parameters to the callback function */
    switch ( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
        gatt_status = le_app_connect_handler( &p_event_data->connection_status );
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        gatt_status = le_app_server_handler( &p_event_data->attribute_request );
        break;
    /* GATT buffer request, typically sized to max of bearer mtu - 1 */
    case GATT_GET_RESPONSE_BUFFER_EVT:
        p_event_data->buffer_request.buffer.p_app_rsp_buffer =
        le_app_alloc_buffer(p_event_data->buffer_request.len_requested);
        p_event_data->buffer_request.buffer.p_app_ctxt = (void *)le_app_free_buffer;
        gatt_status = WICED_BT_GATT_SUCCESS;
        break;
    /* GATT buffer transmitted event, check 
     * ref wiced_bt_gatt_buffer_transmitted_t*/
    case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point 
             * to a function to free it. */
            if (pfn_free)
                pfn_free(p_event_data->buffer_xmitted.p_app_data);

            gatt_status = WICED_BT_GATT_SUCCESS;
        }
        break;

    default:
        gatt_status = WICED_BT_GATT_SUCCESS;
        break;
    }

    return gatt_status;
}

/*******************************************************************************
* Function Name: le_app_write_handler
********************************************************************************
* Summary:
*   This function handles Write Requests received from the client device
*
* Parameters:
*  @param conn_id       Connection ID
*  @param opcode        LE GATT request type opcode
*  @param p_write_req   Pointer to LE GATT write request
*  @param len_req       length of data to be written
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
*******************************************************************************/
static wiced_bt_gatt_status_t le_app_write_handler(uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_write_req_t *p_write_req,
                                                    uint16_t len_req)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    uint8_t *p_attr = p_write_req->p_val, *buff = app_custom_service_wifi_password;
    gatt_db_lookup_table_t *puAttribute;
    char * scan_ssid = NULL;

    char linux_cmd[LINUX_COMMAND_LENGTH] = {0};
    FILE *fp;
    char buf[SCAN_BUFFER_LENGTH] = {0}; 

    TRACE_LOG("GATT write handler: handle:0x%X len:%d, opcode:0x%X\n",
           p_write_req->handle, p_write_req->val_len, opcode);

    /* Get the right address for the handle in Gatt DB */
    if (NULL == (puAttribute = le_app_find_by_handle(p_write_req->handle)))
    {
        TRACE_ERR("Write Handle attr not found. Handle:0x%X\n", p_write_req->handle);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    switch (p_write_req->handle)
    {
    /* Write request for the WiFi SSID characteristic. Copy the incoming data
    to the WIFI SSID variable */
    case HDLC_CUSTOM_SERVICE_WIFI_SSID_VALUE:

        /* First copy the value to the GATT DB variable and then to the global
         * variable to be stored in NV memory
         */
        if(p_write_req->val_len >= puAttribute->max_len)
        {
            TRACE_WARNING("SSID data length is too long, max_len = %d, input_len = %d\n", puAttribute->max_len - 1, p_write_req->val_len);
            gatt_status = WICED_BT_GATT_ERROR;
            break;
        }

        memset(app_custom_service_wifi_ssid, 0,
               strlen((char *)app_custom_service_wifi_ssid));
        memcpy(app_custom_service_wifi_ssid, p_attr, p_write_req->val_len);
        puAttribute->cur_len = p_write_req->val_len;
        TRACE_LOG("Wi-Fi SSID: %s\n", app_custom_service_wifi_ssid);
        break;

    /* Write request for the WiFi password characteristic. Accept the password.
    Copy the incoming data to the WIFI Password variable */
    case HDLC_CUSTOM_SERVICE_WIFI_PASSWORD_VALUE:

        /* First copy the value to the GATT DB variable and then to the global
         * variable to be stored in NV memory
         */
        if(p_write_req->val_len >= puAttribute->max_len)
        {
            TRACE_WARNING("PASSWARD data length is too long, max_len = %d, input_len = %d\n", puAttribute->max_len - 1, p_write_req->val_len);
            gatt_status = WICED_BT_GATT_ERROR;
            break;
        }

        memset(app_custom_service_wifi_password, 0,
                strlen((char *)app_custom_service_wifi_password));
        memcpy(app_custom_service_wifi_password, p_attr, p_write_req->val_len);
        puAttribute->cur_len = p_write_req->val_len;
        TRACE_LOG("Wi-Fi Password: %s\n", app_custom_service_wifi_password);
        break;

    /* Write request for the WiFi SSID and password characteristic. Accept the
    values and Copy the incoming data to the WIFI SSID Password variable */
    case HDLC_CUSTOM_SERVICE_WIFI_SSID_PASSWORD_VALUE:
        /* TLV format: Type Length Value
         * 0x01 0x03 0x45 0x46 0x47 0x02 0x04 0x50 0x51 0x52 0x53
         * Type:    0x01 (SSID)
         * Length:  0x03
         * Value:   0x45 0x46 0x47
         * Type:    0x02 (PASSWORD)
         * Length:  0x04
         * Value:   0x50 0x51 0x52 0x53  
         */

        /* Check data length */
        if(p_attr[1] + 2 > p_write_req->val_len)
        {
            TRACE_WARNING("TLV format input type error. First type length: %d, is out of range.\n", p_attr[1]);
            TRACE_WARNING("Kindly check your data again\n");
            gatt_status = WICED_BT_GATT_ERROR;
            break;
        }
        else if (p_write_req->val_len != p_attr[1] + p_attr[p_attr[1] + 3] + 4){
            TRACE_WARNING("TLV format input type error. First type length: %d,  Second type length: %d\n", p_attr[1], p_attr[p_attr[1] + 3]);
            TRACE_WARNING("Kindly check your data again\n");
            gatt_status = WICED_BT_GATT_ERROR;
            break;
        }

        if((p_attr[0] != TYPE_SSID && p_attr[0] != TYPE_PASSWORD) || (p_attr[p_attr[1] + 2] != TYPE_SSID && p_attr[p_attr[1] + 2] != TYPE_PASSWORD))
        {
            TRACE_WARNING("TLV format input type error. First type: %d,  Second type: %d\n", p_attr[0], p_attr[p_attr[1] + 2]);
            gatt_status = WICED_BT_GATT_ERROR;
            break;
        }
        else
        {
            TRACE_LOG("SSID len: %d, PASSWORD len: %d\n", p_attr[1], p_attr[p_attr[1] + 3]);
        }
        TRACE_LOG("Previous Wi-Fi SSID: %s\n", app_custom_service_wifi_ssid);
        TRACE_LOG("Previous Wi-Fi Password: %s\n", app_custom_service_wifi_password);
        puAttribute = le_app_find_by_handle(
            p_attr[0] == TYPE_SSID?HDLC_CUSTOM_SERVICE_WIFI_SSID_VALUE:HDLC_CUSTOM_SERVICE_WIFI_PASSWORD_VALUE);
        if(p_attr[1] >= puAttribute->max_len)
        {
            TRACE_WARNING("%s data length is too long, max_len = %d, input_len = %d\n",
                p_attr[0] == TYPE_SSID ? "SSID" : "PASSWARD", puAttribute->max_len - 1, p_attr[1]);
            gatt_status = WICED_BT_GATT_ERROR;
            break;
        }
        memset(puAttribute->p_data, 0, puAttribute->max_len);
        memcpy(puAttribute->p_data, p_attr + 2, p_attr[1]);
        puAttribute->p_data[p_attr[1] + 1] = '\0';
        puAttribute->cur_len = p_attr[1];

        puAttribute = le_app_find_by_handle(
            p_attr[p_attr[1] + 2] == TYPE_SSID?HDLC_CUSTOM_SERVICE_WIFI_SSID_VALUE:HDLC_CUSTOM_SERVICE_WIFI_PASSWORD_VALUE);
        if(p_attr[p_attr[1] + 3] >= puAttribute->max_len)
        {
            TRACE_WARNING("%s data length is too long, max_len = %d, input_len = %d\n",
                p_attr[p_attr[1] + 2] == TYPE_SSID ? "SSID" : "PASSWARD", puAttribute->max_len - 1, p_attr[p_attr[1] + 3]);
            gatt_status = WICED_BT_GATT_ERROR;
            break;
        }
        memset(puAttribute->p_data, 0, puAttribute->max_len);
        memcpy(puAttribute->p_data, p_attr + 4 + p_attr[1], p_attr[p_attr[1] + 3]);
        puAttribute->p_data[p_attr[p_attr[1] + 3] + 1] = '\0';
        puAttribute->cur_len = p_attr[p_attr[1] + 3];
        TRACE_LOG("Current Wi-Fi SSID: %s\n", app_custom_service_wifi_ssid);
        TRACE_LOG("Current Wi-Fi Password: %s\n", app_custom_service_wifi_password);
        break;


    /* Handle the CCCD values for WIFI NETWORKS characteristic */
    case HDLD_CUSTOM_SERVICE_WIFI_NETWORKS_CLIENT_CHAR_CONFIG:
        app_custom_service_wifi_networks_client_char_config[0] = p_attr[0];
        app_custom_service_wifi_networks_client_char_config[1] = p_attr[1];
        has_subscribed = p_attr[0];
        break;

    /* Write request for the control characteristic. Copy the incoming data
    to the WIFI control variable. Based on the value, start connect, disconnect
     or scan procedure */
    case HDLC_CUSTOM_SERVICE_WIFI_CONTROL_VALUE:
        app_custom_service_wifi_control[0] = p_attr[0];
        puAttribute->cur_len = p_write_req->val_len;
        if(p_write_req->val_len > 2)
        {
            TRACE_ERR("Invalid command, input length: %d\n", p_write_req->val_len);
            gatt_status = WICED_BT_GATT_ERROR;
            break;
        }

        /* Connect command */
        if (WIFI_CONTROL_CONNECT == app_custom_service_wifi_control[0])
        {
            TRACE_LOG("WIFI_CONTROL_CONNECT\n");
            puAttribute = le_app_find_by_handle(HDLC_CUSTOM_SERVICE_WIFI_PASSWORD_VALUE);
            if(puAttribute->cur_len == 0)
            {
                /* Configuration for OPEN Wi-Fi access point which has no need of password */
                strcat(linux_cmd, "bash wlan_config_OPEN.sh \"");
            }
            else
            {
                /* Configuration for WPA2 Wi-Fi access point */
                strcat(linux_cmd, "bash wlan_config.sh \"");
            }
            strcat(linux_cmd, app_custom_service_wifi_ssid);
            strcat(linux_cmd, "\" \"");
            strcat(linux_cmd, app_custom_service_wifi_password);
            strcat(linux_cmd, "\"");
            TRACE_LOG("Wi-Fi connect command: %s\n", linux_cmd);
            fp = popen(linux_cmd, "r");
            if (fp == NULL)
            {
                TRACE_ERR("Failed to run command connect\n" );
                gatt_status = WICED_BT_GATT_ERROR;
                break;
            }
            if(!is_process_ok(buf, SCAN_BUFFER_LENGTH, fp))
            {
                TRACE_ERR("Wi-Fi connect command result is not \"OK\", kindly check your Wi-Fi information, especially the length of PASSWORD.\n");
                gatt_status = WICED_BT_GATT_ERROR;
                sync_wifi_state(WIFI_STATE_DISCONNECTED);
                pclose(fp);
                break;
            }
            wifi_connected_state = WIFI_STATE_CONNECTING;
            pclose(fp);
        }
        /* Disconnect command */
        else if(WIFI_CONTROL_DISCONNECT == app_custom_service_wifi_control[0])
        {
            /* To disconnect from Wi-Fi access point via Wi-Fi interface */
            TRACE_LOG("WIFI_CONTROL_DISCONNECT\n");
            system("bash wlan_remove_network.sh");
            sync_wifi_state(WIFI_STATE_DISCONNECTED);
        }
        /* Scan command */
        else if(WIFI_CONTROL_SCAN == app_custom_service_wifi_control[0])
        {
            /* Calling Wi-Fi interface to do Wi-Fi scan */
            cur_buf = buf_notify;
            TRACE_LOG("WIFI_CONTROL_SCAN\n");
            fp = popen("bash wlan_scan.sh", "r");
            if (fp == NULL)
            {
                TRACE_ERR("Failed to run command scan\n" );
                gatt_status = WICED_BT_GATT_ERROR;
                break;
            }
            if(!is_process_ok(buf, SCAN_BUFFER_LENGTH, fp))
            {
                TRACE_ERR("Wi-Fi Scan status is not OK\n");
                gatt_status = WICED_BT_GATT_ERROR;
                pclose(fp);
                break;
            }
            pclose(fp);

            /* Get scan results cur_buf */
            fp = popen("bash wlan_scan_results.sh", "r");
            if (fp == NULL) {
                TRACE_ERR("Failed to run command scan_results\n");
                gatt_status = WICED_BT_GATT_ERROR;
                break;
            }
            int wifi_security_tags = 0;
            int i = 0;
            while (fgets(buf, sizeof(buf), fp) != NULL) 
            {
                char* ptr_blank = buf;
                while(*ptr_blank && *ptr_blank != ' ') ptr_blank++;
                *ptr_blank = '\0';

                if(++i > 1 && cur_buf - buf_notify <= NOTIFY_BUFFER_LENGTH && strlen(buf) != 0)
                {
                    TRACE_LOG("Scan index: %2x:%80s%32s", i - 1, buf, ptr_blank + 1);
                    max_scan_idx = i;

                    *cur_buf++ = TYPE_SSID; 
                    *cur_buf++ = strlen(ptr_blank + 1) - 1;
                    memcpy (cur_buf, ptr_blank + 1, strlen(ptr_blank + 1) - 1);
                    cur_buf += strlen(ptr_blank + 1) - 1;

                    *cur_buf++ = TYPE_SECURITY;
                    *cur_buf++ = SECURITY_TAG_SIZE;
                    wifi_security_tags = security_parser(buf);
                    memcpy (cur_buf, &wifi_security_tags, SECURITY_TAG_SIZE);
                    cur_buf += SECURITY_TAG_SIZE;
                }
            }

            pclose(fp);
        }
        /* Connect to scanned index with saved PASSWORD */
        else if(WIFI_CONTROL_CONNECT_SCANNED_INDEX == app_custom_service_wifi_control[0])
        {
            scan_ssid = scan_results_finder(p_attr[1]);
            if(!scan_ssid)
            {
                TRACE_ERR("Input wrong scan index: 0x%2x\n", p_attr[1]);
                gatt_status = WICED_BT_GATT_ERROR;
                break;
            }
            snprintf(buf, scan_ssid[1] + 1, "%s", scan_ssid + 2);
            TRACE_LOG("Input index: 0x%2x, found SSID: %s\n", p_attr[1], buf);

            memset(app_custom_service_wifi_ssid, 0, strlen((char *)app_custom_service_wifi_ssid));
            memcpy(app_custom_service_wifi_ssid, buf, scan_ssid[1]);
            puAttribute = le_app_find_by_handle(HDLC_CUSTOM_SERVICE_WIFI_SSID_VALUE);
            puAttribute->cur_len = scan_ssid[1];
            TRACE_LOG("Wi-Fi SSID: %s\n", app_custom_service_wifi_ssid);

            /* Check tag */ 
            if(*(int*)(scan_ssid + scan_ssid[1] + 4) == WPA2_VALUE)
            {
                /* Configuration for WPA2 Wi-Fi access point */
                strcat(linux_cmd, "bash wlan_config.sh \"");
            }
            else
            {
                /* Configuration for OPEN Wi-Fi access point which has no need of password */
                strcat(linux_cmd, "bash wlan_config_OPEN.sh \"");
            }
            strcat(linux_cmd, buf);
            strcat(linux_cmd, "\" \"");
            if(*(int*)(scan_ssid + scan_ssid[1] + 4) == WPA2_VALUE)
            {
                strcat(linux_cmd, app_custom_service_wifi_password);
            }
            strcat(linux_cmd, "\"");
            TRACE_LOG("Wi-Fi connect command: %s\n", linux_cmd);
            fp = popen(linux_cmd, "r");
            if (fp == NULL)
            {
                TRACE_ERR("Failed to run command connect\n" );
                gatt_status = WICED_BT_GATT_ERROR;
                break;
            }
            if(!is_process_ok(buf, SCAN_BUFFER_LENGTH, fp))
            {
                TRACE_ERR("Wi-Fi connect command result is not \"OK\", kindly check your Wi-Fi information, especially the length of PASSWORD.\n");
                gatt_status = WICED_BT_GATT_ERROR;
                sync_wifi_state(WIFI_STATE_DISCONNECTED);
                pclose(fp);
                break;
            }
            wifi_connected_state = WIFI_STATE_CONNECTING;
            pclose(fp);
        }
        else
        {
            TRACE_ERR("Invalid command\n");
            gatt_status = WICED_BT_GATT_ERROR;
        }
        break;

    /* Notification for control characteristic.
     */
    case HDLD_CUSTOM_SERVICE_WIFI_CONTROL_CLIENT_CHAR_CONFIG:
        app_custom_service_wifi_control_client_char_config[0] = p_attr[0];
        app_custom_service_wifi_control_client_char_config[1] = p_attr[1];
        break;

    default:
        TRACE_ERR("Write GATT Handle not found\n");
        gatt_status = WICED_BT_GATT_INVALID_HANDLE;
        break;
    }

    return (gatt_status);
}

/*******************************************************************************
* Function Name: le_app_read_handler
********************************************************************************
* Summary:
*   This function handles Read Requests received from the client device
*
* Parameters:
* @param conn_id       Connection ID
* @param opcode        LE GATT request type opcode
* @param p_read_req    Pointer to read request containing the handle to read
* @param len_req       length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
*******************************************************************************/
static wiced_bt_gatt_status_t le_app_read_handler( uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_read_t *p_read_req,
                                                    uint16_t len_req)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    /* Pointer to an attribute in the GATT db lookup table */
    gatt_db_lookup_table_t *puAttribute;
    int attr_len_to_copy;
    uint8_t *from;
    int to_send;

    if(NULL == p_read_req)
    {
        gatt_status = WICED_BT_GATT_ERROR;
    }
    else
    {
        puAttribute = le_app_find_by_handle(p_read_req->handle);
        if ( NULL == puAttribute )
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                                WICED_BT_GATT_INVALID_HANDLE);
            gatt_status = WICED_BT_GATT_INVALID_HANDLE;
        }
        else
        {

            attr_len_to_copy = puAttribute->cur_len;
            if (p_read_req->offset >= puAttribute->cur_len)
            {
                 wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                                     WICED_BT_GATT_INVALID_OFFSET);
                 gatt_status = WICED_BT_GATT_INVALID_OFFSET;
            }
            else
            {
                to_send = MIN(len_req, attr_len_to_copy - p_read_req->offset);
                from = ((uint8_t *)puAttribute->p_data) + p_read_req->offset;
                /* No need for context, as buff not allocated */
                gatt_status =  wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL);
            }
        }
    }
    return gatt_status;
}

/*******************************************************************************
* Function Name: le_app_connect_handler
********************************************************************************
* Summary:
*   This callback function handles connection status changes.
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_conn_status  : Pointer to data that
*   has connection details
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
*******************************************************************************/
static wiced_bt_gatt_status_t le_app_connect_handler(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS ;

    if ( NULL != p_conn_status )
    {
        if ( p_conn_status->connected )
        {
            /* Device has connected */
            TRACE_LOG("Connected: BDA " );
            print_bd_address(p_conn_status->bd_addr);
            TRACE_LOG("Connection ID '%d' \n", p_conn_status->conn_id );

            /* Update the connection handler.  Save address of the connected device. */
            iwos_state.conn_id = p_conn_status->conn_id;
            memcpy(iwos_state.remote_addr, p_conn_status->bd_addr, BD_ADDR_LEN);

            /* Update the adv/conn state */
            iwos_state.app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
        }
        else
        {
            /* Device has disconnected */
            TRACE_LOG("Disconnected: BDA " );
            print_bd_address(p_conn_status->bd_addr);
            TRACE_LOG("Connection ID '%d', Reason '%s'\n",
                                  p_conn_status->conn_id,
                                  get_bt_gatt_disconn_reason_name(p_conn_status->reason) );

            /* Resetting the device info */
            memset( iwos_state.remote_addr, 0, BD_ADDR_LENGTH);
            iwos_state.conn_id = 0;

            if(wifi_connected_state == WIFI_STATE_CONNECTING)
            {
                /* To disconnect from Wi-Fi access point via Wi-Fi interface */
                sync_wifi_state(WIFI_STATE_DISCONNECTED);
                system("bash wlan_remove_network.sh");
                TRACE_LOG("Stop connecting to Wi-Fi for starting bluetooth advertisement.");
            }

            /* Failed to start advertisement. Stop program execution */
            if (WICED_BT_SUCCESS != wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL))
            {
                TRACE_ERR("failed to start advertisement! \n");
                exit(EXIT_FAILURE);
            }

            /* Update the adv/conn state */
            iwos_state.app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;
        }

    }
    else
    {
        gatt_status = WICED_BT_GATT_ERROR;
    }
    return gatt_status;
}

/*******************************************************************************
* Function Name: le_app_server_handler
********************************************************************************
* Summary:
*   This function handles GATT server events from the BT stack.
*
* Parameters:
*   p_attr_req     Pointer to LE GATT connection status
*
* Return:
*   wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*   in wiced_bt_gatt.h
*
*******************************************************************************/
static wiced_bt_gatt_status_t le_app_server_handler (wiced_bt_gatt_attribute_request_t *p_attr_req)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;
	if(NULL != p_attr_req)
    {
        switch ( p_attr_req->opcode )
        {
        /* Attribute read request */
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
             gatt_status = le_app_read_handler( p_attr_req->conn_id,p_attr_req->opcode,
                                           &p_attr_req->data.read_req,
                                           p_attr_req->len_requested);
             break;
        case GATT_REQ_READ_BY_TYPE:
             gatt_status = le_app_bt_gatt_req_read_by_type_handler(p_attr_req->conn_id, p_attr_req->opcode,
                                                           &p_attr_req->data.read_by_type, p_attr_req->len_requested);
             break;
        
        /* Attribute write request */
        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
        case GATT_CMD_SIGNED_WRITE:
             gatt_status = le_app_write_handler(p_attr_req->conn_id, p_attr_req->opcode,
                                           &p_attr_req->data.write_req,
                                           p_attr_req->len_requested );
            if ( (p_attr_req->opcode == GATT_REQ_WRITE) &&  (gatt_status == WICED_BT_GATT_SUCCESS))
            {
                wiced_bt_gatt_write_req_t   *p_write_request = &p_attr_req->data.write_req;
                wiced_bt_gatt_server_send_write_rsp(p_attr_req->conn_id, p_attr_req->opcode, p_write_request->handle);
            } else {
                wiced_bt_gatt_write_req_t   *p_write_request = &p_attr_req->data.write_req;
                wiced_bt_gatt_server_send_error_rsp(p_attr_req->conn_id, p_attr_req->opcode, p_write_request->handle, gatt_status);
            }
             break;
        
        case GATT_REQ_MTU:
			 TRACE_LOG("Exchanged MTU from client: %d\n", p_attr_req->data.remote_mtu);
             gatt_status = wiced_bt_gatt_server_send_mtu_rsp(p_attr_req->conn_id,
                                                       p_attr_req->data.remote_mtu,
                                                       CY_BT_MTU_SIZE);
             break;
 
        case GATT_REQ_EXECUTE_WRITE:
            TRACE_LOG("Data length is too long, please configure SSID and PASSWORD separately");
            gatt_status = WICED_BT_GATT_SUCCESS;
            break;
        default:
             break;
        }
    }
    return gatt_status;
}

/*******************************************************************************
 * Function Name: le_app_free_buffer
 *******************************************************************************
 * Summary:
 *  This function frees up the memory buffer
 *
 *
 * Parameters:
 *  uint8_t *p_data: Pointer to the buffer to be free
 *
 ******************************************************************************/
static void le_app_free_buffer(uint8_t *p_buf)
{
    wiced_bt_free_buffer(p_buf);
}

/*******************************************************************************
 * Function Name: le_app_alloc_buffer
 *******************************************************************************
 * Summary:
 *  This function allocates a memory buffer.
 *
 *
 * Parameters:
 *  int len: Length to allocate
 *
 ******************************************************************************/
static void* le_app_alloc_buffer(int len)
{
    uint8_t *p = (uint8_t *)wiced_bt_get_buffer(len);
    return p;
}

/*******************************************************************************
 * Function Name : le_app_find_by_handle
 * *****************************************************************************
 * Summary : @brief  Find attribute description by handle
 *
 * @param handle    handle to look up
 *
 * @return gatt_db_lookup_table_t   pointer containing handle data
 ******************************************************************************/
static gatt_db_lookup_table_t *le_app_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (handle == app_gatt_db_ext_attr_tbl[i].handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}

/*******************************************************************************
 * Function Name: le_app_bt_gatt_req_read_by_type_handler
 * *****************************************************************************
 *
 * Summary:
 *  Process read-by-type request from peer device
 *
 * @param conn_id       Connection ID
 * @param opcode        LE GATT request type opcode
 * @param p_read_req    Pointer to read request containing the handle to read
 * @param len_requested length of data requested
 *
 * @return wiced_bt_gatt_status_t  LE GATT status
 ******************************************************************************/
static wiced_bt_gatt_status_t le_app_bt_gatt_req_read_by_type_handler(uint16_t conn_id,
                                                                   wiced_bt_gatt_opcode_t opcode,
                                                                   wiced_bt_gatt_read_by_type_t *p_read_req,
                                                                   uint16_t len_requested)
{
    /* Pointer to an attribute in the GATT db lookup table */
    gatt_db_lookup_table_t *puAttribute;
    uint16_t last_handle = 0;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = le_app_alloc_buffer(len_requested);
    uint8_t pair_len = 0;
    int used_len = 0;

    if (NULL == p_rsp)
    {
        TRACE_ERR("No memory, len_requested: %d!!\r\n",len_requested);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, 
     * between the start and end handles 
     */
    while (WICED_TRUE)
    {
        last_handle = attr_handle;
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle, p_read_req->e_handle,
                                                        &p_read_req->uuid);
        if (0 == attr_handle )
            break;

        if ( NULL == (puAttribute = le_app_find_by_handle(attr_handle)))
        {
            TRACE_ERR("found type but no attribute for %d \r\n",last_handle);
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            le_app_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used_len,
                                                                  len_requested - used_len,
                                                                  &pair_len,
                                                                  attr_handle,
                                                                  puAttribute->cur_len,
                                                                  puAttribute->p_data);
        if (0 == filled)
        {
            break;
        }
        used_len += filled;

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (0 == used_len)
    {
       TRACE_ERR("attr not found  start_handle: 0x%04x  end_handle: 0x%04x  Type: 0x%04x\r\n",
               p_read_req->s_handle, p_read_req->e_handle, p_read_req->uuid.uu.uuid16);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_INVALID_HANDLE);
        le_app_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */

    return wiced_bt_gatt_server_send_read_by_type_rsp(conn_id,
                                                      opcode,
                                                      pair_len,
                                                      used_len,
                                                      p_rsp,
                                                      (void *)le_app_free_buffer);
}

/*******************************************************************************
 * Function Name: iwos_smp_bond_result
 * *****************************************************************************
 *
 * Summary:
 *  Process SMP bonding result. If we successfully paired with the
 *  central device, save its BDADDR in the NVRAM and initialize
 *  associated data
 *
 * @param wiced_result       bond wiced result
 * 
 * @return None
 * 
 ******************************************************************************/
void iwos_smp_bond_result( uint8_t wiced_result )
{
    wiced_result_t status;
    TRACE_LOG( "bond wiced_result: %d\n", wiced_result );

    /* Bonding success */
    if ( wiced_result == WICED_BT_SUCCESS )
    {
        /* Pack the data to be stored into the hostinfo structure */
        memcpy( iwos_hostinfo.bdaddr, iwos_state.remote_addr, BD_ADDR_LEN );

        /* Write to NVRAM */
        wiced_hal_write_nvram( IWOS_VS_ID, sizeof(iwos_hostinfo),
            (uint8_t*)&iwos_hostinfo, &status);
        TRACE_LOG("NVRAM write status: %d\n", status);
    }
}

/*******************************************************************************
 * Function Name: iwos_load_keys_for_address_resolution
 * *****************************************************************************
 *
 * Summary:
 *  Load paired keys for address resolution
 *
 * @param None
 * 
 * @return None
 * 
 ******************************************************************************/
static void iwos_load_keys_for_address_resolution( void )
{
    wiced_bt_device_link_keys_t link_keys;
    wiced_result_t              wiced_result;
    uint8_t                     *p;

    memset( &link_keys, 0, sizeof(wiced_bt_device_link_keys_t));
    p = (uint8_t*)&link_keys;
    wiced_hal_read_nvram(IWOS_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p, &wiced_result);

    if(wiced_result == 0)
    {
        wiced_result = wiced_bt_dev_add_device_to_address_resolution_db ( &link_keys );
    }

    TRACE_LOG("iwos_load_keys_for_address_resolution wiced_result:%d \n", wiced_result );
}

/* END OF FILE [] */
