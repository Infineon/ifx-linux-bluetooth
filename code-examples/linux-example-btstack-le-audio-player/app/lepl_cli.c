/*
 * $ Copyright Cypress Semiconductor $
 */

/* Application includes */
#include <stdio.h>
#include <stdint.h>
#include "lepl.h"
#ifdef WIN32
#include "windows.h"
#include "winsock.h"
#elif __linux__
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif
//#include "app_bt_utils.h"

#ifdef CLI_SUPPORT
#ifdef WIN32
static SOCKET m_ConnectSocket = INVALID_SOCKET;
#elif __linux__
static int m_ConnectSocket = -1;
#endif
#define SOCK_PORT_NUM_START 12012
#define MPAF_TRAN_PKT_TYPE  25
#define INVALID_CONN_ID     0XFFFF
#define INVALID_ABS_VOL     -1
static int initial_sock = 0;
uint8_t zero_bda[6]                 = {0};
#define UNICAST_SINK_SCAN_DEVICE_MAX 50
#define MAX_BROADCAST_CODE_LEN 16

static wiced_ble_ext_scan_results_t unicast_scan_sink_list[UNICAST_SINK_SCAN_DEVICE_MAX]         = {0};
extern wiced_bool_t lepl_rpc_rx_cback(uint16_t opcode, uint8_t* p_data, uint32_t payload_len);
extern int get_spy_instance(void);

typedef enum {
    EXIT,
    PRINT_MENU,
    LE_START_ADV,
    LE_STOP_ADV,
    LE_SCAN_START,
    LE_SCAN_STOP,
    LE_CONNECT,
    LE_DISCONNECT,
    LE_AUDIO_PLAY,      //test_8k, test_16k, test_24k, test48k
    LE_AUDIO_PAUSE,
    LE_AUDIO_VOL_UP,
    LE_AUDIO_VOL_DOWN,
    LE_AUDIO_MUTE,
    LE_AUDIO_UNMUTE,
    LE_AUDIO_ABS_VOL,   //0~255
    LE_AUDIO_BROADCAST_START_STREAM,
    LE_AUDIO_BROADCAST_STOP_STREAM,
    LE_AUDIO_PLACE_CALL,
    LE_AUDIO_REMOTE_HELD_CALL,
    LE_AUDIO_REMOTE_HELD_RETRIVE,
    LE_AUDIO_TERMINATE_CALL,
    LE_AUDIO_START_MIC,
    LE_AUDIO_STOP_MIC,
    LE_AUDIO_HAS_READ_PRESET,
    LE_AUDIO_HAS_WRITE_PRESET,
    LE_AUDIO_HAS_SET_ACTIVE_PRESET,
    LE_AUDIO_HAS_SET_NEXT_PRESET,
    LE_AUDIO_HAS_SET_PREVIOUS_PRESET,
    END
}eapp_menu_t ;

typedef struct {
    wiced_bt_ga_bap_codec_config_t codec_config;
    char *str;
}app_menu_play_t;

enum
{
    SET_ACTIVE_PRESET = 1,
    SET_NEXT_PRESET,
    SET_PREVIOUS_PRESET,
};

typedef struct {
    eapp_menu_t eidx;
    uint16_t opcode;
    char *str;
}app_menu_t;

app_menu_t app_menu_array[] = {
    {EXIT,                  0,  "EXIT"      },
    {PRINT_MENU,            0,  "PRINT MENU"},
    {LE_START_ADV,          HCI_CONTROL_LE_COMMAND_ADVERTISE,  "START ADV" },
    {LE_STOP_ADV,           HCI_CONTROL_LE_COMMAND_ADVERTISE,  "STOP ADV" },
    {LE_SCAN_START,         HCI_CONTROL_LE_COMMAND_SCAN,       "START SCAN" },
    {LE_SCAN_STOP,          HCI_CONTROL_LE_COMMAND_SCAN,       "STOP SCAN" },
    {LE_CONNECT,            HCI_CONTROL_LE_COMMAND_CONNECT,    "CONNECT" },
    {LE_DISCONNECT,         HCI_CONTROL_LE_COMMAND_DISCONNECT,  "DISCONNECT" },
    {LE_AUDIO_PLAY,         HCI_CONTROL_LE_AUDIO_COMMAND_PLAY,  "PLAY" },
    {LE_AUDIO_PAUSE,        HCI_CONTROL_LE_AUDIO_COMMAND_PAUSE, "PAUSE" },
    {LE_AUDIO_VOL_UP,       HCI_CONTROL_LE_AUDIO_COMMAND_VOL_UP, "VOL UP" },
    {LE_AUDIO_VOL_DOWN,     HCI_CONTROL_LE_AUDIO_COMMAND_VOL_DOWN, "VOL DOWN" },
    {LE_AUDIO_MUTE,         HCI_CONTROL_LE_AUDIO_COMMAND_MUTE, "MUTE" },
    {LE_AUDIO_UNMUTE,       HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE, "UNMUTE" },
    {LE_AUDIO_ABS_VOL,      HCI_CONTROL_LE_AUDIO_COMMAND_ABS_VOL,       "SET ABS VOL" },
    {LE_AUDIO_BROADCAST_START_STREAM, HCI_CONTROL_LE_AUDIO_COMMAND_BROADCAST_SOURCE_START_STREAMIMG,      "Broadcast Start Stream" },
    {LE_AUDIO_BROADCAST_STOP_STREAM, HCI_CONTROL_LE_AUDIO_COMMAND_BROADCAST_SOURCE_START_STREAMIMG,      "Broadcast Stop Stream" },
    {LE_AUDIO_PLACE_CALL,   HCI_CONTROL_LE_AUDIO_COMMAND_PLACE_CALL, "Place Call" },
    {LE_AUDIO_REMOTE_HELD_CALL,   HCI_CONTROL_LE_AUDIO_COMMAND_REM_HOLD_CALL, "Remote Held Call" },
    {LE_AUDIO_REMOTE_HELD_RETRIVE,   HCI_CONTROL_LE_AUDIO_COMMAND_REM_HOLD_RETRIEVE, "Remote Held Retrive Call" },
    {LE_AUDIO_TERMINATE_CALL,   HCI_CONTROL_LE_AUDIO_COMMAND_TERMINATE_CALL, "Terminate Call" },
    {LE_AUDIO_START_MIC,   HCI_CONTROL_LE_AUDIO_COMMAND_START_STOP_MIC, "Start Mic" },
    {LE_AUDIO_STOP_MIC,    HCI_CONTROL_LE_AUDIO_COMMAND_START_STOP_MIC, "Stop Mic" },
    {LE_AUDIO_HAS_READ_PRESET,   HCI_CONTROL_LE_AUDIO_COMMAND_HAS_READ_PRESET, "Read Preset" },
    {LE_AUDIO_HAS_WRITE_PRESET,  HCI_CONTROL_LE_AUDIO_COMMAND_HAS_WRITE_PRESET_NAME, "Write Preset" },
    {LE_AUDIO_HAS_SET_ACTIVE_PRESET,    HCI_CONTROL_LE_AUDIO_COMMAND_HAS_SET_PRESET, "Set Active Preset" },
    {LE_AUDIO_HAS_SET_NEXT_PRESET,    HCI_CONTROL_LE_AUDIO_COMMAND_HAS_SET_PRESET, "Set Next Preset" },
    {LE_AUDIO_HAS_SET_PREVIOUS_PRESET,    HCI_CONTROL_LE_AUDIO_COMMAND_HAS_SET_PRESET, "Set Previous Preset" },
};

app_menu_play_t app_menu_play_array[] = {
    {BAP_CODEC_CONFIG_8_2_1,     "8_2_1"},
    {BAP_CODEC_CONFIG_16_2_1,    "16_2_1"},
    {BAP_CODEC_CONFIG_24_2_1,    "24_2_1"},
    {BAP_CODEC_CONFIG_32_2_1,    "32_2_1"},
    {BAP_CODEC_CONFIG_48_2_1,    "48_2_1"},
};

/******************************************************************************
* Function Name: empty_stdin()
*
* Summary:
*   clear stdin buffer
*
* Parameters:
*   None;
*
* Return:
*   None;
*
******************************************************************************/
static void empty_stdin(void)
{
    int c = getchar();

    while (c != '\n' && c != EOF)
        c = getchar();
}

/******************************************************************************
* Function Name: le_pl_cli_add_sink_dev
*******************************************************************************
* Summary: add found sink device to loacl array when scan, the index of the array
*          is the handle of device, need reset the array when start scan
*
* Parameters:
*   wiced_ble_ext_scan_results_t *p_scan_result
*
* Return:
*      None
*
******************************************************************************/
void le_pl_cli_add_sink_dev(wiced_ble_ext_scan_results_t *p_scan_result)
{
    uint8_t idx = 0;

    if (p_scan_result == NULL)
    {
        WICED_BT_TRACE("[%s][ERROR] p_scan_result is NULL\n", __FUNCTION__);
        return;
    }

    for (idx = 0; idx < UNICAST_SINK_SCAN_DEVICE_MAX; idx++)
    {
        if (memcmp(unicast_scan_sink_list[idx].remote_bd_addr, p_scan_result->remote_bd_addr, sizeof(zero_bda)) == 0)
        {
            return; 
        }
        if (memcmp(unicast_scan_sink_list[idx].remote_bd_addr, zero_bda, sizeof(zero_bda)) == 0)
        {
            break;
        }
    }
    memcpy(&unicast_scan_sink_list[idx], p_scan_result, sizeof(wiced_ble_ext_scan_results_t));
    WICED_BT_TRACE("Remote_bd_addr:%B Handle:%d\n",unicast_scan_sink_list[idx].remote_bd_addr, idx);
}

/******************************************************************************
* Function Name: le_pl_show_sink_dev
*******************************************************************************
* Summary: show current saving sink device
*
* Parameters:
*   None
*
* Return:
*   uint8_t: number of Unicast Sink device found
*
******************************************************************************/
uint8_t le_pl_show_sink_dev( void )
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < UNICAST_SINK_SCAN_DEVICE_MAX; i++)
    {
        if (memcmp(unicast_scan_sink_list[i].remote_bd_addr, zero_bda, sizeof(zero_bda)) != 0)
        {
            count++;
            WICED_BT_TRACE("%B, Index:%d\n", unicast_scan_sink_list[i].remote_bd_addr, i);
        }
    }
    return count;
}

/******************************************************************************
* Function Name: le_pl_get_scan_result
*******************************************************************************
* Summary: get the saving sink device info
*
* Parameters:
*   uint8_t idx: handle of the sink device
*
* Return:
*   None
*
******************************************************************************/
wiced_ble_ext_scan_results_t* le_pl_get_scan_result(uint8_t idx)
{
    if (idx >= UNICAST_SINK_SCAN_DEVICE_MAX)
    {
        WICED_BT_TRACE("index out of range:%d, MAX:%d\n", idx, UNICAST_SINK_SCAN_DEVICE_MAX-1);
        return NULL;
    }
    if (memcmp(unicast_scan_sink_list[idx].remote_bd_addr, zero_bda, sizeof(zero_bda)) == 0)
    {
        WICED_BT_TRACE("input Handle wrong, no sink device");
        return NULL;
    }

    return &unicast_scan_sink_list[idx];
}

/******************************************************************************
* Function Name: le_pl_clear_sink_dev
*******************************************************************************
* Summary: clear the saving sink device
*
* Parameters:
*   None
*
* Return:
*   None
*
******************************************************************************/
void le_pl_clear_sink_dev( void )
{
    memset(unicast_scan_sink_list, 0, sizeof(unicast_scan_sink_list));
}

void lepl_print_menu()
{
    for (uint16_t i = 0; i < sizeof(app_menu_array) / sizeof(app_menu_t); i++)
    {
        printf("%d.\t", i);
        printf("%s\n", app_menu_array[i].str);
    }
}

static void le_pl_print_menu_play(void)
{

    for (uint8_t i = 0; i < sizeof(app_menu_play_array) / sizeof(app_menu_play_t); i++)
    {
        printf("%d.\t", i);
        printf("%s\n", app_menu_play_array[i].str);
    }
}

static void read_local_bda( void )
{
    wiced_bt_device_address_t bda = { 0 };

    wiced_bt_dev_read_local_addr(bda);
    WICED_BT_TRACE("Local Bluetooth Address: [%B]\n", bda);
}

void le_pl_cli_send_pkt(uint16_t opcode, uint8_t* p_data, uint32_t payload_len)
{
    uint8_t temp_data[512];
    uint8_t* p = temp_data;
    UINT8_TO_STREAM(p, MPAF_TRAN_PKT_TYPE);
    UINT16_TO_STREAM(p, opcode);
    UINT16_TO_STREAM(p, payload_len);
    ARRAY_TO_STREAM(p, p_data, payload_len);
    send(m_ConnectSocket, temp_data, payload_len+5,0);
    //lepl_rpc_rx_cback(opcode, p_data, payload_len);
}

uint16_t le_pl_cli_input_conn_id(void)
{
    uint8_t index = 0;
    for (uint8_t i = 0; i < MAX_CONNECTION_INSTANCE; i++)
    {
        if (g_lepl_gatt_cb.unicast_clcb[i].in_use)
        {
            lepl_clcb_t *p_clcb = &g_lepl_gatt_cb.unicast_clcb[i];
            WICED_BT_TRACE("Index %d conn_id %d addr_type %d BDA %B ", i, p_clcb->conn_id, p_clcb->addr_type, p_clcb->bda);
        }
    }
    WICED_BT_TRACE("Enter Index :\n");
    if ((scanf("%hhd", &index) == EOF) || (index >= MAX_CONNECTION_INSTANCE) || (g_lepl_gatt_cb.unicast_clcb[index].in_use == WICED_FALSE))
    {
        WICED_BT_TRACE("Invalid Index \n");
        return 0xFFFF;
    }
    return g_lepl_gatt_cb.unicast_clcb[index].conn_id;
}

/******************************************************************************
* Function Name: le_pl_cli_input_abs_vol
*******************************************************************************
* Summary:
*   Reads a user-entered volume input (0-100), checks its validity,
*   and converts it to an corresponding absolute volume level (0-255).
*
* Parameters:
*   abs_vol - [out] pointer to store the converted volume value
*
* Return:
*   0                 - Success
*   INVALID_ABS_VOL   - The input is not a numeric value or out of range
*
******************************************************************************/
static int le_pl_cli_input_abs_vol(uint8_t* abs_vol)
{
    #define MAX_VOL     100
    #define MIN_VOL     0
    #define DEFAULT_VOL 10 //a non-zero value to avoid silent audio output on initial use

    int vol_input = DEFAULT_VOL; //user input range: 0 - 100

    WICED_BT_TRACE("Enter Volume (0 ~ 100) :\n");
    if(scanf("%d", &vol_input) != 1)
    {
        printf("The input is not a number. \nPlease enter a numeric value !!!\n");
        return INVALID_ABS_VOL;
    }
    if(vol_input > MAX_VOL || vol_input < MIN_VOL)
    {
        printf("Out of range!!! \nPlease enter a value between 0 and 100.\n");
        return INVALID_ABS_VOL;
    }

    *abs_vol = (uint8_t)((vol_input * 255) / 100);

    return 0;
}

void le_pl_set_has_preset(uint16_t opcode, uint8_t action)
{
    uint16_t conn_id = le_pl_cli_input_conn_id();
    uint8_t preset_index = 0;
    uint8_t data[4];
    uint8_t* p = data;
    if (conn_id != 0xFFFF)
    {
        if (SET_ACTIVE_PRESET == action)
        {
            WICED_BT_TRACE("Enter Preset Index (>= 1) : ");
            if ((scanf("%hhd", &preset_index) == EOF))
            {
                WICED_BT_TRACE("Input index Invalid\n");
                return;
            }
        }
        UINT16_TO_STREAM(p, conn_id);
        UINT8_TO_STREAM(p, action);
        UINT8_TO_STREAM(p, preset_index);
        le_pl_cli_send_pkt(opcode, data, sizeof(data));
    }
}

void le_pl_input_broadcast_stream_config(void)
{
    uint8_t data[100];
    uint8_t *p = data;
    uint8_t index;
    uint8_t ch_count;
    uint8_t bis_count;
    uint8_t encryption;
    uint32_t broadcast_id;
    uint8_t input_broadcast_code[MAX_BROADCAST_CODE_LEN] = {0};
    WICED_BT_TRACE("Enter the index for codec configuration:\n");
    le_pl_print_menu_play();
    if ( (scanf("%hhd", &index) == EOF) ||  (index >= sizeof(app_menu_play_array) / sizeof(app_menu_play_t)))
    {
        WICED_BT_TRACE("Input index Invalid\n"); 
        return;
    }
    WICED_BT_TRACE("Enter Encryption config 1) Enable 0) Disable : ");
    if ( (scanf("%hhd", &encryption) == EOF) ||  (encryption>1))
    {
        WICED_BT_TRACE("Input Invalid\n"); 
        return;
    }
    WICED_BT_TRACE("Enter Stream Channel Count (Mono 1 or Stereo 2): ");
    if ( (scanf("%hhd", &ch_count) == EOF) ||  (ch_count==0) || (ch_count>2))
    {
        WICED_BT_TRACE("Input Invalid\n"); 
        return;
    }
    WICED_BT_TRACE("Enter Broadcast ID : ");
    if ( (scanf("%d", &broadcast_id) == EOF) )
    {
        WICED_BT_TRACE("Input Invalid\n"); 
        return;
    }
    WICED_BT_TRACE("Enter BIS Count : ");
    if ( (scanf("%hhd", &bis_count) == EOF) )
    {
        WICED_BT_TRACE("Input Invalid\n"); 
        return;
    }
    if (encryption)
    {
        int code_count = 0;
        char input;
        WICED_BT_TRACE("Enter Broadcast Code : \n");
        while (code_count <= MAX_BROADCAST_CODE_LEN - 1)
        {
            if ( (scanf("%c", &input) == EOF) )
            {
                WICED_BT_TRACE("Enter allowed input error!");
                break;
            }
            else
            {
                if (input != '\n')
                {
                    input_broadcast_code[code_count] = input;
                }
                else
                {
                    if (code_count == 0)
                        continue;
                    break;
                }
                code_count++;
            }
        }
    }
    UINT8_TO_STREAM(p, 1); // start
    UINT32_TO_STREAM(p, app_menu_play_array[index].codec_config); // Codec Configuration
    UINT8_TO_STREAM(p, encryption); // encryption
    UINT32_TO_STREAM(p, ch_count); // Channel Count
    UINT32_TO_STREAM(p, broadcast_id); // Broadcast ID
    ARRAY_TO_STREAM(p, input_broadcast_code, MAX_BROADCAST_CODE_LEN); // Broadcast Code
    UINT8_TO_STREAM(p, bis_count); // BIS Count
    le_pl_cli_send_pkt(app_menu_array[LE_AUDIO_BROADCAST_START_STREAM].opcode, data, p - data);
}
#ifdef WIN32
wiced_bool_t le_pl_connect_to_socket()
{
    SOCKADDR_IN service;

    // Create a local SOCKET
    if (INVALID_SOCKET == (m_ConnectSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)))
    {
        WICED_BT_TRACE_CRIT("listen socket failed with error: %ld, socket thread exiting\n", WSAGetLastError());
        return WICED_FALSE;
    }

    service.sin_family = AF_INET;
    service.sin_addr.s_addr = inet_addr("127.0.0.1");
    service.sin_port = htons(get_spy_instance() + SOCK_PORT_NUM_START);
  
    if (connect(m_ConnectSocket, &service, sizeof(service)) < 0) 
    {
        WICED_BT_TRACE_CRIT("Connection failed");
        close(m_ConnectSocket);
        return WICED_FALSE;
    }
    return WICED_TRUE;

}

#elif __linux__
wiced_bool_t le_pl_connect_to_socket()
{
    struct sockaddr_in service;
    int addrlen = sizeof(service);

   // Create a local SOCKET
    if (0 == (m_ConnectSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)))
    {
        WICED_BT_TRACE_CRIT("socket failed");
        return WICED_FALSE;
    }

    service.sin_family = AF_INET;
    service.sin_addr.s_addr = INADDR_ANY;
    service.sin_port = htons(get_spy_instance() + SOCK_PORT_NUM_START);
    memset(service.sin_zero, 0, sizeof(service.sin_zero));

    if (connect(m_ConnectSocket, (struct sockaddr *) & service, sizeof(service)) < 0)
    {
        WICED_BT_TRACE_CRIT("Connection failed");
        close(m_ConnectSocket);
        return WICED_FALSE;
    }
    return WICED_TRUE;

}
#endif


int le_pl_cli_routine(void)
{
    int         ret                     = 0;
    int         input                   = 0;
	char        inputstr[3]             = {0};
    if (initial_sock == 0)
    {
        initial_sock = 1;
        if (le_pl_connect_to_socket() == WICED_FALSE)
        {
            return 0;
        }
    }
	memset(inputstr, 0, sizeof(inputstr));
    empty_stdin();
    WICED_BT_TRACE("Press 1 and Click Enter to Show Menu Option!!\n");
    ret = scanf ("%[^\n]s", inputstr);
    if (ret == 0)
    {
        return -1;
    }
    input = atoi(inputstr);
    WICED_BT_TRACE("---------------------------------------------------\n");
	switch (input)
	{
	case EXIT:
        return 0;
        break;
    case PRINT_MENU:
        read_local_bda();
        lepl_print_menu();
        break;
    case LE_START_ADV:
        {
            uint8_t start = 1;
            le_pl_cli_send_pkt(app_menu_array[input].opcode, &start, sizeof(start));
        }
        break;
    case LE_STOP_ADV:
        {
            uint8_t start = 0;
            le_pl_cli_send_pkt(app_menu_array[input].opcode, &start, sizeof(start));
        }
        break;
    case LE_SCAN_START:
        {
            uint8_t data[2] = { 1,1 };
            le_pl_clear_sink_dev();
            le_pl_cli_send_pkt(app_menu_array[input].opcode, data, sizeof(data));
        }
        break;
    case LE_SCAN_STOP:
        {
            uint8_t data[2] = { 0,1 };
            le_pl_cli_send_pkt(app_menu_array[input].opcode, data, sizeof(data));
        }
        break;
    case LE_CONNECT:
        {
            uint8_t index = 0;
            uint8_t num_dev;
            uint8_t data[7];
            uint8_t *p = data;
            wiced_ble_ext_scan_results_t *p_sink_device = NULL;
            num_dev = le_pl_show_sink_dev();
            if (num_dev == 0)
            {
                WICED_BT_TRACE("No Unicast Sink Device found\n");
                break;
            }
            WICED_BT_TRACE("Enter the Handle of Sink Device:\n");
            if ((scanf("%hhd", &index) == EOF) || (index > num_dev))
            {
                WICED_BT_TRACE("Input Invalid ");
                break;
            }
            p_sink_device = le_pl_get_scan_result(index);
            UINT8_TO_STREAM(p, p_sink_device->ble_addr_type);
            BDADDR_TO_STREAM(p, p_sink_device->remote_bd_addr);

            le_pl_cli_send_pkt(app_menu_array[input].opcode, data, sizeof(data));
        }
        break;
    case LE_DISCONNECT:
    case LE_AUDIO_PAUSE:
    case LE_AUDIO_VOL_UP:
    case LE_AUDIO_VOL_DOWN:
    case LE_AUDIO_MUTE:
    case LE_AUDIO_UNMUTE:
    case LE_AUDIO_HAS_READ_PRESET:
        {
            uint16_t conn_id = le_pl_cli_input_conn_id();
            if (conn_id != INVALID_CONN_ID)
            {
                le_pl_cli_send_pkt(app_menu_array[input].opcode, (uint8_t* ) & conn_id, sizeof(conn_id));
            }
        }
        break;
    case LE_AUDIO_ABS_VOL:
        {
            uint8_t abs_vol = 0;

            uint16_t conn_id = le_pl_cli_input_conn_id();
            if (conn_id == INVALID_CONN_ID)
                break;

            int ret = le_pl_cli_input_abs_vol(&abs_vol);
            if (ret == INVALID_ABS_VOL)
                break;

            uint8_t data[3] = {0};
            uint8_t* p = data;

            UINT16_TO_STREAM(p, conn_id);
            UINT8_TO_STREAM(p, abs_vol);
            const uint32_t payload_len = (uint32_t)(p - data);

            le_pl_cli_send_pkt(app_menu_array[input].opcode, data, payload_len);
        }
        break;
    case LE_AUDIO_PLAY:
        {
            uint8_t data[6];
            uint8_t     *p                      = data;
            uint8_t index;
            uint16_t conn_id = le_pl_cli_input_conn_id();
            if (conn_id != INVALID_CONN_ID)
            {
                WICED_BT_TRACE("Enter the index for codec configuration:\n");
                le_pl_print_menu_play();
                if ( (scanf("%hhd", &index) == EOF) ||  (index >= sizeof(app_menu_play_array) / sizeof(app_menu_play_t)))
                {
                    WICED_BT_TRACE("Input index Invalid\n"); 
                    break;
                }
                UINT16_TO_STREAM(p, conn_id);
                UINT32_TO_STREAM(p, app_menu_play_array[index].codec_config);
                le_pl_cli_send_pkt(app_menu_array[input].opcode, data, sizeof(data));
            }
        }
        break;
    case LE_AUDIO_HAS_WRITE_PRESET:
        {
            uint16_t conn_id = le_pl_cli_input_conn_id();
            uint8_t preset_index;
            uint8_t data[250];
            char str[200];
            uint8_t* p = data;
            if (conn_id != INVALID_CONN_ID)
            {
                WICED_BT_TRACE("Enter Preset Index : ");
                if ((scanf("%hhd", &preset_index) == EOF))
                {
                    WICED_BT_TRACE("Input index Invalid\n");
                    break;
                }
                UINT16_TO_STREAM(p, conn_id);
                UINT8_TO_STREAM(p, preset_index);
                WICED_BT_TRACE("Enter Preset String : ");
                ret = scanf("%s", str);
                if ((ret == 0) || (ret == EOF))
                {
                    WICED_BT_TRACE("Input String invalid \n");
                    break;
                }
                ARRAY_TO_STREAM(p, str, strlen(str));
                le_pl_cli_send_pkt(app_menu_array[input].opcode, data, p - data);
            }
        }
        break;
    case LE_AUDIO_HAS_SET_ACTIVE_PRESET:
        le_pl_set_has_preset(app_menu_array[input].opcode, SET_ACTIVE_PRESET);
        break;
    case LE_AUDIO_HAS_SET_NEXT_PRESET:
        le_pl_set_has_preset(app_menu_array[input].opcode, SET_NEXT_PRESET);
        break;
    case LE_AUDIO_HAS_SET_PREVIOUS_PRESET:
        le_pl_set_has_preset(app_menu_array[input].opcode, SET_PREVIOUS_PRESET);
        break;
    case LE_AUDIO_BROADCAST_START_STREAM:
        le_pl_input_broadcast_stream_config();
        break;
    case LE_AUDIO_BROADCAST_STOP_STREAM:
        {
            uint8_t data[28] = { 0 };
            le_pl_cli_send_pkt(app_menu_array[input].opcode, data, sizeof(data));
        }
        break;
    case LE_AUDIO_PLACE_CALL:
        {
            uint8_t data[250];
            uint8_t uri_len,frd_name_len;
            uint8_t* p = data;
            char uri[WICED_BT_GA_TBS_BEARER_URI_MAX_SIZE],frd_name[WICED_BT_GA_TBS_FRIENDLY_NAME_MAX_SIZE];
            uint16_t conn_id = le_pl_cli_input_conn_id();
            if (conn_id == INVALID_CONN_ID)
                break;

            WICED_BT_TRACE("Enter URI : ");
            ret = scanf("%s", uri);

            if ((ret == 0) || (ret == EOF))
            {
                WICED_BT_TRACE("Invalid Input \n");
                break;
            }
            WICED_BT_TRACE("Enter Friendly Name : ");
            ret = scanf("%s", frd_name);

            if ((ret == 0) || (ret == EOF))
            {
                WICED_BT_TRACE("Invalid Input \n");
                break;
            }
            uri_len = (uint8_t)strlen(uri);
            frd_name_len = (uint8_t)strlen(frd_name);
            UINT16_TO_STREAM(p, conn_id);
            UINT8_TO_STREAM(p, uri_len);
            ARRAY_TO_STREAM(p, uri, uri_len);
            UINT8_TO_STREAM(p, frd_name_len);
            ARRAY_TO_STREAM(p, frd_name, frd_name_len);
            le_pl_cli_send_pkt(app_menu_array[input].opcode, data, p-data);
        }
        break;
    case LE_AUDIO_REMOTE_HELD_CALL:
    case LE_AUDIO_REMOTE_HELD_RETRIVE:
        {
            uint8_t call_id = 0;
            WICED_BT_TRACE("Enter the call ID :\n");
            if ((scanf("%hhd", &call_id) == EOF))
            {
                WICED_BT_TRACE("Input Invalid ");
                break;
            }
            le_pl_cli_send_pkt(app_menu_array[input].opcode, &call_id, sizeof(call_id));
        }
        break;
    case LE_AUDIO_TERMINATE_CALL:
        {
            uint8_t data[4];
            uint8_t* p = data;
            uint8_t call_id = 0;
            uint16_t conn_id = le_pl_cli_input_conn_id();
            uint8_t terminate_reason = WICED_BT_GA_TBS_CLIENT_TERMINATED;
            if (conn_id == INVALID_CONN_ID)
                break;
            WICED_BT_TRACE("Enter the Call ID :\n");
            if ((scanf("%hhd", &call_id) == EOF))
            {
                WICED_BT_TRACE("Input Invalid ");
                break;
            }
            UINT16_TO_STREAM(p, conn_id);
            UINT8_TO_STREAM(p, call_id);
            UINT8_TO_STREAM(p, terminate_reason);
            le_pl_cli_send_pkt(app_menu_array[input].opcode, data, p-data);
        }
        break;
    case LE_AUDIO_START_MIC:
        {
            uint8_t data[5];
            uint8_t     *p                      = data;
            uint8_t index;
            uint16_t conn_id = le_pl_cli_input_conn_id();
            uint8_t start = 1;
            if (conn_id != INVALID_CONN_ID)
            {
                WICED_BT_TRACE("Enter the index for codec configuration:\n");
                le_pl_print_menu_play();
                if ( (scanf("%hhd", &index) == EOF) ||  (index >= sizeof(app_menu_play_array) / sizeof(app_menu_play_t)))
                {
                    WICED_BT_TRACE("Input index Invalid\n"); 
                    break;
                }
                UINT16_TO_STREAM(p, conn_id);
                UINT8_TO_STREAM(p, start);
                UINT16_TO_STREAM(p, app_menu_play_array[index].codec_config);
                le_pl_cli_send_pkt(app_menu_array[input].opcode, data, p-data);
            }
        }
        break;
    case LE_AUDIO_STOP_MIC:
        {
            uint8_t data[5] = { 0 };
            uint8_t     *p                      = data;
            uint16_t conn_id = le_pl_cli_input_conn_id();
            uint8_t start = 0;
            if (conn_id != INVALID_CONN_ID)
            {
                UINT16_TO_STREAM(p, conn_id);
                UINT8_TO_STREAM(p, start);
                UINT16_TO_STREAM(p, 0);
                le_pl_cli_send_pkt(app_menu_array[input].opcode, data, p-data);
            }
        }
        break;
	default:
        WICED_BT_TRACE("Input error!!\n");
		break;
	}
    return 1;
}

#endif // CLI_SUPPORT
