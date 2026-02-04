/*
 * $ Copyright Cypress Semiconductor $
 */

/* Application includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "lehs.h"
#ifdef WIN32
#include "windows.h"
#include "winsock.h"
#elif __linux__
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <log.h>
#endif
//#include "app_bt_utils.h"

#ifdef CLI_SUPPORT
#ifdef WIN32
static SOCKET m_ConnectSocket = INVALID_SOCKET;
#elif __linux__
static int m_ConnectSocket = -1;
#endif
static int initial_sock = 0;
extern wiced_bt_ga_csis_sirk_t      default_sirk;
extern char* csis_cfg_str[CSIS_CFG_STR_SIZE];
extern csis_cfg_t csis_cfg[CSIS_CFG_STR_SIZE];
#define SOCK_PORT_NUM_START 12012
#define MPAF_TRAN_PKT_TYPE  25
#define INVALID_CONN_ID     0XFFFF
#define INVALID_ABS_VOL     -1
extern broadcast_sink_cb_t *lehs_get_broadcast_sink_cb(void);
extern wiced_bool_t lehs_rpc_rx_callback(uint16_t opcode, uint8_t* p_data, uint32_t payload_len);
extern int get_spy_instance(void);
typedef enum {
    EXIT,
    PRINT_MENU,
    LEHS_SET_CSIS_PARAMS,
    LE_START_ADV,
    LE_STOP_ADV,
    LE_CONNECT,
    LE_DISCONNECT,
    LE_AUDIO_PLAY,      //test_8k, test_16k, test_24k, test48k
    LE_AUDIO_PAUSE,
    LE_AUDIO_VOL_UP,
    LE_AUDIO_VOL_DOWN,
    LE_AUDIO_MUTE,
    LE_AUDIO_UNMUTE,
    LE_AUDIO_ABS_VOL,   //0~255
    LE_AUDIO_BROADCAST_START_FIND_SOURCE,
    LE_AUDIO_BROADCAST_STOP_FIND_SOURCE,
    LE_AUDIO_BROADCAST_SINK_SYNC_TO_SOURCES,
    LE_AUDIO_BROADCAST_SINK_TERMINATE_SYNC,
    LE_AUDIO_BROADCAST_GET_INFO,
    LE_AUDIO_ACCEPT_CALL,
    LE_AUDIO_HOLD_CALL,
    LE_AUDIO_RETRIVE_CALL,
    LE_AUDIO_ORIGINATE_CALL,
    LE_AUDIO_JOIN_CALL,
    LE_AUDIO_TERMINATE_CALL,
    LE_AUDIO_MICP_MUTE,
    LE_AUDIO_MICP_AICS_MUTE,
    LE_AUDIO_MICP_AICS_SET_GAIN,
    END
}eapp_menu_t ;

typedef struct {
    eapp_menu_t eidx;
    uint16_t opcode;
    char *str;
}app_menu_t;

app_menu_t app_menu_array[] = {
    {EXIT,                  0,  "EXIT"      },
    {PRINT_MENU,            0,  "PRINT MENU"},
    {LEHS_SET_CSIS_PARAMS,  0,  "SETUP CSIS PARAMS"},
    {LE_START_ADV,          HCI_CONTROL_LE_COMMAND_ADVERTISE,  "START ADV" },
    {LE_STOP_ADV,           HCI_CONTROL_LE_COMMAND_ADVERTISE,  "STOP ADV" },
    {LE_CONNECT,            HCI_CONTROL_LE_COMMAND_CONNECT,    "CONNECT" },
    {LE_DISCONNECT,         HCI_CONTROL_LE_COMMAND_DISCONNECT,  "DISCONNECT" },
    {LE_AUDIO_PLAY,         HCI_CONTROL_LE_AUDIO_COMMAND_PLAY,  "PLAY" },
    {LE_AUDIO_PAUSE,        HCI_CONTROL_LE_AUDIO_COMMAND_PAUSE, "PAUSE" },
    {LE_AUDIO_VOL_UP,       HCI_CONTROL_LE_AUDIO_COMMAND_VOL_UP, "VOL UP" },
    {LE_AUDIO_VOL_DOWN,     HCI_CONTROL_LE_AUDIO_COMMAND_VOL_DOWN, "VOL DOWN" },
    {LE_AUDIO_MUTE,         HCI_CONTROL_LE_AUDIO_COMMAND_MUTE, "MUTE" },
    {LE_AUDIO_UNMUTE,       HCI_CONTROL_LE_AUDIO_COMMAND_UNMUTE, "UNMUTE" },
    {LE_AUDIO_ABS_VOL,      HCI_CONTROL_LE_AUDIO_COMMAND_ABS_VOL,       "SET ABS VOL" },
    {LE_AUDIO_BROADCAST_START_FIND_SOURCE, HCI_CONTROL_LE_AUDIO_COMMAND_BROADCAST_SINK_FIND_SOURCES,      "Broadcast Sink Find Source" },
    {LE_AUDIO_BROADCAST_STOP_FIND_SOURCE, HCI_CONTROL_LE_AUDIO_COMMAND_BROADCAST_SINK_FIND_SOURCES,      "Broadcast Sink Stop Find Source" },
    {LE_AUDIO_BROADCAST_SINK_SYNC_TO_SOURCES,   HCI_CONTROL_LE_AUDIO_COMMAND_BROADCAST_SINK_SYNC_TO_SOURCES, "Broadcast Sink Sync to Source" },
    {LE_AUDIO_BROADCAST_SINK_TERMINATE_SYNC,    HCI_CONTROL_LE_AUDIO_COMMAND_BROADCAST_SINK_SYNC_TO_SOURCES, "Broadcast Sink Terminate" },
    {LE_AUDIO_BROADCAST_GET_INFO,  HCI_CONTROL_LE_AUDIO_COMMAND_GET_BIS_INFO, "Get BIS Information" },
    {LE_AUDIO_ACCEPT_CALL,         HCI_CONTROL_LE_AUDIO_COMMAND_ACCEPT_CALL, "Accept Call" },
    {LE_AUDIO_HOLD_CALL,           HCI_CONTROL_LE_AUDIO_COMMAND_HOLD_CALL, "HOLD Call" },
    {LE_AUDIO_RETRIVE_CALL,        HCI_CONTROL_LE_AUDIO_COMMAND_RETRIEVE_CALL, "Retrive Call" },
    {LE_AUDIO_ORIGINATE_CALL,      HCI_CONTROL_LE_AUDIO_COMMAND_ORIGINATE_CALL, "Originate Call" },
    {LE_AUDIO_JOIN_CALL,           HCI_CONTROL_LE_AUDIO_COMMAND_JOIN_CALL, "Join Call" },
    {LE_AUDIO_TERMINATE_CALL,      HCI_CONTROL_LE_AUDIO_COMMAND_TERMINATE_CALL, "Terminate Call" },
    {LE_AUDIO_MICP_MUTE,           HCI_CONTROL_LE_AUDIO_COMMAND_MICP_MUTE, "MICP Mute " },
    {LE_AUDIO_MICP_AICS_MUTE,      HCI_CONTROL_LE_AUDIO_COMMAND_MICP_AICS_MUTE, "MICP AICS Mute" },
    {LE_AUDIO_MICP_AICS_SET_GAIN,  HCI_CONTROL_LE_AUDIO_COMMAND_MICP_AICS_SET_GAIN, "MICP AICS Set Gain" },
};

static const char enter_braodcast_id_str[] = "\n\
Please enter broadcast ID (hex): \n";

static wiced_bool_t input_hex(int* p_number);
static void input_chars(uint8_t* buf, int size);

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
* Function Name: error_check()
*******************************************************************************
* Summary:
*   check the scanf result
*       EOF: CTRL-D, user cancel input, use clearerr(stdin), restore stdin
*       result == 0: input not match the scanf type, eg: %d, or %x
*
* Parameters:
*   int result: scanf retrun result
*
* Return:
*   BOOL32: WICED_TRUE: no error
*           WICED_FALSE: error occur
*
******************************************************************************/
static wiced_bool_t error_check(int result)
{
   if (result == EOF)
   {
        TRACE_ERR("USER CANCEL INPUT\n");
        clearerr(stdin);
        return WICED_FALSE;
   } else if (result == 0) {
        TRACE_ERR("USER INPUT FORMAT WRONG\n");
        empty_stdin();
        return WICED_FALSE;
   }
   return WICED_TRUE;
}

void lehs_print_menu()
{
    for (uint16_t i = 0; i < sizeof(app_menu_array) / sizeof(app_menu_t); i++)
    {
        printf("%d.\t", i);
        printf("%s\n", app_menu_array[i].str);
    }
}

void le_hs_cli_send_pkt(uint16_t opcode, uint8_t* p_data, uint32_t payload_len)
{
    uint8_t temp_data[512];
    uint8_t* p = temp_data;

    UINT8_TO_STREAM(p, MPAF_TRAN_PKT_TYPE);
    UINT16_TO_STREAM(p, opcode);
    UINT16_TO_STREAM(p, payload_len);
    ARRAY_TO_STREAM(p, p_data, payload_len);
    send(m_ConnectSocket, temp_data, payload_len+5,0);
    //lehs_rpc_rx_callback(opcode, p_data, payload_len);
}

#ifdef WIN32
wiced_bool_t le_hs_connect_to_socket()
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
    WICED_BT_TRACE("Connect to sockect success");
    return WICED_TRUE;

}

#elif __linux__
wiced_bool_t le_hs_connect_to_socket()
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

static void read_local_bda( void )
{
    wiced_bt_device_address_t bda = { 0 };

    wiced_bt_dev_read_local_addr(bda);
    WICED_BT_TRACE("Local Bluetooth Address: [%B]\n", bda);
}

uint16_t le_hs_cli_input_conn_id(void)
{
    uint8_t index = 0;
    for (uint8_t i = 0; i < MAX_CONNECTION_INSTANCE; i++)
    {
        if (g_lehs_gatt_cb.clcb[i].in_use)
        {
            lehs_clcb_t *p_clcb = &g_lehs_gatt_cb.clcb[i];
            WICED_BT_TRACE("Index %d conn_id %d addr_type %d BDA %B ", i, p_clcb->conn_id, p_clcb->addr_type, p_clcb->bda);
        }
    }
    WICED_BT_TRACE("Enter Index :\n");
    if ((scanf("%hhd", &index) == EOF) || (index >= MAX_CONNECTION_INSTANCE) || (g_lehs_gatt_cb.clcb[index].in_use == WICED_FALSE))
    {
        WICED_BT_TRACE("Invalid Index \n");
        return 0xFFFF;
    }
    return g_lehs_gatt_cb.clcb[index].conn_id;
}

int le_hs_cli_input_abs_vol(uint8_t *abs_vol)
{
    int MAX_VOL = 100, MIN_VOL = 0;
    int vol_input = MIN_VOL; //user input range: 0 ~ 100

    WICED_BT_TRACE("Enter Volume (0 ~ 100) :\n");
    if(scanf("%d", &vol_input) == -1)
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

int le_hs_cli_routine(void)
{
    int         ret                     = 0;
    int         input                   = 0;
    char        inputstr[3]             = {0};
    if (initial_sock == 0)
    {
        initial_sock = 1;
        if (le_hs_connect_to_socket() == WICED_FALSE)
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
    case PRINT_MENU:
        read_local_bda();
        lehs_print_menu();
        break;
    case LEHS_SET_CSIS_PARAMS:
    {
        char *file_name = NULL;
        printf("Please Enter CSIS cfg\n");
        printf("1: MIC_LEFT\n");
        printf("2: MIC_RIGHT\n");
        ret = scanf("%d", &input);
        if(error_check(ret) == WICED_FALSE)
        {
            TRACE_ERR("Input error!!\n");
            break;
        }
        if (input != 1 && input != 2)
        {
            TRACE_ERR("CSIS_CFG input:%d out of support range:\n", input);
            break;
        }
        file_name = (input == 1) ? CSIS_CFG_MIC_LEFT_NAME : CSIS_CFG_MIC_RIGHT_NAME;
        app_init_csis_by_cfg_file(file_name);
    }
        break;
    case LE_START_ADV:
    case LE_AUDIO_BROADCAST_START_FIND_SOURCE:
        {
            uint8_t start = 1;
            le_hs_cli_send_pkt(app_menu_array[input].opcode, &start, sizeof(start));
        }
        break;
    case LE_STOP_ADV:
    case LE_AUDIO_BROADCAST_STOP_FIND_SOURCE:
        {
            uint8_t start = 0;
            le_hs_cli_send_pkt(app_menu_array[input].opcode, &start, sizeof(start));
        }
        break;
    case LE_CONNECT:
        {
            WICED_BT_TRACE("Not Handled \n");
        }
        break;
    case LE_DISCONNECT:
    case LE_AUDIO_PLAY:
    case LE_AUDIO_PAUSE:
    case LE_AUDIO_VOL_UP:
    case LE_AUDIO_VOL_DOWN:
    case LE_AUDIO_MUTE:
    case LE_AUDIO_UNMUTE:
        {
            uint16_t conn_id = le_hs_cli_input_conn_id();
            if (conn_id != INVALID_CONN_ID)
            {
                le_hs_cli_send_pkt(app_menu_array[input].opcode, (uint8_t* ) & conn_id, sizeof(conn_id));
            }
        }
        break;
    case LE_AUDIO_ABS_VOL:
        {
            uint8_t abs_vol = 0;

            uint16_t conn_id = le_hs_cli_input_conn_id();
            if (conn_id == INVALID_CONN_ID)
                break;

            int ret = le_hs_cli_input_abs_vol(&abs_vol);
            if (ret == INVALID_ABS_VOL)
                break;

            uint8_t data[3] = {0};
            uint8_t* p = data;

            UINT16_TO_STREAM(p, conn_id);
            UINT8_TO_STREAM(p, abs_vol);
            const uint32_t payload_len = (uint32_t)(p - data);

            le_hs_cli_send_pkt(app_menu_array[input].opcode, data, payload_len);
        }
        break;
    case LE_AUDIO_BROADCAST_SINK_SYNC_TO_SOURCES:
        {
            uint32_t enter_braodcast_id = 0x01;
            uint8_t data[1 + 4 + 16]; //flag + br_id + br_code
            uint8_t br_code[16] = {0};
            fprintf(stdout, "%s", enter_braodcast_id_str);
            if(input_hex(&enter_braodcast_id) == FALSE)
            {
               break;
            }
            input_chars(br_code, 16);
            uint8_t* p = data;

            UINT8_TO_STREAM(p, 1);
            UINT32_TO_STREAM(p, enter_braodcast_id);
            ARRAY_TO_STREAM(p, br_code, 16);
            TRACE_LOG("br_id:%x", enter_braodcast_id);
            le_hs_cli_send_pkt(app_menu_array[input].opcode, data, sizeof(data));
        }
        break;
    case LE_AUDIO_BROADCAST_SINK_TERMINATE_SYNC:
        {
            broadcast_sink_cb_t *p_cb = lehs_get_broadcast_sink_cb();
            uint8_t index;
            uint8_t data[5];
            uint8_t* p = data;
            for (size_t i = 0; i < MAX_BIG; i++)
            {
                WICED_BT_TRACE("Index %d Broadcast ID %d in use:%d \n", i, p_cb[i].base.broadcast_id, p_cb[i].in_use);
            }
            WICED_BT_TRACE("Enter Index :\n");
            if ((scanf("%hhd", &index) == EOF) || (index >= MAX_BIG))
            {
                WICED_BT_TRACE("Invalid Index \n");
                break;
            }
            UINT8_TO_STREAM(p, 0);
            UINT32_TO_STREAM(p, p_cb[index].base.broadcast_id);
            le_hs_cli_send_pkt(app_menu_array[input].opcode, data, sizeof(data));
        }
        break;
    case LE_AUDIO_BROADCAST_GET_INFO:
        TRACE_LOG("Not support");
        break;
    case LE_AUDIO_ACCEPT_CALL:
        TRACE_LOG("Not support");
        break;
    case LE_AUDIO_HOLD_CALL:
        TRACE_LOG("Not support");
        break;
    case LE_AUDIO_RETRIVE_CALL:
        TRACE_LOG("Not support");
        break;
    case LE_AUDIO_ORIGINATE_CALL:
        TRACE_LOG("Not support");
        break;
    case LE_AUDIO_JOIN_CALL:
        TRACE_LOG("Not support");
        break;
    case LE_AUDIO_TERMINATE_CALL:
        TRACE_LOG("Not support");
        break;
    case LE_AUDIO_MICP_MUTE:
        TRACE_LOG("Not support");
        break;
    case LE_AUDIO_MICP_AICS_MUTE:
        TRACE_LOG("Not support");
        break;
    case LE_AUDIO_MICP_AICS_SET_GAIN:
        TRACE_LOG("Not support");
        break;
    default:
             WICED_BT_TRACE("Input error!!\n");
             break;
    }
    return 1;
}

uint8_t app_init_csis_by_cfg_file( char *file_name )
{
    uint32_t read_len       = 0;
    unsigned char buf[50]   = {0};
    uint8_t size            = 0;
    uint8_t rank            = 0;
    uint8_t location        = 0;
    FILE * file_fd          = NULL;
    uint8_t i               = 0;
    int fileseek            = 0;

    wiced_bt_ga_csis_sirk_data_t sirk = {0};

    sirk.is_oob     = 0;
    sirk.sirk_type  = WICED_BT_GA_CSIS_SIRK_PLAIN;
    //TODO: sirk read from cfg file
    memcpy(&sirk.sirk, default_sirk, sizeof(wiced_bt_ga_csis_sirk_t));

    file_fd = fopen(file_name, "r");
    if (!file_fd)
    {
        return 0;
    }
    while(WICED_TRUE)
    {
        read_len = fread(buf, 1, sizeof(buf), file_fd);

        if(read_len == -1)
        {
            printf("File read error!\n");
            return 0;
        }
        else if(read_len == 0)
        {
            break;
        }
       else
        {
            //TODO: read sirk
            for (uint8_t csis_cfg_idx = 0; csis_cfg_idx < sizeof(csis_cfg)/sizeof(csis_cfg[0]); csis_cfg_idx++)
            {
                char *csis_str = strstr(buf, csis_cfg[csis_cfg_idx].csis_str);
                //printf("csis_str:%s\n", csis_str);     
                char *tmp = strstr(csis_str, "=");       
                //char *tmp = strtok(csis_str, "=");     
                csis_cfg[csis_cfg_idx].csis_value.number = atoi(tmp+1);
                //printf("csis_cfg[csis_cfg_idx].csis_value.number:%d\n", csis_cfg[csis_cfg_idx].csis_value.number);
            }
        }
    }
    fclose(file_fd);

    TRACE_LOG("size:%d, rank:%d, location:%d", csis_cfg[CSIS_SIZE].csis_value.number, csis_cfg[CSIS_RANK].csis_value.number, csis_cfg[CSIS_LOCATION].csis_value.number);

    lehs_csis_set_sirk(&sirk);
    lehs_csis_set_size(csis_cfg[CSIS_SIZE].csis_value.number);
    lehs_csis_set_rank(csis_cfg[CSIS_RANK].csis_value.number);
    lehs_set_audio_location(csis_cfg[CSIS_LOCATION].csis_value.number);
}

/******************************************************************************
 * Function Name: input_hex
 *******************************************************************************
 * Summary:
 *   Get user input hex number
 *
 * Parameters:
 *   int* p_number : pointer to int buffer
 *
 * Return:
 *   bool : get any error from input or not
 *
 ******************************************************************************/
static wiced_bool_t input_hex(int* p_number)
{
    fflush(stdin);
    if(0 == scanf("%x", p_number))
    {
        fprintf(stdout, "Invalid input hex\n");
        return FALSE;
    }
    fflush(stdin);
    while(getchar() != '\n');
    return TRUE;
}
/******************************************************************************
 * Function Name: input_chars
 *******************************************************************************
 * Summary:
 *   Get user input characteristics, enter again if input is invalid
 *
 * Parameters:
 *   char* buf  : pointer to cahr buffer
 *   int size   : buffer size
 * 
 * Return:
 *
 ******************************************************************************/
static void input_chars(uint8_t* buf, int size){
    for(int i = 0, c = 0;;)
    {
        fflush(stdin);
        memset(buf, 0, size);
        fprintf(stdout, "Enter broadcast code (16 bytes): ");
        for(; (c = getchar()) != EOF && c != '\n'; i++)
        {
            if(i < size)
            {
                buf[i] = c;
            }
        }
        if(i <= size)
        {
            return;
        }
        else
        {
            fprintf(stdout, "Invalid input\n");
            i = 0;
        }
    }

}

#endif // CLI_SUPPORT
