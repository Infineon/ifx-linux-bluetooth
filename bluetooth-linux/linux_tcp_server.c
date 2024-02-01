/*
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#include <stdio.h>
#include"wiced_result.h"
#include"wiced_memory.h"
#include"wiced_bt_trace.h"
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include<pthread.h>
#include <stddef.h>
#include"wiced_bt_types.h"

#define INVALID_SOCKET -1
#define MPAF_TRAN_PKT_TYPE 25
#define PKT_BUFF_SIZE 1500

static int  m_ClientSocket = INVALID_SOCKET;
static int SOCK_PORT_NUM[] = {12012, 12012, 12013};

static int readHostTCPpkt(unsigned char* pPkt);
static void* rpcReceiveThread(void* p);
extern uint32_t hci_control_proc_rx_cmd(uint8_t *p_buffer, uint32_t length);

extern int wicedx_emulator_instance;

/*
* Note: Using above genric pools max supportable payload size = 252(16 bytes for internal headers).
* @param[in]    code                   :Group codeand command code
* @param[in]    p_data                :Pointer to the payload
* @param[in]    length                 :Payload length
*
* @return   wiced_result_t
*/
wiced_result_t wiced_transport_send_data(uint16_t type, uint8_t* p_data, uint16_t data_size)
{
    uint8_t     buf[1500];

    buf[0] = MPAF_TRAN_PKT_TYPE;
    buf[1] = (uint8_t)type;
    buf[2] = (uint8_t)(type >> 8);
    buf[3] = (uint8_t)data_size;
    buf[4] = (uint8_t)(data_size >> 8);

    // copy received data to transport buffer
    memcpy(&buf[5], p_data, data_size);

    if (m_ClientSocket == INVALID_SOCKET)
    {
        WICED_BT_TRACE_CRIT("!!!!  hci_control_send_script_event() - no TCP socket - dropping data !!!");
        return (-1);
    }

    if (-1 == (send(m_ClientSocket, buf, data_size + 5, 0)))
    {
        WICED_BT_TRACE_CRIT("send failed\n");
        return WICED_INVALID_SOCKET;
    }

    return WICED_SUCCESS;
}

void InitializeRpc(void)
{
    pthread_t thread_address;

    // Create a thread to read HCI packets from the host via a TCP socket. Give it time to start.
    if (pthread_create(&thread_address, NULL, rpcReceiveThread, NULL) < 0)
    {
        WICED_BT_TRACE_CRIT("pthread_create for rpcReceiveThread failed\n");
    }
}

static void *rpcReceiveThread(void *p)
{
    uint8_t pkt[PKT_BUFF_SIZE];
    struct sockaddr_in service;
    int bytes_rcvd;
    int error = 0;
    uint16_t paylen;
    int rpc_socket_descriptor = INVALID_SOCKET;

    WICED_BT_TRACE("[%s] \n", __FUNCTION__);

    // Create a local SOCKET for incoming connection
    if (INVALID_SOCKET == (rpc_socket_descriptor = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)))
    {
        WICED_BT_TRACE_CRIT("listen socket failed - socket thread exiting\n");
        return 0;
    }

    memset(&service, 0, sizeof(service));
    service.sin_family = AF_INET;
    service.sin_addr.s_addr = INADDR_ANY;
    service.sin_port = htons(SOCK_PORT_NUM[1]);

    error = bind(rpc_socket_descriptor, (struct sockaddr *)&service, sizeof(service));
    if (error < 0)
    {
        WICED_BT_TRACE_CRIT("bind failed %d- socket thread exiting\n", error);
        close(rpc_socket_descriptor);
        rpc_socket_descriptor = INVALID_SOCKET;
        return 0;
    }

    for (;;)
    {
        WICED_BT_TRACE("[%s] [TCP Socket] [Protobuf] Listening for Client to connect ...\n", __FUNCTION__);

        if ((error = listen(rpc_socket_descriptor, 1)) != 0)
        {
            WICED_BT_TRACE_CRIT("TCP socket listen failed with error: %d\n", error);
            break;
        }

        // Accept the client TCP socket
        if (INVALID_SOCKET == (m_ClientSocket = accept(rpc_socket_descriptor, NULL, NULL)))
        {
            WICED_BT_TRACE_CRIT("Client TCP socket accept failed with error");
            break;
        }

        WICED_BT_TRACE("[%s] [TCP Socket] [Protobuf] Accepted Client connection\n", __FUNCTION__);

        // Receive until the peer shuts down the connection
        for (;;)
        {
            bytes_rcvd = readHostTCPpkt((unsigned char *)&pkt);
            if (bytes_rcvd <= 0)
            {
                WICED_BT_TRACE("[%s] [TCP Socket] [Protobuf] Receive failed.. Closing socket\n", __FUNCTION__);
                close(m_ClientSocket);
                m_ClientSocket = INVALID_SOCKET;
                break;
            }

            paylen = pkt[3] | (pkt[4] << 8);

            hci_control_proc_rx_cmd(&pkt[1], paylen);
        }
    }

    // sockets will get closed when program exits
    WICED_BT_TRACE("[%s] [TCP Socket] [Protobuf] Exiting Socket thread\n", __FUNCTION__);

    return 0;
}

static int readHostTCPpkt(unsigned char* pPkt)
{
    unsigned int readLen, hdrLen, dataLen;

    if ((readLen = recv(m_ClientSocket, (char*)pPkt, 1, 0)) != 1)
    {
        WICED_BT_TRACE_CRIT("readHostTCPpkt() Expected 1, got: %d", readLen);
        return (-1);
    }

    WICED_BT_TRACE("RX Packet = %d", pPkt[0]);

    // ACL and WICED-HCI share the same basic format
    if (pPkt[0] == MPAF_TRAN_PKT_TYPE)
    {
        if ((hdrLen = recv(m_ClientSocket, (char*)&pPkt[1], 4, 0)) != 4)
        {
            WICED_BT_TRACE_CRIT("readHostTCPpkt() Expected 4, got: %d", readLen);
            return (-1);
        }
        dataLen = pPkt[3] | (pPkt[4] << 8);
    }
    else
    {
        WICED_BT_TRACE_CRIT("!!!!Unknown Type: %u", pPkt[0]);
        return (-1);
    }

    if (dataLen != 0)
    {
        if ((readLen = recv(m_ClientSocket, (char*)&pPkt[1 + hdrLen], PKT_BUFF_SIZE - hdrLen - 1, 0)) != dataLen)
        {
            WICED_BT_TRACE_CRIT("readHostTCPpkt() Expected to read datalen of %u, actually got: %d", dataLen, readLen);
            return (-1);
        }
    }

    return (1 + hdrLen + dataLen);
}


wiced_result_t hci_control_send_watch_event(int type, uint8_t* p_data, uint16_t data_size)
{
    uint8_t     buf[1500];

    buf[0] = MPAF_TRAN_PKT_TYPE;
    buf[1] = (uint8_t)type;
    buf[2] = (uint8_t)(type >> 8);
    buf[3] = (uint8_t)data_size;
    buf[4] = (uint8_t)(data_size >> 8);

    // copy received data to transport buffer
    memcpy(&buf[5], p_data, data_size);

    if (m_ClientSocket == INVALID_SOCKET)
    {
        WICED_BT_TRACE_CRIT("!!!!  hci_control_send_script_event() - no TCP socket - dropping data !!!");
        return (-1);
    }

    if (-1 == (send(m_ClientSocket, buf, data_size + 5, 0)))
    {
        WICED_BT_TRACE_CRIT("send failed\n");
        return WICED_INVALID_SOCKET;
    }

    return WICED_SUCCESS;
}

wiced_result_t hci_control_send_script_event(int type, uint8_t* p_data, uint16_t data_size)
{
    uint8_t     buf[1500];

    //WICED_BT_TRACE_ARRAY(p_data, data_size, "[hci_control_send_script_event]:")

    buf[0] = MPAF_TRAN_PKT_TYPE;
    buf[1] = (uint8_t)type;
    buf[2] = (uint8_t)(type >> 8);
    buf[3] = (uint8_t)data_size;
    buf[4] = (uint8_t)(data_size >> 8);

    // copy received data to transport buffer
    memcpy(&buf[5], p_data, data_size);

    if (m_ClientSocket == INVALID_SOCKET)
    {
        WICED_BT_TRACE_CRIT("!!!!  hci_control_send_script_event() - no TCP socket - dropping data !!!");
        return (-1);
    }

    if (-1 == (send(m_ClientSocket, buf, data_size + 5, 0)))
    {
        WICED_BT_TRACE_CRIT("send failed\n");
        return WICED_INVALID_SOCKET;
    }

    return WICED_SUCCESS;
}
