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

/*****************************************************************************
 **
 **  Name:          spy_ux.c
 **
 **  Description:   Send traces to BT Spy via socket (Linux)
 **
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "data_types.h"

#include <unistd.h>
#include <errno.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "log.h"


#define INVALID_SOCKET -1
#define MAX_DESC_LEN 50
#define SPY_TCP_SOCKET_PORT_NUM_START 11011
#define SPY_UDP_SOCKET_PORT_NUM_START 9876
#define INVALID_SPY_INSTANCE -1

static int spy_socket_descriptor = INVALID_SOCKET;
static int spy_client_socket = INVALID_SOCKET;
extern char g_peer_ip_addr[];

typedef void (*route_data_to_client_control_t)(BYTE type, BYTE *buffer, uint16_t length, uint8_t spy_instance);
route_data_to_client_control_t g_route_data_to_client_control;

static BOOL32 g_is_TCP = 0;

void TraceHciPkt(BYTE type, BYTE *buffer, UINT16 length, int spy_instance);

void route_hci_data_to_CC_init(route_data_to_client_control_t send_data_to_client_control)
{
    g_route_data_to_client_control = send_data_to_client_control;
}

void set_TCP_enabled(BOOL32 val)
{
    g_is_TCP = val;
}

void detect_peer_device_disconnection(int response)
{
    if (response != -1 || errno != EPIPE) return;

    shutdown(spy_client_socket, SHUT_WR);
    close(spy_client_socket);
    spy_client_socket = INVALID_SOCKET;
    TRACE_LOG("Peer device hung up, Trying to listen for connection.....");
}

void send_data_over_tcp_socket(BYTE type, BYTE *buffer, UINT16 length, UINT8 spy_instance)
{
    struct sockaddr_in socket_addr;
    char description[MAX_DESC_LEN];
    int listen_res;
    int response = -1;

    memset(&socket_addr, 0, sizeof(socket_addr));
    socket_addr.sin_family = AF_INET;
    socket_addr.sin_addr.s_addr = INADDR_ANY;
    socket_addr.sin_port = htons(SPY_TCP_SOCKET_PORT_NUM_START + spy_instance);

    if (spy_socket_descriptor == INVALID_SOCKET)
    {
        spy_socket_descriptor = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (spy_socket_descriptor == INVALID_SOCKET)
        {
            strncpy(description, "listen socket failed , socket thread exiting\n", MAX_DESC_LEN);
            TRACE_ERR("%s\n", description);
            return;
        }

        int err = bind(spy_socket_descriptor, (struct sockaddr *)&socket_addr, sizeof(socket_addr));
        if (err != 0)
        {
            TRACE_ERR("binding... spy_sock_descriptor %d] [%s] \n", spy_socket_descriptor, strerror(errno));
            close(spy_socket_descriptor);
            spy_socket_descriptor = INVALID_SOCKET;
            return;
        }
    }

    if (INVALID_SOCKET == spy_client_socket)
    {
        if (INVALID_SOCKET == (listen_res = listen(spy_socket_descriptor, 1)))
        {
            strncpy(description, "TCP socket listen failed\n", MAX_DESC_LEN);
            TRACE_ERR("%s\n", description);
            return;
        }

        // Accept the client TCP socket
        if (INVALID_SOCKET == (spy_client_socket = accept(spy_socket_descriptor, NULL, NULL)))
        {
            strncpy(description, "TCP socket accept failed\n", MAX_DESC_LEN);
            TRACE_ERR("%s\n", description);
            return;
        }

        strncpy(description, "TCP socket accepted OK !!!\n", MAX_DESC_LEN);
        TRACE_ERR("%s\n", description);
    }

    response = send(spy_client_socket, (const char *)buffer, length, MSG_NOSIGNAL);
    detect_peer_device_disconnection(response);
}

void send_data_over_udp_socket(BYTE type, BYTE *buffer, UINT16 length, UINT8 spy_instance)
{
    static struct sockaddr_in socket_addr, client_addr;


    if (spy_socket_descriptor == INVALID_SOCKET)
    {

        memset(&socket_addr, 0, sizeof(socket_addr));
        socket_addr.sin_family = AF_INET;
        socket_addr.sin_addr.s_addr = INADDR_ANY;
        socket_addr.sin_port = htons(SPY_UDP_SOCKET_PORT_NUM_START + spy_instance);

        memset(&client_addr, 0, sizeof(client_addr));
        client_addr.sin_family = AF_INET;
        client_addr.sin_addr.s_addr = inet_addr(g_peer_ip_addr);
        client_addr.sin_port = htons(SPY_UDP_SOCKET_PORT_NUM_START + spy_instance);

        spy_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (spy_socket_descriptor == INVALID_SOCKET) return;

        int err = bind(spy_socket_descriptor, (struct sockaddr *)&socket_addr, sizeof(socket_addr));
        if (err != 0)
        {
            close(spy_socket_descriptor);
            spy_socket_descriptor = INVALID_SOCKET;
            return;
        }
    }

    int err = sendto(spy_socket_descriptor,
           (const char *)buffer,
           length,
           0,
           (struct sockaddr *)&client_addr,
           sizeof(client_addr));
}

// Send traces to BT Spy via socket
void TraceHciPkt(BYTE type, BYTE *p_buffer, UINT16 length, int spy_instance)
{
    BYTE buf[1100];
    USHORT *p = (USHORT *)buf;
    BYTE *p_data = NULL;

    /* check if spy socket is configured */
    if (spy_instance == INVALID_SPY_INSTANCE) return;

    if ((char)type != -1)
    {
        *p++ = type;
        *p++ = length;
        *p++ = 0;
        *p++ = spy_instance;

        memcpy(p, p_buffer, length);
        length += 8;

        p_data = buf;
    }
    else
    {
        p_data = p_buffer;
    }

    if (length > 1024) length = 1024;

    //send data to client control
    if (g_route_data_to_client_control)
    {
        //g_route_data_to_client_control(type, p_buffer, length, spy_instance);
        //return;
    }

    if (g_is_TCP)
    {
        send_data_over_tcp_socket(type, p_data, length, spy_instance);
    }
    else //UDP
    {
        send_data_over_udp_socket(type, p_data, length, spy_instance);
    }
}
