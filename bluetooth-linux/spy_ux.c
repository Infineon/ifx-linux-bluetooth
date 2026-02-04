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
#include <stdint.h>

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <pthread.h>
#include <semaphore.h>

#include "data_types.h"

#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "log.h"


#define INVALID_SOCKET -1
#define MAX_DESC_LEN 50
#define SPY_TCP_SOCKET_PORT_NUM_START 11011
#define SPY_UDP_SOCKET_PORT_NUM_START 9876
#define INVALID_SPY_INSTANCE -1
#define MAX_QUEUE_SIZE 100

/* Queue item structure */
typedef struct {
    BYTE *data;
    uint16_t length;
} queue_item_t;

/* Queue structure */
typedef struct {
    queue_item_t items[MAX_QUEUE_SIZE];
    int head;
    int tail;
    int count;
    pthread_mutex_t mutex;
} data_queue_t;

/* Global state structure */
typedef struct {
    int spy_socket_descriptor;
    int spy_client_socket;
    pthread_t tcp_listener_thread;
    int tcp_listener_running;
    data_queue_t tcp_send_queue;
    BOOL32 is_TCP;
} spy_ux_state_t;

static spy_ux_state_t g_spy_state = {
    .spy_socket_descriptor = INVALID_SOCKET,
    .spy_client_socket = INVALID_SOCKET,
    .tcp_listener_thread = 0,
    .tcp_listener_running = 0,
    .tcp_send_queue = {0},
    .is_TCP = 0
};

char description[MAX_DESC_LEN];
extern char g_peer_ip_addr[];

typedef void (*route_data_to_client_control_t)(BYTE type, BYTE *buffer, uint16_t length, uint8_t spy_instance);
route_data_to_client_control_t g_route_data_to_client_control;


void TraceHciPkt(BYTE type, BYTE *buffer, UINT16 length, int spy_instance);

/*
 * Queue management functions
 */

/* Initialize queue */
static void queue_init(data_queue_t *queue)
{
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    pthread_mutex_init(&queue->mutex, NULL);
}

/* Check if queue is full */
static int queue_is_full(data_queue_t *queue)
{
    pthread_mutex_lock(&queue->mutex);
    int is_full = (queue->count >= MAX_QUEUE_SIZE);
    pthread_mutex_unlock(&queue->mutex);
    return is_full;
}

/* Check if queue is empty */
static int queue_is_empty(data_queue_t *queue)
{
    pthread_mutex_lock(&queue->mutex);
    int is_empty = (queue->count == 0);
    pthread_mutex_unlock(&queue->mutex);
    return is_empty;
}

/* Get current queue count */
static int queue_get_count(data_queue_t *queue)
{
    pthread_mutex_lock(&queue->mutex);
    int count = queue->count;
    pthread_mutex_unlock(&queue->mutex);
    return count;
}

/* Enqueue data item */
static int queue_enqueue(data_queue_t *queue, const BYTE *data, uint16_t length)
{
    if (length == 0) {
        return -1; /* Invalid length */
    }

    pthread_mutex_lock(&queue->mutex);

    if (queue->count >= MAX_QUEUE_SIZE) {
        pthread_mutex_unlock(&queue->mutex);
        return -1; /* Queue full */
    }

    /* Allocate memory for the data */
    BYTE *allocated_data = (BYTE *)malloc(length);
    if (allocated_data == NULL) {
        pthread_mutex_unlock(&queue->mutex);
        debug_PrintError("Failed to allocate memory for queue item\n");
        return -1;
    }

    /* Copy data and add item to queue */
    memcpy(allocated_data, data, length);
    queue->items[queue->tail].data = allocated_data;
    queue->items[queue->tail].length = length;
    queue->tail = (queue->tail + 1) % MAX_QUEUE_SIZE;
    queue->count++;

    pthread_mutex_unlock(&queue->mutex);
    return 0;
}

/* Dequeue data item */
static int queue_dequeue(data_queue_t *queue, BYTE **data, uint16_t *length)
{
    pthread_mutex_lock(&queue->mutex);

    if (queue->count == 0) {
        pthread_mutex_unlock(&queue->mutex);
        return -1; /* Queue empty */
    }

    /* Get item from queue */
    *data = queue->items[queue->head].data;
    *length = queue->items[queue->head].length;
    queue->head = (queue->head + 1) % MAX_QUEUE_SIZE;
    queue->count--;

    pthread_mutex_unlock(&queue->mutex);
    return 0;
}

/* Clear entire queue */
static void queue_clear(data_queue_t *queue)
{
    pthread_mutex_lock(&queue->mutex);

    /* Free all allocated memory */
    for (int i = 0; i < queue->count; i++) {
        int idx = (queue->head + i) % MAX_QUEUE_SIZE;
        if (queue->items[idx].data != NULL) {
            free(queue->items[idx].data);
            queue->items[idx].data = NULL;
        }
    }

    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    pthread_mutex_unlock(&queue->mutex);
}

void route_hci_data_to_CC_init(route_data_to_client_control_t send_data_to_client_control)
{
    g_route_data_to_client_control = send_data_to_client_control;

    /* Initialize queue on first use */
    static int queue_initialized = 0;
    if (!queue_initialized) {
        queue_init(&g_spy_state.tcp_send_queue);
        queue_initialized = 1;
    }
}

/*
 * Thread function that waits for TCP socket connections
 * This function runs in a separate thread and listens for incoming TCP connections
 * on the specified port.
 */
static void* tcp_listener_thread_func(void *arg)
{
    struct sockaddr_in socket_addr;
    struct sockaddr_in client_addr;
    socklen_t client_addr_len;
    int listen_res;
    uint8_t spy_instance = (uintptr_t)arg;

    memset(&socket_addr, 0, sizeof(socket_addr));
    socket_addr.sin_family = AF_INET;
    socket_addr.sin_addr.s_addr = INADDR_ANY;
    socket_addr.sin_port = htons(SPY_TCP_SOCKET_PORT_NUM_START + spy_instance);

    TRACE_LOG("Starting TCP listener thread on port %d\n", SPY_TCP_SOCKET_PORT_NUM_START + spy_instance);

    // Create server socket
    g_spy_state.spy_socket_descriptor = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (g_spy_state.spy_socket_descriptor == INVALID_SOCKET)
    {
        TRACE_ERR("Failed to create TCP listener socket\n");
        g_spy_state.tcp_listener_running = 0;
        return NULL;
    }

    // Set socket options to allow reuse
    int opt = 1;
    if (setsockopt(g_spy_state.spy_socket_descriptor, SOL_SOCKET, SO_REUSEADDR,
                   (const char*)&opt, sizeof(opt)) < 0)
    {
        TRACE_ERR("Failed to set socket options\n");
        close(g_spy_state.spy_socket_descriptor);
        g_spy_state.spy_socket_descriptor = INVALID_SOCKET;
        g_spy_state.tcp_listener_running = 0;
        return NULL;
    }

    // Bind socket
    if (bind(g_spy_state.spy_socket_descriptor, (struct sockaddr *)&socket_addr, sizeof(socket_addr)) != 0)
    {
        TRACE_LOG("[Error binding TCP listener socket %d] [%s]\n",
                     g_spy_state.spy_socket_descriptor, strerror(errno));
        close(g_spy_state.spy_socket_descriptor);
        g_spy_state.spy_socket_descriptor = INVALID_SOCKET;
        g_spy_state.tcp_listener_running = 0;
        return NULL;
    }

    // Listen for connections
    if (listen(g_spy_state.spy_socket_descriptor, 1) == INVALID_SOCKET)
    {
        TRACE_ERR("TCP socket listen failed\n");
        close(g_spy_state.spy_socket_descriptor);
        g_spy_state.spy_socket_descriptor = INVALID_SOCKET;
        g_spy_state.tcp_listener_running = 0;
        return NULL;
    }

    snprintf(description, MAX_DESC_LEN, "TCP listener thread started on port %d\n",
             SPY_TCP_SOCKET_PORT_NUM_START + spy_instance);
    TRACE_LOG("%s", description);

    // Accept connections in a loop
    while (g_spy_state.tcp_listener_running)
    {
        client_addr_len = sizeof(client_addr);

        int client_socket = accept(g_spy_state.spy_socket_descriptor,
                                   (struct sockaddr *)&client_addr,
                                   &client_addr_len);
        if (client_socket == INVALID_SOCKET)
        {
            if (g_spy_state.tcp_listener_running)
            {
                debug_PrintError("TCP socket accept failed\n");
            }
            break;
        }

        g_spy_state.spy_client_socket = client_socket;
        snprintf(description, MAX_DESC_LEN, "TCP client connected from %s\n",
                 inet_ntoa(client_addr.sin_addr));
        TRACE_LOG("%s", description);

        /* Drain the queue now that client is connected */
        BYTE *queue_data;
        uint16_t queue_length;
        int drained_count = 0;

        while (queue_dequeue(&g_spy_state.tcp_send_queue, &queue_data, &queue_length) == 0)
        {
            int response = send(g_spy_state.spy_client_socket, (const char *)queue_data, queue_length, MSG_NOSIGNAL);
            free(queue_data);

            if (response < 0)
            {
                debug_PrintError("Failed to send queued data\n");
                break;
            }
            drained_count++;
        }

        if (drained_count > 0)
        {
            snprintf(description, MAX_DESC_LEN, "Sent %d queued items\n", drained_count);
            TRACE_LOG("%s", description);
        }
    }

    // Cleanup
    if (g_spy_state.spy_client_socket != INVALID_SOCKET)
    {
        close(g_spy_state.spy_client_socket);
        g_spy_state.spy_client_socket = INVALID_SOCKET;
    }

    if (g_spy_state.spy_socket_descriptor != INVALID_SOCKET)
    {
        close(g_spy_state.spy_socket_descriptor);
        g_spy_state.spy_socket_descriptor = INVALID_SOCKET;
    }

    TRACE_LOG("TCP listener thread exiting\n");
    g_spy_state.tcp_listener_running = 0;
    return NULL;
}

/*
 * Start TCP listener thread
 * Creates and starts a new thread that waits for TCP socket connections
 * on the specified spy_instance port
 *
 * Parameters:
 *   spy_instance - The spy instance number (determines the port)
 *
 * Returns:
 *   0 on success, -1 on failure
 */
int start_tcp_listener_thread(uint8_t spy_instance)
{
    if (g_spy_state.tcp_listener_running)
    {
        debug_Printf("TCP listener thread already running\n");
        return -1;
    }

    g_spy_state.tcp_listener_running = 1;

    if (pthread_create(&g_spy_state.tcp_listener_thread, NULL, tcp_listener_thread_func,
                       (void *)(uintptr_t)spy_instance) != 0)
    {
        TRACE_ERR("Failed to create TCP listener thread\n");
        g_spy_state.tcp_listener_running = 0;
        return -1;
    }

    return 0;
}

/*
 * Stop TCP listener thread
 * Stops the running TCP listener thread and cleanup resources
 *
 * Returns:
 *   0 on success, -1 on failure
 */
int stop_tcp_listener_thread(void)
{
    if (!g_spy_state.tcp_listener_running || g_spy_state.tcp_listener_thread == 0)
    {
        return -1;
    }

    g_spy_state.tcp_listener_running = 0;

    // Close the listening socket to unblock accept()
    if (g_spy_state.spy_socket_descriptor != INVALID_SOCKET)
    {
        close(g_spy_state.spy_socket_descriptor);
        g_spy_state.spy_socket_descriptor = INVALID_SOCKET;
    }

    // Wait for thread to finish
    if (pthread_join(g_spy_state.tcp_listener_thread, NULL) != 0)
    {
        TRACE_ERR("Failed to join TCP listener thread\n");
        return -1;
    }

    g_spy_state.tcp_listener_thread = 0;
    return 0;
}

void set_TCP_enabled(BOOL32 val)
{
    extern int wicedx_emulator_instance;
    g_spy_state.is_TCP = val;
    start_tcp_listener_thread(wicedx_emulator_instance);

    // Waiting for TCP client to establish connection, before bt stack startup.
    while(g_spy_state.tcp_listener_thread && g_spy_state.spy_client_socket == INVALID_SOCKET){
        // sleep for avoid busy waiting.
        usleep(10*1000);
    }
}

void detect_peer_device_disconnection(int response)
{
    if ((response == -1) && (errno == EPIPE))
    {
        shutdown(g_spy_state.spy_client_socket, SHUT_WR);
        close(g_spy_state.spy_client_socket);
        g_spy_state.spy_client_socket = INVALID_SOCKET;
        TRACE_ERR("Peer device hung up, Trying to listen for connection.....");
    }
}

void send_data_over_tcp_socket(BYTE type, BYTE *buffer, UINT16 length, UINT8 spy_instance)
{
    int response;

    /* If client is not connected, queue the data */
    if (g_spy_state.spy_client_socket == INVALID_SOCKET)
    {
        if (queue_is_full(&g_spy_state.tcp_send_queue))
        {
            //printf("TCP send queue is full, dropping data\n");
            return;
        }

        if (queue_enqueue(&g_spy_state.tcp_send_queue, buffer, length) != 0)
        {
            strncpy(description, "Failed to enqueue TCP data\n", MAX_DESC_LEN);
            TRACE_ERR("%s\n", description);
            return;
        }
        // for debug purpose
        //int queue_count = queue_get_count(&g_spy_state.tcp_send_queue);
        //printf("Data queued. Queue size: %d/%d\n", queue_count, MAX_QUEUE_SIZE);
        return;
    }

    /* Client is connected, send data directly */
    response = send(g_spy_state.spy_client_socket, (const char *)buffer, length, MSG_NOSIGNAL);
    // log for debug purpose
    //printf("response = 0x%x\n", response);
    detect_peer_device_disconnection(response);
}

void send_data_over_udp_socket(BYTE type, BYTE *buffer, UINT16 length, UINT8 spy_instance)
{
    static struct sockaddr_in socket_addr, client_addr;

    if (g_spy_state.spy_socket_descriptor == INVALID_SOCKET)
    {

        memset(&socket_addr, 0, sizeof(socket_addr));
        socket_addr.sin_family = AF_INET;
        socket_addr.sin_addr.s_addr = INADDR_ANY;
        socket_addr.sin_port = htons(SPY_UDP_SOCKET_PORT_NUM_START + spy_instance);

        memset(&client_addr, 0, sizeof(client_addr));
        client_addr.sin_family = AF_INET;
        client_addr.sin_addr.s_addr = inet_addr(g_peer_ip_addr);
        client_addr.sin_port = htons(SPY_UDP_SOCKET_PORT_NUM_START + spy_instance);

        g_spy_state.spy_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (g_spy_state.spy_socket_descriptor == INVALID_SOCKET) return;

        int err = bind(g_spy_state.spy_socket_descriptor, (struct sockaddr *)&socket_addr, sizeof(socket_addr));
        if (err != 0)
        {
            close(g_spy_state.spy_socket_descriptor);
            g_spy_state.spy_socket_descriptor = INVALID_SOCKET;
            return;
        }
    }

    sendto(g_spy_state.spy_socket_descriptor,
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
    static int serial_num = 0;

    /* check if spy socket is configured */
    if (spy_instance == INVALID_SPY_INSTANCE){
        return;
    }

    // log for debug purpose
    //printf("[%s] len %d\n",__FUNCTION__,length);

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
        USHORT *p = (USHORT *)p_buffer;
        //printf("[%s] type = %d %04x %04x %04x %04x", __FUNCTION__,type, p[0], p[1], p[2], p[3]);
        p_data = p_buffer;
    }

    if (length > 1024){
        length = 1024;
    }

    //send data to client control
    if (g_route_data_to_client_control)
    {
        //g_route_data_to_client_control(type, p_buffer, length, spy_instance);
        //return;
    }

    if (g_spy_state.is_TCP)
    {
        send_data_over_tcp_socket(type, p_data, length, spy_instance);
    }
    else //UDP
    {
        send_data_over_udp_socket(type, p_data, length, spy_instance);
    }
}
