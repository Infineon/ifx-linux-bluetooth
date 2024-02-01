#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include "sco_process.h"
#include "log.h"


/******************************************************
*           Variable
 *****************************************************/
static wiced_bt_pool_t* sco_pool;
static wiced_bt_lock_t sco_pool_lock;
static pthread_mutex_t sco_pool_mutex = PTHREAD_MUTEX_INITIALIZER;

static wiced_bt_buffer_q_t sco_queue;
static wiced_bt_lock_t sco_qlock;
static pthread_mutex_t sco_mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_t sco_thread;
sem_t sco_sem;

static BOOL32 sco_queue_ready = FALSE;

/******************************************************
*          Function prototypes 
 *****************************************************/
extern void ProcessScoFromHCI(uint8_t *pData, uint32_t length);
extern void ProcessIsocFromHCI (uint8_t *pData, uint32_t length);
static void sco_dequeue(wiced_bt_buffer_t* p_sco_buffer);
static void* scoRcvd (void *p);

/******************************************************
*          Function Def
 *****************************************************/

/*******************************************************************************
 **
 ** Function:   sco_mutex_lock
 **
 ** Description:
 **     linux layer mutex lock for sco pool use
 **
 ** Parametre:
 **     void* p_lock_context
 ** Returns
 **     void
 *******************************************************************************/
static void sco_mutex_lock(void* p_lock_context)
{
    pthread_mutex_lock(p_lock_context);
}

/*******************************************************************************
 **
 ** Function:   sco_mutex_unlock
 **
 ** Description:
 **     linux layer mutex unlock for sco pool use
 **
 ** Parametre:
 **     void* p_lock_context
 ** Returns
 **     void
 *******************************************************************************/
static void sco_mutex_unlock(void* p_lock_context)
{
    pthread_mutex_unlock(p_lock_context);
}

/*******************************************************************************
 **
 ** Function:  init_sco_queue 
 **
 ** Description:
 **     init sco_sem, init sco heap, init sco queue, create sco rcvd thread 
 **     use ENABLE_SCO_QUEUE macro to enable it.
 **     use macro SCO_PKT_NUM to set the size of sco heap pool,
 **     default number is 10 if no define.
 **
 ** Parametre:
 **     void 
 ** Returns
 **     BOOL32: true: init complete
 **             false: init fail 
 *******************************************************************************/
BOOL32 init_sco_queue(void)
{
#ifdef ENABLE_SCO_QUEUE
    TRACE_LOG("init\n");

    sem_init(&sco_sem, 0, 0);

    //init sco heap
    sco_pool_lock.p_lock_context = (void*)&sco_pool_mutex;
    sco_pool_lock.pf_lock_func = sco_mutex_lock;
    sco_pool_lock.pf_unlock_func = sco_mutex_unlock;
    sco_pool = wiced_bt_create_pool("scobuffer", sizeof(tUART_RX), SCO_PKT_NUM, &sco_pool_lock);
    if (sco_pool == NULL)
    {
        TRACE_ERR("wiced_bt_create_pool scobuffer fail\n");
        return FALSE;
    }

    //init sco queue
    sco_qlock.p_lock_context = &sco_mutex;
    sco_qlock.pf_lock_func = sco_mutex_lock;
    sco_qlock.pf_unlock_func = sco_mutex_unlock;
    wiced_bt_init_q(&sco_queue, &sco_qlock);

    pthread_attr_t thread_attr;
    pthread_attr_init(&thread_attr);
    pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);

    sco_queue_ready = TRUE;
    if (pthread_create(&(sco_thread), &thread_attr, scoRcvd, NULL) < 0)
    {
        TRACE_ERR("pthread_create failed\n");
        return FALSE;
    }
    return TRUE;
#endif
    TRACE_LOG("Not enable SCO QUEUE\n");
    return TRUE;
}

/*******************************************************************************
 **
 ** Function:  scoRcvd 
 **
 ** Description:
 **     sco thread, get the sco pkt from sco queue. call the btstack api to process it
 **
 ** Parametre:
 **     void  *p: no use pointer
 ** Returns
 **     void
 *******************************************************************************/
static void* scoRcvd (void *p)
{
    tUART_RX sco_pkt = {0};
    TRACE_LOG("start\n");
    while (sco_queue_ready)
    {
        sco_dequeue(&sco_pkt);

        if (sco_pkt.type == HCIT_TYPE_SCO_DATA)
        {
            ProcessScoFromHCI(sco_pkt.data, sco_pkt.len);
        }
        else if (sco_pkt.type == HCIT_TYPE_ISOC_DATA)
        {
            ProcessIsocFromHCI(sco_pkt.data, sco_pkt.len);
        }
        else if (sco_pkt.type == HCIT_TYPE_WICED_HCI)
        {
            TRACE_ERR("!!!!Dropping WICED SCO Packet Length: %u\n", sco_pkt.len);
        }
        else
        {
            TRACE_ERR("!!!!Unexpected SCO H4 Packet Type: %u  Length: %u\n", sco_pkt.type, sco_pkt.len);
        }
        memset (&sco_pkt, 0, sizeof (tUART_RX));
    }
    TRACE_ERR("sco rcvd thread end\n");
    return NULL;
}

/*******************************************************************************
 **
 ** Function:  sco_enqueue
 **
 ** Description:
 **     sco_queue enqueue, hci_thread get the buffer from rx_queue and if it is pkt of SCO or ISCO
 **     copy the pkt into sco queue to prevent if sco processing too much time and block the hci thread
 **     if hci thread blocking time greater then uart packet input rate, uart will blocking.
 **     if not enough buffer in sco_pool, skip the sco packet
 **
 ** Parametre:
 **     void  *p: no use pointer
 ** Returns
 **     void
 *******************************************************************************/
void sco_enqueue(tUART_RX* p_sco_pkt)
{
    if (wiced_bt_get_pool_free_count(sco_pool) == 0)
    {
        TRACE_LOG("!!! No Free Pool, Skip SCO\n");
        return;
    }

    wiced_bt_buffer_t* p_sco_queue_pkt = wiced_bt_get_buffer_from_pool(sco_pool);

    memcpy(p_sco_queue_pkt, p_sco_pkt, sizeof(tUART_RX));

    wiced_bt_enqueue(&sco_queue, p_sco_queue_pkt);

    sem_post(&sco_sem);
}

/*******************************************************************************
 **
 ** Function:  sco_dequeue
 **
 ** Description:
 **     get the sco packet from sco_queue 
 **     copy the sco packet to the sco_buffer
 **     release the sco packet to sco pool
 **
 ** Parametre:
 **     wiced_bt_buffer_t* p_sco_buffer
 ** Returns
 **     void
 *******************************************************************************/
static void sco_dequeue(wiced_bt_buffer_t* p_sco_buffer)
{
    sem_wait(&sco_sem);

    wiced_bt_buffer_t* p_sco_queue_pkt = wiced_bt_dequeue(&sco_queue);

    memcpy(p_sco_buffer, p_sco_queue_pkt, sizeof(tUART_RX));

    wiced_bt_free_buffer(p_sco_queue_pkt);
}


/*******************************************************************************
 **
 ** Function:  sco_get_queue_ready
 **
 ** Description:
 **     get the status of sco queue init
 **
 ** Parametre:
 **     void 
 ** Returns
 **     BOOL32: 
 **         true
 **         false
 *******************************************************************************/
BOOL32 sco_get_queue_ready(void)
{
    return sco_queue_ready;
}


