/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company)
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @file
 *
 * Handsfree AT command response interpreter.
 */

#include <string.h>
#include "wiced_bt_hfp_hf_int.h"
#include "wiced_memory.h"
/*****************************************************************************
**  Constants
*****************************************************************************/
/* Maximum number of AT command retransmissions. */
#define WICED_BT_HFP_HF_AT_CMD_RETRANS_MAX_NUM  2

/* AT command retransmission timeout. */
#define WICED_BT_HFP_HF_AT_CMD_RETRANS_TIMEOUT  1000    // 1 second

/*****************************************************************************
**  External Definitions
*****************************************************************************/
/* AT command interpreter states */
enum
{
    WICED_BT_HFP_HF_PARSE_INIT_ST, /* Init */
    WICED_BT_HFP_HF_PARSE_CMD_ST,  /* Processing command */
    WICED_BT_HFP_HF_PARSE_ERR_ST   /* Error */
};

/* AT command retransmission info. */
typedef struct
{
    uint8_t         num;
}wiced_bt_hfp_hf_at_retrans_info_t;

static wiced_bt_hfp_hf_at_retrans_info_t wiced_bt_hfp_hf_at_retrans_info = {0};

static void wiced_bt_hfp_hf_at_cmd_retrans_info_reset(void);

/*****************************************************************************
** Function         wiced_bt_hfp_hf_at_init
** Description      Initialize the AT command parser control block.
** Returns          void
*****************************************************************************/
void wiced_bt_hfp_hf_at_init(wiced_bt_hfp_hf_at_cb_t *p_cb)
{
    p_cb->p_res_buf = NULL;
    p_cb->res_pos = 0;
    p_cb->state = WICED_BT_HFP_HF_PARSE_INIT_ST;

    wiced_bt_hfp_hf_at_cmd_retrans_info_reset();
}

/*****************************************************************************
** Function         wiced_bt_hfp_hf_at_reinit
** Description      Re-initialize the AT command parser control block.  This
**                  function resets the AT command parser state and frees
**                  any GKI buffer.
** Returns          void
*****************************************************************************/
void wiced_bt_hfp_hf_at_reinit(wiced_bt_hfp_hf_at_cb_t *p_cb)
{
    if (p_cb->p_res_buf != NULL)
    {
        wiced_bt_free_buffer(p_cb->p_res_buf);
        p_cb->p_res_buf = NULL;
    }
    p_cb->res_pos = 0;
    p_cb->state = WICED_BT_HFP_HF_PARSE_INIT_ST;

    wiced_bt_hfp_hf_at_cmd_retrans_info_reset();
}

/*****************************************************************************
** Function         wiced_bt_hfp_hf_at_parse
** Description      Parse AT commands.  This function will take the input
**                  character string and parse it for AT commands according to
**                  the AT command table passed in the control block.
** Returns          BOOLEAN TRUE if done with string. Flase if waiting for new data
*****************************************************************************/
wiced_bool_t wiced_bt_hfp_hf_at_parse(wiced_bt_hfp_hf_at_cb_t *p_cb,
    char *p_buf, uint16_t len)
{
    int           i;
    uint16_t      idx;
    wiced_bool_t  found_return = FALSE;
    char         *p_arg;

    for(i = 0; i < len-1;)
    {
        switch(p_cb->state)
        {
            case WICED_BT_HFP_HF_PARSE_INIT_ST:
                /* Scan through looking for '\n' */
                for (; ((i<len) && (p_buf[i] < ' ')); i++);

                if(i < len-1)
                {
                    p_cb->state = WICED_BT_HFP_HF_PARSE_CMD_ST;
                    p_cb->p_res_buf = (char *) wiced_bt_get_buffer(p_cb->res_max_len);
                }
                break;


            case WICED_BT_HFP_HF_PARSE_CMD_ST:
                /* Scan through, copying string looking for return */
                while ((i < len) && (p_cb->res_pos < p_cb->res_max_len))
                {
                    if ((p_cb->p_res_buf[p_cb->res_pos] = p_buf[i++]) == '\r')
                    {
                        p_cb->p_res_buf[p_cb->res_pos] = 0;
                        found_return = TRUE;
                        break;
                    }
                    p_cb->res_pos++;
                }

                /* If we found return, we're done, parse string */
                if (found_return)
                {
                    WICED_BTHFP_TRACE("%s: rcvd AT cmd: %s\n",
                        __FUNCTION__, p_cb->p_res_buf);
                    found_return = FALSE;

                    /* Loop through at command table looking for match */
                    for (idx = 0; wiced_bt_hfp_hf_res[idx].p_res[0] != 0; idx++)
                    {
                        if (!wiced_bt_hfp_hf_utils_strucmp(wiced_bt_hfp_hf_res[idx].p_res,
                                p_cb->p_res_buf))
                        {
                            p_arg = p_cb->p_res_buf + strlen(wiced_bt_hfp_hf_res[idx].p_res);
                            wiced_bt_hfp_hf_at_cback(p_cb->p_user, idx, p_arg);
                            wiced_bt_hfp_hf_at_reinit(p_cb);
                            break;
                        }
                    }
                    /* If no match call error callback */
                    if (wiced_bt_hfp_hf_res[idx].p_res[0] == 0)
                    {
                        /* Go to idle if this is an unknown AT command */
                        wiced_bt_hfp_hf_at_err_cback(p_cb->p_user, TRUE, p_cb->p_res_buf);
                        wiced_bt_hfp_hf_at_reinit(p_cb);
                    }
                }
                /* Else if string too long skip command */
                else if (p_cb->res_pos == p_cb->res_max_len)
                {
                    WICED_BTHFP_ERROR("%s: AT Parser reached max length: %d\n",
                        __FUNCTION__, p_cb->res_max_len );
                    wiced_bt_hfp_hf_at_reinit(p_cb);
                    return FALSE;
                }
                break;

            case WICED_BT_HFP_HF_PARSE_ERR_ST:
                /* Skip data until we get a return */
                for(; (i < len) && (p_buf[i] != '\n'); i++);

                if(i < len)
                {
                    wiced_bt_hfp_hf_at_reinit(p_cb);
                    i++;
                }
                break;
        }
    }

    return TRUE;
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_at_timer_cback
** Description      Handsfree timer callback.
*******************************************************************************/
void wiced_bt_hfp_hf_at_timer_cback(void * params)
{
    if (wiced_bt_hfp_hf_cb.scb[0].in_use == TRUE)
        wiced_bt_hfp_hf_at_cmd_queue_timeout(&wiced_bt_hfp_hf_cb.scb[0]);
    else if ((WICED_BT_HFP_HF_MAX_CONN > 1) && (wiced_bt_hfp_hf_cb.scb[1].in_use == TRUE))
        wiced_bt_hfp_hf_at_cmd_queue_timeout(&wiced_bt_hfp_hf_cb.scb[1]);
}

/******************************************************************************
** Function         wiced_bt_hfp_hf_at_cmd_queue_init
** Description
******************************************************************************/
void wiced_bt_hfp_hf_at_cmd_queue_init(wiced_bt_hfp_hf_scb_t *p_scb)
{
    wiced_result_t result;

    WICED_BTHFP_TRACE("%s: AT command send queue initialized\n", __FUNCTION__);
    wiced_bt_init_q (&p_scb->wiced_bt_hfp_hf_at_cmd_queue, NULL);
    p_scb->wiced_bt_hfp_hf_at_cmd_queue_depth = 0;
    result = wiced_init_timer (&p_scb->wiced_bt_hfp_hf_at_cmd_queue_timer, wiced_bt_hfp_hf_at_timer_cback, p_scb, WICED_MILLI_SECONDS_TIMER);

    if (result != WICED_BT_SUCCESS)
    {
        WICED_BTHFP_ERROR("wiced_init_timer fail (%d)\n", result);
    }
}

/******************************************************************************
** Function         wiced_bt_hfp_hf_at_cmd_queue_free
** Description      Function to delete all array elements + array list
******************************************************************************/
void wiced_bt_hfp_hf_at_cmd_queue_free(wiced_bt_hfp_hf_scb_t *p_scb)
{
	wiced_bt_hfp_hf_api_send_at_cmd_t *p_buf = NULL;

    wiced_stop_timer(&p_scb->wiced_bt_hfp_hf_at_cmd_queue_timer);

    while((p_buf = (wiced_bt_hfp_hf_api_send_at_cmd_t *)wiced_bt_dequeue(&p_scb->wiced_bt_hfp_hf_at_cmd_queue)) != NULL)
    {
        wiced_bt_free_buffer(p_buf);
    }

    p_scb->wiced_bt_hfp_hf_at_cmd_queue_depth = 0;
    WICED_BTHFP_TRACE("%s: AT command send queue freed\n", __FUNCTION__);
}

/******************************************************************************
** Function         wiced_bt_hfp_hf_at_cmd_queue_enqueue
** Description
******************************************************************************/
void wiced_bt_hfp_hf_at_cmd_queue_enqueue(wiced_bt_hfp_hf_scb_t *p_scb,
    uint8_t cmd, char *buf, uint16_t len)
{
    wiced_bt_hfp_hf_api_send_at_cmd_t *p_data = (wiced_bt_hfp_hf_api_send_at_cmd_t *)wiced_bt_get_buffer((uint16_t)(len+sizeof(wiced_bt_hfp_hf_api_send_at_cmd_t)));

    if(p_data != NULL)
    {
        p_data->len = len;
        p_data->cmd_id = cmd;
        memcpy((uint8_t *)(p_data->at_cmd), buf, len);
        wiced_bt_enqueue(&p_scb->wiced_bt_hfp_hf_at_cmd_queue, (wiced_bt_buffer_t *)p_data);
        p_scb->wiced_bt_hfp_hf_at_cmd_queue_depth++;

        /*send if 1st (and only) command in the queue*/
        if(p_scb->wiced_bt_hfp_hf_at_cmd_queue_depth == 1)
        {
            wiced_bt_hfp_hf_at_cmd_queue_send(p_scb);
        }
    }
    else
    {
        WICED_BTHFP_ERROR("%s: No GKI buffer for cmd %s\n", __FUNCTION__, buf);
    }
}

/******************************************************************************
** Function         wiced_bt_hfp_hf_cmd_queue_handle_res
** Description      If OK/ERROR, returns the command id that this ok/error belongs to
******************************************************************************/
uint8_t wiced_bt_hfp_hf_at_cmd_queue_handle_res(wiced_bt_hfp_hf_scb_t *p_scb)
{
	wiced_bt_hfp_hf_api_send_at_cmd_t *p_data;
    uint8_t at_cmd_id = 0xFF;

    wiced_stop_timer(&p_scb->wiced_bt_hfp_hf_at_cmd_queue_timer);

    /* Reset AT command retransmission info. */
    wiced_bt_hfp_hf_at_cmd_retrans_info_reset();

    p_data = (wiced_bt_hfp_hf_api_send_at_cmd_t *)wiced_bt_dequeue(&p_scb->wiced_bt_hfp_hf_at_cmd_queue);

    if(p_data)
    {
        p_scb->wiced_bt_hfp_hf_at_cmd_queue_depth--;

        /* api_cmd_id, not at_cmd_id, should be passed to the application. */
        at_cmd_id = (uint8_t)p_data->cmd_id;

        wiced_bt_free_buffer(p_data);
    }
    else
    {
        WICED_BTHFP_TRACE("%s: Received AT response, but no AT command pending\n",
            __FUNCTION__);
    }

    if (!wiced_bt_queue_is_empty(&p_scb->wiced_bt_hfp_hf_at_cmd_queue))
        wiced_bt_hfp_hf_at_cmd_queue_send(p_scb);

    return at_cmd_id;
}

/******************************************************************************
** Function         wiced_bt_hfp_hf_at_cmd_queue_flush
** Description
******************************************************************************/
void wiced_bt_hfp_hf_at_cmd_queue_flush(wiced_bt_hfp_hf_scb_t *p_scb)
{
    wiced_bt_hfp_hf_api_send_at_cmd_t *p_buf = NULL;

    wiced_stop_timer( &p_scb->wiced_bt_hfp_hf_at_cmd_queue_timer );

    while((p_buf = (wiced_bt_hfp_hf_api_send_at_cmd_t *)wiced_bt_dequeue(&p_scb->wiced_bt_hfp_hf_at_cmd_queue)) != NULL)
    {
        wiced_bt_free_buffer(p_buf);
    }

    p_scb->wiced_bt_hfp_hf_at_cmd_queue_depth = 0;
    WICED_BTHFP_TRACE("%s: AT command send queue flushed\n", __FUNCTION__);
}

/******************************************************************************
** Function         wiced_bt_hfp_hf_at_cmd_queue_send
** Description
******************************************************************************/
void wiced_bt_hfp_hf_at_cmd_queue_send(wiced_bt_hfp_hf_scb_t *p_scb)
{
    wiced_bt_hfp_hf_api_send_at_cmd_t   *p_buf = (wiced_bt_hfp_hf_api_send_at_cmd_t *)wiced_bt_getfirst(&p_scb->wiced_bt_hfp_hf_at_cmd_queue);
    wiced_bt_rfcomm_result_t result;

    if(p_buf == NULL)
    {
        WICED_BTHFP_ERROR("%s NULL SCB\n", __FUNCTION__);
        return;
    }

    /* Send to RFCOMM */
    result = wiced_bt_rfcomm_write_data(p_scb->rfcomm_handle, p_buf->at_cmd, p_buf->len);

    WICED_BTHFP_TRACE("%s: Sending AT cmd: %s (%d)\n", __FUNCTION__, p_buf->at_cmd,result);

    if (result == WICED_BT_RFCOMM_SUCCESS)
    {
        wiced_start_timer(&p_scb->wiced_bt_hfp_hf_at_cmd_queue_timer,
                          WICED_BT_HFP_HF_CMD_TIMEOUT_VALUE);
    }
    else
    {
        wiced_start_timer(&p_scb->wiced_bt_hfp_hf_at_cmd_queue_timer,
                          WICED_BT_HFP_HF_AT_CMD_RETRANS_TIMEOUT);
    }
}

/******************************************************************************
** Function         wiced_bt_hfp_hf_at_cmd_queue_timeout
** Description
******************************************************************************/
void wiced_bt_hfp_hf_at_cmd_queue_timeout(wiced_bt_hfp_hf_scb_t *p_scb)
{
    wiced_bt_hfp_hf_api_send_at_cmd_t *p_data;

    WICED_BTHFP_TRACE("%s: No response received for previous AT command (%d)\n",
                      __FUNCTION__,
                      wiced_bt_hfp_hf_at_retrans_info.num);

    /* Check if the AT command queue is empty. */
    if (wiced_bt_queue_is_empty(&p_scb->wiced_bt_hfp_hf_at_cmd_queue))
    {   // AT command queue is empty.
        /* Reset At command retransmission info. and discard the operation.*/
        wiced_bt_hfp_hf_at_cmd_retrans_info_reset();

        return;
    }

    /* Check if the retransmission exceeds the maximum number. */
    if (wiced_bt_hfp_hf_at_retrans_info.num >= WICED_BT_HFP_HF_AT_CMD_RETRANS_MAX_NUM)
    {   // Exceed the maximum retransmission number.
        /* Discard the AT command that cannot be transmitted. */
        p_data = (wiced_bt_hfp_hf_api_send_at_cmd_t *)wiced_bt_dequeue(&p_scb->wiced_bt_hfp_hf_at_cmd_queue);

        if(p_data)
        {
            p_scb->wiced_bt_hfp_hf_at_cmd_queue_depth--;
            wiced_bt_free_buffer(p_data);
        }

        /* Reset At command retransmission info..*/
        wiced_bt_hfp_hf_at_cmd_retrans_info_reset();

        /* Transmit next AT command if there is. */
        if (!wiced_bt_queue_is_empty(&p_scb->wiced_bt_hfp_hf_at_cmd_queue))
        {
            wiced_bt_hfp_hf_at_cmd_queue_send(p_scb);
        }
    }
    else
    {
        /* Increment retransmission number. */
        wiced_bt_hfp_hf_at_retrans_info.num++;

        /* Try to transmit the AT command again. */
        wiced_bt_hfp_hf_at_cmd_queue_send(p_scb);
    }
}

/******************************************************************************
** Function         wiced_bt_hfp_hf_at_cmd_flush_cmd_in_queue
** Description
******************************************************************************/
void wiced_bt_hfp_hf_at_cmd_flush_cmd_in_queue(wiced_bt_hfp_hf_scb_t *p_scb, uint8_t cmd)
{
    /* Ignore the first element, as we might have already sent it over the air */
    wiced_bt_hfp_hf_api_send_at_cmd_t *p_buf = (wiced_bt_hfp_hf_api_send_at_cmd_t *)wiced_bt_getfirst(&p_scb->wiced_bt_hfp_hf_at_cmd_queue);


    while(p_buf)
    {
        p_buf = (wiced_bt_hfp_hf_api_send_at_cmd_t *)wiced_bt_getnext(p_buf);
        if(p_buf)
        {
            if(cmd == p_buf->cmd_id)
            {
                WICED_BTHFP_TRACE("Removing AT cmd %d from queue\n", cmd);
                wiced_bt_remove_from_queue(&p_scb->wiced_bt_hfp_hf_at_cmd_queue, p_buf);
                p_scb->wiced_bt_hfp_hf_at_cmd_queue_depth--;
                wiced_bt_free_buffer(p_buf);
                return;
            }
        }
    }

    return;
}

/******************************************************************************
** Function         wiced_bt_hfp_hf_at_cmd_retrans_info_reset
** Description      Reset the AT command retransmission info.
******************************************************************************/
static void wiced_bt_hfp_hf_at_cmd_retrans_info_reset(void)
{
    memset((void *) &wiced_bt_hfp_hf_at_retrans_info,
           0,
           sizeof(wiced_bt_hfp_hf_at_retrans_info_t));
}
