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
 * File Name: bt_decode_queue.h
 *
 * Description: This is the header file for bt a2dp sink decode queue structure
 *              and function, using linking list to implement queue.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

#ifndef __BT_DECCODE_H__
#define __BT_DECCODE_H__

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
*                                 MACROS
*******************************************************************************/

typedef struct node_t node_t;
typedef struct queue_t queue_t;

struct node_t{
    uint8_t *node_data;
    uint16_t node_len;
    node_t *prev;
    node_t *next;
};

struct queue_t{
    node_t *head;
    node_t *tail;
    uint32_t count;
};

/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/

/*****************************************************************************
* Function Name: queue_create
******************************************************************************
* Summary: create queue funciton, use this function to create a new 
*          queue_t structure. 
*     
*
* Parameters:  none
*   
*
* Return:  the pointer of queue_t 
*   
*
****************************************************************************/
queue_t *queue_create(void);


/*****************************************************************************
* Function Name: queue_delete
******************************************************************************
* Summary: delete queue funciton, use this function to delete a 
*          queue_t structure and release the memory
*     
*
* Parameters: 
*   const queue_t *queue:
*         the queue_t pointer to the structure that you want to delete
*   
*
* Return:  none
*   
*
****************************************************************************/
void queue_delete(queue_t* queue);

/*****************************************************************************
* Function Name: queue_is_empty
******************************************************************************
* Summary: the funtion to check the queue is empty or not
*          
*     
*
* Parameters: 
*   const queue_t *queue:
*         the queue_t pointer to the structure that you want to check
*   
*
* Return:  bool: 
*               true: is empty
*               false: not empty
*   
*
****************************************************************************/
bool queue_is_empty(const queue_t *queue);

/*****************************************************************************
* Function Name: queue_dequeue
******************************************************************************
* Summary: the dequeue funtion, return a node_t pointer, the node_t will be removed
*          from queue, and remember to release (free()) the node_data in node_t 
*          
*     
*
* Parameters: 
*   const queue_t *queue:
*         the queue_t pointer to the structure that you want to dequeue
*   
*
* Return:  node_t: 
*               the pointer to the dequeue node_t, if queue is empty, will return 
*               NULL 
*   
*
****************************************************************************/
node_t* queue_dequeue(queue_t *queue);

/*****************************************************************************
* Function Name: queue_enqueue
******************************************************************************
* Summary: the enqueue funtion, return bool to check the insert is success or not 
*          
*     
*
* Parameters: 
*   const queue_t *queue:
*         the queue_t pointer to the structure that you want to enqueue
*   
*   node_t* insert:
*         the node_t pointer that wants to insert into queue
* 
* Return:  bool: 
*               true: insert successful
*               false: insert failure      
*   
*
****************************************************************************/
bool queue_enqueue(queue_t *queue, node_t* insert);

/*****************************************************************************
* Function Name: queue_node_count
******************************************************************************
* Summary: the queue count funtion, return the queue size 
*          
*     
*
* Parameters: 
*   const queue_t *queue:
*         the queue_t pointer to the structure that you want to count
*   
* 
* Return:  uint32_t: 
*               the number of the queue    
*   
*
****************************************************************************/
uint32_t queue_node_count(const queue_t *queue);


/*****************************************************************************
* Function Name: node_create
******************************************************************************
* Summary: the node_create funtion, return the node_t pointer
*          
*     
*
* Parameters: 
*   uint16_t node_len:
*         the node len for SBC decode use
*   uint8_t* node_data:
*         the raw data for SBC decode use
* 
* Return:  node_t*: 
*               the node_t pointer, please free it when dequeue  
*   
*
****************************************************************************/
node_t* node_create(uint16_t node_len, uint8_t* node_data);


#endif /*__LINUX_QUEUE_H__ */

/* [] END OF FILE */
