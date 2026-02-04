/*
 *
 *
 *    copyright 2024 Cypress Semiconductor Corporation (an Infineon company)
 *    or an affiliate of Cypress Semiconductor Corporation. All rights reserved.
 *    This software, including source code, documentation and related materials
 *    ("Software") is owned by Cypress Semiconductor Corporation or one of its
 *    affiliates ("Cypress") and is protected by and subject to
 *    worldwide patent protection (United States and foreign),
 *    United State copyright laws and international treaty provisions.
 *    Therefore, you may use this Software only as provided in the license agreement
 *    accompanying the software package from which you obtained this software ("EULA").
 *    If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 *    non-transferable license to copy, modify, and compile the Software source code
 *    solely for use in connection with Cypress's integrated circuit products.
 *    Any reproduction, modification, translation, compilation, or representation
 *    of this Software except as specified above is prohibited without
 *    the expresswritten permission of Cypress.
 *    Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 *    EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT,
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *    Cypress reserves the right to make changes to the Software without notice.
 *    Cypress does not assume any liability arising out of the application or
 *    use of the Software or any product or circuit described in the Software.
 *    Cypress does not authorize its products for use in any products where a malfunction
 *    or failure of the Cypress product may reasonably be expected to result in
 *    sidnificant property damage, injury or death ("High Risk Product").
 *    By including Cypress's product in a High Risk Product, the manufacturer
 *    of such system or application assumes all risk of such use and in doing so
 *    agrees to indemnify Cypress against all liability.
 *
 *
 */

#ifndef _BT_TTY_H
#define _BT_TTY_H

#define BTTTY_RB_SIZE           PAGE_SIZE

#define BTTTY_RB_IS_EMPTY(rb)   ((rb)->head == (rb)->tail)
#define BTTTY_RB_CLEAR(rb)      ((rb)->head = (rb)->tail = 0)

#define BTTTY_RB_OCCPD(rb)      \
        (CIRC_CNT((rb)->head, (rb)->tail, BTTTY_RB_SIZE))
#define BTTTY_RB_AVAIL(rb)      \
        (CIRC_SPACE((rb)->head, (rb)->tail, BTTTY_RB_SIZE))
#define BTTTY_RB_HEAD_PTR(rb)   \
        (&((rb)->buf[(rb)->head]))
#define BTTTY_RB_TAIL_PTR(rb)   \
        (&((rb)->buf[(rb)->tail]))

#define BTTTY_RB_USED(rb,size)                                  \
        do                                                      \
        {                                                       \
            (rb)->head = (((rb)->head + size) % BTTTY_RB_SIZE); \
        } while (0)
#define BTTTY_RB_RELEASED(rb,size)                              \
        do                                                      \
        {                                                       \
            (rb)->tail = (((rb)->tail + size) % BTTTY_RB_SIZE); \
        } while (0)

#define BTTTY_WAIT_BUS_NEXT     1000

#define BTTTY_RECV_PAKET_SIZE(size_type, pkt_size, ptr, index)  \
        do                                                      \
        {                                                       \
            size_type   offset0 = ptr[index++];                 \
            size_type   offset1 = ptr[index++];                 \
            size_type   offset2 = ptr[index++];                 \
            pkt_size =  offset0 +                               \
                       (offset1 << 8) +                         \
                       (offset2 << 16);                         \
        } while (0)

#define BTTTY_SEND_PAKET_SIZE(ptr, pkt_size, index)             \
        do                                                      \
        {                                                       \
            ptr[index++] = (pkt_size) & 0xff;                   \
            ptr[index++] = ((pkt_size) >> 8) & 0xff;            \
            ptr[index++] = ((pkt_size) >> 16) & 0xff;           \
        } while (0)

#endif  /* _BT_TTY_H */
