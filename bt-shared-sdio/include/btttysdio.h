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

#ifndef _BT_TTY_SDIO_H
#define _BT_TTY_SDIO_H

#if (defined(BTTTYSDIO_WITH_TTY_DEV) && (BTTTYSDIO_WITH_TTY_DEV == TRUE))
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#endif  /* BTTTYSDIO_WITH_TTY_DEV == TRUE */

#define BTTTYSDIO_MTU           2000
#define DEFAULT_SDIO_BLOCK_SIZE 256
#define MOD_PARAM_PATHLEN       256
#define BTS_BUF_ALIGN           4

typedef int (register_handler)(void *);
typedef int (sdio_io_handler)(void *, uint32_t, unsigned long);

struct sdio_buf
{
    uint8_t *                   data;
    uint32_t                    len;

    struct list_head list;
};

struct uart_sdio_port
{
    struct work_struct          sdio_work;
#if defined(BT_SDIO_USE_POLLING)
    struct delayed_work         poll_work;
#endif //BT_SDIO_USE_POLLING

    struct semaphore            work_lock;

    struct circ_buf             rrbuf;              /* tty port read buffer */
    struct circ_buf             wrbuf;              /* tty port write buffer */

    struct sdio_buf *           tx_buf_head;
    struct sdio_buf *           rx_buf_head;

    char *                      fwpath;

    int                         ttyindex;

    uint32_t                    mtu;

    register_handler *          regist;
    sdio_io_handler *           iofunc;
    struct sdio_func *          func;

    /* The following data is for sdio interface */
    void *                      p_wlan_bus;
    void *                      btinfo;

    uint8_t *                   rblk;               /* SDIO read block */
    uint8_t *                   roffptr;
    uint8_t *                   wblk;               /* SDIO write block */
    uint8_t *                   ioblk;

    unsigned int                tx_intr_flags;
    unsigned int                rx_intr_flags;

    wait_queue_head_t           rx_wait_q;
    bool                        open;
    bool                        ready;
    bool                        rx_active;
    bool                        tx_active;
    /* The following data is for tty port */
#if (defined(BTTTYSDIO_WITH_TTY_DEV) && (BTTTYSDIO_WITH_TTY_DEV == TRUE))
    struct tty_port    	        ttyport;
    struct tty_struct *         tty;
    struct device *             dev;
    unsigned int                modem_state;
#endif  /* BTTTYSDIO_WITH_TTY_DEV == TRUE */
};    /* TTY and SDIO data type */


#endif  /* _BT_TTY_SDIO_H */
