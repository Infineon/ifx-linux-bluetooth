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

#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/circ_buf.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <linux/sched/signal.h>
#endif  /* LINUX_VERSION_CODE >= 4.10.0 */
#include <btsdio_interface.h>
#include "btttysdio.h"
#include "btttysdio_delay.h"
#include "btsdio_utils.h"
#include "btttysdio_ioctl.h"
#include "bts_typedefs.h"
#include "bts_msg.h"
#include "bttty.h"
#include "btdbg.h"

#define UART_NR                 2

static struct mutex     uart_sdio_table_lock;
static struct mutex     ttysdio_rx_lock;
static wait_queue_head_t read_wait_queue;

static struct uart_sdio_port uart_sdio_table[UART_NR];

static ssize_t ttysdio_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t ttysdio_write(struct file *, const char __user *, size_t, loff_t *);
static long ttysdio_ioctl(struct file *, unsigned int, unsigned long);
static int ttysdio_open(struct inode *, struct file *);
static int ttysdio_release(struct inode *, struct file *);

static struct file_operations ttysdio_fops =
{
    .owner              = THIS_MODULE,
//    .llseek             = no_llseek,
    .read               = ttysdio_read,
    .write              = ttysdio_write,
    .unlocked_ioctl     = ttysdio_ioctl,
    .open               = ttysdio_open,
    .release            = ttysdio_release,
};

static struct miscdevice ttysdio_miscdev =
{
    .minor              = MISC_DYNAMIC_MINOR,
    .name               = "ttySDIO",
    .fops               = &ttysdio_fops
};

/******************************************************************
 * Function: ttysdio_get_avail_port
 *
 * Description: Get available SDIO port when open port from application
 *
 * Parameter:
 *  void
 *
 * Return:
 *  TS_Data_t: port available and return usp pointer
 *  NULL:      not available
 *
 *****************************************************************/
static TS_Data_t * ttysdio_get_avail_port(void)
{
    int i;

    for (i=0; i < UART_NR; i++)
    {
        TS_Data_t * usp = &uart_sdio_table[i];

        /**
	 * Currently, find the device with SDIO func has bound,
	 * but TTY device did not initialize TTY device yet
	 * */
        if ((usp->regist) && (usp->ttyindex != -1))
        {
            BTS_DBG("usp = 0x%p, uart_sdio_table[%d]\n", usp, i);
            return usp;
        }
    }

    return NULL;
}

/******************************************************************
 * Function: ttysdio_create_port
 *
 * Description: Get uart_sdio_port *usp from ttysdio_get_avail_port when application opens port
 *
 * Parameter:
 *  struct uart_sdio_port **port: ttysdio_get_avail_port()
 *
 * Return:
 *  int:
 *     success: 0
 *     fail:    -ENOSPC
 *
 *****************************************************************/
static int ttysdio_create_port(struct uart_sdio_port **port)
{
    struct uart_sdio_port *usp = ttysdio_get_avail_port();

    if (!usp)
    {
        BTS_DBG("Can't get port");
        return -ENOSPC;
    }

    *port = usp;

    return 0;
}

/******************************************************************
 * Function: rx_copy_from_buf
 *
 * Description: Read data from buffer
 *
 * Parameter:
 *  RingBuf_t * rb:      buffer
 *  uint8_t __user ** b: pointer directing to user space
 *  size_t * size:       pointer directing to data size
 *
 * Return:
 *  int:
 *     success: 0
 *     fail:    others
 *
 *****************************************************************/
static int rx_copy_from_buf(RingBuf_t * rb, uint8_t __user ** b, size_t * size)
{
    size_t              n;
    int                 ret     = 0;

#ifdef DEBUG
    BTS_DBG("ocp = %ld, (BRS - tail) = %ld, size = %ld",
        BTTTY_RB_OCCPD(rb), (BTTTY_RB_SIZE - rb->tail), *size);
#endif  /* DEBUG */

    n = MIN(BTTTY_RB_OCCPD(rb), BTTTY_RB_SIZE - rb->tail);
    n = MIN(*size, n);

    if(n)
    {
#ifdef DEBUG
        BTS_DumpData("Read out", &rb->buf[rb->tail], n);
#endif  /* DEBUG */
        ret = copy_to_user(*b, &rb->buf[rb->tail], n);
        if (ret < 0)
            BTS_ERR("Error! copy to user, ret = %d, n = %ld", ret, n);
        else
        {
            BTTTY_RB_RELEASED(rb, n);
            *b    += n;
            *size -= n;
        }
    }

#ifdef DEBUG
    BTS_TRC("-- h = %d, t = %d, ret = %d", rb->head, rb->tail, ret);
#endif  /* DEBUG */

    return ret;
}

/******************************************************************
 * Function: ttysdio_read
 *
 * Description: Read the SDIO port
 *
 * Parameter:
 *  struct file *filp:  pointer directing to flag
 *  char __user ** ptr: pointer directing to user buffer
 *  size_t * size:      data length
 *  loff_t *off:        read file offset(no use)
 *
 * Return:
 *  int:
 *     success:             0
 *     usp error:           -EFAULT
 *     lock error:          -EAGAIN
 *     interruptible Error: -ERESTARTSYS
 *
 *****************************************************************/
static ssize_t ttysdio_read(struct file *filp, char __user *ptr, size_t size, loff_t *off)
{
    TS_Data_t *         usp = (struct uart_sdio_port *)filp->private_data;
    RingBuf_t *         rb  = &usp->rrbuf;
    uint8_t __user *    b = ptr;
    size_t              remaining_sz = size;
    long                timeout = MAX_SCHEDULE_TIMEOUT;
    int ret = 0;
    DECLARE_WAITQUEUE(wait, current);

    BTS_TRC("++ size = %d, off = 0x%p, *off = 0x%llx", (int)size, off, *off);
    if (!usp)
    {
        BTS_ERR("usp is NULL");
        return -EFAULT;
    }

    /* verify user access to buffer */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0))
    if (!access_ok(VERIFY_WRITE, ptr, size))
#else
    if (!access_ok(ptr, size))
#endif
    {
        BTS_ERR("Can't verify user buffer");
        return -EFAULT;
    }

#ifdef DEBUG
    BTS_DBG("++BTTTY_RB_OCCPD(rb) = %ld", BTTTY_RB_OCCPD(rb));
    BTS_DBG("++BTTTY_RB_TAIL_PTR(rb) = x%p", BTTTY_RB_TAIL_PTR(rb));
#endif  /* DEBUG */

    /*
     * NON_BLOCKING I/O
     * */
    if (filp->f_flags & O_NONBLOCK)
    {
        if (!mutex_trylock(&ttysdio_rx_lock))
        {
            BTS_ERR("Try lock Error!, EAGAIN");
            return -EAGAIN;
        }
    }
    else
    {
        if (mutex_lock_interruptible(&ttysdio_rx_lock))
        {
            BTS_ERR("Lock interruptible Error! ERESTARTSYS!");
            return -ERESTARTSYS;
        }
    }

    add_wait_queue(&read_wait_queue, &wait);
    while(remaining_sz)
    {
        set_current_state(TASK_INTERRUPTIBLE);
        if (BTTTY_RB_OCCPD(rb))
        {
            __set_current_state(TASK_RUNNING);
            down(&usp->work_lock);
            usp->rx_active = true;
            ret = rx_copy_from_buf(rb, (uint8_t __user **)&b, &remaining_sz);
            usp->rx_active = false;
            up(&usp->work_lock);
            if (ret < 0)
                break;
        }
        else if (!BTTTY_RB_OCCPD(rb) && (size - remaining_sz))
        {
            break;
        }
        else
        {
            if (!timeout)
            {
                BTS_TRC("timeout");
                break;
            }

            if (filp->f_flags & O_NONBLOCK)
            {
                ret = -EAGAIN;
                break;
            }

            schedule();

            if(signal_pending(current))
            {
                ret = -ERESTARTSYS;
                BTS_TRC("-ERESTARTSYS:%d", -ERESTARTSYS);
                break;
            }
        }
    }

    remove_wait_queue(&read_wait_queue, &wait);
    __set_current_state(TASK_RUNNING);
    mutex_unlock(&ttysdio_rx_lock);

    if (ret == 0)
    {
        ret = size - remaining_sz;
        BTS_TRC("ret:%d size:%ld, remaining_sz:%ld", ret, size, remaining_sz);
    }

#ifdef BT_SDIO_USE_POLLING
    if (usp->open)
        schedule_work(&usp->sdio_work);
#endif

#ifdef DEBUG
    BTS_TRC("-- h = %d, t = %d, ret = %d, size = %ld", rb->head, rb->tail, ret, size);
#endif  /* DEBUG */

    return ret;
}

/******************************************************************
 * Function: ttysdio_write
 *
 * Description: Write the SDIO port
 *
 * Parameter:
 *  struct file * filp:       pointer directing to flag
 *  const char __user ** ptr: pointer directing to user buffer
 *  size_t * size:            data length
 *  loff_t * off:             write file offset(no use)
 *
 * Return:
 *  ssize_t:
 *         success: size of input data
 *         fail:    other values(0, -ENOMEM, -EFAULT)
 *
 *****************************************************************/
static ssize_t ttysdio_write(struct file * filp,
    const char __user * ptr, size_t size, loff_t * off)
{
    ssize_t             written  = 0;
    TS_Data_t *         usp      = (struct uart_sdio_port *)filp->private_data;
    RingBuf_t *         rb       = &usp->wrbuf;
    uint8_t *           p        = usp->wblk;
    uint8_t             offset   = 0;
    uint32_t            hdr      = 3;
    int                 ret      = 0;
    struct sdio_buf *   tx_buf   = NULL;

    BTS_TRC("++ h = %d, t = %d, size = %ld, mtu=%d", rb->head, rb->tail, size, usp->mtu);

    if (!usp)
    {
        BTS_ERR("usp is NULL\n");
        return written;
    }

    down(&usp->work_lock);
    tx_buf = kmalloc(sizeof(struct sdio_buf), GFP_KERNEL);
    p = tx_buf->data = kmalloc((size + hdr), GFP_KERNEL);
    if (!tx_buf || !p) {
        ret = -ENOMEM;
        goto err;
    }

    INIT_LIST_HEAD(&tx_buf->list);
    if (usp->tx_buf_head == NULL)
        usp->tx_buf_head = tx_buf;
    else
        list_add_tail(&tx_buf->list, &usp->tx_buf_head->list);

    tx_buf->len = size + hdr;

    /* Reserve 3 bytes for length */
    if (copy_from_user((void *)(p + hdr), (void __user *)ptr, size))
    {
        BTS_ERR("Can't copy data from user: 0x%08x", (uint32_t)(uintptr_t)ptr);
        ret = -EFAULT;
        goto err;
    }

    BTTTY_SEND_PAKET_SIZE(p, hdr + size, offset);

#ifdef DEBUG
    BTS_DBG("usp->tx_buf_head = %p, hdr = %u, sz = %ld", &usp->tx_buf_head->list, hdr, size);
    BTS_DBG("tx_buf->list = %p, tx_buf->len %d packetType 0x%x \n",
        &tx_buf->list, tx_buf->len, tx_buf->data[3]);
#endif  /* DEBUG */
    BTS_DumpData("uart write", tx_buf->data, hdr + size);

    up(&usp->work_lock);
    schedule_work(&usp->sdio_work);

err:
    if (ret)
    {
        if (tx_buf)
        {
            kfree(tx_buf);
        }
        if (p)
        {
            kfree(p);
        }
    }
    return size;
}

/******************************************************************
 * Function: ttysdio_ioctl
 *
 * Description: I/O control function of SDIO driver
 *
 * Parameter:
 *  struct file * filp: pointer directing to private data
 *  unsigned int cmd:   BTTTYSDIO_IOC_CLK enable or not
 *  unsigned long arg:  memory of user space
 *
 * Return:
 *  long:
 *      success: 0
 *      fail:    -EFAULT
 *               -ENOIOCTLCMD
 *
 *****************************************************************/
static long ttysdio_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
    long   err = 0;
    TS_Data_t *         usp      = (struct uart_sdio_port *)filp->private_data;

    BTS_TRC("++ cmd = 0x%x\n", cmd);

    if (_IOC_DIR(cmd) & _IOC_READ)
    {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0))
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
#else
        err = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
#endif
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0))
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
#else
        err = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
#endif
    }

	BTS_NOTI("%s: BTTTYSDIO_IOC", __FUNCTION__);
    if (err)
        return -EFAULT;

#if defined(DEBUG_PORT)
    if ((_IOC_TYPE(cmd) == BTTTYSDIO_IOC_MAGIC) &&
       (_IOC_NR(cmd) <= BTTTYSDIO_IOC_MAX_NR))
        err = usp->iofunc(usp, cmd, arg);
    else
#endif  /* DEBUG_PORT */
        switch (cmd)
        {
            case BTTTYSDIO_IOC_CLK_ENA:
                BTS_NOTI("%s: BTTTYSDIO_IOC_CLK_ENA", __FUNCTION__);
                sdio_bus_clk_enable(usp->p_wlan_bus);
                break;
            case BTTTYSDIO_IOC_CLK_DIS:
                BTS_NOTI("%s: BTTTYSDIO_IOC_CLK_DIS", __FUNCTION__);
                sdio_bus_clk_disable(usp->p_wlan_bus);
                break;
            default:
                err = -ENOIOCTLCMD;
                break;
        }

    BTS_TRC("--");
    return err;
}

/******************************************************************
 * Function: ttysdio_open
 *
 * Description: Open the SDIO port
 *
 * Parameter:
 *  struct inode *nodp: pointer directing to SDIO port
 *  struct file *filp:  pointer directing to private data
 *
 * Return:
 *  int:
 *     success: 0
 *     fail:    others
 *              -EBUSY
 *              -EFAULT
 *
 *****************************************************************/
static int ttysdio_open(struct inode *nodp, struct file *filp)
{
    int ret;
    TS_Data_t *         usp = NULL;
    register_handler *  regfunc;

    mutex_lock(&uart_sdio_table_lock);

    ret = ttysdio_create_port(&usp);

    if(ret < 0)
    {
         mutex_unlock(&uart_sdio_table_lock);
         return ret;
    }

    if (usp->open)
    {
        mutex_unlock(&uart_sdio_table_lock);
        BTS_ERR("Already opened");
        return -EBUSY;
    }

    filp->private_data = usp;
    BTS_TRC("++ usp = 0x%p, ttyindex = %d\n", usp, usp->ttyindex);

    if (usp->regist)
    {
        regfunc = (register_handler *)usp->regist;
        ret = regfunc(usp);
    }
    else
        ret = -EFAULT;

    if (ret)
    {
        BTS_ERR("Open fail! ret = %d", ret);
        mutex_unlock(&uart_sdio_table_lock);
        return ret;
    }

    usp->open = true;
    mutex_unlock(&uart_sdio_table_lock);

    nonseekable_open(nodp, filp);

    BTS_TRC("--");

    return ret;
}

/******************************************************************
 * Function: ttysdio_release
 *
 * Description: Release the SDIO port
 *
 * Parameter:
 *  struct inode *nodp: pointer directing to SDIO port
 *  struct file *filp:  pointer directing to private data
 *
 * Return:
 *  int 0(always success): if we got 0 as return, we can know that SDIO must release successfully
 *
 *****************************************************************/
static int ttysdio_release(struct inode *nodp, struct file *filp)
{
    struct uart_sdio_port *usp;

    BTS_TRC("++ nodp = 0x%p, filp = 0x%p, ust = 0x%p\n",
        nodp, filp, uart_sdio_table);

    mutex_lock(&uart_sdio_table_lock);
    usp = filp->private_data;

#if defined(BT_SDIO_USE_POLLING)
    flush_delayed_work(&usp->poll_work);
#endif //BT_SDIO_USE_POLLING

    flush_work(&usp->sdio_work);
    while (usp->rx_active)
    {
         BTS_DBG("Wait for RX release resource!");
         BTS_Delay(1000);
    }
    down(&usp->work_lock);
    usp->open = false;

    free_page((unsigned long)usp->rrbuf.buf);
    usp->rrbuf.buf = NULL;

    free_page((unsigned long)usp->wrbuf.buf);
    usp->wrbuf.buf = NULL;

    if (usp->rblk)
    {
        kfree(usp->rblk);
        usp->rblk = NULL;
    }

    if (usp->wblk)
    {
        kfree(usp->wblk);
        usp->wblk = NULL;
    }

    usp->roffptr = NULL;
    up(&usp->work_lock);

    sdio_bt_detach(usp->p_wlan_bus);

    mutex_unlock(&uart_sdio_table_lock);
    BTS_TRC("--");

    return 0;
}

/******************************************************************
 * Function: UART_SDIO_RingBuf_Space
 *
 * Description: Check how many space does the buffer have
 *
 * Parameter:
 *  RingBuf_t * rb: pointer directing to the buffer address
 *
 * Return:
 *  uint32_t: the buffer space
 *
 *****************************************************************/
uint32_t UART_SDIO_RingBuf_Space(RingBuf_t * rb)
{
    BTS_TRC("head = %d, tail = %d", rb->head, rb->tail);
    return BTTTY_RB_AVAIL(rb);
}

/******************************************************************
 * Function: UART_SDIO_Update_RingBuf
 *
 * Description: Update the buffer space
 *
 * Parameter:
 *  TS_Data_t * usp: pointer directing to the empty data
 *  uint32_t size:   the buffer size
 *
 * Return:
 *  void
 *
 *****************************************************************/
void UART_SDIO_Update_RingBuf(TS_Data_t * usp, uint32_t size)
{
    BTTTY_RB_USED(&usp->rrbuf, size);

    if (usp->open)
        wake_up_interruptible(&read_wait_queue);
}

/******************************************************************
 * Function: UART_SDIO_Bind
 *
 * Description: Find a empty usp data
 *
 * Parameter:
 *  void
 *
 * Return:
 *  (void *)usp: pointer directing to the empty data
 *
 *****************************************************************/
void * UART_SDIO_Bind(void)
{
    int             i;

    BTS_TRC("++");

    for (i = 0; i < UART_NR; i++)
    {
        TS_Data_t * usp = &uart_sdio_table[i];
        if (usp->ttyindex == -1)
        {
            BTS_TRC("usp = 0x%p, uart_sdio_table[%d]", usp, i);
            usp->ttyindex = i;
            return (void *)usp;
        }
    }
    BTS_TRC("--");

    return NULL;
}

/******************************************************************
 * Function: UART_SDIO_Init
 *
 * Description: Init the SDIO port
 *
 * Parameter:
 *  void
 *
 * Return:
 *  int 0(always success): if we got 0 as return, we can know that SDIO must init successfully
 *
 *****************************************************************/
int UART_SDIO_Init(void)
{
    int i;
    int ret = misc_register(&ttysdio_miscdev);

    BTS_TRC("++");
    if (ret)
    {
        BTS_ERR("MISC register fail: %d", ret);
        return ret;
    }

    mutex_init(&uart_sdio_table_lock);
    mutex_init(&ttysdio_rx_lock);

    // -- initialize the WAIT QUEUE head
    init_waitqueue_head(&read_wait_queue);

    memset(uart_sdio_table, 0, sizeof(uart_sdio_table));

    for (i = 0; i < UART_NR; i++)
    {
        uart_sdio_table[i].ttyindex  = -1;
        uart_sdio_table[i].open      = false;
        uart_sdio_table[i].rx_active = false;
        uart_sdio_table[i].tx_active = false;
    }

    BTS_TRC("--");

    return 0;
}

/******************************************************************
 * Function: UART_SDIO_Exit
 *
 * Description: Exit SDIO
 *
 * Parameter:
 *  void
 *
 * Return:
 *  void
 *
 *****************************************************************/
void UART_SDIO_Exit(void)
{
    mutex_destroy(&uart_sdio_table_lock);
    mutex_destroy(&ttysdio_rx_lock);
    misc_deregister(&ttysdio_miscdev);
}
