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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/circ_buf.h>
#include <linux/uaccess.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/semaphore.h>
#include <linux/time.h>
#include <linux/skbuff.h>
#include <btsdio_interface.h>
#include <linux/version.h>
#include "btttysdio.h"
#include "bts_typedefs.h"
#include "bts_version.h"
#include "btsdio.h"
#include "btttysdio_delay.h"
#include "btsdio_utils.h"
#include "btsdio_funcs.h"
#include "uart_sdio_funcs.h"
#include "btttysdio_ioctl.h"
#include "bts_msg.h"
#include "btdbg.h"
#include "bttty.h"
#include <linux/list.h>

#include <net/bluetooth/bluetooth.h>

/* Extern Variables */
extern uint32_t btttysdio_msg_level;
uint32_t sdio_block_size = DEFAULT_SDIO_BLOCK_SIZE;
/* Fragmentation maximum packet length is passed as module param */
char btfw[MOD_PARAM_PATHLEN];
uint32_t mtu = BTTTYSDIO_MTU;
uint32_t control = 0;
bool     dled  = false;
wlan_bt_handle_t g_dhd_handle = NULL;
bt_wlan_shared_info_t g_bt_wlan_shared_info;

static btsdio_info_t btinfo;

static int btsdio_rrb_input(TS_Data_t *usp);

/* Forward Declarations */

extern int BTDLFW_Partial_Download_BTFW(wlan_bt_handle_t p_wlan_bus, char *fwpath);

/******************************************************************
 * Function: btsdio_bus_cfg_status
 * 
 * Description: Observe the SDIO status
 *
 * Parameter:
 *  TS_Data_t *usp: pointer of private data (TS_Data)
 *
 * Return:
 *  void
 *
 *****************************************************************/
static void btsdio_bus_cfg_status(TS_Data_t *usp)
{
    int intrd = 0;
    int err = 0;

    if (usp == NULL)
    {
        BTS_ERR("%s: usp is NULL", __FUNCTION__);
        return;
    }

    //read SDIO bus F3 Interrupt register
    intrd = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, REG_INTRD, &err);
    if (err)
    {
        BTS_ERR("%s: read REG_INTRD err %d", __FUNCTION__, err);
        return;
    }
    BTS_DBG("INTR flags 0x%x err:%d \n", intrd, err);
    //clear flag
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, REG_CL_INTRD, intrd, &err);
    if (err)
    {
        BTS_ERR("%s: write REG_CL_INTRD err %d", __FUNCTION__, err);
        return;
    }

    //set flag for schedule work use
    if (intrd & BTSDIO_INT_TX_BIT) {
        usp->tx_intr_flags = true;
    }

    if (intrd & BTSDIO_INT_RX_BIT) {
        usp->rx_intr_flags = true;
    }
}

#if defined(BT_SDIO_USE_POLLING)
uint32_t polling_ms = 0;

/******************************************************************
 * Function: btsdio_poll_work
 * 
 * Description: SDIO Polling work
 *
 * Parameter:
 *  struct work_struct *work: usp points to sdio_work
 *
 * Return:
 *  void
 *
 *****************************************************************/
static void btsdio_poll_work(struct work_struct *work)
{
    struct delayed_work *poll_work =
        container_of(work, struct delayed_work, work);
    TS_Data_t *usp =
        container_of(poll_work, TS_Data_t, poll_work);

    //BTS_TRC("++");
    if (usp->open)
    {
        btsdio_bus_cfg_status(usp);

        schedule_work(&usp->sdio_work);

        /* reschedule polling work whenever tty sdio port is opened */
        schedule_delayed_work(&usp->poll_work, msecs_to_jiffies(polling_ms));
    }
    else
    {
        BTS_DBG("usp not opend\n");
    }
    //BTS_TRC("--");
}
#else //!defined(BT_SDIO_USE_POLLING)

/******************************************************************
 * Function: btsdio_bs_int_handler
 *
 * Description: Wlan driver uses interrupt handler function when interrupt occurs
 *
 * Parameter:
 *  void *u: TS_Data_t to check the status
 *
 * Return:
 *  void
 *
 *****************************************************************/
static void btsdio_bs_int_handler(void *u)
{
    TS_Data_t *usp = (TS_Data_t *)u;

    BTS_TRC("++");

    if (usp->open)
    {
        btsdio_bus_cfg_status(usp);

        schedule_work(&usp->sdio_work);
    }
    else
    {
        BTS_DBG("usp not opend\n");
    }
    BTS_TRC("--");
}
#endif //BT_SDIO_USE_POLLING

#if defined(DEBUG_PORT)

/******************************************************************
 * Function: btsdio_writemem_addr
 *
 * Description: Write the address into memory
 *
 * Parameter:
 *  TS_Data_t * usp:  pointer of private data (TS_Data)
 *  uint32_t a:       address
 *  uint8_t * d:      pointer directing to memory address
 *  unsigned int sz:  number of the copied data string
 *
 * Return:
 *  int: wlan memory bytes
 *
 *****************************************************************/
static int btsdio_writemem_addr(TS_Data_t * usp, uint32_t a,
    uint8_t * d, unsigned int sz)
{
    int       ret  = 0;
    uint8_t * mptr = usp->ioblk;
    uint32_t  sa = a, sd, ea, ed, i;
    uint32_t  wc = 0;   /* written counter */

    if ((uint32_t)(uintptr_t)mptr % BTSDIO_SD_ALIGN)
        mptr += (BTSDIO_SD_ALIGN -
            ((uint32_t)(uintptr_t)mptr % BTSDIO_SD_ALIGN));

    if (!ISALIGNED(a, 4))
    {
        sa  = ROUNDDN(a, 4);
        sd  = sdio_bus_reg_read(usp->p_wlan_bus, sa);
        for (i = 0; i < (sa % 4); i++, wc++)
            mptr[wc] = (uint8_t)((uint8_t *)&sd)[i];
    }

    memcpy(&(mptr[wc]), d, sz);
    wc += sz;

    ea = sa + wc;
    if (!ISALIGNED(ea, 4))
    {
        ea = ROUNDDN(ea, 4);
        ed = sdio_bus_reg_read(usp->p_wlan_bus, ea);
        for (i = (ea % 4); i < 4; i++, wc++)
            mptr[wc] = (uint8_t)((uint8_t *)&ed)[i];
    }

#ifdef DEBUG
    BTS_DumpData("IO write", mptr, wc);
#endif  /* DEBUG */

    ret = wlan_membytes(usp->p_wlan_bus, BS_SDIORB_ACT_WRITE, sa, mptr, wc);
    if (ret)
        BTS_ERR("error = %d on writing %d bytes at x%08x", ret, wc, sa);

    return ret;

}

/******************************************************************
 * Function: btsdio_readmem_addr
 * 
 * Description: Read the memory address
 *
 * Parameter:
 *  TS_Data_t * usp: pointer of private data (TS_Data)
 *  uint32_t a:      start address
 *  uint8_t * d:     data copied
 *  unsigned int sz: number of the copied data
 *
 * Return: 
 *  int:
 *     success: return value of the wlan_membytes
 *     fail:    -EFAULT
 *
 *****************************************************************/
static int btsdio_readmem_addr(TS_Data_t * usp, uint32_t a,
    uint8_t * d, unsigned int sz)
{
    int               ret  = 0;
    uint8_t *         mptr = usp->ioblk;
    uint32_t          sa = a;

    if (sz > BTTTYSDIO_MTU)
    {
        BTS_ERR("sz = %d can't exceed ring buffer %d", sz, BTTTYSDIO_MTU);
        return -EFAULT;
    }

    if ((uint32_t)(uintptr_t)mptr % BTSDIO_SD_ALIGN)
        mptr += (BTSDIO_SD_ALIGN -
            ((uint32_t)(uintptr_t)mptr % BTSDIO_SD_ALIGN));

    if (!ISALIGNED(a, 4))
        sa  = ROUNDDN(a, 4);
    
    ret = wlan_membytes(usp->p_wlan_bus, BS_SDIORB_ACT_READ, sa, mptr, sz);
    if (ret)
        BTS_ERR("error = %d on reading %d bytes at x%08x", ret, sz, sa);

    memcpy(d, mptr, sz);
    BTS_DumpData("IO read", d, sz);

    return ret;
}

/******************************************************************
 * Function: btsdio_io_handler
 *
 * Description: I/O handler to SDIO
 *
 * Parameter:
 *  void * u:          TS_Data_t 
 *  uint32_t cmd:      BTTTYSDIO_IOC_ cases
 *  unsigned long arg: memory of user space
 *
 * Return:
 *  int:
 *     success: return value according to each case of cmd
 *     fail:    -EFAULT
 *
 *****************************************************************/
static int btsdio_io_handler(void * u, uint32_t cmd, unsigned long arg)
{
    TS_Data_t *usp = (TS_Data_t *)u;

    switch (cmd)
    {
    case BTTTYSDIO_IOC_DLFW:
        {
            BTDLFW_Msg msg;
            long       ret;
            memset(&msg, 0, sizeof(BTDLFW_Msg));
            copy_from_user((char *)&msg, (char __user *)arg, sizeof(BTDLFW_Msg));
            //ret = BTDLFW_Download_BTFW(usp->p_wlan_bus, msg.path);
            ret = dhd_download_btfw(usp->p_wlan_bus, msg.fwpath);
            return ret;
        }

    case BTTTYSDIO_IOC_RMEM:
	{
            BT_RWMem_Msg    msg;
            int             ret = 0;

            memset(&msg, 0, sizeof(BT_RWMem_Msg));
            if (copy_from_user((void *)&msg, (void __user *) arg, sizeof(BT_RWMem_Msg)))
            {
                BTS_ERR("Can't copy data from user: x%08lx", arg);
                return -EFAULT;
            }

            BTS_DBG("read addr = x%x, size = %d", msg.addr, msg.sz);

            ret = btsdio_readmem_addr(usp, msg.addr, msg.data, msg.sz);
            if (ret)
                return ret;

            if (copy_to_user((void __user *)arg, (void *)&msg, sizeof(BT_RWMem_Msg)))
            {
                BTS_ERR("Can't copy data to user: x%08lx", arg);
                return -EFAULT;
            }
            return ret;
	}

    case BTTTYSDIO_IOC_WMEM:
        {
            BT_RWMem_Msg msg;

            memset(&msg, 0, sizeof(msg));
            if (copy_from_user((void *)&msg, (void __user *)arg, sizeof(msg)))
            {
                BTS_ERR("Can't copy data from user: x%08lx", arg);
                return -EFAULT;
            }
            BTS_DBG("waddr = x%x, wsize = %d", msg.addr, msg.sz);

            btsdio_writemem_addr(usp, msg.addr, msg.data, msg.sz);
            return 0;
        }

    case BTTTYSDIO_IOC_RREG:
        {
            BT_RWMem_Msg msg;

            memset(&msg, 0, sizeof(msg));
            if (copy_from_user((void *)&msg, (void __user *)arg, sizeof(msg)))
            {
                BTS_ERR("Can't copy data from user: x%08lx", arg);
                return -EFAULT;
            }

            msg.value = sdio_bus_reg_read(usp->p_wlan_bus, msg.addr);
            BTS_DBG("addr = x%x, size = %d, v = x%x", msg.addr, msg.sz, msg.value);

            if (copy_to_user((void __user *)arg, (void *)&msg, sizeof(msg)))
            {
                BTS_ERR("Can't copy data to user: x%08lx", arg);
                return -EFAULT;
            }

            return 0;
        }

    case BTTTYSDIO_IOC_WREG:
        {
            BT_RWMem_Msg    msg;

            memset(&msg, 0, sizeof(msg));
            if (copy_from_user((void *)&msg, (void __user *)arg, sizeof(msg)))
            {
                BTS_ERR("Can't copy data from user: x%08lx", arg);
                return -EFAULT;
            }
            BTS_DBG("addr = x%x, size = %d", msg.addr, msg.sz);

            sdio_bus_reg_write(usp->p_wlan_bus, msg.addr, msg.value);
            return 0;
        }
    case BTTTYSDIO_IOC_EINT:
        {
            btsdio_bs_int_handler(usp);
            return 0;
        }
    }

    return 0;
}
#endif  /* DEBUG_PORT */


/******************************************************************
 * Function: btsdio_rrb_input
 *
 * Description: Read data from sdio bus and put data into a ring buffer
 *
 * Parameter:
 *  TS_Data_t *usp: pointer of private data (TS_Data)
 *
 * Return:
 * 	int:
 *     success: 0
 *     error:   -EBUSY
 *     !usp:    -1
 *
 *****************************************************************/
static int btsdio_rrb_input(TS_Data_t *usp)
{
    int ret = 0;
    if (!usp)
    {
        BTS_ERR("usp is NULL");
        goto err;
    }

    RingBuf_t *rb   = &usp->rrbuf;
    uint8_t *rbptr  = BTTTY_RB_HEAD_PTR(rb);
    uint8_t *p      = usp->rblk;
    uint32_t sz     = 0;
    uint8_t offset  = 0;
    uint32_t len    = 0;
    static uint8_t *hdr = NULL;
    static bool continue_flag = false;

    if (!hdr)
    {
        hdr = kmalloc(PACKET_HEADER, GFP_KERNEL);
        if (!hdr)
        {
            ret = -ENOMEM;
            goto err;
        }

        ret = sdio_bus_recv_buf(usp->p_wlan_bus, 0, SDIO_FUNC_3, hdr, PACKET_HEADER);
        if (ret)
        {
            BTS_ERR("%s can't get packet header (%d)", __FUNCTION__, ret);
            goto err;
        }
    }

#ifdef DEBUG
    BTS_DumpData("btsdio_rrb_input rx hdr dump", hdr, PACKET_HEADER);
#endif
    BTTTY_RECV_PAKET_SIZE(uint32_t, len, hdr, offset);

    if (len < PACKET_HEADER)
    {
        BTS_ERR("%s invalid packet length (%d)", __FUNCTION__, len);
        ret = -EINVAL;
        goto err;
    }

    if (continue_flag)
    {
        sz = len - PACKET_HEADER;
    }
    else
    {
        *p++ = hdr[3] & ~CONTINUE_PACKET_MASK;
        sz = len - PACKET_HEADER + 1;
    }

    if (sz > UART_SDIO_RingBuf_Space(rb))
    {
        BTS_WAR("size = %d > local available space = %d",
            sz, UART_SDIO_RingBuf_Space(rb));
        return -EBUSY;
    }

    ret = sdio_bus_recv_buf(usp->p_wlan_bus, 0, SDIO_FUNC_3, p, len - PACKET_HEADER);
    if (ret)
    {
        BTS_ERR("%s can't get packet payload (%d)", __FUNCTION__, ret);
        goto err;
    }
#ifdef DEBUG
    BTS_DumpData("btsdio_rrb_input rx dump", p, len);
#endif

    if (hdr[3] & CONTINUE_PACKET_MASK)
    {
        BTS_DBG("recv with continue_flag");
        continue_flag = true;
    }
    else
    {
        continue_flag = false;
    }

    if (rbptr == NULL || usp->rblk == NULL)
    {
        BTS_ERR("rbptr is NULL or usp->rblk is NULL");
        goto err;
    }

    if ((rb->head + sz) <= BTTTY_RB_SIZE)
    {
        memcpy(rbptr, usp->rblk, sz);
#ifdef DEBUG
        BTS_DBG("rbptr = x%p, sz = %d", rbptr , sz);
        BTS_DumpData("btsdio_rrb_input", rbptr , sz);
#endif  /* DEBUG */
    }
    else
    {
        uint32_t w1 = BTTTY_RB_SIZE - rb->head;
        uint32_t w2 = sz - w1;

        if (rb->buf == NULL)
        {
            BTS_ERR("rb->buf NULL");
            goto err;
        }

        memcpy(rbptr, usp->rblk, w1);
        memcpy(rb->buf, usp->rblk + w1, w2);

#ifdef DEBUG
        BTS_DBG("rbptr = x%p, w1 = %d", rbptr, w1);
        BTS_DumpData("btsdio_rrb_input", rbptr, w1);
        BTS_DBG("rb->buf = x%p, w2 = %d", rb->buf, w2);
        BTS_DumpData("btsdio_rrb_input", rb->buf, w2);
#endif  /* DEBUG */

    }

    UART_SDIO_Update_RingBuf(usp, sz);

err:
    if (hdr)
    {
        kfree(hdr);
        hdr = NULL;
    }

    return ret;
}

/******************************************************************
 * Function: btsdio_work
 *
 * Description: Main work for read and write data to SDIO bus
 *
 * Parameter:
 *  struct work_struct *work: usp points to sdio_work
 *
 * Return:
 *  void
 *
 *****************************************************************/
static void btsdio_work(struct work_struct *work)
{
    TS_Data_t * usp =
        container_of(work, TS_Data_t, sdio_work);
    int ret = 0;
    struct sdio_buf *p_buf = NULL;

    BTS_TRC("++, tx_intr:%d, rx_intr:%d", usp->tx_intr_flags, usp->rx_intr_flags);

    down(&usp->work_lock);

    if (!usp->open)
    {
        up(&usp->work_lock);
        BTS_ERR("!! usp not open\n");
        return;
    }

    if (!usp->tx_intr_flags && !usp->rx_intr_flags)
    {   
        up(&usp->work_lock);
        BTS_TRC("%s: TX and RX flag not set\n", __FUNCTION__);
        return;
    }

    if (usp->rx_intr_flags)
    {
        usp->rx_active = true;
        ret = btsdio_rrb_input(usp);
        if (!ret)
            usp->rx_intr_flags = false;
        else
        {
            BTS_NOTI("%s RX not finished, ret = %d,\n", __FUNCTION__, ret);
            schedule_work(&usp->sdio_work);
        }
        usp->rx_active = false;
    }

    if (usp->tx_intr_flags)
    {
        p_buf = usp->tx_buf_head;
        if (p_buf)
        {

#ifdef DEBUG
            BTS_DumpData("btsdio tx dump", p_buf->data, p_buf->len);
#endif  /* DEBUG */
            ret = sdio_bus_send_buf(usp->p_wlan_bus, 0, SDIO_FUNC_3, p_buf->data, p_buf->len);
            BTS_TRC("%s: &p_buf->list = %p, ret = %d,\n", __FUNCTION__, &p_buf->list, ret);

            if (!ret)
            {
                usp->tx_intr_flags = false;
                if(list_is_last(&usp->tx_buf_head->list, &p_buf->list))
                {
                    usp->tx_buf_head = NULL;
                }
                else
                {
                    usp->tx_buf_head = container_of(usp->tx_buf_head->list.next, struct sdio_buf, list);;
                    list_del(&p_buf->list);
                }
                kfree(p_buf->data);
                kfree(p_buf);
            }
        }
        else if (p_buf == NULL)
        {
            BTS_TRC("%s: No TX packets!!!\n", __FUNCTION__);
        }
    }

    up(&usp->work_lock);
    BTS_TRC("--");
}

/******************************************************************
 * Function: btsdio_free_buf
 * 
 * Description: Free the data buffer
 *
 * Parameter:
 *  TS_Data_t * usp: pointer of private data (TS_Data)
 *
 * Return:
 *  void
 *
 *****************************************************************/
static void btsdio_free_buf(TS_Data_t * usp)
{
    if (!usp)
        return;

    if (usp->rblk)
        kfree(usp->rblk);
    if (usp->wblk)
        kfree(usp->wblk);
    if (usp->rrbuf.buf)
        free_page((unsigned long)usp->rrbuf.buf);
    if (usp->wrbuf.buf)
        free_page((unsigned long)usp->wrbuf.buf);
}

/******************************************************************
 * Function: btsdio_check_chipid
 *
 * Description: Show the chip ID string and number
 *
 * Parameter:
 *  uint16_t g_bt_wlan_shared_info_device_id: chip ID from wlan
 *
 * Return:
 *  void
 *
 *****************************************************************/
void btsdio_check_chipid(uint16_t g_bt_wlan_shared_info_device_id)
{
    BTS_INFO("=======device id - 0x%x =======\n", g_bt_wlan_shared_info_device_id);
    switch(g_bt_wlan_shared_info_device_id){
        case SDIO_DEVICE_ID_BROADCOM_CYPRESS_43022:
            BTS_INFO("The chip is BROADCOM_CYPRESS_43022.\n");
            break;
        case SDIO_DEVICE_ID_BROADCOM_CYPRESS_43012:
            BTS_INFO("The chip is BROADCOM_CYPRESS_43012.\n");
            break;
        case SDIO_DEVICE_ID_CYPRESS_55500:
            BTS_INFO("The chip is CYPRESS_55500.\n");
            break;
        case SDIO_DEVICE_ID_CYPRESS_43022:
            BTS_INFO("The chip is CYPRESS_43022.\n");
            break;
        default:
    BTS_INFO("The chip is not 43022, 43012 or 55500.\n");
    }
}

/******************************************************************
 * Function: btsdio_download_43022_FW
 *
 * Description: When the chip is dectected as 43022, download FW 
 *
 * Parameter:
 *  void * d: TS_Data_t *usp points to
 *
 * Return:
 *  void
 *
 *****************************************************************/
void btsdio_download_43022_FW(void * d)
{
    TS_Data_t *     usp   = (TS_Data_t *)d;
    int             ret   = 0;

    BTS_TRC("PARTIAL DW FW\n");
	/* download firmware */ //43022
    if (usp->fwpath[0] != '\0')
    {
        if (control & (1 << BS_CNTRL_FWDL_EVERY_OPEN_SH))
        {
            BTS_DBG("fw path = %s", usp->fwpath);
            ret = BTDLFW_Partial_Download_BTFW(usp->p_wlan_bus, usp->fwpath);
            BTS_Delay(150000);
        }
        else
        {
            if (dled == false)
            {
                BTS_DBG("fw path = %s", usp->fwpath);
                ret = BTDLFW_Partial_Download_BTFW(usp->p_wlan_bus, usp->fwpath);
                if (!ret)
                {
                    BTS_DBG("Download \"%s\" Done!", usp->fwpath);
                    dled = true;
                    BTS_Delay(150000);
                }
                else
                {
                    BTS_DBG("BTDLFW_Partial_Download_BTFW FAIL");
                }
            }
        }
    }    
}

/******************************************************************
 * Function: btsdio_download_H1_FW
 *
 * Description: When the chip is dectected as H1, download FW 
 *
 * Parameter:
 *  void * d: TS_Data_t *usp points to
 *
 * Return:
 *  void
 *
 *****************************************************************/
void btsdio_download_H1_FW(void * d)
{
    TS_Data_t *     usp   = (TS_Data_t *)d;
    unsigned char f3_ready = 0;
    int             ret   = 0;
    
    BTS_TRC("55500 DW FW\n");
    if (!(f3_ready & (SDIOD_D2H_MSG_FW_VALIDATION_RESULT | SDIOD_D2H_MSG_FW_VALIDATION_DONE))) 
    {
        /* download firmware */ //H1
        if (usp->fwpath[0] != '\0')
        {
            BTS_INFO("fw path = %s", usp->fwpath);
            ret = BTDLFW_Download_BTFW(usp, usp->fwpath);
            if (!ret)
            {
                BTS_INFO("Download Done!");
            }
            else
            {
                BTS_ERR("Download Fail!");
                return -EFAULT;
            }
        }
    }
    else
    {
        BTS_INFO("FW Download Already");
    }
}

/******************************************************************
 * Function: BTSDIO_Check_To_Host_Mailbox_Data
 *
 * Description: Check data which FW sends to host
 *
 * Parameter:
 *  wlan_bt_handler_t p_wlan_bus: hex file data should be downloaded
 *  uint32_t checkdata:           correct data used to compare with
 *
 * Return:
 *  int:
 *     success: 0
 *     fail:    -EFAULT   
 *
 *****************************************************************/
int BTSDIO_Check_To_Host_Mailbox_Data(wlan_bt_handle_t p_wlan_bus, uint32_t checkdata)
{
    int           ret = 0;
    int 		  err = 0;
    unsigned char f3_int_pending = 0;
    unsigned char u8ToHostMailBoxData = 0;
    uint32_t 	  u32ToHostMailBoxData = 0;
    unsigned char retry_cnt = FW_IORDY_DELAY;

    /* wait till F3 interrupt pending bit 7 is 1 */
    while(retry_cnt)
    {
        f3_int_pending = sdio_bus_cfg_read(p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_INTERRUPT_PENDING, &err);
        if (err)
        {
            BTS_ERR("%s: read SDIOD_F3_INTERRUPT_PENDING err %d", __FUNCTION__, err);
            return -EFAULT;
        }
        BTS_TRC("%s: read f3_int_pending :0x%x err :%d ", __FUNCTION__, f3_int_pending, err);
        if(f3_int_pending & 0x80)
        {
            break;
        }
        msleep(FW_IORDY_DELAY);
        retry_cnt--;
        if(!retry_cnt)
        {
            BTS_ERR("%s: read f3_int_pending error :0x%x ", __FUNCTION__, f3_int_pending);
            return -EFAULT;
        }
    }
	
    /* clear F3 interrupt peding bit 7 */
    f3_int_pending = 0x80;
    sdio_bus_cfg_write(p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_INTERRUPT_PENDING, f3_int_pending, &err);
    if (err)
    {
        BTS_ERR("%s: write SDIOD_F3_INTERRUPT_PENDING err %d", __FUNCTION__, err);
        return -EFAULT;
    }

    /* read F3 ToHostMailBoxData */
    u8ToHostMailBoxData = sdio_bus_cfg_read(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_HOST_MAILBOX_DATA_BYTE0, &err);
    if (err)
    {
        BTS_ERR("%s: read SDIO_TO_HOST_MAILBOX_DATA_BYTE0 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    u32ToHostMailBoxData = u8ToHostMailBoxData;
    BTS_TRC("SDIO_TO_HOST_MAILBOX_DATA_BYTE0 :0x%x ", u32ToHostMailBoxData);
    u8ToHostMailBoxData = sdio_bus_cfg_read(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_HOST_MAILBOX_DATA_BYTE1, &err);
    if (err)
    {
        BTS_ERR("%s: read SDIO_TO_HOST_MAILBOX_DATA_BYTE1 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    u32ToHostMailBoxData |= u8ToHostMailBoxData << 8;
    BTS_TRC("SDIO_TO_HOST_MAILBOX_DATA_BYTE1 :0x%x ", u32ToHostMailBoxData);
    u8ToHostMailBoxData = sdio_bus_cfg_read(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_HOST_MAILBOX_DATA_BYTE2, &err);
    if (err)
    {
        BTS_ERR("%s: read SDIO_TO_HOST_MAILBOX_DATA_BYTE2 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    u32ToHostMailBoxData |= u8ToHostMailBoxData << 16;
    BTS_TRC("SDIO_TO_HOST_MAILBOX_DATA_BYTE2 :0x%x ", u32ToHostMailBoxData);
    u8ToHostMailBoxData = sdio_bus_cfg_read(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_HOST_MAILBOX_DATA_BYTE3, &err);
    if (err)
    {
        BTS_ERR("%s: read SDIO_TO_HOST_MAILBOX_DATA_BYTE3 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    u32ToHostMailBoxData |= u8ToHostMailBoxData << 24;
    BTS_TRC("SDIO_TO_HOST_MAILBOX_DATA_BYTE3 :0x%x ", u32ToHostMailBoxData);

    if(u32ToHostMailBoxData != checkdata)
    {
        BTS_ERR("%s: ToHostMailBoxData:0x%x not same checkdata:0x%x ", __FUNCTION__, checkdata, u32ToHostMailBoxData);
        ret = -EFAULT;
    }
    else
    {
        BTS_TRC("u32ToHostMailBoxData :0x%x checkdata:0x%x same ", u32ToHostMailBoxData, checkdata);
    }

    return ret;
}

/******************************************************************
 * Function: btsdio_43022_reg
 *
 * Description: Codes in register only for 43022
 *
 * Parameter:          
 *  void * d:                TS_Data_t *usp points to
 *  unsigned char f3_enable: observe function is enabled or not 
 * 
 * Return:
 *  int ret:
 *      success: 0
 *      fail:    others
 *
 *****************************************************************/
int btsdio_43022_reg(void * d, unsigned char f3_enable)
{
    TS_Data_t *     usp   = (TS_Data_t *)d;
    uint32_t        reg;
    int             ret = 0;
    int             err = 0;

    unsigned char fw_ready = 0;
    unsigned char f3_ready = 0;
    unsigned char f3_int_en = 0;
    unsigned char f3_frame_ctrl = 0;
    int f3_ready_cnt = 0;

    /* Wait till F3 is enabled */
    while (!(f3_ready & SDIO_FUNC_ENABLE_3))
    {
        /* write magic number 0xCAFEFOOD to BT RAM and BT FW will enable F3 */
        if (dled == false)
        {
            uint32_t magicNumber = 0xCAFEF00D;
            ret = wlan_membytes(usp->p_wlan_bus, BS_SDIORB_ACT_WRITE, BTFW_MEM_OFFSET | BT_RAM_GROUP5, (uint8_t *)&magicNumber, 4);
            if(ret)
            {
                BTS_ERR("magic number error ");
                return -EFAULT;
            }
            else
            {
                BTS_INFO("magic number ok:ret:%d", ret);
                ret = 0;    //0 is success
            }
        }
        //BTS_Delay(FW_IORDY_DELAY);
        msleep(FW_IORDY_DELAY);
        /* Read the REG to check enabled is success */
        f3_ready = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_0, SDIOD_CCCR_IORDY, &err);
        if (err)
        {
            BTS_ERR("%s: SDIOD_CCCR_IORDY err %d", __FUNCTION__, err);
            return -EFAULT;
        }

        f3_ready_cnt++;
		BTS_INFO("%s IOREADY cnt = %d f3_ready = 0x%x enable=0x%x ", __FUNCTION__, f3_ready_cnt, f3_ready, f3_enable);
        if (f3_ready_cnt == FW_IORDY_CNT)
       	{
            BTS_ERR("IOREADY timeout cnt = %d ready = 0x%x\n", f3_ready_cnt, f3_ready);
            return -EFAULT;
        }
	}
    BTS_INFO("IOREADY cnt = %d ready = 0x%x\n", f3_ready_cnt, f3_ready);
    if (!(f3_ready & SDIO_FUNC_ENABLE_3))
    {
        BTS_ERR("F3 Not ready:%d", f3_ready);
        return -EFAULT;
    }

    f3_int_en = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_0, SDIOD_CCCR_INTEN, &err);
    if (err)
    {
        BTS_ERR("%s: SDIOD_F3_INTERRUPT_ENABLE read err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    BTS_INFO("SDIOD_F3_INTERRUPT_ENABLE read 0x%x\n", f3_int_en);

    /* Enable F3 interrupts by F0 */
    f3_int_en |= BTSDIO_INT_ENABLE;
    BTS_INFO("%s Enable F3 Interrupts 0x%x", __FUNCTION__, f3_int_en);

    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_0, SDIOD_CCCR_INTEN, f3_int_en, &err);
    if (err)
    {
        BTS_ERR("%s: Could not enable F3 0x%x ints", __FUNCTION__, f3_int_en);
        return -EFAULT;
    }
    else
    {
        int int_enable = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_0, SDIOD_CCCR_INTEN, &err);
        if (err)
        {
            BTS_ERR("%s: Enabled F3 ints err %d", __FUNCTION__, err);
            return -EFAULT;
        }
        BTS_INFO("%s: Enabled F3 0x%x ints. int_enable=0x%x", __FUNCTION__, f3_int_en, int_enable);
    }

    f3_int_en = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_INTERRUPT_ENABLE, &err);
    if (err)
    {
        BTS_ERR("%s: SDIOD_F3_INTERRUPT_ENABLE read err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    BTS_INFO("SDIOD_F3_INTERRUPT_ENABLE flags 0x%x\n", f3_int_en);

    /* Enable F3 Mailbox interrupts by F3 */
    f3_int_en |= BTSDIO_INT_TOHOSTMAILBOX;
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_INTERRUPT_ENABLE, f3_int_en, &err);
    if (err)
    {
        BTS_ERR("%s: Could not enable F3 mailbox 0x%x ints", __FUNCTION__, f3_int_en);
        return -EFAULT;
    }
    else
    {
        int int_enable = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_INTERRUPT_ENABLE, &err);
        if (err)
        {
            BTS_ERR("%s: Enabled F3 mailbox read err %d", __FUNCTION__, err);
            return -EFAULT;
        }
        BTS_INFO("%s: Enabled F3 mailbox 0x%x ints. int_enable=0x%x", __FUNCTION__, f3_int_en, int_enable);
    }

    /* Set "BtFrameCtrl0"::"EnBusyIndication" to 1 */
    f3_frame_ctrl = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_BT_FRAME_CTRL_0, &err);
    if (err)
    {
        BTS_ERR("%s: SDIOD_F3_BT_FRAME_CTRL_0 read err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    BTS_INFO("SDIOD_F3_BT_FRAME_CTRL_0 flags 0x%x\n", f3_frame_ctrl);

    /* Enable F3 BUSY_INDICATION_BIT */
    f3_frame_ctrl |= SDIOD_F3_ENABLE_BUSY_INDICATION_BIT;
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_BT_FRAME_CTRL_0, f3_frame_ctrl, &err);
    if (err)
    {
        BTS_ERR("%s: Could not enable F3 frame ctrl 0x%x ", __FUNCTION__, f3_frame_ctrl);
        return -EFAULT;
    }
    else
    {
        f3_frame_ctrl = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_BT_FRAME_CTRL_0, &err);
        if (err)
        {
            BTS_ERR("%s: SDIOD_F3_BT_FRAME_CTRL_0 read err %d", __FUNCTION__, err);
            return -EFAULT;
        }
        BTS_INFO("F3 frame control ok 0x:%x", f3_frame_ctrl);
    }

    
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_RX_ALMOST_FULL_CNT, 0x20, &err);
    if (err)
    {
        BTS_ERR("%s: Could not set F3 rx almost full cnt 0x%x ", __FUNCTION__, err);
        return -EFAULT;
    }
    else
    {
        f3_frame_ctrl = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_RX_ALMOST_FULL_CNT, &err);
        if (err)
        {
            BTS_ERR("%s: SDIOD_F3_RX_ALMOST_FULL_CNT read err %d", __FUNCTION__, err);
            return -EFAULT;
        }
        BTS_INFO("F3 f3_rx_almost_full_cnt ok 0x:%x", f3_frame_ctrl);
    }

	/* Set F3 block size */
	sdio_bus_set_blocksize(usp->p_wlan_bus, SDIO_FUNC_3, DEFAULT_SDIO_BLOCK_SIZE);
	BTS_INFO("Set SDIO F3 block size to %u\n", DEFAULT_SDIO_BLOCK_SIZE);
	
    /* wait FW Bootloader ready */
    if (dled == false)
    {
        if(BTSDIO_Check_To_Host_Mailbox_Data(usp->p_wlan_bus, SDIOD_D2H_MSG_BL_READY))
        {
            BTS_ERR("%s: FW Bootloader not ready", __FUNCTION__);
            return -EFAULT;
        }
        else
        {
            BTS_INFO("FW Bootloader ready");
        }
    }
    return ret;
}

/******************************************************************
 * Function: btsdio_H1_reg
 *
 * Description: Codes in register only for H1
 *
 * Parameter:          
 *  void * d:                TS_Data_t *usp points to
 *  unsigned char f3_enable: observe function is enabled or not 
 * 
 * Return:
 *  int ret:
 *      success: 0
 *      fail:    others
 *
 *****************************************************************/
int btsdio_H1_reg(void * d, unsigned char f3_enable)
{
    TS_Data_t *     usp   = (TS_Data_t *)d;
    uint32_t        reg;
    int             ret = 0;
    int             err = 0;

    unsigned char fw_ready = 0;
    unsigned char f3_ready = 0;
    unsigned char f3_int_en = 0;
    unsigned char f3_frame_ctrl = 0;
    int f3_ready_cnt = 0;

    /* Wait till F3 is enabled */
    while (!(f3_ready & SDIO_FUNC_ENABLE_3))
    {
        BTS_Delay(FW_IORDY_DELAY);
        /* Read the REG to check enabled is success */
        f3_ready = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_0, SDIOD_CCCR_IORDY, &err);
        if (err)
        {
            BTS_ERR("%s: SDIOD_CCCR_IORDY err %d", __FUNCTION__, err);
            return -EFAULT;
        }

        f3_ready_cnt++;
        BTS_INFO("%s IOREADY cnt = %d f3_ready = 0x%x enable=0x%x ", __FUNCTION__, f3_ready_cnt, f3_ready, f3_enable);
        if (f3_ready_cnt == FW_IORDY_CNT)
        {
            BTS_ERR("IOREADY timeout cnt = %d ready = 0x%x\n", f3_ready_cnt, f3_ready);
            return -EFAULT;
        }
    }
    BTS_INFO("IOREADY cnt = %d ready = 0x%x\n", f3_ready_cnt, f3_ready);

    fw_ready = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_DEVICE_TO_HOST_MSG_0, &err);
    if (err)
    {
        BTS_ERR("%s: SDIOD_F3_DEVICE_TO_HOST_MSG_0 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    BTS_DBG("FW Ready Flags 0x%x\n", fw_ready);

    
    /* Set "BtFrameCtrl0"::"EnBusyIndication" to 1 */
    f3_frame_ctrl = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_BT_FRAME_CTRL_0, &err);
    if (err)
    {
        BTS_ERR("%s: SDIOD_F3_BT_FRAME_CTRL_0 read err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    BTS_INFO("SDIOD_F3_BT_FRAME_CTRL_0 flags 0x%x\n", f3_frame_ctrl);

    /* Enable F3 BUSY_INDICATION_BIT */
    f3_frame_ctrl |= SDIOD_F3_ENABLE_BUSY_INDICATION_BIT;
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_BT_FRAME_CTRL_0, f3_frame_ctrl, &err);
    if (err) 
    {
        BTS_ERR("%s: Could not enable F3 frame ctrl 0x%x ", __FUNCTION__, f3_frame_ctrl);
        return -EFAULT;
    }
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_RX_ALMOST_FULL_CNT, 0x20, &err);
    if (err)
    {
        BTS_ERR("%s: SDIOD_F3_RX_ALMOST_FULL_CNT err %d", __FUNCTION__, err);
        return -EFAULT;
    }

    sdio_bus_set_blocksize(usp->p_wlan_bus, SDIO_FUNC_3, sdio_block_size);
    BTS_INFO("Set SDIO F3 block size to %u\n", sdio_block_size);

    f3_int_en = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_INTERRUPT_ENABLE, &err);
    if (err)
    {
        BTS_ERR("%s: read SDIOD_F3_INTERRUPT_ENABLE err %d", __FUNCTION__, err);
        return -EFAULT;
    }

    /* Enable F3 interrupts */
    f3_int_en |= BTSDIO_INT_BITS;
    BTS_INFO("SDIOD_F3_INTERRUPT_ENABLE flags 0x%x\n", f3_int_en);

    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_INTERRUPT_ENABLE, f3_int_en, &err);
    if (err)
    {
        BTS_ERR("%s: Could not enable F3 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    else
    {
        int int_enable = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_INTERRUPT_ENABLE, &err);
        if (err)
        {
            BTS_ERR("%s: read SDIOD_F3_INTERRUPT_ENABLE err %d", __FUNCTION__, err);
            return -EFAULT;
        }
        BTS_INFO("%s: Enabled F3 int_enable=0x%x", __FUNCTION__, int_enable);
    }
    return ret;

}

/******************************************************************
 * Function: btsdio_register
 *
 * Description: SDIO main function for register
 *
 * Parameter:
 *  void * d: TS_Data_t *usp points to
 *
 * Return:
 *  int:
 *     success: 0
 *     fail:    -EFAULT
 *
 *****************************************************************/
static int btsdio_register(void * d)
{
    TS_Data_t *     usp   = (TS_Data_t *)d;
    int             ret   = 0;
    btsdio_info_t * bi    = &btinfo;
    RingBuf_t *     wrb   = &usp->wrbuf;
    RingBuf_t *     rrb   = &usp->rrbuf;
    unsigned long   page;
    uint32_t        reg;
    int err = 0;

    unsigned char card_ctrl = 0;
    unsigned char f3_enable = 0;

    BTS_DBG("++");
#ifdef DEBUG_REG_N_TIME
    uint32_t        reg_hi, reg_ho, reg_bi, reg_bo;
    struct timespec otv, ntv;
#endif  /* DEBUG_REG_N_TIME */

    if (!usp)
    {
        BTS_ERR("Fatal error: No UART SDIO share data registered");
        return -EFAULT;
    }

#if defined(BT_SDIO_USE_POLLING)
    BTS_NOTI("%s: Use polling mechanism", __FUNCTION__);
    #if defined(FMAC_BUILD)
    g_bt_wlan_shared_info.bt_data = usp;
    g_bt_wlan_shared_info.bt_int_fun = NULL;
    err = sdio_bt_attach(BTS_VERSION, &g_bt_wlan_shared_info);
    if (err)
    {
        BTS_ERR("sdio_bt_attach err %d", err);
        return -EFAULT;
    }
    usp->p_wlan_bus = g_bt_wlan_shared_info.wlan_bus_if;
    BTS_INFO("BTS_VERSION %d wlan_bus_if 0x%x enum_addr 0x%x \n", BTS_VERSION, g_bt_wlan_shared_info.wlan_bus_if, g_bt_wlan_shared_info.enum_addr);
    #else
    usp->p_wlan_bus = dhd_bt_get_pub_hndl();
    #endif
#else
    BTS_NOTI("%s: Use interrupt mechanism", __FUNCTION__);

    g_bt_wlan_shared_info.bt_data = usp;
    g_bt_wlan_shared_info.bt_int_fun = btsdio_bs_int_handler;
    err = sdio_bt_attach(BTS_VERSION, &g_bt_wlan_shared_info);
    if (err)
    {
        BTS_ERR("sdio_bt_attach err %d", err);
        return -EFAULT;
    }
    usp->p_wlan_bus = g_bt_wlan_shared_info.wlan_bus_if;

    btsdio_check_chipid(g_bt_wlan_shared_info.device_id);

    BTS_INFO("BTS_VERSION %d wlan_bus_if 0x%x enum_addr 0x%x \n", BTS_VERSION, g_bt_wlan_shared_info.wlan_bus_if, g_bt_wlan_shared_info.enum_addr);

#endif //BT_SDIO_USE_POLLING

    if (!usp->p_wlan_bus)
    {
        BTS_ERR("Can't get wlan_bus handle? usp = x%p, wlan_bus = x%p", usp, usp->p_wlan_bus);
        return -ENOENT;
    }

    sdio_bus_clk_enable(usp->p_wlan_bus);

    /* Set "Card Control"::"btResetOnRES" to 1 */
    card_ctrl = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_0, SDIOD_CARD_CONTROL, &err);
    if (err)
    {
        BTS_ERR("%s: SDIOD_CARD_CONTROL err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    BTS_INFO("SDIOD_CARD_CONTROL flags 0x%x\n", card_ctrl);

    /* Enable btResetOnRES */
    card_ctrl |= SDIOD_CARD_CONTROL_BT_RESET_ON_RES;

    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_0, SDIOD_CARD_CONTROL, card_ctrl, &err);
    if (err) {
        BTS_ERR("%s: Could not enable F0 card control 0x%x err %d ", __FUNCTION__, card_ctrl, err);
        return -EFAULT;
    }

    f3_enable = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_0, SDIOD_CCCR_IOEN, &err);
    if (err)
    {
        BTS_ERR("%s: SDIOD_CCCR_IOEN err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    BTS_INFO("SDIOD_CCCR_IOEN flags 0x%x\n", f3_enable);

    /* Enable F3 Function */
    f3_enable |= SDIO_FUNC_ENABLE_3;
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_0, SDIOD_CCCR_IOEN, f3_enable, &err);

    if (err)
    {
        BTS_ERR("%s: Could not enable F3 err %d", __FUNCTION__, err);
        return -EFAULT;
    }

    switch (g_bt_wlan_shared_info.device_id)
    {
        case SDIO_DEVICE_ID_BROADCOM_CYPRESS_43022:
        case SDIO_DEVICE_ID_BROADCOM_CYPRESS_43012:
        case SDIO_DEVICE_ID_CYPRESS_43022:
            ret = btsdio_43022_reg(usp, f3_enable);
            break;
        case SDIO_DEVICE_ID_CYPRESS_55500:
            ret = btsdio_H1_reg(usp, f3_enable);
            break;
        default:
            break;
    }
    if (ret != 0)
    {   
        BTS_ERR("Register Fail:%d", ret);
        btsdio_free_buf(usp);
        return ret;
    }
    
    switch(g_bt_wlan_shared_info.device_id)
    {
        case SDIO_DEVICE_ID_BROADCOM_CYPRESS_43022:
        case SDIO_DEVICE_ID_BROADCOM_CYPRESS_43012:
        case SDIO_DEVICE_ID_CYPRESS_43022:
            btsdio_download_43022_FW(usp);
            break;
        case SDIO_DEVICE_ID_CYPRESS_55500:
            btsdio_download_H1_FW(usp);
            break;
        default:
            break;
    }

    usp->ready = true;
    usp->tx_intr_flags = true;
    usp->rx_intr_flags = false;

#if defined(BT_SDIO_USE_POLLING)
    schedule_delayed_work(&usp->poll_work, msecs_to_jiffies(50));
#endif //BT_SDIO_USE_POLLING

    usp->mtu = mtu;

    page = get_zeroed_page(GFP_KERNEL);
    if (!page)
    {
        BTS_ERR("Can't allocate memory for RX");
        btsdio_free_buf(usp);
        return -ENOMEM;
    }
    rrb->buf = (unsigned char *)page;
    rrb->head = rrb->tail = 0;

    page = get_zeroed_page(GFP_KERNEL);
    if (!page)
    {
        BTS_ERR("Can't allocate memory for TX");
        btsdio_free_buf(usp);
        return -ENOMEM;
    }
    wrb->buf = (unsigned char *)page;
    wrb->head = wrb->tail = 0;

    memset(usp->wrbuf.buf, 0, BS_SDIORB_SIZE);
    memset(usp->rrbuf.buf, 0, BS_SDIORB_SIZE);

    usp->rblk = kmalloc((mtu + BTFW_SD_ALIGN), GFP_KERNEL);
    usp->wblk = kmalloc((mtu + BTFW_SD_ALIGN), GFP_KERNEL);

    if ((!usp->rblk) || (!usp->wblk))
    {
        if (!usp->rblk)
            BTS_ERR("Can't allocate memory for reading index of BT");
        if (!usp->wblk)
            BTS_ERR("Can't allocate memory for read TX buffer from user");

        btsdio_free_buf(usp);
        return -ENOMEM;
    }

    usp->roffptr = usp->rblk;
    if ((uint32_t)(uintptr_t)usp->roffptr % BTFW_SD_ALIGN)
        usp->roffptr += BTFW_SD_ALIGN -
            ((uint32_t)(uintptr_t)usp->rblk % BTFW_SD_ALIGN);

    if (ret)
        btsdio_free_buf(usp);

    BTS_DBG("--");
    return ret;
}

/******************************************************************
 * Function: btttysdio_init
 *
 * Description: Init the SDIO
 *
 * Parameter:
 *  void
 *
 * Return:
 *  int:
 *     success: 0
 *     fail:    -ENOSPC
 *
 *****************************************************************/
static int __init btttysdio_init(void)
{
    int ret = 0;
    struct uart_sdio_port *usp = NULL;

    BTS_DBG("++");
    BTS_NOTI("Generic Bluetooth TTY SDIO driver ver %s, build %s, log_level 0x%X",
        VERSION, BUILD, btttysdio_msg_level);

    ret = UART_SDIO_Init();

    if (ret)
        goto err;

    /**
     *  The following session adhere,
     *  due to SDIO probe funcs occupied by dhd.
     *  Currently, TTY port is bound hardcode
     */
    usp = (TS_Data_t *)UART_SDIO_Bind();
    if (!usp)
    {
        ret = -ENOSPC;
        goto err;
    }

    sema_init(&usp->work_lock, 1);

    INIT_WORK(&usp->sdio_work, btsdio_work);

#if defined(BT_SDIO_USE_POLLING)
    if (polling_ms < 5 || polling_ms > 100)
    {
        BTS_WAR("undefined polling period: %dms (5~100)", polling_ms);
        polling_ms = 30;
    }
    BTS_INFO("setting polling period to %d ms", polling_ms);
    INIT_DELAYED_WORK(&usp->poll_work, btsdio_poll_work);
#endif //BT_SDIO_USE_POLLING

    usp->regist = btsdio_register;
    usp->fwpath = btfw;

err:
    if (ret)
    {
        BTS_ERR("err = %d, deregister TTY\n", ret);
        UART_SDIO_Exit();

        if (usp && usp->rblk)
            kfree(usp->rblk);

        if (usp && usp->wblk)
            kfree(usp->wblk);
    }

    BTS_DBG("--");

    return ret;
}

/******************************************************************
 * Function: btttysdio_exit
 *
 * Description: Exit the SDIO
 *
 * Parameter:
 *  void
 *
 * Return:
 *  void
 *
 *****************************************************************/
static void __exit btttysdio_exit(void)
{
    BTS_DBG("++");
    UART_SDIO_Exit();
    BTS_DBG("--");
}

module_init(btttysdio_init);
module_exit(btttysdio_exit);

module_param(mtu, int, S_IRUGO);
module_param(control, int, S_IRUGO);
module_param(dled, bool, S_IRUGO);
module_param(sdio_block_size, int, S_IRUGO);
module_param(btttysdio_msg_level, int, S_IRUGO);
module_param_string(btfw, btfw, MOD_PARAM_PATHLEN, 0660);
#if defined(BT_SDIO_USE_POLLING)
module_param(polling_ms, int, S_IRUGO);
#endif //BT_SDIO_USE_POLLING

MODULE_DESCRIPTION("Generic Bluetooth SDIO driver ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
