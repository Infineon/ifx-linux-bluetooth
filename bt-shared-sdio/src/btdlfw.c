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

#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/circ_buf.h>
#include "btsdio.h"
#include "btsdio_utils.h"
#include "btttysdio_delay.h"
#include "btsdio_interface.h"
#include "btttysdio_ioctl.h"
#include "btdbg.h"
#include "btttysdio.h"
#include "bts_typedefs.h"
#include <linux/version.h>
#include <net/bluetooth/bluetooth.h>

#define PARTIAL_BTDLFW_UNITEST 0
#define FW_TRANSPORT_READY_CNT 50
extern int BTSDIO_Check_To_Host_Mailbox_Data(wlan_bt_handle_t p_wlan_bus, uint32_t checkdata);

typedef struct hex_file_data
{
    int         addr_mode;  //mode of destination addr
    uint16_t    hi_addr;    //high byte of destination addr
    uint32_t    dest_addr;  //destination addr
    uint8_t     *ds;        //allocate memory for hex file data
} hex_file_data;

/******************************************************************
 * Function: btdlfw_extract_fw_hex_field
 *
 * Description: Extract the BT FW data and display in a standard form
 *
 * Parameter:
 *  char *str:   pointer directing to the FW data should be extracted
 *  uint16_t sp: the original FW data
 *  uint16_t cn: first cn characters to copy and extract
 *
 * Return:
 *  uint16_t: the extracted FW data
 *
 *****************************************************************/
static uint16_t btdlfw_extract_fw_hex_field(char *str, uint16_t sp, uint16_t cn)
{
    char field[8];
    uint16_t v;

    strncpy(field, str + sp, cn);
    field[cn] = '\0';

    sscanf(field, "%hX", &v);

    return v;
}

/******************************************************************
 * Function: btdlfw_hexfile_readline
 *
 * Description: Read the BT FW data string length by pointer
 *
 * Parameter:
 *  char *str:       pointer to FW data
 *  int len:         starting length of FW data
 *  struct file *fp: pointer directing to FW data struct
 *
 * Return:
 *  int: FW data length
 *
 *****************************************************************/
static int btdlfw_hexfile_readline(char *str, int len, struct file *fp)
{
    int         rd_len;
    uint        str_len = 0;
    char *      str_end = NULL;
    loff_t      pos;

    if (!fp || !str)
        return str_len;

    pos = fp->f_pos;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
    rd_len  = kernel_read(fp, str, len, &pos);
#else
    rd_len  = kernel_read(fp, pos, str, len);
#endif
    str_end = strnchr(str, len, '\n');
    if (str_end == NULL)
        return str_len;

    str_len = (uint)(str_end - str);

    /* Advance file pointer past the string length */
    fp->f_pos += str_len + 1;
    memset(str_end, 0, (rd_len - str_len));

    return str_len;
}

//#if PARTIAL_BTDLFW

/******************************************************************
 * Function: btdlfw_partial_hexfile_get_fw_data
 * 
 * Description: Get BT fw hex file data using btdlfw_extract_fw_hex_field
 *
 * Parameter:
 *  struct file *file:  pointer directing to struct file
 *  char *str:          pointer directing to FW data
 *  hex_file_data *hfd: pointer directing to where FW hex file data stored
 *
 * Return:
 *  uint32_t:
 *          success: num_bytes + 2 base address + 2 offset address + 2 data length
 *          fail:    0
 *
 *****************************************************************/
static uint32_t btdlfw_partial_hexfile_get_fw_data(struct file *file, char *str,
    hex_file_data *hfd)
{
    uint      str_len;
    uint16_t  num_bytes, addr, data_pos, type, w, i;
    uint32_t  totalbytes = 0;

    uint8_t   tempdata[BTFW_MAX_STR_LEN];
    memset(tempdata, 0, BTFW_MAX_STR_LEN);

    if (!str || !hfd->ds)
    {
        BTS_ERR("String or data string NULL");
        return totalbytes;
    }

    while (totalbytes == 0)
    {
        str_len = btdlfw_hexfile_readline(str, BTFW_MAX_STR_LEN, file);

        BTS_TRC("Len :0x%x  %s", str_len, str);

        if (str_len == 0)
        {
            break;
        }
        else if (str_len > 9)
        {
            num_bytes = btdlfw_extract_fw_hex_field(str, 1, 2);
            addr      = btdlfw_extract_fw_hex_field(str, 3, 4);
            type      = btdlfw_extract_fw_hex_field(str, 7, 2);

            data_pos = 9;

            for (i = 0; i < num_bytes; i++)
            {
                w = btdlfw_extract_fw_hex_field(str, data_pos, 2);
                tempdata[i] = (uint8_t)(w & 0x00FF);
                data_pos += 2;
            }

            if (type == BTFW_HEX_LINE_TYPE_EXTENDED_ADDRESS)
            {
                hfd->ds[2] = tempdata[1];
                hfd->ds[3] = tempdata[0];
            }
            else if (type == BTFW_HEX_LINE_TYPE_DATA)
            {
                hfd->ds[0] = addr & 0xFF;
                hfd->ds[1] = addr >> 8;
                hfd->ds[4] = num_bytes & 0xFF;
                hfd->ds[5] = num_bytes >> 8;	
                memcpy(&hfd->ds[6], tempdata, num_bytes);
                totalbytes = num_bytes + 6; //6 is 2 baseaddr + 2 offsetaddr + 2 datalen
            }
            else if(type == BTFW_HEX_LINE_TYPE_EXTENDED_SEGMENT_ADDRESS || type == BTFW_HEX_LINE_TYPE_ABSOLUTE_32BIT_ADDRESS)
            {
                BTS_ERR("hex unsupport type :%d", type);
                break;
            }
            else
            {
                break;
            }
        }
    }

    BTS_DumpData("DLFW:", hfd->ds, totalbytes);
	
    return totalbytes;
}
//#else

/******************************************************************
 * Function: btdlfw_hexfile_get_fw_data
 * 
 * Description: Get the BT FW hex file data
 *
 * Parameter:
 *  struct file *file:  pointer directing to struct file
 *  char *str:          pointer directing to FW data
 *  hex_file_data *hfd: pointer directing to where FW hex file data stored
 *
 * Return:
 *  uint32_t: FW data string read from its address
 *
 *****************************************************************/
static uint32_t btdlfw_hexfile_get_fw_data(struct file *file, char *str,
    hex_file_data *hfd)
{
    int       str_len;
    uint16_t  num_bytes, addr, data_pos, type, w, i;
    uint32_t  abs_base_addr32 = 0;
    uint32_t  nbytes = 0;

    if (!str || !hfd->ds)
    {
        BTS_ERR("String or data string NULL");
        return nbytes;
    }

    while (nbytes == 0)
    {
        str_len = btdlfw_hexfile_readline(str, BTFW_MAX_STR_LEN, file);

        BTS_TRC("Len :0x%x  %s", str_len, str);

        if (str_len == 0)
        {
            break;
        }
        else if (str_len > 9)
        {
            num_bytes = btdlfw_extract_fw_hex_field(str, 1, 2);
            addr      = btdlfw_extract_fw_hex_field(str, 3, 4);
            type      = btdlfw_extract_fw_hex_field(str, 7, 2);

            data_pos = 9;

            for (i = 0; i < num_bytes; i++)
            {
                w = btdlfw_extract_fw_hex_field(str, data_pos, 2);
                hfd->ds[i] = (uint8_t)(w & 0x00FF);
                data_pos += 2;
            }

            if (type == BTFW_HEX_LINE_TYPE_EXTENDED_ADDRESS)
            {
                hfd->hi_addr = (hfd->ds[0] << 8) | hfd->ds[1];
                hfd->addr_mode = BTFW_ADDR_MODE_EXTENDED;
            }
            else if (type == BTFW_HEX_LINE_TYPE_EXTENDED_SEGMENT_ADDRESS)
            {
                hfd->hi_addr = (hfd->ds[0] << 8) | hfd->ds[1];
                hfd->addr_mode = BTFW_ADDR_MODE_SEGMENT;
            }
            else if (type == BTFW_HEX_LINE_TYPE_ABSOLUTE_32BIT_ADDRESS)
            {
                abs_base_addr32 = (hfd->ds[0] << 24) | (hfd->ds[1] << 16) |
                    (hfd->ds[2] << 8) | hfd->ds[3];
                hfd->addr_mode = BTFW_ADDR_MODE_LINEAR32;
            }
            else if (type == BTFW_HEX_LINE_TYPE_DATA)
            {
                hfd->dest_addr = addr;

                if (hfd->addr_mode == BTFW_ADDR_MODE_EXTENDED)
                    hfd->dest_addr += (hfd->hi_addr << 16);
                else if (hfd->addr_mode == BTFW_ADDR_MODE_SEGMENT)
                    hfd->dest_addr += (hfd->hi_addr << 4);
                else if (hfd->addr_mode == BTFW_ADDR_MODE_LINEAR32)
                    hfd->dest_addr += abs_base_addr32;

                nbytes = num_bytes;
            }
        }
    }

    return nbytes;
}
//#endif

//#if PARTIAL_BTDLFW

/******************************************************************
 * Function: BTDLFW_Partial_Download_BTFW
 *
 * Description: Read BT fw hex file and download it into BT ram
 *
 * Parameter:
 *  wlan_bt_handler_t p_wlan_bus: hex file data should be downloaded
 *  char *fwpath:                 pointer directing to firmware's path
 *
 * Return:
 *  int:
 *     not open:  -ENOENT
 *     no memory: -ENOMEM
 *     no ack:    -EFAULT
 *     timeout:   -EFAULT
 *     success:   0
 *
 *****************************************************************/
int BTDLFW_Partial_Download_BTFW(wlan_bt_handle_t p_wlan_bus, char *fwpath)
{
    hex_file_data    hfd = {BTFW_ADDR_MODE_EXTENDED, 0, 0, NULL};
    uint32_t         linedatalen;
    char             *str = NULL;
    int              ret  = 0;
    struct file      *fp = filp_open(fwpath, O_RDONLY, 0);
    unsigned char 	 u8ToSBMailBoxData = 0;
    int              error = 0;
    #if PARTIAL_BTDLFW_UNITEST
    uint8_t*		 unitestmembyte = NULL;
    #endif
    unsigned char 	 retry_cnt = FW_TRANSPORT_READY_CNT;

    if (IS_ERR(fp))
    {
        BTS_ERR("Can't open %s", fwpath);
        ret = -ENOENT;
        fp  = NULL;
        goto err;
    }

    hfd.ds = kmalloc(BTFW_MAX_STR_LEN, GFP_KERNEL);
    if (!hfd.ds)
    {
        BTS_ERR("Fail to allocate memory for hex file data");

        ret = -ENOMEM;
        goto err;
    }

    str = kmalloc(BTFW_MAX_STR_LEN, GFP_KERNEL);
    if (!str)
    {
        BTS_ERR("Fail to allocate memory for read hex file string");

        ret = -ENOMEM;
        goto err;
    }
    memset(str, 0, BTFW_MAX_STR_LEN);

    #if PARTIAL_BTDLFW_UNITEST
    unitestmembyte = kmalloc(BTFW_MAX_STR_LEN, GFP_KERNEL);
    if (!unitestmembyte)
    {
        BTS_ERR("Fail to allocate memory for unitestmembyte");

        ret = -ENOMEM;
        goto err;
    }
    memset(unitestmembyte, 0, BTFW_MAX_STR_LEN);
    #endif
    
    while ((linedatalen = btdlfw_partial_hexfile_get_fw_data(fp, str, &hfd)) > 0)
    {
        ret = wlan_membytes(p_wlan_bus, BS_SDIORB_ACT_WRITE, BTFW_MEM_OFFSET | BT_RAM_GROUP5, hfd.ds, linedatalen);
        if (ret)
        {
            BTS_ERR("error %d on writing %d membytes at 0x%08x", ret, linedatalen, BTFW_MEM_OFFSET);
            goto err;
        }
        else
        {
            BTS_TRC("wlan_membytes ok linedatalen:%d ", linedatalen);
        }
        /* tell fw download batch done */
        u8ToSBMailBoxData = SDIOD_H2D_MSG_BOOT_FW_DOWNLOAD_BATCH & 0xFF;
        sdio_bus_cfg_write(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_SB_MAILBOX_DATA_BYTE0, u8ToSBMailBoxData, &error);
        if (error)
        {
            ret = error;
            BTS_ERR("%s: write SDIO_TO_SB_MAILBOX_DATA_BYTE0 error:%d ", __FUNCTION__, error);
            goto err;
        }
        u8ToSBMailBoxData = (SDIOD_H2D_MSG_BOOT_FW_DOWNLOAD_BATCH >> 8) & 0xFF;
        sdio_bus_cfg_write(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_SB_MAILBOX_DATA_BYTE1, u8ToSBMailBoxData, &error);
        if (error)
        {
            ret = error;
            BTS_ERR("%s: write SDIO_TO_SB_MAILBOX_DATA_BYTE1 error:%d ", __FUNCTION__, error);
            goto err;
        }
        u8ToSBMailBoxData = (SDIOD_H2D_MSG_BOOT_FW_DOWNLOAD_BATCH >> 16) & 0xFF;
        sdio_bus_cfg_write(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_SB_MAILBOX_DATA_BYTE2, u8ToSBMailBoxData, &error);
        if (error)
        {
            ret = error;
            BTS_ERR("%s: write SDIO_TO_SB_MAILBOX_DATA_BYTE2 error:%d ", __FUNCTION__, error);
            goto err;
        }
        u8ToSBMailBoxData = (SDIOD_H2D_MSG_BOOT_FW_DOWNLOAD_BATCH >> 24) & 0xFF;
        sdio_bus_cfg_write(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_SB_MAILBOX_DATA_BYTE3, u8ToSBMailBoxData, &error);
        if (error)
        {
            ret = error;
            BTS_ERR("%s: write SDIO_TO_SB_MAILBOX_DATA_BYTE3 error:%d ", __FUNCTION__, error);
            goto err;
        }
        /* set mailbox bit 3 */
        sdio_bus_cfg_write(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_SB_MAILBOX_BIT, 8, &error);
        if (error)
        {
            ret = error;
            BTS_ERR("%s: write SDIO_TO_SB_MAILBOX_BIT error:%d ", __FUNCTION__, error);
            goto err;
        }

        /* wait fw ack host */
        if(BTSDIO_Check_To_Host_Mailbox_Data(p_wlan_bus, SDIOD_D2H_MSG_FW_DOWNLOAD_ACK))
        {
            BTS_ERR("%s: FW download no ack ", __FUNCTION__);
            ret = -EFAULT;
            goto err;
        }
        else
        {
        	BTS_TRC("%s: FW download ack ok", __FUNCTION__);
        }

        #if PARTIAL_BTDLFW_UNITEST
        /* unitest fw download */
        uint32_t    unitest_addr;

        unitest_addr = (hfd.ds[3] << 24) | (hfd.ds[2] << 16) | (hfd.ds[1] << 8) | (hfd.ds[0]);
        BTS_ERR("unitest fw download len:%d unitest_addr:0x%x ", linedatalen, unitest_addr);
        unitest_addr = ROUNDDN(unitest_addr, 4);
        BTS_ERR("unitest fw download ROUNDDN unitest_addr:0x%x ", unitest_addr);
        ret = wlan_membytes(p_wlan_bus, BS_SDIORB_ACT_READ, BTFW_MEM_OFFSET | unitest_addr, unitestmembyte, linedatalen);
        if(ret)
        {
            BTS_ERR("unitest_addrreadmembyte error:%d linedatalen:%d ", ret, linedatalen);
            goto err;
        }
        else
        {
            BTS_DumpData("unitest_addrreadmembyte:", unitestmembyte, linedatalen);
        }
        #endif
    }

    /* tell fw download complete */
    u8ToSBMailBoxData = SDIOD_H2D_MSG_BOOT_FW_DOWNLOADED & 0xFF;
    sdio_bus_cfg_write(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_SB_MAILBOX_DATA_BYTE0, u8ToSBMailBoxData, &error);
    if (error)
    {
        ret = error;
        BTS_ERR("%s: write SDIO_TO_SB_MAILBOX_DATA_BYTE0 error:%d ", __FUNCTION__, error);
        goto err;
    }
    u8ToSBMailBoxData = (SDIOD_H2D_MSG_BOOT_FW_DOWNLOADED >> 8) & 0xFF;
    sdio_bus_cfg_write(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_SB_MAILBOX_DATA_BYTE1, u8ToSBMailBoxData, &error);
    if (error)
    {
        ret = error;
        BTS_ERR("%s: write SDIO_TO_SB_MAILBOX_DATA_BYTE1 error:%d ", __FUNCTION__, error);
        goto err;
    }
    u8ToSBMailBoxData = (SDIOD_H2D_MSG_BOOT_FW_DOWNLOADED >> 16) & 0xFF;
    sdio_bus_cfg_write(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_SB_MAILBOX_DATA_BYTE2, u8ToSBMailBoxData, &error);
    if (error)
    {
        ret = error;
        BTS_ERR("%s: write SDIO_TO_SB_MAILBOX_DATA_BYTE2 error:%d ", __FUNCTION__, error);
        goto err;
    }
    u8ToSBMailBoxData = (SDIOD_H2D_MSG_BOOT_FW_DOWNLOADED >> 24) & 0xFF;
    sdio_bus_cfg_write(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_SB_MAILBOX_DATA_BYTE3, u8ToSBMailBoxData, &error);
    if (error)
    {
        ret = error;
        BTS_ERR("%s: write SDIO_TO_SB_MAILBOX_DATA_BYTE3 error:%d ", __FUNCTION__, error);
        goto err;
    }
    /* set mailbox bit 3 */
    sdio_bus_cfg_write(p_wlan_bus, SDIO_FUNC_3, SDIO_TO_SB_MAILBOX_BIT, 8, &error);
    if (error)
    {
        ret = error;
        BTS_ERR("%s: write SDIO_TO_SB_MAILBOX_BIT error:%d ", __FUNCTION__, error);
        goto err;
    }

    /* wait fw transport ready */
    while(retry_cnt)
    {
        msleep(FW_READY_DELAY);
        if(BTSDIO_Check_To_Host_Mailbox_Data(p_wlan_bus, SDIOD_D2H_MSG_TRANSPORT_READY))
        {
            BTS_TRC("%s: FW download TRANSPORT_READY wait", __FUNCTION__);
        }
        else
        {
            BTS_TRC("%s: FW download TRANSPORT_READY ok", __FUNCTION__);
            break;
        }

        retry_cnt--;
        if(!retry_cnt)
        {
            BTS_ERR("%s: FW download timeout 5s ", __FUNCTION__);
            ret = -EFAULT;
            goto err;
        }
    }


err:
    if (hfd.ds)
        kfree(hfd.ds);

    if (str)
        kfree(str);

    if (fp)
        filp_close(fp, NULL);

    #if PARTIAL_BTDLFW_UNITEST
    if (unitestmembyte)
        kfree(unitestmembyte);
    #endif

    return ret;
}
//#else

/******************************************************************
 * Function: _BTDLFW_Download_BTFW
 *
 * Description: Download FW for 43022
 *
 * Parameter:
 *  wlan_bt_handler_t p_wlan_bus: hex file data should be downloaded
 *  char *fwpath:                 pointer directing to firmware's path
 *
 * Return:
 *  int:
 *     not open:  -ENOENT
 *     no memory: -ENOMEM
 *     success:   0
 *
 *****************************************************************/
int _BTDLFW_Download_BTFW(wlan_bt_handle_t p_wlan_bus, char *fwpath)
{
    hex_file_data    hfd = {BTFW_ADDR_MODE_EXTENDED, 0, 0, NULL};
    uint8_t          *mem_blk = NULL;
    uint8_t          *mem_ptr = NULL;
    uint32_t         rd;
    char             *str = NULL;
    int              ret  = 0;
    struct file      *fp = filp_open(fwpath, O_RDONLY, 0);
    int              error = 0;
    if (IS_ERR(fp))
    {
        BTS_ERR("Can't open %s", fwpath);
        ret = -ENOENT;
        fp  = NULL;
        goto err;
    }

    mem_blk = mem_ptr = kmalloc(BTFW_DOWNLOAD_BLK_SIZE + BTFW_SD_ALIGN, GFP_KERNEL);
    if (!mem_ptr)
    {
        BTS_ERR("Fail to allocate memory for BTFWDL block");

        ret = -ENOMEM;
        goto err;
    }

    if ((uint32_t)(uintptr_t)mem_ptr % BTFW_SD_ALIGN)
        mem_ptr += (BTFW_SD_ALIGN - ((uint32_t)(uintptr_t)mem_blk % BTFW_SD_ALIGN));

    hfd.ds = kmalloc(BTFW_MAX_STR_LEN, GFP_KERNEL);
    if (!hfd.ds)
    {
        BTS_ERR("Fail to allocate memory for hex file data");

        ret = -ENOMEM;
        goto err;
    }

    str = kmalloc(BTFW_MAX_STR_LEN, GFP_KERNEL);
    if (!str)
    {
        BTS_ERR("Fail to allocate memory for read hex file string");

        ret = -ENOMEM;
        goto err;
    }
    memset(str, 0, BTFW_MAX_STR_LEN);

    while ((rd = btdlfw_hexfile_get_fw_data(fp, str, &hfd)) > 0)
    {
        uint32_t start_addr, start_data, end_addr, end_data, i, wbc, pad;

        BTS_TRC("\tread %d bytes at address %08x", rd, hfd.dest_addr);

        start_addr = BTFW_MEM_OFFSET | (hfd.dest_addr & 0x00FFFFFF);
        wbc        = 0;

        /**
         * Make sure the start address is 4 byte aligned to avoid alignment issues
         * with SD host controllers
         */
        if (!ISALIGNED(start_addr, 4))
        {
            pad        = start_addr % 4;
            start_addr = ROUNDDN(start_addr, 4);
            start_data = sdio_bus_reg_read(p_wlan_bus, start_addr, &error);
            if (error)
            {
                BTS_ERR("start_addr:0x%x sdio_bus_reg_read error:%d", start_addr, error);
                goto err;
            }
            for (i = 0; i < pad; i++, wbc++)
            {
                mem_ptr[wbc] = (uint8_t)((uint8_t *)&start_data)[i];
            }
        }
        memcpy(&(mem_ptr[wbc]), hfd.ds, rd);
        wbc += rd;

        /**
         * Make sure the length is multiple of 4bytes to avoid alignment issues
         * with SD host controllers
         */
        end_addr = start_addr + wbc;
        if (!ISALIGNED(end_addr, 4))
        {
            end_data = sdio_bus_reg_read(p_wlan_bus, ROUNDDN(end_addr, 4), &error);
            if (error)
            {
                BTS_ERR("end_addr:0x%x sdio_bus_reg_read error:%d", end_addr, error);
                goto err;
            }
            for (i = (end_addr % 4); i < 4; i++, wbc++)
            {
                mem_ptr[wbc] = (uint8_t)((uint8_t *)&end_data)[i];
            }
        }

        BTS_DumpData("DLFW:", mem_ptr, wbc);
        if (((start_addr & 0xFFF) + wbc) <= 0x1000)
        {
            ret = wlan_membytes(p_wlan_bus, BS_SDIORB_ACT_WRITE, start_addr, mem_ptr, wbc);
            if (ret)
            {
                BTS_ERR("error %d on writing %d membytes at 0x%08x",
                    ret, rd, start_addr);
                goto err;
            }
        }
        else
        {
            uint32_t wb = 0x1000 - (start_addr & 0xFFF);

            ret = wlan_membytes(p_wlan_bus, BS_SDIORB_ACT_WRITE, start_addr, mem_ptr, wb);
            if (ret)
            {
                BTS_ERR("error %d on writing %d membytes at 0x%08x",
                    ret, rd, start_addr);
                goto err;
            }

            BTS_Delay(BS_SDIORB_NEXT_DELAY);

            ret = wlan_membytes(p_wlan_bus, BS_SDIORB_ACT_WRITE, (start_addr + wb),
                (mem_ptr + wb), (wbc - wb));
            if (ret)
            {
                BTS_ERR("error %d on writing %d membytes at 0x%08x",
                    ret, rd, start_addr);
                goto err;
            }
        }
        memset(str, 0, BTFW_MAX_STR_LEN);
    }

err:
    if (mem_blk)
        kfree(mem_blk);

    if (hfd.ds)
        kfree(hfd.ds);

    if (str)
        kfree(str);

    if (fp)
        filp_close(fp, NULL);

    return ret;
}

/******************************************************************
 * Function: BTDLFW_Download_BTFW
 *
 * Description: Download FW for H1
 *
 * Parameter:
 *  TS_Data_t *usp: pointer of private data (TS_Data)
 *  char *fwpath:   pointer directing to firmware's path
 *
 * Return:
 *  int:
 *     error:   -EFAULT
 *     success: 0
 *
 *****************************************************************/
int BTDLFW_Download_BTFW(TS_Data_t *usp, char *fwpath)
{
    int ret = 0;
    int err = 0;

    unsigned char bl_ready = 0;
    unsigned int bl_ready_cnt = 0;
    #ifdef DEBUG
    uint8_t      data = 0;
    #endif
    
    /* Check if BL ready */
    while (!(bl_ready)) {
        BTS_Delay(FW_ROM_BOOT_DELAY);
        /* Read the REG to check bootloader is ready */
        bl_ready = SDIOD_D2H_MSG_BL_READY & sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_DEVICE_TO_HOST_MSG_0, &err);
        if (err)
        {
            BTS_ERR("%s: SDIOD_F3_DEVICE_TO_HOST_MSG_0 err %d", __FUNCTION__, err);
            return -EFAULT;
        }
        bl_ready_cnt++;
        if (bl_ready_cnt == FW_IORDY_CNT)
        {
            BTS_ERR("%s BLREADY timeout, cnt = %d BLREADY = 0x%x", __FUNCTION__, bl_ready_cnt, bl_ready);
            return -EFAULT;
        }
    }
    BTS_INFO("%s BLREADY cnt = %d BLREADY = 0x%x", __FUNCTION__, bl_ready_cnt, bl_ready);

#ifdef DEBUG
    data = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_DEVICE_TO_HOST_MSG_0, &err);
    if (err)
    {
        BTS_ERR("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_0 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    else
    {
        BTS_DBG("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_0 data %d", __FUNCTION__, data);
    }
    data = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_DEVICE_TO_HOST_MSG_1, &err);
    if (err)
    {
        BTS_ERR("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_1 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    else
    {
        BTS_DBG("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_1 data %d", __FUNCTION__, data);
    }
    data = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_DEVICE_TO_HOST_MSG_2, &err);
    if (err)
    {
        BTS_ERR("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_2 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    else
    {
        BTS_DBG("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_2 data %d", __FUNCTION__, data);
    }
    data = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_DEVICE_TO_HOST_MSG_3, &err);
    if (err)
    {
        BTS_ERR("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_3 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    else
    {
        BTS_DBG("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_3 data %d", __FUNCTION__, data);
    }
#endif

    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_HOST_TO_DEVICE_MSG_4, ((CONFIG_LOAD_ADDR >> 0) & 0xFF), &err);
    if (err)
    {
        BTS_ERR("%s: write SDIOD_F3_HOST_TO_DEVICE_MSG_4 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_HOST_TO_DEVICE_MSG_5, ((CONFIG_LOAD_ADDR >> 8) & 0xFF), &err);
    if (err)
    {
        BTS_ERR("%s: write SDIOD_F3_HOST_TO_DEVICE_MSG_5 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_HOST_TO_DEVICE_MSG_6, ((CONFIG_LOAD_ADDR >> 16) & 0xFF), &err);
    if (err)
    {
        BTS_ERR("%s: write SDIOD_F3_HOST_TO_DEVICE_MSG_6 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_HOST_TO_DEVICE_MSG_7, ((CONFIG_LOAD_ADDR >> 24) & 0xFF), &err);
    if (err)
    {
        BTS_ERR("%s: write SDIOD_F3_HOST_TO_DEVICE_MSG_7 err %d", __FUNCTION__, err);
        return -EFAULT;
    }

    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_HOST_TO_DEVICE_MSG_0, ((SDIOD_H2D_MSG_BOOT_FW_LOAD_STARTED >> 0) & 0xFF), &err);
    if (err)
    {
        BTS_ERR("%s: write SDIOD_F3_HOST_TO_DEVICE_MSG_0 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_HOST_TO_DEVICE_MSG_1, ((SDIOD_H2D_MSG_BOOT_FW_LOAD_STARTED >> 8) & 0xFF), &err);
    if (err)
    {
        BTS_ERR("%s: write SDIOD_F3_HOST_TO_DEVICE_MSG_1 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_HOST_TO_DEVICE_MSG_2, ((SDIOD_H2D_MSG_BOOT_FW_LOAD_STARTED >> 16) & 0xFF), &err);
    if (err)
    {
        BTS_ERR("%s: write SDIOD_F3_HOST_TO_DEVICE_MSG_2 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_HOST_TO_DEVICE_MSG_3, ((SDIOD_H2D_MSG_BOOT_FW_LOAD_STARTED >> 24) & 0xFF), &err);
    if (err)
    {
        BTS_ERR("%s: write SDIOD_F3_HOST_TO_DEVICE_MSG_3 err %d", __FUNCTION__, err);
        return -EFAULT;
    }

    ret = _BTDLFW_Download_BTFW(usp->p_wlan_bus, usp->fwpath);
    if (ret)
    {
        BTS_ERR("%s: BTFW download fail ret %d", __FUNCTION__, ret);
        return -EFAULT;
    }

    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_HOST_TO_DEVICE_MSG_0, ((SDIOD_H2D_MSG_BOOT_FW_DOWNLOADED >> 0) & 0xFF), &err);
    if (err)
    {
        BTS_ERR("%s: write SDIOD_F3_HOST_TO_DEVICE_MSG_0 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_HOST_TO_DEVICE_MSG_1, ((SDIOD_H2D_MSG_BOOT_FW_DOWNLOADED >> 8) & 0xFF), &err);
    if (err)
    {
        BTS_ERR("%s: write SDIOD_F3_HOST_TO_DEVICE_MSG_1 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_HOST_TO_DEVICE_MSG_2, ((SDIOD_H2D_MSG_BOOT_FW_DOWNLOADED >> 16) & 0xFF), &err);
    if (err)
    {
        BTS_ERR("%s: write SDIOD_F3_HOST_TO_DEVICE_MSG_2 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    sdio_bus_cfg_write(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_HOST_TO_DEVICE_MSG_3, ((SDIOD_H2D_MSG_BOOT_FW_DOWNLOADED >> 24) & 0xFF), &err);
    if (err)
    {
        BTS_ERR("%s: write SDIOD_F3_HOST_TO_DEVICE_MSG_3 err %d", __FUNCTION__, err);
        return -EFAULT;
    }

#ifdef DEBUG
    data = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_DEVICE_TO_HOST_MSG_0, &err);
    if (err)
    {
        BTS_ERR("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_0 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    else
    {
        BTS_DBG("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_0 data %d", __FUNCTION__, data);
    }
    data = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_DEVICE_TO_HOST_MSG_1, &err);
    if (err)
    {
        BTS_ERR("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_1 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    else
    {
        BTS_DBG("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_1 data %d", __FUNCTION__, data);
    }
    data = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_DEVICE_TO_HOST_MSG_2, &err);
    if (err)
    {
        BTS_ERR("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_2 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    else
    {
        BTS_DBG("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_2 data %d", __FUNCTION__, data);
    }
    data = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_DEVICE_TO_HOST_MSG_3, &err);
    if (err)
    {
        BTS_ERR("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_3 err %d", __FUNCTION__, err);
        return -EFAULT;
    }
    else
    {
        BTS_DBG("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_3 data %d", __FUNCTION__, data);
    }
#endif

    /* Check if FW DL ready */
    while (!(bl_ready & SDIOD_D2H_MSG_FW_TRANSPORT_READY)) {
        BTS_Delay(FW_ROM_BOOT_DELAY);
        /* Read the REG to check transport is ready */
        bl_ready = sdio_bus_cfg_read(usp->p_wlan_bus, SDIO_FUNC_3, SDIOD_F3_DEVICE_TO_HOST_MSG_0, &err);
        if (err)
        {
            BTS_ERR("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_0 err %d", __FUNCTION__, err);
            return -EFAULT;
        }
        else
        {
            BTS_DBG("%s: read SDIOD_F3_DEVICE_TO_HOST_MSG_0 data %d", __FUNCTION__, data);
        }
        bl_ready_cnt++;
        if (bl_ready_cnt == FW_IORDY_CNT)
        {
            BTS_ERR("%s FW READY timeout, cnt = %d BLREADY = 0x%x", __FUNCTION__, bl_ready_cnt, bl_ready);
            return -EFAULT;
        }
    }
    BTS_INFO("%s BLREADY cnt = %d BLREADY = 0x%x", __FUNCTION__, bl_ready_cnt, bl_ready);
    return ret;
}
//#endif
