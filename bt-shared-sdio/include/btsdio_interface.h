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

#ifndef _btsdio_interface.h_
#define _btsdio_interface.h_
#include <linux/mmc/sdio_func.h>

typedef enum {
	WLAN_MODULE = 0,
	BT_MODULE
} bus_owner_t;

typedef struct btsdio_info {
        uint32_t bt_buf_reg_addr;
        uint32_t host_ctrl_reg_addr;
        uint32_t bt_ctrl_reg_addr;
        uint32_t bt_buf_addr;
        uint32_t wlan_buf_addr;
} btsdio_info_t;

#define BTS_VER_MAJOR 1
#define BTS_VER_MINOR 1
#define BTS_VER_PATCH 1
#define BTS_VERSION (BTS_VER_MAJOR << 24 | BTS_VER_MINOR << 16 | BTS_VER_PATCH << 8)
typedef struct bt_wlan_shared_info {
    /* bt info */
    void *bt_data;
    void (*bt_int_fun)(void *data);
    /* wlan info */
    void *wlan_bus_if;
    u16 device_id;
    u32 enum_addr;
} bt_wlan_shared_info_t;

typedef  void * wlan_bt_handle_t;
typedef void (*bs_int_handler)(void *data);
typedef void (*dhd_hang_notification)(struct sdio_func *func, bool wifi_state);

/* Shared Layer Init function */
extern wlan_bt_handle_t dhd_bt_get_pub_hndl(void);

extern int dhd_download_btfw(wlan_bt_handle_t handle, char* btfw_path);
extern int dhd_bus_get(wlan_bt_handle_t handle, bus_owner_t owner);
extern int dhd_bus_put(wlan_bt_handle_t handle, bus_owner_t owner);
extern unsigned char dhd_bus_cfg_read(void *h, unsigned int fun_num, unsigned int addr, int *err);
extern void dhd_bus_cfg_write(void *h, unsigned int fun_num, unsigned int addr, unsigned char val, int *err);
extern int dhd_bus_recv_buf(void *h, uint32_t addr, unsigned int fn, uint8_t *buf, unsigned int nbytes);
extern int dhd_bus_send_buf(void *h, uint32_t addr, unsigned int fn, uint8_t *buf, unsigned int nbytes);
extern int dhd_bus_set_blocksize(void *h, unsigned int fun_num, unsigned int block_size);

extern uint32_t dhd_bus_reg_read(void *, uint32_t);
extern uint32_t ifx_bus_reg_read(void *, uint32_t);
extern void dhd_bus_reg_write(void *, uint32_t, uint32_t);
extern void ifx_bus_reg_write(void *, uint32_t, uint32_t);
extern int dhd_bus_membytes(void *, bool, uint32_t, uint8_t *, unsigned int);
extern int ifx_bus_membytes(struct brcmf_bus *bus_if, bool set, u32 address, u8 *data, unsigned int size);
extern wlan_bt_handle_t bt_sdio_attach(void *btdata, bs_int_handler bs_int_fun);
extern wlan_bt_handle_t brcmf_bt_sdio_attach(void *btdata, bs_int_handler bs_int_fun);
extern void bt_sdio_detach(wlan_bt_handle_t handle);
/*
 * Functions to be called from other layers to enable/disable Bus clock
 * can_wait - Callers pass TRUE, if they want & can wait until the
 * clock configuration takes effect (there is a register poll until the
 * PLLs are locked). If the caller cannot wait they can simply pass
 * FALSE.
 */
extern int dhd_bus_clk_enable(wlan_bt_handle_t handle, bus_owner_t owner);
extern int ifx_bus_clk_enable(struct brcmf_bus *bus_if);
extern int dhd_bus_clk_disable(wlan_bt_handle_t handle, bus_owner_t owner);
extern int ifx_bus_clk_disable(struct brcmf_bus *bus_if);
extern void dhd_bus_reset_bt_use_count(wlan_bt_handle_t handle);
extern int dhd_get_wlan_info(wlan_bt_handle_t handle, btsdio_info_t *bs_info);
extern int ifx_bus_attach(uint32_t ver, void *info);
extern void ifx_bus_detach(struct brcmf_bus *bus_if);
extern u8 ifx_bus_reg_readb(struct brcmf_bus *bus_if, u8 fn, u32 addr, int *err);
extern void ifx_bus_reg_writeb(struct brcmf_bus *bus_if, u8 fn, u32 addr, u8 val, int *err);
extern int ifx_bus_recv_buf(struct brcmf_bus *bus_if, u8 *buf, u32 nbytes);
extern int ifx_bus_send_buf(struct brcmf_bus *bus_if, u8 *buf, u32 nbytes);
extern u32 ifx_bus_reg_readl(struct brcmf_bus *bus_if, u32 addr, int *err);
extern void ifx_bus_reg_writel(struct brcmf_bus *bus_if, u32 addr, u32 val, int *err);
extern int ifx_bus_set_blocksz(struct brcmf_bus *bus_if, u16 blocksz);
int dhd_bus_set_blocksize(void *h, unsigned int fun_num, unsigned int block_size);
extern int dhd_btsdio_attach(uint32_t ver, void *info);
extern void dhd_btsdio_detach(struct brcmf_bus *bus_if);

#if defined(FMAC_BUILD) && (FMAC_BUILD == TRUE)
#define sdio_bus_reg_read ifx_bus_reg_readl
#define sdio_bus_reg_write ifx_bus_reg_write
#define wlan_membytes ifx_bus_membytes
#define sdio_bt_attach ifx_bus_attach
#define sdio_bt_detach ifx_bus_detach
#define sdio_bus_clk_enable ifx_bus_clk_enable
#define sdio_bus_clk_disable ifx_bus_clk_disable
#define sdio_bus_cfg_read ifx_bus_reg_readb
#define sdio_bus_cfg_write ifx_bus_reg_writeb
#define sdio_bus_recv_buf(_bus, addr, _fn, buf, nbytes) ifx_bus_recv_buf(_bus, buf, nbytes)
#define sdio_bus_send_buf(_bus, addr, _fn, buf, nbytes) ifx_bus_send_buf(_bus, buf, nbytes)
#define sdio_bus_set_blocksize(_bus, _fn, blocksz) ifx_bus_set_blocksz(_bus, blocksz)

#else
#define wlan_membytes dhd_bus_membytes

#define sdio_bus_reg_read dhd_bus_reg_read
#define sdio_bus_reg_write dhd_bus_reg_write

#define sdio_bus_cfg_read dhd_bus_cfg_read
#define sdio_bus_cfg_write dhd_bus_cfg_write

#define sdio_bus_recv_buf(_bus, addr, _fn, buf, nbytes) dhd_bus_recv_buf(_bus, addr, _fn, buf, nbytes)
#define sdio_bus_send_buf(_bus, addr, _fn, buf, nbytes) dhd_bus_send_buf(_bus, addr, _fn, buf, nbytes)
#define sdio_bus_set_blocksize(_bus, _fn, blocksz) dhd_bus_set_blocksize(_bus, _fn, blocksz)

#define sdio_bt_detach dhd_btsdio_detach
#define sdio_bt_attach bt_sdio_attach
#define sdio_bus_clk_enable dhd_bus_clk_enable
#define sdio_bus_clk_disable dhd_bus_clk_disable
#define sdio_bus_reset_bt_use_count dhd_bus_reset_bt_use_count
#define get_wlan_info dhd_get_wlan_info
#endif
#endif /* _btsdio_interface.h_ */
