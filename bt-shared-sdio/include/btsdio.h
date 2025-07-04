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

#ifndef	_BTSDIO_H
#define	_BTSDIO_H

#define BTFW_MEM_OFFSET         0x19000000
#define BT_RAM_GROUP5           0x00250000

/* BIT0 => WLAN Power UP and BIT1=> WLAN Wake */
#define BT2WLAN_PWRUP_WAKE      0x03
#define BT2WLAN_PWRUP_ADDR      0x640894	/* This address is specific to 43012B0 */

#define BTFW_MAX_STR_LEN            600
#define BTFW_DOWNLOAD_BLK_SIZE      (BTFW_MAX_STR_LEN/2 + 8)
#define BTFW_SD_ALIGN               32
#define BTSD_BUF_ALIGN              4

#define BTFW_ADDR_MODE_UNKNOWN      0
#define BTFW_ADDR_MODE_EXTENDED     1
#define BTFW_ADDR_MODE_SEGMENT      2
#define BTFW_ADDR_MODE_LINEAR32     3

#define BTFW_HEX_LINE_TYPE_DATA                     0
#define BTFW_HEX_LINE_TYPE_END_OF_DATA              1
#define BTFW_HEX_LINE_TYPE_EXTENDED_SEGMENT_ADDRESS 2
#define BTFW_HEX_LINE_TYPE_EXTENDED_ADDRESS         4
#define BTFW_HEX_LINE_TYPE_ABSOLUTE_32BIT_ADDRESS   5

#define BTSDIO_SD_ALIGN 32

#define BTSDIO_DHD_HANDLE_RETRY         3
#define BTSDIO_BT_FW_READY_RETRY        300
#define BTSDIO_RETRY_CHECK_BT_RB        5

#define BS_SDIORB_SIZE                  0x1000
#define BS_SDIORB_MASK                  (BS_SDIORB_SIZE - 1)

#define BTSDIO_HOSTW_OFFSET             0
#define BTSDIO_HOSTR_OFFSET             BS_SDIORB_SIZE

#define BS_REG_DATA_VALID_SHIFT         1
#define BS_REG_WAKE_BT_SHIFT            17
#define BS_REG_SW_RDY_SHIFT             24
#define BS_REG_FW_RDY_SHIFT             24
#define BS_REG_BT_AWAKE_SHIFT           8

/* short the following marco header BTSDIO to BS */
/* Define Ring buffer offsets address */
#define BS_OFFSET_HOST2BT_IN            0x00002000
#define BS_OFFSET_HOST2BT_OUT           0x00002004
#define BS_OFFSET_BT2HOST_IN            0x00002008
#define BS_OFFSET_BT2HOST_OUT           0x0000200C

#define BS_SDIORB_ACT_READ              0
#define BS_SDIORB_ACT_WRITE             1

#define BS_SDIORB_DELAY                 10
#define BS_SDIORB_NEXT_DELAY            1000

#define BS_CNTRL_FWDL_EVERY_OPEN_SH     0

#define BS_SDIORB_OCCPD(in,out)      \
    (CIRC_CNT(in,out,BS_SDIORB_SIZE))
#define BS_SDIORB_AVAIL(in,out)      \
    (CIRC_SPACE(in,out,BS_SDIORB_SIZE))

#define BTSDIO_BUS_TIMEOUT 1000 /* Request BT SDIO bus timeout in MS */

/* io_en */
#define SDIO_FUNC_ENABLE_1  0x02    /* function 1 I/O enable */
#define SDIO_FUNC_ENABLE_2  0x04    /* function 2 I/O enable */
#define SDIO_FUNC_ENABLE_3  0x08    /* function 3 I/O enable */
#define SDIO_FUNC_DISABLE_3 0xF0

/* misc defines */
#define SDIO_FUNC_0 0
#define SDIO_FUNC_1 1
#define SDIO_FUNC_2 2
#define SDIO_FUNC_3 3
    
/* SDIO Device CCCR offsets */
#define SDIOD_CCCR_IOEN     0x02
#define SDIOD_CCCR_IORDY    0x03
#define SDIOD_CCCR_INTEN    0x04
#define SDIOD_CCCR_INTPEND  0x05

#define SDIOD_CARD_CONTROL  0xF1

#define SDIOD_CARD_CONTROL_CIS_LOADED                (0x01 << 0)
#define SDIOD_CARD_CONTROL_WLAN_RESET_ON_RES         (0x01 << 1)
#define SDIOD_CARD_CONTROL_BT_RESET_ON_RES           (0x01 << 2)
#define SDIOD_CARD_CONTROL_WLAN_RESET_ON_F2_DIS      (0x01 << 3)
#define SDIOD_CARD_CONTROL_BT_RESET_ON_F3_DIS        (0x01 << 4)
#define SDIOD_CARD_CONTROL_SDIO_RESET_ON_WLAN_RESET  (0x01 << 5)
#define SDIOD_CARD_CONTROL_SDIO_RESET_ON_BT_RES      (0x01 << 6)

#define SDIOD_F3_INTERRUPT_PENDING  0x00013 /* Interrupt Pending */
#define SDIOD_F3_INTERRUPT_ENABLE   0x00014 /* Interrupt Enable */

/* define from SDIO Device Core Programmers' Guide */
#define SDIOD_F3_BT_FRAME_CTRL_0                        0x10021
#define SDIOD_F3_ENABLE_BUSY_INDICATION_BIT             (0x01 << 0)
#define SDIOD_F3_CRC_ERROR_ON_WRITE_OUT_OF_SYNC_BIT     (0x01 << 1)
#define SDIOD_F3_CRC_ERROR_ON_READ_OUT_OF_SYNC_BIT      (0x01 << 2)
#define SDIOD_F3_FORCE_WRITE_CRC_ERROR_TILL_RETRY_BIT   (0x01 << 3)
#define SDIOD_F3_FORCE_READ_CRC_ERROR_TILL_RETRY_BIT    (0x01 << 4)
#define SDIOD_F3_ENABLE_F3_WATERMARK_BIT                (0x01 << 5)
#define SDIOD_F3_RX_ALMOST_FULL_CNT                     0x10030

#define SDIOD_F3_HOST_TO_DEVICE_MSG_0 0x10031
#define SDIOD_F3_HOST_TO_DEVICE_MSG_1 0x10032
#define SDIOD_F3_HOST_TO_DEVICE_MSG_2 0x10033
#define SDIOD_F3_HOST_TO_DEVICE_MSG_3 0x10034
#define SDIOD_F3_HOST_TO_DEVICE_MSG_4 0x10035
#define SDIOD_F3_HOST_TO_DEVICE_MSG_5 0x10036
#define SDIOD_F3_HOST_TO_DEVICE_MSG_6 0x10037
#define SDIOD_F3_HOST_TO_DEVICE_MSG_7 0x10038

#define SDIOD_F3_DEVICE_TO_HOST_MSG_0 0x10039
#define SDIOD_F3_DEVICE_TO_HOST_MSG_1 0x1003A
#define SDIOD_F3_DEVICE_TO_HOST_MSG_2 0x1003B
#define SDIOD_F3_DEVICE_TO_HOST_MSG_3 0x1003C
#define SDIOD_F3_DEVICE_TO_HOST_MSG_4 0x1003D
#define SDIOD_F3_DEVICE_TO_HOST_MSG_5 0x1003E
#define SDIOD_F3_DEVICE_TO_HOST_MSG_6 0x1003F
#define SDIOD_F3_DEVICE_TO_HOST_MSG_7 0x10040

#define CONFIG_LOAD_ADDR                    0x80420000

/* H2D  Host to Device */
#define SDIOD_H2D_MSG_BOOT_FW_DOWNLOADED  0x00000002
#define SDIOD_H2D_MSG_BOOT_FW_DOWNLOAD_BATCH 0x00000010
#define SDIOD_H2D_MSG_BOOT_FW_LOAD_STARTED  0x00000001

/* D2H  Device to Host */
#define SDIOD_D2H_MSG_BL_READY              (0x01 << 1)
#define SDIOD_D2H_MSG_FW_TRANSPORT_READY    (0x01 << 2)
#define SDIOD_D2H_MSG_FW_VALIDATION_RESULT  (0x01 << 5)
#define SDIOD_D2H_MSG_FW_VALIDATION_DONE    (0x01 << 6)

#define SDIOD_D2H_MSG_BL_READY				 0x00000002
#define SDIOD_D2H_MSG_TRANSPORT_READY		 0x00000004
#define SDIOD_D2H_MSG_FW_DOWNLOAD_ACK	 	 0x00000080

#define SDIO_TO_SB_MAILBOX_BIT		    0x10000
#define SDIO_TO_SB_MAILBOX_DATA_BYTE0   0x10004
#define SDIO_TO_SB_MAILBOX_DATA_BYTE1   0x10005
#define SDIO_TO_SB_MAILBOX_DATA_BYTE2   0x10006
#define SDIO_TO_SB_MAILBOX_DATA_BYTE3   0x10007

#define SDIO_TO_HOST_MAILBOX_DATA_BYTE0 0x10008
#define SDIO_TO_HOST_MAILBOX_DATA_BYTE1 0x10009
#define SDIO_TO_HOST_MAILBOX_DATA_BYTE2 0x1000A
#define SDIO_TO_HOST_MAILBOX_DATA_BYTE3 0x1000B

#define REG_RDAT     0x00   /* Receiver Data */
#define REG_TDAT     0x00   /* Transmitter Data */
#define REG_PC_RRT   0x10   /* Read Packet Control */
#define REG_PC_WRT   0x11   /* Write Packet Control */
#define REG_RTC_STAT 0x12   /* Retry Control Status */
#define REG_RTC_SET  0x12   /* Retry Control Set */
#define REG_INTRD    0x13   /* Interrupt Indication */
#define REG_CL_INTRD 0x13   /* Interrupt Clear */
#define REG_EN_INTRD 0x14   /* Interrupt Enable */
#define REG_MD_STAT  0x20   /* Bluetooth Mode Status */

#define BTSDIO_INT_TX_BIT           0x10
#define BTSDIO_INT_RX_BIT           0x20
#define BTSDIO_INT_BITS             (BTSDIO_INT_TX_BIT | BTSDIO_INT_RX_BIT)
#define BTSDIO_INT_TOHOSTMAILBOX    0xF0
#define BTSDIO_INT_ENABLE           0x08

#define PACKET_HEADER               4
#define CONTINUE_PACKET_MASK        (1 << 7)
#define MAX_ACL_SIZE                1050
#define MOD_PARAM_PATHLEN           256
#define TX_FRAG_LEN                 448
#define FW_IORDY_DELAY              10 /* in msecs */
#define FW_READY_DELAY              100 /* Worst case Warm Reboot delay in milliseconds */
#define FW_ROM_BOOT_DELAY           100 /* in msecs */
#define FW_IORDY_CNT                1000 /* wait for 10000 msec till F3 is enabled */
#define FW_BLRDY_CNT                1000 /* wait for 10000 msec till BL is ready */

#endif /* _BCMBTSDIO_H */
