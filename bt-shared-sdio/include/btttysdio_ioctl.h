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

#ifndef _IOC_BT_TTY_SDIO_H
#define _IOC_BT_TTY_SDIO_H

#include <asm/ioctls.h>
#include "bts_io_msg.h"

#define BTTTYSDIO_IOC_MAGIC     'H'

#define BTTTYSDIO_IOC_DLFW      _IOW(BTTTYSDIO_IOC_MAGIC,1,BTDLFW_Msg)
#define BTTTYSDIO_IOC_RMEM      _IOWR(BTTTYSDIO_IOC_MAGIC,2,BT_RWMem_Msg)
#define BTTTYSDIO_IOC_WMEM      _IOW(BTTTYSDIO_IOC_MAGIC,3,BT_RWMem_Msg)
#define BTTTYSDIO_IOC_RREG      _IOWR(BTTTYSDIO_IOC_MAGIC,4,BT_RWMem_Msg)
#define BTTTYSDIO_IOC_WREG      _IOW(BTTTYSDIO_IOC_MAGIC,5,BT_RWMem_Msg)
#define BTTTYSDIO_IOC_EINT      _IO(BTTTYSDIO_IOC_MAGIC,6)
#define BTTTYSDIO_IOC_CLK_ENA   _IO(BTTTYSDIO_IOC_MAGIC,7)
#define BTTTYSDIO_IOC_CLK_DIS   _IO(BTTTYSDIO_IOC_MAGIC,8)

#define BTTTYSDIO_IOC_LAST      BTTTYSDIO_IOC_EINT
#define BTTTYSDIO_IOC_MAX_NR    BTTTYSDIO_IOC_LAST

#endif  /* _IOC_BT_TTY_SDIO_H */
