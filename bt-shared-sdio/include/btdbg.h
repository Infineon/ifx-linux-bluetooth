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

#ifndef __BT_DBG_H
#define __BT_DBG_H

#include "bts_msg.h"

extern uint32_t btttysdio_msg_level;

#define BTS_MODULE_NAME "BT_TTY_SDIO"
#ifdef DEBUG
#define BTS_ERR(fmt, ...)                                       \
        do                                                      \
        {                                                       \
            if (btttysdio_msg_level & BTS_ERR_BIT)              \
                printk(KERN_ERR "[%s] %s: " fmt,                \
                    BTS_MODULE_NAME,                            \
                    __FUNCTION__, ##__VA_ARGS__);               \
        } while (0)
#define BTS_WAR(fmt, ...)                                       \
        do                                                      \
        {                                                       \
            if (btttysdio_msg_level & BTS_WARNING_BIT)          \
                printk(KERN_WARNING "[%s] %s: " fmt,            \
                    BTS_MODULE_NAME,                            \
                    __FUNCTION__, ##__VA_ARGS__);               \
        } while (0)
#define BTS_EVT(fmt, ...)                                       \
        do                                                      \
        {                                                       \
            if (btttysdio_msg_level & BTS_EVENT_BIT)            \
                printk(KERN_DEBUG "[%s] %s: " fmt,              \
                    BTS_MODULE_NAME,                            \
                    __FUNCTION__, ##__VA_ARGS__);               \
        } while (0)
#define BTS_INFO(fmt, ...)                                      \
        do                                                      \
        {                                                       \
            if (btttysdio_msg_level & BTS_INFO_BIT)             \
                printk(KERN_DEBUG "[%s] %s: " fmt,              \
                    BTS_MODULE_NAME,                            \
                    __FUNCTION__, ##__VA_ARGS__);               \
        } while (0)
#define BTS_NOTI(fmt, ...)                                      \
        do                                                      \
        {                                                       \
                printk(KERN_NOTICE "[%s] " fmt,                 \
                    BTS_MODULE_NAME, ##__VA_ARGS__);            \
        } while (0)
#define BTS_DBG(fmt, ...)                                       \
        do                                                      \
        {                                                       \
            if (btttysdio_msg_level & BTS_DBG_BIT)              \
                printk(KERN_DEBUG "[%s] %s: " fmt,              \
                    BTS_MODULE_NAME,                            \
                    __FUNCTION__, ##__VA_ARGS__);               \
        } while (0)
#define BTS_TRC(fmt, ...)                                       \
        do                                                      \
        {                                                       \
            if (btttysdio_msg_level & BTS_TRC_BIT)              \
                printk(KERN_DEFAULT "[%s] %s: " fmt,            \
                    BTS_MODULE_NAME,                            \
                    __FUNCTION__, ##__VA_ARGS__);               \
        } while (0)
#else
#define BTS_ERR(fmt, ...)                                       \
        do                                                      \
        {                                                       \
            printk("[%s] %s: " fmt, BTS_MODULE_NAME,            \
                __FUNCTION__, ##__VA_ARGS__);                   \
        } while (0)
#define BTS_NOTI(fmt, ...)                                      \
        do                                                      \
        {                                                       \
            printk(KERN_NOTICE "[%s] " fmt,                     \
                BTS_MODULE_NAME, ##__VA_ARGS__);                \
        } while (0)
#define BTS_WAR(fmt, ...)
#define BTS_EVT(fmt, ...)
#define BTS_INFO(fmt, ...)
#define BTS_DBG(fmt, ...)
#define BTS_TRC(fmt, ...)
#endif  /* !DEBUG */

extern void BTS_DumpData(char *, uint8_t *, uint32_t);

#endif  /* __BT_DBG_H */
