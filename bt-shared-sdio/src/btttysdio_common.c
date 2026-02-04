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

#include <linux/module.h>
#include <linux/delay.h>
#include "btsdio_utils.h"
#include "bts_msg.h"

#define DUMP_BUF_SIZE           64
#define DUMP_MAX_SIZE           256
#define DUMP_CHAR_NUM_NL        16

/**
 * Bitwise 0 - 31
 * BTS_ERR_SHIFT                   0
 * BTS_WARNING_SHIFT               1
 * BTS_EVENT_SHIFT                 2
 * BTS_INFO_SHIFT                  3
 * BTS_DBG_SHIFT                   4
 * BTS_TRC_SHIFT                   5
 */
uint32_t btttysdio_msg_level =
    BTS_ERR_BIT | BTS_WARNING_BIT | BTS_DBG_BIT | BTS_TRC_BIT | BTS_INFO_BIT;

/******************************************************************
 * Function: BTS_Delay
 *
 * Description: Delay in microsecond
 *
 * Parameter:
 *  unsigned int usec: how many microsecond in delay
 *
 * Return:
 *  void
 *
 *****************************************************************/
void BTS_Delay(unsigned int usec)
{
    unsigned int d;

    while (usec > 0)
    {
        d = MIN(usec, 1000);
        udelay(d);
        usec -= d;
    }
}

/******************************************************************
 * Function: BTS_DumpData
 *
 * Description: Display data at kernel log, and the the maximum size shown is DUMP_MAX_SIZE(256)
 *
 * Parameter:
 *  char * rwstr:  pointer directing to data should be read
 *  uint8_t * ptr: pointer directing to buffer
 *  uint32_t size: original data size
 *
 * Return:
 *  void
 *
 *****************************************************************/
void BTS_DumpData(char * rwstr, uint8_t * ptr, uint32_t size)
{
    char     buf[DUMP_BUF_SIZE];
    char *   bufptr = buf;
    int      i;
    uint32_t bufsz  = (size > DUMP_MAX_SIZE) ? DUMP_MAX_SIZE : size;

    if (!(btttysdio_msg_level & BTS_TRC_BIT))
        return ;

    if (size > DUMP_MAX_SIZE)
        sprintf(bufptr, "(print at most %d bytes)", DUMP_MAX_SIZE);
    else
        sprintf(bufptr, "print %d bytes", size);

    printk("%s: %s", rwstr, bufptr);

    memset(buf, 0, DUMP_BUF_SIZE);
    for (i = 0; i < bufsz; i++)
    {
        if (((i + 1) % DUMP_CHAR_NUM_NL) == 1)
        {
            sprintf(bufptr, "%08x:  ", (uint32_t)(uintptr_t)&ptr[i]);
            bufptr += 11;
        }
        sprintf(bufptr, "%02x ", ptr[i]);
        bufptr += 3;

        if (((i + 1) % DUMP_CHAR_NUM_NL) == 0)
        {
            printk(buf);
            memset(buf, 0, DUMP_BUF_SIZE);
            bufptr = buf;
        }
        else if (((i + 1) % (DUMP_CHAR_NUM_NL / 2)) == 0)
        {
            bufptr[0] = ' ';
            bufptr++;
        }
    }

    if (((i + 1) % DUMP_CHAR_NUM_NL) == 0)
        printk("\n");
    else
        printk(buf);
}
