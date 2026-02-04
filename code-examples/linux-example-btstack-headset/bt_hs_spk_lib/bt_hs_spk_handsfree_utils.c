/*
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*/

/** @file
 *
 * This file implements the utility functions for handsfree
 */

#include "bt_hs_spk_handsfree_utils.h"

#include <stdlib.h>
#include <string.h>

#include "bt_hs_spk_control.h"


#define UTIL_AM_VOL_LEVEL_HIGH 10
/*
 * This utility counts the characteers in a string
 * Returns  number of characters ( excluding the terminating '0' )
 */
int utl_strlen( char *p_str )
{
    register int  xx = 0;

    while ( *p_str++ != 0 )
        xx++;

    return ( xx );
}

int util_itoa(int i, char *p_s)
{
    int         j, k;
    char        *p = p_s;
    int         fill = 0;

    if (i == 0)
    {
        /* take care of zero case */
        *p++ = '0';
    }
    else
    {
        for(j = 10000; j > 0; j /= 10)
        {
            k = i / j;
            i %= j;
            if (k > 0 || fill)
            {
              *p++ = k + '0';
              fill = 1;
            }
        }
    }
    *p = 0;
    return (int) (p - p_s);
}

/**
 *
 * Volume levels passed from the application to Audio Manager should be in the range 0 to 10
 * calculating from 0 to 15 levels to 0 to 10 levels
 *
 * @param           int32_t  : vol from app.
 *
 * @return          volume in AM level
 */
int32_t bt_hs_spk_handsfree_utils_hfp_volume_to_am_volume(int32_t vol)
{
    uint32_t remainder;
    int32_t am_level;

    am_level    = (vol * UTIL_AM_VOL_LEVEL_HIGH) / HFP_VOLUME_HIGH;
    remainder   = (vol * UTIL_AM_VOL_LEVEL_HIGH) % HFP_VOLUME_HIGH;

    if (remainder >= UTIL_AM_VOL_LEVEL_HIGH)
    {
        am_level++;
    }

    return am_level;
}
