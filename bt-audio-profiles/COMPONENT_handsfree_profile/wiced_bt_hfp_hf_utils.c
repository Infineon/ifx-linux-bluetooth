/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company)
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @file
 *
 * This file contains utility functions for Handsfree.
 */

#include "wiced_bt_types.h"

/*******************************************************************************
** Function         wiced_bt_hfp_hf_utils_bdcpy
** Description      Copy bd addr b to a
** Returns          void
*******************************************************************************/
void wiced_bt_hfp_hf_utils_bdcpy(wiced_bt_device_address_t a,
    const wiced_bt_device_address_t b)
{
    int i;
    for(i = BD_ADDR_LEN; i!=0; i--)
    {
        *a++ = *b++;
    }
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_utils_bdcmp
** Description      Compare bd addr b to a
** Returns          Zero if b==a, nonzero otherwise (like memcmp)
*******************************************************************************/
int wiced_bt_hfp_hf_utils_bdcmp(const wiced_bt_device_address_t a,
    const wiced_bt_device_address_t b)
{
    int i;
    for(i = BD_ADDR_LEN; i!=0; i--)
    {
        if( *a++ != *b++ )
        {
            return -1;
        }
    }
    return 0;
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_utils_strucmp
** Description      This utility function compares two strings in uppercase.
**                  String p_s must be uppercase.  String p_t is converted to
**                  uppercase if lowercase.  If p_s ends first, the substring
**                  match is counted as a match.
** Returns          0 if strings match, nonzero otherwise.
*******************************************************************************/
int wiced_bt_hfp_hf_utils_strucmp(const char *p_s, const char *p_t)
{
    char c;

    while (*p_s && *p_t)
    {
        c = *p_t++;
        if (c >= 'a' && c <= 'z')
        {
            c -= 0x20;
        }
        if (*p_s++ != c)
        {
            return -1;
        }
    }
    /* if p_t hit null first, no match */
    if (*p_t == 0 && *p_s != 0)
    {
        return 1;
    }
    /* else p_s hit null first, count as match */
    else
    {
        return 0;
    }
}

/*******************************************************************************
**
** Function         wiced_bt_hfp_hf_utils_itoa
**
** Description      This utility function converts a uint16_t to a string.  The
**                  string is NULL-terminated.  The length of the string is
**                  returned;
**
**
** Returns          Length of string.
**
*******************************************************************************/
uint8_t wiced_bt_hfp_hf_utils_itoa(uint16_t i, char *p_s)
{
    uint16_t     j, k;
    char        *p = p_s;
    wiced_bool_t fill = FALSE;

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
              fill = TRUE;
            }
        }
    }
    *p = 0;
    return (uint8_t) (p - p_s);
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_utils_str2int
** Description      This utility function converts a character string to an
**                  integer.  Acceptable values in string are 0-9.  If invalid
**                  string or string value too large, -1 is returned.  Leading
**                  spaces are skipped.
** Returns          Integer value or -1 on error.
*******************************************************************************/
int16_t wiced_bt_hfp_hf_utils_str2int(const char *p_s)
{
    int32_t val = 0;

    for (;*p_s == ' ' && *p_s != 0; p_s++);

    if (*p_s == 0) return -1;

    for (;;)
    {
        if ((*p_s < '0') || (*p_s > '9')) return -1;

        val += (int32_t) (*p_s++ - '0');

        if (val > 32767) return -1;

        if (*p_s == 0)
        {
            return (int16_t) val;
        }
        else
        {
            val *= 10;
        }
    }
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_utils_str2uint16
** Description      This utility function converts a character string to an
**                  uint16_t value.  Acceptable values in string are 0-9.  If invalid
**                  string or string value too large, -1 is returned.  Leading
**                  spaces are skipped.
** Returns          Integer value or -1 on error.
*******************************************************************************/
int32_t wiced_bt_hfp_hf_utils_str2uint16(const char *p_s)
{
    int32_t val = 0;

    for (;*p_s == ' ' && *p_s != 0; p_s++);

    if (*p_s == 0) return -1;

    for (;;)
    {
        if ((*p_s < '0') || (*p_s > '9')) return -1;

        val += (int32_t) (*p_s++ - '0');

        if (val > 0xFFFF)
            return -1;

        if (*p_s == 0)
            return val;
        else
            val *= 10;
    }
}

char *wiced_bt_hfp_hf_utils_strcpy(char *p_dst, const char *p_src)
{
    register char *pd = p_dst;
    register const char *ps = p_src;

    while ( *ps )
        *pd++ = *ps++;

    *pd++ = 0;
    return ( p_dst );
}
