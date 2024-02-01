/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company)
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
 *
 * Bluetooth WICED Utility functions
 *
 */
#include "wiced_bt_utils.h"
#include "string.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include <stdlib.h>

#define wiced_hal_get_pseudo_rand_number rand

#ifndef __LONG_MAX__
#define __LONG_MAX__ 2147483647
#endif

#undef LONG_MAX
#define LONG_MAX __LONG_MAX__
/* Maximum value an `unsigned long int' can hold.  (Minimum is 0).  */
#undef ULONG_MAX
#define ULONG_MAX (LONG_MAX * 2UL + 1UL)


/*****************************************************************************
**  Constants
*****************************************************************************/

/* global constant for "any" bd addr */
BD_ADDR bd_addr_any = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
BD_ADDR bd_addr_null= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


/*****************************************************************************
**  Functions
*****************************************************************************/

//
// Implementation of standard char manipulation functions
// Currently only used here.  If necessary, can be exposed by removing static inline
//
static inline int
wiced_bt_utils_isupper(char c)
{
    return (c >= 'A' && c <= 'Z');
}

static inline int
wiced_bt_utils_isalpha(char c)
{
    return ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'));
}

static inline int
wiced_bt_utils_isspace(char c)
{
    return (c == ' ' || c == '\t' || c == '\n' || c == '\12');
}

static inline int
wiced_bt_utils_isdigit(char c)
{
    return (c >= '0' && c <= '9');
}

//
// Implementation of standard string manipulation functions
//
// Note:  strlen strncpy strcmp and strstr defined in chip ROMs
//        strchr defined in all chip ROMs except 20706A2 and 43012C0

int utl_strncmp(const char *s1, const char *s2, int n)
{
    for ( ; n > 0; s1++, s2++, --n)
    if (*s1 != *s2)
        return ((*(unsigned char *)s1 < *(unsigned char *)s2) ? -1 : +1);
    else if (*s1 == '\0')
        return 0;
    return 0;
}

char *utl_strcat(char *s1, const char *s2)
{
    utl_strcpy(&s1[strlen(s1)], (char *)s2);
    return s1;
}

char *utl_strcpy(char *p_dst, const char *p_src)
{
    register char *pd = p_dst;
    register const char *ps = p_src;

    while ( *ps )
        *pd++ = *ps++;

    *pd++ = 0;
    return ( p_dst );
}

char *utl_strrchr(char *s, int c)
{
    char* ret=0;
    do {
        if( *s == (char)c )
            ret=s;
    } while(*s++);
    return ret;
}

#if defined (CYW20706A2) || defined (CYW43012C0)
// strchr defined in ROM for most chips, define here for the missing ones
char *utl_strchr(const char *s, int c)
{
    const char ch = c;

    for ( ; *s != ch; s++)
        if (*s == '\0')
            return 0;
    return (char *)s;
}
#endif

char utl_toupper(char c)
{
    if( c>='a' && c<='z')
        return (c = c +'A' - 'a');
    else
        return c;
}

/*******************************************************************************
** Function         utl_str2int
** Description      This utility function converts a character string to an
**                  integer.  Acceptable values in string are 0-9.  If invalid
**                  string or string value too large, -1 is returned.  Leading
**                  spaces are skipped.
** Returns          Integer value or -1 on error.
*******************************************************************************/
int16_t utl_str2int(const char *p_s)
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
** Function         utl_itoa
** Description      This utility function converts a uint16_t to a string
**                  The string is NULL-terminated
** Returns          Length of string
*******************************************************************************/
uint8_t utl_itoa(uint16_t i, char *p_s)
{
    uint16_t    j, k;
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
    return (uint8_t) (p - p_s);
}

/*******************************************************************************
** Function         utl_strucmp
** Description      This utility function compares two strings in uppercase.
**                  String p_s must be uppercase.  String p_t is converted to
**                  uppercase if lowercase.  If p_s ends first, the substring
**                  match is counted as a match.
** Returns          0 if strings match, nonzero otherwise.
*******************************************************************************/
int utl_strucmp(const char *p_s, const char *p_t)
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
** Function             utl_strtoul
** Description          Convert a string of ASCII digits to unsigned long
**
** const char *nptr     String of ASCII digits, possibly preceded by white
**                      space.  For bases greater than 10, either lower- or
**                      upper-case digits may be used.
**
** char **endPtr        Where to store address of terminating character, or NULL
**
** int base;            Base for conversion.  Must be less than 37.  If 0, then the
**                      base is chosen from the leading characters of string:
**                      "0x" means hex, "0" means octal, anything else means decimal.
**
** Returns              converted unsigned long
*******************************************************************************/
unsigned long utl_strtoul(const char *nptr, char **endptr, int base)
{
     register const char *s;
     register unsigned long acc, cutoff;
     register int c;
     register int neg, any, cutlim;

     /*
      * See strtol for comments as to the logic used.
      */
     s = nptr;
     do {
         c = (unsigned char) *s++;
     } while (wiced_bt_utils_isspace(c));
     if (c == '-') {
         neg = 1;
         c = *s++;
     } else {
         neg = 0;
         if (c == '+')
             c = *s++;
     }
     if ((base == 0 || base == 16) && c == '0' && (*s == 'x' || *s == 'X')) {
         c = s[1];
         s += 2;
         base = 16;
     }
     if (base == 0)
         base = c == '0' ? 8 : 10;
     cutoff = ULONG_MAX / (unsigned long) base;
     cutlim = ULONG_MAX % (unsigned long) base;
     for (acc = 0, any = 0;; c = (unsigned char) *s++) {
         if (wiced_bt_utils_isdigit(c))
             c -= '0';
         else if (wiced_bt_utils_isalpha(c))
             c -= wiced_bt_utils_isupper(c) ? 'A' - 10 : 'a' - 10;
         else
             break;
         if (c >= base)
             break;
         if (any < 0)
             continue;
         if ((acc > cutoff || acc == cutoff) && c > cutlim) {
             any = -1;
             acc = ULONG_MAX;
         } else {
             any = 1;
             acc *= (unsigned long) base;
             acc += c;
         }
     }
     if (neg && any > 0)
         acc *= (-1);
     if (endptr != 0)
         *endptr = (char *) (any ? s - 1 : nptr);
     return (acc);
}


//
// Bluetooth Device Address (BD_ADDR) manipulation functions
//

/*******************************************************************************
** Function         utl_bdcpy
** Description      Copy bd addr b to a.
** Returns          void
*******************************************************************************/
void utl_bdcpy(BD_ADDR a, const BD_ADDR b)
{
    int i;

    for (i = BD_ADDR_LEN; i != 0; i--)
    {
        *a++ = *b++;
    }
}

/*******************************************************************************
** Function         utl_bdcmp
** Description      Compare bd addr b to a.
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
*******************************************************************************/
int utl_bdcmp(const BD_ADDR a, const BD_ADDR b)
{
    int i;

    for (i = BD_ADDR_LEN; i != 0; i--)
    {
        if (*a++ != *b++)
        {
            return -1;
        }
    }
    return 0;
}

/*******************************************************************************
** Function         utl_bdcmpany
** Description      Compare bd addr to "any" bd addr.
** Returns          Zero if a equals bd_addr_any.
*******************************************************************************/
int utl_bdcmpany(const BD_ADDR a)
{
    return utl_bdcmp(a, bd_addr_any);
}

/*******************************************************************************
** Function         utl_bdsetany
** Description      Set bd addr to "any" bd addr.
** Returns          void
*******************************************************************************/
void utl_bdsetany(BD_ADDR a)
{
    utl_bdcpy(a, bd_addr_any);
}

/*******************************************************************************
** Function         utl_freebuf
** Description      Frees a GKI buffer obtained from GKI_getbuf()
** Returns          void
*******************************************************************************/
void utl_freebuf(void **p)
{
    if (*p != NULL)
    {
        wiced_bt_free_buffer(*p);
        *p = NULL;
    }
}

/*******************************************************************************
** Function         utl_ignore_spaces
** Description      Removes spaces from mutable string
** Returns          void
*******************************************************************************/

void utl_ignore_spaces(char* str) {
    char* tmp = str;
    while(*str != '\0')
    {
        while (*tmp == ' ')
        {
            ++tmp;
        }
        *str++ = *tmp++;
    }
}


/*******************************************************************
 * Function         wiced_bt_read_raw_rssi
 *
 *                  returns the raw or actual RSSI
 *
 * @param[in]       connection_handle   : peer connection handle
 * @param[in]       p_callback_in       : application callback to receive RSSI
 *
 * @return          void
*******************************************************************/
wiced_bt_read_raw_rssi_command_complete_cback_t * p_callback = NULL;
void read_raw_rssi_callback(wiced_bt_dev_vendor_specific_command_complete_params_t *p_cmd_cplt_param)
{
    uint8_t* p_data = p_cmd_cplt_param->p_param_buf;
    WICED_BT_TRACE("opcode: %x\n", p_cmd_cplt_param->opcode);       //opcode 0xfc48
    WICED_BT_TRACE("param_len: %x\n", p_cmd_cplt_param->param_len); //length
    WICED_BT_TRACE("p_data - %02x\n", p_data[0]);                   //status
    WICED_BT_TRACE("p_data - %02x %02x\n", p_data[1], p_data[2]);   //connection_handle
    WICED_BT_TRACE("p_data - %02x (%d dB)\n", p_data[3], (signed char) p_data[3]); //rssi
    if(p_callback)
        p_callback(p_cmd_cplt_param);
}

#define opcode_vsc_read_raw_rssi 0xFC48
wiced_bt_dev_status_t wiced_bt_read_raw_rssi(uint16_t connection_handle, wiced_bt_read_raw_rssi_command_complete_cback_t *p_callback_in)
{
    wiced_bt_dev_status_t bt_status = WICED_BT_ERROR;
    uint8_t  buffer[2];
    uint8_t  *p = buffer;
    p_callback = p_callback_in;
    UINT16_TO_STREAM(p, connection_handle);
    WICED_BT_TRACE("opcode_vsc_read_raw_rssi:%x, buffer[0]:%02x, buffer[1]:%02x\n", opcode_vsc_read_raw_rssi, buffer[0], buffer[1]);
    bt_status = wiced_bt_dev_vendor_specific_command(opcode_vsc_read_raw_rssi, 2, buffer, read_raw_rssi_callback);
    if (bt_status != WICED_BT_PENDING)
    {
        return bt_status;
    }
    return WICED_BT_SUCCESS;
}

void wiced_hal_get_pseudo_rand_number_array(uint32_t* randNumberArrayPtr, uint32_t length)
{
    uint32_t i;
    uint32_t *p = randNumberArrayPtr;

    for (i = 0; i < length; i++)
    {
        *p++ = wiced_hal_get_pseudo_rand_number();
    }
}
