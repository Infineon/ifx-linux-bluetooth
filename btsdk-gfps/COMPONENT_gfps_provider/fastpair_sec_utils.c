/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
* The security functions that Google Fast Pair Service needs
*
*
*/
#include <stdlib.h>
#include "wiced_bt_trace.h"
#include "wiced_memory.h"

//SHA-256 declaration
typedef struct
{
    uint32_t    H[8];
    uint8_t     buf[64];
    uint32_t    bufIdx;
    uint32_t    size;
} sha256_context_t;

extern void sha256_init(sha256_context_t* context);
extern void sha256_update(sha256_context_t* context, uint8_t *input, uint32_t input_len);
extern void sha256_final(sha256_context_t* context, uint32_t* output);

//AES-128 declaration
typedef uint8_t uint_8t;
typedef uint_8t return_type;
typedef uint_8t length_type;

#define N_ROW                   4
#define N_COL                   4
#define N_BLOCK   (N_ROW * N_COL)
#define N_MAX_ROUNDS           14

typedef struct
{
    uint_8t ksch[(N_MAX_ROUNDS + 1) * N_BLOCK];
    uint_8t rnd;
} aes_context;

return_type smp_aes_set_key( const unsigned char key[],
                         length_type keylen,
                         aes_context ctx[1] );

return_type smp_aes_encrypt( const unsigned char in[N_BLOCK],
                         unsigned char out[N_BLOCK],
                         const aes_context ctx[1] );

return_type smp_aes_decrypt( const unsigned char in[N_BLOCK],
                        unsigned char out[N_BLOCK],
                        const aes_context ctx[1] );

//uECC declaration
#if (defined(CYW43012C0) || BTSTACK_VER >= 0x03000001)
#define KEY_LENGTH_DWORDS_P256  8
#define BT_OCTET32_LEN          32
struct _point
{
    uint32_t x[KEY_LENGTH_DWORDS_P256];
    uint32_t y[KEY_LENGTH_DWORDS_P256];
    uint32_t z[KEY_LENGTH_DWORDS_P256];
};
typedef struct _point Point;

extern void ECC_PM_B_NAF(Point *q, Point *p, uint32_t *n, uint32_t keyLength);
#else
#define uECC_BYTES   32
extern int uECC_shared_secret(const uint8_t public_key[uECC_BYTES*2],
                       const uint8_t private_key[uECC_BYTES],
                       uint8_t secret[uECC_BYTES]);
#endif

void reverse_input(uint8_t *in, uint16_t in_len, uint8_t *out)
{
    uint8_t *temp;
    uint16_t i;
    temp = wiced_memory_allocate(in_len);
    if (temp == NULL)
        return;
    memcpy(temp, in, in_len);
    for (i = 0; i < in_len; i++)
        out[i] = temp[in_len - i - 1];
    wiced_memory_free(temp);
}

void fastpair_sec_sha256(uint8_t *p_in, uint16_t in_len, uint8_t *p_out)
{
    #define SHA256_LEN  32
    sha256_context_t    context;
    uint8_t *p_new_in;

    p_new_in = wiced_memory_allocate(in_len);
    if (p_new_in == NULL)
        return;
    reverse_input(p_in, in_len, p_new_in);
    sha256_init(&context);
    sha256_update(&context, p_new_in, in_len);
    sha256_final(&context, (uint32_t *)p_out);
    reverse_input(p_out, SHA256_LEN, p_out);
    wiced_memory_free(p_new_in);
}

void fastpair_sec_aes_ecb_128_encrypt(uint8_t *p_dout, uint8_t *p_din, uint8_t *p_key)
{
    aes_context     aes[1];

    smp_aes_set_key(p_key, 16, aes);
    smp_aes_encrypt(p_din, p_dout, aes);
}

void fastpair_sec_aes_ecb_128_decrypt(uint8_t *p_dout, uint8_t *p_din, uint8_t *p_key)
{
    aes_context     aes[1];

    smp_aes_set_key(p_key, 16, aes);
    smp_aes_decrypt(p_din, p_dout, aes);
}

static void memcpy_r(void* to, const void* from, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        ((uint8_t*)to)[i] = ((uint8_t*)from)[len - 1 - i];
    }
}

int fastpair_sec_uecc_shared_secret(uint8_t *peer_pub_key, uint8_t *priv_key, uint8_t *secret)
{
#if (defined(CYW43012C0) || BTSTACK_VER >= 0x03000001)
    Point       peer_publ_key, new_publ_key;
    uint8_t     private_key[32];
    uint8_t     sec[32];

    WICED_BT_TRACE("run ROM secret\n");
    memcpy_r(private_key, priv_key, BT_OCTET32_LEN);
    memcpy_r(peer_publ_key.x, peer_pub_key, BT_OCTET32_LEN);
    memcpy_r(peer_publ_key.y, peer_pub_key + BT_OCTET32_LEN, BT_OCTET32_LEN);

    ECC_PM_B_NAF(&new_publ_key, &peer_publ_key, (uint32_t*)private_key, KEY_LENGTH_DWORDS_P256);

    memcpy_r(secret, new_publ_key.x, BT_OCTET32_LEN);

    return 1;

#else
    return uECC_shared_secret(peer_pub_key, priv_key, secret);
#endif
}
