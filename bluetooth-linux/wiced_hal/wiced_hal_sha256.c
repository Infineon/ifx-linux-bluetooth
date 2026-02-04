/*
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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
/*******************************************************************************
*
* File Name: wiced_hal_sha256.c
*
* Abstract: sha-256 and hmac-sha256 implementation for simple pairing
*   Ref: http://csrc.nist.gov/CryptoToolkit/shs/sha256-384-512.pdf
*        http://en.wikipedia.org/wiki/HMAC
*        http://w3.antd.nist.gov/iip_pubs/draft-ietf-ipsec-ciph-sha-256-01.txt
*
*******************************************************************************/

#include "wiced_bt_ble.h"
#include "data_types.h"

#define BT_MEMSET memset
#define BT_MEMCPY memcpy

typedef struct
{
    uint32_t    H[8];
    uint8_t     buf[64];
    uint32_t    bufIdx;
    uint32_t    size;
} sha256_context_t;

static void sha256_block(uint32_t input[16], uint32_t H[8]);

///////////////////////////////////////////////////////////////////////////////
// SHA-256 implementation
///////////////////////////////////////////////////////////////////////////////

//PLACE_IN_DROM 
const uint32_t Hs[8] =   //232 times the square root of the first 8 primes 2..19
{
    0x5be0cd19,
    0x1f83d9ab,
    0x9b05688c,
    0x510e527f,
    0xa54ff53a,
    0x3c6ef372,
    0xbb67ae85,
    0x6a09e667,

};

//PLACE_IN_DROM 
const uint32_t Ks[64] =   //232 times the cube root of the first 64 primes 2..311
{
    0xc67178f2, 0xbef9a3f7, 0xa4506ceb, 0x90befffa, 0x8cc70208, 0x84c87814, 0x78a5636f, 0x748f82ee,
    0x682e6ff3, 0x5b9cca4f, 0x4ed8aa4a, 0x391c0cb3, 0x34b0bcb5, 0x2748774c, 0x1e376c08, 0x19a4c116,
    0x106aa070, 0xf40e3585, 0xd6990624, 0xd192e819, 0xc76c51a3, 0xc24b8b70, 0xa81a664b, 0xa2bfe8a1,
    0x92722c85, 0x81c2c92e, 0x766a0abb, 0x650a7354, 0x53380d13, 0x4d2c6dfc, 0x2e1b2138, 0x27b70a85, 
    0x14292967, 0x06ca6351, 0xd5a79147, 0xc6e00bf3, 0xbf597fc7, 0xb00327c8, 0xa831c66d, 0x983e5152, 
    0x76f988da, 0x5cb0a9dc, 0x4a7484aa, 0x2de92c6f, 0x240ca1cc, 0x0fc19dc6, 0xefbe4786, 0xe49b69c1, 
    0xc19bf174, 0x9bdc06a7, 0x80deb1fe, 0x72be5d74, 0x550c7dc3, 0x243185be, 0x12835b01, 0xd807aa98,     
    0xab1c5ed5, 0x923f82a4, 0x59f111f1, 0x3956c25b, 0xe9b5dba5, 0xb5c0fbcf, 0x71374491, 0x428a2f98,
};


#define RightRotate(w, n) (((w) >> (n)) | ((w) << (32 - (n))))
#define RightShift(w, n)    ((w) >> (n))


/*****************************************************************
* Function: sha256_init
*
* Abstract: initialize sha256 context
*
* Input/Output:
*     sha256_context_t* context
*
* Return:
*     void
*****************************************************************/
void sha256_init(sha256_context_t* context)
{
    BT_MEMSET(context, 0, sizeof(sha256_context_t));
    BT_MEMCPY(context->H, Hs, sizeof(Hs));
}

/*****************************************************************
* Function: sha256_update
*
* Abstract: consume more data
*
* Input/Output:
*    sha256_context_t* context
*    uint8_t* input
*    uint32_t input_len
*
* Return:
*    void
*****************************************************************/
void sha256_update(sha256_context_t* context, uint8_t* input, uint32_t input_len)
{
    BOOL32  done = FALSE;
    uint32_t  len;
    input += input_len;
    
    while(!done)
    {
        len = MIN(64 - context->bufIdx, input_len);
        BT_MEMCPY(context->buf + 64 - context->bufIdx - len, input - len, len);
        context->bufIdx += len;
        if(context->bufIdx == 64)
        {
            sha256_block((uint32_t*)context->buf, context->H);
            context->bufIdx = 0;
            context->size += 512;
        }
        input -= len;
        input_len -= len;
        if(input_len == 0)
            done = TRUE;
    }
}

/*****************************************************************
* Function: sha256_final
*
* Abstract: generate output data
*
* Input/Output:
*   sha256_context_t* context
*   uint32_t* output
*
* Return:
*   void
*****************************************************************/
void sha256_final(sha256_context_t* context, uint32_t* output)
{
    uint32_t i;

    if(context->bufIdx)
    {
        // we need to put pad there
        context->size += context->bufIdx * 8;
        context->buf[63 - context->bufIdx ++] = 0x80;

        if(context->bufIdx > 56)
        {
            for(i = context->bufIdx; i < 64; i ++)
            {
                context->buf[63 - i] = 0;
            }
            sha256_block((uint32_t*)context->buf, context->H);
            context->bufIdx = 0;
        }
        for(i = context->bufIdx; i < 62; i ++)
        {
            context->buf[63 - i] = 0;
        }
        
        // Put the size there
        context->buf[1] = context->size >> 8;
        context->buf[0] = context->size & 0xff;
        sha256_block((uint32_t*)context->buf, context->H);
    }
    BT_MEMCPY(output, context->H, sizeof(context->H));
}

/*****************************************************************
* Function: hmac_sha256
*
* Abstract: implementation of hmac_sha256
*
* Input/Output:
*    uint8_t* key
*    uint32_t key_len
*    uint8_t* inout
*    uint32_t input_len
*
* Return:
*    void
*****************************************************************/
void hmac_sha256(uint8_t* key, uint32_t key_len, uint8_t* inout, uint32_t input_len)
{
    uint32_t*  pad;
    uint32_t  i;
    sha256_context_t   context;

    sha256_init(&context);
    pad = (uint32_t*)context.buf;
    
    BT_MEMSET(pad, 0x0, 64);
    BT_MEMCPY(((uint8_t*)pad) + 64 - key_len, key, key_len);
    for(i = 0; i < 16; i ++)
    {
        pad[i] ^= 0x36363636;
    }

    sha256_update(&context, (uint8_t*)pad, 64);
    sha256_update(&context, inout, input_len);
    sha256_final(&context, (uint32_t*)inout);


    sha256_init(&context);
    BT_MEMSET(pad, 0x0, 64);
    BT_MEMCPY(((uint8_t*)pad) + 64 - key_len, key, key_len);
    for(i = 0; i < 16; i ++)
    {
        pad[i] ^= 0x5c5c5c5c;
    }

    sha256_update(&context, (uint8_t*)pad, 64);
    sha256_update(&context, (uint8_t*)inout, 32);
    sha256_final(&context, (uint32_t*)inout);

}



/////////////////////////////////////////////////////////////////////
// Local functions
/////////////////////////////////////////////////////////////////////


/*****************************************************************
* Function: sha256_block
*
* Abstract: for each block of 512 bit, process the data
*
* Input/Output:
*   uint32_t input[16]
*   uint32_t H[8]
*
* Return:
*   void
*****************************************************************/
static void sha256_block(uint32_t input[16], uint32_t H[8])
{
    uint32_t W[17];
    int32_t i, j;
    uint32_t s0, s1;
    uint32_t a, b, c, d, e, f, g, h;
    uint32_t maj;
    uint32_t t2, t1;
    uint32_t ch;

    BT_MEMCPY(W + 1, input, 64);
    
    //Initialize hash value for this chunk:
    a = H[7];
    b = H[6];
    c = H[5];
    d = H[4];
    e = H[3];
    f = H[2];
    g = H[1];
    h = H[0];

    //Main loop:
    for(i = 63; i >= 0; i --)
    {
        s0 = RightRotate(a, 2) ^ RightRotate(a, 13) ^ RightRotate(a, 22);
        maj = (a & b) ^ (a & c) ^ (b & c);
        t2 = s0 + maj;
        s1 = RightRotate(e, 6) ^ RightRotate(e, 11) ^ RightRotate(e, 25);
        ch = (e & f) ^ ((~e) & g);
        t1 = h + s1 + ch + Ks[i] + W[16];

        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;

        if(i >= 16)
        {
            s0 = RightRotate(W[15], 7) ^ RightRotate(W[15], 18) ^ RightShift(W[15], 3);
            s1 = RightRotate(W[2], 17) ^ RightRotate(W[2], 19) ^ RightShift(W[2], 10);
            W[0] = W[16] + s0 + W[7] + s1;
        }
        
        for(j = 16; j >= 1; j --)
        {
            W[j] = W[j-1];
        }
    

    }    

    //Add this chunk's hash to result so far:
    H[7] += a;
    H[6] += b;
    H[5] += c;
    H[4] += d;
    H[3] += e;
    H[2] += f;
    H[1] += g;
    H[0] += h;    
}
