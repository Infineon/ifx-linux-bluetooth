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

/******************************************************************************
 * File Name: lc3_codec.c 
 *
 * Description: This file include the wrapper layer of LC3 codec
 * 
 * Related Document: See README.md
 *
 ******************************************************************************/

#include "lc3.h"
#include "lc3_codec.h"

#include "stdio.h"
#include <stdlib.h>

#include "wiced_bt_trace.h"
#include "wiced_bt_types.h"

#include "log.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/

#ifdef TAG
#undef TAG
#define TAG "[lc3_codec]"
#endif

#define LC3_PLC_OPERATE     (1u)
#define LC3_WRONG_PARAMETER (-1)
#define LC3_SUCCESS         (0)
//STRIDE: count between two consecutives samples
//in LE-Audio CE we get left and right channel seperatly, and encode/decode left and right channel seperatly, so the STRIDE is 1
//if want to encode/decode interleaved left right samples, the stride should be 2
#define STRIDE              (1u)

lc3_encoder_t enc[ISO_AUDIO_MAX_PARAM_COUNT];
lc3_decoder_t dec[ISO_AUDIO_MAX_PARAM_COUNT];

lc3_config_t enc_audio_cfg[ISO_AUDIO_MAX_PARAM_COUNT];
lc3_config_t dec_audio_cfg[ISO_AUDIO_MAX_PARAM_COUNT];

/*******************************************************************************
*
*  Function: lc3_codec_reset
*
*  Description: free allocated codec resources
*
*  Input/Output: None
*
*  Return: None
*
*********************************************************************************/
void lc3_codec_reset(void)
{
    uint8_t index;

    for (index = 0; index < ISO_AUDIO_MAX_PARAM_COUNT; index++)
    {
        if (enc[index] != NULL)
        {
            lc3_codec_releaseEncoder(index);
            lc3_codec_releaseDecoder(index);
        }
    }
}

/*******************************************************************************
*
*  Function: lc3_codec_initializeDecoder
*
*  Description: the interface of google lc3 codec lc3_setup_coder api
*
*  Input/Output:
*   uint8_t index:
*       the index of decoder
*
*   lc3_config_t *p_lc3Config:
*       the decoder audio config
*
*  Return:
*   wiced_bool_t
*
*********************************************************************************/
wiced_bool_t lc3_codec_initializeDecoder(uint8_t index, lc3_config_t *p_lc3Config)
{
    TRACE_LOG("frame_us:%d, srate_hz:%d, octestPerFrame:%d, samepleWidthInBits:%d\n", p_lc3Config->sduInterval, p_lc3Config->sampleRate, p_lc3Config->octetsPerFrame, p_lc3Config->sampleWidthInBits);
    if (dec[index] != NULL)
    {
        TRACE_LOG("dec at index:%d not NULL, release it\n", index);
        lc3_codec_releaseDecoder(index);
    }
    dec[index] = lc3_setup_decoder(p_lc3Config->sduInterval, p_lc3Config->sampleRate, p_lc3Config->sampleRate,
                                    malloc(lc3_decoder_size(p_lc3Config->sduInterval, p_lc3Config->sampleRate)));

    memcpy(&dec_audio_cfg[index], p_lc3Config, sizeof(lc3_config_t));

    return dec[index] == NULL ? FALSE :TRUE;
}

/*******************************************************************************
*
*  Function: lc3_codec_releaseDecoder
*
*  Description: relase decoder and clear decoder audio config
*
*  Input/Output:
*   uint8_t index:
*       the decoder index
*
*  Return: None
*
*********************************************************************************/
void lc3_codec_releaseDecoder(uint8_t index)
{
    TRACE_LOG("index:%d\n", index);
    if (dec[index] != NULL)
    {
        free(dec[index]);
        dec[index] = NULL;
    }
    else
    {
        TRACE_ERR("dec[%d] is NULL\n", index);
    }
    memset(&dec_audio_cfg[index], 0, sizeof(lc3_config_t));

}

/*******************************************************************************
*
*  Function: lc3_codec_Decode
*
*  Description:  interface of google lc3_codec_decode api
*
*  Input/Output:
*   uint8_t index:
*       the decoder index
*
*   uint8_t pktStatus:
*       no use
*
*   void *inBuf:
*       the input lc3 data buffer
*
*   uint16_t inLenBytes
*       the input lc3 frame size in bytes
*
*   void *outBuf
*       the pcm output data buffer
*
*   uint16_t outLenBytes
*       the pcm data sizes in bytes, no use.
*
* Return: outLenBytes
*
*********************************************************************************/
uint32_t lc3_codec_Decode(uint8_t index, uint8_t pktStatus, void *inBuf, uint16_t inLenBytes, void *outBuf, uint16_t outLenBytes)
{
    int res = 0;
    uint8_t pcm_sbytes = dec_audio_cfg[index].sampleWidthInBits / 8;
    enum lc3_pcm_format pcm_fmt =
        pcm_sbytes == 32/8 ? LC3_PCM_FORMAT_S24 :
        pcm_sbytes == 24/8 ? LC3_PCM_FORMAT_S24_3LE : LC3_PCM_FORMAT_S16;

    memset(outBuf, 0, outLenBytes);

    res = lc3_decode(dec[index], inBuf, inLenBytes, pcm_fmt, outBuf, STRIDE);

    if (res == LC3_PLC_OPERATE)
    {
        TRACE_LOG("PLC Operated\n");
    }
    if (res == LC3_WRONG_PARAMETER)
    {
        TRACE_ERR("Wrong parameter\n");
    }

    return outLenBytes;
}

/*******************************************************************************
*
*  Function: lc3_codec_initializeEncoder
*
*  Description: interface of google lc3 codec lc3_setup_encoder api
*
*  Input/Output:
*   uint8_t index:
*       the encoder index
*
*   lc3_config_t *p_lc3Config:
*       the audio config
*
*  Return:
*       wiced_bool_t
*
*********************************************************************************/
wiced_bool_t lc3_codec_initializeEncoder(uint8_t index, lc3_config_t *p_lc3Config)
{
    TRACE_LOG("frame_us:%d, srate_hz:%d, octestPerFrame:%d, samepleWidthInBits:%d\n", p_lc3Config->sduInterval, p_lc3Config->sampleRate, p_lc3Config->octetsPerFrame, p_lc3Config->sampleWidthInBits);
    if (enc[index] != NULL)
    {
        TRACE_LOG("enc at index:%d not NULL, release it\n", index);
        lc3_codec_releaseEncoder(index);
    }
    enc[index] = lc3_setup_encoder(p_lc3Config->sduInterval, p_lc3Config->sampleRate, p_lc3Config->sampleRate,
                            malloc(lc3_encoder_size(p_lc3Config->sduInterval, p_lc3Config->sampleRate)));

    memcpy(&enc_audio_cfg[index], p_lc3Config, sizeof(lc3_config_t));

    return enc[index] == NULL ? FALSE : TRUE;
}

/*******************************************************************************
*
*  Function: lc3_codec_releaseEncoder
*
*  Description: release the allocate encoder, and clear the encdoer audio cfg
*
*  Input/Output:
*   uint8_t index:
*       the encoder index
*
*  Return: None
*
*********************************************************************************/
void lc3_codec_releaseEncoder(uint8_t index)
{
    TRACE_LOG("index:%d\n", index);
    if (enc[index] != NULL)
    {
        free(enc[index]);
        enc[index] = NULL;
    }
    else
    {
        TRACE_ERR("enc[%d] is NULL\n", index);
    }
    memset(&enc_audio_cfg[index], 0, sizeof(lc3_config_t));
}

/*******************************************************************************
*
*  Function: lc3_codec_Encode
*
*  Description: interface of google lc3 codec encode api
*
*  Input/Output:
*   uint8_t index:
*       the encoder index
*
*   void *inBuf:
*       the input pcm data buffer
*
*   uint16_t inLenBytes:
*       the input data Bytes, no use
*
*   void *outBuf:
*       the lc3 data output buffer
*
*   uint16_t outLenBytes:
*       the lc3 frame size in bytes, use this for google lc3 encode api
*
*  Return: outLenBytes, from input, dont care
*
*********************************************************************************/
uint32_t lc3_codec_Encode(uint8_t index, void *inBuf, uint16_t inLenBytes, void *outBuf, uint16_t outLenBytes)
{
    uint8_t pcm_sbytes = enc_audio_cfg[index].sampleWidthInBits / 8;
    enum lc3_pcm_format pcm_fmt =
        pcm_sbytes == 32/8 ? LC3_PCM_FORMAT_S24 :
        pcm_sbytes == 24/8 ? LC3_PCM_FORMAT_S24_3LE : LC3_PCM_FORMAT_S16;

    memset(outBuf, 0, outLenBytes);

    if (lc3_encode(enc[index], pcm_fmt, inBuf, STRIDE, outLenBytes, outBuf) != LC3_SUCCESS)
    {
        TRACE_ERR("lc3_encode fail\n");
    }

    return outLenBytes;
}

