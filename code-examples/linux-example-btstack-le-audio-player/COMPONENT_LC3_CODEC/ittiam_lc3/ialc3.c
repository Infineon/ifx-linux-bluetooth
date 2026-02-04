/*******************************************************************************
* THIS INFORMATION IS PROPRIETARY TO BROADCOM CORP
*
* ------------------------------------------------------------------------------
*
* Copyright (c) 2007 Broadcom Corp.
*
*          ALL RIGHTS RESERVED
*
********************************************************************************
*
* File Name: ialc3.c
*
* Abstract:  This file provides iso_audio related definitions.
*
*
* Functions:
*
*******************************************************************************/

#include "ittiam_lc3/ia_apicmd_standards.h"
#include "ittiam_lc3/ia_lc3_dec_config_params.h"
#include "ittiam_lc3/ia_lc3_enc_config_params.h"
#include "lc3_codec.h"

#include "stdio.h"
#include <stdlib.h>

#include "wiced_bt_trace.h"
#include "wiced_bt_types.h"

#include "log.h"

#ifdef TAG
#undef TAG
#define TAG "[lc3_codec]"
#endif

// Macros for NULL checking
#define CHECK_FOR_NULL_AND_RETURN_VALUE(x, error_return_value)      \
    if (!x)                                                         \
    {                                                               \
        WICED_BT_TRACE_CRIT("[%s] %s is NULL\n", __FUNCTION__, #x); \
        return error_return_value;                                  \
    }

#define CHECK_FOR_NULL_AND_RETURN(x)                                \
    if (!x)                                                         \
    {                                                               \
        WICED_BT_TRACE_CRIT("[%s] %s is NULL\n", __FUNCTION__, #x); \
        return;                                                     \
    }

#define IA_NO_ERROR 0x00000000
#define IA_DECODE 1
#define IA_ENCODE 2
#define IA_MEMTYPE_INPUT 0x02
#define IA_MEMTYPE_OUTPUT 0x03
#define LC3_EV_DO_WORK 0x01

typedef enum
{
    IA_MEM_API,
    IA_MEM_TABLES,
    IA_MEM_TABLE1,
    IA_MEM_TABLE2,
    IA_MEM_TABLE3,
    IA_MEM_TABLE4,
    IA_MEM_LAST
} IA_MEMORY_TYPES_t;

typedef enum
{
    IA_CODEC_IDLE,
    IA_CODEC_INITIALIZED,
} IA_DECODER_STATES_t;

typedef struct
{
    void *allocatedMemory[IA_MEM_LAST];
    void *pInputBuf;
    void *pOutputBuf;
    uint16_t inputBufSize;
    uint16_t outputBufSize;
    uint16_t pcmWordSize;
    uint16_t codecState;
} IA_CODEC_CONTEXT_t;

#define IA_LC3_DEBUG 1

#if IA_LC3_DEBUG > 0
#include "stdio.h"
#endif

/**********************************************************************
** Externals
**********************************************************************/
uint32_t ia_lc3_dec_api(void *p_ia_module_obj, uint32_t i_cmd, uint32_t i_idx, void *pv_value);
uint32_t ia_lc3_enc_api(void *p_ia_module_obj, uint32_t i_cmd, uint32_t i_idx, void *pv_value);

/**********************************************************************
** Variables
**********************************************************************/

IA_CODEC_CONTEXT_t *ialc3_decoderContext[ISO_AUDIO_MAX_PARAM_COUNT] = {NULL, NULL};
IA_CODEC_CONTEXT_t *ialc3_encoderContext[ISO_AUDIO_MAX_PARAM_COUNT] = {NULL, NULL};

#if IA_LC3_DEBUG > 0
char ialc3_errBuf[64];
#endif // IA_LC3_DEBUG

/**********************************************************************
** Routines
**********************************************************************/

static uint32_t *ia_lc3_allocate_memory(uint32_t size, const char *desc)
{
    uint32_t *p = calloc(size, 1);
    CHECK_FOR_NULL_AND_RETURN_VALUE(p, NULL);

    WICED_BT_TRACE("[%s] Allocating %d bytes : 0x%p\n", desc, size, p);

    return p;
}

static void ia_lc3_free_memory(uint32_t *memPtr)
{
    WICED_BT_TRACE("freeing 0x%p\n", memPtr);
    free(memPtr);
}

/*******************************************************************************
*
*  Function: ialc3_clearDecoderContext()
*
*  Abstract: free allocated resources and initialize
*
*  Input/Output: Index
*
*
*  Return: None
*
*********************************************************************************/
static void ialc3_clearDecoderContext(uint8_t index, wiced_bool_t freeContext)
{
    if (ialc3_decoderContext[index] != NULL)
    {
        uint8_t i;

        // free any allocated memory
        for (i = 0; i < IA_MEM_LAST; i++)
        {
            if (NULL != ialc3_decoderContext[index]->allocatedMemory[i])
            {
                ia_lc3_free_memory(ialc3_decoderContext[index]->allocatedMemory[i]);
                ialc3_decoderContext[index]->allocatedMemory[i] = NULL;
            }
        }
        ialc3_decoderContext[index]->pInputBuf = NULL;
        ialc3_decoderContext[index]->pOutputBuf = NULL;
        ialc3_decoderContext[index]->inputBufSize = 0;
        ialc3_decoderContext[index]->outputBufSize = 0;
        ialc3_decoderContext[index]->codecState = IA_CODEC_IDLE;
        if (freeContext)
        {
            ia_lc3_free_memory((uint32_t *)(ialc3_decoderContext[index]));
            ialc3_decoderContext[index] = NULL;
        }
    }
}

/*******************************************************************************
*
*  Function: ialc3_clearEncoderContext()
*
*  Abstract: free allocated resources and initialize
*
*  Input/Output: Index
*
*
*  Return: None
*
*********************************************************************************/
static void ialc3_clearEncoderContext(uint8_t index, wiced_bool_t freeContext)
{
    uint8_t i;

    if (ialc3_encoderContext[index] != NULL)
    {
        // free any allocated memory
        for (i = 0; i < IA_MEM_LAST; i++)
        {
            if (NULL != ialc3_encoderContext[index]->allocatedMemory[i])
            {
                ia_lc3_free_memory(ialc3_encoderContext[index]->allocatedMemory[i]);
                ialc3_encoderContext[index]->allocatedMemory[i] = NULL;
            }
        }
        ialc3_encoderContext[index]->pInputBuf = NULL;
        ialc3_encoderContext[index]->pOutputBuf = NULL;
        ialc3_encoderContext[index]->inputBufSize = 0;
        ialc3_encoderContext[index]->outputBufSize = 0;
        ialc3_encoderContext[index]->codecState = IA_CODEC_IDLE;
        if (freeContext)
        {
            ia_lc3_free_memory((uint32_t *)(ialc3_encoderContext[index]));
            ialc3_encoderContext[index] = NULL;
        }
    }
}

/*******************************************************************************
*
*  Function: lc3_codec_reset()
*
*  Abstract: free allocated resources and initialize all codec instances
*
*  Input/Output: None
*
*
*  Return: None
*
*********************************************************************************/
void lc3_codec_reset(void)
{
    uint8_t index;

    for (index = 0; index < ISO_AUDIO_MAX_PARAM_COUNT; index++)
    {
        ialc3_clearDecoderContext(index, TRUE);
        ialc3_clearEncoderContext(index, TRUE);
    }
}

/*******************************************************************************
*
*  Function: ialc3_checkError()
*
*  Abstract: Check for errors and print error string
*
*  Input/Output: Codec and error code
*
*
*  Return: None
*
*********************************************************************************/
static wiced_bool_t ialc3_checkError(uint8_t codec, uint32_t code)
{
#if IA_LC3_DEBUG > 0
    uint32_t isFatal, errClass, errSubCode;
    uint8_t charIndex = 0;
#endif // IA_LC3_DEBUG

    if (code == IA_NO_ERROR)
    {
        return FALSE;
    }
#if IA_LC3_DEBUG > 0
    isFatal = (((uint32_t)code & 0x8000) >> 15);
    errClass = (((uint32_t)code & 0x7800) >> 11);
    errSubCode = (((uint32_t)code & 0x07FF));

    if (!isFatal)
    {
        charIndex = sprintf(&ialc3_errBuf[charIndex], "Non-");
    }
    if (codec == IA_DECODE)
    {
        charIndex = sprintf(&ialc3_errBuf[charIndex], "Fatal error: Ittiam lc3_dec ");
    }
    else
    {
        charIndex = sprintf(&ialc3_errBuf[charIndex], "Fatal error: Ittiam lc3_enc ");
    }
    switch (errClass)
    {
    case 0:
        sprintf(&ialc3_errBuf[charIndex], "API code:%d\n", errSubCode);
        break;
    case 1:
        sprintf(&ialc3_errBuf[charIndex], "Configuration code:%d\n", errSubCode);
        break;
    case 2:
        sprintf(&ialc3_errBuf[charIndex], "Initialization code:%d\n", errSubCode);
        break;
    case 3:
        sprintf(&ialc3_errBuf[charIndex], "Execution code:%d\n", errSubCode);
        break;
    default:
        sprintf(&ialc3_errBuf[charIndex], "class:%d code:%d\n", errClass, errSubCode);
        break;
    }
    printf("%s\n", ialc3_errBuf);
#endif // IA_LC3_DEBUG
    return TRUE;
}

/*********************************************************************************************************
** Decoder
*********************************************************************************************************/

/*******************************************************************************
*
*  Function: lc3_codec_initializeDecoder()
*
*  Abstract: Allocate all resources and initialize a decoder
*
*  Input/Output: index
*
*
*  Return: Success or failure
*
*********************************************************************************/
wiced_bool_t lc3_codec_initializeDecoder(uint8_t codec_index, lc3_config_t *p_lc3Config)
{
    uint32_t errCode;
    uint32_t memSize, param, i;
    void *p_iaProcessApiObj = NULL;
    IA_CODEC_CONTEXT_t *context = NULL;

    // if needed allocate a context block
    if (ialc3_decoderContext[codec_index] == NULL)
    {
        ialc3_decoderContext[codec_index] =
            (IA_CODEC_CONTEXT_t *)ia_lc3_allocate_memory(sizeof(IA_CODEC_CONTEXT_t), __FUNCTION__);
        if (ialc3_decoderContext[codec_index] == NULL)
        {
            return FALSE;
        }
        ialc3_clearDecoderContext(codec_index, FALSE);
    }
    // set the current context
    context = ialc3_decoderContext[codec_index];
    // don't re-initialize or double allocate memory
    if (context->codecState != IA_CODEC_IDLE)
    {
        TRACE_ERR("codec IA_CODEC_INITIALIZED\n");
        return TRUE;
    }
    // get the API size
    errCode = ia_lc3_dec_api(NULL, IA_API_CMD_GET_API_SIZE, 0, &memSize);
    // allocate the API memory
    if (FALSE == ialc3_checkError(IA_DECODE, errCode))
    {
        context->allocatedMemory[IA_MEM_API] = ia_lc3_allocate_memory(memSize, __FUNCTION__);
        if (NULL == context->allocatedMemory[IA_MEM_API])
        {
            goto cleanUp;
        }
        p_iaProcessApiObj = context->allocatedMemory[IA_MEM_API];
    }
    else
    {
        goto cleanUp;
    }
    // default the codec parameters
    errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_INIT,
                             IA_CMD_TYPE_INIT_API_PRE_CONFIG_PARAMS,
                             NULL);
    if (TRUE == ialc3_checkError(IA_DECODE, errCode))
    {
        goto cleanUp;
    }

    // set the sample width
    context->pcmWordSize = p_lc3Config->sampleWidthInBits;
    // set the codec parameters
    param = p_lc3Config->sampleWidthInBits;
    errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_SET_CONFIG_PARAM, IA_LC3_DEC_CONFIG_PARAM_PCM_WDSZ, &param);
    if (TRUE == ialc3_checkError(IA_DECODE, errCode))
    {
        goto cleanUp;
    }
    param = p_lc3Config->sampleRate;
    errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_SET_CONFIG_PARAM, IA_LC3_DEC_CONFIG_PARAM_SAMP_FREQ, &param);
    if (TRUE == ialc3_checkError(IA_DECODE, errCode))
    {
        goto cleanUp;
    }
    param = 1;
    errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_SET_CONFIG_PARAM, IA_LC3_DEC_CONFIG_PARAM_NUM_CHANNELS, &param);
    if (TRUE == ialc3_checkError(IA_DECODE, errCode))
    {
        goto cleanUp;
    }
    param = 800 * p_lc3Config->octetsPerFrame;
    if (p_lc3Config->sduInterval == 7500)
    {
        param = (4 * param) / 3;
    }
    if (p_lc3Config->sampleRate == 44100)
    {
        param = (147 * param) / 160;
    }
    errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_SET_CONFIG_PARAM, IA_LC3_DEC_CONFIG_PARAM_BITRATE, &param);
    if (TRUE == ialc3_checkError(IA_DECODE, errCode))
    {
        goto cleanUp;
    }
    param = p_lc3Config->sduInterval / 100;
    if (p_lc3Config->sampleRate == 44100)
    {
        param = (25 * param) / 27;
    }
    errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_SET_CONFIG_PARAM, IA_LC3_DEC_CONFIG_PARAM_FRAME_MS, &param);
    if (TRUE == ialc3_checkError(IA_DECODE, errCode))
    {
        goto cleanUp;
    }
    // get info table memory size
    errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_GET_MEMTABS_SIZE, 0, &memSize);
    if (TRUE == ialc3_checkError(IA_DECODE, errCode))
    {
        goto cleanUp;
    }
    context->allocatedMemory[IA_MEM_TABLES] = ia_lc3_allocate_memory(memSize, __FUNCTION__);
    if (NULL == context->allocatedMemory[IA_MEM_TABLES])
    {
        goto cleanUp;
    }
    errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_SET_MEMTABS_PTR,
                             0, context->allocatedMemory[IA_MEM_TABLES]);
    if (TRUE == ialc3_checkError(IA_DECODE, errCode))
    {
        goto cleanUp;
    }
    // register all the parameters
    errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_INIT, IA_CMD_TYPE_INIT_API_POST_CONFIG_PARAMS, NULL);
    if (TRUE == ialc3_checkError(IA_DECODE, errCode))
    {
        goto cleanUp;
    }
    // get number of memory tables required
    errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_GET_N_MEMTABS, 0, &param);
    if (TRUE == ialc3_checkError(IA_DECODE, errCode))
    {
        goto cleanUp;
    }
    for (i = 0; i < param; i++)
    {
        uint32_t alignment, type;
        uint32_t *p_mem;

        // get memory size, alignment and type
        errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_GET_MEM_INFO_SIZE, i, &memSize);
        errCode |= ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_GET_MEM_INFO_ALIGNMENT, i, &alignment);
        errCode |= ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_GET_MEM_INFO_TYPE, i, &type);
        if (IA_NO_ERROR != errCode)
        {
            goto cleanUp;
        }
        if (alignment > 4)
        {
            memSize += alignment - 4;
        }
        context->allocatedMemory[IA_MEM_TABLE1 + i] = ia_lc3_allocate_memory(memSize, __FUNCTION__);
        if (NULL == context->allocatedMemory[IA_MEM_TABLE1 + i])
        {
            goto cleanUp;
        }
        p_mem = context->allocatedMemory[IA_MEM_TABLE1 + i];
        if (alignment > 4)
        {
            p_mem = (uint32_t *)(((uintptr_t)p_mem + (alignment - 4)) & ~(uintptr_t)(alignment - 1));
        }
        errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_SET_MEM_PTR, i, p_mem);
        if (TRUE == ialc3_checkError(IA_DECODE, errCode))
        {
            goto cleanUp;
        }
        if (type == IA_MEMTYPE_INPUT)
        {
            context->pInputBuf = p_mem;
            context->inputBufSize = memSize;
        }
        if (type == IA_MEMTYPE_OUTPUT)
        {
            context->pOutputBuf = p_mem;
            context->outputBufSize = memSize;
        }
    }

    // initialize process
    do
    {
        errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_INIT, IA_CMD_TYPE_INIT_PROCESS, NULL);
        errCode |= ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_INIT, IA_CMD_TYPE_INIT_DONE_QUERY, &param);
        if (errCode != IA_NO_ERROR)
        {
            goto cleanUp;
        }
    } while (!param);
    context->codecState = IA_CODEC_INITIALIZED;
    return TRUE;

cleanUp:
    ialc3_clearDecoderContext(codec_index, TRUE);
    return FALSE;
}

/*******************************************************************************
*
*  Function: lc3_codec_releaseDecoder()
*
*  Abstract: release all decoder resources
*
*  Input/Output: index
*
*
*  Return: None
*
*********************************************************************************/
void lc3_codec_releaseDecoder(uint8_t index)
{
    TRACE_LOG("\n");
    ialc3_clearDecoderContext(index, TRUE);
}

/*******************************************************************************
*
*  Function: ia_lc3_getDecoder()
*
*  Abstract: Extract output from decoder based on PCM word size
*
*  Input/Output: pb_out_buf, p_pcm_out, num_bytes, pcm_wd_sz
*
*
*  Return: None
*
*********************************************************************************/
static void ia_lc3_getDecoder(int32_t *pb_out_buf, void *p_pcm_out, uint16_t num_bytes, uint16_t pcm_wd_sz)
{
    uint32_t i, num_samples;

    // 16-bit sample size
    if (pcm_wd_sz == 16)
    {
        int16_t *p_pcm_s16le;

        p_pcm_s16le = (int16_t *)p_pcm_out;
        num_samples = num_bytes >> 1;
        for (i = 0; i < num_samples; i++)
        {
            p_pcm_s16le[i] = (int16_t)(pb_out_buf[i]);
        }
    }
    else // 24-bit sample size
    {
        int8_t *p_pcm_s24le;

        // this assumes that 24-bit samples should be byte packed
        p_pcm_s24le = (int8_t *)p_pcm_out;
        num_samples = num_bytes / 3;
        for (i = 0; i < num_samples; i++)
        {
            *p_pcm_s24le++ = (int8_t)((pb_out_buf[i] << 24) >> 24);
            *p_pcm_s24le++ = (int8_t)((pb_out_buf[i] << 16) >> 24);
            *p_pcm_s24le++ = (int8_t)((pb_out_buf[i] << 8) >> 24);
        }
    }
}

/*******************************************************************************
*
*  Function: ialc3_Decode()
*
*  Abstract: decode an LC3 packet into samples
*
*  Input/Output: index, input buf, size, output buf, size
*
*
*  Return: Number of bytes filled
*
*********************************************************************************/
uint32_t lc3_codec_Decode(uint8_t index, uint8_t pktStatus, void *inBuf, uint16_t inLenBytes, void *outBuf, uint16_t outLenBytes)
{
    IA_CODEC_CONTEXT_t *context = NULL;
    void *p_iaProcessApiObj = NULL;
    uint32_t param;
    uint32_t errCode;

    // fill the output buffer with silence
    memset(outBuf, 0, outLenBytes);

    // get the decoder context
    context = ialc3_decoderContext[index];
    if ((context == NULL) || (context->codecState != IA_CODEC_INITIALIZED))
    {
        ialc3_checkError(IA_DECODE, 0xE8);
        return outLenBytes;
    }
    // point to the API memory
    p_iaProcessApiObj = context->allocatedMemory[IA_MEM_API];
    // check the input buffer size against what was allocated
    if ((4 * inLenBytes) <= context->inputBufSize)
    {
        // move the packet to the input buffer
        memcpy(context->pInputBuf, inBuf, inLenBytes);
        // set the input buffer size
        param = inLenBytes;
        errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_SET_INPUT_BYTES, 0, &param);
        if (TRUE == ialc3_checkError(IA_DECODE, errCode))
        {
            return outLenBytes;
        }
        param = 0;
        errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_SET_BFI_EXT, 0, &param);
        if (TRUE == ialc3_checkError(IA_DECODE, errCode))
        {
            return outLenBytes;
        }
        // process the buffer
        errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_EXECUTE, IA_CMD_TYPE_DO_EXECUTE, NULL);
        if (TRUE == ialc3_checkError(IA_DECODE, errCode))
        {
            return outLenBytes;
        }
        // get bytes processed
        errCode = ia_lc3_dec_api(p_iaProcessApiObj, IA_API_CMD_GET_OUTPUT_BYTES, 0, &param);
        if (TRUE == ialc3_checkError(IA_DECODE, errCode))
        {
            return outLenBytes;
        }
        if (param != outLenBytes)
        {
            ialc3_checkError(IA_DECODE, 0xE9);
            WICED_BT_TRACE("[param : %d] [outLenBytes : %d]\n", param, outLenBytes);
            return outLenBytes;
        }
        // Decode takes so long that the context may have been deleted while it was running
        if ((NULL != ialc3_decoderContext[index]) && (NULL != context->pOutputBuf))
        {
            ia_lc3_getDecoder(context->pOutputBuf, outBuf, outLenBytes, context->pcmWordSize);
        }
    }
    return outLenBytes;
}

/*********************************************************************************************************
** Encoder
*********************************************************************************************************/

/*******************************************************************************
*
*  Function: lc3_codec_initializeEncoder()
*
*  Abstract: Allocate all resources and initialize an encoder
*
*  Input/Output: index
*
*
*  Return: Success or failure
*
*********************************************************************************/
wiced_bool_t lc3_codec_initializeEncoder(uint8_t index, lc3_config_t *p_lc3Config)
{
    uint32_t errCode;
    uint32_t memSize, param, i;
    void *p_iaProcessApiObj = NULL;
    IA_CODEC_CONTEXT_t *context = NULL;

    // if needed allocate a context block
    if (ialc3_encoderContext[index] == NULL)
    {
        ialc3_encoderContext[index] =
            (IA_CODEC_CONTEXT_t *)ia_lc3_allocate_memory(sizeof(IA_CODEC_CONTEXT_t), __FUNCTION__);
        if (ialc3_encoderContext[index] == NULL)
        {
            return FALSE;
        }
        ialc3_clearEncoderContext(index, FALSE);
    }
    // set the current context
    context = ialc3_encoderContext[index];
    // don't re-initialize or double allocate memory
    if (context->codecState != IA_CODEC_IDLE)
    {
        return TRUE;
    }
    // get the API size
    errCode = ia_lc3_enc_api(NULL, IA_API_CMD_GET_API_SIZE, 0, &memSize);
    // allocate the API memory
    if (FALSE == ialc3_checkError(IA_ENCODE, errCode))
    {
        context->allocatedMemory[IA_MEM_API] = ia_lc3_allocate_memory(memSize, __FUNCTION__);
        if (NULL == context->allocatedMemory[IA_MEM_API])
        {
            goto cleanUp;
        }
        p_iaProcessApiObj = context->allocatedMemory[IA_MEM_API];
    }
    else
    {
        goto cleanUp;
    }
    // default the codec parameters
    errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_INIT,
                             IA_CMD_TYPE_INIT_API_PRE_CONFIG_PARAMS,
                             NULL);
    if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
    {
        goto cleanUp;
    }

    // set the sample width
    context->pcmWordSize = p_lc3Config->sampleWidthInBits;
    // set input to PCM data not wave data
    param = 1;
    errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_SET_CONFIG_PARAM, IA_LC3_ENC_CONFIG_PARAM_PCM, &param);
    if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
    {
        goto cleanUp;
    }
    // set the codec parameters
    param = p_lc3Config->sampleWidthInBits;
    errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_SET_CONFIG_PARAM, IA_LC3_ENC_CONFIG_PARAM_PCM_WDSZ, &param);
    if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
    {
        goto cleanUp;
    }
    param = 1;
    errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_SET_CONFIG_PARAM, IA_LC3_ENC_CONFIG_PARAM_NUM_CHANNELS, &param);
    if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
    {
        goto cleanUp;
    }
    param = p_lc3Config->sampleRate;
    errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_SET_CONFIG_PARAM, IA_LC3_ENC_CONFIG_PARAM_SAMP_FREQ, &param);
    if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
    {
        goto cleanUp;
    }
    param = 800 * p_lc3Config->octetsPerFrame;
    if (p_lc3Config->sduInterval == 7500 || p_lc3Config->sduInterval == 8163)
    {
        param = ((4 * param) / 3) + 1;
    }
    if (p_lc3Config->sampleRate == 44100)
    {
        param = (147 * param) / 160;
    }
    errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_SET_CONFIG_PARAM, IA_LC3_ENC_CONFIG_PARAM_BITRATE, &param);
    if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
    {
        goto cleanUp;
    }
    param = p_lc3Config->sduInterval / 100;
    if (p_lc3Config->sampleRate == 44100)
    {
        param = (25 * param) / 27;
    }
    errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_SET_CONFIG_PARAM, IA_LC3_ENC_CONFIG_PARAM_FRAME_MS, &param);
    if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
    {
        goto cleanUp;
    }
    // get info table memory size
    errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_GET_MEMTABS_SIZE, 0, &memSize);
    if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
    {
        goto cleanUp;
    }
    context->allocatedMemory[IA_MEM_TABLES] = ia_lc3_allocate_memory(memSize, __FUNCTION__);
    if (NULL == context->allocatedMemory[IA_MEM_TABLES])
    {
        goto cleanUp;
    }
    errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_SET_MEMTABS_PTR,
                             0, context->allocatedMemory[IA_MEM_TABLES]);
    if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
    {
        goto cleanUp;
    }
    // register all the parameters
    errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_INIT, IA_CMD_TYPE_INIT_API_POST_CONFIG_PARAMS, NULL);
    if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
    {
        goto cleanUp;
    }
    // get number of memory tables required
    errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_GET_N_MEMTABS, 0, &param);
    if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
    {
        goto cleanUp;
    }
    for (i = 0; i < param; i++)
    {
        uint32_t alignment, type;
        uint32_t *p_mem;

        // get memory size, alignment and type
        errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_GET_MEM_INFO_SIZE, i, &memSize);
        errCode |= ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_GET_MEM_INFO_ALIGNMENT, i, &alignment);
        errCode |= ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_GET_MEM_INFO_TYPE, i, &type);

        printf("memSize %d alignment %d type %d\n", memSize, alignment, type);

        if (alignment > 4)
        {
            memSize += alignment - 4;
        }
        if (IA_NO_ERROR != errCode)
        {
            goto cleanUp;
        }
        context->allocatedMemory[IA_MEM_TABLE1 + i] = ia_lc3_allocate_memory(memSize, __FUNCTION__);
        if (NULL == context->allocatedMemory[IA_MEM_TABLE1 + i])
        {
            goto cleanUp;
        }
        p_mem = context->allocatedMemory[IA_MEM_TABLE1 + i];

        if (alignment > 4)
        {
            p_mem = (uint32_t *)(((uintptr_t)p_mem + (alignment - 4)) & ~(uintptr_t)(alignment - 1));
        }

        errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_SET_MEM_PTR, i, p_mem);
        if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
        {
            goto cleanUp;
        }

        if (type == IA_MEMTYPE_INPUT)
        {
            context->pInputBuf = p_mem;
            context->inputBufSize = memSize;
        }

        if (type == IA_MEMTYPE_OUTPUT)
        {
            context->pOutputBuf = p_mem;
            context->outputBufSize = memSize;
        }
    }

    // initialize process
    errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_INIT, IA_CMD_TYPE_INIT_PROCESS, NULL);
    if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
    {
        goto cleanUp;
    }
    context->codecState = IA_CODEC_INITIALIZED;
    return TRUE;

cleanUp:
    ialc3_clearEncoderContext(index, TRUE);
    return FALSE;
}

/*******************************************************************************
*
*  Function: lc3_codec_releaseEncoder()
*
*  Abstract: release all encoder resources
*
*  Input/Output: index
*
*
*  Return: None
*
*********************************************************************************/
void lc3_codec_releaseEncoder(uint8_t index)
{
    TRACE_LOG("\n");
    ialc3_clearEncoderContext(index, TRUE);
}

/*******************************************************************************
*
*  Function: ia_lc3_loadEncoder()
*
*  Abstract: Load input to encoder based on PCM word size
*
*  Input/Output: pIn_buf, pPcm_buf, num_bytes, pcm_wd_sz
*
*
*  Return: None
*
*********************************************************************************/
static void ia_lc3_loadEncoder(int32_t *pIn_buf, void *pPcm_buf, uint16_t num_bytes, uint16_t pcm_wd_sz)
{
    uint32_t i, num_samples;

    // 16-bit case
    if (pcm_wd_sz == 16)
    {
        int16_t *p_pcm_s16le;

        p_pcm_s16le = (int16_t *)pPcm_buf;
        num_samples = num_bytes >> 1;
        for (i = 0; i < num_samples; i++)
        {
            pIn_buf[i] = (int32_t)(p_pcm_s16le[i]);
        }
    }
    else // 24-bit case
    {
        int8_t *p_pcm_s24le;
        int32_t val;

        // this code assumes that 24-bit samples are byte packed
        // they maybe in 32-bit container already
        p_pcm_s24le = (int8_t *)pPcm_buf;
        num_samples = num_bytes / 3;
        for (i = 0; i < num_samples; i++)
        {
            val = *p_pcm_s24le++;
            val = (val << 8) | *p_pcm_s24le++;
            val = (val << 8) | *p_pcm_s24le++;
            if (val >= 0x800000)
            {
                val |= 0xff000000;
            }
            pIn_buf[i] = (int32_t)(val);
        }
    }
}

/*******************************************************************************
*
*  Function: ialc3_Encode()
*
*  Abstract: Encode samples into an LC3 packet
*
*  Input/Output: index, input buf, size, output buf, size
*
*
*  Return: Number of bytes filled
*
*********************************************************************************/
uint32_t lc3_codec_Encode(uint8_t index, void *inBuf, uint16_t inLenBytes, void *outBuf, uint16_t outLenBytes)
{
    IA_CODEC_CONTEXT_t *context = NULL;
    void *p_iaProcessApiObj = NULL;
    uint32_t param;
    uint32_t errCode;

    // WICED_BT_TRACE("[%s] index %d outLenBytes %d", __FUNCTION__, index, outLenBytes);

    // fill the output buffer with silence
    memset(outBuf, 0, outLenBytes);

    // get the encoder context
    context = ialc3_encoderContext[index];
    if ((context == NULL) || (context->codecState != IA_CODEC_INITIALIZED))
    {
        ialc3_checkError(IA_DECODE, 0xE1);
        return outLenBytes;
    }
    // point to the API memory
    p_iaProcessApiObj = context->allocatedMemory[IA_MEM_API];
    // check the input buffer size against what was allocated
    if ((4 * inLenBytes) <= context->inputBufSize)
    {
        // move the packet to the input buffer
        ia_lc3_loadEncoder(context->pInputBuf, inBuf, inLenBytes, context->pcmWordSize);
        // process the buffer
        errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_EXECUTE, IA_CMD_TYPE_DO_EXECUTE, NULL);
        if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
        {
            return outLenBytes;
        }
        // get bytes processed
        errCode = ia_lc3_enc_api(p_iaProcessApiObj, IA_API_CMD_GET_OUTPUT_BYTES, 0, &param);
        if (TRUE == ialc3_checkError(IA_ENCODE, errCode))
        {
            return outLenBytes;
        }
        if (param != outLenBytes)
        {
            printf("[param : %d] [outLenBytes : %d]\n", param, outLenBytes);
            ialc3_checkError(IA_DECODE, 0xE2);
            return outLenBytes;
        }
        // Encode takes so long that the context may have been deleted while it was running
        if ((NULL != ialc3_encoderContext[index]) && (NULL != context->pOutputBuf))
        {
            memcpy(outBuf, context->pOutputBuf, outLenBytes);
        }
    }
    return outLenBytes;
}
