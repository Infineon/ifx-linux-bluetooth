/*
* $ Copyright Cypress Semiconductor $
*/
#pragma once

#include "wiced_data_types.h"

typedef struct
{
    uint16_t sampleRate;
    uint16_t sduInterval;
    uint16_t octetsPerFrame;
    uint8_t sampleWidthInBits;
} lc3_config_t;

#define ISO_AUDIO_MAX_PARAM_COUNT 4 // 2 channels per device for 2 devices

void lc3_codec_reset(void);

wiced_bool_t lc3_codec_initializeDecoder(uint8_t index, lc3_config_t *p_lc3Config);
void lc3_codec_releaseDecoder(uint8_t index);

wiced_bool_t lc3_codec_initializeEncoder(uint8_t index, lc3_config_t *p_lc3Config);
void lc3_codec_releaseEncoder(uint8_t index);

uint32_t lc3_codec_Encode(uint8_t index, void *inBuf, uint16_t inLenBytes, void *outBuf, uint16_t outLenBytes);
uint32_t lc3_codec_Decode(uint8_t index, uint8_t pktStatus, void *inBuf, uint16_t inLenBytes, void *outBuf, uint16_t outLenBytes);
