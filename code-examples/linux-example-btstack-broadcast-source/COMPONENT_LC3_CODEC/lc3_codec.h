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
 * File Name: lc3_codec.h 
 *
 * Description: This file include the wrapper layer header of LC3 codec
 * 
 * Related Document: See README.md
 *
 ******************************************************************************/
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
