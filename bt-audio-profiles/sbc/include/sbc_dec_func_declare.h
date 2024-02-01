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
 */

/** @file
 *
 * SBC decoder function declarations.
 */


#ifndef SBCFUNCDECLARE_H
#define SBCFUNCDECLARE_H

#include "sbc_decoder.h"
void Mult64(SINT32 s32In1, SINT32 s32In2, SINT32 *s32OutLow, SINT32 *s32OutHi);
void sbc_dec_bit_alloc_mono(SBC_DEC_PARAMS *CodecParams);
void sbc_dec_bit_alloc_ste(SBC_DEC_PARAMS *CodecParams);
SINT16 SBC_Decoder(SBC_DEC_PARAMS *strDecParams);
SINT16 SBC_Decoder_decoder (SBC_DEC_PARAMS *strDecParams, uint8_t * sbc_in, uint32_t len, SINT16 * pcm_out);
SINT16 SBC_DecoderZIR(SBC_DEC_PARAMS *strDecParams, short * pcm_out);
SINT16 SBC_Decoder_Init(SBC_DEC_PARAMS *);
SINT16 SBC_Decoder_PLC_Init(SBC_DEC_PARAMS *strDecParams);
void SbcSynthesisFilter(SBC_DEC_PARAMS *strDecParams);
SINT16 DecUnPacking(SBC_DEC_PARAMS *strDecParams);
void SbcSynthesisFilter4sb(SBC_DEC_PARAMS *strDecParams);
void SbcSynthesisFilter8sb(SBC_DEC_PARAMS *strDecParams);

#endif
