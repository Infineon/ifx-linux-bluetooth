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
 * Definitions for the fast DCT.
 */

#ifndef SBC_DCT_H
#define SBC_DCT_H

#if (SBC_ARM_OPT==TRUE)
#define SBC_MULT_32_16_SIMPLIFIED(s16In2, s32In1, s32OutLow)					\
{																			    \
    __asm																		\
{																				\
    MUL s32OutLow,(s32In1>>15),(SINT32)s16In2							\
}																				\
}
#else
#if (SBC_C5402_OPT==TRUE)
#define SBC_MULT_32_16_SIMPLIFIED(s16In2, s32In1 , s32OutLow) SBC_Multiply_32_16_Simplified((SINT32)s16In2,s32In1,&s32OutLow);
#else
#if (SBC_IPAQ_OPT==TRUE)
#define SBC_MULT_32_16_SIMPLIFIED(s16In2, s32In1 , s32OutLow) s32OutLow=(SINT32)((SINT32)(s16In2)*(SINT32)(s32In1>>15));
#else
#define SBC_MULT_32_16_SIMPLIFIED(s16In2, s32In1 , s32OutLow)                   \
{                                                                               \
    s32In1Temp = s32In1;                                                        \
    s32In2Temp = (SINT32)s16In2;                                                \
                                                                                \
    /* Multiply one +ve and the other -ve number */                             \
    if (s32In1Temp < 0)                                                         \
    {                                                                           \
        s32In1Temp ^= 0xFFFFFFFF;                                               \
        s32In1Temp++;                                                           \
        s32OutLow  = (s32In2Temp * (s32In1Temp >> 16));                         \
        s32OutLow += (( s32In2Temp * (s32In1Temp & 0xFFFF)) >> 16);             \
        s32OutLow ^= 0xFFFFFFFF;                                                \
        s32OutLow++;                                                            \
    }                                                                           \
    else                                                                        \
    {                                                                           \
        s32OutLow  = (s32In2Temp * (s32In1Temp >> 16));                         \
        s32OutLow += (( s32In2Temp * (s32In1Temp & 0xFFFF)) >> 16);             \
    }                                                                           \
    s32OutLow <<= 1;                                                            \
}
#endif
#endif
#endif
#define SBC_MULT_64(s32In1, s32In2, s32OutLow, s32OutHi)  \
{\
        s32OutLow=(SINT32)(((SINT64)s32In1*(SINT64)s32In2)& 0x00000000FFFFFFFF);\
        s32OutHi=(SINT32)(((SINT64)s32In1*(SINT64)s32In2)>>32);\
}
#if 0
#define SBC_MULT_32_32(s32In2, s32In1, s32OutLow)                           \
{                                                                           \
    s32HiTemp = 0;                                                          \
    SBC_MULT_64(s32In2,s32In1 , s32OutLow, s32HiTemp);                      \
    s32OutLow   = (((s32OutLow>>15)&0x1FFFF) | (s32HiTemp << 17));          \
}
#else
#define SBC_MULT_32_32(s32In2, s32In1, s32OutLow)                           \
{                                                                           \
    s64Temp = ((SINT64) s32In2) * ((SINT64) s32In1)/ 0x80000000;            \
    s32OutLow = (SINT32) s64Temp;                                                    \
}
#endif
#endif
