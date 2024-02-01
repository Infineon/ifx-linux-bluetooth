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
 * This file contains Data type declarations.
 */

#ifndef SBC_TYPES_H
#define SBC_TYPES_H

typedef short SINT16;
typedef int SINT32;
#ifdef LINUX_PLATFORM
typedef long long SINT64;
#else
typedef __int64 SINT64;
#endif


#define abs32(x) ( (x >= 0) ? x : (-x) )

#ifndef DATA_TYPES_DEFINED
#define DATA_TYPES_DEFINED
//==================================================================================================
// Types
//==================================================================================================

//! Unsigned 8-bit integer.
typedef unsigned char uint8_t;

//! Signed 8-bit integer.
typedef signed char int8_t;

//! Unsigned 16-bit integer.
typedef unsigned short int uint16_t;

//! Signed 16-bit integer.
typedef signed short int int16_t;

//! Unsigned 32-bit integer.
typedef unsigned int uint32_t;
// NOTE: not long int, because on 64-bit compilers (and more to the point, coverity running on
// 64-bit linux), long int is treated as 64 bits.

//! Signed 32-bit integer.
typedef signed int int32_t;
// NOTE: not long int, because on 64-bit compilers (and more to the point, coverity running on
// 64-bit linux), long int is treated as 64 bits.

#ifndef LINUX_PLATFORM
//! Unsigned 64-bit integer.
typedef unsigned long long int uint64_t;
#endif

//! Byte type (unsigned 8-bit integer).
typedef unsigned char BYTE;

//! Boolean type in its most efficient form, for use in function arguments and return values.
typedef unsigned int BOOL32;

//! Boolean type in its most size-efficient form, for use in structures.
typedef unsigned char BOOL8;

#ifdef _64_BIT_
typedef INT64           INTPTR;
typedef uint64_t          UINTPTR;
typedef uint64_t          TIME_STAMP;
#else
typedef int32_t           INTPTR;
typedef uint32_t        UINTPTR;
typedef uint32_t        TIME_STAMP;
#endif

#endif

#endif
