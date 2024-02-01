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
 * This file has the BitAlloc function declaration.
 */


#ifndef SBC_DECODER_H
#define SBC_DECODER_H

// Based on 0018
#define DECODER_VERSION 004

#ifdef BUILDCFG 
    #include "bt_target.h"
#endif


#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE (!FALSE)
#endif


/* For SBC WB */
enum
{
	SBC_DEC_MODE_A2DP,		// Original SBC for A2DP
	SBC_DEC_MODE_WB,			// SBC for WideBand
	SBC_DEC_MODE_MONO_A2DP	// Mono A2DP : PCM output is always Mono / Fs 16 kHz 
};

/* up/down sampling ratio for Mono A2DP */
#define SBC_sf32000_DS 2
#define SBC_sf44100_DS 11
#define SBC_sf44100_US 4
#define SBC_sf48000_DS 3
 
/* Mono A2DP */
#define MONO_A2DP_FILTER_LEN 64



#define SBC_HDR_SYNC		0x9C
#define SBC_WBS_HDR_SYNC	0xAD

#define SBC_H2_HDR_LEN			2	// H2 header is 2 bytes

//                                     -----------------------------------	
//					    | SBC packet = 57 Bytes                          | 
//                                     -----------------------------------	
// | H2 header = 2 Bytes  | 28 Bytes for SBC |    | 29 Bytes for SBC |  1 Byte Padding  | 
//  ------------------------------------	    --------------------------------	
// |       EV3 # 1 packet = 28+2 = 30 Bytes    |    |  EV3 # 2 packet = 29+1= 30 Bytes  |
//  ------------------------------------	    --------------------------------	

#define SBC_EV3_1_LEN			28	// SBC packet length in first EV3 packet
#define SBC_EV3_2_LEN			29	// SBC packet length in second EV3 packet


#define SBC_NB_OF_UUI 1
#define SBC_MAX_NUM_OF_BLOCKS_WB   15
#define SBC_MAX_NUM_OF_RETRANS	2
#define SBC_MAX_NUM_OF_ESCO_FRAME (1+SBC_MAX_NUM_OF_RETRANS)  /* Assume 1 SBC frame + N retransmission */
#define SBC_HEADER_SIZE 4 /* sync byte, ch mode/num subb/num blk byte, bit pool byte, and crc */

#ifndef SBC_MAX_PACKET_LENGTH
#define SBC_MAX_PACKET_LENGTH 520
#endif

#define SBC_WB_MAX_PCM_SAMPLES 120	// 2EV3 case, with 8 sb and 15 blk = 120 samples 
#define SBC_WB_MAX_PACKET_LEN 57		// 2EV3 case, with 8 sb and 15 blk = 57 bytes 


/*constants used for index calculation*/
#define CHL	SBC_MAX_NUM_OF_SUBBANDS
#define BLK	(SBC_MAX_NUM_OF_CHANNELS*CHL)

#define DEC_VX_MINIMUM_BUFFER_SIZE (SBC_MAX_NUM_OF_SUBBANDS*SBC_MAX_NUM_OF_CHANNELS*20)

/* Set SET_SYNC_ON_ESCO_FAIL to TRUE to enable overwriting the sync field */
/* in the case that the esco CRC fails */
#ifndef SBC_SET_SYNC_ON_ESCO_FAIL
#define SBC_SET_SYNC_ON_ESCO_FAIL TRUE
#endif

/* Set SBC_SET_RESERVED_ON_ESCO_FAIL to TRUE to track the reserved bytes and   */
/* over-write the values with prior received values if it is found that the    */
/* received values never change for correctly received frames (eSCO CRC passes */
#ifndef SBC_SET_RESERVED_ON_ESCO_FAIL
#define SBC_SET_RESERVED_ON_ESCO_FAIL TRUE
#endif

/* Set SBC_PLC to TRUE to enable WB-PLC for 16kHz SBC speech stream */
#ifndef SBC_PLC
#define SBC_PLC FALSE
#endif

/* Set SBC_BEC_1_2 to TRUE to enable BEC-1 and BEC-2 */
#ifndef SBC_BEC_1_2
#define SBC_BEC_1_2 FALSE
#endif

/* Set SBC_HEADER_BEC to TRUE to enable SBC Header BEC (aka BEC-0) */
#ifndef SBC_HEADER_BEC
#define SBC_HEADER_BEC FALSE
#endif

/* Set SBC_NEW_API  to TRUE to enable New API requested by FW team. */
#ifndef SBC_NEW_API
#define SBC_NEW_API TRUE
#endif

/* Set SBC_WB to TRUE to enable Wide Band Speech SBC Codec */
#ifndef SBC_WB
#define SBC_WB	TRUE
#endif

#ifndef DEC_VX_BUFFER_SIZE
#define DEC_VX_BUFFER_SIZE (DEC_VX_MINIMUM_BUFFER_SIZE+(20*SBC_SUB_BANDS_8))
/*#define DEC_VX_BUFFER_SIZE DEC_VX_MINIMUM_BUFFER_SIZE*/
#endif

/* this  SBC_ARM_OPT flag should not be change, it woudl degrate the quality bellow the sig accptance criteria */
#ifndef SBC_ARM_OPT
#define SBC_ARM_OPT FALSE
#endif 

/* this  SBC_ARM_INLINE_ASM it to used only if SBC_ARM_OPT == TRUE. please refer to the above comment */
#ifndef SBC_ARM_INLINE_ASM
#define SBC_ARM_INLINE_ASM FALSE
#endif 

#ifndef SBC_C5402_OPT
#define SBC_C5402_OPT FALSE
#endif

#ifndef SBC_OPTIMIZATION
#define SBC_OPTIMIZATION TRUE 
#endif 

/* 32 and 64 bit mult will be performed using SINT64 ( usualy __int64 ) cast that usualy give optimal performance if supported */
#ifndef SBC_IPAQ_OPT
#define SBC_IPAQ_OPT TRUE
#endif 

/* TRUE to perform the dequantification 32x32, 16x32 operation in 64 bit, false to perform those in 32 bit */
#ifndef SBC_IS_64_BITS_IN_DEQUANTIZER
#define SBC_IS_64_BITS_IN_DEQUANTIZER TRUE
#endif

/* TRUE to perform the windowing MAC 32x32, 16x32 operation in 64 bit, false to perform those in 32 bit */
#ifndef SBC_IS_64_BITS_IN_WINDOWING
#define SBC_IS_64_BITS_IN_WINDOWING FALSE
#endif

/* TRUE to hold the VX vector in 16 bit, FALSE for 32 bit */
#ifndef SBC_VECTOR_VX_ON_16BITS
#define SBC_VECTOR_VX_ON_16BITS FALSE
#endif

/* TRUE to perform the DCT 32x32, 16x32 operation in 64 bit, false to perform those in 32 bit */
#ifndef SBC_IS_64_MULT_IN_DCT
#define SBC_IS_64_MULT_IN_DCT TRUE
#endif

/* 32 and 64 bit mult will be performed using SINT64 ( long long ) cast that usualy give optimal performance if supported */
#ifndef SBC_FOR_EMBEDDED_LINUX
#define SBC_FOR_EMBEDDED_LINUX FALSE
#endif

#ifndef SBC_GKI_BUFFERBASED
#define SBC_GKI_BUFFERBASED FALSE
#endif /* SBC_GKI_BUFFERBASED */

#ifndef SBC_MINIMUM_BUFFER_COPIES
#define SBC_MINIMUM_BUFFER_COPIES FALSE
#endif

/* this SBC_DEC_SANITY_CHECK verify that we have enough byte to decode before starting the decoding */
/* this is sometime necessary when the applicaiton is not able to provide the data require when they call sbc decoder */
#ifndef SBC_DEC_SANITY_CHECK
#define SBC_DEC_SANITY_CHECK FALSE
#endif

/* exclude/include 4 subband code */
#ifndef SBC_DEC_4_SUBD_INCLUDED
#define SBC_DEC_4_SUBD_INCLUDED TRUE
#endif
/* exclude/include 8 subband code */
#ifndef SBC_DEC_8_SUBD_INCLUDED
#define SBC_DEC_8_SUBD_INCLUDED TRUE
#endif
/* exclude/include stereo code */
#ifndef SBC_DEC_STEREO_INCLUDED
#define SBC_DEC_STEREO_INCLUDED TRUE
#endif
/* exclude/include MONO A2DP code */
#ifndef SBC_DEC_MONO_A2DP_INCLUDED
#define SBC_DEC_MONO_A2DP_INCLUDED FALSE
#endif


#if (SBC_NEW_API== TRUE && SBC_OPTIMIZATION==FALSE) 
#error SBC_NEW_API only available if SBC_OPTIMIZATION = TRUE !!
#endif



/*DEFINES*/
#define SBC_CRC_FAILURE -1
#define SBC_SYNC_FAILURE -2
#define SBC_EMPTY_FAILURE -3
#define SBC_MEM_FAILURE -4
#define SBC_BAD_WB_UUI -5
#define SBC_PLC_FAILURE -6
#define SBC_INIT_FAILURE -7
#define SBC_WAIT_FOR_LAST_FRAME -8
#define SBC_WAIT_FOR_NEXT_EV3 -9
#define SBC_FAILURE 0
#define SBC_SUCCESS 1

#define SBC_NULL 0

#define SBC_MAX_NUM_OF_SUBBANDS	8
#define	SBC_MAX_NUM_OF_CHANNELS	2

 #define SBC_MAX_NUM_OF_BLOCKS	16
 
#if (SBC_NEW_API)
#define PCM_BUF 0
#define SBC_BUF 0
#else
#define PCM_BUF (SBC_MAX_NUM_OF_BLOCKS*SBC_MAX_NUM_OF_SUBBANDS)
#define SBC_BUF (SBC_MAX_PACKET_LENGTH)
#endif

#if SBC_SET_RESERVED_ON_ESCO_FAIL
#define SBC_RESERVED_STATUS_PASS 0
#define SBC_RESERVED_STATUS_FAIL 1
#endif

#define SBC_LOUDNESS	0
#define SBC_SNR	1

#define SBC_SUB_BANDS_8	8
#define SBC_SUB_BANDS_4	4

#define SBC_sf16000	0
#define SBC_sf32000	1
#define SBC_sf44100	2
#define SBC_sf48000	3

#define SBC_MONO	0
#define SBC_DUAL	1
#define SBC_STEREO	2
#define SBC_JOINT_STEREO	3

#define SBC_BLOCK_0	4
#define SBC_BLOCK_1	8
#define SBC_BLOCK_2	12
#define SBC_BLOCK_3	16

/* sbc_status bit field */
#define SBC_STATUS_RESET_REQ	0x0001
#define SBC_STATUS_WAIT_FOR_FIRST_EV3	0x0002		// To indicate we are waiting for the first EV3 packet of a new SBC packet
#define SBC_STATUS_EV3_LOST				0x0004		// To indicate that at least 1 EV3 packet has been lost 

/* sbc_option bit field */
#define SBC_OPTION_PLC_MODE_MASK	(0x07)			// SBC PLC MODE
#define SBC_OPTION_DYNAMIC_UUI	(1<<3)			// For SBC WB: Enable Dynamic UUI for variable bit-rate
#define SBC_OPTION_ESCO_CRC_ERR_AS_LOST 			(1<<4)	// For PLC: Process eSCO CRC error frame as Lost frame. Default case is process eSCO CRC Error as Good SBC frame 
#define SBC_OPTION_DISABLE_BEC0 					(1<<5)	// For BEC0: Disable BEC0. Default setting is enabled
#define SBC_OPTION_DISABLE_BEC1_2 				(1<<6)	// For BEC1 and BEC2: Disable BEC1 and BEC2. Default setting is enabled
#define SBC_OPTION_DISABLE_BEC2_BURST_PROTECT 	(1<<7)	// For BEC2: Disable Burst Error Protection if short on MHz. Default is enabled
#define SBC_OPTION_DISABLE_SYNC_OVERWRITE 		(1<<8)	// For WideBand: Overwrite SBC Sync byte, since the SBC frame synchronizatio is done by H2 Header. Default is enabled
#define SBC_OPTION_DISABLE_RESERVED_OVERWRITE  	(1<<9)	// For WideBand: Overwrite Reserved 16 bits, since thsi field is not supposed to be used. Default is enabled
#define SBC_OPTION_ENABLE_EV3				 		(1<<10)	// For WideBand: 1: Enabled EV3 Packet Type. 0: Default Packet Type (2EV3)

/* frame_status */
#define SBC_FRAME_STATUS_MODE_MASK	(0x07)			// SBC Frame Satus
#define SBC_ON_GOING_FRAME			(1<<3)			// On Going Frame bit


/* For SBC_OPTION_PLC_MODE */
enum
{
	SBC_PLC_DISABLED,		
	SBC_PLC_ENABLED,		
	SBC_PLC_TBD2,		
	SBC_PLC_TBD3,		
	SBC_PLC_TBD4,		
	SBC_PLC_TBD5,		
	SBC_PLC_TBD6,		
	SBC_PLC_TBD7		
};

/* For SBC PLC WB frame_status */
enum
{
	SBC_GOOD_ESCO_CRC,		
	SBC_BAD_ESCO_CRC,		
	SBC_LOST		
};


/* Memory allocation  */
#if (SBC_PLC)
//#include "typedef.h"
//#include "lcplc.h"
#endif

#if (SBC_VECTOR_VX_ON_16BITS==TRUE)
#define SBC_STATIC_MEM_SIZE (sizeof(SINT32)*(DEC_VX_BUFFER_SIZE/2 + (SBC_BUF)))	// in Bytes
#else
#define SBC_STATIC_MEM_SIZE (sizeof(SINT32)*(DEC_VX_BUFFER_SIZE+ (SBC_BUF)))	
#endif

#if (SBC_ARM_OPT==TRUE)
#define SBC_SCRATCH_MEM_SIZE (4*(240+128+PCM_BUF+8+8))	// in Bytes
#else
#define SBC_SCRATCH_MEM_SIZE (4*(240+256+PCM_BUF+8+8))
#endif

#if ((SBC_PLC == TRUE) && (LC_PLC_SCRATCH_MEM_SIZE > SBC_SCRATCH_MEM_SIZE) )
#error PLC Scratch Memory will exceed SBC Scratch Memory !! 
#endif



#include "sbc_types.h"

typedef struct SBC_DEC_PARAMS_TAG
{
    SINT16 samplingFreq;			/*16k, 32k, 44.1k or 48k*/
    SINT16 channelMode;			/*mono, dual, streo or joint streo*/
    SINT16 numOfSubBands;		/*4 or 8*/
    SINT16 numOfChannels;
    SINT16 numOfBlocks;			/*4, 8, 12 or 16*/
    SINT16 allocationMethod;		/*loudness or SNR*/
    SINT16 bitPool;				/* 16*numOfSb for mono & dual; 32*numOfSb for stereo & joint stereo */
    SINT16 *scaleFactor;
    SINT16 ScratchMemForBitAlloc[SBC_MAX_NUM_OF_CHANNELS*SBC_MAX_NUM_OF_SUBBANDS];
    SINT16 *bits;
    uint8_t  *packet;

    SINT32 *s32ScratchMem;
    SINT32 *s32StaticMem;
    SINT16 *pcmBuffer;

#if (SBC_ARM_OPT==TRUE)
    SINT16 *sbBuffer;
#else
    SINT32 *sbBuffer;
#endif
#if (SBC_ARM_OPT==FALSE && SBC_IPAQ_OPT==FALSE && SBC_OPTIMIZATION==FALSE) 
    SINT32 *scartchMemForFilter;
#endif
#if (SBC_VECTOR_VX_ON_16BITS==TRUE)
    SINT16 *VX;
#else
    SINT32 *VX;
#endif
    SINT32 ShiftCounter[2];
    uint8_t join[SBC_MAX_NUM_OF_SUBBANDS];       /*0 if not joint stereo*/
    uint16_t u16PrevPacLen;
    uint16_t u16ReaminingBytes;
    uint16_t sbc_status;				/* For internal state machine */
    uint16_t sbc_option;					
#if (SBC_WB)	
    uint16_t sbc_mode;				/*0: A2DP mode, 1: WideBand mode */
    uint16_t uui_id;					/*UUI id  */
#endif
//#if SBC_DEC_MONO_A2DP_INCLUDED /* filter memory */
	SINT16 ds_filter[MONO_A2DP_FILTER_LEN];
	uint8_t ds_idx;				/* Last downsampling index*/	
//#endif
#if (SBC_PLC)
    struct LCPLC_State * plc_state;	/* PLC struct */
    uint16_t frame_status;			/* Frame status: GOOD_ESCO_CRC, BAD_ESCO_CRC, LOST */
    uint16_t good_pkt_len;			/* SBC Packet Len of good frame */
    uint8_t nb_frame;				/* Number of received frame stored in buffer sbc_retr */	
#if SBC_BEC_1_2					/* Store re-transmitted eSCO frame when CRC fails for BEC-1 BEC-2 alogrithms */
    uint8_t sbc_retr[SBC_MAX_NUM_OF_ESCO_FRAME][SBC_WB_MAX_PACKET_LEN];
#endif
#if SBC_SET_RESERVED_ON_ESCO_FAIL
    uint8_t   reserved[2];
    uint16_t  reserved_status;
#endif
#endif
    uint8_t sbc_retr[SBC_MAX_NUM_OF_ESCO_FRAME][SBC_WB_MAX_PACKET_LEN];
    SINT16 *scartchMemForBitAlloc;

}SBC_DEC_PARAMS;

#if (SBC_WB)	
/* In SBC WideBand, the SBC parameters are defined according to UUI */
typedef struct SBC_DEC_WB_UUI_TAG
{
	SINT16 NumOfSubBands;
	SINT16 NumOfBlock;
	SINT16 Bitpool;
	uint16_t uui_id;
}SBC_DEC_WB_UUI_PARAM;
#endif



/*FUNCTION DECLARATIONS*/
extern const SINT16 DCTcoeff4[];
extern const SINT16 DCTcoeff8[];
void SbcSynthesisFilter(SBC_DEC_PARAMS *);

#ifdef __cplusplus
extern "C"
{
#endif
#ifndef SBC_API
#define SBC_API
#endif

#if (SBC_NEW_API)
SBC_API extern SINT16 SBC_Decoder_decode(SBC_DEC_PARAMS *strDecParams, unsigned char * sbc_in, short * pcm_out, unsigned int len);
SBC_API extern SINT16 SBC_Decoder_decode_Init(SBC_DEC_PARAMS *strDecParams);
SBC_API extern SINT16 SBC_Decoder_ReInit(SBC_DEC_PARAMS *strDecParams, unsigned char * sbc_in, unsigned int len);
#else
SBC_API extern SINT16 SBC_Decoder(SBC_DEC_PARAMS *strDecParams);
SBC_API extern SINT16 SBC_Decoder_Init(SBC_DEC_PARAMS *strDecParams);
#endif

#ifdef __cplusplus
}
#endif


#if (SBC_WB)
signed int  SBC_Decoder_find_uui(uint16_t new_uui);
#endif

#if ( SBC_DEC_MONO_A2DP_INCLUDED ==TRUE)
SINT16 SBC_sample_rate_conv(SBC_DEC_PARAMS *strDecParams);
SINT16 SBC_decimate(SBC_DEC_PARAMS *strDecParams, const SINT16 * h, INT16 h_len, INT16 ds_ratio);
SINT16 SBC_resampling(SBC_DEC_PARAMS *strDecParams, const SINT16 * h, INT16 h_len, INT16 us_ratio, INT16 ds_ratio);

#endif


SINT16 DecUnPacking(SBC_DEC_PARAMS *strDecParams);

#if (SBC_C5402_OPT==TRUE)
    void SBC_Multiply_32_16_Simplified(SINT16 s32In2Temp,SINT32  s32In1Temp ,SINT32 *s32OutLow);
    void SBC_Multiply_64(SINT32 s32In1Temp,SINT32 s32In2Temp,SINT32 *s32OutLow,SINT32 *s32OutHi);
    void SBC_Multiply_32_32(SINT32 s32In2Temp,SINT32 s32In1Temp,SINT32 *s32OutLow);
    void SBC_Multiply_32_32_Simplified(SINT32 s32In2Temp,SINT32 s32In1Temp,SINT32 *s32OutLow);
#endif
#endif
