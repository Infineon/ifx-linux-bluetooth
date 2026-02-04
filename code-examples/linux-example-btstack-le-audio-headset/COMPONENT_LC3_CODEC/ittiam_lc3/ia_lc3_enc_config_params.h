/*****************************************************************************/
/*                                                                           */
/*                        LC3 Plus Encoder                                   */
/*                                                                           */
/*                   ITTIAM SYSTEMS PVT LTD, BANGALORE                       */
/*                          COPYRIGHT(C) 2020                                */
/*                                                                           */
/*  This program is proprietary to Ittiam Systems Pvt. Ltd. and is protected */
/*  under Indian Copyright Act as an unpublished work.Its use and disclosure */
/*  is  limited by  the terms and conditions of a license  agreement. It may */
/*  be copied or  otherwise reproduced or  disclosed  to persons outside the */
/*  licensee 's  organization  except  in  accordance  with  the  terms  and */
/*  conditions of  such an agreement. All  copies and reproductions shall be */
/*  the  property  of Ittiam Systems Pvt.  Ltd. and  must  bear  this notice */
/*  in its entirety.                                                         */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  File Name        : IA_LC3_ENC_config_params.h                        */
/*                                                                           */
/*  Description      : Configuration parameters list                         */
/*                                                                           */
/*  List of Functions: None                                                  */
/*                                                                           */
/*  Issues / Problems: None                                                  */
/*                                                                           */
/*  Revision History :                                                       */
/*                                                                           */
/*        DD MM YYYY       Author                Changes                     */
/*        29 07 2005       Ittiam                Created                     */
/*                                                                           */
/*****************************************************************************/

#ifndef __IA_LC3_ENC_CONFIG_PARAMS_H__
#define __IA_LC3_ENC_CONFIG_PARAMS_H__

/* LC3 plus_enc */

#define IA_LC3_ENC_CONFIG_PARAM_PCM_WDSZ				0x0000
#define IA_LC3_ENC_CONFIG_PARAM_SAMP_FREQ				0x0001
#define IA_LC3_ENC_CONFIG_PARAM_NUM_CHANNELS			0x0002
#define IA_LC3_ENC_CONFIG_PARAM_DELAY_COMP				0x0003
#define IA_LC3_ENC_CONFIG_PARAM_G192_ENABLE				0x0004
#define IA_LC3_ENC_CONFIG_PARAM_BITRATE					0x0005
#define IA_LC3_ENC_CONFIG_PARAM_FRAME_MS				0x0006
#define IA_LC3_ENC_CONFIG_PARAM_EPMODE					0x0007

#define IA_LC3_ENC_CONFIG_PARAM_BANDWIDTH				0x0009
#define IA_LC3_ENC_CONFIG_PARAM_EPF						0x000A
#define IA_LC3_ENC_CONFIG_PARAM_EDF						0x000B
#define IA_LC3_ENC_CONFIG_PARAM_PCM						0x000C
#define IA_LC3_ENC_CONFIG_PARAM_INPUT_BUF_SIZE			0x000D

#define IA_LC3_ENC_CONFIG_PARAM_G192                    0x001D
#define IA_LC3_ENC_CONFIG_PARAM_DELAY                   0x001E
#define IA_LC3_ENC_CONFIG_PARAM_NSAMPLES                0x001F

#ifdef CONFORMANCE_TESTS
#define IA_LC3_ENC_CONFIG_PARAM_EPT						0x0008
#endif
//#define IA_LC3_ENC_CONFIG_PARAM_NUM_CHANNELS			0x0002
//#define IA_LC3_ENC_CONFIG_PARAM_CHANNEL_MASK			0x0003
//#define IA_LC3_ENC_CONFIG_PARAM_CHANNEL_MODE			0x0004




#endif /* __IA_LC3_ENC_CONFIG_PARAMS_H__ */
