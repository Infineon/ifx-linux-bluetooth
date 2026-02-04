/*
 * Copyright 2026, Cypress Semiconductor Corporation (an Infineon company)
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
 * This module contains utility functions for dealing with MPEG-D, USAC Audio data frames and codec capabilities.
 */
#include "wiced_bt_types.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_mdu.h"


/******************************************************************************
*
* Function Name: wiced_bt_a2dp_mdu_cfg_for_cap
*
* \brief Determine the preferred MPEG-D, USAC codec configuration for the given codec capabilities.
*
* \details  When we reeive response for getCap then this should be called to prepare packet for
*           setConfig. This will basically find commmon capabilities between what peer has, what we suppport (p_cap)
*           and what we want (p_pref).
*           The final negotiated capability is sent back in p_peer.
*           If we are not able to find a match, status is non zero.
*
*  \param p_peer[in/out]    Codec configuration received from Peer. The final negotiated codec
*                           capability is returned back in this.
*  \param p_cap[in]         Local Codec capability.
*  \param p_pref[in]            Local preferred codec capability.
*
*  \return uint8_t          0 in case of success, A2DP error code in case of error
*
******************************************************************************/
uint8_t wiced_bt_a2dp_mdu_cfg_for_cap(uint8_t *p_peer,
                                      wiced_bt_a2d_mdu_cie_t *p_cap,
                                      wiced_bt_a2d_mdu_cie_t *p_pref,
                                      wiced_bt_a2d_mdu_cie_t *output_cap)
{
    uint8_t                status = 0;
    wiced_bt_a2d_mdu_cie_t peer_cie;

    /* parse peer capabilities */
    if ((status = wiced_bt_a2d_pars_mdu_info(&peer_cie, p_peer, TRUE)) != 0)
    {
        return status;
    }

    /* Object type */
    if (peer_cie.obj_type & p_pref->obj_type)
    {
        peer_cie.obj_type = p_pref->obj_type;
    }
    else
    {
        peer_cie.obj_type &= p_cap->obj_type;

        if (peer_cie.obj_type & A2D_MDU_IE_OBJ_USAC)
        {
            peer_cie.obj_type = A2D_MDU_IE_OBJ_USAC;
        }
        else
        {
            /* Bad peer. must support at least A2D_MDU_IE_OBJ_USAC */
            status = A2D_NS_OBJ_TYPE;
            goto wiced_bt_a2dp_mdu_parse_error;
        }
    }

    /* Sampling frequency */
    if (peer_cie.samp_freq & p_pref->samp_freq)
    {
        peer_cie.samp_freq = p_pref->samp_freq;
    }
    else
    {
        peer_cie.samp_freq &= p_cap->samp_freq;

        if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_7)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_7;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_80)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_80;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_8)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_8;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_9)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_9;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_11)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_11;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_117)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_117;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_12)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_12;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_128)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_128;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_14)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_14;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_16)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_16;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_17)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_17;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_19)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_19;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_22)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_22;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_24)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_24;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_29)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_29;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_32)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_32;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_35)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_35;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_38)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_38;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_44)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_44;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_48)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_48;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_58)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_58;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_64)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_64;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_70)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_70;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_76)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_76;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_88)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_88;
        }
        else if (peer_cie.samp_freq & A2D_MDU_IE_SAMP_FREQ_96)
        {
            peer_cie.samp_freq = A2D_MDU_IE_SAMP_FREQ_96;
        }
        else
        {
            /* Bad peer. must support at least 48 k and 44.1 k */
            status = A2D_NS_SAMP_FREQ;
            goto wiced_bt_a2dp_mdu_parse_error;
        }
    }

    /* Channel mode */
    if (peer_cie.chnl & p_pref->chnl)
    {
        peer_cie.chnl = p_pref->chnl;
    }
    else
    {
        /* Bad sink...sink must support all channel modes */
        status = A2D_NS_CH_MODE;
        goto wiced_bt_a2dp_mdu_parse_error;
    }

    /* Variable Bit Rate */
    peer_cie.vbr = p_pref->vbr;

    /* Bit rate index */
    peer_cie.bitrate = p_pref->bitrate;


wiced_bt_a2dp_mdu_parse_error:
    if (status == 0)
    {
        memcpy(output_cap, &peer_cie, sizeof(wiced_bt_a2d_mdu_cie_t));
    }

    return status;
}

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_m24_cfg_in_cap
*
* \brief Determine if the params for setConfig received from peer are correct and supported.
*
* \details  When we recieve setConfig then this should be called to
*           to check if the packet is correct as well as do we support it.
*
*  \param p_cfg[in]         Configuration received from Peer in setConfig
*  \param p_cap[in]         Local Codec capability.
*
*  \return uint8_t          0 in case of success, A2DP error code in case of error
*
******************************************************************************/
uint8_t wiced_bt_a2dp_mdu_cfg_in_cap(uint8_t *p_cfg, wiced_bt_a2d_mdu_cie_t *p_cap)
{
    uint8_t                status = 0;
    wiced_bt_a2d_mdu_cie_t cfg_cie;

    /* parse configuration */
    if ((status = wiced_bt_a2d_pars_mdu_info(&cfg_cie, p_cfg, FALSE)) != 0)
    {
        return status;
    }

    /* verify that each parameter is in range */

    /* Object Type */
    if ((cfg_cie.obj_type & p_cap->obj_type) == 0)
    {
        status = A2D_NS_OBJ_TYPE;
    }
    /* sample frequency */
    else if ((cfg_cie.samp_freq & p_cap->samp_freq) == 0)
    {
        status = A2D_NS_SAMP_FREQ;
    }
    /* channel mode */
    else if ((cfg_cie.chnl & p_cap->chnl) == 0)
    {
        status = A2D_NS_CHANNEL;
    }
    /* variable bit rate */
    else if ((cfg_cie.vbr == TRUE) && (p_cap->vbr == FALSE))
    {
        status = A2D_NS_VBR;
    }
    return status;
}

wiced_bool_t wiced_bt_a2dp_mdu_are_equal(wiced_bt_a2d_mdu_cie_t *cap_1, wiced_bt_a2d_mdu_cie_t *cap_2)
{
    return (cap_1->bitrate == cap_2->bitrate) && (cap_1->chnl == cap_2->chnl) && (cap_1->obj_type == cap_2->obj_type) &&
           (cap_1->samp_freq == cap_2->samp_freq) && (cap_1->vbr == cap_2->vbr);

}
