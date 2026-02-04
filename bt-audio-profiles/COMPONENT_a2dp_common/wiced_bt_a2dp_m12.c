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
 * This module contains utility functions for dealing with MPEG-1, 2 Audio data frames and codec capabilities.
 */

#include "wiced_bt_types.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_m12.h"

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_m12_cfg_for_cap
*
* \brief Determine the preferred MPEG-1/2 codec configuration for the given codec capabilities.
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
uint8_t wiced_bt_a2dp_m12_cfg_for_cap(uint8_t *p_peer,
                                      wiced_bt_a2d_m12_cie_t *p_cap,
                                      wiced_bt_a2d_m12_cie_t *p_pref,
                                      wiced_bt_a2d_m12_cie_t *output_cap)
{
    uint8_t                status = 0;
    wiced_bt_a2d_m12_cie_t peer_cie;

    /* parse peer capabilities */
    if ((status = wiced_bt_a2d_pars_m12info(&peer_cie, p_peer, TRUE)) != 0)
    {
        return status;
    }

    /* layer */
    if (peer_cie.layer & p_pref->layer)
    {
        peer_cie.layer = p_pref->layer;
    }
    else
    {
        peer_cie.layer &= p_cap->layer;
        if(peer_cie.layer & A2D_M12_IE_LAYER3)
        {
            peer_cie.layer = A2D_M12_IE_LAYER3;
        }
        else if(peer_cie.layer & A2D_M12_IE_LAYER2)
        {
            peer_cie.layer = A2D_M12_IE_LAYER2;
        }
        else if(peer_cie.layer & A2D_M12_IE_LAYER1)
        {
            peer_cie.layer = A2D_M12_IE_LAYER1;
        }
        else
        {
            status = A2D_NS_LAYER;
            goto wiced_bt_a2dp_m12_parse_error;
        }
    }

    /* channel mode */
    if (peer_cie.ch_mode & p_pref->ch_mode)
    {
        peer_cie.ch_mode = p_pref->ch_mode;
    }
    else
    {
        status = A2D_NS_CH_MODE;
        goto wiced_bt_a2dp_m12_parse_error;
    }

    /* crc */
    peer_cie.crc = p_pref->crc;

    /* payload format */
    peer_cie.mpf = p_pref->mpf;

    /* sample frequency */
    if (peer_cie.samp_freq & p_pref->samp_freq)
    {
        peer_cie.samp_freq = p_pref->samp_freq;
    }
    else
    {
        peer_cie.samp_freq &= p_cap->samp_freq;
        if (peer_cie.samp_freq & A2D_M12_IE_SAMP_FREQ_44)
        {
            peer_cie.samp_freq = A2D_M12_IE_SAMP_FREQ_44;
        }
        else if (peer_cie.samp_freq & A2D_M12_IE_SAMP_FREQ_48)
        {
            peer_cie.samp_freq = A2D_M12_IE_SAMP_FREQ_48;
        }
        else if (peer_cie.samp_freq & A2D_M12_IE_SAMP_FREQ_32)
        {
            peer_cie.samp_freq = A2D_M12_IE_SAMP_FREQ_32;
        }
        else if (peer_cie.samp_freq & A2D_M12_IE_SAMP_FREQ_24)
        {
            peer_cie.samp_freq = A2D_M12_IE_SAMP_FREQ_24;
        }
        else if (peer_cie.samp_freq & A2D_M12_IE_SAMP_FREQ_22)
        {
            peer_cie.samp_freq = A2D_M12_IE_SAMP_FREQ_22;
        }
        else if (peer_cie.samp_freq & A2D_M12_IE_SAMP_FREQ_16)
        {
            peer_cie.samp_freq = A2D_M12_IE_SAMP_FREQ_16;
        }
        else
        {
            status = A2D_NS_SAMP_FREQ;
            goto wiced_bt_a2dp_m12_parse_error;
        }
    }

    /* variable bit rate */
    peer_cie.vbr = p_pref->vbr;

    /* bit rate index */
    if (peer_cie.bitrate & p_pref->bitrate)
    {
        peer_cie.bitrate = p_pref->bitrate;
    }
    else
    {
        peer_cie.bitrate &= p_cap->bitrate;
        if (peer_cie.bitrate & A2D_M12_IE_BITRATE_9)
        {
            peer_cie.bitrate = A2D_M12_IE_BITRATE_9;
        }
        else if (peer_cie.bitrate & A2D_M12_IE_BITRATE_8)
        {
            peer_cie.bitrate = A2D_M12_IE_BITRATE_8;
        }
        else if (peer_cie.bitrate & A2D_M12_IE_BITRATE_10)
        {
            peer_cie.bitrate = A2D_M12_IE_BITRATE_10;
        }
        else if (peer_cie.bitrate & A2D_M12_IE_BITRATE_11)
        {
            peer_cie.bitrate = A2D_M12_IE_BITRATE_11;
        }
        else if (peer_cie.bitrate & A2D_M12_IE_BITRATE_12)
        {
            peer_cie.bitrate = A2D_M12_IE_BITRATE_12;
        }
        else if (peer_cie.bitrate & A2D_M12_IE_BITRATE_7)
        {
            peer_cie.bitrate = A2D_M12_IE_BITRATE_7;
        }
        else if (peer_cie.bitrate & A2D_M12_IE_BITRATE_5)
        {
            peer_cie.bitrate = A2D_M12_IE_BITRATE_5;
        }
        else if (peer_cie.bitrate & A2D_M12_IE_BITRATE_0)
        {
            peer_cie.bitrate = A2D_M12_IE_BITRATE_0;
        }
        else
        {
            status = A2D_NS_BIT_RATE;
            goto wiced_bt_a2dp_m12_parse_error;
        }
    }

wiced_bt_a2dp_m12_parse_error:
    if (status == 0)
    {
        memcpy(output_cap, &peer_cie, sizeof(wiced_bt_a2d_m12_cie_t));
    }
    return status;
}

/******************************************************************************
*
* Function Name: wiced_bt_a2dp_m12_cfg_in_cap
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
uint8_t wiced_bt_a2dp_m12_cfg_in_cap(uint8_t *p_cfg, wiced_bt_a2d_m12_cie_t *p_cap)
{
    uint8_t                status = 0;
    wiced_bt_a2d_m12_cie_t cfg_cie;

    /* parse configuration */
    if ((status = wiced_bt_a2d_pars_m12info(&cfg_cie, p_cfg, FALSE)) != 0)
    {
        return status;
    }

    /* verify that each parameter is in range */

    /* layer */
    if ((cfg_cie.layer & p_cap->layer) == 0)
    {
        status = A2D_NS_LAYER;
    }
    /* crc */
    else if ((cfg_cie.crc == TRUE) && (p_cap->crc == FALSE))
    {
        status = A2D_NS_CRC;
    }
    /* payload format */
    else if ((cfg_cie.mpf == 1) && (p_cap->mpf == 0))
    {
        status = A2D_NS_MPF;
    }
    /* sample frequency */
    else if ((cfg_cie.samp_freq & p_cap->samp_freq) == 0)
    {
        status = A2D_NS_SAMP_FREQ;
    }
    /* variable bit rate */
    else if ((cfg_cie.vbr == TRUE) && (p_cap->vbr == FALSE))
    {
        status = A2D_NS_VBR;
    }
    /* bit rate index */
    else if ((cfg_cie.bitrate & p_cap->bitrate) == 0)
    {
        status = A2D_NS_BIT_RATE;
    }

    return status;
}


wiced_bool_t wiced_bt_a2dp_m12_are_equal(wiced_bt_a2d_m12_cie_t *cap_1, wiced_bt_a2d_m12_cie_t *cap_2)
{
    return (cap_1->bitrate == cap_2->bitrate) && (cap_1->vbr == cap_2->vbr) && (cap_1->samp_freq == cap_2->samp_freq) &&
           (cap_1->ch_mode == cap_2->ch_mode) && (cap_1->mpf == cap_2->mpf) && (cap_1->crc == cap_2->crc) &&
           (cap_1->layer == cap_2->layer);

}
