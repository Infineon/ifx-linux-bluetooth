/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company)
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
#include "wiced_bt_a2dp_sink_int.h"

#if (WICED_BT_A2DP_SINK_CO_M12_SUPPORT == TRUE)
/*******************************************************************************
**
** Function         wiced_bt_a2dp_m12_cfg_for_cap
**
** Description      Determine the preferred MPEG-1, 2 Audio codec configuration for the
**                  given codec capabilities.  The function is passed the
**                  preferred codec configuration and the peer codec
**                  capabilities for the stream.  The function attempts to
**                  match the preferred capabilities with the configuration
**                  as best it can.  The resulting codec configuration is
**                  returned in the same memory used for the capabilities.
**
** Returns          0 if ok, nonzero if error.
**                  Codec configuration in p_cap.
**
*******************************************************************************/
uint8_t wiced_bt_a2dp_m12_cfg_for_cap(uint8_t *p_peer,
    wiced_bt_a2d_m12_cie_t *p_cap, wiced_bt_a2d_m12_cie_t *p_pref)
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
        }
    }

    if (status == 0)
    {
        /* build configuration */
        wiced_bt_a2d_bld_m12info(A2D_MEDIA_TYPE_AUDIO, &peer_cie, p_peer);
    }
    return status;
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_m12_cfg_in_cap
**
** Description      This function checks whether an MPEG-1, 2 Audio codec configuration
**                  is allowable for the given codec capabilities.
**
** Returns          0 if ok, nonzero if error.
**
*******************************************************************************/
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
#endif // WICED_BT_A2DP_SINK_CO_M12_SUPPORT
