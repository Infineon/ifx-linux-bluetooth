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
 * This module contains utility functions for dealing with SBC data frames and codec capabilities.
 */

#include "wiced_bt_types.h"
#include "wiced_result.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"
#include "wiced_bt_a2dp_src_int.h"

/*******************************************************************************
**
** Function         wiced_bt_a2dp_src_sbc_format_check
**
** Description      Determine the preferred SBC codec configuration for the
**                  given codec capabilities.  The function is passed the
**                  preferred codec configuration and the peer codec
**                  capabilities for the stream.  The function attempts to
**                  match the preferred capabilities with the configuration
**                  as best it can.  The resulting codec configuration is
**                  returned in the same memory used for the capabilities.
**
** Returns          TRUE if ok, FALSE if error.
**                  Codec configuration in p_cap.
**
*******************************************************************************/
wiced_bool_t wiced_bt_a2dp_src_sbc_format_check( uint8_t *peer_codec_info, wiced_bt_a2d_sbc_cie_t supported_cap, wiced_bt_a2d_sbc_cie_t *default_cap, wiced_bt_a2d_sbc_cie_t *output_cap )
{
    wiced_bt_a2d_sbc_cie_t codec_info;
    wiced_bt_a2d_sbc_cie_t temp_codec;
    wiced_bool_t           ret_val = WICED_FALSE;
    wiced_bt_a2d_status_t            a2d_status;

    /*
     * By specification all sinks must support the full set of SBC codec parameters.
     * Still it is a good idea to check the capabilities of the endpoint just to be sure.
     */
    WICED_BTA2DP_SRC_TRACE("[%s] \n",__FUNCTION__);
    if (default_cap == NULL)
    {
        /* copy the supported SBC codec parameters */
        memcpy( &temp_codec, &supported_cap, sizeof(wiced_bt_a2d_sbc_cie_t) );
    }
    else
    {
        /* copy the default SBC codec parameters */
        memcpy(&temp_codec, default_cap, sizeof(wiced_bt_a2d_sbc_cie_t));
    }

    a2d_status = wiced_bt_a2d_pars_sbc_info( &codec_info, peer_codec_info, WICED_TRUE );

    WICED_BTA2DP_SRC_TRACE( "[%s]: wiced_bt_a2d_pars_sbc_info status = %d codec type: %d \n", __FUNCTION__,
                    a2d_status, peer_codec_info[2]);

    /* make sure the endpoint is at least SBC */
    if ( ( a2d_status == A2D_SUCCESS ) && ( peer_codec_info[2] == A2D_MEDIA_CT_SBC ) )
    {
        WICED_BTA2DP_SRC_TRACE( "[%s]: samp_freq: 0x%x vs 0x%x\r\n", __FUNCTION__, temp_codec.samp_freq, codec_info.samp_freq);
        ret_val = ( temp_codec.samp_freq & codec_info.samp_freq ) ? WICED_TRUE : WICED_FALSE;
    }

    if ( ret_val )
    {
        WICED_BT_TRACE( "[%s]: ch_mode: 0x%x vs 0x%x\r\n", __FUNCTION__, temp_codec.ch_mode, codec_info.ch_mode);
        ret_val = ( temp_codec.ch_mode & codec_info.ch_mode ) ? WICED_TRUE : WICED_FALSE;
    }

    if ( ret_val )
    {
        /* make sure there is an overlap in the bitpool values */
        if ( codec_info.min_bitpool > temp_codec.min_bitpool )
            temp_codec.min_bitpool = codec_info.min_bitpool;

        if ( codec_info.max_bitpool < temp_codec.max_bitpool )
            temp_codec.max_bitpool = codec_info.max_bitpool;

        /* Sanity check */
        if ( temp_codec.min_bitpool > temp_codec.max_bitpool )
        {
            ret_val = WICED_FALSE;
        }
        else
        {
            memcpy(output_cap,&temp_codec,sizeof(wiced_bt_a2d_sbc_cie_t));
        }
    }

    WICED_BTA2DP_SRC_TRACE( "[%s]: Exit Format Match: %s\n", __FUNCTION__, ret_val ? "TRUE" : "FALSE");
    return ret_val;
}
