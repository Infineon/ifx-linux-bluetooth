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
 * This is the implementation file for audio  call-out functions.
 */

#include "wiced_bt_sdp.h"
#include "wiced_bt_a2dp_source.h"
#include "wiced_bt_a2dp_src_int.h"
//#include "wiced_bt_audio.h"

/****************************************************************************\
* Audio definitions
\****************************************************************************/

#if (WICED_BT_A2DP_SOURCE_CO_M12_SUPPORT == TRUE)
#include "wiced_bt_a2d_m12.h"
#endif

#if (WICED_BT_A2DP_SOURCE_CO_M24_SUPPORT == TRUE)
#include "wiced_bt_a2d_m24.h"
#endif

#if (WICED_BT_A2DP_SOURCE_CO_MDU_SUPPORT == TRUE)
#include "wiced_bt_a2d_mdu.h"
#endif

/*******************************************************************************
**
** Function         wiced_bt_a2dp_source_cfg_init
**
** Description      This function is used to initialize and build
**                  codec configuration information from codec capability structure.
**
** Returns          Stream codec and content protection capabilities info.
**
*******************************************************************************/
wiced_bool_t wiced_bt_a2dp_source_cfg_init(wiced_bt_a2dp_codec_info_t *p_codec_info,
        uint8_t *p_built_codec_info,
        uint8_t *p_num_protect, uint8_t *p_protect_info)
{
    uint8_t a2d_status = A2D_FAIL;

    WICED_BTA2DP_SRC_TRACE("%s: codec_id:%d \n", __FUNCTION__, p_codec_info->codec_id);

    switch(p_codec_info->codec_id)
    {
    case WICED_BT_A2DP_CODEC_SBC: /* SBC */
        /* Setup for SBC codec */
        a2d_status = wiced_bt_a2d_bld_sbc_info( AVDT_MEDIA_AUDIO,
                                    (wiced_bt_a2d_sbc_cie_t *) &p_codec_info->cie.sbc,
                                    p_built_codec_info);

        WICED_BTA2DP_SRC_TRACE("%s: status: %d \n", __FUNCTION__, a2d_status);
        WICED_BTA2DP_SRC_TRACE("sbc [0x%02x;0x%02x;0x%02x;0x%02x;0x%02x;0x%02x] \n",
            p_built_codec_info[1], p_built_codec_info[2], p_built_codec_info[3],
            p_built_codec_info[4], p_built_codec_info[5], p_built_codec_info[6]);

        return (A2D_SUCCESS == a2d_status) ? WICED_TRUE : WICED_FALSE;
#if (WICED_BT_A2DP_SOURCE_CO_M12_SUPPORT == TRUE)
    case WICED_BT_A2DP_CODEC_M12: /* MP3 */
        /* Set up for MP3 codec */
        a2d_status = wiced_bt_a2d_bld_m12info(AVDT_MEDIA_AUDIO,
                                 (wiced_bt_a2d_m12_cie_t *)&p_codec_info->cie.m12,
                                 p_built_codec_info);
        WICED_BTA2DP_SRC_TRACE("m12 [0x%02x;0x%02x;0x%02x;0x%02x;0x%02x;0x%02x] \n",
                           p_built_codec_info[1],
                           p_built_codec_info[2],
                           p_built_codec_info[3],
                           p_built_codec_info[4],
                           p_built_codec_info[5],
                           p_built_codec_info[6]);
        return (A2D_SUCCESS == a2d_status) ? WICED_TRUE : WICED_FALSE;
#endif //WICED_BT_A2DP_SOURCE_CO_M12_SUPPORT

#if (WICED_BT_A2DP_SOURCE_CO_M24_SUPPORT == TRUE)
    case WICED_BT_A2DP_CODEC_M24: /* AAC */
        /* Set up for AAC codec */
        a2d_status = wiced_bt_a2d_bld_m24info(AVDT_MEDIA_AUDIO,
                                 (wiced_bt_a2d_m24_cie_t *)&p_codec_info->cie.m24,
                                 p_built_codec_info);
        WICED_BTA2DP_SRC_TRACE("m24 [0x%02x;0x%02x;0x%02x;0x%02x;0x%02x;0x%02x] \n",
                           p_built_codec_info[1],
                           p_built_codec_info[2],
                           p_built_codec_info[3],
                           p_built_codec_info[4],
                           p_built_codec_info[5],
                           p_built_codec_info[6]);
        return (A2D_SUCCESS == a2d_status) ? WICED_TRUE : WICED_FALSE;
#endif //WICED_BT_A2DP_SOURCE_CO_M24_SUPPORT

#if (WICED_BT_A2DP_SOURCE_CO_MDU_SUPPORT == TRUE)
    case WICED_BT_A2DP_CODEC_MDU: /* MPEG-D USAC*/
        /* Set up for MPEG-D USAC codec */
        a2d_status = wiced_bt_a2d_bld_mdu_info(AVDT_MEDIA_AUDIO,
                                 (wiced_bt_a2d_mdu_cie_t *)&p_codec_info->cie.mdu,
                                 p_built_codec_info);
        WICED_BTA2DP_SRC_TRACE("mdu [0x%02x;0x%02x;0x%02x;0x%02x;0x%02x;0x%02x;0x%02x] \n",
                               p_built_codec_info[1],
                               p_built_codec_info[2],
                               p_built_codec_info[3],
                               p_built_codec_info[4],
                               p_built_codec_info[5],
                               p_built_codec_info[6],
                               p_built_codec_info[7]);
        return (A2D_SUCCESS == a2d_status) ? WICED_TRUE : WICED_FALSE;
#endif //WICED_BT_A2DP_SOURCE_CO_MDU_SUPPORT

#if (WICED_BT_A2DP_SOURCE_CO_VENDOR_SPECIFIC_SUPPORT == TRUE)
    case WICED_BT_A2DP_CODEC_VENDOR_SPECIFIC: /* Vendor specific */
        // Not supported
        break;
#endif //WICED_BT_A2DP_SOURCE_CO_VENDOR_SPECIFIC_SUPPORT

    default:
        break;
    }

    return a2d_status;
}
