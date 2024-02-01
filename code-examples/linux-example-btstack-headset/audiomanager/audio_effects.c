/*
* Copyright 2019-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*/

/** @file
 * Audio effects API is a high level interface for applications to use platform audio effects capabilities.
 */

#include <wiced_audio_manager.h>
#include <wiced_bt_trace.h>
#include <platform_audio_device.h>
#include "audio_effects.h"

int32_t *audio_effect_type_list;
platform_audio_effect_descrip_t *audio_effect_desc_list;
uint32_t total_no_effects = 0;

int32_t audio_effect_init(uint32_t effect_type, void *args, uint32_t *effect_id)
{
#ifdef AUDIO_EFFECTS_ENABLE
    if(WICED_SUCCESS != platform_audio_effect_init((platform_audio_effect_type_t)effect_type, (platform_audio_effect_config_t *)args, effect_id))
        return -1;
#else
    WICED_BT_TRACE("Audio effects not supported with this codec \n");
    /* returning with no error as effects are not supported for this codec.*/
#endif
    return 0;
}

int32_t audio_effect_deinit(int32_t effect_id)
{
#ifdef AUDIO_EFFECTS_ENABLE
    if(WICED_SUCCESS != platform_audio_effect_deinit((uint32_t)effect_id))
        return -1;
#else
    WICED_BT_TRACE("Audio effects not supported with this codec \n");
    /* returning with no error as effects are not supported for this codec.*/
#endif
    return 0;

}

int32_t audio_effect_process(int32_t effect_id, uint32_t in_size, uint8_t *in_buf,uint32_t *out_size,uint8_t *out_buf,uint32_t ref_size,uint8_t *ref_buf )
{
#ifdef AUDIO_EFFECTS_ENABLE
    if(WICED_SUCCESS != platform_audio_effect_process((uint32_t) effect_id,
                                                in_size,in_buf,
                                                ref_size,ref_buf,
                                                out_size,out_buf))
    {
        return -1;
    }
#else
    WICED_BT_TRACE("Audio effects not supported with this codec \n");
    /* returning with no error as effects are not supported for this codec.*/
#endif
    return 0;
}

int32_t* audio_effect_get_effects( int32_t *num_of_effects)
{

#ifdef AUDIO_EFFECTS_ENABLE
    audio_effect_type_list = platform_audio_effect_get_effects_type(&total_no_effects);

    if(audio_effect_type_list != NULL )
    {
        *num_of_effects = total_no_effects;
        return audio_effect_type_list;
    }
#else
    WICED_BT_TRACE("Audio effects not supported with this codec \n");
    /* returning NULL pointer as effects are not supported for this codec.*/
#endif
    return NULL;
}

audio_effect_desc_t* audio_effect_get_effects_descriptor(int32_t effect_type)
{
#ifdef AUDIO_EFFECTS_ENABLE
    return platform_audio_effect_get_effects_descriptor((platform_audio_effect_type_t)effect_type);
#else
    WICED_BT_TRACE("Audio effects not supported with this codec \n");
    /* returning NULL pointer as effects are not supported for this codec.*/
#endif
    return NULL;
}

int32_t audio_effect_ioctl(int32_t effect_id, audio_effect_ioctl_t cmd, audio_effect_ioctl_data *cmd_data)
{
#ifdef AUDIO_EFFECTS_ENABLE
    if(WICED_SUCCESS != platform_audio_effect_ioctl(effect_id, cmd, cmd_data))
        return -1;
#else
    WICED_BT_TRACE("Audio effects not supported with this codec \n");
    /* returning with no error as effects are not supported for this codec.*/
#endif
    return 0;
}
