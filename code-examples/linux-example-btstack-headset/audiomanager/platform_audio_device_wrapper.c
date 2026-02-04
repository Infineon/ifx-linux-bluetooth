/*
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "wiced_bt_trace.h"

#include "platform_audio_device.h"
#include "platform_audio_interface.h"

wiced_result_t  platform_audio_device_init ( const platform_audio_device_id_t device_id )
{
	WICED_BT_TRACE("[%s] device_id: %d\n",__func__, device_id);
	if (device_id == PLATFORM_DEVICE_PLAY)
	{
		audio_device_init(PLAYER);
	}
	else if (device_id == PLATFORM_DEVICE_PLAY_RECORD)
	{
		audio_device_init(RECORDER);
	}
	else{
		WICED_BT_TRACE("[%s] device_id: %d Wrapper !!\n",__func__, device_id);
	}
	return WICED_SUCCESS;
}
wiced_result_t platform_audio_device_configure ( const platform_audio_device_id_t device_id, platform_audio_config_t* config )
{
	return WICED_SUCCESS;
}
wiced_result_t platform_audio_device_set_output_device ( const platform_audio_device_id_t device_id, platform_audio_io_device_t sink )
{
	return WICED_SUCCESS;
}

wiced_result_t platform_audio_device_set_sr ( const platform_audio_device_id_t device_id, int32_t sr )
{
	return WICED_SUCCESS;
}

wiced_result_t platform_audio_device_set_volume ( const platform_audio_device_id_t device_id, int32_t volume_in_db )
{
    return WICED_SUCCESS;
}

wiced_result_t platform_audio_device_set_mic_gain ( const platform_audio_device_id_t device_id, int32_t volume_in_db )
{
	return WICED_SUCCESS;
}

wiced_result_t platform_audio_device_get_volume ( const platform_audio_device_id_t device_id, int32_t *volume_in_db )
{
	return WICED_SUCCESS;
}

wiced_result_t platform_audio_device_get_volume_range ( const platform_audio_device_id_t device_id, int32_t *min_volume_in_db, int32_t *max_volume_in_db )
{
	return WICED_SUCCESS;
}

wiced_result_t platform_audio_device_deinit ( const platform_audio_device_id_t device_id )
{
	WICED_BT_TRACE("[%s] device_id: %d\n",__func__, device_id);
	if (device_id == PLATFORM_DEVICE_PLAY)
	{
		audio_device_deinit(PLAYER);
	}
	else if (device_id == PLATFORM_DEVICE_PLAY_RECORD)
	{
		audio_device_deinit(RECORDER);
	}
	else{
		WICED_BT_TRACE("[%s] device_id: %d Wrapper !!\n",__func__, device_id);
	}
	return WICED_SUCCESS;
}
wiced_result_t platform_audio_device_start ( const platform_audio_device_id_t device_id )
{
	return WICED_SUCCESS;
}

wiced_result_t platform_audio_device_stop ( const platform_audio_device_id_t device_id )
{
	return WICED_SUCCESS;
}

wiced_result_t platform_audio_device_ioctl ( const platform_audio_device_id_t device_id, platform_audio_device_ioctl_t cmd, platform_audio_device_ioctl_data_t* cmd_data )
{
	return WICED_SUCCESS;
}