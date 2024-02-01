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
/******************************************************************************
 * File Name: platform_audio_interface.h
 *
 * Description: This is the header file for platform audio interface
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include <assert.h>
#include <stdbool.h>

#include "wiced_result.h"
#include "wiced_bt_a2dp_defs.h"
#include "alsa/asoundlib.h"

/*******************************************************************************
*                                 MACROS
*******************************************************************************/
typedef enum
{
    PLAYER = 0x1,
    RECORDER = 0x2
}audio_device_type_t;

enum
{
    WICED_BT_ALSA_SUCCESS           =   0,
    WICED_BT_ALSA_INIT_PCM_FAIL     =   1,
    WICED_BT_ALSA_INIT_CAP_FAIL     =   2
};

/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/

void audio_decode_init(void);

wiced_result_t audio_device_init(audio_device_type_t device_type);

wiced_result_t audio_device_deinit(audio_device_type_t device_type);

wiced_result_t a2dp_audio_device_configure(wiced_bt_a2dp_codec_info_t *codec_info);

wiced_result_t a2dp_set_audio_streaming(wiced_bool_t start_audio);

void a2dp_audio_data_cback(uint8_t *p_rx_media, uint32_t media_len);

wiced_result_t hf_audio_process_sco_data(uint16_t sco_channel, uint16_t length, uint8_t* p_sco_data);

uint8_t audio_capture_device_init(void);

uint32_t audio_capture_mic_data(uint8_t *mic_data, snd_pcm_sframes_t _frames);

bool a2dp_audio_decode();
