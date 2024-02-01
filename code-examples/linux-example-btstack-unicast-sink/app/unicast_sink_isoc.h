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

#pragma once

#include "wiced_bt_ga_bap.h"
#include "wiced_bt_isoc.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define MAX_STREAMS_SUPPORTED 2
#define MAX_INPUT_SAMPLE_SIZE_IN_BYTES 480 * 2 //48khz @ 10ms interval

/*******************************************************************************
*                               STRUCTURES AND ENUMERATIONS
*******************************************************************************/
typedef struct
{
    wiced_bool_t is_cis;
    uint16_t conn_hdl;
    uint16_t octets_per_frame;
    uint8_t num_of_channels;
    wiced_bool_t b_stream_active;
    uint32_t frame_duration;
    uint32_t sampling_frequency;
} ga_iso_audio_stream_info_t;

/****************************************************************************
 *                              FUNCTION DECLARATION
 ***************************************************************************/
void init_stream_info(uint16_t conn_hdl,
                      wiced_bool_t is_cis,
                      uint32_t sampling_frequency,
                      uint16_t octets_per_frame,
                      uint32_t frame_duration,
                      uint8_t num_of_channels);

ga_iso_audio_stream_info_t *get_stream_info(uint16_t conn_hdl);

void unicast_sink_isoc_dhm_init(const wiced_bt_cfg_isoc_t *p_iso_cfg);

void unicast_sink_isoc_dhm_setup_stream(uint16_t conn_hdl,
                                        wiced_bool_t is_cis,
                                        uint16_t direction,
                                        wiced_bt_ga_bap_csc_t *p_csc);

void unicast_sink_isoc_dhm_start_stream(void);

void unicast_sink_isoc_dhm_free_stream(uint16_t conn_hdl,
                                       wiced_bt_isoc_data_path_bit_t dir,
                                       uint8_t *idx_list,
                                       int num_channels);

wiced_result_t iso_audio_setup_data_path(uint16_t conn_id, uint16_t direction, wiced_bt_ga_bap_csc_t *p_csc);

void iso_audio_remove_data_path(uint16_t conn_hdl, wiced_bt_isoc_data_path_bit_t dir, uint8_t *idx_list);

void iso_audio_start_stream(uint16_t conn_id);

void iso_audio_stop_stream(uint16_t conn_id);
