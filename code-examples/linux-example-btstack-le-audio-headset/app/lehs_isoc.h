/*
* $ Copyright Cypress Semiconductor $
*/

#ifndef __LEHS_ISOC_H__
#define __LEHS_ISOC_H__

#include "wiced_bt_ga_bap.h"
#include "wiced_bt_isoc.h"

#define MAX_STREAMS_SUPPORTED 2
#define MAX_INPUT_SAMPLE_SIZE_IN_BYTES 480 * 2 //48khz @ 10ms interval

/**
* Stores informations about the configured audio streams along with connection handle
*/
typedef struct
{
    uint16_t conn_hdl;                 // BIS or CIS connection handle
    uint16_t octets_per_frame;         // Octets per codec frame (Number of Octets)
    uint8_t b_stream_active;           // True if stream is active
    uint32_t frame_duration;           // Frame duration in micriseconds
    uint32_t sampling_frequency;       // Sampling Frequency in kHz
    uint16_t psn;
    uint32_t audio_location;
    uint8_t num_of_channels;
} ga_iso_audio_stream_info_t;

/**
 *
 * Function        :lehs_isoc_dhm_init
 *                  Initiates ISOC and audio data handling module for specific platform (host or embedded)
 *
 */
void lehs_isoc_dhm_init(void);

/**
 *
 * Function        :lehs_isoc_dhm_setup_cis_stream
 *                  Sets up the ISO datapath and initializes the codec encoder or decoder
 *
 *
 * @param[in]       p_ase  : ASE to configure stream
 *
 * @return          result
 *
 */
wiced_result_t lehs_isoc_dhm_setup_cis_stream(lehs_ase_data_t *p_ase);

/**
 *
 * Function        :lehs_isoc_dhm_setup_bis_stream
 *                  Sets up the ISO datapath and initializes the codec encoder or decoder
 *
 *
 * @param[in]       conn_hdl  : BIS connection handle
 * @param[in]       p_csc     : codec specific configuration (\ ref wiced_bt_ga_bap_csc_t)
 * @param[in]       bis_count : number of bis stream
 *
 * @return          result
 *
 */
wiced_result_t lehs_isoc_dhm_setup_bis_stream(broadcast_sink_cb_t *p_big, uint8_t bis_count);

/**
 *
 * Function        :lehs_isoc_dhm_start_stream
 *                  Starts audio stream (Sending or receiving)
 *
 */
void lehs_isoc_dhm_start_stream(uint16_t conn_hdl, uint8_t ase_type);

/**
 *
 * Function        :lehs_isoc_dhm_free_cis_stream
 *                  Removes ISOC datapath and releases the resources
 *                  used for stream
 *
 *
 * @param[in]       conn_hdl  : CIS connection handle
 * @param[in]       dir       : direction bit-field for CIS datapath
 *
 * @return          result
 *
 */
void lehs_isoc_dhm_free_cis_stream(uint16_t conn_hdl, wiced_ble_isoc_data_path_bit_t dir);

/**
 *
 * Function        :lehs_isoc_dhm_free_bis_stream
 *                  Removes ISOC datapath and releases the resources
 *                  used for stream
 *
 * @param[in]       conn_hdl  : BIS connection handle
 *
 * @return          result
 *
 */
void lehs_isoc_dhm_free_bis_stream(uint16_t *conn_hdl_list, uint8_t bis_count);

/**
 *
 * Function        :lehs_isoc_audio_stop_stream
 *                  Stops audio stream (Sending or receiving)
 *
 */

void lehs_isoc_audio_stop_stream(uint16_t conn_hdl);

#endif
