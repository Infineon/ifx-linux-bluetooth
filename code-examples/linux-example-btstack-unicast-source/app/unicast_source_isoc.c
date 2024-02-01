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

#include "iso_data_handler.h"
#include "lc3_codec.h"
#include "unicast_source_gatt.h"
#include "unicast_source_iso.h"
#include "unicast_source_ascs.h"
#include "wiced_bt_cfg.h"

#include "audio_driver.h"
#include "wiced_bt_ga_ascs.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "audio_parse_wave.h"

#include "log.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define MAX_INPUT_SAMPLE_SIZE_IN_BYTES 480 * 2 //48khz @ 10ms interval
#define MAX_STREAMS_SUPPORTED 2
#define FIRST_ISOC_DATA 4
#define LINUX_ALSA_LATENCY              500U

wiced_bt_pool_t *ga_iso_audio_pool = NULL;
uint8_t *p_buf = NULL;

#ifdef debug_googlelc3
FILE *fp_out = NULL;
extern void lc3bin_write_header(FILE *fp,
    int frame_us, int srate_hz, int bitrate, int nchannels, int nsamples);
extern void lc3bin_write_data(FILE *fp,
    const void *data, int nchannels, int frame_bytes);
extern int lc3_frame_bytes(int, int);
#endif

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

static ga_iso_audio_stream_info_t g_stream_info[MAX_STREAMS_SUPPORTED] = {0};

/******************************************************************************
 * Function Name: init_stream_info
 *
 * Summary: init stream info when iso_audio_setup_data_path API
 *
 * Parameters:
 *  uint16_t        conn_hdl,
 *  wiced_bool_t    is_cis,
 *  uint32_t        sampling_frequency,
 *  uint16_t        octets_per_frame,
 *  uint32_t        frame_duration,
 *  uint8_t         num_of_channels
 *
 * Return:
 *  None
 *
******************************************************************************/
static void init_stream_info(uint16_t conn_hdl,
                             wiced_bool_t is_cis,
                             uint32_t sampling_frequency,
                             uint16_t octets_per_frame,
                             uint32_t frame_duration,
                             uint8_t num_of_channels)
{
    WICED_BT_TRACE("[%s] [is_cis %d] [SF %d] [OPF %d] [FD %d] [num_of_channels %d]\n",
                   __FUNCTION__,
                   is_cis,
                   sampling_frequency,
                   octets_per_frame,
                   frame_duration,
                   num_of_channels);

    for (size_t i = 0; i < MAX_STREAMS_SUPPORTED; i++) {
        if (!g_stream_info[i].conn_hdl) {
            g_stream_info[i].conn_hdl = conn_hdl;
            g_stream_info[i].is_cis = is_cis;
            g_stream_info[i].octets_per_frame = octets_per_frame;
            g_stream_info[i].num_of_channels = num_of_channels;
            g_stream_info[i].sampling_frequency = sampling_frequency;
            g_stream_info[i].frame_duration = frame_duration;
            g_stream_info[i].b_stream_active = 1;

            WICED_BT_TRACE("[%s] conn_hdl 0x%x \n", __FUNCTION__, conn_hdl);
            return;
        }
    }
}

/******************************************************************************
 * Function Name: get_stream_info
 *
 * Summary: get the stream info by cis connection handle
 *
 * Parameters:
 *  uint16_t conn_hdl: cis connection handle
 *
 * Return:
 *  ga_iso_audio_stream_info_t*
 *
******************************************************************************/
static ga_iso_audio_stream_info_t *get_stream_info(uint16_t conn_hdl)
{
    for (size_t i = 0; i < MAX_STREAMS_SUPPORTED; i++) {
        if (conn_hdl == g_stream_info[i].conn_hdl) {
            return &g_stream_info[i];
        }
    }

    return NULL;
}

/******************************************************************************
 * Function Name: deinit_stream_info
 *
 * Summary: deinit the stream info by cis connection handle
 *
 * Parameters:
 *  uint16_t conn_hdl
 *
 * Return:
 *  None
******************************************************************************/
static void deinit_stream_info(uint16_t conn_hdl)
{
    WICED_BT_TRACE("[%s] [conn_hdl %d]\n", __FUNCTION__, conn_hdl);

    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);
    if (!p_stream_info) return;

    memset(p_stream_info, 0, sizeof(ga_iso_audio_stream_info_t));
    WICED_BT_TRACE("[%s] deinit successful\n", __FUNCTION__);
}

/******************************************************************************
 * Function Name: count_set_bits
 *
 * Summary: utils to count the set bits
 *
 * Parameters:
 *  uint32_t n
 *
 * Return:
 *  uint32_t
 *
******************************************************************************/
static uint32_t count_set_bits(uint32_t n)
{
    uint32_t count = 0;
    while (n) {
        n &= (n - 1);
        count++;
    }
    return count;
}

/******************************************************************************
 * Function Name: iso_audio_setup_data_path
 *
 * Summary: iso audio setup data path when CIS connection establish
 *
 * Parameters:
 *  uint16_t                conn_hdl
 *      : CIS conneciton handle
 *
 *  uint16_t                direction
 *      : CIS direction
 *
 *  wiced_bt_ga_bap_csc_t   *p_csc
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t iso_audio_setup_data_path(uint16_t conn_hdl, uint16_t direction, wiced_bt_ga_bap_csc_t *p_csc)
{
    lc3_config_t codec_config;
    wiced_bool_t codec_config_sts;
    wiced_bt_isoc_data_path_direction_t data_path_dir = WICED_BLE_ISOC_DPD_MAX_DIRECTIONS;
    int numOfChannels = 1;
    wiced_bool_t is_cis = FALSE;

    WICED_BT_TRACE("[%s] conn_hdl [0x%x] dir [%d] p_csc [0x%x]\n", __FUNCTION__, conn_hdl, direction, p_csc);
    if (!p_csc) return WICED_BADARG;

    /* check if CIS is established or BIS is created before setting up data path */
    is_cis = wiced_bt_isoc_is_cis_connected_by_conn_id(conn_hdl);
    if (!is_cis) {
        return WICED_BADARG;
    }

    WICED_BT_TRACE("[%s] is_cis %d \n", __FUNCTION__, is_cis);

    /* determine num of channels required and configure codec accordingly
    (if not configured default to 1) */
    if (p_csc->audio_channel_allocation) {
        numOfChannels = count_set_bits(p_csc->audio_channel_allocation);
        if (numOfChannels > ISO_AUDIO_MAX_PARAM_COUNT) {
            return WICED_UNSUPPORTED;
        }
    }

    // if (!p_csc->csc_metadata.sampling_frequency_present) p_csc->sampling_frequency = 48000;
    // if (!p_csc->csc_metadata.frame_duration_present) p_csc->frame_duration = 10000;
    // if (!p_csc->csc_metadata.octets_per_codec_frame_present) p_csc->octets_per_codec_frame = 155;

    data_path_dir = (direction == WICED_BLE_ISOC_DPD_INPUT_BIT) ? WICED_BLE_ISOC_DPD_INPUT : WICED_BLE_ISOC_DPD_OUTPUT;

    WICED_BT_TRACE("[%s] dir %s sampleRate %d sduInterval %d octetsPerFrame %d numOfChannels %d conn_hdl 0x%x\n",
                   __FUNCTION__,
                   (data_path_dir == WICED_BLE_ISOC_DPD_INPUT) ? "Source" : "Sink",
                   p_csc->sampling_frequency,
                   p_csc->frame_duration,
                   p_csc->octets_per_codec_frame,
                   numOfChannels,
                   conn_hdl);

    if (!p_csc->sampling_frequency || !p_csc->frame_duration || !p_csc->octets_per_codec_frame) return WICED_ERROR;

    codec_config.sampleRate = p_csc->sampling_frequency;
    codec_config.sduInterval = p_csc->frame_duration;
    codec_config.octetsPerFrame = p_csc->octets_per_codec_frame;
    codec_config.sampleWidthInBits = 16;

    // setup ISO data path and LC3 codec for INPUT from controller or OUTPUT to controller
    if (!wiced_bt_isoc_setup_data_path(conn_hdl, is_cis, data_path_dir, WICED_BLE_ISOC_DPID_HCI, 0)) {
        return WICED_ERROR;
    }

    for (int i = 0; i < numOfChannels; i++) {
        codec_config_sts = (direction == WICED_BLE_ISOC_DPD_INPUT_BIT) ? lc3_codec_initializeEncoder(i, &codec_config)
                                                                       : lc3_codec_initializeDecoder(i, &codec_config);
        if (FALSE == codec_config_sts) {
            WICED_BT_TRACE_CRIT("[%s] lc3_codec_initialize NOT SUCCESSFUL", __FUNCTION__);
            return WICED_ERROR;
        }
    }

    init_stream_info(conn_hdl,
                     is_cis,
                     p_csc->sampling_frequency,
                     p_csc->octets_per_codec_frame,
                     p_csc->frame_duration,
                     numOfChannels);

    audio_driver_init(direction, numOfChannels, codec_config.sampleRate, LINUX_ALSA_LATENCY);
    audio_driver_load_wave_file(direction, codec_config.sampleRate);

    return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: iso_audio_remove_data_path
 *
 * Summary: iso audio remove data path when CIS disconnection
 *
 * Parameters:
 *  uint16_t                        conn_hdl
 *  wiced_bt_isoc_data_path_bit_t   dir
 *  uint8_t                         *idx_list
 *
 * Return:
 *  None
 *
******************************************************************************/
void iso_audio_remove_data_path(uint16_t conn_hdl, wiced_bt_isoc_data_path_bit_t dir, uint8_t *idx_list)
{
    wiced_bool_t is_cis = FALSE;
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);

    if (p_stream_info == NULL) return;

    int num_channels = p_stream_info->num_of_channels;

    WICED_BT_TRACE("[%s] [dir %d] [conn_hdl %x] [num of channel %x]",
                   __FUNCTION__,
                   dir,
                   conn_hdl,
                   p_stream_info->num_of_channels);

    audio_driver_deinit(dir);
    deinit_stream_info(conn_hdl);
    // if removing paths for both directions release both decoder instances,
    // otherwise only release the one corresponding to the removed path

    if (WICED_BLE_ISOC_DPD_INPUT_BIT & dir) {
        for (int i = 0; i < num_channels; i++) {
            lc3_codec_releaseEncoder(i);
        }
    }
    else if (WICED_BLE_ISOC_DPD_OUTPUT_BIT & dir) {
        for (int i = 0; i < num_channels; i++) {
            lc3_codec_releaseDecoder(i);
        }
    }

    /* check if CIS is established before setting up data path */
    is_cis = wiced_bt_isoc_is_cis_connected_by_conn_id(conn_hdl);
    if (!is_cis) return;

    if (!wiced_bt_isoc_remove_data_path(conn_hdl, is_cis, dir))
        WICED_BT_TRACE_CRIT("[%s] No active data path\n", __FUNCTION__);
}

//read from wav file
//Encode (figureout number of channels based on configuration)
/******************************************************************************
 * Function Name: tx_iso_data
 *
 * Summary: send ISOC data, do lc3 encode
 *
 * Parameters:
 *  wiced_bool_t    is_cis
 *  uint16_t        conn_handle
 *  uint16_t        octets_per_frame
 *  uint8_t         num_of_channels
 *  uint16_t        frame_duration
 *
 * Return:
 *  None
******************************************************************************/
static void tx_iso_data(wiced_bool_t is_cis, uint16_t conn_handle, uint16_t octets_per_frame, uint8_t num_of_channels, uint16_t frame_duration)
{
    uint8_t wav_data_l[MAX_INPUT_SAMPLE_SIZE_IN_BYTES] = {0};
    uint8_t wav_data_r[MAX_INPUT_SAMPLE_SIZE_IN_BYTES] = {0};
    int num_of_samples = 0;
    int sample_width_in_bytes = 2; //FIXME: remove hardcoding to 16 bit
    wiced_bt_ga_bap_csc_t *p_csc = NULL;
    uint8_t *p_data = NULL;

    //WICED_BT_TRACE("[%s] is_cis %d octets_per_frame %d num_of_channels %d conn_handle 0x%x",
    //               __FUNCTION__,
    //               is_cis,
    //               octets_per_frame,
    //               num_of_channels,
    //               conn_handle);

    if (!p_buf) {
        WICED_BT_TRACE_CRIT("[%s] p_buf is NULL\n", __FUNCTION__);
        return;
    }

    num_of_samples = audio_module_get_wave_data(wav_data_l, wav_data_r, frame_duration);

    p_data = p_buf  + iso_dhm_get_header_size(); //leave space for header

    //TRACE_LOG("num_of_samples:%d, sample_width_in_bytes:%d, octets_per_frame:%d, frame_duration:%d\n", num_of_samples, sample_width_in_bytes, octets_per_frame, frame_duration);
    lc3_codec_Encode(0, wav_data_l, num_of_samples * sample_width_in_bytes, p_data, octets_per_frame);

    if (2 == num_of_channels)
        lc3_codec_Encode(1,
                         wav_data_r,
                         num_of_samples * sample_width_in_bytes,
                         p_data + octets_per_frame,
                         octets_per_frame);

#ifdef debug_googlelc3
    lc3bin_write_data(fp_out, p_data, 2, octets_per_frame);
#endif

    iso_dhm_send_packet(is_cis, conn_handle, 0, p_buf, octets_per_frame * num_of_channels);
}

/******************************************************************************
 * Function Name: iso_audio_start_stream
 *
 * Summary: get a buffer and start send isco data
 *
 * Parameters:
 *  uint16_t conn_hdl
 *
 * Return:
 *  None
 *
******************************************************************************/
void iso_audio_start_stream(uint16_t conn_hdl)
{
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);
    if (p_buf != NULL)
    {
        TRACE_ERR("p_buf != NULL already get the buffer\n");
        return;
    }
    p_buf = (uint8_t *)wiced_bt_get_buffer_from_pool(ga_iso_audio_pool);

    WICED_BT_TRACE("[%s] p_stream_info 0x%x conn_hdl 0x%x\n", __FUNCTION__, p_stream_info, conn_hdl);
    if (!p_stream_info) return;

    for (int i = 0; i < FIRST_ISOC_DATA; i++) {
        tx_iso_data(p_stream_info->is_cis, conn_hdl, p_stream_info->octets_per_frame, p_stream_info->num_of_channels, p_stream_info->frame_duration);
    }
}

/******************************************************************************
 * Function Name: iso_audio_stop_stream
 *
 * Summary: stop send isoc data and free the buffer when data path remove
 *
 * Parameters:
 *  uint16_t conn_hdl
 *
 * Return:
 *  None
 *
******************************************************************************/
void iso_audio_stop_stream(uint16_t conn_hdl)
{
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);
    if(p_stream_info)
        p_stream_info->b_stream_active = 0;
    wiced_bt_free_buffer(p_buf);
    p_buf = NULL;
}

/******************************************************************************
 * Function Name: num_complete_handler
 *
 * Summary: send ISOC data complete, callback for app send more data
 *
 * Parameters:
 *  uint16_t conn_hdl
 *  uint16_t num_sent
 *
 * Return:
 *  None
 *
******************************************************************************/
static void num_complete_handler(uint16_t conn_hdl, uint16_t num_sent)
{
    ga_iso_audio_stream_info_t *p_stream_info = get_stream_info(conn_hdl);

    for (int i = 0; i < num_sent; i++) {
        if (!p_stream_info || !p_stream_info->b_stream_active) return;
        tx_iso_data(p_stream_info->is_cis, conn_hdl, p_stream_info->octets_per_frame, p_stream_info->num_of_channels, p_stream_info->frame_duration);
    }
}

/******************************************************************************
 * Function Name: rx_handler
 *
 * Summary: no use in Unicast source
 *
 * Parameters:
 *  uint16_t conn_hdl
 *  uint8_t *p_data
 *  uint32_t length
 *
 * Return:
 *  None
******************************************************************************/
static void rx_handler(uint16_t conn_hdl, uint8_t *p_data, uint32_t length)
{
}

/******************************************************************************
 * Function Name: iso_create_pool
 *
 * Summary: create isoc buufer pool when iso_audio_init
 *
 * Parameters:
 *  const wiced_bt_cfg_isoc_t *p_iso_cfg
 *
 * Return:
 *  None
 *
******************************************************************************/
void iso_create_pool(const wiced_bt_cfg_isoc_t *p_iso_cfg)
{
    //create a iso pool
    int buff_size = iso_dhm_get_buffer_size(p_iso_cfg);

    // Allocate only once, allowing multiple calls to update callbacks
    if (!ga_iso_audio_pool)
        ga_iso_audio_pool = wiced_bt_create_pool("ISO SDU", buff_size, p_iso_cfg->max_buffers_per_cis, NULL);

    if (!ga_iso_audio_pool)
    {
        WICED_BT_TRACE_CRIT("[%s] ga_iso_audio_pool is NULL\n", __FUNCTION__);
        return;
    }

    WICED_BT_TRACE("[%s] g_cis_iso_pool 0x%x size %d count %d",
                   __FUNCTION__,
                   ga_iso_audio_pool,
                   buff_size,
                   p_iso_cfg->max_buffers_per_cis);

    iso_dhm_init(num_complete_handler, rx_handler);
}

/******************************************************************************
 * Function Name: iso_audio_init
 *
 * Summary: isoc audio init and reset lc3 codec 
 *
 * Parameters:
 *  const wiced_bt_cfg_isoc_t *p_iso_cfg
 *
 * Return:
 *  None
 *
******************************************************************************/
void iso_audio_init(const wiced_bt_cfg_isoc_t *p_iso_cfg)
{
    iso_create_pool(p_iso_cfg);
    lc3_codec_reset();

#ifdef debug_googlelc3
    fp_out = fopen("16klc3.out", "wb");
    lc3bin_write_header(fp_out,
          10000, 16000, 64000, 2, 474059);
#endif

}

/******************************************************************************
 * Function Name: unicast_source_find_ase_with_cis_id
 *
 * Summary: find the ase
 *
 * Parameters:
 *  uint8_t     cig_id,
 *  uint8_t     cis_id,
 *  uint8_t     char_type,
 *  uint16_t    *p_conn_id
 *
 * Return:
 *  unicast_source_ase_data_t*
 *
******************************************************************************/
unicast_source_ase_data_t *unicast_source_find_ase_with_cis_id(uint8_t cig_id,
                                                               uint8_t cis_id,
                                                               uint8_t char_type,
                                                               uint16_t *p_conn_id)
{
    unicast_source_clcb_t *p_clcb = g_unicast_source_gatt_cb.unicast_clcb;
    int limit = sizeof(g_unicast_source_gatt_cb.unicast_clcb) / sizeof(g_unicast_source_gatt_cb.unicast_clcb[0]);
    unicast_source_ase_data_t *p_ase = NULL;

    while (limit--) {
        if (char_type == ASCS_SOURCE_ASE_CHARACTERISTIC) {
            p_ase = p_clcb->p_remote_ase_data;
            int num_ase = p_clcb->num_remote_ases;
            for (; num_ase--; p_ase++) {
                if (p_ase->data.ase_state < WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED) {
                    continue;
                }
                if ((p_ase->data.qos_configured.cig_id == cig_id) && (p_ase->data.qos_configured.cis_id == cis_id)) {
                    if (p_conn_id) {
                        *p_conn_id = p_clcb->conn_id;
                    }
                    return p_ase;
                }
            }
        }
        p_clcb++;
    }

    return NULL;
}

/******************************************************************************
 * Function Name: set_conn_on_ase_id
 *
 * Summary: No use now
 *
 * Parameters:
 *  wiced_bt_isoc_cis_request_data_t *p_cis_req
 *  int type
 *
 * Return:
 *  int
 *
******************************************************************************/
int set_conn_on_ase_id(wiced_bt_isoc_cis_request_data_t *p_cis_req, int type)
{
    uint16_t conn_id;
    unicast_source_ase_data_t *p_ase =
        unicast_source_find_ase_with_cis_id(p_cis_req->cig_id, p_cis_req->cis_id, type, &conn_id);
    int ase_id = (p_ase) ? p_ase->data.p_ase_info->ase_id : 0xFF;

    WICED_BT_TRACE("cis_id %d cig_id %d cis_conn_handle %d acl_handle %d p_ase %x ase_id %d\n",
                   __FUNCTION__,
                   p_cis_req->cis_id,
                   p_cis_req->cig_id,
                   p_cis_req->cis_conn_handle,
                   p_cis_req->acl_handle,
                   p_ase,
                   ase_id);

    if (p_ase) {
        p_ase->cis_conn_hdl = p_cis_req->cis_conn_handle;
    }

    return ase_id;
}

/******************************************************************************
 * Function Name: unicast_source_cis_handle_connection
 *
 * Summary: when CIS ESTABLISHED handle the connection
 *
 * Parameters:
 *  wiced_bool_t is_client,
 *  uint8_t cig_id,
 *  uint8_t cis_id,
 *  uint8_t char_type
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_cis_handle_connection(wiced_bool_t is_client,
                                                 uint8_t cig_id,
                                                 uint8_t cis_id,
                                                 uint8_t char_type)
{
    wiced_result_t res = WICED_ERROR;
    uint16_t conn_id;
    unicast_source_ase_data_t *p_ase_data = unicast_source_find_ase_with_cis_id(cig_id, cis_id, char_type, &conn_id);
    wiced_bt_isoc_data_path_direction_t data_path_dir = WICED_BLE_ISOC_DPD_MAX_DIRECTIONS;

    CHECK_FOR_NULL_AND_RETURN(p_ase_data);

    WICED_BT_TRACE("[%s] ase_id %d, state : %d characteristic_type %d",
                   __FUNCTION__,
                   p_ase_data->data.p_ase_info->ase_id,
                   p_ase_data->data.ase_state,
                   p_ase_data->data.p_ase_info->ase_type);

    /*assign CIS to ASE*/
    p_ase_data->cis_conn_hdl = wiced_bt_isoc_get_cis_conn_handle(cig_id, cis_id);

    if (is_client) {
        res = iso_audio_setup_data_path(p_ase_data->cis_conn_hdl,
                                        p_ase_data->data.p_ase_info->ase_type,
                                        &p_ase_data->data.codec_configured.csc);
    }
    else {
        if (ASCS_SINK_ASE_CHARACTERISTIC == p_ase_data->data.p_ase_info->ase_type)
            res = iso_audio_setup_data_path(p_ase_data->cis_conn_hdl,
                                            WICED_BLE_ISOC_DPD_OUTPUT_BIT,
                                            &p_ase_data->data.codec_configured.csc);
        else
            res = iso_audio_setup_data_path(p_ase_data->cis_conn_hdl,
                                            WICED_BLE_ISOC_DPD_INPUT_BIT,
                                            &p_ase_data->data.codec_configured.csc);
    }

    if (res) {
        WICED_BT_TRACE_CRIT("[%s] data path setup unsuccessful...(err:%d)\n", __FUNCTION__, res);
    }
}

/******************************************************************************
 * Function Name: unicast_source_cis_handle_disconnection
 *
 * Summary: handle disconnection when CIS DISCONNECTED
 *
 * Parameters:
 *  wiced_bool_t is_client,
 *  uint8_t cig_id,
 *  uint8_t cis_id,
 *  uint8_t char_type
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_cis_handle_disconnection(wiced_bool_t is_client,
                                                    uint8_t cig_id,
                                                    uint8_t cis_id,
                                                    uint8_t char_type)
{
    unicast_source_ase_data_t *p_ase_data = NULL;
    uint16_t conn_id = 0;

    if (is_client) return;

    p_ase_data = unicast_source_find_ase_with_cis_id(cig_id, cis_id, char_type, &conn_id);
    CHECK_FOR_NULL_AND_RETURN(p_ase_data);

    iso_audio_remove_data_path(p_ase_data->cis_conn_hdl, p_ase_data->data.p_ase_info->ase_type, &p_ase_data->lc3_index);
}

/******************************************************************************
 * Function Name: unicast_source_get_ase_app_data_ptr_by_cis_conn_hdl
 *
 * Summary: get ase data by cis connection handle
 *
 * Parameters:
 *  uint16_t cis_conn_hdl,
 *  uint8_t char_type,
 *  uint16_t *p_conn_id
 *
 * Return:
 *  unicast_source_ase_data_t*
 *
******************************************************************************/
unicast_source_ase_data_t *unicast_source_get_ase_app_data_ptr_by_cis_conn_hdl(uint16_t cis_conn_hdl,
                                                                               uint8_t char_type,
                                                                               uint16_t *p_conn_id)
{
    unicast_source_clcb_t *p_clcb = g_unicast_source_gatt_cb.unicast_clcb;
    int limit = sizeof(g_unicast_source_gatt_cb.unicast_clcb) / sizeof(g_unicast_source_gatt_cb.unicast_clcb[0]);
    unicast_source_ase_data_t *p_ase = NULL;
    int num_ase = 0;
    while (limit--) {
        p_ase = p_clcb->p_remote_ase_data;
        num_ase = p_clcb->num_remote_ases;
        for (; num_ase--; p_ase++) {
            if (p_ase->data.ase_state < WICED_BT_GA_ASCS_STATE_QOS_CONFIGURED) {
                continue;
            }
            if (p_ase->cis_conn_hdl == cis_conn_hdl) {
                if (p_conn_id) {
                    *p_conn_id = p_clcb->conn_id;
                }
                return p_ase;
            }
        }
        p_clcb++;
    }

    return NULL;
}

/******************************************************************************
 * Function Name: unicast_source_cis_handle_data_path_setup
 *
 * Summary: when WICED_BLE_ISOC_DATA_PATH_SETUP event
 *
 * Parameters:
 *  uint16_t        cis_conn_hdl
 *  wiced_bool_t    is_cl
 *  wiced_bt_isoc_data_path_direction_t dpd
 *
 * Return:
 *  None
 *
******************************************************************************/
static void unicast_source_cis_handle_data_path_setup(uint16_t cis_conn_hdl,
                                                      wiced_bool_t is_cl,
                                                      wiced_bt_isoc_data_path_direction_t dpd)
{
    wiced_bool_t is_inp = (WICED_BLE_ISOC_DPD_INPUT == dpd);
    uint8_t char_type_local = (is_inp) ? ASCS_SOURCE_ASE_CHARACTERISTIC : ASCS_SINK_ASE_CHARACTERISTIC;
    uint8_t char_type_peer = (is_inp) ? ASCS_SINK_ASE_CHARACTERISTIC : ASCS_SOURCE_ASE_CHARACTERISTIC;
    uint8_t char_type = (is_cl) ? char_type_peer : char_type_local;
    uint16_t conn_id = 0;
    unicast_source_ase_data_t *p_ase_data =
        unicast_source_get_ase_app_data_ptr_by_cis_conn_hdl(cis_conn_hdl, char_type, &conn_id);
    gatt_intf_service_object_t *p_service = NULL;

    CHECK_FOR_NULL_AND_RETURN(p_ase_data);

    WICED_BT_TRACE("[%s] is_inp %d is_cl %d\n", __FUNCTION__, is_inp, is_cl);

    p_service = gatt_interface_get_service_by_uuid_and_conn_id(is_cl ? conn_id : 0, &ga_service_uuid_ascs);
    CHECK_FOR_NULL_AND_RETURN(p_service);

    // Update ASE data to indicate data path is setup successfully
    p_ase_data->data_path_established = 1;

    if (is_cl) {
        // if client + source and in streaming state, start audio streaming
        // if client + sink and in enabling state, send receiver start ready

        if (ASCS_SINK_ASE_CHARACTERISTIC == char_type && WICED_BT_GA_ASCS_STATE_STREAMING == p_ase_data->data.ase_state)
        {
            iso_audio_start_stream(cis_conn_hdl);
        }
        else if (ASCS_SOURCE_ASE_CHARACTERISTIC == char_type &&
                 WICED_BT_GA_ASCS_STATE_ENABLING == p_ase_data->data.ase_state) {
            wiced_bt_ga_ascs_send_receiver_start_stop_ready(conn_id,
                                                            p_service,
                                                            p_ase_data->data.p_ase_info->ase_id,
                                                            TRUE);
        }
        else {
            WICED_BT_TRACE_CRIT("[%s] cl unexpected state %d char_type %d\n",
                                __FUNCTION__,
                                p_ase_data->data.ase_state,
                                char_type);
        }
    }
    else {
        // if server + source and in streaming state, start audio streaming
        // if server + sink and in enabling state, transition to streaming state

        if (ASCS_SOURCE_ASE_CHARACTERISTIC == char_type &&
            WICED_BT_GA_ASCS_STATE_STREAMING == p_ase_data->data.ase_state) {
            iso_audio_start_stream(cis_conn_hdl);
        }
        else if (ASCS_SINK_ASE_CHARACTERISTIC == char_type &&
                 WICED_BT_GA_ASCS_STATE_ENABLING == p_ase_data->data.ase_state) {
            gatt_intf_attribute_t ase_char;

            p_ase_data->data.ase_state = WICED_BT_GA_ASCS_STATE_STREAMING;
            gatt_interface_notify_characteristic(conn_id,
                                                 p_service,
                                                 ascs_init_characteristic(&ase_char, p_ase_data),
                                                 &p_ase_data->data);
        }
        else {
            WICED_BT_TRACE_CRIT("[%s] sr unexpected state %d char_type %d\n",
                                __FUNCTION__,
                                p_ase_data->data.ase_state,
                                char_type);
        }
    }
}

/******************************************************************************
 * Function Name: unicast_source_isoc_event_handler
 *
 * Summary: ISOC EVENT Handler
 *      event:  WICED_BLE_ISOC_SET_CIG_CMD_COMPLETE
 *              WICED_BLE_ISOC_CIS_ESTABLISHED
 *              WICED_BLE_ISOC_CIS_DISCONNECTED
 *              WICED_BLE_ISOC_DATA_PATH_SETUP
 *              WICED_BLE_ISOC_DATA_PATH_REMOVED
 *
 * Parameters:
 *  wiced_bt_isoc_event_t event
 *  wiced_bt_isoc_event_data_t *p_event_data
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_isoc_event_handler(wiced_bt_isoc_event_t event, wiced_bt_isoc_event_data_t *p_event_data)
{
    static wiced_bool_t is_client = TRUE;
    unicast_source_ase_data_t *p_ase_data = NULL;
    wiced_bt_isoc_cig_status_data_t *p_cig_status_data = NULL;

    WICED_BT_TRACE("[%s] event %d ", __FUNCTION__, event);

    switch (event)
    {
    case WICED_BLE_ISOC_SET_CIG_CMD_COMPLETE:

        p_cig_status_data = &p_event_data->cig_status_data;

        WICED_BT_TRACE("status %d cig_id %d cis_count %d CIS Handle %d\n",
                       p_cig_status_data->status,
                       p_cig_status_data->cig_id,
                       p_cig_status_data->cis_count,
                       p_cig_status_data->cis_connection_handle_list[0]);

        if (WICED_BT_SUCCESS != p_cig_status_data->status) return;

        wiced_bt_isoc_update_cis_conn_handle(p_cig_status_data->cig_id,
                                             1,
                                             p_cig_status_data->cis_connection_handle_list[0]);
        break;

    case WICED_BLE_ISOC_CIS_ESTABLISHED:
        if (p_event_data->cis_established_data.status)
        {
            WICED_BT_TRACE_CRIT("[%s] status %d \n", __FUNCTION__, p_event_data->cis_established_data.status);
            return;
        }

        // Setup data path after CIS establishment for sink role as server/client,
        // Data path is setup upon receiving streaming notification / receiver start ready
        // as client and server
        if (is_client)
        {
            unicast_source_cis_handle_connection(is_client,
                                                 p_event_data->cis_established_data.cig_id,
                                                 p_event_data->cis_established_data.cis_id,
                                                 ASCS_SOURCE_ASE_CHARACTERISTIC);
        }
        else
        {
            unicast_source_cis_handle_connection(is_client,
                                                 p_event_data->cis_established_data.cig_id,
                                                 p_event_data->cis_established_data.cis_id,
                                                 ASCS_SINK_ASE_CHARACTERISTIC);
        }
        break;

    case WICED_BLE_ISOC_CIS_DISCONNECTED:
        // in case of bi-directional CIS handle for both the ASE's attached to the CIS
        unicast_source_cis_handle_disconnection(is_client,
                                                p_event_data->cis_disconnect.cig_id,
                                                p_event_data->cis_disconnect.cis_id,
                                                ASCS_SINK_ASE_CHARACTERISTIC);

        unicast_source_cis_handle_disconnection(is_client,
                                                p_event_data->cis_disconnect.cig_id,
                                                p_event_data->cis_disconnect.cis_id,
                                                ASCS_SOURCE_ASE_CHARACTERISTIC);
        break;

    case WICED_BLE_ISOC_DATA_PATH_SETUP:
        if (p_event_data->datapath.status)
        {
            WICED_BT_TRACE_CRIT("[%s] Data path setup not successful\n", __FUNCTION__);
            return;
        }

        if (wiced_bt_isoc_is_cis_connected_by_conn_id(p_event_data->datapath.conn_hdl))
        {
            unicast_source_cis_handle_data_path_setup(p_event_data->datapath.conn_hdl,
                                                      is_client,
                                                      p_event_data->datapath.data_path_dir);
        }
        break;

    case WICED_BLE_ISOC_DATA_PATH_REMOVED:
        if (p_event_data->datapath.status)
        {
            WICED_BT_TRACE_CRIT("[%s] Data path removal not successful\n", __FUNCTION__);
        }

        if (is_client)
        { // TODO: check id data path is removed in both directions
            WICED_BT_TRACE("[%s] disconnecting CIS..\n", __FUNCTION__);
            wiced_bt_isoc_disconnect_cis(p_event_data->datapath.conn_hdl);
            iso_audio_stop_stream(p_event_data->datapath.conn_hdl);
        }
        break;

    default:
        break;
    }
}
