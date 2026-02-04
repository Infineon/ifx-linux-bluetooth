/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __WICED_BT_GA_BROADCAST_H__
#define __WICED_BT_GA_BROADCAST_H__

#include "wiced_bt_ga_bap.h"
#include "wiced_data_types.h"
#include "le_audio_pbp.h"

/**
 * @addtogroup Stream_Control_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_bap_broadcast
 * @{
 */


#define BROADCAST_MAX_SUB_GROUP 2 /**< Max subgroups */
#define BROADCAST_MAX_BIS_PER_SUB_GROUP 2 /**< Max BIS per sub group */
#define BROADCAST_AUDIO_ANNOUNCEMENT_SIZE 6 /**< Size of Broadcast Announcement */
#define SOLICITATION_REQ_SIZE 3 /**< Size of Solicitation request */

#define BAP_BROADCAST_TRACE(...) /**< Enable this to get BAP Broadcast traces */
#define BAP_BROADCAST_TRACE_CRIT(...) /**< Enable this to get BAP Broadcast traces */

/**
* @brief Broadcast States
*/
typedef enum
{
    BAP_BROADCAST_STATE_IDLE, /**< Broadcast Source is in idle state */
    BAP_BROADCAST_STATE_CONFIGURED, /**< Broadcast Source is in configured state */
    BAP_BROADCAST_STATE_STREAMING, /**< Broadcast Source is in Streaming state */
} le_audio_bap_broadcast_state_t;

/**
* @brief Broadcast BIS information
*/
typedef struct
{
    uint8_t bis_idx; /**< Index of the bis */
    wiced_bool_t b_bis_data_path_setup; /**< is datapath setup */
    wiced_bt_ga_bap_csc_t bis_csc; /**< BIS codec specific configuration */
} le_audio_bap_broadcast_bis_group_t;

/**
* @brief Broadcast Subgroup information
*/
typedef struct
{
    wiced_bt_ga_bap_codec_id_t codec_id; /**< codec id for the subgroup */
    wiced_bt_ga_bap_csc_t csc; /**< codec specific configuration */
    wiced_bt_ga_bap_metadata_t metadata; /**< metadata specific for subgroup */
    uint8_t bis_cnt; /**< Num of BIS in the subgroup */
    le_audio_bap_broadcast_bis_group_t bis_config[BROADCAST_MAX_BIS_PER_SUB_GROUP];/**< List of BIS configuraiton*/
} le_audio_bap_broadcast_sub_group_t;

/**
* @brief Broadcast BASE configuration
*/
typedef struct
{
    uint32_t broadcast_id;                                                 /**< Broadcast id of the stream */
    uint32_t presentation_delay;                                           /**< Presentation delay value to be used */
    le_audio_bap_broadcast_sub_group_t sub_group[BROADCAST_MAX_SUB_GROUP]; /**< List of Subgroups */
    le_audio_bap_broadcast_state_t state;                                  /**< Broadcast state */
    uint8_t sub_group_cnt;                                                 /**< Number of subgroups*/
} le_audio_bap_broadcast_base_t;


/**
 * @brief Check if Advertisement has Broadcast audio announcement information
 *
 * @param[in] adv_len : length of the advertisement report in \p p_adv_data
 * @param[in]   p_adv_data : Adv data received from peer
 * @param[out]   p_br_id : Broadcast ID from the advertisement
 * @return      status true or false
 */
wiced_bool_t le_audio_bap_broadcast_is_broadcast_announcement(uint16_t adv_len, uint8_t *p_adv_data, uint32_t *p_br_id);

/**
 * @brief Configure broadcast stream and start extended adv and periodic adv
 *
 * @param[in]   adv_sid : ADV sid value to be used
 * @param[in]   p_base : base information to be used in adv
 * @param[in]   p_ext_adv : extended adv data
 * @param[in]   adv_len : length of the extended adv data
 * @return      status true or false
 */
wiced_result_t le_audio_bap_broadcast_configure(uint8_t adv_sid, le_audio_bap_broadcast_base_t *p_base, uint8_t *p_ext_adv, uint8_t adv_len);

/**
 * @brief Check if Advertisement has Basic audio announcement information
 *
 * @param[in]  p_adv_data : Adv data received from peer
 * @param[in]  adv_len : length of p_adv_data
 * @param[out] p_base_len: length of the base adv in p_adv_data
 * @return  pointer to start of basic announcement of length in p_base_len, else NULL
 */
uint8_t *le_audio_bap_broadcast_is_basic_announcement(uint8_t *p_adv_data, uint16_t adv_len, uint8_t *p_base_len);


/**
 * @brief Parse Broadcast base information
 *
 * @param[in]  p_adv_data : Adv data received from peer
 * @param[in]  adv_data_len : length of the base information
 * @param[out]  p_base : Parsed BASE data
 * @return      status of the parsing operation
 */

wiced_result_t le_audio_bap_broadcast_parse_base_info(uint8_t *p_adv_data,
                                                      uint8_t adv_data_len,
                                                      le_audio_bap_broadcast_base_t *p_base);


/**
 * @brief Reconfigure basic audio announcements
 *
 * @param[in]  adv_sid : Adv set ID to indicate the corresponding Periodic Adv train
 * @param[in]  p_base : base information
 * @return     status of the reconfiguration
 */
wiced_result_t le_audio_bap_broadcast_reconfigure(uint8_t adv_sid, le_audio_bap_broadcast_base_t *p_base);

/**
 * @brief Only Metadata can be updated in the Streaming state (Upper layer should
 * ensure that only Metadata is modified in the BASE data)
 *
 * @param adv_sid Adv. set ID to indicate the corresponding Periodic Adv train
 * @param p_base BASE information
 * @return wiced_result_t WICED_SUCCESS if successful
 */
wiced_result_t le_audio_bap_broadcast_update_metadata(uint8_t adv_sid, le_audio_bap_broadcast_base_t *p_base);

/**
 * @brief Reconfigure basic audio announcements
 *
 * @param[in]  adv_sid : Adv set ID to indicate the corresponding Periodic Adv train
 * @param[in]  dev_name : device name to be used in solicitation request as a scan delegator
 * @param[in]  name_length : length of device name
 * @return     status of the request
 */
wiced_result_t le_audio_bap_broadcast_start_solicitation_requests(uint8_t adv_sid,
                                                                     char *dev_name,
                                                                     uint8_t name_length);

/**
 * @brief Build broadcast announcement adv data
 *
 * @param[out]  p_data : buffer where the adv data is stored after building, allocation has to be done before calling
 * @param[out]  len : length of the adv data after building
 * @param[in]  broadcast_id : broadcast ID of the stream
 * @return     status of the request
 */
wiced_result_t le_audio_bap_build_broadcast_annoncement_adv_data(uint8_t *p_data,
                                                                    uint8_t *len,
                                                                    uint32_t broadcast_id);

uint8_t le_audio_bap_build_base_data(le_audio_bap_broadcast_base_t *p_base, uint8_t *p_dst, uint32_t dst_len);
/**@} wiced_bt_ga_mics */
/**@} Volume_Control_APIs */

#endif //__WICED_BT_GA_BROADCAST_H__
