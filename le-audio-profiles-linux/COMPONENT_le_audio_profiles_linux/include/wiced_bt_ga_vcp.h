/*
 * $ Copyright Cypress Semiconductor $
 */
 
/** @file
 *
 * Volume Control Profile (VCP) Application Programming Interface
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_aics.h"

#define VCP_TRACE(...)
#define VCP_TRACE_CRIT(...)

/**
 * @addtogroup Volume_Control_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_vcp
 * @{
 */

/** VCS Structure */
extern const gatt_intf_service_methods_t vcs_methods;

/**
 * \brief Change local change_counter value
 * \details Sets the local change counter value
 *
 * @param[in]   conn_id : GATT Connection ID
 * @param[in]   p_service : Instance of the volume control client
 * @param[in]   change_counter : Change counter for VCS
 */
void wiced_bt_ga_vcp_set_change_counter (uint16_t conn_id, gatt_intf_service_object_t* p_service, uint8_t change_counter);

/**
 * \brief Change vocs change_counter value
 * \details Sets the vocs change counter value
 *
 * @param[in]   p_service : instance of the volume control client
 * @param[in]   vocs_index : index of the instance of Volume offset control client
 * @param[in]   change_counter : Change counter for VCS
 */
void wiced_bt_ga_vcp_vocs_set_change_counter(gatt_intf_service_object_t* p_service, uint8_t vocs_index, uint8_t change_counter);

/**
 * \brief Set the volume offset value
 * \details Sets the volume offset value of the volume offset control service
 *
 * @param[in]   conn_id : GATT Connection ID
 * @param[in]   p_service :       instance of the volume control client
 * @param[in]   vocs_index :      index of the instance of Volume offset control client
 * @param[in]   volume_offset :   new volume offset value to be set

 * @return  wiced_bt_gatt_status_t result of the write operation. In case the operation is not successful due to invalid
 *          change counter value or invalid opcode an WICED_BT_VCP_ERROR event will be sent to the registered application callback with appropriate error code.
*/
wiced_bt_gatt_status_t wiced_bt_ga_vcp_set_volume_offset (uint16_t conn_id, gatt_intf_service_object_t* p_service, uint8_t vocs_index, int16_t volume_offset);

/**
 * \brief Set the audio location of the device
 * \details Sets the audio location of the volume offset control service
 *
 * @param[in]   conn_id : GATT Connection ID
 * @param[in]   p_service  :     instance of the volume control client
 * @param[in]   vocs_index :     index of the instance of Volume offset control client
 * @param[in]   audio_location :  audio location value to be set

 * @return  wiced_bt_gatt_status_t result of the write operation.
*/
wiced_bt_gatt_status_t wiced_bt_ga_vcp_set_audio_location (uint16_t conn_id, gatt_intf_service_object_t* p_service, uint8_t vocs_index, wiced_bt_ga_volume_audio_location_t audio_location);

/**
 * \brief Set the audio output description of the device
 * \details Sets the audio output description of the volume offset control service
 *
 * @param[in]   conn_id : GATT Connection ID
 * @param[in]   p_service  :     instance of the volume control client
 * @param[in]   vocs_index :     index of the instance of Volume offset control client
 * @param[in]   description :    volume output description value to be set

 * @return  wiced_bt_gatt_status_t result of the write operation.
*/
wiced_bt_gatt_status_t wiced_bt_ga_vcp_set_audio_output_description (uint16_t conn_id, gatt_intf_service_object_t* p_service, uint8_t vocs_index, char *description);

/**
 * \brief Change gain setting
 * \details changes the gain setting of the Audio input control service
 *
 * @param[in]   conn_id : GATT Connection ID
 * @param[in]   p_service :       instance of the volume control client
 * @param[in]   aics_index :      AICS instance id
 * @param[in]   gain_val :        Desired Gain value
 *
 * @return  wiced_bt_gatt_status_t result of the write operation. In case the operation is not successful due to invalid
 *          change counter value or invalid opcode or value out of range an WICED_BT_AICS_ERROR event will be sent to the registered application callback with appropriate error code.
*/
wiced_bt_gatt_status_t wiced_bt_ga_vcp_set_gain_setting(uint16_t conn_id, gatt_intf_service_object_t* p_service, uint8_t aics_index, int8_t gain_val);

/**
 * \brief Change the mute mode
 * \details changes the mute mode of the Audio input control service
 *
 * @param[in]   conn_id : GATT Connection ID
 * @param[in]   p_service :       instance of the volume control client
 * @param[in]   aics_index :      AICS instance id
 * @param[in]   mute_mode :       Desired mute mode (WICED_BT_GA_AICS_UNMUTE or WICED_BT_GA_AICS_MUTE)
 *
 * @return  wiced_bt_gatt_status_t result of the write operation. In case the operation is not successful due to invalid
 *          change counter value or invalid opcode an WICED_BT_INTERNAL_ERROR_INVALID_CHANGE_COUNTER or WICED_BT_AICS_ERROR event respectively will be sent to the registered application callback with appropriate error code.
*/
wiced_bt_gatt_status_t wiced_bt_ga_vcp_set_mute_mode(uint16_t conn_id, gatt_intf_service_object_t* p_service, uint8_t aics_index, wiced_bt_ga_aics_mute_t mute_mode);

/**
 * \brief Change the gain mode
 * \details changes the gain mode of the Audio input control service
 *
 * @param[in]   conn_id : GATT Connection ID
 * @param[in]   p_service :       instance of the volume control client
 * @param[in]   aics_index :      AICS instance id
 * @param[in]   gain_mode :       Desired gain mode (WICED_BT_GA_AICS_GAIN_MODE_MANUAL or WICED_BT_GA_AICS_GAIN_MODE_AUTO)
 *
 * @return  wiced_bt_gatt_status_t result of the write operation. In case the operation is not successful due to invalid
 *          change counter value or invalid opcode an WICED_BT_INTERNAL_ERROR_INVALID_CHANGE_COUNTER or WICED_BT_AICS_ERROR event respectively will be sent to the registered application callback with appropriate error code.
*/
wiced_bt_gatt_status_t wiced_bt_ga_vcp_set_gain_mode(uint16_t conn_id, gatt_intf_service_object_t* p_service, uint8_t aics_index, wiced_bt_ga_aics_gain_mode_t gain_mode);

/**
 * \brief Set the audio input description of the device
 * \details Sets the audio input description of the Audio Input Control Service
 *
 * @param[in]   conn_id : GATT Connection ID
 * @param[in]   p_service :      instance of the volume control client
 * @param[in]   aics_index :      index of the instance of Audio Input control client
 * @param[in]   audio_desc :      Audio Description

 * @return  wiced_bt_gatt_status_t result of the write operation.
*/
wiced_bt_gatt_status_t wiced_bt_ga_vcp_set_audio_description(uint16_t conn_id, gatt_intf_service_object_t* p_service, uint8_t aics_index, char *audio_desc);

/**
 * \brief Change AICS change_counter value
 * \details Sets the aics change counter value
 *
 * @param[in]   p_service :       instance of the volume control client
 * @param[in]   aics_index :      index of the instance of AICS
 * @param[in]   change_counter :  Change counter value
 */
void wiced_bt_ga_vcp_aics_set_change_counter(gatt_intf_service_object_t* p_service, uint8_t aics_index, uint8_t change_counter);



#ifdef __cplusplus
}
#endif
/**@} wiced_bt_ga_vcp */
/**@} Volume_Control_APIs */
