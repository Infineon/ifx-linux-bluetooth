/*
 * $ Copyright Cypress Semiconductor $
 */

/** @file
 *
 * Co-ordinate Set Identification Service (CSIS) Application Programming Interface
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "gatt_interface.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_csis_common.h"

#define CSIS_TRACE(...)
#define CSIS_TRACE_CRIT(...)
#define CSIS_TRACE_ARRAY(...)

/**
 * @addtogroup Coordinate_Set_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_csis
 * @{
 */

/**
 * \brief Generate PSRI valiue using the SIRK of the coordinated set.
 * \details PSRI value will be used during advertisement procedure to inform the clients regarding the specific set.
 *
 * @param[in]   sirk           SIRK value to be used for the specific coordinated set
 * @return  PSRI           PSRI value generated
 */
PSRI* wiced_bt_ga_csis_generate_psri(SIRK* sirk);

/**
 * \brief Set PSRI adv data.
 * \details Application use this api to advertise the set members coordinate set.
 *
 * @param[in]   psri           psri value to be set in the advertisement
 * @param[in]   adv_elem       memory in which the advertisement data has to be filled
 */
void wiced_bt_ga_csis_get_adv_data(PSRI* psri, wiced_bt_ble_advert_elem_t* adv_elem);

/**
 * \brief Set Lock timeout in seconds
 * \details Higher layer profile can set timeout value in seconds, it is by default set to 60 seconds.
 *
 * @param[in]   p_service    instance of the coordinated set identification service
 * @param[in]   timeout_in_sec timeout  value in seconds
 * @return  WICED_TRUE is operation is successful otherwise WICED_FALSE.
 */
void wiced_bt_ga_csis_set_lock_timeout_value(gatt_intf_service_object_t* p_service, uint8_t timeout_in_sec);

/**
 * \brief wiced_bt_ga_csis_is_operation_allowed
 * \details Other profile should invoke this API to check whether operation is allowed or not for given connection id.
 *
  * @param[in]   conn_id Connection id
 * @return  WICED_TRUE is operation is allowed otherwise WICED_FALSE.
 */
wiced_bool_t wiced_bt_ga_csis_is_operation_allowed(uint16_t conn_id);

/**
* Initialize the CSIS service_type/profile
* @param[in] p_cfg : Generic Audio configuration
*/
wiced_result_t wiced_bt_ga_csis_init(ga_cfg_t* p_cfg);

/**@} wiced_bt_ga_csis */
/**@} Coordinate_Set_APIs */
#ifdef __cplusplus
}
#endif

