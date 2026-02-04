/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __WICED_BT_GA_MICS_H__
#define __WICED_BT_GA_MICS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "gatt_interface.h"
#include "wiced_bt_ga_common.h"
#include "wiced_bt_ga_aics.h"

/**
 * @addtogroup Volume_And_Gain_Control_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_mics
 * @{
 * @brief MICS is declared on devices that can control the mute state of a microphone’s audio. Only one instance of MICS is allowed.
 */

/**
* @brief Included structure of MICS
*/
typedef struct {
    uint8_t index;                           /**< index of the included service viz, vocs or aics */
    union {
        wiced_bt_ga_aics_data_t* p_aics;        /**< aics data */
    };
} mics_included_t;

/**
* @brief MICS data structure
*/
typedef union {
    wiced_bt_ga_mute_val_t           mute_val;        /**< mute state */
    mics_included_t                  mics_included;   /**< volume included service data */
} wiced_bt_ga_mics_data_t;


/**
* @brief microphone State data
*/
typedef struct
{
	wiced_bt_ga_mute_val_t	mute_state; 					   /**< current mute state value of the peer*/
} wiced_bt_ga_microphone_state_data_t;

/**
* @brief microphone Control client status Data
*/
typedef union
{
	wiced_bt_ga_mute_val_t mute_state;					   /**< changed microphone setting value */
	uint8_t error_status;								   /**< error opcode in case of error event */
	wiced_bt_ga_aics_data_t aics_data;			           /**< AICS Status data */
} wiced_bt_ga_microphone_control_client_status_data_t;

/* MICS initialization data */
typedef struct
{
    uint8_t max_aics; /**< Number of AICS instances for each of the MICS instances */
} wiced_bt_ga_mics_init_data_t;

/**
 * @brief Initialize the MICS service/profile
 *
 * @param[in] num_instances : Number of instances
 * @param[in] p_ini : see #wiced_bt_ga_mics_init_data_t
 *
 * @result result of the init operation
 */
wiced_result_t wiced_bt_ga_mics_init(uint8_t num_instances, wiced_bt_ga_mics_init_data_t *p_ini);

/** Enable MICS server module */
void wiced_bt_ga_mics_enable_server(void);

/** Enable MICS client module */
void wiced_bt_ga_mics_enable_client(void);

#ifdef __cplusplus
}
#endif
/**@} wiced_bt_ga_mics */
/**@} Volume_And_Gain_Control_APIs */


#endif /* __WICED_BT_GA_MICS_H__ */
