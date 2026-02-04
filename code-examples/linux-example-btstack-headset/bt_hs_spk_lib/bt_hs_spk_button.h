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

#include "wiced_bt_trace.h"
#include "wiced_button_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define BT_HS_SPK_CHECK_RESULT(expr)   result = (wiced_result_t)(expr);\
                                     if (result != WICED_SUCCESS)\
                                     {WICED_BT_TRACE("Error@%s(%d)\n", __func__, __LINE__);\
                                      goto _exit;\
                                     }\

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/
/*App assigned button roles*/
typedef enum
{
    /*
     * PLAY_PAUSE_BUTTON: Discovery, Play, Pause, Reject
     */
    PLAY_PAUSE_BUTTON,
    /*
     * VOLUME_UP_NEXT_TRACK_BUTTON: Volume Up, Next Track
     */
    VOLUME_UP_NEXT_TRACK_BUTTON,
    /*
     * VOLUME_DOWN_PREVIOUS_TRACK_BUTTON: Volume Down, Previous Track
     */
    VOLUME_DOWN_PREVIOUS_TRACK_BUTTON,
    /*
     * VOICE_REC_BUTTON: Voice Recognition
     */
    VOICE_REC_BUTTON,
} application_button_t;

typedef enum
{
    NO_ACTION                           = 0x00,
    ACTION_MULTI_FUNCTION_SHORT_RELEASE = 0x01,
    ACTION_MULTI_FUNCTION_LONG_RELEASE  = 0x02,
    ACTION_PAUSE_PLAY                   = 0x03,
    ACTION_STOP                         = 0x04,
    ACTION_FORWARD                      = 0x05,
    ACTION_BACKWARD                     = 0x06,
    ACTION_FAST_FORWARD_HELD            = 0x07,
    ACTION_FAST_FORWARD_RELEASE         = 0x08,
    ACTION_FAST_REWIND_HELD             = 0x09,
    ACTION_FAST_REWIND_RELEASE          = 0x0a,
    ACTION_VOLUME_UP                    = 0x0b,
    ACTION_VOLUME_DOWN                  = 0x0c,
    ACTION_BT_DISCOVERABLE              = 0x0d,
    ACTION_BT_CALL_REJECT               = ACTION_BT_DISCOVERABLE,
    ACTION_VOICE_RECOGNITION            = 0x0e,
    ACTION_TRANSPORT_DETECT_ON          = 0x0f,
    ACTION_BT_AUTO_DISCOVERABLE         = 0x10,
    ACTION_BT_REDIAL_LAST_CALL          = 0x11,
    ACTION_BT_ENCHANCED_VOICE_REC       = 0x12,
} app_service_action_t;

/******************************************************
 *
 *
 *                 Type Definitions
 ******************************************************/
typedef wiced_result_t (*WICED_APP_BUTTON_HANDLER)( app_service_action_t action );
typedef wiced_bool_t (BT_BS_SPK_BUTTON_HANDLER)(platform_button_t button, button_manager_event_t event, button_manager_button_state_t state, uint32_t repeat);
/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    app_service_action_t            action;
    application_button_t            button;
    button_manager_event_t          event;
    button_manager_button_state_t   state;
} bt_hs_spk_button_action_t;

typedef struct
{
    bt_hs_spk_button_action_t   *p_action;
    uint32_t                    number_of_actions;
} bt_hs_spk_button_action_config_t;

typedef struct bt_hs_spk_button_config
{
    button_manager_t                        *p_manager;
    wiced_button_manager_configuration_t    *p_configuration;
    button_manager_button_t                 *p_app_buttons;
    uint32_t                                number_of_buttons;

    /*
     * Pre-handler: If the p_pre_handler is assigned by the user application, the Pre-handler will
     *              be executed first and the default button handler (assigned in the
     *              wiced_button_manager_configuration_t) will be executed if the
     *              return value of the Pre-handler is set to WICED_TRUE;
     *              This filed is valid only when the event_handler filed in the
     *              p_configuration is set to NULL.
     */
    BT_BS_SPK_BUTTON_HANDLER                *p_pre_handler;
    bt_hs_spk_button_action_config_t        button_action_config;
} bt_hs_spk_button_config_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t service_bt_discovery(void);
wiced_result_t service_bt_auto_discovery(void);
void bt_audio_set_connection_state(wiced_bool_t state, wiced_bt_transport_t transport);

/**
 * bt_hs_spk_button_event_emulator
 *
 * Emulate the button event.
 *
 * @param[in]   button
 * @param[in]   event
 * @param[in]   state
 * @param[in]   repeat
 */
void bt_hs_spk_button_event_emulator(platform_button_t button, button_manager_event_t event, button_manager_button_state_t state, uint32_t repeat);

/**
 * bt_hs_spk_button_set_discovery
 *
 * Emulate the button event.
 *
 * @param[in]   enable
 */
void bt_hs_spk_button_set_discovery(wiced_bool_t enable);

/**
 * Run the specified app service action.
 *
 * @param[in] action : The action to be run.
 */
void bt_hs_spk_app_service_action_run(app_service_action_t action);

/**
 * bt_hs_spk_button_get_remain_bt_service_timer
 *
 * Get remain service timer for lrac switch transferred data
 *
 * @param[out] : The remain value of bt_service_timer
 */
uint16_t bt_hs_spk_button_get_remain_bt_service_timer(void);

/**
 * bt_hs_spk_button_lrac_switch_restore_visibility
 *
 * Restore dissoverable and connectable by application after ps-switch
 *
 * @param[in]   dissoverable
 * @param[in]   connectable
 * @param[in]   remain_time
 */
void bt_hs_spk_button_lrac_switch_restore_visibility(wiced_bool_t dissoverable, wiced_bool_t connectable, uint16_t remain_time);

#ifdef __cplusplus
} /* extern "C" */
#endif
