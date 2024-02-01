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

/*
 * @bt_hs_spk__button.c : Application's Button-list entries
 */

#include "bt_hs_spk_control.h"
#include "bt_hs_spk_button.h"
#include "bt_hs_spk_handsfree.h"
#include "wiced_button_manager.h"
#if defined(STACK_INSIDE_FREE_RTOS) && (STACK_INSIDE_FREE_RTOS == TRUE)
#include "cyabs_rtos.h"
#endif
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
#include "clock_timer.h"
#include "wiced_led_manager.h"
#include "wiced_transport.h"
#endif
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE) || WIN_EMULATOR
#include "wiced_timer.h"
#endif
/******************************************************
 *                      Macros
 ******************************************************/
#define BT_DISCOVERY_LEDBLINK_PERIOD (500)  /*500msec*/

#define BUTTON_EVENT_ID(app_button, event, state)      ( ((uint32_t)app_button << 24) + ((uint32_t)event<<1) + state )

#define TRANSPORT_DETECT_SECONDS    3       //unit: second


/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef wiced_result_t  (*button_event_function_t)(void);

typedef struct
{
    /* Logical action-enum */
    app_service_action_t    action;

    /* Primary routine which will be executed to service above action */
    button_event_function_t    primary;
    /* Secondary routine which need to be executed only when either primary routine has not been defined
     * or when Primary routines fails to service the action */
    button_event_function_t    secondary;
} button_action_function_table_t;

static wiced_bool_t bt_connected_le;

static wiced_bool_t b_auto_discovery = WICED_TRUE;
extern uint8_t vrec_enable;

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    BT_BS_SPK_BUTTON_HANDLER    *p_pre_handler;
    bt_hs_spk_button_action_t   *p_action;
    uint32_t                    number_of_actions;
} bt_hs_spk_button_cb_t;

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t service_multi_functions_short_release(void);
static wiced_result_t service_multi_functions_long_release(void);
static wiced_result_t service_media_backward(void);
static wiced_result_t service_media_forward(void);
static wiced_result_t service_media_fast_forward_held(void);
static wiced_result_t service_media_fast_forward_release(void);
static wiced_result_t service_media_fast_rewind_held(void);
static wiced_result_t service_media_fast_rewind_release(void);
static wiced_result_t service_pause_play(void);
static wiced_result_t service_media_stop(void);
static wiced_result_t service_volume_up(void);
static wiced_result_t service_volume_down(void);
static wiced_result_t service_bt_voice_command(void);
static wiced_result_t service_bt_redial_call(void);
static wiced_result_t service_transport_set_detect_on(void);
static wiced_result_t service_bt_enchanced_voice_command(void);

static void app_service_action_arbitrator( app_service_action_t action );
static void app_button_event_handler( const button_manager_button_t* button, button_manager_event_t event, button_manager_button_state_t state );

extern  wiced_result_t bt_audio_hfp_dial_last_call(void);

#if defined(STACK_INSIDE_FREE_RTOS) && (STACK_INSIDE_FREE_RTOS == TRUE)
static void bt_service_timer_cb(uint32_t arg);
#endif
#ifdef WIN_EMULATOR
static void bt_service_timer_cb(WICED_TIMER_PARAM_TYPE arg);
#endif // WIN_EMULATOR

/******************************************************
 *               Variable Definitions
 ******************************************************/
/*timer for button event timeout */
#if defined(STACK_INSIDE_BT_CTRL) && (STACK_INSIDE_BT_CTRL == TRUE) || WIN_EMULATOR || LINUX_PLATFORM
wiced_timer_t bt_service_timer;
#endif

#if defined(STACK_INSIDE_FREE_RTOS) && (STACK_INSIDE_FREE_RTOS == TRUE)
cy_timer_t bt_service_timer;
#endif
button_manager_button_t *buttons;

/* A table for deciding how a particular action will be handled...
 * Table must be in same order as app_service_action_t enum variables */
static const button_action_function_table_t action_table[] =
{
    { NO_ACTION,                                NULL,                                    NULL },
    { ACTION_MULTI_FUNCTION_SHORT_RELEASE,      service_multi_functions_short_release,   NULL },
    { ACTION_MULTI_FUNCTION_LONG_RELEASE,       service_multi_functions_long_release,    NULL },
    { ACTION_PAUSE_PLAY,                        service_pause_play,                      NULL },
    { ACTION_STOP,                              service_media_stop,                      NULL },
    { ACTION_FORWARD,                           service_media_forward,                   NULL },
    { ACTION_BACKWARD,                          service_media_backward,                  NULL },
    { ACTION_FAST_FORWARD_HELD,                 service_media_fast_forward_held,         NULL },
    { ACTION_FAST_FORWARD_RELEASE,              service_media_fast_forward_release,      NULL },
    { ACTION_FAST_REWIND_HELD,                  service_media_fast_rewind_held,          NULL },
    { ACTION_FAST_REWIND_RELEASE,               service_media_fast_rewind_release,       NULL },
    { ACTION_VOLUME_UP,                         service_volume_up,                       NULL },
    { ACTION_VOLUME_DOWN,                       service_volume_down,                     NULL },
    { ACTION_BT_DISCOVERABLE,                   service_bt_discovery,                    NULL },
    { ACTION_VOICE_RECOGNITION,                 service_bt_voice_command,                NULL },
    { ACTION_TRANSPORT_DETECT_ON,               service_transport_set_detect_on,         NULL },
    { ACTION_BT_AUTO_DISCOVERABLE,              service_bt_auto_discovery,               NULL },
    { ACTION_BT_REDIAL_LAST_CALL,               service_bt_redial_call,                  NULL },
    {ACTION_BT_ENCHANCED_VOICE_REC,             service_bt_enchanced_voice_command,      NULL },
};

static bt_hs_spk_button_cb_t bt_hs_spk_button_cb = {0};

/******************************************************
 *               Function Definitions
 ******************************************************/

static wiced_bool_t is_service_valid(wiced_app_service_t*  service)
{
    return ( (service != NULL && service->button_handler != NULL) ? WICED_TRUE: WICED_FALSE );
}

static wiced_result_t service_volume_up(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return result;
    }
    result = service->button_handler(ACTION_VOLUME_UP);

    return result;
}

static wiced_result_t service_volume_down(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_VOLUME_DOWN);
    return result;
}

static wiced_result_t service_media_backward(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_BACKWARD);
    return result;
}
#if defined(STACK_INSIDE_FREE_RTOS) && (STACK_INSIDE_FREE_RTOS == TRUE)
static void bt_service_timer_cb(uint32_t arg)
#endif
#if defined(WIN_EMULATOR)
static void bt_service_timer_cb(WICED_TIMER_PARAM_TYPE arg)
#endif
#if defined(WIN_EMULATOR) || (defined(STACK_INSIDE_FREE_RTOS) && (STACK_INSIDE_FREE_RTOS == TRUE))
{
    bt_hs_spk_button_set_discovery(WICED_FALSE);
}
#endif
void bt_hs_spk_button_set_discovery(wiced_bool_t enable)
{
    WICED_BT_TRACE("%s() %d\n", __func__, enable);
#if defined(STACK_INSIDE_FREE_RTOS) && (STACK_INSIDE_FREE_RTOS == TRUE)
    wiced_bool_t isrunning;
#endif

    if (enable)
    {
        if (bt_hs_spk_control_connection_status_check_be_edr(WICED_TRUE) && bt_connected_le)
            return;

#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
        bt_hs_spk_pm_disable();
#endif

        if(bt_hs_spk_control_connection_status_check_be_edr(WICED_TRUE) == WICED_FALSE)
        {
            bt_hs_spk_control_handle_set_visibility(WICED_TRUE,WICED_TRUE,BT_TRANSPORT_BR_EDR);
        }
        if(!bt_connected_le)
        {
            bt_hs_spk_control_handle_set_visibility(WICED_TRUE,WICED_TRUE,BT_TRANSPORT_LE);
        }
        bt_hs_spk_control_handle_set_pairability(WICED_TRUE);
// #ifndef PLATFORM_LED_DISABLED
//         wiced_led_manager_blink_led(PLATFORM_LED_1,BT_DISCOVERY_LEDBLINK_PERIOD,BT_DISCOVERY_LEDBLINK_PERIOD);
// #endif
// #if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE) || WIN_EMULATOR
//             /* stop the discovery timer if running */
//         if (wiced_is_timer_in_use(&bt_service_timer))
//         {
//             wiced_stop_timer(&bt_service_timer);
//         }
//         if (WICED_SUCCESS != wiced_start_timer( &bt_service_timer,bt_hs_spk_control_discoverable_timeout_get() ))
// 		WICED_BT_TRACE("%s timer start failed\n",__func__);
// #endif
// #if defined(STACK_INSIDE_FREE_RTOS) && (STACK_INSIDE_FREE_RTOS == TRUE)
//         uint32_t timeout = bt_hs_spk_control_discoverable_timeout_get(); //this returns seconds
//         if (WICED_SUCCESS != cy_rtos_start_timer( &bt_service_timer,(timeout*1000) ))
// 			WICED_BT_TRACE("%s timer start failed\n",__func__);
// #endif

    }
    else
    {
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE) || WIN_EMULATOR
        /* stop timer */
        if (wiced_is_timer_in_use(&bt_service_timer))
        {
            wiced_stop_timer(&bt_service_timer);
        }
#endif
#if defined(STACK_INSIDE_FREE_RTOS) && (STACK_INSIDE_FREE_RTOS == TRUE)
        /* stop timer */
        if ((!cy_rtos_is_running_timer(&bt_service_timer, (bool*)&isrunning)) && isrunning)
        {
            cy_rtos_stop_timer(&bt_service_timer);
        }
#endif

        WICED_BT_TRACE("%s:bt_hs_spk_control_handle_set_visibility(WICED_FALSE,WICED_FALSE,BT_TRANSPORT_BR_EDR)\n", __func__);
        bt_hs_spk_control_handle_set_visibility(WICED_FALSE,WICED_FALSE,BT_TRANSPORT_BR_EDR);
        WICED_BT_TRACE("%s:bt_hs_spk_control_handle_set_visibility(WICED_FALSE,WICED_FALSE,BT_TRANSPORT_LE)\n", __func__);
        bt_hs_spk_control_handle_set_visibility(WICED_FALSE,WICED_FALSE,BT_TRANSPORT_LE);
        WICED_BT_TRACE("%s:bt_hs_spk_control_handle_set_pairability(WICED_FALSE)\n", __func__);
        bt_hs_spk_control_handle_set_pairability(WICED_FALSE);
#ifndef PLATFORM_LED_DISABLED
        /*disable led blinking */
        wiced_led_manager_disable_led(PLATFORM_LED_1);
        if(bt_hs_spk_control_connection_status_check_be_edr(WICED_FALSE) || bt_connected_le) {
            /*turn led on */
            wiced_led_manager_enable_led(PLATFORM_LED_1);
        }
#endif
    }
}

wiced_result_t service_bt_discovery(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return result;
    }
    /*If we are in HFP service this button event is to reject incoming call*/
    if(service->active_service == SERVICE_BT_HFP)
    {
        result = service->button_handler(ACTION_BT_CALL_REJECT);
    }
    else
    {
        bt_hs_spk_button_set_discovery(WICED_TRUE);
        result = WICED_SUCCESS;
    }

    return result;
}

wiced_result_t service_bt_auto_discovery(void)
{
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return WICED_ERROR;
    }

    if(b_auto_discovery)
    {
        b_auto_discovery = WICED_FALSE;
    }
    else
    {
        b_auto_discovery = WICED_TRUE;
        bt_hs_spk_button_set_discovery(WICED_TRUE);
    }
    WICED_BT_TRACE("[%s]:b_auto_discovery = %d ", __func__, b_auto_discovery);
    return WICED_SUCCESS;

}

void bt_audio_set_connection_state(wiced_bool_t state, wiced_bt_transport_t transport)
{
    if (state == WICED_TRUE)
    {
        if (transport == BT_TRANSPORT_BR_EDR)
        {
            if (bt_hs_spk_control_connection_status_check_be_edr(WICED_TRUE) == WICED_TRUE)
            {
                /* stop timer */
                if (wiced_is_timer_in_use(&bt_service_timer))
                {
                    wiced_stop_timer(&bt_service_timer);
                }
                bt_hs_spk_control_handle_set_visibility(WICED_FALSE, WICED_FALSE, BT_TRANSPORT_BR_EDR);
            }
        }
        else
        {
            bt_connected_le = WICED_TRUE;
        }
    }
    else
    {
        if (transport == BT_TRANSPORT_BR_EDR)
        {
            WICED_BT_TRACE("%s b_auto_discovery = %d\n",__func__,b_auto_discovery);
            if(b_auto_discovery)
            {
               bt_hs_spk_button_set_discovery(WICED_TRUE);
            }

#ifdef LOW_POWER_MEASURE_MODE
            /* disable connectable later for low power test mode */
            if (WICED_SUCCESS != wiced_start_timer( &bt_service_timer,bt_hs_spk_control_discoverable_timeout_get() ))
                WICED_BT_TRACE("%s timer start failed\n",__func__);
#endif
        }
        else
        {
            bt_connected_le = WICED_FALSE;
        }
    }
}


static wiced_result_t service_media_forward(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_FORWARD);
    return result;
}

static wiced_result_t service_media_fast_forward_held(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_FAST_FORWARD_HELD);
    return result;
}

static wiced_result_t service_media_fast_forward_release(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_FAST_FORWARD_RELEASE);
    return result;
}

static wiced_result_t service_media_fast_rewind_held(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_FAST_REWIND_HELD);
    return result;
}

static wiced_result_t service_media_fast_rewind_release(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_FAST_REWIND_RELEASE);
    return result;
}

static wiced_result_t service_pause_play(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();

    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return WICED_UNSUPPORTED;
    }

    result = service->button_handler(ACTION_PAUSE_PLAY);

    return result;
}

static wiced_result_t service_media_stop(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return  result;
    }

    result = service->button_handler(ACTION_STOP);
    return result;
}

static wiced_result_t service_multi_functions_short_release(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_MULTI_FUNCTION_SHORT_RELEASE);
    return result;
}

static wiced_result_t service_multi_functions_long_release(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        WICED_BT_TRACE("%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_MULTI_FUNCTION_LONG_RELEASE);
    return result;
}

static wiced_result_t service_bt_voice_command(void)
{
    if(vrec_enable == FALSE)
    {
        vrec_enable = TRUE;
        return bt_hs_spk_handsfree_voice_recognition_activate();
    }
    else
    {
        vrec_enable = FALSE;
        return bt_hs_spk_handsfree_voice_recognition_deactivate();
    }
}

static wiced_result_t service_bt_enchanced_voice_command(void)
{

        return bt_hs_spk_handsfree_enhanced_voice_rec_cmd();
}


static wiced_result_t service_bt_redial_call(void)
{
    return bt_audio_hfp_dial_last_call();
}


static wiced_result_t service_transport_set_detect_on(void)
{
#ifdef APP_TRANSPORT_DETECT_ON
    wiced_result_t result = WICED_ERROR;

    result = wiced_transport_set_detect_on(TRANSPORT_DETECT_SECONDS);
    if ( result != WICED_SUCCESS )
    {
        WICED_BT_TRACE("%s(): detect on not support\n", __func__);
    }

    return result;
#else
    return WICED_ERROR;
#endif
}

static void app_service_action_arbitrator( app_service_action_t action )
{
    wiced_result_t result = WICED_ERROR;
    WICED_BT_TRACE("[%s] ",__func__);
    if ( action == NO_ACTION )
    {
        return;
    }

    if ( action_table[action].primary )
    {
        result = action_table[action].primary();
    }

    if ( result != WICED_SUCCESS && action_table[action].secondary )
    {
        result = action_table[action].secondary();
    }

    return;
}

static void bt_hs_spk_button_event_handler(platform_button_t button, button_manager_event_t event, button_manager_button_state_t state, uint32_t repeat)
{
    uint32_t button_event_id = BUTTON_EVENT_ID(button, event, state);
    uint32_t i;
    app_service_action_t new_action = NO_ACTION;
    bt_hs_spk_button_action_t *p_action;

    /* Pre-handler. */
    if (bt_hs_spk_button_cb.p_pre_handler)
    {
        if ((*bt_hs_spk_button_cb.p_pre_handler)(button, event, state, repeat) == WICED_FALSE)
        {
            return;
        }
    }

    /* Get the corresponding button action assigned by the user application. */
    for (i = 0 ; i < bt_hs_spk_button_cb.number_of_actions ; i++)
    {
        p_action = bt_hs_spk_button_cb.p_action + i;

        if (button_event_id == BUTTON_EVENT_ID(p_action->button, p_action->event, p_action->state))
        {
            new_action = p_action->action;
            break;
        }
    }

    /* Execute the corresponding action handler. */
    app_service_action_arbitrator(new_action);
}

/**
 *
 * Button events thread callback
 *
 * @param button
 * @param state
 * @return
 */
static void app_button_event_handler( const button_manager_button_t* button, button_manager_event_t event, button_manager_button_state_t state )
{
    WICED_BT_TRACE("[%s] :Button-%d [event: %x state:%x, repeat: %d]\n", __func__,button->configuration->button, (unsigned int) event, (unsigned int) state, button->repeat);

    bt_hs_spk_button_event_handler(button->configuration->button, event, state, button->repeat);
}

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
void bt_hs_spk_button_event_emulator(platform_button_t button, button_manager_event_t event, button_manager_button_state_t state, uint32_t repeat)
{
    bt_hs_spk_button_event_handler(button, event, state, repeat);
}

void bt_hs_spk_app_service_action_run(app_service_action_t action)
{
    app_service_action_arbitrator(action);
}

#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
uint16_t bt_hs_spk_button_get_remain_bt_service_timer(void)
{
    OSAPI_TIMER *timer = (OSAPI_TIMER *)&bt_service_timer;
    uint32_t r = clock_SystemTimeMicroseconds64();

    if (r < timer->target_time)
        r = timer->target_time - r;
    else
        r=0;

    /* return value unit from usec to sec */
    r = r / 1000000;
    if (r > bt_hs_spk_control_discoverable_timeout_get())
        r = 0;

    return (uint16_t) r;
}
#endif


/**
 * This function is for ps-switch only, to restore_visibility after ps-switch done.
 * It continues button discovery and re-connect, and then stop when timeout.
 */
#if defined(STACK_INSIDE_BT_CTRLR) && (STACK_INSIDE_BT_CTRLR == TRUE)
void bt_hs_spk_button_lrac_switch_restore_visibility(wiced_bool_t dissoverable, wiced_bool_t connectable, uint16_t remain_time)
{
    bt_hs_spk_control_handle_set_visibility(dissoverable, connectable, BT_TRANSPORT_BR_EDR);

    if (dissoverable == WICED_TRUE)
    {
#ifndef PLATFORM_LED_DISABLED
        wiced_led_manager_blink_led(PLATFORM_LED_1,BT_DISCOVERY_LEDBLINK_PERIOD,BT_DISCOVERY_LEDBLINK_PERIOD);
#endif
        if (WICED_SUCCESS != wiced_start_timer(&bt_service_timer, remain_time))
            WICED_BT_TRACE("%s timer start failed\n", __func__);
    }
#ifdef LOW_POWER_MEASURE_MODE
    else if (dissoverable == WICED_FALSE && connectable == WICED_TRUE)
    {
        if (WICED_SUCCESS != wiced_start_timer( &bt_service_timer, remain_time ))
            WICED_BT_TRACE("%s timer start failed\n",__func__);
    }
#endif
}
#endif
