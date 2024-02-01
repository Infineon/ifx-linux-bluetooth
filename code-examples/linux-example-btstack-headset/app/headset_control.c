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

/** @file
 *
 * Headset Sample Application for the Audio Shield platform.
 *
 * The sample app demonstrates Bluetooth A2DP sink, HFP and AVRCP Controller (and Target for absolute volume control).
 *
 * Features demonstrated
 *  - A2DP Sink and AVRCP Controller (Target for absolute volume)
 *  - Handsfree Device
 *  - GATT
 *  - SDP and GATT descriptor/attribute configuration
 *  - This app is targeted for the Audio Shield platform
 *  - This App doesn't support HCI UART for logging, PUART is supported.
 *  - HCI Client Control is not supported.
 *
 * Setting up Connection
 * 1. press and hold the SW15 on BT board for at least 2 seconds.
 * 2. This will set device in discovery mode(A2DP,HFP and BLE) and LED will start blinking.
 * 3. Scan for 'headsetpro' device on the peer source device, and pair with the headsetpro.
 * 4. Once connected LED will stop blinking and turns on.
 * 5. If no connection is established within 30sec,LED will turn off and device is not be discoverable,repeat instructions from step 1 to start again.
 *
 * A2DP Play back
 * 1. Start music play back from peer device, you should be able to hear music from headphones(use J27 headphone jack on Audio board)
 * 2. You can control play back and volume from peer device (Play, Pause, Stop) controls.
 *
 * AVRCP
 * 1. We can use buttons connected to the BT EVAL board for AVRCP control
 * 2. SW15 - Discoverable/Play/Pause    - Long press this button to enter discoverable mode. Click the button to Play/Pause the music play back.
 * 3. SW16 -                            - No function
 * 4. SW17 - Volume Up/Forward          - Click this button to increase volume or long press the button to forward
 * 5. SW18 - Volume Down/Backward       - Click this button to decrease volume or long press the button to backward
 *                                      (There are 16 volume steps).
 * 6. SW19 - Voice Recognition          - Long press to voice control
 *
 * Hands-free
 * 1. Make a phone call to the peer device.
 * 2. In case of in-band ring mode is supported from peer device, you will hear the set ring tone
 * 3. In case of out-of-band ring tone, no tone will be heard on headset.
 * 4. SW15  is used as multi-function button to accept,hang-up or reject call.
 * 5. Long press SW15 to reject the incoming call.
 * 6. Click SW15 to accept the call or hang-up the active call.
 * 7. If the call is on hold click SW15 to hang-up the call.
 * 8. Every click of SW17(Volume Up) button will increase the volume
 * 9. Every click of SW18(Volume down) button will decrease the volume
 *
 * BLE
 *  - To connect Ble device: set bt headset in discovery mode by long press of SW15 button
 *    search for 'headsetpro' device in peer side phone app (Ex:BLEScanner for Android and LightBlue for iOS) and connect.
 *  - From the peer side app you should be able to do GATT read/write of the elements listed.
 */



#include "headset_control.h"

#include "wiced_bt_stack.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_memory.h"
#include "wiced_app_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_dev.h"

#include "bt_hs_spk_handsfree.h"
#include "bt_hs_spk_control.h"
#include "wiced_bt_a2dp_sink.h"
#include "headset_nvram.h"
#include "platform_audio_interface.h"
#include "headset_control_le.h"
#include "log.h"
#include "app_bt_utils.h"
#ifdef FASTPAIR_ENABLE
#include "wiced_bt_gfps.h"
#endif
#include "bt_hs_spk_audio.h"
#include "log.h"
/*****************************************************************************
**  Constants
*****************************************************************************/
#define SECURITY_LOCAL_KEY_DATA_LEN        16
#define WICED_PIN_CODE_LEN                  4
/*****************************************************************************
**  Structures
*****************************************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t btheadset_control_management_callback( wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data );

extern uint32_t wiced_hal_wrapper_read_nvram(uint16_t config_item_id, uint32_t len, uint8_t *buf);
extern uint32_t wiced_hal_wrapper_write_nvram(uint16_t config_item_id, uint32_t len, uint8_t *buf);
extern void bt_audio_sink_data_cb(uint8_t* p_rx_media, uint32_t media_len);
void hci_control_send_pairing_completed_evt(uint8_t status, wiced_bt_device_address_t bdaddr);
extern uint8_t config_VS_Delete(uint16_t config_item_id);

extern wiced_result_t wiced_transport_send_data(uint16_t type, uint8_t* p_data, uint16_t data_size);

static void headset_control_local_irk_update(uint8_t *p_key);

/******************************************************
 *               Variables Definitions
 ******************************************************/
extern wiced_bt_a2dp_config_data_t  bt_audio_config;
extern uint8_t bt_avrc_ct_supported_events[];

extern const uint8_t btheadset_sdp_db[];

/* Local Identify Resolving Key. */
typedef struct
{
    wiced_bt_local_identity_keys_t  local_irk;
    wiced_result_t                  result;
} headset_control_local_irk_info_t;

static headset_control_local_irk_info_t local_irk_info = {0};
/* External variables */

uint8_t rm_deviceBDAddr[BD_ADDR_LEN] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

extern const uint8_t pincode[WICED_PIN_CODE_LEN];
/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Restore local Identity Resolving Key
 */
static void headset_control_local_irk_restore(void)
{
    uint16_t nb_bytes;
    nb_bytes = wiced_hal_read_nvram(HEADSET_NVRAM_ID_LOCAL_IRK,
                                    SECURITY_LOCAL_KEY_DATA_LEN,
				    local_irk_info.local_irk.id_keys.irk,
                                    &local_irk_info.result);
    TRACE_LOG("headset_control_local_irk_restore (result: %d, nb_bytes: %d)",
                   local_irk_info.result,
                   nb_bytes);
}

/*
 * Update local Identity Resolving Key
 */
static void headset_control_local_irk_update(uint8_t *p_key)
{
    uint16_t nb_bytes;
    wiced_result_t result = WICED_BT_ERROR;

    /* Check if the IRK shall be updated. */
    if (memcmp((void *) p_key,
               (void *) local_irk_info.local_irk.id_keys.irk,
               SECURITY_LOCAL_KEY_DATA_LEN) != 0)
    {
        nb_bytes = wiced_hal_wrapper_write_nvram(HEADSET_NVRAM_ID_LOCAL_IRK,
                                         SECURITY_LOCAL_KEY_DATA_LEN,
                                         p_key);

        
        if ((nb_bytes == SECURITY_LOCAL_KEY_DATA_LEN))
        {
            result = WICED_BT_SUCCESS;
            memcpy((void *) local_irk_info.local_irk.id_keys.irk,
                   (void *) p_key,
                   SECURITY_LOCAL_KEY_DATA_LEN);

            local_irk_info.result = result;
        }
        TRACE_LOG("Update local IRK (result: %d, nb_bytes: %d)",
                       result,
                       nb_bytes);

    }
}

/*******************************************************************************
* Function Name: btheadset_control_init
********************************************************************************
* Summary:
*   initial btstack, run at application start
*
* Parameters:
*   void
*
* Return:
*   none
*
*******************************************************************************/
void btheadset_control_init( void )
{
    wiced_result_t ret = WICED_BT_ERROR;
    TRACE_LOG( "#########################" );
    TRACE_LOG( "#  headset APP START    #" );
    TRACE_LOG( "#########################" );

    ret = wiced_bt_stack_init(btheadset_control_management_callback, &wiced_bt_cfg_settings);
    if( ret != WICED_BT_SUCCESS )
    {
        TRACE_ERR("wiced_bt_stack_init returns error: %d\n", ret);
        return;
    }

    /* Restore local Identify Resolving Key (IRK) for BLE Private Resolvable Address. */
    headset_control_local_irk_restore();
}

/*******************************************************************************
* Function Name: btheadset_post_bt_init
********************************************************************************
* Summary:
*   initial profile feature, run when stack initial is done (BTM_ENABLED_EVT)
*
* Parameters:
*   void
*
* Return:
*   wiced_result_t: result
*
*******************************************************************************/
wiced_result_t btheadset_post_bt_init(void)
{
    wiced_bool_t ret = WICED_FALSE;
    bt_hs_spk_control_config_t config = {0};
    bt_hs_spk_eir_config_t eir = {0};

    eir.p_dev_name              = (char *) wiced_bt_cfg_settings.device_name;
    eir.default_uuid_included   = WICED_TRUE;

    if(WICED_SUCCESS != bt_hs_spk_write_eir(&eir))
    {
        TRACE_WNG("Write EIR Failed");
    }

    ret = wiced_bt_sdp_db_init( ( uint8_t * )btheadset_sdp_db, wiced_app_cfg_sdp_record_get_size());
    if( ret != TRUE )
    {
        TRACE_ERR("Failed to Initialize SDP databse");
        return WICED_BT_ERROR;
    }

    config.conn_status_change_cb            = NULL;
#ifdef LOW_POWER_MEASURE_MODE
    config.discoverable_timeout             = 60;   /* 60 Sec */
#else
    config.discoverable_timeout             = 240;  /* 240 Sec */
#endif
    config.acl3mbpsPacketSupport            = WICED_TRUE;
    config.audio.a2dp.p_audio_config        = &bt_audio_config;
    config.audio.a2dp.p_pre_handler         = NULL;
    config.audio.a2dp.post_handler          = NULL;
    config.audio.avrc_ct.p_supported_events = bt_avrc_ct_supported_events;
    config.hfp.rfcomm.buffer_size           = 700;
    config.hfp.rfcomm.buffer_count          = 4;
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
    config.hfp.feature_mask                 = WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                              WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                              WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                              WICED_BT_HFP_HF_FEATURE_HF_INDICATORS | \
                                              WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION | \
                                              WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION | \
                                              WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_STATUS | \
                                              WICED_BT_HFP_HF_FEATURE_ESCO_S4_SETTINGS_SUPPORT;
#else
    config.hfp.feature_mask                 = WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                              WICED_BT_HFP_HF_FEATURE_ECNR | \
                                              WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                              WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                              WICED_BT_HFP_HF_FEATURE_HF_INDICATORS | \
                                              WICED_BT_HFP_HF_FEATURE_ENHANCED_VOICE_RECOGNITION | \
                                              WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION | \
                                              WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_STATUS | \
                                              WICED_BT_HFP_HF_FEATURE_ESCO_S4_SETTINGS_SUPPORT;
#endif

#ifdef TBD
    config.sleep_config.enable                  = WICED_TRUE;
    config.sleep_config.sleep_mode              = WICED_SLEEP_MODE_NO_TRANSPORT;
    config.sleep_config.host_wake_mode          = WICED_SLEEP_WAKE_ACTIVE_HIGH;
    config.sleep_config.device_wake_mode        = WICED_SLEEP_WAKE_ACTIVE_LOW;
    config.sleep_config.device_wake_source      = WICED_SLEEP_WAKE_SOURCE_GPIO;
    config.sleep_config.device_wake_gpio_num    = WICED_P02;
#endif
    config.nvram.link_key.id            = HEADSET_NVRAM_ID_LINK_KEYS;
    config.nvram.link_key.p_callback    = NULL;

    //Set HFP
    if(WICED_SUCCESS != bt_hs_spk_post_stack_init(&config))
    {
        TRACE_ERR("bt_audio_post_stack_init failed");
        return WICED_BT_ERROR;
    }

    /*Set audio sink*/
    bt_hs_spk_set_audio_sink(AM_UART);

#if (WICED_APP_LE_INCLUDED == TRUE)
    hci_control_le_enable();
#endif

#ifdef FASTPAIR_DISABLE
    wiced_bt_gfps_provider_disable();
#endif

    /*we will use the channel map provided by the phone*/
    ret = wiced_bt_dev_set_afh_channel_assessment(WICED_FALSE);
    TRACE_LOG("wiced_bt_dev_set_afh_channel_assessment status:%d",ret);
    if(ret != WICED_BT_SUCCESS)
    {
        return WICED_BT_ERROR;
    }
#ifdef AUTO_ELNA_SWITCH
    wiced_hal_rfm_auto_elna_switch(1, TX_PU, RX_PU);
#endif

    //A2DP datacallback
    wiced_bt_a2dp_sink_register_data_cback(a2dp_audio_data_cback);
    return WICED_SUCCESS;
}


void detete_paired_keys_from_nv(void)
{
    wiced_result_t result = WICED_SUCCESS;
    int i = 0;

    for (i = 0; i < 2; i++)
    {
        int nvram_id = 515 + i;
        result = (wiced_result_t)config_VS_Delete(nvram_id);
    }

    TRACE_LOG("result %d", result);
}


void configure_bluetooth()
{
    wiced_bt_device_address_t         local_bda;
    uint8_t *p = rm_deviceBDAddr;

    wiced_bt_dev_read_local_addr(local_bda);
    TRACE_LOG("Local bdaddr:"); 
    if ((p[0] | p[1] | p[2] | p[3] | p[4] | p[5]) != 0)
    {
        wiced_bt_set_local_bdaddr(p, BLE_ADDR_PUBLIC);
        print_bd_address(p);
    } else {
        print_bd_address(local_bda);
    }

    /* Disable pairing bydefault */
    wiced_bt_set_pairable_mode(FALSE, FALSE );
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t btheadset_control_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    uint8_t                             pairing_result;

    TRACE_LOG("btheadset bluetooth management callback event: 0x%x %s\n",event, get_bt_event_name(event));

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            if( p_event_data->enabled.status != WICED_BT_SUCCESS )
            {
                TRACE_LOG("arrived with failure\n");
            }
            else
            {
                configure_bluetooth();

                wiced_bt_dev_set_connectability(BTM_CONNECTABLE, BTM_DEFAULT_CONN_WINDOW,BTM_DEFAULT_CONN_INTERVAL);
                wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE, BTM_DEFAULT_DISC_WINDOW, BTM_DEFAULT_DISC_INTERVAL);
                
                result = btheadset_post_bt_init();
                if (result != WICED_SUCCESS){
                    TRACE_ERR("btheadset_post_bt_init Failed!");
                }
                else{
                    TRACE_LOG("btheadset_post_bt_init Done\n");
                }
            }
            break;
        case BTM_SECURITY_FAILED_EVT:
            TRACE_WNG("Security failed: %d / %d\n", p_event_data->security_failed.status, p_event_data->security_failed.hci_status);
            detete_paired_keys_from_nv();
            break;
        case BTM_DISABLED_EVT:
            //hci_control_send_device_error_evt( p_event_data->disabled.reason, 0 );
            break;

        case BTM_PIN_REQUEST_EVT:
            TRACE_LOG("BTM_PIN_REQUEST_EVT remote address:");
            print_bd_address(*p_event_data->pin_request.bd_addr);
            //wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr, WICED_BT_SUCCESS, WICED_PIN_CODE_LEN, (uint8_t *)&pincode[0]);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            // If this is just works pairing, accept. Otherwise send event to the MCU to confirm the same value.
            trace_log_with_bdaddr("BTM_USER_CONFIRMATION_REQUEST_EVT", p_event_data->user_confirmation_request.bd_addr);
            if (p_event_data->user_confirmation_request.just_works)
            {
                TRACE_LOG("BTM_USER_CONFIRMATION_REQUEST_EVT just_works");
                wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
            }
            else
            {
                TRACE_LOG("Need to send user_confirmation_request, Key %d", p_event_data->user_confirmation_request.numeric_value);
#ifdef FASTPAIR_ENABLE
                wiced_bt_gfps_provider_seeker_passkey_set(p_event_data->user_confirmation_request.numeric_value);
#endif
                wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
            }
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            print_bd_address(p_event_data->user_passkey_notification.bd_addr);
            TRACE_LOG("PassKey Notification. BDA, Key %d",p_event_data->user_passkey_notification.passkey );
            //hci_control_send_user_confirmation_request_evt(p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            trace_log_with_bdaddr("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);

#ifdef FASTPAIR_ENABLE
            if (wiced_bt_gfps_provider_pairing_state_get())
            {   // Google Fast Pair service Seeker triggers this pairing process.
                /* Set local capability to Display/YesNo to identify local device is not a
                 * man-in-middle device.
                 * Otherwise, the Google Fast Pair Service Seeker will terminate this pairing
                 * process. */
                p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            }
            else
#endif
            {
                p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            }

            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT:
            TRACE_LOG("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT (io_cap: 0x%02X)",
                           p_event_data->pairing_io_capabilities_br_edr_response.io_cap);
            print_bd_address(p_event_data->pairing_io_capabilities_br_edr_response.bd_addr);

#ifdef FASTPAIR_ENABLE
            if (wiced_bt_gfps_provider_pairing_state_get())
            {   // Google Fast Pair service Seeker triggers this pairing process.
                /* If the device capability is set to NoInput/NoOutput, end pairing, to avoid using
                 * Just Works pairing method. todo*/
                if (p_event_data->pairing_io_capabilities_br_edr_response.io_cap == BTM_IO_CAPABILITIES_NONE)
                {
                    TRACE_LOG("Terminate the pairing process\n");
                }
            }
#endif
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            /* Use the default security for BLE */
            trace_log_with_bdaddr("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT bda",
                    p_event_data->pairing_io_capabilities_ble_request.bd_addr);

            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_MITM_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 16;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_cmpl = &p_event_data->pairing_complete;
            if (p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
                TRACE_WNG("BREDR Pairing Result: %02x", pairing_result);
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
                TRACE_WNG("BLE Pairing Result: %02x", pairing_result);
            }
            hci_control_send_pairing_completed_evt(pairing_result, p_event_data->pairing_complete.bd_addr);
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;
 
            print_bd_address(p_encryption_status->bd_addr);
            TRACE_LOG( "Encryption Status:res:%d", p_encryption_status->result );

            bt_hs_spk_control_btm_event_handler_encryption_status(p_encryption_status);

            break;

        case BTM_SECURITY_REQUEST_EVT:
            TRACE_LOG( "Security Request Event, Pairing allowed %d", bt_hs_spk_control_pairability_get());
            TRACE_LOG( "Security Request Event, Pairing allowed %d", hci_control_cb.pairing_allowed );
            if ( bt_hs_spk_control_pairability_get() )
            {
                wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            }
            else
            {
                // Pairing not allowed, return error
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            TRACE_LOG("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT");
            result = bt_hs_spk_control_btm_event_handler_link_key(event, &p_event_data->paired_device_link_keys_update) ? WICED_BT_SUCCESS : WICED_BT_ERROR;
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            TRACE_LOG("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT");
            result = bt_hs_spk_control_btm_event_handler_link_key(event, &p_event_data->paired_device_link_keys_request) ? WICED_BT_SUCCESS : WICED_BT_ERROR;
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            TRACE_LOG("BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT");
            headset_control_local_irk_update(p_event_data->local_identity_keys_update.id_keys.irk);
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /*
             * Request to restore local identity keys from NVRAM
             * (requested during Bluetooth start up)
             * */
            TRACE_LOG("BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT (%d)", local_irk_info.result);
            if (local_irk_info.result == WICED_BT_SUCCESS)
            {
                memcpy((void *) p_event_data->local_identity_keys_request.id_keys.irk,
                       (void *) local_irk_info.local_irk.id_keys.irk,
                       SECURITY_LOCAL_KEY_DATA_LEN);
            }
            else
            {
                result = WICED_BT_NO_RESOURCES;
            }
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            TRACE_LOG("BTM_BLE_SCAN_STATE_CHANGED_EVT");
            //hci_control_le_scan_state_changed( p_event_data->ble_scan_state_changed );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            TRACE_LOG("BTM_BLE_ADVERT_STATE_CHANGED_EVT");
            hci_control_le_advert_state_changed( p_event_data->ble_advert_state_changed );
            break;

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            bt_hs_spk_control_btm_event_handler_power_management_status(&p_event_data->power_mgmt_notification);
            break;

        case BTM_SCO_CONNECTED_EVT:
        case BTM_SCO_DISCONNECTED_EVT:
        case BTM_SCO_CONNECTION_REQUEST_EVT:
        case BTM_SCO_CONNECTION_CHANGE_EVT:
            hf_sco_management_callback(event, p_event_data);
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            TRACE_LOG("BTM_BLE_CONNECTION_PARAM_UPDATE remote bd addr:");
            print_bd_address(p_event_data->ble_connection_param_update.bd_addr);
            TRACE_LOG("BTM_BLE_CONNECTION_PARAM_UPDATE (status: %d, conn_interval: %d, conn_latency: %d, supervision_timeout: %d)",
                           p_event_data->ble_connection_param_update.status,
                           p_event_data->ble_connection_param_update.conn_interval,
                           p_event_data->ble_connection_param_update.conn_latency,
                           p_event_data->ble_connection_param_update.supervision_timeout);
            break;
        case BTM_BLE_PHY_UPDATE_EVT:
            /* BLE PHY Update to 1M or 2M */
            TRACE_LOG("PHY config is updated as TX_PHY : %dM, RX_PHY : %d",
                    p_event_data->ble_phy_update_event.tx_phy,
                    p_event_data->ble_phy_update_event.rx_phy);
            break;
        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;
   }
    return result;
}

/*
 * headset_control_proc_rx_cmd_button
 *
 * Handle the received button event.
 *
 * The format of incoming button event:
 * Byte: |     0     |       1      |      2       |
 * Data: | BUTTON_ID | BUTTON_EVENT | BUTTON_STATE |
 *
 */
static void headset_control_proc_rx_cmd_button(uint8_t* p_data, uint32_t length)
{
    uint8_t button_id;
    uint8_t button_event;
    uint8_t button_state;

    /* Check data length. */
    if (length != sizeof(button_id) + sizeof(button_event) + sizeof(button_state))
    {
        return;
    }

    STREAM_TO_UINT8(button_id, p_data);
    STREAM_TO_UINT8(button_event, p_data);
    STREAM_TO_UINT8(button_state, p_data);

    /* Process this button event. */
    bt_hs_spk_button_event_emulator((platform_button_t)button_id,
        (button_manager_event_t)button_event,
        (button_manager_button_state_t)button_state,
        0);
}


/*
 * Handle received command over UART. Please refer to the WICED Smart Ready
 * Software User Manual (WICED-Smart-Ready-SWUM100-R) for details on the
 * HCI UART control protocol.
*/
uint32_t hci_control_proc_rx_cmd(uint8_t* p_data, uint32_t data_len)
{
    uint16_t op_code;
    uint16_t payload_len;
    //Expected minimum 4 byte as the wiced header
    if (p_data == NULL)
    {
        TRACE_LOG("invalid params");
         return WICED_BT_ERROR;
    }

    STREAM_TO_UINT16(op_code, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    /* Process the incoming command. */
    switch (op_code)
    {
    case HCI_CONTROL_HCI_AUDIO_COMMAND_BUTTON:
        headset_control_proc_rx_cmd_button(p_data, payload_len);
        break;
    default:
        break;
    }
	return WICED_BT_SUCCESS;
}

/*
 *  Send Pairing Completed event through UART
 */
void hci_control_send_pairing_completed_evt(uint8_t status, wiced_bt_device_address_t bdaddr)
{
    int i;

    uint8_t event_data[BD_ADDR_LEN + sizeof(uint8_t)];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = status;

    for (i = 0; i < BD_ADDR_LEN; i++)                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    WICED_BT_TRACE("pairing complete evt: %B as %B status %d", bdaddr, &event_data[1], status);

    wiced_transport_send_data(HCI_CONTROL_EVENT_PAIRING_COMPLETE, event_data, sizeof(event_data));
}


/*******************************************************************************
* Function Name: headset_get_statues
********************************************************************************
* Summary:
*   print current status for connect, sink, call active
*
* Parameters:
*   void
*
* Return:
*   none
*
*******************************************************************************/
void headset_get_statues(void){
    TRACE_LOG("Connect status:%d",bt_hs_spk_control_connection_status_check_be_edr(WICED_TRUE));
    TRACE_LOG("Handsfree call active:%d",bt_hs_spk_handsfree_is_call_active());
    TRACE_LOG("wiced_bt_avrc_playstate_t:%d",headset_get_playstate());
}

/*******************************************************************************
* Function Name: headset_get_br_connect
********************************************************************************
* Summary:
*   return the headset BR/EDR is connected or not
*
* Parameters:
*   void
*
* Return:
*   bool false: disconenct true: connected

*******************************************************************************/
bool headset_get_br_connect(void){
    bool res = bt_hs_spk_control_connection_status_check_be_edr(WICED_TRUE);
    TRACE_LOG("headset_get_br_connect:%d",res);
    return res;
}

/*******************************************************************************
* Function Name: headset_get_call_active
********************************************************************************
* Summary:
*   return the call in active or not
*
* Parameters:
*   void
*
* Return:
*   bool false: no active true: call active

*******************************************************************************/
bool headset_get_call_active(void){
    bool res = bt_hs_spk_handsfree_is_call_active();
    TRACE_LOG("bt_hs_spk_handsfree_is_call_active:%d",res);
    return res;
}

/*******************************************************************************
* Function Name: headset_get_playstate
********************************************************************************
* Summary:
*   return the headset sink play state
*
* Parameters:
*   void
*
* Return:
*   uint8_t return the sink play state, refer to wiced_bt_avrc_playstate_t
*    0x00    < Stopped 
*    0x01    < Playing
*    0x02    < Paused
*    0x03    < Fwd Seek
*    0x04    < Rev Seek
*    0xFF    < Error
*******************************************************************************/
uint8_t headset_get_playstate(void){
    uint8_t res = bt_hs_spk_audio_avrc_playstate_get();
    //TRACE_LOG("wiced_bt_avrc_playstate_t:%d",res);
    return res;
}