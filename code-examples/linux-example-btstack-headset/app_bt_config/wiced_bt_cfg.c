/*
* Copyright 2020-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
/******************************************************************************
* File Name: wiced_bt_cfg.c
*
* Description: This is the source file for Bluetooth and BLE configurations.
*
* Related Document: See README.md
*
*******************************************************************************/

/*******************************************************************************
*                                   INCLUDES
*******************************************************************************/
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"

/*******************************************************************************
*                                   MACROS
*******************************************************************************/
/* Silicon generated 'Company assigned' part of device address */
#define CY_BT_SILICON_DEVICE_ADDRESS_EN                       ( 0 )

/* Appearance */
#define CY_BT_APPEARANCE                                      ( 0 )

/* TX Power Level */
#define CY_BT_TX_POWER                                        ( 0 )

/* Interval of random address refreshing */
#define CY_BT_RPA_TIMEOUT                                     ( 0 )

/* Maximum attribute length */
#define CY_BT_MAX_ATTR_LEN                                    ( 512 )
/* Maximum attribute MTU size */
#define CY_BT_MTU_SIZE                                        ( 512 )

/* Maximum connections */
#define CY_BT_SERVER_MAX_LINKS                                ( 1 )
#define CY_BT_CLIENT_MAX_LINKS                                ( 0 )

/* BLE white list size */
#define CY_BT_WHITE_LIST_SIZE                                 ( 0 )

/* L2CAP configuration */
#define CY_BT_L2CAP_MAX_LE_PSM                                ( 1 )
#define CY_BT_L2CAP_MAX_LE_CHANNELS                           ( 1 )
#define CY_BT_L2CAP_MTU_SIZE                                  ( 512 )

/* Security level */
#define CY_BT_SECURITY_LEVEL                                  BTM_SEC_BEST_EFFORT

/* Scan configuration */
#define CY_BT_SCAN_MODE                                       BTM_BLE_SCAN_MODE_ACTIVE

#define CY_BT_HIGH_DUTY_SCAN_INTERVAL                         ( 96 )
#define CY_BT_HIGH_DUTY_SCAN_WINDOW                           ( 48 )
#define CY_BT_HIGH_DUTY_SCAN_DURATION                         ( 0 )

#define CY_BT_LOW_DUTY_SCAN_INTERVAL                          ( 2048 )
#define CY_BT_LOW_DUTY_SCAN_WINDOW                            ( 1800 )
#define CY_BT_LOW_DUTY_SCAN_DURATION                          ( 0 )

#define CY_BT_HIGH_DUTY_CONN_SCAN_INTERVAL                    ( 96 )
#define CY_BT_HIGH_DUTY_CONN_SCAN_WINDOW                      ( 48 )
#define CY_BT_HIGH_DUTY_CONN_SCAN_DURATION                    ( 30 )

#define CY_BT_LOW_DUTY_CONN_SCAN_INTERVAL                     ( 2048 )
#define CY_BT_LOW_DUTY_CONN_SCAN_WINDOW                       ( 1800 )
#define CY_BT_LOW_DUTY_CONN_SCAN_DURATION                     ( 30 )

/* Connection configuration */
#define CY_BT_CONN_MIN_INTERVAL                               ( 6 )
#define CY_BT_CONN_MAX_INTERVAL                               ( 40 )
#define CY_BT_CONN_LATENCY                                    ( 0 )
#define CY_BT_CONN_SUPERVISION_TIMEOUT                        ( 1000 )

/* Advertisement settings */
#define CY_BT_CHANNEL_MAP                                     ( BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39 )

#define CY_BT_HIGH_DUTY_ADV_MIN_INTERVAL                      ( 48 )
#define CY_BT_HIGH_DUTY_ADV_MAX_INTERVAL                      ( 48 )
#define CY_BT_HIGH_DUTY_ADV_DURATION                          ( 60 )

#define CY_BT_LOW_DUTY_ADV_MIN_INTERVAL                       ( 2048 )
#define CY_BT_LOW_DUTY_ADV_MAX_INTERVAL                       ( 2048 )
#define CY_BT_LOW_DUTY_ADV_DURATION                           ( 30 )

#define CY_BT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL             ( 400 )
#define CY_BT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL             ( 800 )

#define CY_BT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL              ( 48 )
#define CY_BT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL              ( 48 )
#define CY_BT_LOW_DUTY_DIRECTED_ADV_DURATION                  ( 30 )

#define CY_BT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL              ( 160 )
#define CY_BT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL              ( 160 )
#define CY_BT_HIGH_DUTY_NONCONN_ADV_DURATION                  ( 30 )

#define CY_BT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL               ( 2048 )
#define CY_BT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL               ( 2048 )
#define CY_BT_LOW_DUTY_NONCONN_ADV_DURATION                   ( 30 )

/******************************************************************************
*        Structures and Enumerations
******************************************************************************/
/* Advertisement and scan response packets defines */
#define CY_BT_ADV_PACKET_DATA_SIZE                            1
/* BLE scan settings */
const wiced_bt_cfg_ble_scan_settings_t cy_bt_cfg_scan_settings =
{
    .scan_mode                       = CY_BT_SCAN_MODE,                                               /* BLE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE) */

    /* Advertisement scan configuration */
    .high_duty_scan_interval         = CY_BT_HIGH_DUTY_SCAN_INTERVAL,                                 /* High duty scan interval (in slots (1 slot = 0.625 ms)) */
    .high_duty_scan_window           = CY_BT_HIGH_DUTY_SCAN_WINDOW,                                   /* High duty scan window (in slots (1 slot = 0.625 ms)) */
    .high_duty_scan_duration         = CY_BT_HIGH_DUTY_SCAN_DURATION,                                 /* High duty scan duration in seconds (0 for infinite) */

    .low_duty_scan_interval          = CY_BT_LOW_DUTY_SCAN_INTERVAL,                                  /* Low duty scan interval (in slots (1 slot = 0.625 ms)) */
    .low_duty_scan_window            = CY_BT_LOW_DUTY_SCAN_WINDOW,                                    /* Low duty scan window (in slots (1 slot = 0.625 ms)) */
    .low_duty_scan_duration          = CY_BT_LOW_DUTY_SCAN_DURATION,                                  /* Low duty scan duration in seconds (0 for infinite) */

    /* Connection scan configuration */
    .high_duty_conn_scan_interval    = CY_BT_HIGH_DUTY_CONN_SCAN_INTERVAL,                            /* High duty cycle connection scan interval (in slots (1 slot = 0.625 ms)) */
    .high_duty_conn_scan_window      = CY_BT_HIGH_DUTY_CONN_SCAN_WINDOW,                              /* High duty cycle connection scan window (in slots (1 slot = 0.625 ms)) */
    .high_duty_conn_duration         = CY_BT_HIGH_DUTY_CONN_SCAN_DURATION,                            /* High duty cycle connection duration in seconds (0 for infinite) */

    .low_duty_conn_scan_interval     = CY_BT_LOW_DUTY_CONN_SCAN_INTERVAL,                             /* Low duty cycle connection scan interval (in slots (1 slot = 0.625 ms)) */
    .low_duty_conn_scan_window       = CY_BT_LOW_DUTY_CONN_SCAN_WINDOW,                               /* Low duty cycle connection scan window (in slots (1 slot = 0.625 ms)) */
    .low_duty_conn_duration          = CY_BT_LOW_DUTY_CONN_SCAN_DURATION,                             /* Low duty cycle connection duration in seconds (0 for infinite) */

    /* Connection configuration */
    .conn_min_interval               = CY_BT_CONN_MIN_INTERVAL,                                       /* Minimum connection interval (in slots (1 slot = 1.25 ms)) */
    .conn_max_interval               = CY_BT_CONN_MAX_INTERVAL,                                       /* Maximum connection interval (in slots (1 slot = 1.25 ms)) */
    .conn_latency                    = CY_BT_CONN_LATENCY,                                            /* Connection latency */
    .conn_supervision_timeout        = CY_BT_CONN_SUPERVISION_TIMEOUT,                                /* Connection link supervision timeout (in 10 ms) */
};

/* BLE advertisement settings */
const wiced_bt_cfg_ble_advert_settings_t cy_bt_cfg_adv_settings =
{
    .channel_map                     = CY_BT_CHANNEL_MAP,                                             /* Advertising channel map (mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39) */

    .high_duty_min_interval          = CY_BT_HIGH_DUTY_ADV_MIN_INTERVAL,                              /* High duty undirected connectable minimum advertising interval (in slots (1 slot = 0.625 ms)) */
    .high_duty_max_interval          = CY_BT_HIGH_DUTY_ADV_MAX_INTERVAL,                              /* High duty undirected connectable maximum advertising interval (in slots (1 slot = 0.625 ms)) */
    .high_duty_duration              = CY_BT_HIGH_DUTY_ADV_DURATION,                                  /* High duty undirected connectable advertising duration in seconds (0 for infinite) */

    .low_duty_min_interval           = CY_BT_LOW_DUTY_ADV_MIN_INTERVAL,                               /* Low duty undirected connectable minimum advertising interval (in slots (1 slot = 0.625 ms)) */
    .low_duty_max_interval           = CY_BT_LOW_DUTY_ADV_MAX_INTERVAL,                               /* Low duty undirected connectable maximum advertising interval (in slots (1 slot = 0.625 ms)) */
    .low_duty_duration               = CY_BT_LOW_DUTY_ADV_DURATION,                                   /* Low duty undirected connectable advertising duration in seconds (0 for infinite) */

    .high_duty_directed_min_interval = CY_BT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,                     /* High duty directed connectable minimum advertising interval (in slots (1 slot = 0.625 ms)) */
    .high_duty_directed_max_interval = CY_BT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,                     /* High duty directed connectable maximum advertising interval (in slots (1 slot = 0.625 ms)) */

    .low_duty_directed_min_interval  = CY_BT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,                      /* Low duty directed connectable minimum advertising interval (in slots (1 slot = 0.625 ms)) */
    .low_duty_directed_max_interval  = CY_BT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,                      /* Low duty directed connectable maximum advertising interval (in slots (1 slot = 0.625 ms)) */
    .low_duty_directed_duration      = CY_BT_LOW_DUTY_DIRECTED_ADV_DURATION,                          /* Low duty directed connectable advertising duration in seconds (0 for infinite) */

    .high_duty_nonconn_min_interval  = CY_BT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,                      /* High duty non-connectable minimum advertising interval (in slots (1 slot = 0.625 ms)) */
    .high_duty_nonconn_max_interval  = CY_BT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,                      /* High duty non-connectable maximum advertising interval (in slots (1 slot = 0.625 ms)) */
    .high_duty_nonconn_duration      = CY_BT_HIGH_DUTY_NONCONN_ADV_DURATION,                          /* High duty non-connectable advertising duration in seconds (0 for infinite) */

    .low_duty_nonconn_min_interval   = CY_BT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,                       /* Low duty non-connectable minimum advertising interval (in slots (1 slot = 0.625 ms)) */
    .low_duty_nonconn_max_interval   = CY_BT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,                       /* Low duty non-connectable maximum advertising interval (in slots (1 slot = 0.625 ms)) */
    .low_duty_nonconn_duration       = CY_BT_LOW_DUTY_NONCONN_ADV_DURATION                            /* Low duty non-connectable advertising duration in seconds (0 for infinite) */
};

/* BLE configuration settings */
const wiced_bt_cfg_ble_t cy_bt_cfg_ble = {
    .ble_max_simultaneous_links      = (CY_BT_CLIENT_MAX_LINKS + CY_BT_SERVER_MAX_LINKS),             /* Max number for simultaneous connections for a layer, profile, protocol */
    .ble_max_rx_pdu_size             = CY_BT_L2CAP_MTU_SIZE,                                          /* Maximum size allowed for any received L2CAP PDU
                                                                                                        * Minimum value - 65 (to support SM)
                                                                                                        * Maximum GATT MTU over legacy bearers shall be set to <= this value
                                                                                                        * Maximum MPS for EATT channels shall be set to <= this value */
    .appearance                      = CY_BT_APPEARANCE,                                              /* GATT appearance (see gatt_appearance_e) */
    .rpa_refresh_timeout             = CY_BT_RPA_TIMEOUT,                                             /* Interval of  random address refreshing - secs */
    .host_addr_resolution_db_size    = 5,                                                             /* LE Address Resolution DB settings - effective only for pre 4.2 controller */
    .p_ble_scan_cfg                  = &cy_bt_cfg_scan_settings,                                      /* BLE scan settings */
    .p_ble_advert_cfg                = &cy_bt_cfg_adv_settings,                                       /* BLE advertisement settings */
    .default_ble_power_level         = CY_BT_TX_POWER,                                                /* Default LE power level, Refer lm_TxPwrTable table for the power range */
};

/* GATT settings */
const wiced_bt_cfg_gatt_t cy_bt_cfg_gatt = {
    .max_db_service_modules          = 0,                                                             /* Maximum number of service modules in the DB */
    .max_eatt_bearers                = 0,                                                             /* Maximum number of allowed gatt bearers */
};

/* Application-managed L2CAP protocol configuration */
const wiced_bt_cfg_l2cap_application_t cy_bt_cfg_l2cap =
{
    .max_app_l2cap_psms              = CY_BT_L2CAP_MAX_LE_PSM,                                        /* Maximum number of application-managed PSMs */
    .max_app_l2cap_channels          = CY_BT_L2CAP_MAX_LE_CHANNELS,                                   /* Maximum number of application-managed channels */
    .max_app_l2cap_le_fixed_channels = 0,                                                             /* Maximum number of application-managed fixed channels supported */
};
