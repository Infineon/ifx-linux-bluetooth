/*
 * (c) 2016-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
 */
/** @file
*
* Google Fast Pair Service header file
*
* This file provides definitions and function prototypes for Google Fast Pair Service (GFPS)
*
*/
#pragma once

#include <stdint.h>
#include "wiced_bt_ble.h"
#include "wiced.h"
#include "wiced_bt_gatt.h"

/* Definition used for LE Advertisement Data. */
/** 16-bit UUID used for Google Fast Pair Service */
#define WICED_BT_GFPS_UUID16                            0xFE2C

/* definitions for fast pair service characteristic */
#define WICED_BT_GFPS_UUID_CHARACTERISTIC_KEY_PAIRING   0x1234
#define WICED_BT_GFPS_UUID_CHARACTERISTIC_PASSKEY       0x1235
#define WICED_BT_GFPS_UUID_CHARACTERISTIC_ACCOUNT_KEY   0x1236

/** Length of Anti-Spoofing Public Key */
#define WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PUBLIC      64

/** Length of Anti-Spoofing Private Key */
#define WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PRIVATE     32

/** Minimum Account Key List Size */
#define WICED_BT_GFPS_ACCOUNT_KEY_SIZE_MIN              5

/** Maximum Account Key List Size */
#define WICED_BT_GFPS_ACCOUNT_KEY_SIZE_MAX              12

/**
 * Google Fast Pair Service Provider module configuration
 */
typedef struct WICED_BT_GFPS_PROVIDER_CONF
{
    /** LE tx power level */
    int8_t                  ble_tx_pwr_level;

    /** Callback for GATT events. */
    wiced_bt_gatt_cback_t   *p_gatt_cb;

    /* Assigned handles for GATT attributes those are used for Google Fast Pair Service operation. */
    struct
    {
        uint16_t key_pairing_val;
        uint16_t key_pairing_cfg_desc;
        uint16_t passkey_val;
        uint16_t passkey_cfg_desc;
        uint16_t account_key_val;
    } gatt_db_handle;

    /** fast pair provider model id applied from Google. */
    uint32_t                model_id;

    /** Assigned Anti-Spoofing Key from Google */
    struct
    {
        uint8_t public[WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PUBLIC];
        uint8_t private[WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PRIVATE];
    } anti_spoofing_key;

    /**
     * Generate Account Key filter using one-byte random number
     *
     * WICED_TRUE: The Account Key filter will be generated using Random number.
     * WICED_FALSE: The Account Key filter will be generated using current LE Resolvable Private
     *              Address
     */
    wiced_bool_t            account_key_filter_generate_random;

    /** Number of Account Keys can be stored. */
    uint8_t                 account_key_list_size;

    /** NVRAM ID used for fast pair to store Account Key list */
    uint16_t                account_key_list_nvram_id;

    /** Advertisement data appended to the fast pair specific advertisement data. */
    struct
    {
        uint8_t                     elem_num;   /** element count */
        wiced_bt_ble_advert_elem_t  *p_elem;    /** pointer to the element(s). */
    } appended_adv_data;
} wiced_bt_gfps_provider_conf_t;

/** Account Key container */
typedef struct WICED_BT_GFPS_ACCOUNT_KEY
{
    wiced_bt_link_key_t key;
} wiced_bt_gfps_account_key_t;

/**
 * Acquire current LE Advertisement Data used for Google Fast Pair Service
 *
 * @param[out]  p_elem - address to the memory where stores current advertisement data content
 *
 * @return      total number of elements used now
 *
 * Note that the Advertisement Data used for Google Fast Pair Service is different between
 * discoverable and not discoverable modes.
 */
uint8_t wiced_bt_gfps_provider_advertisement_data_get(wiced_bt_ble_advert_elem_t **p_elem);

/**
 * Update LE advertisement data
 *
 * @param[in]   advert_type - advertisement data type
 * @param[in]   data_len - length of element data
 * @param[in]   p_data - pointer to the element data
 *
 * Note: This utility ONLY updates the advertisement data which is set by configuration while
 *       initializing the GFPS provider module.
 *       If user application wants to substitute all the appended advertisement data set by
 *       configuration while initializing the GFPS module, user shall use
 *       wiced_bt_gfps_provider_advertisement_data_appended_data_update().
 */
void wiced_bt_gfps_provider_advertisement_data_update(wiced_bt_ble_advert_type_t advert_type, uint16_t data_len, uint8_t *p_data);

/**
 * Update user appended LE advertisement data
 *
 * @param[in]   p_elem - pointer to the element(s)
 * @param[in]   elem_num - element count
 * @return      WICED_TRUE - Success
 *              WICED_FALSE - lack of memory
 */
wiced_bool_t wiced_bt_gfps_provider_advertisement_data_appended_data_update(wiced_bt_ble_advert_elem_t *p_elem, uint8_t elem_num);

/**
 * Start the Google Fast Pair Service Provider LE Advertisement
 *
 * @param[in]   discoverability
 *              0: Not Discoverability Advertisement
 *              1: Discoverable Advertisement
 */
void wiced_bt_gfps_provider_advertisement_start(uint8_t discoverability);

/**
 * Initialize the Google Fast Pair Service Provider module
 *
 * @param[in]   p_conf - configuration used for GFPS Provider Module
 * @return      WICED_TRUE: Success\n
 *              WICED_FALSE: Fail
 */
wiced_bool_t wiced_bt_gfps_provider_init(wiced_bt_gfps_provider_conf_t *p_conf);

/**
 * Set seeker's passkey
 * @param[in]   passkey - current passkey used for seeker
 *
 */
void wiced_bt_gfps_provider_seeker_passkey_set(uint32_t passkey);

/**
 * Set LE discoverability. Calling this utility effects the LE advertisement data content
 *
 * @param[in]   discoverable
 */
void wiced_bt_gfps_provider_discoverablility_set(wiced_bool_t discoverable);

/**
 * Acquire current GFPS pairing state
 *
 * @return  WICED_TRUE: GFPS is under pairing process
 */
wiced_bool_t wiced_bt_gfps_provider_pairing_state_get(void);

/**
 * Disable Google Fast Pair Servicer provider module.
 *
 * By calling this utility, the following capabilities will be terminated:
 *  1. LE advertisement
 *  2. Existent LE connection for Google Fast Pair Service
 *
 */
void wiced_bt_gfps_provider_disable(void);

/**
 * Enable Google Fast Pair Service provider module.
 *
 * The GFPS provider module is enabled in default by calling wiced_bt_gfps_provider_init()
 * utility to initiate the provider module.
 * That is, user do NOT need to call this utility again to enable the provider module unless
 * user called the wiced_bt_gfps_provider_disable() utility in previous operation to disable
 * the provider module for some reasons.
 *
 * By calling this utility, the provider module will:
 * 1. Update the LE advertisement data (including the discoverable and the not discoverable data).
 * 2. Start the LE advertisement again
 *
 *
 * Note:
 * 1. The LE advertisement will be set to not-discoverable mode. User application shall
 *    use wiced_bt_gfps_provider_discoverability_set() to switch to discoverable mode if
 *    application wants to switch to discoverable mode after enabling.
 */
void wiced_bt_gfps_provider_enable(void);

/*
 * Get current account key list
 */
/**
 * Get current account key list
 *
 * @param[out]  p_data - pointer to the output data
 *
 * @return  WICED_FALSE: Fail to fill data
 *          WICED_TRUE: Success
 */
wiced_bool_t wiced_bt_gfps_provider_account_key_list_get(uint8_t *p_data);

/**
 * Update the GFPS Provider's Account Key list database
 *
 * @param   p_account_key_list - pointer to the account key list that shall be updated to the
 *                               database
 *
 * @return  WICED_FALSE: Fail to update data
 *          WICED_TRUE: Success
 */
wiced_bool_t wiced_bt_gfps_provider_account_key_list_update(wiced_bt_gfps_account_key_t *p_account_key_list);

/*
 * Get current GFPS provider's discoverability
 */
/**
 * Get current GFPS provider's discoverability
 *
 * @return current GFPS provider's discoverability
 */
wiced_bool_t wiced_bt_gfps_provider_discoverability_get(void);
