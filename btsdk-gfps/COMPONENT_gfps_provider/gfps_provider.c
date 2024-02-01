/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
* Google Fast Pair Service Provider module
*
*
*/
#include <stdlib.h>
#include "wiced_bt_cfg.h"
#include "wiced_bt_gfps.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_gatt.h"
#include "wiced_hal_nvram.h"
#include "fastpair_sec_utils.h"
#include "wiced_bt_ble.h"
#include "wiced_memory.h"
#include "wiced_hal_rand.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"

#define GFPS_DEBUG_ENABLE   0

#if (GFPS_DEBUG_ENABLE != 0)
#define GFPS_TRACE(format, ...) \
        WICED_BT_TRACE(format, ##__VA_ARGS__)
#else
#define GFPS_TRACE(...)
#endif

#ifndef _countof
/** Macro to determine the number of elements in an array. */
#define _countof(x) (sizeof(x) / sizeof(x[0]))
#endif

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define REVERSE_PASSKEY(a)                  ((a & 0xff) << 16) | \
                                             (a & 0xff00) | \
                                             ((a & 0xff0000) >> 16)

/* Account Key Data Type */
#define GFPS_ACCOUNT_KEY_DATA_TYPE_FILTER   0
#define GFPS_ACCOUNT_KEY_DATA_TYPE_SALT     1

/* Maximum Account Key Filter Length */
#define GFPS_ACCOUNT_KEY_FILTER_LEN_MAX     (WICED_BT_GFPS_ACCOUNT_KEY_SIZE_MAX * 1) + \
                                            (WICED_BT_GFPS_ACCOUNT_KEY_SIZE_MAX * 2 / 10) + \
                                            3

/* Definition used for BLE Advertisement Data. */
#define GFPS_ACCOUNT_KEY_DATA_RANDOM_SALT_FIELD_LEN             sizeof(uint8_t) + sizeof(uint8_t)
#define GFPS_ACCOUNT_KEY_DATA_ACCOUNT_KEY_FILTER_FIELD_LEN_MAX  GFPS_ACCOUNT_KEY_FILTER_LEN_MAX + sizeof(uint8_t)
#define GFPS_ACCOUNT_KEY_DATA_LEN_MAX                           GFPS_ACCOUNT_KEY_DATA_ACCOUNT_KEY_FILTER_FIELD_LEN_MAX + \
                                                                GFPS_ACCOUNT_KEY_DATA_RANDOM_SALT_FIELD_LEN
#define GFPS_ACCOUNT_DATA_LEN_MAX                               GFPS_ACCOUNT_KEY_DATA_LEN_MAX + sizeof(uint8_t)

#define GFPS_BLE_ADV_DATA_LEN_MAX           31  // maximum BLE advertisement data length

/* Maximum decrypt failure count for provider. */
#define GFPS_PROVIDER_DECRYPT_FAILURE_COUNT_MAX             10

/* Decrypt failure count reset timeout for provider */
#define GFPS_PROVIDER_DECRYPT_FAILURE_COUNT_RESET_TIMEOUT   (5 * 60)    // in seconds

/* Maximum Provider Discoverable Time. */
#define GFPS_PROVIDER_DISCOVERABILITY_TIME                  10  // in seconds

/* Random Salt Update time. */
#define GFPS_PROVIDER_RANDOM_SALT_UPDATE_TIME               (15 * 60)   // in seconds

/* Raw Request Message Type. */
enum
{
    GFPS_RAW_REQ_MSG_TYPE_KEY_BASED_PAIRING_REQ = 0x00,
};

/* Raw Request Flags. */
enum
{
    GFPS_RAW_REQ_FLAGS_REQ_DISCOVERABILITY      = 0x01,
    GFPS_RAW_REQ_FLAGS_PROVIDER_INIT_BONDING    = 0x02,
};

/* Raw Response Message Type. */
enum
{
    GFPS_RAW_RSP_MSG_TYPE_KEY_BASED_PAIRING_RSP = 0x01,
};

/* Raw Passkey Block Message Type. */
enum
{
    GFPS_RAW_PASSKEY_MSG_TYPE_SEEKER_PASSKEY    = 0x02,
    GFPS_RAW_PASSKEY_MSG_TYPE_PROVIDER_PASSKEY  = 0x03,
};

/* element index for discoverable advertisement data elements */
enum
{
    GFPS_PROVIDER_ADV_DATA_DISC_ELEM_IDX_TX_POWER,      // Mandatory - Do NOT delete
    GFPS_PROVIDER_ADV_DATA_DISC_ELEM_IDX_SERVICE_DATA,  // Mandatory - Do NOT delete
    GFPS_PROVIDER_ADV_DATA_DISC_ELEM_IDX_FLAG,
    GFPS_PROVIDER_ADV_DATA_DISC_ELEM_IDX_MAX,
};

/* element index for not discoverable advertisement data elements */
enum
{
    GFPS_PROVIDER_ADV_DATA_NOT_DISC_ELEM_IDX_TX_POWER,      // Mandatory - Do NOT delete
    GFPS_PROVIDER_ADV_DATA_NOT_DISC_ELEM_IDX_SERVICE_DATA,  // Mandatory - Do NOT delete
    GFPS_PROVIDER_ADV_DATA_NOT_DISC_ELEM_IDX_FLAG,
    GFPS_PROVIDER_ADV_DATA_NOT_DISC_ELEM_IDX_MAX,
};

/******************************************************************************
 *                                Structures
 ******************************************************************************/
/* Fast pair GATT service packet formant */
#pragma pack(1)

// Raw Request - Decrypted from the Encryted Request
typedef struct
{
    uint8_t                     message_type;   // refer to < Raw Request Message Type >
    uint8_t                     flags;          // refer to < Raw Request Flags >
    wiced_bt_device_address_t   provider_addr;
    wiced_bt_device_address_t   seeker_addr;
    uint8_t                     reserved[2];
} gfps_raw_request_t;

typedef struct
{
    gfps_raw_request_t  encrypted_req;
    uint8_t             public_key[WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PUBLIC];
} gfps_encrypted_request_t;

typedef struct
{
    uint8_t                     message_type;       // refer to < Raw Response Message Type >
    wiced_bt_device_address_t   provider_addr;
    uint8_t                     random_value[9];
} gfps_raw_response_t;

typedef struct
{
    uint8_t encrypted_passkey[16];
} gfps_encrypted_passkey_t;

typedef struct
{
    uint8_t     message_type;   // refer to < Raw Passkey Block Message Type >
    uint32_t    passkey;
    uint8_t     reserved[11];
} gfps_raw_passkey_t;

#pragma pack()

/* Fast Pair Service module control block content. */
typedef struct GFPS_CB
{
    wiced_bt_gfps_provider_conf_t   conf;

    wiced_bool_t                discoverable;           // device discoverability
    wiced_bt_link_key_t         anti_spoofing_aes_key;  // Calculated ANTI-SPOOFING AES KEY
    wiced_bt_device_address_t   seeker_addr;            // Seeker Address
    uint16_t                    conn_id;                // BLE connection ID used for seeker
    uint32_t                    seeker_passkey;
    uint8_t                     random_salt;            // Random Salt number

    struct
    {
        gfps_encrypted_request_t            keybase_pairing_data;
        wiced_bt_gatt_client_char_config_t  keybase_pairing_cfg;
        gfps_raw_passkey_t                  passkey;
        wiced_bt_gatt_client_char_config_t  passkey_cfg;
        wiced_bt_link_key_t                 accountkey;
    } gatt;

    wiced_bt_gfps_account_key_t             *p_account_key;
    uint8_t             decrypt_failure_count;      // failure count for decrypt raw request
    wiced_timer_t       random_salt_update_timer;   // used only when the account_key_filter_generate_random is set
    wiced_timer_t       decrypt_failure_reset_timer;
    wiced_timer_t       discoverable_timer;
    struct
    {
        struct
        {
            uint8_t                     flag;
            uint8_t                     service_data[5];

            uint8_t                     elem_num;
            wiced_bt_ble_advert_elem_t  *p_elem;
        } discoverable; // discoverable advertisement data

        struct
        {
            uint8_t                     flag;
            uint8_t                     service_data[GFPS_ACCOUNT_DATA_LEN_MAX + sizeof(uint16_t)];

            uint8_t                     elem_num;
            wiced_bt_ble_advert_elem_t  *p_elem;
        } not_discoverable; // not discoverable advertisement data
    } adv_data;

    wiced_bool_t    pairing_started;    // TRUE: GFPS in doing the pairing process
    wiced_bool_t    enabled;            // TRUE: the Provider service is enabled
} gfps_provider_cb_t;

/******************************************************************************
 *                             Variables Definitions
 ******************************************************************************/
gfps_provider_cb_t gfps_provider_cb = {0};

/******************************************************************************
 *                             Function Definitions
 ******************************************************************************/
static wiced_bool_t             gfps_provider_account_data_fill(void *p_account_data, uint8_t *p_account_data_len);
static uint8_t                  gfps_provider_account_key_filter_len_count(void);
static void                     gfps_provider_account_key_update(uint8_t *p_accountkey);
static uint8_t                  gfps_provider_account_key_used_num_get(void);
static void                     gfps_provider_advertisement_data_disc_update(wiced_bool_t model_id);
static wiced_bool_t             gfps_provider_advertisement_data_init(void);
static void                     gfps_provider_advertisement_data_not_disc_update(void);
static void                     gfps_provider_decrypt_failure_count_reset(TIMER_PARAM_TYPE params);
static void                     gfps_provider_discoverable_stop(TIMER_PARAM_TYPE params);
static wiced_bt_gatt_status_t   gfps_provider_gatt_event_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data );
static wiced_bool_t             gfps_provider_key_aes_calculate(uint8_t *public_key, uint8_t *private_key, uint8_t *aes_key);
static wiced_bool_t             gfps_provider_raw_request_content_check(gfps_raw_request_t *p_raw_request);
static wiced_bt_gatt_status_t   gfps_provider_raw_response_send(uint8_t *aes_key);
static void                     gfps_provider_random_salt_update(TIMER_PARAM_TYPE params);
//static void                     gfps_provider_set_disc_advertisement_data(wiced_bool_t model_id);
//static void                     gfps_provider_set_non_disc_advertisement_data( void );
static wiced_bt_gatt_status_t   gfps_provider_raw_passkey_send(uint8_t *aes_key);
static void                     gfps_provider_set_br_edr_discoverable(uint8_t second, wiced_bool_t force);

#if (GFPS_DEBUG_ENABLE)
/*
 * helper function
 */
void gfps_provider_data_hex_display(uint8_t * p_data, uint16_t len)
{
    uint16_t i;
    uint8_t *p_index = p_data;

    for (i = 0 ; i < len ; i++)
    {
        GFPS_TRACE("%02X ", *p_index);
        p_index++;
    }

    GFPS_TRACE("\n");
}

void gfps_provider_account_key_list_display(void)
{
    uint8_t i;
    wiced_bt_gfps_account_key_t *p_account_key = gfps_provider_cb.p_account_key;

    GFPS_TRACE("Account Key List: \n");
    for (i = 0 ; i < gfps_provider_cb.conf.account_key_list_size ; i++)
    {
        GFPS_TRACE("   [%d]: ", i);
        gfps_provider_data_hex_display((uint8_t *) p_account_key->key, sizeof(wiced_bt_link_key_t));

        p_account_key++;
    }
}

#else   // GFPS_DEBUG_ENABLE
void gfps_provider_data_hex_display(uint8_t * p_data, uint16_t len)
{

}

void gfps_provider_account_key_list_display(void)
{

}
#endif  // GFPS_DEBUG_ENABLE

/*
 * helper function to check if the data content is valid
 */
static wiced_bool_t gfps_provider_misc_data_content_check(uint8_t *p_data, uint32_t len)
{
    uint32_t i;
    uint8_t *p_index = p_data;

    for (i = 0 ; i < len ; i++)
    {
        if (*p_index != 0)
        {
            return WICED_TRUE;
        }

        p_index++;
    }

    return WICED_FALSE;
}

/*
 * This function initialize the google fast pair provider module
 *
 * Note: Please do NOT register the GATT event callback again since the
 *       Google Fast Pair Service already registers this.
 *       User shall register its own GATT event callback here via the configuration.
 */
wiced_bool_t wiced_bt_gfps_provider_init(wiced_bt_gfps_provider_conf_t *p_conf)
{
    uint16_t nb_bytes;
    wiced_result_t status;

    GFPS_TRACE("wiced_bt_gfps_provider_init\n");

    /* Check parameter. */
    // Account Key list size
    if ((p_conf->account_key_list_size < WICED_BT_GFPS_ACCOUNT_KEY_SIZE_MIN) ||
        (p_conf->account_key_list_size > WICED_BT_GFPS_ACCOUNT_KEY_SIZE_MAX))
    {
        return WICED_FALSE;
    }

    /* Save configuration. */
    memcpy((void *) &gfps_provider_cb.conf,
           (void *) p_conf,
           sizeof(wiced_bt_gfps_provider_conf_t));

    /* Account Key list */
    // Allocate memory
    gfps_provider_cb.p_account_key = (wiced_bt_gfps_account_key_t *) wiced_memory_allocate(gfps_provider_cb.conf.account_key_list_size * sizeof(wiced_bt_gfps_account_key_t));

    if (gfps_provider_cb.p_account_key == NULL)
    {
        return WICED_FALSE;
    }

    memset((void *) gfps_provider_cb.p_account_key,
           0,
           gfps_provider_cb.conf.account_key_list_size * sizeof(wiced_bt_gfps_account_key_t));

    GFPS_TRACE("  gatt_cb: 0x%08X\n", gfps_provider_cb.conf.p_gatt_cb);
    GFPS_TRACE("  model id: 0x%08X\n", gfps_provider_cb.conf.model_id);
    GFPS_TRACE("  public key: ");
    gfps_provider_data_hex_display(&gfps_provider_cb.conf.anti_spoofing_key.public[0], WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PUBLIC);
    GFPS_TRACE("  private key: ");
    gfps_provider_data_hex_display(&gfps_provider_cb.conf.anti_spoofing_key.private[0], WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PRIVATE);
    GFPS_TRACE("  Account Key Filter Salt Random: %s\n", gfps_provider_cb.conf.account_key_filter_generate_random ? "true" : "false");
    GFPS_TRACE("  Account Key list size: %d\n", gfps_provider_cb.conf.account_key_list_size);
    GFPS_TRACE("  Account Key list NVRAM ID: 0x%04X\n", gfps_provider_cb.conf.account_key_list_nvram_id);

    // Load Account Key list
    nb_bytes = wiced_hal_read_nvram(gfps_provider_cb.conf.account_key_list_nvram_id,
                                    gfps_provider_cb.conf.account_key_list_size * sizeof(wiced_bt_gfps_account_key_t),
                                    (uint8_t *) gfps_provider_cb.p_account_key,
                                    &status);

    if ((nb_bytes == (gfps_provider_cb.conf.account_key_list_size * sizeof(wiced_bt_gfps_account_key_t))) &&
        (status == WICED_BT_SUCCESS))
    {
        gfps_provider_account_key_list_display();
    }

    /* Init. the Random Salt update timer if set. */
    if (gfps_provider_cb.conf.account_key_filter_generate_random)
    {
        wiced_init_timer(&gfps_provider_cb.random_salt_update_timer,
                         &gfps_provider_random_salt_update,
                         0,
                         WICED_SECONDS_TIMER);
    }

    /* Generate the random salt number. */
    if (gfps_provider_cb.conf.account_key_filter_generate_random)
    {
        gfps_provider_cb.random_salt = wiced_hal_rand_gen_num() % 0xff;
    }

    /* Init. the decrypt failure reset timer. */
    wiced_init_timer(&gfps_provider_cb.decrypt_failure_reset_timer,
                     &gfps_provider_decrypt_failure_count_reset,
                     0,
                     WICED_SECONDS_TIMER);

    /* Init. the discoverability timer. */
    wiced_init_timer(&gfps_provider_cb.discoverable_timer,
                     &gfps_provider_discoverable_stop,
                     0,
                     WICED_SECONDS_TIMER);

    /* Init. advertisement data. */
    if (gfps_provider_advertisement_data_init() == WICED_FALSE)
    {
        GFPS_TRACE("Fail to init. advertisement data.\n");
        goto WICED_BT_GFPS_PROVIDER_INIT_FAIL;
    }

    /* GATT registration */
    if (wiced_bt_gatt_register(&gfps_provider_gatt_event_callback) != WICED_BT_GATT_SUCCESS)
    {
        goto WICED_BT_GFPS_PROVIDER_INIT_FAIL;
    }

    /* Start the not discoverable advertisement */
    wiced_bt_gfps_provider_advertisement_start(0);

    /* Start the Random Salt Update timer. */
    if (gfps_provider_cb.conf.account_key_filter_generate_random)
    {
        wiced_start_timer(&gfps_provider_cb.random_salt_update_timer,
                          GFPS_PROVIDER_RANDOM_SALT_UPDATE_TIME);
    }

    /* Set the enabled flag. */
    gfps_provider_cb.enabled = WICED_TRUE;

    return WICED_TRUE;

WICED_BT_GFPS_PROVIDER_INIT_FAIL:

    if (gfps_provider_cb.p_account_key)
    {
        wiced_memory_free((void *) gfps_provider_cb.p_account_key);
        gfps_provider_cb.p_account_key = NULL;
    }

    return WICED_FALSE;
}

/*
 * Update the discoverable advertisement data
 */
static void gfps_provider_advertisement_data_disc_update(wiced_bool_t model_id)
{
    wiced_bt_ble_advert_elem_t *p_elem = NULL;
    wiced_result_t result;

    // Service Data
    p_elem = gfps_provider_cb.adv_data.discoverable.p_elem +
             GFPS_PROVIDER_ADV_DATA_DISC_ELEM_IDX_SERVICE_DATA;

    if (model_id == WICED_TRUE)
    {   // Fast Pair Model ID shall be included.
        p_elem->len= _countof(gfps_provider_cb.adv_data.discoverable.service_data);
    }
    else
    {   // Fast Pair Model ID shall be excluded.
        p_elem->len= sizeof(uint16_t);
    }

    /* Update the not discoverable advertisement data to controller if required. */
    if (gfps_provider_cb.discoverable == WICED_TRUE)
    {
        wiced_bt_ble_set_raw_advertisement_data(gfps_provider_cb.adv_data.discoverable.elem_num,
                                                gfps_provider_cb.adv_data.discoverable.p_elem);
    }
}

static void gfps_provider_advertisement_data_not_disc_update(void)
{
    wiced_bt_ble_advert_elem_t *p_elem = NULL;
    uint8_t account_data_len = GFPS_ACCOUNT_DATA_LEN_MAX;

    GFPS_TRACE("gfps_provider_advertisement_data_not_disc_update (%d)\n", gfps_provider_cb.discoverable);

    // Service Data
    p_elem = gfps_provider_cb.adv_data.not_discoverable.p_elem +
             GFPS_PROVIDER_ADV_DATA_NOT_DISC_ELEM_IDX_SERVICE_DATA;

    if (gfps_provider_account_data_fill((void *) &gfps_provider_cb.adv_data.not_discoverable.service_data[sizeof(uint16_t)],
                                        &account_data_len) == WICED_FALSE)
    {
        GFPS_TRACE("gfps_provider_account_data_fill fail\n");
        return;
    }

    p_elem->len         = sizeof(uint16_t) + account_data_len;

    /* Update the not discoverable advertisement data to controller if required. */
    if (gfps_provider_cb.discoverable == WICED_FALSE)
    {
        wiced_bt_ble_set_raw_advertisement_data(gfps_provider_cb.adv_data.not_discoverable.elem_num,
                                                gfps_provider_cb.adv_data.not_discoverable.p_elem);
    }
}

static wiced_bool_t gfps_provider_advertisement_data_init(void)
{
    wiced_bt_ble_advert_elem_t *p_elem = NULL;
    uint8_t account_data_len = GFPS_ACCOUNT_DATA_LEN_MAX;

    /* Allocate memory. */
    // discoverable advertisement data
    gfps_provider_cb.adv_data.discoverable.elem_num = GFPS_PROVIDER_ADV_DATA_DISC_ELEM_IDX_MAX +
                                                      gfps_provider_cb.conf.appended_adv_data.elem_num;

    gfps_provider_cb.adv_data.discoverable.p_elem = (wiced_bt_ble_advert_elem_t *) wiced_memory_allocate(gfps_provider_cb.adv_data.discoverable.elem_num * sizeof(wiced_bt_ble_advert_elem_t));

    if (gfps_provider_cb.adv_data.discoverable.p_elem == NULL)
    {
        return WICED_FALSE;
    }

    // not discoverable advertisement data
    gfps_provider_cb.adv_data.not_discoverable.elem_num = GFPS_PROVIDER_ADV_DATA_NOT_DISC_ELEM_IDX_MAX +
                                                          gfps_provider_cb.conf.appended_adv_data.elem_num;

    gfps_provider_cb.adv_data.not_discoverable.p_elem = (wiced_bt_ble_advert_elem_t *) wiced_memory_allocate(gfps_provider_cb.adv_data.not_discoverable.elem_num * sizeof(wiced_bt_ble_advert_elem_t));

    if (gfps_provider_cb.adv_data.not_discoverable.p_elem == NULL)
    {
        goto GFPF_PROVIDER_ADVERTISEMENT_DATA_INIT_FAIL;
    }

    /* Discoverable advertisement data */
    // Tx Power level
    p_elem              = gfps_provider_cb.adv_data.discoverable.p_elem +
                          GFPS_PROVIDER_ADV_DATA_DISC_ELEM_IDX_TX_POWER;
    p_elem->advert_type = BTM_BLE_ADVERT_TYPE_TX_POWER;
    p_elem->len         = sizeof(int8_t);
    p_elem->p_data      = (uint8_t*) &gfps_provider_cb.conf.ble_tx_pwr_level;

    // Service Data
    gfps_provider_cb.adv_data.discoverable.service_data[0] = (uint8_t) WICED_BT_GFPS_UUID16;
    gfps_provider_cb.adv_data.discoverable.service_data[1] = (uint8_t) (WICED_BT_GFPS_UUID16 >> 8);
    gfps_provider_cb.adv_data.discoverable.service_data[2] = (uint8_t) (gfps_provider_cb.conf.model_id >> 16);
    gfps_provider_cb.adv_data.discoverable.service_data[3] = (uint8_t) (gfps_provider_cb.conf.model_id >> 8);
    gfps_provider_cb.adv_data.discoverable.service_data[4] = (uint8_t) (gfps_provider_cb.conf.model_id);

    p_elem              = gfps_provider_cb.adv_data.discoverable.p_elem +
                          GFPS_PROVIDER_ADV_DATA_DISC_ELEM_IDX_SERVICE_DATA;
    p_elem->advert_type = BTM_BLE_ADVERT_TYPE_SERVICE_DATA;
    p_elem->len         = _countof(gfps_provider_cb.adv_data.discoverable.service_data);
    p_elem->p_data      = gfps_provider_cb.adv_data.discoverable.service_data;

    // Flag
    gfps_provider_cb.adv_data.discoverable.flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG |
                                                  BTM_BLE_BREDR_NOT_SUPPORTED |
                                                  BTM_BLE_SIMULTANEOUS_DUAL_MODE_TO_SAME_DEVICE_HOST_SUPPORTED;

    p_elem              = gfps_provider_cb.adv_data.discoverable.p_elem +
                          GFPS_PROVIDER_ADV_DATA_DISC_ELEM_IDX_FLAG;
    p_elem->advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    p_elem->len         = sizeof(uint8_t);
    p_elem->p_data      = &gfps_provider_cb.adv_data.discoverable.flag;

    // user appended data
    p_elem              = gfps_provider_cb.adv_data.discoverable.p_elem +
                          GFPS_PROVIDER_ADV_DATA_DISC_ELEM_IDX_MAX;

    memcpy((void *) p_elem,
           (void *) gfps_provider_cb.conf.appended_adv_data.p_elem,
           gfps_provider_cb.conf.appended_adv_data.elem_num * sizeof(wiced_bt_ble_advert_elem_t));

    /* Not discoverable advertisement data. */
    // Tx Power level
    p_elem              = gfps_provider_cb.adv_data.not_discoverable.p_elem +
                          GFPS_PROVIDER_ADV_DATA_NOT_DISC_ELEM_IDX_TX_POWER;
    p_elem->advert_type = BTM_BLE_ADVERT_TYPE_TX_POWER;
    p_elem->len         = sizeof(int8_t);
    p_elem->p_data      = (uint8_t*) &gfps_provider_cb.conf.ble_tx_pwr_level;

    // Service Data
    gfps_provider_cb.adv_data.not_discoverable.service_data[0] = (uint8_t) WICED_BT_GFPS_UUID16;
    gfps_provider_cb.adv_data.not_discoverable.service_data[1] = (uint8_t) (WICED_BT_GFPS_UUID16 >> 8);

    if (gfps_provider_account_data_fill((void *) &gfps_provider_cb.adv_data.not_discoverable.service_data[sizeof(uint16_t)],
                                        &account_data_len) == WICED_FALSE)
    {
        GFPS_TRACE("gfps_provider_account_data_fill fail\n");
        goto GFPF_PROVIDER_ADVERTISEMENT_DATA_INIT_FAIL;
    }

    p_elem              = gfps_provider_cb.adv_data.not_discoverable.p_elem +
                          GFPS_PROVIDER_ADV_DATA_NOT_DISC_ELEM_IDX_SERVICE_DATA;
    p_elem->advert_type = BTM_BLE_ADVERT_TYPE_SERVICE_DATA;
    p_elem->len         = sizeof(uint16_t) + account_data_len;
    p_elem->p_data      = gfps_provider_cb.adv_data.not_discoverable.service_data;

    // Flag
    gfps_provider_cb.adv_data.not_discoverable.flag = BTM_BLE_BREDR_NOT_SUPPORTED |
                                                      BTM_BLE_SIMULTANEOUS_DUAL_MODE_TO_SAME_DEVICE_HOST_SUPPORTED;

    p_elem              = gfps_provider_cb.adv_data.not_discoverable.p_elem +
                          GFPS_PROVIDER_ADV_DATA_NOT_DISC_ELEM_IDX_FLAG;
    p_elem->advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    p_elem->len         = sizeof(uint8_t);
    p_elem->p_data      = &gfps_provider_cb.adv_data.not_discoverable.flag;

    // user appended data
    p_elem              = gfps_provider_cb.adv_data.not_discoverable.p_elem +
                          GFPS_PROVIDER_ADV_DATA_NOT_DISC_ELEM_IDX_MAX;

    memcpy((void *) p_elem,
           (void *) gfps_provider_cb.conf.appended_adv_data.p_elem,
           gfps_provider_cb.conf.appended_adv_data.elem_num * sizeof(wiced_bt_ble_advert_elem_t));

    return WICED_TRUE;

GFPF_PROVIDER_ADVERTISEMENT_DATA_INIT_FAIL:

    if (gfps_provider_cb.adv_data.discoverable.p_elem)
    {
        wiced_memory_free((void *) gfps_provider_cb.adv_data.discoverable.p_elem);
        gfps_provider_cb.adv_data.discoverable.p_elem = NULL;
    }

    if (gfps_provider_cb.adv_data.not_discoverable.p_elem)
    {
        wiced_memory_free((void *) gfps_provider_cb.adv_data.discoverable.p_elem);
        gfps_provider_cb.adv_data.not_discoverable.p_elem = NULL;
    }

    return WICED_FALSE;
}

static void gfps_provider_gatt_event_connection_status(wiced_bt_gatt_event_data_t *p_event_data)
{
    GFPS_TRACE("gfps_provider_gatt_event_connection_status (connected: %d)\n",
               p_event_data->connection_status.connected);

    if (p_event_data->connection_status.connected)
    {   // BLE is connected
        /* We do NOT need to stop the advertisement since the advertisement will be stopped
         * automatically (refer to the description for wiced_bt_start_advertisements()). */
        /* Save information. */
        // Seeker's address
        memcpy(gfps_provider_cb.seeker_addr,
               p_event_data->connection_status.bd_addr,
               sizeof(wiced_bt_device_address_t));

        // connection id
        gfps_provider_cb.conn_id = p_event_data->connection_status.conn_id;
    }
    else
    {   // BLE is disconnected
        gfps_provider_cb.conn_id = 0;
        gfps_provider_cb.pairing_started = WICED_FALSE;

        /* Start the not discoverable advertisement. */
        wiced_bt_gfps_provider_advertisement_start(0);
    }
}

static wiced_bool_t gfps_provider_gatt_event_attribute_request_handler_read(wiced_bt_gatt_event_data_t *p_event_data, wiced_bt_gatt_status_t *p_result)
{
    uint16_t attr_len;
    uint8_t  *p_attr;
    uint16_t attr_len_to_copy;
    uint8_t *p_data_dst;

    GFPS_TRACE("gfps_provider_gatt_event_attribute_request_handler_read (handle: 0x%04X)\n",
               p_event_data->attribute_request.data.read_req.handle);

    if ((p_event_data->attribute_request.data.read_req.handle != gfps_provider_cb.conf.gatt_db_handle.key_pairing_val) &&
        (p_event_data->attribute_request.data.read_req.handle != gfps_provider_cb.conf.gatt_db_handle.key_pairing_cfg_desc) &&
        (p_event_data->attribute_request.data.read_req.handle != gfps_provider_cb.conf.gatt_db_handle.passkey_val) &&
        (p_event_data->attribute_request.data.read_req.handle != gfps_provider_cb.conf.gatt_db_handle.passkey_cfg_desc) &&
        (p_event_data->attribute_request.data.read_req.handle != gfps_provider_cb.conf.gatt_db_handle.account_key_val))
    {
        return WICED_FALSE;
    }

    if (p_event_data->attribute_request.data.read_req.handle == gfps_provider_cb.conf.gatt_db_handle.key_pairing_val)
    {
        attr_len    = sizeof(gfps_encrypted_request_t);
        p_attr      = (uint8_t *) &gfps_provider_cb.gatt.keybase_pairing_data;
    }
    else if (p_event_data->attribute_request.data.read_req.handle == gfps_provider_cb.conf.gatt_db_handle.key_pairing_cfg_desc)
    {
        attr_len    = sizeof(wiced_bt_gatt_client_char_config_t);
        p_attr      = (uint8_t *) &gfps_provider_cb.gatt.keybase_pairing_cfg;
    }
    else if (p_event_data->attribute_request.data.read_req.handle == gfps_provider_cb.conf.gatt_db_handle.passkey_val)
    {
        attr_len    = sizeof(gfps_raw_passkey_t);
        p_attr      = (uint8_t *) &gfps_provider_cb.gatt.passkey;
    }
    else if (p_event_data->attribute_request.data.read_req.handle == gfps_provider_cb.conf.gatt_db_handle.passkey_cfg_desc)
    {
        attr_len    = sizeof(wiced_bt_gatt_client_char_config_t);
        p_attr      = (uint8_t *) &gfps_provider_cb.gatt.passkey_cfg;
    }
    else
    {
        attr_len    = sizeof(wiced_bt_link_key_t);
        p_attr      = (uint8_t *) gfps_provider_cb.gatt.accountkey;
    }

#if BTSTACK_VER >= 0x03000001
    if (p_event_data->attribute_request.data.read_req.offset >= attr_len)
    {
        GFPS_TRACE("offset:%d larger than attribute length:%d\n",
                p_event_data->attribute_request.data.read_req.offset,
                attr_len);
        wiced_bt_gatt_server_send_error_rsp(
                p_event_data->attribute_request.conn_id,
                p_event_data->attribute_request.opcode,
                p_event_data->attribute_request.data.read_req.handle,
                WICED_BT_GATT_INVALID_OFFSET);
        *p_result = WICED_BT_GATT_INVALID_OFFSET;
        return WICED_TRUE;
    }

    attr_len_to_copy = MIN(p_event_data->attribute_request.len_requested,
            attr_len - p_event_data->attribute_request.data.read_req.offset);

    wiced_bt_gatt_server_send_read_handle_rsp(
            p_event_data->attribute_request.conn_id,
            p_event_data->attribute_request.opcode,
            attr_len_to_copy,
            p_attr + p_event_data->attribute_request.data.read_req.offset,
            NULL);

#else /* !BTSTACK_VER */
    /* Check total data length to be filled. */
    if (p_event_data->attribute_request.data.read_req.offset >= attr_len)
    {
        attr_len_to_copy = 0;
    }
    else
    {
        attr_len_to_copy = attr_len;

        if (attr_len_to_copy > *p_event_data->attribute_request.data.read_req.p_val_len)
        {
            attr_len_to_copy = *p_event_data->attribute_request.data.read_req.p_val_len;
        }
    }

    /* Fill in information. */
    if (attr_len_to_copy)
    {
        memcpy((void *) p_event_data->attribute_request.data.read_req.p_val,
               (void *) p_attr + p_event_data->attribute_request.data.read_req.offset,
               attr_len_to_copy);
    }

    *p_event_data->attribute_request.data.read_req.p_val_len = attr_len_to_copy;
#endif /* BTSTACK_VER */

    *p_result = WICED_BT_GATT_SUCCESS;

    return WICED_TRUE;
}

static wiced_bool_t gfps_provider_gatt_event_attribute_request_handler_write_key_pairing_val(wiced_bt_gatt_event_data_t *p_event_data, wiced_bt_gatt_status_t *p_result)
{
    uint8_t *p_index = (uint8_t *) &gfps_provider_cb.gatt.keybase_pairing_data;
    wiced_bool_t public_key_present = WICED_FALSE;
    wiced_bool_t decrypt_result = WICED_FALSE;
    gfps_raw_request_t decrypted_raw_request;
    uint8_t i;
    wiced_bt_gfps_account_key_t *p_account_key = NULL;

    /* Check the decrypt failure count. */
    if (gfps_provider_cb.decrypt_failure_count == GFPS_PROVIDER_DECRYPT_FAILURE_COUNT_MAX)
    {
        GFPS_TRACE("Wait till the failure count resets\n");

        *p_result = WICED_BT_GATT_WRONG_STATE;

        return WICED_TRUE;
    }

    /* Check parameters. */
    // offset
    if (p_event_data->attribute_request.data.write_req.offset >= sizeof(gfps_encrypted_request_t))
    {
        *p_result = WICED_BT_GATT_INVALID_OFFSET;

        return WICED_TRUE;
    }
    // length
    if (p_event_data->attribute_request.data.write_req.val_len >
        (sizeof(gfps_encrypted_request_t) - p_event_data->attribute_request.data.write_req.offset))
    {
        *p_result = WICED_BT_GATT_INVALID_ATTR_LEN;

        return WICED_TRUE;
    }

    /* Update information. */
    p_index += p_event_data->attribute_request.data.write_req.offset;
    memcpy((void *) p_index,
           (void *) p_event_data->attribute_request.data.write_req.p_val,
           p_event_data->attribute_request.data.write_req.val_len);

    /* Check if the incoming Encrypted Request is a complete packet and if the Public Key is present. */
    if (p_event_data->attribute_request.data.write_req.offset + p_event_data->attribute_request.data.write_req.val_len == sizeof(gfps_encrypted_request_t))
    {
        public_key_present = WICED_TRUE;
    }
    else if (p_event_data->attribute_request.data.write_req.offset + p_event_data->attribute_request.data.write_req.val_len == sizeof(gfps_raw_request_t))
    {
        public_key_present = WICED_FALSE;
    }
    else
    {
        *p_result = WICED_BT_GATT_SUCCESS;

        return WICED_TRUE;
    }

    UNUSED_VARIABLE(public_key_present);
    GFPS_TRACE("public_key_present: %d\n", public_key_present);

    /* Process this Encrypted Request. */
    if (p_event_data->attribute_request.data.write_req.offset + p_event_data->attribute_request.data.write_req.val_len == sizeof(gfps_encrypted_request_t))
    {   // The optional Public Key field is present.
        /* Check current pairing mode. */
        if (gfps_provider_cb.discoverable == WICED_FALSE)
        {
            GFPS_TRACE("Pair mode is set to false now\n");

            /* Ignore the write and exit. */
            *p_result = WICED_BT_GATT_WRONG_STATE;

            return WICED_TRUE;
        }

        /* Calculate the Anti-Spoofing AES key using the encapsulated Public Key . */
        if (gfps_provider_key_aes_calculate(&gfps_provider_cb.gatt.keybase_pairing_data.public_key[0],
                                            &gfps_provider_cb.conf.anti_spoofing_key.private[0],
                                            (uint8_t *) gfps_provider_cb.anti_spoofing_aes_key) != WICED_TRUE)
        {
            *p_result = WICED_BT_GATT_AUTH_FAIL;

            return WICED_TRUE;
        }

        /* Decrypt the encrypted raw request. */

        fastpair_sec_aes_ecb_128_decrypt((uint8_t *) &decrypted_raw_request,
                            (uint8_t *) &gfps_provider_cb.gatt.keybase_pairing_data.encrypted_req,
                            gfps_provider_cb.anti_spoofing_aes_key);

        /* Check the content of Raw Request. */
        decrypt_result = gfps_provider_raw_request_content_check(&decrypted_raw_request);
    }
    else
    {   // The optional Public Key field is not present.
        /* Try to decrypt the encrypted request using existent Account Key. */
        p_account_key = gfps_provider_cb.p_account_key;
        for (i = 0 ; i < gfps_provider_cb.conf.account_key_list_size ; i++)
        {
            if (gfps_provider_misc_data_content_check((uint8_t *) p_account_key->key,
                                                      sizeof(wiced_bt_link_key_t)) == WICED_TRUE)
            {
                GFPS_TRACE("Try account key %d\n", i);

                /* Decrypt the encrypted raw request. */
                fastpair_sec_aes_ecb_128_decrypt((uint8_t *) &decrypted_raw_request,
                                    (uint8_t *) &gfps_provider_cb.gatt.keybase_pairing_data.encrypted_req,
                                    p_account_key->key);

                /* Check the content of Raw Request. */
                if (gfps_provider_raw_request_content_check(&decrypted_raw_request) == WICED_TRUE)
                {
                    GFPS_TRACE("Key is found\n");

                    /* Update the Anti-Spoofing AES key. */
                    memcpy((void *) gfps_provider_cb.anti_spoofing_aes_key,
                           (void *) p_account_key->key,
                           sizeof(wiced_bt_link_key_t));

                    /* Subsequent pairing, need turn on BR/EDR for connection and pairing */
                    gfps_provider_set_br_edr_discoverable(GFPS_PROVIDER_DISCOVERABILITY_TIME, WICED_TRUE);

                    break;
                }
            }

            p_account_key++;
        }

        if (i < gfps_provider_cb.conf.account_key_list_size)
        {
            decrypt_result = WICED_TRUE;
        }
    }

    if (decrypt_result == WICED_FALSE)
    {
        /* Increment the decrypt failure count. */
        gfps_provider_cb.decrypt_failure_count++;

        if (gfps_provider_cb.decrypt_failure_count == GFPS_PROVIDER_DECRYPT_FAILURE_COUNT_MAX)
        {
            /* Start timer to reset the decrypt failure count after 5 minutes. */
            wiced_start_timer(&gfps_provider_cb.decrypt_failure_reset_timer,
                              GFPS_PROVIDER_DECRYPT_FAILURE_COUNT_RESET_TIMEOUT);
        }

        *p_result = WICED_BT_GATT_AUTH_FAIL;
        return WICED_TRUE;
    }
    else
    {
        /* Reset the decrypt failure count. */
        gfps_provider_decrypt_failure_count_reset(0);
    }

    GFPS_TRACE("Anti Spoofing AES Key: ");
    gfps_provider_data_hex_display((uint8_t *) gfps_provider_cb.anti_spoofing_aes_key, sizeof(wiced_bt_link_key_t));

    GFPS_TRACE("Decrypted Raw Request: ");
    gfps_provider_data_hex_display((uint8_t *) &decrypted_raw_request, sizeof(gfps_raw_request_t));

    /* Check the discoverablitiy field value. */
    if (decrypted_raw_request.flags & GFPS_RAW_REQ_FLAGS_REQ_DISCOVERABILITY)
    {
        gfps_provider_set_br_edr_discoverable(GFPS_PROVIDER_DISCOVERABILITY_TIME, WICED_FALSE);
    }

    /* Produce the 16-byte Raw Response. */
    *p_result = gfps_provider_raw_response_send(gfps_provider_cb.anti_spoofing_aes_key);

    if (*p_result == WICED_BT_GATT_SUCCESS)
    {
        /* Set current pairing state to during pairing state. */
        gfps_provider_cb.pairing_started = WICED_TRUE;

        /* Check if the provider shall initiate bonding to the seeker's BR/EDR address. todo */
        if (decrypted_raw_request.flags & GFPS_RAW_REQ_FLAGS_PROVIDER_INIT_BONDING)
        {
            GFPS_TRACE("the Seeker (%B) requests that the Provider shall initiate bonding. \n", decrypted_raw_request.seeker_addr);
        }
    }

    return WICED_TRUE;
}

static wiced_bool_t gfps_provider_gatt_event_attribute_request_handler_write_key_pairing_cfg_desc(wiced_bt_gatt_event_data_t *p_event_data, wiced_bt_gatt_status_t *p_result)
{
    /* Check parameters. */
    // offset
    if (p_event_data->attribute_request.data.write_req.offset >= sizeof(wiced_bt_gatt_client_char_config_t))
    {
        *p_result = WICED_BT_GATT_INVALID_OFFSET;

        return WICED_TRUE;
    }
    // length
    if (p_event_data->attribute_request.data.write_req.val_len >
        (sizeof(wiced_bt_gatt_client_char_config_t) - p_event_data->attribute_request.data.write_req.offset ))
    {
        *p_result = WICED_BT_GATT_INVALID_ATTR_LEN;

        return WICED_TRUE;
    }

    /* Update information. */
    memcpy((void *) ((uint8_t *) &gfps_provider_cb.gatt.keybase_pairing_cfg) + p_event_data->attribute_request.data.write_req.offset,
           (void *) p_event_data->attribute_request.data.write_req.p_val,
           p_event_data->attribute_request.data.write_req.val_len);

    *p_result = WICED_BT_GATT_SUCCESS;

    return WICED_TRUE;
}

static wiced_bool_t gfps_provider_gatt_event_attribute_request_handler_write_passkey_val(wiced_bt_gatt_event_data_t *p_event_data, wiced_bt_gatt_status_t *p_result)
{
    uint8_t *p_index = (uint8_t *) &gfps_provider_cb.gatt.passkey;
    gfps_raw_passkey_t decrypted_raw_passkey;
    uint32_t rev_passkey;

    /* Check parameters. */
    // offset
    if (p_event_data->attribute_request.data.write_req.offset >= sizeof(gfps_raw_passkey_t))
    {
        *p_result = WICED_BT_GATT_INVALID_OFFSET;

        return WICED_TRUE;
    }
    // length
    if (p_event_data->attribute_request.data.write_req.val_len >
        (sizeof(gfps_raw_passkey_t) - p_event_data->attribute_request.data.write_req.offset ))
    {
        *p_result = WICED_BT_GATT_INVALID_ATTR_LEN;

        return WICED_TRUE;
    }

    /* Update information. */
    p_index += p_event_data->attribute_request.data.write_req.offset;
    memcpy((void *) p_index,
           (void *) p_event_data->attribute_request.data.write_req.p_val,
           p_event_data->attribute_request.data.write_req.val_len);


    if (p_event_data->attribute_request.data.write_req.offset + p_event_data->attribute_request.data.write_req.val_len != sizeof(gfps_raw_passkey_t))
    {
        *p_result = WICED_BT_GATT_SUCCESS;

        return WICED_TRUE;
    }

    /* Decrypt this Encrypted Raw Passkey. */
    fastpair_sec_aes_ecb_128_decrypt((uint8_t *) &decrypted_raw_passkey,
                        (uint8_t *) &gfps_provider_cb.gatt.passkey,
                        gfps_provider_cb.anti_spoofing_aes_key);

    GFPS_TRACE("Decrypted Raw Passkey: ");
    gfps_provider_data_hex_display((uint8_t *) &decrypted_raw_passkey, sizeof(gfps_raw_passkey_t));

    /* Check Message Type. */
    if (decrypted_raw_passkey.message_type != GFPS_RAW_PASSKEY_MSG_TYPE_SEEKER_PASSKEY)
    {
        *p_result = WICED_BT_GATT_AUTH_FAIL;

        return WICED_TRUE;
    }

    /* Check passkey. */
    rev_passkey = REVERSE_PASSKEY(decrypted_raw_passkey.passkey);
    if (rev_passkey != gfps_provider_cb.seeker_passkey)
    {
        GFPS_TRACE("passkey not match 0x%08X <-> 0x%08X\n", rev_passkey, gfps_provider_cb.seeker_passkey);

        *p_result = WICED_BT_GATT_AUTH_FAIL;
    }
    else
    {
        *p_result = WICED_BT_GATT_SUCCESS;
    }

    /* Transmit our own passkey. */
    gfps_provider_raw_passkey_send(gfps_provider_cb.anti_spoofing_aes_key);

    return WICED_TRUE;
}

static wiced_bool_t gfps_provider_gatt_event_attribute_request_handler_write_passkey_cfg_desc(wiced_bt_gatt_event_data_t *p_event_data, wiced_bt_gatt_status_t *p_result)
{
    /* Check parameters. */
    // offset
    if (p_event_data->attribute_request.data.write_req.offset >= sizeof(wiced_bt_gatt_client_char_config_t))
    {
        *p_result = WICED_BT_GATT_INVALID_OFFSET;

        return WICED_TRUE;
    }
    // length
    if (p_event_data->attribute_request.data.write_req.val_len >
        (sizeof(wiced_bt_gatt_client_char_config_t) - p_event_data->attribute_request.data.write_req.offset ))
    {
        *p_result = WICED_BT_GATT_INVALID_ATTR_LEN;

        return WICED_TRUE;
    }

    /* Update information. */
    memcpy((void *) ((uint8_t *) &gfps_provider_cb.gatt.passkey_cfg) + p_event_data->attribute_request.data.write_req.offset,
           (void *) p_event_data->attribute_request.data.write_req.p_val,
           p_event_data->attribute_request.data.write_req.val_len);

    *p_result = WICED_BT_GATT_SUCCESS;

    return WICED_TRUE;
}

static wiced_bool_t gfps_provider_gatt_event_attribute_request_handler_write_account_key_val(wiced_bt_gatt_event_data_t *p_event_data, wiced_bt_gatt_status_t *p_result)
{
    uint8_t *p_index = (uint8_t *) gfps_provider_cb.gatt.accountkey;
    wiced_bt_link_key_t decrypted_account_key;

    /* Check parameters. */
    // offset
    if (p_event_data->attribute_request.data.write_req.offset >= sizeof(wiced_bt_link_key_t))
    {
        *p_result = WICED_BT_GATT_INVALID_OFFSET;

        return WICED_TRUE;
    }
    // length
    if (p_event_data->attribute_request.data.write_req.val_len >
        (sizeof(wiced_bt_link_key_t) - p_event_data->attribute_request.data.write_req.offset ))
    {
        *p_result = WICED_BT_GATT_INVALID_ATTR_LEN;

        return WICED_TRUE;
    }

    /* Update information. */
    p_index += p_event_data->attribute_request.data.write_req.offset;
    memcpy((void *) p_index,
           (void *) p_event_data->attribute_request.data.write_req.p_val,
           p_event_data->attribute_request.data.write_req.val_len);


    if (p_event_data->attribute_request.data.write_req.offset + p_event_data->attribute_request.data.write_req.val_len != sizeof(wiced_bt_link_key_t))
    {
        *p_result = WICED_BT_GATT_SUCCESS;

        return WICED_TRUE;
    }

    /* Decrypt this Encrypted Account Key. */
    fastpair_sec_aes_ecb_128_decrypt((uint8_t *) decrypted_account_key,
                        (uint8_t *) gfps_provider_cb.gatt.accountkey,
                        gfps_provider_cb.anti_spoofing_aes_key);

    GFPS_TRACE("Decrypted Account Key: ");
    gfps_provider_data_hex_display(decrypted_account_key, sizeof(wiced_bt_link_key_t));

    /* Update Account Key list. */
    gfps_provider_account_key_update(decrypted_account_key);

    *p_result = WICED_BT_GATT_SUCCESS;

    return WICED_TRUE;
}

static wiced_bool_t gfps_provider_gatt_event_attribute_request_handler_write(wiced_bt_gatt_event_data_t *p_event_data, wiced_bt_gatt_status_t *p_result)
{
    GFPS_TRACE("gfps_provider_gatt_event_attribute_request_handler_write (handle: 0x%04X, offset: %d, val_len: %d)\n",
               p_event_data->attribute_request.data.write_req.handle,
               p_event_data->attribute_request.data.write_req.offset,
               p_event_data->attribute_request.data.write_req.val_len);

    GFPS_TRACE("val: ");
    gfps_provider_data_hex_display(p_event_data->attribute_request.data.write_req.p_val,
                                   p_event_data->attribute_request.data.write_req.val_len);

    if (p_event_data->attribute_request.data.write_req.handle == gfps_provider_cb.conf.gatt_db_handle.key_pairing_val)
    {
        return gfps_provider_gatt_event_attribute_request_handler_write_key_pairing_val(p_event_data, p_result);
    }

    if (p_event_data->attribute_request.data.write_req.handle == gfps_provider_cb.conf.gatt_db_handle.key_pairing_cfg_desc)
    {
        return gfps_provider_gatt_event_attribute_request_handler_write_key_pairing_cfg_desc(p_event_data, p_result);
    }

    if (p_event_data->attribute_request.data.write_req.handle == gfps_provider_cb.conf.gatt_db_handle.passkey_val)
    {
        return gfps_provider_gatt_event_attribute_request_handler_write_passkey_val(p_event_data, p_result);
    }

    if (p_event_data->attribute_request.data.write_req.handle == gfps_provider_cb.conf.gatt_db_handle.passkey_cfg_desc)
    {
        return gfps_provider_gatt_event_attribute_request_handler_write_passkey_cfg_desc(p_event_data, p_result);
    }

    if (p_event_data->attribute_request.data.write_req.handle == gfps_provider_cb.conf.gatt_db_handle.account_key_val)
    {
        return gfps_provider_gatt_event_attribute_request_handler_write_account_key_val(p_event_data, p_result);
    }

    return WICED_FALSE;
}

static wiced_bool_t gfps_provider_gatt_event_attribute_request_handler(wiced_bt_gatt_event_data_t *p_event_data, wiced_bt_gatt_status_t *p_result)
{
#if BTSTACK_VER >= 0x03000001
    wiced_bool_t ret;

    GFPS_TRACE("gfps_provider_gatt_event_attribute_request_handler (opcode: %d)\n",
               p_event_data->attribute_request.opcode);

    switch (p_event_data->attribute_request.opcode)
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
            return gfps_provider_gatt_event_attribute_request_handler_read(p_event_data, p_result);

        /* TODO */
#if 0
        case GATT_REQ_READ_BY_TYPE:
            return gfps_provider_gatt_event_attribute_request_handler_read_by_type(p_event_data, p_result);

        case GATT_REQ_READ_MULTI:
        case GATT_REQ_READ_MULTI_VAR_LENGTH:
            return gfps_provider_gatt_event_attribute_request_handler_read_multi(p_event_data, p_result);
#endif

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
        case GATT_CMD_SIGNED_WRITE:
            ret = gfps_provider_gatt_event_attribute_request_handler_write(p_event_data, p_result);
            if (ret)
            {
                if (*p_result == WICED_BT_GATT_SUCCESS)
                {
                    wiced_bt_gatt_server_send_write_rsp(
                            p_event_data->attribute_request.conn_id,
                            p_event_data->attribute_request.opcode,
                            p_event_data->attribute_request.data.write_req.handle);
                }
                else
                {
                    wiced_bt_gatt_server_send_error_rsp(
                            p_event_data->attribute_request.conn_id,
                            p_event_data->attribute_request.opcode,
                            p_event_data->attribute_request.data.write_req.handle,
                            *p_result);
                }
            }

            return ret;
    }

#else /* !BTSTACK_VER */
    GFPS_TRACE("gfps_provider_gatt_event_attribute_request_handler (type: %d)\n",
               p_event_data->attribute_request.request_type);

    switch (p_event_data->attribute_request.request_type)
    {
    case GATTS_REQ_TYPE_READ:
        return gfps_provider_gatt_event_attribute_request_handler_read(p_event_data, p_result);
    case GATTS_REQ_TYPE_WRITE:
        return gfps_provider_gatt_event_attribute_request_handler_write(p_event_data, p_result);
    case GATTS_REQ_TYPE_PREP_WRITE:
        break;
    case GATTS_REQ_TYPE_WRITE_EXEC:
        break;
    case GATTS_REQ_TYPE_MTU:
        break;
    case GATTS_REQ_TYPE_CONF:
        break;
    }
#endif /* BTSTACK_VER */

    return WICED_FALSE;
}

static wiced_bt_gatt_status_t gfps_provider_gatt_event_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_REQ_NOT_SUPPORTED;

    GFPS_TRACE("gfps_provider_gatt_event_callback (%d, event: %d)\n",
               gfps_provider_cb.enabled,
               event);

    /* Check if this event callback shall be processed by Google Fast Pair Service module. */
    if (gfps_provider_cb.enabled)
    {
        switch (event)
        {
        case GATT_CONNECTION_STATUS_EVT:
            gfps_provider_gatt_event_connection_status(p_event_data);
            result = WICED_BT_GATT_SUCCESS;
            break;
        case GATT_ATTRIBUTE_REQUEST_EVT:
            if (gfps_provider_gatt_event_attribute_request_handler(p_event_data, &result) == WICED_TRUE)
            {
                return result;
            }
            break;
#if BTSTACK_VER >= 0x03000001
        case GATT_GET_RESPONSE_BUFFER_EVT:
            p_event_data->buffer_request.buffer.p_app_rsp_buffer =
                wiced_bt_get_buffer(p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt = wiced_bt_free_buffer;
            return WICED_BT_GATT_SUCCESS;

        case GATT_APP_BUFFER_TRANSMITTED_EVT:
            {
                void (*pfn_free)(uint8_t *) =
                    (void (*)(uint8_t *))p_event_data->buffer_xmitted.p_app_ctxt;

                /* If the buffer is dynamic, the context will point to a function to free it. */
                if (pfn_free)
                    pfn_free(p_event_data->buffer_xmitted.p_app_data);

                return WICED_BT_GATT_SUCCESS;
            }
            break;
#endif /* BTSTACK_VER */
        default:
            break;
        }
    }

    /* Call application callback to handle this event. */
    if (gfps_provider_cb.conf.p_gatt_cb)
    {
        return (*gfps_provider_cb.conf.p_gatt_cb)(event, p_event_data);
    }

    return result;
}

/*
 * Process write request or write command from peer device
 */
void wiced_bt_gfps_provider_advertisement_start(uint8_t discoverability)
{
    GFPS_TRACE("wiced_bt_gfps_provider_advertisement_start (discoverability: %d)\n", discoverability);

    if (discoverability != 0)
    {
        gfps_provider_cb.discoverable = WICED_TRUE;

        wiced_bt_ble_set_raw_advertisement_data(gfps_provider_cb.adv_data.discoverable.elem_num,
                                                gfps_provider_cb.adv_data.discoverable.p_elem);

#if BTSTACK_VER >= 0x03000001
        /* In newer BTSTACK, it had been updated to use new HCI command set of
         * extended advertisement in all related WICED APIs. Per spec,
         * SCAN_RESP data is mandotary for HCI_LE_Set_Extended_Advertising_Enable
         * with scannable advertising and it's uncompatible with original
         * HCI_LE_Set_Advertising_Enable.
         * To keep backward compatibility, here we set user appended data as
         * SCAN_RESP data. This method here only works if user appended data
         * is not empty.
         * NOTE: set SCAN_RESP data is required to be done after
         * wiced_bt_ble_set_raw_advertisement_data
         */
        wiced_bt_ble_set_raw_scan_response_data(
                gfps_provider_cb.conf.appended_adv_data.elem_num,
                gfps_provider_cb.conf.appended_adv_data.p_elem);
#endif

        wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    }
    else
    {
        gfps_provider_cb.discoverable = WICED_FALSE;

        wiced_bt_ble_set_raw_advertisement_data(gfps_provider_cb.adv_data.not_discoverable.elem_num,
                                                gfps_provider_cb.adv_data.not_discoverable.p_elem);

        wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
    }
}

static wiced_bt_gatt_status_t gfps_provider_raw_response_send( uint8_t *aes_key )
{
    gfps_raw_response_t raw_resp;
    gfps_raw_response_t en_resp;
    wiced_bt_device_address_t local_bt_addr;
    uint32_t rand_val;
    uint8_t i;

    /* Acquire local BT device address. */
    wiced_bt_dev_read_local_addr(local_bt_addr);

    /* produce raw response */
    raw_resp.message_type = GFPS_RAW_RSP_MSG_TYPE_KEY_BASED_PAIRING_RSP;
    memcpy(raw_resp.provider_addr, local_bt_addr, sizeof(wiced_bt_device_address_t));
    rand_val = wiced_hal_rand_gen_num();
    memcpy(raw_resp.random_value, &rand_val, sizeof(rand_val));
    rand_val = wiced_hal_rand_gen_num();
    memcpy(raw_resp.random_value + sizeof(rand_val), &rand_val, sizeof(rand_val));
    rand_val = wiced_hal_rand_gen_num();
    memcpy(raw_resp.random_value + sizeof(rand_val) * 2, &rand_val, 1);

    GFPS_TRACE("raw_resp: ");
    gfps_provider_data_hex_display((uint8_t *) &raw_resp, sizeof(gfps_raw_response_t));

    fastpair_sec_aes_ecb_128_encrypt((uint8_t *)&en_resp, (uint8_t *)&raw_resp, aes_key);

    GFPS_TRACE("encrypted raw_resp: ");
    gfps_provider_data_hex_display((uint8_t *) &en_resp, sizeof(gfps_raw_response_t));

    /* notify key-based pairing characteristic */
#if BTSTACK_VER >= 0x03000001
    return wiced_bt_gatt_server_send_notification(
            gfps_provider_cb.conn_id,
            gfps_provider_cb.conf.gatt_db_handle.key_pairing_val,
            sizeof(en_resp),
            (uint8_t *)&en_resp,
            NULL);
#else
    return wiced_bt_gatt_send_notification (gfps_provider_cb.conn_id,
            gfps_provider_cb.conf.gatt_db_handle.key_pairing_val, sizeof(en_resp), (uint8_t *)&en_resp );
#endif

}

static wiced_bt_gatt_status_t gfps_provider_raw_passkey_send(uint8_t *aes_key)
{
    gfps_raw_passkey_t raw_passkey;
    gfps_encrypted_passkey_t en_passkey;
    uint32_t rev_passkey = REVERSE_PASSKEY(gfps_provider_cb.seeker_passkey);
    uint8_t i;
    uint8_t *raw = (uint8_t *)&raw_passkey;

    /* produce raw response */
    raw_passkey.message_type = GFPS_RAW_PASSKEY_MSG_TYPE_PROVIDER_PASSKEY;
    memcpy(&raw_passkey.passkey, &rev_passkey, 3);

    GFPS_TRACE("raw_passkey: ");
    gfps_provider_data_hex_display((uint8_t *) &raw_passkey, sizeof(gfps_raw_passkey_t));

    fastpair_sec_aes_ecb_128_encrypt((uint8_t *)&en_passkey, (uint8_t *)&raw_passkey, aes_key);

    /* notify key-based pairing characteristic */
#if BTSTACK_VER >= 0x03000001
    return wiced_bt_gatt_server_send_notification(
            gfps_provider_cb.conn_id,
            gfps_provider_cb.conf.gatt_db_handle.passkey_val,
            sizeof(en_passkey),
            (uint8_t *)&en_passkey,
            NULL);
#else
    return wiced_bt_gatt_send_notification (gfps_provider_cb.conn_id,
            gfps_provider_cb.conf.gatt_db_handle.passkey_val, sizeof(en_passkey), (uint8_t *)&en_passkey );
#endif
}

void wiced_bt_gfps_provider_seeker_passkey_set(uint32_t passkey)
{
    GFPS_TRACE("wiced_bt_gfps_provider_seeker_passkey_set (passkey: %d)\n", passkey);

    gfps_provider_cb.seeker_passkey = passkey;
}

/*
 * This function generates final account key filter, return filter length
 */
static wiced_bool_t gfps_provider_account_data_fill(void *p_account_data, uint8_t *p_account_data_len)
{
    //uint8_t account_key_num;
    uint8_t account_key_filter_len;
    uint8_t account_key_data_len;
    uint8_t account_data_len;
    uint8_t *p_account_key_filter = NULL;
    wiced_bt_gfps_account_key_t *p_account_key = NULL;
    uint8_t i, j;
    uint8_t *p_index = p_account_data;
    uint8_t sha_source[sizeof(wiced_bt_link_key_t) + sizeof(wiced_bt_device_address_t)];
    uint8_t sha_result[32] = {0};
    uint8_t sha_source_len;
    uint32_t X[8] = {0};
    uint32_t value;
    uint8_t K;
    wiced_bt_device_address_t local_rpa;

    /* 1. Compute the filter size in bytes: s.*/
    account_key_filter_len = gfps_provider_account_key_filter_len_count();

    if (gfps_provider_cb.conf.account_key_filter_generate_random)
    {   // Random number is used as the Salt
        account_key_data_len = sizeof(uint8_t) +        // Field length and type for Account Key Filter
                               account_key_filter_len +
                               sizeof(uint8_t) +        // Field length and type for Salt
                               sizeof(uint8_t);         // Salt
    }
    else
    {   // BLE RPA is used as the Salt
        account_key_data_len = sizeof(uint8_t) +        // Field length and type for Account Key Filter
                               account_key_filter_len;
    }

    account_data_len = sizeof(uint8_t) +        // Flags
                       account_key_data_len;

    if (account_data_len > *p_account_data_len)
    {
        return WICED_FALSE;
    }

    *p_account_data_len = account_data_len;

    /* 2. Initialize the filter data. */
    p_account_key_filter = (uint8_t *) (p_index + sizeof(uint8_t) + sizeof(uint8_t));
    memset((void *) p_account_key_filter, 0, account_key_filter_len);

    /* 3. Compute the Account Key Filter. */
    memcpy(local_rpa, (void *)wiced_btm_get_private_bda(), sizeof(wiced_bt_device_address_t));

    p_account_key = gfps_provider_cb.p_account_key;
    for (i = 0 ; i < gfps_provider_cb.conf.account_key_list_size ; i++)
    {
        if (gfps_provider_misc_data_content_check((uint8_t *) p_account_key->key,
                                                  sizeof(wiced_bt_link_key_t)) == WICED_TRUE)
        {
            /* a. Produce the SHA256 source (V). */
            memcpy((void *) &sha_source[0],
                   (void *) p_account_key->key,
                   sizeof(wiced_bt_link_key_t));

            sha_source_len = sizeof(wiced_bt_link_key_t);

            if (gfps_provider_cb.conf.account_key_filter_generate_random)
            {   // Random number is used as the Salt
                memcpy((void *) &sha_source[sha_source_len],
                       (void *) &gfps_provider_cb.random_salt,
                       sizeof(uint8_t));

                sha_source_len += sizeof(uint8_t);
            }
            else
            {   // BLE RPA is used as the Salt
                memcpy((void *) &sha_source[sha_source_len],
                       (void *) local_rpa,
                       sizeof(wiced_bt_device_address_t));

                sha_source_len += sizeof(wiced_bt_device_address_t);
            }

            /* b. Obtain the 256-bit hashed value (H). */

            fastpair_sec_sha256(sha_source, sha_source_len, sha_result);

            /* c. Divide H into eight 32-bit unsigned integers, X={X0,...,X7} */
            for (j = 0 ; j < _countof(X) ; j++)
            {
                value = (uint32_t) sha_result[j * 4];
                X[j] = value << 24;

                value = (uint32_t) sha_result[(j * 4) + 1];
                X[j] += (value << 16);

                value = (uint32_t) sha_result[(j * 4) + 2];
                X[j] += (value << 8);

                value = (uint32_t) sha_result[(j * 4) + 3];
                X[j] += value;
            }

            /* d. Fill data to Account Key Filter. */
            for (j = 0 ; j < _countof(X) ; j++)
            {
                K = X[j] % (account_key_filter_len * 8);
                p_index = p_account_key_filter + (K / 8);
                *p_index |= (1 << (K % 8));
            }
        }

        p_account_key++;
    }

    /* Fill the Account Data except for the Account Key Filter. */
    // Flags
    p_index = (uint8_t *) p_account_data;
    *p_index++ = 0;

    // Account Key Data
    // Account Key Data - Field length and type for Account Key Filter
    *p_index++ = (account_key_filter_len << 4) | GFPS_ACCOUNT_KEY_DATA_TYPE_FILTER;

    if (gfps_provider_cb.conf.account_key_filter_generate_random)
    {
        // Account Key Data - Field length and type for Salt
        p_index += account_key_filter_len;
        *p_index++ = (1 << 4) | GFPS_ACCOUNT_KEY_DATA_TYPE_SALT;

        // Account Key Data - Salt
        *p_index = gfps_provider_cb.random_salt;
    }

    return WICED_TRUE;
}

/*
 * This function calculates the number of current used account key entries
 */
static uint8_t gfps_provider_account_key_used_num_get(void)
{
    uint8_t i;
    uint8_t used_num = 0;
    wiced_bt_gfps_account_key_t *p_account_key = gfps_provider_cb.p_account_key;

    for (i = 0 ; i < gfps_provider_cb.conf.account_key_list_size ; i++)
    {
        if (gfps_provider_misc_data_content_check((uint8_t *) p_account_key->key,
                                                  sizeof(wiced_bt_link_key_t)) == WICED_TRUE)
        {
            used_num++;
        }

        p_account_key++;
    }

    return used_num;
}

/*
 * This function calculates the length of current account key filter
 */
static uint8_t gfps_provider_account_key_filter_len_count(void)
{
    uint8_t filter_len = 0;
    uint8_t account_key_num = gfps_provider_account_key_used_num_get();

    /* Calculate the truncated value of (1.2 * n + 3),
     * where n is the number of stored Account Keys. */
    filter_len = (account_key_num * 1 ) +
                 (account_key_num * 2 / 10) +
                 3;

    GFPS_TRACE("gfps_provider_account_key_filter_len_count (filter_len: %d)\n", filter_len);

    return filter_len;
}

/*
 * Push the new entry to the first entry of the account key list
 */
static void gfps_provider_account_key_sort(uint8_t new_entry)
{
    wiced_bt_gfps_account_key_t tmp;
    wiced_bt_gfps_account_key_t *p_target;
    wiced_bt_gfps_account_key_t *p_source;
    uint8_t i;
    uint8_t last_valid_entry;
    int16_t j;

    /* Check parameter. */
    if (new_entry >= gfps_provider_cb.conf.account_key_list_size)
    {
        return;
    }

    /* Copy the new account key to the temporary space. */
    p_source = gfps_provider_cb.p_account_key + new_entry;
    memcpy((void *) tmp.key,
           (void *) p_source->key,
           sizeof(wiced_bt_link_key_t));

    if (new_entry == gfps_provider_cb.conf.account_key_list_size - 1)
    {
        last_valid_entry = new_entry - 1;
    }
    else
    {
        for (i = new_entry + 1 ; i < gfps_provider_cb.conf.account_key_list_size ; i++)
        {
            p_target = gfps_provider_cb.p_account_key + ( i - 1 );
            p_source = gfps_provider_cb.p_account_key + i;

            if (gfps_provider_misc_data_content_check((uint8_t *) p_source->key,
                                                      sizeof(wiced_bt_link_key_t)) == WICED_TRUE)
            {
                memcpy((void *) p_target->key,
                       (void *) p_source->key,
                       sizeof(wiced_bt_link_key_t));
            }
            else
            {
                break;
            }
        }

        last_valid_entry = i - 2;
    }

    for (j = last_valid_entry ; j >= 0 ; j--)
    {
        p_source = gfps_provider_cb.p_account_key + j;
        p_target = gfps_provider_cb.p_account_key + ( j + 1 );

        memcpy((void *) p_target->key,
               (void *) p_source->key,
               sizeof(wiced_bt_link_key_t));
    }

    p_target = gfps_provider_cb.p_account_key;

    memcpy((void *) p_target->key,
           (void *) tmp.key,
           sizeof(wiced_bt_link_key_t));
}

static void gfps_provider_account_key_update(uint8_t *p_accountkey)
{
    wiced_result_t status;
    uint16_t nb_bytes;
    uint8_t i, j;
    wiced_bt_gfps_account_key_t *p_account_key;

    GFPS_TRACE("gfps_provider_account_key_update: ");
    gfps_provider_data_hex_display(p_accountkey, sizeof(wiced_bt_link_key_t));

    /* Check if the Account Key already exists in the Account Key list. */
    p_account_key = gfps_provider_cb.p_account_key;
    for (i = 0 ; i < gfps_provider_cb.conf.account_key_list_size ; i++)
    {
        if (memcmp((void *) p_account_key->key,
                   (void *) p_accountkey,
                   sizeof(wiced_bt_link_key_t)) == 0)
        {
            break;
        }

        p_account_key++;
    }

    if (i < gfps_provider_cb.conf.account_key_list_size)
    {
        /* Sort the account key list. Push this account key to the first entry of the list. */
        if (i == 0)
        {
            return;
        }

        gfps_provider_account_key_sort(i);

        goto FASTPAIR_ACCOUNT_KEY_UPDATE_NVRAM_WRITE;
    }

    /* Find a free space */
    p_account_key = gfps_provider_cb.p_account_key;
    for (i = 0 ; i < gfps_provider_cb.conf.account_key_list_size ; i++)
    {
        if (gfps_provider_misc_data_content_check((uint8_t *) p_account_key->key,
                                                  sizeof(wiced_bt_link_key_t)) == WICED_FALSE)
        {
            memcpy((void *) p_account_key->key,
                   (void *) p_accountkey,
                   sizeof(wiced_bt_link_key_t));

            break;
        }

        p_account_key++;
    }

    if (i < gfps_provider_cb.conf.account_key_list_size)
    {
        /* Sort the account key list. Push this account key to the first entry of the list. */
        if (i != 0)
        {
            gfps_provider_account_key_sort(i);
        }

        goto FASTPAIR_ACCOUNT_KEY_UPDATE_NVRAM_WRITE;
    }

    /* Delete the least recently used value from the list and add the new value to the list. */
    i = gfps_provider_cb.conf.account_key_list_size - 1;

    p_account_key = gfps_provider_cb.p_account_key + i;
    memcpy((void *) p_account_key->key,
           (void *) p_accountkey,
           sizeof(wiced_bt_link_key_t));

    /* Sort the account key list. Push this account key to the first entry of the list. */
    gfps_provider_account_key_sort(i);

FASTPAIR_ACCOUNT_KEY_UPDATE_NVRAM_WRITE:

    /* Write to NVRAM. */
    nb_bytes = wiced_hal_write_nvram(gfps_provider_cb.conf.account_key_list_nvram_id,
                                     gfps_provider_cb.conf.account_key_list_size * sizeof(wiced_bt_gfps_account_key_t),
                                     (uint8_t *) gfps_provider_cb.p_account_key,
                                     &status);

    if ((nb_bytes == (gfps_provider_cb.conf.account_key_list_size * sizeof(wiced_bt_gfps_account_key_t))) &&
        (status == WICED_BT_SUCCESS))
    {
        GFPS_TRACE("gfps_provider_account_key_update success\n");
    }
    else
    {
        GFPS_TRACE("gfps_provider_account_key_update fail\n");
    }

    /* Update the not discoverable advertisement data content. */
    gfps_provider_advertisement_data_not_disc_update();

    gfps_provider_account_key_list_display();
}

static void gfps_provider_random_salt_update(TIMER_PARAM_TYPE params)
{
    GFPS_TRACE("gfps_provider_random_salt_update\n");

    /* Update the random salt. */
    gfps_provider_cb.random_salt = wiced_hal_rand_gen_num() % 0xff;

    /* Update the not discoverable advertisement data. */
    gfps_provider_advertisement_data_not_disc_update();
}

static void gfps_provider_decrypt_failure_count_reset(TIMER_PARAM_TYPE params)
{
    GFPS_TRACE("gfps_provider_decrypt_failure_count_reset\n");

    gfps_provider_cb.decrypt_failure_count = 0;
}

static void gfps_provider_discoverable_stop(TIMER_PARAM_TYPE params)
{
    GFPS_TRACE("gfps_provider_discoverable_stop\n");

    /* Set BR/EDR discoverability and connectivity. */
    wiced_bt_dev_set_discoverability(BTM_NON_DISCOVERABLE,
                                     BTM_DEFAULT_DISC_WINDOW,
                                     BTM_DEFAULT_DISC_INTERVAL);

    wiced_bt_dev_set_connectability(WICED_FALSE,
                                    BTM_DEFAULT_CONN_WINDOW,
                                    BTM_DEFAULT_CONN_INTERVAL);

    wiced_bt_set_pairable_mode( WICED_FALSE, 0 );

    gfps_provider_advertisement_data_disc_update(WICED_TRUE);

    /* Start the not discoverable advertisement */
    wiced_bt_gfps_provider_advertisement_start(0);
}

static wiced_bool_t gfps_provider_key_aes_calculate(uint8_t *public_key, uint8_t *private_key, uint8_t *aes_key)
{
    //uint8_t i;
    int ret = 0;
    wiced_result_t result = WICED_FALSE;
    uint8_t shared_key[32] = {0};
    uint8_t sha_result[32] = {0};

    ret = fastpair_sec_uecc_shared_secret(public_key, private_key, shared_key);
    if (ret != 1)
    {
        GFPS_TRACE("shared secret error:\n");
        return WICED_FALSE;
    }

    fastpair_sec_sha256(shared_key, 32, sha_result);
    memcpy(aes_key, sha_result, 16);

    return WICED_TRUE;
}

static wiced_bool_t gfps_provider_raw_request_content_check(gfps_raw_request_t *p_raw_request)
{
    wiced_bt_device_address_t local_addr;
    /* Message type */
    if (p_raw_request->message_type != GFPS_RAW_REQ_MSG_TYPE_KEY_BASED_PAIRING_REQ)
    {
        return WICED_FALSE;
    }

    /* Provider's Address */
    memcpy(local_addr, (void *)wiced_btm_get_private_bda(), sizeof(wiced_bt_device_address_t));
    GFPS_TRACE("Current RPA: %B\n", local_addr);

    if (memcmp(p_raw_request->provider_addr, local_addr, sizeof(wiced_bt_device_address_t)) != 0)
    {
        wiced_bt_dev_read_local_addr(local_addr);
        GFPS_TRACE("Local BT Addr: %B\n", local_addr);

        if (memcmp(p_raw_request->provider_addr, local_addr, sizeof(wiced_bt_device_address_t)) != 0)
        {
            return WICED_FALSE;
        }
    }

    return WICED_TRUE;
}

/*
 * Acquire current BLE Advertisement Data used for Fast Pair Service
 *
 * Note that the Advertisement Data used for Google Fast Pair Service is different between
 * discoverable and not discoverable modes.
 *
 *
 */
uint8_t wiced_bt_gfps_provider_advertisement_data_get(wiced_bt_ble_advert_elem_t **p_elem)
{
    if (gfps_provider_cb.discoverable)
    {   // currently work in discoverable mode
        *p_elem = gfps_provider_cb.adv_data.discoverable.p_elem;
        return gfps_provider_cb.adv_data.discoverable.elem_num;
    }
    else
    {   // currently work in no discoverable mode
        *p_elem = gfps_provider_cb.adv_data.not_discoverable.p_elem;
        return gfps_provider_cb.adv_data.not_discoverable.elem_num;
    }
}

/*
 * Update user appended BLE advertisement data
 */
wiced_bool_t wiced_bt_gfps_provider_advertisement_data_appended_data_update(wiced_bt_ble_advert_elem_t *p_elem, uint8_t elem_num)
{
    /* Free current advertisement data. */
    if (gfps_provider_cb.adv_data.discoverable.p_elem)
    {
        wiced_memory_free((void *) gfps_provider_cb.adv_data.discoverable.p_elem);
        gfps_provider_cb.adv_data.discoverable.p_elem = NULL;
    }

    if (gfps_provider_cb.adv_data.not_discoverable.p_elem)
    {
        wiced_memory_free((void *) gfps_provider_cb.adv_data.not_discoverable.p_elem);
        gfps_provider_cb.adv_data.not_discoverable.p_elem = NULL;
    }

    /* Update configuration. */
    gfps_provider_cb.conf.appended_adv_data.p_elem = p_elem;
    gfps_provider_cb.conf.appended_adv_data.elem_num = elem_num;

    /* Update advertisement data. */
    if (gfps_provider_advertisement_data_init() == WICED_FALSE)
    {
        return WICED_FALSE;
    }

    /* Set advertisement data. */
    if (gfps_provider_cb.discoverable)
    {
        wiced_bt_ble_set_raw_advertisement_data(gfps_provider_cb.adv_data.discoverable.elem_num,
                                                gfps_provider_cb.adv_data.discoverable.p_elem);
    }
    else
    {
        wiced_bt_ble_set_raw_advertisement_data(gfps_provider_cb.adv_data.not_discoverable.elem_num,
                                                gfps_provider_cb.adv_data.not_discoverable.p_elem);
    }

    return WICED_TRUE;
}

/*
 * Update BLE advertisement data
 *
 * Note: This utility ONLY updates the advertisement data which is set by configuration while
 *       initializing the GFPS provider module.
 *       If user application wants to substitute all the appended advertisement data set by
 *       configuration while initializing the GFPS module, user shall use
 *       wiced_bt_gfps_provider_advertisement_data_appended_data_update().
 *
 */
void wiced_bt_gfps_provider_advertisement_data_update(wiced_bt_ble_advert_type_t advert_type, uint16_t data_len, uint8_t *p_data)
{
    wiced_bt_ble_advert_elem_t *p_elem = NULL;
    uint8_t i = 0;

    if (p_data == NULL)
        return;

    if (advert_type == BTM_BLE_ADVERT_TYPE_TX_POWER)
    {
        if (data_len != sizeof(int8_t))
        {
            return;
        }

        gfps_provider_cb.conf.ble_tx_pwr_level = (int8_t) *p_data;

        goto WICED_BT_GFPS_PROVIDER_ADVERTISEMENT_DATA_UPDATE_SET;
    }

    /* Discoverable advertisement user appended data. */
    for (i = 0 ; i < gfps_provider_cb.conf.appended_adv_data.elem_num ; i++)
    {
        p_elem = gfps_provider_cb.adv_data.discoverable.p_elem +
                 GFPS_PROVIDER_ADV_DATA_DISC_ELEM_IDX_MAX +
                 i;

        if (p_elem->advert_type == advert_type)
        {
            p_elem->len = data_len;
            p_elem->p_data = p_data;
        }
    }

    /* Not discoverable advertisement user appended data. */
    for (i = 0 ; i < gfps_provider_cb.conf.appended_adv_data.elem_num ; i++)
    {
        p_elem = gfps_provider_cb.adv_data.not_discoverable.p_elem +
                 GFPS_PROVIDER_ADV_DATA_NOT_DISC_ELEM_IDX_MAX +
                 i;

        if (p_elem->advert_type == advert_type)
        {
            p_elem->len = data_len;
            p_elem->p_data = p_data;
        }
    }

WICED_BT_GFPS_PROVIDER_ADVERTISEMENT_DATA_UPDATE_SET:

    if (gfps_provider_cb.discoverable)
    {
        wiced_bt_ble_set_raw_advertisement_data(gfps_provider_cb.adv_data.discoverable.elem_num,
                                                gfps_provider_cb.adv_data.discoverable.p_elem);
    }
    else
    {
        wiced_bt_ble_set_raw_advertisement_data(gfps_provider_cb.adv_data.not_discoverable.elem_num,
                                                gfps_provider_cb.adv_data.not_discoverable.p_elem);
    }
}

/*
 * Set current Google Fast Pair discoverabiltiy
 */
void wiced_bt_gfps_provider_discoverablility_set(wiced_bool_t discoverable)
{
    GFPS_TRACE("wiced_bt_gfps_provider_discoverablility_set (discoverable: %d)\n", discoverable);

    if (gfps_provider_cb.enabled)
    {
        wiced_bt_gfps_provider_advertisement_start((uint8_t) discoverable);
    }
    else
    {
        gfps_provider_cb.discoverable = discoverable;
    }
}

/*
 * Acquire current GFPS pairing state
 */
wiced_bool_t wiced_bt_gfps_provider_pairing_state_get(void)
{
    return gfps_provider_cb.pairing_started;
}

/*
 * Get current account key list
 */
wiced_bool_t wiced_bt_gfps_provider_account_key_list_get(uint8_t *p_data)
{
    uint8_t *p_index = p_data;
    uint8_t i;

    /* Check parameter. */
    if (p_data == NULL)
    {
        return WICED_FALSE;
    }

    /* Fill data. */
    memcpy((void *) p_data,
           (void *) gfps_provider_cb.p_account_key,
           gfps_provider_cb.conf.account_key_list_size * sizeof(wiced_bt_gfps_account_key_t));

    return WICED_TRUE;
}

/*
 * Update the GFPS Provider's Account Key list database
 */
wiced_bool_t wiced_bt_gfps_provider_account_key_list_update(wiced_bt_gfps_account_key_t *p_account_key_list)
{
    wiced_result_t status;
    uint16_t nb_bytes;

    /* Check parameter. */
    if (p_account_key_list == NULL)
    {
        return WICED_FALSE;
    }

    /* Compare database. */
    if (memcmp((void *) gfps_provider_cb.p_account_key,
               (void *) p_account_key_list,
               gfps_provider_cb.conf.account_key_list_size * sizeof(wiced_bt_gfps_account_key_t)) == 0)
    {
        return WICED_TRUE;
    }

    /* Update Account Key list. */
    memcpy((void *) gfps_provider_cb.p_account_key,
           (void *) p_account_key_list,
           gfps_provider_cb.conf.account_key_list_size * sizeof(wiced_bt_gfps_account_key_t));

    /* Write to NVRAM. */
    nb_bytes = wiced_hal_write_nvram(gfps_provider_cb.conf.account_key_list_nvram_id,
                                     gfps_provider_cb.conf.account_key_list_size * sizeof(wiced_bt_gfps_account_key_t),
                                     (uint8_t *) gfps_provider_cb.p_account_key,
                                     &status);

    if ((nb_bytes == (gfps_provider_cb.conf.account_key_list_size * sizeof(wiced_bt_gfps_account_key_t))) &&
        (status == WICED_BT_SUCCESS))
    {
        GFPS_TRACE("wiced_bt_gfps_provider_account_key_list_update success\n");
    }
    else
    {
        GFPS_TRACE("wiced_bt_gfps_provider_account_key_list_update fail\n");
        return WICED_FALSE;
    }

    /* Update the not discoverable advertisement data content. */
    gfps_provider_advertisement_data_not_disc_update();

    return WICED_TRUE;
}

/*
 * Get current GFPS provider's discoverability
 */
wiced_bool_t wiced_bt_gfps_provider_discoverability_get(void)
{
    return gfps_provider_cb.discoverable;
}

/*
 * Enable Google Fast Pair Service provider module.
 *
 * The GFPS provider module is enabled in default by calling wiced_bt_gfps_provider_init()
 * utility to initiate the provider module.
 * That is, user do NOT need to call this utility again to enable the provider module unless
 * user called the wiced_bt_gfps_provider_disable() utility in previous operation to disable
 * the provider module for some reasons.
 *
 * By calling this utility, the provider module will:
 * 1. Update the BLE advertisement data (including the discoverable and the not discoverable data).
 * 2. Start the BLE advertisement again
 *
 * Note:
 * 1. The BLE advertisement will be set to not-discoverable mode. User application shall
 *    use wiced_bt_gfps_provider_discoverability_set() to switch to discoverable mode if
 *    application wants to switch to discoverable mode after enabling.
 */
void wiced_bt_gfps_provider_enable(void)
{
    if (gfps_provider_cb.enabled)
    {
        return;
    }

    /* Update the BLE advertisement data. */
    // Update the random salt number.
    if (gfps_provider_cb.conf.account_key_filter_generate_random)
    {
        gfps_provider_cb.random_salt = wiced_hal_rand_gen_num() % 0xff;
    }

    // update advertisement data and set current advertisement data to controller
    wiced_bt_gfps_provider_advertisement_data_appended_data_update(gfps_provider_cb.conf.appended_adv_data.p_elem,
                                                                   gfps_provider_cb.conf.appended_adv_data.elem_num);

    /* Start timer. */
    // Random Salt update timer
    if (gfps_provider_cb.conf.account_key_filter_generate_random)
    {
        wiced_start_timer(&gfps_provider_cb.random_salt_update_timer,
                          GFPS_PROVIDER_RANDOM_SALT_UPDATE_TIME);
    }

    /* Start the BLE advertisement. */
    wiced_bt_gfps_provider_advertisement_start((uint8_t) gfps_provider_cb.discoverable);

    /* Set the enabled flag. */
    gfps_provider_cb.enabled = WICED_TRUE;
#if 0
    uint16_t nb_bytes;
    wiced_result_t status;

    GFPS_TRACE("wiced_bt_gfps_provider_init\n");

    /* Init. the decrypt failure reset timer. */
    wiced_init_timer(&gfps_provider_cb.decrypt_failure_reset_timer,
                     &gfps_provider_decrypt_failure_count_reset,
                     0,
                     WICED_SECONDS_TIMER);

    /* Init. the discoverability timer. */
    wiced_init_timer(&gfps_provider_cb.discoverable_timer,
                     &gfps_provider_discoverable_stop,
                     0,
                     WICED_SECONDS_TIMER);

    /* Init. advertisement data. */
    if (gfps_provider_advertisement_data_init() == WICED_FALSE)
    {
        GFPS_TRACE("Fail to init. advertisement data.\n");
        goto WICED_BT_GFPS_PROVIDER_INIT_FAIL;
    }

    /* GATT registration */
    if (wiced_bt_gatt_register(&gfps_provider_gatt_event_callback) != WICED_BT_GATT_SUCCESS)
    {
        goto WICED_BT_GFPS_PROVIDER_INIT_FAIL;
    }



    return WICED_TRUE;

WICED_BT_GFPS_PROVIDER_INIT_FAIL:

    if (gfps_provider_cb.p_account_key)
    {
        wiced_memory_free((void *) gfps_provider_cb.p_account_key);
        gfps_provider_cb.p_account_key = NULL;
    }

    return WICED_FALSE;
#endif
}

/*
 * Disable Google Fast Pair Service provider module.
 *
 * By calling this utility, the following capabilities will be terminated:
 * 1. BLE advertisement
 * 2. Existent BLE connection for Google Fast Pair Service
 */
void wiced_bt_gfps_provider_disable(void)
{
    if (gfps_provider_cb.enabled == WICED_FALSE)
    {
        return;
    }

    /* Stop BLE advertisement. */
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

    /* Terminate BLE connection with GFPS Seeker. */
    if (gfps_provider_cb.conn_id)
    {
        wiced_bt_gatt_disconnect(gfps_provider_cb.conn_id);
    }

    /* Stop timers. */
    // Random Salt update timer
    if (gfps_provider_cb.conf.account_key_filter_generate_random)
    {
        if (wiced_is_timer_in_use(&gfps_provider_cb.random_salt_update_timer))
        {
            wiced_stop_timer(&gfps_provider_cb.random_salt_update_timer);
        }
    }

    // decrypt failure reset timer
    if (wiced_is_timer_in_use(&gfps_provider_cb.decrypt_failure_reset_timer))
    {
        wiced_stop_timer(&gfps_provider_cb.decrypt_failure_reset_timer);
    }

    // discoverability timer
    if (wiced_is_timer_in_use(&gfps_provider_cb.discoverable_timer))
    {
        wiced_stop_timer(&gfps_provider_cb.discoverable_timer);
    }

    /* Set flag to bypass the GATT events.
     * Note here we don't de-register the GATT callback since the GFPS Provider service
     * cares only the BLE connection with Seeker. The other BLE connections shall be handled
     * by application or other modules those are enabled by application. */
    gfps_provider_cb.enabled = WICED_FALSE;
}

static void gfps_provider_set_br_edr_discoverable(uint8_t second, wiced_bool_t force)
{
    GFPS_TRACE("set_br_edr_discoverable %d seconds\n", second);

    if ((force == WICED_FALSE) && wiced_is_timer_in_use(&gfps_provider_cb.discoverable_timer))
    {
        return;
    }

    wiced_stop_timer(&gfps_provider_cb.discoverable_timer);

    /* Set BR/EDR discoverability and connectivity. */
    wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE,
                                     BTM_DEFAULT_DISC_WINDOW,
                                     BTM_DEFAULT_DISC_INTERVAL);

    wiced_bt_dev_set_connectability(WICED_TRUE,
                                    BTM_DEFAULT_CONN_WINDOW,
                                    BTM_DEFAULT_CONN_INTERVAL);

    wiced_bt_set_pairable_mode( WICED_TRUE, 0 );

    /* Set the discoverable advertisement data without Fast Pair Model Id. */
    gfps_provider_advertisement_data_disc_update(WICED_FALSE);

    /* Start the discoverable advertisement. */
    wiced_bt_gfps_provider_advertisement_start(1);

    /* Start timer to disable the discoverablity after timeout. */
    wiced_start_timer(&gfps_provider_cb.discoverable_timer, second);
}
