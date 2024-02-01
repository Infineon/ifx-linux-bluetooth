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

#define WICED_BT_GA_CSIS_SET_IDENTITY_RESOLVING_KEY_LEN 16
#define WICED_BT_GA_CSIS_PRIVATE_SET_RANDOM_IDENTIFIER_LEN 6

typedef uint8_t wiced_bt_ga_csis_sirk_t[WICED_BT_GA_CSIS_SET_IDENTITY_RESOLVING_KEY_LEN]; /**< Device address length */
typedef uint8_t wiced_bt_ga_csis_psri_t[WICED_BT_GA_CSIS_PRIVATE_SET_RANDOM_IDENTIFIER_LEN]; /**< Device address length */

#define SIRK wiced_bt_ga_csis_sirk_t
#define PSRI wiced_bt_ga_csis_psri_t

/**
 * @anchor LOCK_VALUE
 * @name Definition for lock values which can be set by application
 * @{ */
typedef enum
{
    WICED_BT_GA_CSIS_UNLOCKED = 1, /**< Lock value unlocked*/
    WICED_BT_GA_CSIS_LOCKED = 2,   /**< Lock value locked */
} wiced_bt_ga_csis_lock_val_t;
/** @} LOCK_VALUE */

/**
 * @anchor SIRK_TYPE
 * @name Definition for SIRK types which can be set by application
 * @{ */
typedef enum
{
    WICED_BT_GA_CSIS_SIRK_ENCR = 0,  /**< SIRK in encypted format */
    WICED_BT_GA_CSIS_SIRK_PLAIN = 1, /**< SIRK in plain text */
} wiced_bt_ga_csis_sirk_type_t;
/** @} SIRK_TYPE */

/** SIRK Data */
typedef struct
{
    wiced_bool_t                 is_oob;    /**< is SIRK to be obtained via OOB methods */
    wiced_bt_ga_csis_sirk_type_t sirk_type; /**< SIRK type */
    SIRK                         sirk;      /**< SIRK key */
} wiced_bt_ga_csis_sirk_data_t;


/** CSIS Data */
typedef union
{
    wiced_bt_ga_csis_sirk_data_t sirk_data;     /**< set identity resolving key */
    uint8_t                      size;          /**< number of devices in the coordinated set values from 0x02 to 0xFF. Values 0x00 and 0x01 are Prohibited.*/
    uint8_t                      rank;          /**< rank of the device in the coordinated set */
    wiced_bt_ga_csis_lock_val_t  lock_val;      /**< lock value */
    wiced_bt_gatt_status_t       status;        /**< procedure status */
} wiced_bt_ga_csis_data_t;

/**@} wiced_bt_ga_csis */
/**@} Coordinate_Set_APIs */

#ifdef __cplusplus
}
#endif
