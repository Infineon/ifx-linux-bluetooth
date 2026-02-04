/*
 * $ Copyright Cypress Semiconductor $
 */

/** @file
 *
 * Co-ordinate Set Identification Service (CSIS) Application Programming Interface
 */
 
#ifndef __WICED_BT_GA_CSIS_COMMON_H__
#define __WICED_BT_GA_CSIS_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup Coordinate_Set_APIs
 * @{
 */

/**
 * @addtogroup wiced_bt_ga_csis_common
 * @{
 */

#define WICED_BT_GA_CSIS_SET_IDENTITY_RESOLVING_KEY_LEN 16 /**< Length of SIRK */
#define WICED_BT_GA_CSIS_PRIVATE_SET_RANDOM_IDENTIFIER_LEN 6 /**< Length of PSRI */

typedef uint8_t wiced_bt_ga_csis_sirk_t[WICED_BT_GA_CSIS_SET_IDENTITY_RESOLVING_KEY_LEN]; /**< Device address length */
typedef uint8_t wiced_bt_ga_csis_psri_t[WICED_BT_GA_CSIS_PRIVATE_SET_RANDOM_IDENTIFIER_LEN]; /**< Device address length */

#define SIRK wiced_bt_ga_csis_sirk_t /**< SIRK */
#define PSRI wiced_bt_ga_csis_psri_t /**< PSRI */

/**
 * @brief Definition for lock values which can be set by application
 */
enum wiced_bt_ga_csis_lock_val_e
{
    WICED_BT_GA_CSIS_UNLOCKED = 1, /**< Lock value unlocked*/
    WICED_BT_GA_CSIS_LOCKED = 2,   /**< Lock value locked */
};

typedef uint8_t wiced_bt_ga_csis_lock_val_t;  /**< CSIS lock values (see #wiced_bt_ga_csis_lock_val_e) */

/**
 * @brief Definition for SIRK types which can be set by application
 */
enum wiced_bt_ga_csis_sirk_type_e
{
    WICED_BT_GA_CSIS_SIRK_ENCR = 0,  /**< SIRK in encypted format */
    WICED_BT_GA_CSIS_SIRK_PLAIN = 1, /**< SIRK in plain text */
};

typedef uint8_t wiced_bt_ga_csis_sirk_type_t; /**< CSIS SIRK values (see #wiced_bt_ga_csis_sirk_type_e) */


/** @brief SIRK Data */
typedef struct
{
    wiced_bt_ga_csis_sirk_t sirk;           /**< SIRK key */
    wiced_bt_ga_csis_sirk_type_t sirk_type; /**< SIRK type */
    uint8_t is_oob;                    /**< is SIRK to be obtained via OOB methods */
} wiced_bt_ga_csis_sirk_data_t;


/** @brief CSIS Data */
typedef union
{
    wiced_bt_ga_csis_sirk_data_t sirk_data;     /**< set identity resolving key */
    uint8_t                      size;          /**< number of devices in the coordinated set values from 0x02 to 0xFF. Values 0x00 and 0x01 are Prohibited.*/
    uint8_t                      rank;          /**< rank of the device in the coordinated set */
    wiced_bt_ga_csis_lock_val_t  lock_val;      /**< lock value */
    wiced_bt_gatt_status_t       status;        /**< procedure status */
} wiced_bt_ga_csis_data_t;

/**@} wiced_bt_ga_csis_common */
/**@} Coordinate_Set_APIs */

#ifdef __cplusplus
}
#endif

#endif //__WICED_BT_GA_CSIS_COMMON_H__
