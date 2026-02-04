/***************************************************************************//**
* \file <wiced_bt_pbc_api.h>
*
* \brief
* 	Contains Phone Book Access Client APIs and definitions.
*
*//*****************************************************************************
* Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
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
*******************************************************************************/
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced_bt_types.h"
#include "wiced_result.h"
#include "bt_types.h"

/**
 * @addtogroup  wiced_bt_pbc_api_functions        PBC Client API
 * @ingroup     wicedbt
 *
 * Phone Book Access Client library provides a simple method for an application
 * to integrate PBAP functionality. Application can connect to PBAP server, browse phone books,
 * and retrieve contact information.
 *
 * @{
 */


/*****************************************************************************
**  Constants and data types
*****************************************************************************/

/**************************
**  Client Definitions
***************************/

#ifndef WICED_BT_PBC_DEBUG
#define WICED_BT_PBC_DEBUG          FALSE
#endif

#define WICED_BT_PBC_FLAG_NONE       0
#define WICED_BT_PBC_FLAG_BACKUP     1

typedef uint8_t wiced_bt_pbc_flag_t;


#define WICED_BT_PBC_PASSWORD       "0000"
#define WICED_BT_PBC_AUTH_DIGEST_SIZE 16
#define WICED_BT_PBC_AUTH_FIELD_SIZE 27



/* Client supported feature bits */
#define WICED_BT_PBC_SUP_FEA_DOWNLOADING                         0x00000001      /* Downloading */
#define WICED_BT_PBC_SUP_FEA_BROWSING                            0x00000002      /* Browsing */
#define WICED_BT_PBC_SUP_FEA_DATABASE_ID                         0x00000004      /* Database identifier */
#define WICED_BT_PBC_SUP_FEA_FOLDER_VER_COUNTER                  0x00000008      /* Folder version counter */
#define WICED_BT_PBC_SUP_FEA_VCARD_SELECTING                     0x00000010      /* Vcard selecting */
#define WICED_BT_PBC_SUP_FEA_ENH_MISSED_CALLS                    0x00000020      /* Enhanced missed calls */
#define WICED_BT_PBC_SUP_FEA_UCI_VCARD_FIELD                     0x00000040      /* UCI Vcard field */
#define WICED_BT_PBC_SUP_FEA_UID_VCARD_FIELD                     0x00000080      /* UID Vcard field */
#define WICED_BT_PBC_SUP_FEA_CONTACT_REF                         0x00000100      /* Contact Referencing */
#define WICED_BT_PBC_SUP_FEA_DEF_CONTACT_IMAGE_FORMAT            0x00000200      /* Default contact image format */

typedef unsigned int wiced_bt_pbc_sup_fea_mask_t;

#define WICED_BT_PBC_FILTER_VERSION              ((UINT64)1<<0)  /* Version */
#define WICED_BT_PBC_FILTER_FN                   ((UINT64)1<<1)  /* Formatted Name */
#define WICED_BT_PBC_FILTER_N                    ((UINT64)1<<2)  /* Structured Presentation of Name */
#define WICED_BT_PBC_FILTER_PHOTO                ((UINT64)1<<3)  /* Associated Image or Photo */
#define WICED_BT_PBC_FILTER_BDAY                 ((UINT64)1<<4)  /* Birthday */
#define WICED_BT_PBC_FILTER_ADR                  ((UINT64)1<<5)  /* Delivery Address */
#define WICED_BT_PBC_FILTER_LABEL                ((UINT64)1<<6)  /* Delivery */
#define WICED_BT_PBC_FILTER_TEL                  ((UINT64)1<<7)  /* Telephone Number */
#define WICED_BT_PBC_FILTER_EMAIL                ((UINT64)1<<8)  /* Electronic Mail Address */
#define WICED_BT_PBC_FILTER_MAILER               ((UINT64)1<<9)  /* Electronic Mail */
#define WICED_BT_PBC_FILTER_TZ                   ((UINT64)1<<10)  /* Time Zone */
#define WICED_BT_PBC_FILTER_GEO                  ((UINT64)1<<11) /* Geographic Position */
#define WICED_BT_PBC_FILTER_TITLE                ((UINT64)1<<12) /* Job */
#define WICED_BT_PBC_FILTER_ROLE                 ((UINT64)1<<13) /* Role within the Organization */
#define WICED_BT_PBC_FILTER_LOGO                 ((UINT64)1<<14) /* Organization Logo */
#define WICED_BT_PBC_FILTER_AGENT                ((UINT64)1<<15) /* vCard of Person Representing */
#define WICED_BT_PBC_FILTER_ORG                  ((UINT64)1<<16) /* Name of Organization */
#define WICED_BT_PBC_FILTER_NOTE                 ((UINT64)1<<17) /* Comments */
#define WICED_BT_PBC_FILTER_REV                  ((UINT64)1<<18) /* Revision */
#define WICED_BT_PBC_FILTER_SOUND                ((UINT64)1<<19) /* Pronunciation of Name */
#define WICED_BT_PBC_FILTER_URL                  ((UINT64)1<<20) /* Uniform Resource Locator */
#define WICED_BT_PBC_FILTER_UID                  ((UINT64)1<<21) /* Unique ID */
#define WICED_BT_PBC_FILTER_KEY                  ((UINT64)1<<22) /* Public Encryption Key */
#define WICED_BT_PBC_FILTER_NICKNAME             ((UINT64)1<<23) /* Nickname */
#define WICED_BT_PBC_FILTER_CATEGORIES           ((UINT64)1<<24) /* Categories */
#define WICED_BT_PBC_FILTER_PROID                ((UINT64)1<<25) /* Product ID */
#define WICED_BT_PBC_FILTER_CLASS                ((UINT64)1<<26) /* Class information */
#define WICED_BT_PBC_FILTER_SORT_STRING          ((UINT64)1<<27) /* String used for sorting operation */
#define WICED_BT_PBC_FILTER_CALL_DATETIME        ((UINT64)1<<28) /* Time stamp */
#define WICED_BT_PBC_FILTER_X_BT_SPEEDDIALKEY    ((UINT64)1<<29) /* Speed-dial shortcut */
#define WICED_BT_PBC_FILTER_X_BT_UCI             ((UINT64)1<<30) /* Uniform Caller Identifier field */
#define WICED_BT_PBC_FILTER_X_BT_UID             ((UINT64)1<<31) /* Bluetooth Contact Unique Identifier */
#define WICED_BT_PBC_FILTER_ALL      (0)
typedef unsigned long long wiced_bt_pbc_filter_mask_t;

/* Profile supported repositories */
#define WICED_BT_PBC_REPOSIT_LOCAL      0x01    /* Local PhoneBook */
#define WICED_BT_PBC_REPOSIT_SIM        0x02    /* SIM card PhoneBook */
#define WICED_BT_PBC_REPOSIT_SPEED_DIAL 0x04    /* Speed Dial */
#define WICED_BT_PBC_REPOSIT_FAVORITES  0x08    /* Favorites */

typedef uint8_t wiced_bt_pbc_sup_reposit_mask_t;

enum
{
    WICED_BT_PBC_FORMAT_CARD_21, /* vCard format 2.1 */
    WICED_BT_PBC_FORMAT_CARD_30, /* vCard format 3.0 */
    WICED_BT_PBC_FORMAT_MAX
};
typedef uint8_t wiced_bt_pbc_format_t;

enum
{
    WICED_BT_PBC_ORDER_INDEXED = 0,  /* indexed */
    WICED_BT_PBC_ORDER_ALPHANUM,     /* alphanumeric */
    WICED_BT_PBC_ORDER_PHONETIC,      /* phonetic */
    WICED_BT_PBC_ORDER_MAX
};
typedef uint8_t wiced_bt_pbc_order_t;

enum
{
    WICED_BT_PBC_ATTR_NAME = 0,      /* name */
    WICED_BT_PBC_ATTR_NUMBER,        /* number */
    WICED_BT_PBC_ATTR_SOUND,         /* sound */
    WICED_BT_PBC_ATTR_MAX
};
typedef uint8_t wiced_bt_pbc_attr_t;


/* Client callback function events */
#define WICED_BT_PBC_ENABLE_EVT      0   /* Phone Book Access client is enabled. */
#define WICED_BT_PBC_OPEN_EVT        1   /* Connection to peer is open. */
#define WICED_BT_PBC_CLOSE_EVT       2   /* Connection to peer closed. */
#define WICED_BT_PBC_AUTH_EVT        3   /* Request for Authentication key and user id */
#define WICED_BT_PBC_LIST_EVT        4   /* Event contains a directory entry (wiced_bt_PBC_LIST) */
#define WICED_BT_PBC_PROGRESS_EVT    5   /* Number of bytes read or written so far */
#define WICED_BT_PBC_GETFILE_EVT     6   /* Get complete */
#define WICED_BT_PBC_CHDIR_EVT       7   /* Change Directory complete */
#define WICED_BT_PBC_PHONEBOOK_EVT   8   /* Report the Application Parameters for wiced_bt_pbcGetPhoneBook response */
#define WICED_BT_PBC_DISABLE_EVT     9   /* Phone Book Access client is disabled. */

typedef uint8_t wiced_bt_pbc_evt_t;

/* Client callback function event data */

#define WICED_BT_PBC_OK              0
#define WICED_BT_PBC_FAIL            1
#define WICED_BT_PBC_NO_PERMISSION   2
#define WICED_BT_PBC_NOT_FOUND       3
#define WICED_BT_PBC_FULL            4
#define WICED_BT_PBC_BUSY            5
#define WICED_BT_PBC_ABORTED         6
#define WICED_BT_PBC_PRECONDITION_FAILED     7

typedef uint8_t wiced_bt_pbc_status_t;

typedef uint8_t wiced_bt_service_id_t;

typedef struct
{
    /* @deprecated : This value will not be set by the profile.
        Connection is open with PBAP */
    wiced_bt_service_id_t service;
    wiced_bt_pbc_sup_fea_mask_t   peer_features;      /* Peer supported features */
    wiced_bt_pbc_sup_reposit_mask_t   peer_repositories;  /* Peer supported repositories */
} wiced_bt_pbc_open_t;

typedef struct
{
    uint16_t          phone_book_size;
    wiced_bool_t         pbs_exist;          /* phone_book_size is present in the response */
    uint8_t           new_missed_calls;
    wiced_bool_t         nmc_exist;          /* new_missed_calls is present in the response */
} wiced_bt_pbc_pb_param_t;

typedef struct
{
    wiced_bt_pbc_pb_param_t *p_param;
    uint8_t           *data;
    uint16_t           len;
    wiced_bool_t          final;     /* If TRUE, entry is last of the series */
    wiced_bt_pbc_status_t  status;    /* Fields are valid when status is WICED_BT_PBC_OK */
} wiced_bt_pbc_list_t;

typedef struct
{
    UINT32 file_size;   /* Total size of file (WICED_BT_PBC_LEN_UNKNOWN if unknown) */
    uint16_t bytes;       /* Number of bytes read or written since last progress event */
} wiced_bt_pbc_progress_t;

typedef struct
{
    uint8_t  *p_realm;
    uint8_t   realm_len;
    uint8_t   realm_charset;
    wiced_bool_t userid_required;    /* If TRUE, a user ID must be sent */
} wiced_bt_pbc_auth_t;


typedef union
{
    wiced_bt_pbc_status_t     status;
    wiced_bt_pbc_open_t       open;
    wiced_bt_pbc_list_t       list;
    wiced_bt_pbc_progress_t   prog;
    wiced_bt_pbc_auth_t       auth;
    wiced_bt_pbc_pb_param_t   pb;
} wiced_bt_pbc_t;

/**
 * @typedef wiced_bt_pbc_cback_t
 * @details PBC Client callback function type. The application implements a callback of this type to receive PBC control events.
 * @param[in] event   ID of event being notified to app.
 * @param[in] p_data  Pointer to data associated with the event.
 * @return None.
 */
typedef void wiced_bt_pbc_cback_t(wiced_bt_pbc_evt_t event, wiced_bt_pbc_t *p_data);

/**
 * @typedef wiced_bt_pbc_data_cback_t
 * @details PBC Client data callback function type. The application implements a callback of this type to receive PBC data.
 * @param[in] p_buf  Pointer to the data buffer.
 * @param[in] nbytes Number of bytes in the data buffer.
 * @return None.
 */
typedef void wiced_bt_pbc_data_cback_t(const uint8_t *p_buf, uint16_t nbytes);

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/


/**************************
**  Client Functions
***************************/

/**
 * @brief Enable the phone book access client.
 * @details This function must be called before any other functions in the PBC API are called.
 *          When the enable operation is complete the callback function
 *          will be called with an WICED_BT_PBC_ENABLE_EVT event.
 * @param[in] p_cback       Callback function for receiving PBC events.
 * @param[in] p_data_cback  Callback function for receiving PBC data.
 * @param[in] app_id        Application ID.
 * @param[in] local_features Local supported features mask.
 * @return None.
 */
extern void wiced_bt_pbc_op_enable(wiced_bt_pbc_cback_t *p_cback,
                                    wiced_bt_pbc_data_cback_t *p_data_cback,
                                    uint8_t app_id,
                                    wiced_bt_pbc_sup_fea_mask_t local_features);
/**
 * @brief Disable the phone book access client.
 * @details If the client is currently connected to a peer device the connection will be closed.
 * @return None.
 */
extern void wiced_bt_pbc_op_disable(void);

/**
 * @brief Open a connection to an PBAP server.
 * @details When the connection is open the callback function
 *          will be called with a WICED_BT_PBC_OPEN_EVT. If the connection
 *          fails or otherwise is closed the callback function will be
 *          called with a WICED_BT_PBC_CLOSE_EVT.
 *          Note: Pbc always enable (BTA_SEC_AUTHENTICATE | BTA_SEC_ENCRYPT)
 * @param[in] bd_addr   Bluetooth device address of the PBAP server.
 * @param[in] sec_mask  Security mask.
 * @return None.
 */
extern void wiced_bt_pbc_op_open(wiced_bt_device_address_t bd_addr, uint8_t sec_mask);

/**
 * @brief Close the current connection to the server.
 * @details Aborts any active PBAP transfer.
 * @return None.
 */
extern void wiced_bt_pbc_op_close(void);


/**
 * @brief Retrieve a PhoneBook from the peer device and copy it to the local file system.
 * @details This function can only be used when the client is connected in PBAP mode.
 *          Local file name is specified with a fully qualified path.
 *          Remote file name is absolute path in UTF-8 format
 *          (telecom/pb.vcf or SIM1/telecom/pb.vcf).
 * @param[in] p_local_name        Local file name with fully qualified path.
 * @param[in] p_remote_name       Remote file name in absolute path UTF-8 format.
 * @param[in] filter              Filter mask for vCard attributes.
 * @param[in] format              vCard format (2.1 or 3.0).
 * @param[in] max_list_count      Maximum number of entries to retrieve.
 * @param[in] list_start_offset   Starting offset for list retrieval.
 * @param[in] is_reset_miss_calls Flag to reset missed calls counter.
 * @param[in] selector            Selector filter mask.
 * @param[in] selector_op         Selector operation.
 * @return None.
 */
extern void wiced_bt_pbc_op_getphonebook(char *p_local_name, char *p_remote_name,
                         wiced_bt_pbc_filter_mask_t filter, wiced_bt_pbc_format_t format,
                         uint16_t max_list_count, uint16_t list_start_offset,
                         wiced_bool_t is_reset_miss_calls, wiced_bt_pbc_filter_mask_t selector,
                         uint8_t selector_op);

/**
 * @brief Retrieve a vCard from the peer device and copy it to the local file system.
 * @details This function can only be used when the client is connected in PBAP mode.
 *          Local file name is specified with a fully qualified path.
 *          Remote file name is relative path in UTF-8 format.
 * @param[in] p_local_name  Local file name with fully qualified path.
 * @param[in] p_remote_name Remote file name in relative path UTF-8 format.
 * @param[in] filter        Filter mask for vCard attributes.
 * @param[in] format        vCard format (2.1 or 3.0).
 * @return None.
 */
extern void wiced_bt_pbc_op_getcard(char *p_local_name, char *p_remote_name,
                    wiced_bt_pbc_filter_mask_t filter, wiced_bt_pbc_format_t format);


/**
 * @brief Change PB path on the peer device.
 * @details This function can only be used when the client is connected in PBAP mode.
 * @param[in] p_dir Directory path to change to.
 * @param[in] flag  Change directory flags.
 * @return None.
 */
extern void wiced_bt_pbc_op_chdir(char *p_dir, wiced_bt_pbc_flag_t flag);

/**
 * @brief Sends a response to an OBEX authentication challenge to the connected OBEX server.
 * @details Called in response to an WICED_BT_PBC_AUTH_EVT event.
 *          If the "userid_required" is TRUE in the WICED_BT_PBC_AUTH_EVT event,
 *          then p_userid is required, otherwise it is optional.
 *          p_password must be less than WICED_BT_PBC_MAX_AUTH_KEY_SIZE (16 bytes)
 *          p_userid must be less than WICED_BT_OBX_MAX_REALM_LEN (defined in target.h)
 * @param[in] p_password Authentication password string.
 * @param[in] p_userid   User ID string (optional based on auth event).
 * @return None.
 */
extern void wiced_bt_pbc_op_authrsp (char *p_password, char *p_userid);

/**
 * @brief Retrieve a directory listing from the peer device.
 * @details When the operation is complete the callback function will
 *          be called with one or more WICED_BT_PBC_LIST_EVT events
 *          containing directory list information formatted as described
 *          in the PBAP Spec, Version 0.9, section 3.1.6.
 *          This function can only be used when the client is connected
 *          to a peer device in PBAP mode.
 * @param[in] p_dir               Name of directory to retrieve listing of.
 * @param[in] order               Sort order for the listing.
 * @param[in] p_value             Search value string.
 * @param[in] attribute           Search attribute type.
 * @param[in] max_list_count      Maximum number of entries to retrieve.
 * @param[in] list_start_offset   Starting offset for list retrieval.
 * @param[in] is_reset_miss_calls Flag to reset missed calls counter.
 * @param[in] selector            Selector filter mask.
 * @param[in] selector_op         Selector operation.
 * @return None.
 */
extern void wiced_bt_pbc_op_listcards(char *p_dir, wiced_bt_pbc_order_t order, char *p_value,
                      wiced_bt_pbc_attr_t attribute, uint16_t max_list_count,
                      uint16_t list_start_offset, wiced_bool_t is_reset_miss_calls,
                      wiced_bt_pbc_filter_mask_t selector, uint8_t selector_op);


/**
 * @brief Aborts any active PBC operation.
 * @details This function cancels any ongoing PBAP operation.
 * @return None.
 */
extern void wiced_bt_pbc_op_abort(void);

/**@} wiced_bt_pbc_api_functions */

#ifdef __cplusplus
}
#endif
