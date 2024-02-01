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

/* Application includes */
#include "unicast_sink_gatt.h"
#include "unicast_sink_rpc.h"

/* App Library includes */
#include "gatt_interface.h"
#include "wiced_bt_ga_ascs.h"
#include "wiced_bt_ga_pacs.h"
#include "wiced_bt_ga_vcs.h"
#include "le_audio_rpc.h"

/* BT Stack includes */
#include "wiced_bt_ga_mcp.h"

#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_uuid.h"
#include "wiced_memory.h"
#include "data_types.h"

/*******************************************************************************
*                               MACROS
*******************************************************************************/
#define AD_FLAG_SIZE 2
#define AUDIO_STREAM_CONTROL_SERVICE_SIZE 9
#define ADV_NAME_SIZE 16
#define ADV_SIZE (AD_FLAG_SIZE + 1 + AUDIO_STREAM_CONTROL_SERVICE_SIZE + 1 + ADV_NAME_SIZE + 1) // +1 for length itself

/******************************************************************************
 *                              EXTERNS
 *****************************************************************************/
extern wiced_bt_cfg_ble_t       unicast_sink_ble_cfg;
extern wiced_bt_cfg_settings_t  unicast_sink_cfg_settings;
extern wiced_bt_heap_t          *p_default_heap;

/****************************************************************************
 *                              FUNCTION DECLARATION
 ***************************************************************************/
extern void unicast_sink_menu_clear_info(void);

/*******************************************************************************
*                               STRUCTURES AND ENUMERATIONS
*******************************************************************************/
enum
{
    HDLS_GENERIC_ATTRIBUTE = 1, // 0x0001 , 1

    HDLS_VCS = 16,                              // 0x0010 , 16
    HDLI_VCS_INCLUDED_AICS,                     // 0x0011 , 17
    HDLI_VCS_INCLUDED_VOCS,                     // 0x0012 , 18
    HDLC_VCS_VOLUME_STATE,                      // 0x0013 , 19
    HDLC_VCS_VOLUME_STATE_VALUE,                // 0x0014 , 20
    HDLD_VCS_VOLUME_STATE_CLIENT_CONFIGURATION, // 0x0015 , 21
    HDLC_VCS_VOLUME_CONTROL_POINT_,             // 0x0016 , 22
    HDLC_VCS_VOLUME_CONTROL_POINT__VALUE,       // 0x0017 , 23
    HDLC_VCS_VOLUME_FLAGS,                      // 0x0018 , 24
    HDLC_VCS_VOLUME_FLAGS_VALUE,                // 0x0019 , 25
    HDLD_VCS_VOLUME_FLAGS_CLIENT_CONFIGURATION, // 0x001A , 26

    HDLS_VOCS = 32,                                          // 0x0020 , 32
    HDLC_VOCS_OFFSET_STATE,                                  // 0x0021 , 33
    HDLC_VOCS_OFFSET_STATE_VALUE,                            // 0x0022 , 34
    HDLD_VOCS_OFFSET_STATE_CLIENT_CONFIGURATION,             // 0x0023 , 35
    HDLC_VOCS_AUDIO_LOCATION,                                // 0x0024 , 36
    HDLC_VOCS_AUDIO_LOCATION_VALUE,                          // 0x0025 , 37
    HDLD_VOCS_AUDIO_LOCATION_CLIENT_CONFIGURATION,           // 0x0026 , 38
    HDLC_VOCS_VOLUME_OFFSET_CONTROL_POINT_,                  // 0x0027 , 39
    HDLC_VOCS_VOLUME_OFFSET_CONTROL_POINT__VALUE,            // 0x0028 , 40
    HDLC_VOCS_AUDIO_OUTPUT_DESCRIPTION,                      // 0x0029 , 41
    HDLC_VOCS_AUDIO_OUTPUT_DESCRIPTION_VALUE,                // 0x002A , 42
    HDLD_VOCS_AUDIO_OUTPUT_DESCRIPTION_CLIENT_CONFIGURATION, // 0x002B , 43

    HDLS_VCS_AICS = 48,                                         // 0x0030 , 48
    HDLC_VCS_AICS_INPUT_STATE,                                  // 0x0031 , 49
    HDLC_VCS_AICS_INPUT_STATE_VALUE,                            // 0x0032 , 50
    HDLD_VCS_AICS_INPUT_STATE_CLIENT_CONFIGURATION,             // 0x0033 , 51
    HDLC_VCS_AICS_GAIN_SETTING_ATTR,                            // 0x0034 , 52
    HDLC_VCS_AICS_GAIN_SETTING_ATTR_VALUE,                      // 0x0035 , 53
    HDLC_VCS_AICS_INPUT_TYPE,                                   // 0x0036 , 54
    HDLC_VCS_AICS_INPUT_TYPE_VALUE,                             // 0x0037 , 55
    HDLC_VCS_AICS_INPUT_STATUS,                                 // 0x0038 , 56
    HDLC_VCS_AICS_INPUT_STATUS_VALUE,                           // 0x0039 , 57
    HDLD_VCS_AICS_INPUT_STATUS_CLIENT_CONFIGURATION,            // 0x003A , 58
    HDLC_VCS_AICS_AUDIO_INPUT_CONTROL_POINT_,                   // 0x003B , 59
    HDLC_VCS_AICS_AUDIO_INPUT_CONTROL_POINT__VALUE,             // 0x003C , 60
    HDLC_VCS_AICS_AUDIO_INPUT_DESCRIPTION,                      // 0x003D , 61
    HDLC_VCS_AICS_AUDIO_INPUT_DESCRIPTION_VALUE,                // 0x003E , 62
    HDLD_VCS_AICS_AUDIO_INPUT_DESCRIPTION_CLIENT_CONFIGURATION, // 0x003F , 63

    HDLS_PACS = 208,                          // 0x00D0 , 208
    HDLC_PACS_SINK_PAC,                       // 0x00D1 , 209
    HDLC_PACS_SINK_PAC_VALUE,                 // 0x00D2 , 210
    HDLD_PACS_SINK_PAC_CCCD,                  // 0x00D3 , 211
    HDLC_PACS_SINK_AUDIO_LOCATIONS,           // 0x00D4 , 212
    HDLC_PACS_SINK_AUDIO_LOCATIONS_VALUE,     // 0x00D5 , 213
    HDLC_PACS_SINK_AUDIO_LOCATIONS_CCCD,      // 0x00D6 , 214

    HDLC_PACS_AVAILABLE_AUDIO_CONTEXTS,       // 0x00D7 , 215
    HDLC_PACS_AVAILABLE_AUDIO_CONTEXTS_VALUE, // 0x00D8 , 216
    HDLD_PACS_AVAILABLE_AUDIO_CONTEXTS_CCCD,  // 0x00D9 , 217
    HDLC_PACS_SUPPORTED_AUDIO_CONTEXTS,       // 0x00DA , 218
    HDLC_PACS_SUPPORTED_AUDIO_CONTEXTS_VALUE, // 0x00DB , 219
    HDLC_PACS_SUPPORTED_AUDIO_CONTEXTS_CCCD,  // 0x00DC , 220

    HDLS_ASCS = 240,                   // 0x00F0 , 240
    HDLC_ASCS_ASE_SINK,                // 0x00F1 , 241
    HDLC_ASCS_ASE_SINK_VALUE,          // 0x00F2 , 242
    HDLD_ASCS_ASE_SINK_CCCD,           // 0x00F3 , 243
    HDLC_ASCS_ASE_CONTROL_POINT,       // 0x00F7 , 244
    HDLC_ASCS_ASE_CONTROL_POINT_VALUE, // 0x00F8 , 245
    HDLD_ASCS_ASE_CONTROL_POINT_CCCD,  // 0x00F9 , 246

    HDLS_CAS = 300, // 0x012C , 246
};

/******************************************************************************
*                               GLOBAL VARIABLES
******************************************************************************/
unicast_sink_gatt_cb_t g_unicast_sink_gatt_cb;

const uint8_t unicast_sink_gatt_database[] = {

    /* Primary Service 'Generic Attribute' */
    PRIMARY_SERVICE_UUID16(HDLS_GENERIC_ATTRIBUTE, UUID_SERVICE_GATT),

    /* Primary Service 'vcs' */
    PRIMARY_SERVICE_UUID16(HDLS_VCS, WICED_BT_UUID_VOLUME_CONTROL),

    /* Characteristic 'Volume State' */
    CHARACTERISTIC_UUID16(HDLC_VCS_VOLUME_STATE,
                          HDLC_VCS_VOLUME_STATE_VALUE,
                          WICED_BT_UUID_VOLUME_STATE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_VCS_VOLUME_STATE_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
                                        GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_WRITABLE */),

    /* Characteristic 'Volume Control Point ' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_VCS_VOLUME_CONTROL_POINT_,
                                   HDLC_VCS_VOLUME_CONTROL_POINT__VALUE,
                                   WICED_BT_UUID_CONTROL_POINT,
                                   GATTDB_CHAR_PROP_WRITE,
                                   GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ /*| GATTDB_PERM_WRITABLE */),

    /* Characteristic 'Volume Flags' */
    CHARACTERISTIC_UUID16(HDLC_VCS_VOLUME_FLAGS,
                          HDLC_VCS_VOLUME_FLAGS_VALUE,
                          WICED_BT_UUID_VOLUME_FLAG,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_VCS_VOLUME_FLAGS_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
                                        GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_WRITABLE */),

    PRIMARY_SERVICE_UUID16(HDLS_PACS, WICED_BT_UUID_PUBLISHED_AUDIO_CAPABILITY),

    CHARACTERISTIC_UUID16(HDLC_PACS_SINK_PAC,
                          HDLC_PACS_SINK_PAC_VALUE,
                          WICED_BT_UUID_PACS_SINK_PAC,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_PACS_SINK_PAC_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),

    CHARACTERISTIC_UUID16_WRITABLE(HDLC_PACS_SINK_AUDIO_LOCATIONS,
                                   HDLC_PACS_SINK_AUDIO_LOCATIONS_VALUE,
                                   WICED_BT_UUID_PACS_SINK_AUDIO_LOCATIONS,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLC_PACS_SINK_AUDIO_LOCATIONS_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),
    CHARACTERISTIC_UUID16(HDLC_PACS_AVAILABLE_AUDIO_CONTEXTS,
                          HDLC_PACS_AVAILABLE_AUDIO_CONTEXTS_VALUE,
                          WICED_BT_UUID_PACS_AUDIO_CONTEXT_AVAILABILITY,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_PACS_AVAILABLE_AUDIO_CONTEXTS_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),

    CHARACTERISTIC_UUID16(HDLC_PACS_SUPPORTED_AUDIO_CONTEXTS,
                          HDLC_PACS_SUPPORTED_AUDIO_CONTEXTS_VALUE,
                          WICED_BT_UUID_PACS_SUPPORTED_AUDIO_CONTEXT,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLC_PACS_SUPPORTED_AUDIO_CONTEXTS_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),

    PRIMARY_SERVICE_UUID16(HDLS_ASCS, WICED_BT_UUID_AUDIO_STREAM_CONTROL),

    CHARACTERISTIC_UUID16(HDLC_ASCS_ASE_SINK,
                          HDLC_ASCS_ASE_SINK_VALUE,
                          WICED_BT_UUID_ASCS_SINK_ASE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_ASCS_ASE_SINK_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),

    CHARACTERISTIC_UUID16_WRITABLE(HDLC_ASCS_ASE_CONTROL_POINT,
                                   HDLC_ASCS_ASE_CONTROL_POINT_VALUE,
                                   WICED_BT_UUID_ASCS_ASE_CONTROL_POINT,
                                   GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE |
                                       GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_ASCS_ASE_CONTROL_POINT_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),

    PRIMARY_SERVICE_UUID16(HDLS_CAS, WICED_BT_UUID_COMMON_AUDIO),
};

/******************************************************************************
 * Function Name: unicast_sink_gatt_alloc_cb
 ******************************************************************************
 * Summary: allocate clcb to save connected Source device
 *
 * Parameters:
 *  uint8_t *p_bd_addr
 *  wiced_bt_ble_address_type_t addr_type
 *  uint16_t conn_id
 *  uint16_t link_role
 *
 * Return:
 *  unicast_sink_clcb_t *
 *
******************************************************************************/
unicast_sink_clcb_t *unicast_sink_gatt_alloc_cb(uint8_t *p_bd_addr,
                                                wiced_bt_ble_address_type_t addr_type,
                                                uint16_t conn_id,
                                                uint16_t link_role)
{
    int index;
    unicast_sink_clcb_t *p_clcb = NULL;
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        p_clcb = &g_unicast_sink_gatt_cb.unicast_clcb[index];
        if (p_clcb->in_use == FALSE)
        {
            p_clcb->in_use = TRUE;
            p_clcb->conn_id = conn_id;
            p_clcb->addr_type = addr_type;
            memcpy(p_clcb->bda, p_bd_addr, BD_ADDR_LEN);
            p_clcb->b_is_central = (HCI_ROLE_CENTRAL == link_role) ? TRUE : FALSE;
            return p_clcb;
        }
    }
    return p_clcb;
}

/******************************************************************************
 * Function Name: unicast_sink_gatt_get_clcb
 ******************************************************************************
 * Summary: get the clcb by bd_addr
 *
 * Parameters:
 *  uint8_t *p_bd_addr
 *
 * Return:
 *  unicast_sink_clcb_t *
 *
******************************************************************************/
unicast_sink_clcb_t *unicast_sink_gatt_get_clcb(uint8_t *p_bd_addr)
{
    unicast_sink_clcb_t *p_clcb = NULL;
    int index;
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        p_clcb = &g_unicast_sink_gatt_cb.unicast_clcb[index];
        if (p_clcb->in_use && !WICED_MEMCMP(p_clcb->bda, p_bd_addr, BD_ADDR_LEN))
        {
            return p_clcb;
        }
    }
    return NULL;
}

/******************************************************************************
 * Function Name: unicast_source_gatt_free_cb
 ******************************************************************************
 * Summary: free saved clcb by bd_addr
 *
 * Parameters:
 *  uint8_t *p_bd_addr
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
******************************************************************************/
wiced_bt_gatt_status_t unicast_sink_gatt_free_cb(uint8_t *p_bd_addr)
{
    unicast_sink_clcb_t *p_clcb = unicast_sink_gatt_get_clcb(p_bd_addr);
    if (!p_clcb) return WICED_ERROR;

    p_clcb->in_use = FALSE;
    unicast_sink_menu_clear_info();
    return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_sink_gatt_get_clcb_by_conn_id
 ******************************************************************************
 * Summary:  get the clcb by conn_id
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  unicast_sink_clcb_t *
 *
******************************************************************************/
unicast_sink_clcb_t *unicast_sink_gatt_get_clcb_by_conn_id(uint16_t conn_id)
{
    unicast_sink_clcb_t *p_clcb = NULL;
    int index;
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        p_clcb = &g_unicast_sink_gatt_cb.unicast_clcb[index];
        if (p_clcb->in_use && (p_clcb->conn_id == conn_id))
        {
            return p_clcb;
        }
    }
    return NULL;
}

/******************************************************************************
 * Function Name: unicast_sink_gatt_start_stop_adv
 ******************************************************************************
 * Summary: start or stop adv
 *
 * Parameters:
 *  uint32_t b_start
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_gatt_start_stop_adv(uint32_t b_start)
{
    wiced_result_t gatt_status;

    if (wiced_bt_ble_is_ext_adv_supported())
    {

        wiced_bt_ble_ext_adv_duration_config_t duration_cfg;
        uint8_t ascs_data[] = {1, 4, 0, 0, 0, 0};
        wiced_bt_device_address_t null_addr = {0};
        uint8_t data[ADV_SIZE] = {0};
        uint8_t *p_ext_adv_data = data;
        wiced_bt_dev_status_t sts;
        uint8_t addr_type =
            (unicast_sink_cfg_settings.p_ble_cfg->rpa_refresh_timeout) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;

        WICED_BT_TRACE("[%s]\n", __FUNCTION__);

        // Set ext adv params
        if (b_start)
        {
            wiced_bt_ble_set_ext_adv_parameters(
                UNICAST_SINK_EXT_ADV_HANDLE,
                WICED_BT_BLE_EXT_ADV_EVENT_CONNECTABLE_ADV,
                40,
                40,
                (BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39),
                addr_type,
                addr_type,
                null_addr,
                BTM_BLE_ADV_POLICY_ACCEPT_CONN_AND_SCAN,
                0x7F,
                WICED_BT_BLE_EXT_ADV_PHY_1M,
                0,
                WICED_BT_BLE_EXT_ADV_PHY_1M,
                1,
                WICED_BT_BLE_EXT_ADV_SCAN_REQ_NOTIFY_ENABLE);

            UINT8_TO_STREAM(p_ext_adv_data, AD_FLAG_SIZE);
            UINT8_TO_STREAM(p_ext_adv_data, BTM_BLE_ADVERT_TYPE_FLAG);
            UINT8_TO_STREAM(p_ext_adv_data, BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED );

            // update WICED_BT_UUID_AUDIO_STREAM_CONTROL_SERVICE data

            UINT8_TO_STREAM(p_ext_adv_data, 3);
            UINT8_TO_STREAM(p_ext_adv_data, BTM_BLE_ADVERT_TYPE_SERVICE_DATA);
            UINT16_TO_STREAM(p_ext_adv_data, WICED_BT_UUID_VOLUME_CONTROL);

            UINT8_TO_STREAM(p_ext_adv_data, AUDIO_STREAM_CONTROL_SERVICE_SIZE);
            UINT8_TO_STREAM(p_ext_adv_data, BTM_BLE_ADVERT_TYPE_SERVICE_DATA);
            UINT16_TO_STREAM(p_ext_adv_data, WICED_BT_UUID_AUDIO_STREAM_CONTROL);
            ARRAY_TO_STREAM(p_ext_adv_data, &ascs_data, 6);

            UINT8_TO_STREAM(p_ext_adv_data, strlen((const char *)unicast_sink_cfg_settings.device_name) + 1);
            UINT8_TO_STREAM(p_ext_adv_data, BTM_BLE_ADVERT_TYPE_NAME_COMPLETE);
            ARRAY_TO_STREAM(p_ext_adv_data,
                            unicast_sink_cfg_settings.device_name,
                            strlen((const char *)unicast_sink_cfg_settings.device_name));

            // Set adv data in LTV format
            sts = wiced_bt_ble_set_ext_adv_data(UNICAST_SINK_EXT_ADV_HANDLE, (p_ext_adv_data - data), data);
            WICED_BT_TRACE("[%s] sts %d [adv size %d]\n", __FUNCTION__, sts, (p_ext_adv_data - data));
        }

        duration_cfg.adv_handle = UNICAST_SINK_EXT_ADV_HANDLE;
        duration_cfg.adv_duration = 0;
        duration_cfg.max_ext_adv_events = 0;

        // Start adv
        wiced_bt_ble_start_ext_adv(b_start, 1, &duration_cfg);
    }
    else
    {
        if (b_start)
        {
            wiced_bt_ble_advert_elem_t adv_elem[4];
            uint8_t num_elem = 0;
            uint8_t ascs_data[] = {0x4E, 0x18, 1, 4, 0, 0, 0, 0};
            uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

            adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
            adv_elem[num_elem].len = sizeof(uint8_t);
            adv_elem[num_elem].p_data = &flag;
            num_elem++;

            adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
            adv_elem[num_elem].len = (uint8_t)strlen((const char *)unicast_sink_cfg_settings.device_name);
            adv_elem[num_elem].p_data = (uint8_t *)unicast_sink_cfg_settings.device_name;
            num_elem++;

            adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_SERVICE_DATA;
            adv_elem[num_elem].len = 8;
            adv_elem[num_elem].p_data = ascs_data;
            num_elem++;

            wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
            gatt_status = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
        }
        else
        {
            gatt_status = wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
        }

        WICED_BT_TRACE("[%s] Start adv status [%d]\n", __FUNCTION__, gatt_status);
    }
}

/******************************************************************************
 * Function Name: unicast_sink_gatt_handle_connection
 ******************************************************************************
 * Summary: handle connection, send status to RPC
 *
 * Parameters:
 *  wiced_bt_gatt_connection_status_t *p_status
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_gatt_handle_connection(wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_bt_gatt_status_t ret_sts = WICED_ERROR;

    WICED_BT_TRACE("[%s] connected to [%B]\n", __FUNCTION__, p_status->bd_addr);

    /* Inform CC */
    le_audio_rpc_send_connect_event(p_status->addr_type, p_status->bd_addr, p_status->conn_id, p_status->link_role);
}

/******************************************************************************
 * Function Name: unicast_sink_gatt_handle_disconnection
 ******************************************************************************
 * Summary: handle disconnection, send status to RPC
 *
 * Parameters:
 *  wiced_bt_gatt_connection_status_t *p_conn_sts
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_gatt_handle_disconnection(wiced_bt_gatt_connection_status_t *p_conn_sts)
{
    wiced_bt_gatt_status_t ret_sts = WICED_ERROR;

    WICED_BT_TRACE("[%s] disconnected from [%B]\n", __FUNCTION__, p_conn_sts->bd_addr);
    unicast_sink_gatt_free_cb(p_conn_sts->bd_addr);
    unicast_sink_rpc_send_disconnect_evt( p_conn_sts->conn_id, p_conn_sts->reason);
}

/******************************************************************************
 * Function Name: unicast_sink_gatt_cback
 ******************************************************************************
 * Summary: gatt event callback function
 *
 * Parameters:
 *  wiced_bt_gatt_evt_t event
 *  wiced_bt_gatt_event_data_t *p_event_data
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
******************************************************************************/
wiced_bt_gatt_status_t unicast_sink_gatt_cback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_attribute_request_t *p_att_req = &p_event_data->attribute_request;

    WICED_BT_TRACE("[%s] event [%d]\n", __FUNCTION__, event);

    switch (event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            (p_event_data->connection_status.connected)
                ? unicast_sink_gatt_handle_connection(&p_event_data->connection_status)
                : unicast_sink_gatt_handle_disconnection(&p_event_data->connection_status);
            if (p_event_data->connection_status.connected)
            {
                unicast_sink_clcb_t *p_clcb = unicast_sink_gatt_alloc_cb(p_event_data->connection_status.bd_addr,
                                                                         p_event_data->connection_status.addr_type,
                                                                         p_event_data->connection_status.conn_id,
                                                                         p_event_data->connection_status.link_role);

                if (p_event_data->connection_status.link_role == HCI_ROLE_CENTRAL)
                {
                }
                else
                {
                    unicast_sink_ascs_init_data(p_clcb);
                    unicast_sink_pacs_init_data(p_clcb);
                }
            }
            break;
        default:
            WICED_BT_TRACE("Unknown event [%d]", event);
            break;
    }

    /* invoke the gatt interface common handler*/
    if (status == WICED_BT_GATT_SUCCESS)
    {
        status = gatt_interface_invoke_gatt_handler(event, p_event_data);
        WICED_BT_TRACE("[%s] default: %d ", __FUNCTION__, wiced_bt_get_largest_heap_buffer(p_default_heap));
    }

    return status;
}

/******************************************************************************
 * Function Name: unicast_sink_check_to_save_local
 ******************************************************************************
 * Summary: callback of gatt_interface_setup_services_from_local_db
 *          check uuid
 *
 * Parameters:
 *  void *p_app_ctx
 *  wiced_bt_uuid_t *p_uuid
 *
 * Return:
 *  wiced_bool_t 
 *
******************************************************************************/
wiced_bool_t unicast_sink_check_to_save_local(void *p_app_ctx, wiced_bt_uuid_t *p_uuid)
{
    uint16_t uuid = p_uuid->uu.uuid16;
    /* This app supports only 16 bit UUID */
    if (LEN_UUID_16 != p_uuid->len) return FALSE;

    if (uuid == WICED_BT_UUID_PUBLISHED_AUDIO_CAPABILITY || uuid == WICED_BT_UUID_AUDIO_STREAM_CONTROL ||
        uuid == WICED_BT_UUID_VOLUME_CONTROL)
        return TRUE;
    return FALSE;
}

/******************************************************************************
 * Function Name: unicast_sink_check_to_save_peer
 ******************************************************************************
 * Summary: callback of gatt_interface_start_discovery, check the discover UUID
 *
 * Parameters:
 *  void *p_app_ctx
 *  wiced_bt_uuid_t *p_uuid
 *
 * Return:
 *  wiced_bool_t
 *
******************************************************************************/
wiced_bool_t unicast_sink_check_to_save_peer(void *p_app_ctx, wiced_bt_uuid_t *p_uuid)
{
    uint16_t uuid = p_uuid->uu.uuid16;
    /* This app supports only 16 bit UUID */
    if (LEN_UUID_16 != p_uuid->len) return FALSE;

    //peer profile
    if (uuid == WICED_BT_UUID_GENERIC_MEDIA_CONTROL || uuid == WICED_BT_UUID_MEDIA_CONTROL) return TRUE;
    return FALSE;
}

/******************************************************************************
 * Function Name: unicast_sink_store_service_ref_local
 ******************************************************************************
 * Summary: callback of gatt_interface_setup_services_from_local_db
 *
 * Parameters:
 *  void *p_app_ctx
 *  wiced_bt_uuid_t *p_uuid
 *  gatt_intf_service_object_t *p_service
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_store_service_ref_local(void *p_app_ctx, wiced_bt_uuid_t *p_uuid, gatt_intf_service_object_t *p_service)
{
    gatt_intf_service_cb_t p_callback = NULL;
    uint16_t uuid = p_uuid->uu.uuid16;

    /* This app supports only 16 bit UUID */
    if (LEN_UUID_16 != p_uuid->len) return;

    WICED_BT_TRACE("[%s] UUID [0x%X] \n", __FUNCTION__, p_uuid->uu.uuid16);

    unicast_sink_local_profiles_t *p_profile = (unicast_sink_local_profiles_t *)p_app_ctx;

    switch (uuid)
    {
    case WICED_BT_UUID_PUBLISHED_AUDIO_CAPABILITY:
        WICED_BT_TRACE("[%s] PACS \n", __FUNCTION__);
        p_profile->p_pacs = p_service;
        p_callback = (gatt_intf_service_cb_t)unicast_sink_pacs_callback;
        break;
    case WICED_BT_UUID_AUDIO_STREAM_CONTROL:
        WICED_BT_TRACE("[%s] ASCS \n", __FUNCTION__);
        p_profile->p_ascs = p_service;
        p_callback = (gatt_intf_service_cb_t)unicast_sink_ascs_callback;
        break;
    case WICED_BT_UUID_VOLUME_CONTROL:
        WICED_BT_TRACE("[%s] VCS \n", __FUNCTION__);
        p_profile->p_vcs = p_service;
        p_callback = (gatt_intf_service_cb_t)unicast_sink_vcs_callback;
        break;
    default:
        break;
    }

    if (p_callback)
        gatt_interface_set_callback_to_profile(p_service, p_callback, p_app_ctx);
}

/******************************************************************************
 * Function Name: unicast_sink_store_service_ref_peer
 ******************************************************************************
 * Summary: callback of gatt_interface_start_discovery
 *
 * Parameters:
 *  void *p_app_ctx
 *  wiced_bt_uuid_t *p_uuid
 *  gatt_intf_service_object_t *p_service
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_store_service_ref_peer(void *p_app_ctx, wiced_bt_uuid_t *p_uuid, gatt_intf_service_object_t *p_service)
{
    gatt_intf_service_cb_t p_callback = NULL;
    uint16_t uuid = p_uuid->uu.uuid16;

    /* This app supports only 16 bit UUID */
    if (LEN_UUID_16 != p_uuid->len) return;

    WICED_BT_TRACE("[%s] UUID [0x%X] \n", __FUNCTION__, p_uuid->uu.uuid16);

    unicast_sink_peer_profiles_t *p_profile = (unicast_sink_peer_profiles_t *)p_app_ctx;
    switch (uuid)
    {
    case WICED_BT_UUID_MEDIA_CONTROL:
        WICED_BT_TRACE("[%s] MCS \n", __FUNCTION__);
        p_profile->p_mcs = p_service;
        p_callback = (gatt_intf_service_cb_t)unicast_sink_mcs_callback;
        break;
    case WICED_BT_UUID_GENERIC_MEDIA_CONTROL:
        WICED_BT_TRACE("[%s] GMCS \n", __FUNCTION__);
        p_profile->p_gmcs = p_service;
        p_callback = (gatt_intf_service_cb_t)unicast_sink_mcs_callback;
        break;
    default:
        break;
    }

    if (p_callback)
        gatt_interface_set_callback_to_profile(p_service, p_callback, p_app_ctx);
}

/******************************************************************************
 * Function Name: unicast_sink_gatt_init
 ******************************************************************************
 * Summary: gatt init, initial pacs, ascs, vcs, mcs, gmcs, local db
 *
 * Parameters:
 *  int max_connections
 *  int max_mtu, ga_cfg_t *p_ga_cfg
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
******************************************************************************/
wiced_bt_gatt_status_t unicast_sink_gatt_init(int max_connections, int max_mtu, ga_cfg_t *p_ga_cfg)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = {0};

    gatt_status = wiced_bt_gatt_db_init(unicast_sink_gatt_database, sizeof(unicast_sink_gatt_database), NULL);
    if (WICED_BT_SUCCESS != gatt_status) return gatt_status;

    gatt_status = wiced_bt_gatt_register(unicast_sink_gatt_cback);
    if (WICED_BT_SUCCESS != gatt_status) return gatt_status;

    /* Initialize GATT Interface App library */
    gatt_status = gatt_interface_init(max_connections, max_mtu, GATT_AUTH_REQ_NONE);
    if (WICED_BT_SUCCESS != gatt_status) return gatt_status;

    wiced_bt_isoc_register_cb(&unicast_sink_isoc_event_handler);
    unicast_sink_pacs_alloc_memory(p_ga_cfg);
    unicast_sink_ascs_alloc_memory(p_ga_cfg);
    unicast_sink_vcs_initialize_data();

    /* Initialize the supported profiles */
    wiced_bt_ga_pacs_init(p_ga_cfg);
    wiced_bt_ga_ascs_init(p_ga_cfg);
    wiced_bt_ga_vcs_init(p_ga_cfg);

    /* Initialize the peer supported profiles */
    wiced_bt_ga_mcs_init(p_ga_cfg);
    wiced_bt_ga_gmcs_init(p_ga_cfg);

    gatt_interface_setup_services_from_local_db(unicast_sink_check_to_save_local,
                                                unicast_sink_store_service_ref_local,
                                                &g_unicast_sink_gatt_cb.local_profiles);
    gatt_interface_print_linked_handles(bda);

    return gatt_status;
}

/******************************************************************************
 * Function Name: on_init_operation_complete
 ******************************************************************************
 * Summary: callback of gatt_interface_characteristic_operation
 *
 * Parameters:
 *  uint16_t conn_id
 *  gatt_intf_service_object_t *p_service
 *  gatt_intf_operation_t operation
 *  wiced_bt_gatt_status_t status
 *
 * Return:
 *  None
 *
******************************************************************************/
void on_init_operation_complete(uint16_t conn_id,
                                gatt_intf_service_object_t *p_service,
                                gatt_intf_operation_t operation,
                                wiced_bt_gatt_status_t status)
{
    unicast_sink_clcb_t *p_clcb = unicast_sink_gatt_get_clcb_by_conn_id(conn_id);

    WICED_BT_TRACE("[%s] op %d status %d", __FUNCTION__, operation, status);

    if (status != WICED_BT_GATT_SUCCESS) {
        return;
    }

    p_service = gatt_interface_get_next_linked_profile(p_service);
    if (!p_service) {
        switch (operation) {
            case GATT_INTF_OPERATION_NOTIFY_ALL_CHARACTERISTICS:
                operation = GATT_INTF_OPERATION_ENABLE_NOTIFICATIONS;
                p_service = gatt_interface_get_linked_client_profile_at(p_clcb->bda, 0);
                break;
            case GATT_INTF_OPERATION_ENABLE_NOTIFICATIONS:
                operation = GATT_INTF_OPERATION_READ;
                p_service = gatt_interface_get_linked_client_profile_at(p_clcb->bda, 0);
                break;
            case GATT_INTF_OPERATION_READ:
                break;
        }
    }

    if (!p_service) {

        WICED_BT_TRACE("[%s] update state ready", __FUNCTION__);
        return;
    }

    if (p_service) {

        status = gatt_interface_characteristic_operation(conn_id, p_service, operation, on_init_operation_complete);

        if (WICED_BT_GATT_SUCCESS != status) {
            wiced_bt_gatt_disconnect(conn_id);
        }
    }
}

/******************************************************************************
 * Function Name: unicast_sink_gatt_handle_discovery_complete
 ******************************************************************************
 * Summary: handle gatt discovery complete, callback of gatt_interface_start_discovery
 *
 * Parameters:
 *  uint16_t conn_id
 *  wiced_bt_gatt_status_t status
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_gatt_handle_discovery_complete(uint16_t conn_id,  wiced_bt_gatt_status_t status)
{
    unicast_sink_clcb_t *p_clcb = NULL;

    p_clcb = unicast_sink_gatt_get_clcb_by_conn_id(conn_id);
    if (!p_clcb) return;
    if (status)
    {
        WICED_BT_TRACE("[%s] status %d", __FUNCTION__, status);
        return;
    }

    gatt_interface_print_linked_handles(p_clcb->bda);

    {
        gatt_intf_service_object_t *p_service = gatt_interface_get_linked_server_profile_at(0);

        gatt_interface_characteristic_operation(conn_id,
                                                p_service,
                                                GATT_INTF_OPERATION_NOTIFY_ALL_CHARACTERISTICS,
                                                on_init_operation_complete);
    }
}

/******************************************************************************
 * Function Name: unicast_sink_gatt_start_discovery
 ******************************************************************************
 * Summary: start discovery gatt
 *
 * Parameters:
 *  uint8_t *p_bd_addr
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_sink_gatt_start_discovery(uint8_t *p_bd_addr)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    unicast_sink_clcb_t *p_clcb = NULL;

    p_clcb = unicast_sink_gatt_get_clcb(p_bd_addr);
    if (!p_clcb) return;

    status = gatt_interface_start_discovery(p_clcb->conn_id,
                                            unicast_sink_check_to_save_peer,
                                            unicast_sink_store_service_ref_peer,
                                            unicast_sink_gatt_handle_discovery_complete,
                                            &p_clcb->peer_profiles);

    if (status) WICED_BT_TRACE_CRIT("[%s] status [%d] \n", __FUNCTION__, status);
}
