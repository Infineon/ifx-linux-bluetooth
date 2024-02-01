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
#include "unicast_source_gatt.h"
#include "le_audio_rpc.h"
#include "unicast_source_rpc.h"
#include "unicast_source_mcs.h"

/* App Library includes */
#include "gatt_interface.h"
#include "wiced_bt_ga_ascs.h"
#include "wiced_bt_ga_mcs.h"
#include "wiced_bt_ga_pacs.h"
#include "wiced_bt_ga_vcs.h"

/* BT Stack includes */

#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_uuid.h"
#include "wiced_memory.h"

#include "unicast_source.h"
#include "app_bt_utils.h"

#define OBSERVER_ROLE_DURATION    (10u)

extern void unicast_source_menu_clear_info(void);
extern wiced_bt_cfg_ble_t unicast_source_ble_cfg;
extern wiced_bt_heap_t *p_default_heap;
unicast_source_gatt_cb_t g_unicast_source_gatt_cb;

enum
{
    HDLS_GENERIC_ATTRIBUTE = 1, // 0x0001 , 1

    HDLS_MCS = 144,                                                           // 0x0090 , 144
    HDLC_MCS_MEDIA_PLAYER_NAME,                                               // 0x0091 , 145
    HDLC_MCS_MEDIA_PLAYER_NAME_VALUE,                                         // 0x0092 , 146
    HDLD_MCS_PLAYER_NAME_DESCRIPTION_CLIENT_CONFIGURATION,                    // 0x0093 , 147
    HDLC_MCS_MEDIA_TRACK_CHANGED,                                             // 0x0094 , 148
    HDLC_MCS_MEDIA_TRACK_CHANGED_VALUE,                                       // 0x0095 , 149
    HDLD_MCS_TRACK_CHANGED_DESCRIPTION_CLIENT_CONFIGURATION,                  // 0x0096 , 150
    HDLC_MCS_MEDIA_TRACK_TITLE,                                               // 0x0097 , 151
    HDLC_MCS_MEDIA_TRACK_TITLE_VALUE,                                         // 0x0098 , 152
    HDLD_MCS_TRACK_TITLE_DESCRIPTION_CLIENT_CONFIGURATION,                    // 0x0099 , 153
    HDLC_MCS_MEDIA_TRACK_DURATION,                                            // 0x009A , 154
    HDLC_MCS_MEDIA_TRACK_DURATION_VALUE,                                      // 0x009B , 155
    HDLD_MCS_TRACK_DURATION_DESCRIPTION_CLIENT_CONFIGURATION,                 // 0x009C , 156
    HDLC_MCS_MEDIA_TRACK_POSITION,                                            // 0x009D , 157
    HDLC_MCS_MEDIA_TRACK_POSITION_VALUE,                                      // 0x009E , 158
    HDLD_MCS_TRACK_POSITION_DESCRIPTION_CLIENT_CONFIGURATION,                 // 0x009F , 159
    HDLC_MCS_PLAYBACK_SPEED,                                                  // 0x00A0 , 160
    HDLC_MCS_PLAYBACK_SPEED_VALUE,                                            // 0x00A1 , 161
    HDLD_MCS_PLAYBACK_SPEED_DESCRIPTION_CLIENT_CONFIGURATION,                 // 0x00A2 , 162
    HDLC_MCS_SEEKING_SPEED,                                                   // 0x00A3 , 163
    HDLC_MCS_SEEKING_SPEED_VALUE,                                             // 0x00A4 , 164
    HDLD_MCS_SEEKING_SPEED_DESCRIPTION_CLIENT_CONFIGURATION,                  // 0x00A5 , 165
    HDLC_MCS_PLAYING_ORDER,                                                   // 0x00A6 , 166
    HDLC_MCS_PLAYING_ORDER_VALUE,                                             // 0x00A7 , 167
    HDLD_MCS_PLAYING_ORDER_DESCRIPTION_CLIENT_CONFIGURATION,                  // 0x00A8 , 168
    HDLC_MCS_PLAYING_ORDER_SUPPORTED,                                         // 0x00A9 , 169
    HDLC_MCS_PLAYING_ORDER_SUPPORTED_VALUE,                                   // 0x00AA , 170
    HDLC_MCS_MEDIA_STATE,                                                     // 0x00AB , 171
    HDLC_MCS_MEDIA_STATE_VALUE,                                               // 0x00AC , 172
    HDLD_MCS_MEDIA_STATE_DESCRIPTION_CLIENT_CONFIGURATION,                    // 0x00AD , 173
    HDLC_MCS_MEDIA_CONTROL_POINT,                                             // 0x00AE , 174
    HDLC_MCS_MEDIA_CONTROL_POINT_VALUE,                                       // 0x00AF , 175
    HDLD_MCS_CONTROL_POINT_DESCRIPTION_CLIENT_CONFIGURATION,                  // 0x00B0 , 176
    HDLC_MCS_MEDIA_CONTROL_POINT_OPCODE_SUPPORTED,                            // 0x00B1 , 177
    HDLC_MCS_MEDIA_CONTROL_POINT_OPCODE_SUPPORTED_VALUE,                      // 0x00B2 , 178
    HDLD_MCS_CONTROL_POINT_OPCODE_SUPPORTED_DESCRIPTION_CLIENT_CONFIGURATION, // 0x00B3 , 179
    HDLC_MCS_CONTENT_CONTROL_ID,                                              // 0x00B4 , 180
    HDLC_MCS_CONTENT_CONTROL_ID_VALUE,                                        // 0x00B5 , 181

    HDLS_GMCS = 336,                                                           // 0x0150 , 336
    HDLC_GMCS_MEDIA_PLAYER_NAME,                                               // 0x0151 , 337
    HDLC_GMCS_MEDIA_PLAYER_NAME_VALUE,                                         // 0x0152 , 338
    HDLD_GMCS_PLAYER_NAME_DESCRIPTION_CLIENT_CONFIGURATION,                    // 0x0153 , 339
    HDLC_GMCS_MEDIA_TRACK_CHANGED,                                             // 0x0154 , 340
    HDLC_GMCS_MEDIA_TRACK_CHANGED_VALUE,                                       // 0x0155 , 341
    HDLD_GMCS_TRACK_CHANGED_DESCRIPTION_CLIENT_CONFIGURATION,                  // 0x0156 , 342
    HDLC_GMCS_MEDIA_TRACK_TITLE,                                               // 0x0157 , 343
    HDLC_GMCS_MEDIA_TRACK_TITLE_VALUE,                                         // 0x0158 , 344
    HDLD_GMCS_TRACK_TITLE_DESCRIPTION_CLIENT_CONFIGURATION,                    // 0x0159 , 345
    HDLC_GMCS_MEDIA_TRACK_DURATION,                                            // 0x015A , 346
    HDLC_GMCS_MEDIA_TRACK_DURATION_VALUE,                                      // 0x015B , 347
    HDLD_GMCS_TRACK_DURATION_DESCRIPTION_CLIENT_CONFIGURATION,                 // 0x015C , 348
    HDLC_GMCS_MEDIA_TRACK_POSITION,                                            // 0x015D , 349
    HDLC_GMCS_MEDIA_TRACK_POSITION_VALUE,                                      // 0x015E , 350
    HDLD_GMCS_TRACK_POSITION_DESCRIPTION_CLIENT_CONFIGURATION,                 // 0x015F , 351
    HDLC_GMCS_PLAYBACK_SPEED,                                                  // 0x0160 , 352
    HDLC_GMCS_PLAYBACK_SPEED_VALUE,                                            // 0x0161 , 353
    HDLD_GMCS_PLAYBACK_SPEED_DESCRIPTION_CLIENT_CONFIGURATION,                 // 0x0162 , 354
    HDLC_GMCS_SEEKING_SPEED,                                                   // 0x0163 , 355
    HDLC_GMCS_SEEKING_SPEED_VALUE,                                             // 0x0164 , 356
    HDLD_GMCS_SEEKING_SPEED_DESCRIPTION_CLIENT_CONFIGURATION,                  // 0x0165 , 357
    HDLC_GMCS_PLAYING_ORDER,                                                   // 0x0166 , 358
    HDLC_GMCS_PLAYING_ORDER_VALUE,                                             // 0x0167 , 359
    HDLD_GMCS_PLAYING_ORDER_DESCRIPTION_CLIENT_CONFIGURATION,                  // 0x0168 , 360
    HDLC_GMCS_PLAYING_ORDER_SUPPORTED,                                         // 0x0169 , 361
    HDLC_GMCS_PLAYING_ORDER_SUPPORTED_VALUE,                                   // 0x016A , 362
    HDLC_GMCS_MEDIA_STATE,                                                     // 0x016B , 363
    HDLC_GMCS_MEDIA_STATE_VALUE,                                               // 0x016C , 364
    HDLD_GMCS_MEDIA_STATE_DESCRIPTION_CLIENT_CONFIGURATION,                    // 0x016D , 365
    HDLC_GMCS_MEDIA_CONTROL_POINT,                                             // 0x016E , 366
    HDLC_GMCS_MEDIA_CONTROL_POINT_VALUE,                                       // 0x016F , 367
    HDLD_GMCS_CONTROL_POINT_DESCRIPTION_CLIENT_CONFIGURATION,                  // 0x0170 , 368
    HDLC_GMCS_MEDIA_CONTROL_POINT_OPCODE_SUPPORTED,                            // 0x0171 , 369
    HDLC_GMCS_MEDIA_CONTROL_POINT_OPCODE_SUPPORTED_VALUE,                      // 0x0172 , 370
    HDLD_GMCS_CONTROL_POINT_OPCODE_SUPPORTED_DESCRIPTION_CLIENT_CONFIGURATION, // 0x0173 , 371
    HDLC_GMCS_CONTENT_CONTROL_ID,                                              // 0x0174 , 372
    HDLC_GMCS_CONTENT_CONTROL_ID_VALUE,                                        // 0x0175 , 373
};

const uint8_t unicast_source_gatt_database[] = {
    /* Primary Service 'Generic Attribute' */
    PRIMARY_SERVICE_UUID16(HDLS_GENERIC_ATTRIBUTE, UUID_SERVICE_GATT),

    /* Primary Service 'MCS' */
    PRIMARY_SERVICE_UUID16(HDLS_MCS, WICED_BT_UUID_MEDIA_CONTROL),

    /* Characteristic 'Media Player Name' */
    CHARACTERISTIC_UUID16(HDLC_MCS_MEDIA_PLAYER_NAME,
                          HDLC_MCS_MEDIA_PLAYER_NAME_VALUE,
                          WICED_BT_UUID_MEDIA_PLAYER_NAME,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_MCS_PLAYER_NAME_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Track Changed' */
    CHARACTERISTIC_UUID16(HDLC_MCS_MEDIA_TRACK_CHANGED,
                          HDLC_MCS_MEDIA_TRACK_CHANGED_VALUE,
                          WICED_BT_UUID_MEDIA_TRACK_CHANGED,
                          GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_CHAR_PROP_NOTIFY),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_MCS_TRACK_CHANGED_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Track Title' */
    CHARACTERISTIC_UUID16(HDLC_MCS_MEDIA_TRACK_TITLE,
                          HDLC_MCS_MEDIA_TRACK_TITLE_VALUE,
                          WICED_BT_UUID_MEDIA_TRACK_TITLE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_MCS_TRACK_TITLE_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Track Duration' */
    CHARACTERISTIC_UUID16(HDLC_MCS_MEDIA_TRACK_DURATION,
                          HDLC_MCS_MEDIA_TRACK_DURATION_VALUE,
                          WICED_BT_UUID_MEDIA_TRACK_DURATION,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_MCS_TRACK_DURATION_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Track Position' */
    CHARACTERISTIC_UUID16_WRITABLE(
        HDLC_MCS_MEDIA_TRACK_POSITION,
        HDLC_MCS_MEDIA_TRACK_POSITION_VALUE,
        WICED_BT_UUID_MEDIA_TRACK_POSITION,
        GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE | GATTDB_CHAR_PROP_NOTIFY,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_READABLE*/ /*| GATTDB_PERM_AUTH_WRITABLE */),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_MCS_TRACK_POSITION_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Playback Speed' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_MCS_PLAYBACK_SPEED,
                                   HDLC_MCS_PLAYBACK_SPEED_VALUE,
                                   WICED_BT_UUID_MEDIA_PLAYBACK_SPEED,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE |
                                       GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/ | GATTDB_PERM_WRITE_CMD |
                                       GATTDB_PERM_WRITE_REQ /*| GATTDB_PERM_AUTH_WRITABLE */),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_MCS_PLAYBACK_SPEED_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Seeking Speed' */
    CHARACTERISTIC_UUID16(HDLC_MCS_SEEKING_SPEED,
                          HDLC_MCS_SEEKING_SPEED_VALUE,
                          WICED_BT_UUID_MEDIA_SEEKING_SPEED,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_MCS_SEEKING_SPEED_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Playing Order' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_MCS_PLAYING_ORDER,
                                   HDLC_MCS_PLAYING_ORDER_VALUE,
                                   WICED_BT_UUID_MEDIA_PLAYING_ORDER,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE |
                                       GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/ | GATTDB_PERM_WRITE_REQ |
                                       GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_MCS_PLAYING_ORDER_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Playing Order Supported' */
    CHARACTERISTIC_UUID16(HDLC_MCS_PLAYING_ORDER_SUPPORTED,
                          HDLC_MCS_PLAYING_ORDER_SUPPORTED_VALUE,
                          WICED_BT_UUID_MEDIA_PLAYING_ORDER_SUPPORTED,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Media State' */
    CHARACTERISTIC_UUID16(HDLC_MCS_MEDIA_STATE,
                          HDLC_MCS_MEDIA_STATE_VALUE,
                          WICED_BT_UUID_MEDIA_STATE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_MCS_MEDIA_STATE_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Media Control Point ' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_MCS_MEDIA_CONTROL_POINT,
                                   HDLC_MCS_MEDIA_CONTROL_POINT_VALUE,
                                   WICED_BT_UUID_MEDIA_CONTROL_POINT,
                                   GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE |
                                       GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ /*| GATTDB_PERM_AUTH_WRITABLE */),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_MCS_CONTROL_POINT_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Media Control Point Opcode supported' */
    CHARACTERISTIC_UUID16(HDLC_MCS_MEDIA_CONTROL_POINT_OPCODE_SUPPORTED,
                          HDLC_MCS_MEDIA_CONTROL_POINT_OPCODE_SUPPORTED_VALUE,
                          WICED_BT_UUID_MEDIA_CONTROL_OPCODE_SUPPORTED,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_MCS_CONTROL_POINT_OPCODE_SUPPORTED_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
                                        GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */),

    /* Characteristic 'Content Control ID (CCID)' */
    CHARACTERISTIC_UUID16(HDLC_MCS_CONTENT_CONTROL_ID,
                          HDLC_MCS_CONTENT_CONTROL_ID_VALUE,
                          WICED_BT_UUID_MEDIA_CONTENT_CONTROL_ID,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Primary Service 'GMCS' */
    PRIMARY_SERVICE_UUID16(HDLS_GMCS, WICED_BT_UUID_GENERIC_MEDIA_CONTROL),

    /* Characteristic 'Media Player Name' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_MEDIA_PLAYER_NAME,
                          HDLC_GMCS_MEDIA_PLAYER_NAME_VALUE,
                          WICED_BT_UUID_MEDIA_PLAYER_NAME,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_GMCS_PLAYER_NAME_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Track Changed' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_MEDIA_TRACK_CHANGED,
                          HDLC_GMCS_MEDIA_TRACK_CHANGED_VALUE,
                          WICED_BT_UUID_MEDIA_TRACK_CHANGED,
                          GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_CHAR_PROP_NOTIFY),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_GMCS_TRACK_CHANGED_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Track Title' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_MEDIA_TRACK_TITLE,
                          HDLC_GMCS_MEDIA_TRACK_TITLE_VALUE,
                          WICED_BT_UUID_MEDIA_TRACK_TITLE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_GMCS_TRACK_TITLE_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Track Duration' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_MEDIA_TRACK_DURATION,
                          HDLC_GMCS_MEDIA_TRACK_DURATION_VALUE,
                          WICED_BT_UUID_MEDIA_TRACK_DURATION,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_GMCS_TRACK_DURATION_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Track Position' */
    CHARACTERISTIC_UUID16_WRITABLE(
        HDLC_GMCS_MEDIA_TRACK_POSITION,
        HDLC_GMCS_MEDIA_TRACK_POSITION_VALUE,
        WICED_BT_UUID_MEDIA_TRACK_POSITION,
        GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_READABLE*/ /*| GATTDB_PERM_AUTH_WRITABLE */),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_GMCS_TRACK_POSITION_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Playback Speed' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_GMCS_PLAYBACK_SPEED,
                                   HDLC_GMCS_PLAYBACK_SPEED_VALUE,
                                   WICED_BT_UUID_MEDIA_PLAYBACK_SPEED,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY |
                                       GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
                                   GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/ | GATTDB_PERM_WRITE_CMD |
                                       GATTDB_PERM_WRITE_REQ /*| GATTDB_PERM_AUTH_WRITABLE */),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_GMCS_PLAYBACK_SPEED_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Seeking Speed' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_SEEKING_SPEED,
                          HDLC_GMCS_SEEKING_SPEED_VALUE,
                          WICED_BT_UUID_MEDIA_SEEKING_SPEED,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_GMCS_SEEKING_SPEED_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Playing Order' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_GMCS_PLAYING_ORDER,
                                   HDLC_GMCS_PLAYING_ORDER_VALUE,
                                   WICED_BT_UUID_MEDIA_PLAYING_ORDER,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY |
                                       GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
                                   GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/ | GATTDB_PERM_WRITE_REQ |
                                       GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_GMCS_PLAYING_ORDER_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Playing Order Supported' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_PLAYING_ORDER_SUPPORTED,
                          HDLC_GMCS_PLAYING_ORDER_SUPPORTED_VALUE,
                          WICED_BT_UUID_MEDIA_PLAYING_ORDER_SUPPORTED,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Media State' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_MEDIA_STATE,
                          HDLC_GMCS_MEDIA_STATE_VALUE,
                          WICED_BT_UUID_MEDIA_STATE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_GMCS_MEDIA_STATE_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Media Control Point ' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_GMCS_MEDIA_CONTROL_POINT,
                                   HDLC_GMCS_MEDIA_CONTROL_POINT_VALUE,
                                   WICED_BT_UUID_MEDIA_CONTROL_POINT,
                                   GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE |
                                       GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ /*| GATTDB_PERM_AUTH_WRITABLE */),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_GMCS_CONTROL_POINT_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Media Control Point Opcode supported' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_MEDIA_CONTROL_POINT_OPCODE_SUPPORTED,
                          HDLC_GMCS_MEDIA_CONTROL_POINT_OPCODE_SUPPORTED_VALUE,
                          WICED_BT_UUID_MEDIA_CONTROL_OPCODE_SUPPORTED,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
        HDLD_GMCS_CONTROL_POINT_OPCODE_SUPPORTED_DESCRIPTION_CLIENT_CONFIGURATION,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ |
            GATTDB_PERM_WRITE_CMD /*| GATTDB_PERM_AUTH_WRITABLE */ /*| GATTDB_PERM_AUTH_READABLE*/),

    /* Characteristic 'Content Control ID (CCID)' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_CONTENT_CONTROL_ID,
                          HDLC_GMCS_CONTENT_CONTROL_ID_VALUE,
                          WICED_BT_UUID_MEDIA_CONTENT_CONTROL_ID,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE /*| GATTDB_PERM_AUTH_READABLE*/),

};

/******************************************************************************
 * Function Name: unicast_source_gatt_scan_cb
 *
 * Summary: gatt scan callback and scan ble ADV and check UUID of ASCS device
 *
 * Parameters:
 *  wiced_bt_ble_scan_results_t *p_scan_result 
 *
 *  uint8_t *p_adv_data
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_gatt_scan_cb(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
    uint8_t *p_data;
    int offset = 0;
    uint8_t adv_entry_length = 0;
    uint16_t uuid;
    wiced_bt_device_address_t peer_addr;

    if (p_scan_result == NULL) return;

    //WICED_BT_TRACE("[%s] p_scan_result : %B", __FUNCTION__, p_scan_result->remote_bd_addr);

    if (wiced_bt_ble_is_ext_adv_supported() &&
        (!p_scan_result->is_extended && p_scan_result->ble_addr_type != BLE_ADDR_PUBLIC))
        return;

    p_data = wiced_bt_ble_get_next_adv_entry(p_adv_data, &offset, BTM_BLE_ADVERT_TYPE_SERVICE_DATA, &adv_entry_length);
    if (p_data == NULL || adv_entry_length == 0) return;


    WICED_BT_TRACE("[%s] offset %d length %d", __FUNCTION__, offset, adv_entry_length);
    while (adv_entry_length) /*if no entry is found adv_entry_length will be 0*/
    {
        STREAM_TO_UINT16(uuid, p_data);
        WICED_BT_TRACE("[%s] uuid found %x", __FUNCTION__, uuid);
        memcpy(&peer_addr, p_scan_result->remote_bd_addr, BD_ADDR_LEN);
        if (uuid == WICED_BT_UUID_AUDIO_STREAM_CONTROL)
        {
            le_audio_rpc_send_scan_res_event(p_scan_result->remote_bd_addr,
                                             p_scan_result->ble_addr_type,
                                             p_scan_result->ble_evt_type);
            unicast_source_add_sink_dev(p_scan_result);
            return;
        }
        else
        {
            p_data = wiced_bt_ble_get_next_adv_entry(p_adv_data,
                                                     &offset,
                                                     BTM_BLE_ADVERT_TYPE_SERVICE_DATA,
                                                     &adv_entry_length);
        }
    }

}

/******************************************************************************
 * Function Name: unicast_source_gatt_start_stop_scan
 *
 * Summary: start or stop ble scan using wiced_bt_ble_observe api
 *
 * Parameters:
 *  uint32_t start
 *
 * Return:
 *  uint32_t
 *
******************************************************************************/
uint32_t unicast_source_gatt_start_stop_scan(uint32_t start)
{
    wiced_result_t gatt_status;

    gatt_status = wiced_bt_ble_observe((start) ,
                                    0,
                                    unicast_source_gatt_scan_cb);

    return gatt_status;
}

/******************************************************************************
 * Function Name: unicast_source_gatt_alloc_cb
 *
 * Summary: allocate connected remote device info
 *
 * Parameters:
 *  uint8_t *p_bd_addr
 *      : remote bd addr
 *  wiced_bt_ble_address_type_t addr_type
 *      : remote bd addr type
 *  uint16_t conn_id
 *
 *  uint16_t link_role
 *      : Central or Peripheral
 *
 * Return:
 *  unicast_source_clcb_t*
 *
******************************************************************************/
unicast_source_clcb_t *unicast_source_gatt_alloc_cb(uint8_t *p_bd_addr,
                                                    wiced_bt_ble_address_type_t addr_type,
                                                    uint16_t conn_id,
                                                    uint16_t link_role)
{
    int index;
    unicast_source_clcb_t *p_clcb = NULL;
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++) {
        p_clcb = &g_unicast_source_gatt_cb.unicast_clcb[index];
        if (p_clcb->in_use == FALSE) {
            p_clcb->in_use = TRUE;
            p_clcb->conn_id = conn_id;
            p_clcb->addr_type = addr_type;
            memcpy(p_clcb->bda, p_bd_addr, BD_ADDR_LEN);
            p_clcb->b_is_central = (HCI_ROLE_CENTRAL == link_role) ? TRUE : FALSE;
            p_clcb->app_state = UNICAST_SOURCE_CONNECTED;
            p_clcb->cap_data->conn_id = conn_id;
            return p_clcb;
        }
    }
    return p_clcb;
}

/******************************************************************************
 * Function Name: unicast_source_gatt_get_clcb
 *
 * Summary: get store connected device clcb by bd_addr
 *
 * Parameters:
 *  uint8_t *p_bd_addr
 *
 * Return:
 *  unicast_source_clcb_t*
 *
******************************************************************************/
unicast_source_clcb_t *unicast_source_gatt_get_clcb(uint8_t *p_bd_addr)
{
    unicast_source_clcb_t *p_clcb = NULL;
    int index;
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++) {
        p_clcb = &g_unicast_source_gatt_cb.unicast_clcb[index];
        if (p_clcb->in_use && !WICED_MEMCMP(p_clcb->bda, p_bd_addr, BD_ADDR_LEN)) {
            return p_clcb;
        }
    }
    return NULL;
}

/******************************************************************************
 * Function Name: unicast_source_gatt_free_cb
 *
 * Summary: free allocate store clcb by bd_addr
 *
 * Parameters:
 *  uint8_t *p_bd_addr
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
******************************************************************************/
wiced_bt_gatt_status_t unicast_source_gatt_free_cb(uint8_t *p_bd_addr)
{
    unicast_source_clcb_t *p_clcb = unicast_source_gatt_get_clcb(p_bd_addr);
    if (!p_clcb) return WICED_ERROR;

    p_clcb->app_state = UNICAST_SOURCE_DISCONNECTED;
    p_clcb->in_use = FALSE;
    unicast_source_menu_clear_info();
    return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_source_gatt_get_clcb_by_conn_id
 *
 * Summary: get store connected device clcb by conn_id
 *
 * Parameters:
 *  uint16_t conn_id
 *
 * Return:
 *  unicast_source_clcb_t*
 *
******************************************************************************/
unicast_source_clcb_t *unicast_source_gatt_get_clcb_by_conn_id(uint16_t conn_id)
{
    unicast_source_clcb_t *p_clcb = NULL;
    int index;
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++) {
        p_clcb = &g_unicast_source_gatt_cb.unicast_clcb[index];
        if (p_clcb->in_use && (p_clcb->conn_id == conn_id)) {
            return p_clcb;
        }
    }
    return NULL;
}

/******************************************************************************
 * Function Name: unicast_source_gatt_handle_connection
 *
 * Summary: when connect to remote device, handle the information
 *
 * Parameters:
 *  wiced_bt_gatt_connection_status_t *p_conn_sts
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_gatt_handle_connection(wiced_bt_gatt_connection_status_t *p_conn_sts)
{
    wiced_bt_gatt_status_t ret_sts = WICED_ERROR;

    WICED_BT_TRACE("[%s] connected to [%B]\n", __FUNCTION__, p_conn_sts->bd_addr);

    /* Allocate GATT control block */
    if (!unicast_source_gatt_alloc_cb(p_conn_sts->bd_addr,
                                      p_conn_sts->addr_type,
                                      p_conn_sts->conn_id,
                                      p_conn_sts->link_role))
        return;

    /* Configure MTU */
    wiced_bt_gatt_client_configure_mtu(p_conn_sts->conn_id, unicast_source_ble_cfg.ble_max_rx_pdu_size);

    /* Inform CC */
    le_audio_rpc_send_connect_event(p_conn_sts->addr_type,
                                    p_conn_sts->bd_addr,
                                    p_conn_sts->conn_id,
                                    p_conn_sts->link_role);

}

/******************************************************************************
 * Function Name: unicast_source_gatt_handle_disconnection
 *
 * Summary: when disconnect, use this api to handle
 *
 * Parameters:
 *  wiced_bt_gatt_connection_status_t *p_conn_sts
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_gatt_handle_disconnection(wiced_bt_gatt_connection_status_t *p_conn_sts)
{
    wiced_bt_gatt_status_t ret_sts = WICED_ERROR;
    le_audio_rpc_send_disconnect_evt(p_conn_sts->conn_id, p_conn_sts->reason);

    unicast_source_gatt_free_cb(p_conn_sts->bd_addr);
    gatt_interface_free_client_device_ctx(p_conn_sts->bd_addr);

    WICED_BT_TRACE("[%s] disconnected from [%B]\n", __FUNCTION__, p_conn_sts->bd_addr);

    g_unicast_source_gatt_cb.mcs_data.media_state = WICED_BT_GA_MCS_MEDIA_PAUSED;
}

/******************************************************************************
 * Function Name: unicast_source_gatt_cback
 *
 * Summary: gatt callback, use this callback by wiced_bt_gatt_register API
 *
 * Parameters:
 *  wiced_bt_gatt_evt_t event
 *  wiced_bt_gatt_event_data_t *p_event_data
 *
 * Return:
 *  wiced_bt_gatt_status_t  
 *
******************************************************************************/
wiced_bt_gatt_status_t unicast_source_gatt_cback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_attribute_request_t *p_att_req = &p_event_data->attribute_request;
    wiced_bt_gatt_operation_complete_t *p_op_complete = &p_event_data->operation_complete;
    unicast_source_clcb_t *p_clcb = NULL;

    WICED_BT_TRACE("[%s] event [%d]\n", __FUNCTION__, event, get_bt_le_gatt_event_name(event));

    /* invoke the gatt interface common handler*/
    status = gatt_interface_invoke_gatt_handler(event, p_event_data);

    WICED_BT_TRACE("[%s] default buffer size: %d ", __FUNCTION__, wiced_bt_get_largest_heap_buffer(p_default_heap));

    switch (event) {
        case GATT_CONNECTION_STATUS_EVT:
            (p_event_data->connection_status.connected)
                ? unicast_source_gatt_handle_connection(&p_event_data->connection_status)
                : unicast_source_gatt_handle_disconnection(&p_event_data->connection_status);
            break;

        case GATT_OPERATION_CPLT_EVT:
            WICED_BT_TRACE("[%s] op %d", __FUNCTION__, p_event_data->operation_complete.op);

            p_clcb = unicast_source_gatt_get_clcb_by_conn_id(p_op_complete->conn_id);
            if (!p_clcb) return status;

            if (GATTC_OPTYPE_CONFIG_MTU == p_op_complete->op && p_clcb->app_state == UNICAST_SOURCE_CONNECTED) {
                p_clcb->app_state = UNICAST_SOURCE_MTU_CONFIGURED;

                /* TODO: if bonded, get GATT Db info from NVRAM and start encrption, else start bonding*/
                if (p_clcb->b_is_central)
                    wiced_bt_dev_sec_bond(p_clcb->bda, p_clcb->addr_type, BT_TRANSPORT_LE, 0, NULL);

                return WICED_SUCCESS;
            }
            else if (GATTC_OPTYPE_WRITE_WITH_RSP == p_op_complete->op) {
                WICED_BT_TRACE("[%s] op %d state %d", __FUNCTION__, p_op_complete->op, p_clcb->app_state);
            }
            else if (GATTC_OPTYPE_READ_HANDLE == p_op_complete->op) {
                WICED_BT_TRACE("[%s] op %d %d", __FUNCTION__, p_op_complete->op, p_clcb->app_state);
            }

            break;
    }

    return status;
}

/******************************************************************************
 * Function Name: unicast_source_check_to_save_local
 *
 * Summary: Callback for gatt_interface library use, check the UUID 
 *
 * Parameters:
 *  void *p_app_ctx
 *
 *  wiced_bt_uuid_t *p_uuid
 *
 * Return:
 *  wiced_bool_t
 *
******************************************************************************/
wiced_bool_t unicast_source_check_to_save_local(void *p_app_ctx, wiced_bt_uuid_t *p_uuid)
{
    uint16_t uuid = p_uuid->uu.uuid16;

    /* This app supports only 16 bit UUID */
    if (LEN_UUID_16 != p_uuid->len) return FALSE;

    if (uuid == WICED_BT_UUID_GENERIC_MEDIA_CONTROL || uuid == WICED_BT_UUID_MEDIA_CONTROL)
        return TRUE;

    return FALSE;
}

/******************************************************************************
 * Function Name: unicast_source_check_to_save_peer
 *
 * Summary: callback funciton for gatt_interface_start_discovery API use,
 *          check the peer UUID
 *
 * Parameters:
 *  void *p_app_ctx
 *  
 *  wiced_bt_uuid_t *p_uuid
 *
 * Return:
 *  wiced_bool_t 
 *
******************************************************************************/
wiced_bool_t unicast_source_check_to_save_peer(void *p_app_ctx, wiced_bt_uuid_t *p_uuid)
{
    uint16_t uuid = p_uuid->uu.uuid16;

    /* This app supports only 16 bit UUID */
    if (LEN_UUID_16 != p_uuid->len) return FALSE;

    //peer profile
    if (uuid == WICED_BT_UUID_PUBLISHED_AUDIO_CAPABILITY || uuid == WICED_BT_UUID_AUDIO_STREAM_CONTROL ||
        uuid == WICED_BT_UUID_VOLUME_CONTROL)
        return TRUE;
    return FALSE;
}

/******************************************************************************
 * Function Name: unicast_source_store_service_ref_local
 *
 * Summary: callback for gatt_interface_setup_services_from_local_db API use.
 *          set the callback to service by UUID type    
 *          only support mcs callback now
 *
 * Parameters:
 *  void *p_app_ctx
 *
 *  wiced_bt_uuid_t *p_uuid
 *
 *  gatt_intf_service_object_t *p_service
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_store_service_ref_local(void *p_app_ctx, wiced_bt_uuid_t *p_uuid, gatt_intf_service_object_t *p_service)
{
    gatt_intf_service_cb_t p_callback = NULL;
    uint16_t uuid = p_uuid->uu.uuid16;
    unicast_source_local_profiles_t *p_profile = (unicast_source_local_profiles_t *)p_app_ctx;

    /* This app supports only 16 bit UUID */
    if (LEN_UUID_16 != p_uuid->len) return;

    switch (uuid)
    {
    case WICED_BT_UUID_MEDIA_CONTROL:
        WICED_BT_TRACE("[%s] MCS \n", __FUNCTION__);
        p_profile->p_mcs = p_service;
        p_callback = (gatt_intf_service_cb_t)unicast_source_mcs_callback;
        break;
    case WICED_BT_UUID_GENERIC_MEDIA_CONTROL:
        WICED_BT_TRACE("[%s] GMCS \n", __FUNCTION__);
        p_profile->p_gmcs = p_service;
        p_callback = (gatt_intf_service_cb_t)unicast_source_mcs_callback;
        break;
    default:
        break;
    }

    if (p_callback != NULL)
        gatt_interface_set_callback_to_profile(p_service, p_callback, p_app_ctx);
}

/******************************************************************************
 * Function Name: unicast_source_store_service_ref_peer
 *
 * Summary: callback for gatt_interface_start_discovery API.
 *          set callback of pacs, ascs or vcs to service by UUID 
 *
 * Parameters:
 *  void *p_app_ctx
 *
 *  wiced_bt_uuid_t *p_uuid
 *
 *  gatt_intf_service_object_t *p_service
 *
 * Return:
 *  None
******************************************************************************/
void unicast_source_store_service_ref_peer(void *p_app_ctx, wiced_bt_uuid_t *p_uuid, gatt_intf_service_object_t *p_service)
{
    gatt_intf_service_cb_t p_callback = NULL;
    uint16_t uuid = p_uuid->uu.uuid16;
    unicast_source_peer_profiles_t *p_profile = (unicast_source_peer_profiles_t *)p_app_ctx;

    /* This app supports only 16 bit UUID */
    if (LEN_UUID_16 != p_uuid->len) return;

    switch (uuid)
    {
    case WICED_BT_UUID_PUBLISHED_AUDIO_CAPABILITY:
        WICED_BT_TRACE("[%s] PACS \n", __FUNCTION__);
        p_profile->p_pacs = p_service;
        p_callback = (gatt_intf_service_cb_t)unicast_source_pacs_callback;
        break;
    case WICED_BT_UUID_AUDIO_STREAM_CONTROL:
        WICED_BT_TRACE("[%s] ASCS \n", __FUNCTION__);
        p_profile->p_ascs = p_service;
        p_callback = (gatt_intf_service_cb_t)unicast_source_ascs_callback;
        break;
    case WICED_BT_UUID_VOLUME_CONTROL:
        WICED_BT_TRACE("[%s] VCS \n", __FUNCTION__);
        p_profile->p_vcs = p_service;
        p_callback = (gatt_intf_service_cb_t)unicast_source_vcs_callback;
        break;
    default:
        break;
    }

    if (p_callback != NULL)
        gatt_interface_set_callback_to_profile(p_service, p_callback, p_app_ctx);
}

/******************************************************************************
 * Function Name: on_init_operation_complete
 *
 * Summary: callback for gatt_interface_characteristic_operation.
 *         when gatt discovery complete
 *
 * Parameters:
 *  uint16_t conn_id
 *
 *  gatt_intf_service_object_t *p_service
 *
 *  gatt_intf_operation_t operation
 *      :GATT_INTF_OPERATION_NOTIFY_SERVER_CHARACTERISTICS or
 *      :GATT_INTF_OPERATION_ENABLE_NOTIFICATIONS or 
 *      :GATT_INTF_OPERATION_READ
 *
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
    unicast_source_clcb_t *p_clcb = unicast_source_gatt_get_clcb_by_conn_id(conn_id);

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
        p_clcb->app_state = UNICAST_SOURCE_READY;

        WICED_BT_TRACE("[%s] update state ready", __FUNCTION__);
        return;
    }

    if (p_service) {

        status =
            gatt_interface_characteristic_operation(conn_id, p_service, operation, on_init_operation_complete);

        if (WICED_BT_GATT_SUCCESS != status) {
            wiced_bt_gatt_disconnect(conn_id);
        }
    }
}

/******************************************************************************
 * Function Name: unicast_source_gatt_handle_discovery_complete
 *
 * Summary: callback for gatt_interface_start_discovery.
 *          when gatt discovery complete
 *
 * Parameters:
 *  uint16_t conn_id
 *  wiced_bt_gatt_status_t status
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_gatt_handle_discovery_complete(uint16_t conn_id, wiced_bt_gatt_status_t status)
{
    unicast_source_clcb_t *p_clcb = unicast_source_gatt_get_clcb_by_conn_id(conn_id);
    if (!p_clcb) return;

    p_clcb->app_state = UNICAST_SOURCE_DISCOVERY_COMPLETE;
    if (status)
    {
        WICED_BT_TRACE("[%s] status %d", __FUNCTION__, status);
        return;
    }
    gatt_interface_print_linked_handles(p_clcb->bda);

    unicast_source_init_remote_ases(p_clcb);

    /* start ASE discovery (read all the ASE char on the peer to get the list of ASE ID's) */
    p_clcb->app_state = UNICAST_SOURCE_INITING;

    {
        gatt_intf_service_object_t *p_service = gatt_interface_get_linked_server_profile_at(0);

        gatt_interface_characteristic_operation(conn_id,
                                                p_service,
                                                GATT_INTF_OPERATION_NOTIFY_ALL_CHARACTERISTICS,
                                                on_init_operation_complete);
    }
}

/******************************************************************************
 * Function Name: unicast_source_gatt_start_discovery
 *
 * Summary: API for start discovery GATT
 *
 * Parameters:
 *  uint8_t *p_bd_addr
 *
 * Return:
 *  None
 *
******************************************************************************/
void unicast_source_gatt_start_discovery(uint8_t *p_bd_addr)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    unicast_source_clcb_t *p_clcb = NULL;

    p_clcb = unicast_source_gatt_get_clcb(p_bd_addr);
    if (!p_clcb) return;

    status = gatt_interface_start_discovery(p_clcb->conn_id,
                                            unicast_source_check_to_save_peer,
                                            unicast_source_store_service_ref_peer,
                                            unicast_source_gatt_handle_discovery_complete,
                                            &p_clcb->peer_profiles);

    if (status) WICED_BT_TRACE_CRIT("[%s] status [%d] \n", __FUNCTION__, status);
}

/******************************************************************************
 * Function Name: unicast_source_gatt_init
 *
 * Summary: All the GATT Init, register callback, support profile
 *
 * Parameters:
 *  int max_connections
 *
 *  int max_mtu
 *
 *  ga_cfg_t *p_ga_cfg
 *
 * Return:
 *  wiced_bt_gatt_status_t
******************************************************************************/
wiced_bt_gatt_status_t unicast_source_gatt_init(int max_connections, int max_mtu, ga_cfg_t *p_ga_cfg)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = {0};
    int index = 0;

    gatt_status = wiced_bt_gatt_db_init(unicast_source_gatt_database, sizeof(unicast_source_gatt_database), NULL);
    if (WICED_BT_SUCCESS != gatt_status) return gatt_status;

    gatt_status = wiced_bt_gatt_register(unicast_source_gatt_cback);
    if (WICED_BT_SUCCESS != gatt_status) return gatt_status;

    /* Initialize GATT Interface App library */
    gatt_status = gatt_interface_init(max_connections, max_mtu, GATT_AUTH_REQ_NONE);
    if (WICED_BT_SUCCESS != gatt_status) return gatt_status;

    /* Initialize CLCB */
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++) {
        g_unicast_source_gatt_cb.unicast_clcb[index].cap_data = &g_unicast_source_gatt_cb.cap_data_ptr[index];

        g_unicast_source_gatt_cb.unicast_clcb[index].cap_data->pacs_data =
            &g_unicast_source_gatt_cb.unicast_clcb[index].pacs_data;
        g_unicast_source_gatt_cb.unicast_clcb[index].cap_data->vcs_data =
            &g_unicast_source_gatt_cb.unicast_clcb[index].vcs_data;
        WICED_BT_TRACE("%s cap_device[%d] 0x%x", __FUNCTION__, index, g_unicast_source_gatt_cb.cap_data_ptr[index]);

        g_unicast_source_gatt_cb.unicast_clcb[index].cap_data->pacs_data->source_pac_list.record_list =
            g_unicast_source_gatt_cb.unicast_clcb[index].pacs_src_record;
        g_unicast_source_gatt_cb.unicast_clcb[index].cap_data->pacs_data->sink_pac_list.record_list =
            g_unicast_source_gatt_cb.unicast_clcb[index].pacs_sink_record;
    }

    g_unicast_source_gatt_cb.cap_profile_data.device_info_list =
        (wiced_bt_ga_cap_device_data_t *)g_unicast_source_gatt_cb.cap_data_ptr;

    unicast_source_ascs_alloc_memory(p_ga_cfg);

    /*Register CAP callabck */
    wiced_bt_ga_cap_register_cb(&unicast_source_cap_event_cb);

    /*Register ISOC callabck */
    wiced_bt_isoc_register_cb(&unicast_source_isoc_event_handler);

    /* Initialize the supported profiles */
    wiced_bt_ga_mcs_init(p_ga_cfg);
    wiced_bt_ga_gmcs_init(p_ga_cfg);

    /* Initialize the peer supported profiles */
    wiced_bt_ga_ascs_init(p_ga_cfg);
    wiced_bt_ga_pacs_init(p_ga_cfg);
    wiced_bt_ga_vcs_init(p_ga_cfg);

    gatt_interface_setup_services_from_local_db(unicast_source_check_to_save_local,
                                                unicast_source_store_service_ref_local,
                                                &g_unicast_source_gatt_cb.local_profiles);
    gatt_interface_print_linked_handles(bda);

    return gatt_status;
}

/******************************************************************************
 * Function Name: unicast_source_gatt_disconnect
 *
 * Summary: disconnect GATT
 *
 * Parameters:
 *  wiced_bt_device_address_t addr
 *
 * Return:
 *  wiced_result_t 
 *
******************************************************************************/
wiced_result_t unicast_source_gatt_disconnect(wiced_bt_device_address_t addr)
{
    WICED_BT_TRACE("[%s] Peer Device  %B\n", __FUNCTION__, addr);
    uint16_t conn_id;
    unicast_source_clcb_t *p_clcb = unicast_source_gatt_get_clcb(addr);

    if (p_clcb != NULL)
    {
        conn_id = p_clcb->conn_id;
        if (unicast_source_is_streaming())
        {
            WICED_BT_TRACE("[%s] streaming in progress\n", __FUNCTION__);
            p_clcb->app_state = UNICAST_SOURCE_DISCONNECTING;
            unicast_source_mcs_pause(conn_id);
        }
        else
        {
            WICED_BT_TRACE("[%s] streaming not in progress\n", __FUNCTION__);
            wiced_bt_gatt_disconnect(conn_id);
        }
    }
    return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_source_gatt_connect
 *
 * Summary: gatt connect by bd_addr and addr_type
 *
 * Parameters:
 *  wiced_bt_device_address_t addr
 *
 *  uint8_t addr_type
 *
 * Return:
 *  wiced_result_t
******************************************************************************/
wiced_result_t unicast_source_gatt_connect(wiced_bt_device_address_t addr, uint8_t addr_type)
{
    WICED_BT_TRACE("[%s] Peer Device : %B Type : %d \n", __FUNCTION__, addr, addr_type);
    wiced_result_t result =  wiced_bt_gatt_le_connect((uint8_t *)addr, addr_type, BLE_CONN_MODE_HIGH_DUTY, TRUE);
    WICED_BT_TRACE("wiced_bt_gatt_le_connect result:%d\n", result);
    return result;
    //return WICED_SUCCESS;
}

/******************************************************************************
 * Function Name: unicast_source_gatt_handle_disconnecting_state
 *
 * Summary: handle gatt disconnect state
 *
 * Parameters:
 *  None
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_source_gatt_handle_disconnecting_state(void)
{
    wiced_result_t result = WICED_BT_ERROR;
    int index = 0;
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        if (g_unicast_source_gatt_cb.unicast_clcb[index].app_state == UNICAST_SOURCE_DISCONNECTING)
        {
            WICED_BT_TRACE("[%s] disconnecting %x\n",
                           __FUNCTION__,
                           g_unicast_source_gatt_cb.unicast_clcb[index].conn_id);
            result = wiced_bt_gatt_disconnect(g_unicast_source_gatt_cb.unicast_clcb[index].conn_id);
        }
    }
    return result;
}

/******************************************************************************
 * Function Name: unicast_source_gatt_start_stop_scan_menu
 *
 * Summary: handle gatt start or stop scan menu option
 *
 * Parameters:
 *  uint32_t start: start or stop
 *
 * Return:
 *  wiced_result_t
 *
******************************************************************************/
wiced_result_t unicast_source_gatt_start_stop_scan_menu(uint32_t start)
{
    wiced_result_t gatt_status;

    gatt_status = wiced_bt_ble_observe((start),
                                    OBSERVER_ROLE_DURATION,
                                    unicast_source_gatt_scan_cb);

    return gatt_status;
}
