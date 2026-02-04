/*
 * $ Copyright Cypress Semiconductor $
 */

/* Application includes */
#include "lehs.h"

extern wiced_bt_cfg_ble_t lehs_ble_cfg;
extern wiced_bt_cfg_settings_t lehs_cfg_settings;
extern wiced_bt_heap_t *p_lea_default_heap;

lehs_gatt_cb_t g_lehs_gatt_cb;

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
    HDLC_VCS_VOLUME_CONTROL_POINT_VALUE,        // 0x0017 , 23
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

    HDLC_PACS_SOURCE_PAC,                     // 0x00D7 , 215
    HDLC_PACS_SOURCE_PAC_VALUE,               // 0x00D8 , 216
    HDLD_PACS_SOURCE_PAC_CCCD,                // 0x00D9 , 217
    HDLC_PACS_SOURCE_AUDIO_LOCATIONS,         // 0x00DA , 218
    HDLC_PACS_SOURCE_AUDIO_LOCATIONS_VALUE,   // 0x00DB , 219
    HDLC_PACS_SOURCE_AUDIO_LOCATIONS_CCCD,    // 0x00DC , 220

    HDLC_PACS_AVAILABLE_AUDIO_CONTEXTS,       // 0x00DD , 221
    HDLC_PACS_AVAILABLE_AUDIO_CONTEXTS_VALUE, // 0x00DE , 222
    HDLD_PACS_AVAILABLE_AUDIO_CONTEXTS_CCCD,  // 0x00DF , 223
    HDLC_PACS_SUPPORTED_AUDIO_CONTEXTS,       // 0x00F0 , 224
    HDLC_PACS_SUPPORTED_AUDIO_CONTEXTS_VALUE, // 0x00F1 , 225
    HDLC_PACS_SUPPORTED_AUDIO_CONTEXTS_CCCD,  // 0x00F2 , 226

    HDLS_ASCS = 256,                   // 0x0100 , 256
    HDLC_ASCS_ASE_SINK,                // 0x0101 , 257
    HDLC_ASCS_ASE_SINK_VALUE,          // 0x0102 , 258
    HDLD_ASCS_ASE_SINK_CCCD,           // 0x0103 , 259
    HDLC_ASCS_ASE_SOURCE,              // 0x0104 , 260
    HDLC_ASCS_ASE_SOURCE_VALUE,        // 0x0105 , 261
    HDLD_ASCS_ASE_SOURCE_CCCD,         // 0x0106 , 262
    HDLC_ASCS_ASE_SINK_2,              // 0x0107 , 263
    HDLC_ASCS_ASE_SINK_VALUE_2,        // 0x0108 , 264
    HDLD_ASCS_ASE_SINK_CCCD_2,         // 0x0109 , 265
    HDLC_ASCS_ASE_CONTROL_POINT,       // 0x010A , 266
    HDLC_ASCS_ASE_CONTROL_POINT_VALUE, // 0x010B , 267
    HDLD_ASCS_ASE_CONTROL_POINT_CCCD,  // 0x010C , 268

    HDLS_CAS = 320,                    // 0x0140 , 320
    HDLI_CAS_INCLUDE_CSIS,             // 0x0141 , 321

    HDLS_BASS = 384,                                          // 0x0180 , 384
    HDLC_BASS_BROADCAST_AUDIO_SCAN_CONTROL_POINT,             // 0x0181 , 385
    HDLC_BASS_BROADCAST_AUDIO_SCAN_CONTROL_POINT_VALUE,       // 0x0182 , 386
    HDLC_BASS_BROADCAST_RECEIVE_STATE,                        // 0x0183 , 387
    HDLC_BASS_BROADCAST_RECEIVE_STATE_VALUE,                  // 0x0184 , 388
    HDLD_BASS_BROADCAST_RECEIVE_STATE_CLIENT_CONFIGURATION,   // 0x0185 , 389
    HDLC_BASS_BROADCAST_RECEIVE_STATE_2,                      // 0x0186 , 390
    HDLC_BASS_BROADCAST_RECEIVE_STATE_VALUE_2,                // 0x0187 , 391
    HDLD_BASS_BROADCAST_RECEIVE_STATE_CLIENT_CONFIGURATION_2, // 0x0188 , 392

    HDLS_CSIS = 400,                     // 0x0190, 400
    HDLC_CSIS_SIRK,                      // 0x0191, 401
    HDLC_CSIS_SIRK_VALUE,                // 0x0192, 402
    HDLD_CSIS_SIRK_CLIENT_CONFIGURATION, // 0x0193, 403
    HDLC_CSIS_SIZE,                      // 0x0194, 404
    HDLC_CSIS_SIZE_VALUE,                // 0x0195, 405
    HDLD_CSIS_SIZE_CLIENT_CONFIGURATION, // 0x0196, 406
    HDLC_CSIS_LOCK,                      // 0x0197, 407
    HDLC_CSIS_LOCK_VALUE,                // 0x0198, 408
    HDLD_CSIS_LOCK_CLIENT_CONFIGURATION, // 0x0199, 409
    HDLC_CSIS_RANK,                      // 0x019A, 410
    HDLC_CSIS_RANK_VALUE,                // 0x019B, 411

    HDLS_MICS = 420,                                       // 0x01A4, 420
    HDLI_MICS_INCLUDED_AICS,                               // 0x01A5, 421
    HDLC_MICS_MUTE_STATE,                                  // 0x01A6, 422
    HDLC_MICS_MUTE_STATE_VALUE,                            // 0x01A7, 423
    HDLD_MICS_MUTE_STATE_DESCRIPTION_CLIENT_CONFIGURATION, // 0x01A8, 424

    HDLS_MICS_AICS = 432,                                        // 0x01B0, 432
    HDLC_MICS_AICS_INPUT_STATE,                                  // 0x01B1, 433
    HDLC_MICS_AICS_INPUT_STATE_VALUE,                            // 0x01B2, 434
    HDLD_MICS_AICS_INPUT_STATE_CLIENT_CONFIGURATION,             // 0x01B3, 435
    HDLC_MICS_AICS_GAIN_SETTING_ATTR,                            // 0x01B4, 436
    HDLC_MICS_AICS_GAIN_SETTING_ATTR_VALUE,                      // 0x01B5, 437
    HDLC_MICS_AICS_INPUT_TYPE,                                   // 0x01B6, 438
    HDLC_MICS_AICS_INPUT_TYPE_VALUE,                             // 0x01B7, 439
    HDLC_MICS_AICS_INPUT_STATUS,                                 // 0x01B8, 440
    HDLC_MICS_AICS_INPUT_STATUS_VALUE,                           // 0x01B9, 441
    HDLD_MICS_AICS_INPUT_STATUS_CLIENT_CONFIGURATION,            // 0x01BA, 442
    HDLC_MICS_AICS_AUDIO_INPUT_CONTROL_POINT,                    // 0x01BB, 443
    HDLC_MICS_AICS_AUDIO_INPUT_CONTROL_POINT_VALUE,              // 0x01BC, 444
    HDLC_MICS_AICS_AUDIO_INPUT_DESCRIPTION,                      // 0x01BD, 445
    HDLC_MICS_AICS_AUDIO_INPUT_DESCRIPTION_VALUE,                // 0x01BE, 446
    HDLD_MICS_AICS_AUDIO_INPUT_DESCRIPTION_CLIENT_CONFIGURATION, // 0x01BF, 447

    HDLS_HAS = 480,                                                 // 0x1E1, 480
    HDLS_HAS_HEARIND_AID_FEATUES,                                   // 0x1E2, 481
    HDLS_HAS_HEARIND_AID_FEATUES_VALUE,                             // 0x1E3, 482
    HDLS_HAS_HEARIND_AID_FEATUES_CLIENT_CONFIGURATION,              // 0x1E4, 483
    HDLS_HAS_HEARING_AID_PRESET_CONTROL_POINT,                      // 0x1E5, 484
    HDLS_HAS_HEARING_AID_PRESET_CONTROL_POINT_VALUE,                // 0x1E6, 485
    HDLS_HAS_HEARING_AID_PRESET_CONTROL_POINT_CLIENT_CONFIGURATION, // 0x1E7, 486
    HDLS_HAS_ACTIVE_PRESET_INDEX,                                   // 0x1E8, 487
    HDLS_HAS_ACTIVE_PRESET_INDEX_VALUE,                             // 0x1E9, 488
    HDLS_HAS_ACTIVE_PRESET_INDEX_CLIENT_CONFIGURATION,              // 0x1EA, 489
};

const uint8_t lehs_gatt_database[] = {

    /* Primary Service 'Generic Attribute' */
    PRIMARY_SERVICE_UUID16(HDLS_GENERIC_ATTRIBUTE, UUID_SERVICE_GATT),

    /* Primary Service 'vcs' */
    PRIMARY_SERVICE_UUID16(HDLS_VCS, WICED_BT_UUID_VOLUME_CONTROL),

    /* Characteristic 'Volume State' */
    CHARACTERISTIC_UUID16(HDLC_VCS_VOLUME_STATE,
                          HDLC_VCS_VOLUME_STATE_VALUE,
                          WICED_BT_UUID_VOLUME_STATE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_VCS_VOLUME_STATE_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Characteristic 'Volume Control Point ' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_VCS_VOLUME_CONTROL_POINT_,
                                   HDLC_VCS_VOLUME_CONTROL_POINT_VALUE,
                                   WICED_BT_UUID_CONTROL_POINT,
                                   GATTDB_CHAR_PROP_WRITE,
                                    GATTDB_PERM_WRITABLE ),

    /* Characteristic 'Volume Flags' */
    CHARACTERISTIC_UUID16(HDLC_VCS_VOLUME_FLAGS,
                          HDLC_VCS_VOLUME_FLAGS_VALUE,
                          WICED_BT_UUID_VOLUME_FLAG,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_VCS_VOLUME_FLAGS_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE ),

    PRIMARY_SERVICE_UUID16(HDLS_PACS, WICED_BT_UUID_PUBLISHED_AUDIO_CAPABILITY),

    CHARACTERISTIC_UUID16(HDLC_PACS_SINK_PAC,
                          HDLC_PACS_SINK_PAC_VALUE,
                          WICED_BT_UUID_PACS_SINK_PAC,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_PACS_SINK_PAC_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    CHARACTERISTIC_UUID16_WRITABLE(HDLC_PACS_SINK_AUDIO_LOCATIONS,
                                   HDLC_PACS_SINK_AUDIO_LOCATIONS_VALUE,
                                   WICED_BT_UUID_PACS_SINK_AUDIO_LOCATIONS,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLC_PACS_SINK_AUDIO_LOCATIONS_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    CHARACTERISTIC_UUID16(HDLC_PACS_SOURCE_PAC,
                          HDLC_PACS_SOURCE_PAC_VALUE,
                          WICED_BT_UUID_PACS_SOURCE_PAC,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_PACS_SOURCE_PAC_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    CHARACTERISTIC_UUID16_WRITABLE(HDLC_PACS_SOURCE_AUDIO_LOCATIONS,
                                   HDLC_PACS_SOURCE_AUDIO_LOCATIONS_VALUE,
                                   WICED_BT_UUID_PACS_SOURCE_AUDIO_LOCATIONS,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLC_PACS_SOURCE_AUDIO_LOCATIONS_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    CHARACTERISTIC_UUID16(HDLC_PACS_AVAILABLE_AUDIO_CONTEXTS,
                          HDLC_PACS_AVAILABLE_AUDIO_CONTEXTS_VALUE,
                          WICED_BT_UUID_PACS_AUDIO_CONTEXT_AVAILABILITY,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_PACS_AVAILABLE_AUDIO_CONTEXTS_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    CHARACTERISTIC_UUID16(HDLC_PACS_SUPPORTED_AUDIO_CONTEXTS,
                          HDLC_PACS_SUPPORTED_AUDIO_CONTEXTS_VALUE,
                          WICED_BT_UUID_PACS_SUPPORTED_AUDIO_CONTEXT,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLC_PACS_SUPPORTED_AUDIO_CONTEXTS_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    PRIMARY_SERVICE_UUID16(HDLS_ASCS, WICED_BT_UUID_AUDIO_STREAM_CONTROL),

    CHARACTERISTIC_UUID16(HDLC_ASCS_ASE_SINK,
                          HDLC_ASCS_ASE_SINK_VALUE,
                          WICED_BT_UUID_ASCS_SINK_ASE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_ASCS_ASE_SINK_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    CHARACTERISTIC_UUID16(HDLC_ASCS_ASE_SOURCE,
                          HDLC_ASCS_ASE_SOURCE_VALUE,
                          WICED_BT_UUID_ASCS_SOURCE_ASE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_ASCS_ASE_SOURCE_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),


    CHARACTERISTIC_UUID16(HDLC_ASCS_ASE_SINK_2,
                          HDLC_ASCS_ASE_SINK_VALUE_2,
                          WICED_BT_UUID_ASCS_SINK_ASE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_ASCS_ASE_SINK_CCCD_2,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_ASCS_ASE_CONTROL_POINT,
                                   HDLC_ASCS_ASE_CONTROL_POINT_VALUE,
                                   WICED_BT_UUID_ASCS_ASE_CONTROL_POINT,
                                   GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE |
                                       GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_WRITABLE),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_ASCS_ASE_CONTROL_POINT_CCCD,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    PRIMARY_SERVICE_UUID16(HDLS_CAS, WICED_BT_UUID_COMMON_AUDIO),
    INCLUDE_SERVICE_UUID16(HDLI_CAS_INCLUDE_CSIS,
                           HDLS_CSIS,
                           HDLC_CSIS_RANK_VALUE,
                           WICED_BT_UUID_COORDINATE_SET_IDENTIFICATION),

        /* Primary Service 'BASS' */
    PRIMARY_SERVICE_UUID16(HDLS_BASS, WICED_BT_UUID_BROADCAST_AUDIO_SCAN),

    /* Characteristic 'Broadcast Audio Scan Control Point' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_BASS_BROADCAST_AUDIO_SCAN_CONTROL_POINT,
                                   HDLC_BASS_BROADCAST_AUDIO_SCAN_CONTROL_POINT_VALUE,
                                   WICED_BT_UUID_BASS_CONTROL_POINT,
                                   GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
                                   GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE),

    /* Characteristic 'Broadcast Receive State' */
    CHARACTERISTIC_UUID16(HDLC_BASS_BROADCAST_RECEIVE_STATE,
                          HDLC_BASS_BROADCAST_RECEIVE_STATE_VALUE,
                          WICED_BT_UUID_BASS_BROADCAST_RECEIVE_STATE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_BASS_BROADCAST_RECEIVE_STATE_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD |
                                        GATTDB_PERM_AUTH_WRITABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'Broadcast Receive State' */
    CHARACTERISTIC_UUID16(HDLC_BASS_BROADCAST_RECEIVE_STATE_2,
                          HDLC_BASS_BROADCAST_RECEIVE_STATE_VALUE_2,
                          WICED_BT_UUID_BASS_BROADCAST_RECEIVE_STATE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_BASS_BROADCAST_RECEIVE_STATE_CLIENT_CONFIGURATION_2,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD |
                                        GATTDB_PERM_AUTH_WRITABLE | GATTDB_PERM_AUTH_READABLE),
    /* Primary Service 'CSIS' */
    PRIMARY_SERVICE_UUID16(HDLS_CSIS, WICED_BT_UUID_COORDINATE_SET_IDENTIFICATION),

    /* Characteristic 'CSIS SIRK state' */
    CHARACTERISTIC_UUID16(HDLC_CSIS_SIRK,
                          HDLC_CSIS_SIRK_VALUE,
                          WICED_BT_UUID_CSIS_SIRK,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_CSIS_SIRK_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'CSIS SIZE state' */
    CHARACTERISTIC_UUID16(HDLC_CSIS_SIZE,
                          HDLC_CSIS_SIZE_VALUE,
                          WICED_BT_UUID_CSIS_SIZE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_CSIS_SIZE_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Characteristic 'CSIS Lock state' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_CSIS_LOCK,
                                   HDLC_CSIS_LOCK_VALUE,
                                   WICED_BT_UUID_CSIS_LOCK,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITE_REQ |
                                   GATTDB_PERM_AUTH_WRITABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_CSIS_LOCK_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'CSIS Rank state' */
    CHARACTERISTIC_UUID16(HDLC_CSIS_RANK,
                          HDLC_CSIS_RANK_VALUE,
                          WICED_BT_UUID_CSIS_RANK,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

                               /* Primary Service 'MICS' */
    PRIMARY_SERVICE_UUID16(HDLS_MICS, WICED_BT_UUID_MICROPHONE_CONTROL),
    /* Included Service 'AICS' */
    INCLUDE_SERVICE_UUID16(HDLI_MICS_INCLUDED_AICS,
                           HDLS_MICS_AICS,
                           HDLD_MICS_AICS_AUDIO_INPUT_DESCRIPTION_CLIENT_CONFIGURATION,
                           WICED_BT_UUID_AUDIO_INPUT_CONTROL),
    /* Characteristic 'Microphone Mute state' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_MICS_MUTE_STATE,
                                   HDLC_MICS_MUTE_STATE_VALUE,
                                   WICED_BT_UUID_MICS_MUTE_STATE,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITE_REQ |
                                       GATTDB_PERM_AUTH_WRITABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_MICS_MUTE_STATE_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Primary Service 'AICS' */
    SECONDARY_SERVICE_UUID16(HDLS_MICS_AICS, WICED_BT_UUID_AUDIO_INPUT_CONTROL),

    /* Characteristic 'Input State' */
    CHARACTERISTIC_UUID16(HDLC_MICS_AICS_INPUT_STATE,
                          HDLC_MICS_AICS_INPUT_STATE_VALUE,
                          WICED_BT_UUID_INPUT_STATE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_MICS_AICS_INPUT_STATE_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Characteristic 'Gain Setting Attribute' */
    CHARACTERISTIC_UUID16(HDLC_MICS_AICS_GAIN_SETTING_ATTR,
                          HDLC_MICS_AICS_GAIN_SETTING_ATTR_VALUE,
                          WICED_BT_UUID_GAIN_SETTING_ATTRIBUTE,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'Input Type' */
    CHARACTERISTIC_UUID16(HDLC_MICS_AICS_INPUT_TYPE,
                          HDLC_MICS_AICS_INPUT_TYPE_VALUE,
                          WICED_BT_UUID_INPUT_TYPE,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'Input Status' */
    CHARACTERISTIC_UUID16(HDLC_MICS_AICS_INPUT_STATUS,
                          HDLC_MICS_AICS_INPUT_STATUS_VALUE,
                          WICED_BT_UUID_INPUT_STATUS,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_MICS_AICS_INPUT_STATUS_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Characteristic 'Audio Input Control Point ' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_MICS_AICS_AUDIO_INPUT_CONTROL_POINT,
                                   HDLC_MICS_AICS_AUDIO_INPUT_CONTROL_POINT_VALUE,
                                   WICED_BT_UUID_AUDIO_INPUT_CONTROL_POINT,
                                   GATTDB_CHAR_PROP_WRITE,
                                   GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE),

    /* Characteristic 'Audio Output Description' */
    CHARACTERISTIC_UUID16_WRITABLE(
        HDLC_MICS_AICS_AUDIO_INPUT_DESCRIPTION,
        HDLC_MICS_AICS_AUDIO_INPUT_DESCRIPTION_VALUE,
        WICED_BT_UUID_AUDIO_INPUT_DESCRIPTION,
        GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE | GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITE_CMD |
                                       GATTDB_PERM_AUTH_WRITABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_MICS_AICS_AUDIO_INPUT_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),
#if HAS_ENABLED

    /* Primary Service 'HAS' */
    PRIMARY_SERVICE_UUID16(HDLS_HAS, WICED_BT_UUID_HEARING_ACCESS),

    /* Characteristic 'HAS hearing aid features' */
    CHARACTERISTIC_UUID16(HDLS_HAS_HEARIND_AID_FEATUES,
                          HDLS_HAS_HEARIND_AID_FEATUES_VALUE,
                          WICED_BT_UUID_HAS_HEARING_AID_FEATURES,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLS_HAS_HEARIND_AID_FEATUES_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Characteristic 'HAS preset control point' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLS_HAS_HEARING_AID_PRESET_CONTROL_POINT,
                          HDLS_HAS_HEARING_AID_PRESET_CONTROL_POINT_VALUE,
                          WICED_BT_UUID_HAS_HEARING_AID_PRESET_CONTROL_POINT,
                          GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY | GATTDB_CHAR_PROP_INDICATE,
                                   GATTDB_PERM_WRITABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLS_HAS_HEARING_AID_PRESET_CONTROL_POINT_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_WRITABLE),

    /* Characteristic 'HAS active preset index' */
    CHARACTERISTIC_UUID16(HDLS_HAS_ACTIVE_PRESET_INDEX,
                          HDLS_HAS_ACTIVE_PRESET_INDEX_VALUE,
                          WICED_BT_UUID_HAS_HEARING_AID_ACTIVE_PRESET_INDEX,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLS_HAS_ACTIVE_PRESET_INDEX_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),
#endif

};

static char *gatt_event_name[] = {
    "GATT_CONNECTION_STATUS_EVT",      /* 0 */
    "GATT_OPERATION_CPLT_EVT",         /* 1 */
    "GATT_DISCOVERY_RESULT_EVT",       /* 2 */
    "GATT_DISCOVERY_CPLT_EVT",         /* 3 */
    "GATT_ATTRIBUTE_REQUEST_EVT",      /* 4 */
    "GATT_CONGESTION_EVT",             /* 5 */
    "GATT_GET_RESPONSE_BUFFER_EVT",    /* 6 */
    "GATT_APP_BUFFER_TRANSMITTED_EVT", /* 7 */
};

wiced_bt_gatt_status_t lehs_send_disconnect(uint16_t conn_id, char * from)
{
    WICED_BT_TRACE("[%s] conn_id %d from %s\n", __FUNCTION__, conn_id, from ? from : "");

    le_audio_rpc_send_app_status(conn_id, HCI_CONTROL_LEA_APP_STATE_DISCONNECTING, 0);

    return wiced_bt_gatt_disconnect(conn_id);
}

lehs_clcb_t *lehs_gatt_alloc_cb(uint8_t *p_bd_addr,
                                                wiced_bt_ble_address_type_t addr_type,
                                                uint16_t conn_id,
                                                uint16_t link_role)
{
    int index;
    lehs_clcb_t *p_clcb = NULL;
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        p_clcb = &g_lehs_gatt_cb.clcb[index];
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

lehs_clcb_t *lehs_gatt_get_clcb(uint8_t *p_bd_addr)
{
    lehs_clcb_t *p_clcb = NULL;
    int index;
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        p_clcb = &g_lehs_gatt_cb.clcb[index];
        if (p_clcb->in_use && !WICED_MEMCMP(p_clcb->bda, p_bd_addr, BD_ADDR_LEN))
        {
            return p_clcb;
        }
    }
    return NULL;
}

wiced_bt_gatt_status_t lehs_gatt_free_cb(uint8_t *p_bd_addr)
{
    lehs_clcb_t *p_clcb = lehs_gatt_get_clcb(p_bd_addr);
    if (!p_clcb) return WICED_ERROR;

    p_clcb->in_use = FALSE;
    return WICED_SUCCESS;
}

lehs_clcb_t *lehs_gatt_get_clcb_by_conn_id(uint16_t conn_id)
{
    lehs_clcb_t *p_clcb = NULL;
    int index;
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        p_clcb = &g_lehs_gatt_cb.clcb[index];
        if (p_clcb->in_use && (p_clcb->conn_id == conn_id))
        {
            return p_clcb;
        }
    }
    return NULL;
}

typedef struct {
    uint16_t min_interval;
    uint16_t max_interval;
    uint16_t duration;
} adv_state_params_t;

const adv_state_params_t st_params[] = {
    { .min_interval = 0 , .max_interval = 0, .duration = 0 },
    { .min_interval = 32 , .max_interval = 32, .duration = 6000 },
    { .min_interval = 60 , .max_interval = 60, .duration = 6000 },
    { .min_interval = 40 , .max_interval = 40, .duration = 0 },
};

static void lehs_get_adv_duty_cycle_params(adv_state_t adv_state,
                                           wiced_ble_ext_adv_params_t *p_params,
                                           wiced_ble_ext_adv_duration_config_t *p_dur)
{
    if (ADV_STATE_MAX > ((uint8_t)adv_state))
    {
        const adv_state_params_t *p_st = &st_params[adv_state];

        p_params->primary_adv_int_min = p_st->min_interval;
        p_params->primary_adv_int_max = p_st->max_interval;
        p_dur->adv_duration = p_st->duration;
    }

    return;
}

//lehs
const uint8_t ascs_data[] = {0x4E,
                             0x18, //UUID
                             0x01, // Announcement Type (0x00: Genaral, 0x01: Targeted)
                             (BAP_CONTEXT_TYPE_CONVERSATIONAL | BAP_CONTEXT_TYPE_MEDIA | BAP_CONTEXT_TYPE_UNSPECIFIED),
                             0x02,
                             BAP_CONTEXT_TYPE_CONVERSATIONAL | BAP_CONTEXT_TYPE_UNSPECIFIED,
                             0,
                             0}; // metadata len


int lehs_gatt_get_adv_data(adv_state_t adv_state, wiced_bt_adv_ctx_t *p_ctx)
{
    uint16_t adv_opt = g_lehs_gatt_cb.adv_data_options;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint16_t volume_uuid = WICED_BT_UUID_VOLUME_CONTROL;
    //uint16_t aics_uuid = WICED_BT_UUID_AUDIO_INPUT_CONTROL;
    uint16_t bscan_uuid = WICED_BT_UUID_BROADCAST_AUDIO_SCAN;
    wiced_bt_ga_csis_sirk_data_t *p_sirk = lehs_csis_get_sirk();
    PSRI *psri = wiced_bt_ga_csis_generate_psri(&p_sirk->sirk);

    wiced_bt_ble_advert_elem_t adv_data_elems[] = {
        {.advert_type = BTM_BLE_ADVERT_TYPE_FLAG, .p_data = &flag, .len = sizeof(flag)},
        {.advert_type = BTM_BLE_ADVERT_TYPE_SERVICE_DATA, .p_data = (uint8_t *)&volume_uuid, .len = sizeof(volume_uuid)},
        {.advert_type = BTM_BLE_ADVERT_TYPE_SERVICE_DATA, .p_data = (uint8_t *)&ascs_data, .len = sizeof(ascs_data)},
        {.advert_type = BTM_BLE_ADVERT_TYPE_SERVICE_DATA,.p_data = (uint8_t *)&bscan_uuid, .len = sizeof(bscan_uuid)},
        {.advert_type = BTM_BLE_ADVERT_TYPE_PSRI, .p_data = (uint8_t *)psri, .len = sizeof(PSRI)},
        //.elem = {.advert_type = BTM_BLE_ADVERT_TYPE_SERVICE_DATA, .p_data = (uint8_t *)&aics_uuid, .len = sizeof(aics_uuid)},
    };
    wiced_bt_ble_advert_elem_t *p_element = adv_data_elems;
    uint16_t adv_len = 0;

    WICED_BT_TRACE("[%s] 0x%x\n", __FUNCTION__, adv_opt);


    //Swift pair LTV
    if ((adv_state == ADV_STATE_SWIFT_PAIR_HIGH_DUTY_CYCLE) || (adv_state == ADV_STATE_SWIFT_PAIR_LOW_DUTY_CYCLE))
    {
        const uint8_t swift_pair_LTV[5 + sizeof(DEVICE_NAME)] = {0x06, 0x00, 0x03, 0x00, 0x80};
        WICED_MEMCPY(&swift_pair_LTV[5], lehs_cfg_settings.device_name, sizeof(DEVICE_NAME));
        wiced_bt_ble_advert_elem_t elem = {.advert_type = BTM_BLE_ADVERT_TYPE_MANUFACTURER,
                                           .p_data = (uint8_t *)swift_pair_LTV,
                                           .len = sizeof(swift_pair_LTV)};
        adv_len += wiced_ble_adv_data_build(p_ctx, &elem);
    }
    else if (adv_opt & 32)
    {
        wiced_bt_ble_advert_elem_t elem = {.advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,
                                           .p_data = lehs_cfg_settings.device_name,
                                           .len = sizeof(DEVICE_NAME)};
        adv_len += wiced_ble_adv_data_build(p_ctx, &elem);
    }

    for (int i = 0; i < sizeof(adv_data_elems) / sizeof(adv_data_elems[0]); i++)
    {
        if (adv_opt & (1 << i))
        {
            adv_len += wiced_ble_adv_data_build(p_ctx, p_element);
        }
        p_element++;
    }

    WICED_BT_TRACE("[%s] [len %d %d]\n", __FUNCTION__, adv_len, p_ctx->offset);

    return p_ctx->offset;
}

void lehs_gatt_start_stop_adv(uint32_t b_start, adv_state_t adv_state)
{
    wiced_ble_ext_adv_duration_config_t duration_cfg = {.adv_handle = UNICAST_SINK_EXT_ADV_HANDLE };
    uint8_t addr_type = (lehs_cfg_settings.p_ble_cfg->rpa_refresh_timeout) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
    wiced_ble_ext_adv_params_t params = {
            .event_properties = WICED_BLE_EXT_ADV_EVENT_PROPERTY_CONNECTABLE_ADV,
            .primary_adv_channel_map = (BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39),
            .own_addr_type = addr_type,
            .peer_addr_type = 0,
            .adv_filter_policy = BTM_BLE_ADV_POLICY_ACCEPT_CONN_AND_SCAN,
            .primary_adv_phy = WICED_BLE_EXT_ADV_PHY_1M,
            .secondary_adv_phy = WICED_BLE_EXT_ADV_PHY_1M,
            .adv_sid = 1
    };

    lehs_get_adv_duty_cycle_params(adv_state, &params, &duration_cfg);
    WICED_BT_TRACE("[%s] adv_state : %d", __FUNCTION__, adv_state );

    if (b_start)
    {
        uint8_t adv_data[60];
        wiced_bt_adv_ctx_t ctx = {.p_adv = adv_data, .adv_len = sizeof(adv_data)};
        int adv_data_len = lehs_gatt_get_adv_data(adv_state, &ctx);

        if (adv_data_len < 31)
        {
            params.event_properties = WICED_BLE_EXT_ADV_EVENT_PROPERTY_CONNECTABLE_ADV |
                                      WICED_BLE_EXT_ADV_EVENT_PROPERTY_SCANNABLE_ADV |
                                      WICED_BLE_EXT_ADV_EVENT_PROPERTY_LEGACY_ADV;
        }

            // Set adv data in LTV format
        wiced_bt_dev_status_t sts = wiced_ble_ext_adv_set_params(UNICAST_SINK_EXT_ADV_HANDLE, &params);
        if(WICED_SUCCESS == sts){
		sts = wiced_ble_ext_adv_set_adv_data(UNICAST_SINK_EXT_ADV_HANDLE, ctx.offset, ctx.p_adv);
        }
    }

    // Start/Stop adv
    wiced_ble_ext_adv_enable(b_start, 1, &duration_cfg);

    le_audio_rpc_send_advertisement_state(adv_state);
}

const adv_state_t swift_pair_next_state[] = {ADV_STATE_SWIFT_PAIR_HIGH_DUTY_CYCLE,
                                             ADV_STATE_SWIFT_PAIR_LOW_DUTY_CYCLE,
                                             ADV_STATE_REGULAR_ADV,
                                             ADV_STATE_IDLE,
                                             ADV_STATE_IDLE};
const adv_state_t regular_adv_next_state[] = {ADV_STATE_REGULAR_ADV,
                                              ADV_STATE_IDLE,
                                              ADV_STATE_IDLE,
                                              ADV_STATE_IDLE,
                                              ADV_STATE_IDLE};

adv_state_t lehs_move_to_next_adv_state(adv_state_t current, char *from)
{
    const adv_state_t *p_transition = regular_adv_next_state;

    if (g_lehs_gatt_cb.do_swift_pair)
    {
        p_transition = swift_pair_next_state;
    }

    WICED_BT_TRACE("[%s]: state :%d next %d", __FUNCTION__, current, p_transition[current]);

    g_lehs_gatt_cb.adv_state = p_transition[current];

    if (g_lehs_gatt_cb.adv_state != ADV_STATE_IDLE)
    {
        lehs_gatt_start_stop_adv(1, g_lehs_gatt_cb.adv_state);
    }
    else
    {
        lehs_gatt_start_stop_adv(0, g_lehs_gatt_cb.adv_state);
    }

    return g_lehs_gatt_cb.adv_state;
}

void lehs_gatt_handle_connection(wiced_bt_gatt_connection_status_t *p_status)
{
    if (p_status->connected)
    {
        lehs_clcb_t *p_clcb = lehs_gatt_alloc_cb(p_status->bd_addr,
                                                 p_status->addr_type,
                                                 p_status->conn_id, p_status->link_role);

        if (p_clcb && p_status->link_role != HCI_ROLE_CENTRAL)
        {
            lehs_ascs_init_data(p_clcb);
            lehs_pacs_init_data();
        }
    }

    /* Inform CC */
    le_audio_rpc_send_connect_event(p_status);
    le_audio_rpc_send_app_status(p_status->conn_id, HCI_CONTROL_LEA_APP_STATE_CONNECTED, 0);
}

void lehs_gatt_handle_disconnection(wiced_bt_gatt_connection_status_t *p_sts)
{

    WICED_BT_TRACE("[%s] disconnected from [%B]\n", __FUNCTION__, p_sts->bd_addr);
    lehs_gatt_free_cb(p_sts->bd_addr);
    le_audio_rpc_send_disconnect_evt(p_sts);
    le_audio_rpc_send_app_status(p_sts->conn_id, HCI_CONTROL_LEA_APP_STATE_DISCONNECTED, 0);
}

wiced_bt_gatt_status_t lehs_gatt_cback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    WICED_BT_TRACE("[%s] event [0x%x] max_heap %d\n", __FUNCTION__, event, wiced_bt_get_largest_heap_buffer(p_lea_default_heap));

    switch (event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            (p_event_data->connection_status.connected)
                ? lehs_gatt_handle_connection(&p_event_data->connection_status)
                : lehs_gatt_handle_disconnection(&p_event_data->connection_status);

            break;
        case GATT_OPERATION_CPLT_EVT:
        case GATT_DISCOVERY_RESULT_EVT:
        case GATT_DISCOVERY_CPLT_EVT:
        case GATT_ATTRIBUTE_REQUEST_EVT:
        case GATT_CONGESTION_EVT:
        case GATT_GET_RESPONSE_BUFFER_EVT:
        case GATT_APP_BUFFER_TRANSMITTED_EVT:
            //hanled in comman handler gatt_interface_invoke_gatt_handler below
            WICED_BT_TRACE("[%s]", gatt_event_name[event]);
            break;

        default:
            WICED_BT_TRACE("Unknown event [0x%x]", event);
            break;
    }

    /* invoke the gatt interface common handler*/
    if (status == WICED_BT_GATT_SUCCESS)
    {
        status = gatt_interface_invoke_gatt_handler(event, p_event_data);
    }

    return status;
}

gatt_intf_service_object_t **lehs_get_profile_ptr_addr_int(gatt_intf_service_object_t **p_profile_list,
                                                           uint32_t limit,
                                                           uint8_t *p_num,
                                                           int incr)
{
    gatt_intf_service_object_t **pp_profile = NULL;

    if ((*p_num) < limit)
    {
        pp_profile = p_profile_list + (*p_num);
    }

    if (pp_profile)
    {
        (*p_num) += incr;
    }

    return pp_profile;
}

gatt_intf_service_object_t **lehs_get_local_profile_ptr_addr(lehs_local_profiles_t *p_profiles,
                                                             gatt_intf_service_info_t *p_srv_info,
                                                             int incr,
                                                             gatt_intf_service_cb_t *pp_callback)
{
    wiced_bt_uuid_t *p_uuid = &p_srv_info->group.service_type;
    gatt_intf_service_object_t **pp_profile = NULL;
    gatt_intf_service_cb_t p_callback = NULL;

    /* This app supports only 16 bit UUID */
    if (LEN_UUID_16 != p_uuid->len)
    {
        return NULL;
    }

    switch (p_uuid->uu.uuid16)
    {
    case WICED_BT_UUID_VOLUME_CONTROL:
        pp_profile = &p_profiles->p_vcs;
        p_callback = lehs_vcs_callback;
        break;
    case WICED_BT_UUID_MICROPHONE_CONTROL:
        pp_profile = &p_profiles->p_mics;
        p_callback = lehs_mics_callback;
        break;
    case WICED_BT_UUID_COORDINATE_SET_IDENTIFICATION:
        pp_profile = &p_profiles->p_csis;
        p_callback = lehs_csis_callback;
        break;
    case WICED_BT_UUID_AUDIO_STREAM_CONTROL:
        pp_profile = &p_profiles->p_ascs;
        p_callback = lehs_ascs_callback;
        break;
    case WICED_BT_UUID_PUBLISHED_AUDIO_CAPABILITY:
        pp_profile = &p_profiles->p_pacs;
        p_callback = lehs_pacs_callback;
        break;
    case WICED_BT_UUID_HEARING_ACCESS:
        pp_profile = &p_profiles->p_has;
        p_callback = lehs_has_callback;
        break;
    case WICED_BT_UUID_AUDIO_INPUT_CONTROL:
    {
        uint16_t parent_uuid = p_srv_info->included_by_uuid.uu.uuid16;

        if (parent_uuid == WICED_BT_UUID_MICROPHONE_CONTROL)
        {
            pp_profile =
                lehs_get_profile_ptr_addr_int(p_profiles->p_mics_aics, MAX_MICS_AICS, &p_profiles->num_mics_aics, incr);
            p_callback = lehs_mics_aics_callback;
        }
    }
    break;
    case WICED_BT_UUID_BROADCAST_AUDIO_SCAN:
        pp_profile = &p_profiles->p_bass;
        p_callback = lehs_bass_callback;
        break;
    default:
        break;
    }

    if (pp_callback)
    {
        *pp_callback = p_callback;
    }
    return pp_profile;
}

gatt_intf_service_object_t **lehs_get_peer_profile_ptr_addr(lehs_peer_profiles_t *p_profiles,
                                                            gatt_intf_service_info_t *p_srv_info,
                                                            int incr,
                                                            gatt_intf_service_cb_t *pp_callback)
{
    wiced_bt_uuid_t *p_uuid = &p_srv_info->group.service_type;
    gatt_intf_service_object_t **pp_profile = NULL;
    gatt_intf_service_cb_t p_callback = NULL;

    /* This app supports only 16 bit UUID */
    if (LEN_UUID_16 != p_uuid->len)
    {
        return NULL;
    }

    switch (p_uuid->uu.uuid16)
    {
    case WICED_BT_UUID_GENERIC_MEDIA_CONTROL:
        pp_profile = &p_profiles->p_gmcs;
        p_callback = lehs_mcs_callback;
        break;
    case WICED_BT_UUID_GENERIC_TELEPHONE_BEARER:
        pp_profile = &p_profiles->p_gtbs;
        p_callback = lehs_ccp_callback;
        break;
    default:
        break;
    }

    if (pp_callback)
    {
        *pp_callback = p_callback;
    }
    return pp_profile;
}

wiced_bool_t lehs_check_to_save_local(void *p_app_ctx, gatt_intf_service_info_t *p_srv_info)
{
    lehs_local_profiles_t *p_profile = (lehs_local_profiles_t *)p_app_ctx;
    gatt_intf_service_object_t **pp_service = lehs_get_local_profile_ptr_addr(p_profile, p_srv_info, 0, NULL);

    return (pp_service) ? TRUE : FALSE;
}

wiced_bool_t lehs_check_to_save_peer(void *p_app_ctx, gatt_intf_service_info_t *p_srv_info)
{
    lehs_peer_profiles_t *p_profile = (lehs_peer_profiles_t *)p_app_ctx;
    gatt_intf_service_object_t **pp_service = lehs_get_peer_profile_ptr_addr(p_profile, p_srv_info, 0, NULL);

    return (pp_service) ? TRUE : FALSE;
}

void lehs_store_service_ref_local(void *p_app_ctx,
                                  gatt_intf_service_info_t *p_srv_info,
                                  gatt_intf_service_object_t *p_service)
{
    wiced_bt_gatt_group_value_t *p_gp = &p_srv_info->group;
    lehs_local_profiles_t *p_profile = (lehs_local_profiles_t *)p_app_ctx;
    gatt_intf_service_cb_t p_callback = NULL;
    gatt_intf_service_object_t **pp_service = lehs_get_local_profile_ptr_addr(p_profile, p_srv_info, 0, &p_callback);

    if (pp_service)
    {
        *pp_service = p_service;
    }
    if (p_callback != NULL)
    {
        gatt_interface_set_callback_to_profile(p_service, p_callback, p_app_ctx);
    }
}

void lehs_store_service_ref_peer(void *p_app_ctx,
                                 gatt_intf_service_info_t *p_srv_info,
                                 gatt_intf_service_object_t *p_service)
{
    wiced_bt_gatt_group_value_t *p_gp = &p_srv_info->group;
    lehs_peer_profiles_t *p_profile = (lehs_peer_profiles_t *)p_app_ctx;
    gatt_intf_service_cb_t p_callback = NULL;
    gatt_intf_service_object_t **pp_service = lehs_get_peer_profile_ptr_addr(p_profile, p_srv_info, 0, &p_callback);

    if (pp_service)
    {
        *pp_service = p_service;
    }
    if (p_callback != NULL)
    {
        gatt_interface_set_callback_to_profile(p_service, p_callback, p_app_ctx);
    }
}

wiced_bt_gatt_status_t lehs_gatt_init(int max_connections, int max_mtu, lehs_ga_init_data_t p_ga_data)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = {0};

    gatt_status = wiced_bt_gatt_db_init(lehs_gatt_database, sizeof(lehs_gatt_database), NULL);
    if (WICED_BT_SUCCESS != gatt_status) return gatt_status;

    gatt_status = wiced_bt_gatt_register(lehs_gatt_cback);
    if (WICED_BT_SUCCESS != gatt_status) return gatt_status;

    /* Initialize GATT Interface App library */
    gatt_status = gatt_interface_init(max_connections, max_mtu, GATT_AUTH_REQ_NONE);
    if (WICED_BT_SUCCESS != gatt_status) return gatt_status;

    wiced_ble_ext_adv_register_cback(lehs_ext_adv_cback);
    lehs_pacs_alloc_memory();
    lehs_ascs_alloc_memory();
    lehs_vcs_initialize_data();
    lehs_csis_initialize_data();
    lehs_mics_initialize_data();
#if HAS_ENABLED
    lehs_has_initialize_data();
#endif

    /* Initialize the supported profiles */
    wiced_bt_ga_pacs_init(1, p_ga_data.p_pacs_init_data);
    wiced_bt_ga_pacs_enable_server();
    wiced_bt_ga_ascs_init(1, p_ga_data.p_ascs_init_data);
    wiced_bt_ga_ascs_enable_server();
    wiced_bt_ga_vcs_init (1, p_ga_data.p_vcs_init_data);
    wiced_bt_ga_vcs_enable_server();
    wiced_bt_ga_csis_init(1, NULL);
    wiced_bt_ga_csis_enable_server();
    wiced_bt_ga_mics_init(1, p_ga_data.p_mics_init_data);
    wiced_bt_ga_mics_enable_server();
    wiced_bt_ga_aics_init(MAX_MICS_AICS, NULL);
    wiced_bt_ga_aics_enable_server();
    wiced_bt_ga_bass_init(1, p_ga_data.p_bass_init_data);
    wiced_bt_ga_bass_enable_server();
#if HAS_ENABLED
    wiced_bt_ga_has_init (1, NULL);
    wiced_bt_ga_has_enable_server();
#endif

    /* Initialize the peer supported profiles */
    // wiced_bt_ga_mcs_init (1, NULL);
    // wiced_bt_ga_mcs_enable_client();
    wiced_bt_ga_gmcs_init(1, NULL);
    wiced_bt_ga_gmcs_enable_client();
    // wiced_bt_ga_tbs_init (1, NULL);
    // wiced_bt_ga_tbs_enable_client();
    wiced_bt_ga_gtbs_init(1, NULL);
    wiced_bt_ga_gtbs_enable_client();

    gatt_interface_setup_services_from_local_db(lehs_check_to_save_local,
                                                lehs_store_service_ref_local,
                                                &g_lehs_gatt_cb.local_profiles);
    gatt_interface_print_linked_handles(bda);

    return gatt_status;
}

void on_init_operation_complete(uint16_t conn_id,
                                gatt_intf_service_object_t *p_service,
                                gatt_intf_operation_t operation,
                                wiced_bt_gatt_status_t status)
{
    lehs_clcb_t *p_clcb = lehs_gatt_get_clcb_by_conn_id(conn_id);

    WICED_BT_TRACE("[%s] op %d status %d service %x %s",__FUNCTION__,
                   operation,
                   status,
                   p_service,
                   gatt_interface_get_service_name(p_service));

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
        le_audio_rpc_send_app_status(conn_id, HCI_CONTROL_LEA_APP_STATE_READY, 0);
        WICED_BT_TRACE("[%s] update state ready", __FUNCTION__);
        return;
    }

    if (p_service) {

        status = gatt_interface_characteristic_operation(conn_id, p_service, operation, on_init_operation_complete);

        if (WICED_BT_GATT_SUCCESS != status) {
            wiced_bt_gatt_disconnect(conn_id);
        }
        else
        {
            le_audio_rpc_send_app_operation_status(conn_id, p_service, operation);
        }
    }
}

void lehs_gatt_handle_discovery_complete(uint16_t conn_id,  wiced_bt_gatt_status_t status)
{
    lehs_clcb_t *p_clcb = NULL;

    p_clcb = lehs_gatt_get_clcb_by_conn_id(conn_id);
    if (!p_clcb) return;
    if (status)
    {
        WICED_BT_TRACE("[%s] status %d", __FUNCTION__, status);
        // wiced_bt_gatt_disconnect here
        return;
    }

    le_audio_rpc_send_app_status(conn_id, HCI_CONTROL_LEA_APP_STATE_DISCOVERY_COMPLETE, status);
    gatt_interface_print_linked_handles(p_clcb->bda);

    {
        gatt_intf_service_object_t *p_service = gatt_interface_get_linked_server_profile_at(0);

        status = gatt_interface_characteristic_operation(conn_id,
                                                p_service,
                                                GATT_INTF_OPERATION_NOTIFY_ALL_CHARACTERISTICS,
                                                on_init_operation_complete);
        if (status == WICED_BT_GATT_SUCCESS)
        {
            le_audio_rpc_send_app_status(conn_id,
                                         HCI_CONTROL_LEA_APP_STATE_INITING,
                                         HCI_CONTROL_LEA_APP_STATE_INIT_NOTIFYING);
        }
    }

}

void lehs_gatt_start_discovery(uint8_t *p_bd_addr)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    lehs_clcb_t *p_clcb = NULL;

    p_clcb = lehs_gatt_get_clcb(p_bd_addr);
    if (!p_clcb) return;

    status = gatt_interface_start_discovery(p_clcb->conn_id,
                                            lehs_check_to_save_peer,
                                            lehs_store_service_ref_peer,
                                            lehs_gatt_handle_discovery_complete,
                                            &p_clcb->peer_profiles);

    if (status) WICED_BT_TRACE_CRIT("[%s] status [%d] \n", __FUNCTION__, status);
}


lehs_gatt_cb_t *broadcast_sink_gatt_get_cb(uint8_t *p_bd_addr)
{
    lehs_gatt_cb_t *p_gatt_cb = &g_lehs_gatt_cb;

    return p_gatt_cb;
}

lehs_gatt_cb_t *broadcast_sink_gatt_get_cb_by_conn_id(uint16_t conn_id)
{
    lehs_gatt_cb_t *p_gatt_cb = &g_lehs_gatt_cb;

    return p_gatt_cb;
}

gatt_intf_service_object_t *lehs_gatt_get_bass_service_instance(void)
{
    return g_lehs_gatt_cb.local_profiles.p_bass;
}
