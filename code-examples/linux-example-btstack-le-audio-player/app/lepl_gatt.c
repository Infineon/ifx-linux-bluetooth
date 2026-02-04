/*
 * $ Copyright Cypress Semiconductor $
 */

/* Application includes */
#include "lepl.h"

extern wiced_bt_cfg_ble_t lepl_ble_cfg;
extern wiced_bt_heap_t *p_unicast_heap;
extern wiced_bt_ga_tbs_call_operation_result_t terminate_call(uint8_t call_id,
                                                       wiced_bt_ga_tbs_call_termination_reason_t *reason);
lepl_gatt_cb_t g_lepl_gatt_cb;

enum
{
    HDLS_GENERIC_ATTRIBUTE = 1, // 0x0001 , 1
    #if (ENABLE_MCS == 1)
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
    #endif // ENABLE_MCS == 1

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

    #if (ENABLE_TBS == 1)
    HDLS_TBS = 400,                                         // 0x0190 , 400
    HDLC_TBS_BEARER_PROVIDER_NAME,                          // 0x0191 , 401
    HDLC_TBS_BEARER_PROVIDER_NAME_VALUE,                    // 0x0192 , 402
    HDLD_TBS_BEARER_PROVIDER_NAME_CLIENT_CONFIGURATION,     // 0x0193 , 403
    HDLC_TBS_BEARER_UCI,                                    // 0x0194 , 404
    HDLC_TBS_BEARER_UCI_VALUE,                              // 0x0195 , 405
    HDLC_TBS_BEARER_TECHNOLOGY,                             // 0x0196 , 406
    HDLC_TBS_BEARER_TECHNOLOGY_VALUE,                       // 0x0197 , 407
    HDLD_TBS_BEARER_TECHNOLOGY_CLIENT_CONFIGURATION,        // 0x0198 , 408
    HDLC_TBS_BEARER_URI_SCHEMES,                            // 0x0199 , 409
    HDLC_TBS_BEARER_URI_SCHEMES_VALUE,                      // 0x019A , 410
    HDLD_TBS_BEARER_URI_SCHEMES_CLIENT_CONFIGURATION,       // 0x019B , 411
    HDLC_TBS_BEARER_SIGNAL_STRENGTH,                        // 0x019C , 412
    HDLC_TBS_BEARER_SIGNAL_STRENGTH_VALUE,                  // 0x019D , 413
    HDLD_TBS_BEARER_SIGNAL_STRENGTH_CLIENT_CONFIGURATION,   // 0x019E , 414
    HDLC_TBS_BEARER_SIG_STR_REPORTING_INTERVAL,             // 0x019F , 415
    HDLC_TBS_BEARER_SIG_STR_REPORTING_INTERVAL_VALUE,       // 0x0200 , 416
    HDLC_TBS_BEARER_LIST_CURRENT_CALL,                      // 0x0201 , 417
    HDLC_TBS_BEARER_LIST_CURRENT_CALL_VALUE,                // 0x0202 , 418
    HDLD_TBS_BEARER_LIST_CURRENT_CALL_CLIENT_CONFIGURATION, // 0x0203 , 419
    HDLC_TBS_CONTENT_CONTROL_ID,                            // 0x0204 , 420
    HDLC_TBS_CONTENT_CONTROL_ID_VALUE,                      // 0x0205 , 421
    HDLC_TBS_STATUS_FLAG,                                   // 0x0206 , 422
    HDLC_TBS_STATUS_FLAG_VALUE,                             // 0x0207 , 423
    HDLD_TBS_STATUS_FLAG_CLIENT_CONFIGURATION,              // 0x0208 , 424
    HDLC_TBS_INCOMING_TG_URI,                               // 0x0209 , 425
    HDLC_TBS_INCOMING_TG_URI_VALUE,                         // 0x020A , 426
    HDLD_TBS_INCOMING_TG_CALLER_ID_CLIENT_CONFIGURATION,    // 0x020B , 427
    HDLC_TBS_CALL_STATE,                                    // 0x020C , 428
    HDLC_TBS_CALL_STATE_VALUE,                              // 0x020D , 429
    HDLD_TBS_CALL_STATE_CLIENT_CONFIGURATION,               // 0x020E , 430
    HDLC_TBS_CALL_CONTROL_POINT,                            // 0x020F , 431
    HDLC_TBS_CALL_CONTROL_POINT_VALUE,                      // 0x0210 , 432
    HDLD_TBS_CALL_CONTROL_POINT_CLIENT_CONFIGURATION,       // 0x0211 , 433
    HDLC_TBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE,            // 0x0212 , 434
    HDLC_TBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE_VALUE,      // 0x0213 , 435
    HDLC_TBS_TERMINATION_REASON,                            // 0x0214 , 436
    HDLC_TBS_TERMINATION_REASON_VALUE,                      // 0x0215 , 437
    HDLD_TBS_TERMINATION_REASON_CLIENT_CONFIGURATION,       // 0x0216 , 438
    HDLC_TBS_INCOMING_CALL,                                 // 0x0217 , 439
    HDLC_TBS_INCOMING_CALL_VALUE,                           // 0x0218 , 430
    HDLD_TBS_INCOMING_CALL_CLIENT_CONFIGURATION,            // 0x0219 , 431
    HDLC_TBS_CALL_FRIENDLY_NAME,                            // 0x021A , 432
    HDLC_TBS_CALL_FRIENDLY_NAME_VALUE,                      // 0x021B , 433
    HDLD_TBS_CALL_FRIENDLY_NAME_CLIENT_CONFIGURATION,       // 0x021C , 434
    #endif // ENABLE_TBS = 1

    HDLS_GTBS = 544,                                         // 0x0220 , 544
    HDLC_GTBS_BEARER_PROVIDER_NAME,                          // 0x0221 , 545
    HDLC_GTBS_BEARER_PROVIDER_NAME_VALUE,                    // 0x0222 , 546
    HDLD_GTBS_BEARER_PROVIDER_NAME_CLIENT_CONFIGURATION,     // 0x0223 , 547
    HDLC_GTBS_BEARER_UCI,                                    // 0x0224 , 548
    HDLC_GTBS_BEARER_UCI_VALUE,                              // 0x0225 , 549
    HDLC_GTBS_BEARER_TECHNOLOGY,                             // 0x0226 , 550
    HDLC_GTBS_BEARER_TECHNOLOGY_VALUE,                       // 0x0227 , 551
    HDLD_GTBS_BEARER_TECHNOLOGY_CLIENT_CONFIGURATION,        // 0x0228 , 552
    HDLC_GTBS_BEARER_URI_SCHEMES,                            // 0x0229 , 553
    HDLC_GTBS_BEARER_URI_SCHEMES_VALUE,                      // 0x022A , 554
    HDLD_GTBS_BEARER_URI_SCHEMES_CLIENT_CONFIGURATION,       // 0x022B , 555
    HDLC_GTBS_BEARER_SIGNAL_STRENGTH,                        // 0x022C , 556
    HDLC_GTBS_BEARER_SIGNAL_STRENGTH_VALUE,                  // 0x022D , 557
    HDLD_GTBS_BEARER_SIGNAL_STRENGTH_CLIENT_CONFIGURATION,   // 0x022E , 558
    HDLC_GTBS_BEARER_SIG_STR_REPORTING_INTERVAL,             // 0x022F , 559
    HDLC_GTBS_BEARER_SIG_STR_REPORTING_INTERVAL_VALUE,       // 0x0230 , 560
    HDLC_GTBS_BEARER_LIST_CURRENT_CALL,                      // 0x0231 , 561
    HDLC_GTBS_BEARER_LIST_CURRENT_CALL_VALUE,                // 0x0232 , 562
    HDLD_GTBS_BEARER_LIST_CURRENT_CALL_CLIENT_CONFIGURATION, // 0x0233 , 563
    HDLC_GTBS_CONTENT_CONTROL_ID,                            // 0x0234 , 564
    HDLC_GTBS_CONTENT_CONTROL_ID_VALUE,                      // 0x0235 , 565
    HDLC_GTBS_INCOMING_TG_URI,                               // 0x0236 , 566
    HDLC_GTBS_INCOMING_TG_URI_VALUE,                         // 0x0237 , 567
    HDLD_GTBS_INCOMING_TG_CALLER_ID_CLIENT_CONFIGURATION,    // 0x0238 , 568
    HDLC_GTBS_STATUS_FLAG,                                   // 0x0239 , 569
    HDLC_GTBS_STATUS_FLAG_VALUE,                             // 0x023A , 570
    HDLD_GTBS_STATUS_FLAG_CLIENT_CONFIGURATION,              // 0x023B , 571
    HDLC_GTBS_CALL_STATE,                                    // 0x023C , 572
    HDLC_GTBS_CALL_STATE_VALUE,                              // 0x023D , 573
    HDLD_GTBS_CALL_STATE_CLIENT_CONFIGURATION,               // 0x023E , 574
    HDLC_GTBS_CALL_CONTROL_POINT,                            // 0x023F , 575
    HDLC_GTBS_CALL_CONTROL_POINT_VALUE,                      // 0x0240 , 576
    HDLD_GTBS_CALL_CONTROL_POINT_CLIENT_CONFIGURATION,       // 0x0241 , 577
    HDLC_GTBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE,            // 0x0242 , 578
    HDLC_GTBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE_VALUE,      // 0x0243 , 579
    HDLC_GTBS_TERMINATION_REASON,                            // 0x0244 , 580
    HDLC_GTBS_TERMINATION_REASON_VALUE,                      // 0x0245 , 581
    HDLD_GTBS_TERMINATION_REASON_CLIENT_CONFIGURATION,       // 0x0246 , 582
    HDLC_GTBS_INCOMING_CALL,                                 // 0x0247 , 583
    HDLC_GTBS_INCOMING_CALL_VALUE,                           // 0x0248 , 584
    HDLD_GTBS_INCOMING_CALL_CLIENT_CONFIGURATION,            // 0x0249 , 585
    HDLC_GTBS_CALL_FRIENDLY_NAME,                            // 0x024A , 586
    HDLC_GTBS_CALL_FRIENDLY_NAME_VALUE,                      // 0x024B , 587
    HDLD_GTBS_CALL_FRIENDLY_NAME_CLIENT_CONFIGURATION,       // 0x024C , 588
};

const uint8_t lepl_gatt_database[] = {
    /* Primary Service 'Generic Attribute' */
    PRIMARY_SERVICE_UUID16(HDLS_GENERIC_ATTRIBUTE, UUID_SERVICE_GATT),

    #if (ENABLE_MCS == 1)
    /* Primary Service 'MCS' */
    PRIMARY_SERVICE_UUID16(HDLS_MCS, WICED_BT_UUID_MEDIA_CONTROL),

    /* Characteristic 'Media Player Name' */
    CHARACTERISTIC_UUID16(HDLC_MCS_MEDIA_PLAYER_NAME,
                          HDLC_MCS_MEDIA_PLAYER_NAME_VALUE,
                          WICED_BT_UUID_MEDIA_PLAYER_NAME,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_MCS_PLAYER_NAME_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'Track Changed' */
    CHARACTERISTIC_UUID16(HDLC_MCS_MEDIA_TRACK_CHANGED,
                          HDLC_MCS_MEDIA_TRACK_CHANGED_VALUE,
                          WICED_BT_UUID_MEDIA_TRACK_CHANGED,
                          GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_NONE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_MCS_TRACK_CHANGED_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'Track Title' */
    CHARACTERISTIC_UUID16(HDLC_MCS_MEDIA_TRACK_TITLE,
                          HDLC_MCS_MEDIA_TRACK_TITLE_VALUE,
                          WICED_BT_UUID_MEDIA_TRACK_TITLE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_MCS_TRACK_TITLE_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'Track Duration' */
    CHARACTERISTIC_UUID16(HDLC_MCS_MEDIA_TRACK_DURATION,
                          HDLC_MCS_MEDIA_TRACK_DURATION_VALUE,
                          WICED_BT_UUID_MEDIA_TRACK_DURATION,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_MCS_TRACK_DURATION_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE
                                    | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'Track Position' */
    CHARACTERISTIC_UUID16_WRITABLE(
        HDLC_MCS_MEDIA_TRACK_POSITION,
        HDLC_MCS_MEDIA_TRACK_POSITION_VALUE,
        WICED_BT_UUID_MEDIA_TRACK_POSITION,
        GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE | GATTDB_CHAR_PROP_NOTIFY,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_MCS_TRACK_POSITION_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE),
    #if 0
    /* Characteristic 'Playback Speed' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_MCS_PLAYBACK_SPEED,
                                   HDLC_MCS_PLAYBACK_SPEED_VALUE,
                                   WICED_BT_UUID_MEDIA_PLAYBACK_SPEED,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE |
                                       GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_MCS_PLAYBACK_SPEED_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE
                                    | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'Seeking Speed' */
    CHARACTERISTIC_UUID16(HDLC_MCS_SEEKING_SPEED,
                          HDLC_MCS_SEEKING_SPEED_VALUE,
                          WICED_BT_UUID_MEDIA_SEEKING_SPEED,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_MCS_SEEKING_SPEED_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'Playing Order' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_MCS_PLAYING_ORDER,
                                   HDLC_MCS_PLAYING_ORDER_VALUE,
                                   WICED_BT_UUID_MEDIA_PLAYING_ORDER,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE |
                                       GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_MCS_PLAYING_ORDER_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),

    /* Characteristic 'Playing Order Supported' */
    CHARACTERISTIC_UUID16(HDLC_MCS_PLAYING_ORDER_SUPPORTED,
                          HDLC_MCS_PLAYING_ORDER_SUPPORTED_VALUE,
                          WICED_BT_UUID_MEDIA_PLAYING_ORDER_SUPPORTED,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    #endif

    /* Characteristic 'Media State' */
    CHARACTERISTIC_UUID16(HDLC_MCS_MEDIA_STATE,
                          HDLC_MCS_MEDIA_STATE_VALUE,
                          WICED_BT_UUID_MEDIA_STATE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_MCS_MEDIA_STATE_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),

    /* Characteristic 'Media Control Point ' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_MCS_MEDIA_CONTROL_POINT,
                                   HDLC_MCS_MEDIA_CONTROL_POINT_VALUE,
                                   WICED_BT_UUID_MEDIA_CONTROL_POINT,
                                   GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE |
                                       GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_WRITABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_MCS_CONTROL_POINT_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),

    /* Characteristic 'Media Control Point Opcode supported' */
    CHARACTERISTIC_UUID16(HDLC_MCS_MEDIA_CONTROL_POINT_OPCODE_SUPPORTED,
                          HDLC_MCS_MEDIA_CONTROL_POINT_OPCODE_SUPPORTED_VALUE,
                          WICED_BT_UUID_MEDIA_CONTROL_OPCODE_SUPPORTED,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_MCS_CONTROL_POINT_OPCODE_SUPPORTED_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Characteristic 'Content Control ID (CCID)' */
    CHARACTERISTIC_UUID16(HDLC_MCS_CONTENT_CONTROL_ID,
                          HDLC_MCS_CONTENT_CONTROL_ID_VALUE,
                          WICED_BT_UUID_MEDIA_CONTENT_CONTROL_ID,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    #endif // ENABLE_MCS == 1

    /* Primary Service 'GMCS' */
    PRIMARY_SERVICE_UUID16(HDLS_GMCS, WICED_BT_UUID_GENERIC_MEDIA_CONTROL),

    /* Characteristic 'Media Player Name' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_MEDIA_PLAYER_NAME,
                          HDLC_GMCS_MEDIA_PLAYER_NAME_VALUE,
                          WICED_BT_UUID_MEDIA_PLAYER_NAME,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_GMCS_PLAYER_NAME_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),

    /* Characteristic 'Track Changed' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_MEDIA_TRACK_CHANGED,
                          HDLC_GMCS_MEDIA_TRACK_CHANGED_VALUE,
                          WICED_BT_UUID_MEDIA_TRACK_CHANGED,
                          GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_NONE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_GMCS_TRACK_CHANGED_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),

    /* Characteristic 'Track Title' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_MEDIA_TRACK_TITLE,
                          HDLC_GMCS_MEDIA_TRACK_TITLE_VALUE,
                          WICED_BT_UUID_MEDIA_TRACK_TITLE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE  | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_GMCS_TRACK_TITLE_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),

    /* Characteristic 'Track Duration' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_MEDIA_TRACK_DURATION,
                          HDLC_GMCS_MEDIA_TRACK_DURATION_VALUE,
                          WICED_BT_UUID_MEDIA_TRACK_DURATION,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE  | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_GMCS_TRACK_DURATION_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),

    /* Characteristic 'Track Position' */
    CHARACTERISTIC_UUID16_WRITABLE(
        HDLC_GMCS_MEDIA_TRACK_POSITION,
        HDLC_GMCS_MEDIA_TRACK_POSITION_VALUE,
        WICED_BT_UUID_MEDIA_TRACK_POSITION,
        GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_GMCS_TRACK_POSITION_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),

    #if 0
    /* Characteristic 'Playback Speed' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_GMCS_PLAYBACK_SPEED,
                                   HDLC_GMCS_PLAYBACK_SPEED_VALUE,
                                   WICED_BT_UUID_MEDIA_PLAYBACK_SPEED,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY |
                                       GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
                                   GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_GMCS_PLAYBACK_SPEED_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),

    /* Characteristic 'Seeking Speed' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_SEEKING_SPEED,
                          HDLC_GMCS_SEEKING_SPEED_VALUE,
                          WICED_BT_UUID_MEDIA_SEEKING_SPEED,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE  | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_GMCS_SEEKING_SPEED_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),

    /* Characteristic 'Playing Order' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_GMCS_PLAYING_ORDER,
                                   HDLC_GMCS_PLAYING_ORDER_VALUE,
                                   WICED_BT_UUID_MEDIA_PLAYING_ORDER,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY |
                                       GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
                                   GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_GMCS_PLAYING_ORDER_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),

    /* Characteristic 'Playing Order Supported' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_PLAYING_ORDER_SUPPORTED,
                          HDLC_GMCS_PLAYING_ORDER_SUPPORTED_VALUE,
                          WICED_BT_UUID_MEDIA_PLAYING_ORDER_SUPPORTED,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    #endif
    /* Characteristic 'Media State' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_MEDIA_STATE,
                          HDLC_GMCS_MEDIA_STATE_VALUE,
                          WICED_BT_UUID_MEDIA_STATE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_GMCS_MEDIA_STATE_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),

    /* Characteristic 'Media Control Point ' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_GMCS_MEDIA_CONTROL_POINT,
                                   HDLC_GMCS_MEDIA_CONTROL_POINT_VALUE,
                                   WICED_BT_UUID_MEDIA_CONTROL_POINT,
                                   GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE |
                                       GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_WRITABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_GMCS_CONTROL_POINT_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE  | GATTDB_PERM_AUTH_READABLE ),

    /* Characteristic 'Media Control Point Opcode supported' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_MEDIA_CONTROL_POINT_OPCODE_SUPPORTED,
                          HDLC_GMCS_MEDIA_CONTROL_POINT_OPCODE_SUPPORTED_VALUE,
                          WICED_BT_UUID_MEDIA_CONTROL_OPCODE_SUPPORTED,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
                                    HDLD_GMCS_CONTROL_POINT_OPCODE_SUPPORTED_DESCRIPTION_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITABLE | GATTDB_PERM_AUTH_READABLE ),

    /* Characteristic 'Content Control ID (CCID)' */
    CHARACTERISTIC_UUID16(HDLC_GMCS_CONTENT_CONTROL_ID,
                          HDLC_GMCS_CONTENT_CONTROL_ID_VALUE,
                          WICED_BT_UUID_MEDIA_CONTENT_CONTROL_ID,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

#if (ENABLE_TBS == 1)

             /* Primary Service 'generic telephone_bearer_service' */
    PRIMARY_SERVICE_UUID16(HDLS_TBS, WICED_BT_UUID_TELEPHONE_BEARER),

    /* Characteristic 'bearer_provider_name' */
    CHARACTERISTIC_UUID16(HDLC_TBS_BEARER_PROVIDER_NAME,
                          HDLC_TBS_BEARER_PROVIDER_NAME_VALUE,
                          WICED_BT_UUID_TBS_BEARER_PROVIDER_NAME,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_TBS_BEARER_PROVIDER_NAME_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE |
                                        GATTDB_PERM_WRITE_CMD),

    /* Characteristic 'bearer_UCI' */
    CHARACTERISTIC_UUID16(HDLC_TBS_BEARER_UCI,
                          HDLC_TBS_BEARER_UCI_VALUE,
                          WICED_BT_UUID_TBS_BEARER_UCI,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'bearer_technology' */
    CHARACTERISTIC_UUID16(HDLC_TBS_BEARER_TECHNOLOGY,
                          HDLC_TBS_BEARER_TECHNOLOGY_VALUE,
                          WICED_BT_UUID_TBS_BEARER_TECHNOLOGY,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_TBS_BEARER_TECHNOLOGY_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE |
                                        GATTDB_PERM_WRITE_CMD),

    /* Characteristic 'bearer_URI_prefix' */
    CHARACTERISTIC_UUID16(HDLC_TBS_BEARER_URI_SCHEMES,
                          HDLC_TBS_BEARER_URI_SCHEMES_VALUE,
                          WICED_BT_UUID_TBS_BEARER_URI_SCHEMES,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_TBS_BEARER_URI_SCHEMES_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE |
                                        GATTDB_PERM_WRITE_CMD),

    /* Characteristic 'bearer_signal_strength' */
    CHARACTERISTIC_UUID16(HDLC_TBS_BEARER_SIGNAL_STRENGTH,
                          HDLC_TBS_BEARER_SIGNAL_STRENGTH_VALUE,
                          WICED_BT_UUID_TBS_BEARER_SIGNAL_STRENGTH,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_TBS_BEARER_SIGNAL_STRENGTH_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE |
                                        GATTDB_PERM_WRITE_CMD),

    /* Characteristic 'bearer_sig_str_reporting_interval' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_TBS_BEARER_SIG_STR_REPORTING_INTERVAL,
                                   HDLC_TBS_BEARER_SIG_STR_REPORTING_INTERVAL_VALUE,
                                   WICED_BT_UUID_TBS_SIG_STR_REPORTING_INTERVAL,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
                                   GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ |
                                       GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_AUTH_WRITABLE),

    /* Characteristic 'bearer_list_current_call' */
    CHARACTERISTIC_UUID16(HDLC_TBS_BEARER_LIST_CURRENT_CALL,
                          HDLC_TBS_BEARER_LIST_CURRENT_CALL_VALUE,
                          WICED_BT_UUID_TBS_LIST_CURRENT_CALL,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_TBS_BEARER_LIST_CURRENT_CALL_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE |
                                        GATTDB_PERM_WRITE_CMD),

    /* Characteristic 'content_control_ID' */
    CHARACTERISTIC_UUID16(HDLC_TBS_CONTENT_CONTROL_ID,
                          HDLC_TBS_CONTENT_CONTROL_ID_VALUE,
                          WICED_BT_UUID_TBS_CONTENT_CONTROL_ID,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'status_flags' */
    CHARACTERISTIC_UUID16(HDLC_TBS_STATUS_FLAG,
                          HDLC_TBS_STATUS_FLAG_VALUE,
                          WICED_BT_UUID_TBS_STATUS_FLAGS,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_TBS_STATUS_FLAG_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE |
                                        GATTDB_PERM_WRITE_CMD),

    /* Characteristic 'incoming_tg_caller_ID' */
    CHARACTERISTIC_UUID16(HDLC_TBS_INCOMING_TG_URI,
                          HDLC_TBS_INCOMING_TG_URI_VALUE,
                          WICED_BT_UUID_TBS_INCOMING_TG_URI,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_TBS_INCOMING_TG_CALLER_ID_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE |
                                        GATTDB_PERM_WRITE_CMD),

    /* Characteristic 'call_state' */
    CHARACTERISTIC_UUID16(HDLC_TBS_CALL_STATE,
                          HDLC_TBS_CALL_STATE_VALUE,
                          WICED_BT_UUID_TBS_CALL_STATE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_TBS_CALL_STATE_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD |
                                        GATTDB_PERM_AUTH_WRITABLE | GATTDB_PERM_WRITE_CMD),

    /* Characteristic 'call_control_point' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_TBS_CALL_CONTROL_POINT,
                                   HDLC_TBS_CALL_CONTROL_POINT_VALUE,
                                   WICED_BT_UUID_TBS_CALL_CONTROL_POINT,
                                   GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE |
                                       GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_TBS_CALL_CONTROL_POINT_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE |
                                        GATTDB_PERM_WRITE_CMD),

    /* Characteristic 'call control point optional opcodes' */
    CHARACTERISTIC_UUID16(HDLC_TBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE,
                          HDLC_TBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE_VALUE,
                          WICED_BT_UUID_TBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'termination_reason' */
    CHARACTERISTIC_UUID16(HDLC_TBS_TERMINATION_REASON,
                          HDLC_TBS_TERMINATION_REASON_VALUE,
                          WICED_BT_UUID_TBS_TERMINATION_REASON,
                          GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_NONE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_TBS_TERMINATION_REASON_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE |
                                        GATTDB_PERM_WRITE_CMD),

    /* Characteristic 'incoming call' */
    CHARACTERISTIC_UUID16(HDLC_TBS_INCOMING_CALL,
                          HDLC_TBS_INCOMING_CALL_VALUE,
                          WICED_BT_UUID_TBS_INCOMING_CALL,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_TBS_INCOMING_CALL_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE |
                                        GATTDB_PERM_WRITE_CMD),

    /* Characteristic 'incoming call friendly name' */
    CHARACTERISTIC_UUID16(HDLC_TBS_CALL_FRIENDLY_NAME,
                          HDLC_TBS_CALL_FRIENDLY_NAME_VALUE,
                          WICED_BT_UUID_TBS_CALL_FRIENDLY_NAME,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_TBS_CALL_FRIENDLY_NAME_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE |
                                        GATTDB_PERM_WRITE_CMD),
#endif // (ENABLE_TBS == 1)


    /* Primary Service 'generic telephone_bearer_service' */
    PRIMARY_SERVICE_UUID16(HDLS_GTBS, WICED_BT_UUID_GENERIC_TELEPHONE_BEARER),

    /* Characteristic 'bearer_provider_name' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_BEARER_PROVIDER_NAME,
                          HDLC_GTBS_BEARER_PROVIDER_NAME_VALUE,
                          WICED_BT_UUID_TBS_BEARER_PROVIDER_NAME,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_GTBS_BEARER_PROVIDER_NAME_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Characteristic 'bearer_UCI' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_BEARER_UCI,
                          HDLC_GTBS_BEARER_UCI_VALUE,
                          WICED_BT_UUID_TBS_BEARER_UCI,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'bearer_technology' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_BEARER_TECHNOLOGY,
                          HDLC_GTBS_BEARER_TECHNOLOGY_VALUE,
                          WICED_BT_UUID_TBS_BEARER_TECHNOLOGY,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_GTBS_BEARER_TECHNOLOGY_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Characteristic 'bearer_URI_prefix' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_BEARER_URI_SCHEMES,
                          HDLC_GTBS_BEARER_URI_SCHEMES_VALUE,
                          WICED_BT_UUID_TBS_BEARER_URI_SCHEMES,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_GTBS_BEARER_URI_SCHEMES_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    #if 0
    /* Characteristic 'bearer_signal_strength' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_BEARER_SIGNAL_STRENGTH,
                          HDLC_GTBS_BEARER_SIGNAL_STRENGTH_VALUE,
                          WICED_BT_UUID_TBS_BEARER_SIGNAL_STRENGTH,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_GTBS_BEARER_SIGNAL_STRENGTH_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),


    /* Characteristic 'bearer_sig_str_reporting_interval' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_GTBS_BEARER_SIG_STR_REPORTING_INTERVAL,
                                   HDLC_GTBS_BEARER_SIG_STR_REPORTING_INTERVAL_VALUE,
                                   WICED_BT_UUID_TBS_SIG_STR_REPORTING_INTERVAL,
                                   GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
                                   GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),
    #endif

    /* Characteristic 'bearer_list_current_call' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_BEARER_LIST_CURRENT_CALL,
                          HDLC_GTBS_BEARER_LIST_CURRENT_CALL_VALUE,
                          WICED_BT_UUID_TBS_LIST_CURRENT_CALL,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_GTBS_BEARER_LIST_CURRENT_CALL_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Characteristic 'content_control_ID' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_CONTENT_CONTROL_ID,
                          HDLC_GTBS_CONTENT_CONTROL_ID_VALUE,
                          WICED_BT_UUID_TBS_CONTENT_CONTROL_ID,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'status_flags' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_STATUS_FLAG,
                          HDLC_GTBS_STATUS_FLAG_VALUE,
                          WICED_BT_UUID_TBS_STATUS_FLAGS,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_GTBS_STATUS_FLAG_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    #if 0
    /* Characteristic 'incoming_tg_caller_ID' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_INCOMING_TG_URI,
                          HDLC_GTBS_INCOMING_TG_URI_VALUE,
                          WICED_BT_UUID_TBS_INCOMING_TG_URI,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),
    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_GTBS_INCOMING_TG_CALLER_ID_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),
    #endif

    /* Characteristic 'call_state' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_CALL_STATE,
                          HDLC_GTBS_CALL_STATE_VALUE,
                          WICED_BT_UUID_TBS_CALL_STATE,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_GTBS_CALL_STATE_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Characteristic 'call_control_point' */
    CHARACTERISTIC_UUID16_WRITABLE(HDLC_GTBS_CALL_CONTROL_POINT,
                                   HDLC_GTBS_CALL_CONTROL_POINT_VALUE,
                                   WICED_BT_UUID_TBS_CALL_CONTROL_POINT,
                                   GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_WRITE_NO_RESPONSE |
                                       GATTDB_CHAR_PROP_NOTIFY,
                                   GATTDB_PERM_WRITABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_GTBS_CALL_CONTROL_POINT_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Characteristic 'call control point optional opcodes' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE,
                          HDLC_GTBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE_VALUE,
                          WICED_BT_UUID_TBS_CALL_CONTROL_POINT_OPTIONAL_OPCODE,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Characteristic 'termination_reason' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_TERMINATION_REASON,
                          HDLC_GTBS_TERMINATION_REASON_VALUE,
                          WICED_BT_UUID_TBS_TERMINATION_REASON,
                          GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_NONE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_GTBS_TERMINATION_REASON_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Characteristic 'incoming call' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_INCOMING_CALL,
                          HDLC_GTBS_INCOMING_CALL_VALUE,
                          WICED_BT_UUID_TBS_INCOMING_CALL,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_GTBS_INCOMING_CALL_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

    /* Characteristic 'incoming call friendly name' */
    CHARACTERISTIC_UUID16(HDLC_GTBS_CALL_FRIENDLY_NAME,
                          HDLC_GTBS_CALL_FRIENDLY_NAME_VALUE,
                          WICED_BT_UUID_TBS_CALL_FRIENDLY_NAME,
                          GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                          GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE),

    /* Descriptor 'Client Characteristic Configuration' */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_GTBS_CALL_FRIENDLY_NAME_CLIENT_CONFIGURATION,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITABLE),

};

static void lepl_gatt_scan_cb(wiced_ble_ext_scan_results_t *p_scr, uint16_t adv_len, uint8_t *p_adv_data)
{
    uint8_t *p_data;
    uint16_t adv_entry_length = 0;
    uint16_t uuid;
    wiced_bt_device_address_t peer_addr;

    if (p_scr == NULL)
    {
        return;
    }
#ifndef CLI_SUPPORT
    WICED_BT_TRACE("[%s] p_scan_result : %B", __FUNCTION__, p_scr->remote_bd_addr);
#endif
    if (g_lepl_gatt_cb.enable_uuid_filter == WICED_TRUE)
    {
        wiced_bt_adv_ctx_t ctx = {.adv_len = adv_len, .p_adv = p_adv_data};
        p_data = wiced_ble_adv_data_search(&ctx, BTM_BLE_ADVERT_TYPE_SERVICE_DATA, & adv_entry_length);
        WICED_BT_TRACE("[%s] offset %d length %d", __FUNCTION__, ctx.offset, adv_entry_length);
        if (p_data == NULL || adv_entry_length == 0)
        {
            return;
        }

        while (adv_entry_length) /*if no entry is found adv_entry_length will be 0*/
        {
            STREAM_TO_UINT16(uuid, p_data);
            WICED_BT_TRACE("[%s] uuid found %x", __FUNCTION__, uuid);
            memcpy(&peer_addr, p_scr->remote_bd_addr, BD_ADDR_LEN);
            if (uuid == WICED_BT_UUID_AUDIO_STREAM_CONTROL)
            {
#ifdef CLI_SUPPORT
                le_pl_cli_add_sink_dev(p_scr);
#else
                WICED_BT_TRACE("[%s] found ASCS device", __FUNCTION__);
#endif
                le_audio_rpc_send_scan_res_event(p_scr, p_adv_data);
                return;
            }
            else
            {
                p_data =
                    wiced_ble_adv_data_search(&ctx, BTM_BLE_ADVERT_TYPE_SERVICE_DATA,& adv_entry_length);
            }
        }
    }
    else
    {
        le_audio_rpc_send_scan_res_event(p_scr, p_adv_data);
        WICED_BT_TRACE("[%s] filter disabled",__FUNCTION__);
    }

}

wiced_ble_ext_scan_params_t scan_params = {.own_addr_type = BLE_ADDR_PUBLIC,
                                              .scanning_phys = WICED_BLE_EXT_ADV_PHY_1M_BIT,
                                              .scan_filter_policy = WICED_BLE_SCAN_BASIC_UNFILTERED_SP,
                                              .sp_1m.scan_type = BTM_BLE_SCAN_MODE_ACTIVE,
                                              .sp_1m.scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,
                                              .sp_1m.scan_window = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW};

wiced_ble_ext_scan_enable_params_t scan_enable_params = {.filter_duplicates = 0,
                                                            .scan_period = 0,
                                                            .scan_duration = 0};

wiced_result_t lepl_start_stop_scan(uint32_t start, wiced_ble_ext_scan_result_cback_t *p_cback)
{
    wiced_result_t status;

    status = wiced_ble_ext_scan_register_cb(p_cback);
    wiced_ble_ext_scan_configure_reassembly(255, 2);
    status = wiced_ble_ext_scan_set_params(&scan_params);
    {
        wiced_ble_ext_scan_enable_params_t enable = scan_enable_params;
        status = wiced_ble_ext_scan_enable(start, &enable);
    }

    return status;
}


wiced_result_t lepl_gatt_start_stop_scan(uint32_t start, uint8_t enable_uuid_filter)
{
    g_lepl_gatt_cb.enable_uuid_filter = enable_uuid_filter;
    wiced_result_t status = lepl_start_stop_scan(start, lepl_gatt_scan_cb);

    return status;
}

lepl_clcb_t *lepl_gatt_alloc_cb(uint8_t *p_bd_addr,
                                wiced_bt_ble_address_type_t addr_type,
                                uint16_t conn_id,
                                uint16_t link_role)
{
    int index;
    lepl_clcb_t *p_clcb = NULL;
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        p_clcb = &g_lepl_gatt_cb.unicast_clcb[index];
        if (p_clcb->in_use == FALSE)
        {
            p_clcb->in_use = TRUE;
            p_clcb->conn_id = conn_id;
            p_clcb->addr_type = addr_type;
            memcpy(p_clcb->bda, p_bd_addr, BD_ADDR_LEN);
            p_clcb->b_is_central = (HCI_ROLE_CENTRAL == link_role) ? TRUE : FALSE;
            p_clcb->app_state = LEPL_GATT_STATE_CONNECTED;
            p_clcb->p_cap->conn_id = conn_id;
            return p_clcb;
        }
    }
    return p_clcb;
}

lepl_clcb_t *lepl_gatt_get_clcb(uint8_t *p_bd_addr)
{
    lepl_clcb_t *p_clcb = NULL;
    int index;
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++) {
        p_clcb = &g_lepl_gatt_cb.unicast_clcb[index];
        if (p_clcb->in_use && !WICED_MEMCMP(p_clcb->bda, p_bd_addr, BD_ADDR_LEN)) {
            return p_clcb;
        }
    }
    return NULL;
}

wiced_bt_gatt_status_t lepl_gatt_free_cb(uint8_t *p_bd_addr)
{
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb(p_bd_addr);
    if (!p_clcb) return WICED_ERROR;

    p_clcb->app_state = LEPL_GATT_STATE_DISCONNECTED;
    p_clcb->in_use = FALSE;
    return WICED_SUCCESS;
}

lepl_clcb_t *lepl_gatt_get_clcb_by_conn_id(uint16_t conn_id)
{
    lepl_clcb_t *p_clcb = NULL;
    int index;
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++) {
        p_clcb = &g_lepl_gatt_cb.unicast_clcb[index];
        if (p_clcb->in_use && (p_clcb->conn_id == conn_id)) {
            return p_clcb;
        }
    }
    return NULL;
}

static void lepl_gatt_handle_connection(wiced_bt_gatt_connection_status_t *p_conn_sts)
{
    lepl_clcb_t *p_clcb =
        lepl_gatt_alloc_cb(p_conn_sts->bd_addr, p_conn_sts->addr_type, p_conn_sts->conn_id, p_conn_sts->link_role);

    WICED_BT_TRACE("[%s] connected to [%B] clcb 0x%x\n", __FUNCTION__, p_conn_sts->bd_addr, p_clcb);

    /* Allocate GATT control block */
    if (!p_clcb)
    {
        // Assert !!
        return;
    }
    extern void set_current_device(uint16_t conn_id, wiced_bt_device_address_t address);
    set_current_device(p_conn_sts->conn_id, p_conn_sts->bd_addr);

    le_audio_rpc_send_app_status(p_conn_sts->conn_id, HCI_CONTROL_LEA_APP_STATE_CONNECTED, 0);
    lepl_init_remote_ases(p_clcb);

    /* Configure MTU */
    wiced_bt_gatt_client_configure_mtu(p_conn_sts->conn_id, lepl_ble_cfg.ble_max_rx_pdu_size);

    /* Inform CC */
    le_audio_rpc_send_connect_event(p_conn_sts);
}

void lepl_gatt_handle_disconnection(wiced_bt_gatt_connection_status_t *p_conn_sts)
{
    le_audio_rpc_send_disconnect_evt(p_conn_sts);
    le_audio_rpc_send_app_status(p_conn_sts->conn_id, HCI_CONTROL_LEA_APP_STATE_DISCONNECTED, 0);

    lepl_csis_handle_disconnection(p_conn_sts->bd_addr);
    lepl_gatt_free_cb(p_conn_sts->bd_addr);
    lepl_cap_reset_application_state();

    WICED_BT_TRACE("[%s] disconnected from [%B]\n", __FUNCTION__, p_conn_sts->bd_addr);
}

wiced_bt_gatt_status_t lepl_gatt_cback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_operation_complete_t *p_op_complete = &p_event_data->operation_complete;
    lepl_clcb_t *p_clcb = NULL;

    WICED_BT_TRACE("[%s] event [0x%x] max_heap %d\n", __FUNCTION__,
                    event, wiced_bt_get_largest_heap_buffer(p_unicast_heap));

    /* invoke the gatt interface common handler*/
    status = gatt_interface_invoke_gatt_handler(event, p_event_data);

    switch (event) {
        case GATT_CONNECTION_STATUS_EVT:
            (p_event_data->connection_status.connected)
                ? lepl_gatt_handle_connection(&p_event_data->connection_status)
                : lepl_gatt_handle_disconnection(&p_event_data->connection_status);
            break;

        case GATT_OPERATION_CPLT_EVT:
            WICED_BT_TRACE("[%s] op %d", __FUNCTION__, p_event_data->operation_complete.op);

            p_clcb = lepl_gatt_get_clcb_by_conn_id(p_op_complete->conn_id);
            if (!p_clcb)
            {
                return status;
            }

            if (GATTC_OPTYPE_CONFIG_MTU == p_op_complete->op && p_clcb->app_state == LEPL_GATT_STATE_CONNECTED) {
                p_clcb->app_state = LEPL_GATT_STATE_MTU_CONFIGURED;

                le_audio_rpc_send_app_status(p_clcb->conn_id, HCI_CONTROL_LEA_APP_STATE_MTU_CONFIGURED, wiced_bt_gatt_get_bearer_mtu(p_clcb->conn_id));
                /* TODO: if bonded, get GATT Db info from NVRAM and start encrption, else start bonding*/
                if (p_clcb->b_is_central)
                {
                    wiced_bt_dev_sec_bond(p_clcb->bda, p_clcb->addr_type, BT_TRANSPORT_LE, 0, NULL);
                }

                return WICED_SUCCESS;
            }
            else if (GATTC_OPTYPE_WRITE_WITH_RSP == p_op_complete->op) {
                WICED_BT_TRACE("[%s] op %d state %d", __FUNCTION__, p_op_complete->op, p_clcb->app_state);
            }
            else if (GATTC_OPTYPE_READ_HANDLE == p_op_complete->op) {
                WICED_BT_TRACE("[%s] op %d %d", __FUNCTION__, p_op_complete->op, p_clcb->app_state);
            }

            break;

        default:
            break;

    }

    return status;
}

wiced_bool_t lepl_check_to_save_local(void *p_app_ctx, gatt_intf_service_info_t *p_srv_info)
{
    uint16_t uuid = p_srv_info->group.service_type.uu.uuid16;

    /* This app supports only 16 bit UUID */
    if (LEN_UUID_16 != p_srv_info->group.service_type.len) return FALSE;

    if (uuid == WICED_BT_UUID_GENERIC_MEDIA_CONTROL || uuid == WICED_BT_UUID_MEDIA_CONTROL ||
        uuid == WICED_BT_UUID_GENERIC_TELEPHONE_BEARER /* || uuid == WICED_BT_UUID_TELEPHONE_BEARER*/)
        return TRUE;

    return FALSE;
}

wiced_bool_t lepl_check_to_save_peer(void *p_app_ctx, gatt_intf_service_info_t *p_srv_info)
{
    uint16_t uuid = p_srv_info->group.service_type.uu.uuid16;

    WICED_BT_TRACE("[%s] UUID 0x%x, Len %d\n", __FUNCTION__, uuid, p_srv_info->group.service_type.len);

    /* This app supports only 16 bit UUID */
    if (LEN_UUID_16 != p_srv_info->group.service_type.len) return FALSE;

    //peer profile
    if (uuid == WICED_BT_UUID_PUBLISHED_AUDIO_CAPABILITY || uuid == WICED_BT_UUID_AUDIO_STREAM_CONTROL ||
        uuid == WICED_BT_UUID_VOLUME_CONTROL || uuid == WICED_BT_UUID_COORDINATE_SET_IDENTIFICATION ||
        uuid == WICED_BT_UUID_MICROPHONE_CONTROL
#if HAP_ENABLED
        || uuid == WICED_BT_UUID_HEARING_ACCESS
#endif
        )
        return TRUE;
    return FALSE;
}

void lepl_store_service_ref_local(void *p_app_ctx, gatt_intf_service_info_t *p_srv_info, gatt_intf_service_object_t *p_service)
{
    gatt_intf_service_cb_t p_callback = NULL;
    uint16_t uuid = p_srv_info->group.service_type.uu.uuid16;
    lepl_local_profiles_t *p_profile = (lepl_local_profiles_t *)p_app_ctx;

    /* This app supports only 16 bit UUID */
    if (LEN_UUID_16 != p_srv_info->group.service_type.len) return;

    switch (uuid)
    {
    case WICED_BT_UUID_MEDIA_CONTROL:
        WICED_BT_TRACE("[%s] MCS \n", __FUNCTION__);
        p_profile->p_mcs = p_service;
        p_callback = (gatt_intf_service_cb_t)lepl_mcs_callback;
        break;
    case WICED_BT_UUID_GENERIC_MEDIA_CONTROL:
        WICED_BT_TRACE("[%s] GMCS \n", __FUNCTION__);
        p_profile->p_gmcs = p_service;
        p_callback = (gatt_intf_service_cb_t)lepl_mcs_callback;
        break;
    case WICED_BT_UUID_GENERIC_TELEPHONE_BEARER:
        WICED_BT_TRACE("[%s] GTBS \n", __FUNCTION__);
        p_profile->p_gtbs = p_service;
        p_callback = (gatt_intf_service_cb_t)lepl_tbs_callback;
        break;
    /* case WICED_BT_UUID_TELEPHONE_BEARER:
        WICED_BT_TRACE("[%s] TBS \n", __FUNCTION__);
        p_profile->p_gtbs = p_service;
        p_callback = (gatt_intf_service_cb_t)lepl_tbs_callback;
        */
    default:
        break;
    }

    if (p_callback != NULL)
        gatt_interface_set_callback_to_profile(p_service, p_callback, p_app_ctx);
}

gatt_intf_service_object_t **lepl_get_profile_ptr_addr_int(gatt_intf_service_object_t **p_profile_list,
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

gatt_intf_service_object_t **lepl_get_peer_profile_ptr_addr(lepl_peer_profiles_t *p_profiles,
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
        WICED_BT_TRACE("[%s] VCS \n", __FUNCTION__);
        pp_profile = &p_profiles->p_vcs;
        p_callback = lepl_vcs_callback;
        break;
    case WICED_BT_UUID_MICROPHONE_CONTROL:
        WICED_BT_TRACE("[%s] MICS ", __FUNCTION__);
        pp_profile = &p_profiles->p_mics;
        p_callback = lepl_mics_callback;
        break;
    case WICED_BT_UUID_COORDINATE_SET_IDENTIFICATION:
        WICED_BT_TRACE("[%s] CSIS ", __FUNCTION__);
        pp_profile = &p_profiles->p_csis;
        p_callback = lepl_csis_callback;
        break;
    case WICED_BT_UUID_AUDIO_STREAM_CONTROL:
        WICED_BT_TRACE("[%s] ASCS \n", __FUNCTION__);
        pp_profile = &p_profiles->p_ascs;
        p_callback = lepl_ascs_callback;
        break;
    case WICED_BT_UUID_PUBLISHED_AUDIO_CAPABILITY:
        WICED_BT_TRACE("[%s] PACS \n", __FUNCTION__);
        pp_profile = &p_profiles->p_pacs;
        p_callback = lepl_pacs_callback;
        break;
    case WICED_BT_UUID_HEARING_ACCESS:
        WICED_BT_TRACE("[%s] HAS ", __FUNCTION__);
        pp_profile = &p_profiles->p_has;
        p_callback = lepl_has_callback;
        break;
        #if 0
    case UUID_SERVICE_IMMEDIATE_ALERT:
        pp_profile = &p_profiles->p_ias;
        p_callback = (gatt_intf_service_cb_t)lepl_ias_callback;
        break;
        #endif

    case WICED_BT_UUID_AUDIO_INPUT_CONTROL:
    {
        uint16_t parent_uuid = p_srv_info->included_by_uuid.uu.uuid16;

        if (parent_uuid == WICED_BT_UUID_MICROPHONE_CONTROL)
        {
            pp_profile = lepl_get_profile_ptr_addr_int(p_profiles->p_mics_aics,
                                                            MAX_MICS_AICS,
                                                            &p_profiles->num_mics_aics,
                                                            incr);
            p_callback = lepl_mics_aics_callback;
        }
    }
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

void lepl_store_service_ref_peer(void *p_app_ctx,
                                 gatt_intf_service_info_t *p_srv_info,
                                 gatt_intf_service_object_t *p_service)
{
    lepl_peer_profiles_t *p_profile = (lepl_peer_profiles_t
        *)p_app_ctx;
    gatt_intf_service_cb_t p_callback = NULL;

    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    gatt_intf_service_object_t **pp_service =
        lepl_get_peer_profile_ptr_addr(p_profile, p_srv_info, 0, &p_callback);

    if (pp_service)
    {
        *pp_service = p_service;
    }
    if (p_callback != NULL)
    {
        gatt_interface_set_callback_to_profile(p_service, p_callback, p_app_ctx);
    }
}

void on_init_operation_complete(uint16_t conn_id,
                                        gatt_intf_service_object_t *p_service,
                                        gatt_intf_operation_t operation,
                                        wiced_bt_gatt_status_t status)
{
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);

    WICED_BT_TRACE("[%s] op %d status %d service %x %s", __FUNCTION__,
                   operation,
                   status,
                   p_service,
                   gatt_interface_get_service_name(p_service));

    if (status != WICED_BT_GATT_SUCCESS && status != WICED_BT_GATT_ATTRIBUTE_NOT_FOUND)
    {
        lepl_gatt_disconnect(conn_id);
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
        p_clcb->app_state = LEPL_GATT_STATE_READY;
        le_audio_rpc_send_app_status(conn_id, HCI_CONTROL_LEA_APP_STATE_READY, 0);
        if (p_clcb->peer_profiles.p_csis) {
            lepl_start_stop_set_member_discovery(p_clcb->csis_data.sirk_data.sirk, WICED_TRUE);
        }
        WICED_BT_TRACE("[%s] update state ready", __FUNCTION__);
        return;
    }

    if (p_service)
    {
        status = gatt_interface_characteristic_operation(conn_id, p_service, operation, on_init_operation_complete);

        if (WICED_BT_GATT_SUCCESS != status)
        {
            lepl_gatt_disconnect(conn_id);
        }
        else
        {
            le_audio_rpc_send_app_operation_status(conn_id, p_service, operation);
        }
    }

}

void lepl_gatt_handle_discovery_complete(uint16_t conn_id, wiced_bt_gatt_status_t status)
{
    lepl_clcb_t *p_clcb = lepl_gatt_get_clcb_by_conn_id(conn_id);
    if (!p_clcb) return;

    p_clcb->app_state = LEPL_GATT_STATE_DISCOVERY_COMPLETE;
    le_audio_rpc_send_app_status(conn_id, HCI_CONTROL_LEA_APP_STATE_DISCOVERY_COMPLETE, status);
    if (status)
    {
        WICED_BT_TRACE("[%s] status %d", __FUNCTION__, status);
        return;
    }
    gatt_interface_print_linked_handles(p_clcb->bda);

    // Store the handles into the cap data.


    /* start ASE discovery (read all the ASE char on the peer to get the list of ASE ID's) */
    p_clcb->app_state = LEPL_GATT_STATE_INITING;

    {
        gatt_intf_service_object_t *p_service = gatt_interface_get_linked_server_profile_at(0);

        status = gatt_interface_characteristic_operation(conn_id,
                                                p_service,
                                                GATT_INTF_OPERATION_NOTIFY_ALL_CHARACTERISTICS,
                                                on_init_operation_complete);
        if (status == WICED_BT_GATT_SUCCESS)
        {
            le_audio_rpc_send_app_status(conn_id, HCI_CONTROL_LEA_APP_STATE_INITING, HCI_CONTROL_LEA_APP_STATE_INIT_NOTIFYING);
        }
    }
}

void lepl_gatt_start_discovery(uint8_t *p_bd_addr)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    lepl_clcb_t *p_clcb = NULL;

    p_clcb = lepl_gatt_get_clcb(p_bd_addr);
    if (!p_clcb) return;

    status = gatt_interface_start_discovery(p_clcb->conn_id,
                                            lepl_check_to_save_peer,
                                            lepl_store_service_ref_peer,
                                            lepl_gatt_handle_discovery_complete,
                                            &p_clcb->peer_profiles);

    if (status) WICED_BT_TRACE_CRIT("[%s] status [%d] \n", __FUNCTION__, status);
}

wiced_bt_gatt_status_t lepl_gatt_init(int max_connections, int max_mtu, lepl_ga_init_data_t *p_ga_init_data)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = {0};
    int index = 0;

    gatt_status = wiced_bt_gatt_db_init(lepl_gatt_database, sizeof(lepl_gatt_database), NULL);
    if (WICED_BT_SUCCESS != gatt_status) return gatt_status;

    gatt_status = wiced_bt_gatt_register(lepl_gatt_cback);
    if (WICED_BT_SUCCESS != gatt_status) return gatt_status;

    /* Initialize GATT Interface App library */
    gatt_status = gatt_interface_init(max_connections, max_mtu, GATT_AUTH_REQ_NONE);
    if (WICED_BT_SUCCESS != gatt_status) return gatt_status;

    /* Initialize CLCB */
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        lepl_clcb_t *p_clcb = &g_lepl_gatt_cb.unicast_clcb[index];
        p_clcb->p_cap = &g_lepl_gatt_cb.cap_data[index];

        p_clcb->p_cap->p_pacs_data = &p_clcb->pacs_data;
        p_clcb->p_cap->vcs_data = &p_clcb->vcs_data;
        WICED_BT_TRACE("%s cap_device[%d] 0x%x", __FUNCTION__, index, g_lepl_gatt_cb.cap_data[index]);

        p_clcb->p_cap->p_pacs_data->source_pac_list.record_list = p_clcb->pacs_src_record;
        p_clcb->p_cap->p_pacs_data->sink_pac_list.record_list = p_clcb->pacs_sink_record;
    }

    g_lepl_gatt_cb.cap_profile_data.device_info_list = (le_audio_cap_device_data_t *)g_lepl_gatt_cb.cap_data;

    lepl_ascs_alloc_memory(p_ga_init_data);

    /*Register CAP callabck */
    le_audio_cap_register_cb(&lepl_cap_event_cb);

//     /* Initialize the supported profiles */
//     wiced_bt_ga_mcs_init(1, NULL);
//     wiced_bt_ga_gmcs_init(p_ga_init_data);
//    // wiced_bt_ga_tbs_init(p_ga_init_data);
//     wiced_bt_ga_gtbs_init(p_ga_init_data);

//     /* Initialize the peer supported profiles */
//     wiced_bt_ga_ascs_init(p_ga_init_data);
//     wiced_bt_ga_pacs_init(p_ga_init_data);
//     wiced_bt_ga_vcs_init(p_ga_init_data);
//     wiced_bt_ga_csis_init(p_ga_init_data);
//     wiced_bt_ga_mics_init(p_ga_init_data);
// #if HAP_ENABLED
//     wiced_bt_ga_has_init(p_ga_init_data);
// #endif

//     extern void wiced_bt_ga_cap_disable_csis_locking(void);
//     wiced_bt_ga_cap_disable_csis_locking();

    /* Initialize the supported profiles */
    wiced_bt_ga_gmcs_init(1, NULL);
    wiced_bt_ga_gmcs_enable_server();
    wiced_bt_ga_gtbs_init(1, NULL);
    wiced_bt_ga_gtbs_enable_server();
#if TMAP_ENABLED
    wiced_bt_ga_tmap_init(1, NULL);
    wiced_bt_ga_tmap_enable_server();
#endif

    /* Initialize the peer supported profiles */
    wiced_bt_ga_ascs_init(1, p_ga_init_data->p_ascs_init_data);
    wiced_bt_ga_ascs_enable_client();
    wiced_bt_ga_pacs_init(1, p_ga_init_data->p_pacs_init_data);
    wiced_bt_ga_pacs_enable_client();
    wiced_bt_ga_vcs_init(1, p_ga_init_data->p_vcs_init_data);
    wiced_bt_ga_vcs_enable_client();
    wiced_bt_ga_csis_init(1, NULL);
    wiced_bt_ga_csis_enable_client();
    wiced_bt_ga_mics_init(1, p_ga_init_data->p_mics_init_data);
    wiced_bt_ga_mics_enable_client();
    wiced_bt_ga_aics_init(1 * MAX_MICS_AICS, NULL);
    wiced_bt_ga_aics_enable_client();
#if HAP_ENABLED
    wiced_bt_ga_has_init(1, NULL);
    wiced_bt_ga_has_enable_client();
#endif

    lepl_cap_reset_application_state();

    gatt_interface_setup_services_from_local_db(lepl_check_to_save_local,
                                                lepl_store_service_ref_local,
                                                &g_lepl_gatt_cb.local_profiles);
    gatt_interface_print_linked_handles(bda);

    return gatt_status;
}

wiced_result_t lepl_gatt_disconnect(uint16_t conn_id)
{
    lepl_clcb_t *p_clcb = NULL;
    uint8_t call_id;
    wiced_bt_ga_tbs_call_termination_reason_t reason;
    call_id = lepl_ccs_get_active_call_id();
    if (call_id)
        terminate_call(call_id, &reason);

    for (uint8_t i = 0; i < MAX_CONNECTION_INSTANCE; i++)
    {
        if (g_lepl_gatt_cb.unicast_clcb[i].in_use)
            p_clcb = &g_lepl_gatt_cb.unicast_clcb[i];
        if (p_clcb != NULL)
        {
            le_audio_rpc_send_app_status(p_clcb->conn_id, HCI_CONTROL_LEA_APP_STATE_DISCONNECTING, 0);
            if (lepl_mcs_is_streaming())
            {
                WICED_BT_TRACE("[%s] streaming in progress\n", __FUNCTION__);
                p_clcb->app_state = LEPL_GATT_STATE_DISCONNECTING;
                lepl_mcs_pause(p_clcb->conn_id);
            }
            else
            {
                WICED_BT_TRACE("[%s] streaming not in progress\n", __FUNCTION__);
                wiced_bt_gatt_disconnect(p_clcb->conn_id);
            }
        }
    }
    return WICED_SUCCESS;
}

wiced_result_t lepl_gatt_handle_disconnecting_state(void)
{
    wiced_result_t result = WICED_BT_ERROR;
    int index = 0;
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);
    for (index = 0; index < MAX_CONNECTION_INSTANCE; index++)
    {
        lepl_clcb_t *p_clcb = &g_lepl_gatt_cb.unicast_clcb[index];
        if (p_clcb->app_state == LEPL_GATT_STATE_DISCONNECTING)
        {
            WICED_BT_TRACE("[%s] disconnecting %x\n", __FUNCTION__, p_clcb->conn_id);
            result = wiced_bt_gatt_disconnect(p_clcb->conn_id);
        }
    }

    return result;
}

#define UNICAST_SOURCE_EXT_ADV_HANDLE 1

void lepl_gatt_start_stop_adv(uint32_t b_start)
{
    wiced_result_t status;

#define AD_FLAG_SIZE 2
#define AUDIO_STREAM_CONTROL_SERVICE_SIZE 9
#define BASS_SOLICITATION_SIZE 4
#define ADV_NAME_SIZE 16
#define ADV_SIZE                                                                                                       \
    (AD_FLAG_SIZE + 1 + AUDIO_STREAM_CONTROL_SERVICE_SIZE + 1 + ADV_NAME_SIZE + 1 +                                    \
     BASS_SOLICITATION_SIZE) // +1 for length itself

    wiced_ble_ext_adv_duration_config_t duration_cfg;
    uint8_t data[ADV_SIZE] = {0};
    uint8_t *p_ext_adv_data = data;
    wiced_bt_dev_status_t sts;
    uint8_t addr_type = (lepl_cfg_settings.p_ble_cfg->rpa_refresh_timeout) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;

    WICED_BT_TRACE("[%s] %s adv\n", __FUNCTION__, b_start ? "start" : "stop");

    // Set ext adv params
    if (b_start)
    {
        wiced_ble_ext_adv_params_t params = {
            .event_properties = WICED_BLE_EXT_ADV_EVENT_PROPERTY_CONNECTABLE_ADV,
            .primary_adv_int_min = 40,
            .primary_adv_int_max = 40,
            .primary_adv_channel_map = (BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39),
            .own_addr_type = addr_type,
            .peer_addr_type = addr_type,
            .peer_addr = {0, 0, 0, 0, 0, 0},
            .adv_filter_policy = BTM_BLE_ADV_POLICY_ACCEPT_CONN_AND_SCAN,
            .adv_tx_power = 0x7f,
            .primary_adv_phy = WICED_BLE_EXT_ADV_PHY_1M,
            .secondary_adv_max_skip = 0,
            .secondary_adv_phy = WICED_BLE_EXT_ADV_PHY_1M,
            .adv_sid = 1,
            .scan_request_not = WICED_BLE_EXT_ADV_SCAN_REQ_NOTIFY_ENABLE,
            .primary_phy_opts = 0,
            .secondary_phy_opts = 0};

        wiced_ble_ext_adv_set_params(UNICAST_SOURCE_EXT_ADV_HANDLE, &params);

        if (addr_type == BLE_ADDR_RANDOM)
        {
            wiced_ble_ext_adv_set_random_address(UNICAST_SOURCE_EXT_ADV_HANDLE, g_lepl_gatt_cb.own_addr);
        }

        UINT8_TO_STREAM(p_ext_adv_data, AD_FLAG_SIZE);
        UINT8_TO_STREAM(p_ext_adv_data, BTM_BLE_ADVERT_TYPE_FLAG);
        UINT8_TO_STREAM(p_ext_adv_data, BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED);

        UINT8_TO_STREAM(p_ext_adv_data, strlen((const char *)lepl_cfg_settings.device_name) + 1);
        UINT8_TO_STREAM(p_ext_adv_data, BTM_BLE_ADVERT_TYPE_NAME_COMPLETE);
        ARRAY_TO_STREAM(p_ext_adv_data,
                        lepl_cfg_settings.device_name,
                        strlen((const char *)lepl_cfg_settings.device_name));

        // Set adv data in LTV format
        sts = wiced_ble_ext_adv_set_adv_data(UNICAST_SOURCE_EXT_ADV_HANDLE, (p_ext_adv_data - data), data);
        WICED_BT_TRACE("[%s] sts %d [adv size %d]\n", __FUNCTION__, sts, (p_ext_adv_data - data));
    }

    duration_cfg.adv_handle = UNICAST_SOURCE_EXT_ADV_HANDLE;
    duration_cfg.adv_duration = 0;
    duration_cfg.max_ext_adv_events = 0;

    // Start adv
    status = wiced_ble_ext_adv_enable(b_start, 1, &duration_cfg);

    WICED_BT_TRACE("[%s] status 0x%x", __FUNCTION__, status);
}