/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef CSIS_PSRI_KEY_H
#define CSIS_PSRI_KEY_H

#include "wiced_bt_dev.h"
#include "wiced_bt_ga_csis.h"

#define CSIS_KEY_LEN      16
#define CSIS_R_LEN        3
#define CSIS_HASH_LEN     3
#define CSIS_PRAND_LEN    3
#define CSIS_PADDING_LEN  13
#define CSIS_HASH_VAL     16777216 /*2^24 */
#define CSIS_PRAND_MAX_RANDOM    4194304 /*2^22*/

void ga_csis_calc_hash(uint8_t* k, uint8_t* r, uint8_t* h);
void ga_csis_generate_prand(uint8_t* p_prand);
PSRI* ga_csis_generate_psri(SIRK* sirk);

void ga_csis_sirk_encyption_func(SIRK* sirk_plain_text, wiced_bt_device_link_keys_t* link_keys, SIRK* sirk_encr);
void ga_csis_sirk_decryption_func(SIRK* sirk_enc, wiced_bt_device_link_keys_t* link_keys, SIRK* sirk_plain_text);

#endif // CSIS_PSRI_KEY_H
