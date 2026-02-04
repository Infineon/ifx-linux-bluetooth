/*
 * $ Copyright Cypress Semiconductor $
 */

#ifndef __LEHS_NVRAM_H__
#define __LEHS_NVRAM_H__

#include "lehs.h"
#include "wiced_hal_nvram.h"

#define MAX_NUM_DEVICES_IN_NVRAM 2
#define MAX_SERVICES_PER_DEVICE_NVRAM 16

#define UNICAST_APP_MAX_IDS_PER_DEVICE                                                                                  \
    (1 +                                        /* Link keys */                                                        \
     1 +                                        /* Client DB Meta data */                                              \
     1 +                                        /* Server DB Meta data */                                              \
     MAX_SERVICES_PER_DEVICE_NVRAM +            /* Client DB Data */                                                   \
     MAX_SERVICES_PER_DEVICE_NVRAM              /* Server DB Data */                                                   \
    )
#define UNICAST_APP_MAX_IDS_PER_DEVICE_ALIGNED ALIGN_SIZE(UNICAST_APP_MAX_IDS_PER_DEVICE, (1 << 8))

enum
{
    UNICAST_APP_NVRAM_ID_START = (WICED_NVRAM_VSID_START),
    UNICAST_APP_NVRAM_ID_LOCAL_IRK,      // 0x201
    UNICAST_APP_NVRAM_ID_LAST_PAIRED_KEY, // 0x202

    UNICAST_APP_NVRAM_ID_PAIRED_KEYS = ALIGN_SIZE(UNICAST_APP_NVRAM_ID_LAST_PAIRED_KEY + 1, 0x100), // 0x203
    UNICAST_APP_NVRAM_ID_PAIRED_KEYS_MAX =
        (UNICAST_APP_NVRAM_ID_PAIRED_KEYS + MAX_NUM_DEVICES_IN_NVRAM * UNICAST_APP_MAX_IDS_PER_DEVICE_ALIGNED),

    UNICAST_APP_NVRAM_ID_END
};

#define SCRIPT_APP_MAX_NVRAM_ENTRIES (UNICAST_APP_NVRAM_ID_END - UNICAST_APP_NVRAM_ID_START)

typedef struct
{
    wiced_bt_device_link_keys_t link_keys;
} lehs_nvram_data_t;

int lehs_nvram_read(int nvram_id, wiced_bt_device_address_t bdaddr, uint8_t *p_data, uint32_t len);
int lehs_nvram_write(int nvram_id, wiced_bt_device_address_t bdaddr, uint8_t *p_data, uint32_t len);
void lehs_nvram_delete(int nvram_id, wiced_bt_device_address_t bdaddr);

int lehs_nvram_read_keys(wiced_bt_device_link_keys_t *p_linkkeys);
int lehs_nvram_write_keys(wiced_bt_device_link_keys_t *p_linkkeys);

#endif
