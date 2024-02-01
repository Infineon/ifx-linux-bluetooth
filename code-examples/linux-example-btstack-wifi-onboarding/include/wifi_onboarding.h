/******************************************************************************
 * File Name: wifi_onboarding.h
 *
 * Description: This is the header file for Linux wifi_onboarding CE.
 *
 * Related Document: See README.md
 ******************************************************************************
* $ Copyright 2022-YEAR Cypress Semiconductor $
 *****************************************************************************/

#ifndef __APP_WIFI_ONBOARDING_H__
#define __APP_WIFI_ONBOARDING_H__

#include "wiced_hal_nvram.h"

#define IWOS_VS_ID                  WICED_NVRAM_VSID_START
#define IWOS_LOCAL_KEYS_VS_ID       WICED_NVRAM_VSID_START + 1
#define IWOS_PAIRED_KEYS_VS_ID      WICED_NVRAM_VSID_START + 2

/******************************************************************************
*       FUNCTION PROTOTYPES
******************************************************************************/

/*
 *  Set device configuration and start BT stack initialization. The actual
 *  application initialization will happen when stack reports that BT device
 *  is ready.
 */

void application_start( void );
void *wifi_tracker(void *);
#endif /* __APP_WIFI_ONBOARDING_H__ */
