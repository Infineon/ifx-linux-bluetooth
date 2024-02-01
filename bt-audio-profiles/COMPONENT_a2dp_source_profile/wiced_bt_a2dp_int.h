/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company)
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @file
 *
 * This is the private file for the a2dp common functionality.
 */

#pragma once

#include "wiced_bt_a2d.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_a2d_sbc.h"

/* Codec related functions */
extern uint8_t wiced_bt_a2dp_sbc_cfg_for_cap(uint8_t *p_peer,
    wiced_bt_a2d_sbc_cie_t *p_cap, wiced_bt_a2d_sbc_cie_t *p_pref);
extern uint8_t wiced_bt_a2dp_sbc_cfg_in_cap(uint8_t *p_cfg,
    wiced_bt_a2d_sbc_cie_t *p_cap);
