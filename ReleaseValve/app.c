/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

// Capstone I16
// Written by Finn Rush
// PROTOTYPE

#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "app.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "gatt_db.h"
#include <string.h>

// GPIO pin to drive based on temperature threshold
#define TEMP_GPIO_PORT      gpioPortA
#define TEMP_GPIO_PIN       3

// Temperature threshold in °C (×10 for comparison with temp_x10)
#define TEMP_THRESHOLD_X10  250   // 25.0°C

// Length of the T###P# identifier prefix (shared between advertiser and scanner)
#define IDENTIFIER_LEN      6

// Local copy of this device's own name read from GATT
static uint8_t own_name[32];
static uint16_t own_name_len = 0;

SL_WEAK void app_init(void)
{
    CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinModeSet(TEMP_GPIO_PORT, TEMP_GPIO_PIN, gpioModePushPull, 0);
}

SL_WEAK void app_process_action(void)
{
}

void sl_bt_on_event(sl_bt_msg_t *evt)
{
    sl_status_t sc;

    switch (SL_BT_MSG_ID(evt->header)) {

        case sl_bt_evt_system_boot_id:
            // Read this device's own name from its local GATT database
            // e.g. "T001P1 VALVE"
            sl_bt_gatt_server_read_attribute_value(gattdb_device_name,
                                                   0,
                                                   sizeof(own_name),
                                                   &own_name_len,
                                                   own_name);

            // Set scan parameters: passive scan, 10ms interval, 10ms window
            sc = sl_bt_scanner_set_parameters(
                sl_bt_scanner_scan_mode_passive,
                16,   // interval: 16 × 0.625ms = 10ms
                16);  // window:   16 × 0.625ms = 10ms
            app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to set scan parameters\n", (int)sc);

            // Start scanning on 1M PHY, discover all devices
            sc = sl_bt_scanner_start(
                sl_bt_scanner_scan_phy_1m,
                sl_bt_scanner_discover_generic);
            app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to start scanning\n", (int)sc);
            break;

        case sl_bt_evt_scanner_scan_report_id: {
            uint8_t *data = evt->data.evt_scanner_scan_report.data.data;
            uint8_t  len  = evt->data.evt_scanner_scan_report.data.len;

            // Only proceed if own name was read successfully and is long enough
            if (own_name_len < IDENTIFIER_LEN) break;

            // Walk through AD structures
            uint8_t i = 0;
            while (i < len) {
                uint8_t ad_len  = data[i];
                uint8_t ad_type = data[i + 1];

                // Look for Manufacturer Specific Data (0xFF)
                // Advertiser payload: [len][0xFF][T###P# (6 bytes)][temp_lo][temp_hi]
                // Minimum ad_len = 1 (type) + 6 (name) + 2 (temp) = 9
                if (ad_type == 0xFF && ad_len >= 9) {
                    uint8_t *payload = &data[i + 2]; // points to name bytes

                    // Compare first 6 chars of scanner's own name (T###P#)
                    // against the 6-byte identifier in the received packet
                    if (memcmp(payload, own_name, IDENTIFIER_LEN) == 0) {

                        // Extract temperature (bytes after the 6-char identifier)
                        int16_t temp_x10 = (int16_t)(payload[6] | (payload[7] << 8));

                        // Drive GPIO based on threshold
                        if (temp_x10 >= TEMP_THRESHOLD_X10) {
                            GPIO_PinOutSet(TEMP_GPIO_PORT, TEMP_GPIO_PIN);
                        } else {
                            GPIO_PinOutClear(TEMP_GPIO_PORT, TEMP_GPIO_PIN);
                        }
                    }
                }
                i += ad_len + 1;
            }
            break;
        }

        default:
            break;
    }
}
