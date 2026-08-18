#pragma once

// Bluetooth host stack API selector for ESP32 builds.
//
// With the default Bluedroid stack this pulls the ESP-IDF Bluedroid API
// headers. With the NimBLE stack (esp32_ble: stack: nimble) those headers are
// not on the include path, so a source-compatible mirror of the subset used
// by ESPHome is provided instead; the functions are implemented on top of
// NimBLE in esp32_ble/nimble_*.cpp.
//
// Components must include this header instead of the Bluedroid headers
// (<esp_gap_ble_api.h>, <esp_gattc_api.h>, <esp_gatts_api.h>, ...) directly.

#include "esphome/core/defines.h"

#ifdef USE_ESP32_BLE_NIMBLE
#include "nimble_compat.h"
#else
#include <esp_bt_defs.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>
#include <esp_gatt_defs.h>
#include <esp_gattc_api.h>
#include <esp_gatts_api.h>
#endif
