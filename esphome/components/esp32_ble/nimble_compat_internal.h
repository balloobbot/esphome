#pragma once

// Internal state shared between the NimBLE facade translation units
// (nimble_core.cpp, nimble_gap.cpp, nimble_gattc.cpp, nimble_gatts.cpp).
// Not part of the public API surface; consumers include ble_stack_api.h.

#include "esphome/core/defines.h"

#ifdef USE_ESP32_BLE_NIMBLE

#include "nimble_compat.h"

#include <atomic>
#include <cstring>

#include <sdkconfig.h>

extern "C" {
#include <nimble/ble.h>
}

// NimBLE compiles connection and security-manager code out when neither the
// central nor the peripheral role is enabled (e.g. a beacon-only build);
// facade code referencing those symbols must be guarded the same way.
#if defined(CONFIG_BT_NIMBLE_ROLE_CENTRAL) || defined(CONFIG_BT_NIMBLE_ROLE_PERIPHERAL)
#define ESPHOME_NIMBLE_HAS_CONNECT
#endif

// The NimBLE GAP service (device name) only exists with a GATT server.
#if defined(CONFIG_BT_NIMBLE_GATT_SERVER) && defined(CONFIG_BT_NIMBLE_GAP_SERVICE)
#define ESPHOME_NIMBLE_HAS_GAP_SVC
#endif

// NimBLE GAP event type; forward-declared at global scope so the hook
// signatures below reference the real type, not a namespaced one.
struct ble_gap_event;

namespace esphome::esp32_ble::nimble_compat {

// Registered Bluedroid-style callbacks (set via esp_ble_*_register_callback).
extern esp_gap_ble_cb_t gap_cb;
extern esp_gattc_cb_t gattc_cb;
extern esp_gatts_cb_t gatts_cb;

// True once the NimBLE host has synced with the controller.
extern std::atomic<bool> host_synced;

// Host lifecycle, called from ESP32BLE::ble_setup_()/ble_dismantle_().
bool host_setup();
bool host_teardown();

// Short spinlock protecting facade state shared between the main loop and the
// NimBLE host task (connection registry, GATT client tables). Hold only for
// brief lookups/frees; never across blocking calls.
void facade_lock();
void facade_unlock();

struct FacadeLockGuard {
  FacadeLockGuard() { facade_lock(); }
  ~FacadeLockGuard() { facade_unlock(); }
  FacadeLockGuard(const FacadeLockGuard &) = delete;
  FacadeLockGuard &operator=(const FacadeLockGuard &) = delete;
};

#ifdef USE_ESP32_BLE_CLIENT
// Reset GATT client facade statics (app slots, pending open, tables); called
// from host_teardown() so a BLE disable/enable cycle starts clean.
void gattc_reset();
#endif
#ifdef USE_ESP32_BLE_SERVER
// Same for the GATT server facade (app registration, service registry).
void gatts_reset();
#endif

// Device name cached by esp_ble_gap_set_device_name(); used for advertising
// even when the NimBLE GAP service is not compiled in.
const char *get_device_name();

// Security-manager settings staged by esp_ble_gap_set_security_param and
// applied to ble_hs_cfg; kept so re-enabling BLE restores them.
struct SecurityConfig {
  uint8_t io_cap{3};  // ESP_IO_CAP_NONE
  uint8_t auth_req{0};
  uint8_t max_key_size{16};
  uint8_t min_key_size{7};
};
extern SecurityConfig security_config;
void apply_security_config();

// Address conversion: Bluedroid esp_bd_addr_t is MSB-first, NimBLE ble_addr_t
// is LSB-first.
inline void addr_ble_to_bd(const uint8_t *ble_val, esp_bd_addr_t out) {
  for (int i = 0; i < 6; i++)
    out[i] = ble_val[5 - i];
}
inline void addr_bd_to_ble(const esp_bd_addr_t bda, uint8_t *ble_val) {
  for (int i = 0; i < 6; i++)
    ble_val[i] = bda[5 - i];
}

// Dispatch helpers: invoke the registered Bluedroid-style callback directly.
// Only safe from the NimBLE host task (the BLEEvent queue producer side).
void dispatch_gap(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void dispatch_gattc(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
void dispatch_gatts(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// Deferred dispatch for facade calls made from the main loop. Bluedroid
// delivers *_COMPLETE/REG events from its own task; the BLEEvent queue is
// single-producer (host task), so completions synthesized on the main loop are
// posted to the NimBLE host task and dispatched from there.
bool post_gap(esp_gap_ble_cb_event_t event, const esp_ble_gap_cb_param_t &param);
bool post_gattc(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, const esp_ble_gattc_cb_param_t &param);
bool post_gatts(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, const esp_ble_gatts_cb_param_t &param);

// Convenience: post a GAP event whose param is a lone esp_bt_status_t.
inline bool post_gap_status(esp_gap_ble_cb_event_t event, esp_bt_status_t status) {
  esp_ble_gap_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.scan_param_cmpl.status = status;  // all status-only arms alias offset 0
  return post_gap(event, param);
}

// Per-connection registry shared by the GAP and GATT client facades.
// conn_handle is the NimBLE handle; it doubles as the Bluedroid conn_id.
struct ConnEntry {
  uint16_t conn_handle{0xFFFF};
  esp_gatt_if_t gattc_if{ESP_GATT_IF_NONE};
  esp_bd_addr_t addr{};  // MSB-first
  esp_ble_addr_type_t addr_type{BLE_ADDR_TYPE_PUBLIC};
  bool in_use{false};
  // Attribute table owned by nimble_gattc.cpp (GattTable*); freed on release.
  void *gattc_table{nullptr};
};

ConnEntry *conn_by_handle(uint16_t conn_handle);
ConnEntry *conn_by_addr(const esp_bd_addr_t addr);
ConnEntry *conn_alloc(uint16_t conn_handle);
void conn_release(ConnEntry *entry);

#ifdef USE_ESP32_BLE_CLIENT
// Free a connection's attribute table (safe against concurrent cache walks).
void gattc_free_table(ConnEntry *entry);
#endif

// Map a NimBLE host return code to a Bluedroid GATT status (ATT error codes
// pass through; anything else collapses to ESP_GATT_ERROR). In nimble_gattc.cpp.
esp_gatt_status_t hs_to_gatt_status(int rc);

// Security events shared by the client and peripheral GAP callbacks
// (ENC_CHANGE, PASSKEY_ACTION, REPEAT_PAIRING). Returns the value the GAP
// callback must return, or -1 when the event was not handled.
int handle_common_gap_event(struct ble_gap_event *event);
// Synthesize ESP_GAP_BLE_AUTH_CMPL_EVT from a NimBLE encryption change.
void emit_auth_cmpl(uint16_t conn_handle, int hs_status);

#ifdef USE_ESP32_BLE_SERVER
// GATT server facade hooks, called from the advertising GAP callback in
// nimble_gap.cpp for peripheral-link events; implemented in nimble_gatts.cpp.
void gatts_on_connect(const struct ble_gap_event *event);
void gatts_on_disconnect(const struct ble_gap_event *event);
void gatts_on_subscribe(const struct ble_gap_event *event);
#endif

}  // namespace esphome::esp32_ble::nimble_compat

#endif  // USE_ESP32_BLE_NIMBLE
