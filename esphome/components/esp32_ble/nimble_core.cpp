#include "esphome/core/defines.h"

#ifdef USE_ESP32_BLE_NIMBLE

#include "nimble_compat_internal.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

extern "C" {
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <host/ble_hs_id.h>
#include <host/ble_store.h>
#include <host/util/util.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
void ble_store_config_init(void);
}

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace esphome::esp32_ble::nimble_compat {

static const char *const TAG = "esp32_ble.nimble";

esp_gap_ble_cb_t gap_cb = nullptr;
esp_gattc_cb_t gattc_cb = nullptr;
esp_gatts_cb_t gatts_cb = nullptr;

std::atomic<bool> host_synced{false};
SecurityConfig security_config;

// MSB-first public address cache for esp_bt_dev_get_address()
static uint8_t own_address_msb_first[6];  // NOLINT(readability-identifier-naming)
static bool host_task_running = false;    // NOLINT(readability-identifier-naming)
// BLE names are limited to 20 chars in ESPHome; keep headroom + terminator.
static char cached_device_name[33];  // NOLINT(readability-identifier-naming)

const char *get_device_name() { return cached_device_name; }

void dispatch_gap(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (gap_cb != nullptr)
    gap_cb(event, param);
}
void dispatch_gattc(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
  if (gattc_cb != nullptr)
    gattc_cb(event, gattc_if, param);
}
void dispatch_gatts(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  if (gatts_cb != nullptr)
    gatts_cb(event, gatts_if, param);
}

// ---------------------------------------------------------------------------
// Deferred dispatch: completions synthesized on the main loop are posted to
// the NimBLE host task so the BLEEvent queue keeps a single producer.
// ---------------------------------------------------------------------------

struct PendingDispatch {
  ble_npl_event ev;
  uint8_t kind;  // 0 = GAP, 1 = GATTC, 2 = GATTS
  esp_gap_ble_cb_event_t gap_event;
  esp_gattc_cb_event_t gattc_event;
  esp_gatts_cb_event_t gatts_event;
  esp_gatt_if_t iface;
  union {
    esp_ble_gap_cb_param_t gap;
    esp_ble_gattc_cb_param_t gattc;
    esp_ble_gatts_cb_param_t gatts;
  } param;
  std::atomic<bool> in_use{false};
};

// Sized generously: while a GATT server read blocks the host task (RSP_BY_APP
// emulation) the pool cannot drain, and the main loop may post several
// completions in that window.
static constexpr size_t PENDING_DISPATCH_POOL_SIZE = 16;
static PendingDispatch pending_pool[PENDING_DISPATCH_POOL_SIZE];  // NOLINT(readability-identifier-naming)

// Spinlock for facade state shared across tasks; see nimble_compat_internal.h.
static portMUX_TYPE facade_mux = portMUX_INITIALIZER_UNLOCKED;  // NOLINT(readability-identifier-naming)
void facade_lock() { taskENTER_CRITICAL(&facade_mux); }
void facade_unlock() { taskEXIT_CRITICAL(&facade_mux); }

static void pending_dispatch_run(ble_npl_event *ev) {
  auto *slot = static_cast<PendingDispatch *>(ble_npl_event_get_arg(ev));
  switch (slot->kind) {
    case 0:
      dispatch_gap(slot->gap_event, &slot->param.gap);
      break;
    case 1:
      dispatch_gattc(slot->gattc_event, slot->iface, &slot->param.gattc);
      break;
    case 2:
      dispatch_gatts(slot->gatts_event, slot->iface, &slot->param.gatts);
      break;
  }
  slot->in_use.store(false, std::memory_order_release);
}

static PendingDispatch *acquire_pending_slot() {
  // The host task drains posted events quickly; retry briefly before failing.
  for (int attempt = 0; attempt < 50; attempt++) {
    for (auto &slot : pending_pool) {
      bool expected = false;
      if (slot.in_use.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        return &slot;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  ESP_LOGE(TAG, "Pending dispatch pool exhausted");
  return nullptr;
}

static bool post_pending(PendingDispatch *slot) {
  ble_npl_event_init(&slot->ev, pending_dispatch_run, slot);
  ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &slot->ev);
  return true;
}

bool post_gap(esp_gap_ble_cb_event_t event, const esp_ble_gap_cb_param_t &param) {
  PendingDispatch *slot = acquire_pending_slot();
  if (slot == nullptr)
    return false;
  slot->kind = 0;
  slot->gap_event = event;
  slot->param.gap = param;
  return post_pending(slot);
}

bool post_gattc(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, const esp_ble_gattc_cb_param_t &param) {
  PendingDispatch *slot = acquire_pending_slot();
  if (slot == nullptr)
    return false;
  slot->kind = 1;
  slot->gattc_event = event;
  slot->iface = gattc_if;
  slot->param.gattc = param;
  return post_pending(slot);
}

bool post_gatts(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, const esp_ble_gatts_cb_param_t &param) {
  PendingDispatch *slot = acquire_pending_slot();
  if (slot == nullptr)
    return false;
  slot->kind = 2;
  slot->gatts_event = event;
  slot->iface = gatts_if;
  slot->param.gatts = param;
  return post_pending(slot);
}

// ---------------------------------------------------------------------------
// Connection registry
// ---------------------------------------------------------------------------

// +1: a server (peripheral) link can exist alongside client connections.
static ConnEntry connections[USE_ESP32_BLE_MAX_CONNECTIONS + 1];  // NOLINT(readability-identifier-naming)

ConnEntry *conn_by_handle(uint16_t conn_handle) {
  facade_lock();
  for (auto &entry : connections) {
    if (entry.in_use && entry.conn_handle == conn_handle) {
      facade_unlock();
      return &entry;
    }
  }
  facade_unlock();
  return nullptr;
}

ConnEntry *conn_by_addr(const esp_bd_addr_t addr) {
  facade_lock();
  for (auto &entry : connections) {
    if (entry.in_use && memcmp(entry.addr, addr, 6) == 0) {
      facade_unlock();
      return &entry;
    }
  }
  facade_unlock();
  return nullptr;
}

ConnEntry *conn_alloc(uint16_t conn_handle) {
  facade_lock();
  for (auto &entry : connections) {
    if (!entry.in_use) {
      // The gattc facade frees its table before releasing an entry.
      entry = ConnEntry{};
      entry.in_use = true;
      entry.conn_handle = conn_handle;
      facade_unlock();
      return &entry;
    }
  }
  facade_unlock();
  return nullptr;
}

void conn_release(ConnEntry *entry) {
  if (entry == nullptr)
    return;
  facade_lock();
  *entry = ConnEntry{};
  facade_unlock();
}

// ---------------------------------------------------------------------------
// Host lifecycle
// ---------------------------------------------------------------------------

void apply_security_config() {
  ble_hs_cfg.sm_io_cap = security_config.io_cap;
  ble_hs_cfg.sm_bonding = (security_config.auth_req & ESP_LE_AUTH_BOND) != 0;
  ble_hs_cfg.sm_mitm = (security_config.auth_req & ESP_LE_AUTH_REQ_MITM) != 0;
  ble_hs_cfg.sm_sc = (security_config.auth_req & ESP_LE_AUTH_REQ_SC_ONLY) != 0;
  // Distribute/accept both identity and encryption keys so bonding works with
  // RPA-using peers (parity with Bluedroid's default key distribution).
  ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
}

static void on_host_sync() {
  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) {
    ESP_LOGE(TAG, "ensure_addr failed: %d", rc);
  }
  uint8_t addr_lsb_first[6] = {};
  rc = ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr_lsb_first, nullptr);
  if (rc == 0) {
    addr_ble_to_bd(addr_lsb_first, own_address_msb_first);
  }
  host_synced.store(true, std::memory_order_release);
}

static void on_host_reset(int reason) { ESP_LOGE(TAG, "Host reset, reason %d", reason); }

static void host_task_fn(void *param) {
  nimble_port_run();  // returns after nimble_port_stop()
  nimble_port_freertos_deinit();
}

bool host_setup() {
  host_synced.store(false, std::memory_order_release);

  esp_err_t err = nimble_port_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init failed: %d", err);
    return false;
  }

  ble_hs_cfg.sync_cb = on_host_sync;
  ble_hs_cfg.reset_cb = on_host_reset;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  apply_security_config();

#ifdef ESPHOME_NIMBLE_HAS_GAP_SVC
  ble_svc_gap_init();
  if (cached_device_name[0] != '\0') {
    ble_svc_gap_device_name_set(cached_device_name);
  }
#endif
#ifdef USE_ESP32_BLE_SERVER
  ble_svc_gatt_init();
#endif
  ble_store_config_init();

  nimble_port_freertos_init(host_task_fn);
  host_task_running = true;

  // Wait for controller/host sync; normally a few ms.
  uint32_t start = millis();
  while (!host_synced.load(std::memory_order_acquire)) {
    if (millis() - start > 5000) {
      ESP_LOGE(TAG, "Host sync timeout");
      return false;
    }
    delay(1);
  }
  return true;
}

bool host_teardown() {
  host_synced.store(false, std::memory_order_release);
  if (host_task_running) {
    int rc = nimble_port_stop();
    if (rc != 0) {
      ESP_LOGE(TAG, "nimble_port_stop failed: %d", rc);
      return false;
    }
    host_task_running = false;
  }
  esp_err_t err = nimble_port_deinit();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_deinit failed: %d", err);
    return false;
  }
  // Reset facade statics so a later enable starts clean; consumers re-register
  // their apps and re-create services from scratch.
#ifdef USE_ESP32_BLE_CLIENT
  gattc_reset();
#endif
#ifdef USE_ESP32_BLE_SERVER
  gatts_reset();
#endif
  for (auto &entry : connections) {
#ifdef USE_ESP32_BLE_CLIENT
    if (entry.gattc_table != nullptr)
      gattc_free_table(&entry);
#endif
    entry = ConnEntry{};
  }
  // The host task is gone; any events still parked in the pool will never run.
  for (auto &slot : pending_pool)
    slot.in_use.store(false, std::memory_order_release);
  return true;
}

}  // namespace esphome::esp32_ble::nimble_compat

// ---------------------------------------------------------------------------
// Bluedroid-shaped public functions (global namespace)
// ---------------------------------------------------------------------------

namespace nc = esphome::esp32_ble::nimble_compat;

const uint8_t *esp_bt_dev_get_address(void) { return nc::own_address_msb_first; }

esp_err_t esp_ble_gap_register_callback(esp_gap_ble_cb_t callback) {
  nc::gap_cb = callback;
  return ESP_OK;
}

esp_err_t esp_ble_gattc_register_callback(esp_gattc_cb_t callback) {
  nc::gattc_cb = callback;
  return ESP_OK;
}

esp_err_t esp_ble_gatts_register_callback(esp_gatts_cb_t callback) {
  nc::gatts_cb = callback;
  return ESP_OK;
}

esp_err_t esp_ble_gap_set_device_name(const char *name) {
  // Cache for advertising; the GAP service (device name characteristic) only
  // exists when a GATT server is compiled in.
  strlcpy(nc::cached_device_name, name, sizeof(nc::cached_device_name));
#ifdef ESPHOME_NIMBLE_HAS_GAP_SVC
  int rc = ble_svc_gap_device_name_set(name);
  return rc == 0 ? ESP_OK : ESP_FAIL;
#else
  return ESP_OK;
#endif
}

esp_err_t esp_ble_gap_set_security_param(esp_ble_sm_param_t param_type, void *value, uint8_t len) {
  if (value == nullptr || len == 0)
    return ESP_ERR_INVALID_ARG;
  auto *bytes = static_cast<uint8_t *>(value);
  switch (param_type) {
    case ESP_BLE_SM_IOCAP_MODE:
      nc::security_config.io_cap = bytes[0];
      break;
    case ESP_BLE_SM_AUTHEN_REQ_MODE:
      nc::security_config.auth_req = bytes[0];
      break;
    case ESP_BLE_SM_MAX_KEY_SIZE:
      // Stored but not applied: NimBLE has no host-level key size setting
      // (it always negotiates up to 16; minimum is a compile-time option).
      nc::security_config.max_key_size = bytes[0];
      break;
    case ESP_BLE_SM_MIN_KEY_SIZE:
      nc::security_config.min_key_size = bytes[0];
      break;
    default:
      return ESP_ERR_NOT_SUPPORTED;
  }
  nc::apply_security_config();
  return ESP_OK;
}

#endif  // USE_ESP32_BLE_NIMBLE
