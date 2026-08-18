#include "esphome/core/defines.h"

#if defined(USE_ESP32_BLE_NIMBLE) && defined(USE_ESP32_BLE_CLIENT)

#include "nimble_compat_internal.h"

#include "esphome/core/log.h"

#include <memory>
#include <vector>

extern "C" {
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <os/os_mbuf.h>
}

#ifndef USE_ESP32_BLE_CONNECTION_TIMEOUT_MS
#define USE_ESP32_BLE_CONNECTION_TIMEOUT_MS 20000
#endif

namespace esphome::esp32_ble::nimble_compat {

static const char *const TAG = "esp32_ble.nimble";

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

esp_gatt_status_t hs_to_gatt_status(int rc) {
  if (rc == 0)
    return ESP_GATT_OK;
  if (rc >= BLE_HS_ERR_ATT_BASE && rc < BLE_HS_ERR_ATT_BASE + 0x100)
    return static_cast<esp_gatt_status_t>(rc - BLE_HS_ERR_ATT_BASE);
  return ESP_GATT_ERROR;
}

static esp_gatt_conn_reason_t hs_to_conn_reason(int reason) {
  if (reason >= BLE_HS_ERR_HCI_BASE && reason < BLE_HS_ERR_HCI_BASE + 0x100)
    return static_cast<esp_gatt_conn_reason_t>(reason - BLE_HS_ERR_HCI_BASE);
  if (reason == BLE_HS_ETIMEOUT || reason == BLE_HS_ETIMEOUT_HCI)
    return ESP_GATT_CONN_TIMEOUT;
  return ESP_GATT_CONN_UNKNOWN;
}

static void uuid_any_to_bt(const ble_uuid_any_t &in, esp_bt_uuid_t &out) {
  switch (in.u.type) {
    case BLE_UUID_TYPE_16:
      out.len = ESP_UUID_LEN_16;
      out.uuid.uuid16 = in.u16.value;
      break;
    case BLE_UUID_TYPE_32:
      out.len = ESP_UUID_LEN_32;
      out.uuid.uuid32 = in.u32.value;
      break;
    default:
      out.len = ESP_UUID_LEN_128;
      memcpy(out.uuid.uuid128, in.u128.value, 16);
      break;
  }
}

// ---------------------------------------------------------------------------
// Per-connection attribute table (fills Bluedroid's synchronous cache role)
// ---------------------------------------------------------------------------

struct GattcSvcEntry {
  uint16_t start_handle;
  uint16_t end_handle;
  ble_uuid_any_t uuid;
};

struct GattcChrEntry {
  uint16_t def_handle;
  uint16_t val_handle;
  uint8_t properties;
  ble_uuid_any_t uuid;
};

struct GattcDscEntry {
  uint16_t handle;
  uint16_t chr_val_handle;  // owning characteristic's value handle
  ble_uuid_any_t uuid;
};

struct GattTable {
  std::vector<GattcSvcEntry> svcs;
  std::vector<GattcChrEntry> chrs;
  std::vector<GattcDscEntry> dscs;
  // Descriptor-discovery cursor (index into chrs)
  size_t disc_chr_index{0};
  bool complete{false};
};

static GattTable *get_table(ConnEntry *entry, bool create) {
  if (entry->gattc_table == nullptr && create)
    entry->gattc_table = new GattTable();  // NOLINT(cppcoreguidelines-owning-memory)
  return static_cast<GattTable *>(entry->gattc_table);
}

// Frees under the facade lock: the host task frees on disconnect while the
// main loop may be inside a cache-walk of the same table.
void gattc_free_table(ConnEntry *entry) {
  facade_lock();
  auto *table = static_cast<GattTable *>(entry->gattc_table);
  entry->gattc_table = nullptr;
  facade_unlock();
  delete table;  // NOLINT(cppcoreguidelines-owning-memory)
}

// ---------------------------------------------------------------------------
// App registry: gattc_if = slot index + 1
// ---------------------------------------------------------------------------

struct GattcApp {
  bool in_use{false};
  uint16_t app_id{0};
};

static GattcApp gattc_apps[USE_ESP32_BLE_MAX_CONNECTIONS + 1];  // NOLINT(readability-identifier-naming)

// Pending direct connection (one at a time, like Bluedroid's connect mailbox)
struct PendingOpen {
  bool active{false};
  esp_gatt_if_t iface{ESP_GATT_IF_NONE};
  esp_bd_addr_t addr{};
  esp_ble_addr_type_t addr_type{BLE_ADDR_TYPE_PUBLIC};
};
static PendingOpen pending_open;  // NOLINT(readability-identifier-naming)

// Preferred connection parameters staged via esp_ble_gap_set_prefer_conn_params
struct PreferredParams {
  bool valid{false};
  esp_bd_addr_t addr{};
  uint16_t min_int{0};
  uint16_t max_int{0};
  uint16_t latency{0};
  uint16_t timeout{0};
};
static PreferredParams preferred_params;  // NOLINT(readability-identifier-naming)

// Accumulates ble_gattc_read_long chunks so consumers receive one Bluedroid
// style READ event with the full value (Bluedroid reads long attributes
// transparently). One slot per connection; only touched on the host task.
struct ReadAccum {
  uint16_t conn_handle{0xFFFF};
  uint16_t handle{0};
  uint16_t len{0};
  bool active{false};
  std::unique_ptr<uint8_t[]> buf;  // ESP_GATT_MAX_ATTR_LEN, lazily allocated
};
static ReadAccum read_accums[USE_ESP32_BLE_MAX_CONNECTIONS + 1];  // NOLINT(readability-identifier-naming)

static ReadAccum *read_accum_for(uint16_t conn_handle) {
  ReadAccum *free_slot = nullptr;
  for (auto &accum : read_accums) {
    if (accum.active && accum.conn_handle == conn_handle)
      return &accum;
    if (!accum.active && free_slot == nullptr)
      free_slot = &accum;
  }
  if (free_slot != nullptr) {
    free_slot->conn_handle = conn_handle;
    free_slot->len = 0;
    if (free_slot->buf == nullptr)
      free_slot->buf = std::make_unique<uint8_t[]>(ESP_GATT_MAX_ATTR_LEN);
  }
  return free_slot;
}

static void read_accum_deactivate(uint16_t conn_handle) {
  for (auto &accum : read_accums) {
    if (accum.conn_handle == conn_handle)
      accum.active = false;
  }
}

// ---------------------------------------------------------------------------
// Service discovery driver (host task context)
// ---------------------------------------------------------------------------

static void finish_discovery(ConnEntry *entry, esp_gatt_status_t status) {
  GattTable *table = get_table(entry, true);
  esp_ble_gattc_cb_param_t param;
  if (status == ESP_GATT_OK) {
    table->complete = true;
    for (auto &svc : table->svcs) {
      memset(&param, 0, sizeof(param));
      param.search_res.conn_id = entry->conn_handle;
      param.search_res.start_handle = svc.start_handle;
      param.search_res.end_handle = svc.end_handle;
      param.search_res.is_primary = true;
      uuid_any_to_bt(svc.uuid, param.search_res.srvc_id.uuid);
      dispatch_gattc(ESP_GATTC_SEARCH_RES_EVT, entry->gattc_if, &param);
    }
  }
  memset(&param, 0, sizeof(param));
  param.search_cmpl.status = status;
  param.search_cmpl.conn_id = entry->conn_handle;
  dispatch_gattc(ESP_GATTC_SEARCH_CMPL_EVT, entry->gattc_if, &param);
}

static void discover_next_dscs(ConnEntry *entry);

static int dsc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t chr_val_handle,
                       const struct ble_gatt_dsc *dsc, void *arg) {
  auto *entry = static_cast<ConnEntry *>(arg);
  if (!entry->in_use || entry->conn_handle != conn_handle)
    return 0;
  GattTable *table = get_table(entry, true);
  if (error->status == 0 && dsc != nullptr) {
    table->dscs.push_back(GattcDscEntry{dsc->handle, chr_val_handle, dsc->uuid});
    return 0;
  }
  if (error->status == BLE_HS_EDONE || error->status == BLE_HS_ATT_ERR(0x0A)) {
    // Done with this characteristic's range (ATT "attribute not found" means
    // the range was empty).
    table->disc_chr_index++;
    discover_next_dscs(entry);
    return 0;
  }
  ESP_LOGW(TAG, "Descriptor discovery failed: %d", error->status);
  finish_discovery(entry, ESP_GATT_ERROR);
  return 0;
}

// Determine the descriptor range of chrs[i] and start its discovery, skipping
// characteristics with empty ranges; finishes discovery after the last one.
static void discover_next_dscs(ConnEntry *entry) {
  GattTable *table = get_table(entry, true);
  while (table->disc_chr_index < table->chrs.size()) {
    const auto &chr = table->chrs[table->disc_chr_index];
    // Range end: attribute before the next characteristic declaration, or the
    // end of the service containing this characteristic.
    uint16_t end_handle = 0xFFFF;
    if (table->disc_chr_index + 1 < table->chrs.size()) {
      end_handle = table->chrs[table->disc_chr_index + 1].def_handle - 1;
    }
    for (const auto &svc : table->svcs) {
      if (chr.val_handle >= svc.start_handle && chr.val_handle <= svc.end_handle) {
        if (svc.end_handle < end_handle)
          end_handle = svc.end_handle;
        break;
      }
    }
    if (end_handle <= chr.val_handle) {
      table->disc_chr_index++;
      continue;
    }
    int rc = ble_gattc_disc_all_dscs(entry->conn_handle, chr.val_handle, end_handle, dsc_disc_cb, entry);
    if (rc != 0) {
      ESP_LOGW(TAG, "disc_all_dscs failed: %d", rc);
      finish_discovery(entry, ESP_GATT_ERROR);
    }
    return;
  }
  finish_discovery(entry, ESP_GATT_OK);
}

static int chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr,
                       void *arg) {
  auto *entry = static_cast<ConnEntry *>(arg);
  if (!entry->in_use || entry->conn_handle != conn_handle)
    return 0;
  GattTable *table = get_table(entry, true);
  if (error->status == 0 && chr != nullptr) {
    table->chrs.push_back(GattcChrEntry{chr->def_handle, chr->val_handle, chr->properties, chr->uuid});
    return 0;
  }
  if (error->status == BLE_HS_EDONE) {
    table->disc_chr_index = 0;
    discover_next_dscs(entry);
    return 0;
  }
  ESP_LOGW(TAG, "Characteristic discovery failed: %d", error->status);
  finish_discovery(entry, ESP_GATT_ERROR);
  return 0;
}

static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *service,
                       void *arg) {
  auto *entry = static_cast<ConnEntry *>(arg);
  if (!entry->in_use || entry->conn_handle != conn_handle)
    return 0;
  GattTable *table = get_table(entry, true);
  if (error->status == 0 && service != nullptr) {
    table->svcs.push_back(GattcSvcEntry{service->start_handle, service->end_handle, service->uuid});
    return 0;
  }
  if (error->status == BLE_HS_EDONE) {
    if (table->svcs.empty()) {
      finish_discovery(entry, ESP_GATT_OK);
      return 0;
    }
    int rc = ble_gattc_disc_all_chrs(conn_handle, 0x0001, 0xFFFF, chr_disc_cb, entry);
    if (rc != 0) {
      ESP_LOGW(TAG, "disc_all_chrs failed: %d", rc);
      finish_discovery(entry, ESP_GATT_ERROR);
    }
    return 0;
  }
  ESP_LOGW(TAG, "Service discovery failed: %d", error->status);
  finish_discovery(entry, ESP_GATT_ERROR);
  return 0;
}

// Called from host_teardown(); the host task is stopped, so plain resets are
// safe. Attribute tables still attached to connection entries are freed by
// host_teardown() itself via gattc_free_table().
void gattc_reset() {
  for (auto &app : gattc_apps)
    app = GattcApp{};
  pending_open = PendingOpen{};
  preferred_params = PreferredParams{};
  for (auto &accum : read_accums)
    accum.active = false;
}

// ---------------------------------------------------------------------------
// Connection GAP callback (client links)
// ---------------------------------------------------------------------------

static int client_gap_event_cb(struct ble_gap_event *event, void *arg) {
  esp_ble_gattc_cb_param_t param;
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT: {
      if (!pending_open.active)
        return 0;
      esp_gatt_if_t iface = pending_open.iface;
      if (event->connect.status != 0) {
        pending_open.active = false;
        ESP_LOGW(TAG, "Connect failed: %d", event->connect.status);
        memset(&param, 0, sizeof(param));
        param.open.status = ESP_GATT_ERROR;
        param.open.conn_id = 0xFFFF;
        memcpy(param.open.remote_bda, pending_open.addr, 6);
        dispatch_gattc(ESP_GATTC_OPEN_EVT, iface, &param);
        return 0;
      }
      ConnEntry *entry = conn_alloc(event->connect.conn_handle);
      if (entry == nullptr) {
        pending_open.active = false;
        ble_gap_terminate(event->connect.conn_handle, 0x13);
        return 0;
      }
      entry->gattc_if = iface;
      memcpy(entry->addr, pending_open.addr, 6);
      entry->addr_type = pending_open.addr_type;
      pending_open.active = false;

      memset(&param, 0, sizeof(param));
      param.connect.conn_id = entry->conn_handle;
      memcpy(param.connect.remote_bda, entry->addr, 6);
      param.connect.ble_addr_type = entry->addr_type;
      param.connect.conn_handle = entry->conn_handle;
      dispatch_gattc(ESP_GATTC_CONNECT_EVT, iface, &param);

      memset(&param, 0, sizeof(param));
      param.open.status = ESP_GATT_OK;
      param.open.conn_id = entry->conn_handle;
      memcpy(param.open.remote_bda, entry->addr, 6);
      dispatch_gattc(ESP_GATTC_OPEN_EVT, iface, &param);
      return 0;
    }
    case BLE_GAP_EVENT_DISCONNECT: {
      ConnEntry *entry = conn_by_handle(event->disconnect.conn.conn_handle);
      if (entry == nullptr)
        return 0;
      esp_gatt_if_t iface = entry->gattc_if;
      esp_gatt_conn_reason_t reason = hs_to_conn_reason(event->disconnect.reason);

      memset(&param, 0, sizeof(param));
      param.disconnect.reason = reason;
      param.disconnect.conn_id = entry->conn_handle;
      memcpy(param.disconnect.remote_bda, entry->addr, 6);
      dispatch_gattc(ESP_GATTC_DISCONNECT_EVT, iface, &param);

      memset(&param, 0, sizeof(param));
      param.close.status = ESP_GATT_OK;
      param.close.conn_id = entry->conn_handle;
      memcpy(param.close.remote_bda, entry->addr, 6);
      param.close.reason = reason;
      dispatch_gattc(ESP_GATTC_CLOSE_EVT, iface, &param);

      gattc_free_table(entry);
      read_accum_deactivate(entry->conn_handle);
      conn_release(entry);
      return 0;
    }
    case BLE_GAP_EVENT_MTU: {
      ConnEntry *entry = conn_by_handle(event->mtu.conn_handle);
      if (entry == nullptr)
        return 0;
      memset(&param, 0, sizeof(param));
      param.cfg_mtu.status = ESP_GATT_OK;
      param.cfg_mtu.conn_id = entry->conn_handle;
      param.cfg_mtu.mtu = event->mtu.value;
      dispatch_gattc(ESP_GATTC_CFG_MTU_EVT, entry->gattc_if, &param);
      return 0;
    }
    case BLE_GAP_EVENT_NOTIFY_RX: {
      ConnEntry *entry = conn_by_handle(event->notify_rx.conn_handle);
      if (entry == nullptr)
        return 0;
      uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
      uint8_t buf[ESP_GATT_MAX_ATTR_LEN];
      if (len > sizeof(buf))
        len = sizeof(buf);
      os_mbuf_copydata(event->notify_rx.om, 0, len, buf);
      memset(&param, 0, sizeof(param));
      param.notify.conn_id = entry->conn_handle;
      memcpy(param.notify.remote_bda, entry->addr, 6);
      param.notify.handle = event->notify_rx.attr_handle;
      param.notify.value = buf;
      param.notify.value_len = len;
      param.notify.is_notify = !event->notify_rx.indication;
      dispatch_gattc(ESP_GATTC_NOTIFY_EVT, entry->gattc_if, &param);
      return 0;
    }
    default: {
      int rc = handle_common_gap_event(event);
      return rc < 0 ? 0 : rc;
    }
  }
}

// ---------------------------------------------------------------------------
// Read/write completion callbacks
// ---------------------------------------------------------------------------

// arg encodes the Bluedroid event to synthesize on completion.
static constexpr uintptr_t OP_READ_CHAR = 0;
static constexpr uintptr_t OP_READ_DESCR = 1;
static constexpr uintptr_t OP_WRITE_CHAR = 2;
static constexpr uintptr_t OP_WRITE_DESCR = 3;

// ble_gattc_read_long delivers the value in chunks (one call per ATT read /
// read-blob response) followed by a final call with BLE_HS_EDONE. Accumulate
// and deliver one Bluedroid-style READ event with the complete value.
static int read_attr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr,
                        void *arg) {
  ReadAccum *accum = read_accum_for(conn_handle);
  if (error->status == 0 && attr != nullptr) {
    if (accum == nullptr)
      return 0;  // no slot; final event reports what accumulated (nothing)
    if (!accum->active) {
      accum->active = true;
      accum->handle = attr->handle;
      accum->len = 0;
    }
    uint16_t chunk = OS_MBUF_PKTLEN(attr->om);
    if (accum->len + chunk > ESP_GATT_MAX_ATTR_LEN)
      chunk = ESP_GATT_MAX_ATTR_LEN - accum->len;
    os_mbuf_copydata(attr->om, 0, chunk, accum->buf.get() + accum->len);
    accum->len += chunk;
    return 0;
  }

  // Completion: BLE_HS_EDONE on success, anything else is an error.
  ConnEntry *entry = conn_by_handle(conn_handle);
  bool ok = error->status == BLE_HS_EDONE && accum != nullptr && accum->active;
  if (entry != nullptr) {
    esp_ble_gattc_cb_param_t param;
    memset(&param, 0, sizeof(param));
    param.read.conn_id = conn_handle;
    if (ok) {
      param.read.status = ESP_GATT_OK;
      param.read.handle = accum->handle;
      param.read.value = accum->buf.get();
      param.read.value_len = accum->len;
    } else {
      param.read.status = hs_to_gatt_status(error->status);  // EDONE w/o data maps to ESP_GATT_ERROR
      param.read.handle = error->att_handle;
    }
    auto op = reinterpret_cast<uintptr_t>(arg);
    dispatch_gattc(op == OP_READ_DESCR ? ESP_GATTC_READ_DESCR_EVT : ESP_GATTC_READ_CHAR_EVT, entry->gattc_if, &param);
  }
  if (accum != nullptr)
    accum->active = false;
  return 0;
}

static int write_attr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr,
                         void *arg) {
  ConnEntry *entry = conn_by_handle(conn_handle);
  if (entry == nullptr)
    return 0;
  esp_ble_gattc_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.write.status = hs_to_gatt_status(error->status);
  param.write.conn_id = conn_handle;
  param.write.handle = attr != nullptr ? attr->handle : error->att_handle;
  auto op = reinterpret_cast<uintptr_t>(arg);
  dispatch_gattc(op == OP_WRITE_DESCR ? ESP_GATTC_WRITE_DESCR_EVT : ESP_GATTC_WRITE_CHAR_EVT, entry->gattc_if, &param);
  return 0;
}

}  // namespace esphome::esp32_ble::nimble_compat

// ---------------------------------------------------------------------------
// Bluedroid-shaped public functions
// ---------------------------------------------------------------------------

namespace nc = esphome::esp32_ble::nimble_compat;

esp_err_t esp_ble_gattc_app_register(uint16_t app_id) {
  for (size_t i = 0; i < sizeof(nc::gattc_apps) / sizeof(nc::gattc_apps[0]); i++) {
    if (!nc::gattc_apps[i].in_use) {
      nc::gattc_apps[i].in_use = true;
      nc::gattc_apps[i].app_id = app_id;
      esp_ble_gattc_cb_param_t param;
      memset(&param, 0, sizeof(param));
      param.reg.status = ESP_GATT_OK;
      param.reg.app_id = app_id;
      nc::post_gattc(ESP_GATTC_REG_EVT, static_cast<esp_gatt_if_t>(i + 1), param);
      return ESP_OK;
    }
  }
  return ESP_ERR_NO_MEM;
}

esp_err_t esp_ble_gap_set_prefer_conn_params(esp_bd_addr_t bd_addr, uint16_t min_conn_int, uint16_t max_conn_int,
                                             uint16_t slave_latency, uint16_t supervision_tout) {
  nc::preferred_params.valid = true;
  memcpy(nc::preferred_params.addr, bd_addr, 6);
  nc::preferred_params.min_int = min_conn_int;
  nc::preferred_params.max_int = max_conn_int;
  nc::preferred_params.latency = slave_latency;
  nc::preferred_params.timeout = supervision_tout;
  return ESP_OK;
}

esp_err_t esp_ble_gattc_open(esp_gatt_if_t gattc_if, esp_bd_addr_t remote_bda, esp_ble_addr_type_t remote_addr_type,
                             bool is_direct) {
  if (nc::pending_open.active)
    return ESP_ERR_INVALID_STATE;

  ble_addr_t peer;
  peer.type = static_cast<uint8_t>(remote_addr_type);
  nc::addr_bd_to_ble(remote_bda, peer.val);

  struct ble_gap_conn_params params = {};
  struct ble_gap_conn_params *params_ptr = nullptr;
  if (nc::preferred_params.valid && memcmp(nc::preferred_params.addr, remote_bda, 6) == 0) {
    params.scan_itvl = 0x0010;
    params.scan_window = 0x0010;
    params.itvl_min = nc::preferred_params.min_int;
    params.itvl_max = nc::preferred_params.max_int;
    params.latency = nc::preferred_params.latency;
    params.supervision_timeout = nc::preferred_params.timeout;
    params_ptr = &params;
  }

  nc::pending_open.active = true;
  nc::pending_open.iface = gattc_if;
  memcpy(nc::pending_open.addr, remote_bda, 6);
  nc::pending_open.addr_type = remote_addr_type;

  int rc = ble_gap_connect(0 /* BLE_OWN_ADDR_PUBLIC */, &peer, USE_ESP32_BLE_CONNECTION_TIMEOUT_MS, params_ptr,
                           nc::client_gap_event_cb, nullptr);
  if (rc != 0) {
    nc::pending_open.active = false;
    ESP_LOGW(nc::TAG, "ble_gap_connect failed: %d", rc);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t esp_ble_gattc_close(esp_gatt_if_t gattc_if, uint16_t conn_id) {
  nc::ConnEntry *entry = nc::conn_by_handle(conn_id);
  if (entry == nullptr) {
    // No live link; cancel a pending direct connection if it belongs here.
    if (nc::pending_open.active && nc::pending_open.iface == gattc_if) {
      ble_gap_conn_cancel();
    }
    return ESP_OK;
  }
  int rc = ble_gap_terminate(entry->conn_handle, 0x13 /* remote user terminated */);
  return (rc == 0 || rc == BLE_HS_ENOTCONN || rc == BLE_HS_EALREADY) ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_ble_gattc_send_mtu_req(esp_gatt_if_t gattc_if, uint16_t conn_id) {
  int rc = ble_gattc_exchange_mtu(conn_id, nullptr, nullptr);
  return (rc == 0 || rc == BLE_HS_EALREADY) ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_ble_gattc_search_service(esp_gatt_if_t gattc_if, uint16_t conn_id, esp_bt_uuid_t *filter_uuid) {
  nc::ConnEntry *entry = nc::conn_by_handle(conn_id);
  if (entry == nullptr)
    return ESP_ERR_INVALID_STATE;
  // ESPHome always discovers all services (filter_uuid is nullptr in-tree).
  nc::gattc_free_table(entry);
  nc::get_table(entry, true);
  int rc = ble_gattc_disc_all_svcs(conn_id, nc::svc_disc_cb, entry);
  if (rc != 0) {
    ESP_LOGW(nc::TAG, "disc_all_svcs failed: %d", rc);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_gatt_status_t esp_ble_gattc_get_attr_count(esp_gatt_if_t gattc_if, uint16_t conn_id, esp_gatt_db_attr_type_t type,
                                               uint16_t start_handle, uint16_t end_handle, uint16_t char_handle,
                                               uint16_t *count) {
  nc::ConnEntry *entry = nc::conn_by_handle(conn_id);
  if (entry == nullptr || count == nullptr)
    return ESP_GATT_INVALID_HANDLE;
  // Hold the facade lock for the whole walk; the host task frees the table
  // on disconnect (see gattc_free_table).
  nc::FacadeLockGuard guard;
  nc::GattTable *table = nc::get_table(entry, false);
  if (table == nullptr || !table->complete)
    return ESP_GATT_WRONG_STATE;
  uint16_t n = 0;
  switch (type) {
    case ESP_GATT_DB_PRIMARY_SERVICE:
      for (const auto &svc : table->svcs) {
        if (svc.start_handle >= start_handle && svc.start_handle <= end_handle)
          n++;
      }
      break;
    case ESP_GATT_DB_SECONDARY_SERVICE:
      break;  // not tracked; NimBLE's all-svcs discovery returns primary only
    case ESP_GATT_DB_CHARACTERISTIC:
      for (const auto &chr : table->chrs) {
        if (chr.val_handle >= start_handle && chr.val_handle <= end_handle)
          n++;
      }
      break;
    case ESP_GATT_DB_DESCRIPTOR:
      for (const auto &dsc : table->dscs) {
        if (dsc.chr_val_handle == char_handle)
          n++;
      }
      break;
    default:
      return ESP_GATT_ILLEGAL_PARAMETER;
  }
  *count = n;
  return ESP_GATT_OK;
}

esp_gatt_status_t esp_ble_gattc_get_service(esp_gatt_if_t gattc_if, uint16_t conn_id, esp_bt_uuid_t *svc_uuid,
                                            esp_gattc_service_elem_t *result, uint16_t *count, uint16_t offset) {
  nc::ConnEntry *entry = nc::conn_by_handle(conn_id);
  if (entry == nullptr || result == nullptr || count == nullptr)
    return ESP_GATT_INVALID_HANDLE;
  // Hold the facade lock for the whole walk; the host task frees the table
  // on disconnect (see gattc_free_table).
  nc::FacadeLockGuard guard;
  nc::GattTable *table = nc::get_table(entry, false);
  if (table == nullptr || !table->complete)
    return ESP_GATT_WRONG_STATE;
  uint16_t out = 0;
  for (size_t i = offset; i < table->svcs.size() && out < *count; i++) {
    const auto &svc = table->svcs[i];
    result[out].is_primary = true;
    result[out].start_handle = svc.start_handle;
    result[out].end_handle = svc.end_handle;
    nc::uuid_any_to_bt(svc.uuid, result[out].uuid);
    out++;
  }
  *count = out;
  return out > 0 ? ESP_GATT_OK : ESP_GATT_NOT_FOUND;
}

esp_gatt_status_t esp_ble_gattc_get_all_char(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t start_handle,
                                             uint16_t end_handle, esp_gattc_char_elem_t *result, uint16_t *count,
                                             uint16_t offset) {
  nc::ConnEntry *entry = nc::conn_by_handle(conn_id);
  if (entry == nullptr || result == nullptr || count == nullptr)
    return ESP_GATT_INVALID_HANDLE;
  // Hold the facade lock for the whole walk; the host task frees the table
  // on disconnect (see gattc_free_table).
  nc::FacadeLockGuard guard;
  nc::GattTable *table = nc::get_table(entry, false);
  if (table == nullptr || !table->complete)
    return ESP_GATT_WRONG_STATE;
  // Collect matches, then apply offset within the match list.
  uint16_t matched = 0;
  uint16_t out = 0;
  for (const auto &chr : table->chrs) {
    if (chr.val_handle < start_handle || chr.val_handle > end_handle)
      continue;
    if (matched++ < offset)
      continue;
    if (out >= *count)
      break;
    result[out].char_handle = chr.val_handle;
    result[out].properties = chr.properties;
    nc::uuid_any_to_bt(chr.uuid, result[out].uuid);
    out++;
  }
  *count = out;
  return out > 0 ? ESP_GATT_OK : ESP_GATT_NOT_FOUND;
}

esp_gatt_status_t esp_ble_gattc_get_all_descr(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t char_handle,
                                              esp_gattc_descr_elem_t *result, uint16_t *count, uint16_t offset) {
  nc::ConnEntry *entry = nc::conn_by_handle(conn_id);
  if (entry == nullptr || result == nullptr || count == nullptr)
    return ESP_GATT_INVALID_HANDLE;
  // Hold the facade lock for the whole walk; the host task frees the table
  // on disconnect (see gattc_free_table).
  nc::FacadeLockGuard guard;
  nc::GattTable *table = nc::get_table(entry, false);
  if (table == nullptr || !table->complete)
    return ESP_GATT_WRONG_STATE;
  uint16_t matched = 0;
  uint16_t out = 0;
  for (const auto &dsc : table->dscs) {
    if (dsc.chr_val_handle != char_handle)
      continue;
    if (matched++ < offset)
      continue;
    if (out >= *count)
      break;
    result[out].handle = dsc.handle;
    nc::uuid_any_to_bt(dsc.uuid, result[out].uuid);
    out++;
  }
  *count = out;
  return out > 0 ? ESP_GATT_OK : ESP_GATT_NOT_FOUND;
}

esp_gatt_status_t esp_ble_gattc_get_descr_by_char_handle(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t char_handle,
                                                         esp_bt_uuid_t descr_uuid, esp_gattc_descr_elem_t *result,
                                                         uint16_t *count) {
  nc::ConnEntry *entry = nc::conn_by_handle(conn_id);
  if (entry == nullptr || result == nullptr || count == nullptr)
    return ESP_GATT_INVALID_HANDLE;
  // Hold the facade lock for the whole walk; the host task frees the table
  // on disconnect (see gattc_free_table).
  nc::FacadeLockGuard guard;
  nc::GattTable *table = nc::get_table(entry, false);
  if (table == nullptr || !table->complete)
    return ESP_GATT_WRONG_STATE;
  uint16_t out = 0;
  for (const auto &dsc : table->dscs) {
    if (dsc.chr_val_handle != char_handle)
      continue;
    esp_bt_uuid_t uuid;
    nc::uuid_any_to_bt(dsc.uuid, uuid);
    if (uuid.len != descr_uuid.len)
      continue;
    if (uuid.len == ESP_UUID_LEN_16 && uuid.uuid.uuid16 != descr_uuid.uuid.uuid16)
      continue;
    if (uuid.len == ESP_UUID_LEN_32 && uuid.uuid.uuid32 != descr_uuid.uuid.uuid32)
      continue;
    if (uuid.len == ESP_UUID_LEN_128 && memcmp(uuid.uuid.uuid128, descr_uuid.uuid.uuid128, 16) != 0)
      continue;
    if (out < *count) {
      result[out].handle = dsc.handle;
      result[out].uuid = uuid;
      out++;
    }
  }
  *count = out;
  return out > 0 ? ESP_GATT_OK : ESP_GATT_NOT_FOUND;
}

esp_err_t esp_ble_gattc_read_char(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t handle,
                                  esp_gatt_auth_req_t auth_req) {
  // read_long continues with ATT Read Blob requests so values larger than
  // MTU-1 arrive complete, matching Bluedroid's esp_ble_gattc_read_char.
  int rc = ble_gattc_read_long(conn_id, handle, 0, nc::read_attr_cb, reinterpret_cast<void *>(nc::OP_READ_CHAR));
  return rc == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_ble_gattc_read_char_descr(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t handle,
                                        esp_gatt_auth_req_t auth_req) {
  int rc = ble_gattc_read_long(conn_id, handle, 0, nc::read_attr_cb, reinterpret_cast<void *>(nc::OP_READ_DESCR));
  return rc == 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t gattc_write_common(uint16_t conn_id, uint16_t handle, uint16_t value_len, uint8_t *value,
                                    esp_gatt_write_type_t write_type, uintptr_t op) {
  if (write_type == ESP_GATT_WRITE_TYPE_NO_RSP) {
    int rc = ble_gattc_write_no_rsp_flat(conn_id, handle, value, value_len);
    if (rc != 0)
      return ESP_FAIL;
    // Bluedroid still delivers a WRITE event for no-response writes.
    esp_ble_gattc_cb_param_t param;
    memset(&param, 0, sizeof(param));
    param.write.status = ESP_GATT_OK;
    param.write.conn_id = conn_id;
    param.write.handle = handle;
    nc::ConnEntry *entry = nc::conn_by_handle(conn_id);
    if (entry != nullptr) {
      nc::post_gattc(op == nc::OP_WRITE_DESCR ? ESP_GATTC_WRITE_DESCR_EVT : ESP_GATTC_WRITE_CHAR_EVT, entry->gattc_if,
                     param);
    }
    return ESP_OK;
  }
  int rc = ble_gattc_write_flat(conn_id, handle, value, value_len, nc::write_attr_cb, reinterpret_cast<void *>(op));
  return rc == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_ble_gattc_write_char(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t handle, uint16_t value_len,
                                   uint8_t *value, esp_gatt_write_type_t write_type, esp_gatt_auth_req_t auth_req) {
  return gattc_write_common(conn_id, handle, value_len, value, write_type, nc::OP_WRITE_CHAR);
}

esp_err_t esp_ble_gattc_write_char_descr(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t handle, uint16_t value_len,
                                         uint8_t *value, esp_gatt_write_type_t write_type,
                                         esp_gatt_auth_req_t auth_req) {
  return gattc_write_common(conn_id, handle, value_len, value, write_type, nc::OP_WRITE_DESCR);
}

esp_err_t esp_ble_gattc_register_for_notify(esp_gatt_if_t gattc_if, esp_bd_addr_t server_bda, uint16_t handle) {
  // NimBLE delivers all notifications without registration; only the
  // Bluedroid-shaped acknowledgment event is needed.
  esp_ble_gattc_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.reg_for_notify.status = ESP_GATT_OK;
  param.reg_for_notify.handle = handle;
  nc::post_gattc(ESP_GATTC_REG_FOR_NOTIFY_EVT, gattc_if, param);
  return ESP_OK;
}

esp_err_t esp_ble_gattc_unregister_for_notify(esp_gatt_if_t gattc_if, esp_bd_addr_t server_bda, uint16_t handle) {
  esp_ble_gattc_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.unreg_for_notify.status = ESP_GATT_OK;
  param.unreg_for_notify.handle = handle;
  nc::post_gattc(ESP_GATTC_UNREG_FOR_NOTIFY_EVT, gattc_if, param);
  return ESP_OK;
}

esp_err_t esp_ble_gattc_cache_clean(esp_bd_addr_t remote_bda) {
  nc::ConnEntry *entry = nc::conn_by_addr(remote_bda);
  if (entry != nullptr) {
    nc::gattc_free_table(entry);
  }
  return ESP_OK;
}

#endif  // USE_ESP32_BLE_NIMBLE && USE_ESP32_BLE_CLIENT
