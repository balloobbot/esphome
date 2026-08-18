#include "esphome/core/defines.h"

#if defined(USE_ESP32_BLE_NIMBLE) && defined(USE_ESP32_BLE_SERVER)

#include "nimble_compat_internal.h"

#include "esphome/core/log.h"

#include <vector>

extern "C" {
#include <host/ble_att.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <host/ble_hs_mbuf.h>
#include <os/os_mbuf.h>
}

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace esphome::esp32_ble::nimble_compat {

static const char *const TAG = "esp32_ble.nimble";

// The facade allocates its own attribute handle space (Bluedroid hands out
// handles at creation time; NimBLE only at registration). Synthetic handles
// are what consumers see; they are mapped to NimBLE handles where needed.
static uint16_t next_synthetic_handle = 0x0100;  // NOLINT(readability-identifier-naming)

static void uuid_bt_to_any(const esp_bt_uuid_t &in, ble_uuid_any_t &out) {
  switch (in.len) {
    case ESP_UUID_LEN_16:
      out.u16.u.type = BLE_UUID_TYPE_16;
      out.u16.value = in.uuid.uuid16;
      break;
    case ESP_UUID_LEN_32:
      out.u32.u.type = BLE_UUID_TYPE_32;
      out.u32.value = in.uuid.uuid32;
      break;
    default:
      out.u128.u.type = BLE_UUID_TYPE_128;
      memcpy(out.u128.value, in.uuid.uuid128, 16);
      break;
  }
}

struct FacadeDsc {
  esp_bt_uuid_t uuid{};
  uint16_t handle{0};  // synthetic
  bool is_cccd{false};
  uint16_t max_len{0};
  esp_gatt_perm_t perm{0};
  std::vector<uint8_t> value;  // AUTO_RSP storage
};

struct FacadeChr {
  esp_bt_uuid_t uuid{};
  ble_uuid_any_t nimble_uuid{};
  uint16_t handle{0};      // synthetic (Bluedroid attr_handle of the value)
  uint16_t val_handle{0};  // NimBLE-assigned at registration
  esp_gatt_char_prop_t properties{0};
  std::vector<FacadeDsc *> dscs;
};

struct FacadeSvc {
  esp_gatt_srvc_id_t id{};
  ble_uuid_any_t nimble_uuid{};
  uint16_t handle{0};  // synthetic service handle
  bool registered{false};
  std::vector<FacadeChr *> chrs;
  // Persistent NimBLE definition arrays; NimBLE keeps pointers to these while
  // the service is registered.
  std::vector<ble_gatt_chr_def> chr_defs;
  std::vector<std::vector<ble_gatt_dsc_def>> dsc_defs;
  std::vector<ble_uuid_any_t> dsc_uuids;
  ble_gatt_svc_def svc_def[2]{};
};

static std::vector<FacadeSvc *> services;  // NOLINT(readability-identifier-naming)
static bool gatts_app_registered = false;  // NOLINT(readability-identifier-naming)
static constexpr esp_gatt_if_t GATTS_IF = 1;

static FacadeSvc *find_service(uint16_t service_handle) {
  for (auto *svc : services) {
    if (svc->handle == service_handle)
      return svc;
  }
  return nullptr;
}

static FacadeChr *find_chr_by_synthetic(uint16_t handle) {
  for (auto *svc : services) {
    for (auto *chr : svc->chrs) {
      if (chr->handle == handle)
        return chr;
    }
  }
  return nullptr;
}

static FacadeChr *find_chr_by_val_handle(uint16_t val_handle) {
  for (auto *svc : services) {
    for (auto *chr : svc->chrs) {
      if (chr->val_handle == val_handle)
        return chr;
    }
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// RSP_BY_APP emulation: the access callback blocks the host task until the
// main loop answers with esp_ble_gatts_send_response() (same contract as
// Bluedroid's ESP_GATT_RSP_BY_APP). Access callbacks are serialized on the
// host task, so a single pending slot suffices.
// ---------------------------------------------------------------------------

struct PendingResponse {
  SemaphoreHandle_t sem{nullptr};
  uint32_t trans_id{0};
  bool waiting{false};    // guarded by the facade lock
  bool responded{false};  // guarded by the facade lock
  esp_gatt_status_t status{ESP_GATT_OK};
  uint16_t len{0};
  uint8_t value[ESP_GATT_MAX_ATTR_LEN];
};
static PendingResponse pending_rsp;  // NOLINT(readability-identifier-naming)
static uint32_t next_trans_id = 1;   // NOLINT(readability-identifier-naming)
// Keep short: while an app-supplied read response is pending the whole NimBLE
// host task is parked (see the comment on chr_access_cb below), so the worst
// case must stay well below connection supervision timeouts.
static constexpr uint32_t RSP_TIMEOUT_MS = 1000;

static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
  auto *chr = static_cast<FacadeChr *>(arg);
  esp_ble_gatts_cb_param_t param;
  switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR: {
      // Bluedroid's ESP_GATT_RSP_BY_APP contract: deliver READ_EVT to the
      // main loop and wait for esp_ble_gatts_send_response(). NimBLE requires
      // the value before this callback returns, so the host task blocks here
      // for one main-loop round trip (bounded by RSP_TIMEOUT_MS).
      if (pending_rsp.sem == nullptr)
        pending_rsp.sem = xSemaphoreCreateBinary();
      // Drain a stale token left by a response that raced a previous timeout.
      xSemaphoreTake(pending_rsp.sem, 0);
      uint32_t trans_id = next_trans_id++;
      facade_lock();
      pending_rsp.trans_id = trans_id;
      pending_rsp.waiting = true;
      pending_rsp.responded = false;
      facade_unlock();

      memset(&param, 0, sizeof(param));
      param.read.conn_id = conn_handle;
      param.read.trans_id = trans_id;
      param.read.handle = chr->handle;
      param.read.offset = 0;
      param.read.is_long = false;
      param.read.need_rsp = true;
      ble_gap_conn_desc desc;
      if (ble_gap_conn_find(conn_handle, &desc) == 0) {
        addr_ble_to_bd(desc.peer_ota_addr.val, param.read.bda);
      }
      dispatch_gatts(ESP_GATTS_READ_EVT, GATTS_IF, &param);

      bool got_token = xSemaphoreTake(pending_rsp.sem, pdMS_TO_TICKS(RSP_TIMEOUT_MS)) == pdTRUE;
      facade_lock();
      bool responded = pending_rsp.responded && pending_rsp.trans_id == trans_id;
      pending_rsp.waiting = false;
      facade_unlock();
      if (!got_token || !responded) {
        ESP_LOGW(TAG, "Read response timeout on handle 0x%04X", chr->handle);
        return BLE_ATT_ERR_UNLIKELY;
      }
      if (pending_rsp.status != ESP_GATT_OK)
        return pending_rsp.status;  // ATT error codes share values
      if (os_mbuf_append(ctxt->om, pending_rsp.value, pending_rsp.len) != 0)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
      return 0;
    }
    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
      uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
      uint8_t buf[ESP_GATT_MAX_ATTR_LEN];
      if (len > sizeof(buf))
        len = sizeof(buf);
      os_mbuf_copydata(ctxt->om, 0, len, buf);
      memset(&param, 0, sizeof(param));
      param.write.conn_id = conn_handle;
      param.write.trans_id = next_trans_id++;
      param.write.handle = chr->handle;
      param.write.offset = 0;
      // NimBLE acknowledges write requests itself once this callback returns.
      param.write.need_rsp = false;
      param.write.is_prep = false;
      param.write.len = len;
      param.write.value = buf;
      ble_gap_conn_desc desc;
      if (ble_gap_conn_find(conn_handle, &desc) == 0) {
        addr_ble_to_bd(desc.peer_ota_addr.val, param.write.bda);
      }
      dispatch_gatts(ESP_GATTS_WRITE_EVT, GATTS_IF, &param);
      return 0;
    }
    default:
      return BLE_ATT_ERR_UNLIKELY;
  }
}

static int dsc_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
  auto *dsc = static_cast<FacadeDsc *>(arg);
  switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_DSC:
      // AUTO_RSP: answer from stored value.
      if (os_mbuf_append(ctxt->om, dsc->value.data(), dsc->value.size()) != 0)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
      return 0;
    case BLE_GATT_ACCESS_OP_WRITE_DSC: {
      uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
      if (dsc->max_len != 0 && len > dsc->max_len)
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
      dsc->value.resize(len);
      os_mbuf_copydata(ctxt->om, 0, len, dsc->value.data());
      esp_ble_gatts_cb_param_t param;
      memset(&param, 0, sizeof(param));
      param.write.conn_id = conn_handle;
      param.write.trans_id = next_trans_id++;
      param.write.handle = dsc->handle;
      param.write.need_rsp = false;
      param.write.len = len;
      param.write.value = dsc->value.data();
      ble_gap_conn_desc desc;
      if (ble_gap_conn_find(conn_handle, &desc) == 0) {
        addr_ble_to_bd(desc.peer_ota_addr.val, param.write.bda);
      }
      dispatch_gatts(ESP_GATTS_WRITE_EVT, GATTS_IF, &param);
      return 0;
    }
    default:
      return BLE_ATT_ERR_UNLIKELY;
  }
}

// ---------------------------------------------------------------------------
// Peripheral link events (called from the advertising GAP callback)
// ---------------------------------------------------------------------------

void gatts_on_connect(const struct ble_gap_event *event) {
  if (event->connect.status != 0)
    return;
  ConnEntry *entry = conn_alloc(event->connect.conn_handle);
  esp_ble_gatts_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.connect.conn_id = event->connect.conn_handle;
  param.connect.conn_handle = event->connect.conn_handle;
  ble_gap_conn_desc desc;
  if (ble_gap_conn_find(event->connect.conn_handle, &desc) == 0) {
    addr_ble_to_bd(desc.peer_ota_addr.val, param.connect.remote_bda);
    param.connect.ble_addr_type = static_cast<esp_ble_addr_type_t>(desc.peer_ota_addr.type);
    if (entry != nullptr) {
      memcpy(entry->addr, param.connect.remote_bda, 6);
      entry->addr_type = param.connect.ble_addr_type;
    }
  }
  dispatch_gatts(ESP_GATTS_CONNECT_EVT, GATTS_IF, &param);
}

void gatts_on_disconnect(const struct ble_gap_event *event) {
  ConnEntry *entry = conn_by_handle(event->disconnect.conn.conn_handle);
  esp_ble_gatts_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.disconnect.conn_id = event->disconnect.conn.conn_handle;
  addr_ble_to_bd(event->disconnect.conn.peer_ota_addr.val, param.disconnect.remote_bda);
  param.disconnect.reason = static_cast<esp_gatt_conn_reason_t>(
      event->disconnect.reason >= 0x200 && event->disconnect.reason < 0x300 ? event->disconnect.reason - 0x200
                                                                            : ESP_GATT_CONN_UNKNOWN);
  dispatch_gatts(ESP_GATTS_DISCONNECT_EVT, GATTS_IF, &param);
  if (entry != nullptr)
    conn_release(entry);
}

void gatts_on_subscribe(const struct ble_gap_event *event) {
  // Translate NimBLE's subscribe event into a Bluedroid-style CCCD write so
  // BLECharacteristic's notify bookkeeping works unchanged.
  FacadeChr *chr = find_chr_by_val_handle(event->subscribe.attr_handle);
  if (chr == nullptr)
    return;
  FacadeDsc *cccd = nullptr;
  for (auto *dsc : chr->dscs) {
    if (dsc->is_cccd) {
      cccd = dsc;
      break;
    }
  }
  if (cccd == nullptr)
    return;
  uint8_t value[2] = {0, 0};
  if (event->subscribe.cur_notify)
    value[0] |= 0x01;
  if (event->subscribe.cur_indicate)
    value[0] |= 0x02;
  cccd->value.assign(value, value + 2);

  esp_ble_gatts_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.write.conn_id = event->subscribe.conn_handle;
  param.write.trans_id = next_trans_id++;
  param.write.handle = cccd->handle;
  param.write.need_rsp = false;
  param.write.len = 2;
  param.write.value = value;
  ble_gap_conn_desc desc;
  if (ble_gap_conn_find(event->subscribe.conn_handle, &desc) == 0) {
    addr_ble_to_bd(desc.peer_ota_addr.val, param.write.bda);
  }
  dispatch_gatts(ESP_GATTS_WRITE_EVT, GATTS_IF, &param);
}

// ---------------------------------------------------------------------------
// Service registration
// ---------------------------------------------------------------------------

static int register_service(FacadeSvc *svc) {
  svc->chr_defs.clear();
  svc->dsc_defs.clear();
  svc->dsc_uuids.clear();
  svc->chr_defs.reserve(svc->chrs.size() + 1);
  svc->dsc_defs.resize(svc->chrs.size());
  // Reserve so pointers into dsc_uuids stay stable while building.
  size_t total_dscs = 0;
  for (auto *chr : svc->chrs)
    total_dscs += chr->dscs.size();
  svc->dsc_uuids.reserve(total_dscs);

  for (size_t ci = 0; ci < svc->chrs.size(); ci++) {
    FacadeChr *chr = svc->chrs[ci];
    auto &dscs = svc->dsc_defs[ci];
    for (auto *dsc : chr->dscs) {
      if (dsc->is_cccd)
        continue;  // NimBLE manages the real CCCD itself
      svc->dsc_uuids.emplace_back();
      uuid_bt_to_any(dsc->uuid, svc->dsc_uuids.back());
      ble_gatt_dsc_def def = {};
      def.uuid = &svc->dsc_uuids.back().u;
      def.att_flags = 0;
      if (dsc->perm & ESP_GATT_PERM_READ)
        def.att_flags |= BLE_ATT_F_READ;
      if (dsc->perm & ESP_GATT_PERM_WRITE)
        def.att_flags |= BLE_ATT_F_WRITE;
      def.access_cb = dsc_access_cb;
      def.arg = dsc;
      dscs.push_back(def);
    }
    if (!dscs.empty()) {
      dscs.push_back(ble_gatt_dsc_def{});  // terminator
    }

    ble_gatt_chr_def def = {};
    def.uuid = &chr->nimble_uuid.u;
    def.access_cb = chr_access_cb;
    def.arg = chr;
    def.descriptors = dscs.empty() ? nullptr : dscs.data();
    // Bluedroid property bits and NimBLE characteristic flags share the low
    // bit layout (broadcast/read/write_nr/write/notify/indicate).
    def.flags = chr->properties;
    def.val_handle = &chr->val_handle;
    svc->chr_defs.push_back(def);
  }
  svc->chr_defs.push_back(ble_gatt_chr_def{});  // terminator

  svc->svc_def[0] = {};
  svc->svc_def[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
  svc->svc_def[0].uuid = &svc->nimble_uuid.u;
  svc->svc_def[0].characteristics = svc->chr_defs.data();
  svc->svc_def[1] = {};  // terminator

  int rc = ble_gatts_add_dynamic_svcs(svc->svc_def);
  if (rc != 0) {
    ESP_LOGE(TAG, "add_dynamic_svcs failed: %d", rc);
    return rc;
  }
  svc->registered = true;
  return 0;
}

}  // namespace esphome::esp32_ble::nimble_compat

// ---------------------------------------------------------------------------
// Bluedroid-shaped public functions
// ---------------------------------------------------------------------------

namespace nc = esphome::esp32_ble::nimble_compat;

esp_err_t esp_ble_gatts_app_register(uint16_t app_id) {
  if (nc::gatts_app_registered)
    return ESP_ERR_INVALID_STATE;
  nc::gatts_app_registered = true;
  esp_ble_gatts_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.reg.status = ESP_GATT_OK;
  param.reg.app_id = app_id;
  nc::post_gatts(ESP_GATTS_REG_EVT, nc::GATTS_IF, param);
  return ESP_OK;
}

esp_err_t esp_ble_gatts_create_service(esp_gatt_if_t gatts_if, esp_gatt_srvc_id_t *service_id, uint16_t num_handle) {
  auto *svc = new nc::FacadeSvc();  // NOLINT(cppcoreguidelines-owning-memory)
  svc->id = *service_id;
  nc::uuid_bt_to_any(service_id->id.uuid, svc->nimble_uuid);
  svc->handle = nc::next_synthetic_handle++;
  nc::services.push_back(svc);

  esp_ble_gatts_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.create.status = ESP_GATT_OK;
  param.create.service_handle = svc->handle;
  param.create.service_id = *service_id;
  nc::post_gatts(ESP_GATTS_CREATE_EVT, gatts_if, param);
  return ESP_OK;
}

esp_err_t esp_ble_gatts_add_char(uint16_t service_handle, esp_bt_uuid_t *char_uuid, esp_gatt_perm_t perm,
                                 esp_gatt_char_prop_t property, esp_attr_value_t *char_val,
                                 esp_attr_control_t *control) {
  nc::FacadeSvc *svc = nc::find_service(service_handle);
  if (svc == nullptr)
    return ESP_ERR_INVALID_ARG;
  auto *chr = new nc::FacadeChr();  // NOLINT(cppcoreguidelines-owning-memory)
  chr->uuid = *char_uuid;
  nc::uuid_bt_to_any(*char_uuid, chr->nimble_uuid);
  chr->properties = property;
  chr->handle = nc::next_synthetic_handle++;
  svc->chrs.push_back(chr);

  esp_ble_gatts_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.add_char.status = ESP_GATT_OK;
  param.add_char.attr_handle = chr->handle;
  param.add_char.service_handle = service_handle;
  param.add_char.char_uuid = *char_uuid;
  nc::post_gatts(ESP_GATTS_ADD_CHAR_EVT, nc::GATTS_IF, param);
  return ESP_OK;
}

esp_err_t esp_ble_gatts_add_char_descr(uint16_t service_handle, esp_bt_uuid_t *descr_uuid, esp_gatt_perm_t perm,
                                       esp_attr_value_t *char_descr_val, esp_attr_control_t *control) {
  nc::FacadeSvc *svc = nc::find_service(service_handle);
  if (svc == nullptr || svc->chrs.empty())
    return ESP_ERR_INVALID_ARG;
  nc::FacadeChr *chr = svc->chrs.back();

  auto *dsc = new nc::FacadeDsc();  // NOLINT(cppcoreguidelines-owning-memory)
  dsc->uuid = *descr_uuid;
  dsc->handle = nc::next_synthetic_handle++;
  dsc->is_cccd = descr_uuid->len == ESP_UUID_LEN_16 && descr_uuid->uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
  dsc->perm = perm;
  if (char_descr_val != nullptr && char_descr_val->attr_value != nullptr) {
    dsc->value.assign(char_descr_val->attr_value, char_descr_val->attr_value + char_descr_val->attr_len);
    dsc->max_len = char_descr_val->attr_max_len;
  }
  chr->dscs.push_back(dsc);

  esp_ble_gatts_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.add_char_descr.status = ESP_GATT_OK;
  param.add_char_descr.attr_handle = dsc->handle;
  param.add_char_descr.service_handle = service_handle;
  param.add_char_descr.descr_uuid = *descr_uuid;
  nc::post_gatts(ESP_GATTS_ADD_CHAR_DESCR_EVT, nc::GATTS_IF, param);
  return ESP_OK;
}

esp_err_t esp_ble_gatts_start_service(uint16_t service_handle) {
  nc::FacadeSvc *svc = nc::find_service(service_handle);
  if (svc == nullptr)
    return ESP_ERR_INVALID_ARG;
  esp_gatt_status_t status = ESP_GATT_OK;
  if (!svc->registered) {
    if (nc::register_service(svc) != 0) {
      status = ESP_GATT_ERROR;
    }
  } else {
    // Re-starting a stopped service: restore its visibility.
    uint16_t real_handle = 0;
    if (ble_gatts_find_svc(&svc->nimble_uuid.u, &real_handle) == 0) {
      ble_gatts_svc_set_visibility(real_handle, 1);
    }
  }
  esp_ble_gatts_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.start.status = status;
  param.start.service_handle = service_handle;
  nc::post_gatts(ESP_GATTS_START_EVT, nc::GATTS_IF, param);
  return status == ESP_GATT_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_ble_gatts_stop_service(uint16_t service_handle) {
  nc::FacadeSvc *svc = nc::find_service(service_handle);
  esp_ble_gatts_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.stop.status = ESP_GATT_OK;
  param.stop.service_handle = service_handle;
  if (svc != nullptr && svc->registered) {
    uint16_t real_handle = 0;
    if (ble_gatts_find_svc(&svc->nimble_uuid.u, &real_handle) == 0) {
      ble_gatts_svc_set_visibility(real_handle, 0);
    }
  }
  nc::post_gatts(ESP_GATTS_STOP_EVT, nc::GATTS_IF, param);
  return ESP_OK;
}

static void free_service(nc::FacadeSvc *svc) {
  for (auto *chr : svc->chrs) {
    for (auto *dsc : chr->dscs)
      delete dsc;  // NOLINT(cppcoreguidelines-owning-memory)
    delete chr;    // NOLINT(cppcoreguidelines-owning-memory)
  }
  delete svc;  // NOLINT(cppcoreguidelines-owning-memory)
}

namespace esphome::esp32_ble::nimble_compat {
// Called from host_teardown(); the host task is stopped, so freeing is safe.
void gatts_reset() {
  for (auto *svc : services)
    free_service(svc);
  services.clear();
  gatts_app_registered = false;
  pending_rsp.waiting = false;
  pending_rsp.responded = false;
  next_synthetic_handle = 0x0100;
}
}  // namespace esphome::esp32_ble::nimble_compat

esp_err_t esp_ble_gatts_delete_service(uint16_t service_handle) {
  esp_ble_gatts_cb_param_t param;
  memset(&param, 0, sizeof(param));
  param.del.status = ESP_GATT_OK;
  param.del.service_handle = service_handle;
  for (auto it = nc::services.begin(); it != nc::services.end(); ++it) {
    nc::FacadeSvc *svc = *it;
    if (svc->handle != service_handle)
      continue;
    if (svc->registered) {
      // Keyed by UUID: duplicate service UUIDs with distinct inst_ids would
      // delete the first NimBLE instance. No in-tree consumer does this.
      ble_gatts_delete_svc(&svc->nimble_uuid.u);
    }
    if (nc::pending_rsp.waiting) {
      // An access callback may still hold pointers into this service; leak
      // rather than free under it. Deleting services is rare (disable flow).
      ESP_LOGW(nc::TAG, "Deleting service 0x%04X while a read is in flight; leaking its definition", service_handle);
    } else {
      free_service(svc);
    }
    nc::services.erase(it);
    break;
  }
  nc::post_gatts(ESP_GATTS_DELETE_EVT, nc::GATTS_IF, param);
  return ESP_OK;
}

esp_err_t esp_ble_gatts_send_response(esp_gatt_if_t gatts_if, uint16_t conn_id, uint32_t trans_id,
                                      esp_gatt_status_t status, esp_gatt_rsp_t *rsp) {
  // Validate and commit under the facade lock so a response racing a timeout
  // cannot be attributed to a newer transaction.
  nc::facade_lock();
  if (!nc::pending_rsp.waiting || nc::pending_rsp.trans_id != trans_id) {
    nc::facade_unlock();
    return ESP_ERR_INVALID_STATE;
  }
  nc::pending_rsp.status = status;
  nc::pending_rsp.len = 0;
  if (rsp != nullptr) {
    uint16_t len = rsp->attr_value.len;
    if (len > sizeof(nc::pending_rsp.value))
      len = sizeof(nc::pending_rsp.value);
    memcpy(nc::pending_rsp.value, rsp->attr_value.value, len);
    nc::pending_rsp.len = len;
  }
  nc::pending_rsp.responded = true;
  nc::facade_unlock();
  xSemaphoreGive(nc::pending_rsp.sem);
  return ESP_OK;
}

esp_err_t esp_ble_gatts_send_indicate(esp_gatt_if_t gatts_if, uint16_t conn_id, uint16_t attr_handle,
                                      uint16_t value_len, uint8_t *value, bool need_confirm) {
  nc::FacadeChr *chr = nc::find_chr_by_synthetic(attr_handle);
  if (chr == nullptr || chr->val_handle == 0)
    return ESP_ERR_INVALID_ARG;
  struct os_mbuf *om = ble_hs_mbuf_from_flat(value, value_len);
  if (om == nullptr)
    return ESP_ERR_NO_MEM;
  int rc;
  if (need_confirm) {
    rc = ble_gatts_indicate_custom(conn_id, chr->val_handle, om);
  } else {
    rc = ble_gatts_notify_custom(conn_id, chr->val_handle, om);
  }
  return rc == 0 ? ESP_OK : ESP_FAIL;
}

#endif  // USE_ESP32_BLE_NIMBLE && USE_ESP32_BLE_SERVER
