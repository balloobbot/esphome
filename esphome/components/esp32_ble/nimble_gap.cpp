#include "esphome/core/defines.h"

#ifdef USE_ESP32_BLE_NIMBLE

#include "nimble_compat_internal.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

extern "C" {
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <host/ble_sm.h>
#include <host/ble_store.h>
#include <host/util/util.h>
}

namespace esphome::esp32_ble::nimble_compat {

static const char *const TAG = "esp32_ble.nimble";

// ---------------------------------------------------------------------------
// Shared security event handling
// ---------------------------------------------------------------------------

#ifdef ESPHOME_NIMBLE_HAS_CONNECT
void emit_auth_cmpl(uint16_t conn_handle, int hs_status) {
  esp_ble_gap_cb_param_t param;
  memset(&param, 0, sizeof(param));
  auto &cmpl = param.ble_security.auth_cmpl;
  ble_gap_conn_desc desc;
  if (ble_gap_conn_find(conn_handle, &desc) == 0) {
    addr_ble_to_bd(desc.peer_ota_addr.val, cmpl.bd_addr);
    cmpl.addr_type = static_cast<esp_ble_addr_type_t>(desc.peer_ota_addr.type);
    uint8_t auth_mode = 0;
    if (desc.sec_state.bonded)
      auth_mode |= ESP_LE_AUTH_BOND;
    if (desc.sec_state.authenticated)
      auth_mode |= ESP_LE_AUTH_REQ_MITM;
    cmpl.auth_mode = auth_mode;
  }
  cmpl.dev_type = ESP_BT_DEVICE_TYPE_BLE;
  cmpl.success = hs_status == 0;
  cmpl.fail_reason = static_cast<uint8_t>(hs_status);
  dispatch_gap(ESP_GAP_BLE_AUTH_CMPL_EVT, &param);
}

static void handle_passkey_action(struct ble_gap_event *event) {
  esp_ble_gap_cb_param_t param;
  memset(&param, 0, sizeof(param));
  ble_gap_conn_desc desc;
  if (ble_gap_conn_find(event->passkey.conn_handle, &desc) == 0) {
    addr_ble_to_bd(desc.peer_ota_addr.val, param.ble_security.key_notif.bd_addr);
  }
  switch (event->passkey.params.action) {
    case BLE_SM_IOACT_DISP: {
      // NimBLE expects the application to generate and display the passkey;
      // Bluedroid generates it and notifies. Generate, inject, then notify.
      struct ble_sm_io pkey = {};
      pkey.action = BLE_SM_IOACT_DISP;
      pkey.passkey = random_uint32() % 1000000;
      ble_sm_inject_io(event->passkey.conn_handle, &pkey);
      param.ble_security.key_notif.passkey = pkey.passkey;
      dispatch_gap(ESP_GAP_BLE_PASSKEY_NOTIF_EVT, &param);
      break;
    }
    case BLE_SM_IOACT_INPUT:
      // Peer displays; consumer answers via esp_ble_passkey_reply().
      dispatch_gap(ESP_GAP_BLE_PASSKEY_REQ_EVT, &param);
      break;
    case BLE_SM_IOACT_NUMCMP:
      param.ble_security.key_notif.passkey = event->passkey.params.numcmp;
      dispatch_gap(ESP_GAP_BLE_NC_REQ_EVT, &param);
      break;
    default:
      ESP_LOGW(TAG, "Unsupported passkey action %d", event->passkey.params.action);
      break;
  }
}

int handle_common_gap_event(struct ble_gap_event *event) {
  switch (event->type) {
    case BLE_GAP_EVENT_ENC_CHANGE:
      emit_auth_cmpl(event->enc_change.conn_handle, event->enc_change.status);
      return 0;
    case BLE_GAP_EVENT_PASSKEY_ACTION:
      handle_passkey_action(event);
      return 0;
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
      // Peer lost its bond; drop ours and tell the host to retry pairing.
      ble_gap_conn_desc desc;
      if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
        ble_store_util_delete_peer(&desc.peer_id_addr);
      }
      return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    default:
      return -1;
  }
}
#else
int handle_common_gap_event(struct ble_gap_event *event) { return -1; }
#endif  // ESPHOME_NIMBLE_HAS_CONNECT

// ---------------------------------------------------------------------------
// Scanning
// ---------------------------------------------------------------------------

static esp_ble_scan_params_t scan_params;   // NOLINT(readability-identifier-naming)
static uint8_t scan_own_addr_type = 0;      // NOLINT(readability-identifier-naming)
static bool scan_cancel_requested = false;  // NOLINT(readability-identifier-naming)

static int scan_gap_event_cb(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
      const auto &disc = event->disc;
      if (disc.length_data > ESP_BLE_ADV_DATA_LEN_MAX + ESP_BLE_SCAN_RSP_DATA_LEN_MAX)
        return 0;
      esp_ble_gap_cb_param_t param;
      memset(&param, 0, sizeof(param));
      auto &rst = param.scan_rst;
      rst.search_evt = ESP_GAP_SEARCH_INQ_RES_EVT;
      addr_ble_to_bd(disc.addr.val, rst.bda);
      rst.ble_addr_type = static_cast<esp_ble_addr_type_t>(disc.addr.type);
      rst.dev_type = ESP_BT_DEVICE_TYPE_BLE;
      rst.rssi = disc.rssi;
      // Bluedroid reports adv data and scan response concatenated in ble_adv
      // with the two lengths split. NimBLE reports each PDU separately; mark
      // scan responses via scan_rsp_len so consumers see the same shape.
      memcpy(rst.ble_adv, disc.data, disc.length_data);
      if (disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP) {
        rst.scan_rsp_len = disc.length_data;
      } else {
        rst.adv_data_len = disc.length_data;
      }
      dispatch_gap(ESP_GAP_BLE_SCAN_RESULT_EVT, &param);
      return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE: {
      if (scan_cancel_requested) {
        // Stop was explicit; SCAN_STOP_COMPLETE is delivered from the stop
        // call and Bluedroid does not also signal INQ_CMPL there.
        scan_cancel_requested = false;
        return 0;
      }
      esp_ble_gap_cb_param_t param;
      memset(&param, 0, sizeof(param));
      param.scan_rst.search_evt = ESP_GAP_SEARCH_INQ_CMPL_EVT;
      dispatch_gap(ESP_GAP_BLE_SCAN_RESULT_EVT, &param);
      return 0;
    }
    default: {
      int rc = handle_common_gap_event(event);
      return rc < 0 ? 0 : rc;
    }
  }
}

}  // namespace esphome::esp32_ble::nimble_compat

namespace nc = esphome::esp32_ble::nimble_compat;

esp_err_t esp_ble_gap_set_scan_params(esp_ble_scan_params_t *params) {
  if (params == nullptr)
    return ESP_ERR_INVALID_ARG;
  nc::scan_params = *params;
  nc::scan_own_addr_type = static_cast<uint8_t>(params->own_addr_type);
  nc::post_gap_status(ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT, ESP_BT_STATUS_SUCCESS);
  return ESP_OK;
}

esp_err_t esp_ble_gap_start_scanning(uint32_t duration) {
  struct ble_gap_disc_params params = {};
  params.itvl = nc::scan_params.scan_interval;
  params.window = nc::scan_params.scan_window;
  params.passive = nc::scan_params.scan_type == BLE_SCAN_TYPE_PASSIVE;
  params.filter_duplicates = 0;
  params.filter_policy = 0;
  nc::scan_cancel_requested = false;
  int32_t duration_ms = duration == 0 ? BLE_HS_FOREVER : static_cast<int32_t>(duration) * 1000;
  int rc = ble_gap_disc(nc::scan_own_addr_type, duration_ms, &params, nc::scan_gap_event_cb, nullptr);
  nc::post_gap_status(ESP_GAP_BLE_SCAN_START_COMPLETE_EVT, rc == 0 ? ESP_BT_STATUS_SUCCESS : ESP_BT_STATUS_FAIL);
  return rc == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_ble_gap_stop_scanning(void) {
  nc::scan_cancel_requested = true;
  int rc = ble_gap_disc_cancel();
  bool ok = rc == 0 || rc == BLE_HS_EALREADY;
  nc::post_gap_status(ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT, ok ? ESP_BT_STATUS_SUCCESS : ESP_BT_STATUS_FAIL);
  return ok ? ESP_OK : ESP_FAIL;
}

// ---------------------------------------------------------------------------
// Advertising
// ---------------------------------------------------------------------------

namespace esphome::esp32_ble::nimble_compat {

static esp_ble_adv_params_t adv_params_cache;  // NOLINT(readability-identifier-naming)
static bool adv_params_valid = false;          // NOLINT(readability-identifier-naming)

static int adv_gap_event_cb(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
#ifdef USE_ESP32_BLE_SERVER
    case BLE_GAP_EVENT_CONNECT:
      gatts_on_connect(event);
      break;
    case BLE_GAP_EVENT_DISCONNECT:
      gatts_on_disconnect(event);
      break;
    case BLE_GAP_EVENT_SUBSCRIBE:
      gatts_on_subscribe(event);
      break;
#endif
    default: {
      int rc = handle_common_gap_event(event);
      return rc < 0 ? 0 : rc;
    }
  }
  return 0;
}

// Build NimBLE adv fields from the Bluedroid-shaped descriptor and apply.
static esp_err_t apply_adv_fields(esp_ble_adv_data_t *data) {
  struct ble_hs_adv_fields fields = {};
  // Slave preferred connection interval range (AD type 0x12 in Bluedroid's
  // packer; NimBLE uses the same 4-byte little-endian min/max encoding).
  uint8_t slave_itvl[4];

  if (!data->set_scan_rsp && data->flag != 0) {
    fields.flags = data->flag;
  }
  if (data->include_name) {
    const char *name = get_device_name();
    fields.name = reinterpret_cast<const uint8_t *>(name);
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
  }
  if (data->include_txpower) {
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.tx_pwr_lvl_is_present = 1;
  }
  if (data->appearance != 0) {
    fields.appearance = data->appearance;
    fields.appearance_is_present = 1;
  }
  if (data->min_interval > 0 && data->max_interval >= data->min_interval) {
    slave_itvl[0] = data->min_interval & 0xFF;
    slave_itvl[1] = (data->min_interval >> 8) & 0xFF;
    slave_itvl[2] = data->max_interval & 0xFF;
    slave_itvl[3] = (data->max_interval >> 8) & 0xFF;
    fields.slave_itvl_range = slave_itvl;
  }
  if (data->manufacturer_len > 0 && data->p_manufacturer_data != nullptr) {
    fields.mfg_data = data->p_manufacturer_data;
    fields.mfg_data_len = data->manufacturer_len;
  }
  if (data->service_data_len > 0 && data->p_service_data != nullptr) {
    // Bluedroid packs service data as AD 0x16: uuid16 prefix + payload.
    fields.svc_data_uuid16 = data->p_service_data;
    fields.svc_data_uuid16_len = data->service_data_len;
  }
  // ESPHome always advertises service UUIDs in 128-bit form (16 bytes each).
  ble_uuid128_t uuids128[3];
  if (data->service_uuid_len > 0 && data->p_service_uuid != nullptr) {
    uint8_t num = data->service_uuid_len / 16;
    if (num > 3)
      num = 3;
    for (uint8_t i = 0; i < num; i++) {
      uuids128[i].u.type = BLE_UUID_TYPE_128;
      memcpy(uuids128[i].value, data->p_service_uuid + (i * 16), 16);
    }
    fields.uuids128 = uuids128;
    fields.num_uuids128 = num;
    fields.uuids128_is_complete = 1;
  }

  int rc;
  if (data->set_scan_rsp) {
    rc = ble_gap_adv_rsp_set_fields(&fields);
  } else {
    rc = ble_gap_adv_set_fields(&fields);
  }
  if (rc != 0) {
    ESP_LOGE(TAG, "adv set fields failed: %d", rc);
  }
  return rc == 0 ? ESP_OK : ESP_FAIL;
}

}  // namespace esphome::esp32_ble::nimble_compat

esp_err_t esp_ble_gap_config_adv_data(esp_ble_adv_data_t *adv_data) {
  if (adv_data == nullptr)
    return ESP_ERR_INVALID_ARG;
  esp_err_t err = nc::apply_adv_fields(adv_data);
  esp_bt_status_t status = err == ESP_OK ? ESP_BT_STATUS_SUCCESS : ESP_BT_STATUS_FAIL;
  nc::post_gap_status(
      adv_data->set_scan_rsp ? ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT : ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT,
      status);
  return err;
}

esp_err_t esp_ble_gap_config_adv_data_raw(uint8_t *raw_data, uint32_t raw_data_len) {
  int rc = ble_gap_adv_set_data(raw_data, static_cast<int>(raw_data_len));
  nc::post_gap_status(ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT, rc == 0 ? ESP_BT_STATUS_SUCCESS : ESP_BT_STATUS_FAIL);
  return rc == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_ble_gap_start_advertising(esp_ble_adv_params_t *adv_params) {
  if (adv_params == nullptr)
    return ESP_ERR_INVALID_ARG;
  nc::adv_params_cache = *adv_params;
  nc::adv_params_valid = true;

  if (ble_gap_adv_active()) {
    ble_gap_adv_stop();
  }

  struct ble_gap_adv_params params = {};
  switch (adv_params->adv_type) {
    case ADV_TYPE_IND:
      params.conn_mode = BLE_GAP_CONN_MODE_UND;
      params.disc_mode = BLE_GAP_DISC_MODE_GEN;
      break;
    case ADV_TYPE_SCAN_IND:
    case ADV_TYPE_NONCONN_IND:
      params.conn_mode = BLE_GAP_CONN_MODE_NON;
      params.disc_mode = BLE_GAP_DISC_MODE_GEN;
      break;
    default:
      return ESP_ERR_NOT_SUPPORTED;
  }
  params.itvl_min = adv_params->adv_int_min;
  params.itvl_max = adv_params->adv_int_max;
  params.channel_map = adv_params->channel_map == ADV_CHNL_ALL ? 0 : adv_params->channel_map;
  params.filter_policy = adv_params->adv_filter_policy;

  int rc = ble_gap_adv_start(static_cast<uint8_t>(adv_params->own_addr_type), nullptr, BLE_HS_FOREVER, &params,
                             nc::adv_gap_event_cb, nullptr);
  nc::post_gap_status(ESP_GAP_BLE_ADV_START_COMPLETE_EVT, rc == 0 ? ESP_BT_STATUS_SUCCESS : ESP_BT_STATUS_FAIL);
  if (rc != 0) {
    ESP_LOGE(nc::TAG, "ble_gap_adv_start failed: %d", rc);
  }
  return rc == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_ble_gap_stop_advertising(void) {
  int rc = ble_gap_adv_stop();
  bool ok = rc == 0 || rc == BLE_HS_EALREADY;
  nc::post_gap_status(ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT, ok ? ESP_BT_STATUS_SUCCESS : ESP_BT_STATUS_FAIL);
  return ok ? ESP_OK : ESP_FAIL;
}

// ---------------------------------------------------------------------------
// Connection parameters / RSSI / security entry points
// ---------------------------------------------------------------------------

#ifdef ESPHOME_NIMBLE_HAS_CONNECT

esp_err_t esp_ble_gap_update_conn_params(esp_ble_conn_update_params_t *params) {
  if (params == nullptr)
    return ESP_ERR_INVALID_ARG;
  nc::ConnEntry *entry = nc::conn_by_addr(params->bda);
  if (entry == nullptr)
    return ESP_ERR_INVALID_STATE;
  struct ble_gap_upd_params upd = {};
  upd.itvl_min = params->min_int;
  upd.itvl_max = params->max_int;
  upd.latency = params->latency;
  upd.supervision_timeout = params->timeout;
  int rc = ble_gap_update_params(entry->conn_handle, &upd);
  return rc == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_ble_gap_read_rssi(esp_bd_addr_t remote_addr) {
  nc::ConnEntry *entry = nc::conn_by_addr(remote_addr);
  esp_ble_gap_cb_param_t param;
  memset(&param, 0, sizeof(param));
  memcpy(param.read_rssi_cmpl.remote_addr, remote_addr, 6);
  int8_t rssi = 0;
  if (entry == nullptr || ble_gap_conn_rssi(entry->conn_handle, &rssi) != 0) {
    param.read_rssi_cmpl.status = ESP_BT_STATUS_FAIL;
  } else {
    param.read_rssi_cmpl.status = ESP_BT_STATUS_SUCCESS;
    param.read_rssi_cmpl.rssi = rssi;
  }
  nc::post_gap(ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT, param);
  return ESP_OK;
}

esp_err_t esp_ble_set_encryption(esp_bd_addr_t bd_addr, esp_ble_sec_act_t sec_act) {
  nc::ConnEntry *entry = nc::conn_by_addr(bd_addr);
  if (entry == nullptr)
    return ESP_ERR_INVALID_STATE;
  int rc = ble_gap_security_initiate(entry->conn_handle);
  return (rc == 0 || rc == BLE_HS_EALREADY) ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_ble_gap_security_rsp(esp_bd_addr_t bd_addr, bool accept) {
  // NimBLE answers peer security requests internally; nothing to do.
  return ESP_OK;
}

esp_err_t esp_ble_passkey_reply(esp_bd_addr_t bd_addr, bool accept, uint32_t passkey) {
  nc::ConnEntry *entry = nc::conn_by_addr(bd_addr);
  if (entry == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (!accept) {
    // NimBLE has no reject path for a passkey request; failing the pairing by
    // terminating matches Bluedroid's observable outcome.
    ble_gap_terminate(entry->conn_handle, 0x05 /* authentication failure */);
    return ESP_OK;
  }
  struct ble_sm_io pkey = {};
  pkey.action = BLE_SM_IOACT_INPUT;
  pkey.passkey = passkey;
  int rc = ble_sm_inject_io(entry->conn_handle, &pkey);
  return rc == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_ble_confirm_reply(esp_bd_addr_t bd_addr, bool accept) {
  nc::ConnEntry *entry = nc::conn_by_addr(bd_addr);
  if (entry == nullptr)
    return ESP_ERR_INVALID_STATE;
  struct ble_sm_io pkey = {};
  pkey.action = BLE_SM_IOACT_NUMCMP;
  pkey.numcmp_accept = accept ? 1 : 0;
  int rc = ble_sm_inject_io(entry->conn_handle, &pkey);
  return rc == 0 ? ESP_OK : ESP_FAIL;
}

#endif  // ESPHOME_NIMBLE_HAS_CONNECT

esp_err_t esp_ble_remove_bond_device(esp_bd_addr_t bd_addr) {
  // Bluedroid keys bonds by address alone; try both identity address types.
  ble_addr_t addr;
  nc::addr_bd_to_ble(bd_addr, addr.val);
  addr.type = BLE_ADDR_PUBLIC;
  int rc_pub = ble_store_util_delete_peer(&addr);
  addr.type = BLE_ADDR_RANDOM;
  int rc_rnd = ble_store_util_delete_peer(&addr);
  return (rc_pub == 0 || rc_rnd == 0) ? ESP_OK : ESP_FAIL;
}

#endif  // USE_ESP32_BLE_NIMBLE
