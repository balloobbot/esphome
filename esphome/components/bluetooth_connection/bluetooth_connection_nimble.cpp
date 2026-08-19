#include "bluetooth_connection_nimble.h"

#if defined(USE_ESP32_BLE_NIMBLE) && defined(USE_BLE_GATT_CLIENT)

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <new>

extern "C" {
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <os/os_mbuf.h>
}

#ifndef USE_ESP32_BLE_CONNECTION_TIMEOUT_MS
#define USE_ESP32_BLE_CONNECTION_TIMEOUT_MS 20000
#endif

namespace esphome::bluetooth_connection {

static const char *const TAG = "bluetooth_connection.nimble";

using ble_device_base::ClientState;
using ble_device_base::ConnectionType;
using ble_device_base::GattServiceTable;

// NimBLE addresses are LSB-first; the neutral/tracker surface is MSB-first.
static void bda_to_ble_addr(const esp_bd_addr_t bda, uint8_t addr_type, ble_addr_t &out) {
  out.type = addr_type;
  for (int i = 0; i < 6; i++)
    out.val[i] = bda[5 - i];
}

// NimBLE rc -> neutral error domain: ATT codes pass through, core BLE_HS
// codes are shifted out of the ATT window so they stay opaque platform codes
// (never 0/-1/-2 or a fake ATT code by accident).
static int nimble_to_neutral(int rc) {
  if (rc == 0)
    return 0;
  if (rc == BLE_HS_ENOTCONN)
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  if (rc == BLE_HS_ENOMEM)
    return ble_device_base::GATT_ERR_NO_MEMORY;
  if (rc >= BLE_HS_ERR_ATT_BASE && rc < BLE_HS_ERR_ATT_BASE + 0x100)
    return rc - BLE_HS_ERR_ATT_BASE;
  return 0x1000 + rc;
}

static ble_device_base::ESPBTUUID uuid_from_nimble(const ble_uuid_any_t &uuid) {
  switch (uuid.u.type) {
    case BLE_UUID_TYPE_16:
      return ble_device_base::ESPBTUUID::from_uint16(uuid.u16.value);
    case BLE_UUID_TYPE_32:
      return ble_device_base::ESPBTUUID::from_uint32(uuid.u32.value);
    default:
      // NimBLE stores 128-bit UUIDs little-endian, the layout from_raw expects.
      return ble_device_base::ESPBTUUID::from_raw(uuid.u128.value);
  }
}

static int disconnect_reason(int hs_reason) {
  if (hs_reason >= BLE_HS_ERR_HCI_BASE && hs_reason < BLE_HS_ERR_HCI_BASE + 0x100)
    return hs_reason - BLE_HS_ERR_HCI_BASE;
  return hs_reason;
}

// ---------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------

void NimbleGattClient::setup() {
  // Pre-create the event pool so the host-task producers never allocate.
  this->event_pool_.warm();
  // Nothing to register with the stack (no Bluedroid-style app); the slot
  // idles until the first contract connect().
  this->set_idle_();
  this->disable_loop();
}

void NimbleGattClient::loop() {
  // Teardown safety net: a lost disconnect completion must not wedge the slot.
  if (this->state() == ClientState::DISCONNECTING &&
      millis() - this->disconnecting_started_ > ble_device_base::GATT_DISCONNECT_TIMEOUT_MS) {
    ESP_LOGE(TAG, "Timeout waiting for teardown, forcing IDLE");
    this->release_services();
    this->set_idle_();  // also clears want_disconnect_
    this->listener_->on_connection_state(false, 0, ble_device_base::GATT_ERR_NOT_CONNECTED);
  }

  GattEvent *event;
  while ((event = this->events_.pop()) != nullptr) {
    this->drain_event_(event);
    this->event_pool_.release(event);
  }
  if (this->state() == ClientState::IDLE) {
    this->disable_loop();
  }
}

void NimbleGattClient::dump_config() { ESP_LOGCONFIG(TAG, "NimBLE GATT client"); }

// ---------------------------------------------------------------------------
// Host-task producers
// ---------------------------------------------------------------------------

void NimbleGattClient::enqueue_(GattEvent::Type type, uint16_t handle, int32_t status, uint16_t len) {
  GattEvent *event = this->event_pool_.allocate();
  if (event == nullptr) {
    // A dropped control event would wedge the consumer's state machine; the
    // teardown watchdog is the recovery path, so make the drop loud.
    ESP_LOGE(TAG, "Event pool exhausted, dropping event %d", static_cast<int>(type));
    return;
  }
  event->type = type;
  event->heap_data = false;
  event->handle = handle;
  event->status = status;
  event->len = len;
  this->events_.push(event);
  this->enable_loop_soon_any_context();
}

void NimbleGattClient::enqueue_notify_(uint16_t handle, const uint8_t *data, uint16_t len) {
  GattEvent *event = this->event_pool_.allocate();
  if (event == nullptr) {
    ESP_LOGW(TAG, "Event pool exhausted, dropping notification");  // lossy by design
    return;
  }
  event->type = GattEvent::Type::NOTIFY_DATA;
  event->handle = handle;
  event->status = 0;
  event->len = len;
  if (len <= GattEvent::INLINE_DATA_SIZE) {
    event->heap_data = false;
    memcpy(event->payload.inline_data, data, len);
  } else {
    event->payload.heap = new (std::nothrow) uint8_t[len];  // NOLINT(cppcoreguidelines-owning-memory)
    if (event->payload.heap == nullptr) {
      event->heap_data = false;
      this->event_pool_.release(event);
      return;
    }
    event->heap_data = true;
    memcpy(event->payload.heap, data, len);
  }
  this->events_.push(event);
  this->enable_loop_soon_any_context();
}

int NimbleGattClient::gap_event_cb_s(struct ble_gap_event *event, void *arg) {
  return static_cast<NimbleGattClient *>(arg)->gap_event_(event);
}

int NimbleGattClient::gap_event_(struct ble_gap_event *event) {
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status != 0) {
        this->conn_handle_ = UNSET_CONN_HANDLE;
        this->enqueue_(GattEvent::Type::CONNECT_FAILED, 0, event->connect.status, 0);
        return 0;
      }
      // Stamp in the host task so a racing disconnect routes correctly.
      this->conn_handle_ = event->connect.conn_handle;
      // The connected report waits for the MTU exchange; a refused request or
      // failed exchange reports the default instead (mtu_cb_s handles the
      // async failure - BLE_GAP_EVENT_MTU only fires on success).
      if (ble_gattc_exchange_mtu(event->connect.conn_handle, mtu_cb_s, this) != 0) {
        this->enqueue_(GattEvent::Type::CONNECTED, 0, 0, ble_device_base::DEFAULT_ATT_MTU);
      }
      return 0;
    case BLE_GAP_EVENT_MTU:
      this->enqueue_(GattEvent::Type::CONNECTED, 0, 0, event->mtu.value);
      return 0;
    case BLE_GAP_EVENT_DISCONNECT:
      this->conn_handle_ = UNSET_CONN_HANDLE;
      this->enqueue_(GattEvent::Type::DISCONNECTED, 0, disconnect_reason(event->disconnect.reason), 0);
      return 0;
    case BLE_GAP_EVENT_NOTIFY_RX: {
      uint16_t handle = event->notify_rx.attr_handle;
      if (!this->subscribed_(handle))
        return 0;
      uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
      // Flatten into a stack buffer bounded by the maximum ATT payload.
      uint8_t flat[517];
      if (len > sizeof(flat))
        len = sizeof(flat);
      os_mbuf_copydata(event->notify_rx.om, 0, len, flat);
      this->enqueue_notify_(handle, flat, len);
      return 0;
    }
    case BLE_GAP_EVENT_ENC_CHANGE:
      this->enqueue_(GattEvent::Type::PAIRING_RESULT, 0, nimble_to_neutral(event->enc_change.status), 0);
      return 0;
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
      // Peer lost its bond; drop ours and let the pairing retry.
      ble_gap_conn_desc desc;
      if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
        ble_store_util_delete_peer(&desc.peer_id_addr);
      }
      return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    default:
      return 0;
  }
}

// ---- discovery (host task) ----

int NimbleGattClient::svc_disc_cb_s(uint16_t conn_handle, const struct ble_gatt_error *error,
                                    const struct ble_gatt_svc *svc, void *arg) {
  auto *self = static_cast<NimbleGattClient *>(arg);
  ServiceArena *arena = self->arena_;
  if (arena == nullptr || conn_handle != self->conn_handle_)
    return 0;
  if (error->status == 0 && svc != nullptr) {
    if (arena->service_count >= MAX_SERVICES) {
      arena->truncated = true;
      return 0;
    }
    auto &out = arena->services[arena->service_count++];
    out.uuid = uuid_from_nimble(svc->uuid);
    out.start_handle = svc->start_handle;
    out.end_handle = svc->end_handle;
    out.first_characteristic = 0;
    out.characteristic_count = 0;
    return 0;
  }
  if (error->status == BLE_HS_EDONE) {
    self->disc_svc_index_ = 0;
    self->advance_chr_discovery_();
    return 0;
  }
  self->finish_discovery_(nimble_to_neutral(error->status));
  return 0;
}

// Start characteristic discovery for the next service with a non-empty range;
// moves on to descriptors after the last service.
void NimbleGattClient::advance_chr_discovery_() {
  ServiceArena *arena = this->arena_;
  this->disc_phase_.store(DiscoveryPhase::CHARACTERISTICS, std::memory_order_release);
  while (this->disc_svc_index_ < arena->service_count) {
    auto &svc = arena->services[this->disc_svc_index_];
    svc.first_characteristic = arena->characteristic_count;
    int rc = ble_gattc_disc_all_chrs(this->conn_handle_, svc.start_handle, svc.end_handle, chr_disc_cb_s, this);
    if (rc == 0)
      return;
    this->finish_discovery_(nimble_to_neutral(rc));
    return;
  }
  this->disc_chr_index_ = 0;
  this->disc_phase_.store(DiscoveryPhase::DESCRIPTORS, std::memory_order_release);
  this->advance_dsc_discovery_();
}

int NimbleGattClient::chr_disc_cb_s(uint16_t conn_handle, const struct ble_gatt_error *error,
                                    const struct ble_gatt_chr *chr, void *arg) {
  auto *self = static_cast<NimbleGattClient *>(arg);
  ServiceArena *arena = self->arena_;
  if (arena == nullptr || conn_handle != self->conn_handle_)
    return 0;
  auto &svc = arena->services[self->disc_svc_index_];
  if (error->status == 0 && chr != nullptr) {
    if (arena->characteristic_count >= MAX_CHARACTERISTICS) {
      arena->truncated = true;
      return 0;
    }
    uint16_t index = arena->characteristic_count++;
    // The previous characteristic in this service ends where this one's
    // declaration begins.
    if (index > svc.first_characteristic) {
      arena->characteristics[index - 1].end_handle = chr->def_handle - 1;
    }
    auto &out = arena->characteristics[index];
    out.uuid = uuid_from_nimble(chr->uuid);
    out.value_handle = chr->val_handle;
    out.end_handle = svc.end_handle;  // patched by the next characteristic
    out.properties = chr->properties;
    out.first_descriptor = 0;
    out.descriptor_count = 0;
    svc.characteristic_count++;
    return 0;
  }
  if (error->status == BLE_HS_EDONE) {
    self->disc_svc_index_++;
    self->advance_chr_discovery_();
    return 0;
  }
  self->finish_discovery_(nimble_to_neutral(error->status));
  return 0;
}

// Start descriptor discovery for the next characteristic with a non-empty
// range; completes the discovery after the last one.
void NimbleGattClient::advance_dsc_discovery_() {
  ServiceArena *arena = this->arena_;
  while (this->disc_chr_index_ < arena->characteristic_count) {
    auto &chr = arena->characteristics[this->disc_chr_index_];
    chr.first_descriptor = arena->descriptor_count;
    if (chr.end_handle <= chr.value_handle) {
      this->disc_chr_index_++;
      continue;
    }
    int rc = ble_gattc_disc_all_dscs(this->conn_handle_, chr.value_handle, chr.end_handle, dsc_disc_cb_s, this);
    if (rc == 0)
      return;
    this->finish_discovery_(nimble_to_neutral(rc));
    return;
  }
  this->finish_discovery_(arena->truncated ? 0x11 /* ATT insufficient resources */ : 0);
}

int NimbleGattClient::dsc_disc_cb_s(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t chr_val_handle,
                                    const struct ble_gatt_dsc *dsc, void *arg) {
  auto *self = static_cast<NimbleGattClient *>(arg);
  ServiceArena *arena = self->arena_;
  if (arena == nullptr || conn_handle != self->conn_handle_)
    return 0;
  if (error->status == 0 && dsc != nullptr) {
    if (arena->descriptor_count >= MAX_DESCRIPTORS) {
      arena->truncated = true;
      return 0;
    }
    auto &out = arena->descriptors[arena->descriptor_count++];
    out.uuid = uuid_from_nimble(dsc->uuid);
    out.handle = dsc->handle;
    arena->characteristics[self->disc_chr_index_].descriptor_count++;
    return 0;
  }
  if (error->status == BLE_HS_EDONE || error->status == BLE_HS_ATT_ERR(0x0A)) {
    // Done with this characteristic (ATT attribute-not-found = empty range).
    self->disc_chr_index_++;
    self->advance_dsc_discovery_();
    return 0;
  }
  self->finish_discovery_(nimble_to_neutral(error->status));
  return 0;
}

void NimbleGattClient::finish_discovery_(int status) {
  this->disc_phase_.store(DiscoveryPhase::NONE, std::memory_order_release);
  if (status == 0 && this->connection_type_ == ConnectionType::V3_WITHOUT_CACHE) {
    // Discovery ran on FAST params; settle to MEDIUM for steady state.
    struct ble_gap_upd_params params = {};
    params.itvl_min = ble_device_base::MEDIUM_MIN_CONN_INTERVAL;
    params.itvl_max = ble_device_base::MEDIUM_MAX_CONN_INTERVAL;
    params.latency = 0;
    params.supervision_timeout = ble_device_base::MEDIUM_CONN_TIMEOUT;
    ble_gap_update_params(this->conn_handle_, &params);
  }
  this->enqueue_(GattEvent::Type::DISCOVERY_DONE, 0, status, 0);
}

// ---- read/write completions (host task) ----

int NimbleGattClient::mtu_cb_s(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t mtu, void *arg) {
  auto *self = static_cast<NimbleGattClient *>(arg);
  if (conn_handle != self->conn_handle_)
    return 0;
  if (error->status != 0) {
    // Peer rejected the exchange (or it timed out): proceed on the default,
    // matching the Bluedroid backend. The success case reports via
    // BLE_GAP_EVENT_MTU; duplicates are suppressed at the drain.
    self->enqueue_(GattEvent::Type::CONNECTED, 0, 0, ble_device_base::DEFAULT_ATT_MTU);
  }
  return 0;
}

int NimbleGattClient::read_cb_s(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr,
                                void *arg) {
  auto *self = static_cast<NimbleGattClient *>(arg);
  if (conn_handle != self->conn_handle_)
    return 0;
  if (error->status == 0 && attr != nullptr) {
    // Chunk of a (possibly long) read; accumulate.
    if (!self->op_read_active_) {
      self->op_read_active_ = true;
      self->op_len_ = 0;
    }
    uint16_t chunk = OS_MBUF_PKTLEN(attr->om);
    if (self->op_len_ + chunk > 517)
      chunk = 517 - self->op_len_;
    os_mbuf_copydata(attr->om, 0, chunk, self->op_buffer_.get() + self->op_len_);
    self->op_len_ += chunk;
    self->op_handle_ = attr->handle;
    return 0;
  }
  // A value that is an exact multiple of MTU-1 makes the trailing read-blob
  // probe fail with INVALID_OFFSET / ATTRIBUTE_NOT_LONG; with data already
  // accumulated that is a completed read (rp2 backend parity).
  bool ok = error->status == BLE_HS_EDONE;
  if (!ok && self->op_read_active_ && self->op_len_ > 0 &&
      (error->status == BLE_HS_ATT_ERR(0x07) || error->status == BLE_HS_ATT_ERR(0x0B))) {
    ok = true;
  }
  ok = ok && self->op_read_active_;
  self->op_read_active_ = false;
  if (ok) {
    self->enqueue_(GattEvent::Type::READ_RESULT, self->op_handle_, 0, self->op_len_);
  } else {
    // error->att_handle is 0 on timeouts; use the requested handle so the
    // consumer's per-handle bookkeeping resolves.
    self->enqueue_(GattEvent::Type::READ_RESULT, self->op_req_handle_,
                   nimble_to_neutral(error->status == BLE_HS_EDONE ? BLE_HS_EUNKNOWN : error->status), 0);
  }
  return 0;
}

int NimbleGattClient::write_cb_s(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr,
                                 void *arg) {
  auto *self = static_cast<NimbleGattClient *>(arg);
  if (conn_handle != self->conn_handle_)
    return 0;
  uint16_t handle = attr != nullptr ? attr->handle : error->att_handle;
  self->enqueue_(GattEvent::Type::WRITE_RESULT, handle, nimble_to_neutral(error->status), 0);
  return 0;
}

// ---------------------------------------------------------------------------
// Main-loop event drain
// ---------------------------------------------------------------------------

void NimbleGattClient::drain_event_(GattEvent *event) {
  switch (event->type) {
    case GattEvent::Type::CONNECTED: {
      if (this->state() == ClientState::IDLE) {
        // Late completion on a freed slot: close the won link.
        this->unconditional_disconnect_();
        return;
      }
      if (this->state() == ClientState::DISCONNECTING) {
        // Teardown owns the link; close the won race now (terminal comes
        // from the resulting DISCONNECTED).
        this->unconditional_disconnect_();
        return;
      }
      if (this->mtu_reported_)
        return;  // duplicate MTU report
      this->mtu_reported_ = true;
      this->set_state(ClientState::ESTABLISHED);
      ESP_LOGI(TAG, "Connected (mtu %u)", event->len);
      this->listener_->on_connection_state(true, event->len, 0);
      return;
    }
    case GattEvent::Type::CONNECT_FAILED:
      ESP_LOGW(TAG, "Connect failed: %d", static_cast<int>(event->status));
      this->set_idle_();
      this->listener_->on_connection_state(false, 0, nimble_to_neutral(event->status));
      return;
    case GattEvent::Type::DISCONNECTED: {
      auto st = this->state();
      if (st == ClientState::IDLE || st == ClientState::DISCOVERED)
        return;  // stale completion on a settled or re-parked slot
      ESP_LOGI(TAG, "Disconnected (reason 0x%02X)", static_cast<unsigned>(event->status));
      this->release_services();
      this->set_idle_();
      this->listener_->on_connection_state(false, 0, event->status);
      return;
    }
    case GattEvent::Type::DISCOVERY_DONE:
      this->listener_->on_service_discovery_done(event->status);
      if (this->release_pending_) {
        this->release_pending_ = false;
        this->release_services();
      }
      return;
    case GattEvent::Type::READ_RESULT:
      if (event->status == 0) {
        this->listener_->on_read_result(event->handle, this->op_buffer_.get(), event->len, 0);
      } else {
        this->listener_->on_read_result(event->handle, nullptr, 0, event->status);
      }
      return;
    case GattEvent::Type::WRITE_RESULT:
      this->listener_->on_write_result(event->handle, event->status);
      return;
    case GattEvent::Type::PAIRING_RESULT:
      this->listener_->on_pairing_result(event->status);
      return;
    case GattEvent::Type::NOTIFY_DATA:
      this->listener_->on_notify_data(event->handle, event->data(), event->len);
      return;
  }
}

// ---------------------------------------------------------------------------
// Tracker surface
// ---------------------------------------------------------------------------

void NimbleGattClient::connect() { this->native_connect_(); }

void NimbleGattClient::disconnect() { this->gatt_disconnect(); }

void NimbleGattClient::native_connect_() {
  auto st = this->state();
  if (st != ClientState::DISCOVERED) {
    ESP_LOGW(TAG, "Connect ignored in state %d", static_cast<int>(st));
    return;
  }
  ESP_LOGI(TAG, "0x%02x Connecting", this->remote_addr_type_);
  this->mtu_reported_ = false;
  this->enable_loop();
  this->set_state(ClientState::CONNECTING);

  ble_addr_t peer;
  bda_to_ble_addr(this->remote_bda_, this->remote_addr_type_, peer);
  struct ble_gap_conn_params params = {};
  params.scan_itvl = 0x0010;
  params.scan_window = 0x0010;
  if (this->connection_type_ == ConnectionType::V3_WITHOUT_CACHE) {
    params.itvl_min = ble_device_base::FAST_MIN_CONN_INTERVAL;
    params.itvl_max = ble_device_base::FAST_MAX_CONN_INTERVAL;
    params.supervision_timeout = ble_device_base::FAST_CONN_TIMEOUT;
  } else {
    params.itvl_min = ble_device_base::MEDIUM_MIN_CONN_INTERVAL;
    params.itvl_max = ble_device_base::MEDIUM_MAX_CONN_INTERVAL;
    params.supervision_timeout = ble_device_base::MEDIUM_CONN_TIMEOUT;
  }
  int rc = ble_gap_connect(0 /* public */, &peer, USE_ESP32_BLE_CONNECTION_TIMEOUT_MS, &params, gap_event_cb_s, this);
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_gap_connect failed: %d", rc);
    this->set_idle_();
    this->listener_->on_connection_state(false, 0, nimble_to_neutral(rc));
  }
}

// ---------------------------------------------------------------------------
// Contract ops (main loop)
// ---------------------------------------------------------------------------

int NimbleGattClient::connect(uint64_t address, uint8_t addr_type) {
  if (this->state() != ClientState::IDLE) {
    ESP_LOGW(TAG, "Connect rejected, slot busy");
    return 0x84;  // GATT busy
  }
  ble_device_base::uint64_to_mac_msb_first(address, this->remote_bda_);
  this->remote_addr_type_ = addr_type;
  // Hand the request to the tracker's promote loop: it stops the scan and
  // calls the connect() override - the tracker owns connect timing here.
  this->set_state(ClientState::DISCOVERED);
  return 0;
}

int NimbleGattClient::gatt_disconnect() {
  auto st = this->state();
  if (st == ClientState::DISCONNECTING)
    return 0;
  if (st == ClientState::IDLE)
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  if (st == ClientState::DISCOVERED) {
    // Parked for the tracker promote loop, never opened.
    this->set_idle_();
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  }
  if (st == ClientState::CONNECTING) {
    // Cancel the in-flight connect: the completion arrives as a CONNECT
    // event with a nonzero status and provides the terminal report. A won
    // race (already connected) resolves at the CONNECTED drain, which sees
    // DISCONNECTING and closes the link.
    ESP_LOGD(TAG, "Cancelling in-flight connect");
    ble_gap_conn_cancel();
    this->set_disconnecting_();
    return 0;
  }
  if (this->conn_handle_ == UNSET_CONN_HANDLE) {
    // Connected but the handle is already gone: the peer dropped the link
    // and its DISCONNECTED event is queued (or lost - the watchdog covers
    // that). Never report synchronously from here: the listener re-enters
    // the hub, which is still inside its own disconnect() call.
    this->set_disconnecting_();
    return 0;
  }
  this->unconditional_disconnect_();
  return 0;
}

void NimbleGattClient::unconditional_disconnect_() {
  if (this->conn_handle_ == UNSET_CONN_HANDLE) {
    // Nothing to close; let the queued completion or the watchdog settle it.
    this->set_disconnecting_();
    return;
  }
  ESP_LOGI(TAG, "Disconnecting (conn %u)", this->conn_handle_);
  int rc = ble_gap_terminate(this->conn_handle_, 0x13 /* remote user terminated */);
  if (rc != 0 && rc != BLE_HS_EALREADY && rc != BLE_HS_ENOTCONN) {
    ESP_LOGE(TAG, "ble_gap_terminate failed: %d", rc);
  }
  this->set_disconnecting_();
}

bool NimbleGattClient::cancel_gatt_disconnect() {
  // Teardown starts closing immediately (conn_cancel/terminate), so there is
  // never a cancellable window; the wrapper reconnects after the terminal.
  return false;
}

int NimbleGattClient::discover_services() {
  if (this->conn_handle_ == UNSET_CONN_HANDLE)
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  if (this->disc_phase_.load(std::memory_order_acquire) != DiscoveryPhase::NONE)
    return 0x84;  // GATT busy: one discovery at a time
  this->release_pending_ = false;
  if (this->arena_ == nullptr) {
    RAMAllocator<ServiceArena> allocator;
    this->arena_ = allocator.allocate(1);
    if (this->arena_ == nullptr)
      return ble_device_base::GATT_ERR_NO_MEMORY;
    new (this->arena_) ServiceArena();
  } else {
    this->arena_->~ServiceArena();
    new (this->arena_) ServiceArena();
  }
  if (this->op_buffer_ == nullptr) {
    this->op_buffer_ = std::make_unique<uint8_t[]>(517);
  }
  this->disc_phase_.store(DiscoveryPhase::SERVICES, std::memory_order_release);
  int rc = ble_gattc_disc_all_svcs(this->conn_handle_, svc_disc_cb_s, this);
  if (rc != 0)
    this->disc_phase_.store(DiscoveryPhase::NONE, std::memory_order_release);
  return this->map_rc_("disc_all_svcs", rc);
}

GattServiceTable NimbleGattClient::get_service_table() {
  if (this->arena_ == nullptr)
    return {};
  return GattServiceTable{
      this->arena_->services,      this->arena_->characteristics,      this->arena_->descriptors,
      this->arena_->service_count, this->arena_->characteristic_count, this->arena_->descriptor_count};
}

void NimbleGattClient::release_services() {
  if (this->arena_ == nullptr)
    return;
  if (this->disc_phase_.load(std::memory_order_acquire) != DiscoveryPhase::NONE) {
    // Discovery callbacks on the host task still write the arena; defer the
    // free to the DISCOVERY_DONE drain.
    this->release_pending_ = true;
    return;
  }
  this->arena_->~ServiceArena();
  RAMAllocator<ServiceArena> allocator;
  allocator.deallocate(this->arena_, 1);
  this->arena_ = nullptr;
}

int NimbleGattClient::read_characteristic(uint16_t handle) {
  if (this->conn_handle_ == UNSET_CONN_HANDLE)
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  if (this->op_buffer_ == nullptr) {
    this->op_buffer_ = std::make_unique<uint8_t[]>(517);
  }
  this->op_read_active_ = false;
  this->op_req_handle_ = handle;
  return this->map_rc_("read", ble_gattc_read_long(this->conn_handle_, handle, 0, read_cb_s, this));
}

int NimbleGattClient::read_descriptor(uint16_t handle) { return this->read_characteristic(handle); }

int NimbleGattClient::write_characteristic(uint16_t handle, const uint8_t *data, uint16_t len, bool response) {
  if (this->conn_handle_ == UNSET_CONN_HANDLE)
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  if (!response) {
    int rc = this->map_rc_("write_no_rsp", ble_gattc_write_no_rsp_flat(this->conn_handle_, handle, data, len));
    if (rc == 0) {
      // The neutral contract reports unacknowledged writes too (rp2 model);
      // delivered during the op call, which the contract permits.
      this->listener_->on_write_result(handle, 0);
    }
    return rc;
  }
  return this->map_rc_("write", ble_gattc_write_flat(this->conn_handle_, handle, data, len, write_cb_s, this));
}

int NimbleGattClient::write_descriptor(uint16_t handle, const uint8_t *data, uint16_t len) {
  if (this->conn_handle_ == UNSET_CONN_HANDLE)
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  // Descriptor writes are always acknowledged in the neutral contract.
  return this->map_rc_("write_dsc", ble_gattc_write_flat(this->conn_handle_, handle, data, len, write_cb_s, this));
}

int NimbleGattClient::notify_characteristic(uint16_t handle, bool enable) {
  // Local registration only; the CCCD write is the API client's job. NimBLE
  // delivers all notifications, so the list filters delivery.
  if (enable) {
    for (uint16_t &slot : this->notify_subscriptions_) {
      if (slot == handle) {
        this->listener_->on_notify_state(handle, true, 0);
        return 0;
      }
    }
    for (uint16_t &slot : this->notify_subscriptions_) {
      if (slot == 0) {
        slot = handle;
        this->listener_->on_notify_state(handle, true, 0);
        return 0;
      }
    }
    return ble_device_base::GATT_ERR_NO_MEMORY;
  }
  for (uint16_t &slot : this->notify_subscriptions_) {
    if (slot == handle)
      slot = 0;
  }
  this->listener_->on_notify_state(handle, false, 0);
  return 0;
}

bool NimbleGattClient::subscribed_(uint16_t handle) const {
  for (uint16_t slot : this->notify_subscriptions_) {
    if (slot == handle)
      return true;
  }
  return false;
}

int NimbleGattClient::pair() {
  if (this->conn_handle_ == UNSET_CONN_HANDLE)
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  int rc = ble_gap_security_initiate(this->conn_handle_);
  if (rc == BLE_HS_EALREADY)
    return 0;
  return this->map_rc_("security_initiate", rc);
}

int NimbleGattClient::update_connection_params(uint16_t min_interval, uint16_t max_interval, uint16_t latency,
                                               uint16_t timeout) {
  if (this->conn_handle_ == UNSET_CONN_HANDLE)
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  struct ble_gap_upd_params params = {};
  params.itvl_min = min_interval;
  params.itvl_max = max_interval;
  params.latency = latency;
  params.supervision_timeout = timeout;
  return this->map_rc_("update_params", ble_gap_update_params(this->conn_handle_, &params));
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void NimbleGattClient::set_idle_() {
  this->set_state(ClientState::IDLE);
  this->conn_handle_ = UNSET_CONN_HANDLE;
  this->disc_phase_.store(DiscoveryPhase::NONE, std::memory_order_release);
  this->op_read_active_ = false;
  for (uint16_t &slot : this->notify_subscriptions_)
    slot = 0;
}

void NimbleGattClient::set_disconnecting_() {
  this->disconnecting_started_ = millis();
  this->set_state(ClientState::DISCONNECTING);
  this->enable_loop();
}

int NimbleGattClient::map_rc_(const char *operation, int rc) {
  if (rc == 0)
    return 0;
  ESP_LOGW(TAG, "%s failed: %d", operation, rc);
  if (rc == BLE_HS_ENOTCONN)
    return ble_device_base::GATT_ERR_NOT_CONNECTED;
  return nimble_to_neutral(rc);
}

}  // namespace esphome::bluetooth_connection

#endif  // USE_ESP32_BLE_NIMBLE && USE_BLE_GATT_CLIENT
