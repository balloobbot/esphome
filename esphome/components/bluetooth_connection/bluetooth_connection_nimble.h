// NimBLE (esp32, stack: nimble) GATT client backend: the native arm of the
// ble_device_base::BLEGattConnection alias. Unlike the legacy paths it does
// not go through the Bluedroid-compatible facade — every operation calls the
// NimBLE host directly and completions are translated straight into the
// neutral listener callbacks.
//
// Like the Bluedroid backend it carries the tracker's ESPBTClient surface:
// the tracker promote loop owns scan-stop/coex/one-connect-at-a-time, so the
// contract's connect() only parks the address in DISCOVERED and the real
// ble_gap_connect happens in the tracker-invoked connect() override.
//
// Threading: NimBLE callbacks run in the host task. They only copy-and-
// enqueue into a lock-free queue (and write the discovery arena, which the
// main loop reads only after the completion event); loop() drains the queue
// and is the only place the listener is invoked, per the neutral contract.

#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP32_BLE_NIMBLE) && defined(USE_BLE_GATT_CLIENT)

#include "bluetooth_connection.h"

#include "esphome/components/ble_device_base/ble_gatt_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/core/component.h"
#include "esphome/core/event_pool.h"
#include "esphome/core/helpers.h"
#include "esphome/core/lock_free_queue.h"

#include <atomic>

struct ble_gap_event;
struct ble_gatt_error;
struct ble_gatt_svc;
struct ble_gatt_chr;
struct ble_gatt_dsc;
struct ble_gatt_attr;

namespace esphome::bluetooth_connection {

class NimbleGattClient final : public esp32_ble_tracker::ESPBTClient, public Component {
 public:
  static constexpr uint16_t UNSET_CONN_HANDLE = 0xFFFF;
  // Discovery arena caps; a peer exceeding them fails discovery with the ATT
  // insufficient-resources code instead of streaming a partial table.
  static constexpr uint16_t MAX_SERVICES = 32;
  static constexpr uint16_t MAX_CHARACTERISTICS = 128;
  static constexpr uint16_t MAX_DESCRIPTORS = 128;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_BLUETOOTH; }

  // Wired by codegen before setup and invariant for the device lifetime.
  void set_listener(ble_device_base::GattClientListener *listener) { this->listener_ = listener; }

  // ---- esp32_ble_tracker::ESPBTClient ----
  bool gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override {
    return false;  // native backend; the facade's GATTC events are not consumed
  }
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override {}
  void connect() override;
  void disconnect() override;
  bool wants_parsed_advertisements() override { return false; }
  void on_scan_end() override {}
  bool parse_device(const ble_device_base::ESPBTDevice &device) override { return false; }

  // ---- ble_device_base::BLEGattConnection contract ----
  int connect(uint64_t address, uint8_t addr_type);
  int gatt_disconnect();
  bool cancel_gatt_disconnect();
  int discover_services();
  int read_characteristic(uint16_t handle);
  int write_characteristic(uint16_t handle, const uint8_t *data, uint16_t len, bool response);
  int read_descriptor(uint16_t handle);
  int write_descriptor(uint16_t handle, const uint8_t *data, uint16_t len);
  int notify_characteristic(uint16_t handle, bool enable);
  int pair();
  int update_connection_params(uint16_t min_interval, uint16_t max_interval, uint16_t latency, uint16_t timeout);
  ble_device_base::GattServiceTable get_service_table();
  void release_services();
  void set_connection_type(ble_device_base::ConnectionType ct) { this->connection_type_ = ct; }

 protected:
  // One completion, copied out of the host task. NOTIFY payloads larger than
  // the inline buffer go to the heap (rare; freed on drain), the same
  // inline-or-heap pattern the core BLEEvent uses.
  struct GattEvent {
    enum class Type : uint8_t {
      CONNECTED,       // len = negotiated MTU
      CONNECT_FAILED,  // status = NimBLE rc
      DISCONNECTED,    // status = HCI reason
      DISCOVERY_DONE,  // status = 0 or ATT error
      READ_RESULT,     // payload in op_buffer_, len bytes
      WRITE_RESULT,
      PAIRING_RESULT,
      NOTIFY_DATA,
    };
    static constexpr uint16_t INLINE_DATA_SIZE = 64;

    Type type;
    bool heap_data;
    uint16_t handle;
    uint16_t len;
    int32_t status;
    union {
      uint8_t inline_data[INLINE_DATA_SIZE];
      uint8_t *heap;
    } payload;

    const uint8_t *data() const { return this->heap_data ? this->payload.heap : this->payload.inline_data; }
    // EventPool contract: free owned resources when the pool reclaims us.
    void release() {
      if (this->heap_data) {
        delete[] this->payload.heap;  // NOLINT(cppcoreguidelines-owning-memory)
        this->heap_data = false;
      }
    }
  };

  // Discovery arena: flat neutral-table storage, allocated per discovery and
  // freed in release_services(). Written only by the host task during
  // discovery; the main loop reads it only after DISCOVERY_DONE.
  struct ServiceArena {
    ble_device_base::GattService services[MAX_SERVICES];
    ble_device_base::GattCharacteristic characteristics[MAX_CHARACTERISTICS];
    ble_device_base::GattDescriptor descriptors[MAX_DESCRIPTORS];
    uint16_t service_count{0};
    uint16_t characteristic_count{0};
    uint16_t descriptor_count{0};
    bool truncated{false};
  };

  enum class DiscoveryPhase : uint8_t { NONE, SERVICES, CHARACTERISTICS, DESCRIPTORS };

  // ---- NimBLE callbacks (host task) ----
  static int gap_event_cb_s(struct ble_gap_event *event, void *arg);
  int gap_event_(struct ble_gap_event *event);
  static int svc_disc_cb_s(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc,
                           void *arg);
  static int chr_disc_cb_s(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr,
                           void *arg);
  static int dsc_disc_cb_s(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t chr_val_handle,
                           const struct ble_gatt_dsc *dsc, void *arg);
  static int read_cb_s(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg);
  static int mtu_cb_s(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t mtu, void *arg);
  static int write_cb_s(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr,
                        void *arg);

  // ---- host-task helpers ----
  void enqueue_(GattEvent::Type type, uint16_t handle, int32_t status, uint16_t len);
  void enqueue_notify_(uint16_t handle, const uint8_t *data, uint16_t len);
  void advance_chr_discovery_();
  void advance_dsc_discovery_();
  void finish_discovery_(int status);

  // ---- main-loop helpers ----
  void drain_event_(GattEvent *event);
  void native_connect_();
  void unconditional_disconnect_();
  void set_idle_();
  void set_disconnecting_();
  int map_rc_(const char *operation, int rc);
  bool subscribed_(uint16_t handle) const;

  // Group 1: pointers / composed objects
  ble_device_base::GattClientListener *listener_{nullptr};
  ServiceArena *arena_{nullptr};
  esphome::LockFreeQueue<GattEvent, 9> events_;
  esphome::EventPool<GattEvent, 8> event_pool_;
  std::unique_ptr<uint8_t[]> op_buffer_;  // single outstanding read op

  // Group 2: 4-byte types
  uint32_t disconnecting_started_{0};

  // Group 3: 2-byte types
  uint16_t conn_handle_{UNSET_CONN_HANDLE};
  uint16_t op_len_{0};
  uint16_t op_handle_{0};
  uint16_t op_req_handle_{0};
  uint16_t notify_subscriptions_[16]{};
  // Descriptor-discovery cursor (index into arena characteristics)
  uint16_t disc_chr_index_{0};

  // Group 4: 1-byte types
  uint8_t remote_addr_type_{0};
  esp_bd_addr_t remote_bda_{};  // MSB-first, converted at the NimBLE boundary
  ble_device_base::ConnectionType connection_type_{ble_device_base::ConnectionType::V3_WITHOUT_CACHE};
  std::atomic<DiscoveryPhase> disc_phase_{DiscoveryPhase::NONE};
  bool release_pending_{false};
  uint8_t disc_svc_index_{0};  // per-service characteristic discovery cursor
  bool op_read_active_{false};
  bool mtu_reported_{false};
};

}  // namespace esphome::bluetooth_connection

#endif  // USE_ESP32_BLE_NIMBLE && USE_BLE_GATT_CLIENT
