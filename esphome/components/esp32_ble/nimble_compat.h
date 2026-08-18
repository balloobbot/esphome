#pragma once

// Source-compatible mirror of the Bluedroid API subset used by ESPHome, for
// builds that select the NimBLE host stack (esp32_ble: stack: nimble). With
// NimBLE enabled, ESP-IDF does not put the Bluedroid headers on the include
// path, so the types, constants and function declarations consumers rely on
// are provided here and implemented on top of NimBLE in esp32_ble/nimble_*.cpp.
//
// Type layouts and constant values mirror ESP-IDF's Bluedroid headers
// (components/bt/host/bluedroid/api/include/api, Apache-2.0). Only the arms
// and fields ESPHome reads are reproduced; each callback-param union carries a
// padding arm so its size matches the values asserted in ble_event.h.

#include "esphome/core/defines.h"

#ifdef USE_ESP32_BLE_NIMBLE

#include <esp_err.h>
#include <cstdbool>
#include <cstdint>

// ---------------------------------------------------------------------------
// esp_bt_defs.h subset
// ---------------------------------------------------------------------------

typedef enum {
  ESP_BT_STATUS_SUCCESS = 0,
  ESP_BT_STATUS_FAIL,
  ESP_BT_STATUS_NOT_READY,
  ESP_BT_STATUS_NOMEM,
  ESP_BT_STATUS_BUSY,
  ESP_BT_STATUS_DONE = 5,
  ESP_BT_STATUS_UNSUPPORTED,
  ESP_BT_STATUS_PARM_INVALID,
  ESP_BT_STATUS_UNHANDLED,
  ESP_BT_STATUS_AUTH_FAILURE,
  ESP_BT_STATUS_RMT_DEV_DOWN,
  ESP_BT_STATUS_AUTH_REJECTED,
  ESP_BT_STATUS_INVALID_STATIC_RAND_ADDR,
  ESP_BT_STATUS_PENDING,
  ESP_BT_STATUS_UNACCEPT_CONN_INTERVAL,
  ESP_BT_STATUS_PARAM_OUT_OF_RANGE,
  ESP_BT_STATUS_TIMEOUT,
} esp_bt_status_t;

#define ESP_BD_ADDR_LEN 6
typedef uint8_t esp_bd_addr_t[ESP_BD_ADDR_LEN];

#define ESP_UUID_LEN_16 2
#define ESP_UUID_LEN_32 4
#define ESP_UUID_LEN_128 16

typedef struct {
  uint16_t len;
  union {
    uint16_t uuid16;
    uint32_t uuid32;
    uint8_t uuid128[ESP_UUID_LEN_128];
  } uuid;
} __attribute__((packed)) esp_bt_uuid_t;

typedef enum {
  BLE_ADDR_TYPE_PUBLIC = 0x00,
  BLE_ADDR_TYPE_RANDOM = 0x01,
  BLE_ADDR_TYPE_RPA_PUBLIC = 0x02,
  BLE_ADDR_TYPE_RPA_RANDOM = 0x03,
} esp_ble_addr_type_t;

// ---------------------------------------------------------------------------
// esp_gatt_defs.h subset
// ---------------------------------------------------------------------------

typedef enum {
  ESP_GATT_OK = 0x0,
  ESP_GATT_INVALID_HANDLE = 0x01,
  ESP_GATT_READ_NOT_PERMIT = 0x02,
  ESP_GATT_WRITE_NOT_PERMIT = 0x03,
  ESP_GATT_INSUF_AUTHENTICATION = 0x05,
  ESP_GATT_REQ_NOT_SUPPORTED = 0x06,
  ESP_GATT_INVALID_OFFSET = 0x07,
  ESP_GATT_INSUF_AUTHORIZATION = 0x08,
  ESP_GATT_NOT_FOUND = 0x0a,
  ESP_GATT_INSUF_ENCRYPTION = 0x0f,
  ESP_GATT_INVALID_ATTR_LEN = 0x0d,
  ESP_GATT_ERR_UNLIKELY = 0x0e,
  ESP_GATT_NO_RESOURCES = 0x80,
  ESP_GATT_INTERNAL_ERROR = 0x81,
  ESP_GATT_WRONG_STATE = 0x82,
  ESP_GATT_DB_FULL = 0x83,
  ESP_GATT_BUSY = 0x84,
  ESP_GATT_ERROR = 0x85,
  ESP_GATT_CMD_STARTED = 0x86,
  ESP_GATT_ILLEGAL_PARAMETER = 0x87,
  ESP_GATT_AUTH_FAIL = 0x89,
  ESP_GATT_CONGESTED = 0x8f,
  ESP_GATT_ALREADY_OPEN = 0x91,
  ESP_GATT_STACK_RSP = 0xe0,
  ESP_GATT_APP_RSP = 0xe1,
  ESP_GATT_UNKNOWN_ERROR = 0xef,
} esp_gatt_status_t;

typedef enum {
  ESP_GATT_CONN_UNKNOWN = 0,
  ESP_GATT_CONN_L2C_FAILURE = 1,
  ESP_GATT_CONN_TIMEOUT = 0x08,
  ESP_GATT_CONN_TERMINATE_PEER_USER = 0x13,
  ESP_GATT_CONN_TERMINATE_LOCAL_HOST = 0x16,
  ESP_GATT_CONN_FAIL_ESTABLISH = 0x3e,
  ESP_GATT_CONN_LMP_TIMEOUT = 0x22,
  ESP_GATT_CONN_CONN_CANCEL = 0x0100,
  ESP_GATT_CONN_NONE = 0x0101,
} esp_gatt_conn_reason_t;

typedef struct {
  esp_bt_uuid_t uuid;
  uint8_t inst_id;
} __attribute__((packed)) esp_gatt_id_t;

typedef struct {
  esp_gatt_id_t id;
  bool is_primary;
} __attribute__((packed)) esp_gatt_srvc_id_t;

typedef enum {
  ESP_GATT_AUTH_REQ_NONE = 0,
  ESP_GATT_AUTH_REQ_NO_MITM = 1,
  ESP_GATT_AUTH_REQ_MITM = 3,
} esp_gatt_auth_req_t;

typedef enum {
  ESP_GATT_WRITE_TYPE_NO_RSP = 1,
  ESP_GATT_WRITE_TYPE_RSP = 2,
} esp_gatt_write_type_t;

#define ESP_GATT_PERM_READ (1 << 0)
#define ESP_GATT_PERM_READ_ENCRYPTED (1 << 1)
#define ESP_GATT_PERM_WRITE (1 << 4)
#define ESP_GATT_PERM_WRITE_ENCRYPTED (1 << 5)
typedef uint16_t esp_gatt_perm_t;

#define ESP_GATT_CHAR_PROP_BIT_BROADCAST (1 << 0)
#define ESP_GATT_CHAR_PROP_BIT_READ (1 << 1)
#define ESP_GATT_CHAR_PROP_BIT_WRITE_NR (1 << 2)
#define ESP_GATT_CHAR_PROP_BIT_WRITE (1 << 3)
#define ESP_GATT_CHAR_PROP_BIT_NOTIFY (1 << 4)
#define ESP_GATT_CHAR_PROP_BIT_INDICATE (1 << 5)
#define ESP_GATT_CHAR_PROP_BIT_AUTH (1 << 6)
#define ESP_GATT_CHAR_PROP_BIT_EXT_PROP (1 << 7)
typedef uint8_t esp_gatt_char_prop_t;

#define ESP_GATT_UUID_CHAR_CLIENT_CONFIG 0x2902
#define ESP_GATT_UUID_CHAR_DESCRIPTION 0x2901

#define ESP_GATT_MAX_ATTR_LEN 517
#define ESP_GATT_IF_NONE 0xff
typedef uint8_t esp_gatt_if_t;

#define ESP_GATT_RSP_BY_APP 0
#define ESP_GATT_AUTO_RSP 1

typedef struct {
  uint8_t auto_rsp;
} esp_attr_control_t;

typedef struct {
  uint16_t attr_max_len;
  uint16_t attr_len;
  uint8_t *attr_value;
} esp_attr_value_t;

typedef struct {
  uint8_t value[ESP_GATT_MAX_ATTR_LEN];
  uint16_t handle;
  uint16_t offset;
  uint16_t len;
  uint8_t auth_req;
} esp_gatt_value_t;

typedef union {
  esp_gatt_value_t attr_value;
  uint16_t handle;
} esp_gatt_rsp_t;

typedef enum {
  ESP_GATT_PREP_WRITE_CANCEL = 0x00,
  ESP_GATT_PREP_WRITE_EXEC = 0x01,
} esp_gatt_prep_write_type;

typedef enum {
  ESP_GATT_DB_PRIMARY_SERVICE,
  ESP_GATT_DB_SECONDARY_SERVICE,
  ESP_GATT_DB_CHARACTERISTIC,
  ESP_GATT_DB_DESCRIPTOR,
  ESP_GATT_DB_INCLUDED_SERVICE,
  ESP_GATT_DB_ALL,
} esp_gatt_db_attr_type_t;

typedef struct {
  bool is_primary;
  uint16_t start_handle;
  uint16_t end_handle;
  esp_bt_uuid_t uuid;
} esp_gattc_service_elem_t;

typedef struct {
  uint16_t char_handle;
  esp_gatt_char_prop_t properties;
  esp_bt_uuid_t uuid;
} esp_gattc_char_elem_t;

typedef struct {
  uint16_t handle;
  esp_bt_uuid_t uuid;
} esp_gattc_descr_elem_t;

// ---------------------------------------------------------------------------
// esp_gap_ble_api.h subset
// ---------------------------------------------------------------------------

#define ESP_BLE_ADV_FLAG_LIMIT_DISC (0x01 << 0)
#define ESP_BLE_ADV_FLAG_GEN_DISC (0x01 << 1)
#define ESP_BLE_ADV_FLAG_BREDR_NOT_SPT (0x01 << 2)

#define ESP_BLE_ADV_DATA_LEN_MAX 31
#define ESP_BLE_SCAN_RSP_DATA_LEN_MAX 31

typedef enum {
  ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT = 0,
  ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT = 1,
  ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT = 2,
  ESP_GAP_BLE_SCAN_RESULT_EVT = 3,
  ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT = 4,
  ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT = 5,
  ESP_GAP_BLE_ADV_START_COMPLETE_EVT = 6,
  ESP_GAP_BLE_SCAN_START_COMPLETE_EVT = 7,
  ESP_GAP_BLE_AUTH_CMPL_EVT = 8,
  ESP_GAP_BLE_KEY_EVT = 9,
  ESP_GAP_BLE_SEC_REQ_EVT = 10,
  ESP_GAP_BLE_PASSKEY_NOTIF_EVT = 11,
  ESP_GAP_BLE_PASSKEY_REQ_EVT = 12,
  ESP_GAP_BLE_OOB_REQ_EVT = 13,
  ESP_GAP_BLE_LOCAL_IR_EVT = 14,
  ESP_GAP_BLE_LOCAL_ER_EVT = 15,
  ESP_GAP_BLE_NC_REQ_EVT = 16,
  ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT = 17,
  ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT = 18,
  ESP_GAP_BLE_SET_STATIC_RAND_ADDR_EVT = 19,
  ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT = 20,
  ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT = 21,
  ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT = 22,
  ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT = 23,
  ESP_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT = 24,
  ESP_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT = 25,
  ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT = 26,
  ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT = 55,
  ESP_GAP_BLE_CHANNEL_SELECT_ALGORITHM_EVT = 60,
  ESP_GAP_BLE_EVT_MAX = 61,
} esp_gap_ble_cb_event_t;

typedef enum {
  ESP_GAP_SEARCH_INQ_RES_EVT = 0,
  ESP_GAP_SEARCH_INQ_CMPL_EVT = 1,
} esp_gap_search_evt_t;

typedef enum {
  ADV_TYPE_IND = 0x00,
  ADV_TYPE_DIRECT_IND_HIGH = 0x01,
  ADV_TYPE_SCAN_IND = 0x02,
  ADV_TYPE_NONCONN_IND = 0x03,
  ADV_TYPE_DIRECT_IND_LOW = 0x04,
} esp_ble_adv_type_t;

typedef enum {
  ADV_CHNL_37 = 0x01,
  ADV_CHNL_38 = 0x02,
  ADV_CHNL_39 = 0x04,
  ADV_CHNL_ALL = 0x07,
} esp_ble_adv_channel_t;

typedef enum {
  ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY = 0x00,
  ADV_FILTER_ALLOW_SCAN_WLST_CON_ANY,
  ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST,
  ADV_FILTER_ALLOW_SCAN_WLST_CON_WLST,
} esp_ble_adv_filter_t;

typedef enum {
  BLE_SCAN_TYPE_PASSIVE = 0x0,
  BLE_SCAN_TYPE_ACTIVE = 0x1,
} esp_ble_scan_type_t;

typedef enum {
  BLE_SCAN_FILTER_ALLOW_ALL = 0x0,
  BLE_SCAN_FILTER_ALLOW_ONLY_WLST = 0x1,
} esp_ble_scan_filter_t;

typedef enum {
  BLE_SCAN_DUPLICATE_DISABLE = 0x0,
  BLE_SCAN_DUPLICATE_ENABLE = 0x1,
} esp_ble_scan_duplicate_t;

typedef struct {
  uint16_t adv_int_min;
  uint16_t adv_int_max;
  esp_ble_adv_type_t adv_type;
  esp_ble_addr_type_t own_addr_type;
  esp_bd_addr_t peer_addr;
  esp_ble_addr_type_t peer_addr_type;
  esp_ble_adv_channel_t channel_map;
  esp_ble_adv_filter_t adv_filter_policy;
} esp_ble_adv_params_t;

typedef struct {
  bool set_scan_rsp;
  bool include_name;
  bool include_txpower;
  int min_interval;
  int max_interval;
  int appearance;
  uint16_t manufacturer_len;
  uint8_t *p_manufacturer_data;
  uint16_t service_data_len;
  uint8_t *p_service_data;
  uint16_t service_uuid_len;
  uint8_t *p_service_uuid;
  uint8_t flag;
} esp_ble_adv_data_t;

typedef struct {
  esp_ble_scan_type_t scan_type;
  esp_ble_addr_type_t own_addr_type;
  esp_ble_scan_filter_t scan_filter_policy;
  uint16_t scan_interval;
  uint16_t scan_window;
  esp_ble_scan_duplicate_t scan_duplicate;
} esp_ble_scan_params_t;

typedef struct {
  esp_bd_addr_t bda;
  uint16_t min_int;
  uint16_t max_int;
  uint16_t latency;
  uint16_t timeout;
} esp_ble_conn_update_params_t;

typedef struct {
  uint16_t interval;
  uint16_t latency;
  uint16_t timeout;
} esp_gap_conn_params_t;

typedef enum {
  ESP_BLE_SEC_ENCRYPT = 1,
  ESP_BLE_SEC_ENCRYPT_NO_MITM,
  ESP_BLE_SEC_ENCRYPT_MITM,
} esp_ble_sec_act_t;

typedef enum {
  ESP_BLE_SM_PASSKEY = 0,
  ESP_BLE_SM_AUTHEN_REQ_MODE,
  ESP_BLE_SM_IOCAP_MODE,
  ESP_BLE_SM_SET_INIT_KEY,
  ESP_BLE_SM_SET_RSP_KEY,
  ESP_BLE_SM_MAX_KEY_SIZE,
  ESP_BLE_SM_MIN_KEY_SIZE,
  ESP_BLE_SM_SET_STATIC_PASSKEY,
  ESP_BLE_SM_CLEAR_STATIC_PASSKEY,
  ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH,
  ESP_BLE_SM_OOB_SUPPORT,
  ESP_BLE_SM_MAX_PARAM,
} esp_ble_sm_param_t;

// I/O capabilities and authentication request modes (SMP spec values)
typedef uint8_t esp_ble_io_cap_t;
#define ESP_IO_CAP_OUT 0
#define ESP_IO_CAP_IO 1
#define ESP_IO_CAP_IN 2
#define ESP_IO_CAP_NONE 3
#define ESP_IO_CAP_KBDISP 4

typedef uint8_t esp_ble_auth_req_t;
#define ESP_LE_AUTH_NO_BOND 0x00
#define ESP_LE_AUTH_BOND 0x01
#define ESP_LE_AUTH_REQ_MITM (1 << 2)
#define ESP_LE_AUTH_REQ_BOND_MITM (ESP_LE_AUTH_BOND | ESP_LE_AUTH_REQ_MITM)
#define ESP_LE_AUTH_REQ_SC_ONLY (1 << 3)
#define ESP_LE_AUTH_REQ_SC_BOND (ESP_LE_AUTH_BOND | ESP_LE_AUTH_REQ_SC_ONLY)
#define ESP_LE_AUTH_REQ_SC_MITM (ESP_LE_AUTH_REQ_MITM | ESP_LE_AUTH_REQ_SC_ONLY)
#define ESP_LE_AUTH_REQ_SC_MITM_BOND (ESP_LE_AUTH_REQ_MITM | ESP_LE_AUTH_REQ_SC_ONLY | ESP_LE_AUTH_BOND)

typedef enum {
  ESP_BT_DEVICE_TYPE_BREDR = 0x01,
  ESP_BT_DEVICE_TYPE_BLE = 0x02,
  ESP_BT_DEVICE_TYPE_DUMO = 0x03,
} esp_bt_dev_type_t;

typedef struct {
  esp_bd_addr_t bd_addr;
  bool key_present;
  uint8_t key[16];
  uint8_t key_type;
  bool success;
  uint8_t fail_reason;
  esp_ble_addr_type_t addr_type;
  esp_bt_dev_type_t dev_type;
  esp_ble_auth_req_t auth_mode;
} esp_ble_auth_cmpl_t;

typedef struct {
  esp_bd_addr_t bd_addr;
} esp_ble_sec_req_t;

typedef struct {
  esp_bd_addr_t bd_addr;
  uint32_t passkey;
} esp_ble_sec_key_notif_t;

typedef union {
  esp_ble_sec_key_notif_t key_notif;
  esp_ble_sec_req_t ble_req;
  esp_ble_auth_cmpl_t auth_cmpl;
} esp_ble_sec_t;

typedef union {
  // Named nested struct types are required by the static_asserts in
  // ble_event.h; keep the Bluedroid tag names.
  struct ble_adv_data_cmpl_evt_param {
    esp_bt_status_t status;
  } adv_data_cmpl;
  struct ble_scan_rsp_data_cmpl_evt_param {
    esp_bt_status_t status;
  } scan_rsp_data_cmpl;
  struct ble_scan_param_cmpl_evt_param {
    esp_bt_status_t status;
  } scan_param_cmpl;
  struct ble_scan_result_evt_param {
    esp_gap_search_evt_t search_evt;
    esp_bd_addr_t bda;
    esp_bt_dev_type_t dev_type;
    esp_ble_addr_type_t ble_addr_type;
    int rssi;
    uint8_t ble_adv[ESP_BLE_ADV_DATA_LEN_MAX + ESP_BLE_SCAN_RSP_DATA_LEN_MAX];
    uint8_t flag;
    uint8_t num_resps;
    uint8_t adv_data_len;
    uint8_t scan_rsp_len;
  } scan_rst;
  struct ble_adv_data_raw_cmpl_evt_param {
    esp_bt_status_t status;
  } adv_data_raw_cmpl;
  struct ble_scan_rsp_data_raw_cmpl_evt_param {
    esp_bt_status_t status;
  } scan_rsp_data_raw_cmpl;
  struct ble_adv_start_cmpl_evt_param {
    esp_bt_status_t status;
  } adv_start_cmpl;
  struct ble_scan_start_cmpl_evt_param {
    esp_bt_status_t status;
  } scan_start_cmpl;
  esp_ble_sec_t ble_security;
  struct ble_adv_stop_cmpl_evt_param {
    esp_bt_status_t status;
  } adv_stop_cmpl;
  struct ble_scan_stop_cmpl_evt_param {
    esp_bt_status_t status;
  } scan_stop_cmpl;
  struct ble_update_conn_params_evt_param {
    esp_bt_status_t status;
    esp_bd_addr_t bda;
    uint16_t min_int;
    uint16_t max_int;
    uint16_t latency;
    uint16_t conn_int;
    uint16_t timeout;
  } update_conn_params;
  struct ble_read_rssi_cmpl_evt_param {
    esp_bt_status_t status;
    int8_t rssi;
    esp_bd_addr_t remote_addr;
  } read_rssi_cmpl;
} esp_ble_gap_cb_param_t;

typedef void (*esp_gap_ble_cb_t)(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// ---------------------------------------------------------------------------
// esp_gattc_api.h subset
// ---------------------------------------------------------------------------

typedef enum {
  ESP_GATTC_REG_EVT = 0,
  ESP_GATTC_UNREG_EVT = 1,
  ESP_GATTC_OPEN_EVT = 2,
  ESP_GATTC_READ_CHAR_EVT = 3,
  ESP_GATTC_WRITE_CHAR_EVT = 4,
  ESP_GATTC_CLOSE_EVT = 5,
  ESP_GATTC_SEARCH_CMPL_EVT = 6,
  ESP_GATTC_SEARCH_RES_EVT = 7,
  ESP_GATTC_READ_DESCR_EVT = 8,
  ESP_GATTC_WRITE_DESCR_EVT = 9,
  ESP_GATTC_NOTIFY_EVT = 10,
  ESP_GATTC_PREP_WRITE_EVT = 11,
  ESP_GATTC_EXEC_EVT = 12,
  ESP_GATTC_SRVC_CHG_EVT = 15,
  ESP_GATTC_CFG_MTU_EVT = 18,
  ESP_GATTC_CONGEST_EVT = 24,
  ESP_GATTC_REG_FOR_NOTIFY_EVT = 38,
  ESP_GATTC_UNREG_FOR_NOTIFY_EVT = 39,
  ESP_GATTC_CONNECT_EVT = 40,
  ESP_GATTC_DISCONNECT_EVT = 41,
  ESP_GATTC_QUEUE_FULL_EVT = 43,
  ESP_GATTC_DIS_SRVC_CMPL_EVT = 46,
} esp_gattc_cb_event_t;

typedef union {
  struct gattc_reg_evt_param {
    esp_gatt_status_t status;
    uint16_t app_id;
  } reg;
  struct gattc_open_evt_param {
    esp_gatt_status_t status;
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
    uint16_t mtu;
  } open;
  struct gattc_close_evt_param {
    esp_gatt_status_t status;
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
    esp_gatt_conn_reason_t reason;
  } close;
  struct gattc_cfg_mtu_evt_param {
    esp_gatt_status_t status;
    uint16_t conn_id;
    uint16_t mtu;
  } cfg_mtu;
  struct gattc_search_cmpl_evt_param {
    esp_gatt_status_t status;
    uint16_t conn_id;
  } search_cmpl;
  struct gattc_search_res_evt_param {
    uint16_t conn_id;
    uint16_t start_handle;
    uint16_t end_handle;
    esp_gatt_id_t srvc_id;
    bool is_primary;
  } search_res;
  struct gattc_read_char_evt_param {
    esp_gatt_status_t status;
    uint16_t conn_id;
    uint16_t handle;
    uint8_t *value;
    uint16_t value_len;
  } read;
  struct gattc_write_evt_param {
    esp_gatt_status_t status;
    uint16_t conn_id;
    uint16_t handle;
    uint16_t offset;
  } write;
  struct gattc_notify_evt_param {
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
    uint16_t handle;
    uint16_t value_len;
    uint8_t *value;
    bool is_notify;
  } notify;
  struct gattc_connect_evt_param {
    uint16_t conn_id;
    uint8_t link_role;
    esp_bd_addr_t remote_bda;
    esp_gap_conn_params_t conn_params;
    esp_ble_addr_type_t ble_addr_type;
    uint16_t conn_handle;
  } connect;
  struct gattc_disconnect_evt_param {
    esp_gatt_conn_reason_t reason;
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
  } disconnect;
  struct gattc_reg_for_notify_evt_param {
    esp_gatt_status_t status;
    uint16_t handle;
  } reg_for_notify;
  struct gattc_unreg_for_notify_evt_param {
    esp_gatt_status_t status;
    uint16_t handle;
  } unreg_for_notify;
  // Keeps sizeof(esp_ble_gattc_cb_param_t) equal to Bluedroid's (asserted in
  // ble_event.h); the mirrored arms above are all smaller.
  uint8_t esphome_size_padding[28];
} esp_ble_gattc_cb_param_t;

typedef void (*esp_gattc_cb_t)(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

// ---------------------------------------------------------------------------
// esp_gatts_api.h subset
// ---------------------------------------------------------------------------

typedef enum {
  ESP_GATTS_REG_EVT = 0,
  ESP_GATTS_READ_EVT = 1,
  ESP_GATTS_WRITE_EVT = 2,
  ESP_GATTS_EXEC_WRITE_EVT = 3,
  ESP_GATTS_MTU_EVT = 4,
  ESP_GATTS_CONF_EVT = 5,
  ESP_GATTS_UNREG_EVT = 6,
  ESP_GATTS_CREATE_EVT = 7,
  ESP_GATTS_ADD_INCL_SRVC_EVT = 8,
  ESP_GATTS_ADD_CHAR_EVT = 9,
  ESP_GATTS_ADD_CHAR_DESCR_EVT = 10,
  ESP_GATTS_DELETE_EVT = 11,
  ESP_GATTS_START_EVT = 12,
  ESP_GATTS_STOP_EVT = 13,
  ESP_GATTS_CONNECT_EVT = 14,
  ESP_GATTS_DISCONNECT_EVT = 15,
  ESP_GATTS_OPEN_EVT = 16,
  ESP_GATTS_CANCEL_OPEN_EVT = 17,
  ESP_GATTS_CLOSE_EVT = 18,
  ESP_GATTS_CONGEST_EVT = 20,
  ESP_GATTS_RESPONSE_EVT = 21,
} esp_gatts_cb_event_t;

typedef union {
  struct gatts_reg_evt_param {
    esp_gatt_status_t status;
    uint16_t app_id;
  } reg;
  struct gatts_read_evt_param {
    uint16_t conn_id;
    uint32_t trans_id;
    esp_bd_addr_t bda;
    uint16_t handle;
    uint16_t offset;
    bool is_long;
    bool need_rsp;
  } read;
  struct gatts_write_evt_param {
    uint16_t conn_id;
    uint32_t trans_id;
    esp_bd_addr_t bda;
    uint16_t handle;
    uint16_t offset;
    bool need_rsp;
    bool is_prep;
    uint16_t len;
    uint8_t *value;
  } write;
  struct gatts_exec_write_evt_param {
    uint16_t conn_id;
    uint32_t trans_id;
    esp_bd_addr_t bda;
    uint8_t exec_write_flag;
  } exec_write;
  struct gatts_mtu_evt_param {
    uint16_t conn_id;
    uint16_t mtu;
  } mtu;
  struct gatts_conf_evt_param {
    esp_gatt_status_t status;
    uint16_t conn_id;
    uint16_t handle;
  } conf;
  struct gatts_create_evt_param {
    esp_gatt_status_t status;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
  } create;
  struct gatts_add_char_evt_param {
    esp_gatt_status_t status;
    uint16_t attr_handle;
    uint16_t service_handle;
    esp_bt_uuid_t char_uuid;
  } add_char;
  struct gatts_add_char_descr_evt_param {
    esp_gatt_status_t status;
    uint16_t attr_handle;
    uint16_t service_handle;
    esp_bt_uuid_t descr_uuid;
  } add_char_descr;
  struct gatts_delete_evt_param {
    esp_gatt_status_t status;
    uint16_t service_handle;
  } del;
  struct gatts_start_evt_param {
    esp_gatt_status_t status;
    uint16_t service_handle;
  } start;
  struct gatts_stop_evt_param {
    esp_gatt_status_t status;
    uint16_t service_handle;
  } stop;
  struct gatts_connect_evt_param {
    uint16_t conn_id;
    uint8_t link_role;
    esp_bd_addr_t remote_bda;
    esp_gap_conn_params_t conn_params;
    esp_ble_addr_type_t ble_addr_type;
    uint16_t conn_handle;
  } connect;
  struct gatts_disconnect_evt_param {
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
    esp_gatt_conn_reason_t reason;
  } disconnect;
  // Keeps sizeof(esp_ble_gatts_cb_param_t) equal to Bluedroid's (asserted in
  // ble_event.h); the mirrored arms above are all smaller.
  uint8_t esphome_size_padding[32];
} esp_ble_gatts_cb_param_t;

typedef void (*esp_gatts_cb_t)(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// ---------------------------------------------------------------------------
// Function declarations, implemented on top of NimBLE in esp32_ble/nimble_*.cpp
// ---------------------------------------------------------------------------

// Device / GAP
const uint8_t *esp_bt_dev_get_address(void);
esp_err_t esp_ble_gap_register_callback(esp_gap_ble_cb_t callback);
esp_err_t esp_ble_gap_set_device_name(const char *name);
esp_err_t esp_ble_gap_set_security_param(esp_ble_sm_param_t param_type, void *value, uint8_t len);
esp_err_t esp_ble_gap_config_adv_data(esp_ble_adv_data_t *adv_data);
esp_err_t esp_ble_gap_config_adv_data_raw(uint8_t *raw_data, uint32_t raw_data_len);
esp_err_t esp_ble_gap_start_advertising(esp_ble_adv_params_t *adv_params);
esp_err_t esp_ble_gap_stop_advertising(void);
esp_err_t esp_ble_gap_set_scan_params(esp_ble_scan_params_t *scan_params);
esp_err_t esp_ble_gap_start_scanning(uint32_t duration);
esp_err_t esp_ble_gap_stop_scanning(void);
esp_err_t esp_ble_gap_read_rssi(esp_bd_addr_t remote_addr);
esp_err_t esp_ble_gap_update_conn_params(esp_ble_conn_update_params_t *params);
esp_err_t esp_ble_gap_set_prefer_conn_params(esp_bd_addr_t bd_addr, uint16_t min_conn_int, uint16_t max_conn_int,
                                             uint16_t slave_latency, uint16_t supervision_tout);
esp_err_t esp_ble_gap_security_rsp(esp_bd_addr_t bd_addr, bool accept);
esp_err_t esp_ble_set_encryption(esp_bd_addr_t bd_addr, esp_ble_sec_act_t sec_act);
esp_err_t esp_ble_passkey_reply(esp_bd_addr_t bd_addr, bool accept, uint32_t passkey);
esp_err_t esp_ble_confirm_reply(esp_bd_addr_t bd_addr, bool accept);
esp_err_t esp_ble_remove_bond_device(esp_bd_addr_t bd_addr);

// GATT client
esp_err_t esp_ble_gattc_register_callback(esp_gattc_cb_t callback);
esp_err_t esp_ble_gattc_app_register(uint16_t app_id);
esp_err_t esp_ble_gattc_open(esp_gatt_if_t gattc_if, esp_bd_addr_t remote_bda, esp_ble_addr_type_t remote_addr_type,
                             bool is_direct);
esp_err_t esp_ble_gattc_close(esp_gatt_if_t gattc_if, uint16_t conn_id);
esp_err_t esp_ble_gattc_send_mtu_req(esp_gatt_if_t gattc_if, uint16_t conn_id);
esp_err_t esp_ble_gattc_search_service(esp_gatt_if_t gattc_if, uint16_t conn_id, esp_bt_uuid_t *filter_uuid);
esp_gatt_status_t esp_ble_gattc_get_service(esp_gatt_if_t gattc_if, uint16_t conn_id, esp_bt_uuid_t *svc_uuid,
                                            esp_gattc_service_elem_t *result, uint16_t *count, uint16_t offset);
esp_gatt_status_t esp_ble_gattc_get_all_char(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t start_handle,
                                             uint16_t end_handle, esp_gattc_char_elem_t *result, uint16_t *count,
                                             uint16_t offset);
esp_gatt_status_t esp_ble_gattc_get_all_descr(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t char_handle,
                                              esp_gattc_descr_elem_t *result, uint16_t *count, uint16_t offset);
esp_gatt_status_t esp_ble_gattc_get_descr_by_char_handle(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t char_handle,
                                                         esp_bt_uuid_t descr_uuid, esp_gattc_descr_elem_t *result,
                                                         uint16_t *count);
esp_gatt_status_t esp_ble_gattc_get_attr_count(esp_gatt_if_t gattc_if, uint16_t conn_id, esp_gatt_db_attr_type_t type,
                                               uint16_t start_handle, uint16_t end_handle, uint16_t char_handle,
                                               uint16_t *count);
esp_err_t esp_ble_gattc_read_char(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t handle,
                                  esp_gatt_auth_req_t auth_req);
esp_err_t esp_ble_gattc_read_char_descr(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t handle,
                                        esp_gatt_auth_req_t auth_req);
esp_err_t esp_ble_gattc_write_char(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t handle, uint16_t value_len,
                                   uint8_t *value, esp_gatt_write_type_t write_type, esp_gatt_auth_req_t auth_req);
esp_err_t esp_ble_gattc_write_char_descr(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t handle, uint16_t value_len,
                                         uint8_t *value, esp_gatt_write_type_t write_type,
                                         esp_gatt_auth_req_t auth_req);
esp_err_t esp_ble_gattc_register_for_notify(esp_gatt_if_t gattc_if, esp_bd_addr_t server_bda, uint16_t handle);
esp_err_t esp_ble_gattc_unregister_for_notify(esp_gatt_if_t gattc_if, esp_bd_addr_t server_bda, uint16_t handle);
esp_err_t esp_ble_gattc_cache_clean(esp_bd_addr_t remote_bda);

// GATT server
esp_err_t esp_ble_gatts_register_callback(esp_gatts_cb_t callback);
esp_err_t esp_ble_gatts_app_register(uint16_t app_id);
esp_err_t esp_ble_gatts_create_service(esp_gatt_if_t gatts_if, esp_gatt_srvc_id_t *service_id, uint16_t num_handle);
esp_err_t esp_ble_gatts_delete_service(uint16_t service_handle);
esp_err_t esp_ble_gatts_start_service(uint16_t service_handle);
esp_err_t esp_ble_gatts_stop_service(uint16_t service_handle);
esp_err_t esp_ble_gatts_add_char(uint16_t service_handle, esp_bt_uuid_t *char_uuid, esp_gatt_perm_t perm,
                                 esp_gatt_char_prop_t property, esp_attr_value_t *char_val,
                                 esp_attr_control_t *control);
esp_err_t esp_ble_gatts_add_char_descr(uint16_t service_handle, esp_bt_uuid_t *descr_uuid, esp_gatt_perm_t perm,
                                       esp_attr_value_t *char_descr_val, esp_attr_control_t *control);
esp_err_t esp_ble_gatts_send_response(esp_gatt_if_t gatts_if, uint16_t conn_id, uint32_t trans_id,
                                      esp_gatt_status_t status, esp_gatt_rsp_t *rsp);
esp_err_t esp_ble_gatts_send_indicate(esp_gatt_if_t gatts_if, uint16_t conn_id, uint16_t attr_handle,
                                      uint16_t value_len, uint8_t *value, bool need_confirm);

#endif  // USE_ESP32_BLE_NIMBLE
