"""Tests for the esp32_ble host stack selection (bluedroid vs nimble)."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import pytest

from esphome import config_validation as cv
from esphome.components.esp32.const import KEY_ESP32, KEY_SDKCONFIG_OPTIONS
from esphome.components.esp32_ble import CONFIG_SCHEMA
from esphome.const import PlatformFramework
from esphome.core import CORE

from ..types import SetCoreConfigCallable


def _sdkconfig() -> dict:
    return CORE.data[KEY_ESP32][KEY_SDKCONFIG_OPTIONS]


def test_nimble_stack_sets_nimble_sdkconfig(
    generate_main: Callable[[str | Path], str],
    component_config_path: Callable[[str], Path],
) -> None:
    """stack: nimble enables NimBLE, disables Bluedroid, and sets roles."""
    main_cpp = generate_main(component_config_path("stack_nimble.yaml"))

    opts = _sdkconfig()
    assert opts["CONFIG_BT_NIMBLE_ENABLED"] is True
    assert opts["CONFIG_BT_BLUEDROID_ENABLED"] is False
    assert opts["CONFIG_BT_NIMBLE_MAX_CONNECTIONS"] == 4
    # tracker present -> observer/central; server present -> peripheral/broadcaster
    assert opts["CONFIG_BT_NIMBLE_ROLE_OBSERVER"] is True
    assert opts["CONFIG_BT_NIMBLE_ROLE_CENTRAL"] is True
    assert opts["CONFIG_BT_NIMBLE_ROLE_PERIPHERAL"] is True
    assert opts["CONFIG_BT_NIMBLE_ROLE_BROADCASTER"] is True
    assert opts["CONFIG_BT_NIMBLE_DYNAMIC_SERVICE"] is True
    assert opts["CONFIG_BT_NIMBLE_SECURITY_ENABLE"] is True
    # Bluedroid host options must not be emitted
    assert "CONFIG_BT_ACL_CONNECTIONS" not in opts
    assert "CONFIG_BT_GATTC_ENABLE" not in opts
    assert "CONFIG_BT_BTU_TASK_STACK_SIZE" not in opts
    assert "CONFIG_BT_GATTC_NOTIF_REG_MAX" not in opts
    # The esp32 reconciler must not enable the Bluedroid feature flags
    assert opts["CONFIG_BT_BLE_42_FEATURES_SUPPORTED"] is False
    assert main_cpp is not None


def test_nimble_beacon_only_disables_connection_features(
    generate_main: Callable[[str | Path], str],
    component_config_path: Callable[[str], Path],
) -> None:
    """A broadcast-only build must not enable roles or the security manager.

    NimBLE's SM and connection code only link when a connection role exists.
    """
    generate_main(component_config_path("stack_nimble_beacon.yaml"))

    opts = _sdkconfig()
    assert opts["CONFIG_BT_NIMBLE_ROLE_CENTRAL"] is False
    assert opts["CONFIG_BT_NIMBLE_ROLE_PERIPHERAL"] is False
    assert opts["CONFIG_BT_NIMBLE_ROLE_BROADCASTER"] is True
    assert opts["CONFIG_BT_NIMBLE_SECURITY_ENABLE"] is False
    assert "CONFIG_BT_NIMBLE_MAX_CONNECTIONS" not in opts
    assert "CONFIG_BT_NIMBLE_NVS_PERSIST" not in opts
    assert "CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU" not in opts


def test_default_stack_is_bluedroid(
    generate_main: Callable[[str | Path], str],
    component_config_path: Callable[[str], Path],
) -> None:
    """Without stack:, the Bluedroid options are emitted as before."""
    generate_main(component_config_path("stack_default.yaml"))

    opts = _sdkconfig()
    assert "CONFIG_BT_NIMBLE_ENABLED" not in opts
    assert opts["CONFIG_BT_GATTC_ENABLE"] is True
    assert opts["CONFIG_BT_ACL_CONNECTIONS"] == 4  # max_connections 3 + 1
    assert opts["CONFIG_BT_BLE_42_FEATURES_SUPPORTED"] is True


def test_nimble_requires_esp_idf(set_core_config: SetCoreConfigCallable) -> None:
    """stack: nimble is rejected on the Arduino framework."""
    set_core_config(PlatformFramework.ESP32_ARDUINO)
    with pytest.raises(cv.Invalid):
        CONFIG_SCHEMA({"stack": "nimble"})
    # and accepted (schema level) on esp-idf
    set_core_config(PlatformFramework.ESP32_IDF)
    assert CONFIG_SCHEMA({"stack": "nimble"})["stack"] == "nimble"
