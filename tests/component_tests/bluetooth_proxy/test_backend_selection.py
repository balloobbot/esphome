"""The esp32 GATT backend follows the esp32_ble host stack selection."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path


def test_nimble_stack_selects_native_backend(
    generate_main: Callable[[str | Path], str],
    component_config_path: Callable[[str], Path],
) -> None:
    """stack: nimble binds the proxy slots to NimbleGattClient."""
    main_cpp = generate_main(component_config_path("proxy_nimble.yaml"))
    assert "NimbleGattClient" in main_cpp
    assert "BluedroidGattClient" not in main_cpp


def test_default_stack_selects_bluedroid_backend(
    generate_main: Callable[[str | Path], str],
    component_config_path: Callable[[str], Path],
) -> None:
    """The default stack keeps the Bluedroid backend."""
    main_cpp = generate_main(component_config_path("proxy_bluedroid.yaml"))
    assert "BluedroidGattClient" in main_cpp
    assert "NimbleGattClient" not in main_cpp
