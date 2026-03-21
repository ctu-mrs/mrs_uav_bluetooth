"""DBus lifecycle helpers used by the ROS Bluetooth node."""

import threading
import time
from typing import Any, Callable, Dict, Optional

import dbus
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib

from .bluetooth_bridge_state import TopicExportBridgeState
from .dbus_advertisement import Advertisement
from .dbus_agent import PairingAgent
from .dbus_client import BleClient
from .dbus_common import BLUEZ_SERVICE_PATH, BLUEZ_SERVICE_NAME, DBUS_PROP_IFACE, GATT_MANAGER_IFACE, LE_ADVERTISING_MANAGER_IFACE, AGENT_MANAGER_IFACE, ADAPTER_IFACE
from .dbus_gatt import Application, Profile, find_adapter
from .gatt_services import TIME_SERVICE_UUID, WIFI_SERVICE_UUID, TimeService, TopicBridgeService, WifiService


class GlibMainLoopThread:
    def __init__(self):
        self._mainloop = None
        self._thread = None
        self._started = threading.Event()

    def start(self):
        if self._mainloop is not None:
            return
        self._mainloop = GLib.MainLoop()
        self._started.clear()

        def _run_mainloop():
            self._started.set()
            self._mainloop.run()

        self._thread = threading.Thread(target=_run_mainloop, daemon=True, name="glib-mainloop")
        self._thread.start()
        if not self._started.wait(timeout=2.0):
            raise RuntimeError("GLib main loop did not start")

    def stop(self):
        if self._mainloop is not None and self._mainloop.is_running():
            try:
                self.invoke(self._mainloop.quit)
            except RuntimeError:
                self._mainloop.quit()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._mainloop = None
        self._thread = None
        self._started.clear()

    def invoke(self, callback: Callable[[], None]):
        if self._mainloop is None:
            raise RuntimeError("GLib main loop is not running")

        def _run_once():
            callback()
            return False

        GLib.idle_add(_run_once)


class BluetoothDbusRuntime:
    _LEGACY_ADV_MAX_BYTES = 31
    _ADV_FLAGS_BYTES = 3
    _ADV_TX_POWER_BYTES = 3
    _GATT_APPLICATION_SETTLE_S = 0.2

    def __init__(
        self,
        local_name: str,
        logger: Any,
        log_verbose: Callable[[str], None],
        on_pairing_event: Callable[..., None],
    ):
        self._local_name = local_name
        self._logger = logger
        self._log_verbose = log_verbose
        self._on_pairing_event = on_pairing_event

        self._glib = GlibMainLoopThread()
        self._bus = None
        self._adapter_path = ""
        self._client = None
        self._app = None
        self._advertisement = None
        self._agent = None
        self._wifi_service = None
        self._time_service = None
        self._pairing_agent_registered = False
        self._server_generation = 0

    @property
    def adapter_path(self) -> str:
        return self._adapter_path

    @property
    def bus(self):
        return self._bus

    @property
    def client(self) -> Optional[BleClient]:
        return self._client

    @property
    def app(self):
        return self._app

    @property
    def advertisement(self):
        return self._advertisement

    @property
    def wifi_service(self):
        return self._wifi_service

    @property
    def time_service(self):
        return self._time_service

    def setup(self):
        if self._client is not None:
            return
        DBusGMainLoop(set_as_default=True)
        self._bus = dbus.SystemBus()
        self._glib.start()
        self._adapter_path = find_adapter(self._bus)
        if not self._adapter_path:
            raise RuntimeError("No Bluetooth adapter with GattManager1 found")
        self._logger.info(f"Using adapter {self._adapter_path}")
        self.set_adapter_props(powered=True)
        self._client = BleClient(self._bus, self._adapter_path)

    def ensure_pairing_agent(self, *, auto_accept: bool, auto_trust: bool, capability: str):
        self._require_setup()
        if self._pairing_agent_registered:
            return
        self._agent = PairingAgent(
            self._bus,
            auto_accept=auto_accept,
            auto_trust=auto_trust,
            on_event=self._on_pairing_event,
        )
        agent_mgr = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, BLUEZ_SERVICE_PATH), AGENT_MANAGER_IFACE)
        agent_mgr.RegisterAgent(PairingAgent.AGENT_PATH, capability, timeout=10.0)
        agent_mgr.RequestDefaultAgent(PairingAgent.AGENT_PATH, timeout=10.0)
        self._pairing_agent_registered = True
        self._logger.info(f"Pairing agent registered (capability={capability})")

    def rebuild_server(
        self,
        topic_exports: Dict[str, TopicExportBridgeState],
        *,
        client_profile_uuids,
        enable_wifi_service: bool,
        enable_time_service: bool,
        advertise_mode: str,
        discoverable_timeout: int,
        wifi_name_cb: Callable[[], str],
        wifi_apply_cb: Callable[[str], None],
        wifi_password_write_cb: Callable[[str], None],
        wifi_password_read_cb: Callable[[], str],
        time_writeback_cb: Optional[Callable[[bytes, dict, int], None]] = None,
    ):
        self._require_setup()
        self.unregister_server_objects(topic_exports)
        self._server_generation += 1
        generation = self._server_generation

        app = Application(self._bus)
        service_index = 0
        profile_index = 0
        if enable_wifi_service:
            self._wifi_service = WifiService(
                self._bus,
                service_index,
                wifi_name_cb=wifi_name_cb,
                wifi_apply_cb=wifi_apply_cb,
                wifi_password_write_cb=wifi_password_write_cb,
                wifi_password_read_cb=wifi_password_read_cb,
            )
            app.add_service(self._wifi_service)
            service_index += 1
        if enable_time_service:
            self._time_service = TimeService(self._bus, service_index, writeback_cb=time_writeback_cb)
            app.add_service(self._time_service)
            service_index += 1
        for state in topic_exports.values():
            state.service = TopicBridgeService(
                self._bus,
                service_index,
                topic_name=state.topic_name,
                message_type=state.message_type,
                bridge_name=state.bridge_name,
                bridge_key=state.bridge_key,
                member_specs=state.member_specs,
                rate_hz=state.rate_hz,
                payload_format=state.payload_format,
            )
            app.add_service(state.service)
            service_index += 1

        profile_uuids = [str(uuid) for uuid in client_profile_uuids if str(uuid).strip()]
        if profile_uuids:
            app.add_profile(Profile(self._bus, profile_index, profile_uuids))

        service_uuids = []
        if self._time_service is not None:
            service_uuids.append(TIME_SERVICE_UUID)
        if self._wifi_service is not None:
            service_uuids.append(WIFI_SERVICE_UUID)
        for state in topic_exports.values():
            if state.service is not None:
                service_uuids.append(state.service.uuid)

        self.set_adapter_props(
            alias=self._local_name,
            discoverable_timeout=discoverable_timeout,
            discoverable=True,
            pairable=True,
        )

        self._app = app
        self._advertisement = Advertisement(self._bus, 0, advertise_mode)
        self._advertisement.local_name = self._local_name
        gatt_mgr = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, self._adapter_path), GATT_MANAGER_IFACE)

        def _register_app_reply():
            if generation != self._server_generation or self._app is not app:
                return
            self._logger.info(
                f"GATT application registered ({service_index} services, {len(profile_uuids)} client profile UUIDs)"
            )
            GLib.timeout_add(
                max(1, int(self._GATT_APPLICATION_SETTLE_S * 1000)),
                lambda: self._register_advertisement(service_uuids, generation),
            )

        def _register_app_error(error):
            self._logger.error(f"Failed to register GATT application: {error}")
            self._cleanup_failed_server_build(app, topic_exports, generation)

        self._logger.info(
            f"Registering GATT application ({service_index} services, {len(profile_uuids)} client profile UUIDs)"
        )
        gatt_mgr.RegisterApplication(
            app.get_path(),
            {},
            reply_handler=_register_app_reply,
            error_handler=_register_app_error,
        )

    def unregister_server_objects(self, topic_exports: Dict[str, TopicExportBridgeState]):
        self._server_generation += 1
        for state in topic_exports.values():
            state.service = None
        if self._advertisement is not None:
            try:
                ad_mgr = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, self._adapter_path), LE_ADVERTISING_MANAGER_IFACE)
                ad_mgr.UnregisterAdvertisement(
                    self._advertisement.get_path(),
                    reply_handler=lambda: None,
                    error_handler=lambda error: self._log_verbose(f"Ignore advertisement unregister error: {error}"),
                    timeout=10.0,
                )
            except Exception:
                pass
            self._advertisement.destroy()
            self._advertisement = None
        if self._app is not None:
            try:
                gatt_mgr = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, self._adapter_path), GATT_MANAGER_IFACE)
                gatt_mgr.UnregisterApplication(
                    self._app.get_path(),
                    reply_handler=lambda: None,
                    error_handler=lambda error: self._log_verbose(f"Ignore GATT app unregister error: {error}"),
                    timeout=10.0,
                )
            except Exception:
                pass
            self._app.destroy()
            self._app = None
        self._wifi_service = None
        self._time_service = None

    def set_adapter_props(self, powered=None, discoverable=None, pairable=None, alias=None, discoverable_timeout=None):
        self._require_setup()
        props = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, self._adapter_path), DBUS_PROP_IFACE)
        if powered is not None:
            self._set_adapter_property(
                props,
                "Powered",
                dbus.Boolean(powered, variant_level=1),
                ignore_busy=False,
            )
        if alias is not None:
            self._set_adapter_property(
                props,
                "Alias",
                dbus.String(alias, variant_level=1),
                ignore_busy=True,
            )
        if discoverable_timeout is not None:
            self._set_adapter_property(
                props,
                "DiscoverableTimeout",
                dbus.UInt32(discoverable_timeout, variant_level=1),
                ignore_busy=True,
            )
        if discoverable is not None:
            self._set_adapter_property(
                props,
                "Discoverable",
                dbus.Boolean(discoverable, variant_level=1),
                ignore_busy=True,
            )
        if pairable is not None:
            self._set_adapter_property(
                props,
                "Pairable",
                dbus.Boolean(pairable, variant_level=1),
                ignore_busy=True,
            )

    def _set_adapter_property(self, props, prop_name: str, value, *, ignore_busy: bool):
        try:
            props.Set(ADAPTER_IFACE, prop_name, value, timeout=10.0)
        except dbus.DBusException as exc:
            if ignore_busy and self._is_bluez_busy_error(exc):
                self._logger.warning(f"BlueZ busy while setting adapter {prop_name}; continuing")
                self._log_verbose(f"Adapter property deferred prop={prop_name} error={exc}")
                return
            raise

    def _is_bluez_busy_error(self, exc: Exception) -> bool:
        if not isinstance(exc, dbus.DBusException):
            return False
        name = str(exc.get_dbus_name() or "")
        message = str(exc)
        return name == "org.bluez.Error.Busy" or "org.bluez.Error.Busy" in message or "Busy" in message

    def _register_advertisement(self, service_uuids, generation: int):
        advertisement = self._advertisement
        if advertisement is None or generation != self._server_generation:
            return False
        adv_capabilities = self._get_advertising_capabilities()
        max_tx_power = adv_capabilities.get("max_tx_power")
        if max_tx_power is not None:
            advertisement.tx_power = max_tx_power
            if adv_capabilities.get("can_set_tx_power"):
                self._logger.info(f"Using maximum BLE advertisement TxPower ({max_tx_power} dBm)")
            else:
                self._logger.warning(
                    "Controller reported MaxTxPower but not CanSetTxPower; "
                    f"still attempting TxPower={max_tx_power} dBm"
                )
        else:
            advertisement.tx_power = None

        attempts = []
        include_tx_power_candidates = [True, False]
        if not adv_capabilities.get("supports_tx_power_include"):
            include_tx_power_candidates = [False]

        for include_tx_power in include_tx_power_candidates:
            attempts.append(
                (
                    include_tx_power,
                    self._select_advertised_service_uuids(service_uuids, include_tx_power=include_tx_power),
                )
            )
        attempts.append((False, []))

        ad_mgr = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, self._adapter_path), LE_ADVERTISING_MANAGER_IFACE)
        attempted_configs = set()
        filtered_attempts = []
        for include_tx_power, advertised_uuids in attempts:
            config_key = (include_tx_power, tuple(advertised_uuids))
            if config_key in attempted_configs:
                continue
            attempted_configs.add(config_key)
            if len(advertised_uuids) != len(service_uuids):
                self._logger.warning(
                    "BLE advertisement trimmed from "
                    f"{len(service_uuids)} to {len(advertised_uuids)} service UUIDs to fit controller limits"
                )
            filtered_attempts.append((include_tx_power, advertised_uuids))

        def _attempt(index: int):
            if self._advertisement is not advertisement or generation != self._server_generation:
                return
            if index >= len(filtered_attempts):
                self._logger.error("BLE advertisement unavailable; continuing without advertising")
                advertisement.destroy()
                if self._advertisement is advertisement:
                    self._advertisement = None
                return
            include_tx_power, advertised_uuids = filtered_attempts[index]
            advertisement.include_tx_power = include_tx_power
            advertisement.service_uuids = list(advertised_uuids)
            self._logger.info(
                f"Registering BLE advertisement (name={self._local_name}, uuids={len(advertised_uuids)}, "
                f"tx_power={'ON' if include_tx_power else 'OFF'})"
            )

            def _reply_handler():
                if self._advertisement is not advertisement or generation != self._server_generation:
                    return
                self._logger.info(
                    f"BLE advertisement registered (name={self._local_name}, uuids={len(advertised_uuids)}, "
                    f"tx_power={'ON' if include_tx_power else 'OFF'})"
                )

            def _error_handler(error):
                self._logger.warning(
                    f"BLE advertisement registration attempt failed (uuids={len(advertised_uuids)}, "
                    f"tx_power={'ON' if include_tx_power else 'OFF'}): {error}"
                )
                _attempt(index + 1)

            ad_mgr.RegisterAdvertisement(
                advertisement.get_path(),
                {},
                reply_handler=_reply_handler,
                error_handler=_error_handler,
            )

        _attempt(0)
        return False

    def _get_advertising_capabilities(self):
        supports_tx_power_include = True
        can_set_tx_power = False
        max_tx_power = None

        props = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, self._adapter_path), DBUS_PROP_IFACE)
        try:
            adv_props = self._call_dbus_method(
                props.GetAll,
                "read advertising capabilities",
                LE_ADVERTISING_MANAGER_IFACE,
                timeout=10.0,
            )
        except Exception as exc:
            self._logger.warning(f"Unable to query LEAdvertisingManager1 capabilities: {exc}")
            return {
                "supports_tx_power_include": supports_tx_power_include,
                "can_set_tx_power": can_set_tx_power,
                "max_tx_power": max_tx_power,
            }

        supported_includes = {str(value) for value in adv_props.get("SupportedIncludes", [])}
        if supported_includes:
            supports_tx_power_include = "tx-power" in supported_includes

        supported_features = {str(value) for value in adv_props.get("SupportedFeatures", [])}
        capabilities = adv_props.get("SupportedCapabilities", {})
        if "MaxTxPower" in capabilities:
            try:
                max_tx_power = int(capabilities["MaxTxPower"])
            except Exception:
                max_tx_power = None
        can_set_tx_power = "CanSetTxPower" in supported_features and max_tx_power is not None
        if max_tx_power is None:
            self._log_verbose("LEAdvertisingManager1 did not report MaxTxPower capability")

        return {
            "supports_tx_power_include": supports_tx_power_include,
            "can_set_tx_power": can_set_tx_power,
            "max_tx_power": max_tx_power,
        }

    def _select_advertised_service_uuids(self, service_uuids, *, include_tx_power: bool):
        # Legacy LE advertising payload is limited to 31 bytes including flags.
        remaining_bytes = self._LEGACY_ADV_MAX_BYTES - self._ADV_FLAGS_BYTES
        local_name = self._local_name or ""
        if local_name:
            remaining_bytes -= 2 + len(local_name.encode("utf-8"))
        if include_tx_power:
            remaining_bytes -= self._ADV_TX_POWER_BYTES
        if remaining_bytes <= 2:
            return []

        advertised = []
        used_bytes = 2
        for uuid in service_uuids:
            uuid_size = self._uuid_advertising_size(uuid)
            if used_bytes + uuid_size > remaining_bytes:
                break
            advertised.append(uuid)
            used_bytes += uuid_size
        return advertised

    def _uuid_advertising_size(self, uuid: str) -> int:
        compact = str(uuid).replace("-", "")
        if len(compact) == 4:
            return 2
        if len(compact) == 8:
            return 4
        return 16

    def start_scan(self, transport: str) -> bool:
        if self._client is None:
            return False
        ok = self._client.start_scan(transport=transport)
        if ok:
            self._log_verbose(f"Scan started (transport={transport})")
        else:
            self._logger.warning(f"Failed to start scan (transport={transport})")
        return ok

    def stop_scan(self) -> bool:
        if self._client is None:
            return False
        return self._client.stop_scan()

    def shutdown(self, topic_exports: Dict[str, TopicExportBridgeState]):
        self._log_verbose("Shutting down Bluetooth DBus runtime...")
        self.unregister_server_objects(topic_exports)
        if self._pairing_agent_registered:
            try:
                agent_mgr = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, BLUEZ_SERVICE_PATH), AGENT_MANAGER_IFACE)
                self._call_dbus_method(agent_mgr.UnregisterAgent, "unregister pairing agent", PairingAgent.AGENT_PATH, timeout=10.0)
            except Exception:
                pass
            self._pairing_agent_registered = False
        try:
            self.set_adapter_props(discoverable=False, pairable=False)
        except Exception:
            pass
        self._glib.stop()
        self._client = None
        self._bus = None
        self._adapter_path = ""
        self._agent = None

    def _call_dbus_method(self, method, operation: str, *args, timeout: float = 30.0):
        del operation, timeout
        return method(*args)

    def _cleanup_failed_server_build(
        self,
        app: Application,
        topic_exports: Dict[str, TopicExportBridgeState],
        generation: int,
    ):
        if generation != self._server_generation:
            return
        if self._advertisement is not None:
            try:
                self._advertisement.destroy()
            except Exception:
                pass
            self._advertisement = None
        try:
            app.destroy()
        except Exception:
            pass
        if self._app is app:
            self._app = None
        self._wifi_service = None
        self._time_service = None
        for state in topic_exports.values():
            state.service = None

    def _require_setup(self):
        if self._bus is None or not self._adapter_path:
            raise RuntimeError("Bluetooth DBus runtime has not been initialized")