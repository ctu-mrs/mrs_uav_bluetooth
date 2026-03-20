"""DBus lifecycle helpers used by the ROS Bluetooth node."""

from collections import deque
import threading
import time
from typing import Any, Callable, Deque, Dict, Optional

import dbus
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib

from .bluetooth_bridge_state import TopicExportBridgeState
from .dbus_advertisement import Advertisement
from .dbus_agent import PairingAgent
from .dbus_client import BleClient
from .dbus_common import BLUEZ_SERVICE_PATH, BLUEZ_SERVICE_NAME, DBUS_PROP_IFACE, GATT_MANAGER_IFACE, LE_ADVERTISING_MANAGER_IFACE, AGENT_MANAGER_IFACE, ADAPTER_IFACE
from .dbus_gatt import Application, find_adapter
from .gatt_services import TIME_SERVICE_UUID, WIFI_SERVICE_UUID, TimeService, TopicBridgeService, WifiService


class GlibMainLoopThread:
    def __init__(self):
        self._mainloop = None
        self._thread = None

    def start(self):
        if self._mainloop is not None:
            return
        self._mainloop = GLib.MainLoop()
        self._thread = threading.Thread(target=self._mainloop.run, daemon=True, name="glib-mainloop")
        self._thread.start()

    def stop(self):
        if self._mainloop is not None and self._mainloop.is_running():
            self._mainloop.quit()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._mainloop = None
        self._thread = None

    def invoke(self, callback: Callable[[], None]):
        if self._mainloop is None:
            raise RuntimeError("GLib main loop is not running")

        def _run_once():
            callback()
            return False

        GLib.idle_add(_run_once)


class SerializedDbusQueue:
    class _Request:
        def __init__(self, runner, operation: str, timeout: float, on_reply=None, on_error=None):
            self.runner = runner
            self.operation = operation
            self.timeout = timeout
            self.on_reply = on_reply
            self.on_error = on_error
            self.completed = threading.Event()
            self.reply = ()
            self.error = None
            self.timer = None

    def __init__(self, dispatcher: Callable[[Callable[[], None]], None], logger: Any, log_verbose: Callable[[str], None]):
        self._dispatcher = dispatcher
        self._logger = logger
        self._log_verbose = log_verbose
        self._lock = threading.RLock()
        self._pending: Deque[SerializedDbusQueue._Request] = deque()
        self._active = None

    def submit_call(
        self,
        func: Callable[[], Any],
        operation: str,
        *,
        timeout: float = 30.0,
        wait: bool = False,
        on_reply=None,
        on_error=None,
    ):
        def _runner(request):
            try:
                reply = func()
            except Exception as exc:
                self._finish(request, error=exc)
                return
            if reply is None:
                reply_args = ()
            elif isinstance(reply, tuple):
                reply_args = reply
            else:
                reply_args = (reply,)
            self._finish(request, reply=reply_args)

        return self._submit(_runner, operation, timeout=timeout, wait=wait, on_reply=on_reply, on_error=on_error)

    def submit_async_method(
        self,
        method,
        operation: str,
        *args,
        timeout: float = 30.0,
        wait: bool = False,
        on_reply=None,
        on_error=None,
    ):
        def _runner(request):
            try:
                method(
                    *args,
                    reply_handler=lambda *reply_args: self._finish(request, reply=reply_args),
                    error_handler=lambda error: self._finish(request, error=error),
                    timeout=timeout,
                )
            except Exception as exc:
                self._finish(request, error=exc)

        return self._submit(_runner, operation, timeout=timeout, wait=wait, on_reply=on_reply, on_error=on_error)

    def _submit(self, runner, operation: str, *, timeout: float, wait: bool, on_reply=None, on_error=None):
        request = self._Request(runner, operation, timeout, on_reply=on_reply, on_error=on_error)
        with self._lock:
            self._pending.append(request)
            should_dispatch = self._active is None
        request.timer = threading.Timer(max(1.0, float(timeout)) + 0.5, lambda: self._expire(request))
        request.timer.daemon = True
        request.timer.start()
        if should_dispatch:
            self._dispatcher(self._pump)
        if not wait:
            return True
        if not request.completed.wait(max(1.0, float(timeout)) + 1.0):
            raise TimeoutError(f"Timed out while waiting to {operation}")
        if request.error is not None:
            raise request.error
        return request.reply

    def _pump(self):
        with self._lock:
            if self._active is not None or not self._pending:
                return
            request = self._pending.popleft()
            self._active = request
        request.runner(request)

    def _finish(self, request, *, reply=(), error=None):
        with self._lock:
            if request.completed.is_set():
                return
            request.reply = reply or ()
            request.error = error
            timer = request.timer
            request.timer = None
            if self._active is request:
                self._active = None
        if timer is not None:
            timer.cancel()
        try:
            if error is not None:
                if request.on_error is not None:
                    request.on_error(error)
            elif request.on_reply is not None:
                request.on_reply(*request.reply)
        except Exception as exc:
            self._logger.warning(f"DBus callback failed while handling {request.operation}: {exc}")
            self._log_verbose(f"DBus callback failure operation={request.operation} error={exc}")
        request.completed.set()
        self._dispatcher(self._pump)

    def _expire(self, request):
        message = f"Timed out while waiting to {request.operation}"
        self._logger.warning(message)
        self._log_verbose(f"DBus queue timeout operation={request.operation}")
        self._finish(request, error=TimeoutError(message))


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
        self._dbus_queue = None

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
        self._adapter_path = find_adapter(self._bus)
        if not self._adapter_path:
            raise RuntimeError("No Bluetooth adapter with GattManager1 found")
        self._logger.info(f"Using adapter {self._adapter_path}")
        self._glib.start()
        self._dbus_queue = SerializedDbusQueue(self._glib.invoke, self._logger, self._log_verbose)
        self.set_adapter_props(powered=True)
        self._client = BleClient(self._bus, self._adapter_path, dbus_queue=self._dbus_queue)

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
        agent_mgr.RegisterAgent(PairingAgent.AGENT_PATH, capability)
        agent_mgr.RequestDefaultAgent(PairingAgent.AGENT_PATH)
        self._pairing_agent_registered = True
        self._logger.info(f"Pairing agent registered (capability={capability})")

    def rebuild_server(
        self,
        topic_exports: Dict[str, TopicExportBridgeState],
        *,
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

        app = Application(self._bus)
        service_index = 0
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

        gatt_mgr = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, self._adapter_path), GATT_MANAGER_IFACE)
        advertisement = Advertisement(self._bus, 0, advertise_mode)
        advertisement.local_name = self._local_name

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

        ad_mgr = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, self._adapter_path), LE_ADVERTISING_MANAGER_IFACE)
        app_registered = False
        advertisement_registered = False
        try:
            self._logger.info(f"Registering GATT application ({service_index} services)")
            self._call_dbus_method_async(
                gatt_mgr.RegisterApplication,
                "register the GATT application",
                app.get_path(),
                {},
            )
            app_registered = True
            self._app = app
            self._logger.info(f"GATT application registered ({service_index} services)")
            time.sleep(self._GATT_APPLICATION_SETTLE_S)

            advertisement_registered = self._register_advertisement(ad_mgr, advertisement, service_uuids)
            if advertisement_registered:
                self._advertisement = advertisement
        except Exception:
            if advertisement_registered:
                try:
                    self._call_dbus_method_async(
                        ad_mgr.UnregisterAdvertisement,
                        "unregister the BLE advertisement",
                        advertisement.get_path(),
                        timeout=10.0,
                    )
                except Exception:
                    pass
            if app_registered:
                try:
                    self._call_dbus_method_async(
                        gatt_mgr.UnregisterApplication,
                        "unregister the GATT application",
                        app.get_path(),
                        timeout=10.0,
                    )
                except Exception:
                    pass
            advertisement.destroy()
            app.destroy()
            self._advertisement = None
            self._app = None
            self._wifi_service = None
            self._time_service = None
            for state in topic_exports.values():
                state.service = None
            raise

    def unregister_server_objects(self, topic_exports: Dict[str, TopicExportBridgeState]):
        for state in topic_exports.values():
            state.service = None
        if self._advertisement is not None:
            try:
                ad_mgr = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, self._adapter_path), LE_ADVERTISING_MANAGER_IFACE)
                self._call_dbus_method_async(
                    ad_mgr.UnregisterAdvertisement,
                    "unregister the BLE advertisement",
                    self._advertisement.get_path(),
                    timeout=10.0,
                )
            except Exception:
                pass
            self._advertisement.destroy()
            self._advertisement = None
        if self._app is not None:
            try:
                gatt_mgr = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, self._adapter_path), GATT_MANAGER_IFACE)
                self._call_dbus_method_async(
                    gatt_mgr.UnregisterApplication,
                    "unregister the GATT application",
                    self._app.get_path(),
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
        iface = ADAPTER_IFACE
        if powered is not None:
            props.Set(iface, "Powered", dbus.Boolean(powered, variant_level=1))
        if alias is not None:
            props.Set(iface, "Alias", dbus.String(alias, variant_level=1))
        if discoverable_timeout is not None:
            props.Set(iface, "DiscoverableTimeout", dbus.UInt32(discoverable_timeout, variant_level=1))
        if discoverable is not None:
            props.Set(iface, "Discoverable", dbus.Boolean(discoverable, variant_level=1))
        if pairable is not None:
            props.Set(iface, "Pairable", dbus.Boolean(pairable, variant_level=1))

    def _register_advertisement(self, ad_mgr, advertisement: Advertisement, service_uuids) -> bool:
        last_error = None
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

        attempted_configs = set()
        for include_tx_power, advertised_uuids in attempts:
            config_key = (include_tx_power, tuple(advertised_uuids))
            if config_key in attempted_configs:
                continue
            attempted_configs.add(config_key)

            advertisement.include_tx_power = include_tx_power
            advertisement.service_uuids = list(advertised_uuids)

            if len(advertised_uuids) != len(service_uuids):
                self._logger.warning(
                    "BLE advertisement trimmed from "
                    f"{len(service_uuids)} to {len(advertised_uuids)} service UUIDs to fit controller limits"
                )

            try:
                self._logger.info(
                    f"Registering BLE advertisement (name={self._local_name}, uuids={len(advertised_uuids)}, "
                    f"tx_power={'ON' if include_tx_power else 'OFF'})"
                )
                self._call_dbus_method_async(
                    ad_mgr.RegisterAdvertisement,
                    "register the BLE advertisement",
                    advertisement.get_path(),
                    {},
                )
                self._logger.info(
                    f"BLE advertisement registered (name={self._local_name}, uuids={len(advertised_uuids)}, "
                    f"tx_power={'ON' if include_tx_power else 'OFF'})"
                )
                return True
            except Exception as exc:
                last_error = exc
                self._logger.warning(
                    f"BLE advertisement registration attempt failed (uuids={len(advertised_uuids)}, "
                    f"tx_power={'ON' if include_tx_power else 'OFF'}): {exc}"
                )

        self._logger.error(f"BLE advertisement unavailable; continuing without advertising: {last_error}")
        advertisement.destroy()
        return False

    def _get_advertising_capabilities(self):
        supports_tx_power_include = True
        can_set_tx_power = False
        max_tx_power = None

        props = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, self._adapter_path), DBUS_PROP_IFACE)
        try:
            adv_props = props.GetAll(LE_ADVERTISING_MANAGER_IFACE)
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
                agent_mgr.UnregisterAgent(PairingAgent.AGENT_PATH)
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
        self._dbus_queue = None

    def _call_dbus_method_async(self, method, operation: str, *args, timeout: float = 30.0):
        if self._dbus_queue is None:
            raise RuntimeError("Bluetooth DBus runtime queue is not initialized")
        return self._dbus_queue.submit_async_method(method, operation, *args, timeout=timeout, wait=True)

    def _require_setup(self):
        if self._bus is None or not self._adapter_path:
            raise RuntimeError("Bluetooth DBus runtime has not been initialized")