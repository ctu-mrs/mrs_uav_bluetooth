"""DBus lifecycle helpers used by the ROS Bluetooth node."""

import threading
from typing import Any, Callable, Dict, Optional

import dbus
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib

from .bluetooth_bridge_state import TopicExportBridgeState
from .dbus_advertisement import Advertisement
from .dbus_agent import AGENT_PATH, PairingAgent
from .dbus_client import BleClient
from .dbus_common import BLUEZ_SERVICE_NAME, DBUS_PROP_IFACE, GATT_MANAGER_IFACE, LE_ADVERTISING_MANAGER_IFACE
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


class BluetoothDbusRuntime:
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
        agent_mgr = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, "/org/bluez"), "org.bluez.AgentManager1")
        agent_mgr.RegisterAgent(AGENT_PATH, capability)
        agent_mgr.RequestDefaultAgent(AGENT_PATH)
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
            self._time_service = TimeService(self._bus, service_index)
            app.add_service(self._time_service)
            service_index += 1
        for state in topic_exports.values():
            state.service = TopicBridgeService(
                self._bus,
                service_index,
                topic_name=state.topic_name,
                message_type=state.message_type,
                bridge_name=state.bridge_name,
                member_paths=state.member_paths,
                rate_hz=state.rate_hz,
                payload_format=state.payload_format,
            )
            app.add_service(state.service)
            service_index += 1

        gatt_mgr = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, self._adapter_path), GATT_MANAGER_IFACE)
        advertisement = Advertisement(self._bus, 0, advertise_mode)
        advertisement.local_name = self._local_name

        service_uuids = []
        if self._wifi_service is not None:
            service_uuids.append(WIFI_SERVICE_UUID)
        if self._time_service is not None:
            service_uuids.append(TIME_SERVICE_UUID)
        for state in topic_exports.values():
            if state.service is not None:
                service_uuids.append(state.service.uuid)
        advertisement.service_uuids = service_uuids
        advertisement.include_tx_power = True

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

            self._logger.info(f"Registering BLE advertisement (name={self._local_name}, uuids={len(service_uuids)})")
            self._call_dbus_method_async(
                ad_mgr.RegisterAdvertisement,
                "register the BLE advertisement",
                advertisement.get_path(),
                {},
            )
            advertisement_registered = True
            self._advertisement = advertisement
            self._logger.info(f"BLE advertisement registered (name={self._local_name}, uuids={len(service_uuids)})")
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
        iface = "org.bluez.Adapter1"
        if powered is not None:
            props.Set(iface, "Powered", dbus.Boolean(powered))
        if alias is not None:
            props.Set(iface, "Alias", dbus.String(alias))
        if discoverable_timeout is not None:
            props.Set(iface, "DiscoverableTimeout", dbus.UInt32(discoverable_timeout))
        if discoverable is not None:
            props.Set(iface, "Discoverable", dbus.Boolean(discoverable))
        if pairable is not None:
            props.Set(iface, "Pairable", dbus.Boolean(pairable))

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
                agent_mgr = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, "/org/bluez"), "org.bluez.AgentManager1")
                agent_mgr.UnregisterAgent(AGENT_PATH)
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

    def _call_dbus_method_async(self, method, operation: str, *args, timeout: float = 30.0):
        completed = threading.Event()
        result = {}

        def reply_handler(*reply_args):
            result["reply"] = reply_args
            completed.set()

        def error_handler(error):
            result["error"] = error
            completed.set()

        method(
            *args,
            reply_handler=reply_handler,
            error_handler=error_handler,
            timeout=timeout,
        )
        if not completed.wait(timeout + 1.0):
            raise TimeoutError(f"Timed out while waiting to {operation}")
        error = result.get("error")
        if error is not None:
            raise error
        return result.get("reply", ())

    def _require_setup(self):
        if self._bus is None or not self._adapter_path:
            raise RuntimeError("Bluetooth DBus runtime has not been initialized")