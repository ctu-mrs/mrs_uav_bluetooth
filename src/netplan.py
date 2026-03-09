"""Helpers for reading and switching the active Wi-Fi netplan profile."""

import os
import re
import shutil
import subprocess
import threading
from typing import List, Optional, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover - runtime dependency on target system
    yaml = None


class NmcliWrapper:
    def __init__(self, executable: str = "nmcli"):
        self._executable = executable

    def available(self) -> bool:
        return shutil.which(self._executable) is not None

    def _run(self, *args: str, timeout: float = 20.0) -> Tuple[bool, str]:
        if not self.available():
            return False, "nmcli not available"
        try:
            completed = subprocess.run(
                [self._executable, *args],
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return False, str(exc)
        output = (completed.stdout or "").strip()
        error = (completed.stderr or "").strip()
        detail = output or error or f"nmcli exited with code {completed.returncode}"
        return completed.returncode == 0, detail

    def get_current_ssid(self) -> str:
        ok, detail = self._run(
            "--terse",
            "--escape",
            "no",
            "--fields",
            "DEVICE,TYPE,STATE,CONNECTION",
            "device",
            "status",
            timeout=10.0,
        )
        if not ok:
            return ""
        for line in detail.splitlines():
            parts = line.split(":", 3)
            if len(parts) != 4:
                continue
            _, device_type, state, connection = parts
            if device_type != "wifi":
                continue
            if not state.startswith("connected") and not state.startswith("connecting"):
                continue
            if connection and connection != "--":
                return connection
        return ""

    def list_visible_ssids(self) -> List[str]:
        ok, detail = self._run(
            "--terse",
            "--escape",
            "no",
            "--fields",
            "SSID",
            "device",
            "wifi",
            "list",
            "--rescan",
            "no",
            timeout=15.0,
        )
        if not ok:
            return []
        seen = []
        for line in detail.splitlines():
            ssid = line.strip()
            if not ssid or ssid in seen:
                continue
            seen.append(ssid)
        return seen

    def connect(self, ssid: str, password: str = "") -> Tuple[bool, str]:
        args = ["--wait", "20", "device", "wifi", "connect", ssid]
        if password:
            args.extend(["password", password])
        return self._run(*args, timeout=25.0)


class NetplanConfiguration:
    def __init__(
        self,
        netplan_config_file: str,
        autoscripts_dir: str,
        allowed_networks: Optional[List[str]] = None,
    ):
        self._netplan_config_file = netplan_config_file
        self._autoscripts_dir = autoscripts_dir
        self._allowed_networks = list(allowed_networks or [])
        self._lock = threading.RLock()
        self._proc: Optional[subprocess.Popen] = None
        self._nmcli = NmcliWrapper()

    @property
    def allowed_networks(self) -> List[str]:
        return list(self._allowed_networks)

    @allowed_networks.setter
    def allowed_networks(self, value: List[str]):
        self._allowed_networks = list(value or [])

    def _poll_proc(self):
        if self._proc is not None and self._proc.poll() is not None:
            self._proc = None

    def busy(self) -> bool:
        with self._lock:
            self._poll_proc()
            return self._proc is not None

    def get_current_ssid(self) -> str:
        with self._lock:
            self._poll_proc()
            ssid = self._nmcli.get_current_ssid()
            if ssid:
                return ssid
            try:
                with open(self._netplan_config_file, "r", encoding="utf-8") as handle:
                    text = handle.read()
            except OSError:
                return ""
            matches = re.findall(r'"([^"]+)"', text)
            for match in matches:
                if match.startswith("mrs_ctu") or not self._allowed_networks:
                    return match
            return matches[0] if matches else ""

    def list_known_ssids(self) -> List[str]:
        names = []
        for ssid in self._allowed_networks:
            if ssid and ssid not in names:
                names.append(ssid)
        if os.path.isdir(self._autoscripts_dir):
            for entry in sorted(os.listdir(self._autoscripts_dir)):
                if not entry.endswith(".sh"):
                    continue
                ssid = entry[:-3]
                if ssid and ssid not in names:
                    names.append(ssid)
        for ssid in self._nmcli.list_visible_ssids():
            if ssid not in names:
                names.append(ssid)
        return names

    def resolve_script(self, target: str) -> Tuple[Optional[str], str]:
        target = target.strip()
        if not target:
            return None, "empty target SSID"
        direct = os.path.join(self._autoscripts_dir, f"{target}.sh")
        if os.path.exists(direct):
            return direct, ""
        return None, f"no netplan script for '{target}'"

    def _apply_script(self, script_path: str) -> Tuple[bool, str]:
        try:
            self._proc = subprocess.Popen(["bash", script_path])
        except OSError as exc:
            self._proc = None
            return False, str(exc)
        return True, script_path

    def _select_wifi_interface(self, config: dict) -> Tuple[str, dict]:
        network = config.setdefault("network", {})
        network.setdefault("version", 2)
        wifis = network.setdefault("wifis", {})
        if wifis:
            interface_name = next(iter(wifis.keys()))
            return interface_name, wifis[interface_name]
        wifis["wlan0"] = {"dhcp4": True, "optional": True}
        return "wlan0", wifis["wlan0"]

    def _write_netplan(self, ssid: str, password: str) -> Tuple[bool, str]:
        if yaml is None:
            return False, "python3-yaml is required for direct Wi-Fi configuration"
        try:
            with open(self._netplan_config_file, "r", encoding="utf-8") as handle:
                config = yaml.safe_load(handle) or {}
        except FileNotFoundError:
            config = {}
        except OSError as exc:
            return False, str(exc)

        _, iface_cfg = self._select_wifi_interface(config)
        iface_cfg.setdefault("dhcp4", True)
        iface_cfg.setdefault("optional", True)
        access_point = {}
        if password:
            access_point["password"] = password
        iface_cfg["access-points"] = {ssid: access_point}

        try:
            with open(self._netplan_config_file, "w", encoding="utf-8") as handle:
                yaml.safe_dump(config, handle, sort_keys=False)
            self._proc = subprocess.Popen(["netplan", "apply"])
        except OSError as exc:
            self._proc = None
            return False, str(exc)
        return True, f"netplan:{ssid}"

    def set_current_network(self, ssid: str, password: str = "") -> Tuple[bool, str]:
        with self._lock:
            self._poll_proc()
            if self._proc is not None:
                return False, "netplan change already in progress"
            target = ssid.strip()
            if not target:
                return False, "empty target SSID"
            script_path, _ = self.resolve_script(target)
            if script_path is not None and not password:
                return self._apply_script(script_path)
            if self._nmcli.available():
                ok, detail = self._nmcli.connect(target, password=password)
                if ok:
                    return True, f"nmcli:{target}"
            return self._write_netplan(target, password)

    def set_current_ssid(self, target: str) -> Tuple[bool, str]:
        return self.set_current_network(target, "")
