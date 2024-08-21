#!/bin/bash
set -e

echo "Unblocking bluetooth (hci0) via rfkill"
rfkill unblock 0

echo "Restarting bluetooth service"
systemctl restart bluetooth

echo "Turning bluetooth (hci0) on via hciconfig"
hciconfig hci0 up

echo "Powering bletooth on via bluetoothctl"
bluetoothctl -- power on

echo "Making bluetooth device discoverable via bluetoothctl"
bluetoothctl -- discoverable on

echo "Starting Python BLE configurator script"
python3 /etc/ctu-mrs/uav-bluetooth/bt_app.py

echo "Done."
exit 0

