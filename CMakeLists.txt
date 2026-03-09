cmake_minimum_required(VERSION 3.8)
project(mrs_uav_bluetooth)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(msg_files
  "msg/BleDevice.msg"
  "msg/BleDeviceArray.msg"
  "msg/BleGattService.msg"
  "msg/BleGattCharacteristic.msg"
  "msg/BleGattDescriptor.msg"
  "msg/BleNotification.msg"
)

set(srv_files
  "srv/ConnectDevice.srv"
  "srv/DisconnectDevice.srv"
  "srv/PairDevice.srv"
  "srv/SetDeviceTrust.srv"
  "srv/RemoveDevice.srv"
  "srv/ListGattServices.srv"
  "srv/ListGattCharacteristics.srv"
  "srv/ListGattDescriptors.srv"
  "srv/ReadGattValue.srv"
  "srv/WriteGattValue.srv"
  "srv/SetNotify.srv"
  "srv/SetScanEnabled.srv"
  "srv/GetDevice.srv"
  "srv/ListDevices.srv"
  "srv/FindGattPath.srv"
  "srv/ConfigureNotificationBridge.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_python_install_package(${PROJECT_NAME}_pkg PACKAGE_DIR src)

install(PROGRAMS
  src/__init__.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bluetooth_node
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()