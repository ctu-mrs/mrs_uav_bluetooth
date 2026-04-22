cmake_minimum_required(VERSION 3.14)
project(mrs_uav_bluetooth)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

# ---- ROS 2 / ament dependencies ----
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# ---- External dependencies ----
find_package(yaml-cpp REQUIRED)

# Debian/Bloom package builds need the packaged sdbus-c++ from /usr. Both
# CMake and pkg-config usually search /usr/local first, which can make the
# binaries link against an unpackaged local copy that dh_shlibdeps rejects.
set(_sdbus_system_cmake_paths)
if(CMAKE_LIBRARY_ARCHITECTURE)
  list(APPEND _sdbus_system_cmake_paths
    "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/sdbus-c++")
endif()
list(APPEND _sdbus_system_cmake_paths
  "/usr/${CMAKE_INSTALL_LIBDIR}/cmake/sdbus-c++"
  "/usr/lib/cmake/sdbus-c++")

find_package(sdbus-c++ CONFIG QUIET
  PATHS ${_sdbus_system_cmake_paths}
  NO_DEFAULT_PATH
)

if(NOT TARGET SDBusCpp::sdbus-c++)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    set(_sdbus_saved_pkg_config_libdir "$ENV{PKG_CONFIG_LIBDIR}")
    set(_sdbus_system_pkgconfig_dirs "/usr/share/pkgconfig:/usr/lib/pkgconfig")
    if(CMAKE_LIBRARY_ARCHITECTURE)
      string(PREPEND _sdbus_system_pkgconfig_dirs
        "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/pkgconfig:")
    endif()

    set(ENV{PKG_CONFIG_LIBDIR} "${_sdbus_system_pkgconfig_dirs}")
    pkg_check_modules(SDBUSCPP QUIET IMPORTED_TARGET sdbus-c++)

    if(DEFINED _sdbus_saved_pkg_config_libdir)
      set(ENV{PKG_CONFIG_LIBDIR} "${_sdbus_saved_pkg_config_libdir}")
    else()
      unset(ENV{PKG_CONFIG_LIBDIR})
    endif()

    if(SDBUSCPP_FOUND AND NOT TARGET SDBusCpp::sdbus-c++)
      add_library(SDBusCpp::sdbus-c++ ALIAS PkgConfig::SDBUSCPP)
    endif()
  endif()
endif()

if(NOT TARGET SDBusCpp::sdbus-c++)
  find_package(sdbus-c++ REQUIRED)
endif()

# =====================================================================
# ROS interface generation (msg / srv) – unchanged
# =====================================================================

set(msg_files
  "msg/BleDevice.msg"
  "msg/BleDeviceArray.msg"
  "msg/BleGattService.msg"
  "msg/BleGattCharacteristic.msg"
  "msg/BleGattDescriptor.msg"
  "msg/BleNotification.msg"
  "msg/BlePeerTimeStatus.msg"
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
  "srv/SetActiveConfig.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

# =====================================================================
# C++ utility library
# =====================================================================

add_library(${PROJECT_NAME}_util STATIC
  src/util/device_utils.cpp
  src/util/string_utils.cpp
  src/util/uuid_utils.cpp
  src/util/hostname_utils.cpp
  src/util/topic_utils.cpp
  src/util/logging_utils.cpp
)
target_include_directories(${PROJECT_NAME}_util PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_util
  rclcpp
  std_msgs
  builtin_interfaces
)

# =====================================================================
# C++ config library
# =====================================================================

add_library(${PROJECT_NAME}_config STATIC
  src/config/config_loader.cpp
  src/config/overlay_config_manager.cpp
  src/config/shared_topic_config.cpp
)
target_include_directories(${PROJECT_NAME}_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_config
  ${PROJECT_NAME}_util
  yaml-cpp::yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME}_config
  rclcpp
  std_msgs
  builtin_interfaces
)

# =====================================================================
# C++ bridge (payload) library
# =====================================================================

add_library(${PROJECT_NAME}_bridge STATIC
  src/bridge/payload_codec.cpp
  src/bridge/generic_message_bridge.cpp
)
target_include_directories(${PROJECT_NAME}_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_bridge
  ${PROJECT_NAME}_util
)
ament_target_dependencies(${PROJECT_NAME}_bridge
  rclcpp
  rcpputils
  builtin_interfaces
  nav_msgs
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_cpp
)

# =====================================================================
# C++ bluez library
# =====================================================================

add_library(${PROJECT_NAME}_bluez STATIC
  src/bluez/dbus_connection.cpp
  src/bluez/adapter_controller.cpp
  src/bluez/object_manager_cache.cpp
  src/bluez/bluez_client.cpp
  src/bluez/bluez_client_gatt.cpp
  src/bluez/bluez_pairing_agent.cpp
)
target_include_directories(${PROJECT_NAME}_bluez PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_bluez
  ${PROJECT_NAME}_util
  SDBusCpp::sdbus-c++
)
ament_target_dependencies(${PROJECT_NAME}_bluez
  rclcpp
)

# =====================================================================
# C++ GATT export library
# =====================================================================

add_library(${PROJECT_NAME}_gatt STATIC
  src/gatt/gatt_application.cpp
  src/gatt/advertisement.cpp
  src/gatt/builtin_gatt.cpp
  src/gatt/bridge_naming.cpp
  src/gatt/remote_gatt_inspector.cpp
  src/gatt/services/wifi_service.cpp
  src/gatt/services/time_service.cpp
  src/gatt/services/topic_bridge_service.cpp
)
target_include_directories(${PROJECT_NAME}_gatt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_gatt
  ${PROJECT_NAME}_util
  ${PROJECT_NAME}_bridge
  ${PROJECT_NAME}_bluez
  yaml-cpp::yaml-cpp
  SDBusCpp::sdbus-c++
)
ament_target_dependencies(${PROJECT_NAME}_gatt
  rclcpp
)

# =====================================================================
# C++ network library
# =====================================================================

add_library(${PROJECT_NAME}_network STATIC
  src/network/netplan_manager.cpp
)
target_include_directories(${PROJECT_NAME}_network PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_network
  yaml-cpp::yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME}_network
  rclcpp
)

# =====================================================================
# C++ ROS interface library
# =====================================================================

add_library(${PROJECT_NAME}_ros STATIC
  src/ros/status_publisher.cpp
  src/ros/service_servers.cpp
  src/ros/ros_interface_manager.cpp
)
target_include_directories(${PROJECT_NAME}_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}_util
  ${PROJECT_NAME}_config
  ${PROJECT_NAME}_bridge
  ${PROJECT_NAME}_bluez
  ${PROJECT_NAME}_gatt
  ${PROJECT_NAME}_network
)
ament_target_dependencies(${PROJECT_NAME}_ros
  rclcpp std_msgs std_srvs nav_msgs rcl_interfaces
)

# =====================================================================
# C++ peer management library
# =====================================================================

add_library(${PROJECT_NAME}_peer STATIC
  src/peer/peer_manager.cpp
  src/peer/peer_time_bridge.cpp
)
target_include_directories(${PROJECT_NAME}_peer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_peer
  ${PROJECT_NAME}_util
  ${PROJECT_NAME}_config
  ${PROJECT_NAME}_bluez
  ${PROJECT_NAME}_gatt
  ${PROJECT_NAME}_bridge
)
ament_target_dependencies(${PROJECT_NAME}_peer
  rclcpp std_msgs
)

# =====================================================================
# C++ bridge manager library
# =====================================================================

add_library(${PROJECT_NAME}_bridge_mgr STATIC
  src/bridge/export_bridge_manager.cpp
  src/bridge/import_bridge_manager.cpp
)
target_include_directories(${PROJECT_NAME}_bridge_mgr PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_bridge_mgr
  ${PROJECT_NAME}_util
  ${PROJECT_NAME}_config
  ${PROJECT_NAME}_bridge
  ${PROJECT_NAME}_bluez
  ${PROJECT_NAME}_gatt
)
ament_target_dependencies(${PROJECT_NAME}_bridge_mgr
  rclcpp std_msgs nav_msgs rcpputils rosidl_runtime_cpp rosidl_typesupport_introspection_cpp
)

# =====================================================================
# Link generated message/service headers into our libraries
# =====================================================================
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(${PROJECT_NAME}_util ${cpp_typesupport_target})
target_link_libraries(${PROJECT_NAME}_config ${cpp_typesupport_target})
target_link_libraries(${PROJECT_NAME}_bridge ${cpp_typesupport_target})
target_link_libraries(${PROJECT_NAME}_bluez ${cpp_typesupport_target})
target_link_libraries(${PROJECT_NAME}_gatt ${cpp_typesupport_target})
target_link_libraries(${PROJECT_NAME}_network ${cpp_typesupport_target})
target_link_libraries(${PROJECT_NAME}_ros ${cpp_typesupport_target})
target_link_libraries(${PROJECT_NAME}_peer ${cpp_typesupport_target})
target_link_libraries(${PROJECT_NAME}_bridge_mgr ${cpp_typesupport_target})

# =====================================================================
# Executables (stubs during early migration)
# =====================================================================

add_executable(service_node
  src/app/main_service_node.cpp
  src/app/service_node.cpp
  src/app/service_node_cache_events.cpp
  src/app/service_node_notifications.cpp
  src/app/service_node_peer_orchestration.cpp
  src/app/service_node_status.cpp
  src/app/service_node_services.cpp
)
target_include_directories(service_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(service_node
  ${PROJECT_NAME}_util
  ${PROJECT_NAME}_config
  ${PROJECT_NAME}_bridge
  ${PROJECT_NAME}_bridge_mgr
  ${PROJECT_NAME}_bluez
  ${PROJECT_NAME}_gatt
  ${PROJECT_NAME}_network
  ${PROJECT_NAME}_ros
  ${PROJECT_NAME}_peer
  ${cpp_typesupport_target}
)
ament_target_dependencies(service_node
  rclcpp std_msgs std_srvs builtin_interfaces nav_msgs rcl_interfaces ament_index_cpp
)

add_executable(user_node
  src/app/main_user_node.cpp
  src/app/user_overlay_node.cpp
)
target_include_directories(user_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(user_node
  ${PROJECT_NAME}_util
  ${PROJECT_NAME}_config
  ${cpp_typesupport_target}
)
ament_target_dependencies(user_node
  rclcpp std_msgs std_srvs builtin_interfaces ament_index_cpp
)

add_executable(tui_node
  src/app/main_tui_node.cpp
  src/app/central_client_runtime.cpp
  src/app/raw_terminal.cpp
  src/app/tui_node.cpp
)
target_include_directories(tui_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(tui_node
  ${PROJECT_NAME}_util
  ${PROJECT_NAME}_bluez
  ${PROJECT_NAME}_gatt
  ${cpp_typesupport_target}
)
ament_target_dependencies(tui_node
  rclcpp
)

add_executable(test_node
  src/app/main_test_node.cpp
  src/app/test_node.cpp
)
target_include_directories(test_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(test_node
  ${PROJECT_NAME}_util
)
ament_target_dependencies(test_node
  rclcpp std_msgs nav_msgs
)

# =====================================================================
# Install targets
# =====================================================================

install(TARGETS service_node user_node tui_node test_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()