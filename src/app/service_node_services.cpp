// SPDX-License-Identifier: BSD-3-Clause
/// \file src/app/service_node_services.cpp
/// \brief Implements the service node services component of the ROS 2 application and operator-tool layer.

#include "mrs_uav_bluetooth/app/service_node.hpp"

#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <algorithm>

namespace mrs_uav_bluetooth::app {

namespace {

/// \brief Report whether GATT ownership currently prevents connectionless radio use.
/// \param config Mode and adapter policy used to decide whether connections may share the radio.
/// \return True if current mode reserves low-energy radio time for connectionless traffic; otherwise false.
bool connection_oriented_le_blocked(const config::NodeConfig& config) {
    // Report whether GATT ownership currently prevents connectionless radio use.
    return config.enable_mesh || config.advertise_mode == "broadcast";
}

constexpr const char* kExclusiveRadioMessage =
    "connection-oriented LE/GATT operations are unavailable while Mesh or "
    "broadcast advertisement mode owns the radio";

}  // namespace

void ServiceNode::handle_list_devices(const std::shared_ptr<mrs_uav_bluetooth::srv::ListDevices::Request> request,
                                      std::shared_ptr<mrs_uav_bluetooth::srv::ListDevices::Response> response) {
    // Snapshot cached peers and apply the request’s connected-only filter.
    const auto devices = request->connected_only ? client_->get_connected_devices() : client_->get_devices();
    response->success = true;
    response->message = "ok";
    for (const auto& device : devices) {
        response->devices.push_back(to_device_msg(device));
    }
}

void ServiceNode::handle_get_device(const std::shared_ptr<mrs_uav_bluetooth::srv::GetDevice::Request> request,
                                    std::shared_ptr<mrs_uav_bluetooth::srv::GetDevice::Response> response) {
    // Normalize the address and return its current cached Device1 snapshot.
    const auto device = client_->get_device(request->mac);
    if (!device) {
        response->success = false;
        response->message = "device not found";
        return;
    }
    response->success = true;
    response->message = "ok";
    response->device = to_device_msg(*device);
}

void ServiceNode::handle_connect_device(const std::shared_ptr<mrs_uav_bluetooth::srv::ConnectDevice::Request> request,
                                        std::shared_ptr<mrs_uav_bluetooth::srv::ConnectDevice::Response> response) {
    // Run the bounded peer connection outside duplicate service work.
    if (connection_oriented_le_blocked(active_config_)) {
        response->success = false;
        response->message = kExclusiveRadioMessage;
        response->resolved_mac = request->mac;
        return;
    }
    const double timeout_s = std::max(1.0f, request->timeout);
    bool success = client_->connect(request->mac, timeout_s);
    std::string detail = success ? "ok" : "failed to connect";
    if (success && request->wait_for_services) {
        success = client_->wait_services_resolved(request->mac, timeout_s);
        detail = success ? "ok" : "services unresolved";
    }
    const auto device = client_->get_device(request->mac);
    response->success = success;
    response->message = success ? (detail.empty() ? "ok" : detail) : (detail.empty() ? "failed" : detail);
    response->resolved_mac = device ? device->mac : request->mac;
    response->device_path = device ? device->object_path : std::string{};
}

void ServiceNode::handle_disconnect_device(const std::shared_ptr<mrs_uav_bluetooth::srv::DisconnectDevice::Request> request,
                                           std::shared_ptr<mrs_uav_bluetooth::srv::DisconnectDevice::Response> response) {
    // Cancel pending connection work and wait for the selected peer to disconnect.
    response->success = client_->disconnect(request->mac, std::max(1.0f, request->timeout));
    response->message = response->success ? "ok" : "failed to disconnect";
}

void ServiceNode::handle_pair_device(const std::shared_ptr<mrs_uav_bluetooth::srv::PairDevice::Request> request,
                                     std::shared_ptr<mrs_uav_bluetooth::srv::PairDevice::Response> response) {
    // Start pairing with the requested timeout and preserve BlueZ failure detail.
    if (connection_oriented_le_blocked(active_config_)) {
        response->success = false;
        response->message = kExclusiveRadioMessage;
        return;
    }
    std::string detail;
    bool success = client_->pair(request->mac, std::max(1.0f, request->timeout), &detail);
    detail = success ? "ok" : (detail.empty() ? "failed to pair" : detail);
    if (success && request->trust_after_pair) {
        success = client_->trust(request->mac);
        if (!success) {
            detail = "trust_after_pair failed";
        }
    }
    response->success = success;
    response->message = success ? (detail.empty() ? "ok" : detail) : (detail.empty() ? "failed" : detail);
}

void ServiceNode::handle_set_device_trust(const std::shared_ptr<mrs_uav_bluetooth::srv::SetDeviceTrust::Request> request,
                                          std::shared_ptr<mrs_uav_bluetooth::srv::SetDeviceTrust::Response> response) {
    // Set or clear the selected peer’s persistent BlueZ Trusted property.
    response->success = request->trusted ? client_->trust(request->mac) : client_->untrust(request->mac);
    response->message = response->success ? "ok" : "failed";
}

void ServiceNode::handle_remove_device(const std::shared_ptr<mrs_uav_bluetooth::srv::RemoveDevice::Request> request,
                                       std::shared_ptr<mrs_uav_bluetooth::srv::RemoveDevice::Response> response) {
    // Remove the selected Device1 object and its stored security material.
    response->success = client_->remove(request->mac);
    response->message = response->success ? "ok" : "failed";
}

void ServiceNode::handle_list_gatt_services(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattServices::Request> request,
                                            std::shared_ptr<mrs_uav_bluetooth::srv::ListGattServices::Response> response) {
    // Return cached services belonging to the normalized peer address.
    if (connection_oriented_le_blocked(active_config_)) {
        response->success = false;
        response->message = kExclusiveRadioMessage;
        return;
    }
    response->success = true;
    response->message = "ok";
    for (const auto& item : client_->list_services(request->mac)) {
        response->services.push_back(to_service_msg(item));
    }
}

void ServiceNode::handle_list_gatt_characteristics(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattCharacteristics::Request> request,
                                                   std::shared_ptr<mrs_uav_bluetooth::srv::ListGattCharacteristics::Response> response) {
    // Return cached characteristics below the requested service path.
    if (connection_oriented_le_blocked(active_config_)) {
        response->success = false;
        response->message = kExclusiveRadioMessage;
        return;
    }
    response->success = true;
    response->message = "ok";
    for (const auto& item : client_->list_characteristics(request->mac)) {
        response->characteristics.push_back(to_characteristic_msg(item));
    }
}

void ServiceNode::handle_list_gatt_descriptors(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattDescriptors::Request> request,
                                               std::shared_ptr<mrs_uav_bluetooth::srv::ListGattDescriptors::Response> response) {
    // Return cached descriptors below the requested characteristic path.
    if (connection_oriented_le_blocked(active_config_)) {
        response->success = false;
        response->message = kExclusiveRadioMessage;
        return;
    }
    response->success = true;
    response->message = "ok";
    for (const auto& item : client_->list_descriptors(request->mac, request->characteristic_path)) {
        response->descriptors.push_back(to_descriptor_msg(item));
    }
}

void ServiceNode::handle_find_gatt_path(const std::shared_ptr<mrs_uav_bluetooth::srv::FindGattPath::Request> request,
                                        std::shared_ptr<mrs_uav_bluetooth::srv::FindGattPath::Response> response) {
    // Resolve the requested service, characteristic, or descriptor UUID path.
    if (connection_oriented_le_blocked(active_config_)) {
        response->success = false;
        response->message = kExclusiveRadioMessage;
        return;
    }
    const auto uuid = util::resolve_uuid(request->uuid);
    const std::string path = request->descriptor
        ? client_->find_descriptor(request->mac, uuid, request->characteristic_path)
        : client_->find_characteristic(request->mac, uuid);
    response->success = !path.empty();
    response->message = response->success ? "ok" : "not found";
    response->path = path;
}

void ServiceNode::handle_read_gatt_value(const std::shared_ptr<mrs_uav_bluetooth::srv::ReadGattValue::Request> request,
                                         std::shared_ptr<mrs_uav_bluetooth::srv::ReadGattValue::Response> response) {
    // Read the requested characteristic or descriptor and return its raw bytes.
    if (connection_oriented_le_blocked(active_config_)) {
        response->success = false;
        response->message = kExclusiveRadioMessage;
        return;
    }
    const auto data = request->descriptor ? client_->read_descriptor(request->path)
                                          : client_->read_characteristic(request->path);
    response->success = true;
    response->message = "ok";
    response->value = data;
}

void ServiceNode::handle_write_gatt_value(const std::shared_ptr<mrs_uav_bluetooth::srv::WriteGattValue::Request> request,
                                          std::shared_ptr<mrs_uav_bluetooth::srv::WriteGattValue::Response> response) {
    // Write raw bytes to the requested characteristic or descriptor path.
    if (connection_oriented_le_blocked(active_config_)) {
        response->success = false;
        response->message = kExclusiveRadioMessage;
        return;
    }
    const bool success = request->descriptor
        ? client_->write_descriptor(request->path, request->value)
        : client_->write_characteristic(request->path, request->value, request->with_response);
    response->success = success;
    response->message = success ? "ok" : "failed";
}

void ServiceNode::handle_set_notify(const std::shared_ptr<mrs_uav_bluetooth::srv::SetNotify::Request> request,
                                    std::shared_ptr<mrs_uav_bluetooth::srv::SetNotify::Response> response) {
    // Start or stop notification ownership for the requested characteristic.
    if (connection_oriented_le_blocked(active_config_)) {
        response->success = false;
        response->message = kExclusiveRadioMessage;
        return;
    }
    response->success = request->enable ? client_->start_notify(request->path) : client_->stop_notify(request->path);
    response->message = response->success ? "ok" : "failed";
}

void ServiceNode::handle_set_scan_enabled(const std::shared_ptr<mrs_uav_bluetooth::srv::SetScanEnabled::Request> request,
                                          std::shared_ptr<mrs_uav_bluetooth::srv::SetScanEnabled::Response> response) {
    // Persist the requested discovery state and reconcile adapter scanning.
    const bool discoverable_while_scanning =
        active_config_.advertise_mode != "broadcast" &&
        active_config_.advertise_discoverable.value_or(
            active_config_.enable_server ||
            active_config_.enable_serial_port_profile);
    const bool success = request->enabled
        ? client_->start_scan(
              request->transport.empty() ? active_config_.scan_mode
                                         : request->transport,
              discoverable_while_scanning)
        : client_->stop_scan();
    response->success = success;
    response->message = success ? "ok" : "failed";
    response->scanning = client_->is_scanning();
}

void ServiceNode::handle_reload_config(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // Reload defaults plus the active overlay as one serialized configuration transaction.
    (void)request;
    const auto [success, message] = overlay_config_->reload();
    response->success = success;
    response->message = message;
}

void ServiceNode::handle_set_active_config(const std::shared_ptr<mrs_uav_bluetooth::srv::SetActiveConfig::Request> request,
                                           std::shared_ptr<mrs_uav_bluetooth::srv::SetActiveConfig::Response> response) {
    // Activate or clear an overlay lease and return the resulting source path.
    std::pair<bool, std::string> result;
    if (request->config_path.empty()) {
        result = overlay_config_->revert_to_default();
    } else {
        result = overlay_config_->activate_overlay(request->config_path, request->hold_seconds);
    }
    response->success = result.first;
    response->message = result.second;
    response->active_config_path = overlay_config_->active_source();
    response->overlay_active = overlay_config_->overlay_active();
}

}  // namespace mrs_uav_bluetooth::app
