// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/service_node.hpp"

#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <algorithm>

namespace mrs_uav_bluetooth::app {

void ServiceNode::handle_list_devices(const std::shared_ptr<mrs_uav_bluetooth::srv::ListDevices::Request> request,
                                      std::shared_ptr<mrs_uav_bluetooth::srv::ListDevices::Response> response) {
    const auto devices = request->connected_only ? client_->get_connected_devices() : client_->get_devices();
    response->success = true;
    response->message = "ok";
    for (const auto& device : devices) {
        response->devices.push_back(to_device_msg(device));
    }
}

void ServiceNode::handle_get_device(const std::shared_ptr<mrs_uav_bluetooth::srv::GetDevice::Request> request,
                                    std::shared_ptr<mrs_uav_bluetooth::srv::GetDevice::Response> response) {
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
    response->success = client_->disconnect(request->mac, std::max(1.0f, request->timeout));
    response->message = response->success ? "ok" : "failed to disconnect";
}

void ServiceNode::handle_pair_device(const std::shared_ptr<mrs_uav_bluetooth::srv::PairDevice::Request> request,
                                     std::shared_ptr<mrs_uav_bluetooth::srv::PairDevice::Response> response) {
    std::string detail;
    bool success = client_->pair(request->mac, std::max(1.0f, request->timeout));
    detail = success ? "ok" : "failed to pair";
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
    response->success = request->trusted ? client_->trust(request->mac) : client_->untrust(request->mac);
    response->message = response->success ? "ok" : "failed";
}

void ServiceNode::handle_remove_device(const std::shared_ptr<mrs_uav_bluetooth::srv::RemoveDevice::Request> request,
                                       std::shared_ptr<mrs_uav_bluetooth::srv::RemoveDevice::Response> response) {
    response->success = client_->remove(request->mac);
    response->message = response->success ? "ok" : "failed";
}

void ServiceNode::handle_list_gatt_services(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattServices::Request> request,
                                            std::shared_ptr<mrs_uav_bluetooth::srv::ListGattServices::Response> response) {
    response->success = true;
    response->message = "ok";
    for (const auto& item : client_->list_services(request->mac)) {
        response->services.push_back(to_service_msg(item));
    }
}

void ServiceNode::handle_list_gatt_characteristics(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattCharacteristics::Request> request,
                                                   std::shared_ptr<mrs_uav_bluetooth::srv::ListGattCharacteristics::Response> response) {
    response->success = true;
    response->message = "ok";
    for (const auto& item : client_->list_characteristics(request->mac)) {
        response->characteristics.push_back(to_characteristic_msg(item));
    }
}

void ServiceNode::handle_list_gatt_descriptors(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattDescriptors::Request> request,
                                               std::shared_ptr<mrs_uav_bluetooth::srv::ListGattDescriptors::Response> response) {
    response->success = true;
    response->message = "ok";
    for (const auto& item : client_->list_descriptors(request->mac, request->characteristic_path)) {
        response->descriptors.push_back(to_descriptor_msg(item));
    }
}

void ServiceNode::handle_find_gatt_path(const std::shared_ptr<mrs_uav_bluetooth::srv::FindGattPath::Request> request,
                                        std::shared_ptr<mrs_uav_bluetooth::srv::FindGattPath::Response> response) {
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
    const auto data = request->descriptor ? client_->read_descriptor(request->path)
                                          : client_->read_characteristic(request->path);
    response->success = true;
    response->message = "ok";
    response->value = data;
}

void ServiceNode::handle_write_gatt_value(const std::shared_ptr<mrs_uav_bluetooth::srv::WriteGattValue::Request> request,
                                          std::shared_ptr<mrs_uav_bluetooth::srv::WriteGattValue::Response> response) {
    const bool success = request->descriptor
        ? client_->write_descriptor(request->path, request->value)
        : client_->write_characteristic(request->path, request->value, request->with_response);
    response->success = success;
    response->message = success ? "ok" : "failed";
}

void ServiceNode::handle_set_notify(const std::shared_ptr<mrs_uav_bluetooth::srv::SetNotify::Request> request,
                                    std::shared_ptr<mrs_uav_bluetooth::srv::SetNotify::Response> response) {
    response->success = request->enable ? client_->start_notify(request->path) : client_->stop_notify(request->path);
    response->message = response->success ? "ok" : "failed";
}

void ServiceNode::handle_set_scan_enabled(const std::shared_ptr<mrs_uav_bluetooth::srv::SetScanEnabled::Request> request,
                                          std::shared_ptr<mrs_uav_bluetooth::srv::SetScanEnabled::Response> response) {
    const bool success = request->enabled
        ? client_->start_scan(request->transport.empty() ? active_config_.scan_mode : request->transport)
        : client_->stop_scan();
    response->success = success;
    response->message = success ? "ok" : "failed";
    response->scanning = client_->is_scanning();
}

void ServiceNode::handle_reload_config(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    const auto [success, message] = overlay_config_->reload();
    response->success = success;
    response->message = message;
}

void ServiceNode::handle_set_active_config(const std::shared_ptr<mrs_uav_bluetooth::srv::SetActiveConfig::Request> request,
                                           std::shared_ptr<mrs_uav_bluetooth::srv::SetActiveConfig::Response> response) {
    std::pair<bool, std::string> result;
    if (request->config_path.empty()) {
        result = overlay_config_->revert_to_default();
    } else {
        result = overlay_config_->activate_overlay(request->config_path);
    }
    response->success = result.first;
    response->message = result.second;
    response->active_config_path = overlay_config_->active_source();
    response->overlay_active = overlay_config_->overlay_active();
}

}  // namespace mrs_uav_bluetooth::app