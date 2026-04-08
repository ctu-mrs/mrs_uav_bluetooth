// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/central_client_runtime.hpp"

#include <stdexcept>

namespace mrs_uav_bluetooth::app {

CentralClientRuntime::CentralClientRuntime(rclcpp::Logger logger,
                                           CentralClientRuntimeOptions options)
    : logger_(logger),
      options_(std::move(options)) {
    dbus_ = std::make_unique<bluez::DbusConnection>(logger_, "tui-client");
    adapter_path_ = dbus_->find_adapter_path();
    if (adapter_path_.empty()) {
        throw std::runtime_error("No local BlueZ adapter found");
    }

    cache_ = std::make_unique<bluez::ObjectManagerCache>(*dbus_, logger_);
    cache_->start();

    adapter_ = std::make_unique<bluez::AdapterController>(*dbus_, adapter_path_, logger_);
    adapter_->power_on();
    adapter_->set_connectable(true);
    adapter_->set_pairable(false);
    adapter_->set_pairable_timeout(0);
    if (!options_.adapter_alias.empty()) {
        adapter_->set_alias(options_.adapter_alias);
    }

    client_ = std::make_unique<bluez::BluezClient>(*dbus_, *cache_, adapter_path_, logger_);
    if (options_.scan_on_start) {
        scan_desired_ = true;
        client_->start_scan(options_.scan_mode);
    }
}

const std::string& CentralClientRuntime::adapter_path() const {
    return adapter_path_;
}

bool CentralClientRuntime::is_scanning() const {
    return client_ != nullptr && client_->is_scanning();
}

bool CentralClientRuntime::scan_desired() const {
    return scan_desired_;
}

void CentralClientRuntime::set_scan_enabled(bool enabled, const std::string& transport) {
    if (!client_) {
        return;
    }
    scan_desired_ = enabled;
    if (enabled) {
        client_->start_scan(transport.empty() ? options_.scan_mode : transport);
        return;
    }
    client_->stop_scan();
}

void CentralClientRuntime::refresh_scan(const std::string& transport) {
    if (!client_ || !scan_desired_) {
        return;
    }

    (void)client_->stop_scan();
    client_->start_scan(transport.empty() ? options_.scan_mode : transport);
}

bluez::ObjectManagerCache& CentralClientRuntime::cache() {
    return *cache_;
}

bluez::BluezClient& CentralClientRuntime::client() {
    return *client_;
}

}  // namespace mrs_uav_bluetooth::app