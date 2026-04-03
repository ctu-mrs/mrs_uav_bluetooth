// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/tui_node.hpp"

#include "mrs_uav_bluetooth/gatt/builtin_gatt.hpp"
#include "mrs_uav_bluetooth/util/string_utils.hpp"
#include "mrs_uav_bluetooth/util/device_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <future>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace mrs_uav_bluetooth::app {

namespace {

std::string yes_no(bool value) {
    return value ? "yes" : "no";
}

uint64_t wall_time_ns() {
    const auto now = std::chrono::time_point_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now());
    return static_cast<uint64_t>(now.time_since_epoch().count());
}

std::string shorten(std::string value, size_t width) {
    if (value.size() <= width) {
        return value;
    }
    if (width <= 1) {
        return value.substr(0, width);
    }
    return value.substr(0, width - 1) + "~";
}

std::string pad_right(std::string value, size_t width) {
    if (value.size() < width) {
        value.append(width - value.size(), ' ');
    }
    return value;
}

std::string frame_line(std::string value, size_t width) {
    return pad_right(shorten(std::move(value), width), width) + "\n";
}

std::string format_remote_time(uint64_t remote_time_ns) {
    if (remote_time_ns == 0) {
        return "-";
    }
    const auto seconds = static_cast<std::time_t>(remote_time_ns / 1000000000ULL);
    std::tm tm{};
    localtime_r(&seconds, &tm);
    std::ostringstream out;
    out << std::put_time(&tm, "%F %T");
    return out.str();
}

}  // namespace

TuiNode::TuiNode()
    : rclcpp::Node("mrs_uav_bluetooth_tui") {
    configure_parameters();

    CentralClientRuntimeOptions options;
    options.adapter_alias = adapter_alias_;
    options.scan_mode = scan_mode_;
    options.scan_on_start = true;
    runtime_ = std::make_unique<CentralClientRuntime>(get_logger(), std::move(options));
    inspector_ = std::make_unique<gatt::RemoteGattInspector>(runtime_->client());
    notification_token_ = runtime_->client().add_notification_handler(
        [this](const std::vector<uint8_t>& data,
               const std::string& uuid,
               const std::string& characteristic_path) {
            on_notification(data, uuid, characteristic_path);
        });
    status_message_ = "ready";

    ui_timer_ = create_wall_timer(
        std::chrono::duration<double>(render_period_sec_),
        [this]() { ui_tick(); });
}

TuiNode::~TuiNode() {
    if (notification_token_ != 0 && runtime_) {
        runtime_->client().remove_notification_handler(notification_token_);
    }
    for (auto& [_, entry] : topic_counts_) {
        if (entry.pending && entry.future.valid()) {
            entry.future.wait();
        }
    }
    for (auto& [_, entry] : time_samples_) {
        if (entry.pending && entry.future.valid()) {
            entry.future.wait();
        }
    }
    for (auto& [_, entry] : wifi_state_) {
        if (entry.pending && entry.future.valid()) {
            entry.future.wait();
        }
    }
    if (action_future_.valid()) {
        action_future_.wait();
    }
}

void TuiNode::configure_parameters() {
    declare_parameter<std::string>("adapter_alias", "");
    declare_parameter<std::string>("scan_mode", "le");
    declare_parameter<std::string>("uav_name_pattern", "^uav[0-9]{2}$");
    declare_parameter<double>("refresh_period_sec", 1.0);
    declare_parameter<double>("render_period_sec", 0.1);
    declare_parameter<double>("topic_count_refresh_sec", 5.0);
    declare_parameter<bool>("hide_non_uav", false);

    adapter_alias_ = get_parameter("adapter_alias").as_string();
    scan_mode_ = get_parameter("scan_mode").as_string();
    uav_name_pattern_ = get_parameter("uav_name_pattern").as_string();
    refresh_period_sec_ = std::max(0.2, get_parameter("refresh_period_sec").as_double());
    render_period_sec_ = std::max(0.05, get_parameter("render_period_sec").as_double());
    topic_count_refresh_sec_ = std::max(1.0, get_parameter("topic_count_refresh_sec").as_double());
    hide_non_uav_ = get_parameter("hide_non_uav").as_bool();
}

void TuiNode::ui_tick() {
    handle_input();
    collect_action_result();
    refresh_device_cache();
    ensure_builtin_subscriptions();
    apply_pending_notifications();
    collect_topic_count_results();
    collect_time_sample_results();
    collect_wifi_results();

    for (const auto& device : current_devices_) {
        request_topic_count_refresh(device);
    }

    if (const auto* device = selected_device()) {
        request_wifi_refresh(*device);
    }

    render_dashboard(current_devices_);
}

void TuiNode::handle_input() {
    while (true) {
        const auto event = terminal_.read_key();
        if (!event.has_value()) {
            return;
        }
        handle_key_event(*event);
    }
}

void TuiNode::handle_key_event(const TerminalKeyEvent& event) {
    if (prompt_mode_ != PromptMode::None) {
        if (event.kind == TerminalKeyKind::Escape) {
            prompt_mode_ = PromptMode::None;
            prompt_buffer_.clear();
            status_message_ = "input cancelled";
            return;
        }
        if (event.kind == TerminalKeyKind::Backspace) {
            if (!prompt_buffer_.empty()) {
                prompt_buffer_.pop_back();
            }
            return;
        }
        if (event.kind == TerminalKeyKind::Enter) {
            trigger_wifi_write(prompt_mode_, prompt_buffer_);
            prompt_mode_ = PromptMode::None;
            prompt_buffer_.clear();
            return;
        }
        if (event.kind == TerminalKeyKind::Character) {
            prompt_buffer_.push_back(event.ch);
        }
        return;
    }

    switch (event.kind) {
    case TerminalKeyKind::Up:
        move_selection(-1);
        break;
    case TerminalKeyKind::Down:
        move_selection(1);
        break;
    case TerminalKeyKind::Character:
        switch (event.ch) {
        case 'k':
            move_selection(-1);
            break;
        case 'j':
            move_selection(1);
            break;
        case 's':
            trigger_scan_toggle();
            break;
        case 'c':
            trigger_connect_toggle();
            break;
        case 't':
            if (const auto* device = selected_device()) {
                request_time_sample(*device, true);
            }
            break;
        case 'w':
            if (const auto* device = selected_device()) {
                request_wifi_refresh(*device, true);
            }
            break;
        case 'i':
            if (selected_device() != nullptr) {
                prompt_mode_ = PromptMode::WifiSsid;
                prompt_buffer_.clear();
                status_message_ = "enter wifi ssid and press Enter";
            }
            break;
        case 'p':
            if (selected_device() != nullptr) {
                prompt_mode_ = PromptMode::WifiPassword;
                prompt_buffer_.clear();
                status_message_ = "enter wifi password and press Enter";
            }
            break;
        case 'f':
            hide_non_uav_ = !hide_non_uav_;
            select_first_visible_device();
            status_message_ = hide_non_uav_ ? "filtering UAVs only" : "showing all devices";
            break;
        case 'r':
            status_message_ = "refresh requested";
            runtime_->refresh_scan(scan_mode_);
            refresh_device_cache(true);
            last_frame_.clear();
            break;
        case 'q':
            rclcpp::shutdown();
            break;
        default:
            break;
        }
        break;
    case TerminalKeyKind::Escape:
        rclcpp::shutdown();
        break;
    default:
        break;
    }
}

void TuiNode::move_selection(int delta) {
    const auto devices = visible_devices();
    if (devices.empty()) {
        selected_mac_.clear();
        return;
    }

    size_t index = 0;
    for (size_t i = 0; i < devices.size(); ++i) {
        if (devices[i]->mac == selected_mac_) {
            index = i;
            break;
        }
    }

    const auto next = static_cast<int>(index) + delta;
    if (next < 0) {
        selected_mac_ = devices.front()->mac;
    } else if (static_cast<size_t>(next) >= devices.size()) {
        selected_mac_ = devices.back()->mac;
    } else {
        selected_mac_ = devices[static_cast<size_t>(next)]->mac;
    }
}

void TuiNode::select_first_visible_device() {
    const auto devices = visible_devices();
    selected_mac_ = devices.empty() ? std::string{} : devices.front()->mac;
}

void TuiNode::refresh_device_cache(bool force) {
    const auto now = std::chrono::steady_clock::now();
    if (!force && last_device_refresh_ != std::chrono::steady_clock::time_point{} &&
        (now - last_device_refresh_) < std::chrono::duration<double>(refresh_period_sec_)) {
        return;
    }

    current_devices_ = runtime_->client().get_devices();
    std::sort(current_devices_.begin(), current_devices_.end(), [this](const auto& lhs, const auto& rhs) {
        const bool lhs_is_uav = !util::device_hostname_guess(lhs, uav_name_pattern_).empty();
        const bool rhs_is_uav = !util::device_hostname_guess(rhs, uav_name_pattern_).empty();
        if (lhs_is_uav != rhs_is_uav) {
            return lhs_is_uav > rhs_is_uav;
        }
        const auto lhs_name = util::device_hostname_guess(lhs, uav_name_pattern_).empty()
            ? util::device_display_name(lhs)
            : util::device_hostname_guess(lhs, uav_name_pattern_);
        const auto rhs_name = util::device_hostname_guess(rhs, uav_name_pattern_).empty()
            ? util::device_display_name(rhs)
            : util::device_hostname_guess(rhs, uav_name_pattern_);
        if (lhs_name != rhs_name) {
            return lhs_name < rhs_name;
        }
        if (lhs.connected != rhs.connected) {
            return lhs.connected > rhs.connected;
        }
        if (lhs.rssi != rhs.rssi) {
            return lhs.rssi > rhs.rssi;
        }
        return lhs.mac < rhs.mac;
    });

    const auto it = std::find_if(current_devices_.begin(), current_devices_.end(), [&](const auto& device) {
        return device.mac == selected_mac_;
    });
    if (it == current_devices_.end()) {
        select_first_visible_device();
    } else if (!it->connected || !it->services_resolved) {
        auto time_it = time_samples_.find(it->mac);
        if (time_it != time_samples_.end()) {
            time_it->second.available = false;
            time_it->second.pending = false;
            time_it->second.metrics_available = false;
            time_it->second.error = !it->connected ? "device not connected" : "services not resolved";
        }
        auto wifi_it = wifi_state_.find(it->mac);
        if (wifi_it != wifi_state_.end()) {
            wifi_it->second.available = false;
            wifi_it->second.pending = false;
            wifi_it->second.config_known = false;
            wifi_it->second.error = !it->connected ? "device not connected" : "services not resolved";
        }
    }
    last_device_refresh_ = now;
}

void TuiNode::apply_pending_notifications() {
    std::vector<PendingNotification> pending;
    {
        std::lock_guard<std::mutex> lock(pending_notification_mutex_);
        pending.swap(pending_notifications_);
    }

    if (pending.empty()) {
        return;
    }

    for (const auto& notification : pending) {
        if (notification.kind == NotificationKind::Time) {
            auto& entry = time_samples_[notification.mac];
            entry.available = true;
            entry.pending = false;
            entry.remote_time_ns = notification.remote_time_ns;
            entry.error.clear();
            if (!entry.metrics_available) {
                entry.rtt_ms = 0.0;
                entry.offset_ms = 0.0;
            }
            continue;
        }

        auto& entry = wifi_state_[notification.mac];
        entry.available = true;
        entry.pending = false;
        entry.status = notification.wifi_status;
        entry.error.clear();
    }
}

void TuiNode::collect_topic_count_results() {
    for (auto& [_, entry] : topic_counts_) {
        if (!entry.pending || !entry.future.valid()) {
            continue;
        }
        if (entry.future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            continue;
        }
        const auto result = entry.future.get();
        entry.count = result.discovered_topics;
        entry.available = true;
        entry.pending = false;
        entry.last_refresh = std::chrono::steady_clock::now();
    }
}

void TuiNode::collect_time_sample_results() {
    for (auto& [_, entry] : time_samples_) {
        if (!entry.pending || !entry.future.valid()) {
            continue;
        }
        if (entry.future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            continue;
        }
        const auto result = entry.future.get();
        entry.remote_time_ns = std::get<0>(result);
        entry.rtt_ms = std::get<1>(result);
        entry.offset_ms = std::get<2>(result);
        entry.error = std::get<3>(result);
        entry.available = entry.error.empty();
        entry.metrics_available = entry.error.empty();
        entry.pending = false;
    }
}

void TuiNode::collect_wifi_results() {
    for (auto& [_, entry] : wifi_state_) {
        if (!entry.pending || !entry.future.valid()) {
            continue;
        }
        if (entry.future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            continue;
        }
        const auto result = entry.future.get();
        entry.ssid = std::get<0>(result);
        entry.status = std::get<1>(result);
        entry.password_configured = std::get<2>(result);
        entry.error = std::get<3>(result);
        entry.available = entry.error.empty();
        entry.config_known = entry.error.empty();
        entry.pending = false;
    }
}

void TuiNode::collect_action_result() {
    if (!action_future_.valid()) {
        return;
    }
    if (action_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;
    }
    status_message_ = action_future_.get();
    action_future_ = {};
    action_label_.clear();
    runtime_->refresh_scan(scan_mode_);
    refresh_device_cache(true);
    last_frame_.clear();
}

void TuiNode::request_topic_count_refresh(const bluez::DeviceInfo& device) {
    auto& entry = topic_counts_[device.mac];
    if (!device.connected || !device.services_resolved) {
        entry.pending = false;
        entry.available = false;
        return;
    }

    const auto refresh_age = std::chrono::steady_clock::now() - entry.last_refresh;
    if (entry.pending ||
        (entry.available && refresh_age < std::chrono::duration<double>(topic_count_refresh_sec_))) {
        return;
    }

    const auto mac = device.mac;
    entry.pending = true;
    entry.future = std::async(std::launch::async, [this, mac]() {
        return inspector_->count_exported_topics(mac);
    }).share();
}

void TuiNode::request_time_sample(const bluez::DeviceInfo& device, bool force) {
    auto& entry = time_samples_[device.mac];
    if (!device.connected || !device.services_resolved) {
        entry.available = false;
        entry.pending = false;
        entry.error = "device not connected";
        return;
    }
    if (entry.pending || (entry.available && !force)) {
        return;
    }

    const auto mac = device.mac;
    entry.pending = true;
    entry.future = std::async(std::launch::async, [this, mac]() {
        try {
            const auto paths = inspector_->resolve_builtin_paths(mac);
            if (paths.time_characteristic_path.empty()) {
                return std::make_tuple(uint64_t{0}, 0.0, 0.0, std::string{"time service not found"});
            }

            const auto before_ns = wall_time_ns();
            const auto payload = runtime_->client().read_characteristic(paths.time_characteristic_path);
            const auto after_ns = wall_time_ns();
            if (payload.size() != sizeof(uint64_t)) {
                return std::make_tuple(uint64_t{0}, 0.0, 0.0, std::string{"unexpected time payload size"});
            }

            uint64_t remote_time_ns = 0;
            std::memcpy(&remote_time_ns, payload.data(), sizeof(remote_time_ns));
            const auto midpoint_ns = before_ns + ((after_ns - before_ns) / 2ULL);
            const auto rtt_ms = static_cast<double>(after_ns - before_ns) / 1000000.0;
            const auto offset_ms =
                (static_cast<double>(remote_time_ns) - static_cast<double>(midpoint_ns)) / 1000000.0;
            return std::make_tuple(remote_time_ns, rtt_ms, offset_ms, std::string{});
        } catch (const std::exception& e) {
            return std::make_tuple(uint64_t{0}, 0.0, 0.0, std::string{e.what()});
        }
    }).share();
}

void TuiNode::request_wifi_refresh(const bluez::DeviceInfo& device, bool force) {
    auto& entry = wifi_state_[device.mac];
    if (!device.connected || !device.services_resolved) {
        entry.available = false;
        entry.pending = false;
        entry.error = "device not connected";
        return;
    }
    if (entry.pending || (entry.available && !force)) {
        return;
    }

    const auto mac = device.mac;
    entry.pending = true;
    entry.future = std::async(std::launch::async, [this, mac]() {
        try {
            const auto paths = inspector_->resolve_builtin_paths(mac);
            if (!paths.has_wifi()) {
                return std::make_tuple(std::string{}, std::string{}, false, std::string{"wifi service not found"});
            }

            const auto to_text = [](const std::vector<uint8_t>& payload) {
                return std::string(payload.begin(), payload.end());
            };
            const auto ssid = to_text(runtime_->client().read_characteristic(paths.wifi_ssid_characteristic_path));
            const auto password = to_text(runtime_->client().read_characteristic(paths.wifi_password_characteristic_path));
            const auto status = to_text(runtime_->client().read_characteristic(paths.wifi_status_characteristic_path));
            return std::make_tuple(ssid, status, !password.empty(), std::string{});
        } catch (const std::exception& e) {
            return std::make_tuple(std::string{}, std::string{}, false, std::string{e.what()});
        }
    }).share();
}

void TuiNode::trigger_connect_toggle() {
    const auto* device = selected_device();
    if (device == nullptr || action_future_.valid()) {
        return;
    }

    const auto mac = device->mac;
    const bool connected = device->connected;
    action_label_ = connected ? "disconnect" : "connect";
    status_message_ = action_label_ + " requested for " + mac;
    action_future_ = std::async(std::launch::async, [this, mac, connected]() {
        try {
            if (connected) {
                const bool ok = runtime_->client().disconnect(mac, 10.0);
                return ok ? std::string{"disconnected "} + mac
                          : std::string{"disconnect failed for "} + mac;
            }

            const bool connected_ok = runtime_->client().connect(mac, 15.0);
            if (!connected_ok) {
                return std::string{"connect failed for "} + mac;
            }

            const bool services_ok = runtime_->client().wait_services_resolved(mac, 15.0);
            if (services_ok) {
                return std::string{"connected "} + mac;
            }

            const auto device = runtime_->client().get_device(mac);
            if (device && device->connected) {
                return std::string{"connected but services unresolved for "} + mac;
            }
            return std::string{"connection dropped before services resolved for "} + mac;
        } catch (const std::exception& e) {
            return std::string{"connect action failed: "} + e.what();
        }
    }).share();
}

void TuiNode::trigger_scan_toggle() {
    runtime_->set_scan_enabled(!runtime_->is_scanning(), scan_mode_);
    status_message_ = runtime_->is_scanning() ? "scan enabled" : "scan stopped";
}

void TuiNode::trigger_wifi_write(PromptMode mode, std::string value) {
    const auto* device = selected_device();
    if (device == nullptr || action_future_.valid()) {
        return;
    }

    const auto mac = device->mac;
    action_label_ = mode == PromptMode::WifiSsid ? "set ssid" : "set password";
    status_message_ = action_label_ + " requested for " + mac;
    wifi_state_[mac].available = false;

    action_future_ = std::async(std::launch::async, [this, mac, mode, value = std::move(value)]() {
        try {
            const auto paths = inspector_->resolve_builtin_paths(mac);
            if (!paths.has_wifi()) {
                return std::string{"wifi service not found"};
            }

            const std::vector<uint8_t> payload(value.begin(), value.end());
            const auto& target_path = mode == PromptMode::WifiSsid
                ? paths.wifi_ssid_characteristic_path
                : paths.wifi_password_characteristic_path;
            const bool ok = runtime_->client().write_characteristic(target_path, payload, true);
            if (!ok) {
                return std::string{"wifi write failed"};
            }
            const auto status_payload = runtime_->client().read_characteristic(paths.wifi_status_characteristic_path);
            return std::string(status_payload.begin(), status_payload.end());
        } catch (const std::exception& e) {
            return std::string{"wifi write failed: "} + e.what();
        }
    }).share();
}

const bluez::DeviceInfo* TuiNode::selected_device() const {
    const auto it = std::find_if(current_devices_.begin(), current_devices_.end(), [&](const auto& device) {
        return device.mac == selected_mac_;
    });
    return it == current_devices_.end() ? nullptr : &(*it);
}

std::vector<const bluez::DeviceInfo*> TuiNode::visible_devices() const {
    std::vector<const bluez::DeviceInfo*> result;
    const auto uavs = visible_uav_devices();
    result.insert(result.end(), uavs.begin(), uavs.end());
    const auto others = visible_other_devices();
    result.insert(result.end(), others.begin(), others.end());
    return result;
}

std::vector<const bluez::DeviceInfo*> TuiNode::visible_uav_devices() const {
    std::vector<const bluez::DeviceInfo*> result;
    for (const auto& device : current_devices_) {
        const auto guessed = util::device_hostname_guess(device, uav_name_pattern_);
        if (guessed.empty()) {
            continue;
        }
        result.push_back(&device);
    }
    return result;
}

std::vector<const bluez::DeviceInfo*> TuiNode::visible_other_devices() const {
    std::vector<const bluez::DeviceInfo*> result;
    if (hide_non_uav_) {
        return result;
    }
    for (const auto& device : current_devices_) {
        if (!util::device_hostname_guess(device, uav_name_pattern_).empty()) {
            continue;
        }
        result.push_back(&device);
    }
    return result;
}

void TuiNode::ensure_builtin_subscriptions() {
    std::vector<std::string> active_macs;
    active_macs.reserve(current_devices_.size());

    for (const auto& device : current_devices_) {
        if (util::device_hostname_guess(device, uav_name_pattern_).empty()) {
            continue;
        }
        if (!device.connected || !device.services_resolved) {
            continue;
        }
        active_macs.push_back(device.mac);
        ensure_builtin_subscription(device);
    }

    for (auto it = builtin_subscriptions_.begin(); it != builtin_subscriptions_.end();) {
        if (std::find(active_macs.begin(), active_macs.end(), it->first) != active_macs.end()) {
            ++it;
            continue;
        }

        if (!it->second.paths.time_characteristic_path.empty()) {
            runtime_->client().stop_notify(it->second.paths.time_characteristic_path);
        }
        if (!it->second.paths.wifi_status_characteristic_path.empty()) {
            runtime_->client().stop_notify(it->second.paths.wifi_status_characteristic_path);
        }
        it = builtin_subscriptions_.erase(it);
    }
}

void TuiNode::ensure_builtin_subscription(const bluez::DeviceInfo& device) {
    gatt::RemoteBuiltinPaths resolved_paths;
    try {
        resolved_paths = inspector_->resolve_builtin_paths(device.mac);
    } catch (...) {
        return;
    }

    auto& subscription = builtin_subscriptions_[device.mac];
    if (!subscription.paths.time_characteristic_path.empty() &&
        subscription.paths.time_characteristic_path != resolved_paths.time_characteristic_path) {
        runtime_->client().stop_notify(subscription.paths.time_characteristic_path);
    }
    if (!subscription.paths.wifi_status_characteristic_path.empty() &&
        subscription.paths.wifi_status_characteristic_path != resolved_paths.wifi_status_characteristic_path) {
        runtime_->client().stop_notify(subscription.paths.wifi_status_characteristic_path);
    }
    subscription.paths = resolved_paths;

    if (!subscription.paths.time_characteristic_path.empty() &&
        !runtime_->client().is_notify_active(subscription.paths.time_characteristic_path)) {
        runtime_->client().start_notify(subscription.paths.time_characteristic_path);
    }
    if (!subscription.paths.wifi_status_characteristic_path.empty() &&
        !runtime_->client().is_notify_active(subscription.paths.wifi_status_characteristic_path)) {
        runtime_->client().start_notify(subscription.paths.wifi_status_characteristic_path);
    }
    if (subscription.paths.has_wifi() && !subscription.wifi_seeded) {
        request_wifi_refresh(device, true);
        subscription.wifi_seeded = true;
    }
}

void TuiNode::on_notification(const std::vector<uint8_t>& data,
                              const std::string& uuid,
                              const std::string& characteristic_path) {
    std::string mac;
    if (const auto characteristic = runtime_->cache().characteristic(characteristic_path)) {
        if (const auto service = runtime_->cache().service(characteristic->service_path)) {
            if (const auto device = runtime_->cache().device(service->device_path)) {
                mac = device->mac;
            }
        }
    }
    if (mac.empty()) {
        return;
    }

    PendingNotification notification;
    notification.mac = mac;
    if (uuid == gatt::time_characteristic_uuid()) {
        if (data.size() < sizeof(uint64_t)) {
            return;
        }
        notification.kind = NotificationKind::Time;
        std::memcpy(&notification.remote_time_ns, data.data(), sizeof(notification.remote_time_ns));
    } else if (uuid == gatt::wifi_status_characteristic_uuid()) {
        notification.kind = NotificationKind::WifiStatus;
        notification.wifi_status = util::trim_ascii_copy(std::string(data.begin(), data.end()));
    } else {
        return;
    }

    std::lock_guard<std::mutex> lock(pending_notification_mutex_);
    pending_notifications_.push_back(std::move(notification));
}

std::string TuiNode::adapter_local_mac() {
    const auto adapter = runtime_->cache().adapter(runtime_->adapter_path());
    if (!adapter || adapter->address.empty()) {
        return "-";
    }
    return adapter->address;
}

void TuiNode::render_dashboard(std::vector<bluez::DeviceInfo> devices) {
    (void)devices;
    const auto size = terminal_.size();
    const size_t frame_width = static_cast<size_t>(std::max(20, size.columns));
    const size_t left_width = static_cast<size_t>(std::max(46, std::min(size.columns / 2, 68)));
    const size_t right_width = static_cast<size_t>(std::max(28, size.columns - static_cast<int>(left_width) - 3));

    std::vector<std::string> left_lines;
    left_lines.push_back("UAVs");
    left_lines.push_back("sel host      name                  rssi  conn srv exp");

    for (const auto* device : visible_uav_devices()) {
        const auto guessed = util::device_hostname_guess(*device, uav_name_pattern_);
        std::string exports = "-";
        if (device->connected && device->services_resolved) {
            const auto it = topic_counts_.find(device->mac);
            if (it == topic_counts_.end() || it->second.pending) {
                exports = "...";
            } else if (it->second.available) {
                exports = std::to_string(it->second.count);
            } else {
                exports = "?";
            }
        }

        std::ostringstream line;
        line << (device->mac == selected_mac_ ? ">  " : "   ")
             << std::left << std::setw(9) << shorten(guessed.empty() ? std::string{"-"} : guessed, 9)
             << std::setw(22) << shorten(util::device_display_name(*device), 22)
             << std::setw(6) << device->rssi
             << std::setw(5) << (device->connected ? "Y" : "N")
             << std::setw(4) << (device->services_resolved ? "Y" : "N")
             << exports;
        left_lines.push_back(line.str());
    }
    if (left_lines.size() == 2) {
        left_lines.push_back("(no matching uavs)");
    }

    left_lines.push_back("");
    left_lines.push_back(hide_non_uav_ ? "Other devices (hidden by filter)" : "Other devices");
    left_lines.push_back("sel id        name                  rssi  conn srv exp");
    const auto other_section_start = left_lines.size();

    if (!hide_non_uav_) {
        for (const auto* device : visible_other_devices()) {
            std::ostringstream line;
            line << (device->mac == selected_mac_ ? ">  " : "   ")
                 << std::left << std::setw(9) << shorten(device->mac, 9)
                 << std::setw(22) << shorten(util::device_display_name(*device), 22)
                 << std::setw(6) << device->rssi
                 << std::setw(5) << (device->connected ? "Y" : "N")
                 << std::setw(4) << (device->services_resolved ? "Y" : "N")
                 << "-";
            left_lines.push_back(line.str());
        }
    }
    if (left_lines.size() == other_section_start) {
        left_lines.push_back(hide_non_uav_ ? "(section hidden)" : "(no other devices)");
    }

    std::vector<std::string> right_lines;
    right_lines.push_back("Selection");
    if (const auto* device = selected_device()) {
        const auto guessed = util::device_hostname_guess(*device, uav_name_pattern_);
        right_lines.push_back("host:   " + (guessed.empty() ? std::string{"-"} : guessed));
        right_lines.push_back("name:   " + util::device_display_name(*device));
        right_lines.push_back("mac:    " + device->mac);
        right_lines.push_back("link:   connected=" + yes_no(device->connected) +
                              " services=" + yes_no(device->services_resolved));
        right_lines.push_back("trust:  paired=" + yes_no(device->paired || device->bonded) +
                              " trusted=" + yes_no(device->trusted));

        std::string export_count = "-";
        if (const auto it = topic_counts_.find(device->mac); it != topic_counts_.end()) {
            export_count = it->second.pending ? "refreshing" :
                (it->second.available ? std::to_string(it->second.count) : "-");
        }
        right_lines.push_back("exports: " + export_count + " discovered topics");
        right_lines.push_back("");

        right_lines.push_back("Time");
        if (const auto it = time_samples_.find(device->mac); it != time_samples_.end()) {
            if (it->second.pending) {
                right_lines.push_back("state: refreshing");
            } else if (!it->second.error.empty()) {
                right_lines.push_back("error: " + it->second.error);
            } else if (it->second.available) {
                std::ostringstream line;
                line << "peer:  " << format_remote_time(it->second.remote_time_ns);
                right_lines.push_back(line.str());
                if (it->second.metrics_available) {
                    line.str("");
                    line.clear();
                    line << std::fixed << std::setprecision(2) << "rtt:   " << it->second.rtt_ms << " ms";
                    right_lines.push_back(line.str());
                    line.str("");
                    line.clear();
                    line << std::fixed << std::setprecision(2) << "offset:" << ' ' << it->second.offset_ms << " ms";
                    right_lines.push_back(line.str());
                } else {
                    right_lines.push_back("rtt:   n/a (notify)");
                    right_lines.push_back("offset:n/a (notify)");
                }
            } else {
                right_lines.push_back("state: press t to sample");
            }
        } else {
            right_lines.push_back("state: press t to sample");
        }
        right_lines.push_back("");

        right_lines.push_back("Wi-Fi");
        if (const auto it = wifi_state_.find(device->mac); it != wifi_state_.end()) {
            if (it->second.pending) {
                right_lines.push_back("state: refreshing");
            } else if (!it->second.error.empty()) {
                right_lines.push_back("error: " + it->second.error);
            } else if (it->second.available) {
                right_lines.push_back("ssid:   " + (it->second.config_known
                    ? (it->second.ssid.empty() ? std::string{"-"} : it->second.ssid)
                    : std::string{"(awaiting read)"}));
                right_lines.push_back("pass:   " + std::string(it->second.config_known
                    ? (it->second.password_configured ? "configured" : "empty")
                    : "(awaiting read)"));
                right_lines.push_back("status: " + shorten(it->second.status.empty() ? std::string{"-"} : it->second.status, right_width - 8));
            } else {
                right_lines.push_back("state: press w to refresh");
            }
        } else {
            right_lines.push_back("state: press w to refresh");
        }
    } else {
        right_lines.push_back("no device selected");
    }

    right_lines.push_back("");
    right_lines.push_back("Controls");
    right_lines.push_back("j/k or arrows: move");
    right_lines.push_back("c connect/disconnect");
    right_lines.push_back("s toggle scan");
    right_lines.push_back("t refresh time");
    right_lines.push_back("w refresh Wi-Fi");
    right_lines.push_back("i set SSID");
    right_lines.push_back("p set password");
    right_lines.push_back("f toggle UAV filter");
    right_lines.push_back("r refresh list");
    right_lines.push_back("q quit");

    std::ostringstream out;
    out << "\x1b[H";
    {
        std::ostringstream line;
        line << "\x1b[1;36mMRS UAV Bluetooth TUI\x1b[0m  "
             << "\x1b[2madapter=" << runtime_->adapter_path()
             << "  local_mac=" << adapter_local_mac()
             << "  scan=" << yes_no(runtime_->is_scanning())
             << "  filter=" << (hide_non_uav_ ? "uav-only" : "all")
             << "\x1b[0m";
        out << frame_line(line.str(), frame_width);
    }

    if (prompt_mode_ != PromptMode::None) {
        const auto label = prompt_mode_ == PromptMode::WifiSsid ? "ssid" : "password";
        const auto shown = prompt_mode_ == PromptMode::WifiPassword
            ? std::string(prompt_buffer_.size(), '*')
            : prompt_buffer_;
        out << frame_line(std::string{"\x1b[33minput "} + label + ": " + shown + "\x1b[0m", frame_width);
    } else {
        out << frame_line("\x1b[2m" + status_message_ + "\x1b[0m", frame_width);
    }
    out << frame_line(std::string(frame_width, '='), frame_width);

    const size_t body_rows = static_cast<size_t>(std::max(10, size.rows - 4));
    for (size_t row = 0; row < body_rows; ++row) {
        const auto left = row < left_lines.size() ? left_lines[row] : std::string{};
        const auto right = row < right_lines.size() ? right_lines[row] : std::string{};
        out << frame_line(
            pad_right(shorten(left, left_width), left_width) + " | " + shorten(right, right_width),
            frame_width);
    }

    out << "\x1b[J";

    const auto frame = out.str();
    if (frame == last_frame_) {
        return;
    }
    last_frame_ = frame;
    last_render_ = std::chrono::steady_clock::now();
    if (terminal_.active()) {
        terminal_.write_text(frame);
        return;
    }
    std::cout << frame << std::flush;
}

}  // namespace mrs_uav_bluetooth::app