// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/service_node.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace {

constexpr auto kStatusSummaryLogInterval = std::chrono::seconds(15);
constexpr auto kPeerStatusLogInterval = std::chrono::seconds(30);
constexpr auto kRepeatedLogWindow = std::chrono::seconds(5);

std::string lower_trim(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const auto start = value.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return {};
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(start, end - start + 1);
}

std::string device_hostname_guess(const mrs_uav_bluetooth::bluez::DeviceInfo& device) {
    if (mrs_uav_bluetooth::util::is_uav_hostname(device.name)) {
        return device.name;
    }
    if (mrs_uav_bluetooth::util::is_uav_hostname(device.alias)) {
        return device.alias;
    }
    return {};
}

std::string bridge_characteristic_name_for_service(const std::string& bridge_name) {
    constexpr std::string_view prefix{"bridge:"};
    if (bridge_name.rfind(prefix.data(), 0) == 0) {
        return bridge_name.substr(prefix.size());
    }
    return bridge_name + "/value";
}

bool phase_matches(const std::string& phase, std::initializer_list<const char*> values) {
    for (const char* value : values) {
        if (phase == value) {
            return true;
        }
    }
    return false;
}

bool is_log_interval_elapsed(std::chrono::steady_clock::time_point last,
                             std::chrono::steady_clock::time_point now,
                             std::chrono::steady_clock::duration interval) {
    return last == std::chrono::steady_clock::time_point{} || (now - last) >= interval;
}

bool is_interesting_peer_status(const mrs_uav_bluetooth::peer::PeerConnectionSession& session,
                                bool connected,
                                bool services_resolved,
                                const std::string& bridge_status) {
    return session.desired || connected || services_resolved || bridge_status != "none" ||
           !phase_matches(session.phase, {"idle"});
}

std::string security_mode_summary(const mrs_uav_bluetooth::config::NodeConfig& config) {
    if (config.auto_pair) {
        return config.auto_trust ? "pair=auto trust=auto" : "pair=auto trust=manual";
    }
    return config.auto_trust ? "pair=off trust=auto" : "pair=off trust=manual";
}

std::string peer_security_summary(const std::optional<mrs_uav_bluetooth::bluez::DeviceInfo>& device,
                                  const mrs_uav_bluetooth::config::NodeConfig& config) {
    if (!device) {
        return config.auto_pair ? "pairing" : "manual";
    }

    if (!config.auto_pair) {
        if (device->paired || device->bonded) {
            return "forbidden-bond";
        }
        if (device->trusted) {
            return "trusted";
        }
        return config.auto_trust ? "trust-pending" : "manual";
    }

    if (device->paired && device->bonded && device->trusted) {
        return "paired+trusted";
    }
    if (device->paired && device->trusted) {
        return "paired";
    }
    if (device->bonded && device->trusted) {
        return "bonded+trusted";
    }
    if (device->paired) {
        return device->trusted ? "paired" : "paired/untrusted";
    }
    if (device->bonded) {
        return device->trusted ? "bonded" : "bonded/untrusted";
    }
    if (device->trusted) {
        return "trusted-only";
    }
    return "pairing";
}

std::string peer_gatt_summary(const mrs_uav_bluetooth::peer::PeerConnectionSession& session,
                              bool connected,
                              bool services_resolved,
                              const std::string& bridge_status) {
    if (!connected) {
        return "down";
    }
    if (services_resolved) {
        return "ready";
    }
    if (bridge_status == "ready" || bridge_status == "subscribing" ||
        session.local_time_notify_active_this_connection ||
        session.time_bridge_healthy_this_connection) {
        return "rediscovery";
    }
    return "loading";
}

std::string peer_status_snapshot(const std::string& mac,
                                 const std::string& device_label,
                                 const mrs_uav_bluetooth::peer::PeerConnectionSession& session,
                                 const std::optional<mrs_uav_bluetooth::bluez::DeviceInfo>& device,
                                 const mrs_uav_bluetooth::config::NodeConfig& config,
                                 bool connected,
                                 bool services_resolved,
                                 const std::string& bridge_status) {
    std::ostringstream stream;
    stream << "peer=" << mac
           << " name='" << device_label << "'"
           << " target='" << session.peer_name << "'"
           << " phase=" << session.phase
           << " want=" << (session.desired ? "Y" : "N")
           << " link=" << (connected ? "up" : "down")
           << " gatt=" << peer_gatt_summary(session, connected, services_resolved, bridge_status)
           << " bridge=" << bridge_status
           << " sec=" << peer_security_summary(device, config)
           << " detail=" << session.detail;
    return stream.str();
}

std::string overall_status_word(
    const std::map<std::string, mrs_uav_bluetooth::bluez::DeviceInfo>& devices_map,
    const mrs_uav_bluetooth::peer::PeerManager* peers,
    bool server_active,
    bool advertisement_active,
    bool scan_active,
    bool dbus_ready) {
    if (!dbus_ready) {
        return "error";
    }

    bool any_desired = false;
    bool any_ready = false;
    bool any_connecting = false;
    bool any_recovering = false;
    bool any_blocked = false;

    if (peers) {
        for (const auto& [mac, session] : peers->sessions()) {
            (void)mac;
            if (!session.desired) {
                continue;
            }
            any_desired = true;
            if (phase_matches(session.phase, {"recovering", "stale"})) {
                any_recovering = true;
                continue;
            }
            if (phase_matches(session.phase, {"blocked", "policy_blocked"})) {
                any_blocked = true;
                continue;
            }
            if (phase_matches(session.phase, {"ready"})) {
                any_ready = true;
                continue;
            }
            if (phase_matches(session.phase, {
                    "connecting", "connect_pending", "securing", "connected_unready", "discovered", "disconnected"})) {
                any_connecting = true;
            }
        }
    }

    if (any_recovering) {
        return "recovering";
    }
    if (any_blocked && !any_ready && !any_connecting) {
        return "blocked";
    }
    if (any_connecting) {
        return "connecting";
    }
    if (any_ready) {
        return "ready";
    }
    if (any_desired) {
        return "active";
    }

    const auto connected = std::count_if(devices_map.begin(), devices_map.end(),
        [](const auto& item) { return item.second.connected; });
    if (connected > 0 || server_active || advertisement_active || scan_active) {
        return "active";
    }
    return "idle";
}

}  // namespace

namespace mrs_uav_bluetooth::app {

void ServiceNode::publish_periodic_status() {
    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    std::map<std::string, bluez::DeviceInfo> devices_map;
    if (client_) {
        for (const auto& device : client_->get_devices()) {
            devices_map[device.mac] = device;
        }
    }
    const auto connected_count = std::count_if(devices_map.begin(), devices_map.end(),
                                               [](const auto& pair) { return pair.second.connected; });
    const auto status_word = overall_status_word(
        devices_map,
        peers_.get(),
        static_cast<bool>(gatt_app_),
        static_cast<bool>(advertisement_),
        client_ && client_->is_scanning(),
        client_ != nullptr && !adapter_path_.empty());
    std::vector<std::pair<std::string, std::string>> peer_snapshots;
    size_t suppressed_peers = 0;
    if (peers_) {
        for (const auto& [mac, session] : peers_->sessions()) {
            auto dev_it = devices_map.find(mac);
            const bool conn = dev_it != devices_map.end() && dev_it->second.connected;
            const bool svc = dev_it != devices_map.end() && dev_it->second.services_resolved;
            const std::optional<bluez::DeviceInfo> device =
                dev_it != devices_map.end() ? std::optional<bluez::DeviceInfo>(dev_it->second) : std::nullopt;
            auto bridge_it = peers_->time_bridges().find(mac);
            const std::string bridge_status =
                bridge_it != peers_->time_bridges().end() ? bridge_it->second.status : "none";
            std::string device_label;
            if (dev_it != devices_map.end()) {
                if (!dev_it->second.name.empty()) {
                    device_label = dev_it->second.name;
                } else if (!dev_it->second.alias.empty()) {
                    device_label = dev_it->second.alias;
                }
            }
            if (!is_interesting_peer_status(session, conn, svc, bridge_status)) {
                suppressed_peers += 1;
                continue;
            }
            peer_snapshots.emplace_back(mac,
                                        peer_status_snapshot(mac,
                                                             device_label,
                                                             session,
                                                             device,
                                                             active_config_,
                                                             conn,
                                                             svc,
                                                             bridge_status));
        }
    }

    const auto summary = "state=" + status_word +
        " " + security_mode_summary(active_config_) +
        " devices=" + std::to_string(devices_map.size()) +
        " connected=" + std::to_string(static_cast<size_t>(connected_count)) +
        " sessions=" + std::to_string(peers_ ? peers_->sessions().size() : 0u) +
        " interesting_peers=" + std::to_string(peer_snapshots.size()) +
        " suppressed_peers=" + std::to_string(suppressed_peers) +
        " time_bridges=" + std::to_string(peers_ ? peers_->time_bridges().size() : 0u) +
        " server=" + std::string(gatt_app_ ? "active" : "off") +
        " adv=" + std::string(advertisement_ ? "active" : "off") +
        " scan=" + std::string(client_ && client_->is_scanning() ? "on" : "off");
    const auto now = std::chrono::steady_clock::now();
    bool emit_summary = false;
    std::vector<std::string> peer_logs_to_emit;
    {
        std::lock_guard<std::mutex> lock(log_state_mutex_);
        const bool summary_due = is_log_interval_elapsed(last_status_log_time_, now, kStatusSummaryLogInterval);
        if (summary != last_status_log_summary_ || summary_due) {
            last_status_log_summary_ = summary;
            last_status_log_time_ = now;
            emit_summary = true;
        }

        const bool peer_dump_due = is_log_interval_elapsed(last_peer_status_log_time_, now, kPeerStatusLogInterval);
        std::set<std::string> seen_peers;
        for (const auto& [mac, snapshot] : peer_snapshots) {
            seen_peers.insert(mac);
            auto it = last_peer_status_log_.find(mac);
            if (it == last_peer_status_log_.end() || it->second != snapshot || peer_dump_due) {
                last_peer_status_log_[mac] = snapshot;
                peer_logs_to_emit.push_back(snapshot);
            }
        }

        for (auto it = last_peer_status_log_.begin(); it != last_peer_status_log_.end();) {
            if (seen_peers.count(it->first) == 0) {
                it = last_peer_status_log_.erase(it);
            } else {
                ++it;
            }
        }

        if (!peer_logs_to_emit.empty()) {
            last_peer_status_log_time_ = now;
        }
    }

    if (emit_summary) {
        RCLCPP_INFO(get_logger(), "[status] %s", summary.c_str());
    }
    for (const auto& snapshot : peer_logs_to_emit) {
        RCLCPP_INFO(get_logger(), "[status] %s", snapshot.c_str());
    }

    ros_->status_publisher().publish_report(status_word);
    ros_->status_publisher().publish_log(build_detailed_status_report(devices_map));
    ros_->status_publisher().publish_devices(devices_map);
}

void ServiceNode::log_info_coalesced(const std::string& key, const std::string& message) {
    size_t suppressed_count = 0;
    {
        std::lock_guard<std::mutex> lock(repeated_log_mutex_);
        auto& entry = repeated_log_entries_["info:" + key];
        const auto now = std::chrono::steady_clock::now();
        if (entry.last_emit_time != std::chrono::steady_clock::time_point{} &&
            (now - entry.last_emit_time) < kRepeatedLogWindow) {
            ++entry.suppressed_count;
            return;
        }
        suppressed_count = entry.suppressed_count;
        entry.suppressed_count = 0;
        entry.last_emit_time = now;
    }

    if (suppressed_count > 0) {
        RCLCPP_INFO(get_logger(), "%s (and %zu same messages received)",
                    message.c_str(), suppressed_count);
    } else {
        RCLCPP_INFO(get_logger(), "%s", message.c_str());
    }
}

void ServiceNode::log_warn_coalesced(const std::string& key, const std::string& message) {
    size_t suppressed_count = 0;
    {
        std::lock_guard<std::mutex> lock(repeated_log_mutex_);
        auto& entry = repeated_log_entries_["warn:" + key];
        const auto now = std::chrono::steady_clock::now();
        if (entry.last_emit_time != std::chrono::steady_clock::time_point{} &&
            (now - entry.last_emit_time) < kRepeatedLogWindow) {
            ++entry.suppressed_count;
            return;
        }
        suppressed_count = entry.suppressed_count;
        entry.suppressed_count = 0;
        entry.last_emit_time = now;
    }

    if (suppressed_count > 0) {
        RCLCPP_WARN(get_logger(), "%s (and %zu same messages received)",
                    message.c_str(), suppressed_count);
    } else {
        RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }
}

std::string ServiceNode::build_detailed_status_report(
    const std::map<std::string, bluez::DeviceInfo>& devices_map) const {
    std::vector<std::string> lines;

    const auto now_wall = std::time(nullptr);
    std::tm local_tm{};
    localtime_r(&now_wall, &local_tm);
    std::ostringstream ts;
    ts << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");

    const auto bool_text = [](bool value) {
        return value ? "True" : "False";
    };
    const auto upper_state = [](bool value) {
        return value ? "ACTIVE" : "OFF";
    };
    const auto display_name_for_device = [](const bluez::DeviceInfo& device) {
        if (!device.alias.empty()) {
            return device.alias;
        }
        if (!device.name.empty()) {
            return device.name;
        }
        return std::string{"?"};
    };

    const auto wifi_service_uuid = util::named_service_uuid("wifi");
    const auto time_service_uuid = util::named_service_uuid("time");
    const auto wifi_ssid_characteristic_uuid = util::named_characteristic_uuid("wifi/ssid");
    const auto wifi_password_characteristic_uuid = util::named_characteristic_uuid("wifi/password");
    const auto wifi_status_characteristic_uuid = util::named_characteristic_uuid("wifi/status");
    const auto time_characteristic_uuid = util::named_characteristic_uuid("time/ns");
    const auto time_value_descriptor_uuid = util::named_descriptor_uuid("time/ns/value");
    const auto time_writeback_descriptor_uuid = util::named_descriptor_uuid("time/ns/writeback");

    std::map<std::string, const bridge::TopicExportBridgeState*> exports_by_service_path;
    for (const auto& [_, state] : bridge_registry_.exports()) {
        if (state.service && state.service->service()) {
            exports_by_service_path[state.service->service()->path()] = &state;
        }
    }

    const auto descriptor_name_for = [&](const std::string& service_uuid,
                                         const bridge::TopicExportBridgeState* export_state,
                                         const std::string& descriptor_uuid) {
        if (service_uuid == time_service_uuid) {
            if (descriptor_uuid == time_value_descriptor_uuid) {
                return std::string{"time/ns/value"};
            }
            if (descriptor_uuid == time_writeback_descriptor_uuid) {
                return std::string{"time/ns/writeback"};
            }
        }
        if (export_state != nullptr) {
            const auto characteristic_name = bridge_characteristic_name_for_service(export_state->bridge_name);
            const std::vector<std::pair<std::string, std::string>> bridge_descriptors{{"/type", "type"},
                                                                                      {"/format", "format"},
                                                                                      {"/members", "members"},
                                                                                      {"/rate_hz", "rate_hz"}};
            for (const auto& [suffix, label] : bridge_descriptors) {
                if (descriptor_uuid == util::named_descriptor_uuid(characteristic_name + suffix)) {
                    return characteristic_name + "/" + label;
                }
            }
        }
        return descriptor_uuid;
    };

    lines.push_back("--- Service Node Status @ " + ts.str() + " ---");
    lines.push_back("  adapter:    " + adapter_path_);
    lines.push_back("  hostname:   " + hostname_);
    lines.push_back("  prefix:     " + active_config_.node_topics_prefix);
    lines.push_back("  config:     " + (overlay_config_ ? overlay_config_->active_source() : std::string{}));
    if (cache_) {
        if (const auto adapter = cache_->adapter(adapter_path_)) {
            lines.push_back("  alias:      " + (adapter->alias.empty() ? adapter->name : adapter->alias));
            lines.push_back("  powered:    " + std::string(bool_text(adapter->powered)));
            lines.push_back("  connectable:" + std::string(adapter->connectable ? " True" : " False"));
            lines.push_back("  pairable:   " + std::string(bool_text(adapter->pairable)));
            lines.push_back("  discoverable: " + std::string(bool_text(adapter->discoverable)) +
                            " timeout=" + std::to_string(adapter->discoverable_timeout));
        }
    }
    lines.push_back("  scanning:   " + std::string(bool_text(client_ && client_->is_scanning())));
    lines.push_back("  server:     " + std::string(upper_state(static_cast<bool>(gatt_app_))));
    lines.push_back("  advertise:  " + std::string(upper_state(static_cast<bool>(advertisement_))));
    if (wifi_service_) {
        lines.push_back("  wifi-svc:   enabled");
    }
    if (time_service_) {
        lines.push_back("  time-svc:   enabled");
    }

    if (gatt_app_) {
        const auto& local_services = gatt_app_->services();
        size_t local_characteristics = 0;
        size_t local_descriptors = 0;
        for (const auto& service : local_services) {
            local_characteristics += service->characteristics().size();
            for (const auto& characteristic : service->characteristics()) {
                local_descriptors += characteristic->descriptors().size();
            }
        }
        lines.push_back("  local gatt: services=" + std::to_string(local_services.size()) +
                        " characteristics=" + std::to_string(local_characteristics) +
                        " descriptors=" + std::to_string(local_descriptors));
        for (const auto& service : local_services) {
            const auto export_it = exports_by_service_path.find(service->path());
            const auto* export_state = export_it != exports_by_service_path.end() ? export_it->second : nullptr;
            std::string service_name = service->uuid();
            if (service->uuid() == wifi_service_uuid) {
                service_name = "wifi";
            } else if (service->uuid() == time_service_uuid) {
                service_name = "time";
            } else if (export_state != nullptr) {
                service_name = export_state->bridge_name;
            }
            lines.push_back("    service '" + service_name + "' UUID=" + service->uuid());
            for (const auto& characteristic : service->characteristics()) {
                std::string characteristic_name = characteristic->uuid();
                if (service->uuid() == wifi_service_uuid && characteristic->uuid() == wifi_ssid_characteristic_uuid) {
                    characteristic_name = "wifi/ssid";
                } else if (service->uuid() == wifi_service_uuid && characteristic->uuid() == wifi_password_characteristic_uuid) {
                    characteristic_name = "wifi/password";
                } else if (service->uuid() == wifi_service_uuid && characteristic->uuid() == wifi_status_characteristic_uuid) {
                    characteristic_name = "wifi/status";
                } else if (service->uuid() == time_service_uuid && characteristic->uuid() == time_characteristic_uuid) {
                    characteristic_name = "time/ns";
                } else if (export_state != nullptr && characteristic->uuid() == export_state->bridge_uuid) {
                    characteristic_name = bridge_characteristic_name_for_service(export_state->bridge_name);
                }
                lines.push_back("      chrc '" + characteristic_name + "' UUID=" + characteristic->uuid());
                for (const auto& descriptor : characteristic->descriptors()) {
                    lines.push_back("        desc '" +
                                    descriptor_name_for(service->uuid(), export_state, descriptor->uuid()) +
                                    "' UUID=" + descriptor->uuid());
                }
            }
        }
    }

    const auto now_mono = peers_ ? peers_->now_monotonic() : 0.0;
    std::vector<const bluez::DeviceInfo*> connected_peers;
    std::vector<const bluez::DeviceInfo*> other_connected;
    for (const auto& [_, device] : devices_map) {
        if (!device.connected) {
            continue;
        }
        const auto guessed_name = device_hostname_guess(device);
        if (!guessed_name.empty() && util::is_uav_hostname(guessed_name, active_config_.auto_connect_pattern)) {
            connected_peers.push_back(&device);
        } else {
            other_connected.push_back(&device);
        }
    }

    lines.push_back("  discovered: " + std::to_string(devices_map.size()) + " devices");
    lines.push_back("  connected:  " + std::to_string(connected_peers.size() + other_connected.size()) + " devices");
    for (const auto* device : connected_peers) {
        const peer::PeerTimeBridge* bridge = nullptr;
        const peer::PeerConnectionSession* session = nullptr;
        if (peers_) {
            const auto bridge_it = peers_->time_bridges().find(device->mac);
            if (bridge_it != peers_->time_bridges().end()) {
                bridge = &bridge_it->second;
            }
            const auto session_it = peers_->sessions().find(device->mac);
            if (session_it != peers_->sessions().end()) {
                session = &session_it->second;
            }
        }

        double inactivity = -1.0;
        if (bridge != nullptr && bridge->last_activity_monotonic > 0.0) {
            inactivity = std::max(0.0, now_mono - bridge->last_activity_monotonic);
        }

        std::vector<std::string> status_parts;
        const bool pairing_ready = device->paired || device->bonded;
        status_parts.push_back("security=" +
                               peer_security_summary(std::optional<bluez::DeviceInfo>(*device), active_config_));
        if (bridge != nullptr && bridge->status == "ready") {
            status_parts.push_back("time-bridge-ready");
        } else {
            status_parts.push_back(device->services_resolved ? "services-resolved" : "services-resolving");
            if (bridge != nullptr) {
                if (!bridge->status.empty()) {
                    status_parts.push_back("bridge=" + bridge->status);
                }
                if (bridge->services_wait_started_monotonic > 0.0 && bridge->services_wait_grace_s > 0.0) {
                    const auto waited = std::max(0.0, now_mono - bridge->services_wait_started_monotonic);
                    std::ostringstream wait_stream;
                    wait_stream << std::fixed << std::setprecision(1)
                                << "wait=" << waited << "/" << bridge->services_wait_grace_s << "s";
                    status_parts.push_back(wait_stream.str());
                }
                if (bridge->pairing_failures > 0) {
                    status_parts.push_back("pair_failures=" + std::to_string(bridge->pairing_failures));
                }
                if (!bridge->detail.empty()) {
                    status_parts.push_back(bridge->detail);
                }
            } else if (session != nullptr && !session->detail.empty()) {
                status_parts.push_back(session->detail);
            }
        }
        if (inactivity >= 0.0) {
            std::ostringstream inactivity_part;
            inactivity_part << std::fixed << std::setprecision(1) << "bridge_idle=" << inactivity << "s";
            status_parts.push_back(inactivity_part.str());
        }

        std::ostringstream inactivity_stream;
        inactivity_stream << std::fixed << std::setprecision(1) << inactivity;
        std::ostringstream status_stream;
        for (size_t index = 0; index < status_parts.size(); ++index) {
            if (index != 0) {
                status_stream << ',';
            }
            status_stream << status_parts[index];
        }

        lines.push_back("    peer: " + device->mac + " " + display_name_for_device(*device) +
                        " RSSI=" + std::to_string(device->rssi) +
                        " secure=" + std::string(bool_text(pairing_ready)) +
                        " paired=" + std::string(bool_text(device->paired)) +
                        " bonded=" + std::string(bool_text(device->bonded)) +
                        " trusted=" + std::string(bool_text(device->trusted)) +
                        " inactive_s=" + inactivity_stream.str() +
                        " status=" + status_stream.str());
    }
    if (connected_peers.empty()) {
        lines.push_back("    peers: (none)");
    }
    if (other_connected.empty()) {
        lines.push_back("    other: (none)");
    } else {
        lines.push_back("    other:");
        for (const auto* device : other_connected) {
            lines.push_back("      " + device->mac + " " + display_name_for_device(*device) +
                            " RSSI=" + std::to_string(device->rssi));
        }
    }

    if (!bridge_registry_.exports().empty()) {
        lines.push_back("  export bridges (" + std::to_string(bridge_registry_.exports().size()) + "):");
        for (const auto& [_, state] : bridge_registry_.exports()) {
            const auto path = state.service ? state.service->transport_path() : std::string{"?"};
            lines.push_back("    " + state.topic_name + " -> " + state.bridge_key + " [" + path + "]");
        }
    }
    size_t active_import_bridges = 0;
    for (const auto& [_, state] : bridge_registry_.imports()) {
        if (!state.path.empty()) {
            active_import_bridges += 1;
        }
    }
    if (active_import_bridges > 0) {
        lines.push_back("  import bridges (" + std::to_string(active_import_bridges) + "):");
        for (const auto& [_, state] : bridge_registry_.imports()) {
            if (state.path.empty()) {
                continue;
            }
            lines.push_back("    " + state.mac + " " + state.bridge_key + " -> " + state.resolved_topic_name);
        }
    }

    std::map<std::string, std::vector<std::string>> peer_topics;
    if (peers_) {
        for (const auto& [mac, state] : peers_->time_bridges()) {
            std::ostringstream hz_stream;
            hz_stream << std::fixed << std::setprecision(2) << state.current_hz;
            peer_topics[mac].push_back(state.status_topic_name + " @ " + hz_stream.str() + " Hz");
        }
    }
    for (const auto& [_, state] : bridge_registry_.imports()) {
        if (state.path.empty()) {
            continue;
        }
        std::ostringstream hz_stream;
        hz_stream << std::fixed << std::setprecision(2) << state.current_hz;
        peer_topics[state.mac].push_back(state.resolved_topic_name + " @ " + hz_stream.str() + " Hz");
    }
    if (!peer_topics.empty()) {
        lines.push_back("  peer topics:");
        for (const auto& [mac, topics] : peer_topics) {
            std::string peer_name = mac;
            if (peers_) {
                const auto session_it = peers_->sessions().find(mac);
                if (session_it != peers_->sessions().end() && !session_it->second.peer_name.empty()) {
                    peer_name = session_it->second.peer_name;
                }
            }
            std::ostringstream topics_stream;
            for (size_t index = 0; index < topics.size(); ++index) {
                if (index != 0) {
                    topics_stream << ", ";
                }
                topics_stream << topics[index];
            }
            lines.push_back("    " + peer_name + " (" + mac + "): " + topics_stream.str());
        }
    }

    lines.push_back("---");

    std::ostringstream report;
    for (size_t index = 0; index < lines.size(); ++index) {
        if (index != 0) {
            report << '\n';
        }
        report << lines[index];
    }
    return report.str();
}

mrs_uav_bluetooth::msg::BleDevice ServiceNode::to_device_msg(const bluez::DeviceInfo& device) const {
    mrs_uav_bluetooth::msg::BleDevice msg;
    msg.mac = device.mac;
    msg.path = device.object_path;
    msg.adapter = device.adapter;
    msg.address_type = device.address_type;
    msg.name = device.name;
    msg.alias = device.alias;
    msg.hostname = device_hostname_guess(device);
    msg.icon = device.icon;
    msg.appearance = device.appearance;
    msg.rssi = device.rssi;
    msg.tx_power = device.tx_power;
    msg.pathloss = 0;
    msg.connected = device.connected;
    msg.paired = device.paired;
    msg.bonded = device.bonded;
    msg.trusted = device.trusted;
    msg.blocked = device.blocked;
    msg.services_resolved = device.services_resolved;
    msg.uuids = device.uuids;
    for (const auto& [key, value] : device.manufacturer_data) {
        std::ostringstream hex;
        hex << key << ":";
        for (uint8_t byte : value) {
            constexpr char kHex[] = "0123456789abcdef";
            hex << kHex[(byte >> 4) & 0xF] << kHex[byte & 0xF];
        }
        msg.manufacturer_data_hex.push_back(hex.str());
    }
    for (const auto& [key, value] : device.service_data) {
        std::ostringstream hex;
        hex << key << ":";
        for (uint8_t byte : value) {
            constexpr char kHex[] = "0123456789abcdef";
            hex << kHex[(byte >> 4) & 0xF] << kHex[byte & 0xF];
        }
        msg.service_data_hex.push_back(hex.str());
    }
    msg.last_seen = get_clock()->now();
    return msg;
}

mrs_uav_bluetooth::msg::BleGattService ServiceNode::to_service_msg(const bluez::GattServiceInfo& item) const {
    mrs_uav_bluetooth::msg::BleGattService msg;
    msg.path = item.object_path;
    msg.uuid = item.uuid;
    msg.primary = item.primary;
    msg.device_path = item.device_path;
    msg.includes = item.includes;
    return msg;
}

mrs_uav_bluetooth::msg::BleGattCharacteristic ServiceNode::to_characteristic_msg(const bluez::GattCharacteristicInfo& item) const {
    mrs_uav_bluetooth::msg::BleGattCharacteristic msg;
    msg.path = item.object_path;
    msg.service_path = item.service_path;
    msg.uuid = item.uuid;
    msg.flags = item.flags;
    msg.notifying = item.notifying;
    msg.mtu = item.mtu;
    return msg;
}

mrs_uav_bluetooth::msg::BleGattDescriptor ServiceNode::to_descriptor_msg(const bluez::GattDescriptorInfo& item) const {
    mrs_uav_bluetooth::msg::BleGattDescriptor msg;
    msg.path = item.object_path;
    msg.characteristic_path = item.characteristic_path;
    msg.uuid = item.uuid;
    msg.flags = item.flags;
    return msg;
}

}  // namespace mrs_uav_bluetooth::app