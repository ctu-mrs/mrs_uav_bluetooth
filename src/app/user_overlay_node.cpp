// SPDX-License-Identifier: BSD-3-Clause
/// \file src/app/user_overlay_node.cpp
/// \brief Implements the user overlay node component of the ROS 2 application and operator-tool layer.

#include "mrs_uav_bluetooth/app/user_overlay_node.hpp"

#include "mrs_uav_bluetooth/config/config_loader.hpp"
#include "mrs_uav_bluetooth/util/hostname_utils.hpp"
#include "mrs_uav_bluetooth/util/string_utils.hpp"
#include "mrs_uav_bluetooth/util/topic_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <set>
#include <cctype>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <sstream>

using namespace std::chrono_literals;

namespace mrs_uav_bluetooth::app {

namespace {

constexpr size_t kBridgeFrameHeaderBytes = 5;

/// \brief Treat export and bidirectional bridges as producers for status reporting.
/// \param bridge Topic bridge direction flags to test.
/// \return Configured outgoing bridge map.
bool exports(const config::SharedTopicConfig& bridge) {
    // Treat export and bidirectional bridges as producers for status reporting.
    return bridge.mode == "export" || bridge.mode == "both";
}

/// \brief Treat import and bidirectional bridges as consumers for status reporting.
/// \param bridge Topic bridge direction flags to test.
/// \return Configured incoming bridge map.
bool imports(const config::SharedTopicConfig& bridge) {
    // Treat import and bidirectional bridges as consumers for status reporting.
    return bridge.mode == "import" || bridge.mode == "both";
}

/// \brief Resolve the ROS topic component used for a named or address-only peer.
/// \param name Peer hostname, when resolved from BlueZ or Mesh membership.
/// \param mac Bluetooth address used when the hostname is unavailable.
/// \return The same ROS-safe peer component used by import publishers.
std::string peer_topic_token(const std::string& name, const std::string& mac) {
    // Match GATT and transport-bridge naming, including address fallback.
    auto token = util::sanitize_topic_suffix(name);
    if (name.empty() || token == "ble_device")
        token = "mac_" + util::sanitize_topic_suffix(mac);
    if (!token.empty() &&
        std::isdigit(static_cast<unsigned char>(token.front())) != 0)
        token = "peer_" + token;
    return token;
}

/// \brief Render a boolean as the short yes/no text used in reports.
/// \param value Boolean to render.
/// \return Literal yes when true; otherwise no.
std::string yes_no(bool value) {
    // Render a boolean as the short yes/no text used in reports.
    return value ? "yes" : "no";
}

/// \brief Format a 16-bit Mesh identifier as zero-padded hexadecimal.
/// \param value Mesh identifier to render as four hexadecimal digits.
/// \return Zero-padded four-digit hexadecimal value.
std::string hex16(uint16_t value) {
    // Format a 16-bit Mesh identifier as zero-padded hexadecimal.
    std::ostringstream stream;
    stream << "0x" << std::hex << std::setw(4) << std::setfill('0') << value;
    return stream.str();
}

/// \brief Validate the bridge frame header and extract its nonzero channel identifier.
/// \param payload Advertisement bridge frame whose channel header is inspected.
/// \return Nonzero bridge channel from a valid frame; otherwise std::nullopt.
std::optional<uint16_t> advertisement_channel(
    const std::vector<uint8_t>& payload) {
    // Validate the bridge frame header and extract its nonzero channel identifier.
    if (payload.size() < kBridgeFrameHeaderBytes || payload[0] != 'M' ||
        payload[1] != 'B' || payload[2] != 1) {
        return std::nullopt;
    }
    const auto channel = static_cast<uint16_t>(
        payload[3] | (static_cast<uint16_t>(payload[4]) << 8U));
    return channel == 0 ? std::nullopt : std::optional<uint16_t>{channel};
}

/// \brief Calculate monotonic sample age, reserving a negative value for no sample.
/// \param received_at steady-clock time when the sample was observed.
/// \param now current monotonic time.
/// \return Monotonic sample age in seconds, or a negative value when unset.
double age_seconds(std::chrono::steady_clock::time_point received_at,
                   std::chrono::steady_clock::time_point now) {
    // Calculate monotonic sample age, reserving a negative value for no sample.
    if (received_at == std::chrono::steady_clock::time_point{}) return -1.0;
    return std::chrono::duration<double>(now - received_at).count();
}

}  // namespace

UserOverlayNode::UserOverlayNode()
    : rclcpp::Node("mrs_uav_bluetooth_user") {
    configure_parameters();
    load_overlay_context();

    keepalive_pub_ = create_publisher<std_msgs::msg::Empty>(keepalive_topic_, 10);
    configure_transport_reporting();
    keepalive_timer_ = create_wall_timer(
        std::chrono::duration<double>(keepalive_publish_period_sec_),
        [this]() {
            // Refresh the overlay lease before the service node can expire it.
            publish_keepalive();
        });
    set_active_config_client_ = create_client<mrs_uav_bluetooth::srv::SetActiveConfig>(set_active_config_service_);

    activate_overlay();
    {
        // Start periodic reporting after the requested overlay is active.
        transport_report_timer_ = create_wall_timer(
            std::chrono::duration<double>(transport_report_period_sec_),
            [this]() {
                // Publish a common snapshot of peers, links, and measured topic deliveries.
                print_transport_status();
            });
    }
}

UserOverlayNode::~UserOverlayNode() {
    // Release this overlay lease so the service node can restore its default configuration.
    revert_overlay();
}

void UserOverlayNode::configure_parameters() {
    // Reject missing overlay paths and invalid timeouts before creating service clients or timers.
    declare_parameter<std::string>("config_path", "");
    declare_parameter<double>("service_wait_timeout_sec", 30.0);
    declare_parameter<double>("service_call_timeout_sec", 45.0);
    declare_parameter<double>("deactivate_service_wait_timeout_sec", 2.0);
    declare_parameter<std::string>("sentinel_topic_suffix", "overlay_keepalive");
    declare_parameter<double>("sentinel_publish_period_sec", 1.0);
    declare_parameter<double>("min_sentinel_publish_period_sec", 0.2);
    declare_parameter<double>("transport_report_period_sec", 2.0);

    config_path_ = get_parameter("config_path").as_string();
    service_wait_timeout_sec_ = std::max(0.0, get_parameter("service_wait_timeout_sec").as_double());
    service_call_timeout_sec_ = std::max(0.0, get_parameter("service_call_timeout_sec").as_double());
    deactivate_service_wait_timeout_sec_ = std::max(0.0, get_parameter("deactivate_service_wait_timeout_sec").as_double());
    sentinel_topic_suffix_ = get_parameter("sentinel_topic_suffix").as_string();
    keepalive_publish_period_sec_ = std::max(0.0, get_parameter("sentinel_publish_period_sec").as_double());
    min_keepalive_publish_period_sec_ = std::max(0.0, get_parameter("min_sentinel_publish_period_sec").as_double());
    transport_report_period_sec_ =
        get_parameter("transport_report_period_sec").as_double();

    if (config_path_.empty()) {
        throw std::runtime_error("config_path parameter is required");
    }
    if (!std::filesystem::is_regular_file(config_path_)) {
        throw std::runtime_error("Overlay config file not found: " + config_path_);
    }

    sentinel_topic_suffix_ = util::sanitize_topic_suffix(sentinel_topic_suffix_);
    if (sentinel_topic_suffix_.empty()) {
        throw std::runtime_error("sentinel_topic_suffix must not be empty");
    }

    keepalive_publish_period_sec_ = std::max(min_keepalive_publish_period_sec_, keepalive_publish_period_sec_);
    if (keepalive_publish_period_sec_ <= 0.0) {
        throw std::runtime_error("sentinel_publish_period_sec must be > 0");
    }
    if (!std::isfinite(transport_report_period_sec_) ||
        transport_report_period_sec_ <= 0.0) {
        throw std::runtime_error(
            "transport_report_period_sec must be finite and > 0");
    }
}

void UserOverlayNode::load_overlay_context() {
    // Resolve hostname placeholders, merge the selected overlay, and choose the
    // transport-specific status streams shown while its lease is active.
    hostname_ = mrs_uav_bluetooth::util::sanitize_topic_suffix(
        mrs_uav_bluetooth::util::system_hostname().empty() ? "mrs-uav" : mrs_uav_bluetooth::util::system_hostname());
    const auto share_dir = ament_index_cpp::get_package_share_directory("mrs_uav_bluetooth");
    default_config_path_ = share_dir + "/config/default.yaml";
    effective_config_ = config::load_effective_config(
        default_config_path_, config_path_, hostname_);

    for (const auto& bridge : effective_config_.shared_topics) {
        report_gatt_ = report_gatt_ || bridge.transport == "gatt";
        report_advertisements_ =
            report_advertisements_ || bridge.transport == "advertisement";
        report_mesh_ = report_mesh_ || bridge.transport == "mesh";
    }
    report_mesh_ = report_mesh_ || effective_config_.enable_mesh;
    if (!report_gatt_ && !report_advertisements_ && !report_mesh_) {
        report_gatt_ = true;
    }

    // Service and heartbeat/topic paths deliberately share the hostname-rooted
    // hierarchy used by service_node; no ROS namespace remapping is required.
    set_active_config_service_ = mrs_uav_bluetooth::util::normalize_ros_topic(
        "/" + hostname_ + "/bluetooth/config/set_active");
    keepalive_topic_ = mrs_uav_bluetooth::util::normalize_ros_topic(
        "/" + hostname_ + "/bluetooth/config/" + sentinel_topic_suffix_);
}

void UserOverlayNode::configure_transport_reporting() {
    // Observe the complete device cache for connection state and nearby UAV identity.
    const auto topic_root = util::normalize_ros_topic(effective_config_.node_topics_prefix);
    devices_sub_ = create_subscription<mrs_uav_bluetooth::msg::BleDeviceArray>(
        util::normalize_ros_topic(topic_root + "/le/devices"), 10,
        [this](const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message) {
            // Cache the most recent BlueZ device snapshot for the next report.
            handle_devices(message);
        });

    for (const auto& bridge : effective_config_.shared_topics) {
        if (!exports(bridge) || bridge.export_topic.empty()) continue;
        const auto topic = util::normalize_ros_topic(bridge.export_topic);
        if (observed_topics_.contains(topic)) continue;
        auto subscription = create_generic_subscription(
            topic, bridge.message_type, rclcpp::QoS(10),
            [this, topic](std::shared_ptr<rclcpp::SerializedMessage>) {
                // Count source deliveries independently of the bridge's send timer.
                record_topic_sample(topic);
            });
        TopicObservation observation;
        observation.type = bridge.message_type;
        observation.subscription = subscription;
        observation.started_at = std::chrono::steady_clock::now();
        observed_topics_.emplace(topic, std::move(observation));
    }

    if (report_advertisements_) {
        advertisements_sub_ = create_subscription<mrs_uav_bluetooth::msg::BleDeviceArray>(
            util::normalize_ros_topic(topic_root + "/le/advertisements"), 10,
            [this](const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message) {
                // Retain recent application advertisements across short scan gaps.
                handle_advertisements(message);
            });
    }
    if (report_mesh_) {
        mesh_status_sub_ = create_subscription<mrs_uav_bluetooth::msg::MeshStatus>(
            util::normalize_ros_topic(topic_root + "/mesh/status"), 10,
            [this](const mrs_uav_bluetooth::msg::MeshStatus::SharedPtr message) {
                // Cache the daemon attachment and authenticated swarm membership.
                handle_mesh_status(message);
            });
        mesh_event_sub_ = create_subscription<mrs_uav_bluetooth::msg::MeshEvent>(
            util::normalize_ros_topic(topic_root + "/mesh/events"), 50,
            [this](const mrs_uav_bluetooth::msg::MeshEvent::SharedPtr message) {
                // Retain the latest Mesh lifecycle transition and cumulative count.
                handle_mesh_event(message);
            });
        mesh_message_sub_ = create_subscription<mrs_uav_bluetooth::msg::MeshMessage>(
            util::normalize_ros_topic(topic_root + "/mesh/rx"), 100,
            [this](const mrs_uav_bluetooth::msg::MeshMessage::SharedPtr message) {
                // Count received Mesh access packets for neighborhood diagnostics.
                handle_mesh_message(message);
            });
    }
}

void UserOverlayNode::record_topic_sample(const std::string& topic) {
    // Keep deliveries from the last 20 seconds and the latest lifetime count.
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(transport_status_mutex_);
    auto it = observed_topics_.find(topic);
    if (it == observed_topics_.end()) return;
    auto& observation = it->second;
    observation.samples.push_back(now);
    observation.last_at = now;
    ++observation.total;
    while (!observation.samples.empty() &&
           now - observation.samples.front() > 20s) {
        observation.samples.pop_front();
    }
}

void UserOverlayNode::refresh_import_observers() {
    // Subscribe to real peer publishers so imported topics appear as peers join.
    std::map<std::string, std::vector<std::string>> topics;
    try {
        // A transient ROS graph error should leave the previous rate observers alive.
        topics = get_topic_names_and_types();
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
            "Could not refresh peer topic graph: %s", error.what());
        return;
    }
    const auto prefix = util::normalize_ros_topic(
        effective_config_.node_topics_prefix) + "/";
    for (const auto& [topic, types] : topics) {
        if (types.empty() || topic.rfind(prefix, 0) != 0) continue;
        if (topic.find("/peers/") == std::string::npos) continue;
        bool matching_import = false;
        bool time_status = report_gatt_ &&
            topic.size() >= 12 && topic.compare(topic.size() - 12, 12, "/time_status") == 0;
        for (const auto& bridge : effective_config_.shared_topics) {
            if (!imports(bridge)) continue;
            const auto segment = bridge.transport == "mesh" ? "/mesh/peers/" : "/le/peers/";
            const auto suffix = "/" + bridge.import_topic_suffix;
            if (topic.find(segment) != std::string::npos &&
                topic.size() >= suffix.size() &&
                topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0 &&
                std::find(types.begin(), types.end(), bridge.message_type) != types.end()) {
                matching_import = true;
                break;
            }
        }
        if (!matching_import && !time_status) continue;
        {
            std::lock_guard<std::mutex> lock(transport_status_mutex_);
            if (observed_topics_.contains(topic)) continue;
        }
        try {
            rclcpp::SubscriptionBase::SharedPtr subscription;
            if (time_status) {
                subscription = create_subscription<mrs_uav_bluetooth::msg::BlePeerTimeStatus>(
                    topic, 10,
                    [this, topic](const mrs_uav_bluetooth::msg::BlePeerTimeStatus::SharedPtr message) {
                        // Count actual peer clock reports and retain the last round-trip delay.
                        record_topic_sample(topic);
                        std::lock_guard<std::mutex> lock(transport_status_mutex_);
                        peer_rtt_seconds_[topic] = message->last_rtt_s;
                    });
            } else {
                subscription = create_generic_subscription(
                    topic, types.front(), rclcpp::QoS(10),
                    [this, topic](std::shared_ptr<rclcpp::SerializedMessage>) {
                        // Measure application data arriving on this exact peer topic.
                        record_topic_sample(topic);
                    });
            }
            TopicObservation observation;
            observation.type = types.front();
            observation.subscription = subscription;
            observation.started_at = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(transport_status_mutex_);
            observed_topics_.emplace(topic, std::move(observation));
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                "Could not observe peer topic %s: %s", topic.c_str(), error.what());
        }
    }

    // Random-address peers can create new ROS paths over a long scan. Drop a
    // silent observer only after its publisher has left the ROS graph.
    const auto now = std::chrono::steady_clock::now();
    std::vector<std::string> expired_topics;
    {
        std::lock_guard<std::mutex> lock(transport_status_mutex_);
        for (const auto& [topic, observation] : observed_topics_) {
            if (topic.find("/peers/") == std::string::npos) continue;
            const auto last = observation.last_at ==
                std::chrono::steady_clock::time_point{} ?
                observation.started_at : observation.last_at;
            if (now - last > 60s) expired_topics.push_back(topic);
        }
    }
    for (const auto& topic : expired_topics) {
        try {
            if (count_publishers(topic) != 0) continue;
            std::lock_guard<std::mutex> lock(transport_status_mutex_);
            const auto found = observed_topics_.find(topic);
            if (found == observed_topics_.end()) continue;
            const auto last = found->second.last_at ==
                std::chrono::steady_clock::time_point{} ?
                found->second.started_at : found->second.last_at;
            if (now - last > 60s) {
                observed_topics_.erase(found);
                peer_rtt_seconds_.erase(topic);
            }
        } catch (const std::exception&) {
            // Retain the observer until graph lookup succeeds on a later report.
        }
    }
}

void UserOverlayNode::activate_overlay() {
    // Acquire the service-side lease before starting keepalives and status subscriptions.
    if (!set_active_config_client_->wait_for_service(std::chrono::duration<double>(service_wait_timeout_sec_))) {
        throw std::runtime_error(set_active_config_service_ + " service is not available");
    }
    auto response = call_config_service(config_path_);
    if (!response || !response->success) {
        throw std::runtime_error(
            response ? response->message : "No response from " + set_active_config_service_);
    }
    RCLCPP_INFO(get_logger(), "Activated Bluetooth overlay config: %s",
                response->active_config_path.c_str());
    RCLCPP_INFO(get_logger(),
                "Reporting %s overlay peers and observed topic rates every %.1f seconds",
                report_mesh_ ? "Mesh" :
                    report_advertisements_ ? "advertisement" : "GATT",
                transport_report_period_sec_);
    RCLCPP_INFO(get_logger(), "Publishing overlay keep-alive sentinel on: %s",
                keepalive_topic_.c_str());
}

void UserOverlayNode::revert_overlay() {
    if (!set_active_config_client_) {
        return;
    }
    // Guard against calling into ROS after context shutdown (Ctrl-C).
    if (!rclcpp::ok()) {
        return;
    }
    if (!set_active_config_client_->service_is_ready() &&
        !set_active_config_client_->wait_for_service(std::chrono::duration<double>(deactivate_service_wait_timeout_sec_))) {
        return;
    }
    if (!rclcpp::ok()) {
        return;
    }
    auto response = call_config_service("");
    if (response && response->success) {
        RCLCPP_INFO(get_logger(), "Reverted bluetooth service to default config");
    }
}

void UserOverlayNode::publish_keepalive() {
    // Renew the service-side overlay lease with an empty heartbeat message.
    std_msgs::msg::Empty msg;
    keepalive_pub_->publish(msg);
}

mrs_uav_bluetooth::srv::SetActiveConfig::Response::SharedPtr
UserOverlayNode::call_config_service(const std::string& config_path) {
    // Submit the overlay path and bound the synchronous wait for its response.
    if (!rclcpp::ok()) {
        return nullptr;
    }
    auto request = std::make_shared<mrs_uav_bluetooth::srv::SetActiveConfig::Request>();
    request->config_path = config_path;
    request->hold_seconds = 0.0;
    auto future = set_active_config_client_->async_send_request(request);
    if (!rclcpp::ok()) {
        return nullptr;
    }
    const auto result = rclcpp::spin_until_future_complete(
        get_node_base_interface(), future, std::chrono::duration<double>(service_call_timeout_sec_));
    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        return nullptr;
    }
    return future.get();
}

void UserOverlayNode::handle_advertisements(
    const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message) {
    std::lock_guard<std::mutex> lock(transport_status_mutex_);
    const auto now = std::chrono::steady_clock::now();
    advertisements_received_at_ = now;
    // BlueZ may briefly remove a device while an advertisement is replaced.
    // Preserve its last sample across brief gaps for a stable periodic report.
    for (const auto& device : message->devices) {
        const auto key = !device.hostname.empty() ? device.hostname : device.mac;
        recent_advertisements_[key] = {device, now};
    }
    for (auto it = recent_advertisements_.begin();
         it != recent_advertisements_.end();) {
        if (now - it->second.second > 10s) {
            it = recent_advertisements_.erase(it);
        } else {
            ++it;
        }
    }
}

void UserOverlayNode::handle_devices(
    const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message) {
    // Refresh connection, security, and RSSI values from the full BlueZ cache.
    std::lock_guard<std::mutex> lock(transport_status_mutex_);
    latest_devices_ = *message;
    devices_received_at_ = std::chrono::steady_clock::now();
}

void UserOverlayNode::handle_mesh_status(
    const mrs_uav_bluetooth::msg::MeshStatus::SharedPtr message) {
    // Cache the newest attachment snapshot and its local receipt time.
    std::lock_guard<std::mutex> lock(transport_status_mutex_);
    latest_mesh_status_ = *message;
    mesh_status_received_at_ = std::chrono::steady_clock::now();
}

void UserOverlayNode::handle_mesh_event(
    const mrs_uav_bluetooth::msg::MeshEvent::SharedPtr message) {
    // Cache the newest lifecycle event and increment the observed event count.
    std::lock_guard<std::mutex> lock(transport_status_mutex_);
    latest_mesh_event_ = *message;
    mesh_event_received_at_ = std::chrono::steady_clock::now();
    if (message->event == "scan_result" && !message->uuid.empty()) {
        // Remember distinct unprovisioned beacons independently of live members.
        recent_mesh_scans_[message->uuid] = {*message, mesh_event_received_at_};
    }
    ++mesh_event_count_;
}

void UserOverlayNode::handle_mesh_message(
    const mrs_uav_bluetooth::msg::MeshMessage::SharedPtr message) {
    // Cache the newest access message and increment the observed message count.
    std::lock_guard<std::mutex> lock(transport_status_mutex_);
    latest_mesh_message_ = *message;
    mesh_message_received_at_ = std::chrono::steady_clock::now();
    ++mesh_message_count_;
}

void UserOverlayNode::print_transport_status() {
    // Discover peer publishers before copying callback-owned measurements.
    refresh_import_observers();
    const auto now = std::chrono::steady_clock::now();
    std::optional<mrs_uav_bluetooth::msg::BleDeviceArray> devices;
    std::optional<mrs_uav_bluetooth::msg::MeshStatus> mesh_status;
    std::optional<mrs_uav_bluetooth::msg::MeshEvent> mesh_event;
    std::optional<mrs_uav_bluetooth::msg::MeshMessage> mesh_message;
    std::map<std::string, std::pair<mrs_uav_bluetooth::msg::BleDevice,
        std::chrono::steady_clock::time_point>> advertisements;
    std::map<std::string, TopicObservation> topics;
    std::map<std::string, std::pair<mrs_uav_bluetooth::msg::MeshEvent,
        std::chrono::steady_clock::time_point>> mesh_scans;
    std::map<std::string, double> peer_rtt;
    std::chrono::steady_clock::time_point devices_at;
    std::chrono::steady_clock::time_point advertisements_at;
    std::chrono::steady_clock::time_point mesh_status_at;
    std::chrono::steady_clock::time_point mesh_event_at;
    std::chrono::steady_clock::time_point mesh_message_at;
    uint64_t mesh_events = 0;
    uint64_t mesh_messages = 0;
    {
        // Purge old rate samples and take one coherent transport/topic snapshot.
        std::lock_guard<std::mutex> lock(transport_status_mutex_);
        for (auto& [topic, observation] : observed_topics_) {
            (void)topic;
            while (!observation.samples.empty() &&
                   now - observation.samples.front() > 20s) {
                observation.samples.pop_front();
            }
        }
        for (auto it = recent_mesh_scans_.begin(); it != recent_mesh_scans_.end();) {
            if (now - it->second.second > 30s) it = recent_mesh_scans_.erase(it);
            else ++it;
        }
        mesh_scans = recent_mesh_scans_;
        devices = latest_devices_;
        // A stopped scan must age out old addresses even without another callback.
        for (auto it = recent_advertisements_.begin();
             it != recent_advertisements_.end();) {
            if (now - it->second.second > 10s)
                it = recent_advertisements_.erase(it);
            else ++it;
        }
        advertisements = recent_advertisements_;
        mesh_status = latest_mesh_status_;
        mesh_event = latest_mesh_event_;
        mesh_message = latest_mesh_message_;
        topics = observed_topics_;
        peer_rtt = peer_rtt_seconds_;
        devices_at = devices_received_at_;
        advertisements_at = advertisements_received_at_;
        mesh_status_at = mesh_status_received_at_;
        mesh_event_at = mesh_event_received_at_;
        mesh_message_at = mesh_message_received_at_;
        mesh_events = mesh_event_count_;
        mesh_messages = mesh_message_count_;
    }

    const auto wall_now = std::time(nullptr);
    std::tm local_time{};
    localtime_r(&wall_now, &local_time);
    const auto mode = report_mesh_ ? "mesh" :
        report_advertisements_ ? "advertisement" : "gatt";
    std::ostringstream report;
    report << std::fixed << std::setprecision(1);
    report << "--- Bluetooth " << mode << " | "
           << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S") << " ---\n";
    report << "  config: " << std::filesystem::path(config_path_).filename().string()
           << "  rates: last 20s\n";

    const auto format_age = [now](std::chrono::steady_clock::time_point at) {
        // Use a literal for an unobserved sample; show recent silence in seconds.
        if (at == std::chrono::steady_clock::time_point{}) return std::string{"never"};
        std::ostringstream value;
        value << std::fixed << std::setprecision(1) << age_seconds(at, now) << "s";
        return value.str();
    };
    const auto measured_hz = [now](const TopicObservation& observation) {
        // Normalize early observations over at least two seconds to avoid burst spikes.
        const auto elapsed = std::chrono::duration<double>(
            now - observation.started_at).count();
        const auto seconds = std::clamp(elapsed, 2.0, 20.0);
        return static_cast<double>(observation.samples.size()) / seconds;
    };
    const auto append_rate = [&](const std::string& topic, double target_hz) {
        // Pair the configured target with received ROS messages, silence, and gap.
        const auto found = topics.find(topic);
        if (found == topics.end()) {
            report << "0.0Hz/" << target_hz << "Hz age=never";
            return;
        }
        const auto& observation = found->second;
        const auto hz = measured_hz(observation);
        report << hz << "Hz/" << target_hz << "Hz age="
               << format_age(observation.last_at);
        if (target_hz > 0.0) {
            report << " of_target=" << std::lround(100.0 * hz / target_hz) << "%";
        }
        double max_gap = 0.0;
        for (size_t index = 1; index < observation.samples.size(); ++index) {
            max_gap = std::max(max_gap, std::chrono::duration<double>(
                observation.samples[index] - observation.samples[index - 1]).count());
        }
        if (observation.samples.size() >= 2) report << " gap_max=" << max_gap << "s";
        report << " total=" << observation.total;
    };

    const auto device_age = age_seconds(devices_at, now);
    const auto mesh_age = age_seconds(mesh_status_at, now);
    size_t connected = 0;
    size_t resolved = 0;
    if (devices) {
        for (const auto& device : devices->devices) {
            if (device.connected) {
                ++connected;
                if (device.services_resolved) ++resolved;
            }
        }
    }
    size_t active_imports = 0;
    double lowest_ratio = 1.0;
    bool observed_import = false;
    bool warming_import = false;
    for (const auto& bridge : effective_config_.shared_topics) {
        if (!imports(bridge)) continue;
        const auto segment = bridge.transport == "mesh" ? "/mesh/peers/" : "/le/peers/";
        const auto suffix = "/" + bridge.import_topic_suffix;
        for (const auto& [topic, observation] : topics) {
            if (topic.find(segment) == std::string::npos ||
                topic.size() < suffix.size() ||
                topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) != 0)
                continue;
            observed_import = true;
            const auto age = age_seconds(observation.last_at, now);
            const auto fresh_limit = bridge.rate_hz > 0.0 ?
                std::max(3.0, 3.0 / bridge.rate_hz) : 3.0;
            if (age >= 0.0 && age < fresh_limit) ++active_imports;
            if (bridge.rate_hz > 0.0)
                lowest_ratio = std::min(lowest_ratio,
                    measured_hz(observation) / bridge.rate_hz);
            warming_import = warming_import ||
                now - observation.started_at < 5s;
        }
    }
    bool source_active = false;
    for (const auto& bridge : effective_config_.shared_topics) {
        if (exports(bridge) && count_publishers(bridge.export_topic) > 0) {
            source_active = true;
            break;
        }
    }
    size_t expected_peers = 0;
    if (report_mesh_) {
        if (mesh_status) {
            expected_peers = mesh_status->swarm_members.size();
            if (std::find(mesh_status->swarm_members.begin(),
                          mesh_status->swarm_members.end(), hostname_) !=
                mesh_status->swarm_members.end() && expected_peers > 0)
                --expected_peers;
        }
    } else if (report_advertisements_) {
        // Random addresses can leave several payload records for one hostname.
        std::set<std::string> named_advertisers;
        for (const auto& [key, cached] : advertisements) {
            (void)key;
            const auto name = util::lower_trim_copy(cached.first.hostname);
            const auto mac = util::lower_trim_copy(cached.first.mac);
            const bool admitted = effective_config_.peer_whitelist.empty() ||
                std::find(effective_config_.peer_whitelist.begin(),
                          effective_config_.peer_whitelist.end(), name) !=
                    effective_config_.peer_whitelist.end() ||
                std::find(effective_config_.peer_whitelist.begin(),
                          effective_config_.peer_whitelist.end(), mac) !=
                    effective_config_.peer_whitelist.end();
            if (admitted && !name.empty() &&
                name != util::lower_trim_copy(hostname_))
                named_advertisers.insert(name);
        }
        expected_peers = named_advertisers.size();
    } else {
        expected_peers = connected;
    }
    std::string data_health = "idle";
    if (expected_peers > 0 && source_active && active_imports == 0) {
        data_health = "no_rx";
    } else if (observed_import && active_imports > 0) {
        data_health = warming_import ? "warming" :
            lowest_ratio < 0.5 ? "low" :
            active_imports < expected_peers ? "partial" :
            lowest_ratio < 0.9 ? "reduced" : "good";
    }
    report << "  health: link=";
    if (report_mesh_) {
        report << (mesh_status && mesh_age < 5.0 &&
                   mesh_status->attached && mesh_status->application_transport_ready
                   ? "ready" : "waiting");
        report << "  daemon=" << yes_no(mesh_status && mesh_status->daemon_available)
               << " attached=" << yes_no(mesh_status && mesh_status->attached)
               << " status_age=" << format_age(mesh_status_at);
    } else if (report_advertisements_) {
        report << (age_seconds(advertisements_at, now) >= 0.0 &&
                   age_seconds(advertisements_at, now) < 5.0 ? "scanning" : "waiting")
               << "  scan_age=" << format_age(advertisements_at);
    } else {
        report << (resolved > 0 ? "ready" : connected > 0 ? "connecting" : "waiting")
               << "  connected=" << connected << " resolved=" << resolved
               << " cache_age=" << format_age(devices_at);
    }
    report << "  data=" << data_health << " peers=" << expected_peers
           << " active_imports=" << active_imports << '\n';

    report << "  nearby:\n";
    if (report_mesh_) {
        if (!mesh_status) {
            report << "    mesh membership: waiting for status\n";
        } else {
            report << "    mesh membership: " << mesh_status->swarm_members.size()
                   << " live, swarm=" << mesh_status->swarm_id
                   << " last_peer_rx=";
            if (mesh_status->seconds_since_last_heard == UINT32_MAX) report << "never";
            else report << mesh_status->seconds_since_last_heard << "s";
            report << '\n';
            for (const auto& member : mesh_status->swarm_members) {
                report << "    " << member
                       << (util::lower_trim_copy(member) == util::lower_trim_copy(hostname_)
                           ? " (local)" : " (live Mesh member)") << '\n';
            }
        }
        report << "    Mesh discovery: " << mesh_scans.size()
               << " unprovisioned beacons seen within 30s\n";
        for (const auto& [uuid, cached] : mesh_scans) {
            report << "      uuid=" << uuid << " RSSI=" << cached.first.rssi
                   << "dBm age=" << format_age(cached.second);
            if (cached.first.server != 0)
                report << " via=" << hex16(cached.first.server);
            report << '\n';
        }
    } else if (!devices) {
        report << "    BlueZ device snapshot: waiting\n";
    } else {
        size_t other_devices = 0;
        std::map<std::string, std::vector<const mrs_uav_bluetooth::msg::BleDevice*>> peers;
        for (const auto& device : devices->devices) {
            if (device.hostname.empty() && !device.connected) {
                ++other_devices;
                continue;
            }
            const auto name = device.hostname.empty() ?
                (!device.alias.empty() ? device.alias : device.mac) : device.hostname;
            peers[name].push_back(&device);
        }
        for (const auto& [name, records] : peers) {
            // BlueZ may cache both a random advertising address and a paired
            // connection address for one UAV. Combine them into one peer row.
            const auto* best = records.front();
            bool connected_peer = false;
            bool paired_peer = false;
            bool services_ready = false;
            bool admitted = effective_config_.peer_whitelist.empty() ||
                std::find(effective_config_.peer_whitelist.begin(),
                          effective_config_.peer_whitelist.end(),
                          util::lower_trim_copy(name)) !=
                    effective_config_.peer_whitelist.end();
            for (const auto* device : records) {
                if (device->connected || (!best->connected &&
                    device->rssi > best->rssi)) best = device;
                connected_peer = connected_peer || device->connected;
                paired_peer = paired_peer || device->paired;
                services_ready = services_ready || device->services_resolved;
                admitted = admitted ||
                    std::find(effective_config_.peer_whitelist.begin(),
                              effective_config_.peer_whitelist.end(),
                              util::lower_trim_copy(device->mac)) !=
                        effective_config_.peer_whitelist.end();
            }
            report << "    " << name << " RSSI=" << best->rssi << "dBm"
                   << " admitted=" << yes_no(admitted)
                   << " connected=" << yes_no(connected_peer)
                   << " paired=" << yes_no(paired_peer)
                   << " services=" << yes_no(services_ready)
                   << " addresses=";
            for (size_t index = 0; index < records.size(); ++index) {
                if (index != 0) report << ',';
                report << records[index]->mac;
            }
            if (report_advertisements_) {
                auto found = advertisements.find(name);
                if (found == advertisements.end()) {
                    for (const auto* device : records) {
                        found = advertisements.find(device->mac);
                        if (found != advertisements.end()) break;
                    }
                }
                if (found != advertisements.end()) {
                    report << " ad_age=" << format_age(found->second.second)
                           << " ad_mac=" << found->second.first.mac;
                    const auto channel = advertisement_channel(
                        found->second.first.advertising_data);
                    if (channel) report << " channel=" << *channel;
                    report << " bytes=" << found->second.first.advertising_data.size();
                }
            }
            report << '\n';
        }
        report << "    BlueZ cache: " << peers.size() << " named peer(s), "
               << other_devices << " other devices\n";
        if (report_advertisements_) {
            report << "    application advertisements: " << advertisements.size()
                   << " cached payload identities within 10s\n";
        }
    }

    report << "  links:\n";
    if (report_mesh_) {
        if (mesh_status) {
            report << "    local=" << mesh_status->state
                   << " ready=" << yes_no(mesh_status->application_transport_ready)
                   << " address=";
            if (mesh_status->addresses.empty()) report << "none";
            else {
                for (const auto address : mesh_status->addresses) report << hex16(address) << ' ';
            }
            report << " provisioner=" << mesh_status->active_provisioner
                   << " sequence=" << mesh_status->sequence_number << '\n';
            report << "    receive: " << mesh_messages << " packets";
            if (mesh_message) {
                report << " last=" << format_age(mesh_message_at)
                       << " from=" << hex16(mesh_message->source)
                       << " bytes=" << mesh_message->data.size();
            }
            report << '\n';
        } else {
            report << "    waiting for Mesh status\n";
        }
    } else if (report_advertisements_) {
        report << "    broadcast reception; " << advertisements.size()
               << " application advertisers seen recently\n";
    } else {
        report << "    GATT connections=" << connected
               << " services_ready=" << resolved << '\n';
    }

    std::set<std::string> expected_peer_tokens;
    if (report_mesh_ && mesh_status) {
        // Authenticated live members have stable names even before a bridge publishes.
        for (const auto& member : mesh_status->swarm_members) {
            if (util::lower_trim_copy(member) != util::lower_trim_copy(hostname_))
                expected_peer_tokens.insert(peer_topic_token(member, ""));
        }
    } else if (devices) {
        for (const auto& device : devices->devices) {
            const auto name = util::lower_trim_copy(device.hostname);
            const auto mac = util::lower_trim_copy(device.mac);
            const bool admitted = effective_config_.peer_whitelist.empty() ||
                std::find(effective_config_.peer_whitelist.begin(),
                          effective_config_.peer_whitelist.end(), name) !=
                    effective_config_.peer_whitelist.end() ||
                std::find(effective_config_.peer_whitelist.begin(),
                          effective_config_.peer_whitelist.end(), mac) !=
                    effective_config_.peer_whitelist.end();
            if (!admitted || name == util::lower_trim_copy(hostname_)) continue;
            if (report_gatt_ && device.connected)
                expected_peer_tokens.insert(peer_topic_token(device.hostname, device.mac));
            if (report_advertisements_ && !name.empty() &&
                advertisements.contains(device.hostname))
                expected_peer_tokens.insert(peer_topic_token(device.hostname, device.mac));
        }
    }
    if (report_advertisements_) {
        for (const auto& [key, cached] : advertisements) {
            (void)key;
            const auto& name = cached.first.hostname;
            const auto mac = util::lower_trim_copy(cached.first.mac);
            const bool admitted = effective_config_.peer_whitelist.empty() ||
                std::find(effective_config_.peer_whitelist.begin(),
                          effective_config_.peer_whitelist.end(),
                          util::lower_trim_copy(name)) !=
                    effective_config_.peer_whitelist.end() ||
                std::find(effective_config_.peer_whitelist.begin(),
                          effective_config_.peer_whitelist.end(), mac) !=
                    effective_config_.peer_whitelist.end();
            if (admitted && !name.empty() &&
                util::lower_trim_copy(name) != util::lower_trim_copy(hostname_))
                expected_peer_tokens.insert(peer_topic_token(name, mac));
        }
    }
    report << "  topics (observed / configured):\n";
    for (const auto& bridge : effective_config_.shared_topics) {
        const auto name = bridge.name.empty() ? bridge.bridge_name : bridge.name;
        report << "    " << name << " [" << bridge.transport << ", "
               << bridge.mode << ", " << bridge.payload_format;
        if (bridge.transport == "advertisement" && bridge.advertisement_bare)
            report << ", bare payload";
        else if (bridge.transport != "gatt") report << ", channel " << bridge.channel_id;
        report << "]\n";
        if (exports(bridge)) {
            const auto source = util::normalize_ros_topic(bridge.export_topic);
            report << "      source " << source << " ";
            append_rate(source, bridge.rate_hz);
            report << " publishers=" << count_publishers(source) << '\n';
        }
        if (imports(bridge)) {
            const auto segment = bridge.transport == "mesh" ? "/mesh/peers/" : "/le/peers/";
            const auto prefix = util::normalize_ros_topic(
                effective_config_.node_topics_prefix) + segment;
            const auto suffix = "/" + bridge.import_topic_suffix;
            std::set<std::string> shown_topics;
            for (const auto& [topic, observation] : topics) {
                (void)observation;
                if (topic.rfind(prefix, 0) != 0 ||
                    topic.size() < suffix.size() ||
                    topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) != 0)
                    continue;
                shown_topics.insert(topic);
                report << "      received " << topic << " ";
                append_rate(topic, bridge.rate_hz);
                report << '\n';
            }
            for (const auto& token : expected_peer_tokens) {
                const auto topic = prefix + token + suffix;
                if (!shown_topics.insert(topic).second) continue;
                report << "      received " << topic << " ";
                append_rate(topic, bridge.rate_hz);
                report << '\n';
            }
            if (shown_topics.empty()) {
                report << "      received " << prefix << "<peer>" << suffix
                       << " 0.0Hz/" << bridge.rate_hz << "Hz age=never\n";
            }
        }
    }
    if (report_gatt_) {
        bool time_found = false;
        for (const auto& [topic, observation] : topics) {
            (void)observation;
            if (topic.find("/le/peers/") == std::string::npos ||
                topic.size() < 12 ||
                topic.compare(topic.size() - 12, 12, "/time_status") != 0)
                continue;
            time_found = true;
            report << "    clock " << topic << " ";
            append_rate(topic, effective_config_.time_update_period > 0.0
                ? 1.0 / effective_config_.time_update_period : 0.0);
            if (const auto found = peer_rtt.find(topic); found != peer_rtt.end())
                report << " RTT=" << found->second * 1000.0 << "ms";
            report << '\n';
        }
        if (!time_found) report << "    clock: awaiting peer time status\n";
    }

    if (report_mesh_) {
        report << "  mesh detail: relay=" << yes_no(mesh_status && mesh_status->relay_feature)
               << " beacon=" << yes_no(mesh_status && mesh_status->beacon)
               << " events=" << mesh_events;
        if (mesh_event) report << " last=" << mesh_event->event
                               << " age=" << format_age(mesh_event_at);
        report << '\n';
        if (mesh_status && !mesh_status->error.empty()) {
            report << "  issue: " << mesh_status->error << '\n';
        }
    }
    report << "---";
    RCLCPP_INFO(get_logger(), "\n%s", report.str().c_str());
}

}  // namespace mrs_uav_bluetooth::app
