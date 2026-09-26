// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/user_overlay_node.hpp"

#include "mrs_uav_bluetooth/config/config_loader.hpp"
#include "mrs_uav_bluetooth/util/hostname_utils.hpp"
#include "mrs_uav_bluetooth/util/string_utils.hpp"
#include "mrs_uav_bluetooth/util/topic_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
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

bool exports(const config::SharedTopicConfig& bridge) {
    return bridge.mode == "export" || bridge.mode == "both";
}

bool imports(const config::SharedTopicConfig& bridge) {
    return bridge.mode == "import" || bridge.mode == "both";
}

std::string yes_no(bool value) {
    return value ? "yes" : "no";
}

std::string hex16(uint16_t value) {
    std::ostringstream stream;
    stream << "0x" << std::hex << std::setw(4) << std::setfill('0') << value;
    return stream.str();
}

std::string byte_preview(const std::vector<uint8_t>& bytes, size_t limit = 12) {
    if (bytes.empty()) return "(empty)";
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    const auto shown = std::min(bytes.size(), limit);
    for (size_t index = 0; index < shown; ++index) {
        if (index != 0) stream << ' ';
        stream << std::setw(2) << static_cast<unsigned>(bytes[index]);
    }
    if (shown < bytes.size()) stream << " ...";
    return stream.str();
}

std::optional<uint16_t> advertisement_channel(
    const std::vector<uint8_t>& payload) {
    if (payload.size() < kBridgeFrameHeaderBytes || payload[0] != 'M' ||
        payload[1] != 'B' || payload[2] != 1) {
        return std::nullopt;
    }
    return static_cast<uint16_t>(payload[3]) |
        (static_cast<uint16_t>(payload[4]) << 8U);
}

double age_seconds(std::chrono::steady_clock::time_point received_at,
                   std::chrono::steady_clock::time_point now) {
    if (received_at == std::chrono::steady_clock::time_point{}) return -1.0;
    return std::chrono::duration<double>(now - received_at).count();
}

}  // namespace

UserOverlayNode::UserOverlayNode()
    : rclcpp::Node("mrs_uav_bluetooth_user") {
    configure_parameters();
    load_overlay_context();

    keepalive_pub_ = create_publisher<std_msgs::msg::Empty>(keepalive_topic_, 10);
    if (print_text_status_) {
        print_sub_ = create_subscription<std_msgs::msg::String>(
            print_topic_, 200,
            [this](const std_msgs::msg::String::SharedPtr message) {
                handle_print(message);
            });
    }
    configure_transport_reporting();
    keepalive_timer_ = create_wall_timer(
        std::chrono::duration<double>(keepalive_publish_period_sec_),
        [this]() { publish_keepalive(); });
    set_active_config_client_ = create_client<mrs_uav_bluetooth::srv::SetActiveConfig>(set_active_config_service_);

    activate_overlay();
    if (report_advertisements_ || report_mesh_) {
        // Structured subscriptions are installed before activation so they can
        // cache transition events, but reporting starts only after the service
        // confirms that the overlay is active.
        transport_report_timer_ = create_wall_timer(
            std::chrono::duration<double>(transport_report_period_sec_),
            [this]() { print_transport_status(); });
    }
}

UserOverlayNode::~UserOverlayNode() {
    revert_overlay();
}

void UserOverlayNode::configure_parameters() {
    declare_parameter<std::string>("config_path", "");
    declare_parameter<std::string>("print_source", "log");
    declare_parameter<double>("service_wait_timeout_sec", 30.0);
    declare_parameter<double>("service_call_timeout_sec", 45.0);
    declare_parameter<double>("deactivate_service_wait_timeout_sec", 2.0);
    declare_parameter<std::string>("sentinel_topic_suffix", "overlay_keepalive");
    declare_parameter<double>("sentinel_publish_period_sec", 1.0);
    declare_parameter<double>("min_sentinel_publish_period_sec", 0.2);
    declare_parameter<double>("transport_report_period_sec", 2.0);

    config_path_ = get_parameter("config_path").as_string();
    print_source_ = get_parameter("print_source").as_string();
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

    std::transform(print_source_.begin(), print_source_.end(), print_source_.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    if (print_source_ != "status" && print_source_ != "log") {
        throw std::runtime_error("print_source must be 'status' or 'log'");
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
    hostname_ = mrs_uav_bluetooth::util::sanitize_topic_suffix(
        mrs_uav_bluetooth::util::system_hostname().empty() ? "mrs-uav" : mrs_uav_bluetooth::util::system_hostname());
    const auto share_dir = ament_index_cpp::get_package_share_directory("mrs_uav_bluetooth");
    default_config_path_ = share_dir + "/config/default.yaml";
    effective_config_ = config::load_effective_config(
        default_config_path_, config_path_, hostname_);

    bool has_gatt_bridge = false;
    for (const auto& bridge : effective_config_.shared_topics) {
        has_gatt_bridge = has_gatt_bridge || bridge.transport == "gatt";
        report_advertisements_ =
            report_advertisements_ || bridge.transport == "advertisement";
        report_mesh_ = report_mesh_ || bridge.transport == "mesh";
    }
    report_mesh_ = report_mesh_ || effective_config_.enable_mesh;
    // Preserve the original comprehensive service report for GATT and generic
    // overlays. Connectionless modes use their structured status topics so the
    // terminal shows transport-specific information instead of GATT internals.
    print_text_status_ = has_gatt_bridge ||
        (!report_advertisements_ && !report_mesh_);
    // The verbose log publisher is optional in service_node. Falling back to
    // its always-present periodic status keeps a custom GATT overlay useful
    // even when it did not explicitly enable log_topic_enable.
    if (print_text_status_ && print_source_ == "log" &&
        !effective_config_.log_topic_enable) {
        print_source_ = "status";
    }

    // Service and heartbeat/topic paths deliberately share the hostname-rooted
    // hierarchy used by service_node; no ROS namespace remapping is required.
    set_active_config_service_ = mrs_uav_bluetooth::util::normalize_ros_topic(
        "/" + hostname_ + "/bluetooth/config/set_active");
    keepalive_topic_ = mrs_uav_bluetooth::util::normalize_ros_topic(
        "/" + hostname_ + "/bluetooth/config/" + sentinel_topic_suffix_);
    print_topic_ = mrs_uav_bluetooth::util::normalize_ros_topic(
        "/" + hostname_ + "/bluetooth/system/" + print_source_);
}

void UserOverlayNode::configure_transport_reporting() {
    const auto topic_root = util::normalize_ros_topic(
        effective_config_.node_topics_prefix);

    if (report_advertisements_) {
        const auto topic = util::normalize_ros_topic(
            topic_root + "/le/advertisements");
        advertisements_sub_ =
            create_subscription<mrs_uav_bluetooth::msg::BleDeviceArray>(
                topic, 10,
                [this](
                    const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message) {
                    handle_advertisements(message);
                });
    }

    if (report_mesh_) {
        mesh_status_sub_ =
            create_subscription<mrs_uav_bluetooth::msg::MeshStatus>(
                util::normalize_ros_topic(topic_root + "/mesh/status"), 10,
                [this](
                    const mrs_uav_bluetooth::msg::MeshStatus::SharedPtr message) {
                    handle_mesh_status(message);
                });
        mesh_event_sub_ =
            create_subscription<mrs_uav_bluetooth::msg::MeshEvent>(
                util::normalize_ros_topic(topic_root + "/mesh/events"), 50,
                [this](
                    const mrs_uav_bluetooth::msg::MeshEvent::SharedPtr message) {
                    handle_mesh_event(message);
                });
        mesh_message_sub_ =
            create_subscription<mrs_uav_bluetooth::msg::MeshMessage>(
                util::normalize_ros_topic(topic_root + "/mesh/rx"), 100,
                [this](
                    const mrs_uav_bluetooth::msg::MeshMessage::SharedPtr message) {
                    handle_mesh_message(message);
                });
    }

}

void UserOverlayNode::activate_overlay() {
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
    if (print_text_status_) {
        RCLCPP_INFO(get_logger(), "Printing Bluetooth service %s topic: %s",
                    print_source_.c_str(), print_topic_.c_str());
    }
    if (report_advertisements_) {
        RCLCPP_INFO(get_logger(),
                    "Printing advertisement bridge/peer details every %.2f seconds",
                    transport_report_period_sec_);
    }
    if (report_mesh_) {
        RCLCPP_INFO(get_logger(),
                    "Printing Mesh lifecycle/traffic details every %.2f seconds",
                    transport_report_period_sec_);
    }
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
    std_msgs::msg::Empty msg;
    keepalive_pub_->publish(msg);
}

mrs_uav_bluetooth::srv::SetActiveConfig::Response::SharedPtr
UserOverlayNode::call_config_service(const std::string& config_path) {
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

void UserOverlayNode::handle_print(const std_msgs::msg::String::SharedPtr message) {
    if (print_source_ == "status") {
        std::lock_guard<std::mutex> lock(print_mutex_);
        if (message->data == last_print_payload_) {
            return;
        }
        last_print_payload_ = message->data;
        RCLCPP_INFO(get_logger(), "\n%s", message->data.c_str());
        return;
    }

    std::istringstream stream(message->data);
    std::string line;
    while (std::getline(stream, line)) {
        RCLCPP_INFO(get_logger(), "%s", line.c_str());
    }
}


void UserOverlayNode::handle_advertisements(
    const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message) {
    std::lock_guard<std::mutex> lock(transport_status_mutex_);
    latest_advertisements_ = *message;
    advertisements_received_at_ = std::chrono::steady_clock::now();
}

void UserOverlayNode::handle_mesh_status(
    const mrs_uav_bluetooth::msg::MeshStatus::SharedPtr message) {
    std::lock_guard<std::mutex> lock(transport_status_mutex_);
    latest_mesh_status_ = *message;
    mesh_status_received_at_ = std::chrono::steady_clock::now();
}

void UserOverlayNode::handle_mesh_event(
    const mrs_uav_bluetooth::msg::MeshEvent::SharedPtr message) {
    std::lock_guard<std::mutex> lock(transport_status_mutex_);
    latest_mesh_event_ = *message;
    mesh_event_received_at_ = std::chrono::steady_clock::now();
    ++mesh_event_count_;
}

void UserOverlayNode::handle_mesh_message(
    const mrs_uav_bluetooth::msg::MeshMessage::SharedPtr message) {
    std::lock_guard<std::mutex> lock(transport_status_mutex_);
    latest_mesh_message_ = *message;
    mesh_message_received_at_ = std::chrono::steady_clock::now();
    ++mesh_message_count_;
}

void UserOverlayNode::print_transport_status() {
    std::optional<mrs_uav_bluetooth::msg::BleDeviceArray> advertisements;
    std::optional<mrs_uav_bluetooth::msg::MeshStatus> mesh_status;
    std::optional<mrs_uav_bluetooth::msg::MeshEvent> mesh_event;
    std::optional<mrs_uav_bluetooth::msg::MeshMessage> mesh_message;
    std::chrono::steady_clock::time_point advertisements_received_at;
    std::chrono::steady_clock::time_point mesh_status_received_at;
    std::chrono::steady_clock::time_point mesh_event_received_at;
    std::chrono::steady_clock::time_point mesh_message_received_at;
    uint64_t mesh_event_count = 0;
    uint64_t mesh_message_count = 0;
    {
        // Copy under the callback mutex and format after releasing it. This
        // prevents terminal I/O from delaying status/RX callbacks.
        std::lock_guard<std::mutex> lock(transport_status_mutex_);
        advertisements = latest_advertisements_;
        mesh_status = latest_mesh_status_;
        mesh_event = latest_mesh_event_;
        mesh_message = latest_mesh_message_;
        advertisements_received_at = advertisements_received_at_;
        mesh_status_received_at = mesh_status_received_at_;
        mesh_event_received_at = mesh_event_received_at_;
        mesh_message_received_at = mesh_message_received_at_;
        mesh_event_count = mesh_event_count_;
        mesh_message_count = mesh_message_count_;
    }

    const auto steady_now = std::chrono::steady_clock::now();
    const auto wall_now = std::time(nullptr);
    std::tm local_time{};
    localtime_r(&wall_now, &local_time);

    std::ostringstream report;
    report << "--- Bluetooth Overlay Status @ "
           << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S") << " ---\n"
           << "  config: " << config_path_ << '\n';

    if (!effective_config_.peer_whitelist.empty()) {
        report << "  peer priority/admission: ";
        for (size_t index = 0;
             index < effective_config_.peer_whitelist.size(); ++index) {
            if (index != 0) report << " > ";
            report << effective_config_.peer_whitelist[index];
        }
        report << '\n';
    } else if (report_mesh_) {
        report << "  peer priority/admission: open (numeric Mesh fallback)\n";
    } else {
        report << "  peer priority/admission: open (all nearby peers)\n";
    }

    const auto append_bridges = [this, &report](const std::string& transport) {
        size_t count = 0;
        for (const auto& bridge : effective_config_.shared_topics) {
            if (bridge.transport == transport) ++count;
        }
        report << "  " << transport << " bridges (" << count << "):\n";
        for (const auto& bridge : effective_config_.shared_topics) {
            if (bridge.transport != transport) continue;
            const auto display_name = !bridge.name.empty()
                ? bridge.name
                : bridge.bridge_name;
            report << "    " << display_name
                   << ": channel=" << bridge.channel_id
                   << " mode=" << bridge.mode
                   << " format=" << bridge.payload_format
                   << " members=" << bridge.member_specs.size();
            if (bridge.rate_hz > 0.0) {
                report << " rate=" << bridge.rate_hz << "Hz";
            }
            if (transport == "mesh") {
                report << " destination=" << hex16(bridge.mesh_destination)
                       << " app_key=" << bridge.mesh_app_key_index;
            }
            report << '\n';
            if (exports(bridge)) {
                report << "      tx source: " << bridge.export_topic
                       << " (publishers=" << count_publishers(bridge.export_topic)
                       << ")\n";
            }
            if (imports(bridge)) {
                const auto segment = transport == "mesh" ? "mesh" : "le";
                report << "      rx topics: "
                       << effective_config_.node_topics_prefix << '/' << segment
                       << "/peers/<peer>/" << bridge.import_topic_suffix << '\n';
            }
        }
    };

    if (report_advertisements_) {
        append_bridges("advertisement");
        if (!advertisements) {
            report << "  advertisement scan: waiting for first snapshot\n";
        } else {
            report << "  advertisement scan: peers="
                   << advertisements->devices.size()
                   << " snapshot_age=" << std::fixed << std::setprecision(1)
                   << age_seconds(advertisements_received_at, steady_now)
                   << "s\n";
            for (const auto& device : advertisements->devices) {
                const auto normalized_peer = util::lower_trim_copy(device.hostname);
                const bool allowed = effective_config_.peer_whitelist.empty() ||
                    std::find(effective_config_.peer_whitelist.begin(),
                              effective_config_.peer_whitelist.end(),
                              normalized_peer) !=
                        effective_config_.peer_whitelist.end();
                std::string display_name = device.hostname;
                if (display_name.empty()) display_name = device.alias;
                if (display_name.empty()) display_name = device.name;
                if (display_name.empty()) display_name = "unknown";
                report << "    " << display_name << " (" << device.mac << ")"
                       << " RSSI=" << device.rssi << "dBm"
                       << " admitted=" << yes_no(allowed)
                       << " frame_bytes=" << device.advertising_data.size();
                if (const auto channel =
                        advertisement_channel(device.advertising_data)) {
                    report << " channel=" << *channel
                           << " payload_bytes="
                           << (device.advertising_data.size() -
                               kBridgeFrameHeaderBytes);
                } else {
                    report << " channel=unrecognized";
                }
                report << " data=[" << byte_preview(device.advertising_data)
                       << "]\n";
            }
            if (advertisements->devices.empty()) {
                report << "    (no nearby bridge advertisements)\n";
            }
        }
    }

    if (report_mesh_) {
        append_bridges("mesh");
        if (!mesh_status) {
            report << "  Mesh: waiting for first status message\n";
        } else {
            report << "  Mesh: daemon=" << yes_no(mesh_status->daemon_available)
                   << " attached=" << yes_no(mesh_status->attached)
                   << " state=" << mesh_status->state
                   << " status_age=" << std::fixed << std::setprecision(1)
                   << age_seconds(mesh_status_received_at, steady_now) << "s\n"
                   << "    provisioning: automatic="
                   << yes_no(mesh_status->automatic_provisioning)
                   << " configured_preference="
                   << effective_config_.mesh_provisioner_preference
                   << " preferred=" << mesh_status->preferred_provisioner
                   << " active=" << mesh_status->active_provisioner
                   << " local_active="
                   << yes_no(mesh_status->local_is_active_provisioner) << '\n'
                   << "    logical swarm: id=" << mesh_status->swarm_id
                   << " participating=" << yes_no(mesh_status->swarm_participating)
                   << " live_members=" << mesh_status->swarm_members.size() << '\n'
                   << "    transport: ready="
                   << yes_no(mesh_status->application_transport_ready)
                   << " relay=" << yes_no(mesh_status->relay_feature)
                   << " proxy=" << yes_no(mesh_status->proxy_feature)
                   << " friend=" << yes_no(mesh_status->friend_feature)
                   << " low_power=" << yes_no(mesh_status->low_power_feature)
                   << " beacon=" << yes_no(mesh_status->beacon) << '\n'
                   << "    identity: uuid=" << mesh_status->uuid
                   << " token=0x" << std::hex << mesh_status->token << std::dec
                   << " node=" << mesh_status->node_path << '\n'
                   << "    addresses:";
            if (mesh_status->addresses.empty()) {
                report << " (none)";
            } else {
                for (const auto address : mesh_status->addresses) {
                    report << ' ' << hex16(address);
                }
            }
            report << "\n    network: iv_index=" << mesh_status->iv_index
                   << " iv_update=" << yes_no(mesh_status->iv_update)
                   << " sequence=" << mesh_status->sequence_number
                   << " last_peer_rx=" << (mesh_status->seconds_since_last_heard == UINT32_MAX
                        ? std::string{"never"}
                        : std::to_string(mesh_status->seconds_since_last_heard) + "s")
                   << '\n';
            report << "    swarm members:";
            if (mesh_status->swarm_members.empty()) {
                report << " (none)";
            } else {
                for (const auto& member : mesh_status->swarm_members) {
                    report << ' ' << member;
                }
            }
            report << '\n';
            if (!mesh_status->error.empty()) {
                report << "    error: " << mesh_status->error << '\n';
            }
        }

        report << "  Mesh events: count=" << mesh_event_count;
        if (!mesh_event) {
            report << " last=(none)\n";
        } else {
            report << " last=" << mesh_event->event
                   << " age=" << std::fixed << std::setprecision(1)
                   << age_seconds(mesh_event_received_at, steady_now) << "s";
            if (!mesh_event->uuid.empty()) {
                report << " uuid=" << mesh_event->uuid;
            }
            if (mesh_event->unicast != 0) {
                report << " unicast=" << hex16(mesh_event->unicast);
            }
            if (!mesh_event->reason.empty()) {
                report << " reason=" << mesh_event->reason;
            }
            if (!mesh_event->detail.empty()) {
                report << " detail=" << mesh_event->detail;
            }
            report << '\n';
        }

        report << "  Mesh RX: count=" << mesh_message_count;
        if (!mesh_message) {
            report << " last=(none)\n";
        } else {
            report << " age=" << std::fixed << std::setprecision(1)
                   << age_seconds(mesh_message_received_at, steady_now) << "s"
                   << " source=" << hex16(mesh_message->source)
                   << " destination=" << hex16(mesh_message->destination)
                   << " key=" << mesh_message->key_index
                   << " bytes=" << mesh_message->data.size()
                   << " data=[" << byte_preview(mesh_message->data) << "]\n";
        }
    }

    report << "---";
    RCLCPP_INFO(get_logger(), "\n%s", report.str().c_str());
}

}  // namespace mrs_uav_bluetooth::app
