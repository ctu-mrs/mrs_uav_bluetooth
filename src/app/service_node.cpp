// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/service_node.hpp"

#include "mrs_uav_bluetooth/config/shared_topic_config.hpp"

#include "mrs_uav_bluetooth/util/topic_utils.hpp"
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <ctime>
#include <future>
#include <limits>
#include <rclcpp/create_timer.hpp>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace {

constexpr double kLocalReconfigureGraceMin = 5.0;
constexpr size_t kLegacyAdvMaxBytes = 31;
constexpr size_t kAdvFlagsBytes = 3;

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

std::string normalize_direction(const std::string& raw_direction) {
    auto direction = lower_trim(raw_direction);
    if (direction == "in" || direction == "import" || direction == "rx") {
        return "import";
    }
    if (direction == "out" || direction == "export" || direction == "tx") {
        return "export";
    }
    if (direction == "both") {
        return direction;
    }
    throw std::runtime_error("direction must be import, export, or both");
}

std::vector<mrs_uav_bluetooth::config::BridgeMemberSpec> parse_member_specs(
    const std::vector<std::string>& member_paths) {
    std::vector<mrs_uav_bluetooth::config::BridgeMemberSpec> specs;
    specs.reserve(member_paths.size());
    for (const auto& raw_member : member_paths) {
        const auto separator = raw_member.find(':');
        auto path = raw_member.substr(0, separator);
        const auto path_start = path.find_first_not_of(" \t\r\n");
        if (path_start == std::string::npos) {
            throw std::runtime_error("member path must not be empty");
        }
        const auto path_end = path.find_last_not_of(" \t\r\n");
        path = path.substr(path_start, path_end - path_start + 1);

        std::string value_type = "float64";
        if (separator != std::string::npos) {
            value_type = lower_trim(raw_member.substr(separator + 1));
            if (value_type.empty()) {
                throw std::runtime_error("member type must not be empty");
            }
            value_type = mrs_uav_bluetooth::config::normalize_value_type(value_type);
        }
        specs.push_back({path, value_type});
    }
    if (specs.empty()) {
        throw std::runtime_error("member_paths must contain at least one item");
    }
    return specs;
}

std::string manual_bridge_key(const std::string& direction,
                              const std::string& mac,
                              const std::string& topic_name,
                              const std::string& message_type,
                              const std::string& characteristic,
                              const std::vector<mrs_uav_bluetooth::config::BridgeMemberSpec>& member_specs) {
    std::ostringstream source;
    source << direction << '|' << mac << '|' << topic_name << '|' << message_type << '|'
           << characteristic;
    for (const auto& spec : member_specs) {
        source << '|' << spec.path << ':' << spec.value_type;
    }
    std::string uuid = mrs_uav_bluetooth::util::uuid_from_name(source.str());
    uuid.erase(std::remove(uuid.begin(), uuid.end(), '-'), uuid.end());
    return uuid;
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

template<typename DurationT, typename CallbackT>
rclcpp::TimerBase::SharedPtr create_grouped_wall_timer(
    rclcpp::Node& node,
    DurationT period,
    CallbackT&& callback,
    const rclcpp::CallbackGroup::SharedPtr& group) {
    return rclcpp::create_wall_timer(
        period,
        std::forward<CallbackT>(callback),
        group,
        node.get_node_base_interface().get(),
        node.get_node_timers_interface().get());
}

size_t advertising_uuid_size(const std::string& uuid) {
    std::string compact;
    compact.reserve(uuid.size());
    for (const char ch : uuid) {
        if (ch != '-') {
            compact.push_back(ch);
        }
    }
    if (compact.size() == 4) {
        return 2;
    }
    if (compact.size() == 8) {
        return 4;
    }
    return 16;
}

std::vector<std::string> select_advertised_service_uuids(
    const std::vector<std::string>& service_uuids,
    const std::string& local_name) {
    int remaining_bytes = static_cast<int>(kLegacyAdvMaxBytes - kAdvFlagsBytes);
    if (!local_name.empty()) {
        remaining_bytes -= static_cast<int>(2 + local_name.size());
    }
    if (remaining_bytes <= 2) {
        return {};
    }

    std::vector<std::string> advertised;
    advertised.reserve(service_uuids.size());
    int used_bytes = 2;
    for (const auto& uuid : service_uuids) {
        const int uuid_size = static_cast<int>(advertising_uuid_size(uuid));
        if (used_bytes + uuid_size > remaining_bytes) {
            break;
        }
        advertised.push_back(uuid);
        used_bytes += uuid_size;
    }
    return advertised;
}

std::string gatt_layout_signature_for_config(const mrs_uav_bluetooth::config::NodeConfig& cfg) {
    std::vector<std::string> tokens;
    tokens.reserve(cfg.shared_topics.size() + 3);

    if (!cfg.enable_server) {
        return "server:off";
    }

    tokens.push_back("server:on");
    if (cfg.enable_wifi_service) {
        tokens.push_back("svc:wifi");
    }
    if (cfg.enable_time_service) {
        tokens.push_back("svc:time");
    }

    for (const auto& shared_topic : cfg.shared_topics) {
        if (shared_topic.mode != "export" && shared_topic.mode != "both") {
            continue;
        }
        tokens.push_back("bridge:" + shared_topic.bridge_name);
    }

    std::sort(tokens.begin(), tokens.end());
    std::ostringstream stream;
    for (size_t index = 0; index < tokens.size(); ++index) {
        if (index != 0) {
            stream << '|';
        }
        stream << tokens[index];
    }
    return stream.str();
}

}  // namespace

namespace mrs_uav_bluetooth::app {

ServiceNode::ServiceNode()
    : rclcpp::Node("mrs_uav_bluetooth") {
    configure_parameters();
    build_runtime();
}

ServiceNode::~ServiceNode() {
    shutting_down_.store(true);
    if (peer_timer_) {
        peer_timer_->cancel();
        peer_timer_.reset();
    }
    if (status_timer_) {
        status_timer_->cancel();
        status_timer_.reset();
    }
    if (lease_timer_) {
        lease_timer_->cancel();
        lease_timer_.reset();
    }
    if (time_service_timer_) {
        time_service_timer_->cancel();
        time_service_timer_.reset();
    }
    if (wifi_service_timer_) {
        wifi_service_timer_->cancel();
        wifi_service_timer_.reset();
    }
    if (client_ && gatt_event_token_ != 0) {
        client_->remove_gatt_event_handler(gatt_event_token_);
        gatt_event_token_ = 0;
    }
    if (cache_ && cache_observer_token_ != 0) {
        cache_->remove_observer(cache_observer_token_);
        cache_observer_token_ = 0;
    }
    wait_for_peer_tasks();
    if (advertisement_ && !adapter_path_.empty()) {
        try {
            advertisement_->unregister_advertisement(adapter_path_);
        } catch (...) {
        }
    }
    if (gatt_app_ && !adapter_path_.empty()) {
        try {
            gatt_app_->unregister_application(adapter_path_);
        } catch (...) {
        }
    }
}

void ServiceNode::configure_parameters() {
    const auto share_dir = ament_index_cpp::get_package_share_directory("mrs_uav_bluetooth");
    const auto default_config_path = share_dir + "/config/default.yaml";

    declare_parameter<std::string>("default_config_path", default_config_path);
    declare_parameter<std::string>("overlay_keepalive_topic_suffix", "overlay_keepalive");
    declare_parameter<bool>("enable_server", true);
    declare_parameter<bool>("enable_wifi_service", true);
    declare_parameter<bool>("enable_time_service", true);
    declare_parameter<bool>("enable_scan", true);
    declare_parameter<bool>("auto_pair", true);
    declare_parameter<bool>("auto_trust", true);
    declare_parameter<std::string>("pairing_agent", "NoInputNoOutput");
    declare_parameter<std::string>("scan_mode", "le");
    declare_parameter<int>("discoverable_timeout", 0);
}

void ServiceNode::build_runtime() {
    hostname_ = util::system_hostname();
    service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    peer_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    dbus_ = std::make_unique<bluez::DbusConnection>(
        get_logger(),
        "client",
        std::string(bluez::kLocalClientServiceName));
    server_dbus_ = std::make_unique<bluez::DbusConnection>(
        get_logger(),
        "server",
        std::string(bluez::kLocalServerServiceName));
    adapter_path_ = dbus_->find_adapter_path();

    cache_ = std::make_unique<bluez::ObjectManagerCache>(*dbus_, get_logger());
    cache_->start();

    adapter_ = std::make_unique<bluez::AdapterController>(*dbus_, adapter_path_, get_logger());
    client_ = std::make_unique<bluez::BluezClient>(
        *dbus_,
        *cache_,
        adapter_path_,
        get_logger());
    apply_adapter_state(active_config_);
    pairing_agent_ = std::make_unique<bluez::BluezPairingAgent>(
        *dbus_, get_logger(),
        get_parameter("auto_pair").as_bool(),
        get_parameter("auto_trust").as_bool());
    cache_observer_token_ = cache_->add_observer(
        [this](bluez::CacheEvent event, const std::string& object_path) {
            on_cache_event(event, object_path);
        });
    pairing_agent_->set_event_callback(
        [this](const std::string& event_type, const std::string& device_path) {
            on_pairing_event(event_type, device_path);
        });
    pairing_agent_->set_request_policy_callback(
        [this](const std::string& event_type, const std::string& device_path) {
            return should_allow_pairing_request(event_type, device_path);
        });
    pairing_agent_->register_agent(get_parameter("pairing_agent").as_string());
    client_->add_notification_handler(
        [this](const std::vector<uint8_t>& data,
               const std::string& uuid,
               const std::string& characteristic_path) {
            on_notification(data, uuid, characteristic_path);
        });
    gatt_event_token_ = client_->add_gatt_event_handler(
        [this](const std::string& event_type,
               const std::string& object_path,
               const std::string& detail) {
            on_gatt_event(event_type, object_path, detail);
        });

    overlay_config_ = std::make_unique<config::OverlayConfigManager>(
        *this,
        get_logger(),
        get_parameter("default_config_path").as_string(),
        hostname_);
    overlay_config_->on_config_changed([this](const config::NodeConfig& cfg) {
        if (!can_run_callbacks()) {
            return;
        }
        apply_config(cfg);
    });

    netplan_ = std::make_unique<network::NetplanManager>();

    export_bridges_ = std::make_unique<bridge::ExportBridgeManager>(*this, get_logger());
    export_bridges_->set_registry(&bridge_registry_);
    export_bridges_->set_state_mutex(&state_mutex_);
    import_bridges_ = std::make_unique<bridge::ImportBridgeManager>(*this, get_logger());
    import_bridges_->set_registry(&bridge_registry_);
    import_bridges_->set_state_mutex(&state_mutex_);
    peers_ = std::make_unique<peer::PeerManager>(*this, get_logger());
    ros_ = std::make_unique<ros::RosInterfaceManager>(*this);
    create_services();

    status_timer_ = create_grouped_wall_timer(*this, std::chrono::seconds(2), [this]() {
        if (!can_run_callbacks()) {
            return;
        }
        publish_periodic_status();
    }, timer_callback_group_);
    lease_timer_ = create_grouped_wall_timer(*this, std::chrono::seconds(1), [this]() {
        if (!can_run_callbacks()) {
            return;
        }
        overlay_config_->check_lease();
    }, timer_callback_group_);

    overlay_config_->load_initial();
}

bool ServiceNode::can_run_callbacks() const {
    return !shutting_down_.load() && rclcpp::ok();
}

void ServiceNode::apply_adapter_state(const config::NodeConfig& cfg) {
    if (!adapter_) {
        return;
    }

    const auto adapter_info = cache_ ? cache_->adapter(adapter_path_) : std::optional<bluez::AdapterInfo>{};
    const bool should_be_discoverable = cfg.enable_server;

    if (!adapter_info || !adapter_info->powered) {
        adapter_->power_on();
    }
    if (!adapter_info || adapter_info->alias != hostname_) {
        adapter_->set_alias(hostname_);
    }
    if (!adapter_info || !adapter_info->pairable) {
        adapter_->set_pairable(true);
    }
    adapter_->set_pairable_timeout(0);

    if (!adapter_info ||
        adapter_info->discoverable != should_be_discoverable ||
        (should_be_discoverable && adapter_info->discoverable_timeout != cfg.discoverable_timeout)) {
        adapter_->set_discoverable(should_be_discoverable, cfg.discoverable_timeout);
    }
}

void ServiceNode::create_services() {
    ros::ServiceServers::Handlers handlers;
    handlers.list_devices = [this](auto request, auto response) { handle_list_devices(request, response); };
    handlers.get_device = [this](auto request, auto response) { handle_get_device(request, response); };
    handlers.connect_device = [this](auto request, auto response) { handle_connect_device(request, response); };
    handlers.disconnect_device = [this](auto request, auto response) { handle_disconnect_device(request, response); };
    handlers.pair_device = [this](auto request, auto response) { handle_pair_device(request, response); };
    handlers.set_device_trust = [this](auto request, auto response) { handle_set_device_trust(request, response); };
    handlers.remove_device = [this](auto request, auto response) { handle_remove_device(request, response); };
    handlers.list_gatt_services = [this](auto request, auto response) { handle_list_gatt_services(request, response); };
    handlers.list_gatt_characteristics = [this](auto request, auto response) { handle_list_gatt_characteristics(request, response); };
    handlers.list_gatt_descriptors = [this](auto request, auto response) { handle_list_gatt_descriptors(request, response); };
    handlers.find_gatt_path = [this](auto request, auto response) { handle_find_gatt_path(request, response); };
    handlers.read_gatt_value = [this](auto request, auto response) { handle_read_gatt_value(request, response); };
    handlers.write_gatt_value = [this](auto request, auto response) { handle_write_gatt_value(request, response); };
    handlers.set_notify = [this](auto request, auto response) { handle_set_notify(request, response); };
    handlers.set_scan_enabled = [this](auto request, auto response) { handle_set_scan_enabled(request, response); };
    handlers.configure_notification_bridge = [this](auto request, auto response) {
        handle_configure_notification_bridge(request, response);
    };
    handlers.reload_config = [this](auto request, auto response) { handle_reload_config(request, response); };
    handlers.set_active_config = [this](auto request, auto response) { handle_set_active_config(request, response); };

    services_ = ros_->service_servers().register_all(*this, handlers, service_callback_group_);
}

void ServiceNode::apply_config(const config::NodeConfig& cfg) {
    if (!can_run_callbacks()) {
        return;
    }

    const auto new_gatt_layout_signature = gatt_layout_signature_for_config(cfg);
    const bool gatt_layout_changed = !local_gatt_layout_signature_.empty() &&
        local_gatt_layout_signature_ != new_gatt_layout_signature;

    active_config_ = cfg;
    if (pairing_agent_) {
        pairing_agent_->set_auto_pair(cfg.auto_pair);
        pairing_agent_->set_auto_trust(cfg.auto_trust);
    }
    ros_->status_publisher().configure_topics(cfg.node_topics_prefix);
    apply_adapter_state(cfg);

    if (status_timer_) {
        status_timer_->cancel();
        status_timer_.reset();
    }
    if (cfg.status_report_period > 0.0) {
        status_timer_ = create_grouped_wall_timer(
            *this,
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(cfg.status_report_period)),
            [this]() {
                publish_periodic_status();
            },
            timer_callback_group_);
    }

    {
        std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
        for (auto it = bridge_registry_.exports().begin(); it != bridge_registry_.exports().end();) {
            if (it->second.auto_managed) {
                export_bridges_->destroy_export_bridge(it->second);
                it = bridge_registry_.exports().erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = bridge_registry_.imports().begin(); it != bridge_registry_.imports().end();) {
            if (it->second.auto_managed) {
                import_bridges_->destroy_import_bridge(it->second);
                it = bridge_registry_.imports().erase(it);
            } else {
                ++it;
            }
        }

        for (const auto& shared_topic : cfg.shared_topics) {
            if (shared_topic.mode == "export" || shared_topic.mode == "both") {
                bridge::TopicExportBridgeState state;
                state.topic_name = shared_topic.export_topic;
                state.message_type = shared_topic.message_type;
                state.bridge_name = shared_topic.bridge_name;
                state.bridge_key = shared_topic.bridge_key;
                state.bridge_uuid = util::named_characteristic_uuid(
                    bridge_characteristic_name_for_service(shared_topic.bridge_name));
                state.member_specs = shared_topic.member_specs;
                state.rate_hz = shared_topic.rate_hz;
                state.payload_format = shared_topic.payload_format;
                state.auto_managed = true;
                auto [it, inserted] = bridge_registry_.exports().insert_or_assign(shared_topic.bridge_key, std::move(state));
                (void)inserted;
                export_bridges_->configure_export_bridge(shared_topic.bridge_key, it->second);
                export_bridges_->configure_export_rate_timer(shared_topic.bridge_key, it->second);
            }

        }
    }

    netplan_->set_allowed_networks(cfg.allowed_wifi_networks);
    if (cfg.enable_scan) {
        client_->start_scan(cfg.scan_mode, cfg.enable_server);
    } else {
        client_->stop_scan();
    }

    if (peers_ && client_) {
        for (const auto& device : client_->get_devices()) {
            {
                std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
                peers_->sync_device(device, active_config_, device_hostname_guess(device));
            }
            refresh_import_bridges_for_device(device);
        }

    }

    if (gatt_layout_changed) {
        RCLCPP_WARN(get_logger(),
                    "Local GATT layout changed, rebuilding the server without clearing active peer runtime");
    }

    if (!can_run_callbacks()) {
        return;
    }

    {
        std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
        local_server_rebuild_monotonic_ = peers_ ? peers_->now_monotonic()
                                                 : std::chrono::duration<double>(
                                                       std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    rebuild_server_objects();
    {
        std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
        local_server_rebuild_monotonic_ = peers_ ? peers_->now_monotonic()
                                                 : std::chrono::duration<double>(
                                                       std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    local_gatt_layout_signature_ = new_gatt_layout_signature;

    if (time_service_timer_) {
        time_service_timer_->cancel();
        time_service_timer_.reset();
    }
    if (!can_run_callbacks()) {
        return;
    }
    if (cfg.enable_time_service && cfg.time_update_period > 0.0) {
        time_service_timer_ = create_grouped_wall_timer(
            *this,
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(cfg.time_update_period)),
            [this]() {
                if (time_service_) {
                    time_service_->update();
                }
            },
            timer_callback_group_);
    }

    if (wifi_service_timer_) {
        wifi_service_timer_->cancel();
        wifi_service_timer_.reset();
    }
    if (!can_run_callbacks()) {
        return;
    }
    if (cfg.enable_wifi_service && cfg.wifi_refresh_period > 0.0) {
        wifi_service_timer_ = create_grouped_wall_timer(
            *this,
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(cfg.wifi_refresh_period)),
            [this]() {
                if (wifi_service_ && netplan_) {
                    auto ssid = netplan_->get_current_ssid();
                    wifi_service_->update(ssid.empty() ? "unknown" : ssid);
                }
            },
            timer_callback_group_);
    }

    schedule_peer_reconcile();
}

void ServiceNode::rebuild_server_objects() {
    if (!can_run_callbacks()) {
        return;
    }

    RCLCPP_INFO(get_logger(), "[node] rebuild_server_objects: server=%s adv=%s",
                gatt_app_ ? "active" : "null", advertisement_ ? "active" : "null");
    if (advertisement_ && !adapter_path_.empty()) {
        try {
            advertisement_->unregister_advertisement(adapter_path_);
        } catch (...) {
        }
        advertisement_.reset();
    }
    if (gatt_app_ && !adapter_path_.empty()) {
        try {
            gatt_app_->unregister_application(adapter_path_);
        } catch (...) {
        }
        gatt_app_.reset();
    }

    if (!active_config_.enable_server) {
        wifi_service_.reset();
        time_service_.reset();
        return;
    }

    gatt_app_ = std::make_unique<gatt::GattApplication>(*server_dbus_, "/org/bluez/app", get_logger());

    int service_index = 0;
    if (active_config_.enable_wifi_service) {
        wifi_service_ = std::make_unique<gatt::services::WifiService>(
            *server_dbus_, "/org/bluez/app", service_index++,
            [this]() {
                auto ssid = netplan_->get_current_ssid();
                return ssid.empty() ? std::string("unknown") : ssid;
            },
            [this](const std::string& ssid) { netplan_->set_current_network(ssid); },
            [this](const std::string& password) {
                netplan_->set_current_network(netplan_->get_current_ssid(), password);
            },
            [this]() { return netplan_->get_configured_password(); });
        gatt_app_->add_service(wifi_service_->service());
    } else {
        wifi_service_.reset();
    }

    if (active_config_.enable_time_service) {
        time_service_ = std::make_unique<gatt::services::TimeService>(
            *server_dbus_, "/org/bluez/app", service_index++,
            [this](const std::vector<uint8_t>& payload,
                   const std::string& device_path,
                   uint64_t received_time_ns) {
                handle_time_writeback(payload, device_path, received_time_ns);
            });
        // Wire server-side notify observability for the time characteristic.
        for (const auto& chrc : time_service_->service()->characteristics()) {
            chrc->set_force_emit_value(true);
            chrc->set_notify_callback([this, uuid = chrc->uuid()](bool enabled) {
                RCLCPP_INFO(get_logger(), "[server] time characteristic %s: client %s notifications",
                            uuid.c_str(), enabled ? "started" : "stopped");
            });
        }
        gatt_app_->add_service(time_service_->service());
    } else {
        time_service_.reset();
    }

    export_bridges_->rebuild_gatt_services(*gatt_app_, *server_dbus_, "/org/bluez/app");
    gatt_app_->register_application(adapter_path_);

    RCLCPP_INFO(get_logger(), "[node] GATT server registered, setting up advertisement");
    advertisement_ = std::make_unique<gatt::Advertisement>(
        *server_dbus_, "/org/bluez/advertisement0", "peripheral");
    advertisement_->set_local_name(hostname_);
    advertisement_->set_discoverable(active_config_.enable_server);
    advertisement_->set_discoverable_timeout(
        static_cast<uint16_t>(std::min<uint32_t>(active_config_.discoverable_timeout,
                                                 std::numeric_limits<uint16_t>::max())));

    std::vector<std::string> service_uuids;
    service_uuids.reserve(gatt_app_->services().size());
    for (const auto& service : gatt_app_->services()) {
        service_uuids.push_back(service->uuid());
    }

    const auto advertised_service_uuids = select_advertised_service_uuids(service_uuids, hostname_);
    if (advertised_service_uuids.size() != service_uuids.size()) {
        RCLCPP_WARN(get_logger(),
                    "BLE advertisement trimmed from %zu to %zu service UUIDs to fit legacy controller limits",
                    service_uuids.size(), advertised_service_uuids.size());
    }

    std::vector<std::vector<std::string>> attempts;
    attempts.push_back(advertised_service_uuids);
    if (!advertised_service_uuids.empty()) {
        attempts.push_back({});
    }

    bool advertisement_registered = false;
    std::string last_error_message;
    for (const auto& advertised_uuids : attempts) {
        advertisement_->set_service_uuids(advertised_uuids);
        try {
            RCLCPP_INFO(get_logger(),
                        "Registering BLE advertisement (name=%s, uuids=%zu)",
                        hostname_.c_str(), advertised_uuids.size());
            advertisement_->register_advertisement(adapter_path_);
            advertisement_registered = true;
            break;
        } catch (const sdbus::Error& error) {
            last_error_message = error.what();
            RCLCPP_WARN(get_logger(),
                        "BLE advertisement registration attempt failed (uuids=%zu): %s",
                        advertised_uuids.size(), error.what());
        }
    }

    if (!advertisement_registered) {
        throw std::runtime_error(last_error_message.empty() ?
                                     "Failed to register advertisement" :
                                     last_error_message);
    }
}

void ServiceNode::handle_configure_notification_bridge(const std::shared_ptr<mrs_uav_bluetooth::srv::ConfigureNotificationBridge::Request> request,
                                                         std::shared_ptr<mrs_uav_bluetooth::srv::ConfigureNotificationBridge::Response> response) {
    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    try {
        const auto direction = normalize_direction(request->direction);
        const auto message_type = lower_trim(request->message_type);
        const auto resolved_topic = util::normalize_ros_topic(request->topic_name);
        const auto member_specs = parse_member_specs(request->member_paths);

        if (resolved_topic == "/") {
            throw std::runtime_error("topic_name must not be empty");
        }
        if (message_type.empty()) {
            throw std::runtime_error("message_type must not be empty");
        }

        std::string request_uuid;
        if (direction == "import" || direction == "both") {
            if (request->mac.empty()) {
                throw std::runtime_error("mac must not be empty for import bridges");
            }
            request_uuid = util::resolve_uuid(request->characteristic);
        }

        const auto export_key = manual_bridge_key(
            "export", request->mac, resolved_topic, message_type,
            request->characteristic, member_specs);
        const auto import_key = manual_bridge_key(
            "import", request->mac, resolved_topic, message_type,
            request->characteristic, member_specs);

        bool changed_exports = false;
        bool changed_imports = false;
        std::string resolved_path;
        std::string resolved_uuid;

        if (!request->enable) {
            if ((direction == "export" || direction == "both")) {
                auto export_it = bridge_registry_.exports().find(export_key);
                if (export_it != bridge_registry_.exports().end()) {
                    export_bridges_->destroy_export_bridge(export_it->second);
                    bridge_registry_.exports().erase(export_it);
                    changed_exports = true;
                }
            }
            if ((direction == "import" || direction == "both")) {
                auto import_it = bridge_registry_.imports().find(import_key);
                if (import_it != bridge_registry_.imports().end()) {
                    import_bridges_->destroy_import_bridge(import_it->second);
                    bridge_registry_.imports().erase(import_it);
                    changed_imports = true;
                }
            }
            if (changed_exports && active_config_.enable_server) {
                rebuild_server_objects();
            }
            response->success = changed_exports || changed_imports;
            response->message = response->success ? "bridge disabled" : "bridge not found";
            response->resolved_topic = resolved_topic;
            response->resolved_message_type = message_type;
            response->resolved_member_paths = request->member_paths;
            response->resolved_rate_hz = request->rate_hz;
            return;
        }

        if (direction == "export" || direction == "both") {
            if (!active_config_.enable_server) {
                throw std::runtime_error("enable_server must be true for export bridges");
            }
            bridge::TopicExportBridgeState state;
            state.topic_name = resolved_topic;
            state.message_type = message_type;
            state.bridge_name = export_key;
            state.bridge_key = export_key;
            state.bridge_uuid = util::named_characteristic_uuid(export_key + "/value");
            state.member_specs = member_specs;
            state.rate_hz = std::max(0.0f, request->rate_hz);
            state.payload_format = "struct";
            state.auto_managed = false;
            auto [export_it, inserted] = bridge_registry_.exports().insert_or_assign(export_key, std::move(state));
            (void)inserted;
            export_bridges_->configure_export_bridge(export_key, export_it->second);
            export_bridges_->configure_export_rate_timer(export_key, export_it->second);
            changed_exports = true;
            resolved_uuid = export_it->second.bridge_uuid;
        }

        if (direction == "import" || direction == "both") {
            if (!client_) {
                throw std::runtime_error("BlueZ client is not initialized");
            }

            resolved_path = client_->find_characteristic(request->mac, request_uuid);
            if (resolved_path.empty()) {
                throw std::runtime_error("requested remote bridge path was not found");
            }

            bridge::TopicImportBridgeState state;
            state.mac = request->mac;
            state.requested_topic_name = resolved_topic;
            state.resolved_topic_name = resolved_topic;
            state.message_type = message_type;
            state.bridge_name = import_key;
            state.bridge_key = import_key;
            state.bridge_uuid = request_uuid;
            state.member_specs = member_specs;
            state.rate_hz = std::max(0.0f, request->rate_hz);
            state.payload_format = "struct";
            state.path = resolved_path;
            state.auto_managed = false;
            auto [import_it, inserted] = bridge_registry_.imports().insert_or_assign(import_key, std::move(state));
            (void)inserted;
            import_bridges_->configure_import_bridge(import_key, import_it->second, *client_);
            changed_imports = true;
            resolved_uuid = request_uuid;
        }

        if (changed_exports) {
            rebuild_server_objects();
            auto export_it = bridge_registry_.exports().find(export_key);
            if (export_it != bridge_registry_.exports().end() && export_it->second.service) {
                resolved_path = export_it->second.service->transport_path();
                resolved_uuid = export_it->second.bridge_uuid;
            }
        }

        response->success = true;
        response->message = changed_imports && changed_exports
            ? "manual import and export bridges configured"
            : (changed_imports ? "manual import bridge configured" : "manual export bridge configured");
        response->resolved_uuid = resolved_uuid;
        response->resolved_path = resolved_path;
        response->resolved_topic = resolved_topic;
        response->resolved_message_type = message_type;
        response->resolved_rate_hz = std::max(0.0f, request->rate_hz);
        for (const auto& spec : member_specs) {
            response->resolved_member_paths.push_back(spec.path + ":" + spec.value_type);
        }
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

}  // namespace mrs_uav_bluetooth::app
