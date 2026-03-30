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
#include <iomanip>
#include <limits>
#include <rclcpp/create_timer.hpp>
#include <stdexcept>
#include <sstream>
#include <string_view>

namespace {

constexpr double kPeerRepairCooldownMin = 8.0;
constexpr double kBridgeGraceMin = 8.0;
constexpr auto kStatusSummaryLogInterval = std::chrono::seconds(15);
constexpr auto kPeerStatusLogInterval = std::chrono::seconds(30);
constexpr auto kGattReadWriteLogInterval = std::chrono::seconds(10);
constexpr auto kRepeatedLogWindow = std::chrono::seconds(5);
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

std::string bridge_characteristic_name_for_host(const std::string& host,
                                                const std::string& bridge_topic_path) {
    return "/" + mrs_uav_bluetooth::util::sanitize_topic_suffix(host) + bridge_topic_path;
}

std::string bridge_service_name_for_host(const std::string& host,
                                         const std::string& bridge_topic_path) {
    return "bridge:" + bridge_characteristic_name_for_host(host, bridge_topic_path);
}

std::string peer_topic_token(const std::string& peer_name, const std::string& mac) {
    auto token = mrs_uav_bluetooth::util::sanitize_topic_suffix(peer_name);
    if (token.empty() || token == "ble_device") {
        token = "peer_" + mrs_uav_bluetooth::util::sanitize_topic_suffix(mac);
    }
    if (!token.empty() && std::isdigit(static_cast<unsigned char>(token.front())) != 0) {
        token = "peer_" + token;
    }
    return token;
}

std::string trim_topic_segment(const std::string& value) {
    const auto start = value.find_first_not_of(" \t\r\n/");
    if (start == std::string::npos) {
        return {};
    }
    const auto end = value.find_last_not_of(" \t\r\n/");
    return value.substr(start, end - start + 1);
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

std::pair<double, double> update_publish_rate(double last_publish_monotonic) {
    const auto now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (last_publish_monotonic > 0.0 && now > last_publish_monotonic) {
        return {now, 1.0 / (now - last_publish_monotonic)};
    }
    return {now, 0.0};
}

std::string peer_status_snapshot(const std::string& mac,
                                 const std::string& device_label,
                                 const mrs_uav_bluetooth::peer::PeerConnectionSession& session,
                                 bool connected,
                                 bool services_resolved,
                                 const std::string& bridge_status) {
    std::ostringstream stream;
    stream << "peer=" << mac
           << " device='" << device_label << "'"
           << " peer='" << session.peer_name << "'"
           << " phase=" << session.phase
           << " desired=" << (session.desired ? "Y" : "N")
           << " conn=" << (connected ? "Y" : "N")
           << " svc=" << (services_resolved ? "Y" : "N")
           << " bridge=" << bridge_status
           << " detail=" << session.detail;
    return stream.str();
}

bool is_read_write_gatt_event(const std::string& event_type) {
    return event_type == "client_read" ||
           event_type == "client_write" ||
           event_type == "client_descriptor_read" ||
           event_type == "client_descriptor_write";
}

bool is_failed_gatt_event(const std::string& event_type) {
    constexpr auto suffix = "_failed";
    return event_type.size() >= std::char_traits<char>::length(suffix) &&
           event_type.compare(event_type.size() - std::char_traits<char>::length(suffix),
                              std::char_traits<char>::length(suffix),
                              suffix) == 0;
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

std::optional<std::string> device_path_for_cache_event(
    const mrs_uav_bluetooth::bluez::ObjectManagerCache& cache,
    mrs_uav_bluetooth::bluez::CacheEvent event,
    const std::string& object_path) {
    using mrs_uav_bluetooth::bluez::CacheEvent;

    if (event == CacheEvent::DeviceAdded || event == CacheEvent::DevicePropertyChanged) {
        return object_path;
    }
    if (event == CacheEvent::GattServiceAdded || event == CacheEvent::GattServiceRemoved) {
        if (const auto service = cache.service(object_path); service && !service->device_path.empty()) {
            return service->device_path;
        }
    }
    if (event == CacheEvent::GattCharacteristicAdded ||
        event == CacheEvent::GattCharacteristicChanged ||
        event == CacheEvent::GattCharacteristicValueChanged ||
        event == CacheEvent::GattCharacteristicRemoved) {
        if (const auto characteristic = cache.characteristic(object_path)) {
            if (const auto service = cache.service(characteristic->service_path); service && !service->device_path.empty()) {
                return service->device_path;
            }
        }
    }
    if (event == CacheEvent::GattDescriptorAdded ||
        event == CacheEvent::GattDescriptorChanged ||
        event == CacheEvent::GattDescriptorValueChanged ||
        event == CacheEvent::GattDescriptorRemoved) {
        if (const auto descriptor = cache.descriptor(object_path)) {
            if (const auto characteristic = cache.characteristic(descriptor->characteristic_path)) {
                if (const auto service = cache.service(characteristic->service_path); service && !service->device_path.empty()) {
                    return service->device_path;
                }
            }
        }
    }
    const auto service_pos = object_path.find("/service");
    if (service_pos != std::string::npos) {
        return object_path.substr(0, service_pos);
    }
    return std::nullopt;
}

std::optional<std::string> device_path_for_gatt_object(
    const mrs_uav_bluetooth::bluez::ObjectManagerCache& cache,
    const std::string& object_path) {
    if (const auto device = cache.device(object_path)) {
        return device->object_path;
    }
    if (const auto service = cache.service(object_path)) {
        return service->device_path.empty() ? std::nullopt : std::optional<std::string>(service->device_path);
    }
    if (const auto characteristic = cache.characteristic(object_path)) {
        if (const auto service = cache.service(characteristic->service_path); service && !service->device_path.empty()) {
            return service->device_path;
        }
    }
    if (const auto descriptor = cache.descriptor(object_path)) {
        if (const auto characteristic = cache.characteristic(descriptor->characteristic_path)) {
            if (const auto service = cache.service(characteristic->service_path); service && !service->device_path.empty()) {
                return service->device_path;
            }
        }
    }
    const auto service_pos = object_path.find("/service");
    if (service_pos != std::string::npos) {
        return object_path.substr(0, service_pos);
    }
    return std::nullopt;
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
    wait_for_peer_tasks();
    if (client_ && gatt_event_token_ != 0) {
        client_->remove_gatt_event_handler(gatt_event_token_);
        gatt_event_token_ = 0;
    }
    if (cache_ && cache_observer_token_ != 0) {
        cache_->remove_observer(cache_observer_token_);
        cache_observer_token_ = 0;
    }
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
    declare_parameter<bool>("auto_accept_pairing", true);
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

    dbus_ = std::make_unique<bluez::DbusConnection>(get_logger(), "client");
    server_dbus_ = std::make_unique<bluez::DbusConnection>(get_logger(), "server");
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
        get_parameter("auto_accept_pairing").as_bool(),
        get_parameter("auto_trust").as_bool());
    cache_observer_token_ = cache_->add_observer(
        [this](bluez::CacheEvent event, const std::string& object_path) {
            on_cache_event(event, object_path);
        });
    pairing_agent_->set_event_callback(
        [this](const std::string& event_type, const std::string& device_path) {
            on_pairing_event(event_type, device_path);
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
        apply_config(cfg);
    });

    netplan_ = std::make_unique<network::NetplanManager>();

    export_bridges_ = std::make_unique<bridge::ExportBridgeManager>(*this, get_logger());
    export_bridges_->set_registry(&bridge_registry_);
    import_bridges_ = std::make_unique<bridge::ImportBridgeManager>(*this, get_logger());
    import_bridges_->set_registry(&bridge_registry_);
    peers_ = std::make_unique<peer::PeerManager>(*this, get_logger());
    ros_ = std::make_unique<ros::RosInterfaceManager>(*this);
    create_services();

    status_timer_ = create_grouped_wall_timer(*this, std::chrono::seconds(2), [this]() {
        publish_periodic_status();
    }, timer_callback_group_);
    lease_timer_ = create_grouped_wall_timer(*this, std::chrono::seconds(1), [this]() {
        overlay_config_->check_lease();
    }, timer_callback_group_);

    overlay_config_->load_initial();
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
    active_config_ = cfg;
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

    netplan_->set_allowed_networks(cfg.allowed_wifi_networks);
    if (cfg.enable_scan) {
        client_->start_scan(cfg.scan_mode, cfg.enable_server);
    } else {
        client_->stop_scan();
    }

    if (peers_ && client_) {
        for (const auto& device : client_->get_devices()) {
            peers_->sync_device(device, active_config_, device_hostname_guess(device));
            refresh_import_bridges_for_device(device);
        }
    }
    rebuild_server_objects();

    if (time_service_timer_) {
        time_service_timer_->cancel();
        time_service_timer_.reset();
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

void ServiceNode::publish_periodic_status() {
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
    size_t passive_peers = 0;
    if (peers_) {
        for (const auto& [mac, session] : peers_->sessions()) {
            auto dev_it = devices_map.find(mac);
            const bool conn = dev_it != devices_map.end() && dev_it->second.connected;
            const bool svc = dev_it != devices_map.end() && dev_it->second.services_resolved;
            auto bridge_it = peers_->time_bridges().find(mac);
            std::string bridge_status = bridge_it != peers_->time_bridges().end() ? bridge_it->second.status : "none";
            // Show raw device name/alias separately from derived peer_name for debuggability
            std::string device_label;
            if (dev_it != devices_map.end()) {
                if (!dev_it->second.name.empty()) {
                    device_label = dev_it->second.name;
                } else if (!dev_it->second.alias.empty()) {
                    device_label = dev_it->second.alias;
                }
            }
            if (!is_interesting_peer_status(session, conn, svc, bridge_status)) {
                passive_peers += 1;
                continue;
            }
            peer_snapshots.emplace_back(mac,
                                        peer_status_snapshot(mac, device_label, session, conn, svc, bridge_status));
        }
    }

    const auto summary = "state=" + status_word +
        " devices=" + std::to_string(devices_map.size()) +
        " connected=" + std::to_string(static_cast<size_t>(connected_count)) +
        " sessions=" + std::to_string(peers_ ? peers_->sessions().size() : 0u) +
        " interesting_peers=" + std::to_string(peer_snapshots.size()) +
        " passive_peers=" + std::to_string(passive_peers) +
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
        if (device->paired && device->trusted && device->bonded && bridge != nullptr && bridge->status == "ready") {
            status_parts.push_back("fully-paired");
            status_parts.push_back("time-bridge-ready");
        } else {
            status_parts.push_back(device->paired || device->bonded ? "paired" : "pairing-pending");
            status_parts.push_back(device->trusted ? "trusted" : "trust-pending");
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
                        " paired=" + std::string(bool_text(device->paired && device->trusted && device->bonded)) +
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
    if (!bridge_registry_.imports().empty()) {
        lines.push_back("  import bridges (" + std::to_string(bridge_registry_.imports().size()) + "):");
        for (const auto& [_, state] : bridge_registry_.imports()) {
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

void ServiceNode::on_cache_event(bluez::CacheEvent event, const std::string& object_path) {
    if (!peers_ || !cache_) {
        return;
    }

    // Skip high-frequency value changes
    if (event == bluez::CacheEvent::GattCharacteristicValueChanged ||
        event == bluez::CacheEvent::GattDescriptorValueChanged) {
        return;
    }

    // Log cache events with correct enum-to-name mapping
    const char* event_name = [](bluez::CacheEvent e) -> const char* {
        switch (e) {
            case bluez::CacheEvent::DeviceAdded:                    return "DeviceAdded";
            case bluez::CacheEvent::DeviceRemoved:                  return "DeviceRemoved";
            case bluez::CacheEvent::DevicePropertyChanged:          return "DevicePropertyChanged";
            case bluez::CacheEvent::GattServiceAdded:               return "GattServiceAdded";
            case bluez::CacheEvent::GattServiceRemoved:             return "GattServiceRemoved";
            case bluez::CacheEvent::GattCharacteristicAdded:        return "GattCharacteristicAdded";
            case bluez::CacheEvent::GattCharacteristicChanged:      return "GattCharacteristicChanged";
            case bluez::CacheEvent::GattCharacteristicValueChanged: return "GattCharacteristicValueChanged";
            case bluez::CacheEvent::GattCharacteristicRemoved:      return "GattCharacteristicRemoved";
            case bluez::CacheEvent::GattDescriptorAdded:            return "GattDescriptorAdded";
            case bluez::CacheEvent::GattDescriptorChanged:          return "GattDescriptorChanged";
            case bluez::CacheEvent::GattDescriptorValueChanged:     return "GattDescriptorValueChanged";
            case bluez::CacheEvent::GattDescriptorRemoved:          return "GattDescriptorRemoved";
            case bluez::CacheEvent::AdapterChanged:                 return "AdapterChanged";
            default:                                                return "Unknown";
        }
    }(event);

    if (event == bluez::CacheEvent::DeviceAdded) {
        auto device = cache_->device(object_path);
        if (!device) {
            log_warn_coalesced("cache:DeviceAdded-missing:" + object_path,
                               "[node] on_cache_event: DeviceAdded path=" + object_path + " but device not in cache");
            return;
        }
        const auto peer_name = device_hostname_guess(*device);
        log_info_coalesced("cache:DeviceAdded:" + device->mac,
                           "[node] on_cache_event: DeviceAdded mac=" + device->mac +
                               " name='" + device->name + "' peer_name='" + peer_name +
                               "' path=" + object_path);
        peers_->sync_device(*device, active_config_, peer_name);
        refresh_import_bridges_for_device(*device);
    } else if (event == bluez::CacheEvent::DevicePropertyChanged) {
        auto device = cache_->device(object_path);
        if (!device) {
            return;
        }
        RCLCPP_DEBUG(get_logger(), "[node] on_cache_event: DevicePropertyChanged mac=%s conn=%s paired=%s trusted=%s svc_resolved=%s path=%s",
                     device->mac.c_str(),
                     device->connected ? "Y" : "N",
                     device->paired ? "Y" : "N",
                     device->trusted ? "Y" : "N",
                     device->services_resolved ? "Y" : "N",
                     object_path.c_str());
        peers_->sync_device(*device, active_config_, device_hostname_guess(*device));
        refresh_import_bridges_for_device(*device);
    } else if (event == bluez::CacheEvent::DeviceRemoved) {
        // Extract MAC from the removed object_path since the device is no longer in cache.
        // Only mark the specific device as missing, not all sessions.
        std::string removed_mac;
        auto session_it = std::find_if(peers_->sessions().begin(), peers_->sessions().end(),
            [&object_path, this](const auto& pair) {
                // Match by D-Bus path: compare with cached device path
                // Object paths are like /org/bluez/hci0/dev_XX_XX_XX_XX_XX_XX
                const auto& mac = pair.first;
                auto dev = cache_->device_by_mac(mac);
                return dev && dev->object_path == object_path;
            });
        if (session_it != peers_->sessions().end()) {
            removed_mac = session_it->first;
        } else {
            // Device already gone from cache, try to find MAC from path
            // Path format: /org/bluez/hci0/dev_XX_XX_XX_XX_XX_XX
            const auto dev_pos = object_path.rfind("/dev_");
            if (dev_pos != std::string::npos) {
                auto mac_part = object_path.substr(dev_pos + 5);
                std::replace(mac_part.begin(), mac_part.end(), '_', ':');
                removed_mac = mac_part;
            }
        }

        if (!removed_mac.empty()) {
            log_info_coalesced("cache:DeviceRemoved:" + removed_mac,
                               "[node] on_cache_event: DeviceRemoved mac=" + removed_mac +
                                   " path=" + object_path);
            if (peers_->sessions().count(removed_mac)) {
                peers_->note_missing_device(removed_mac, peers_->now_monotonic());
                clear_peer_runtime(removed_mac);
            }
        } else {
            RCLCPP_DEBUG(get_logger(), "[node] on_cache_event: DeviceRemoved path=%s (no matching session)",
                         object_path.c_str());
        }
    } else if (event == bluez::CacheEvent::GattServiceAdded || event == bluez::CacheEvent::GattServiceRemoved) {
        log_info_coalesced(std::string{"cache:"} + event_name + ":" + object_path,
                           std::string{"[node] on_cache_event: "} + event_name + " path=" + object_path);
        if (const auto device_path = device_path_for_cache_event(*cache_, event, object_path)) {
            if (const auto device = cache_->device(*device_path)) {
                peers_->sync_device(*device, active_config_, device_hostname_guess(*device));
                refresh_import_bridges_for_device(*device);
            }
        }
    } else if (event == bluez::CacheEvent::GattCharacteristicAdded || event == bluez::CacheEvent::GattCharacteristicRemoved) {
        if (const auto chrc = cache_->characteristic(object_path)) {
            log_info_coalesced(std::string{"cache:"} + event_name + ":" + object_path,
                               std::string{"[node] on_cache_event: "} + event_name +
                                   " uuid=" + chrc->uuid + " path=" + object_path);
        } else {
            log_info_coalesced(std::string{"cache:"} + event_name + ":" + object_path,
                               std::string{"[node] on_cache_event: "} + event_name + " path=" + object_path);
        }
        if (const auto device_path = device_path_for_cache_event(*cache_, event, object_path)) {
            if (const auto device = cache_->device(*device_path)) {
                peers_->sync_device(*device, active_config_, device_hostname_guess(*device));
                refresh_import_bridges_for_device(*device);
            }
        }
    } else if (event == bluez::CacheEvent::GattDescriptorAdded || event == bluez::CacheEvent::GattDescriptorRemoved) {
        if (const auto desc = cache_->descriptor(object_path)) {
            log_info_coalesced(std::string{"cache:"} + event_name + ":" + object_path,
                               std::string{"[node] on_cache_event: "} + event_name +
                                   " uuid=" + desc->uuid + " chrc=" + desc->characteristic_path +
                                   " path=" + object_path);
        } else {
            log_info_coalesced(std::string{"cache:"} + event_name + ":" + object_path,
                               std::string{"[node] on_cache_event: "} + event_name + " path=" + object_path);
        }
        if (const auto device_path = device_path_for_cache_event(*cache_, event, object_path)) {
            if (const auto device = cache_->device(*device_path)) {
                peers_->sync_device(*device, active_config_, device_hostname_guess(*device));
                refresh_import_bridges_for_device(*device);
            }
        }
    } else {
        RCLCPP_DEBUG(get_logger(), "[node] on_cache_event: %s path=%s", event_name, object_path.c_str());
        if (event == bluez::CacheEvent::AdapterChanged && object_path == adapter_path_) {
            if (const auto adapter = cache_->adapter(object_path)) {
                const bool should_be_discoverable = active_config_.enable_server;
                const bool drifted = !adapter->powered ||
                    !adapter->pairable ||
                    adapter->alias != hostname_ ||
                    adapter->discoverable != should_be_discoverable ||
                    (should_be_discoverable && adapter->discoverable_timeout != active_config_.discoverable_timeout);
                if (drifted) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "[node] adapter state drift detected, reapplying powered/pairable/discoverable/alias settings");
                    apply_adapter_state(active_config_);
                }
            }
        }
        if (const auto device_path = device_path_for_cache_event(*cache_, event, object_path)) {
            if (const auto device = cache_->device(*device_path)) {
                peers_->sync_device(*device, active_config_, device_hostname_guess(*device));
                refresh_import_bridges_for_device(*device);
            }
        }
    }

    schedule_peer_reconcile();
}

void ServiceNode::on_gatt_event(const std::string& event_type,
                                  const std::string& object_path,
                                  const std::string& detail) {
    if (event_type == "client_notify_enabled") {
        log_info_coalesced("gatt:notify-enabled:" + object_path,
                           "[client] notify enabled path=" + object_path);
    } else if (event_type == "client_notify_disabled") {
        log_info_coalesced("gatt:notify-disabled:" + object_path,
                           "[client] notify disabled path=" + object_path);
    } else if (is_failed_gatt_event(event_type)) {
        log_warn_coalesced("gatt:" + event_type + ":" + object_path + ":" + detail,
                           "[client] " + event_type + " path=" + object_path + " detail=" + detail);
    } else if (is_read_write_gatt_event(event_type)) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(),
                              std::chrono::duration_cast<std::chrono::milliseconds>(kGattReadWriteLogInterval).count(),
                              "[client] %s path=%s",
                              event_type.c_str(), object_path.c_str());
    } else {
        log_info_coalesced("gatt:" + event_type + ":" + object_path + ":" + detail,
                           "[client] " + event_type + " path=" + object_path + " detail=" + detail);
    }

    if (!cache_ || !client_) {
        return;
    }

    if (event_type == "client_notify_disabled" || event_type == "client_notify_failed") {
        for (const auto& [mac, bridge] : peers_->time_bridges()) {
            if (bridge.characteristic_path == object_path) {
                clear_peer_runtime(mac);
                schedule_peer_reconcile();
                break;
            }
        }
    }

    if (const auto device_path = device_path_for_gatt_object(*cache_, object_path)) {
        if (const auto device = cache_->device(*device_path)) {
            refresh_import_bridges_for_device(*device);
            peers_->sync_device(*device, active_config_, device_hostname_guess(*device));
            schedule_peer_reconcile();
        }
    }
}

void ServiceNode::on_pairing_event(const std::string& event_type, const std::string& device_path) {
    RCLCPP_INFO(get_logger(), "[node] on_pairing_event: type=%s device=%s",
                event_type.c_str(), device_path.c_str());
    if (!peers_ || !cache_) {
        return;
    }
    peers_->note_pairing_event(device_path, event_type, *cache_);
    schedule_peer_reconcile();
}

void ServiceNode::schedule_peer_reconcile(std::chrono::milliseconds delay) {
    if (!peers_ || !client_) {
        return;
    }

    const auto arm_delay = std::max(delay, std::chrono::milliseconds(1));
    const auto requested_deadline = std::chrono::steady_clock::now() + arm_delay;

    if (peer_timer_ && peer_reconcile_deadline_ != std::chrono::steady_clock::time_point{} &&
        requested_deadline >= peer_reconcile_deadline_) {
        return;
    }

    if (peer_timer_) {
        peer_timer_->cancel();
        peer_timer_.reset();
    }

    peer_reconcile_deadline_ = requested_deadline;
    peer_timer_ = create_grouped_wall_timer(*this, arm_delay, [this]() {
        auto timer = peer_timer_;
        peer_timer_.reset();
        peer_reconcile_deadline_ = std::chrono::steady_clock::time_point{};
        if (timer) {
            timer->cancel();
        }
        reconcile_peers();
    }, peer_callback_group_);
}

bool ServiceNode::run_peer_task_once(const std::string& mac,
                                       const std::string& label,
                                       std::function<void()> task) {
    if (shutting_down_.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(peer_task_mutex_);
    auto existing = peer_tasks_.find(mac);
    if (existing != peer_tasks_.end()) {
        if (existing->second.valid() &&
            existing->second.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            return false;
        }
        peer_tasks_.erase(existing);
    }

    auto future = std::async(std::launch::async, [this, mac, label, task = std::move(task)]() mutable {
        try {
            task();
        } catch (const std::exception& exception) {
            if (!shutting_down_.load()) {
                RCLCPP_WARN(get_logger(), "Peer task %s for %s failed: %s",
                            label.c_str(), mac.c_str(), exception.what());
            }
        } catch (...) {
            if (!shutting_down_.load()) {
                RCLCPP_WARN(get_logger(), "Peer task %s for %s failed with unknown error",
                            label.c_str(), mac.c_str());
            }
        }

        if (!shutting_down_.load()) {
            schedule_peer_reconcile(std::chrono::milliseconds(1));
        }
    }).share();
    peer_tasks_[mac] = std::move(future);
    return true;
}

void ServiceNode::wait_for_peer_tasks() {
    std::vector<std::shared_future<void>> tasks;
    {
        std::lock_guard<std::mutex> lock(peer_task_mutex_);
        for (const auto& [_, future] : peer_tasks_) {
            if (future.valid()) {
                tasks.push_back(future);
            }
        }
        peer_tasks_.clear();
    }

    for (auto& future : tasks) {
        future.wait();
    }
}

void ServiceNode::clear_peer_runtime(const std::string& mac) {
    if (!peers_ || !client_ || !import_bridges_) {
        return;
    }

    import_bridges_->clear_import_paths_for_mac(mac, *client_);

    if (auto bridge = peers_->time_bridges().find(mac); bridge != peers_->time_bridges().end()) {
        const auto characteristic_path = bridge->second.characteristic_path;
        peers_->remove_time_bridge(mac);
        if (!characteristic_path.empty()) {
            client_->stop_notify(characteristic_path);
        }
    }
}

void ServiceNode::refresh_import_bridges_for_device(const bluez::DeviceInfo& device) {
    if (!import_bridges_ || !client_ || !peers_) {
        return;
    }

    const auto session_it = peers_->sessions().find(device.mac);
    const bool desired_peer = session_it != peers_->sessions().end() && session_it->second.desired;
    const auto peer_name = device_hostname_guess(device);
    const bool local_peer = !peer_name.empty() && lower_trim(peer_name) == lower_trim(hostname_);
    const auto now_mono = peers_->now_monotonic();
    const auto retry_period_s = std::max(1.0, active_config_.auto_connect_period);
    const auto missing_path_grace_s = std::max(8.0, retry_period_s * 4.0);

    std::set<std::string> desired_keys;
    if (device.connected && desired_peer && !local_peer) {
        for (const auto& shared_topic : active_config_.shared_topics) {
            if (shared_topic.mode != "import" && shared_topic.mode != "both") {
                continue;
            }

            const auto registry_key = std::string("config-import::") + shared_topic.bridge_key + "::" + device.mac;
            desired_keys.insert(registry_key);

            bridge::TopicImportBridgeState state;
            state.mac = device.mac;
            state.requested_topic_name = shared_topic.import_topic_suffix;
            state.resolved_topic_name = peer_bridge_topic(device.mac, peer_name, shared_topic.import_topic_suffix);
            state.message_type = shared_topic.message_type;
            state.bridge_name = bridge_service_name_for_host(peer_name, shared_topic.bridge_topic_path);
            state.bridge_key = shared_topic.bridge_key;
            state.bridge_uuid = util::named_characteristic_uuid(
                bridge_characteristic_name_for_host(peer_name, shared_topic.bridge_topic_path));
            state.member_specs = shared_topic.member_specs;
            state.rate_hz = shared_topic.rate_hz;
            state.payload_format = shared_topic.payload_format;
            state.auto_managed = true;

            if (auto existing = bridge_registry_.imports().find(registry_key); existing != bridge_registry_.imports().end()) {
                state.path = existing->second.path;
                state.poll_timer = existing->second.poll_timer;
                state.pending_payload = existing->second.pending_payload;
                state.last_payload = existing->second.last_payload;
                state.last_publish_monotonic = existing->second.last_publish_monotonic;
                state.current_hz = existing->second.current_hz;
                if (existing->second.resolved_topic_name == state.resolved_topic_name &&
                    existing->second.message_type == state.message_type) {
                    state.publisher = existing->second.publisher;
                    state.runtime = existing->second.runtime;
                }
            }

            auto [it, inserted] = bridge_registry_.imports().insert_or_assign(registry_key, std::move(state));
            (void)inserted;
            import_bridges_->configure_import_bridge(registry_key, it->second, *client_);
        }

        import_bridges_->refresh_import_paths_for_mac(device.mac, *client_);
    }

    prune_missing_import_bridges(device.mac, desired_keys, now_mono, missing_path_grace_s);

    if (desired_keys.empty()) {
        import_bridges_->clear_import_paths_for_mac(device.mac, *client_);
    }
}

void ServiceNode::prune_missing_import_bridges(const std::string& mac,
                                                 const std::set<std::string>& desired_keys,
                                                 double now_mono,
                                                 double missing_path_grace_s) {
    if (!peers_ || !import_bridges_) {
        return;
    }

    auto session_it = peers_->sessions().find(mac);
    if (session_it == peers_->sessions().end()) {
        return;
    }
    auto& session = session_it->second;

    for (auto it = bridge_registry_.imports().begin(); it != bridge_registry_.imports().end();) {
        if (!it->second.auto_managed || it->second.mac != mac) {
            ++it;
            continue;
        }
        if (desired_keys.find(it->first) == desired_keys.end()) {
            session.import_bridge_missing_since.erase(it->first);
            import_bridges_->destroy_import_bridge(it->second);
            it = bridge_registry_.imports().erase(it);
            continue;
        }
        if (!it->second.path.empty()) {
            session.import_bridge_missing_since.erase(it->first);
            ++it;
            continue;
        }

        auto& missing_since = session.import_bridge_missing_since[it->first];
        if (missing_since <= 0.0) {
            missing_since = now_mono;
            ++it;
            continue;
        }
        if ((now_mono - missing_since) < missing_path_grace_s) {
            ++it;
            continue;
        }

        session.import_bridge_missing_since.erase(it->first);
        import_bridges_->destroy_import_bridge(it->second);
        it = bridge_registry_.imports().erase(it);
    }
}

std::string ServiceNode::peer_status_topic(const std::string& mac, const std::string& peer_name) const {
    return util::normalize_ros_topic(active_config_.node_topics_prefix + "/peers/" +
                                     peer_topic_token(peer_name, mac) + "/time_status");
}

std::string ServiceNode::peer_bridge_topic(const std::string& mac,
                                             const std::string& peer_name,
                                             const std::string& requested_topic_suffix) const {
    std::string topic = active_config_.node_topics_prefix + "/peers/" + peer_topic_token(peer_name, mac);
    const auto suffix = trim_topic_segment(requested_topic_suffix);
    if (!suffix.empty()) {
        topic += "/" + suffix;
    }
    return util::normalize_ros_topic(topic);
}

void ServiceNode::publish_peer_time_status(peer::PeerTimeBridge& bridge) const {
    auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<mrs_uav_bluetooth::msg::BlePeerTimeStatus>>(bridge.publisher);
    if (!publisher) {
        return;
    }

    mrs_uav_bluetooth::msg::BlePeerTimeStatus msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = util::sanitize_topic_suffix(hostname_);
    msg.mac = bridge.mac;
    msg.peer_name = bridge.peer_name;
    msg.peer_stamp.sec = static_cast<int32_t>(bridge.last_time_value_ns / 1000000000ULL);
    msg.peer_stamp.nanosec = static_cast<uint32_t>(bridge.last_time_value_ns % 1000000000ULL);
    msg.last_rtt_s = bridge.last_rtt_s;
    publisher->publish(msg);
}

void ServiceNode::on_notification(const std::vector<uint8_t>& data,
                                    const std::string& uuid,
                                    const std::string& characteristic_path) {
    RCLCPP_DEBUG(get_logger(), "[node] on_notification: uuid=%s path=%s %zu bytes",
                uuid.c_str(), characteristic_path.c_str(), data.size());
    if (!cache_ || !ros_) {
        return;
    }

    std::string mac;
    if (const auto characteristic = cache_->characteristic(characteristic_path)) {
        if (const auto service = cache_->service(characteristic->service_path)) {
            if (const auto device = cache_->device(service->device_path)) {
                mac = device->mac;
            }
        }
    }

    ros_->status_publisher().publish_notification(mac, characteristic_path, uuid, data, hostname_);

    if (import_bridges_) {
        import_bridges_->buffer_notification_payload(mac, characteristic_path, data);
    }

    if (!peers_) {
        return;
    }

    auto bridge_it = peers_->time_bridges().find(mac);
    if (bridge_it == peers_->time_bridges().end() ||
        bridge_it->second.characteristic_path != characteristic_path ||
        data.size() < sizeof(uint64_t)) {
        return;
    }

    uint64_t peer_time_ns = 0;
    std::memcpy(&peer_time_ns, data.data(), sizeof(peer_time_ns));
    auto& bridge = bridge_it->second;
    bridge.last_activity_monotonic = peers_->now_monotonic();
    bridge.last_time_value_ns = peer_time_ns;
    std::tie(bridge.last_publish_monotonic, bridge.current_hz) =
        update_publish_rate(bridge.last_publish_monotonic);
    bridge.status = "ready";
    bridge.detail = "time notification";
    publish_peer_time_status(bridge);

    if (!bridge.writeback_descriptor_path.empty()) {
        client_->write_descriptor_async(bridge.writeback_descriptor_path, data);
    }
}

void ServiceNode::handle_time_writeback(const std::vector<uint8_t>& payload,
                                          const std::string& device_path,
                                          uint64_t received_time_ns) {
    if (!peers_ || payload.size() < sizeof(uint64_t)) {
        return;
    }

    std::string mac;
    if (!device_path.empty() && cache_) {
        if (const auto device = cache_->device(device_path)) {
            mac = device->mac;
        }
    }

    if (mac.empty()) {
        for (const auto& [candidate_mac, bridge] : peers_->time_bridges()) {
            if (!device_path.empty() && !bridge.characteristic_path.empty() &&
                bridge.characteristic_path.find(device_path + "/") == 0) {
                mac = candidate_mac;
                break;
            }
        }
    }

    if (mac.empty()) {
        return;
    }

    auto bridge_it = peers_->time_bridges().find(mac);
    if (bridge_it == peers_->time_bridges().end()) {
        return;
    }

    uint64_t echoed_time_ns = 0;
    std::memcpy(&echoed_time_ns, payload.data(), sizeof(echoed_time_ns));
    if (echoed_time_ns == 0 || received_time_ns < echoed_time_ns) {
        return;
    }

    auto& bridge = bridge_it->second;
    bridge.last_activity_monotonic = peers_->now_monotonic();
    bridge.last_rtt_s = std::max(0.0, static_cast<double>(received_time_ns - echoed_time_ns) / 1e9);
    bridge.status = "ready";
    bridge.detail = "time writeback";
    publish_peer_time_status(bridge);
}

bool ServiceNode::update_peer_time_bridge(const std::string& mac,
                                            const bluez::DeviceInfo& device,
                                            peer::PeerConnectionSession& session) {
    const auto time_characteristic_uuid = util::named_characteristic_uuid("time/ns");
    const auto writeback_descriptor_uuid = util::named_descriptor_uuid("time/ns/writeback");
    const auto peer_name = session.peer_name.empty() ? device_hostname_guess(device) : session.peer_name;
    const auto characteristic_path = client_->find_characteristic(mac, time_characteristic_uuid);
    const auto cached_characteristic = (!characteristic_path.empty() && cache_)
        ? cache_->characteristic(characteristic_path)
        : std::optional<bluez::GattCharacteristicInfo>{};
    auto bridge_it = peers_->time_bridges().find(mac);

    const auto clear_time_bridge = [this, &mac, &bridge_it]() {
        if (bridge_it == peers_->time_bridges().end()) {
            return;
        }
        const auto old_characteristic_path = bridge_it->second.characteristic_path;
        peers_->remove_time_bridge(mac);
        bridge_it = peers_->time_bridges().end();
        if (!old_characteristic_path.empty()) {
            client_->stop_notify(old_characteristic_path);
        }
    };

    if (!characteristic_path.empty() &&
        bridge_it != peers_->time_bridges().end() &&
        bridge_it->second.characteristic_path == characteristic_path &&
        cached_characteristic && cached_characteristic->notifying) {
        auto& bridge = bridge_it->second;
        bridge.mac = mac;
        bridge.peer_name = peer_name;
        bridge.status = "ready";
        bridge.detail = "peer time bridge active";
        session.bridge_wait_started_monotonic = 0.0;
        session.bridge_wait_reason.clear();
        session.phase = "ready";
        session.detail = "peer time bridge active";
        return true;
    }

    // Log available GATT objects only while resolving or repairing the bridge.
    const auto services = client_->list_services(mac);
    const auto characteristics = client_->list_characteristics(mac);
    const auto descriptors = client_->list_descriptors(mac);
    RCLCPP_DEBUG(get_logger(), "[node] update_peer_time_bridge(%s): %zu services, %zu characteristics, %zu descriptors resolved",
                mac.c_str(), services.size(), characteristics.size(), descriptors.size());
    if (characteristic_path.empty()) {
        clear_time_bridge();
        log_info_coalesced("peer-time-bridge-missing:" + mac + ":" + std::to_string(characteristics.size()),
                           "[node] update_peer_time_bridge(" + mac + "): time characteristic uuid=" +
                               time_characteristic_uuid + " not found among " +
                               std::to_string(characteristics.size()) + " characteristics");
        if (session.bridge_wait_started_monotonic <= 0.0) {
            session.bridge_wait_started_monotonic = peers_->now_monotonic();
        }
        session.bridge_wait_reason = "characteristic";
        session.phase = "connected_unready";
        session.detail = "peer time characteristic not available";
        return false;
    }

    if (bridge_it != peers_->time_bridges().end() &&
        !bridge_it->second.characteristic_path.empty() &&
        bridge_it->second.characteristic_path != characteristic_path) {
        clear_time_bridge();
    }

    auto [created_bridge_it, inserted_bridge] = peers_->time_bridges().try_emplace(mac);
    (void)inserted_bridge;
    bridge_it = created_bridge_it;
    auto& bridge = bridge_it->second;

    const auto topic_name = peer_status_topic(mac, peer_name);
    if (!bridge.publisher || bridge.status_topic_name != topic_name) {
        if (bridge.publisher) {
            bridge.publisher.reset();
        }
        bridge.publisher = create_publisher<mrs_uav_bluetooth::msg::BlePeerTimeStatus>(topic_name, 10);
        bridge.status_topic_name = topic_name;
    }

    bridge.mac = mac;
    bridge.peer_name = peer_name;
    bridge.characteristic_path = characteristic_path;

    if (bridge.status == "subscribing") {
        if (session.bridge_wait_started_monotonic <= 0.0) {
            session.bridge_wait_started_monotonic = peers_->now_monotonic();
        }
        session.bridge_wait_reason = "notify";
        session.phase = "connected_unready";
        session.detail = "awaiting peer time notifications";
        return false;
    }

    if (bridge.status == "notify_failed") {
        if (session.bridge_wait_started_monotonic <= 0.0) {
            session.bridge_wait_started_monotonic = peers_->now_monotonic();
        }
        session.bridge_wait_reason = "notify";
        session.phase = "connected_unready";
        session.detail = "failed to enable peer time notifications";
        return false;
    }

    // Descriptors are optional — many devices (phones, etc.) don't resolve them.
    // The writeback descriptor is a nice-to-have for RTT measurement.
    const auto wb_path = client_->find_descriptor(mac, writeback_descriptor_uuid, characteristic_path);
    if (wb_path.empty() && !descriptors.empty()) {
        RCLCPP_DEBUG(get_logger(), "[node] update_peer_time_bridge(%s): writeback descriptor uuid=%s not found (descriptors resolved=%zu)",
                     mac.c_str(), writeback_descriptor_uuid.c_str(), descriptors.size());
    } else if (wb_path.empty()) {
        RCLCPP_DEBUG(get_logger(), "[node] update_peer_time_bridge(%s): no descriptors resolved for this device, writeback disabled",
                     mac.c_str());
    }
    bridge.writeback_descriptor_path = wb_path;
    bridge.status = "subscribing";
    bridge.detail = "enabling peer time notifications";
    bridge.services_wait_grace_s = session.services_wait_grace_s;
    bridge.pairing_failures = session.pairing_failures;

    if (client_->start_notify(characteristic_path)) {
        log_info_coalesced("peer-time-bridge-notify-start:" + mac + ":" + characteristic_path,
                           "[node] update_peer_time_bridge(" + mac + "): notifications requested on " +
                               characteristic_path + " (writeback=" +
                               (wb_path.empty() ? std::string{"none"} : wb_path) + ")");
        if (session.bridge_wait_started_monotonic <= 0.0) {
            session.bridge_wait_started_monotonic = peers_->now_monotonic();
        }
        session.bridge_wait_reason = "notify";
        session.connect_repair_count = 0;
        session.phase = "connected_unready";
        session.detail = "awaiting peer time notifications";
        return false;
    }

    RCLCPP_WARN(get_logger(), "[node] update_peer_time_bridge(%s): failed to enable notifications on %s",
                mac.c_str(), characteristic_path.c_str());
    bridge.status = "notify_failed";
    bridge.detail = "failed to enable peer time notifications";
    if (session.bridge_wait_started_monotonic <= 0.0) {
        session.bridge_wait_started_monotonic = peers_->now_monotonic();
    }
    session.bridge_wait_reason = "notify";
    session.phase = "connected_unready";
    session.detail = "failed to enable peer time notifications";
    return false;
}

void ServiceNode::reconcile_peers() {
    if (!peers_ || !client_) {
        return;
    }

    const auto now = peers_->now_monotonic();
    const auto retry_period_s = std::max(0.5, active_config_.auto_connect_period);
    const bool whitelist_enabled = !active_config_.auto_connect_whitelist.empty();
    std::set<std::string> current_macs;
    for (const auto& device : client_->get_devices()) {
        current_macs.insert(device.mac);
    }

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 10000,
                          "[reconcile] sessions=%zu devices=%zu auto_connect=%s whitelist=%zu",
                          peers_->sessions().size(), current_macs.size(),
                          active_config_.auto_connect_enable ? "on" : "off",
                          active_config_.auto_connect_whitelist.size());

    peers_->prune_sessions(current_macs, now, std::max(5.0, active_config_.peer_connection_timeout));

    bool pending_deadline = false;
    double next_deadline_s = retry_period_s;
    const double repair_cooldown_s = std::max(kPeerRepairCooldownMin, retry_period_s * 2.0);
    const double bridge_grace_s = std::max(kBridgeGraceMin, retry_period_s * 4.0);

    for (auto& [mac, session] : peers_->sessions()) {
        const auto device = client_->get_device(mac);
        const auto device_label = mac + " (" + session.peer_name + ")";
        const bool is_connected = device && device->connected;

        // Only clear runtime state for desired peers that disconnected.
        // Don't clear runtime for non-desired devices (e.g. phones) — they were never managed.
        if (session.desired && !is_connected) {
            clear_peer_runtime(mac);
        } else if (is_connected) {
            refresh_import_bridges_for_device(*device);
        }

        // Policy enforcement: peers not allowed by the current config must not
        // remain connected or bonded, even if the connection was initiated remotely.
        const bool policy_violation = !session.desired && session.peer_candidate &&
            device && (device->connected || device->paired || device->bonded || device->trusted);
        if (policy_violation &&
            (session.last_policy_action_monotonic <= 0.0 || now - session.last_policy_action_monotonic >= retry_period_s)) {
            RCLCPP_INFO(get_logger(),
                        "[reconcile] %s: peer blocked by current config, disconnecting%s",
                        device_label.c_str(),
                        (device->paired || device->bonded || device->trusted) ? " and removing bond" : "");
            if (run_peer_task_once(mac, "policy cleanup", [this, mac, retry_period_s, connected = device->connected,
                                                            paired = device->paired, bonded = device->bonded,
                                                            trusted = device->trusted]() {
                    if (connected) {
                        (void)client_->disconnect(mac, retry_period_s);
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    }
                    if (paired || bonded || trusted) {
                        (void)client_->remove(mac);
                    }
                })) {
                session.phase = "policy_blocked";
                session.detail = whitelist_enabled ? "peer not present in whitelist"
                                                  : "peer not allowed by current config";
                session.last_policy_action_monotonic = now;
                pending_deadline = true;
                next_deadline_s = std::min(next_deadline_s, retry_period_s);
            }
            continue;
        }

        // Skip non-desired sessions entirely — they are not managed by us.
        if (!session.desired) {
            continue;
        }

        if (device && peers_->should_attempt_trust(session, *device, now, retry_period_s)) {
            RCLCPP_INFO(get_logger(), "[reconcile] %s: attempting trust (paired=%s bonded=%s trusted=%s)",
                        device_label.c_str(),
                        device->paired ? "Y" : "N",
                        device->bonded ? "Y" : "N",
                        device->trusted ? "Y" : "N");
            if (run_peer_task_once(mac, "trust", [this, mac]() {
                    (void)client_->trust(mac);
                })) {
                session.phase = "securing";
                session.detail = "trust repair requested";
                session.last_security_attempt_monotonic = now;
                pending_deadline = true;
                next_deadline_s = std::min(next_deadline_s, retry_period_s);
            }
            continue;
        }

        if (peers_->should_attempt_connect(session, now, retry_period_s)) {
            RCLCPP_INFO(get_logger(), "[reconcile] %s: attempting connect (phase=%s)",
                        device_label.c_str(), session.phase.c_str());
            if (run_peer_task_once(mac, "connect", [this, mac, retry_period_s]() {
                    (void)client_->connect(mac, retry_period_s);
                })) {
                session.phase = "connecting";
                session.detail = "auto-connect requested";
                session.last_connect_attempt_monotonic = now;
                session.connect_started_monotonic = now;
                pending_deadline = true;
                next_deadline_s = std::min(next_deadline_s, retry_period_s);
            }
            continue;
        }

        if (peers_->should_attempt_pair(session, now, retry_period_s)) {
            RCLCPP_INFO(get_logger(), "[reconcile] %s: attempting pair (phase=%s)",
                        device_label.c_str(), session.phase.c_str());
            if (run_peer_task_once(mac, "pair", [this, mac, retry_period_s]() {
                    (void)client_->pair(mac, retry_period_s);
                })) {
                session.phase = "securing";
                session.detail = "auto-pair requested";
                session.last_security_attempt_monotonic = now;
                pending_deadline = true;
                next_deadline_s = std::min(next_deadline_s, retry_period_s);
            }
            continue;
        }

        if (is_connected && device->trusted && (device->paired || device->bonded) && !device->services_resolved) {
            if ((session.last_service_retry_monotonic <= 0.0 ||
                 now - session.last_service_retry_monotonic >= retry_period_s) &&
                run_peer_task_once(mac, "wait services", [this, mac, retry_period_s]() {
                    (void)client_->wait_services_resolved(mac, std::max(8.0, retry_period_s * 4.0));
                })) {
                session.last_service_retry_monotonic = now;
                pending_deadline = true;
                next_deadline_s = std::min(next_deadline_s, retry_period_s);
            }
        }

        const bool services_stuck = is_connected && device->trusted && (device->paired || device->bonded) &&
            !device->services_resolved && session.services_wait_started_monotonic > 0.0 &&
            (now - session.services_wait_started_monotonic) >= session.services_wait_grace_s;
        const bool bridge_stuck = is_connected && device->trusted && (device->paired || device->bonded) &&
            device->services_resolved && session.bridge_wait_started_monotonic > 0.0 &&
            (now - session.bridge_wait_started_monotonic) >= bridge_grace_s;
        const bool repair_due = services_stuck || bridge_stuck;
        if (repair_due &&
            (session.last_repair_monotonic <= 0.0 || now - session.last_repair_monotonic >= repair_cooldown_s) &&
            run_peer_task_once(mac, "repair", [this, mac]() {
                (void)client_->disconnect(mac, 5.0);
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                (void)client_->connect(mac, 10.0);
                (void)client_->wait_services_resolved(mac, 10.0);
            })) {
            session.phase = "recovering";
            session.detail = services_stuck ? "repairing unresolved services" : "repairing missing bridge";
            session.last_repair_monotonic = now;
            session.connect_repair_count += 1;
            session.last_connect_attempt_monotonic = now;
            session.connect_started_monotonic = now;
            session.last_service_retry_monotonic = now;
            session.bridge_wait_started_monotonic = 0.0;
            session.bridge_wait_reason.clear();
            pending_deadline = true;
            next_deadline_s = std::min(next_deadline_s, retry_period_s);
            continue;
        }

        if (is_connected && device->trusted && (device->paired || device->bonded) && device->services_resolved) {
            if (!update_peer_time_bridge(mac, *device, session)) {
                pending_deadline = true;
                next_deadline_s = std::min(next_deadline_s, retry_period_s);
            }
            continue;
        }

        // Desired but not yet fully ready — reschedule
        RCLCPP_DEBUG(get_logger(), "[reconcile] %s: desired but pending (phase=%s conn=%s paired=%s trusted=%s svc_resolved=%s)",
                     device_label.c_str(), session.phase.c_str(),
                     is_connected ? "Y" : "N",
                     device ? (device->paired ? "Y" : "N") : "?",
                     device ? (device->trusted ? "Y" : "N") : "?",
                     device ? (device->services_resolved ? "Y" : "N") : "?");
        pending_deadline = true;
        if (session.services_wait_started_monotonic > 0.0 && session.services_wait_grace_s > 0.0) {
            const auto remaining = std::max(0.1,
                session.services_wait_started_monotonic + session.services_wait_grace_s - now);
            next_deadline_s = std::min(next_deadline_s, remaining);
        } else {
            next_deadline_s = std::min(next_deadline_s, retry_period_s);
        }
    }

    if (pending_deadline) {
        schedule_peer_reconcile(std::chrono::milliseconds(
            static_cast<int64_t>(std::max(0.1, next_deadline_s) * 1000.0)));
    }
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

void ServiceNode::handle_configure_notification_bridge(const std::shared_ptr<mrs_uav_bluetooth::srv::ConfigureNotificationBridge::Request> request,
                                                         std::shared_ptr<mrs_uav_bluetooth::srv::ConfigureNotificationBridge::Response> response) {
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
