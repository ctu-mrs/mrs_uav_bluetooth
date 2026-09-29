// SPDX-License-Identifier: BSD-3-Clause
/// \file src/app/service_node.cpp
/// \brief Implements the service node component of the ROS 2 application and operator-tool layer.

#include "mrs_uav_bluetooth/app/service_node.hpp"

#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"
#include "mrs_uav_bluetooth/config/shared_topic_config.hpp"

#include "mrs_uav_bluetooth/gatt/bridge_naming.hpp"
#include "mrs_uav_bluetooth/util/device_utils.hpp"

#include "mrs_uav_bluetooth/util/string_utils.hpp"

#include "mrs_uav_bluetooth/util/topic_utils.hpp"
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <ctime>
#include <future>
#include <filesystem>
#include "mrs_uav_bluetooth/config/config_loader.hpp"
#include <limits>
#include <rclcpp/create_timer.hpp>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <unistd.h>

namespace {

constexpr double kLocalReconfigureGraceMin = 5.0;
constexpr size_t kPrimaryAdvertisementMaxBytes = 31;
constexpr size_t kExtendedAdvertisementMaxBytes = 251;
constexpr size_t kAdvFlagsBytes = 3;
constexpr auto kAdvertisementScanResumeTimeout = std::chrono::milliseconds(100);
constexpr auto kAdvertisementScanPollPeriod = std::chrono::milliseconds(5);
constexpr uint64_t kAdvertisementRadioWindowMinMs = 55;
constexpr uint64_t kAdvertisementRadioWindowMaxMs = 250;
constexpr uint64_t kAdvertisementRadioWindowMarginMs = 20;
constexpr uint64_t kAdvertisementScanDwellVariationMs = 301;
constexpr uint64_t kLegacyControllerActivationBudgetMs = 2000;
constexpr uint64_t kLegacyControllerOnAirMinMs = 1000;
constexpr uint64_t kLegacyControllerScanWindowMinMs = 1200;

/// \brief Choose a bounded interval in which one half-duplex radio role stays active.
/// \param config Effective settings containing the requested advertising interval.
/// \return Radio window duration in milliseconds.
uint64_t advertisement_radio_window_ms(
    const mrs_uav_bluetooth::config::NodeConfig& config) {
    // One complete requested interval lets a peer observe at least one packet.
    const auto requested_interval_ms = static_cast<uint64_t>(
        config.advertise_max_interval.value_or(100U));
    return std::clamp(
        requested_interval_ms + kAdvertisementRadioWindowMarginMs,
        kAdvertisementRadioWindowMinMs,
        kAdvertisementRadioWindowMaxMs);
}

/// \brief Reserve enough transmit time for the controller to start the new packet.
/// \param config Effective settings containing the requested advertising interval.
/// \param adapter_info Controller limits used to identify software-scheduled legacy advertising.
/// \return Transmit-only window duration in milliseconds.
uint64_t advertisement_transmit_window_ms(
    const mrs_uav_bluetooth::config::NodeConfig& config,
    const std::optional<mrs_uav_bluetooth::bluez::AdapterInfo>& adapter_info) {
    const auto ordinary_window_ms = advertisement_radio_window_ms(config);
    if (!adapter_info.has_value() ||
        adapter_info->max_advertisement_length > kPrimaryAdvertisementMaxBytes) {
        return ordinary_window_ms;
    }

    // Linux may defer a legacy advertising instance by its two-second
    // software rotation period after discovery stops. Keep the role long
    // enough for that activation and several packets on all three channels.
    return kLegacyControllerActivationBudgetMs +
        std::max(ordinary_window_ms, kLegacyControllerOnAirMinMs);
}

/// \brief Reserve a useful receive interval on a half-duplex legacy controller.
/// \param config Effective settings containing the requested advertising interval.
/// \param adapter_info Controller limits used to identify legacy radio arbitration.
/// \return Base scan-window duration before per-cycle phase variation.
uint64_t advertisement_scan_window_ms(
    const mrs_uav_bluetooth::config::NodeConfig& config,
    const std::optional<mrs_uav_bluetooth::bluez::AdapterInfo>& adapter_info) {
    const auto ordinary_window_ms = advertisement_radio_window_ms(config);
    if (!adapter_info.has_value() ||
        adapter_info->max_advertisement_length > kPrimaryAdvertisementMaxBytes) {
        return ordinary_window_ms;
    }

    // A longer receive role prevents frequent transmit cycles from starving discovery.
    return std::max(ordinary_window_ms, kLegacyControllerScanWindowMinMs);
}

/// \brief Extract application bytes from the configured raw advertisement field.
/// \param device Cached BlueZ peer whose advertising fields are inspected.
/// \param data_type Raw advertisement field type allocated to bridge data.
/// \return The cached application bytes, or an empty vector when absent.
std::vector<uint8_t> advertisement_user_payload(
    const mrs_uav_bluetooth::bluez::DeviceInfo& device,
    uint8_t data_type) {
    const auto found = device.advertising_data.find(data_type);
    if (found == device.advertising_data.end()) return {};
    return found->second;
}

/// Marks a potentially long reconfiguration section without relying on every
/// early return and exception path to clear the state manually.
class AtomicFlagGuard {
public:
    /// \brief Mark the guarded transition active for this scope.
    /// \param flag atomic transition flag set for the guard lifetime.
    explicit AtomicFlagGuard(std::atomic_bool& flag) : flag_(flag) {
        // Mark the guarded transition active for this scope.
        flag_.store(true);
    }

    /// \brief Clear the transition flag on every exit path, including exceptions.
    ~AtomicFlagGuard() {
        // Clear the transition flag on every exit path, including exceptions.
        flag_.store(false);
    }

    /// \brief Disable copying of the atomic flag guard.
    AtomicFlagGuard(const AtomicFlagGuard&) = delete;
    /// \brief Assign state from another atomic flag guard.
    AtomicFlagGuard& operator=(const AtomicFlagGuard&) = delete;

private:
    std::atomic_bool& flag_;
};

/// Prevent new outbound links during a radio handoff and restore the selected
/// connection policy even if a BlueZ operation throws before reconfiguration ends.
class ConnectionPolicyGuard {
public:
    /// \brief Block new peer connections while an overlay transition owns the adapter.
    /// \param client BlueZ client used for remote discovery and GATT operations.
    /// \param allow_after connection-policy value restored when the guard leaves scope.
    ConnectionPolicyGuard(mrs_uav_bluetooth::bluez::BluezClient* client, bool allow_after)
        : client_(client), allow_after_(allow_after) {
        // Block new peer connections while an overlay transition owns the adapter.
        if (client_) client_->set_connections_allowed(false);
    }
    /// \brief Restore the caller-selected connection policy after transition cleanup.
    ~ConnectionPolicyGuard() {
        // Restore the caller-selected connection policy after transition cleanup.
        if (client_) client_->set_connections_allowed(allow_after_);
    }
    /// \brief Disable copying of the connection policy guard.
    ConnectionPolicyGuard(const ConnectionPolicyGuard&) = delete;
    /// \brief Assign state from another connection policy guard.
    ConnectionPolicyGuard& operator=(const ConnectionPolicyGuard&) = delete;
private:
    mrs_uav_bluetooth::bluez::BluezClient* client_;
    bool allow_after_;
};

/// \brief Match one string against an initializer list of accepted values.
/// \param values Strings searched for an exact match.
/// \param value Exact string to search for.
/// \return True when one accepted string matches; otherwise false.
bool contains_string(const std::vector<std::string>& values, const std::string& value) {
    // Match one string against an initializer list of accepted values.
    return std::find(values.begin(), values.end(), value) != values.end();
}

/// \brief Include the field header when calculating one encoded advertising structure.
/// \param payload_bytes application bytes available after transport framing.
/// \return Encoded bytes including length and type octets or zero for no payload.
size_t advertising_structure_size(size_t payload_bytes) {
    // Include the field header when calculating one encoded advertising structure.
    return payload_bytes == 0 ? 0 : 2 + payload_bytes;
}

/// \brief Map accepted bridge direction aliases to import export or both.
/// \param raw_direction unvalidated bridge direction text from YAML.
/// \return Normalized direction.
std::string normalize_direction(const std::string& raw_direction) {
    // Collapse accepted RX/TX aliases to the three bridge modes stored in runtime state.
    auto direction = mrs_uav_bluetooth::util::lower_trim_copy(raw_direction);
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

/// \brief Parse service-request field selectors and default omitted scalar types to float64.
/// \param member_paths ROS field paths selected for bridge encoding.
/// \return Validated member specs.
std::vector<mrs_uav_bluetooth::config::BridgeMemberSpec> parse_member_specs(
    const std::vector<std::string>& member_paths) {
    // Split optional `path:type` entries and default omitted wire types to float64.
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
            value_type = mrs_uav_bluetooth::util::lower_trim_copy(raw_member.substr(separator + 1));
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

/// \brief Build the stable registry key for a manually requested notification bridge.
/// \param direction normalized bridge direction controlling publisher/subscriber creation.
/// \param mac peer Bluetooth MAC address.
/// \param topic_name Resolved ROS topic included in the bridge identity hash.
/// \param message_type Fully qualified ROS message type included in the bridge identity hash.
/// \param characteristic Characteristic UUID or name included in the bridge identity hash.
/// \param member_specs Ordered ROS fields included in the bridge identity hash.
/// \return Dash-free deterministic UUID identifying the requested bridge definition.
std::string manual_bridge_key(const std::string& direction,
                              const std::string& mac,
                              const std::string& topic_name,
                              const std::string& message_type,
                              const std::string& characteristic,
                              const std::vector<mrs_uav_bluetooth::config::BridgeMemberSpec>& member_specs) {
    // Build the stable registry key for a manually requested notification bridge.
    std::ostringstream source;
    source << direction << '|' << mac << '|' << topic_name << '|' << message_type << '|'
           << characteristic;
    for (const auto& spec : member_specs) {
        source << '|' << spec.target << ':' << spec.value_type << ':' << spec.expression;
    }
    std::string uuid = mrs_uav_bluetooth::util::uuid_from_name(source.str());
    uuid.erase(std::remove(uuid.begin(), uuid.end(), '-'), uuid.end());
    return uuid;
}

template<typename DurationT, typename CallbackT>
/// \brief Bind the timer to the callback group that serializes overlay work.
/// \param node ROS node that owns the interfaces.
/// \param period timer interval used to schedule the callback.
/// \param callback Timer body invoked on each scheduled expiry.
/// \param group ROS callback group that serializes the timer or service.
/// \return New grouped wall timer.
rclcpp::TimerBase::SharedPtr create_grouped_wall_timer(
    rclcpp::Node& node,
    DurationT period,
    CallbackT&& callback,
    const rclcpp::CallbackGroup::SharedPtr& group) {
    // Bind the timer to the callback group that serializes overlay work.
    return rclcpp::create_wall_timer(
        period,
        std::forward<CallbackT>(callback),
        group,
        node.get_node_base_interface().get(),
        node.get_node_timers_interface().get());
}

/// \brief Return the compressed advertising width supported for this UUID.
/// \param uuid Service UUID whose encoded advertisement width is calculated.
/// \return Compressed Bluetooth advertisement width of the UUID: 2 4 or 16 bytes.
size_t advertising_uuid_size(const std::string& uuid) {
    // Return the compressed advertising width supported for this UUID.
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

/// \brief Calculate encoded bytes for UUID groups after Bluetooth width compression.
/// \param uuids Service UUIDs whose encoded advertisement size is calculated.
/// \return Encoded bytes for grouped 16-bit 32-bit and 128-bit UUID lists.
size_t advertising_uuid_list_size(const std::vector<std::string>& uuids) {
    // Calculate encoded bytes for UUID groups after Bluetooth width compression.
    size_t total = 0;
    size_t short_uuid_bytes = 0;
    size_t medium_uuid_bytes = 0;
    size_t long_uuid_bytes = 0;
    for (const auto& uuid : uuids) {
        switch (advertising_uuid_size(uuid)) {
        case 2:
            short_uuid_bytes += 2;
            break;
        case 4:
            medium_uuid_bytes += 4;
            break;
        default:
            long_uuid_bytes += 16;
            break;
        }
    }

    total += advertising_structure_size(short_uuid_bytes);
    total += advertising_structure_size(medium_uuid_bytes);
    total += advertising_structure_size(long_uuid_bytes);
    return total;
}

/// \brief Calculate encoded bytes for every service-data entry.
/// \param service_data service-keyed byte payload included in advertising data.
/// \return Encoded bytes occupied by every service-data structure.
size_t advertising_service_data_size(const std::map<std::string, std::vector<uint8_t>>& service_data) {
    // Calculate encoded bytes for every service-data entry.
    size_t total = 0;
    for (const auto& [uuid, payload] : service_data) {
        total += advertising_structure_size(advertising_uuid_size(uuid) + payload.size());
    }
    return total;
}

/// \brief Calculate encoded bytes for every company-data entry.
/// \param manufacturer_data company-keyed bytes included in advertising data.
/// \return Encoded bytes occupied by every manufacturer-data structure.
size_t advertising_manufacturer_data_size(
    const std::map<uint16_t, std::vector<uint8_t>>& manufacturer_data) {
    // Calculate encoded bytes for every company-data entry.
    size_t total = 0;
    for (const auto& [company_id, payload] : manufacturer_data) {
        static_cast<void>(company_id);
        total += advertising_structure_size(sizeof(uint16_t) + payload.size());
    }
    return total;
}

/// \brief Calculate encoded bytes for all raw advertising field entries.
/// \param advertising_data Raw advertisement fields whose encoded byte count is calculated.
/// \return Encoded bytes occupied by every raw advertisement structure.
size_t advertising_generic_data_size(const std::map<uint8_t, std::vector<uint8_t>>& advertising_data) {
    // Calculate encoded bytes for all raw advertising field entries.
    size_t total = 0;
    for (const auto& [type, payload] : advertising_data) {
        static_cast<void>(type);
        total += advertising_structure_size(payload.size());
    }
    return total;
}

/// \brief Calculate all encoded structures competing for primary advertisement space.
/// \param local_name device name proposed for inclusion in the advertising packet.
/// \param cfg Advertisement fields whose encoded primary-packet size is estimated.
/// \param advertising_data Raw fields placed in the primary advertisement size estimate.
/// \param name_in_primary True places the local name in the primary packet; false uses the scan response.
/// \return Total encoded primary-packet bytes before application service data.
size_t estimate_primary_advertisement_bytes(
    const std::string& local_name,
    const mrs_uav_bluetooth::config::NodeConfig& cfg,
    const std::map<uint8_t, std::vector<uint8_t>>& advertising_data,
    bool name_in_primary) {
    size_t total = kAdvFlagsBytes;

    // BlueZ puts LocalName in the scan response for legacy advertisements.
    // A connectable extended advertisement carries the name in primary data.
    if (name_in_primary && !local_name.empty()) {
        total += advertising_structure_size(local_name.size());
    }
    total += advertising_uuid_list_size(cfg.advertise_solicit_uuids);
    total += advertising_manufacturer_data_size(cfg.advertise_manufacturer_data);
    total += advertising_service_data_size(cfg.advertise_service_data);
    total += advertising_generic_data_size(advertising_data);

    const bool include_tx_power = std::find(cfg.advertise_includes.begin(),
                                            cfg.advertise_includes.end(),
                                            "tx-power") != cfg.advertise_includes.end();
    if (include_tx_power) {
        total += advertising_structure_size(1);
    }

    const bool include_appearance = cfg.advertise_appearance.has_value() ||
        std::find(cfg.advertise_includes.begin(),
                  cfg.advertise_includes.end(),
                  "appearance") != cfg.advertise_includes.end();
    if (include_appearance) {
        total += advertising_structure_size(sizeof(uint16_t));
    }

    return total;
}

/// \brief Calculate fixed advertisement overhead that reduces application payload capacity.
/// \param cfg Advertisement placement settings that reserve space around user data.
/// \param advertising_data Raw fields checked for space competition with application data.
/// \param reserve_primary_flags whether the primary packet must retain space for mandatory flags.
/// \return Encoded fixed overhead that competes with application bytes.
size_t estimate_user_data_competing_advertisement_bytes(
    const mrs_uav_bluetooth::config::NodeConfig& cfg,
    const std::map<uint8_t, std::vector<uint8_t>>& advertising_data,
    bool reserve_primary_flags) {
    // Estimate user data competing advertisement bytes.
    size_t total = 0;
    if (reserve_primary_flags) {
        total += kAdvFlagsBytes;
    }
    total += advertising_uuid_list_size(cfg.advertise_solicit_uuids);
    total += advertising_manufacturer_data_size(cfg.advertise_manufacturer_data);
    total += advertising_service_data_size(cfg.advertise_service_data);
    total += advertising_generic_data_size(advertising_data);

    const bool include_tx_power = std::find(cfg.advertise_includes.begin(),
                                            cfg.advertise_includes.end(),
                                            "tx-power") != cfg.advertise_includes.end();
    if (include_tx_power) {
        total += advertising_structure_size(1);
    }

    const bool include_appearance = cfg.advertise_appearance.has_value() ||
        std::find(cfg.advertise_includes.begin(),
                  cfg.advertise_includes.end(),
                  "appearance") != cfg.advertise_includes.end();
    if (include_appearance) {
        total += advertising_structure_size(sizeof(uint16_t));
    }

    return total;
}

/// \brief Enable extended advertising only when the controller reports usable secondary channels.
/// \param cfg Requested extended-advertising channel policy.
/// \param adapter_info controller limits used to validate advertising payload size and features.
/// \return Usable extended secondary channel or empty for legacy advertising.
std::string select_secondary_channel(
    const mrs_uav_bluetooth::config::NodeConfig& cfg,
    const std::optional<mrs_uav_bluetooth::bluez::AdapterInfo>& adapter_info) {
    // Use legacy advertising unless extended mode and a usable controller channel agree.
    if (cfg.advertise_size != "extended") {
        return {};
    }

    if (!adapter_info.has_value()) {
        return "1M";
    }

    const auto& supported = adapter_info->supported_advertising_secondary_channels;
    if (supported.empty() || adapter_info->max_advertisement_length <= kPrimaryAdvertisementMaxBytes) {
        return {};
    }

    if (contains_string(supported, "1M")) {
        return "1M";
    }
    return supported.empty() ? std::string{} : supported.front();
}

/// \brief Choose the legacy or controller-reported extended advertisement byte limit.
/// \param cfg Configured payload limit and legacy fallback policy.
/// \param secondary_channel extended-advertising secondary radio mode requested from BlueZ.
/// \param adapter_info controller limits used to validate advertising payload size and features.
/// \return Resolved advertisement max bytes.
size_t resolve_advertisement_max_bytes(
    const mrs_uav_bluetooth::config::NodeConfig& cfg,
    const std::string& secondary_channel,
    const std::optional<mrs_uav_bluetooth::bluez::AdapterInfo>& adapter_info) {
    // Choose the 31-byte legacy limit or the controller’s extended capacity.
    if (cfg.advertise_size != "extended" || secondary_channel.empty()) {
        return kPrimaryAdvertisementMaxBytes;
    }

    if (adapter_info.has_value() && adapter_info->max_advertisement_length > 0) {
        return adapter_info->max_advertisement_length;
    }
    return kExtendedAdvertisementMaxBytes;
}

/// \brief Fit raw application data within remaining advertisement capacity.
/// \param payload Raw extra bytes competing for advertisement space.
/// \param local_name device name proposed for inclusion in the advertising packet.
/// \param cfg Advertisement placement and overflow policy applied to the payload.
/// \param max_advertisement_bytes controller limit for the complete encoded advertising packet.
/// \param max_payload_bytes maximum user bytes permitted after framing overhead.
/// \return Original or truncated bytes that fit or std::nullopt when no payload fits.
std::optional<std::vector<uint8_t>> constrain_extra_advertisement_payload(
    const std::vector<uint8_t>& payload,
    const std::string& local_name,
    const mrs_uav_bluetooth::config::NodeConfig& cfg,
    size_t max_advertisement_bytes,
    size_t& max_payload_bytes) {
    // Subtract configured fields and their type/length bytes from controller capacity.
    auto static_advertise_data = cfg.advertise_data;
    static_advertise_data.erase(mrs_uav_bluetooth::bluez::kDefaultAdvertisementExtraDataType);

    static_cast<void>(local_name);
    const bool reserve_primary_flags = max_advertisement_bytes <= kPrimaryAdvertisementMaxBytes;
    const size_t static_bytes = estimate_user_data_competing_advertisement_bytes(
        cfg, static_advertise_data, reserve_primary_flags);
    max_payload_bytes = static_bytes + 2 >= max_advertisement_bytes
        ? 0
        : (max_advertisement_bytes - static_bytes - 2);

    if (max_payload_bytes == 0) {
        return std::nullopt;
    }
    if (payload.size() <= max_payload_bytes) {
        return payload;
    }
    return std::vector<uint8_t>(payload.begin(), payload.begin() + max_payload_bytes);
}

/// \brief Keep the ordered service UUID prefix that fits in the primary packet.
/// \param service_uuids service identifiers proposed for the primary advertising packet.
/// \param remaining_uuid_payload_bytes primary-packet capacity still available for service UUIDs.
/// \return Ordered prefix of service UUIDs that fits the remaining capacity.
std::vector<std::string> select_advertised_service_uuids(
    const std::vector<std::string>& service_uuids,
    size_t remaining_uuid_payload_bytes) {
    // Keep the configured UUID order and stop before the next UUID would overflow.
    if (remaining_uuid_payload_bytes == 0) {
        return {};
    }

    std::vector<std::string> advertised;
    advertised.reserve(service_uuids.size());
    size_t used_bytes = 0;
    for (const auto& uuid : service_uuids) {
        const size_t uuid_size = advertising_uuid_size(uuid);
        if (used_bytes + uuid_size > remaining_uuid_payload_bytes) {
            break;
        }
        advertised.push_back(uuid);
        used_bytes += uuid_size;
    }
    return advertised;
}

/// \brief Append configured service UUIDs after runtime UUIDs without duplicates.
/// \param runtime_uuids UUIDs required by live bridge services.
/// \param configured_uuids service identifiers explicitly requested by configuration.
/// \return Runtime UUIDs followed by nonduplicate configured UUIDs.
std::vector<std::string> merge_service_uuids(const std::vector<std::string>& runtime_uuids,
                                             const std::vector<std::string>& configured_uuids) {
    // Merge service UUIDs.
    std::vector<std::string> merged = runtime_uuids;
    merged.reserve(runtime_uuids.size() + configured_uuids.size());
    for (const auto& uuid : configured_uuids) {
        if (std::find(merged.begin(), merged.end(), uuid) == merged.end()) {
            merged.push_back(uuid);
        }
    }
    return merged;
}

/// \brief GATT descriptors expose the encoder layout. Changing a type or an
/// \param cfg GATT services and bridge exports serialized for change detection.
/// \return Stable sorted signature of exported services profiles and bridge metadata.
std::string gatt_layout_signature_for_config(const mrs_uav_bluetooth::config::NodeConfig& cfg) {
    // Serialize every exported GATT shape into sorted tokens so overlay reloads
    // rebuild BlueZ objects exactly when remote discovery metadata changes.
    std::vector<std::string> tokens;
    tokens.reserve(cfg.shared_topics.size() + cfg.gatt_profile_uuids.size() + 3);

    tokens.push_back(cfg.enable_server ? "server:on" : "server:off");
    if (cfg.enable_server && cfg.enable_wifi_service) {
        tokens.push_back("svc:wifi");
    }
    if (cfg.enable_server && cfg.enable_time_service) {
        tokens.push_back("svc:time");
    }
    for (const auto& uuid : cfg.gatt_profile_uuids) {
        tokens.push_back("profile:" + uuid);
    }

    for (const auto& shared_topic : cfg.shared_topics) {
        if (shared_topic.transport != "gatt" || !cfg.enable_server ||
            (shared_topic.mode != "export" && shared_topic.mode != "both")) {
            continue;
        }
        // GATT descriptors expose the encoder layout. Changing a type or an
        // expression must rebuild that service even when the topic name stays
        // the same, otherwise nearby clients can see stale metadata.
        std::ostringstream bridge_layout;
        bridge_layout << "bridge:" << shared_topic.bridge_name << ':'
                      << shared_topic.message_type << ':'
                      << shared_topic.payload_format << ':' << shared_topic.rate_hz;
        for (const auto& member : shared_topic.member_specs) {
            bridge_layout << '|' << member.target << ':'
                          << member.value_type << ':' << member.expression;
        }
        tokens.push_back(bridge_layout.str());
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
    // Declare parameters first, then construct the Bluetooth and ROS runtime from them.
    configure_parameters();
    build_runtime();
}

ServiceNode::~ServiceNode() {
    // Stop callbacks and timers before tearing down transports and D-Bus objects.
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
    if (mesh_timer_) {
        mesh_timer_->cancel();
        mesh_timer_.reset();
    }
    mesh_swarm_coordinator_.reset();
    mesh_app_.reset();
    mesh_dbus_.reset();
    if (client_ && gatt_event_token_ != 0) {
        client_->remove_gatt_event_handler(gatt_event_token_);
        gatt_event_token_ = 0;
    }
    if (cache_ && cache_observer_token_ != 0) {
        cache_->remove_observer(cache_observer_token_);
        cache_observer_token_ = 0;
    }
    wait_for_peer_tasks();
    serial_profile_.reset();
    serial_ssh_server_.reset();
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
    // Resolve package defaults and reject invalid timing paths and topic settings before D-Bus startup.
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
    // Validate and select the radio mode before touching adapter properties.
    active_config_ = config::load_effective_config(
        get_parameter("default_config_path").as_string(), "", hostname_);
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
    if (adapter_path_.empty()) {
        // A raw-HCI Mesh bearer temporarily removes Adapter1, but the kernel
        // controller still exists. Keep its stable future BlueZ path for the
        // LE handoff after Mesh releases ownership.
        std::vector<std::string> controllers;
        std::error_code error;
        for (const auto& entry : std::filesystem::directory_iterator(
                 "/sys/class/bluetooth", error)) {
            const auto name = entry.path().filename().string();
            if (name.starts_with("hci")) controllers.push_back(name);
        }
        std::sort(controllers.begin(), controllers.end());
        if (controllers.empty()) throw std::runtime_error("No Bluetooth controller is present");
        adapter_path_ = "/org/bluez/" + controllers.front();
    }

    cache_ = std::make_unique<bluez::ObjectManagerCache>(*dbus_, get_logger());
    cache_->start();

    adapter_ = std::make_unique<bluez::AdapterController>(*dbus_, adapter_path_, get_logger());
    client_ = std::make_unique<bluez::BluezClient>(
        *dbus_,
        *cache_,
        adapter_path_,
        get_logger());
    // The initial apply_config releases any leftover Mesh bearer before it
    // powers and configures Adapter1.
    pairing_agent_ = std::make_unique<bluez::BluezPairingAgent>(
        *dbus_, get_logger(),
        get_parameter("auto_pair").as_bool(),
        get_parameter("auto_trust").as_bool());
    cache_observer_token_ = cache_->add_observer(
        [this](bluez::CacheEvent event, const std::string& object_path) {
            // Reconcile peer state whenever BlueZ adds, removes, or changes a cached object.
            on_cache_event(event, object_path);
        });
    pairing_agent_->set_event_callback(
        [this](const std::string& event_type, const std::string& device_path) {
            // Feed pairing-agent decisions back into the peer state machine.
            on_pairing_event(event_type, device_path);
        });
    pairing_agent_->set_request_policy_callback(
        [this](const std::string& event_type, const std::string& device_path) {
            // Admit only pairing requests that match the current peer and overlay policy.
            return should_allow_pairing_request(event_type, device_path);
        });
    pairing_agent_->register_agent(get_parameter("pairing_agent").as_string());
    client_->add_notification_handler(
        [this](const std::vector<uint8_t>& data,
               const std::string& uuid,
               const std::string& characteristic_path) {
            // Decode remote GATT notifications into configured import bridges.
            on_notification(data, uuid, characteristic_path);
        });
    gatt_event_token_ = client_->add_gatt_event_handler(
        [this](const std::string& event_type,
               const std::string& object_path,
               const std::string& detail) {
            // Record GATT operation outcomes for status and recovery logic.
            on_gatt_event(event_type, object_path, detail);
        });

    overlay_config_ = std::make_unique<config::OverlayConfigManager>(
        *this,
        get_logger(),
        get_parameter("default_config_path").as_string(),
        hostname_);
    overlay_config_->on_config_changed([this](const config::NodeConfig& cfg) {
        // Apply each accepted overlay only while the node still accepts callbacks.
        if (!can_run_callbacks()) {
            return;
        }
        apply_config(cfg);
    });

    netplan_ = std::make_unique<network::NetplanManager>(active_config_.wifi_netplan_config_path);

    export_bridges_ = std::make_unique<bridge::ExportBridgeManager>(*this, get_logger());
    export_bridges_->set_registry(&bridge_registry_);
    export_bridges_->set_state_mutex(&state_mutex_);
    import_bridges_ = std::make_unique<bridge::ImportBridgeManager>(*this, get_logger());
    import_bridges_->set_registry(&bridge_registry_);
    import_bridges_->set_state_mutex(&state_mutex_);
    transport_bridges_ =
        std::make_unique<bridge::TransportBridgeManager>(*this, get_logger());
    transport_bridges_->set_advertisement_sender(
        [this](const std::vector<uint8_t>& payload) {
            // Serialize advertisement updates against overlay changes and reject stale publishers.
            std::unique_lock<std::mutex> lock(config_apply_mutex_, std::try_to_lock);
            if (!lock.owns_lock() || !can_run_callbacks()) return false;
            set_advertisement_payload(payload, false);
            return true;
        });
    transport_bridges_->set_mesh_sender(
        [this](const config::SharedTopicConfig& configured,
               const std::vector<uint8_t>& payload) {
            // Serialize mesh sends against overlay changes and reject stale publishers.
            std::unique_lock<std::mutex> lock(config_apply_mutex_, std::try_to_lock);
            if (!lock.owns_lock() || !can_run_callbacks()) return false;
            send_mesh_bridge_payload(configured, payload);
            return true;
        });
    peers_ = std::make_unique<peer::PeerManager>(*this, get_logger());
    ros_ = std::make_unique<ros::RosInterfaceManager>(*this);
    create_services();

    status_timer_ = create_grouped_wall_timer(*this, std::chrono::seconds(2), [this]() {
        // Publish health immediately from the default status timer once callbacks are enabled.
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
        refresh_advertisement_topic_subscription();
        recover_bluez_daemon_if_requested();
        reconcile_adapter_state_if_requested();
        // Discovery can stop without a device event after BlueZ changes the
        // controller role. Keep the requested scan alive even when there is
        // no peer event to schedule the ordinary connection reconciler.
        if (!config_apply_in_progress_.load() && active_config_.enable_scan &&
            !active_config_.enable_mesh &&
            !advertisement_transmit_window_.load() &&
            client_ && !client_->is_scanning()) {
            schedule_peer_reconcile(std::chrono::milliseconds(1));
        }

    }, timer_callback_group_);

    overlay_config_->load_initial();
}

bool ServiceNode::can_run_callbacks() const {
    // Suppress callbacks once shutdown begins or the ROS context stops.
    return !shutting_down_.load() && rclcpp::ok();
}

void ServiceNode::apply_adapter_state(const config::NodeConfig& cfg) {
    if (!adapter_) {
        return;
    }

    const bool mesh_exclusive = cfg.enable_mesh && !cfg.enable_server &&
        !cfg.enable_scan && !cfg.enable_serial_port_profile &&
        !cfg.auto_connect_enable;
    if (mesh_exclusive) {
        // bluetooth-meshd owns advertising/scanning state through the kernel
        // management interface in this mode. Adapter1 property writes either
        // fail or contend with Mesh, so leave controller policy to the daemon.
        adapter_state_reconcile_requested_.store(false);
        return;
    }

    // Each setter below can cause an AdapterChanged cache callback before the
    // remaining setters have completed. Without this guard, that callback sees
    // a legitimate intermediate state and recursively reapplies the whole
    // group, eventually flooding BlueZ with overlapping Set requests.
    if (adapter_state_apply_in_progress_.exchange(true)) {
        return;
    }

    const auto clear_apply_guard = [this]() {
        // Release the adapter-transition guard after either success or rollback.
        adapter_state_apply_in_progress_.store(false);
    };

    try {
        const auto adapter_info = cache_ ? cache_->adapter(adapter_path_) : std::optional<bluez::AdapterInfo>{};
        const bool broadcast_mode = cfg.advertise_mode == "broadcast";
        const bool should_be_discoverable = !broadcast_mode &&
            cfg.advertise_discoverable.value_or(
                cfg.enable_server || cfg.enable_serial_port_profile);
        const bool should_be_connectable = !broadcast_mode;
        const bool should_be_pairable = cfg.auto_pair;

        if (!adapter_info || !adapter_info->powered) {
            adapter_->power_on();
        }
        if (!adapter_info || adapter_info->alias != hostname_) {
            adapter_->set_alias(hostname_);
        }
        if (!adapter_info || adapter_info->pairable != should_be_pairable) {
            adapter_->set_pairable(should_be_pairable);
        }
        if (!adapter_info ||
            adapter_info->connectable != should_be_connectable) {
            adapter_->set_connectable(should_be_connectable);
        }
        adapter_->set_pairable_timeout(0);

        if (!adapter_info ||
            adapter_info->discoverable != should_be_discoverable ||
            (should_be_discoverable && adapter_info->discoverable_timeout != cfg.discoverable_timeout)) {
            adapter_->set_discoverable(should_be_discoverable, cfg.discoverable_timeout);
        }
    } catch (...) {
        clear_apply_guard();
        throw;
    }
    clear_apply_guard();
}

void ServiceNode::reconcile_adapter_state_if_requested() {
    // Apply one rate-limited correction after BlueZ property signals finish a
    // multi-property adapter transition.
    if (config_apply_in_progress_.load() ||
        !adapter_state_reconcile_requested_.load()) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (next_adapter_state_reconcile_ != std::chrono::steady_clock::time_point{} &&
        now < next_adapter_state_reconcile_) {
        return;
    }
    adapter_state_reconcile_requested_.store(false);

    const bool mesh_exclusive = active_config_.enable_mesh &&
        !active_config_.enable_server && !active_config_.enable_scan &&
        !active_config_.enable_serial_port_profile &&
        !active_config_.auto_connect_enable;
    if (mesh_exclusive) return;

    const auto adapter_info = cache_ ? cache_->adapter(adapter_path_)
                                     : std::optional<bluez::AdapterInfo>{};
    const bool broadcast_mode =
        active_config_.advertise_mode == "broadcast";
    const bool should_be_discoverable = !broadcast_mode &&
        active_config_.advertise_discoverable.value_or(
            active_config_.enable_server ||
            active_config_.enable_serial_port_profile);
    const bool should_be_connectable = !broadcast_mode;
    const bool drifted = !adapter_info ||
        !adapter_info->powered ||
        adapter_info->connectable != should_be_connectable ||
        adapter_info->pairable != active_config_.auto_pair ||
        adapter_info->alias != hostname_ ||
        adapter_info->discoverable != should_be_discoverable ||
        (should_be_discoverable &&
         adapter_info->discoverable_timeout != active_config_.discoverable_timeout);
    if (!drifted) {
        next_adapter_state_reconcile_ = {};
        return;
    }

    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[node] adapter state drift persists; applying one coalesced correction");
    try {
        apply_adapter_state(active_config_);
    } catch (const std::exception& error) {
        // Timer callbacks must leave ROS control available when the adapter
        // rejects a setting. Keep the desired policy pending for recovery.
        adapter_state_reconcile_requested_.store(true);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Adapter policy reconciliation failed: %s", error.what());
    }
    next_adapter_state_reconcile_ = now + std::chrono::seconds(5);
}

void ServiceNode::recover_bluez_daemon_if_requested() {
    if (!bluez_daemon_recovery_requested_.load() ||
        config_apply_in_progress_.load() || !cache_) {
        return;
    }

    config::NodeConfig recovery_config;
    {
        std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
        if (active_config_.enable_mesh) {
            // bluetoothd is intentionally absent while bluetooth-meshd owns
            // the controller. The ordinary overlay transition will rebuild
            // BlueZ objects when Mesh releases it.
            bluez_daemon_recovery_requested_.store(false);
            return;
        }
        if (!cache_->adapter(adapter_path_)) {
            return;
        }
        recovery_config = active_config_;
    }

    bluez_daemon_recovery_requested_.store(false);
    try {
        RCLCPP_WARN(get_logger(),
                    "BlueZ daemon was replaced; reapplying the active radio configuration");
        apply_config(recovery_config);
    } catch (const std::exception& error) {
        bluez_daemon_recovery_requested_.store(true);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                            "BlueZ daemon recovery failed: %s", error.what());
    }
}

void ServiceNode::configure_serial_profile(const config::NodeConfig& cfg) {
    // Reuse an identical registered profile or rebuild its RFCOMM and SSH handlers.
    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);

    if (!cfg.enable_serial_port_profile) {
        serial_profile_.reset();
        serial_ssh_server_.reset();
        serial_sshd_path_.clear();
        return;
    }

    if (serial_profile_ && serial_profile_->registered() &&
        serial_profile_->options().channel == cfg.serial_port_channel &&
        serial_sshd_path_ == cfg.serial_sshd_path) {
        return;
    }

    serial_profile_.reset();
    serial_ssh_server_.reset();

    auto ssh_server = std::make_unique<serial::SerialSshServer>(
        get_logger(), cfg.serial_sshd_path);
    bluez::SerialPortProfileOptions options;
    options.role = bluez::ProfileRole::Server;
    options.channel = cfg.serial_port_channel;
    options.require_authentication = true;
    options.require_authorization = false;
    options.auto_connect = false;
    options.name = "MRS UAV SSH Serial Port";

    auto profile = std::make_unique<bluez::SerialPortProfile>(
        *server_dbus_,
        "/cz/cvut/mrs/uav/bluetooth/serial_server",
        get_logger(),
        options);
    profile->set_connection_handler(
        [this](const std::string& device_path,
               int socket_fd,
               const std::map<std::string, sdbus::Variant>&) {
            // Hand the accepted RFCOMM socket to the SSH server or close it if shutdown raced.
            std::lock_guard<std::recursive_mutex> callback_lock(state_mutex_);
            if (!can_run_callbacks() || !serial_ssh_server_) {
                ::close(socket_fd);
                throw std::runtime_error("serial SSH service is shutting down");
            }
            serial_ssh_server_->start_session(device_path, socket_fd);
        });
    profile->set_disconnection_handler([this](const std::string& device_path) {
        // Stop the SSH session associated with the peer BlueZ disconnected.
        std::lock_guard<std::recursive_mutex> callback_lock(state_mutex_);
        if (serial_ssh_server_) {
            serial_ssh_server_->stop_session(device_path);
        }
    });
    profile->set_release_handler([this]() {
        // Stop every SSH session because BlueZ released the entire serial profile.
        std::lock_guard<std::recursive_mutex> callback_lock(state_mutex_);
        if (serial_ssh_server_) {
            serial_ssh_server_->stop_all();
        }
    });
    serial_sshd_path_ = cfg.serial_sshd_path;
    serial_ssh_server_ = std::move(ssh_server);
    try {
        profile->register_profile();
        serial_profile_ = std::move(profile);
    } catch (const std::exception& error) {
        serial_ssh_server_.reset();
        serial_sshd_path_.clear();
        RCLCPP_ERROR(get_logger(), "Serial Port Profile is unavailable: %s", error.what());
    } catch (...) {
        serial_ssh_server_.reset();
        serial_sshd_path_.clear();
        RCLCPP_ERROR(get_logger(), "Serial Port Profile is unavailable: unknown error");
    }
}

void ServiceNode::refresh_advertisement_registration(
    bool resume_scan_after_registration) {
    // Re-register the advertisement so BlueZ consumes the current payload and properties.
    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    if (!advertisement_ || adapter_path_.empty()) {
        return;
    }

    const auto& local_name = active_config_.advertise_local_name;
    std::vector<std::string> service_uuids;
    if (gatt_app_) {
        service_uuids.reserve(gatt_app_->services().size());
        for (const auto& service : gatt_app_->services()) {
            service_uuids.push_back(service->uuid());
        }
    }
    service_uuids = merge_service_uuids(service_uuids, active_config_.advertise_service_uuids);

    const auto adapter_info = cache_ ? cache_->adapter(adapter_path_) : std::optional<bluez::AdapterInfo>{};
    const auto secondary_channel = select_secondary_channel(active_config_, adapter_info);
    const size_t advertisement_max_bytes = resolve_advertisement_max_bytes(
        active_config_, secondary_channel, adapter_info);
    if (active_config_.advertise_size == "extended" && secondary_channel.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "advertise_size=extended requested, but adapter does not report usable extended advertising; falling back to legacy 31-byte advertisement");
    }

    advertisement_->set_local_name(local_name);
    advertisement_->set_discoverable(
        active_config_.advertise_mode != "broadcast" &&
        active_config_.advertise_discoverable.value_or(
            active_config_.enable_server ||
            active_config_.enable_serial_port_profile));
    advertisement_->set_discoverable_timeout(
        static_cast<uint16_t>(std::min<uint32_t>(active_config_.discoverable_timeout,
                                                 std::numeric_limits<uint16_t>::max())));
    advertisement_->set_includes(active_config_.advertise_includes);
    advertisement_->set_solicit_uuids(active_config_.advertise_solicit_uuids);
    const auto& effective_config = active_config_;
    advertisement_->set_manufacturer_data(effective_config.advertise_manufacturer_data);
    advertisement_->set_service_data(active_config_.advertise_service_data);
    auto advertise_data = active_config_.advertise_data;
    if (advertisement_extra_payload_) {
        size_t max_payload_bytes = 0;
        auto constrained_payload = constrain_extra_advertisement_payload(
            *advertisement_extra_payload_,
            local_name,
            effective_config,
            advertisement_max_bytes,
            max_payload_bytes);
        if (constrained_payload.has_value()) {
            if (constrained_payload->size() != advertisement_extra_payload_->size()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Trimmed BLE advertisement user data from %zu to %zu bytes to fit adapter MaxAdvLen=%zu (payload budget=%zu)",
                                     advertisement_extra_payload_->size(), constrained_payload->size(),
                                     advertisement_max_bytes, max_payload_bytes);
                advertisement_extra_payload_ = *constrained_payload;
            }
            advertise_data[bluez::kDefaultAdvertisementExtraDataType] = *advertisement_extra_payload_;
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Dropping BLE advertisement user data (%zu bytes): no payload bytes fit adapter MaxAdvLen=%zu",
                                 advertisement_extra_payload_->size(), advertisement_max_bytes);
            advertisement_extra_payload_.reset();
        }
    }
    advertisement_->set_data(advertise_data);
    advertisement_->set_scan_response_service_uuids(active_config_.advertise_scan_response_service_uuids);
    advertisement_->set_scan_response_manufacturer_data(
        active_config_.advertise_scan_response_manufacturer_data);
    advertisement_->set_scan_response_solicit_uuids(
        active_config_.advertise_scan_response_solicit_uuids);
    advertisement_->set_scan_response_service_data(
        active_config_.advertise_scan_response_service_data);
    advertisement_->set_scan_response_data(active_config_.advertise_scan_response_data);
    advertisement_->set_appearance(active_config_.advertise_appearance);
    advertisement_->set_duration(active_config_.advertise_duration);
    advertisement_->set_timeout(active_config_.advertise_timeout);
    advertisement_->set_secondary_channel(secondary_channel);
    advertisement_->set_min_interval(active_config_.advertise_min_interval);
    advertisement_->set_max_interval(active_config_.advertise_max_interval);
    advertisement_->set_tx_power(active_config_.advertise_tx_power);

    const auto estimated_primary_bytes = estimate_primary_advertisement_bytes(
        local_name, effective_config, advertise_data,
        !secondary_channel.empty() && active_config_.advertise_mode == "peripheral");
    const size_t remaining_uuid_payload_bytes = estimated_primary_bytes + 2 >= advertisement_max_bytes
        ? 0
        : (advertisement_max_bytes - estimated_primary_bytes - 2);
    const auto advertised_service_uuids = select_advertised_service_uuids(
        service_uuids, remaining_uuid_payload_bytes);
    if (advertised_service_uuids.size() != service_uuids.size()) {
        RCLCPP_INFO(get_logger(),
                    "Prepared primary BLE advertisement trimmed from %zu to %zu service UUIDs",
                    service_uuids.size(), advertised_service_uuids.size());
    }

    std::vector<std::vector<std::string>> attempts;
    auto add_attempt = [&attempts](const std::vector<std::string>& advertised_uuids) {
        // Queue each reduced UUID set only once while searching for a packet that fits.
        if (std::find(attempts.begin(), attempts.end(), advertised_uuids) == attempts.end()) {
            attempts.push_back(advertised_uuids);
        }
    };
    add_attempt(advertised_service_uuids);
    add_attempt(service_uuids);
    add_attempt({});

    const bool broadcast_scan_enabled = client_ && active_config_.enable_scan &&
        active_config_.advertise_mode == "broadcast";
    if (broadcast_scan_enabled) {
        // This controller may reject Add Advertising while LE discovery is
        // physically running. Stop this process's discovery request before
        // registration; the caller chooses below when discovery resumes.
        (void)client_->stop_scan();
    }

    bool advertisement_registered = false;
    std::string last_error_message;
    for (const auto& advertised_uuids : attempts) {
        advertisement_->set_service_uuids(advertised_uuids);
        try {
            const auto log_secondary_channel = secondary_channel.empty()
                ? std::string{"legacy"}
                : secondary_channel;
            RCLCPP_INFO(get_logger(),
                        "Registering BLE advertisement (name=%s, uuids=%zu, size=%s, secondary=%s, estimated_bytes=%zu, budget=%zu)",
                        local_name.c_str(), advertised_uuids.size(), active_config_.advertise_size.c_str(),
                        log_secondary_channel.c_str(), estimated_primary_bytes, advertisement_max_bytes);
            advertisement_->unregister_advertisement(adapter_path_);
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
        throw std::runtime_error(last_error_message.empty()
                                     ? "Failed to register advertisement"
                                     : last_error_message);
    }

    if (resume_scan_after_registration && broadcast_scan_enabled &&
        !client_->start_scan(active_config_.scan_mode, false)) {
        throw std::runtime_error(
            "Advertisement registered, but the receive scan request failed");
    }
}

void ServiceNode::open_advertisement_scan_window() {
    if (!client_ || !advertisement_ || !advertisement_->is_registered() ||
        !active_config_.enable_scan ||
        active_config_.advertise_mode != "broadcast") {
        return;
    }

    // Removing the packet lets BlueZ resume this process's already-owned
    // discovery session on controllers that serialize scanning and advertising.
    advertisement_->unregister_advertisement(adapter_path_);
    const auto resume_deadline = std::chrono::steady_clock::now() +
        kAdvertisementScanResumeTimeout;
    while (!client_->is_scanning() &&
           std::chrono::steady_clock::now() < resume_deadline) {
        std::this_thread::sleep_for(kAdvertisementScanPollPeriod);
    }

    if (!client_->is_scanning()) {
        // Recreate discovery when the advertisement interval removed BlueZ's
        // previous discovery owner.
        (void)client_->stop_scan();
        if (!client_->start_scan(active_config_.scan_mode, false)) {
            RCLCPP_WARN(get_logger(),
                        "Could not open the advertisement receive window");
            return;
        }
    }

    uint64_t hostname_phase = 0;
    for (const unsigned char ch : hostname_) {
        hostname_phase = hostname_phase * 33U + ch;
    }
    // Mix the unique hostname with the cycle number. The wide changing phase
    // distributes any number of equal-rate peers across transmit and receive
    // roles on successive cycles.
    hostname_phase += advertisement_scan_sequence_++ * 0x9e3779b97f4a7c15ULL;
    hostname_phase =
        (hostname_phase ^ (hostname_phase >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    hostname_phase =
        (hostname_phase ^ (hostname_phase >> 27U)) * 0x94d049bb133111ebULL;
    hostname_phase ^= hostname_phase >> 31U;
    const auto dwell_ms = advertisement_scan_window_ms(
        active_config_,
        cache_ ? cache_->adapter(adapter_path_) :
                 std::optional<bluez::AdapterInfo>{}) +
        (hostname_phase % kAdvertisementScanDwellVariationMs);
    // Keep the packet absent for the resolved receive period plus phase variation.
    // BlueZ replies to UnregisterAdvertisement before the controller confirms
    // removal and immediately makes that instance number reusable. This guard
    // prevents the next registration from racing the queued removal.
    std::this_thread::sleep_for(std::chrono::milliseconds(dwell_ms));
}

void ServiceNode::refresh_advertisement_topic_subscription() {
    if (!can_run_callbacks()) {
        return;
    }

    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    const auto topic = active_config_.advertise_extra_data_topic;
    const bool configured_export = std::any_of(
        active_config_.shared_topics.begin(), active_config_.shared_topics.end(),
        [](const auto& bridge) {
            // Detect whether this overlay exports any topic through advertisements.
            return bridge.transport == "advertisement" &&
                (bridge.mode == "export" || bridge.mode == "both");
        });
    if (configured_export) {
        // The declarative bridge owns this advertisement while configured.
        // Removing the overlay returns ownership to the raw ROS topic.
        advertisement_payload_sub_.reset();
        return;
    }
    // Raw advertisement data uses the connectionless broadcast data plane.
    // Broadcast mode keeps that stream independent from peripheral sessions;
    // Mesh uses its dedicated transport path.
    const bool topic_enabled = active_config_.enable_server &&
        active_config_.advertise_mode == "broadcast" &&
        !active_config_.enable_mesh && !topic.empty();

    if (!topic_enabled) {
        if (advertisement_payload_sub_) {
            RCLCPP_INFO(get_logger(), "Stopped monitoring advertisement payload topic");
            advertisement_payload_sub_.reset();
        }
        if (advertisement_extra_payload_) {
            advertisement_extra_payload_.reset();
            if (advertisement_) {
                refresh_advertisement_registration();
            }
        }
        return;
    }

    bool has_publishers = false;
    try {
        has_publishers = !get_publishers_info_by_topic(topic).empty();
    } catch (const std::exception&) {
        has_publishers = false;
    }

    const bool matching_subscription = advertisement_payload_sub_ &&
        util::normalize_ros_topic(advertisement_payload_sub_->get_topic_name()) == topic;
    if (!has_publishers) {
        if (advertisement_payload_sub_) {
            RCLCPP_INFO(get_logger(), "Advertisement payload topic has no publishers, disabling dynamic payload");
            advertisement_payload_sub_.reset();
        }
        if (advertisement_extra_payload_) {
            advertisement_extra_payload_.reset();
            if (advertisement_) {
                refresh_advertisement_registration();
            }
        }
        return;
    }

    if (matching_subscription) {
        return;
    }

    rclcpp::SubscriptionOptions options;
    options.callback_group = service_callback_group_;
    advertisement_payload_sub_.reset();
    advertisement_payload_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        topic,
        rclcpp::QoS(10),
        [this](const std_msgs::msg::UInt8MultiArray::SharedPtr message) {
            // Replace the live advertisement bytes with the newest ROS payload.
            handle_advertisement_payload(message);
        },
        options);
    RCLCPP_INFO(get_logger(), "Monitoring advertisement payload topic: %s", topic.c_str());
}

void ServiceNode::handle_advertisement_payload(
    const std_msgs::msg::UInt8MultiArray::SharedPtr message) {
    // Accept raw bytes only while the exclusive broadcast overlay owns advertising.
    if (!message) return;
    if (active_config_.advertise_mode != "broadcast" ||
        active_config_.enable_mesh) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Ignoring raw advertisement payload outside exclusive broadcast mode");
        return;
    }
    set_advertisement_payload(
        std::vector<uint8_t>(message->data.begin(), message->data.end()), true);
}

void ServiceNode::set_advertisement_payload(std::vector<uint8_t> payload,
                                            bool allow_truncate) {
    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    if (active_config_.advertise_mode != "broadcast" ||
        active_config_.enable_mesh) {
        // An already queued ROS/timer callback may outlive an overlay switch.
        // Re-check under the same lock that protects the advertisement object;
        // accept connectionless data only for the active broadcast object.
        return;
    }
    if (advertisement_ && advertisement_->was_released()) {
        // Release is a terminal BlueZ lifecycle event. The next configuration
        // transition owns re-registration.
        throw std::runtime_error("BlueZ released the advertisement. See the Bluetooth daemon error.");
    }
    const auto& local_name = active_config_.advertise_local_name;
    const auto adapter_info = cache_ ? cache_->adapter(adapter_path_) : std::optional<bluez::AdapterInfo>{};
    const auto secondary_channel = select_secondary_channel(active_config_, adapter_info);
    const size_t max_advertisement_bytes =
        resolve_advertisement_max_bytes(active_config_, secondary_channel, adapter_info);
    const auto& effective_config = active_config_;
    size_t max_payload_bytes = 0;
    auto constrained_payload = constrain_extra_advertisement_payload(
        payload,
        local_name,
        effective_config,
        max_advertisement_bytes,
        max_payload_bytes);
    if (!constrained_payload.has_value() ||
        constrained_payload->size() != payload.size()) {
        if (!allow_truncate) {
            throw std::runtime_error(
                "configured advertisement bridge frame is " +
                std::to_string(payload.size()) + " bytes, but only " +
                std::to_string(max_payload_bytes) + " bytes fit; reduce the "
                "members mapping or use extended advertising");
        }
        if (constrained_payload.has_value()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Trimmed BLE advertisement user data from %zu to %zu bytes to fit adapter MaxAdvLen=%zu (payload budget=%zu)",
                payload.size(), constrained_payload->size(),
                max_advertisement_bytes, max_payload_bytes);
            payload = std::move(*constrained_payload);
        } else {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Dropping BLE advertisement user data (%zu bytes): no payload bytes fit adapter MaxAdvLen=%zu",
                payload.size(), max_advertisement_bytes);
            payload.clear();
        }
    }

    if (advertisement_ && advertisement_->is_registered() &&
        advertisement_extra_payload_ && *advertisement_extra_payload_ == payload) {
        return;
    }

    const bool data_property_exported = advertisement_extra_payload_.has_value() ||
        !active_config_.advertise_data.empty();
    // Property mode changes Data on the registered D-Bus object. Reregister
    // mode performs a complete LEAdvertisingManager1 unregister/register for
    // each sample. Hardware A/B tests use reregister mode to measure the public
    // D-Bus path independently from BlueZ's asynchronous property-update
    // behavior.
    const bool can_update_in_place = advertisement_ &&
        advertisement_->is_registered() && data_property_exported &&
        active_config_.advertise_update_strategy == "property";

    advertisement_extra_payload_ = std::move(payload);
    if (can_update_in_place) {
        auto advertise_data = active_config_.advertise_data;
        if (!advertisement_extra_payload_->empty()) {
            advertise_data[bluez::kDefaultAdvertisementExtraDataType] = *advertisement_extra_payload_;
        }
        advertisement_->set_data(advertise_data);
        advertisement_->emit_property_changed("Data");
        RCLCPP_DEBUG(get_logger(),
                    "Submitted BLE advertisement data update (%zu bytes)",
                    advertisement_extra_payload_->size());
    } else if (advertisement_) {
        open_advertisement_scan_window();
        // The peer reconciler runs on another callback group. Mark this
        // interval before stopping discovery to hold off the reconciler on
        // controllers that serialize scanning and advertising.
        AtomicFlagGuard transmit_window(advertisement_transmit_window_);
        refresh_advertisement_registration(false);
        // Leave the newly registered payload on air for a complete radio
        // interval before the next queued sample opens another scan window.
        std::this_thread::sleep_for(std::chrono::milliseconds(
            advertisement_transmit_window_ms(
                active_config_,
                cache_ ? cache_->adapter(adapter_path_) :
                         std::optional<bluez::AdapterInfo>{})));
    }
}

void ServiceNode::create_services() {
    ros::ServiceServers::Handlers handlers;
    handlers.list_devices = [this](auto request, auto response) {
        // Return the current BlueZ device-cache snapshot.
        handle_list_devices(request, response);
    };
    handlers.get_device = [this](auto request, auto response) {
        // Look up one cached device by its requested address.
        handle_get_device(request, response);
    };
    handlers.connect_device = [this](auto request, auto response) {
        // Start a bounded connection attempt for the requested device.
        handle_connect_device(request, response);
    };
    handlers.disconnect_device = [this](auto request, auto response) {
        // Cancel pending work and disconnect the requested device.
        handle_disconnect_device(request, response);
    };
    handlers.pair_device = [this](auto request, auto response) {
        // Run pairing for the requested device and return its final result.
        handle_pair_device(request, response);
    };
    handlers.set_device_trust = [this](auto request, auto response) {
        // Apply the requested BlueZ Trusted value to one device.
        handle_set_device_trust(request, response);
    };
    handlers.remove_device = [this](auto request, auto response) {
        // Remove the requested device and its cached bond from the adapter.
        handle_remove_device(request, response);
    };
    handlers.list_gatt_services = [this](auto request, auto response) {
        // List cached services belonging to the requested peer.
        handle_list_gatt_services(request, response);
    };
    handlers.list_gatt_characteristics = [this](auto request, auto response) {
        // List cached characteristics below the requested service.
        handle_list_gatt_characteristics(request, response);
    };
    handlers.list_gatt_descriptors = [this](auto request, auto response) {
        // List cached descriptors below the requested characteristic.
        handle_list_gatt_descriptors(request, response);
    };
    handlers.find_gatt_path = [this](auto request, auto response) {
        // Resolve a service, characteristic, or descriptor UUID to its D-Bus path.
        handle_find_gatt_path(request, response);
    };
    handlers.read_gatt_value = [this](auto request, auto response) {
        // Read the requested remote characteristic or descriptor value.
        handle_read_gatt_value(request, response);
    };
    handlers.write_gatt_value = [this](auto request, auto response) {
        // Write bytes to the requested remote characteristic or descriptor.
        handle_write_gatt_value(request, response);
    };
    handlers.set_notify = [this](auto request, auto response) {
        // Enable or disable notifications on the requested characteristic.
        handle_set_notify(request, response);
    };
    handlers.set_scan_enabled = [this](auto request, auto response) {
        // Start or stop adapter discovery through the scan service.
        handle_set_scan_enabled(request, response);
    };
    handlers.configure_notification_bridge = [this](auto request, auto response) {
        // Create or remove a ROS bridge for one remote notification source.
        handle_configure_notification_bridge(request, response);
    };
    handlers.reload_config = [this](auto request, auto response) {
        // Reload the default and active overlay files from disk.
        handle_reload_config(request, response);
    };
    handlers.set_active_config = [this](auto request, auto response) {
        // Switch overlays while serializing teardown and setup of every transport.
        handle_set_active_config(request, response);
    };

    // Derive a stable API root from the machine hostname across overlay changes.
    // All transport and configuration services share this canonical root while
    // node_topics_prefix remains available for overlay data topics.
    const auto hostname_token = util::sanitize_topic_suffix(
        hostname_.empty() ? "mrs-uav" : hostname_);
    const auto service_root = util::normalize_ros_topic(
        "/" + hostname_token + "/bluetooth");
    services_ = ros_->service_servers().register_all(
        *this, handlers, service_root, service_callback_group_);
    const auto qos = rclcpp::ServicesQoS();
    services_.push_back(create_service<mrs_uav_bluetooth::srv::MeshNetwork>(
        service_root + "/mesh/network",
        [this](const std::shared_ptr<mrs_uav_bluetooth::srv::MeshNetwork::Request> request,
               std::shared_ptr<mrs_uav_bluetooth::srv::MeshNetwork::Response> response) {
            // Attach, join, create, import, leave, or inspect the mesh network through D-Bus.
            handle_mesh_network(request, response);
        },
        qos,
        service_callback_group_));
    services_.push_back(create_service<mrs_uav_bluetooth::srv::MeshSend>(
        service_root + "/mesh/send",
        [this](const std::shared_ptr<mrs_uav_bluetooth::srv::MeshSend::Request> request,
               std::shared_ptr<mrs_uav_bluetooth::srv::MeshSend::Response> response) {
            // Send one application or device-key message through the attached mesh node.
            handle_mesh_send(request, response);
        },
        qos,
        service_callback_group_));
    services_.push_back(create_service<mrs_uav_bluetooth::srv::MeshManagement>(
        service_root + "/mesh/manage",
        [this](const std::shared_ptr<mrs_uav_bluetooth::srv::MeshManagement::Request> request,
               std::shared_ptr<mrs_uav_bluetooth::srv::MeshManagement::Response> response) {
            // Execute key, scan, provisioning, and model-configuration operations through D-Bus.
            handle_mesh_management(request, response);
        },
        qos,
        service_callback_group_));
    services_.push_back(create_service<mrs_uav_bluetooth::srv::MeshSwarm>(
        service_root + "/mesh/swarm",
        [this](const std::shared_ptr<mrs_uav_bluetooth::srv::MeshSwarm::Request> request,
               std::shared_ptr<mrs_uav_bluetooth::srv::MeshSwarm::Response> response) {
            // Execute the automatic multi-peer mesh provisioning operation requested by ROS.
            handle_mesh_swarm(request, response);
        },
        qos,
        service_callback_group_));
}

void ServiceNode::apply_config(const config::NodeConfig& cfg) {
    if (!can_run_callbacks()) {
        return;
    }

    // A hot overlay touches scan, profiles, GATT objects, advertisement state,
    // and adapter properties. Keep those operations as one ordered transition
    // even though the service callback group itself is intentionally reentrant.
    std::lock_guard<std::mutex> config_apply_lock(config_apply_mutex_);
    AtomicFlagGuard config_apply_guard(config_apply_in_progress_);

    const auto new_gatt_layout_signature = gatt_layout_signature_for_config(cfg);
    active_config_ = cfg;
    // Mesh and ordinary LE roles are exclusive radio data planes. A Mesh
    // overlay gives bluetooth-meshd sole bearer ownership; LE overlays create
    // the GATT, advertisement, and Adapter1 discovery objects.
    const bool exclusive_radio_mode =
        cfg.enable_mesh || cfg.advertise_mode == "broadcast";
    // A GATT application re-registration can invalidate remote ATT handles.
    const bool gatt_link_handoff = !local_gatt_layout_signature_.empty() && !exclusive_radio_mode;

    // An older auto-connect task may still be awaiting a D-Bus reply. Gate its
    // fallback/pairing path before cancelling existing and pending connections.
    ConnectionPolicyGuard connection_policy(client_.get(), !exclusive_radio_mode);
    if (cfg.enable_mesh || cfg.advertise_mode != "broadcast") {
        // Dynamic advertisement bridge bytes belong to the overlay that
        // produced them. Clear them before rebuilding another radio mode so the
        // replacement advertisement starts with its own controller budget.
        std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
        advertisement_extra_payload_.reset();
    }
    if (pairing_agent_) {
        pairing_agent_->set_auto_pair(cfg.auto_pair);
        pairing_agent_->set_auto_trust(cfg.auto_trust);
    }
    ros_->status_publisher().configure_topics(cfg.node_topics_prefix);
    ros_->status_publisher().set_advertisement_user_data_type(
        bluez::kDefaultAdvertisementExtraDataType);
    publish_scan_snapshot();

    // Broadcast advertisement data and the Mesh advertising bearer both need
    // uninterrupted controller advertising state. Release any prior GATT
    // connections before configuring either connectionless mode. Adapter
    // policy below then remains non-connectable for the overlay lifetime.
    if ((exclusive_radio_mode || gatt_link_handoff) && client_) {
        client_->stop_scan();
        // Stop every notification subscription before disconnecting. BlueZ
        // otherwise treats those subscriptions as auto-connect requests and a
        // bonded peer can repeatedly re-establish GATT while the connectionless
        // advertisement or Mesh data plane is active.
        const auto devices = client_->get_devices();
        std::vector<std::string> disconnect_targets;
        {
            std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
            for (const auto& device : devices) {
                // A connection request can precede Connected=true. Include
                // peers we contacted and bonded peers with daemon reconnect
                // policy. Discovery-only addresses have no link to release.
                bool pending = false;
                if (peers_) {
                    const auto session = peers_->sessions().find(device.mac);
                    pending = session != peers_->sessions().end() &&
                        (session->second.last_connect_attempt_monotonic > 0.0 ||
                         session->second.pairing_in_progress);
                }
                if (device.connected || device.paired || device.bonded || pending)
                    disconnect_targets.push_back(device.mac);
            }
        }
        // Notification teardown waits for D-Bus callbacks, which also take
        // state_mutex_. Keep those waits outside the snapshot lock.
        for (const auto& device : devices) clear_peer_runtime(device.mac);
        for (const auto& mac : disconnect_targets) {
            if (!client_->disconnect(mac, 5.0)) {
                throw std::runtime_error(
                    "Could not release LE peer " + mac +
                    " before changing the radio mode");
            }
        }
    }
    if (gatt_link_handoff) {
        RCLCPP_INFO(get_logger(),
                    "Released peer links before rebuilding the local GATT server");
    }
    configure_serial_profile(cfg);
    // Stop Mesh before enabling conventional LE roles. In the opposite
    // direction, start it only after the GATT objects are rebuilt below.
    if (!cfg.enable_mesh) configure_mesh(cfg);

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
                // Publish health at the period selected by the newly applied overlay.
                publish_periodic_status();
            },
            timer_callback_group_);
    }

    {
        std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
        for (auto it = bridge_registry_.exports().begin(); it != bridge_registry_.exports().end();) {
            // A manually created GATT service is just as connection-oriented as
            // a declarative one. Drop it when a connectionless/Mesh overlay
            // takes ownership; otherwise retain manual bridges across ordinary
            // GATT overlay reloads.
            if (exclusive_radio_mode || it->second.auto_managed) {
                export_bridges_->destroy_export_bridge(it->second);
                it = bridge_registry_.exports().erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = bridge_registry_.imports().begin(); it != bridge_registry_.imports().end();) {
            if (exclusive_radio_mode || it->second.auto_managed) {
                import_bridges_->destroy_import_bridge(it->second);
                it = bridge_registry_.imports().erase(it);
            } else {
                ++it;
            }
        }

        for (const auto& shared_topic : cfg.shared_topics) {
            if (shared_topic.transport != "gatt") continue;
            if (shared_topic.mode == "export" || shared_topic.mode == "both") {
                bridge::TopicExportBridgeState state;
                state.topic_name = shared_topic.export_topic;
                state.message_type = shared_topic.message_type;
                state.bridge_name = shared_topic.bridge_name;
                state.bridge_key = shared_topic.bridge_key;
                state.bridge_uuid = util::named_characteristic_uuid(
                    gatt::bridge_characteristic_name_for_service(shared_topic.bridge_name));
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

    // Connectionless bridges are reconfigured independently of the GATT
    // registry. Their subscriptions are ready before scanning/advertising is
    // resumed, preserving the first frame received by a hot overlay.
    if (transport_bridges_) {
        transport_bridges_->configure(
            cfg.shared_topics, cfg.node_topics_prefix, cfg.peer_whitelist);
        if (client_ && cfg.advertise_mode == "broadcast") {
            // BlueZ survives this ROS process and may retain the last packet
            // for several random-address Device1 objects. Seed the new bridge
            // generation with those values before scanning begins.
            for (const auto& device : client_->get_devices()) {
                const auto payload = advertisement_user_payload(
                    device, bluez::kDefaultAdvertisementExtraDataType);
                if (payload.empty()) continue;
                transport_bridges_->remember_advertisement(
                    util::device_hostname_guess(
                        device, cfg.auto_connect_pattern, cfg.peer_whitelist),
                    device.mac, payload);
            }
        }
    }

    netplan_->set_config_file(cfg.wifi_netplan_config_path);
    netplan_->set_allowed_networks(cfg.allowed_wifi_networks);
    const bool defer_broadcast_scan = cfg.advertise_mode == "broadcast" &&
        cfg.enable_server;
    if (cfg.enable_scan && !gatt_link_handoff && !defer_broadcast_scan) {
        const bool discoverable_while_scanning =
            cfg.advertise_mode != "broadcast" &&
            cfg.advertise_discoverable.value_or(
                cfg.enable_server || cfg.enable_serial_port_profile);
        client_->start_scan(cfg.scan_mode, discoverable_while_scanning);
    } else {
        client_->stop_scan();
    }

    if (peers_ && client_) {
        for (const auto& device : client_->get_devices()) {
            {
                std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
                const bool preserve_ready_runtime =
                    has_ready_peer_time_bridge(device.mac) ||
                    should_preserve_ready_bridge_during_expected_services_rediscovery(device);
                const bool preserve_active_bridge_runtime =
                    should_preserve_peer_bridge_runtime_during_expected_services_rediscovery(device);
                peers_->sync_device(device,
                                    active_config_,
                                    util::device_hostname_guess(
                                        device, active_config_.auto_connect_pattern,
                                        active_config_.peer_whitelist),
                                    preserve_ready_runtime,
                                    preserve_active_bridge_runtime);
            }
            refresh_import_bridges_for_device(device);
        }

    }

    if (time_service_timer_) {
        time_service_timer_->cancel();
        time_service_timer_.reset();
    }
    if (wifi_service_timer_) {
        wifi_service_timer_->cancel();
        wifi_service_timer_.reset();
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
    local_server_rebuild_in_progress_.store(true);
    try {
        // Adapter Connectable/Discoverable writes are rejected as Busy while
        // an LE advertisement is registered. Remove only the old advertisement
        // first, apply the new radio policy, and let the full rebuild below
        // register the replacement with its new peripheral/broadcast type.
        {
            std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
            if (advertisement_ && !adapter_path_.empty()) {
                try {
                    advertisement_->unregister_advertisement(adapter_path_);
                } catch (const std::exception& error) {
                    RCLCPP_WARN(get_logger(),
                                "Could not unregister the previous advertisement before adapter policy update: %s",
                                error.what());
                }
                advertisement_.reset();
            }
        }
        apply_adapter_state(cfg);
        rebuild_server_objects();
    } catch (const std::exception& error) {
        local_server_rebuild_in_progress_.store(false);
        // The caller reports configuration failure and can retry or reset;
        // the retained inactive state reflects the missing radio objects.
        RCLCPP_ERROR(get_logger(),
                     "Local GATT/advertisement configuration failed: %s",
                     error.what());
        throw;
    } catch (...) {
        local_server_rebuild_in_progress_.store(false);
        RCLCPP_ERROR(get_logger(),
                     "Local GATT/advertisement configuration failed: unknown error");
        throw;
    }
    local_server_rebuild_in_progress_.store(false);
    {
        std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
        local_server_rebuild_monotonic_ = peers_ ? peers_->now_monotonic()
                                                 : std::chrono::duration<double>(
                                                       std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    local_gatt_layout_signature_ = new_gatt_layout_signature;

    // Register the replacement GATT application before resuming discovery and
    // outbound connections, so reconnecting peers receive the complete local
    // database and current ATT handles.
    if (gatt_link_handoff && cfg.enable_scan) {
        const bool discoverable_while_scanning =
            cfg.advertise_mode != "broadcast" &&
            cfg.advertise_discoverable.value_or(
                cfg.enable_server || cfg.enable_serial_port_profile);
        client_->start_scan(cfg.scan_mode, discoverable_while_scanning);
    }

    if (cfg.enable_mesh) configure_mesh(cfg);

    refresh_advertisement_topic_subscription();

    if (!can_run_callbacks()) {
        return;
    }
    if (cfg.enable_time_service && cfg.time_update_period > 0.0) {
        time_service_timer_ = create_grouped_wall_timer(
            *this,
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(cfg.time_update_period)),
            [this]() {
                // Refresh the time characteristic only while the service remains enabled.
                if (time_service_) {
                    time_service_->update();
                }
            },
            timer_callback_group_);
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
                // Refresh the Wi-Fi characteristic from the currently applied network configuration.
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
    // Remove the previous advertisement and GATT tree, then export the complete
    // service layout selected by the active overlay.
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
        if (active_config_.gatt_profile_uuids.empty()) {
            return;
        }
    }

    gatt_app_ = std::make_unique<gatt::GattApplication>(*server_dbus_, "/org/bluez/app", get_logger());
    if (!active_config_.gatt_profile_uuids.empty()) {
        auto profile = std::make_shared<gatt::GattProfile>(
            *server_dbus_,
            "/org/bluez/app/profile0",
            active_config_.gatt_profile_uuids);
        profile->set_release_callback([this]() {
            // Record that BlueZ released the optional local client profile.
            RCLCPP_INFO(get_logger(), "BlueZ released the local GATT client profile");
        });
        gatt_app_->add_profile(std::move(profile));
    }

    int service_index = 0;
    if (active_config_.enable_server && active_config_.enable_wifi_service) {
        wifi_service_ = std::make_unique<gatt::services::WifiService>(
            *server_dbus_, "/org/bluez/app", service_index++,
            [this]() {
                // Read the current network name, substituting a stable value when disconnected.
                auto ssid = netplan_->get_current_ssid();
                return ssid.empty() ? std::string("unknown") : ssid;
            },
            [this](const std::string& ssid) {
                // Treat the written Wi-Fi name as a request to save and activate that network.
                return netplan_->set_current_network(ssid);
            },
            [this](const std::string& password) {
                // Trim the peer-supplied password before asking Netplan to change the network.
                std::string trimmed = password;
                while (!trimmed.empty() &&
                       (trimmed.back() == ' ' || trimmed.back() == '\n' ||
                        trimmed.back() == '\r' || trimmed.back() == '\t')) {
                    trimmed.pop_back();
                }
                if (trimmed.empty()) {
                    return std::make_pair(false, std::string("password update ignored because the written value is empty"));
                }

                const auto ssid = netplan_->get_current_ssid();
                if (ssid.empty() || ssid == "unknown") {
                    return std::make_pair(false, std::string("password update failed because no current SSID is available"));
                }
                return netplan_->set_current_network(ssid, trimmed);
            },
            [this]() {
                // Read back the configured password for the writable GATT field.
                return netplan_->get_configured_password();
            });
        gatt_app_->add_service(wifi_service_->service());
    } else {
        wifi_service_.reset();
    }

    if (active_config_.enable_server && active_config_.enable_time_service) {
        time_service_ = std::make_unique<gatt::services::TimeService>(
            *server_dbus_, "/org/bluez/app", service_index++,
            [this](const std::vector<uint8_t>& payload,
                   const std::string& device_path,
                   uint64_t received_time_ns) {
                // Associate the peer time writeback with its receive timestamp and publish it.
                handle_time_writeback(payload, device_path, received_time_ns);
            });
        // Wire server-side notify observability for the time characteristic.
        for (const auto& chrc : time_service_->service()->characteristics()) {
            chrc->set_force_emit_value(true);
            chrc->set_notify_callback([this, uuid = chrc->uuid()](bool enabled) {
                // Track whether any client needs periodic local-time notifications.
                RCLCPP_INFO(get_logger(), "[server] time characteristic %s: client %s notifications",
                            uuid.c_str(), enabled ? "started" : "stopped");
                note_local_time_notify_state(enabled);
            });
        }
        gatt_app_->add_service(time_service_->service());
    } else {
        time_service_.reset();
    }

    if (active_config_.enable_server) {
        export_bridges_->rebuild_gatt_services(*gatt_app_, *server_dbus_, "/org/bluez/app");
    }
    // BlueZ rejects an empty ObjectManager tree with
    // org.bluez.Error.Failed ("No object received"). Advertisement-only
    // overlays deliberately keep enable_server set so this process owns the
    // LE advertisement, while disabling every GATT service and profile. In
    // that valid configuration there is no GATT application to register.
    if (!gatt_app_->services().empty() || !gatt_app_->profiles().empty()) {
        gatt_app_->register_application(adapter_path_);
    } else {
        RCLCPP_INFO(get_logger(),
                    "No local GATT objects are configured; skipping empty application registration");
        gatt_app_.reset();
    }

    if (active_config_.enable_server) {
        RCLCPP_INFO(get_logger(), "[node] GATT server registered, setting up advertisement");
        advertisement_ = std::make_unique<gatt::Advertisement>(
            *server_dbus_, "/org/bluez/advertisement0", active_config_.advertise_mode);
        refresh_advertisement_registration();
    }
}

void ServiceNode::handle_configure_notification_bridge(const std::shared_ptr<mrs_uav_bluetooth::srv::ConfigureNotificationBridge::Request> request,
                                                         std::shared_ptr<mrs_uav_bluetooth::srv::ConfigureNotificationBridge::Response> response) {
    // Serialize manual bridge rebuilds with overlay transactions and release
    // service state around D-Bus server registration changes.
    std::lock_guard<std::mutex> config_lock(config_apply_mutex_);
    AtomicFlagGuard apply_guard(config_apply_in_progress_);
    std::unique_lock<std::recursive_mutex> state_lock(state_mutex_);
    try {
        if (request->enable &&
            (active_config_.enable_mesh ||
             active_config_.advertise_mode == "broadcast")) {
            throw std::runtime_error(
                "GATT bridges are unavailable while Mesh or broadcast "
                "advertisement mode owns the radio");
        }
        const auto direction = normalize_direction(request->direction);
        const auto message_type = util::lower_trim_copy(request->message_type);
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
                state_lock.unlock();
                rebuild_server_objects();
                state_lock.lock();
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
            state_lock.unlock();
            rebuild_server_objects();
            state_lock.lock();
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
            response->resolved_member_paths.push_back(spec.target + ":" + spec.value_type);
        }
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

}  // namespace mrs_uav_bluetooth::app
