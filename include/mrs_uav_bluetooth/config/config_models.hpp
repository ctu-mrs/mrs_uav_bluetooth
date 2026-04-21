// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::config {

/// Represents a single member in a compact bridge payload.
struct BridgeMemberSpec {
    std::string path;
    std::string value_type;  // bool, int8 .. uint64, float32, float64, time_ns

    bool operator==(const BridgeMemberSpec& o) const {
        return path == o.path && value_type == o.value_type;
    }
};

/// Parsed shared-topic bridge configuration entry.
struct SharedTopicConfig {
    std::string name;
    std::string mode;                // "export", "import", "both"
    std::string bridge_key;          // MD5 hex of the canonical key source
    std::string bridge_name;
    std::string bridge_topic_path;
    std::string export_topic;
    std::string import_topic_suffix;
    std::string message_type;        // e.g. "nav_msgs/msg/Odometry"
    double rate_hz{0.0};
    std::string payload_format;      // "struct" or "ros2"
    std::vector<BridgeMemberSpec> member_specs;
};

/// Top-level node configuration model parsed from YAML.
struct NodeConfig {
    std::string node_topics_prefix = "/{hostname}/ble";
    double scan_publish_period{2.0};
    double time_update_period{1.0};
    double wifi_refresh_period{2.0};
    double auto_connect_period{3.0};
    uint32_t discoverable_timeout{0};
    bool auto_pair{true};
    bool auto_trust{true};
    bool enable_time_service{true};
    bool enable_wifi_service{true};
    bool auto_connect_enable{false};
    std::vector<std::string> auto_connect_whitelist;
    std::string auto_connect_pattern = "^uav[0-9]{1,2}$";
    double peer_connection_timeout{30.0};
    std::string wifi_netplan_config_path = "/etc/netplan/01-netcfg.yaml";
    std::vector<std::string> allowed_wifi_networks;
    double status_report_period{1.0};
    bool log_topic_enable{false};
    bool expire_connections_with_overlay{true};
    std::string overlay_keepalive_topic_suffix = "overlay_keepalive";
    std::string verbose_log_file;

    // Static / rarely changing.
    std::string advertise_mode = "peripheral";
    std::string advertise_local_name;
    std::optional<bool> advertise_discoverable;
    std::vector<std::string> advertise_includes;
    std::vector<std::string> advertise_service_uuids;
    std::vector<std::string> advertise_solicit_uuids;
    std::map<uint16_t, std::vector<uint8_t>> advertise_manufacturer_data;
    std::map<std::string, std::vector<uint8_t>> advertise_service_data;
    std::map<uint8_t, std::vector<uint8_t>> advertise_data;
    std::vector<std::string> advertise_scan_response_service_uuids;
    std::map<uint16_t, std::vector<uint8_t>> advertise_scan_response_manufacturer_data;
    std::vector<std::string> advertise_scan_response_solicit_uuids;
    std::map<std::string, std::vector<uint8_t>> advertise_scan_response_service_data;
    std::map<uint8_t, std::vector<uint8_t>> advertise_scan_response_data;
    std::optional<uint16_t> advertise_appearance;
    std::optional<uint16_t> advertise_duration;
    std::optional<uint16_t> advertise_timeout;
    std::string advertise_secondary_channel;
    std::optional<uint32_t> advertise_min_interval;
    std::optional<uint32_t> advertise_max_interval;
    std::optional<int16_t> advertise_tx_power;
    std::string advertise_extra_data_topic = "/{hostname}/ble/adv";
    std::optional<uint8_t> advertise_extra_data_type = static_cast<uint8_t>(0x26);
    std::string pairing_agent = "NoInputNoOutput";
    bool enable_server{true};
    bool enable_scan{true};
    std::string scan_mode = "le";

    std::vector<SharedTopicConfig> shared_topics;
};

}  // namespace mrs_uav_bluetooth::config
