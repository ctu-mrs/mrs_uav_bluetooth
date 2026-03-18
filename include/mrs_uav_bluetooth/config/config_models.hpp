// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
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
    std::string export_topic;
    std::string import_topic_suffix;
    std::string message_type;        // e.g. "nav_msgs/msg/Odometry"
    double rate_hz{0.0};
    std::string transport_endpoint;  // "characteristic" or "descriptor"
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
    bool auto_accept_pairing{true};
    bool auto_trust{true};
    bool enable_time_service{true};
    bool enable_wifi_service{true};
    bool auto_connect_enable{false};
    std::vector<std::string> auto_connect_whitelist;
    std::string auto_connect_pattern = "^uav[0-9]{2}$";
    double peer_connection_timeout{30.0};
    std::vector<std::string> allowed_wifi_networks;
    double status_report_period{1.0};
    bool log_topic_enable{false};
    bool expire_connections_with_overlay{true};
    std::string overlay_keepalive_topic_suffix = "overlay_keepalive";
    std::string verbose_log_file;

    // Static / rarely changing.
    std::string advertise_mode = "peripheral";
    std::string pairing_agent = "NoInputNoOutput";
    bool enable_server{true};
    bool enable_scan{true};
    std::string scan_mode = "le";
    std::string netplan_config_file = "/etc/netplan/01-netcfg.yaml";
    std::string netplan_scripts_dir = "/etc/ctu-mrs/uav-bluetooth/netplan-scripts";

    std::vector<SharedTopicConfig> shared_topics;
};

}  // namespace mrs_uav_bluetooth::config
