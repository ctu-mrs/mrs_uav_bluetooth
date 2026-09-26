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
    std::string payload_format;      // "struct", "raw", or "ros2"
    std::vector<BridgeMemberSpec> member_specs;

    // GATT discovers bridges through descriptors. Advertisement and Mesh use
    // channel_id as a small, deployment-defined wire routing key.
    std::string transport{"gatt"};  // "gatt", "advertisement", or "mesh"
    uint16_t channel_id{0};

    // Mesh access-message metadata. These values are ignored by GATT and
    // advertisement bridges and intentionally mirror the MeshSend service.
    uint16_t mesh_destination{0xc000};
    uint16_t mesh_app_key_index{0};
    uint8_t mesh_element_index{0};
    bool mesh_force_segmented{true};
    uint8_t mesh_vendor_opcode{0xc1};
    uint16_t mesh_company_id{0x05f1};
};

/// Top-level node configuration model parsed from YAML.
struct NodeConfig {
    std::string node_topics_prefix = "/{hostname}/bluetooth";
    double scan_publish_period{2.0};
    double time_update_period{1.0};
    double wifi_refresh_period{2.0};
    double auto_connect_period{3.0};
    uint32_t discoverable_timeout{0};
    bool auto_pair{true};
    bool auto_trust{true};
    bool enable_time_service{true};
    bool enable_wifi_service{true};
    bool enable_serial_port_profile{true};
    uint16_t serial_port_channel{22};
    std::string serial_sshd_path = "/usr/sbin/sshd";
    bool enable_mesh{false};
    bool mesh_auto_attach{true};
    bool mesh_auto_join{false};
    bool mesh_auto_create_network{false};
    bool mesh_provisioner{false};
    bool mesh_auto_configure_relay{false};
    uint8_t mesh_default_ttl{127};
    uint8_t mesh_relay_retransmit_count{1};
    uint8_t mesh_relay_retransmit_interval_steps{2};
    bool mesh_swarm_auto_provisioning{false};
    /// Private common fleet credential file; empty selects explicit PB-ADV.
    std::string mesh_fleet_config_path;
    /// Validated credentials, loaded before changing any live radio state.
    std::string mesh_fleet_id;
    std::vector<uint8_t> mesh_fleet_network_key;
    std::vector<uint8_t> mesh_fleet_application_key;
    uint32_t mesh_fleet_iv_index{0};
    uint16_t mesh_fleet_unicast{0};
    /// Logical application swarm; zero is reserved for "not participating".
    uint16_t mesh_swarm_id{1};
    bool mesh_swarm_participating{true};
    /// Persists runtime join/leave changes across service and UAV restarts.
    std::string mesh_swarm_state_path =
        "/var/lib/mrs_uav_bluetooth/swarm-{hostname}.txt";
    /// "ordered" uses peer_whitelist order; a hostname has live priority.
    std::string mesh_provisioner_preference{"ordered"};
    /// The selected peer must remain stable this long before creating a network.
    double mesh_swarm_startup_grace{5.0};
    /// Period of BLE and in-Mesh availability/selection heartbeats.
    double mesh_swarm_heartbeat_period{1.0};
    /// A provisioner peer is unavailable after this heartbeat silence.
    double mesh_swarm_provisioner_timeout{4.0};
    uint16_t mesh_swarm_group_address{0xc000};
    uint16_t mesh_swarm_network_index{0};
    uint16_t mesh_swarm_app_key_index{0};
    std::string mesh_unicast_cursor_path =
        "/var/lib/mrs_uav_bluetooth/mesh-unicast-{hostname}.cursor";
    std::string mesh_device_uuid;
    uint64_t mesh_token{0};
    std::string mesh_token_path = "/var/lib/mrs_uav_bluetooth/mesh-{hostname}.token";
    uint16_t mesh_company_id{0x05f1};
    uint16_t mesh_product_id{0x0001};
    uint16_t mesh_version_id{0x0001};
    uint16_t mesh_crpl{100};
    uint16_t mesh_vendor_model_id{0x0001};
    uint16_t mesh_next_unicast{0x0100};
    std::vector<std::string> mesh_agent_capabilities;
    std::vector<std::string> mesh_agent_oob_info;
    std::vector<uint8_t> mesh_agent_static_oob;
    std::vector<uint8_t> mesh_agent_private_key;
    std::vector<uint8_t> mesh_agent_public_key;
    uint32_t mesh_agent_numeric_oob{0};
    std::string mesh_agent_uri;
    std::vector<std::string> gatt_profile_uuids;
    bool auto_connect_enable{false};
    /// Global admission policy for GATT, advertisement, and Mesh peers.
    /// An empty list permits every peer accepted by the transport itself.
    std::vector<std::string> peer_whitelist;
    std::string auto_connect_pattern = "^uav[0-9]{1,5}$";
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
    std::string advertise_size = "legacy";
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
    std::optional<uint32_t> advertise_min_interval;
    std::optional<uint32_t> advertise_max_interval;
    std::optional<int16_t> advertise_tx_power;
    std::string advertise_extra_data_topic = "/{hostname}/bluetooth/le/advertisement";
    std::string pairing_agent = "NoInputNoOutput";
    bool enable_server{true};
    bool enable_scan{true};
    std::string scan_mode = "le";

    std::vector<SharedTopicConfig> shared_topics;
};

}  // namespace mrs_uav_bluetooth::config
