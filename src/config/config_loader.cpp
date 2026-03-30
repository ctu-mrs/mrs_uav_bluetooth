// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/config/config_loader.hpp"
#include "mrs_uav_bluetooth/util/hostname_utils.hpp"
#include "mrs_uav_bluetooth/util/topic_utils.hpp"
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace mrs_uav_bluetooth::config {

namespace {

/// Replace all occurrences of {hostname} in a string.
std::string expand_hostname(const std::string& value, const std::string& hostname) {
    std::string result = value;
    const std::string placeholder = "{hostname}";
    std::string sanitized = util::sanitize_topic_suffix(hostname);
    size_t pos = 0;
    while ((pos = result.find(placeholder, pos)) != std::string::npos) {
        result.replace(pos, placeholder.size(), sanitized);
        pos += sanitized.size();
    }
    return result;
}

std::string bridge_topic_path(const std::string& canonical_topic) {
    const auto normalized_canonical = util::normalize_ros_topic(canonical_topic);
    if (normalized_canonical == "/") {
        return std::string{"/"};
    }
    return normalized_canonical;
}

/// Strip UAV hostname prefix from a canonical topic.
std::string canonical_shared_topic(const std::string& export_topic,
                                   const std::string& pattern) {
    std::string normalized = util::normalize_ros_topic(export_topic);
    // Split into segments.
    std::vector<std::string> segments;
    std::istringstream iss(normalized);
    std::string segment;
    while (std::getline(iss, segment, '/')) {
        if (!segment.empty()) segments.push_back(segment);
    }
    if (segments.empty()) return normalized;
    if (util::is_uav_hostname(segments[0], pattern)) {
        if (segments.size() == 1) return "/";
        std::string result = "/";
        for (size_t i = 1; i < segments.size(); ++i) {
            result += segments[i];
            if (i + 1 < segments.size()) result += "/";
        }
        return result;
    }
    return normalized;
}

/// Map of supported compact value types to their struct sizes.
const std::map<std::string, size_t>& struct_format_sizes() {
    static const std::map<std::string, size_t> sizes = {
        {"bool", 1}, {"int8", 1}, {"uint8", 1},
        {"int16", 2}, {"uint16", 2},
        {"int32", 4}, {"uint32", 4},
        {"int64", 8}, {"uint64", 8},
        {"float32", 4}, {"float64", 8},
        {"time_ns", 8},
    };
    return sizes;
}

/// ROS type name aliases.
const std::map<std::string, std::string>& ros_type_aliases() {
    static const std::map<std::string, std::string> aliases = {
        {"boolean", "bool"},
        {"byte", "int8"},
        {"octet", "uint8"},
        {"char", "uint8"},
        {"float", "float32"},
        {"double", "float64"},
    };
    return aliases;
}

std::string normalize_value_type(const std::string& vt) {
    auto it = ros_type_aliases().find(vt);
    std::string candidate = (it != ros_type_aliases().end()) ? it->second : vt;
    if (struct_format_sizes().count(candidate) == 0) {
        throw std::runtime_error("Unsupported bridge member type: " + vt);
    }
    return candidate;
}

BridgeMemberSpec parse_member_spec(const YAML::Node& item) {
    BridgeMemberSpec spec;
    if (item.IsScalar()) {
        spec.path = item.as<std::string>();
        spec.value_type = "float64";
    } else if (item.IsMap()) {
        spec.path = item["path"].as<std::string>("");
        std::string raw_type = item["type"].as<std::string>("");
        std::transform(raw_type.begin(), raw_type.end(), raw_type.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        spec.value_type = normalize_value_type(raw_type);
    } else {
        throw std::runtime_error("Unsupported member specification format");
    }
    if (spec.path.empty()) {
        throw std::runtime_error("Member path must not be empty");
    }
    return spec;
}

}  // namespace

YAML::Node load_yaml_file(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open config file: " + path);
    }
    YAML::Node doc = YAML::Load(ifs);
    if (!doc.IsMap() && !doc.IsNull()) {
        throw std::runtime_error("Config " + path + " must contain a YAML mapping at the top level");
    }
    return doc;
}

YAML::Node deep_merge(const YAML::Node& base, const YAML::Node& override_node) {
    if (!base.IsDefined() || base.IsNull()) return YAML::Clone(override_node);
    if (!override_node.IsDefined() || override_node.IsNull()) return YAML::Clone(base);
    if (base.IsMap() && override_node.IsMap()) {
        YAML::Node merged = YAML::Clone(base);
        for (auto it = override_node.begin(); it != override_node.end(); ++it) {
            std::string key = it->first.as<std::string>();
            if (merged[key]) {
                merged[key] = deep_merge(merged[key], it->second);
            } else {
                merged[key] = YAML::Clone(it->second);
            }
        }
        return merged;
    }
    // For sequences and scalars, override wins.
    return YAML::Clone(override_node);
}

NodeConfig parse_node_config(const YAML::Node& doc,
                             const std::string& hostname,
                             const std::string& auto_connect_pattern) {
    NodeConfig cfg;
    if (!doc.IsDefined() || doc.IsNull()) return cfg;

    auto str = [&](const std::string& key, const std::string& def) -> std::string {
        return doc[key] ? doc[key].as<std::string>(def) : def;
    };
    auto dbl = [&](const std::string& key, double def) -> double {
        return doc[key] ? doc[key].as<double>(def) : def;
    };
    auto b = [&](const std::string& key, bool def) -> bool {
        return doc[key] ? doc[key].as<bool>(def) : def;
    };
    auto u32 = [&](const std::string& key, uint32_t def) -> uint32_t {
        return doc[key] ? doc[key].as<uint32_t>(def) : def;
    };
    auto str_list = [&](const std::string& key) -> std::vector<std::string> {
        std::vector<std::string> result;
        if (doc[key] && doc[key].IsSequence()) {
            for (const auto& item : doc[key]) {
                result.push_back(item.as<std::string>(""));
            }
        }
        return result;
    };

    cfg.node_topics_prefix = expand_hostname(str("node_topics_prefix", cfg.node_topics_prefix), hostname);
    cfg.scan_publish_period = dbl("scan_publish_period", cfg.scan_publish_period);
    cfg.time_update_period = dbl("time_update_period", cfg.time_update_period);
    cfg.wifi_refresh_period = dbl("wifi_refresh_period", cfg.wifi_refresh_period);
    cfg.auto_connect_period = dbl("auto_connect_period", cfg.auto_connect_period);
    cfg.discoverable_timeout = u32("discoverable_timeout", cfg.discoverable_timeout);
    cfg.auto_accept_pairing = b("auto_accept_pairing", cfg.auto_accept_pairing);
    cfg.auto_trust = b("auto_trust", cfg.auto_trust);
    cfg.enable_time_service = b("enable_time_service", cfg.enable_time_service);
    cfg.enable_wifi_service = b("enable_wifi_service", cfg.enable_wifi_service);
    cfg.auto_connect_enable = b("auto_connect_enable", cfg.auto_connect_enable);
    cfg.auto_connect_whitelist = str_list("auto_connect_whitelist");
    cfg.auto_connect_pattern = str("auto_connect_pattern", cfg.auto_connect_pattern);
    cfg.peer_connection_timeout = dbl("peer_connection_timeout", cfg.peer_connection_timeout);
    cfg.allowed_wifi_networks = str_list("allowed_wifi_networks");
    cfg.status_report_period = dbl("status_report_period", cfg.status_report_period);
    cfg.log_topic_enable = b("log_topic_enable", cfg.log_topic_enable);
    cfg.expire_connections_with_overlay = b("expire_connections_with_overlay", cfg.expire_connections_with_overlay);
    cfg.overlay_keepalive_topic_suffix = str("overlay_keepalive_topic_suffix", cfg.overlay_keepalive_topic_suffix);
    cfg.verbose_log_file = str("verbose_log_file", cfg.verbose_log_file);
    cfg.advertise_mode = str("advertise_mode", cfg.advertise_mode);
    cfg.pairing_agent = str("pairing_agent", cfg.pairing_agent);
    cfg.enable_server = b("enable_server", cfg.enable_server);
    cfg.enable_scan = b("enable_scan", cfg.enable_scan);
    cfg.scan_mode = str("scan_mode", cfg.scan_mode);

    // Use the config's auto_connect_pattern for canonical topic derivation.
    std::string pattern = cfg.auto_connect_pattern.empty()
                          ? auto_connect_pattern : cfg.auto_connect_pattern;

    // Parse shared_topics.
    if (doc["shared_topics"] && doc["shared_topics"].IsSequence()) {
        int index = 0;
        std::map<std::string, bool> seen_keys;
        for (const auto& raw : doc["shared_topics"]) {
            if (!raw.IsMap()) {
                throw std::runtime_error("shared_topics[" + std::to_string(index) + "] must be a mapping");
            }

            SharedTopicConfig stc;
            std::string mode = raw["mode"] ? raw["mode"].as<std::string>("both") : "both";
            std::transform(mode.begin(), mode.end(), mode.begin(),
                           [](unsigned char c) { return std::tolower(c); });
            if (mode.empty()) mode = "both";
            if (mode != "export" && mode != "import" && mode != "both") {
                throw std::runtime_error("shared_topics[" + std::to_string(index) + "].mode must be export, import, or both");
            }
            stc.mode = mode;

            std::string export_topic_raw = raw["export_topic"]
                ? raw["export_topic"].as<std::string>("") : "";
            stc.export_topic = util::normalize_ros_topic(
                expand_hostname(export_topic_raw, hostname));
            if (stc.export_topic == "/") {
                throw std::runtime_error("shared_topics[" + std::to_string(index) + "] requires export_topic");
            }

            std::string canonical = canonical_shared_topic(stc.export_topic, pattern);
            std::string key_source = raw["key"]
                ? raw["key"].as<std::string>(canonical) : canonical;
            if (key_source.empty()) {
                throw std::runtime_error("shared_topics[" + std::to_string(index) + "] produced an empty bridge key");
            }
            const auto sanitized_hostname = util::sanitize_topic_suffix(hostname);
            stc.bridge_topic_path = bridge_topic_path(canonical);
            stc.bridge_name = "bridge:/" + sanitized_hostname + stc.bridge_topic_path;
            {
                std::string full_uuid = util::uuid_from_name(stc.bridge_name);
                std::string hex;
                for (char c : full_uuid) {
                    if (c != '-') hex.push_back(c);
                }
                stc.bridge_key = hex;
            }

            stc.message_type = raw["message_type"]
                ? raw["message_type"].as<std::string>("") : "";
            if (stc.message_type.empty()) {
                throw std::runtime_error("shared_topics[" + std::to_string(index) + "] requires message_type");
            }

            // Parse members.
            if (raw["members"] && raw["members"].IsSequence()) {
                for (const auto& m : raw["members"]) {
                    stc.member_specs.push_back(parse_member_spec(m));
                }
            }
            if (stc.member_specs.empty()) {
                throw std::runtime_error("shared_topics[" + std::to_string(index) + "] requires at least one compact member definition");
            }
            stc.payload_format = "struct";

            std::string import_suffix = raw["import_topic_suffix"]
                ? raw["import_topic_suffix"].as<std::string>(canonical) : canonical;
            stc.import_topic_suffix = util::normalize_ros_topic(import_suffix);
            // Strip leading slash for suffix use.
            if (!stc.import_topic_suffix.empty() && stc.import_topic_suffix[0] == '/') {
                stc.import_topic_suffix = stc.import_topic_suffix.substr(1);
            }

            stc.rate_hz = std::max(0.0,
                raw["rate_hz"] ? raw["rate_hz"].as<double>(0.0) : 0.0);
            stc.name = raw["name"]
                ? raw["name"].as<std::string>(canonical) : canonical;
            if (stc.name.empty()) stc.name = canonical;

            if (seen_keys.count(stc.bridge_key)) {
                throw std::runtime_error("shared_topics[" + std::to_string(index) +
                                         "] duplicates bridge key for " + canonical);
            }
            seen_keys[stc.bridge_key] = true;
            cfg.shared_topics.push_back(std::move(stc));
            ++index;
        }
    }

    return cfg;
}

NodeConfig load_effective_config(const std::string& default_path,
                                 const std::string& overlay_path,
                                 const std::string& hostname) {
    YAML::Node base = load_yaml_file(default_path);
    if (!overlay_path.empty()) {
        YAML::Node overlay = load_yaml_file(overlay_path);
        base = deep_merge(base, overlay);
    }
    return parse_node_config(base, hostname);
}

}  // namespace mrs_uav_bluetooth::config
