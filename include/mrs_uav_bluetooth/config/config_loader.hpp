// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/config/config_models.hpp"

#include <string>

#include <yaml-cpp/yaml.h>

namespace mrs_uav_bluetooth::config {

/// Load a YAML file and return YAML::Node.  Throws on I/O or parse error.
YAML::Node load_yaml_file(const std::string& path);

/// Deep-merge two YAML nodes (override wins for scalars/sequences;
/// maps are recursively merged).  Returns a new node.
YAML::Node deep_merge(const YAML::Node& base, const YAML::Node& override_node);

/// Parse a NodeConfig from a YAML::Node.
/// \param hostname  The local hostname, used to expand {hostname} placeholders.
/// \param auto_connect_pattern  The pattern used to strip UAV hostname prefixes.
NodeConfig parse_node_config(const YAML::Node& doc,
                             const std::string& hostname,
                             const std::string& auto_connect_pattern = "^uav[0-9]{1,2}$");

/// Load the effective config from a default YAML file, optionally overlaid
/// with an overlay YAML file.
NodeConfig load_effective_config(const std::string& default_path,
                                 const std::string& overlay_path,
                                 const std::string& hostname);

}  // namespace mrs_uav_bluetooth::config
