// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/config/config_loader.hpp
/// \brief Declares the config loader component of the YAML configuration layer.

#pragma once

#include "mrs_uav_bluetooth/config/config_models.hpp"

#include <string>

#include <yaml-cpp/yaml.h>

namespace mrs_uav_bluetooth::config {

/// Load a YAML file and return YAML::Node.  Throws on I/O or parse error.
/// \param path YAML configuration file to read.
/// \return Parsed YAML document.
YAML::Node load_yaml_file(const std::string& path);

/// Deep-merge two YAML nodes (override wins for scalars/sequences;
/// maps are recursively merged).  Returns a new node.
/// \param base default YAML node onto which overlay values are merged.
/// \param override_node overlay YAML values merged over the base configuration.
/// \return Merged YAML tree with overlay values taking precedence.
YAML::Node deep_merge(const YAML::Node& base, const YAML::Node& override_node);

/// Parse a NodeConfig from a YAML::Node.
/// \param hostname  The local hostname, used to expand {hostname} placeholders.
/// \param auto_connect_pattern  The pattern used to strip UAV hostname prefixes.
/// \param doc YAML document from which the effective configuration is loaded.
/// \return Validated node configuration.
NodeConfig parse_node_config(const YAML::Node& doc,
                             const std::string& hostname,
                             const std::string& auto_connect_pattern = "^uav[0-9]{1,5}$");

/// Load the effective config from a default YAML file, optionally overlaid
/// with an overlay YAML file.
/// \param default_path Base YAML configuration loaded before the overlay.
/// \param overlay_path Optional YAML overlay merged over the default file.
/// \param hostname UAV hostname used to identify the node.
/// \return Validated default configuration with the selected overlay applied.
NodeConfig load_effective_config(const std::string& default_path,
                                 const std::string& overlay_path,
                                 const std::string& hostname);

}  // namespace mrs_uav_bluetooth::config
