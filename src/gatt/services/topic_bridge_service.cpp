// SPDX-License-Identifier: BSD-3-Clause
/// \file src/gatt/services/topic_bridge_service.cpp
/// \brief Implements the topic bridge service component of the Bluetooth Low Energy GATT layer.

#include "mrs_uav_bluetooth/gatt/services/topic_bridge_service.hpp"

#include "mrs_uav_bluetooth/config/shared_topic_config.hpp"

#include <yaml-cpp/yaml.h>

#include <iomanip>
#include <sstream>
#include <string_view>

namespace mrs_uav_bluetooth::gatt::services {

namespace {

/// \brief Copy the serialized ROS buffer into an owned byte vector.
/// \param value Descriptor text to expose as a byte value.
/// \return Converted bytes.
std::vector<uint8_t> to_bytes(const std::string& value) {
    // Copy the serialized ROS buffer into an owned byte vector.
    return std::vector<uint8_t>(value.begin(), value.end());
}

/// \brief Build the deterministic characteristic name beneath a bridge service.
/// \param bridge_name Stable bridge service name from which the child characteristic name is derived.
/// \return Deterministic characteristic name paired with the bridge service.
std::string characteristic_name_for_bridge(const std::string& bridge_name) {
    // Build the deterministic characteristic name beneath a bridge service.
    constexpr std::string_view prefix{"bridge:"};
    if (bridge_name.rfind(prefix.data(), 0) == 0) {
        return bridge_name.substr(prefix.size());
    }
    return bridge_name + "/value";
}

/// \brief Encode bridge field paths and scalar types for remote descriptor discovery.
/// \param member_specs Ordered field paths and scalar types exposed in the descriptor.
/// \return Newline-delimited field path and scalar-type metadata bytes.
std::vector<uint8_t> serialize_member_specs(const std::vector<config::BridgeMemberSpec>& member_specs) {
    // Serialize member specs.
    YAML::Emitter out;
    out << YAML::Flow << YAML::BeginSeq;
    for (const auto& spec : member_specs) {
        out << YAML::Flow << YAML::BeginMap;
        out << YAML::Key << "target" << YAML::Value << spec.target;
        out << YAML::Key << "type" << YAML::Value << spec.value_type;
        if (!spec.expression.empty()) out << YAML::Key << "expression" << YAML::Value << spec.expression;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    return to_bytes(std::string(out.c_str()));
}

/// \brief Render the configured publication rate without locale-dependent formatting.
/// \param rate_hz requested rate in hertz.
/// \return Fixed-precision publication-rate text.
std::string format_rate_hz(double rate_hz) {
    // Six decimals keep descriptor text stable across locales and restarts.
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6) << rate_hz;
    return stream.str();
}

}  // namespace

using namespace mrs_uav_bluetooth::bluez;
using namespace mrs_uav_bluetooth::util;

TopicBridgeService::TopicBridgeService(
    DbusConnection& dbus,
    const std::string& base_path,
    int index,
    const std::string& topic_name,
    const std::string& message_type,
    const std::string& bridge_name,
    const std::string& bridge_key,
    const std::vector<config::BridgeMemberSpec>& member_specs,
    double rate_hz,
    const std::string& payload_format) {
    // Build one value characteristic plus metadata descriptors for this ROS topic bridge.
    const auto characteristic_name = characteristic_name_for_bridge(bridge_name);
    auto service_uuid = named_service_uuid(bridge_name);
    auto characteristic_uuid = named_characteristic_uuid(characteristic_name);
    auto type_uuid = named_descriptor_uuid(characteristic_name + "/type");
    auto format_uuid = named_descriptor_uuid(characteristic_name + "/format");
    auto members_uuid = named_descriptor_uuid(characteristic_name + "/members");
    auto rate_uuid = named_descriptor_uuid(characteristic_name + "/rate_hz");

    std::string svc_path = base_path + "/service" + std::to_string(index);
    std::string chrc_path = svc_path + "/char0";

    service_ = std::make_shared<GattService>(dbus, svc_path, service_uuid, true);
    characteristic_ = std::make_shared<GattCharacteristic>(
        dbus, chrc_path, characteristic_uuid,
        std::vector<std::string>{"read", "notify"}, *service_);

    auto add_static_desc = [&](int desc_index,
                               const std::string& uuid,
                               const std::vector<uint8_t>& value) {
        // Add one immutable metadata descriptor below the bridge characteristic.
        auto desc = std::make_shared<GattDescriptor>(
            dbus,
            chrc_path + "/desc" + std::to_string(desc_index),
            uuid,
            std::vector<std::string>{"read"},
            *characteristic_);
        desc->set_value(value);
        characteristic_->add_descriptor(desc);
    };

    add_static_desc(0, type_uuid, to_bytes(message_type));
    add_static_desc(1, format_uuid, to_bytes(payload_format));
    add_static_desc(2, members_uuid, serialize_member_specs(member_specs));
    add_static_desc(3, rate_uuid, to_bytes(format_rate_hz(rate_hz)));

    service_->add_characteristic(characteristic_);
}

std::string TopicBridgeService::characteristic_uuid() const {
    // Return the deterministic UUID of this topic bridge value characteristic.
    return characteristic_->uuid();
}

std::string TopicBridgeService::characteristic_path() const {
    // Return the D-Bus path of this exported topic bridge characteristic.
    return characteristic_->path();
}

std::string TopicBridgeService::transport_path() const {
    // Return the active transport object path used for this peer bridge.
    return characteristic_path();
}

void TopicBridgeService::publish(const std::vector<uint8_t>& payload) {
    // The characteristic owns a mutex-protected value; an extra shared
    // scratch vector here races concurrent immediate/rate-limited publishers.
    characteristic_->publish(payload);
}

}  // namespace mrs_uav_bluetooth::gatt::services
