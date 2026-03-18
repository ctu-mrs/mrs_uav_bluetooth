// SPDX-License-Identifier: MIT
#include "mrs_uav_bluetooth/gatt/services/topic_bridge_service.hpp"

#include "mrs_uav_bluetooth/config/shared_topic_config.hpp"

#include <yaml-cpp/yaml.h>

#include <iomanip>
#include <sstream>

namespace mrs_uav_bluetooth::gatt::services {

namespace {

std::vector<uint8_t> to_bytes(const std::string& value) {
    return std::vector<uint8_t>(value.begin(), value.end());
}

std::vector<uint8_t> serialize_member_specs(const std::vector<config::BridgeMemberSpec>& member_specs) {
    YAML::Emitter out;
    out << YAML::Flow << YAML::BeginSeq;
    for (const auto& spec : member_specs) {
        out << YAML::Flow << YAML::BeginMap;
        out << YAML::Key << "path" << YAML::Value << spec.path;
        out << YAML::Key << "type" << YAML::Value << spec.value_type;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    return to_bytes(std::string(out.c_str()));
}

std::string format_rate_hz(double rate_hz) {
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
    auto service_uuid = named_service_uuid("bridge:" + bridge_name);
    auto characteristic_uuid = named_characteristic_uuid(bridge_name);
    auto data_uuid = named_descriptor_uuid(bridge_name + "/data");
    auto topic_uuid = named_descriptor_uuid(bridge_name + "/topic");
    auto type_uuid = named_descriptor_uuid(bridge_name + "/type");
    auto format_uuid = named_descriptor_uuid(bridge_name + "/format");
    auto members_uuid = named_descriptor_uuid(bridge_name + "/members");
    auto rate_uuid = named_descriptor_uuid(bridge_name + "/rate_hz");
    auto key_uuid = named_descriptor_uuid(bridge_name + "/key");

    std::string svc_path = base_path + "/service" + std::to_string(index);
    std::string chrc_path = svc_path + "/char0";

    service_ = std::make_shared<GattService>(dbus, svc_path, service_uuid, true);
    characteristic_ = std::make_shared<GattCharacteristic>(
        dbus, chrc_path, characteristic_uuid,
        std::vector<std::string>{"read", "notify"}, *service_);

    auto data_path = chrc_path + "/desc0";
    data_descriptor_ = std::make_shared<GattDescriptor>(
        dbus, data_path, data_uuid,
        std::vector<std::string>{"read"}, *characteristic_);
    characteristic_->add_descriptor(data_descriptor_);

    auto add_static_desc = [&](int desc_index,
                               const std::string& uuid,
                               const std::vector<uint8_t>& value) {
        auto desc = std::make_shared<GattDescriptor>(
            dbus,
            chrc_path + "/desc" + std::to_string(desc_index),
            uuid,
            std::vector<std::string>{"read"},
            *characteristic_);
        desc->set_value(value);
        characteristic_->add_descriptor(desc);
    };

    add_static_desc(1, topic_uuid, to_bytes(topic_name));
    add_static_desc(2, type_uuid, to_bytes(message_type));
    add_static_desc(3, format_uuid, to_bytes(payload_format));
    add_static_desc(4, members_uuid, serialize_member_specs(member_specs));
    add_static_desc(5, rate_uuid, to_bytes(format_rate_hz(rate_hz)));
    add_static_desc(6, key_uuid, to_bytes(bridge_key));

    service_->add_characteristic(characteristic_);
}

std::string TopicBridgeService::characteristic_uuid() const {
    return characteristic_->uuid();
}

std::string TopicBridgeService::characteristic_path() const {
    return characteristic_->path();
}

std::string TopicBridgeService::data_descriptor_path() const {
    return data_descriptor_->path();
}

std::string TopicBridgeService::transport_path(const std::string& endpoint) const {
    if (endpoint == "descriptor") {
        return data_descriptor_path();
    }
    return characteristic_path();
}

void TopicBridgeService::publish(const std::vector<uint8_t>& payload) {
    payload_ = payload;
    data_descriptor_->set_value(payload_, true);
    characteristic_->publish(payload_);
}

}  // namespace mrs_uav_bluetooth::gatt::services
