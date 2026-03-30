// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/bridge/export_bridge_manager.hpp"

#include "mrs_uav_bluetooth/bridge/generic_message_bridge.hpp"

namespace mrs_uav_bluetooth::bridge {

std::shared_ptr<GenericMessageBridge> ExportBridgeManager::runtime_for(TopicExportBridgeState& state) {
    if (!state.runtime || state.runtime->message_type() != state.message_type) {
        state.runtime = std::make_shared<GenericMessageBridge>(state.message_type);
    }
    return state.runtime;
}

void ExportBridgeManager::configure_export_bridge(const std::string& bridge_key,
                                                  TopicExportBridgeState& state) {
    auto runtime = runtime_for(state);
    state.subscription = runtime->create_subscription(
        node_,
        state.topic_name,
        [this, bridge_key, runtime](std::shared_ptr<rclcpp::SerializedMessage> message) {
            try {
                if (!registry_) {
                    return;
                }
                auto it = registry_->exports().find(bridge_key);
                if (it == registry_->exports().end()) {
                    return;
                }
                auto& state = it->second;
                auto payload = runtime->encode_payload(*message, state.member_specs, state.payload_format);
                publish_export_payload(bridge_key, payload);
            } catch (const std::exception& e) {
                RCLCPP_WARN(logger_, "Failed to encode export bridge %s: %s",
                            bridge_key.c_str(), e.what());
            }
        },
        10);
}

ExportBridgeManager::ExportBridgeManager(rclcpp::Node& node, rclcpp::Logger logger)
    : node_(node), logger_(logger) {}

void ExportBridgeManager::set_registry(BridgeRegistry* registry) {
    registry_ = registry;
}

void ExportBridgeManager::rebuild_gatt_services(gatt::GattApplication& app,
                                                bluez::DbusConnection& dbus,
                                                const std::string& app_base_path) {
    if (!registry_) {
        return;
    }
    int index = 10;
    for (auto& [key, state] : registry_->exports()) {
        runtime_for(state);
        auto service = std::make_shared<gatt::services::TopicBridgeService>(
            dbus,
            app_base_path,
            index++,
            state.topic_name,
            state.message_type,
            state.bridge_name,
            state.bridge_key,
            state.member_specs,
            state.rate_hz,
            state.payload_format);
        state.bridge_uuid = service->characteristic_uuid();
        state.service = service;
        app.add_service(service->service());
    }
}

void ExportBridgeManager::publish_export_payload(const std::string& bridge_key,
                                                 const std::vector<uint8_t>& payload) {
    if (!registry_) {
        return;
    }
    auto it = registry_->exports().find(bridge_key);
    if (it == registry_->exports().end() || !it->second.service) {
        return;
    }
    if (it->second.rate_hz > 0.0) {
        it->second.pending_payload = payload;
        return;
    }
    it->second.service->publish(payload);
}

void ExportBridgeManager::destroy_export_bridge(TopicExportBridgeState& state) {
    if (state.publish_timer) {
        state.publish_timer->cancel();
        state.publish_timer.reset();
    }
    state.subscription.reset();
    state.runtime.reset();
}

void ExportBridgeManager::configure_export_rate_timer(const std::string& bridge_key,
                                                      TopicExportBridgeState& state) {
    if (state.publish_timer) {
        state.publish_timer->cancel();
        state.publish_timer.reset();
    }
    if (state.rate_hz <= 0.0) {
        return;
    }
    state.publish_timer = node_.create_wall_timer(
        std::chrono::duration<double>(1.0 / state.rate_hz),
        [this, bridge_key]() {
            if (!registry_) {
                return;
            }
            auto it = registry_->exports().find(bridge_key);
            if (it == registry_->exports().end() || !it->second.service || it->second.pending_payload.empty()) {
                return;
            }
            auto payload = it->second.pending_payload;
            it->second.pending_payload.clear();
            it->second.service->publish(payload);
        });
}

}  // namespace mrs_uav_bluetooth::bridge
