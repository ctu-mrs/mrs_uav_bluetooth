// SPDX-License-Identifier: MIT
#include "mrs_uav_bluetooth/bridge/import_bridge_manager.hpp"

#include "mrs_uav_bluetooth/bridge/generic_message_bridge.hpp"

namespace mrs_uav_bluetooth::bridge {

std::shared_ptr<GenericMessageBridge> ImportBridgeManager::runtime_for(TopicImportBridgeState& state) {
    if (!state.runtime || state.runtime->message_type() != state.message_type) {
        state.runtime = std::make_shared<GenericMessageBridge>(state.message_type);
    }
    return state.runtime;
}

bool ImportBridgeManager::publish_payload(TopicImportBridgeState& state,
                                          const std::vector<uint8_t>& payload) {
    if (payload.empty()) {
        return false;
    }

    auto publisher = std::dynamic_pointer_cast<rclcpp::GenericPublisher>(state.publisher);
    if (!publisher) {
        return false;
    }

    try {
        auto runtime = runtime_for(state);
        auto serialized = runtime->decode_payload(payload, state.member_specs, state.payload_format);
        publisher->publish(serialized);
        state.last_payload = payload;
        state.pending_payload.clear();
        return true;
    } catch (const std::exception& e) {
        RCLCPP_WARN(logger_, "Failed to decode import bridge %s: %s",
                    state.bridge_key.c_str(), e.what());
        return false;
    }
}

ImportBridgeManager::ImportBridgeManager(rclcpp::Node& node, rclcpp::Logger logger)
    : node_(node), logger_(logger) {}

void ImportBridgeManager::set_registry(BridgeRegistry* registry) {
    registry_ = registry;
}

void ImportBridgeManager::destroy_import_bridge(TopicImportBridgeState& state) {
    if (state.poll_timer) {
        node_.destroy_timer(state.poll_timer);
        state.poll_timer.reset();
    }
    state.publisher.reset();
    state.runtime.reset();
}

void ImportBridgeManager::configure_import_bridge(const std::string& bridge_key,
                                                  TopicImportBridgeState& state,
                                                  bluez::BluezClient& client) {
    auto runtime = runtime_for(state);
    state.publisher = runtime->create_publisher(node_, state.resolved_topic_name, 10);
    configure_import_poll_timer(bridge_key, state, client);
}

void ImportBridgeManager::configure_import_poll_timer(const std::string& bridge_key,
                                                      TopicImportBridgeState& state,
                                                      bluez::BluezClient& client) {
    if (state.poll_timer) {
        node_.destroy_timer(state.poll_timer);
        state.poll_timer.reset();
    }
    if (state.transport_endpoint != "descriptor" && state.rate_hz <= 0.0) {
        return;
    }
    double period_s = state.rate_hz > 0.0 ? 1.0 / state.rate_hz : 1.0;
    state.poll_timer = node_.create_wall_timer(
        std::chrono::duration<double>(period_s),
        [this, &client, bridge_key, &state]() {
            if (state.transport_endpoint == "descriptor") {
                if (state.path.empty()) {
                    return;
                }
                auto payload = client.read_descriptor(state.path);
                if (payload.empty()) {
                    return;
                }
                if (state.rate_hz > 0.0) {
                    state.pending_payload = std::move(payload);
                } else {
                    publish_payload(state, payload);
                }
            }

            if (state.rate_hz <= 0.0 || state.pending_payload.empty()) {
                return;
            }

            auto payload = state.pending_payload;
            if (!publish_payload(state, payload)) {
                RCLCPP_WARN(logger_, "Failed to publish buffered import bridge %s",
                            bridge_key.c_str());
            }
        });
}

bool ImportBridgeManager::buffer_notification_payload(const std::string& mac,
                                                      const std::string& characteristic_path,
                                                      const std::vector<uint8_t>& payload) {
    if (!registry_ || characteristic_path.empty()) {
        return false;
    }

    bool updated = false;
    for (auto& [_, state] : registry_->imports()) {
        if (state.transport_endpoint != "characteristic") {
            continue;
        }
        if (state.path != characteristic_path) {
            continue;
        }
        if (!state.mac.empty() && !mac.empty() && state.mac != mac) {
            continue;
        }
        if (state.rate_hz > 0.0) {
            state.pending_payload = payload;
        } else {
            publish_payload(state, payload);
        }
        updated = true;
    }

    return updated;
}

bool ImportBridgeManager::refresh_import_paths_for_mac(const std::string& mac,
                                                       bluez::BluezClient& client) {
    if (!registry_ || mac.empty()) {
        return false;
    }

    bool changed = false;
    for (auto& [bridge_key, state] : registry_->imports()) {
        if (state.mac != mac || state.bridge_uuid.empty()) {
            continue;
        }

        const auto previous_path = state.path;
        const auto resolved_path = state.transport_endpoint == "descriptor"
            ? client.find_descriptor(mac, state.bridge_uuid)
            : client.find_characteristic(mac, state.bridge_uuid);

        if (resolved_path == previous_path) {
            if (!resolved_path.empty() && state.transport_endpoint == "descriptor" && !state.poll_timer) {
                configure_import_poll_timer(bridge_key, state, client);
                changed = true;
            }
            continue;
        }

        if (!previous_path.empty() && state.transport_endpoint == "characteristic") {
            client.stop_notify(previous_path);
        }

        state.path = resolved_path;
        state.pending_payload.clear();
        state.last_payload.clear();

        if (state.transport_endpoint == "descriptor") {
            configure_import_poll_timer(bridge_key, state, client);
        } else if (state.poll_timer) {
            node_.destroy_timer(state.poll_timer);
            state.poll_timer.reset();
        }

        if (!state.path.empty() && state.transport_endpoint == "characteristic") {
            if (!client.start_notify(state.path)) {
                RCLCPP_WARN(logger_, "Failed to enable notifications for import bridge %s on %s",
                            bridge_key.c_str(), state.path.c_str());
            }
        }

        changed = true;
    }

    return changed;
}

bool ImportBridgeManager::clear_import_paths_for_mac(const std::string& mac,
                                                     bluez::BluezClient& client) {
    if (!registry_ || mac.empty()) {
        return false;
    }

    bool changed = false;
    for (auto& [_, state] : registry_->imports()) {
        if (state.mac != mac) {
            continue;
        }

        if (!state.path.empty() && state.transport_endpoint == "characteristic") {
            client.stop_notify(state.path);
        }
        if (state.poll_timer) {
            node_.destroy_timer(state.poll_timer);
            state.poll_timer.reset();
        }
        if (!state.path.empty() || !state.pending_payload.empty()) {
            changed = true;
        }
        state.path.clear();
        state.pending_payload.clear();
        state.last_payload.clear();
    }

    return changed;
}

}  // namespace mrs_uav_bluetooth::bridge
