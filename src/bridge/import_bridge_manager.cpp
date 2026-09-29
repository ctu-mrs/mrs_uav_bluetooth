// SPDX-License-Identifier: BSD-3-Clause
/// \file src/bridge/import_bridge_manager.cpp
/// \brief Implements the import bridge manager component of the transport-independent ROS message bridge.

#include "mrs_uav_bluetooth/bridge/import_bridge_manager.hpp"

#include "mrs_uav_bluetooth/bridge/generic_message_bridge.hpp"

#include <chrono>

namespace {

struct PublishRateSample {
    double monotonic_now;
    double hz;
};

/// \brief Calculate the latest publication timestamp and observed one-sample frequency.
/// \param last_publish_monotonic steady-clock time of the preceding publication for rate calculation.
/// \return Current monotonic timestamp and frequency since the preceding publication.
PublishRateSample update_publish_rate(double last_publish_monotonic) {
    // Measure frequency from the previous successful publication, leaving the first sample at zero.
    const auto now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (last_publish_monotonic > 0.0 && now > last_publish_monotonic) {
        return {now, 1.0 / (now - last_publish_monotonic)};
    }
    return {now, 0.0};
}

}  // namespace

namespace mrs_uav_bluetooth::bridge {

std::shared_ptr<GenericMessageBridge> ImportBridgeManager::runtime_for(TopicImportBridgeState& state) {
    // Create the type-erased ROS codec lazily and reuse it for this bridge.
    if (!state.runtime || state.runtime->message_type() != state.message_type) {
        state.runtime = std::make_shared<GenericMessageBridge>(state.message_type);
    }
    return state.runtime;
}

bool ImportBridgeManager::publish_payload(TopicImportBridgeState& state,
                                          const std::vector<uint8_t>& payload) {
    // Decode the wire fields into a serialized ROS message and publish it.
    if (payload.empty()) {
        return false;
    }

    auto publisher = std::dynamic_pointer_cast<rclcpp::GenericPublisher>(state.publisher);
    if (!publisher) {
        return false;
    }

    try {
        auto runtime = runtime_for(state);
        auto serialized = runtime->decode_payload(
            payload, state.member_specs, state.payload_format,
            state.decode_assignments);
        publisher->publish(serialized);
        state.last_payload = payload;
        state.pending_payload.clear();
        const auto publish_rate = update_publish_rate(state.last_publish_monotonic);
        state.last_publish_monotonic = publish_rate.monotonic_now;
        state.current_hz = publish_rate.hz;
        return true;
    } catch (const std::exception& e) {
        RCLCPP_WARN(logger_, "Failed to decode import bridge %s: %s",
                    state.bridge_key.c_str(), e.what());
        return false;
    }
}

ImportBridgeManager::ImportBridgeManager(rclcpp::Node& node, rclcpp::Logger logger)
    : node_(node), logger_(logger) {
        // Retain the ROS node and logger used by incoming bridge publishers and timers.
    }

void ImportBridgeManager::set_registry(BridgeRegistry* registry) {
    // Attach the shared registry that owns all configured import bridge state.
    registry_ = registry;
}

void ImportBridgeManager::set_state_mutex(std::recursive_mutex* mutex) {
    // Share the recursive overlay mutex with bridge creation and timer callbacks.
    state_mutex_ = mutex;
}

void ImportBridgeManager::destroy_import_bridge(TopicImportBridgeState& state) {
    // Destroy import bridge.
    if (state.poll_timer) {
        state.poll_timer->cancel();
        state.poll_timer.reset();
    }
    state.publisher.reset();
    state.runtime.reset();
}

void ImportBridgeManager::configure_import_bridge(const std::string& bridge_key,
                                                  TopicImportBridgeState& state,
                                                  bluez::BluezClient& client) {
    // Create the type-erased publisher, then add polling only for a positive rate.
    auto runtime = runtime_for(state);
    state.publisher = runtime->create_publisher(node_, state.resolved_topic_name, 10);
    configure_import_poll_timer(bridge_key, state, client);
}

void ImportBridgeManager::configure_import_poll_timer(const std::string& bridge_key,
                                                      TopicImportBridgeState& state,
                                                      bluez::BluezClient& client) {
    // Replace any old timer and leave notification-driven bridges untimed.
    if (state.poll_timer) {
        state.poll_timer->cancel();
        state.poll_timer.reset();
    }
    if (state.rate_hz <= 0.0) {
        return;
    }
    double period_s = state.rate_hz > 0.0 ? 1.0 / state.rate_hz : 1.0;
    state.poll_timer = node_.create_wall_timer(
        std::chrono::duration<double>(period_s),
        [this, &client, bridge_key]() {
            // Poll this remote bridge at its configured rate and publish a decoded ROS message.
            (void)client;
            std::unique_lock<std::recursive_mutex> state_lock;
            if (state_mutex_ != nullptr) {
                state_lock = std::unique_lock<std::recursive_mutex>(*state_mutex_);
            }
            if (!registry_) {
                return;
            }

            auto it = registry_->imports().find(bridge_key);
            if (it == registry_->imports().end()) {
                return;
            }

            auto& state = it->second;
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
    // Coalesce an incoming notification so bridge processing can run outside cache callbacks.
    std::unique_lock<std::recursive_mutex> state_lock;
    if (state_mutex_ != nullptr) {
        state_lock = std::unique_lock<std::recursive_mutex>(*state_mutex_);
    }
    if (!registry_ || characteristic_path.empty()) {
        return false;
    }

    bool updated = false;
    for (auto& [_, state] : registry_->imports()) {
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
    // Resolve this peer remote GATT paths and stage notification changes outside the state lock.
    std::unique_lock<std::recursive_mutex> state_lock;
    if (state_mutex_ != nullptr) {
        state_lock = std::unique_lock<std::recursive_mutex>(*state_mutex_);
    }
    if (!registry_ || mac.empty()) {
        return false;
    }

    bool changed = false;
    std::vector<std::string> stop_notify_paths;
    std::vector<std::pair<std::string, std::string>> start_notify_requests;
    for (auto& [bridge_key, state] : registry_->imports()) {
        if (state.mac != mac || state.bridge_uuid.empty()) {
            continue;
        }

        const auto previous_path = state.path;
        const auto resolved_path = client.find_characteristic(mac, state.bridge_uuid);

        if (resolved_path == previous_path) {
            if (!resolved_path.empty() && state.rate_hz > 0.0 && !state.poll_timer) {
                configure_import_poll_timer(bridge_key, state, client);
                changed = true;
            }
            continue;
        }

        state.path = resolved_path;
        state.pending_payload.clear();
        state.last_payload.clear();
        state.last_publish_monotonic = 0.0;
        state.current_hz = 0.0;

        if (!previous_path.empty()) {
            stop_notify_paths.push_back(previous_path);
        }

        if (state.rate_hz > 0.0) {
            configure_import_poll_timer(bridge_key, state, client);
        } else if (state.poll_timer) {
            state.poll_timer->cancel();
            state.poll_timer.reset();
        }

        if (!state.path.empty()) {
            start_notify_requests.emplace_back(bridge_key, state.path);
        }

        changed = true;
    }

    if (state_lock.owns_lock()) {
        state_lock.unlock();
    }

    for (const auto& path : stop_notify_paths) {
        client.stop_notify(path);
    }

    for (const auto& [bridge_key, path] : start_notify_requests) {
        if (!client.start_notify(path)) {
            RCLCPP_WARN(logger_, "Failed to enable notifications for import bridge %s on %s",
                        bridge_key.c_str(), path.c_str());
        }
    }

    return changed;
}

bool ImportBridgeManager::clear_import_paths_for_mac(const std::string& mac,
                                                     bluez::BluezClient& client) {
    // Detach every cached characteristic and publisher owned by this peer.
    std::unique_lock<std::recursive_mutex> state_lock;
    if (state_mutex_ != nullptr) {
        state_lock = std::unique_lock<std::recursive_mutex>(*state_mutex_);
    }
    if (!registry_ || mac.empty()) {
        return false;
    }

    bool changed = false;
    std::vector<std::string> stop_notify_paths;
    for (auto& [_, state] : registry_->imports()) {
        if (state.mac != mac) {
            continue;
        }

        if (!state.path.empty()) {
            stop_notify_paths.push_back(state.path);
        }
        if (state.poll_timer) {
            state.poll_timer->cancel();
            state.poll_timer.reset();
        }
        if (!state.path.empty() || !state.pending_payload.empty()) {
            changed = true;
        }
        state.path.clear();
        state.pending_payload.clear();
        state.last_payload.clear();
        state.last_publish_monotonic = 0.0;
        state.current_hz = 0.0;
    }

    if (state_lock.owns_lock()) {
        state_lock.unlock();
    }

    for (const auto& path : stop_notify_paths) {
        client.stop_notify(path);
    }

    return changed;
}

}  // namespace mrs_uav_bluetooth::bridge
