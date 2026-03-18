// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sdbus-c++/sdbus-c++.h>

#include <functional>
#include <memory>
#include <string>

namespace mrs_uav_bluetooth::bluez {

/// Callback for pairing-agent events.
using AgentEventCallback = std::function<void(const std::string& event_type,
                                              const std::string& device_path)>;

/// D-Bus exported BlueZ pairing agent (org.bluez.Agent1).
/// Supports auto-accept and auto-trust modes.
class BluezPairingAgent {
public:
    static constexpr const char* kAgentPath = "/org/bluez/mrs_bt/app/agent";

    BluezPairingAgent(DbusConnection& dbus,
                      rclcpp::Logger logger,
                      bool auto_accept = true,
                      bool auto_trust = true);
    ~BluezPairingAgent();

    /// Register this agent with the BlueZ agent manager.
    /// \param capability  e.g. "NoInputNoOutput", "DisplayOnly", etc.
    void register_agent(const std::string& capability = "NoInputNoOutput");

    /// Unregister the agent.
    void unregister_agent();

    /// Set the event callback.
    void set_event_callback(AgentEventCallback cb);

private:
    void set_trusted(const std::string& device_path);
    void emit(const std::string& event, const std::string& device_path = "");

    DbusConnection& dbus_;
    rclcpp::Logger logger_;
    bool auto_accept_;
    bool auto_trust_;
    AgentEventCallback on_event_;

    std::unique_ptr<sdbus::IObject> exported_object_;
    bool registered_{false};
};

}  // namespace mrs_uav_bluetooth::bluez
