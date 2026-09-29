// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/bluez/bluez_pairing_agent.hpp
/// \brief Declares the bluez pairing agent component of the BlueZ system-D-Bus integration layer.

#pragma once

#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sdbus-c++/sdbus-c++.h>

#include <functional>
#include <chrono>
#include <memory>
#include <string>

namespace mrs_uav_bluetooth::bluez {

/// Callback for pairing-agent events.
using AgentEventCallback = std::function<void(const std::string& event_type,
                                              const std::string& device_path)>;

using AgentRequestPolicyCallback = std::function<bool(const std::string& event_type,
                                                      const std::string& device_path)>;

/// D-Bus exported BlueZ pairing agent (org.bluez.Agent1).
/// Supports auto-accept and auto-trust modes.
class BluezPairingAgent {
public:
    static constexpr const char* kAgentPath = "/org/bluez/mrs_bt/agent";

    /// \brief Create the policy owner for incoming BlueZ pairing prompts.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param logger ROS logger used for diagnostics.
    /// \param auto_accept whether incoming pairing prompts are accepted without operator input.
    /// \param auto_trust whether accepted peers are marked Trusted automatically.
    BluezPairingAgent(DbusConnection& dbus,
                      rclcpp::Logger logger,
                      bool auto_accept = true,
                      bool auto_trust = true);
    /// \brief Unregister the Agent1 object before releasing D-Bus.
    ~BluezPairingAgent();

    /// Register this agent with the BlueZ agent manager.
    /// \param capability  e.g. "NoInputNoOutput", "DisplayOnly", etc.
    void register_agent(const std::string& capability = "NoInputNoOutput");

    /// Unregister Agent1 from BlueZ and remove its exported D-Bus object.
    void unregister_agent();

    /// Install the observer for pairing prompts, completions, cancellations, and release.
    /// \param cb Observer receiving pairing-agent events.
    void set_event_callback(AgentEventCallback cb);

    /// Set the per-request policy callback. Return true to allow the request.
    /// \param cb Policy function deciding whether an incoming pairing request is allowed.
    void set_request_policy_callback(AgentRequestPolicyCallback cb);

    /// Enable or disable unattended pairing requests without re-registering Agent1.
    /// \param auto_pair whether policy permits unattended pairing requests.
    void set_auto_pair(bool auto_pair);
    /// \brief Choose whether successfully authorized peers are marked trusted automatically.
    /// \param auto_trust whether accepted peers are marked Trusted automatically.
    void set_auto_trust(bool auto_trust);

private:
    /// \brief Set BlueZ's Trusted property for the device that completed authorization.
    /// \param device_path BlueZ device path to mark trusted.
    void set_trusted(const std::string& device_path);
    /// \brief Update pending-pairing state and deliver one agent event to the observer.
    /// \param event Pairing-agent event name delivered to observers.
    /// \param device_path BlueZ device path associated with the pairing request.
    void emit(const std::string& event, const std::string& device_path = "");
    /// \brief Apply the current pairing admission policy to one Agent1 request.
    /// \param event Pairing-agent request type evaluated by policy.
    /// \param device_path BlueZ device path associated with the pairing request.
    /// \return True when the request is admitted; otherwise false.
    bool should_allow_request(const std::string& event,
                              const std::string& device_path) const;

    DbusConnection& dbus_;
    rclcpp::Logger logger_;
    bool auto_pair_;
    bool auto_trust_;
    AgentEventCallback on_event_;
    AgentRequestPolicyCallback request_policy_;
    std::string pending_pairing_device_path_;
    std::chrono::steady_clock::time_point pending_pairing_request_time_{};

    std::unique_ptr<sdbus::IObject> exported_object_;
    bool registered_{false};
};

}  // namespace mrs_uav_bluetooth::bluez
