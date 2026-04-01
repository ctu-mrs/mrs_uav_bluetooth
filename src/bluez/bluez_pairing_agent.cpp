// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/bluez/bluez_pairing_agent.hpp"

namespace mrs_uav_bluetooth::bluez {

namespace {

constexpr auto kPendingPairingCancelTimeout = std::chrono::seconds(20);

bool request_requires_pairing_flow(const std::string& event) {
    return event == "request_pin" ||
           event == "request_passkey" ||
           event == "request_confirmation";
}

}

BluezPairingAgent::BluezPairingAgent(DbusConnection& dbus,
                                     rclcpp::Logger logger,
                                     bool auto_pair,
                                     bool auto_trust)
    : dbus_(dbus), logger_(logger), auto_pair_(auto_pair), auto_trust_(auto_trust) {}

BluezPairingAgent::~BluezPairingAgent() {
    if (registered_) {
        try {
            unregister_agent();
        } catch (...) {}
    }
}

void BluezPairingAgent::register_agent(const std::string& capability) {
    // Create and export the agent object.
    exported_object_ = sdbus::createObject(dbus_.connection(),
                                           sdbus::ObjectPath{kAgentPath});

    const std::string agent_iface = std::string(kAgentIface);

    // Release()
    exported_object_->addVTable(
        sdbus::registerMethod("Release")
            .implementedAs([this]() {
                emit("agent_release");
            }),
        sdbus::registerMethod("AuthorizeService")
            .withInputParamNames("device", "uuid")
            .implementedAs([this](const sdbus::ObjectPath& device, const std::string& uuid) {
                if (!should_allow_request("authorize_service", std::string(device))) {
                    throw sdbus::Error(sdbus::Error::Name{"org.bluez.Error.Rejected"},
                                       "Connection rejected");
                }
                emit("authorize_service", std::string(device));
                (void)uuid;
            }),
        sdbus::registerMethod("RequestPinCode")
            .withInputParamNames("device")
            .withOutputParamNames("pincode")
            .implementedAs([this](const sdbus::ObjectPath& device) -> std::string {
                if (!should_allow_request("request_pin", std::string(device))) {
                    throw sdbus::Error(sdbus::Error::Name{"org.bluez.Error.Rejected"},
                                       "PIN code request rejected");
                }
                pending_pairing_device_path_ = std::string(device);
                pending_pairing_request_time_ = std::chrono::steady_clock::now();
                emit("request_pin", std::string(device));
                if (auto_trust_) set_trusted(std::string(device));
                return "";
            }),
        sdbus::registerMethod("RequestPasskey")
            .withInputParamNames("device")
            .withOutputParamNames("passkey")
            .implementedAs([this](const sdbus::ObjectPath& device) -> uint32_t {
                if (!should_allow_request("request_passkey", std::string(device))) {
                    throw sdbus::Error(sdbus::Error::Name{"org.bluez.Error.Rejected"},
                                       "Passkey request rejected");
                }
                pending_pairing_device_path_ = std::string(device);
                pending_pairing_request_time_ = std::chrono::steady_clock::now();
                emit("request_passkey", std::string(device));
                if (auto_trust_) set_trusted(std::string(device));
                return 0;
            }),
        sdbus::registerMethod("DisplayPasskey")
            .withInputParamNames("device", "passkey", "entered")
            .implementedAs([this](const sdbus::ObjectPath& device,
                                  uint32_t passkey, uint16_t entered) {
                emit("display_passkey", std::string(device));
                (void)passkey; (void)entered;
            }),
        sdbus::registerMethod("DisplayPinCode")
            .withInputParamNames("device", "pincode")
            .implementedAs([this](const sdbus::ObjectPath& device,
                                  const std::string& pincode) {
                emit("display_pin", std::string(device));
                (void)pincode;
            }),
        sdbus::registerMethod("RequestConfirmation")
            .withInputParamNames("device", "passkey")
            .implementedAs([this](const sdbus::ObjectPath& device, uint32_t passkey) {
                (void)passkey;
                if (!should_allow_request("request_confirmation", std::string(device))) {
                    throw sdbus::Error(sdbus::Error::Name{"org.bluez.Error.Rejected"},
                                       "Passkey not confirmed");
                }
                pending_pairing_device_path_ = std::string(device);
                pending_pairing_request_time_ = std::chrono::steady_clock::now();
                emit("request_confirmation", std::string(device));
                if (auto_trust_) set_trusted(std::string(device));
            }),
        sdbus::registerMethod("RequestAuthorization")
            .withInputParamNames("device")
            .implementedAs([this](const sdbus::ObjectPath& device) {
                if (!should_allow_request("request_authorization", std::string(device))) {
                    throw sdbus::Error(sdbus::Error::Name{"org.bluez.Error.Rejected"},
                                       "Authorization rejected");
                }
                emit("request_authorization", std::string(device));
            }),
        sdbus::registerMethod("Cancel")
            .implementedAs([this]() {
                if (pending_pairing_device_path_.empty()) {
                    return;
                }
                if ((std::chrono::steady_clock::now() - pending_pairing_request_time_) >
                    kPendingPairingCancelTimeout) {
                    pending_pairing_device_path_.clear();
                    pending_pairing_request_time_ = std::chrono::steady_clock::time_point{};
                    return;
                }
                const auto device_path = pending_pairing_device_path_;
                pending_pairing_device_path_.clear();
                pending_pairing_request_time_ = std::chrono::steady_clock::time_point{};
                emit("cancel", device_path);
            })
    ).forInterface(agent_iface);

    // Register with AgentManager1.
    auto am = sdbus::createProxy(dbus_.connection(),
                                 sdbus::ServiceName{std::string(kBluezServiceName)},
                                 sdbus::ObjectPath{"/org/bluez"});
    am->callMethod("RegisterAgent")
        .onInterface(std::string(kAgentManagerIface))
        .withArguments(sdbus::ObjectPath{kAgentPath}, capability);
    am->callMethod("RequestDefaultAgent")
        .onInterface(std::string(kAgentManagerIface))
        .withArguments(sdbus::ObjectPath{kAgentPath});

    registered_ = true;
    RCLCPP_INFO(logger_, "Pairing agent registered at %s (capability=%s)",
                kAgentPath, capability.c_str());
}

void BluezPairingAgent::unregister_agent() {
    if (!registered_) return;
    try {
        auto am = sdbus::createProxy(dbus_.connection(),
                                     sdbus::ServiceName{std::string(kBluezServiceName)},
                                     sdbus::ObjectPath{"/org/bluez"});
        am->callMethod("UnregisterAgent")
            .onInterface(std::string(kAgentManagerIface))
            .withArguments(sdbus::ObjectPath{kAgentPath});
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "Failed to unregister agent: %s", e.getMessage().c_str());
    }
    exported_object_.reset();
    registered_ = false;
}

void BluezPairingAgent::set_event_callback(AgentEventCallback cb) {
    on_event_ = std::move(cb);
}

void BluezPairingAgent::set_request_policy_callback(AgentRequestPolicyCallback cb) {
    request_policy_ = std::move(cb);
}

void BluezPairingAgent::set_auto_pair(bool auto_pair) {
    auto_pair_ = auto_pair;
    if (!auto_pair_) {
        pending_pairing_device_path_.clear();
        pending_pairing_request_time_ = std::chrono::steady_clock::time_point{};
    }
}

void BluezPairingAgent::set_auto_trust(bool auto_trust) {
    auto_trust_ = auto_trust;
}

void BluezPairingAgent::set_trusted(const std::string& device_path) {
    try {
        auto proxy = sdbus::createProxy(dbus_.connection(),
                                        sdbus::ServiceName{std::string(kBluezServiceName)},
                                        sdbus::ObjectPath{device_path});
        proxy->callMethod("Set")
            .onInterface(std::string(kDbusPropertiesIface))
            .withArguments(std::string{kDeviceIface},
                           std::string{"Trusted"},
                           sdbus::Variant{true});
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "Failed to auto-trust %s: %s",
                    device_path.c_str(), e.getMessage().c_str());
    }
}

void BluezPairingAgent::emit(const std::string& event,
                              const std::string& device_path) {
    if (event == "agent_release") {
        pending_pairing_device_path_.clear();
        pending_pairing_request_time_ = std::chrono::steady_clock::time_point{};
    }
    if (on_event_) {
        try {
            on_event_(event, device_path);
        } catch (...) {}
    }
}

bool BluezPairingAgent::should_allow_request(const std::string& event,
                                             const std::string& device_path) const {
    if (!request_policy_) {
        return auto_pair_ || !request_requires_pairing_flow(event);
    }
    try {
        return request_policy_(event, device_path);
    } catch (...) {
        return false;
    }
}

}  // namespace mrs_uav_bluetooth::bluez
