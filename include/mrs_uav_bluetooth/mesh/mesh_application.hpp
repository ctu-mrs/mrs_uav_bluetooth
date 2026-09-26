// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/config/config_models.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sdbus-c++/sdbus-c++.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace mrs_uav_bluetooth::mesh {

inline constexpr const char* kMeshService = "org.bluez.mesh";
inline constexpr const char* kMeshNetworkInterface = "org.bluez.mesh.Network1";
inline constexpr const char* kMeshNodeInterface = "org.bluez.mesh.Node1";
inline constexpr const char* kMeshManagementInterface = "org.bluez.mesh.Management1";
inline constexpr const char* kMeshApplicationInterface = "org.bluez.mesh.Application1";
inline constexpr const char* kMeshElementInterface = "org.bluez.mesh.Element1";
inline constexpr const char* kMeshAttentionInterface = "org.bluez.mesh.Attention1";
inline constexpr const char* kMeshProvisionerInterface = "org.bluez.mesh.Provisioner1";
inline constexpr const char* kMeshProvisionAgentInterface = "org.bluez.mesh.ProvisionAgent1";

using VariantMap = std::map<std::string, sdbus::Variant>;
using ModelConfiguration = sdbus::Struct<uint16_t, VariantMap>;
using ElementConfiguration = sdbus::Struct<uint8_t, std::vector<ModelConfiguration>>;

struct ReceivedMessage {
    uint8_t element_index{0};
    uint16_t source{0};
    uint16_t destination{0};
    uint16_t key_index{0};
    uint16_t net_index{0};
    bool device_key{false};
    bool remote{false};
    std::vector<uint8_t> data;
};

struct Event {
    std::string event;
    std::string uuid;
    std::string reason;
    int16_t rssi{0};
    std::vector<uint8_t> data;
    uint16_t server{0};
    uint16_t original{0};
    uint16_t unicast{0};
    uint8_t nppi{0};
    uint8_t count{0};
    std::string detail;
};

struct Status {
    bool daemon_available{false};
    bool attached{false};
    std::string state{"disabled"};
    std::string uuid;
    std::string node_path;
    uint64_t token{0};
    std::vector<uint16_t> addresses;
    bool friend_feature{false};
    bool low_power_feature{false};
    bool proxy_feature{false};
    bool relay_feature{false};
    bool beacon{false};
    bool iv_update{false};
    uint32_t iv_index{0};
    uint32_t seconds_since_last_heard{0};
    uint32_t sequence_number{0};
    std::string error;
};

/// Device-key material replicated only among trusted provisioner candidates.
struct DeviceKey {
    uint16_t unicast{0};
    std::vector<uint8_t> key;
};

class MeshApplication {
public:
    using MessageCallback = std::function<void(const ReceivedMessage&)>;
    using EventCallback = std::function<void(const Event&)>;
    using TokenCallback = std::function<void(uint64_t)>;

    MeshApplication(bluez::DbusConnection& dbus,
                    config::NodeConfig config,
                    std::string hostname,
                    rclcpp::Logger logger);
    ~MeshApplication();

    MeshApplication(const MeshApplication&) = delete;
    MeshApplication& operator=(const MeshApplication&) = delete;

    void export_objects();
    bool daemon_available() const;
    void refresh_status();
    Status status() const;
    /// True only after BlueZ confirms the vendor AppKey binding and subscription.
    bool vendor_model_ready() const;
    /// Verify imported keys without logging them; import the AppKey if absent.
    void prepare_fleet_keys();
    /// Load/create a private Device Key before importing a new fleet identity.
    std::vector<uint8_t> fleet_device_key();
    /// Import this node as the first member of a new private, random-key Mesh.
    /// Credentials are generated once and stored mode 0600 beside its token.
    void import_auto_identity(uint16_t unicast);
    /// Ensure this node has an AppKey. A joining node waits until its
    /// provisioner has installed the key through the Mesh Config Server.
    bool prepare_auto_keys(uint16_t unicast);
    /// Set the fixed UAV address returned by the next RequestProvData call.
    void set_auto_provisioning_unicast(uint16_t unicast);

    void set_message_callback(MessageCallback callback);
    void set_event_callback(EventCallback callback);
    void set_token_callback(TokenCallback callback);

    const std::vector<uint8_t>& uuid() const { return uuid_; }
    std::string uuid_hex() const;
    uint64_t token() const;
    std::string element_path(uint8_t index = 0) const;

    void join(const std::vector<uint8_t>& uuid = {});
    void cancel_join();
    void attach(uint64_t token = 0);
    void leave(uint64_t token = 0);
    void create_network(const std::vector<uint8_t>& uuid = {});
    void import_node(const std::vector<uint8_t>& uuid,
                     const std::vector<uint8_t>& device_key,
                     const std::vector<uint8_t>& network_key,
                     uint16_t network_index,
                     bool iv_update,
                     bool key_refresh,
                     uint32_t iv_index,
                     uint16_t unicast);

    /// Configure the attached local node for managed-flooding multi-hop.
    ///
    /// Sends Config Default TTL Set and Config Relay Set to the local
    /// Configuration Server using the local device key.
    /// @param default_ttl Access-message TTL (`0` or `2..127`).
    /// @param retransmit_count Additional relay transmissions (`0..7`).
    /// @param retransmit_interval_steps Ten-millisecond interval steps (`0..31`).
    void configure_local_relay(uint8_t default_ttl,
                               uint8_t retransmit_count,
                               uint8_t retransmit_interval_steps);

    /// Bind the package vendor model to an AppKey and subscribe it to a group.
    ///
    /// Configuration messages use the destination node's device key. Passing
    /// `remote` selects the remote-device-key index used by BlueZ. It must be
    /// true for provisioned peers and also for a local Config Server loopback;
    /// false is retained for raw/manual device-key use cases only.
    void configure_vendor_model(uint16_t destination,
                                bool remote,
                                uint16_t network_index,
                                uint16_t application_index,
                                uint16_t group_address,
                                uint16_t company_id,
                                uint16_t model_id);

    void send(uint8_t element_index, uint16_t destination, uint16_t key_index,
              bool force_segmented, const std::vector<uint8_t>& data);
    void dev_key_send(uint8_t element_index, uint16_t destination, bool remote,
                      uint16_t network_index, bool force_segmented,
                      const std::vector<uint8_t>& data);
    void add_net_key(uint8_t element_index, uint16_t destination,
                     uint16_t subnet_index, uint16_t network_index, bool update);
    void add_app_key(uint8_t element_index, uint16_t destination,
                     uint16_t application_index, uint16_t network_index, bool update);
    void publish(uint8_t element_index, uint16_t model_id,
                 std::optional<uint16_t> vendor_id, bool force_segmented,
                 const std::vector<uint8_t>& data);

    void unprovisioned_scan(const VariantMap& options);
    void unprovisioned_scan_cancel();
    void add_node(const std::vector<uint8_t>& uuid, const VariantMap& options);
    void reprovision(uint16_t unicast, const VariantMap& options);
    void create_subnet(uint16_t network_index);
    void import_subnet(uint16_t network_index, const std::vector<uint8_t>& key);
    void update_subnet(uint16_t network_index);
    void delete_subnet(uint16_t network_index);
    void set_key_phase(uint16_t network_index, uint8_t phase);
    void create_app_key(uint16_t network_index, uint16_t application_index);
    void import_app_key(uint16_t network_index, uint16_t application_index,
                        const std::vector<uint8_t>& key);
    void update_app_key(uint16_t application_index);
    void delete_app_key(uint16_t application_index);
    void import_remote_node(uint16_t primary, uint8_t count,
                            const std::vector<uint8_t>& device_key);
    void delete_remote_node(uint16_t primary, uint8_t count);
    /// Return remote-node device keys for encrypted standby synchronization.
    std::vector<DeviceKey> export_device_keys();
    /// Return a human-readable complete key snapshot for operator backup.
    std::string export_keys_yaml();

private:
    void export_application();
    void update_model_configuration(uint16_t model_id, const VariantMap& config);
    bool vendor_bound_{false};
    bool vendor_subscribed_{false};
    void export_agent();
    void export_element();
    void export_attention();
    void export_provisioner();
    /// Ask the system D-Bus daemon to activate bluetooth-meshd, rate-limited
    /// so a missing host prerequisite cannot create a one-hertz restart storm.
    bool request_daemon_activation();
    void set_token(uint64_t token);
    void emit_event(Event event) const;
    void emit_message(ReceivedMessage message) const;
    std::unique_ptr<sdbus::IProxy> network_proxy() const;
    std::unique_ptr<sdbus::IProxy> node_proxy() const;
    std::unique_ptr<sdbus::IProxy> management_proxy() const;
    void require_attached() const;
    void persist_next_unicast() const;
    static void require_size(const std::vector<uint8_t>& value, size_t expected,
                             const std::string& name);

    bluez::DbusConnection& dbus_;
    config::NodeConfig config_;
    std::string hostname_;
    rclcpp::Logger logger_;
    /// ObjectManager root supplied to BlueZ lifecycle methods.
    std::string root_path_{"/cz/cvut/mrs/uav/bluetooth/mesh"};
    /// Application1 must be a managed child: sdbus-c++ ObjectManager does not
    /// include interfaces placed directly on its own root object.
    std::string application_path_{root_path_ + "/application"};
    std::string agent_path_{root_path_ + "/agent"};
    std::string element_path_{root_path_ + "/ele00"};
    std::vector<uint8_t> uuid_;

    std::unique_ptr<sdbus::IObject> root_object_;
    std::unique_ptr<sdbus::IObject> application_object_;
    std::unique_ptr<sdbus::IObject> agent_object_;
    std::unique_ptr<sdbus::IObject> element_object_;
    std::optional<sdbus::Slot> object_manager_slot_;
    // BlueZ defers these replies while it initializes the temporary/new node.
    // Owning the slots makes the callbacks safe and cancelable at destruction.
    std::optional<sdbus::Slot> join_call_slot_;
    std::optional<sdbus::Slot> create_call_slot_;
    /// AddNode keeps its method reply pending until the provisioning bearer has
    /// started. Own the slot without blocking the ROS executor in the meantime.
    std::optional<sdbus::Slot> add_node_call_slot_;
    std::atomic_bool add_node_start_pending_{false};

    mutable std::mutex mutex_;
    Status status_;
    MessageCallback message_callback_;
    EventCallback event_callback_;
    TokenCallback token_callback_;
    uint16_t next_unicast_{0x0100};
    uint16_t unicast_block_end_{0x7fff};
    uint16_t auto_provisioning_unicast_{0};
    std::chrono::steady_clock::time_point attention_until_{};
    std::chrono::steady_clock::time_point last_daemon_activation_attempt_{};
};

/// Parse a 16-byte Mesh Device UUID from canonical or compact hexadecimal.
std::vector<uint8_t> mesh_uuid_from_string(const std::string& value);

/// Derive a deterministic RFC 4122 version-3 UUID for a named Mesh device.
///
/// The repository's generic name UUID helper intentionally preserves its
/// historical output because those values are also used by existing GATT
/// profiles. Mesh uses this dedicated wrapper because BlueZ rejects UUID byte
/// arrays whose RFC version and variant bits are not set.
std::vector<uint8_t> mesh_uuid_from_name(const std::string& value);
/// Automatic-enrollment UUID also carries the numeric UAV identity so an
/// intentionally open peer list can admit nearby UAVs without a name lookup.
std::vector<uint8_t> mesh_auto_uuid_from_name(const std::string& hostname);
uint16_t mesh_auto_uuid_number(const std::vector<uint8_t>& uuid);

/// Render a byte vector as lowercase hexadecimal without separators.
std::string mesh_bytes_to_hex(const std::vector<uint8_t>& value);

}  // namespace mrs_uav_bluetooth::mesh
