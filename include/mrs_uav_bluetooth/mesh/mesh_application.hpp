// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/mesh/mesh_application.hpp
/// \brief Declares the mesh application component of the Bluetooth Mesh D-Bus layer.

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

/// Exact daemon transfer plus the local attachment that created it.
/// Attachment identity prevents a delayed cleanup from touching a new network.
struct SendHandle {
    uint64_t transfer{0};
    uint64_t attachment{0};
};

/// Access message received from bluetooth-meshd through Element1.MessageReceived.
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

/// Provisioning or daemon lifecycle event reported to ROS and coordination logic.
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

/// Snapshot of attachment, network identity, and local Mesh addresses.
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

/// D-Bus implementation of a complete BlueZ Mesh application.
/// It exports the application, element, provisioner, agent, and management
/// interfaces and uses Network1/Node1 for all runtime control and traffic.
class MeshApplication {
public:
    using MessageCallback = std::function<void(const ReceivedMessage&)>;
    using EventCallback = std::function<void(const Event&)>;
    using TokenCallback = std::function<void(uint64_t)>;

    /// \brief Export the Mesh D-Bus application and manage daemon attachment state.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param config Node configuration defining Mesh identity keys addresses and model settings.
    /// \param hostname UAV hostname used to identify the node.
    /// \param logger ROS logger used for diagnostics.
    MeshApplication(bluez::DbusConnection& dbus,
                    config::NodeConfig config,
                    std::string hostname,
                    rclcpp::Logger logger);
    /// \brief Cancel pending sends and remove every exported Mesh D-Bus object.
    ~MeshApplication();

    /// \brief Disable copying of the mesh application.
    MeshApplication(const MeshApplication&) = delete;
    /// \brief Disable copy assignment of the mesh application.
    MeshApplication& operator=(const MeshApplication&) = delete;

    /// \brief Publish the Mesh application agent element provisioner and management D-Bus interfaces.
    void export_objects();
    /// True after BlueZ owns its bus name and exports the ready Network1 API.
    /// \return True when daemon available; otherwise false.
    bool daemon_available() const;
    /// \brief Reconcile daemon ownership attachment paths addresses keys and model readiness.
    void refresh_status();
    /// \brief Copy current daemon, attachment, and model state under lock.
    /// \return Locked copy of current daemon attachment key model and node state.
    Status status() const;
    /// True only after BlueZ confirms the vendor AppKey binding and subscription.
    /// \return True when vendor model ready; otherwise false.
    bool vendor_model_ready() const;
    /// Load or create the private Device Key for this UAV's Mesh identity.
    /// \return Local node device key returned by the daemon.
    std::vector<uint8_t> local_device_key();
    /// Import this node as the first member of a new private, random-key Mesh.
    /// Credentials are generated once and stored mode 0600 beside its token.
    /// \param unicast Primary unicast address assigned to the automatically imported local node.
    void import_auto_identity(uint16_t unicast);
    /// Ensure this node has an AppKey. A joining node waits until its
    /// provisioner has installed the key through the Mesh Config Server.
    /// \return True if valid local application-key material is ready; otherwise false.
    bool prepare_auto_keys();
    /// Set the fixed UAV address returned by the next RequestProvData call.
    /// \param unicast Primary unicast address reserved for automatic local provisioning.
    void set_auto_provisioning_unicast(uint16_t unicast);

    /// \brief Install the observer for decoded Mesh access messages.
    /// \param callback Observer receiving decoded Mesh access messages.
    void set_message_callback(MessageCallback callback);
    /// \brief Install the observer for Mesh lifecycle and provisioning events.
    /// \param callback Observer receiving Mesh lifecycle and provisioning events.
    void set_event_callback(EventCallback callback);
    /// \brief Install the observer that persists newly assigned attachment tokens.
    /// \param callback Observer receiving newly assigned persistent Mesh tokens.
    void set_token_callback(TokenCallback callback);

    /// \brief Return the immutable 16-byte UUID used to provision this Mesh node.
    /// \return Binary Mesh device UUID exported by the application.
    const std::vector<uint8_t>& uuid() const {
        // Return the immutable 16-byte UUID exported for Mesh provisioning.
        return uuid_;
    }
    /// \brief Format the local provisioning UUID as lowercase hexadecimal.
    /// \return Normalized lowercase hexadecimal Mesh UUID.
    std::string uuid_hex() const;
    /// \brief Read the current daemon attachment token under lock.
    /// \return Current persistent Mesh attachment token or zero while detached.
    uint64_t token() const;
    /// \brief Return the exported path of the single local Mesh element.
    /// \param index Zero-based local Mesh element whose D-Bus path is requested.
    /// \return D-Bus path of the single exported Mesh element.
    std::string element_path(uint8_t index = 0) const;

    /// \brief Ask Network1 to provision this application UUID into a Mesh network.
    /// \param uuid Sixteen-byte Mesh device UUID.
    void join(const std::vector<uint8_t>& uuid = {});
    /// \brief Cancel an outstanding provisioning join request through Network1.
    void cancel_join();
    /// \brief Attach the exported application to a previously provisioned Mesh token.
    /// \param token Persistent Mesh network token supplied by the daemon.
    void attach(uint64_t token = 0);
    /// \brief Remove the provisioned node identified by its persistent token.
    /// \param token Persistent Mesh network token supplied by the daemon.
    void leave(uint64_t token = 0);
    /// \brief Ask the Mesh daemon to create a network and return a persistent token.
    /// \param uuid Sixteen-byte Mesh device UUID.
    void create_network(const std::vector<uint8_t>& uuid = {});
    /// \brief Import a fully specified local Mesh identity through Network1.
    /// \param uuid Sixteen-byte Mesh device UUID assigned to the imported node.
    /// \param device_key 16-byte Mesh device key used for import or device-key messages.
    /// \param network_key 16-byte Mesh network key used when importing a node.
    /// \param network_index Subnet index assigned to the imported local node.
    /// \param iv_update whether the imported Mesh network is performing an IV update.
    /// \param key_refresh whether imported Mesh keys are already in refresh phase.
    /// \param iv_index Mesh IV index associated with imported network state.
    /// \param unicast Primary unicast address assigned to the imported local node.
    void import_node(const std::vector<uint8_t>& uuid,
                     const std::vector<uint8_t>& device_key,
                     const std::vector<uint8_t>& network_key,
                     uint16_t network_index,
                     bool iv_update,
                     bool key_refresh,
                     uint32_t iv_index,
                     uint16_t unicast);

    /// Request the default TTL for locally originated messages (0 or 2..127).
    /// The caller confirms the value through Config Default TTL Status.
    /// \param default_ttl default hop limit used for outgoing Mesh messages.
    void configure_local_default_ttl(uint8_t default_ttl);

    /// Request relaying for multi-hop delivery through the local Config Server.
    /// The caller confirms the enabled state and repetitions through Relay Status.
    /// @param retransmit_count Additional relay transmissions (`0..7`).
    /// @param retransmit_interval_steps Relay interval in 10 ms steps (`0..31`).
    void configure_local_relay(uint8_t retransmit_count,
                               uint8_t retransmit_interval_steps);

    /// Request local repetitions of originated network packets. The caller
    /// must wait for Config Network Transmit Status before treating this as set.
    /// \param retransmit_count number of additional Mesh transmissions requested.
    /// \param retransmit_interval_steps Mesh retransmission interval in specification-defined steps.
    void configure_local_network_transmit(uint8_t retransmit_count,
                                          uint8_t retransmit_interval_steps);

    /// Bind the package vendor model to an AppKey and subscribe it to a group.
    ///
    /// Configuration messages use the destination node's device key. Passing
    /// `remote` selects the remote-device-key index used by BlueZ. It must be
    /// true for provisioned peers and also for a local Config Server loopback;
    /// false is retained for raw/manual device-key use cases only.
    /// \param destination Unicast address of the node being configured.
    /// \param remote whether a device-key message uses the remote node device key.
    /// \param network_index Subnet index whose application key is bound to the remote model.
    /// \param application_index Application key index bound to the vendor model.
    /// \param group_address Mesh group address subscribed by the vendor model.
    /// \param company_id identifier of the company.
    /// \param model_id Vendor model identifier bound and subscribed on the remote node.
    void configure_vendor_model(uint16_t destination,
                                bool remote,
                                uint16_t network_index,
                                uint16_t application_index,
                                uint16_t group_address,
                                uint16_t company_id,
                                uint16_t model_id);

    /// \brief Send an access message to a unicast or group destination through Node1.
    /// \param element_index Zero-based local Mesh element that originates the message.
    /// \param destination Mesh unicast or group address receiving the access message.
    /// \param key_index Application key index used to secure the access message.
    /// \param force_segmented whether BlueZ must use segmented Mesh transport even for a short payload.
    /// \param data Mesh access payload including its opcode.
    void send(uint8_t element_index, uint16_t destination, uint16_t key_index,
              bool force_segmented, const std::vector<uint8_t>& data);
    /// Submit without adding a segmented transfer behind an active one to
    /// the same destination. An empty optional reports daemon backpressure.
    /// A returned handle owns any outstanding unicast SAR. Its transfer field
    /// is zero when there is no outstanding unicast SAR to cancel.
    /// Other D-Bus errors are propagated to the caller.
    /// \param element_index Zero-based local Mesh element that originates the message.
    /// \param destination Mesh unicast or group address receiving the access message.
    /// \param key_index Application key index used to secure the access message.
    /// \param force_segmented whether BlueZ must use segmented Mesh transport even for a short payload.
    /// \param data Mesh access payload including its opcode.
    /// \return Prepared send handle, or std::nullopt when BlueZ reports destination backpressure.
    std::optional<SendHandle> try_send(uint8_t element_index, uint16_t destination, uint16_t key_index,
                  bool force_segmented, const std::vector<uint8_t>& data);
    /// Release the exact transfer and its queued repetitions. Completed
    /// transfers and handles from an earlier attachment are harmless no-ops.
    /// \param handle Prepared Mesh send whose pending resources are cancelled.
    void cancel_send(const SendHandle& handle);
    /// Check whether BlueZ is still repairing this exact unicast transfer.
    /// \param handle Prepared Mesh send checked for completion.
    /// \return True while the exact retained unicast transfer is still pending; otherwise false.
    bool send_pending(const SendHandle& handle) const;
    /// \brief Send one access message using a local or remote device key.
    /// \param element_index Zero-based local Mesh element that originates the message.
    /// \param destination Mesh unicast or group address receiving the access message.
    /// \param remote whether a device-key message uses the remote node device key.
    /// \param network_index Subnet index used to secure the device-key message.
    /// \param force_segmented whether BlueZ must use segmented Mesh transport even for a short payload.
    /// \param data Mesh access payload including its opcode.
    void dev_key_send(uint8_t element_index, uint16_t destination, bool remote,
                      uint16_t network_index, bool force_segmented,
                      const std::vector<uint8_t>& data);
    /// \brief Send a Config NetKey Add or Update message to a node.
    /// \param element_index Zero-based local element hosting the Configuration Client model.
    /// \param destination Unicast address of the node being configured.
    /// \param subnet_index Mesh subnet containing the network key.
    /// \param network_index Existing subnet used to deliver the new network key.
    /// \param update whether an existing Mesh key is updated instead of newly added.
    void add_net_key(uint8_t element_index, uint16_t destination,
                     uint16_t subnet_index, uint16_t network_index, bool update);
    /// \brief Send a Config AppKey Add or Update message to a node.
    /// \param element_index Zero-based local element hosting the Configuration Client model.
    /// \param destination Unicast address of the node being configured.
    /// \param application_index Application key index added to the remote node.
    /// \param network_index Subnet to which the application key belongs.
    /// \param update whether an existing Mesh key is updated instead of newly added.
    void add_app_key(uint8_t element_index, uint16_t destination,
                     uint16_t application_index, uint16_t network_index, bool update);
    /// \brief Publish an access message using one local model’s configured publication.
    /// \param element_index Zero-based local element whose model publishes the message.
    /// \param model_id SIG or vendor model identifier that owns the publication.
    /// \param vendor_id identifier of the vendor.
    /// \param force_segmented whether BlueZ must use segmented Mesh transport even for a short payload.
    /// \param data Mesh access payload including its opcode.
    void publish(uint8_t element_index, uint16_t model_id,
                 std::optional<uint16_t> vendor_id, bool force_segmented,
                 const std::vector<uint8_t>& data);

    /// \brief Start PB-ADV discovery through the management API.
    /// \param options BlueZ Mesh scan filters and timeout options.
    void unprovisioned_scan(const VariantMap& options);
    /// \brief Cancel PB-ADV discovery through the management API.
    void unprovisioned_scan_cancel();
    /// \brief Start provisioning the selected unprovisioned UUID through Management1.
    /// \param uuid Sixteen-byte Mesh device UUID.
    /// \param options Provisioning options passed to the Mesh daemon.
    void add_node(const std::vector<uint8_t>& uuid, const VariantMap& options);
    /// \brief Ask Management1 to reprovision a node at the selected unicast address.
    /// \param unicast Primary unicast address of the node to reprovision.
    /// \param options Reprovisioning options passed to the Mesh daemon.
    void reprovision(uint16_t unicast, const VariantMap& options);
    /// \brief Generate a new network key at the requested subnet index.
    /// \param network_index Subnet index changed by this request.
    void create_subnet(uint16_t network_index);
    /// \brief Install supplied network-key bytes at the requested subnet index.
    /// \param network_index Subnet index assigned to the imported network key.
    /// \param key Sixteen-byte Mesh network key to import.
    void import_subnet(uint16_t network_index, const std::vector<uint8_t>& key);
    /// \brief Begin network-key refresh for the selected subnet.
    /// \param network_index Subnet index changed by this request.
    void update_subnet(uint16_t network_index);
    /// \brief Remove the selected subnet and its dependent keys from the local node.
    /// \param network_index Subnet index changed by this request.
    void delete_subnet(uint16_t network_index);
    /// \brief Advance the selected subnet to the requested key-refresh phase.
    /// \param network_index Subnet index changed by this request.
    /// \param phase Mesh key-refresh phase assigned to the subnet.
    void set_key_phase(uint16_t network_index, uint8_t phase);
    /// \brief Generate an application key bound to the selected subnet.
    /// \param network_index Subnet that owns the application key.
    /// \param application_index Unused application key index reserved by the daemon.
    void create_app_key(uint16_t network_index, uint16_t application_index);
    /// \brief Install supplied application-key bytes at a stable index.
    /// \param network_index Subnet that owns the application key.
    /// \param application_index Application key index assigned to the supplied key bytes.
    /// \param key Sixteen-byte Mesh application key to import.
    void import_app_key(uint16_t network_index, uint16_t application_index,
                        const std::vector<uint8_t>& key);
    /// \brief Rotate the selected application key through the Mesh daemon.
    /// \param application_index Application key index whose material is rotated.
    void update_app_key(uint16_t application_index);
    /// \brief Remove the selected application key from the local node.
    /// \param application_index Application key index removed from the node.
    void delete_app_key(uint16_t application_index);
    /// \brief Store a provisioned peer's address range and device key in the local keyring.
    /// \param primary Primary unicast address of the remote node range.
    /// \param count Number of consecutive element addresses owned by the remote node.
    /// \param device_key 16-byte Mesh device key used for import or device-key messages.
    void import_remote_node(uint16_t primary, uint8_t count,
                            const std::vector<uint8_t>& device_key);
    /// \brief Remove a provisioned peer's address range and device key from the local keyring.
    /// \param primary Primary unicast address of the remote node range.
    /// \param count Number of consecutive element addresses owned by the remote node.
    void delete_remote_node(uint16_t primary, uint8_t count);
    /// Return remote-node device keys for encrypted standby synchronization.
    /// \return Typed device-key records returned by the Mesh daemon.
    std::vector<DeviceKey> export_device_keys();
    /// Return a human-readable complete key snapshot for operator backup.
    /// \return Daemon key material rendered as YAML.
    std::string export_keys_yaml();

private:
    /// \brief Expose the application root and ObjectManager tree required by Network1.
    void export_application();
    /// \brief Record daemon-confirmed bindings subscriptions and publication state for one model.
    /// \param model_id Model identifier whose daemon-reported configuration is cached.
    /// \param config Model configuration dictionary returned by the Mesh daemon.
    void update_model_configuration(uint16_t model_id, const VariantMap& config);
    bool vendor_bound_{false};
    bool vendor_subscribed_{false};
    /// \brief Expose provisioning prompts and capabilities through ProvisionAgent1.
    void export_agent();
    /// \brief Expose the vendor model and receive methods through Element1.
    void export_element();
    /// \brief Expose provisioning attention start and cancel callbacks.
    void export_attention();
    /// \brief Expose provisioning data allocation and completion callbacks.
    void export_provisioner();
    /// Ask the system D-Bus daemon to activate bluetooth-meshd, rate-limited
    /// so a missing host prerequisite cannot create a one-hertz restart storm.
    /// \return True if the Mesh daemon became reachable after D-Bus activation; otherwise false.
    bool request_daemon_activation();
    /// \brief Store the current attachment token and notify persistence observers only on change.
    /// \param token Persistent Mesh network token supplied by the daemon.
    void set_token(uint64_t token);
    /// \brief Deliver one Mesh lifecycle event outside the state lock.
    /// \param event Mesh lifecycle event delivered to application observers.
    void emit_event(Event event) const;
    /// \brief Deliver one decoded Mesh access message outside the D-Bus callback machinery.
    /// \param message Decoded Mesh message delivered to application observers.
    void emit_message(ReceivedMessage message) const;
    /// \brief Create a proxy for the Mesh Network1 object.
    /// \return Proxy bound to the Mesh Network1 object.
    std::unique_ptr<sdbus::IProxy> network_proxy() const;
    /// \brief Create a proxy for the attached Mesh Node1 object.
    /// \return Proxy bound to the attached Mesh Node1 object.
    std::unique_ptr<sdbus::IProxy> node_proxy() const;
    /// \brief Create a proxy for the Mesh Management1 object.
    /// \return Proxy bound to the Mesh Management1 object.
    std::unique_ptr<sdbus::IProxy> management_proxy() const;
    /// \brief Reject node operations until a current Node1 attachment exists.
    void require_attached() const;
    /// \brief Atomically store the next free provisioning address.
    void persist_next_unicast() const;
    /// \brief Reject malformed Mesh keys UUIDs and filters before issuing D-Bus calls.
    /// \param value Byte field whose exact length is required.
    /// \param expected Required byte length.
    /// \param name Field name used in validation errors.
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
    uint64_t attachment_id_{0};
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
/// \param value Hexadecimal Mesh UUID text to decode.
/// \return Parsed 16-byte Mesh UUID.
std::vector<uint8_t> mesh_uuid_from_string(const std::string& value);

/// Derive a deterministic RFC 4122 version-3 UUID for a named Mesh device.
///
/// The repository's generic name UUID helper intentionally preserves its
/// historical output because those values are also used by existing GATT
/// profiles. Mesh uses this dedicated wrapper because BlueZ rejects UUID byte
/// arrays whose RFC version and variant bits are not set.
/// \param value Stable name from which to derive a Mesh UUID.
/// \return Deterministic name-based 16-byte Mesh UUID.
std::vector<uint8_t> mesh_uuid_from_name(const std::string& value);
/// Automatic-enrollment UUID also carries the numeric UAV identity so an
/// intentionally open peer list can admit nearby UAVs without a name lookup.
/// \param hostname UAV hostname used to identify the node.
/// \return Deterministic automatic-provisioning UUID for the UAV hostname.
std::vector<uint8_t> mesh_auto_uuid_from_name(const std::string& hostname);
/// \brief Recover the UAV number embedded in a deterministic automatic Mesh UUID.
/// \param uuid Sixteen-byte Mesh UUID whose numeric node suffix is decoded.
/// \return UAV number embedded in the automatic Mesh UUID.
uint16_t mesh_auto_uuid_number(const std::vector<uint8_t>& uuid);

/// Render a byte vector as lowercase hexadecimal without separators.
/// \param value Bytes to render as lowercase hexadecimal.
/// \return Lowercase hexadecimal representation of the Mesh bytes.
std::string mesh_bytes_to_hex(const std::vector<uint8_t>& value);

}  // namespace mrs_uav_bluetooth::mesh
