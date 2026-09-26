// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/mesh/mesh_application.hpp"

#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <sys/stat.h>
#include <sys/random.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace mrs_uav_bluetooth::mesh {

namespace {

constexpr auto kCallTimeout = std::chrono::seconds(30);
constexpr auto kDaemonActivationRetry = std::chrono::seconds(5);

using SigModel = sdbus::Struct<uint16_t, VariantMap>;
using VendorModel = sdbus::Struct<uint16_t, uint16_t, VariantMap>;
using NetKeyRecord = sdbus::Struct<uint16_t, std::vector<uint8_t>, VariantMap>;
using AppKeyRecord = sdbus::Struct<uint16_t, std::vector<uint8_t>, VariantMap>;
using DevKeyRecord = sdbus::Struct<uint16_t, std::vector<uint8_t>>;

template<typename T>
std::optional<T> variant_value(const VariantMap& values, const std::string& key) {
    const auto it = values.find(key);
    if (it == values.end() || !it->second.containsValueOfType<T>()) {
        return std::nullopt;
    }
    return it->second.get<T>();
}

std::string normalized_uuid_hex(const std::string& value) {
    std::string result;
    result.reserve(32);
    for (const unsigned char ch : value) {
        if (ch == '-') continue;
        if (!std::isxdigit(ch)) {
            throw std::invalid_argument("mesh UUID contains a non-hexadecimal character");
        }
        result.push_back(static_cast<char>(std::tolower(ch)));
    }
    if (result.size() != 32) {
        throw std::invalid_argument("mesh UUID must contain exactly 16 bytes");
    }
    return result;
}

std::string mesh_error_text(const sdbus::Error& error) {
    const auto name = static_cast<std::string>(error.getName());
    const auto message = error.getMessage();
    return name.empty() ? message : name + ": " + message;
}

void clear_node_runtime_status(Status& status) {
    status.addresses.clear();
    status.friend_feature = false;
    status.low_power_feature = false;
    status.proxy_feature = false;
    status.relay_feature = false;
    status.beacon = false;
    status.iv_update = false;
    status.iv_index = 0;
    status.seconds_since_last_heard = 0;
    status.sequence_number = 0;
}

}  // namespace

std::vector<uint8_t> mesh_uuid_from_string(const std::string& value) {
    const auto normalized = normalized_uuid_hex(value);
    std::vector<uint8_t> result;
    result.reserve(16);
    for (size_t index = 0; index < normalized.size(); index += 2) {
        result.push_back(static_cast<uint8_t>(
            std::stoul(normalized.substr(index, 2), nullptr, 16)));
    }
    return result;
}

std::vector<uint8_t> mesh_uuid_from_name(const std::string& value) {
    auto result = mesh_uuid_from_string(util::uuid_from_name(value));
    // RFC 4122 section 4.1.3: deterministic MD5 names are version 3 and use
    // the RFC variant. BlueZ's l_uuid_is_valid() enforces both fields.
    result[6] = static_cast<uint8_t>((result[6] & 0x0fU) | 0x30U);
    result[8] = static_cast<uint8_t>((result[8] & 0x3fU) | 0x80U);
    return result;
}

std::vector<uint8_t> mesh_auto_uuid_from_name(const std::string& hostname) {
    const auto first_digit = hostname.find_last_not_of("0123456789");
    if (first_digit != std::string::npos && first_digit + 1 == hostname.size())
        throw std::invalid_argument("Automatic Mesh hostname needs a UAV number");
    const auto number = std::stoul(first_digit == std::string::npos
        ? hostname : hostname.substr(first_digit + 1));
    if (number == 0 || number > 0x7fff)
        throw std::invalid_argument("Automatic Mesh UAV number must be in 1..32767");
    auto uuid = mesh_uuid_from_name("mrs-uav-bluetooth-mesh-auto:" + hostname);
    uuid[0] = 'M';
    uuid[1] = 'R';
    uuid[2] = 'S';
    uuid[3] = 'B';
    uuid[4] = static_cast<uint8_t>(number);
    uuid[5] = static_cast<uint8_t>(number >> 8U);
    return uuid;
}

uint16_t mesh_auto_uuid_number(const std::vector<uint8_t>& uuid) {
    if (uuid.size() != 16 || uuid[0] != 'M' || uuid[1] != 'R' ||
        uuid[2] != 'S' || uuid[3] != 'B')
        return 0;
    const auto number = static_cast<uint16_t>(
        uuid[4] | (static_cast<uint16_t>(uuid[5]) << 8U));
    return number > 0 && number <= 0x7fff ? number : 0;
}

std::string mesh_bytes_to_hex(const std::vector<uint8_t>& value) {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (const auto byte : value) {
        stream << std::setw(2) << static_cast<unsigned>(byte);
    }
    return stream.str();
}

MeshApplication::MeshApplication(bluez::DbusConnection& dbus,
                                 config::NodeConfig config,
                                 std::string hostname,
                                 rclcpp::Logger logger)
    : dbus_(dbus),
      config_(std::move(config)),
      hostname_(std::move(hostname)),
      logger_(logger),
      next_unicast_(config_.mesh_next_unicast) {
    unicast_block_end_ = 0x7fff;

    // Allocation must survive process restarts. Reusing a unicast address for
    // a different node would corrupt transport replay protection and make
    // provisioner failover unsafe.
    if (!config_.mesh_unicast_cursor_path.empty()) {
        std::ifstream cursor_file(config_.mesh_unicast_cursor_path);
        std::string cursor_text;
        if (cursor_file >> cursor_text) {
            try {
                const auto persisted = std::stoul(cursor_text, nullptr, 0);
                if (persisted >= config_.mesh_next_unicast &&
                    persisted <= static_cast<uint32_t>(unicast_block_end_) + 1U) {
                    next_unicast_ = static_cast<uint16_t>(persisted);
                }
            } catch (...) {
                RCLCPP_WARN(logger_, "Ignoring invalid Mesh unicast cursor file %s",
                            config_.mesh_unicast_cursor_path.c_str());
            }
        }
    }

    if (config_.mesh_device_uuid.empty()) {
        const std::string namespace_name =
            config_.mesh_swarm_auto_provisioning && config_.mesh_fleet_id.empty()
                ? "mrs-uav-bluetooth-mesh-auto:"
                : "mrs-uav-bluetooth-mesh:" +
                    (config_.mesh_fleet_id.empty() ? std::string{} :
                     config_.mesh_fleet_id + ":");
        uuid_ = config_.mesh_swarm_auto_provisioning &&
            config_.mesh_fleet_id.empty()
                ? mesh_auto_uuid_from_name(hostname_)
                : mesh_uuid_from_name(namespace_name + hostname_);
    } else {
        uuid_ = mesh_uuid_from_string(config_.mesh_device_uuid);
        const auto version = static_cast<uint8_t>(uuid_[6] >> 4);
        const auto variant = static_cast<uint8_t>(uuid_[8] >> 6);
        if (version < 1 || version > 5 || variant != 2) {
            throw std::invalid_argument(
                "mesh_device_uuid must use RFC 4122 version and variant bits");
        }
    }

    status_.uuid = uuid_hex();
    status_.state = "configured";
    status_.token = config_.mesh_token;
    if (status_.token == 0 && !config_.mesh_token_path.empty()) {
        std::ifstream token_file(config_.mesh_token_path);
        std::string token_text;
        if (token_file >> token_text) {
            try {
                size_t parsed = 0;
                status_.token = std::stoull(token_text, &parsed, 16);
                if (parsed != token_text.size()) status_.token = 0;
            } catch (...) {
                status_.token = 0;
                RCLCPP_WARN(logger_, "Ignoring invalid Mesh token file %s",
                            config_.mesh_token_path.c_str());
            }
        }
    }
    if (status_.token != 0) status_.state = "provisioned";
}

MeshApplication::~MeshApplication() {
    // Cancel callback delivery before unexporting objects captured by pending
    // Join/CreateNetwork operations.
    join_call_slot_.reset();
    create_call_slot_.reset();
    add_node_call_slot_.reset();
    object_manager_slot_.reset();
    element_object_.reset();
    agent_object_.reset();
    application_object_.reset();
    root_object_.reset();
}

void MeshApplication::export_objects() {
    if (application_object_) return;

    // Keep ObjectManager at app_root and every advertised Mesh interface on a
    // managed child. BlueZ discovers Application1 by scanning the dictionary
    // returned by app_root.GetManagedObjects(); sdbus-c++ correctly reports
    // descendants but does not include interfaces attached to app_root itself.
    root_object_ = sdbus::createObject(
        dbus_.connection(), sdbus::ObjectPath{root_path_});
    application_object_ = sdbus::createObject(
        dbus_.connection(), sdbus::ObjectPath{application_path_});
    export_application();
    export_attention();
    if (config_.mesh_provisioner) export_provisioner();
    export_agent();
    export_element();
    object_manager_slot_.emplace(
        root_object_->addObjectManager(sdbus::return_slot));

    std::lock_guard<std::mutex> lock(mutex_);
    if (status_.state == "configured") status_.state = "exported";
}

void MeshApplication::export_application() {
    application_object_->addVTable(
        sdbus::registerProperty("CompanyID")
            .withGetter([this]() { return config_.mesh_company_id; }),
        sdbus::registerProperty("ProductID")
            .withGetter([this]() { return config_.mesh_product_id; }),
        sdbus::registerProperty("VersionID")
            .withGetter([this]() { return config_.mesh_version_id; }),
        sdbus::registerProperty("CRPL")
            .withGetter([this]() { return config_.mesh_crpl; }),
        sdbus::registerMethod("JoinComplete")
            .withInputParamNames("token")
            .implementedAs([this](uint64_t token) {
                set_token(token);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    status_.state = "provisioned";
                    status_.error.clear();
                }
                Event event;
                event.event = "join_complete";
                event.uuid = uuid_hex();
                emit_event(std::move(event));
            }),
        sdbus::registerMethod("JoinFailed")
            .withInputParamNames("reason")
            .implementedAs([this](const std::string& reason) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    status_.state = "join_failed";
                    status_.error = reason;
                }
                Event event;
                event.event = "join_failed";
                event.uuid = uuid_hex();
                event.reason = reason;
                emit_event(std::move(event));
            })
    ).forInterface(kMeshApplicationInterface);
}

void MeshApplication::export_agent() {
    agent_object_ = sdbus::createObject(
        dbus_.connection(), sdbus::ObjectPath{agent_path_});
    agent_object_->addVTable(
        sdbus::registerProperty("Capabilities")
            .withGetter([this]() { return config_.mesh_agent_capabilities; }),
        sdbus::registerProperty("OutOfBandInfo")
            .withGetter([this]() { return config_.mesh_agent_oob_info; }),
        sdbus::registerProperty("URI")
            .withGetter([this]() { return config_.mesh_agent_uri; }),
        sdbus::registerMethod("PrivateKey")
            .withOutputParamNames("key")
            .implementedAs([this]() -> std::vector<uint8_t> {
                require_size(config_.mesh_agent_private_key, 32, "mesh agent private key");
                return config_.mesh_agent_private_key;
            }),
        sdbus::registerMethod("PublicKey")
            .withOutputParamNames("key")
            .implementedAs([this]() -> std::vector<uint8_t> {
                require_size(config_.mesh_agent_public_key, 64, "mesh agent public key");
                return config_.mesh_agent_public_key;
            }),
        sdbus::registerMethod("DisplayString")
            .withInputParamNames("value")
            .implementedAs([this](const std::string& value) {
                Event event;
                event.event = "agent_display_string";
                event.detail = value;
                emit_event(std::move(event));
            }),
        sdbus::registerMethod("DisplayNumeric")
            .withInputParamNames("type", "number")
            .implementedAs([this](const std::string& type, uint32_t number) {
                Event event;
                event.event = "agent_display_numeric";
                event.detail = type + ":" + std::to_string(number);
                emit_event(std::move(event));
            }),
        sdbus::registerMethod("PromptNumeric")
            .withInputParamNames("type")
            .withOutputParamNames("number")
            .implementedAs([this](const std::string& type) -> uint32_t {
                Event event;
                event.event = "agent_prompt_numeric";
                event.detail = type;
                emit_event(std::move(event));
                return config_.mesh_agent_numeric_oob;
            }),
        sdbus::registerMethod("PromptStatic")
            .withInputParamNames("type")
            .withOutputParamNames("value")
            .implementedAs([this](const std::string& type) -> std::vector<uint8_t> {
                Event event;
                event.event = "agent_prompt_static";
                event.detail = type;
                emit_event(std::move(event));
                require_size(config_.mesh_agent_static_oob, 16, "mesh agent static OOB");
                return config_.mesh_agent_static_oob;
            }),
        sdbus::registerMethod("Cancel")
            .implementedAs([this]() {
                Event event;
                event.event = "agent_cancel";
                emit_event(std::move(event));
            })
    ).forInterface(kMeshProvisionAgentInterface);
}

void MeshApplication::export_element() {
    element_object_ = sdbus::createObject(
        dbus_.connection(), sdbus::ObjectPath{element_path_});
    element_object_->addVTable(
        sdbus::registerProperty("Index")
            .withGetter([]() -> uint8_t { return 0; }),
        sdbus::registerProperty("Models")
            .withGetter([this]() {
                std::vector<SigModel> models;
                if (config_.mesh_provisioner ||
                    config_.mesh_auto_configure_relay) {
                    models.emplace_back(uint16_t{0x0001}, VariantMap{});
                }
                return models;
            }),
        sdbus::registerProperty("VendorModels")
            .withGetter([this]() {
                return std::vector<VendorModel>{VendorModel{
                    config_.mesh_company_id,
                    config_.mesh_vendor_model_id,
                    VariantMap{}}};
            }),
        sdbus::registerProperty("Location")
            .withGetter([]() -> uint16_t { return 0; }),
        sdbus::registerMethod("MessageReceived")
            .withInputParamNames("source", "key_index", "destination", "data")
            .implementedAs([this](uint16_t source, uint16_t key_index,
                                  const sdbus::Variant& destination,
                                  const std::vector<uint8_t>& data) {
                ReceivedMessage message;
                message.source = source;
                message.key_index = key_index;
                message.data = data;
                if (destination.containsValueOfType<uint16_t>()) {
                    message.destination = destination.get<uint16_t>();
                }
                emit_message(std::move(message));
            }),
        sdbus::registerMethod("DevKeyMessageReceived")
            .withInputParamNames("source", "remote", "net_index", "data")
            .implementedAs([this](uint16_t source, bool remote, uint16_t net_index,
                                  const std::vector<uint8_t>& data) {
                ReceivedMessage message;
                message.source = source;
                message.net_index = net_index;
                message.device_key = true;
                message.remote = remote;
                message.data = data;
                emit_message(std::move(message));
            }),
        sdbus::registerMethod("UpdateModelConfiguration")
            .withInputParamNames("model_id", "config")
            .implementedAs([this](uint16_t model_id, const VariantMap& model_config) {
                update_model_configuration(model_id, model_config);
                Event event;
                event.event = "model_configuration";
                event.detail = "model=" + std::to_string(model_id) +
                    " fields=" + std::to_string(model_config.size());
                emit_event(std::move(event));
            })
    ).forInterface(kMeshElementInterface);
}

void MeshApplication::export_attention() {
    application_object_->addVTable(
        sdbus::registerMethod("SetTimer")
            .withInputParamNames("element_index", "time")
            .implementedAs([this](uint8_t element_index, uint16_t seconds) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    attention_until_ = std::chrono::steady_clock::now() +
                        std::chrono::seconds(seconds);
                }
                Event event;
                event.event = "attention";
                event.detail = "element=" + std::to_string(element_index) +
                    " seconds=" + std::to_string(seconds);
                emit_event(std::move(event));
            }),
        sdbus::registerMethod("GetTimer")
            .withInputParamNames("element")
            .withOutputParamNames("time")
            .implementedAs([this](uint16_t) -> uint16_t {
                std::lock_guard<std::mutex> lock(mutex_);
                const auto now = std::chrono::steady_clock::now();
                if (attention_until_ <= now) return 0;
                const auto remaining = std::chrono::duration_cast<std::chrono::seconds>(
                    attention_until_ - now).count();
                return static_cast<uint16_t>(std::min<int64_t>(
                    remaining + 1, std::numeric_limits<uint16_t>::max()));
            })
    ).forInterface(kMeshAttentionInterface);
}

void MeshApplication::export_provisioner() {
    application_object_->addVTable(
        sdbus::registerMethod("ScanResult")
            .withInputParamNames("rssi", "data", "options")
            .implementedAs([this](int16_t rssi, const std::vector<uint8_t>& data,
                                  const VariantMap& options) {
                Event event;
                event.event = "scan_result";
                event.rssi = rssi;
                event.data = data;
                if (data.size() >= 16) {
                    event.uuid = mesh_bytes_to_hex(
                        std::vector<uint8_t>(data.begin(), data.begin() + 16));
                }
                if (const auto server = variant_value<uint16_t>(options, "Server")) {
                    event.server = *server;
                }
                emit_event(std::move(event));
            }),
        sdbus::registerMethod("RequestProvData")
            .withInputParamNames("count")
            .withOutputParamNames("net_index", "unicast")
            .implementedAs([this](uint8_t count) -> std::tuple<uint16_t, uint16_t> {
                std::lock_guard<std::mutex> lock(mutex_);
                if (config_.mesh_swarm_auto_provisioning &&
                    config_.mesh_fleet_id.empty()) {
                    if (count != 1 || auto_provisioning_unicast_ == 0)
                        throw sdbus::Error(
                            sdbus::Error::Name{"org.bluez.mesh.Error.Abort"},
                            "No unique UAV address is assigned to this provisioning request");
                    const uint16_t assigned = auto_provisioning_unicast_;
                    auto_provisioning_unicast_ = 0;
                    return {config_.mesh_swarm_network_index, assigned};
                }
                if (count == 0 || next_unicast_ > unicast_block_end_ ||
                    static_cast<uint32_t>(next_unicast_) + count - 1 >
                        unicast_block_end_) {
                    throw sdbus::Error(
                        sdbus::Error::Name{"org.bluez.mesh.Error.Abort"},
                        "This provisioner's unicast allocation block is exhausted");
                }
                const uint16_t assigned = next_unicast_;
                next_unicast_ = static_cast<uint16_t>(next_unicast_ + count);
                try {
                    persist_next_unicast();
                } catch (const std::exception& error) {
                    throw sdbus::Error(
                        sdbus::Error::Name{"org.bluez.mesh.Error.Abort"}, error.what());
                }
                return {config_.mesh_swarm_network_index, assigned};
            }),
        sdbus::registerMethod("RequestReprovData")
            .withInputParamNames("original", "count")
            .withOutputParamNames("unicast")
            .implementedAs([this](uint16_t original, uint8_t count) -> uint16_t {
                if (count == 0 || original == 0 ||
                    static_cast<uint32_t>(original) + count - 1 > 0x7fff) {
                    throw sdbus::Error(
                        sdbus::Error::Name{"org.bluez.mesh.Error.Abort"},
                        "Invalid reprovisioned unicast range");
                }
                return original;
            }),
        sdbus::registerMethod("AddNodeComplete")
            .withInputParamNames("uuid", "unicast", "count")
            .implementedAs([this](const std::vector<uint8_t>& uuid,
                                  uint16_t unicast, uint8_t count) {
                Event event;
                event.event = "add_node_complete";
                event.uuid = mesh_bytes_to_hex(uuid);
                event.unicast = unicast;
                event.count = count;
                emit_event(std::move(event));
            }),
        sdbus::registerMethod("ReprovComplete")
            .withInputParamNames("original", "nppi", "unicast", "count")
            .implementedAs([this](uint16_t original, uint8_t nppi,
                                  uint16_t unicast, uint8_t count) {
                Event event;
                event.event = "reprovision_complete";
                event.original = original;
                event.nppi = nppi;
                event.unicast = unicast;
                event.count = count;
                emit_event(std::move(event));
            }),
        sdbus::registerMethod("AddNodeFailed")
            .withInputParamNames("uuid", "reason")
            .implementedAs([this](const std::vector<uint8_t>& uuid,
                                  const std::string& reason) {
                Event event;
                event.event = "add_node_failed";
                event.uuid = mesh_bytes_to_hex(uuid);
                event.reason = reason;
                emit_event(std::move(event));
            }),
        sdbus::registerMethod("ReprovFailed")
            .withInputParamNames("unicast", "reason")
            .implementedAs([this](uint16_t unicast, const std::string& reason) {
                Event event;
                event.event = "reprovision_failed";
                event.unicast = unicast;
                event.reason = reason;
                emit_event(std::move(event));
            })
    ).forInterface(kMeshProvisionerInterface);
}

bool MeshApplication::daemon_available() const {
    try {
        auto proxy = sdbus::createProxy(
            dbus_.connection(),
            sdbus::ServiceName{"org.freedesktop.DBus"},
            sdbus::ObjectPath{"/org/freedesktop/DBus"});
        bool owner = false;
        proxy->callMethod("NameHasOwner")
            .onInterface("org.freedesktop.DBus")
            .withArguments(std::string{kMeshService})
            .storeResultsTo(owner);
        return owner;
    } catch (...) {
        return false;
    }
}

bool MeshApplication::request_daemon_activation() {
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (last_daemon_activation_attempt_ != std::chrono::steady_clock::time_point{} &&
            now - last_daemon_activation_attempt_ < kDaemonActivationRetry) {
            return false;
        }
        last_daemon_activation_attempt_ = now;
    }

    try {
        auto proxy = sdbus::createProxy(
            dbus_.connection(),
            sdbus::ServiceName{"org.freedesktop.DBus"},
            sdbus::ObjectPath{"/org/freedesktop/DBus"});
        uint32_t result = 0;
        proxy->callMethod("StartServiceByName")
            .onInterface("org.freedesktop.DBus")
            .withArguments(std::string{kMeshService}, uint32_t{0})
            .storeResultsTo(result);
        // D-Bus returns 1 when it started the service and 2 when another
        // caller already did. Treat both as success and verify ownership in
        // refresh_status() instead of relying on implementation-specific text.
        RCLCPP_INFO(logger_, "Requested bluetooth-meshd D-Bus activation (result=%u)",
                    result);
        return true;
    } catch (const sdbus::Error& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.error = "could not activate bluetooth-meshd: " +
            mesh_error_text(error);
        return false;
    }
}

void MeshApplication::refresh_status() {
    bool available = daemon_available();
    if (!available) {
        request_daemon_activation();
        available = daemon_available();
    }

    std::string node_path;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.daemon_available = available;
        node_path = status_.node_path;
        if (!available) {
            status_.attached = false;
            status_.node_path.clear();
            clear_node_runtime_status(status_);
            if (status_.state != "joining" && status_.state != "creating") {
                status_.state = "daemon_unavailable";
            }
            return;
        }
        if (status_.state == "daemon_unavailable") {
            status_.state = status_.token == 0 ? "exported" : "provisioned";
            status_.error.clear();
        }
        if (node_path.empty()) return;
    }

    try {
        VariantMap properties;
        auto proxy = node_proxy();
        proxy->callMethod("GetAll")
            .onInterface(std::string(bluez::kDbusPropertiesIface))
            .withArguments(std::string{kMeshNodeInterface})
            .storeResultsTo(properties);

        std::lock_guard<std::mutex> lock(mutex_);
        status_.attached = true;
        status_.state = "attached";
        clear_node_runtime_status(status_);
        status_.error.clear();
        if (const auto value = variant_value<std::vector<uint16_t>>(properties, "Addresses"))
            status_.addresses = *value;
        if (const auto value = variant_value<bool>(properties, "Beacon"))
            status_.beacon = *value;
        if (const auto value = variant_value<bool>(properties, "IvUpdate"))
            status_.iv_update = *value;
        if (const auto value = variant_value<uint32_t>(properties, "IvIndex"))
            status_.iv_index = *value;
        if (const auto value = variant_value<uint32_t>(properties, "SecondsSinceLastHeard"))
            status_.seconds_since_last_heard = *value;
        if (const auto value = variant_value<uint32_t>(properties, "SequenceNumber"))
            status_.sequence_number = *value;
        if (const auto features = variant_value<VariantMap>(properties, "Features")) {
            if (const auto value = variant_value<bool>(*features, "Friend"))
                status_.friend_feature = *value;
            if (const auto value = variant_value<bool>(*features, "LowPower"))
                status_.low_power_feature = *value;
            if (const auto value = variant_value<bool>(*features, "Proxy"))
                status_.proxy_feature = *value;
            if (const auto value = variant_value<bool>(*features, "Relay"))
                status_.relay_feature = *value;
        }
    } catch (const sdbus::Error& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.error = mesh_error_text(error);
        const auto name = static_cast<std::string>(error.getName());
        if (name.find("UnknownObject") != std::string::npos ||
            name.find("NotFound") != std::string::npos ||
            name.find("ServiceUnknown") != std::string::npos) {
            status_.attached = false;
            status_.node_path.clear();
            clear_node_runtime_status(status_);
            status_.state = status_.token == 0 ? "exported" : "provisioned";
        }
    }
}

void MeshApplication::update_model_configuration(
    uint16_t model_id, const VariantMap& config) {
    const auto vendor = variant_value<uint16_t>(config, "Vendor");
    if (!vendor || *vendor != config_.mesh_company_id ||
        model_id != config_.mesh_vendor_model_id) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (const auto bindings = variant_value<std::vector<uint16_t>>(config, "Bindings"))
        vendor_bound_ = std::find(bindings->begin(), bindings->end(),
            config_.mesh_swarm_app_key_index) != bindings->end();
    if (const auto subscriptions = variant_value<std::vector<sdbus::Variant>>(config, "Subscriptions")) {
        vendor_subscribed_ = std::any_of(subscriptions->begin(), subscriptions->end(),
            [this](const auto& value) {
                return value.template containsValueOfType<uint16_t>() &&
                    value.template get<uint16_t>() == config_.mesh_swarm_group_address;
            });
    }
}

bool MeshApplication::vendor_model_ready() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_.attached && vendor_bound_ && vendor_subscribed_;
}

std::vector<uint8_t> MeshApplication::fleet_device_key() {
    const auto path = config_.mesh_token_path + ".device-key";
    // Never generate replacement identity material for an already enrolled
    // node: an absent backup requires recovery, not resetting replay state.
    if (!std::filesystem::exists(path) && token() != 0)
        throw std::runtime_error("Fleet Device Key backup missing; restore it before configuring this identity");
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
    if (fd >= 0) {
        std::vector<uint8_t> fresh(16);
        size_t offset = 0;
        while (offset < fresh.size()) {
            const auto count = ::getrandom(fresh.data() + offset, fresh.size() - offset, 0);
            if (count < 0 && errno == EINTR) continue;
            if (count <= 0) { ::close(fd); throw std::runtime_error("Secure random source failed"); }
            offset += static_cast<size_t>(count);
        }
        const bool saved = ::write(fd, fresh.data(), fresh.size()) ==
            static_cast<ssize_t>(fresh.size()) && ::fsync(fd) == 0;
        ::close(fd);
        if (!saved) throw std::runtime_error("Could not persist fleet Device Key");
    } else if (errno != EEXIST) {
        throw std::runtime_error("Could not create private fleet Device Key file");
    }
    const auto permissions = std::filesystem::status(path).permissions();
    if ((permissions & (std::filesystem::perms::group_all |
                        std::filesystem::perms::others_all)) != std::filesystem::perms::none)
        throw std::runtime_error("Fleet Device Key backup must have mode 0600");
    std::ifstream input(path, std::ios::binary);
    std::vector<uint8_t> result{std::istreambuf_iterator<char>(input), {}};
    require_size(result, 16, "persisted fleet Device Key");
    return result;
}

void MeshApplication::prepare_fleet_keys() {
    // Network1.Import creates operational keys, not the Configuration Client
    // keyring. ImportSubnet is idempotent for identical material in BlueZ.
    import_subnet(config_.mesh_swarm_network_index, config_.mesh_fleet_network_key);
    import_remote_node(config_.mesh_fleet_unicast, 1, fleet_device_key());
    VariantMap exported;
    management_proxy()->callMethod("ExportKeys")
        .onInterface(kMeshManagementInterface).storeResultsTo(exported);
    bool found_network = false;
    bool found_application = false;
    if (const auto records = variant_value<std::vector<NetKeyRecord>>(exported, "NetKeys")) {
        for (const auto& record : *records) {
            if (std::get<0>(record) != config_.mesh_swarm_network_index) continue;
            found_network = true;
            if (std::get<1>(record) != config_.mesh_fleet_network_key)
                throw std::runtime_error("Stored Mesh NetKey differs from fleet credentials; use a new fleet for new keys");
            if (const auto apps = variant_value<std::vector<AppKeyRecord>>(std::get<2>(record), "AppKeys")) {
                for (const auto& app : *apps) {
                    if (std::get<0>(app) != config_.mesh_swarm_app_key_index) continue;
                    found_application = true;
                    if (std::get<1>(app) != config_.mesh_fleet_application_key)
                        throw std::runtime_error("Stored Mesh AppKey differs from fleet credentials");
                }
            }
        }
    }
    if (!found_network) throw std::runtime_error("Fleet NetKey is missing from the BlueZ database");
    if (!found_application)
        import_app_key(config_.mesh_swarm_network_index,
            config_.mesh_swarm_app_key_index, config_.mesh_fleet_application_key);
}

namespace {

/// Return the once-generated private NetKey and AppKey for a creator node.
/// A short or exposed file fails closed instead of silently changing a live
/// Mesh identity. Joined nodes never create this file.
std::vector<uint8_t> auto_credentials(const std::string& path, bool create) {
    if (create) {
        const std::filesystem::path target(path);
        std::filesystem::create_directories(target.parent_path());
        const int fd = ::open(path.c_str(),
            O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (fd >= 0) {
            std::vector<uint8_t> fresh(32);
            size_t offset = 0;
            while (offset < fresh.size()) {
                const auto count = ::getrandom(fresh.data() + offset,
                                                fresh.size() - offset, 0);
                if (count < 0 && errno == EINTR) continue;
                if (count <= 0) {
                    ::close(fd);
                    throw std::runtime_error("Secure Mesh random source failed");
                }
                offset += static_cast<size_t>(count);
            }
            const bool saved = ::write(fd, fresh.data(), fresh.size()) ==
                static_cast<ssize_t>(fresh.size()) && ::fsync(fd) == 0;
            ::close(fd);
            if (!saved) throw std::runtime_error("Could not persist automatic Mesh keys");
        } else if (errno != EEXIST) {
            throw std::runtime_error("Could not create private automatic Mesh keys");
        }
    }
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) throw std::runtime_error("Automatic Mesh key file is unavailable");
    struct stat details{};
    if (::fstat(fd, &details) != 0 || !S_ISREG(details.st_mode) ||
        (details.st_mode & 0077) != 0 || details.st_size != 32) {
        ::close(fd);
        throw std::runtime_error("Automatic Mesh key file must be private and 32 bytes");
    }
    std::vector<uint8_t> secret(32);
    size_t offset = 0;
    while (offset < secret.size()) {
        const auto count = ::read(fd, secret.data() + offset, secret.size() - offset);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) {
            ::close(fd);
            throw std::runtime_error("Could not read automatic Mesh keys");
        }
        offset += static_cast<size_t>(count);
    }
    ::close(fd);
    return secret;
}

}  // namespace

void MeshApplication::import_auto_identity(uint16_t unicast) {
    if (unicast == 0 || unicast > 0x7fff)
        throw std::invalid_argument("Automatic Mesh address must be in 1..32767");
    const auto secret = auto_credentials(config_.mesh_token_path + ".credentials", true);
    import_node(uuid_, fleet_device_key(),
        std::vector<uint8_t>(secret.begin(), secret.begin() + 16),
        config_.mesh_swarm_network_index, false, false, 0, unicast);
}

bool MeshApplication::prepare_auto_keys(uint16_t unicast) {
    const auto credentials_path = config_.mesh_token_path + ".credentials";
    const bool creator = std::filesystem::exists(credentials_path);
    std::vector<uint8_t> creator_keys;
    if (creator) {
        creator_keys = auto_credentials(credentials_path, false);
        // Network1.Import installs the operational NetKey but does not create
        // Management1's keyring directory. ExportKeys fails outright until
        // that directory exists, so seed it before checking the snapshot.
        import_subnet(config_.mesh_swarm_network_index,
            std::vector<uint8_t>(creator_keys.begin(), creator_keys.begin() + 16));
        import_remote_node(unicast, 1, fleet_device_key());
    }

    VariantMap exported;
    management_proxy()->callMethod("ExportKeys")
        .onInterface(kMeshManagementInterface).storeResultsTo(exported);
    bool found_network = false;
    bool found_application = false;
    if (const auto records = variant_value<std::vector<NetKeyRecord>>(exported, "NetKeys")) {
        for (const auto& record : *records) {
            if (std::get<0>(record) != config_.mesh_swarm_network_index) continue;
            found_network = true;
            if (creator && !std::equal(
                    std::get<1>(record).begin(), std::get<1>(record).end(),
                    creator_keys.begin(), creator_keys.begin() + 16))
                throw std::runtime_error(
                    "Stored Mesh network differs from this UAV's private creator keys");
            if (const auto apps = variant_value<std::vector<AppKeyRecord>>(
                    std::get<2>(record), "AppKeys")) {
                for (const auto& app : *apps) {
                    if (std::get<0>(app) != config_.mesh_swarm_app_key_index) continue;
                    found_application = true;
                    if (creator && !std::equal(
                            std::get<1>(app).begin(), std::get<1>(app).end(),
                            creator_keys.begin() + 16, creator_keys.end()))
                        throw std::runtime_error(
                            "Stored Mesh AppKey differs from this UAV's private creator keys");
                }
            }
        }
    }
    if (creator) {
        if (!found_network)
            throw std::runtime_error("Automatic creator NetKey is missing");
        if (!found_application) {
            import_app_key(config_.mesh_swarm_network_index,
                config_.mesh_swarm_app_key_index,
                std::vector<uint8_t>(creator_keys.begin() + 16, creator_keys.end()));
        }
        return true;
    }
    // The provisioning Configuration Client installs the AppKey remotely.
    // The joining UAV must never invent a different key to appear ready.
    return found_network && found_application;
}

void MeshApplication::set_auto_provisioning_unicast(uint16_t unicast) {
    if (unicast == 0 || unicast > 0x7fff)
        throw std::invalid_argument("Automatic provisioning UAV number must be in 1..32767");
    std::lock_guard<std::mutex> lock(mutex_);
    auto_provisioning_unicast_ = unicast;
}

Status MeshApplication::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

void MeshApplication::set_message_callback(MessageCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    message_callback_ = std::move(callback);
}

void MeshApplication::set_event_callback(EventCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    event_callback_ = std::move(callback);
}

void MeshApplication::set_token_callback(TokenCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    token_callback_ = std::move(callback);
}

std::string MeshApplication::uuid_hex() const {
    return mesh_bytes_to_hex(uuid_);
}

uint64_t MeshApplication::token() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_.token;
}

std::string MeshApplication::element_path(uint8_t index) const {
    if (index != 0) throw std::invalid_argument("only Mesh element index 0 exists");
    return element_path_;
}

std::unique_ptr<sdbus::IProxy> MeshApplication::network_proxy() const {
    return sdbus::createProxy(dbus_.connection(), sdbus::ServiceName{kMeshService},
                              sdbus::ObjectPath{"/org/bluez/mesh"});
}

std::unique_ptr<sdbus::IProxy> MeshApplication::node_proxy() const {
    std::string node_path;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        node_path = status_.node_path;
    }
    if (node_path.empty()) throw std::runtime_error("Mesh node is not attached");
    return sdbus::createProxy(dbus_.connection(), sdbus::ServiceName{kMeshService},
                              sdbus::ObjectPath{node_path});
}

std::unique_ptr<sdbus::IProxy> MeshApplication::management_proxy() const {
    return node_proxy();
}

void MeshApplication::join(const std::vector<uint8_t>& uuid) {
    const auto& selected = uuid.empty() ? uuid_ : uuid;
    require_size(selected, 16, "mesh UUID");

    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.state = "joining";
        status_.error.clear();
    }

    try {
        // Join is deliberately asynchronous. BlueZ may keep its method reply
        // pending while it creates and introspects the temporary node; blocking
        // here would also block the ROS overlay service and expire its lease.
        auto proxy = std::shared_ptr<sdbus::IProxy>(network_proxy().release());
        join_call_slot_.emplace(
            proxy->callMethodAsync("Join")
                .onInterface(kMeshNetworkInterface)
                .withTimeout(kCallTimeout)
                .withArguments(sdbus::ObjectPath{root_path_}, selected)
                .uponReplyInvoke(
                    [this, proxy](std::optional<sdbus::Error> error) {
                        (void)proxy;
                        if (!error) return;
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            if (status_.state == "joining") {
                                status_.state = status_.token == 0 ? "exported" : "provisioned";
                                status_.error = mesh_error_text(*error);
                            }
                        }
                        Event event;
                        event.event = "join_failed";
                        event.reason = mesh_error_text(*error);
                        emit_event(std::move(event));
                    },
                    sdbus::return_slot));
    } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.state = status_.token == 0 ? "exported" : "provisioned";
        throw;
    }
}

void MeshApplication::cancel_join() {
    network_proxy()->callMethod("Cancel").onInterface(kMeshNetworkInterface);
    // Cancel() completes the daemon-side Join call. We no longer need its
    // expected failure reply, and dropping the slot prevents a stale callback
    // from overwriting the following CreateNetwork state.
    join_call_slot_.reset();
    std::lock_guard<std::mutex> lock(mutex_);
    status_.state = status_.token == 0 ? "exported" : "provisioned";
}

void MeshApplication::attach(uint64_t selected_token) {
    if (selected_token == 0) selected_token = token();
    if (selected_token == 0) throw std::invalid_argument("Mesh token is not set");

    auto proxy = network_proxy();
    auto future = proxy->callMethodAsync("Attach")
        .onInterface(kMeshNetworkInterface)
        .withTimeout(kCallTimeout)
        .withArguments(sdbus::ObjectPath{root_path_}, selected_token)
        .getResultAsFuture<sdbus::ObjectPath, std::vector<ElementConfiguration>>();
    if (future.wait_for(kCallTimeout) != std::future_status::ready) {
        throw std::runtime_error("Mesh Attach timed out");
    }
    auto [node, configuration] = future.get();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        vendor_bound_ = false;
        vendor_subscribed_ = false;
    }
    for (const auto& element : configuration) {
        if (std::get<0>(element) != 0) continue;
        for (const auto& model : std::get<1>(element))
            update_model_configuration(std::get<0>(model), std::get<1>(model));
    }
    set_token(selected_token);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.node_path = static_cast<std::string>(node);
        status_.attached = true;
        status_.state = "attached";
        status_.error.clear();
    }
    refresh_status();
}

void MeshApplication::leave(uint64_t selected_token) {
    if (selected_token == 0) selected_token = token();
    if (selected_token == 0) throw std::invalid_argument("Mesh token is not set");
    network_proxy()->callMethod("Leave")
        .onInterface(kMeshNetworkInterface)
        .withArguments(selected_token);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.token = 0;
        status_.node_path.clear();
        status_.addresses.clear();
        status_.attached = false;
        status_.state = "exported";
        status_.error.clear();
    }
    if (!config_.mesh_token_path.empty()) {
        std::error_code ec;
        std::filesystem::remove(config_.mesh_token_path, ec);
    }
}

void MeshApplication::create_network(const std::vector<uint8_t>& uuid) {
    if (!config_.mesh_provisioner) {
        throw std::runtime_error("mesh_provisioner must be enabled to create a network");
    }
    const auto& selected = uuid.empty() ? uuid_ : uuid;
    require_size(selected, 16, "mesh UUID");

    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.state = "creating";
        status_.error.clear();
    }

    try {
        // Node creation and application introspection are asynchronous inside
        // bluetooth-meshd. JoinComplete supplies the persistent token later.
        auto proxy = std::shared_ptr<sdbus::IProxy>(network_proxy().release());
        create_call_slot_.emplace(
            proxy->callMethodAsync("CreateNetwork")
                .onInterface(kMeshNetworkInterface)
                .withTimeout(kCallTimeout)
                .withArguments(sdbus::ObjectPath{root_path_}, selected)
                .uponReplyInvoke(
                    [this, proxy](std::optional<sdbus::Error> error) {
                        (void)proxy;
                        if (!error) return;
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            if (status_.state == "creating") {
                                status_.state = status_.token == 0 ? "exported" : "provisioned";
                                status_.error = mesh_error_text(*error);
                            }
                        }
                        Event event;
                        event.event = "join_failed";
                        event.reason = mesh_error_text(*error);
                        emit_event(std::move(event));
                    },
                    sdbus::return_slot));
    } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.state = status_.token == 0 ? "exported" : "provisioned";
        throw;
    }
}

void MeshApplication::import_node(const std::vector<uint8_t>& uuid,
                                  const std::vector<uint8_t>& device_key,
                                  const std::vector<uint8_t>& network_key,
                                  uint16_t network_index,
                                  bool iv_update,
                                  bool key_refresh,
                                  uint32_t iv_index,
                                  uint16_t unicast) {
    require_size(uuid, 16, "mesh UUID");
    require_size(device_key, 16, "device key");
    require_size(network_key, 16, "network key");
    if (network_index > 0x0fff) throw std::invalid_argument("network index exceeds 12 bits");
    if (unicast == 0 || unicast > 0x7fff) throw std::invalid_argument("invalid unicast address");
    VariantMap flags;
    flags.emplace("IvUpdate", sdbus::Variant{iv_update});
    flags.emplace("KeyRefresh", sdbus::Variant{key_refresh});
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.state = "importing";
        status_.error.clear();
    }
    // Like CreateNetwork, Import introspects our D-Bus objects and completes
    // through JoinComplete. Never block the ROS overlay lease timer on it.
    auto proxy = std::shared_ptr<sdbus::IProxy>(network_proxy().release());
    create_call_slot_.emplace(
        proxy->callMethodAsync("Import")
            .onInterface(kMeshNetworkInterface)
            .withTimeout(kCallTimeout)
            .withArguments(sdbus::ObjectPath{root_path_}, uuid, device_key,
                           network_key, network_index, flags, iv_index, unicast)
            .uponReplyInvoke(
                [this, proxy](std::optional<sdbus::Error> error) {
                    (void)proxy;
                    if (!error) return;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        status_.state = status_.token == 0 ? "exported" : "provisioned";
                        status_.error = mesh_error_text(*error);
                    }
                    Event event;
                    event.event = "join_failed";
                    event.reason = mesh_error_text(*error);
                    emit_event(std::move(event));
                }, sdbus::return_slot));
}

void MeshApplication::configure_local_relay(
    uint8_t default_ttl,
    uint8_t retransmit_count,
    uint8_t retransmit_interval_steps) {
    if (default_ttl == 1 || default_ttl > 127) {
        throw std::invalid_argument("default Mesh TTL must be 0 or in the range 2..127");
    }
    if (retransmit_count > 7) {
        throw std::invalid_argument("relay retransmit count must be in the range 0..7");
    }
    if (retransmit_interval_steps > 31) {
        throw std::invalid_argument(
            "relay retransmit interval steps must be in the range 0..31");
    }

    require_attached();
    const auto current_status = status();
    if (current_status.addresses.empty()) {
        throw std::runtime_error("attached Mesh node has no local unicast address");
    }

    const uint16_t primary_address = current_status.addresses.front();
    // Config Default TTL Set: two-octet opcode followed by the one-byte TTL.
    // BlueZ treats a Configuration Client -> local Configuration Server
    // loopback as remote-device-key traffic (see node.c DevKeySend). Passing
    // false for a local unicast address is rejected as InvalidArgs.
    dev_key_send(0, primary_address, true, config_.mesh_swarm_network_index, false,
                 {0x80, 0x0d, default_ttl});

    // Config Relay Set packs the three-bit retransmit count below the five-bit
    // 10 ms interval-step field. Relay state 0x01 means enabled.
    const uint8_t retransmit = static_cast<uint8_t>(
        retransmit_count | (retransmit_interval_steps << 3));
    dev_key_send(0, primary_address, true, config_.mesh_swarm_network_index, false,
                 {0x80, 0x27, 0x01, retransmit});
}

void MeshApplication::configure_vendor_model(
    uint16_t destination,
    bool remote,
    uint16_t network_index,
    uint16_t application_index,
    uint16_t group_address,
    uint16_t company_id,
    uint16_t model_id) {
    if (destination == 0 || destination > 0x7fff) {
        throw std::invalid_argument("vendor-model destination must be unicast");
    }
    if (network_index > 0x0fff) {
        throw std::invalid_argument("network index exceeds 12 bits");
    }
    if (application_index > 0x0fff) {
        throw std::invalid_argument("application index exceeds 12 bits");
    }
    if (group_address < 0xc000 || group_address > 0xfeff) {
        throw std::invalid_argument("vendor-model subscription must use a group address");
    }
    const auto append_u16 = [](std::vector<uint8_t>& payload, uint16_t value) {
        payload.push_back(static_cast<uint8_t>(value & 0xff));
        payload.push_back(static_cast<uint8_t>(value >> 8));
    };

    std::vector<uint8_t> bind{0x80, 0x3d};
    bind.reserve(10);
    append_u16(bind, destination);
    append_u16(bind, static_cast<uint16_t>(application_index & 0x0fff));
    append_u16(bind, company_id);
    append_u16(bind, model_id);
    dev_key_send(0, destination, remote, network_index, false, bind);

    std::vector<uint8_t> subscribe{0x80, 0x1b};
    subscribe.reserve(12);
    append_u16(subscribe, destination);
    append_u16(subscribe, group_address);
    append_u16(subscribe, company_id);
    append_u16(subscribe, model_id);
    dev_key_send(0, destination, remote, network_index, false, subscribe);
}

void MeshApplication::persist_next_unicast() const {
    if (config_.mesh_unicast_cursor_path.empty()) return;

    const std::filesystem::path path(config_.mesh_unicast_cursor_path);
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
    }
    const auto temporary = path.string() + ".tmp";
    {
        std::ofstream file(temporary, std::ios::trunc);
        if (!file) throw std::runtime_error("cannot open Mesh unicast cursor file");
        file << static_cast<uint32_t>(next_unicast_) << '\n';
        if (!file) throw std::runtime_error("cannot write Mesh unicast cursor file");
    }
    std::error_code error;
    std::filesystem::rename(temporary, path, error);
    if (error) {
        std::filesystem::remove(temporary);
        throw std::runtime_error(
            "cannot replace Mesh unicast cursor file: " + error.message());
    }
}

void MeshApplication::require_attached() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_.attached || status_.node_path.empty()) {
        throw std::runtime_error("Mesh node is not attached");
    }
}

void MeshApplication::send(uint8_t index, uint16_t destination, uint16_t key_index,
                           bool force_segmented, const std::vector<uint8_t>& data) {
    require_attached();
    VariantMap options;
    options.emplace("ForceSegmented", sdbus::Variant{force_segmented});
    node_proxy()->callMethod("Send")
        .onInterface(kMeshNodeInterface)
        .withArguments(sdbus::ObjectPath{element_path(index)}, destination,
                       key_index, options, data);
}

void MeshApplication::dev_key_send(uint8_t index, uint16_t destination, bool remote,
                                   uint16_t network_index, bool force_segmented,
                                   const std::vector<uint8_t>& data) {
    require_attached();
    VariantMap options;
    options.emplace("ForceSegmented", sdbus::Variant{force_segmented});
    node_proxy()->callMethod("DevKeySend")
        .onInterface(kMeshNodeInterface)
        .withArguments(sdbus::ObjectPath{element_path(index)}, destination,
                       remote, network_index, options, data);
}

void MeshApplication::add_net_key(uint8_t index, uint16_t destination,
                                  uint16_t subnet_index, uint16_t network_index,
                                  bool update) {
    require_attached();
    node_proxy()->callMethod("AddNetKey")
        .onInterface(kMeshNodeInterface)
        .withArguments(sdbus::ObjectPath{element_path(index)}, destination,
                       subnet_index, network_index, update);
}

void MeshApplication::add_app_key(uint8_t index, uint16_t destination,
                                  uint16_t application_index, uint16_t network_index,
                                  bool update) {
    require_attached();
    node_proxy()->callMethod("AddAppKey")
        .onInterface(kMeshNodeInterface)
        .withArguments(sdbus::ObjectPath{element_path(index)}, destination,
                       application_index, network_index, update);
}

void MeshApplication::publish(uint8_t index, uint16_t model_id,
                              std::optional<uint16_t> vendor_id,
                              bool force_segmented,
                              const std::vector<uint8_t>& data) {
    require_attached();
    VariantMap options;
    options.emplace("ForceSegmented", sdbus::Variant{force_segmented});
    if (vendor_id) options.emplace("Vendor", sdbus::Variant{*vendor_id});
    node_proxy()->callMethod("Publish")
        .onInterface(kMeshNodeInterface)
        .withArguments(sdbus::ObjectPath{element_path(index)}, model_id,
                       options, data);
}

void MeshApplication::unprovisioned_scan(const VariantMap& options) {
    management_proxy()->callMethod("UnprovisionedScan")
        .onInterface(kMeshManagementInterface).withArguments(options);
}

void MeshApplication::unprovisioned_scan_cancel() {
    management_proxy()->callMethod("UnprovisionedScanCancel")
        .onInterface(kMeshManagementInterface);
}

void MeshApplication::add_node(const std::vector<uint8_t>& uuid,
                               const VariantMap& options) {
    require_size(uuid, 16, "mesh UUID");
    if (add_node_start_pending_.exchange(true)) {
        throw std::runtime_error("another Mesh provisioning request is starting");
    }

    try {
        // BlueZ defers AddNode's method reply while it opens the provisioning
        // bearer. Keeping this asynchronous prevents a missing member from
        // blocking ROS services, coordination heartbeats, and overlay renewal for
        // the entire provisioning timeout.
        auto proxy = std::shared_ptr<sdbus::IProxy>(management_proxy().release());
        const auto selected_uuid = mesh_bytes_to_hex(uuid);
        add_node_call_slot_.emplace(
            proxy->callMethodAsync("AddNode")
                .onInterface(kMeshManagementInterface)
                .withTimeout(kCallTimeout)
                .withArguments(uuid, options)
                .uponReplyInvoke(
                    [this, proxy, selected_uuid](
                        std::optional<sdbus::Error> error) {
                        (void)proxy;
                        add_node_start_pending_ = false;
                        if (!error) return;
                        Event event;
                        event.event = "add_node_failed";
                        event.uuid = selected_uuid;
                        event.reason = mesh_error_text(*error);
                        emit_event(std::move(event));
                    },
                    sdbus::return_slot));
    } catch (...) {
        add_node_start_pending_ = false;
        throw;
    }
}

void MeshApplication::reprovision(uint16_t unicast, const VariantMap& options) {
    management_proxy()->callMethod("Reprovision")
        .onInterface(kMeshManagementInterface).withArguments(unicast, options);
}

void MeshApplication::create_subnet(uint16_t index) {
    management_proxy()->callMethod("CreateSubnet")
        .onInterface(kMeshManagementInterface).withArguments(index);
}

void MeshApplication::import_subnet(uint16_t index, const std::vector<uint8_t>& key) {
    require_size(key, 16, "network key");
    management_proxy()->callMethod("ImportSubnet")
        .onInterface(kMeshManagementInterface).withArguments(index, key);
}

void MeshApplication::update_subnet(uint16_t index) {
    management_proxy()->callMethod("UpdateSubnet")
        .onInterface(kMeshManagementInterface).withArguments(index);
}

void MeshApplication::delete_subnet(uint16_t index) {
    management_proxy()->callMethod("DeleteSubnet")
        .onInterface(kMeshManagementInterface).withArguments(index);
}

void MeshApplication::set_key_phase(uint16_t index, uint8_t phase) {
    management_proxy()->callMethod("SetKeyPhase")
        .onInterface(kMeshManagementInterface).withArguments(index, phase);
}

void MeshApplication::create_app_key(uint16_t network_index, uint16_t app_index) {
    management_proxy()->callMethod("CreateAppKey")
        .onInterface(kMeshManagementInterface).withArguments(network_index, app_index);
}

void MeshApplication::import_app_key(uint16_t network_index, uint16_t app_index,
                                     const std::vector<uint8_t>& key) {
    require_size(key, 16, "application key");
    management_proxy()->callMethod("ImportAppKey")
        .onInterface(kMeshManagementInterface)
        .withArguments(network_index, app_index, key);
}

void MeshApplication::update_app_key(uint16_t app_index) {
    management_proxy()->callMethod("UpdateAppKey")
        .onInterface(kMeshManagementInterface).withArguments(app_index);
}

void MeshApplication::delete_app_key(uint16_t app_index) {
    management_proxy()->callMethod("DeleteAppKey")
        .onInterface(kMeshManagementInterface).withArguments(app_index);
}

void MeshApplication::import_remote_node(uint16_t primary, uint8_t count,
                                         const std::vector<uint8_t>& key) {
    require_size(key, 16, "remote device key");
    management_proxy()->callMethod("ImportRemoteNode")
        .onInterface(kMeshManagementInterface).withArguments(primary, count, key);
}

void MeshApplication::delete_remote_node(uint16_t primary, uint8_t count) {
    management_proxy()->callMethod("DeleteRemoteNode")
        .onInterface(kMeshManagementInterface).withArguments(primary, count);
}

std::vector<DeviceKey> MeshApplication::export_device_keys() {
    VariantMap exported;
    management_proxy()->callMethod("ExportKeys")
        .onInterface(kMeshManagementInterface).storeResultsTo(exported);

    std::vector<DeviceKey> result;
    if (const auto records =
            variant_value<std::vector<DevKeyRecord>>(exported, "DevKeys")) {
        result.reserve(records->size());
        for (const auto& record : *records) {
            result.push_back(DeviceKey{std::get<0>(record), std::get<1>(record)});
        }
    }
    return result;
}

std::string MeshApplication::export_keys_yaml() {
    VariantMap exported;
    management_proxy()->callMethod("ExportKeys")
        .onInterface(kMeshManagementInterface).storeResultsTo(exported);

    std::ostringstream output;
    output << "net_keys:\n";
    if (const auto records = variant_value<std::vector<NetKeyRecord>>(exported, "NetKeys")) {
        for (const auto& record : *records) {
            const auto& index = std::get<0>(record);
            const auto& key = std::get<1>(record);
            const auto& info = std::get<2>(record);
            output << "  - index: " << index << "\n"
                   << "    key: \"" << mesh_bytes_to_hex(key) << "\"\n";
            if (const auto phase = variant_value<uint8_t>(info, "Phase"))
                output << "    phase: " << static_cast<unsigned>(*phase) << "\n";
            if (const auto old_key = variant_value<std::vector<uint8_t>>(info, "OldKey"))
                output << "    old_key: \"" << mesh_bytes_to_hex(*old_key) << "\"\n";
            output << "    app_keys:\n";
            if (const auto app_keys = variant_value<std::vector<AppKeyRecord>>(info, "AppKeys")) {
                for (const auto& app_key : *app_keys) {
                    output << "      - index: " << std::get<0>(app_key) << "\n"
                           << "        key: \"" << mesh_bytes_to_hex(std::get<1>(app_key))
                           << "\"\n";
                    const auto& app_info = std::get<2>(app_key);
                    if (const auto old_key =
                            variant_value<std::vector<uint8_t>>(app_info, "OldKey")) {
                        output << "        old_key: \"" << mesh_bytes_to_hex(*old_key)
                               << "\"\n";
                    }
                }
            }
        }
    }
    output << "dev_keys:\n";
    if (const auto records = variant_value<std::vector<DevKeyRecord>>(exported, "DevKeys")) {
        for (const auto& record : *records) {
            output << "  - unicast: " << std::get<0>(record) << "\n"
                   << "    key: \"" << mesh_bytes_to_hex(std::get<1>(record)) << "\"\n";
        }
    }
    return output.str();
}

void MeshApplication::set_token(uint64_t value) {
    TokenCallback callback;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.token = value;
        callback = token_callback_;
    }
    if (!config_.mesh_token_path.empty()) {
        try {
            const std::filesystem::path path(config_.mesh_token_path);
            if (!path.parent_path().empty()) {
                std::filesystem::create_directories(path.parent_path());
            }
            const auto temporary = path.string() + ".tmp";
            {
                std::ofstream file(temporary, std::ios::trunc);
                if (!file) throw std::runtime_error("cannot open token file");
                std::filesystem::permissions(temporary,
                    std::filesystem::perms::owner_read | std::filesystem::perms::owner_write,
                    std::filesystem::perm_options::replace);
                file << std::hex << std::setw(16) << std::setfill('0') << value << "\n";
                file.flush();
                if (!file) throw std::runtime_error("cannot write token file");
            }
            std::filesystem::rename(temporary, path);
        } catch (const std::exception& error) {
            RCLCPP_WARN(logger_, "Could not persist Mesh token to %s: %s",
                        config_.mesh_token_path.c_str(), error.what());
        }
    }
    if (callback) callback(value);
}

void MeshApplication::emit_event(Event event) const {
    EventCallback callback;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        callback = event_callback_;
    }
    if (callback) callback(event);
}

void MeshApplication::emit_message(ReceivedMessage message) const {
    MessageCallback callback;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        callback = message_callback_;
    }
    if (callback) callback(message);
}

void MeshApplication::require_size(const std::vector<uint8_t>& value,
                                   size_t expected,
                                   const std::string& name) {
    if (value.size() != expected) {
        throw std::invalid_argument(name + " must contain exactly " +
                                    std::to_string(expected) + " bytes");
    }
}

}  // namespace mrs_uav_bluetooth::mesh
