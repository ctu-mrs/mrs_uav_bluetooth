// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/mesh/swarm_coordinator.hpp"

#include "mrs_uav_bluetooth/util/string_utils.hpp"
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace mrs_uav_bluetooth::mesh {

namespace {

constexpr auto kConfigurationStepDelay = std::chrono::seconds(1);
constexpr auto kRetryDelay = std::chrono::seconds(5);
// Starting a Mesh daemon, switching the controller away from GATT, and
// configuring the first node can take tens of seconds on UAV hardware.
// Keep later candidates unprovisioned long enough for the preferred peer's
// PB-ADV initiator to finish instead of racing into a separate network.
constexpr double kPerPriorityJoinWindowSeconds = 90.0;
constexpr uint8_t kControlOpcode = 0xc0;
constexpr uint8_t kFrameVersion = 4;
constexpr size_t kFrameSize = 8;
constexpr uint8_t kSwarmParticipatingFlag = 0x02;

uint32_t uav_number(const std::string& hostname) {
    const auto first_digit = hostname.find_last_not_of("0123456789");
    const auto digits = first_digit == std::string::npos
        ? hostname
        : hostname.substr(first_digit + 1);
    if (digits.empty()) {
        throw std::invalid_argument(
            "automatic Mesh peer must end in a UAV number: " + hostname);
    }
    size_t parsed = 0;
    const auto value = std::stoul(digits, &parsed, 10);
    if (parsed != digits.size() || value == 0 || value > 0xffffU) {
        throw std::invalid_argument(
            "automatic Mesh UAV number must be in 1..65535: " + hostname);
    }
    return static_cast<uint32_t>(value);
}

uint32_t read_u24(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<uint32_t>(data[offset]) |
        (static_cast<uint32_t>(data[offset + 1]) << 8U) |
        (static_cast<uint32_t>(data[offset + 2]) << 16U);
}

uint16_t read_u16(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<uint16_t>(data[offset]) |
        (static_cast<uint16_t>(data[offset + 1]) << 8U);
}

void append_u24(std::vector<uint8_t>& data, uint32_t value) {
    data.push_back(static_cast<uint8_t>(value));
    data.push_back(static_cast<uint8_t>(value >> 8U));
    data.push_back(static_cast<uint8_t>(value >> 16U));
}

void append_u16(std::vector<uint8_t>& data, uint16_t value) {
    data.push_back(static_cast<uint8_t>(value));
    data.push_back(static_cast<uint8_t>(value >> 8U));
}

/// FNV-1a truncated to 24 bits keeps the Mesh control access message compact
/// while strongly separating independently configured swarm admission lists.
uint32_t swarm_fingerprint(const std::vector<std::string>& members,
                           const std::string& preference) {
    std::vector<std::string> normalized;
    normalized.reserve(members.size());
    for (const auto& member : members) {
        normalized.push_back(util::lower_trim_copy(member));
    }
    std::string canonical = util::lower_trim_copy(preference) + '|';
    if (normalized.empty()) {
        canonical += '*';
    } else {
        for (const auto& member : normalized) canonical += member + ',';
    }
    uint32_t hash = 2166136261U;
    for (const unsigned char byte : canonical) {
        hash ^= byte;
        hash *= 16777619U;
    }
    hash &= 0x00ffffffU;
    return hash == 0 ? 1U : hash;
}

}  // namespace

MeshSwarmCoordinator::MeshSwarmCoordinator(
    MeshApplication& application,
    config::NodeConfig config,
    std::string hostname,
    rclcpp::Logger logger)
    : application_(application),
      config_(std::move(config)),
      hostname_(util::lower_trim_copy(hostname)),
      logger_(logger),
      local_number_(uav_number(hostname_)) {
    automatic_private_ = config_.mesh_fleet_id.empty();
    for (const auto& raw_member : config_.peer_whitelist) {
        const auto member = util::lower_trim_copy(raw_member);
        const auto number = uav_number(member);
        if (!member_by_number_.emplace(number, member).second) {
            throw std::invalid_argument(
                "automatic Mesh peer whitelist has duplicate UAV numbers");
        }
        member_priority_.push_back(number);
    }

    const auto preference = util::lower_trim_copy(
        config_.mesh_provisioner_preference);
    if (preference != "ordered") preferred_number_ = uav_number(preference);
    whitelist_fingerprint_ = swarm_fingerprint(
        config_.peer_whitelist, preference);
    // Candidate order is common fleet policy; enrollment itself needs no leader.
    active_number_ = preferred_number_ != 0
        ? preferred_number_
        : (member_priority_.empty() ? local_number_ : member_priority_.front());
    active_provisioner_ = member_name(active_number_);
    local_is_active_.store(active_number_ == local_number_);
    active_since_ = Clock::now();
    bootstrap_started_at_ = active_since_;
    swarm_id_ = config_.mesh_swarm_id;
    swarm_participating_ = config_.mesh_swarm_participating;
    load_swarm_state();

    RCLCPP_INFO(
        logger_,
        "Automatic Mesh %s uses preference '%s', swarm fingerprint %06x, and local UAV ID %u",
        automatic_private_ ? "PB-ADV enrollment" : "pre-shared fleet",
        preference.c_str(), whitelist_fingerprint_, local_number_);
}

void MeshSwarmCoordinator::handle_event(const Event& event) {
    if (!automatic_private_ ||
        (event.event != "scan_result" &&
         event.event != "add_node_complete" &&
         event.event != "add_node_failed")) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_events_.size() < 128) pending_events_.push_back(event);
}

void MeshSwarmCoordinator::persist_swarm_state(
    uint16_t swarm_id, bool participating) const {
    if (config_.mesh_swarm_state_path.empty()) return;
    const std::filesystem::path path(config_.mesh_swarm_state_path);
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
    }
    const auto temporary = path.string() + ".tmp";
    {
        std::ofstream output(temporary, std::ios::trunc);
        output << swarm_id << ' ' << (participating ? 1 : 0) << '\n';
        output.flush();
        if (!output) {
            throw std::runtime_error("Could not persist Mesh swarm state: " +
                                     path.string());
        }
    }
    std::filesystem::rename(temporary, path);
}

void MeshSwarmCoordinator::load_swarm_state() {
    if (config_.mesh_swarm_state_path.empty()) return;
    std::ifstream input(config_.mesh_swarm_state_path);
    if (!input) return;
    uint32_t stored_id = 0;
    int stored_participating = -1;
    if (!(input >> stored_id >> stored_participating) ||
        stored_id == 0 || stored_id > 0xffffU ||
        (stored_participating != 0 && stored_participating != 1)) {
        RCLCPP_WARN(logger_,
                    "Ignoring invalid persisted Mesh swarm state at %s",
                    config_.mesh_swarm_state_path.c_str());
        return;
    }
    swarm_id_ = static_cast<uint16_t>(stored_id);
    swarm_participating_ = stored_participating != 0;
}

void MeshSwarmCoordinator::join_swarm(uint16_t swarm_id) {
    if (swarm_id == 0) {
        throw std::invalid_argument("swarm_id must be in 1..65535");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    persist_swarm_state(swarm_id, true);
    swarm_id_ = swarm_id;
    swarm_participating_ = true;
    last_heartbeat_ = {};
    RCLCPP_INFO(logger_, "Joined logical Mesh swarm %u without reprovisioning",
                static_cast<unsigned>(swarm_id_));
}

void MeshSwarmCoordinator::leave_swarm() {
    std::lock_guard<std::mutex> lock(mutex_);
    persist_swarm_state(swarm_id_, false);
    swarm_participating_ = false;
    last_heartbeat_ = {};
    RCLCPP_INFO(logger_, "Left logical Mesh swarm %u; radio remains a relay",
                static_cast<unsigned>(swarm_id_));
}

uint16_t MeshSwarmCoordinator::swarm_id() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return swarm_id_;
}

bool MeshSwarmCoordinator::swarm_participating() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return swarm_participating_;
}

std::vector<std::string> MeshSwarmCoordinator::swarm_members() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = Clock::now();
    std::vector<std::string> members;
    if (swarm_participating_) members.push_back(hostname_);
    for (const auto& [number, presence] : peer_presence_) {
        // An AppKey-authenticated heartbeat can only arrive through the
        // attached Mesh. Its 24-bit marker is advisory coordination metadata,
        // and may be reconciled separately on the timer thread.
        if (presence.expires > now && presence.participating &&
            presence.swarm_id == swarm_id_) {
            members.push_back(member_name(number));
        }
    }
    return members;
}

bool MeshSwarmCoordinator::accepts_swarm_payload(uint16_t source) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!swarm_participating_) return false;
    const auto now = Clock::now();
    for (const auto& [number, presence] : peer_presence_) {
        (void)number;
        if (presence.source == source && presence.expires > now &&
            presence.participating && presence.swarm_id == swarm_id_) {
            return true;
        }
    }
    return false;
}

bool MeshSwarmCoordinator::handle_message(const ReceivedMessage& message) {
    // BlueZ's Element1.MessageReceived signal exposes the AppKey index but
    // not the subnet index. ReceivedMessage::net_index is consequently unset
    // for AppKey traffic; testing it would reject every nonzero subnet.
    if (message.device_key ||
        message.key_index != config_.mesh_swarm_app_key_index ||
        message.source == 0) {
        return false;
    }
    size_t offset = 0;
    if (message.data.size() == kFrameSize + 3 &&
        message.data[0] == kControlOpcode &&
        message.data[1] == static_cast<uint8_t>(config_.mesh_company_id) &&
        message.data[2] == static_cast<uint8_t>(config_.mesh_company_id >> 8U)) {
        offset = 3;
    } else if (message.data.size() != kFrameSize) {
        return false;
    }
    std::vector<uint8_t> frame(message.data.begin() + offset, message.data.end());
    if (frame.size() != kFrameSize || frame[0] != 'M' || frame[1] != 'S') {
        return false;
    }
    accept_coordination_frame(frame, message.source, Clock::now());
    return true;
}

void MeshSwarmCoordinator::accept_coordination_frame(
    const std::vector<uint8_t>& frame,
    uint16_t source,
    Clock::time_point now) {
    if (frame.size() != kFrameSize || frame[0] != 'M' || frame[1] != 'S' ||
        (frame[2] >> 4U) != kFrameVersion ||
        read_u24(frame, 3) != whitelist_fingerprint_) {
        return;
    }

    // Source is authenticated by Mesh and is the immutable numeric UAV address.
    // Do not forward application-level leases: Mesh flooding already relays
    // this unsegmented heartbeat to all reachable group members.
    const uint32_t sender = source;
    const uint16_t sender_swarm_id = read_u16(frame, 6);
    if (sender == 0 || sender == local_number_ ||
        sender_swarm_id == 0 || !peer_allowed(sender)) return;
    const auto full_lease = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(config_.mesh_swarm_provisioner_timeout));
    std::lock_guard<std::mutex> lock(mutex_);
    peer_presence_[sender] = PeerPresence{
        source, sender_swarm_id, (frame[2] & kSwarmParticipatingFlag) != 0,
        now + full_lease};
}

bool MeshSwarmCoordinator::peer_allowed(uint32_t number) const {
    return config_.peer_whitelist.empty() || member_by_number_.contains(number);
}

std::string MeshSwarmCoordinator::member_name(uint32_t number) const {
    const auto known = member_by_number_.find(number);
    return known == member_by_number_.end()
        ? std::string{"uav"} + std::to_string(number)
        : known->second;
}

void MeshSwarmCoordinator::update_selection(Clock::time_point now) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto item = peer_presence_.begin(); item != peer_presence_.end();) {
        if (item->second.expires <= now) item = peer_presence_.erase(item);
        else ++item;
    }

    const auto available = [this](uint32_t number) {
        return number == local_number_ || peer_presence_.contains(number);
    };
    uint32_t selected = local_number_;
    if (preferred_number_ != 0 && available(preferred_number_)) {
        selected = preferred_number_;
    } else if (!member_priority_.empty()) {
        for (const uint32_t number : member_priority_) {
            if (available(number)) {
                selected = number;
                break;
            }
        }
    } else {
        // Open admission has no configured order, so numeric order is the only
        // deterministic choice shared by all discovered peers.
        for (const auto& [number, presence] : peer_presence_) {
            (void)presence;
            selected = std::min(selected, number);
        }
    }

    if (selected != active_number_) {
        const auto previous = active_provisioner_;
        active_number_ = selected;
        active_provisioner_ = member_name(selected);
        local_is_active_.store(selected == local_number_);
        active_since_ = now;
        RCLCPP_INFO(logger_,
                    "Active Mesh provisioner changed from %s to %s",
                    previous.c_str(), active_provisioner_.c_str());
    }
}

void MeshSwarmCoordinator::maintain(const Status& status) {
    const auto now = Clock::now();
    // Lifecycle failures are reflected in Status; retries are bounded and
    // never discard a token or reset BlueZ's replay-protection database.
    if (!status.attached) {
        application_ready_ = false;
        local_model_stage_ = 0;
        next_local_model_action_ = {};
        if (!status.daemon_available) {
            last_bootstrap_action_ = {};
            return;
        }
        if (status.token != 0) {
            if (last_bootstrap_action_ != Clock::time_point{} &&
                now - last_bootstrap_action_ < kRetryDelay) return;
            last_bootstrap_action_ = now;
            application_.attach(status.token);
        } else if (automatic_private_) {
            const auto elapsed = std::chrono::duration<double>(
                now - bootstrap_started_at_).count();
            size_t priority = 0;
            if (!member_priority_.empty()) {
                const auto first = preferred_number_ != 0
                    ? std::find(member_priority_.begin(), member_priority_.end(),
                                preferred_number_)
                    : member_priority_.begin();
                const auto own = std::find(member_priority_.begin(),
                                            member_priority_.end(), local_number_);
                priority = first == member_priority_.end() ||
                    own == member_priority_.end() ? member_priority_.size()
                    : static_cast<size_t>(
                        (own - first + member_priority_.size()) %
                        member_priority_.size());
            }
            const double create_after = config_.mesh_swarm_startup_grace +
                kPerPriorityJoinWindowSeconds * priority;
            if (elapsed >= create_after && status.state != "importing") {
                if (status.state == "joining") application_.cancel_join();
                if (last_bootstrap_action_ != Clock::time_point{} &&
                    now - last_bootstrap_action_ < kRetryDelay) return;
                last_bootstrap_action_ = now;
                application_.import_auto_identity(
                    static_cast<uint16_t>(local_number_));
                RCLCPP_INFO(logger_,
                    "Created a private Mesh network after waiting %.0f seconds for a provisioner",
                    create_after);
            } else if (status.state != "joining" &&
                       status.state != "importing" &&
                       (last_bootstrap_action_ == Clock::time_point{} ||
                        now - last_bootstrap_action_ >= kRetryDelay)) {
                last_bootstrap_action_ = now;
                application_.join();
            }
        } else if (status.state != "importing") {
            if (last_bootstrap_action_ != Clock::time_point{} &&
                now - last_bootstrap_action_ < kRetryDelay) return;
            last_bootstrap_action_ = now;
            const auto device_key = application_.fleet_device_key();
            application_.import_node(application_.uuid(), device_key,
                config_.mesh_fleet_network_key, config_.mesh_swarm_network_index,
                false, false, config_.mesh_fleet_iv_index, config_.mesh_fleet_unicast);
        }
        return;
    }
    if (status.addresses.size() != 1 ||
        status.addresses.front() != local_number_) {
        throw std::runtime_error("Stored Mesh address differs from fleet address; refusing unsafe reuse");
    }
    if (std::chrono::duration<double>(now - bootstrap_started_at_).count() >=
        config_.mesh_swarm_startup_grace) update_selection(now);
    maintain_local_model(status, now);
    publish_coordination_heartbeat(now);
    if (automatic_private_) maintain_auto_enrollment(status, now);
    return;
}

void MeshSwarmCoordinator::publish_coordination_heartbeat(Clock::time_point now) {
    if (!application_ready_.load()) return;
    const auto period = std::chrono::duration<double>(config_.mesh_swarm_heartbeat_period);
    std::vector<uint8_t> access{
        kControlOpcode, static_cast<uint8_t>(config_.mesh_company_id),
        static_cast<uint8_t>(config_.mesh_company_id >> 8U)};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (last_heartbeat_ != Clock::time_point{} && now - last_heartbeat_ < period)
            return;
        last_heartbeat_ = now;
        access.push_back('M');
        access.push_back('S');
        access.push_back(static_cast<uint8_t>((kFrameVersion << 4U) |
            (swarm_participating_ ? kSwarmParticipatingFlag : 0)));
        append_u24(access, whitelist_fingerprint_);
        append_u16(access, swarm_id_);
    }
    // Three-byte vendor opcode + eight-byte body fits the 11-byte unsegmented
    // access budget. No SAR queue, no stale relayed lease amplification.
    application_.send(0, config_.mesh_swarm_group_address,
                      config_.mesh_swarm_app_key_index, false, access);
}

void MeshSwarmCoordinator::maintain_local_model(
    const Status& status,
    Clock::time_point now) {
    if (application_ready_ || status.addresses.empty() ||
        (next_local_model_action_ != Clock::time_point{} &&
         now < next_local_model_action_)) {
        return;
    }

    const uint16_t local_address = status.addresses.front();
    try {
        if (local_model_stage_ == 0) {
            if (automatic_private_) {
                if (!application_.prepare_auto_keys(local_address)) {
                    next_local_model_action_ = now + kRetryDelay;
                    return;
                }
            } else {
                application_.prepare_fleet_keys();
            }
            ++local_model_stage_;
        } else if (local_model_stage_ == 1) {
            application_.add_app_key(
                0, local_address, config_.mesh_swarm_app_key_index,
                config_.mesh_swarm_network_index, false);
            ++local_model_stage_;
        } else if (local_model_stage_ == 2) {
            application_.configure_vendor_model(
                local_address, true, config_.mesh_swarm_network_index,
                config_.mesh_swarm_app_key_index,
                config_.mesh_swarm_group_address, config_.mesh_company_id,
                config_.mesh_vendor_model_id);
            ++local_model_stage_;
        } else if (application_.vendor_model_ready()) {
            application_ready_ = true;
            RCLCPP_INFO(logger_,
                        "Automatic Mesh application transport is ready on group 0x%04x",
                        config_.mesh_swarm_group_address);
        } else {
            // A successful D-Bus send only queues a Config message. Retry until
            // BlueZ reports the actual binding and subscription, never merely
            // assume that enough seconds have elapsed.
            local_model_stage_ = 1;
        }
        next_local_model_action_ = now + kConfigurationStepDelay;
    } catch (const std::exception& error) {
        next_local_model_action_ = now + kRetryDelay;
        RCLCPP_WARN(logger_,
                    "Waiting to configure the local Mesh application model: %s",
                    error.what());
    }
}

void MeshSwarmCoordinator::maintain_auto_enrollment(
    const Status& status, Clock::time_point now) {
    if (!status.attached || !application_ready_.load()) return;

    // ScanResult and provisioning completion arrive on the D-Bus dispatch
    // thread. Work through them here so no synchronous D-Bus call can
    // deadlock the callback that delivered it.
    std::deque<Event> events;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        events.swap(pending_events_);
    }
    for (const auto& event : events) {
        if (event.event == "add_node_failed" &&
            event.uuid == pending_provision_uuid_) {
            RCLCPP_WARN(logger_, "Mesh provisioning of %s failed: %s",
                        event.uuid.c_str(), event.reason.c_str());
            pending_provision_uuid_.clear();
            pending_provision_address_ = 0;
            remote_model_stage_ = 0;
            last_scan_at_ = {};
            continue;
        }
        if (event.event == "add_node_complete" &&
            event.uuid == pending_provision_uuid_) {
            if (event.unicast != pending_provision_address_ || event.count != 1) {
                RCLCPP_ERROR(logger_,
                    "Mesh provisioned address differs from the expected UAV address");
                pending_provision_uuid_.clear();
                pending_provision_address_ = 0;
                continue;
            }
            remote_model_stage_ = 1;
            remote_model_action_at_ = now + kConfigurationStepDelay;
            pending_provision_until_ = {};
            RCLCPP_INFO(logger_, "Provisioned %s at address %u",
                        event.uuid.c_str(), event.unicast);
            continue;
        }
        if (event.event != "scan_result" || !local_is_active_.load() ||
            !pending_provision_uuid_.empty() || event.data.size() < 16) {
            continue;
        }
        const std::vector<uint8_t> uuid(event.data.begin(), event.data.begin() + 16);
        const uint16_t number = mesh_auto_uuid_number(uuid);
        if (number == 0 || number == local_number_ || !peer_allowed(number))
            continue;
        if (!config_.peer_whitelist.empty()) {
            const auto known = member_by_number_.find(number);
            if (known == member_by_number_.end() ||
                mesh_auto_uuid_from_name(known->second) != uuid) continue;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (peer_presence_.contains(number)) continue;
        }
        try {
            application_.set_auto_provisioning_unicast(number);
            mesh::VariantMap options;
            options.emplace("Seconds", sdbus::Variant{uint16_t{45}});
            application_.add_node(uuid, options);
            pending_provision_uuid_ = mesh_bytes_to_hex(uuid);
            pending_provision_address_ = number;
            pending_provision_until_ = now + std::chrono::seconds(50);
            RCLCPP_INFO(logger_, "Provisioning nearby %s at address %u",
                        member_name(number).c_str(), number);
        } catch (const std::exception& error) {
            RCLCPP_WARN(logger_, "Could not start nearby Mesh provisioning: %s",
                        error.what());
        }
    }

    if (!pending_provision_uuid_.empty() &&
        pending_provision_until_ != Clock::time_point{} &&
        now >= pending_provision_until_) {
        RCLCPP_WARN(logger_, "Timed out waiting for Mesh provisioning result");
        pending_provision_uuid_.clear();
        pending_provision_address_ = 0;
        last_scan_at_ = {};
    }
    if (remote_model_stage_ != 0 && now >= remote_model_action_at_) {
        try {
            if (remote_model_stage_ == 1) {
                application_.add_app_key(0, pending_provision_address_,
                    config_.mesh_swarm_app_key_index,
                    config_.mesh_swarm_network_index, false);
                remote_model_stage_ = 2;
            } else {
                application_.configure_vendor_model(
                    pending_provision_address_, true,
                    config_.mesh_swarm_network_index,
                    config_.mesh_swarm_app_key_index,
                    config_.mesh_swarm_group_address,
                    config_.mesh_company_id, config_.mesh_vendor_model_id);
                remote_model_stage_ = 0;
                pending_provision_uuid_.clear();
                pending_provision_address_ = 0;
                last_scan_at_ = {};
            }
            remote_model_action_at_ = now + std::chrono::seconds(2);
        } catch (const std::exception& error) {
            remote_model_action_at_ = now + kRetryDelay;
            RCLCPP_WARN(logger_, "Retrying remote Mesh model setup: %s",
                        error.what());
        }
    }
    if (!pending_provision_uuid_.empty()) return;
    // BlueZ's UnprovisionedScan method sends a Remote Provisioning Scan Start
    // message to an RPR Server, even when Server is omitted. Our local vendor
    // model is not an RPR Server. A common ordered peer list already gives us
    // every Device UUID, so directly initiate PB-ADV for absent peers. Every
    // attached UAV may provision a newcomer in its own radio range.
    for (const auto number : member_priority_) {
        if (number == local_number_) continue;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (peer_presence_.contains(number)) continue;
        }
        const auto previous = last_direct_attempt_[number];
        if (previous != Clock::time_point{} &&
            now - previous < std::chrono::seconds(55)) continue;
        last_direct_attempt_[number] = now;
        try {
            const auto uuid = mesh_auto_uuid_from_name(member_name(number));
            application_.set_auto_provisioning_unicast(
                static_cast<uint16_t>(number));
            mesh::VariantMap options;
            options.emplace("Seconds", sdbus::Variant{uint16_t{45}});
            application_.add_node(uuid, options);
            pending_provision_uuid_ = mesh_bytes_to_hex(uuid);
            pending_provision_address_ = static_cast<uint16_t>(number);
            pending_provision_until_ = now + std::chrono::seconds(50);
            RCLCPP_INFO(logger_, "Searching for nearby %s over PB-ADV",
                        member_name(number).c_str());
        } catch (const std::exception& error) {
            RCLCPP_WARN(logger_, "Cannot initiate PB-ADV for %s: %s",
                        member_name(number).c_str(), error.what());
        }
        break;
    }
}

std::string MeshSwarmCoordinator::active_provisioner() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_provisioner_;
}

std::string MeshSwarmCoordinator::preferred_provisioner() const {
    if (preferred_number_ != 0) return member_name(preferred_number_);
    if (!member_priority_.empty()) return member_name(member_priority_.front());
    return "ordered";
}

bool MeshSwarmCoordinator::local_is_active_provisioner() const {
    return local_is_active_.load();
}

bool MeshSwarmCoordinator::application_ready() const {
    return application_ready_.load();
}

}  // namespace mrs_uav_bluetooth::mesh
