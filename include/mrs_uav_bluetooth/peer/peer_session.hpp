// SPDX-License-Identifier: MIT
#pragma once

#include <map>
#include <string>

namespace mrs_uav_bluetooth::peer {

struct PeerConnectionSession {
    std::string mac;
    bool desired{false};
    bool explicit_target{false};
    bool peer_candidate{false};
    std::string peer_name;
    std::string phase{"idle"};
    std::string detail;
    double first_seen_monotonic{0.0};
    double last_seen_monotonic{0.0};
    double last_connect_attempt_monotonic{0.0};
    double connect_started_monotonic{0.0};
    double connected_since_monotonic{0.0};
    double last_security_attempt_monotonic{0.0};
    double last_repair_monotonic{0.0};
    double last_service_retry_monotonic{0.0};
    double missing_since_monotonic{0.0};
    double bridge_wait_started_monotonic{0.0};
    std::string bridge_wait_reason;
    int connect_repair_count{0};
    int pairing_failures{0};
    double services_wait_started_monotonic{0.0};
    double services_wait_grace_s{0.0};
    std::map<std::string, double> import_bridge_missing_since;
};

}  // namespace mrs_uav_bluetooth::peer
