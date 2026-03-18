// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace mrs_uav_bluetooth::peer {

struct PeerTimeBridge {
    std::string mac;
    std::string peer_name;
    std::string status_topic_name;
    std::string characteristic_path;
    std::string writeback_descriptor_path;
    rclcpp::PublisherBase::SharedPtr publisher;
    double last_activity_monotonic{0.0};
    double last_publish_monotonic{0.0};
    double current_hz{0.0};
    uint64_t last_time_value_ns{0};
    double last_rtt_s{0.0};
    std::string status{"connected"};
    std::string detail;
    double services_wait_started_monotonic{0.0};
    double services_wait_grace_s{0.0};
    double pairing_requested_monotonic{0.0};
    int pairing_failures{0};
};

}  // namespace mrs_uav_bluetooth::peer
