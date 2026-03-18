// SPDX-License-Identifier: MIT
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <memory>
#include <mutex>
#include <string>

namespace mrs_uav_bluetooth::util {

/// Thin wrapper that optionally mirrors log lines to a file and/or a ROS topic.
class VerboseLogger {
public:
    explicit VerboseLogger(rclcpp::Logger ros_logger)
        : ros_logger_(ros_logger) {}

    /// Open (or reopen) a file for verbose log output.  Passing an empty path
    /// closes any existing file.
    void open_file(const std::string& path);

    /// Log a verbose line to the ROS logger, the file sink, and (if enabled) a
    /// topic publisher.  Thread-safe.
    void log(const std::string& message);

    /// Set a publisher to use for log topic output.  Pass nullptr to disable.
    void set_topic_publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub);

    /// Enable or disable the log topic output.
    void set_topic_enabled(bool enabled);

private:
    rclcpp::Logger ros_logger_;
    std::mutex mutex_;
    std::ofstream file_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
    bool topic_enabled_{false};
};

}  // namespace mrs_uav_bluetooth::util
