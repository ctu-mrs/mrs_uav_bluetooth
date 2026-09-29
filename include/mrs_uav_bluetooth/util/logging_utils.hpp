// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/util/logging_utils.hpp
/// \brief Declares the logging utils component of the shared utility layer.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <fstream>
#include <memory>
#include <mutex>
#include <string>

namespace mrs_uav_bluetooth::util {

/// Thin wrapper that optionally mirrors log lines to a file and/or a ROS topic.
class VerboseLogger {
public:
    /// \brief Create a logger that can mirror messages to an optional ROS topic.
    /// \param ros_logger ROS logger used for warnings and operation diagnostics.
    explicit VerboseLogger(rclcpp::Logger ros_logger)
        : ros_logger_(ros_logger) {
            // Retain the ROS logger used for normal output; topic mirroring is optional.
        }

    /// Open (or reopen) a file for verbose log output.  Passing an empty path
    /// closes any existing file.
    /// \param path Log file to create or append.
    void open_file(const std::string& path);

    /// Log a verbose line to the ROS logger, the file sink, and (if enabled) a
    /// topic publisher.  Thread-safe.
    /// \param message Complete log line to write.
    void log(const std::string& message);

    /// Set a publisher to use for log topic output.  Pass nullptr to disable.
    /// \param pub ROS publisher used as the optional verbose-log sink.
    void set_topic_publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub);

    /// Enable or disable the log topic output.
    /// \param enabled Whether new log records are also published on ROS.
    void set_topic_enabled(bool enabled);

private:
    rclcpp::Logger ros_logger_;
    std::mutex mutex_;
    std::ofstream file_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
    bool topic_enabled_{false};
};

}  // namespace mrs_uav_bluetooth::util
