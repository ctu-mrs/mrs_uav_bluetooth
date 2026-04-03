// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/util/logging_utils.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <std_msgs/msg/string.hpp>

namespace mrs_uav_bluetooth::util {

void VerboseLogger::open_file(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) {
        file_.close();
    }
    if (path.empty()) return;
    auto dir = std::filesystem::path(path).parent_path();
    if (!dir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
    }
    file_.open(path, std::ios::app);
    if (file_.is_open()) {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        file_ << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S")
              << " [INFO] Verbose file logger started\n";
        file_.flush();
    }
}

void VerboseLogger::log(const std::string& message) {
    RCLCPP_INFO(ros_logger_, "[VERBOSE] %s", message.c_str());

    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        file_ << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S")
              << " [DEBUG] " << message << "\n";
        file_.flush();
    }

    if (topic_enabled_ && log_pub_) {
        auto msg = std_msgs::msg::String();
        msg.data = message;
        try {
            log_pub_->publish(msg);
        } catch (...) {
            // Swallow if shutting down.
        }
    }
}

void VerboseLogger::set_topic_publisher(
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub) {
    std::lock_guard<std::mutex> lock(mutex_);
    log_pub_ = std::move(pub);
}

void VerboseLogger::set_topic_enabled(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    topic_enabled_ = enabled;
}

}  // namespace mrs_uav_bluetooth::util
