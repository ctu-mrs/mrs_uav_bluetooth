// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace mrs_uav_bluetooth::serial {

struct SerialLinkInfo {
    std::string device_path;
    std::string peer_name;
    std::string tty_path;
    std::string pty_path;
    bool active{false};
    bool used_fallback_path{false};
};

/// Bridges BlueZ-provided RFCOMM sockets to Unix pseudo terminals.
class SerialLinkManager {
public:
    SerialLinkManager(rclcpp::Logger logger,
                      std::string preferred_directory = "/dev",
                      std::string fallback_directory = {});
    ~SerialLinkManager();

    SerialLinkManager(const SerialLinkManager&) = delete;
    SerialLinkManager& operator=(const SerialLinkManager&) = delete;

    /// Takes ownership of a valid socket_fd immediately, including on failure.
    SerialLinkInfo attach(const std::string& device_path,
                          const std::string& peer_name,
                          const std::string& peer_mac,
                          int socket_fd);
    void detach(const std::string& device_path);
    void detach_all();

    std::optional<SerialLinkInfo> link_for_device(const std::string& device_path) const;
    std::vector<SerialLinkInfo> links() const;
    bool wait_for_link(const std::string& device_path,
                       std::chrono::milliseconds timeout,
                       SerialLinkInfo* result = nullptr) const;

    static std::string tty_basename_for_peer(const std::string& peer_name,
                                             const std::string& peer_mac);

private:
    struct Link;

    std::string create_tty_link(const std::string& basename,
                                const std::string& pty_path,
                                bool& used_fallback) const;
    static void remove_owned_link(const std::string& link_path,
                                  const std::string& pty_path);
    static void run_bridge(const std::shared_ptr<Link>& link);

    rclcpp::Logger logger_;
    std::string preferred_directory_;
    std::string fallback_directory_;
    mutable std::mutex mutex_;
    std::map<std::string, std::shared_ptr<Link>> links_;
};

/// Runs one OpenSSH sshd inetd session for each incoming RFCOMM connection.
class SerialSshServer {
public:
    explicit SerialSshServer(rclcpp::Logger logger,
                             std::string sshd_path = "/usr/sbin/sshd");
    ~SerialSshServer();

    SerialSshServer(const SerialSshServer&) = delete;
    SerialSshServer& operator=(const SerialSshServer&) = delete;

    /// Takes ownership of a valid socket_fd immediately, including on failure.
    void start_session(const std::string& device_path, int socket_fd);
    void stop_session(const std::string& device_path);
    void stop_all();

    bool active(const std::string& device_path) const;

private:
    struct Session;

    rclcpp::Logger logger_;
    std::string sshd_path_;
    mutable std::mutex mutex_;
    std::map<std::string, std::shared_ptr<Session>> sessions_;
};

}  // namespace mrs_uav_bluetooth::serial
