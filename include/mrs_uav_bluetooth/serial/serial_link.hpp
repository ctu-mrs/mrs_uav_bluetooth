// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/serial/serial_link.hpp
/// \brief Declares the serial link component of the RFCOMM serial and SSH layer.

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

/// Public status for one RFCOMM-backed pseudo-terminal.
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
    /// \brief Create the owner of RFCOMM-to-PTY forwarding links.
    /// \param logger ROS logger used for diagnostics.
    /// \param preferred_directory directory in which stable PTY symlinks should first be created.
    /// \param fallback_directory private directory used after preferred PTY-link creation fails.
    SerialLinkManager(rclcpp::Logger logger,
                      std::string preferred_directory = "/dev",
                      std::string fallback_directory = {});
    /// \brief Stop forwarding threads and remove owned PTY links.
    ~SerialLinkManager();

    /// \brief Disable copying of the serial link manager.
    SerialLinkManager(const SerialLinkManager&) = delete;
    /// \brief Disable copy assignment of the serial link manager.
    SerialLinkManager& operator=(const SerialLinkManager&) = delete;

    /// Takes ownership of a valid socket_fd immediately, including on failure.
    /// \param device_path BlueZ device path that owns the accepted serial connection.
    /// \param peer_name Resolved peer hostname recorded with the serial link.
    /// \param peer_mac Bluetooth address used when no safe peer hostname is available.
    /// \param socket_fd owned RFCOMM socket descriptor transferred into the local consumer.
    /// \return Serial link record that owns the accepted file descriptor.
    SerialLinkInfo attach(const std::string& device_path,
                          const std::string& peer_name,
                          const std::string& peer_mac,
                          int socket_fd);
    /// \brief Close one peer's serial bridge and remove only the terminal link it owns.
    /// \param device_path BlueZ device path whose serial connection is being removed.
    void detach(const std::string& device_path);
    /// \brief Close every active serial bridge and wake threads waiting for links.
    void detach_all();

    /// \brief Copy active serial-link metadata for one device.
    /// \param device_path BlueZ device path used to look up an active serial link.
    /// \return Active serial-link snapshot when present; otherwise std::nullopt.
    std::optional<SerialLinkInfo> link_for_device(const std::string& device_path) const;
    /// \brief Snapshot metadata for every active serial link.
    /// \return Snapshots of all active serial links.
    std::vector<SerialLinkInfo> links() const;
    /// \brief Wait until the named peer owns an incoming RFCOMM-to-PTY link.
    /// \param device_path BlueZ device path whose incoming serial link is awaited.
    /// \param timeout Maximum time to wait for an incoming serial-profile connection.
    /// \param result Optional destination receiving the attached link record.
    /// \return True if the named device attached before the deadline; otherwise false.
    bool wait_for_link(const std::string& device_path,
                       std::chrono::milliseconds timeout,
                       SerialLinkInfo* result = nullptr) const;

    /// \brief Build a safe tty basename from peer hostname or address.
    /// \param peer_name Preferred peer hostname used to construct the stable terminal name.
    /// \param peer_mac Bluetooth address used when no safe peer hostname is available.
    /// \return Safe tty symlink basename for the peer.
    static std::string tty_basename_for_peer(const std::string& peer_name,
                                             const std::string& peer_mac);

private:
    struct Link;

    /// \brief Create a stable peer-named symlink to a pseudo-terminal.
    /// \param basename safe PTY symlink basename created for the peer.
    /// \param pty_path Pseudo-terminal slave path that the stable symlink must target.
    /// \param used_fallback output set when the PTY link had to use the private fallback directory.
    /// \return Created preferred or fallback tty symlink path.
    std::string create_tty_link(const std::string& basename,
                                const std::string& pty_path,
                                bool& used_fallback) const;
    /// \brief Remove the terminal symlink only if it still targets this bridge's PTY.
    /// \param link_path Candidate stable symlink owned by this serial session.
    /// \param pty_path Pseudo-terminal slave path that the stable symlink must target.
    static void remove_owned_link(const std::string& link_path,
                                  const std::string& pty_path);
    /// \brief Copy bytes bidirectionally between the RFCOMM socket and pseudo-terminal until either closes.
    /// \param link serial link shared with its worker thread.
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
    /// \brief Create the owner of per-peer SSH daemon children.
    /// \param logger ROS logger used for diagnostics.
    /// \param sshd_path path of the SSHd.
    explicit SerialSshServer(rclcpp::Logger logger,
                             std::string sshd_path = "/usr/sbin/sshd");
    /// \brief Terminate and reap every per-peer SSH child.
    ~SerialSshServer();

    /// \brief Disable copying of the serial SSH server.
    SerialSshServer(const SerialSshServer&) = delete;
    /// \brief Disable copy assignment of the serial SSH server.
    SerialSshServer& operator=(const SerialSshServer&) = delete;

    /// Takes ownership of a valid socket_fd immediately, including on failure.
    /// \param device_path BlueZ device path used as the serial session key.
    /// \param socket_fd owned RFCOMM socket descriptor transferred into the local consumer.
    void start_session(const std::string& device_path, int socket_fd);
    /// \brief Terminate and reap the SSH child serving one Bluetooth device.
    /// \param device_path BlueZ device path identifying the serial session.
    void stop_session(const std::string& device_path);
    /// \brief Terminate and reap every active Bluetooth SSH child.
    void stop_all();

    /// \brief Report whether this serial proxy session still owns an open link.
    /// \param device_path BlueZ device path identifying the serial session.
    /// \return True when the object or session is active; otherwise false.
    bool active(const std::string& device_path) const;

private:
    struct Session;

    rclcpp::Logger logger_;
    std::string sshd_path_;
    std::string host_key_path_;
    mutable std::mutex mutex_;
    std::map<std::string, std::shared_ptr<Session>> sessions_;
};

}  // namespace mrs_uav_bluetooth::serial
