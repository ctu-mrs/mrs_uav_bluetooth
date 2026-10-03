// SPDX-License-Identifier: BSD-3-Clause
/// \file src/serial/serial_link.cpp
/// \brief Implements the serial link component of the RFCOMM serial and SSH layer.

#include "mrs_uav_bluetooth/serial/serial_link.hpp"

#include <pty.h>
#include <signal.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <utility>

namespace mrs_uav_bluetooth::serial {

namespace {

class OwnedFd {
public:
    /// \brief Take ownership of this descriptor so every later exit path closes it once.
    /// \param fd File descriptor whose ownership is transferred into the guard.
    explicit OwnedFd(int fd = -1) : fd_(fd) {
        // Take ownership of this descriptor so every later exit path closes it once.
    }
    /// \brief Close the descriptor unless ownership was explicitly released.
    ~OwnedFd() {
        // Close the descriptor unless ownership was explicitly released.
        reset();
    }

    /// \brief Disable copying of the owned fd.
    OwnedFd(const OwnedFd&) = delete;
    /// \brief Assign state from another owned fd.
    OwnedFd& operator=(const OwnedFd&) = delete;

    /// \brief Return the descriptor without transferring ownership.
    /// \return Borrowed file descriptor without changing ownership.
    int get() const {
        // Return the descriptor without transferring ownership.
        return fd_;
    }

    /// \brief Transfer descriptor ownership to the caller and invalidate this guard.
    /// \return File descriptor transferred to the caller or -1 when empty.
    int release() {
        // Transfer descriptor ownership to the caller and invalidate this guard.
        const int fd = fd_;
        fd_ = -1;
        return fd;
    }

    /// \brief Close the owned descriptor and clear it to prevent double close.
    void reset() {
        // Close the owned descriptor and clear it to prevent double close.
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

private:
    int fd_;
};

/// \brief Deliver a complete buffer to either the RFCOMM socket or its pseudo-terminal.
/// \param fd Open descriptor receiving the complete buffer.
/// \param data First byte of the contiguous buffer to write.
/// \param size Exact number of bytes that must be written.
/// \param socket True selects send with SIGPIPE suppression; false selects write.
/// \return True only after every requested byte was written; otherwise false.
bool write_all(int fd, const uint8_t* data, size_t size, bool socket) {
    // Retry interruptions and short writes until the entire buffer reaches the socket or PTY.
    size_t offset = 0;
    while (offset < size) {
        const auto written = socket
            ? ::send(fd, data + offset, size - offset, MSG_NOSIGNAL)
            : ::write(fd, data + offset, size - offset);
        if (written > 0) {
            offset += static_cast<size_t>(written);
            continue;
        }
        if (written < 0 && errno == EINTR) {
            continue;
        }
        return false;
    }
    return true;
}

/// \brief Choose the private fallback directory for PTY links.
/// \return Private runtime directory used when the preferred terminal-link directory is unavailable.
std::string default_fallback_directory() {
    // Place fallback links in the caller's private runtime directory.
    if (const char* runtime_dir = std::getenv("XDG_RUNTIME_DIR");
        runtime_dir != nullptr && runtime_dir[0] != '\0') {
        return std::string{runtime_dir} + "/mrs-uav-bluetooth";
    }
    return "/tmp/mrs-uav-bluetooth-" + std::to_string(::getuid());
}

/// \brief Create a runtime directory and enforce owner-only permissions.
/// \param path Runtime directory that must exist with owner-only access.
void ensure_private_directory(const std::string& path) {
    // Create the fallback PTY directory and restrict it to the owning user.
    std::error_code error;
    std::filesystem::create_directories(path, error);
    if (error && !std::filesystem::is_directory(path)) {
        throw std::runtime_error("cannot create serial link directory " + path +
                                 ": " + error.message());
    }
    (void)::chmod(path.c_str(), S_IRWXU);
}
/// \brief Return whether readable system SSH host key is present.
/// \return True if at least one supported system SSH host key is readable; otherwise false.
bool has_readable_system_ssh_host_key() {
    // Any supported private host key is enough for the SSH endpoint to start.
    constexpr const char* paths[] = {
        "/etc/ssh/ssh_host_ed25519_key",
        "/etc/ssh/ssh_host_ecdsa_key",
        "/etc/ssh/ssh_host_rsa_key",
    };
    for (const auto* path : paths) {
        if (::access(path, R_OK) == 0) {
            return true;
        }
    }
    return false;
}

/// \brief Choose the per-user persistent path for a generated SSH host key.
/// \return Per-user Ed25519 host-key path under the package state directory.
std::string user_ssh_host_key_path() {
    // Choose the per-user persistent path for a generated SSH host key.
    if (const char* state_home = std::getenv("XDG_STATE_HOME");
        state_home != nullptr && state_home[0] != '\0' &&
        std::filesystem::path{state_home}.is_absolute()) {
        return (std::filesystem::path{state_home} /
                "mrs-uav-bluetooth" / "ssh_host_ed25519_key").string();
    }
    if (const char* home = std::getenv("HOME");
        home != nullptr && home[0] != '\0' &&
        std::filesystem::path{home}.is_absolute()) {
        return (std::filesystem::path{home} / ".local" / "state" /
                "mrs-uav-bluetooth" / "ssh_host_ed25519_key").string();
    }
    return default_fallback_directory() + "/ssh_host_ed25519_key";
}

/// \brief Install a private per-user SSH host key from a readable system key when absent.
/// \param key_path path of the key.
/// \return True if a private user host key exists after the call; otherwise false.
bool ensure_user_ssh_host_key(const std::string& key_path) {
    // Ensure user SSH host key.
    struct stat key_stat {};
    if (::lstat(key_path.c_str(), &key_stat) == 0) {
        if (!S_ISREG(key_stat.st_mode)) {
            throw std::runtime_error("SSH host key is not a regular file: " + key_path);
        }
        if (::access(key_path.c_str(), R_OK) != 0) {
            throw std::runtime_error("SSH host key is not readable: " + key_path +
                                     ": " + std::strerror(errno));
        }
        (void)::chmod(key_path.c_str(), S_IRUSR | S_IWUSR);
        return false;
    }
    if (errno != ENOENT) {
        throw std::runtime_error("cannot inspect SSH host key " + key_path +
                                 ": " + std::strerror(errno));
    }

    const auto parent = std::filesystem::path{key_path}.parent_path();
    if (parent.empty()) {
        throw std::runtime_error("SSH host key path has no parent directory: " + key_path);
    }
    ensure_private_directory(parent.string());

    const auto temporary_path = key_path + ".tmp." + std::to_string(::getpid());
    const auto temporary_public_path = temporary_path + ".pub";
    (void)::unlink(temporary_path.c_str());
    (void)::unlink(temporary_public_path.c_str());

    const auto pid = ::fork();
    if (pid < 0) {
        throw std::runtime_error("fork for ssh-keygen failed: " +
                                 std::string{std::strerror(errno)});
    }
    if (pid == 0) {
        ::execl("/usr/bin/ssh-keygen", "/usr/bin/ssh-keygen",
                "-q", "-t", "ed25519", "-N", "", "-f",
                temporary_path.c_str(), nullptr);
        _exit(127);
    }

    int status = 0;
    pid_t wait_result = -1;
    do {
        wait_result = ::waitpid(pid, &status, 0);
    } while (wait_result < 0 && errno == EINTR);
    if (wait_result < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        (void)::unlink(temporary_path.c_str());
        (void)::unlink(temporary_public_path.c_str());
        throw std::runtime_error("ssh-keygen failed while creating " + key_path);
    }

    (void)::chmod(temporary_path.c_str(), S_IRUSR | S_IWUSR);
    std::error_code error;
    std::filesystem::rename(temporary_path, key_path, error);
    if (error) {
        (void)::unlink(temporary_path.c_str());
        (void)::unlink(temporary_public_path.c_str());
        throw std::runtime_error("cannot install SSH host key " + key_path +
                                 ": " + error.message());
    }
    std::filesystem::rename(temporary_public_path, key_path + ".pub", error);
    if (error) {
        (void)::unlink(temporary_public_path.c_str());
    }
    return true;
}

}  // namespace

struct SerialLinkManager::Link {
    std::string device_path;
    std::string peer_name;
    std::string tty_path;
    std::string pty_path;
    int socket_fd{-1};
    int pty_master_fd{-1};
    int pty_slave_fd{-1};
    bool used_fallback_path{false};
    std::atomic_bool stop{false};
    std::atomic_bool active{true};
    std::thread worker;
};

SerialLinkManager::SerialLinkManager(rclcpp::Logger logger,
                                     std::string preferred_directory,
                                     std::string fallback_directory)
    : logger_(logger),
      preferred_directory_(std::move(preferred_directory)),
      fallback_directory_(fallback_directory.empty()
          ? default_fallback_directory()
          : std::move(fallback_directory)) {
              // Select a private fallback directory when the preferred device directory is unavailable.
          }

SerialLinkManager::~SerialLinkManager() {
    // Stop every bridge thread and remove only symlinks created by this manager.
    detach_all();
}

std::string SerialLinkManager::tty_basename_for_peer(const std::string& peer_name,
                                                     const std::string& peer_mac) {
    // Prefer a safe hostname in the tty name, otherwise encode the peer address.
    if (!peer_name.empty()) {
        bool valid = true;
        for (const unsigned char ch : peer_name) {
            if (!std::isalnum(ch) && ch != '-' && ch != '_' && ch != '.') {
                valid = false;
                break;
            }
        }
        if (valid) {
            return "ttyBLE_" + peer_name;
        }
    }
    if (peer_mac.size() != 17) {
        throw std::invalid_argument("peer MAC must use XX:XX:XX:XX:XX:XX format");
    }

    std::string normalized;
    normalized.reserve(peer_mac.size());
    for (size_t index = 0; index < peer_mac.size(); ++index) {
        const bool separator = index == 2 || index == 5 || index == 8 ||
            index == 11 || index == 14;
        if (separator) {
            if (peer_mac[index] != ':') {
                throw std::invalid_argument("peer MAC must use XX:XX:XX:XX:XX:XX format");
            }
            normalized.push_back('-');
            continue;
        }
        const auto ch = static_cast<unsigned char>(peer_mac[index]);
        if (!std::isxdigit(ch)) {
            throw std::invalid_argument("peer MAC must contain hexadecimal octets");
        }
        normalized.push_back(static_cast<char>(std::toupper(ch)));
    }
    return "ttyBLE_" + normalized;
}

void SerialLinkManager::remove_owned_link(const std::string& link_path,
                                          const std::string& pty_path) {
    // Unlink only when the current symlink still targets this session’s PTY.
    if (link_path.empty()) {
        return;
    }

    std::vector<char> target(512, '\0');
    const auto size = ::readlink(link_path.c_str(), target.data(), target.size() - 1);
    if (size < 0) {
        return;
    }
    target[static_cast<size_t>(size)] = '\0';
    if (pty_path == target.data()) {
        (void)::unlink(link_path.c_str());
    }
}

std::string SerialLinkManager::create_tty_link(const std::string& basename,
                                                const std::string& pty_path,
                                                bool& used_fallback) const {
    // Link the new PTY under the preferred directory, falling back to the private runtime directory.
    used_fallback = false;
    const auto preferred = preferred_directory_ + "/" + basename;
    if (::symlink(pty_path.c_str(), preferred.c_str()) == 0) {
        return preferred;
    }
    if (errno == EEXIST) {
        std::vector<char> target(512, '\0');
        const auto size = ::readlink(preferred.c_str(), target.data(), target.size() - 1);
        if (size >= 0) {
            target[static_cast<size_t>(size)] = '\0';
            if (pty_path == target.data()) {
                return preferred;
            }
        }
    }

    ensure_private_directory(fallback_directory_);
    const auto fallback = fallback_directory_ + "/" + basename;
    if (::symlink(pty_path.c_str(), fallback.c_str()) != 0) {
        if (errno == EEXIST) {
            std::vector<char> target(512, '\0');
            const auto size = ::readlink(fallback.c_str(), target.data(), target.size() - 1);
            if (size >= 0) {
                target[static_cast<size_t>(size)] = '\0';
                if (pty_path == target.data()) {
                    used_fallback = true;
                    return fallback;
                }
            }
        }
        throw std::runtime_error("cannot create serial device link " + fallback +
                                 ": " + std::strerror(errno));
    }
    used_fallback = true;
    return fallback;
}

SerialLinkInfo SerialLinkManager::attach(const std::string& device_path,
                                         const std::string& peer_name,
                                         const std::string& peer_mac,
                                         int socket_fd) {
    // Take socket ownership, create a PTY pair, publish its stable link, and start forwarding.
    if (socket_fd < 0) {
        throw std::invalid_argument("invalid RFCOMM socket descriptor");
    }
    OwnedFd owned_socket{socket_fd};

    detach(device_path);

    int master_fd = -1;
    int slave_fd = -1;
    char pty_name[128] = {};
    if (::openpty(&master_fd, &slave_fd, pty_name, nullptr, nullptr) != 0) {
        const auto error = std::string{std::strerror(errno)};
        throw std::runtime_error("openpty failed: " + error);
    }
    OwnedFd owned_master{master_fd};
    OwnedFd owned_slave{slave_fd};

    termios attributes{};
    if (::tcgetattr(slave_fd, &attributes) == 0) {
        ::cfmakeraw(&attributes);
        attributes.c_cflag |= CLOCAL | CREAD;
        (void)::tcsetattr(slave_fd, TCSANOW, &attributes);
    }

    auto link = std::make_shared<Link>();
    link->device_path = device_path;
    link->peer_name = peer_name.empty() ? peer_mac : peer_name;
    link->pty_path = pty_name;
    link->socket_fd = socket_fd;
    link->pty_master_fd = master_fd;
    link->pty_slave_fd = slave_fd;

    link->tty_path = create_tty_link(
        tty_basename_for_peer(peer_name, peer_mac),
        link->pty_path,
        link->used_fallback_path);

    try {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto [iterator, inserted] = links_.emplace(device_path, link);
        if (!inserted) {
            throw std::runtime_error("serial link already exists for device");
        }
        try {
            link->worker = std::thread([link]() {
                // Forward bytes bidirectionally between the RFCOMM socket and PTY until either side closes.
                run_bridge(link);
            });
        } catch (...) {
            links_.erase(iterator);
            throw;
        }
    } catch (...) {
        remove_owned_link(link->tty_path, link->pty_path);
        throw;
    }

    (void)owned_socket.release();
    (void)owned_master.release();
    (void)owned_slave.release();

    RCLCPP_INFO(logger_, "Serial link %s -> %s established for %s%s",
                link->tty_path.c_str(), link->pty_path.c_str(), link->peer_name.c_str(),
                link->used_fallback_path ? " (runtime fallback; /dev was unavailable)" : "");

    return SerialLinkInfo{
        link->device_path,
        link->peer_name,
        link->tty_path,
        link->pty_path,
        true,
        link->used_fallback_path};
}

void SerialLinkManager::run_bridge(const std::shared_ptr<Link>& link) {
    // Poll the RFCOMM socket and PTY together, forwarding readable bytes in either direction.
    std::vector<uint8_t> buffer(4096);
    while (!link->stop.load()) {
        pollfd descriptors[2] = {
            {link->socket_fd, POLLIN, 0},
            {link->pty_master_fd, POLLIN, 0},
        };
        const auto ready = ::poll(descriptors, 2, 500);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (ready == 0) {
            continue;
        }

        if ((descriptors[0].revents & POLLIN) != 0) {
            const auto count = ::read(link->socket_fd, buffer.data(), buffer.size());
            if (count <= 0 ||
                !write_all(link->pty_master_fd, buffer.data(), static_cast<size_t>(count), false)) {
                break;
            }
        }
        if ((descriptors[1].revents & POLLIN) != 0) {
            const auto count = ::read(link->pty_master_fd, buffer.data(), buffer.size());
            if (count <= 0 ||
                !write_all(link->socket_fd, buffer.data(), static_cast<size_t>(count), true)) {
                break;
            }
        }

        const short terminal_events = POLLERR | POLLNVAL;
        if ((descriptors[0].revents & (terminal_events | POLLHUP)) != 0 ||
            (descriptors[1].revents & terminal_events) != 0) {
            break;
        }
    }

    // Closing the PTY master is what delivers POLLHUP/EIO to every process
    // holding the slave. Merely marking this link inactive leaves the SSH
    // ProxyCommand blocked forever after RFCOMM or the remote sshd exits.
    if (link->socket_fd >= 0) {
        (void)::shutdown(link->socket_fd, SHUT_RDWR);
    }
    if (link->pty_master_fd >= 0) {
        ::close(link->pty_master_fd);
        link->pty_master_fd = -1;
    }
    link->active.store(false);
    remove_owned_link(link->tty_path, link->pty_path);
}

void SerialLinkManager::detach(const std::string& device_path) {
    // Detach serial link manager.
    std::shared_ptr<Link> link;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = links_.find(device_path);
        if (it == links_.end()) {
            return;
        }
        link = it->second;
        links_.erase(it);
    }

    link->stop.store(true);
    if (link->socket_fd >= 0) {
        (void)::shutdown(link->socket_fd, SHUT_RDWR);
    }
    if (link->worker.joinable()) {
        link->worker.join();
    }
    remove_owned_link(link->tty_path, link->pty_path);
    if (link->socket_fd >= 0) {
        ::close(link->socket_fd);
    }
    if (link->pty_master_fd >= 0) {
        ::close(link->pty_master_fd);
    }
    if (link->pty_slave_fd >= 0) {
        ::close(link->pty_slave_fd);
    }
    link->active.store(false);
    RCLCPP_INFO(logger_, "Serial link for %s closed", link->peer_name.c_str());
}

void SerialLinkManager::detach_all() {
    // Detach all.
    std::vector<std::string> paths;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        paths.reserve(links_.size());
        for (const auto& [path, _] : links_) {
            paths.push_back(path);
        }
    }
    for (const auto& path : paths) {
        detach(path);
    }
}

std::optional<SerialLinkInfo> SerialLinkManager::link_for_device(
    const std::string& device_path) const {
    // Snapshot the active PTY link associated with one BlueZ device path.
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = links_.find(device_path);
    if (it == links_.end() || !it->second->active.load()) {
        return std::nullopt;
    }
    const auto& link = it->second;
    return SerialLinkInfo{
        link->device_path,
        link->peer_name,
        link->tty_path,
        link->pty_path,
        true,
        link->used_fallback_path};
}

std::vector<SerialLinkInfo> SerialLinkManager::links() const {
    // Snapshot active links under lock without exposing mutable worker state.
    std::vector<SerialLinkInfo> result;
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& [_, link] : links_) {
        if (!link->active.load()) {
            continue;
        }
        result.push_back(SerialLinkInfo{
            link->device_path,
            link->peer_name,
            link->tty_path,
            link->pty_path,
            true,
            link->used_fallback_path});
    }
    return result;
}

bool SerialLinkManager::wait_for_link(const std::string& device_path,
                                      std::chrono::milliseconds timeout,
                                      SerialLinkInfo* result) const {
    // Poll until the device link becomes active or the caller timeout expires.
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
        if (const auto link = link_for_device(device_path)) {
            if (result != nullptr) {
                *result = *link;
            }
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
}

struct SerialSshServer::Session {
    pid_t pid{-1};
    std::atomic_bool active{true};
    std::thread waiter;
};

SerialSshServer::SerialSshServer(rclcpp::Logger logger, std::string sshd_path)
    : logger_(logger), sshd_path_(std::move(sshd_path)) {
    // Select the SSH daemon and prepare a per-user host key when no system key is readable.
    if (!has_readable_system_ssh_host_key()) {
        host_key_path_ = user_ssh_host_key_path();
    }
}

SerialSshServer::~SerialSshServer() {
    // Terminate and reap every SSH child before destroying session records.
    stop_all();
}

void SerialSshServer::start_session(const std::string& device_path, int socket_fd) {
    // Give one accepted RFCOMM socket to an inetd-style sshd child and track its PID.
    if (socket_fd < 0) {
        throw std::invalid_argument("invalid RFCOMM socket descriptor");
    }
    OwnedFd owned_socket{socket_fd};
    if (::access(sshd_path_.c_str(), X_OK) != 0) {
        const auto error = std::string{std::strerror(errno)};
        throw std::runtime_error("sshd is not executable at " + sshd_path_ + ": " + error);
    }
    if (!host_key_path_.empty()) {
        const bool generated = ensure_user_ssh_host_key(host_key_path_);
        RCLCPP_INFO(logger_, "%s SSH host key %s for unprivileged serial sshd",
                    generated ? "Generated" : "Using",
                    host_key_path_.c_str());
    }

    stop_session(device_path);
    auto session = std::make_shared<Session>();

    const auto pid = ::fork();
    if (pid < 0) {
        const auto error = std::string{std::strerror(errno)};
        throw std::runtime_error("fork for sshd failed: " + error);
    }
    if (pid == 0) {
        if (::dup2(socket_fd, STDIN_FILENO) < 0 ||
            ::dup2(socket_fd, STDOUT_FILENO) < 0) {
            _exit(126);
        }
        if (socket_fd > STDERR_FILENO) {
            ::close(socket_fd);
        }
        if (host_key_path_.empty()) {
            ::execl(sshd_path_.c_str(), sshd_path_.c_str(), "-i", "-e",
                    "-o", "LoginGraceTime=0", nullptr);
        } else {
            ::execl(sshd_path_.c_str(), sshd_path_.c_str(), "-i", "-e",
                    "-f", "/dev/null",
                    "-h", host_key_path_.c_str(),
                    "-o", "LoginGraceTime=0",
                    "-o", "UsePAM=yes",
                    "-o", "PasswordAuthentication=yes",
                    "-o", "PermitRootLogin=no",
                    "-o", "PermitEmptyPasswords=no", nullptr);
        }
        _exit(127);
    }

    owned_socket.reset();
    session->pid = pid;
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto [iterator, inserted] = sessions_.emplace(device_path, session);
        if (!inserted) {
            throw std::runtime_error("SSH session already exists for device");
        }
        try {
            session->waiter = std::thread([session, device_path, logger = logger_]() {
                // Reap the SSH child, report why the endpoint stopped, and
                // then let peer reconciliation resume normal GATT policy.
                int status = 0;
                pid_t result = -1;
                do {
                    result = ::waitpid(session->pid, &status, 0);
                } while (result < 0 && errno == EINTR);
                session->active.store(false);
                if (result < 0) {
                    RCLCPP_WARN(logger, "Could not reap Bluetooth sshd for %s: %s",
                                device_path.c_str(), std::strerror(errno));
                } else if (WIFEXITED(status)) {
                    const int exit_code = WEXITSTATUS(status);
                    if (exit_code == 0) {
                        RCLCPP_INFO(logger, "Bluetooth sshd for %s exited with status 0",
                                    device_path.c_str());
                    } else {
                        RCLCPP_WARN(logger, "Bluetooth sshd for %s exited with status %d",
                                    device_path.c_str(), exit_code);
                    }
                } else if (WIFSIGNALED(status)) {
                    RCLCPP_WARN(logger, "Bluetooth sshd for %s terminated by signal %d",
                                device_path.c_str(), WTERMSIG(status));
                }
            });
        } catch (...) {
            sessions_.erase(iterator);
            throw;
        }
    } catch (...) {
        (void)::kill(pid, SIGTERM);
        int status = 0;
        while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
        }
        session->active.store(false);
        throw;
    }
    RCLCPP_INFO(logger_, "Started Bluetooth SSH session for %s (pid=%d)",
                device_path.c_str(), static_cast<int>(pid));
}

void SerialSshServer::stop_session(const std::string& device_path) {
    // Remove one device session, terminate a live child, and join its waiter thread.
    std::shared_ptr<Session> session;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = sessions_.find(device_path);
        if (it == sessions_.end()) {
            return;
        }
        session = it->second;
        sessions_.erase(it);
    }

    if (session->active.load() && session->pid > 0) {
        (void)::kill(session->pid, SIGTERM);
    }
    if (session->waiter.joinable()) {
        session->waiter.join();
    }
    session->active.store(false);
}

void SerialSshServer::stop_all() {
    // Snapshot device keys so every session can be stopped outside the map lock.
    std::vector<std::string> paths;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        paths.reserve(sessions_.size());
        for (const auto& [path, _] : sessions_) {
            paths.push_back(path);
        }
    }
    for (const auto& path : paths) {
        stop_session(path);
    }
}

bool SerialSshServer::active(const std::string& device_path) const {
    // Check under lock whether this device still owns a live SSH child.
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = sessions_.find(device_path);
    return it != sessions_.end() && it->second->active.load();
}

}  // namespace mrs_uav_bluetooth::serial
