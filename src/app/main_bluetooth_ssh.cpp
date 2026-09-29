// SPDX-License-Identifier: BSD-3-Clause
/// \file src/app/main_bluetooth_ssh.cpp
/// \brief Implements the main bluetooth ssh component of the ROS 2 application and operator-tool layer.

// Command-line UX and reconnectable serial transport inspired by ttyssh (MIT).
#include <unistd.h>

#include <cctype>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

/// \brief Resolve the directory containing the running launcher binary.
/// \return Canonical directory containing the running launcher executable.
std::string executable_directory() {
    // Resolve the directory containing the running launcher binary.
    std::vector<char> path(4096, '\0');
    const auto count = ::readlink("/proc/self/exe", path.data(), path.size() - 1);
    if (count < 0) {
        return {};
    }
    path[static_cast<size_t>(count)] = '\0';
    return std::filesystem::path(path.data()).parent_path().string();
}

/// \brief Extract the host part from a user-at-host SSH destination.
/// \param destination SSH destination or hostname from which the UAV name is extracted.
/// \return Validated UAV hostname extracted from the SSH destination.
std::string hostname_from_destination(const std::string& destination) {
    // Extract the host part from a user-at-host SSH destination.
    const auto separator = destination.rfind('@');
    return separator == std::string::npos ? destination : destination.substr(separator + 1);
}

/// \brief Build the stable tty symlink path for a validated peer hostname.
/// \param destination SSH destination or hostname from which the UAV name is extracted.
/// \return Terminal path from hostname.
std::string tty_path_from_hostname(const std::string& destination) {
    // Build the stable tty symlink path for a validated peer hostname.
    const auto hostname = hostname_from_destination(destination);
    if (hostname.empty()) {
        return {};
    }
    for (const unsigned char ch : hostname) {
        if (!std::isalnum(ch) && ch != '-' && ch != '.') {
            return {};
        }
    }
    return "/dev/ttyBLE_" + hostname;
}

/// \brief Build the stable tty symlink path for a peer Bluetooth address.
/// \param mac peer Bluetooth MAC address.
/// \return Terminal path from mac.
std::string tty_path_from_mac(const std::string& mac) {
    // Build the stable tty symlink path for a peer Bluetooth address.
    if (mac.size() != 17) {
        return {};
    }

    std::string normalized;
    normalized.reserve(mac.size());
    for (size_t index = 0; index < mac.size(); ++index) {
        const bool separator = index == 2 || index == 5 || index == 8 ||
            index == 11 || index == 14;
        if (separator) {
            if (mac[index] != ':') {
                return {};
            }
            normalized.push_back('-');
            continue;
        }
        const auto ch = static_cast<unsigned char>(mac[index]);
        if (!std::isxdigit(ch)) {
            return {};
        }
        normalized.push_back(static_cast<char>(std::toupper(ch)));
    }
    return "/dev/ttyBLE_" + normalized;
}

/// \brief Quote one argument so the remote shell cannot reinterpret its bytes.
/// \param value Single command-line argument to quote.
/// \return Single-quoted shell word that preserves every argument byte.
std::string shell_quote(const std::string& value) {
    // Quote one argument so the remote shell cannot reinterpret its bytes.
    std::string quoted{"'"};
    for (const char ch : value) {
        if (ch == '\'') {
            quoted += "'\\''";
        } else {
            quoted.push_back(ch);
        }
    }
    quoted.push_back('\'');
    return quoted;
}

/// \brief Format the command-line forms accepted by the Bluetooth SSH launcher.
/// \param program executable name inserted into usage text.
void usage(const char* program) {
    // Format the command-line forms accepted by the Bluetooth SSH launcher.
    std::fprintf(stderr,
                 "usage: %s [--device PATH | --mac MAC] [--user USER] PEER_HOST [-- SSH_OPTIONS...]\n",
                 program);
}

}  // namespace

/// \brief Resolve a peer Bluetooth terminal and replace this process with SSH.
/// \param argc number of command-line arguments.
/// \param argv command-line argument vector.
/// \return Zero on success, or a nonzero process status on failure.
int main(int argc, char** argv) {
    // Validate the peer identity, resolve its Bluetooth terminal, then replace this process with OpenSSH.
    std::string device;
    std::string mac;
    std::string user;
    std::string host;
    std::vector<std::string> ssh_options;

    int index = 1;
    while (index < argc) {
        const std::string argument = argv[index];
        if (argument == "--device" && index + 1 < argc) {
            device = argv[index + 1];
            index += 2;
            continue;
        }
        if (argument == "--mac" && index + 1 < argc) {
            mac = argv[index + 1];
            index += 2;
            continue;
        }
        if (argument == "--user" && index + 1 < argc) {
            user = argv[index + 1];
            index += 2;
            continue;
        }
        if (argument == "--") {
            ++index;
            break;
        }
        if (host.empty()) {
            host = argument;
            ++index;
            continue;
        }
        break;
    }
    while (index < argc) {
        ssh_options.emplace_back(argv[index++]);
    }

    if (host.empty()) {
        usage(argv[0]);
        return 64;
    }
    if (!device.empty() && !mac.empty()) {
        std::fprintf(stderr, "--device and --mac are mutually exclusive\n");
        return 64;
    }
    if (device.empty()) {
        if (!mac.empty()) {
            device = tty_path_from_mac(mac);
            if (device.empty()) {
                std::fprintf(stderr, "invalid MAC: %s\n", mac.c_str());
                return 64;
            }
        } else {
            device = tty_path_from_hostname(host);
            if (device.empty()) {
                std::fprintf(stderr,
                             "invalid peer hostname; select the connected link with --device or --mac\n");
                return 64;
            }
        }
    }

    const auto directory = executable_directory();
    const auto proxy_path = directory.empty()
        ? std::string{"mrs-uav-bluetooth-serial-proxy"}
        : directory + "/mrs-uav-bluetooth-serial-proxy";
    const auto proxy_command =
        "ProxyCommand=" + shell_quote(proxy_path) + " " + shell_quote(device);
    const auto destination = user.empty() || host.find('@') != std::string::npos
        ? host
        : user + "@" + host;

    std::vector<std::string> arguments;
    arguments.emplace_back("ssh");
    arguments.emplace_back("-o");
    arguments.push_back(proxy_command);
    arguments.emplace_back("-o");
    arguments.emplace_back("HostKeyAlias=" + hostname_from_destination(host));
    arguments.insert(arguments.end(), ssh_options.begin(), ssh_options.end());
    arguments.push_back(destination);

    std::vector<char*> raw_arguments;
    raw_arguments.reserve(arguments.size() + 1);
    for (auto& argument : arguments) {
        raw_arguments.push_back(argument.data());
    }
    raw_arguments.push_back(nullptr);

    ::execvp("ssh", raw_arguments.data());
    std::perror("execvp ssh");
    return 69;
}
