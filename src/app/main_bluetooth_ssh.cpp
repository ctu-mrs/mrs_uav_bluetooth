// SPDX-License-Identifier: BSD-3-Clause
// Command-line UX and reconnectable serial transport inspired by ttyssh (MIT).
#include <unistd.h>

#include <cctype>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

std::string executable_directory() {
    std::vector<char> path(4096, '\0');
    const auto count = ::readlink("/proc/self/exe", path.data(), path.size() - 1);
    if (count < 0) {
        return {};
    }
    path[static_cast<size_t>(count)] = '\0';
    return std::filesystem::path(path.data()).parent_path().string();
}

std::string hostname_from_destination(const std::string& destination) {
    const auto separator = destination.rfind('@');
    return separator == std::string::npos ? destination : destination.substr(separator + 1);
}

std::string tty_path_from_hostname(const std::string& destination) {
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

std::string tty_path_from_mac(const std::string& mac) {
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

std::string shell_quote(const std::string& value) {
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

void usage(const char* program) {
    std::fprintf(stderr,
                 "usage: %s [--device PATH | --mac MAC] [--user USER] PEER_HOST [-- SSH_OPTIONS...]\n",
                 program);
}

}  // namespace

int main(int argc, char** argv) {
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
