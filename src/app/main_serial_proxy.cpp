// SPDX-License-Identifier: BSD-3-Clause
// Serial forwarding behavior inspired by b1f6c1c4/ttyssh (MIT).
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

bool write_all(int fd, const char* data, size_t size) {
    size_t offset = 0;
    while (offset < size) {
        const auto written = ::write(fd, data + offset, size - offset);
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

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s /dev/ttyBLE_<hostname-or-AA-BB-CC-DD-EE-FF>\n", argv[0]);
        return 64;
    }

    const int tty_fd = ::open(argv[1], O_RDWR | O_NOCTTY);
    if (tty_fd < 0) {
        std::fprintf(stderr, "cannot open %s: %s\n", argv[1], std::strerror(errno));
        return 66;
    }
    (void)::ioctl(tty_fd, TIOCEXCL);

    termios original{};
    const bool have_original = ::tcgetattr(tty_fd, &original) == 0;
    if (have_original) {
        auto raw = original;
        ::cfmakeraw(&raw);
        raw.c_cflag |= CLOCAL | CREAD;
        (void)::tcsetattr(tty_fd, TCSANOW, &raw);
    }

    std::vector<char> buffer(4096);
    bool running = true;
    while (running) {
        pollfd descriptors[2] = {
            {STDIN_FILENO, POLLIN, 0},
            {tty_fd, POLLIN, 0},
        };
        const auto ready = ::poll(descriptors, 2, -1);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }

        if ((descriptors[0].revents & POLLIN) != 0) {
            const auto count = ::read(STDIN_FILENO, buffer.data(), buffer.size());
            if (count <= 0 || !write_all(tty_fd, buffer.data(), static_cast<size_t>(count))) {
                running = false;
            }
        }
        if ((descriptors[1].revents & POLLIN) != 0) {
            const auto count = ::read(tty_fd, buffer.data(), buffer.size());
            if (count <= 0 || !write_all(STDOUT_FILENO, buffer.data(), static_cast<size_t>(count))) {
                running = false;
            }
        }

        const short terminal_events = POLLERR | POLLHUP | POLLNVAL;
        if ((descriptors[0].revents & terminal_events) != 0 ||
            (descriptors[1].revents & terminal_events) != 0) {
            running = false;
        }
    }

    if (have_original) {
        (void)::tcsetattr(tty_fd, TCSANOW, &original);
    }
    (void)::ioctl(tty_fd, TIOCNXCL);
    ::close(tty_fd);
    return 0;
}
