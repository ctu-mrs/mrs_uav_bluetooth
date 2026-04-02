// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/raw_terminal.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace mrs_uav_bluetooth::app {

namespace {

bool read_single_byte(int fd, char& ch) {
    const auto rc = ::read(fd, &ch, 1);
    return rc == 1;
}

void write_all(int fd, const std::string& text) {
    const char* data = text.data();
    size_t remaining = text.size();
    while (remaining > 0) {
        const auto written = ::write(fd, data, remaining);
        if (written <= 0) {
            break;
        }
        data += written;
        remaining -= static_cast<size_t>(written);
    }
}

}  // namespace

RawTerminal::RawTerminal() {
    const char* term = std::getenv("TERM");
    if (term == nullptr || std::strcmp(term, "dumb") == 0) {
        return;
    }

    tty_fd_ = ::open("/dev/tty", O_RDWR | O_NOCTTY);
    if (tty_fd_ < 0 && ::isatty(STDIN_FILENO)) {
        tty_fd_ = ::dup(STDIN_FILENO);
    }
    if (tty_fd_ < 0 || !::isatty(tty_fd_)) {
        if (tty_fd_ >= 0) {
            ::close(tty_fd_);
            tty_fd_ = -1;
        }
        return;
    }

    original_termios_ = new termios();
    if (::tcgetattr(tty_fd_, original_termios_) != 0) {
        delete original_termios_;
        original_termios_ = nullptr;
        return;
    }
    termios_saved_ = true;

    termios raw = *original_termios_;
    raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
    raw.c_iflag &= static_cast<unsigned long>(~(IXON | ICRNL));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (::tcsetattr(tty_fd_, TCSAFLUSH, &raw) != 0) {
        return;
    }

    const auto flags = ::fcntl(tty_fd_, F_GETFL, 0);
    if (flags >= 0) {
        original_flags_ = flags;
        flags_saved_ = true;
        ::fcntl(tty_fd_, F_SETFL, flags | O_NONBLOCK);
    }

    write_all(tty_fd_, "\x1b[?1049h\x1b[?25l\x1b[2J\x1b[H");
    active_ = true;
}

RawTerminal::~RawTerminal() {
    if (flags_saved_ && tty_fd_ >= 0) {
        ::fcntl(tty_fd_, F_SETFL, original_flags_);
    }
    if (termios_saved_ && original_termios_ != nullptr && tty_fd_ >= 0) {
        ::tcsetattr(tty_fd_, TCSAFLUSH, original_termios_);
    }
    if (active_ && tty_fd_ >= 0) {
        write_all(tty_fd_, "\x1b[?25h\x1b[0m\x1b[?1049l");
    }
    if (tty_fd_ >= 0) {
        ::close(tty_fd_);
    }
    delete original_termios_;
}

bool RawTerminal::active() const {
    return active_;
}

std::optional<TerminalKeyEvent> RawTerminal::read_key() {
    if (!active_) {
        return std::nullopt;
    }

    char ch = 0;
    if (!read_single_byte(tty_fd_, ch)) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return std::nullopt;
        }
        return std::nullopt;
    }

    if (ch == '\r' || ch == '\n') {
        return TerminalKeyEvent{TerminalKeyKind::Enter, 0};
    }
    if (ch == 127 || ch == '\b') {
        return TerminalKeyEvent{TerminalKeyKind::Backspace, 0};
    }
    if (ch == 27) {
        char seq0 = 0;
        char seq1 = 0;
        if (read_single_byte(tty_fd_, seq0) && read_single_byte(tty_fd_, seq1) && seq0 == '[') {
            switch (seq1) {
            case 'A':
                return TerminalKeyEvent{TerminalKeyKind::Up, 0};
            case 'B':
                return TerminalKeyEvent{TerminalKeyKind::Down, 0};
            case 'C':
                return TerminalKeyEvent{TerminalKeyKind::Right, 0};
            case 'D':
                return TerminalKeyEvent{TerminalKeyKind::Left, 0};
            default:
                break;
            }
        }
        return TerminalKeyEvent{TerminalKeyKind::Escape, 0};
    }

    if (ch >= 32 && ch <= 126) {
        return TerminalKeyEvent{TerminalKeyKind::Character, ch};
    }
    return TerminalKeyEvent{TerminalKeyKind::None, 0};
}

TerminalSize RawTerminal::size() const {
    TerminalSize result;
    struct winsize ws {};
    if (tty_fd_ >= 0 && ::ioctl(tty_fd_, TIOCGWINSZ, &ws) == 0) {
        if (ws.ws_col > 0) {
            result.columns = ws.ws_col;
        }
        if (ws.ws_row > 0) {
            result.rows = ws.ws_row;
        }
    }
    return result;
}

void RawTerminal::write_text(const std::string& text) const {
    if (!active_ || tty_fd_ < 0) {
        return;
    }
    write_all(tty_fd_, text);
}

}  // namespace mrs_uav_bluetooth::app