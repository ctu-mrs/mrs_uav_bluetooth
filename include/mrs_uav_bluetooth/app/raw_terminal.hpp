// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <termios.h>

#include <optional>
#include <string>

namespace mrs_uav_bluetooth::app {

enum class TerminalKeyKind {
    None,
    Character,
    Up,
    Down,
    Left,
    Right,
    Enter,
    Escape,
    Backspace,
};

struct TerminalKeyEvent {
    TerminalKeyKind kind{TerminalKeyKind::None};
    char ch{0};
};

struct TerminalSize {
    int columns{120};
    int rows{30};
};

class RawTerminal {
public:
    RawTerminal();
    ~RawTerminal();

    RawTerminal(const RawTerminal&) = delete;
    RawTerminal& operator=(const RawTerminal&) = delete;

    bool active() const;
    std::optional<TerminalKeyEvent> read_key();
    TerminalSize size() const;
    void write_text(const std::string& text) const;

private:
    int tty_fd_{-1};
    bool active_{false};
    int original_flags_{0};
    bool flags_saved_{false};
    bool termios_saved_{false};
    ::termios* original_termios_{nullptr};
};

}  // namespace mrs_uav_bluetooth::app