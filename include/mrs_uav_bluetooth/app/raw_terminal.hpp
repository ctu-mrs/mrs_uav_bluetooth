// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/app/raw_terminal.hpp
/// \brief Declares the raw terminal component of the ROS 2 application and operator-tool layer.

#pragma once

#include <termios.h>

#include <optional>
#include <string>

namespace mrs_uav_bluetooth::app {

/// Logical keys understood by the text user interface.
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

/// One decoded terminal key, including a printable character when applicable.
struct TerminalKeyEvent {
    TerminalKeyKind kind{TerminalKeyKind::None};
    char ch{0};
};

/// Current terminal dimensions in character cells.
struct TerminalSize {
    int columns{120};
    int rows{30};
};

/// RAII wrapper that enters raw terminal mode and restores it on destruction.
class RawTerminal {
public:
    /// \brief Put the terminal in raw mode and create a wakeable input channel.
    RawTerminal();
    /// \brief Restore terminal settings and close the wake channel.
    ~RawTerminal();

    /// \brief Disable copying of the raw terminal.
    RawTerminal(const RawTerminal&) = delete;
    /// \brief Disable copy assignment of the raw terminal.
    RawTerminal& operator=(const RawTerminal&) = delete;

    /// \brief Report whether raw terminal mode is currently installed.
    /// \return True while the terminal owns saved raw-mode state.
    bool active() const;
    /// \brief Restore normal terminal settings temporarily for a child program.
    void suspend();
    /// \brief Re-enter raw mode after a suspended child program exits.
    void resume();
    /// \brief Decode one terminal byte sequence into a navigation or action event.
    /// \return Decoded terminal action or std::nullopt when no complete input is ready.
    std::optional<TerminalKeyEvent> read_key();
    /// \brief Return the current terminal dimensions.
    /// \return Current terminal row and column count.
    TerminalSize size() const;
    /// \brief Write a complete dashboard frame while tolerating interrupted system calls.
    /// \param text terminal or log text written in full.
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
