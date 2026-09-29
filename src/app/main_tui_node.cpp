// SPDX-License-Identifier: BSD-3-Clause
/// \file src/app/main_tui_node.cpp
/// \brief Implements the main tui node component of the ROS 2 application and operator-tool layer.

#include "mrs_uav_bluetooth/app/tui_node.hpp"

#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>

#include <unistd.h>

namespace {

/// \brief Detach process stdio from terminal.
void detach_process_stdio_from_tty() {
    // Release inherited streams so the dashboard can own /dev/tty directly.
    const int tty_fd = ::open("/dev/tty", O_RDWR | O_NOCTTY);
    if (tty_fd < 0) {
        return;
    }
    ::close(tty_fd);

    const int null_fd = ::open("/dev/null", O_RDWR);
    if (null_fd < 0) {
        return;
    }

    ::dup2(null_fd, STDOUT_FILENO);
    ::dup2(null_fd, STDERR_FILENO);
    if (null_fd > STDERR_FILENO) {
        ::close(null_fd);
    }
}

}  // namespace

/// \brief Run the interactive Bluetooth dashboard until ROS shutdown.
/// \param argc number of command-line arguments.
/// \param argv command-line argument vector.
/// \return Zero on success, or a nonzero process status on failure.
int main(int argc, char** argv) {
    // Detach the dashboard from inherited terminal streams before starting the ROS event loop.
    detach_process_stdio_from_tty();
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mrs_uav_bluetooth::app::TuiNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
