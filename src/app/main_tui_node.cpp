// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/tui_node.hpp"

#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>

#include <unistd.h>

namespace {

void detach_process_stdio_from_tty() {
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

int main(int argc, char** argv) {
    detach_process_stdio_from_tty();
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mrs_uav_bluetooth::app::TuiNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}