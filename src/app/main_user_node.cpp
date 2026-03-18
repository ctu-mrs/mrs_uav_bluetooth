// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/user_overlay_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mrs_uav_bluetooth::app::UserOverlayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
