// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/test_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mrs_uav_bluetooth::app::TestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}