// SPDX-License-Identifier: MIT
#include "mrs_uav_bluetooth/app/bluetooth_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mrs_uav_bluetooth::app::BluetoothNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
