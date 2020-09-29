// Copyright (c) 2020 The Plankton Authors.
// All rights reserved.
//
#include <rclcpp/rclcpp.hpp>

/// Listens for nodes asking if the simulation is using sim_time and replies
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("plankton_global_sim_time");

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
