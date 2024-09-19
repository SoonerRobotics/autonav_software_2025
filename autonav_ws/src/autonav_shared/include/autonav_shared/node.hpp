#pragma once

#include "rclcpp/rclcpp.hpp"
#include "types.hpp"

namespace AutoNav
{
    class Node : public rclcpp::Node
    {
        public:
            Node(const std::string & node_name);
            ~Node();

        private:
            AutoNav::DeviceState device_state = AutoNav::DeviceState::OFF;
            AutoNav::SystemState system_state = AutoNav::SystemState::DISABLED;
            bool has_mobility = false;
    };
}