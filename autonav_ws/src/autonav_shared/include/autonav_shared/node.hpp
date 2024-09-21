#pragma once

#include "rclcpp/rclcpp.hpp"
#include "external/termcolor.hpp"
#include "types.hpp"

#include "autonav_msgs/msg/system_state.hpp"
#include "autonav_msgs/msg/device_state.hpp"

namespace AutoNav
{
    class Node : public rclcpp::Node
    {
    public:
        Node(const std::string &node_name);
        ~Node();

    protected:
        /// @brief Log a message with the given log level
        /// @param level 
        /// @param message 
        /// @param function_caller If not provided, the function name will be automatically determined
        /// @param line If not provided, the line number will be automatically determined
        void log(const std::string &message, Logging::LogLevel level = Logging::LogLevel::INFO, const std::string &function_caller = __builtin_FUNCTION(), int line = __builtin_LINE());

    private:
        // State
        AutoNav::SystemState system_state = AutoNav::SystemState::DISABLED;
        std::map<std::string, AutoNav::DeviceState> device_states;
        bool has_mobility = false;

        // Subscribers
        rclcpp::Subscription<autonav_msgs::msg::SystemState>::SharedPtr system_state_sub;
        rclcpp::Subscription<autonav_msgs::msg::DeviceState>::SharedPtr device_state_sub;

        // Functions
        void system_state_callback(const autonav_msgs::msg::SystemState::SharedPtr msg);
        void device_state_callback(const autonav_msgs::msg::DeviceState::SharedPtr msg);
    };
}