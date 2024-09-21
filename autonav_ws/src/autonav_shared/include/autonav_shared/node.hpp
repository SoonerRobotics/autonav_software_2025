#pragma once

#include "rclcpp/rclcpp.hpp"
#include "external/termcolor.hpp"
#include "types.hpp"

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
        AutoNav::DeviceState device_state = AutoNav::DeviceState::OFF;
        AutoNav::SystemState system_state = AutoNav::SystemState::DISABLED;
        bool has_mobility = false;
    };
}