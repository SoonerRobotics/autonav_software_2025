#include "autonav_shared/node.hpp"

namespace AutoNav
{
    Node::Node(const std::string & node_name, const bool has_ownership) : rclcpp::Node(node_name)
    {
        // Set ownership
        this->has_ownership = has_ownership;

        // Setup our device state
        system_state = AutoNav::SystemState::DISABLED;
        device_states.insert_or_assign(node_name, AutoNav::DeviceState::OFF);

        // TODO: Setup all relevant publishers, subscribers, services, clients, etc
        system_state_sub = this->create_subscription<autonav_msgs::msg::SystemState>("/autonav/shared/system", 1, std::bind(&Node::system_state_callback, this, std::placeholders::_1));
        device_state_sub = this->create_subscription<autonav_msgs::msg::DeviceState>("/autonav/shared/device", 1, std::bind(&Node::device_state_callback, this, std::placeholders::_1));

        performance_pub = this->create_publisher<autonav_msgs::msg::Performance>("/autonav/shared/performance", 10);
        log_pub = this->create_publisher<autonav_msgs::msg::Log>("/autonav/shared/log", 10);

        set_device_state_client = this->create_client<autonav_msgs::srv::SetDeviceState>("/autonav/shared/set_device_state");
        set_system_state_client = this->create_client<autonav_msgs::srv::SetSystemState>("/autonav/shared/set_system_state");
    
        set_device_state(AutoNav::DeviceState::WARMING);
    }

    Node::~Node()
    {
        // TODO: Cleanup stuff as required
    }

    void Node::perf_start(const std::string & name)
    {
        start_times.insert_or_assign(name, std::chrono::high_resolution_clock::now());
    }

    void Node::perf_stop(const std::string & name, const bool print_to_console)
    {
        if (start_times.find(name) == start_times.end())
        {
            log("Performance timer " + name + " not found", Logging::LogLevel::ERROR);
            return;
        }

        // Calculate the duration
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_times.at(name));

        // Publish the performance data
        auto performance_msg = autonav_msgs::msg::Performance();
        performance_msg.name = name;
        performance_msg.duration = duration.count();
        performance_pub->publish(performance_msg);

        // Log the performance data
        if (print_to_console)
        {
            log("Performance timer " + name + " took " + std::to_string(duration.count()) + "ms", Logging::LogLevel::DEBUG);
        }

        // Remove the timer
        start_times.erase(name);
    }

    void Node::set_device_state(const std::string & device, AutoNav::DeviceState state)
    {
        if (has_ownership)
        {
            // Assign the new state
            device_states.insert_or_assign(device, state);
            return;
        }

        auto request = std::make_shared<autonav_msgs::srv::SetDeviceState::Request>();
        request->device = device;
        request->state = static_cast<uint8_t>(state);

        // Wait for the service to be available
        while (!set_device_state_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                log("Interrupted while waiting for the set_device_state service", Logging::LogLevel::FATAL);
                return;
            }

            log("Service set_device_state not available, waiting again...", Logging::LogLevel::WARN);
        }

        using ServiceResponseFuture = rclcpp::Client<autonav_msgs::srv::SetDeviceState>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            auto response = future.get();
            if (!response->ok)
            {
                log("Failed to update device state", Logging::LogLevel::ERROR);
            } else {
                log("Successfully updated device state", Logging::LogLevel::DEBUG);
            }
        };

        auto future_result = set_device_state_client->async_send_request(request, response_received_callback);
    }

    void Node::set_system_state(AutoNav::SystemState state, bool has_mobility)
    {
        if (has_ownership)
        {
            system_state = state;
            this->has_mobility = has_mobility;
            return;
        }

        auto request = std::make_shared<autonav_msgs::srv::SetSystemState::Request>();
        request->state = static_cast<uint8_t>(state);
        request->mobility = has_mobility;

        // Wait for the service to be available
        while (!set_system_state_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                log("Interrupted while waiting for the set_system_state service", Logging::LogLevel::FATAL);
                return;
            }

            log("Service set_system_state not available, waiting again...", Logging::LogLevel::WARN);
        }

        using ServiceResponseFuture = rclcpp::Client<autonav_msgs::srv::SetSystemState>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            auto response = future.get();
            if (!response->ok)
            {
                log("Failed to update system state", Logging::LogLevel::ERROR);
            }
        };

        auto future_result = set_system_state_client->async_send_request(request, response_received_callback);
    }

    void Node::system_state_callback(const autonav_msgs::msg::SystemState::SharedPtr msg)
    {
        system_state = static_cast<AutoNav::SystemState>(msg->state);
        has_mobility = msg->mobility;
    }

    void Node::device_state_callback(const autonav_msgs::msg::DeviceState::SharedPtr msg)
    {
        // Log teh raw message
        if (msg->device == get_name())
        {
            log("Received update on our device state from " + AutoNav::DEVICE_STATE_NAMES.at(device_states.at(msg->device)) + " to " + AutoNav::DEVICE_STATE_NAMES.at(static_cast<AutoNav::DeviceState>(msg->state)), Logging::LogLevel::DEBUG);
        }

        if (msg->device != get_name())
        {
            device_states.insert_or_assign(msg->device, static_cast<AutoNav::DeviceState>(msg->state));
            return;
        }
        
        // TODO: Unravel this
        if (device_states.find(msg->device) == device_states.end())
        {
            init();
        } else {
            if (device_states.at(msg->device) == AutoNav::DeviceState::OFF && static_cast<AutoNav::DeviceState>(msg->state) == AutoNav::DeviceState::WARMING)
            {
                init();
            }
        }
    }

    // TODO: Log to file
    void Node::log(const std::string & message, Logging::LogLevel level, const std::string & function_caller, int line)
    {        
        // Current time/date
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm = *std::localtime(&now_time);
        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        
        // Log level
        std::string log_level = AutoNav::Logging::LOG_LEVEL_NAMES.at(level);
        if (log_level.length() < 5)
        {
            log_level += std::string(5 - log_level.length(), ' ');
        }

        // Log to topic
        autonav_msgs::msg::Log log_msg;
        log_msg.timestamp = now_time;
        log_msg.line_number = line;
        log_msg.function_caller = function_caller;
        log_msg.message = message;
        log_msg.node = get_name();
        log_msg.level = static_cast<uint8_t>(level);
        log_pub->publish(log_msg);

        switch (level)
        {
            case Logging::LogLevel::DEBUG:
                std::cout << termcolor::color<99, 150, 79> << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << ":" << std::setfill('0') << std::setw(3) << ms.count() 
                    << termcolor::white << " | " 
                    << termcolor::color<61, 117, 157> << log_level 
                    << termcolor::white << " | " 
                    << termcolor::color<90, 60, 146> << get_name() 
                    << termcolor::white << ":" 
                    << termcolor::color<90, 60, 146> << function_caller 
                    << termcolor::white << ":" 
                    << termcolor::color<90, 60, 146> << line 
                    << termcolor::white << " - " 
                    << termcolor::color<61, 117, 157> << message << termcolor::reset << std::endl;
                break;
            case Logging::LogLevel::INFO:
                std::cout << termcolor::color<99, 150, 79> << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << ":" << std::setfill('0') << std::setw(3) << ms.count() 
                    << termcolor::white << " | " 
                    << termcolor::color<255, 255, 255> << log_level 
                    << termcolor::white << " | " 
                    << termcolor::color<90, 60, 146> << get_name() 
                    << termcolor::white << ":" 
                    << termcolor::color<90, 60, 146> << function_caller 
                    << termcolor::white << ":" 
                    << termcolor::color<90, 60, 146> << line 
                    << termcolor::white << " - " 
                    << termcolor::color<255, 255, 255> << message << termcolor::reset << std::endl;
                break;
            case Logging::LogLevel::WARN:
                std::cout << termcolor::color<99, 150, 79> << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << ":" << std::setfill('0') << std::setw(3) << ms.count() 
                    << termcolor::white << " | " 
                    << termcolor::color<226, 174, 47> << log_level 
                    << termcolor::white << " | " 
                    << termcolor::color<90, 60, 146> << get_name() 
                    << termcolor::white << ":" 
                    << termcolor::color<90, 60, 146> << function_caller 
                    << termcolor::white << ":" 
                    << termcolor::color<90, 60, 146> << line 
                    << termcolor::white << " - " 
                    << termcolor::color<226, 174, 47> << message << termcolor::reset << std::endl;
                break;
            case Logging::LogLevel::ERROR:
                std::cout << termcolor::color<99, 150, 79> << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << ":" << std::setfill('0') << std::setw(3) << ms.count() 
                    << termcolor::white << " | " 
                    << termcolor::color<195, 59, 91> << log_level 
                    << termcolor::white << " | " 
                    << termcolor::color<90, 60, 146> << get_name() 
                    << termcolor::white << ":" 
                    << termcolor::color<90, 60, 146> << function_caller 
                    << termcolor::white << ":" 
                    << termcolor::color<90, 60, 146> << line 
                    << termcolor::white << " - " 
                    << termcolor::color<195, 59, 91> << message << termcolor::reset << std::endl;
                break;
            case Logging::LogLevel::FATAL: // rgb(207, 62, 99)
                std::cout << termcolor::color<99, 150, 79> << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << ":" << std::setfill('0') << std::setw(3) << ms.count() 
                    << termcolor::white << " | " 
                    << termcolor::white << termcolor::on_color<207, 62, 99> << log_level 
                    << termcolor::reset << termcolor::white << " | " 
                    << termcolor::color<90, 60, 146> << get_name() 
                    << termcolor::white << ":" 
                    << termcolor::color<90, 60, 146> << function_caller 
                    << termcolor::white << ":" 
                    << termcolor::color<90, 60, 146> << line 
                    << termcolor::white << " - " 
                    << termcolor::white << termcolor::on_color<207, 62, 99> << message << termcolor::reset << std::endl;
                break;
        }
    }
}