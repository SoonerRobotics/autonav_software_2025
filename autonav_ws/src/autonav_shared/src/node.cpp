#include "autonav_shared/node.hpp"
#include <unistd.h>

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
        configuration_broadcast_sub = this->create_subscription<autonav_msgs::msg::ConfigurationBroadcast>("/autonav/shared/config/requests", 1, std::bind(&Node::config_broadcast_callback, this, std::placeholders::_1));
        configuration_update_sub = this->create_subscription<autonav_msgs::msg::ConfigurationUpdate>("/autonav/shared/config/updates", 1, std::bind(&Node::config_updated_callback, this, std::placeholders::_1));

        performance_pub = this->create_publisher<autonav_msgs::msg::Performance>("/autonav/shared/performance", 10);
        log_pub = this->create_publisher<autonav_msgs::msg::Log>("/autonav/shared/log", 10);
        device_state_pub = this->create_publisher<autonav_msgs::msg::DeviceState>("/autonav/shared/device", 10);
        system_state_pub = this->create_publisher<autonav_msgs::msg::SystemState>("/autonav/shared/system", 10);
        configuration_broadcast_pub = this->create_publisher<autonav_msgs::msg::ConfigurationBroadcast>("/autonav/shared/config/requests", 10);
        configuration_update_pub = this->create_publisher<autonav_msgs::msg::ConfigurationUpdate>("/autonav/shared/config/updates", 10);
    }

    Node::~Node()
    {
        // TODO: Cleanup stuff as required
    }

    void Node::config_updated_callback(const autonav_msgs::msg::ConfigurationUpdate::SharedPtr msg)
    {
        if (msg->device == get_name())
        {
            // log("Received update on our own configuration", Logging::LogLevel::DEBUG);
            json old_config = this->_config;
            // this->_config = json::parse(msg->json);
            apply_config(json::parse(msg->json));
            this->on_config_updated(old_config, this->_config);
        } else {
            // log("Received update on " + msg->device + "'s configuration", Logging::LogLevel::DEBUG);
        }

        other_cfgs.insert_or_assign(msg->device, json::parse(msg->json));
    }

    void Node::config_broadcast_callback(const autonav_msgs::msg::ConfigurationBroadcast::SharedPtr msg)
    {
        if (msg->target_nodes.size() == 0 || std::find(msg->target_nodes.begin(), msg->target_nodes.end(), get_name()) != msg->target_nodes.end())
        {
            log("Received request for our own configuration to be broadcasted", Logging::LogLevel::DEBUG);
            broadcast_config();
        };
    }

    void Node::broadcast_config()
    {
        autonav_msgs::msg::ConfigurationUpdate msg;
        msg.device = get_name();
        msg.json = _config.dump();
        configuration_update_pub->publish(msg);
    }

    void Node::request_all_configs()
    {
        autonav_msgs::msg::ConfigurationBroadcast msg;
        configuration_broadcast_pub->publish(msg);
    }

    void Node::request_config(const std::string & device)
    {
        autonav_msgs::msg::ConfigurationBroadcast msg;
        msg.target_nodes.push_back(device);
        configuration_broadcast_pub->publish(msg);
    }

    void Node::write_config(const json & config)
    {
        this->_config = config;
        broadcast_config();
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
        performance_msg.elapsed = duration.count();
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
        // Publish the update
        device_states.insert_or_assign(device, state);
        autonav_msgs::msg::DeviceState msg;
        msg.device = device;
        msg.state = static_cast<uint8_t>(state);
        device_state_pub->publish(msg);
    }

    void Node::set_system_state(AutoNav::SystemState state, bool has_mobility)
    {
        // Publish the update
        autonav_msgs::msg::SystemState msg;
        msg.state = static_cast<uint8_t>(state);
        msg.mobility = has_mobility;
        system_state_pub->publish(msg);
        system_state = state;
        this->has_mobility = has_mobility;
    }

    void Node::system_state_callback(const autonav_msgs::msg::SystemState::SharedPtr msg)
    {
        AutoNav::SystemState old_state = system_state;
        bool old_mobility = has_mobility;

        system_state = static_cast<AutoNav::SystemState>(msg->state);
        has_mobility = msg->mobility;

        if (system_state != old_state)
        {
            on_system_state_updated(old_state, system_state);
        }

        if (has_mobility != old_mobility)
        {
            on_mobility_updated(old_mobility, has_mobility);
        }

        if (system_state == SystemState::SHUTDOWN)
        {
            kill(getpid(), SIGKILL);
        }
    }

    void Node::device_state_callback(const autonav_msgs::msg::DeviceState::SharedPtr msg)
    {
        if (msg->device != get_name())
        {
            device_states.insert_or_assign(msg->device, static_cast<AutoNav::DeviceState>(msg->state));
            return;
        }
        
        bool is_warming_up = device_states.at(msg->device) == AutoNav::DeviceState::OFF && static_cast<AutoNav::DeviceState>(msg->state) == AutoNav::DeviceState::WARMING;
        if (is_warming_up)
        {
            log("Received warming up state from " + msg->device, Logging::LogLevel::DEBUG);
            init();
        }
        device_states.insert_or_assign(msg->device, static_cast<AutoNav::DeviceState>(msg->state));
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