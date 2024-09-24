#include "rclcpp/rclcpp.hpp"

#include "autonav_msgs/msg/device_state.hpp"
#include "autonav_msgs/msg/system_state.hpp"

#include "autonav_msgs/srv/set_device_state.hpp"
#include "autonav_msgs/srv/set_system_state.hpp"

#include "autonav_shared/node.hpp"
#include "autonav_shared/types.hpp"

class Synchronizer : public AutoNav::Node
{
public:
    Synchronizer() : AutoNav::Node("autonav_synchronizer", true)
    {
        // Publishers
        device_state_pub = this->create_publisher<autonav_msgs::msg::DeviceState>("/autonav/shared/device", 1);
        system_state_pub = this->create_publisher<autonav_msgs::msg::SystemState>("/autonav/shared/system", 1);

        // Services
        set_device_state_srv = this->create_service<autonav_msgs::srv::SetDeviceState>("/autonav/shared/set_device_state", std::bind(&Synchronizer::set_device_state_callback, this, std::placeholders::_1, std::placeholders::_2));
        set_system_state_srv = this->create_service<autonav_msgs::srv::SetSystemState>("/autonav/shared/set_system_state", std::bind(&Synchronizer::set_system_state_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    void set_device_state_callback(const std::shared_ptr<autonav_msgs::srv::SetDeviceState::Request> request, std::shared_ptr<autonav_msgs::srv::SetDeviceState::Response> response)
    {
        log("Setting device state: " + request->device + " to " + std::to_string(request->state), AutoNav::Logging::DEBUG);
        
        // Did this device already exist?
        bool did_exist = has_device_state(request->device);

        // Set the device state locally
        this->set_device_state(request->device, static_cast<AutoNav::DeviceState>(request->state));

        // Respond
        response->ok = true;

        if (!did_exist)
        {
            // Broadcast all other device states
            for (auto it = device_states_begin(); it != device_states_end(); it++)
            {
                autonav_msgs::msg::DeviceState msg;
                msg.device = it->first;
                msg.state = it->second;
                device_state_pub->publish(msg);
            }

            // Broadcast the system state
            autonav_msgs::msg::SystemState system_msg;
            system_msg.state = get_system_state();
            system_msg.mobility = is_mobility();
            system_state_pub->publish(system_msg);
            return;
        } else {
            // Publish the update
            autonav_msgs::msg::DeviceState msg;
            msg.device = request->device;
            msg.state = request->state;
            device_state_pub->publish(msg);
        }
    }

    void set_system_state_callback(const std::shared_ptr<autonav_msgs::srv::SetSystemState::Request> request, std::shared_ptr<autonav_msgs::srv::SetSystemState::Response> response)
    {
        // Set the system state locally
        this->set_system_state(static_cast<AutoNav::SystemState>(request->state), request->mobility);

        // Respond
        response->ok = true;

        // Publish the update
        autonav_msgs::msg::SystemState msg;
        msg.state = request->state;
        msg.mobility = request->mobility;
        system_state_pub->publish(msg);

        log("Setting system state to " + std::to_string(request->state) + " with mobility " + std::to_string(request->mobility), AutoNav::Logging::DEBUG);
    }

private:
    rclcpp::Service<autonav_msgs::srv::SetDeviceState>::SharedPtr set_device_state_srv;
    rclcpp::Service<autonav_msgs::srv::SetSystemState>::SharedPtr set_system_state_srv;

    rclcpp::Publisher<autonav_msgs::msg::DeviceState>::SharedPtr device_state_pub;
    rclcpp::Publisher<autonav_msgs::msg::SystemState>::SharedPtr system_state_pub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Synchronizer>());
    rclcpp::shutdown();
    return 0;
}
