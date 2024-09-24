#pragma once

#include "feeler.cpp" //TODO
#include "autonav_msgs/msg/position.hpp"

class FeelerNode : public rclcpp::Node {
public:
    //TODO waypoints file reading code

    FiltersNode() : AutoNav::Node("autonav_feelers");
    ~FiltersNode() {}

    void init() override {
        //TODO publishers / subscribers

        set_device_state(AutoNav::DeviceState::OPERATING);
    }

    json get_default_config() override {
        FeelerConfig newConfig;
        return newConfig;
        //TOOD
    }

    /**
     * Main callback to command motor output upon receiving the combined image from combination.py
     * @param image a compressedimage message with the combined transformations of all 4 cameras
     */
    void on_image_received(const msg::CompressedImage::SharedPtr image) {
        // if we aren't in autonomous
        if (system_state != AutoNav::SystemState::AUTONOMOUS) {
            return; // return because we don't need to do anything
        }

        // calculate new length of every new feeler
        for (Feeler feeler : this->feelers) {
            feeler.update(mask); //TODO
        }

        //TODO

        // TODO safety lights
        
        // Make the motor output message
        autonav_messages::msg::MotorInput msg;
        msg.forward_velocity = 0.0;
        msg.sideways_velocity = 0.0;
        msg.angular_velocity = 0.0;

        motorPublisher->publish(msg);
    }

    void on_debug_received(const msg::CompressedImage::SharedPtr image); //TODO
    void on_position_received(const msg::CompressedImage::SharedPtr image); //TODO
    
    void main();

private:
    std::vector<Feeler> feelers;
    //TODO ultrasonics, GPS
    //TODO subscribers
    //TODO publishers
    rclcpp::Subscription<autonav_messages::msg::Position>::SharedPtr positionSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr imageSubscriber;
};