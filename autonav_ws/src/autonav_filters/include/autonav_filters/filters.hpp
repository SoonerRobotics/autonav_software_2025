#pragma once

#include "autonav_shared/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/msg/motor_feedback.hpp"
#include "autonav_msgs/msg/gps_feedback.hpp"
#include "autonav_msgs/msg/position.hpp"
#include "autonav_msgs/msg/imu_data.hpp"

#include <chrono>
#include <functional>
#include "autonav_filters/particle_filter.hpp"

#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

struct ParticleFilterConfig {
    int num_particles;
    double latitudeLength;
    double longitudeLength;
    double gps_noise;
    double odom_noise_x;
    double odom_noise_y;
    double odom_noise_theta;
};


class FiltersNode : public AutoNav::Node {
    public:
        FiltersNode() : AutoNav::Node("autonav_filters_pf"), count_(0)
        {
            // subscriptions
            gps_subscription = this->create_subscription<autonav_msgs::msg::GPSFeedback>("/autonav/gps", 
            20, std::bind(&FiltersNode::on_GPS_received, this, std::placeholders::_1));
            motor_subscription = this->create_subscription<autonav_msgs::msg::MotorFeedback>("/autonav/MotorFeedback", 
            20, std::bind(&FiltersNode::on_MotorFeedback_received, this, std::placeholders::_1));

            // publishers
            positionPublisher = this->create_publisher<autonav_msgs::msg::Position>("/autonav/position", 20);
            
            // this->set_device_state(SCR::DeviceState::OPERATING);
            this->on_reset();
        }

    private:
        //particle filter
        ParticleFilter particle_filter;
        ParticleFilterConfig config;
        double latitudeLength;
        double longitudeLength;
        double degreeOffset = 107.0;
        autonav_msgs::msg::GPSFeedback first_gps;
        autonav_msgs::msg::GPSFeedback last_gps;
        bool first_gps_received = false;
        bool last_gps_assigned = false;
        bool first_imu_received = false;
        autonav_msgs::msg::IMUData last_IMU_received;

        void init() override;

        void config_updated(ParticleFilterConfig config);

        ParticleFilterConfig get_default_config();

        void on_reset();
        
        void on_GPS_received(const autonav_msgs::msg::GPSFeedback gps_message);

        void on_MotorFeedback_received(const autonav_msgs::msg::MotorFeedback motorFeedback_message);

        rclcpp::Subscription<autonav_msgs::msg::GPSFeedback>::SharedPtr gps_subscription;
        rclcpp::Subscription<autonav_msgs::msg::MotorFeedback>::SharedPtr motor_subscription;
        rclcpp::Publisher<autonav_msgs::msg::Position>::SharedPtr positionPublisher;
        size_t count_;
};