#pragma once

#include "autonav_filters/filters.hpp"
#include <string>
#include "autonav_msgs/msg/motor_feedback.hpp"
#include "autonav_msgs/msg/gps_feedback.hpp"
#include "autonav_msgs/msg/position.hpp"
#include "autonav_msgs/msg/imu_data.hpp"

#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

ParticleFilterConfig config;

//particle filter
void FiltersNode::init() {
    //double latitudeLength = 111086.2;
    //double longitudeLength = 81978.2;
    // ParticleFilterConfig config = {
    //     .latitudeLength = 111086.2,
    //     .longitudeLength = 81978.2,
    //     .gps_noise = {0.8},

    // };

    ParticleFilterConfig default_config = this->get_default_config();
    this->config = default_config;
    this->latitudeLength = config.latitudeLength;
    this->longitudeLength = config.longitudeLength;
    ParticleFilter particle_filter{config.num_particles, config.latitudeLength, config.longitudeLength, config.gps_noise, config.odom_noise_x, config.odom_noise_y, config.odom_noise_theta};
    this->particle_filter = particle_filter;
    autonav_msgs::msg::GPSFeedback first_gps;
    autonav_msgs::msg::GPSFeedback last_gps;
    bool first_gps_received = false;

    this->on_reset();
};

void FiltersNode::on_reset() {
    // if you're going to seed the pf with the IMU it would go here
    this->particle_filter.init_particles();
}

void FiltersNode::config_updated(ParticleFilterConfig newConfig) {
        config = newConfig;
        ParticleFilter particle_filter{config.num_particles, config.latitudeLength, config.longitudeLength, config.gps_noise, config.odom_noise_x, config.odom_noise_y, config.odom_noise_theta};
        this->particle_filter = particle_filter;
}

ParticleFilterConfig FiltersNode::get_default_config() {
        ParticleFilterConfig newConfig;
        newConfig.latitudeLength = 111086.2;
        newConfig.longitudeLength = 81978.2;
        newConfig.num_particles = 750;
        newConfig.gps_noise = 0.8;
        newConfig.odom_noise_x = 0.05;
        newConfig.odom_noise_y = 0.05;
        newConfig.odom_noise_theta = 0.01;

        return newConfig;
}

void FiltersNode::on_GPS_received(const autonav_msgs::msg::GPSFeedback gps_message) {
    RCLCPP_INFO(this->get_logger(), "RECEIVED GPS");
    RCLCPP_INFO(this->get_logger(), "gps_message.latitude %f\n", gps_message.latitude);
    RCLCPP_INFO(this->get_logger(), "gps_message.longitude %f\n", gps_message.longitude);
    if (gps_message.gps_fix == 0 &&  gps_message.is_locked == false) {
        return;
        RCLCPP_INFO(this->get_logger(), "RETURNING IMMEDIATELY");
    }
    if (this->first_gps_received == false) {
        this->first_gps = gps_message;
        this->first_gps_received = true;
    }

    RCLCPP_INFO(this->get_logger(), "first_gps.latitude %f", first_gps.latitude);
    RCLCPP_INFO(this->get_logger(), "first_gps.longitude %f", first_gps.longitude);
        
    this->last_gps = gps_message;
    this->last_gps_assigned = true;

    this->particle_filter.gps(gps_message);
    
}
void FiltersNode::on_MotorFeedback_received(const autonav_msgs::msg::MotorFeedback motorFeedback_message) {
    RCLCPP_INFO(this->get_logger(), "received motorfeedback %f", 5);
    std::vector<double> averages;
    averages = this->particle_filter.feedback(motorFeedback_message);

    if (averages.size() < 3) {
        return;
    }

    autonav_msgs::msg::Position position;
    position.x = averages[0];
    position.y = averages[1];
    //RCLCPP_INFO(this->get_logger(), "position.x %f\n", position.x);
    //RCLCPP_INFO(this->get_logger(), "position.y %f\n", position.y);
    position.theta = (-1 * M_PI * 2 + averages[2]);
    //RCLCPP_INFO(this->get_logger(), "position.theta %f\n", position.theta);
    if (this->first_gps_received == true) {
        double gps_x = this->first_gps.latitude + position.x / this->latitudeLength;
        double gps_y = this->first_gps.longitude - position.y / this->longitudeLength;

        //RCLCPP_INFO(this->get_logger(), "first_gps.latitude %f", first_gps.latitude);
        //RCLCPP_INFO(this->get_logger(), "first_gps.longitude %f", first_gps.longitude);
        //RCLCPP_INFO(this->get_logger(), "gps_x %f", gps_x);
        //RCLCPP_INFO(this->get_logger(), "gps_y %f", gps_y);


        position.latitude = gps_x;
        //RCLCPP_INFO(this->get_logger(), "position.latitude %f\n", position.latitude);
        position.longitude = gps_y;
        //RCLCPP_INFO(this->get_logger(), "position.longitude %f\n", position.longitude);
    }

    if (this->last_gps_assigned = true) {
        //position.latitude = this->last_gps.latitude;
        //position.longitude = this->last_gps.longitude;
    }
    //RCLCPP_INFO(this->get_logger(), "average 0: %f, average 1: %f, average 2: %f", averages[0], averages[1], averages[2]);
    //RCLCPP_INFO(this->get_logger(), "publishing position x: %f, y: %f, theta: %f", position.x, position.y, position.theta);
    RCLCPP_INFO(this->get_logger(), "position latitude: %f, longitude: %f", position.latitude, position.longitude);
    std::string latitude_log_string = std::to_string(position.latitude);
    std::string longitude_log_string = std::to_string(position.longitude);
    std::string combined_string = latitude_log_string + ", " + longitude_log_string + ", ";
    this->log(combined_string);

    positionPublisher->publish(position);
}

rclcpp::Subscription<autonav_msgs::msg::GPSFeedback>::SharedPtr gps_subscription;
rclcpp::Subscription<autonav_msgs::msg::MotorFeedback>::SharedPtr motor_subscription;
rclcpp::Publisher<autonav_msgs::msg::Position>::SharedPtr positionPublisher;
size_t count_;

int main (int argc, char * argv[]) {
    // rclcpp::init(argc, argv);
    // SCR::Node::run_node(std::make_shared<FiltersNode>());
    // rclcpp::shutdown();
    // return 0;
    rclcpp::init(argc, argv);
    std::shared_ptr<FiltersNode> particle_filter_node = std::make_shared<FiltersNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(particle_filter_node);
    executor.spin();
    executor.remove_node(particle_filter_node);
    rclcpp::shutdown();
    return 0;
}