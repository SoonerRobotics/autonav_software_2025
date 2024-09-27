#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "autonav_shared/node.hpp"
#include "feeler.cpp"
#include "autonav_msgs/msg/position.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

// pretty basic gps point struct
struct GPSPoint {
    double lat;
    double lon;
};

/**
 * Configuration parameters for the feeler node
 * @param max_length is the maximum/default length for a feeler
 * @param number_of_feelers controls how many feelers there are, distributed uniformly in a circle
 * @param start_angle the starting angle offset when building the feelers, in degrees
 */
struct FeelerNodeConfig {
    int max_length;
    int number_of_feelers;
    double start_angle;
    // double ultrasonic_contribution; //TODO
    //TODO other config parameters

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(FeelerNodeConfig, max_length, number_of_feelers);
};

class FeelerNode : public AutoNav::Node {
public:
    FiltersNode() : AutoNav::Node("autonav_feelers") {}
    ~FiltersNode() {}

    void init() override {
        set_device_state(AutoNav::DeviceState::WARMING);

        // === read waypoints from file === (copied and pasted from last year's feat/astar_rewrite_v3)
        std::string line;
        this->waypointsFile.open(this->WAYPOINTS_FILENAME);
        getline(waypointsFile, line); // skip the first line
        while (getline(waypointsFile, line) ) { 
            std::vector<std::string> tokens; // https://www.geeksforgeeks.org/tokenizing-a-string-cpp/
            std::stringstream strstream(line);
            std::string intermediate;
            while(getline(strstream, intermediate, ',')) {
                tokens.push_back(intermediate);
            }

            GPSPoint point;
            point.lat = std::stod(tokens[1]); //https://cplusplus.com/reference/string/stod/
            point.lon = std::stod(tokens[2]);

            // waypoints are stored like {"north":[GPSPoint, GPSPoint]}
            waypointsDict[tokens[0]].push_back(point);
        }
        waypointsFile.close();
        // === /read waypoints ===

        // subscribers
        positionSubscriber = create_subscription<autonav_msgs::msg::Position>("/autonav/position", 1, std::bind(&FeelerNode::onPositionReceived, this, std::placeholders::_1));
        imageSubscriber = create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/vision/combined/filtered", 1, std::bind(&FeelerNode::onImageReceived, this, std::placeholders::_1));
        debugImageSubscriber = create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/vision/combined", 1, std::bind(&FeelerNode::onDebugImageReceived, this, std::placeholders::_1));
        ultrasonicSubscriber = create_subscription<autonav_msgs::msg::Ultrasonics>("/autonav/ultrasonics", 1, std::bind(&FeelerNode::onUltrasonicsReceived, this, std::placeholders::_1));
        
        // publishers
        motorPublisher = create_publisher<autonav_msgs::msg::MotorInput>("/autonav/motor_input", 1);
        debugPublisher = create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/feelers/debug", 1);
        safetyLightsPublisher = create_publisher<autonav_msgs::msg::SafetyLights>("/autonav/safety_lights", 1); 
        audibleFeedbackPublisher = create_publisher<autonav_msgs::msg::AudibleFeedback>("/autonav/audible_feedback", 1); 

        // make all the feelers
        this->buildFeelers();

        // make the feelers for the ultrasonics
        ultrasonic_feelers = std::vector<Feeler>();
        for (double angle = 0.0; angle < 360; angle += 90) { //TODO make these not originate at the origin?
            int x = this->config.max_length * cos(radians(angle)); //int should truncate these to nice whole numbers
            int y = this->config.max_length * sin(radians(angle));

            this->feelers.push_back(Feeler(x, y));
            this->feelers.push_back(Feeler(x, y)); // there are 2 ultrasonic distance sensors per side
        }

        for (Feeler feeler : ultrasonic_feelers) {
            feeler.setColor(cv::Scalar(0, 200, 0)); // ultrasonic feelers are a different color
        }

        set_device_state(AutoNav::DeviceState::READY);
    }

    /**
     * Builds the list of feelers based on configuration parameters.
     * Evenly distributes a num_feelers number of feelers in a circle of radius max_length
     * starting with an offset of start_angle (in degrees).
     */
    void buildFeelers() {
        this->feelers = std::vector<Feeler>();
        for (double angle = this->config.start_angle; angle < 360; angle += (360 / this->config.num_feelers)) {
            int x = this->config.max_length * cos(radians(angle)); //int should truncate these to nice whole numbers, I hope
            int y = this->config.max_length * sin(radians(angle));

            this->feelers.push_back(Feeler(x, y));
        }

        for (Feeler feeler : feelers) {
            feeler.setColor(cv::Scalar(200, 0, 0)); // feelers are blue, openCV is in BGR
        }
    }

    /**
     * This function is called whenever the configuration is updated from the UI.
     * Configurable parameters are defined in the FeelerNodeConfig struct.
     * @param newConfig the new configuration for the feeler node
     */
    void config_updated(json newConfig) override {
        this->config = newConfig.template get<FeelerNodeConfig>();

        this->buildFeelers();
    }

    /**
     * Gets the default configuration of the node for populating the UI.
     * @return the default configuration of the node as a json-ified FeelerNodeConfig struct.
     */
    json get_default_config() override {
        FeelerConfig newConfig;
        newConfig.max_length = 100;
        newConfig.number_of_feelers = 12;
        newConfig.start_angle = 0.0;

        return newConfig;
    }

    /**
     * Main callback to command motor output upon receiving the combined image from combination.py
     * @param image a compressedimage message with the combined transformations of all 4 cameras
     */
    void onImageReceived(sensor_msgs::msg::CompressedImage const image) {
        set_device_state(AutoNav::DeviceState::OPERATING);

        // if we aren't in autonomous
        if (this->get_system_state() != AutoNav::SystemState::AUTONOMOUS) {
            return; // return because we don't need to do anything
        }

        // reinitialize the heading arrow (with a bias towards going 'forwards')
        this->headingArrow = Feeler(0, 25);

        // turn the image into a format we can use
        auto mask = cv_bridge::toCvCopy(image)->image; //TODO what encoding do we want to use?

        // calculate new length of every new feeler
        for (Feeler feeler : this->feelers) {
            feeler.update(&mask);
        }

        // draw debug image (separate from loop because this will modify the image, which wouldn't work multithreaded)
        // also if you drew on the image while modifying it that would mess up some of the feelers
        // also add all the feelers together
        auto debug_image = cv::Mat(); //FIXME

        for (Feeler feeler : this->feelers) {
            feeler.draw(debug_image);
            this->headingArrow = this->headingArrow + feeler;
        }

        // ultrasonics
        for (Feeler feeler : this->ultrasonic_feelers) {
            // ultrasonic feelers contribute twice as much because there are fewer of them
            //FIXME is this what we want?
            this->headingArrow = this->headingArrow + feeler;
            this->headingArrow = this->headingArrow + feeler;

            feeler.draw(debug_image);
        }
        //TODO publish debug image

        //TODO safety lights

        //FIXME this should be in a separate function that gets called like every .05 seconds
        // this callback should just be for updating the feelers, publishing the motor message belongs elsewehere
        // so we don't trip the watchdog in the motor manager firmware
        
        // if we are allowed to move (earlier check means we are already in auto and operating, so don't have to recheck those)
        if (this->is_mobility()) {
            // make the message
            autonav_msgs::msg::MotorInput msg;

            // add a bias forwards
            this->headingArrow = this->headingArrow + Feeler(0, 50);

            // add a bias towards the GPS waypoint
            //FIXME this doesn't account for the rotation of the robot
            this->headingArrow = this->headingArrow + Feeler(goalPoint.lon - this->position.longitude, goalPoint.lat - this->position.latitude); //FIXME cap the length/contribution of this

            this->headingArrow.draw(debug_image);

            // convert headingArrow to motor outputs
            //FIXME we want to be going max speed on the straightaways
            msg.forward_velocity = std::clamp(this->headingArrow.getY(), -5, 5);
            msg.sideways_velocity = std::clamp(this->headingArrow.getX(), -5, 5);
            msg.angular_velocity = 0.0; //TODO figure out when we want to turn

            //publish the motor message
            this->motorPublisher->publish(msg);
            // and publish the debug image
            this->debugPublisher->publish(cv_bridge::CvImage(debug_image).toImageMsg());

            // make the audible feedback message
            autonav_msgs::msg::AudibleFeedback feedback_msg;
            if (weHitAWaypoint) { //FIXME
                feedback_msg.filename = "ding.mp3";
            } else if (goingLeft) { //FIXME
                feedback_msg.filename = "going_left.mp3";
            } else if (goingRight) { //FIXME
                feedback_msg.filename = "going_right.mp3";
            } else if (goingBackwards) { //FIXME
                feedback_msg.filename = "beep_beep.mp3";
            } else {
                // don't publish anything so the music can be heard
            }
            
            // if we are actually wanting to play a file
            if (feedback_msg.filename != nullptr) {
                this->audibleFeedbackPublisher.publish(feedback_msg);
            }

        } else {
            // we are not mobility enabled and thus not allowed to move, so publish velocities of 0 for everything
            autonav_msgs::msg::MotorInput msg;
            msg.forward_velocity = 0.0;
            msg.sideways_velocity = 0.0;
            msg.angular_velocity = 0.0;

            this->motorPublisher->publish(msg);

            // and publish the debug image
            this->debugPublisher->publish(debug_image);
        }
    }

    /**
     * Callback to receive and color image from the cameras to draw the feelers on for debug purposes
     * @param image the compressedImage message to draw the feelers on
     */
    void onDebugReceived(const sensor_msgs::msg::CompressedImage image) {
        this->newDebugImage = true;

        this->debug_image = cv_bridge.toCvCopy(image); //TODO figure out what encoding we want to use
    }

    /**
     * Callback to receive the position of the robot from the particle filter
     * we need to know this so we can figure out which way to move to get to the GPS waypoints
     * @param msg the Postion message from the particle filter
     */
    void onPositionReceived(const autonav_msgs::msg::Position msg) {
        this->position = msg;
    }

    /**
     * Callback to get data from the ultrasonic sensors. This message does not contain the data for every sensor,
     * but rather data for one of the sensors (each sensor will get a message sent for it). For more information,
     * see the SCR 2025 AutoNav CAN specification
     * @param msg an Ultrasonics message from a sensor
     */
    void onUltrasonicsReceived(const autonav_msgs::msg::Ultrasonics msg) {
        this->ultrasonic_feelers[msg.id - 1]->setLength(msg.distance); // minus 1 because the sensors are numbered 1-8
    }

private:
    // feelers
    std::vector<Feeler> feelers;
    std::vector<Feeler> ultrasonic_feelers;
    Feeler headingArrow = Feeler(0, 0);

    GPSPoint goalPoint;
    autonav_msgs::msg::Position position;

    bool newDebugImage = false;
    cv::Mat debug_image;
    
    // subscribers
    rclcpp::Subscription<autonav_msgs::msg::Position>::SharedPtr positionSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr imageSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr debugImageSubscriber;
    rclcpp::Subscription<autonav_msgs::msg::Ultrasonics>::SharedPtr ultrasonicSubscriber;
    
    // publishers
    rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr motorPublisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debugPublisher;
    rclcpp::Publisher<autonav_msgs::msg::SafetyLights>::SharedPtr safetyLightsPublisher;
    rclcpp::Publisher<autonav_msgs::msg::AudibleFeedback>::SharedPtr audibleFeedbackPublisher;

    // stuff for file-reading code (copied and pasted from https://github.com/SoonerRobotics/autonav_software_2024/blob/feat/astar_rewrite_v3/autonav_ws/src/autonav_nav/src/astar.cpp)
    const std::string WAYPOINTS_FILENAME = "./data/waypoints.csv"; // filename for the waypoints (should be CSV file with label,lat,lon,)
    std::ifstream waypointsFile; // actual C++ file object
    std::unordered_map<std::string, std::vector<GPSPoint>> waypointsDict; // dictionairy of lists containing the GPS waypoints we could PID to, choose the waypoints for the correct direction from here
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<FeelerNode> feeler_node = std::make_shared<FeelerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(feeler_node);
    executor.spin();
    executor.remove_node(feeler_node);
    rclcpp::shutdown();
    return 0;
}