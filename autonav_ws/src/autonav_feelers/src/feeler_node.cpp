#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <chrono>

#include "autonav_shared/node.hpp"
#include "feeler.cpp"
#include "cv_bridge/cv_bridge.hpp"
#include "autonav_msgs/msg/position.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_msgs/msg/ultrasonic.hpp"
#include "autonav_msgs/msg/safety_lights.hpp"
#include "autonav_msgs/msg/audible_feedback.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

#define now() (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())).count();

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
    int max_length; // pixels
    int number_of_feelers;
    double start_angle; // degrees
    double waypointPopDist; // meters?
    double ultrasonic_contribution; // weight between 0 and 2 (or higher)
    unsigned long gpsWaitSeconds;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(FeelerNodeConfig, max_length, number_of_feelers, start_angle, waypointPopDist, ultrasonic_contribution, gpsWaitSeconds);
};

class FeelerNode : public AutoNav::Node {
public:
    FeelerNode() : AutoNav::Node("autonav_feelers") {}
    ~FeelerNode() = default;

    void init() override {
        auto config_ = FeelerNodeConfig();
        config_.max_length = 200;
        config_.number_of_feelers = 10;
        config_.start_angle = 5;
        config_.waypointPopDist = 2;
        config_.ultrasonic_contribution = 1;
        config_.gpsWaitSeconds = 0;

        this->write_config(config_);

        // === read waypoints from file === (copied and pasted from last year's feat/astar_rewrite_v3 branch)
        std::string line;
        this->waypointsFile.open(this->WAYPOINTS_FILENAME);
        getline(waypointsFile, line); // skip the first line
        int numWaypoints = 0;
        while (getline(waypointsFile, line)) { 
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
            numWaypoints++;
        }
        waypointsFile.close();
        // === /read waypoints ===

        log("Number of waypoints read: " + std::to_string(numWaypoints), AutoNav::Logging::INFO);

        if (numWaypoints < 1) {
            log("No waypoints read! GPS Feeler will not work", AutoNav::Logging::WARN);
        }

        // subscribers
        positionSubscriber = create_subscription<autonav_msgs::msg::Position>("/autonav/position", 1, std::bind(&FeelerNode::onPositionReceived, this, std::placeholders::_1));
        imageSubscriber = create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/vision/combined/filtered", 1, std::bind(&FeelerNode::onImageReceived, this, std::placeholders::_1));
        debugImageSubscriber = create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/vision/combined", 1, std::bind(&FeelerNode::onDebugImageReceived, this, std::placeholders::_1));
        ultrasonicSubscriber = create_subscription<autonav_msgs::msg::Ultrasonic>("/autonav/ultrasonics", 1, std::bind(&FeelerNode::onUltrasonicsReceived, this, std::placeholders::_1));
        
        // publishers
        motorPublisher = create_publisher<autonav_msgs::msg::MotorInput>("/autonav/motor_input", 1);
        debugPublisher = create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/feelers/debug", 1);
        safetyLightsPublisher = create_publisher<autonav_msgs::msg::SafetyLights>("/autonav/safety_lights", 1);
        audibleFeedbackPublisher = create_publisher<autonav_msgs::msg::AudibleFeedback>("/autonav/audible_feedback", 1);

        // make all the feelers
        this->buildFeelers();

        // make the feelers for the ultrasonics
        ultrasonic_feelers = std::vector<Feeler>();
        for (double angle = 0.0; angle < 360; angle += 90) { // these originate at the origin, which is fine because the only contribute in one axis
            int x = this->config["max_length"].get<int>() * cos(radians(angle)); //int should truncate these to nice whole numbers
            int y = this->config["max_length"].get<int>() * sin(radians(angle));

            this->feelers.push_back(Feeler(x, y));
            this->feelers.push_back(Feeler(x, y)); // there are 2 ultrasonic distance sensors per side
        }

        for (Feeler feeler : ultrasonic_feelers) {
            feeler.setColor(cv::Scalar(0, 200, 0)); // ultrasonic feelers are a different color
        }

        publishTimer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&FeelerNode::publishMotorOutput, this));

        lastTime = now();

        set_device_state(AutoNav::DeviceState::READY);
    }

    /**
     * Builds the list of feelers based on configuration parameters.
     * Evenly distributes a num_feelers number of feelers in a circle of radius max_length
     * starting with an offset of start_angle (in degrees).
     */
    void buildFeelers() {
        this->feelers = std::vector<Feeler>();
        for (double angle = this->config["start_angle"].get<double>(); angle < 360; angle += (360 / this->config["start_angle"].get<double>())) {
            int x = this->config["max_length"].get<int>() * cos(radians(angle)); //int should truncate these to nice whole numbers, I hope
            int y = this->config["max_length"].get<int>() * sin(radians(angle));

            this->feelers.push_back(Feeler(x, y));
        }

        for (Feeler feeler : feelers) {
            feeler.setColor(cv::Scalar(200, 0, 0)); // feelers are blue, openCV is in BGR
        }
    }

    /**
     * Callback for the combined image from the cameras.
     * @param image a compressedimage message with the combined transformations of all 4 cameras
     */
    void onImageReceived(sensor_msgs::msg::CompressedImage const image) {
        set_device_state(AutoNav::DeviceState::OPERATING);

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
        for (Feeler feeler : this->feelers) {
            feeler.draw(debug_image_ptr->image);
            this->headingArrow = this->headingArrow + feeler;
        }

        // ultrasonics
        for (Feeler feeler : this->ultrasonic_feelers) {
            // ultrasonic feelers contribute twice as much because there are fewer of them
            this->headingArrow = this->headingArrow + feeler;

            feeler.draw(debug_image_ptr->image);
        }
    }

    /**
     * Callback to receive and color image from the cameras to draw the feelers on for debug purposes
     * @param image the compressedImage message to draw the feelers on
     */
    void onDebugImageReceived(const sensor_msgs::msg::CompressedImage image) {
        this->newDebugImage = true;

        this->debug_image_ptr = cv_bridge::toCvCopy(image); //TODO figure out what encoding we want to use
    }

    /**
     * Callback to receive the position of the robot from the particle filter
     * we need to know this so we can figure out which way to move to get to the GPS waypoints
     * @param msg the Postion message from the particle filter
     */
    void onPositionReceived(const autonav_msgs::msg::Position msg) {
        this->position = msg;

        this->hasPosition = true;
    }

    /**
     * Callback to get data from the ultrasonic sensors. This message does not contain the data for every sensor,
     * but rather data for one of the sensors (each sensor will get a message sent for it). For more information,
     * see the SCR 2025 AutoNav CAN specification
     * @param msg an Ultrasonic message from a sensor
     */
    void onUltrasonicsReceived(const autonav_msgs::msg::Ultrasonic msg) {
        this->ultrasonic_feelers[msg.id - 1].setLength(msg.distance * this->config["ultrasonic_contribution"].get<double>()); // minus 1 because the sensors are numbered 1-8
    }

    /**
     * Publish the motor output messages. Main feeler node function.
     */
    void publishMotorOutput() {
        // if we aren't in autonomous
        if (this->get_system_state() != AutoNav::SystemState::AUTONOMOUS && this->get_device_state() != AutoNav::DeviceState::OPERATING) {
            return; // return because we don't need to do anything
        }

        if (this->gpsTime == 0 && this->hasPosition) {
            this->gpsTime = now();
        } else if (now() - this->gpsTime > this->config["gpsWaitSeconds"].get<unsigned long>() && this->direction == "") { // if it's been 6 seconds and we haven't set the direction yet
            double heading_degrees = abs(this->position.theta * 180 / PI);
            if (120 < heading_degrees && heading_degrees < 240) {
                this->direction = "south";
                log("PICKING SOUTH WAYPOINTS", AutoNav::Logging::INFO);
            } else {
                this->direction = "north";
                log("PICKING NORTH WAYPOINTS", AutoNav::Logging::INFO);
            }
        }

        // make the safety lights message for publishing
        autonav_msgs::msg::SafetyLights safetyLightsMsg;
        safetyLightsMsg.autonomous = true; // if we passed the system state check at the beginning of the function and reach this line of code then we're in auto
        
        // if we are allowed to move (earlier check means we are already in auto and operating, so don't have to recheck those)
        if (this->is_mobility()) {
            // make the message
            autonav_msgs::msg::MotorInput msg;

            // add a bias forwards
            this->headingArrow = this->headingArrow + Feeler(0, 50);

            // add a bias towards the GPS waypoint
            //FIXME this doesn't account for the rotation of the robot
            //FIXME the clamping should be configurable or something
            double distToWaypoint = 500;
            if (this->direction != "") {
                GPSPoint goalPoint = this->waypointsDict.at(this->direction)[this->waypointIndex];
                this->headingArrow = this->headingArrow + Feeler(std::clamp(goalPoint.lon - this->position.longitude, -200.0, 200.0), std::clamp(goalPoint.lat - this->position.latitude, -200.0, 200.0));
                distToWaypoint = std::sqrt(std::pow((goalPoint.lon - this->position.longitude)*this->latitudeLength, 2) + std::pow((goalPoint.lat - this->position.latitude)*this->longitudeLength, 2));

                // if we are close enough to the waypoint, and we aren't going to cause an out-of-bounds index error
                if (distToWaypoint < config["waypointPopDist"].get<double>() && this->waypointIndex < this->waypointsDict[this->direction].size()-1) {
                    // then go to the next waypoint
                    this->waypointIndex++;
                }
            }

            this->headingArrow.draw(debug_image_ptr->image);

            // convert headingArrow to motor outputs
            //FIXME we want to be going max speed on the straightaways
            msg.forward_velocity = std::clamp(this->headingArrow.getY(), -5, 5);
            msg.sideways_velocity = std::clamp(this->headingArrow.getX(), -5, 5);
            msg.angular_velocity = 0.0; //TODO figure out when we want to turn

            // default in auto should be red
            safetyLightsMsg.red = 255;
            safetyLightsMsg.blue = 0;
            safetyLightsMsg.green = 0;

            //TODO safety lights need to change to other colors and stuff for debug information

            //publish the motor message
            this->motorPublisher->publish(msg);
            // and publish the safety lights message
            this->safetyLightsPublisher->publish(safetyLightsMsg);
            // and publish the debug image
            this->debugPublisher->publish(*(debug_image_ptr->toCompressedImageMsg()));

            // make the audible feedback message
            //TODO figure out what sounds we actually want to play and when
            autonav_msgs::msg::AudibleFeedback feedback_msg;
            bool publishAudible = true;
            if (distToWaypoint < config["waypointPopDist"].get<double>()) {
                feedback_msg.filename = "ding.mp3";
            } else if (msg.forward_velocity < -0.5) { //TODO this is all robot-relative movement, so 'left' and 'right' here refer to the robot's POV, and if we end up moving primarily sideways through ex No-man's land then this could get annoying
                feedback_msg.filename = "beep_beep.mp3";
            } else if (msg.sideways_velocity < -1) {
                feedback_msg.filename = "going_left.mp3";
            } else if (msg.sideways_velocity > 1) {
                feedback_msg.filename = "going_right.mp3";
            } else {
                publishAudible = false;
            }
            
            // if we are actually wanting to play a file
            if (publishAudible) {
                this->audibleFeedbackPublisher->publish(feedback_msg);
            }

        } else {
            // we are not mobility enabled and thus not allowed to move, so publish velocities of 0 for everything
            autonav_msgs::msg::MotorInput msg;
            msg.forward_velocity = 0.0;
            msg.sideways_velocity = 0.0;
            msg.angular_velocity = 0.0;

            // default in auto should be red
            safetyLightsMsg.red = 255;
            safetyLightsMsg.blue = 0;
            safetyLightsMsg.green = 0;

            this->motorPublisher->publish(msg);
            this->safetyLightsPublisher->publish(safetyLightsMsg);
            this->debugPublisher->publish(*(debug_image_ptr->toCompressedImageMsg()));
        }
    }

private:
    // feelers
    std::vector<Feeler> feelers;
    std::vector<Feeler> ultrasonic_feelers;
    Feeler headingArrow = Feeler(0, 0);

    bool newDebugImage = false;
    cv_bridge::CvImagePtr debug_image_ptr;

    // GPS
    GPSPoint goalPoint;
    autonav_msgs::msg::Position position;
    unsigned long int lastTime = 0;
    unsigned long int gpsTime = 0;
    bool hasPosition = false;

    // FeelerNodeConfig config;
    
    // subscribers
    rclcpp::Subscription<autonav_msgs::msg::Position>::SharedPtr positionSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr imageSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr debugImageSubscriber;
    rclcpp::Subscription<autonav_msgs::msg::Ultrasonic>::SharedPtr ultrasonicSubscriber;
    
    // publishers
    rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr motorPublisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debugPublisher;
    rclcpp::Publisher<autonav_msgs::msg::SafetyLights>::SharedPtr safetyLightsPublisher;
    rclcpp::Publisher<autonav_msgs::msg::AudibleFeedback>::SharedPtr audibleFeedbackPublisher;

    rclcpp::TimerBase::SharedPtr publishTimer;

    // stuff for file-reading code (copied and pasted from https://github.com/SoonerRobotics/autonav_software_2024/blob/feat/astar_rewrite_v3/autonav_ws/src/autonav_nav/src/astar.cpp)
    const std::string WAYPOINTS_FILENAME = "./data/waypoints.csv"; // filename for the waypoints (should be CSV file with label,lat,lon,)
    std::ifstream waypointsFile; // actual C++ file object
    std::unordered_map<std::string, std::vector<GPSPoint>> waypointsDict; // dictionairy of lists containing the GPS waypoints we could PID to, choose the waypoints for the correct direction from here
    int waypointIndex = 0;
    std::string direction = "";
    double latitudeLength = 110944.21; // copied/pasted from last year's code, might need to change based on simulator
    double longitudeLength = 81978.2;
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