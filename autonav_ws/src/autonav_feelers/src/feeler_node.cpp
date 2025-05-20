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

//TODO FIXME this is not technically correct, think I need a modulo
double wrapAngle(double deg) {
    if (deg < 0) {
        return deg + 360;
    } else if (deg > 360) {
        return deg - 360;
    }
    return deg;
}

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
    double gpsBiasWeight; // pixels
    double forwardBiasWeight; // pixels
    
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(FeelerNodeConfig, max_length, number_of_feelers, start_angle, waypointPopDist, ultrasonic_contribution, gpsWaitSeconds);
};

class FeelerNode : public AutoNav::Node {
public:
    FeelerNode() : AutoNav::Node("autonav_feelers") {
        // configuration stuff
        auto config = FeelerNodeConfig();
        config.max_length = 200;
        config.number_of_feelers = 35;
        config.start_angle = 5;
        config.waypointPopDist = 2;
        config.ultrasonic_contribution = 1;
        config.gpsWaitSeconds = 5;
        config.gpsBiasWeight = 50;
        config.forwardBiasWeight = 50;

        this->_config = config;
        this->config = config;
    }
    ~FeelerNode() = default;

    void init() override {
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
        this->waypointIndex = 0;
        // === /read waypoints ===

        log("Number of waypoints read: " + std::to_string(numWaypoints), AutoNav::Logging::INFO);

        if (numWaypoints < 1) {
            log("No waypoints read! GPS Feeler will not work", AutoNav::Logging::WARN);
        }

        // make all the feelers
        this->buildFeelers();

        // make the feelers for the ultrasonics
        //TODO this should be part of buildFeelers or its own function or something
        ultrasonic_feelers = std::vector<Feeler>();
        for (double angle = 0.0; angle < 360; angle += 90) { // these originate at the origin, which is fine because the only contribute in one axis
            int x = this->config.max_length * cos(radians(angle)); //int should truncate these to nice whole numbers
            int y = this->config.max_length * sin(radians(angle));

            this->ultrasonic_feelers.push_back(Feeler(x, y));
            this->ultrasonic_feelers.push_back(Feeler(x, y)); // there are 2 ultrasonic distance sensors per side
        }

        for (Feeler &feeler : ultrasonic_feelers) {
            feeler.setColor(cv::Scalar(0, 200, 100)); // ultrasonic feelers are a different color
        }

        lastTime = now();

        // subscribers
        positionSubscriber = create_subscription<autonav_msgs::msg::Position>("/autonav/position", 1, std::bind(&FeelerNode::onPositionReceived, this, std::placeholders::_1));
        imageSubscriber = create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/vision/combined/filtered", 1, std::bind(&FeelerNode::onImageReceived, this, std::placeholders::_1));
        debugImageSubscriber = create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/vision/combined/debug", 1, std::bind(&FeelerNode::onDebugImageReceived, this, std::placeholders::_1));
        ultrasonicSubscriber = create_subscription<autonav_msgs::msg::Ultrasonic>("/autonav/ultrasonics", 1, std::bind(&FeelerNode::onUltrasonicsReceived, this, std::placeholders::_1));
        
        // publishers
        motorPublisher = create_publisher<autonav_msgs::msg::MotorInput>("/autonav/motor_input", 1);
        debugPublisher = create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/feelers/debug", 1);
        safetyLightsPublisher = create_publisher<autonav_msgs::msg::SafetyLights>("/autonav/safety_lights", 1);
        audibleFeedbackPublisher = create_publisher<autonav_msgs::msg::AudibleFeedback>("/autonav/audible_feedback", 1);
        publishTimer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&FeelerNode::publishOutputMessages, this));

        set_device_state(AutoNav::DeviceState::READY);

        log("FEELERS READY!", AutoNav::Logging::WARN); //FIXME TODO
        //FIXME this is for temporary debug purposes while we are minus a UI
        this->set_system_state(AutoNav::SystemState::AUTONOMOUS, true);
    }

    void on_config_updated(const json &old_cfg, const json &new_cfg) override {
        auto new_config = new_cfg.get<FeelerNodeConfig>();
        this->config = new_config;
    }

    /**
     * Builds the list of feelers based on configuration parameters.
     * Evenly distributes a num_feelers number of feelers in a circle of radius max_length
     * starting with an offset of start_angle (in degrees).
     */
    void buildFeelers() {
        this->feelers = std::vector<Feeler>();
        for (double angle = this->config.start_angle; angle < 360; angle += (360 / this->config.number_of_feelers)) {
            int x = this->config.max_length * cos(radians(angle)); //int should truncate these to nice whole numbers, I hope
            int y = this->config.max_length * sin(radians(angle));
            
            this->feelers.push_back(Feeler(x, y));
        }

        for (Feeler feeler : feelers) {
            feeler.setColor(cv::Scalar(200, 0, 0)); // feelers are blue, openCV is in BGR
        }

        log("FEELERS BUILT!", AutoNav::Logging::WARN); //FIXME TODO
        log("NUMBER OF FEELERS: " + std::to_string(this->feelers.size()), AutoNav::Logging::WARN);
    }

    /**
     * Callback for the combined image from the cameras.
     * @param image a compressedimage message with the combined transformations of all 4 cameras
     */
    void onImageReceived(sensor_msgs::msg::CompressedImage const image) {
        // log("FEELERS OPERATING!", AutoNav::Logging::WARN); //FIXME TODO

        // once we've actually gotten an image, we can safely say we're operating pretty well
        if (this->get_device_state() != AutoNav::DeviceState::OPERATING) {
            set_device_state(AutoNav::DeviceState::OPERATING);
        }

        // turn the image into a format we can use
        auto mask = cv_bridge::toCvCopy(image)->image; //TODO what encoding do we want to use?
        // this->feeler_img_ptr = cv_bridge::toCvCopy(image);

        // log("LOGGING REGULAR IMAGE...", AutoNav::Logging::ERROR); //FIXME TODO

        //TODO TEMP HACK FIXME BUG draw on image to see if it has any effect
        // Feeler tempFeeler = Feeler(100, 100);
        // tempFeeler.setColor(cv::Scalar(200, 200, 100));
        // tempFeeler.draw(mask);

        // log("drew that one image", AutoNav::Logging::INFO); //FIXME TODO

        // this->perf_start("FeelerNode::update");

        //TODO FIXME bias the feelers forwards

        // calculate new length of every new feeler
        for (Feeler &feeler : this->feelers) {
            feeler.update(&mask, this);
            // log(feeler.to_string(), AutoNav::Logging::WARN); //FIXME TODO
        }

        // log("first feeler: " + this->feelers.at(0).to_string(), AutoNav::Logging::INFO);

        // this->perf_stop("FeelerNode::update", true);

        // log("FEELERS DRAWING!", AutoNav::Logging::WARN); //FIXME TODO
        // log("FEELERS LENGTH, MASK ROWS, MASK COLS, DEBUG ROWS, DEBUG COLS", AutoNav::Logging::WARN);
        // log(std::to_string(this->feelers.size()), AutoNav::Logging::WARN);
        // log(std::to_string(mask.cols), AutoNav::Logging::WARN);
        // log(std::to_string(mask.rows), AutoNav::Logging::WARN);
        // log(std::to_string(debug_image_ptr->cols), AutoNav::Logging::WARN);
        // log(std::to_string(debug_image_ptr->rows), AutoNav::Logging::WARN);

        // log("chat are we cooked", AutoNav::Logging::WARN); //FIXME TODO

        this->calculateOutputs();
    }

    /**
     * Callback to receive the position of the robot from the particle filter
     * Uses the position to calculate what direction we need to move in to get to next waypoint, as well as popping waypoints once we get close enough
     * Also automatically calculates if we're going north or south
     * All code that is related to position-based stuff is located here, as it doesn't get updated anywhere else
     * No output, but does updates GPS feeler, which is added to headingArrow later to drive us towards the waypoint
     * @param msg the Postion message from the particle filter
     */
    void onPositionReceived(const autonav_msgs::msg::Position msg) {
        this->position = msg;

        // log("GOT GPS!", AutoNav::Logging::INFO);

        // if we haven't set a timestamp yet, but have started the run
        if (this->gpsTime == 0 && this->is_mobility() && this->get_system_state() == AutoNav::SystemState::AUTONOMOUS) {
            this->gpsTime = now(); // then set the timestamp for the start of the run
        // if, however, we have set a timestamp, and it's been long enough that the particle filter should know which direction we're heading
        } else if (now() - this->gpsTime > this->config.gpsWaitSeconds && this->direction == "") {
            // then pick a set of waypoints based on which direction we are heading
            double heading_degrees = abs(this->position.theta * 180 / PI);
            if (120 < heading_degrees && heading_degrees < 240) {
                this->direction = "compSouth";
                log("PICKING SOUTH WAYPOINTS", AutoNav::Logging::INFO);
            } else {
                this->direction = "compNorth";
                log("PICKING NORTH WAYPOINTS", AutoNav::Logging::INFO);
            }
        }

        this->distToWaypoint = 0;
        if (this->direction != "") { // if we have a direction, then we are good to use it to get waypoints and go towards them
            // if we don't have any waypoints, however
            if (this->waypointsDict.size() == 0) {
                //TODO do something???
                return;
            }
            GPSPoint goalPoint = this->waypointsDict.at(this->direction)[this->waypointIndex];
            
            // log("biasing my robot rn", AutoNav::Logging::INFO);

            // make a vector pointing towards the GPS waypoint
            double latError = goalPoint.lat - this->position.latitude;
            double lonError = goalPoint.lon - this->position.longitude;
            Feeler gpsFeeler = Feeler(lonError, latError);

            Feeler velocityFeeler = Feeler(this->position.x_vel, this->position.y_vel);

            // calculate bias for every feeler
            for (Feeler feeler : this->feelers) {
                double gps_bias = this->config.gpsBiasWeight * (feeler * gpsFeeler); // dot product (normalized, don't worry)
                double forward_bias = this->config.forwardBiasWeight * (feeler * velocityFeeler); // dot product

                if (gps_bias < 0.0) {
                    gps_bias = 0.0;
                }

                if (forward_bias < 0.0) {
                    forward_bias = 0.0;
                }

                feeler.bias(gps_bias + forward_bias);
            }

            // log("ROBOT has been BIASED", AutoNav::Logging::INFO);

            this->distToWaypoint = std::sqrt(std::pow((goalPoint.lon - this->position.longitude)*this->latitudeLength, 2) + std::pow((goalPoint.lat - this->position.latitude)*this->longitudeLength, 2));

            // if we are close enough to the waypoint, and we aren't going to cause an out-of-bounds index error
            if (this->distToWaypoint < config.waypointPopDist && this->waypointIndex < this->waypointsDict[this->direction].size()-1) {
                // then go to the next waypoint
                this->waypointIndex++;

                log("NEXT WAYPOINT!", AutoNav::Logging::WARN);
            }
        }

        this->calculateOutputs();
    }

    /**
     * Callback to get data from the ultrasonic sensors. This message does not contain the data for every sensor,
     * but rather data for one of the sensors (each sensor will get a message sent for it). For more information,
     * see the SCR 2025 AutoNav CAN specification
     * @param msg an Ultrasonic message from a sensor
     */
    void onUltrasonicsReceived(const autonav_msgs::msg::Ultrasonic msg) {
        // log("GETTING ULTRASONICS!", AutoNav::Logging::WARN); //FIXME TODO

        this->ultrasonic_feelers[msg.id - 1].setLength(msg.distance * this->config.ultrasonic_contribution); // minus 1 because the sensors are numbered 1-8

        // log("ULTRASONICS GOT!", AutoNav::Logging::WARN); //FIXME TODO

        this->calculateOutputs();
    }

    /**
     * Callback to receive the color image to draw debug information on
     * All draw() calls should be in this function.
     * @param image the compressedImage message to draw the feelers on
     */
    void onDebugImageReceived(const sensor_msgs::msg::CompressedImage image) {
        // update the headingArrow with the most recent information
        this->calculateOutputs();

        // log("GETTING DEBUG IMAGE!", AutoNav::Logging::WARN); //FIXME TODO

        // get the debug image
        this->debug_image_ptr = cv_bridge::toCvCopy(image); //TODO figure out what encoding we want to use

        // don't publish or draw on the image if it doesn't exist
        if (this->debug_image_ptr == nullptr) {
            return;
        }

        // log("LOGGING DEBUG IMAGE RECEIVED...", AutoNav::Logging::ERROR); //FIXME TODO

        // draw feelers on the debug image
        // this->perf_start("FeelerNode::draw");
        for (Feeler &feeler : this->feelers) {
            // log(feeler.to_string(), AutoNav::Logging::WARN);
            // log("==================", AutoNav::Logging::WARN);
            // feeler.setXY(100, 100);
            feeler.draw(this->debug_image_ptr->image);
            // log(feeler.to_string(), AutoNav::Logging::WARN);
        }

        // log("DREW FEELERS!", AutoNav::Logging::ERROR); //FIXME TODO

        // draw the ultrasonic feelers on the image (on top of the vision feelers)
        // for (Feeler feeler : this->ultrasonic_feelers) {
        //     feeler.draw(this->debug_image_ptr->image);
        // }

        // draw the heading arrow on top of everything else
        this->headingArrow.draw(this->debug_image_ptr->image);
        // this->perf_stop("FeelerNode::draw", true);

        // publish the debug image
        this->debugPublisher->publish(*(debug_image_ptr->toCompressedImageMsg())); //TODO
    }

    /**
     * Calculate what the motor output should be, based on all available sensor inputs.
     * Main feeler node function, called in every sensor callback, as new sensor data = new motor outputs.
     * Motor outputs (aka this->headingArrow) calculated here will be read by publishOutputMessages() to be put into message form and sent,
     * as publishOutputMessages() runs much faster than all the sensor inputs for safety reasons, as the firmware
     * on the motor manager PCB will disable the motors if it hasn't received a motor command after a short period of time.
     */
    void calculateOutputs() {
        // reinitialize the heading arrow
        this->headingArrow = Feeler(0, 0);
        this->headingArrow.setColor(cv::Scalar(200, 200, 0));

        // add all the feelers together
        for (Feeler feeler : this->feelers) {
            //FIXME the weight of the feelers should be configurable (outside of MAX_LENGTH), or like give them a custom response curve or something
            this->headingArrow = this->headingArrow + feeler;
        }

        // add all the ultrasonic feelers together
        for (Feeler feeler : this->ultrasonic_feelers) {
            // FIXME TODO use the config ultrasonics_weight to determine how much ultrasonics effect the heading arrow
            // this->headingArrow = this->headingArrow + feeler;
        }
    }

    /**
     * Publish all the output messages (motors, audible feedback, and safety lights).
     * On a short timer so we don't fail the firmware heartbeat watchdog timer check thingamajig.
     */
    void publishOutputMessages() {
        // if we aren't in autonomous
        if (this->get_system_state() != AutoNav::SystemState::AUTONOMOUS && this->get_device_state() != AutoNav::DeviceState::OPERATING) {
            return; // return because we don't need to do anything (so as to avoid conflicting with manual control if that's running)
        }

        // log("PUBLISHING MOTOR OUTPUT!", AutoNav::Logging::WARN); //FIXME TODO

        // make the messages for publishing
        autonav_msgs::msg::SafetyLights safetyLightsMsg;
        autonav_msgs::msg::MotorInput msg;
        autonav_msgs::msg::AudibleFeedback feedback_msg;

        // default in auto should be red
        safetyLightsMsg.red = 255;
        safetyLightsMsg.blue = 0;
        safetyLightsMsg.green = 0;
        safetyLightsMsg.mode = 1; // if we passed the system state check at the beginning of the function and reach this line of code then we're in auto

        // if we are allowed to move (earlier check means we are already in auto and operating, so don't have to recheck those)
        if (this->is_mobility()) {
            // log("WE ARE MOBILE!", AutoNav::Logging::WARN); //FIXME TODO

            // convert headingArrow to motor outputs
            //FIXME we want to be going max speed on the straightaways
            //FIXME the clamping should be configurable or something
            msg.forward_velocity = std::clamp(static_cast<double>(this->headingArrow.getY()) / 20, -3.0, 3.0); //FIXME configure divider number thingy
            msg.sideways_velocity = std::clamp(static_cast<double>(this->headingArrow.getX()) / 20, -3.0, 3.0); //FIXME configure divider number thingy
            msg.angular_velocity = 0.0; //TODO figure out when we want to turn

            //TODO safety lights need to change to other colors and stuff for debug information
        } else {
            // log("NO MOBILITY!", AutoNav::Logging::WARN); //FIXME TODO

            // we are not mobility enabled and thus not allowed to move, so publish velocities of 0 for everything
            msg.forward_velocity = 0.0;
            msg.sideways_velocity = 0.0;
            msg.angular_velocity = 0.0;
        }

        //TODO figure out what sounds we actually want to play and when
        bool publishAudible = true;
        if (distToWaypoint < config.waypointPopDist) {
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
        
        // publish the messages
        this->motorPublisher->publish(msg);
        this->safetyLightsPublisher->publish(safetyLightsMsg);
        // log("MOTOR AND SAFETY LIGHTS PUBLISHED!", AutoNav::Logging::WARN); //FIXME TODO

        // if we are actually wanting to play a file
        if (publishAudible) {
            this->audibleFeedbackPublisher->publish(feedback_msg);
            // log("PUBLISHING AUDIBLE FEEDBACK!", AutoNav::Logging::WARN); //FIXME TODO
        }
    }

private:
    // feelers
    std::vector<Feeler> feelers;
    std::vector<Feeler> ultrasonic_feelers;
    Feeler headingArrow = Feeler(0, 0);

    // config
    FeelerNodeConfig config;

    cv_bridge::CvImagePtr debug_image_ptr;
    cv_bridge::CvImagePtr feeler_img_ptr;

    // GPS
    GPSPoint goalPoint;
    autonav_msgs::msg::Position position;
    double distToWaypoint = 0;
    unsigned long int lastTime = 0;
    unsigned long int gpsTime = 0;

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