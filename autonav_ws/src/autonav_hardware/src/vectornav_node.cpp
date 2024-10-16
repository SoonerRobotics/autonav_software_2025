#include <chrono>
#include <thread>

#include "autonav_shared/node.hpp"
#include "autonav_msgs/msg/position.hpp"
#include "autonav_msgs/msg/gps_feedback.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic" // to supress all the "ISO prevents anonymous structs" warnings
#include "Interface/Errors.hpp"
#include "Interface/Sensor.hpp"
#include "Interface/Registers.hpp"
#include "Interface/Command.hpp"
#pragma GCC diagnostic pop

class VectorNavNode : public AutoNav::Node {
public:
    VectorNavNode() : AutoNav::Node("vectornav_node") {}
    ~VectorNavNode() {
        // disconnect the sensor when we're done with it
        sensor.disconnect();
    }

    void init() override {
        set_device_state(AutoNav::DeviceState::WARMING);

        // connect to the vectornav
        VN::Error e = sensor.connect(this->port, this->baudRate);

        // if the sensor didn't connect, log it
        if (e != VN::Error::None) {
            // log("VectorNav Error: " + *VN::errorCodeToString(e), AutoNav::Logging::ERROR);
            log("VectorNav Connection Error!", AutoNav::Logging::ERROR);

            set_device_state(AutoNav::DeviceState::ERROR);
        }

        // while the sensor is not connected
        while (!sensor.verifySensorConnectivity()) {
            std::this_thread::sleep_for(std::chrono::seconds(2)); // wait 2 seconds
            sensor.autoConnect(this->port); // and try again

            log("Connecting...", AutoNav::Logging::WARN);
        }

        log("VectorNav Connected!", AutoNav::Logging::INFO);

        bool needToWriteSettings = false;

        // turn off the stream of data while we're configuring everything
        this->sensor.asyncOutputEnable(VN::AsyncOutputEnable::State::Disable);

        // configure the binary output register
        this->outputRegister.rateDivisor = 200;
        this->outputRegister.asyncMode.serial1 = true;
        this->outputRegister.gnss.gnss1PosLla = true;

        auto latestError = sensor.writeRegister(&this->outputRegister);
        if (latestError != VN::Error::None) {
            log("Register writing error!", AutoNav::Logging::ERROR);
        }

        //TODO: read the settings from the vectornav registers and stuff
        //TODO: if the settings aren't right then change them to be correct
        //TODO: save the settings to flash and power cycle?

        // if we need to write the settings to flash
        while (needToWriteSettings) {
            if (this->get_system_state() != AutoNav::SystemState::DISABLED) { // only write to flash if we're not moving
                continue;
            }

            //NOTE: this can take anywhere from 500 to 1000ms according to the ICD
            this->sensor.writeSettings();
            log("Writing VectorNav settings, this could take a second", AutoNav::Logging::WARN);

            std::this_thread::sleep_for(std::chrono::seconds(2));
            this->sensor.reset(); // and so reset the Kalman filter
            std::this_thread::sleep_for(std::chrono::seconds(2));

            needToWriteSettings = false;
            log("VectorNav settings burned to flash", AutoNav::Logging::INFO);
        }

        // set the initial heading of the robot
        this->sensor.setInitialHeading(0.0); //FIXME should be in degrees, north-oriented

        // turn the data streams back on
        this->sensor.asyncOutputEnable(VN::AsyncOutputEnable::State::Enable);

        //TODO we need to like, have an INS status thing and like on the UI or robot audible feedback or safety lights or whatever make sure it's in aligned mode,
        // because it needs to warm up for like 30 seconds, then drive over 5 m/s and then other stuff to be fully working right with the kalman filter and everything

        // publishers
        gpsPublisher = create_publisher<autonav_msgs::msg::GPSFeedback>("/autonav/gps", 1);

        // timers
        publishTimer = this->create_wall_timer(std::chrono::milliseconds(this->gpsRate), std::bind(&VectorNavNode::publishGps, this));

        log("VectorNav Node Init Finished!", AutoNav::Logging::INFO);

        set_device_state(AutoNav::DeviceState::READY);
    }

    /**
     * TODO
     */
    void publishGps() {
        // get the next measurement from the queue
        VN::Sensor::CompositeDataQueueReturn compositeData = this->sensor.getNextMeasurement();

        // if there is not a measurement on the queue for us to read
        if (!compositeData) {
            return;
        }

        if (this->get_device_state() != AutoNav::DeviceState::OPERATING) {
            // if we actually have data then we should be operating just fine
            set_device_state(AutoNav::DeviceState::OPERATING);

            log("DATA RECEIVED!!!!", AutoNav::Logging::INFO);
        }


        // if the binary output matches the message we configured it to give
        if (compositeData->matchesMessage(this->outputRegister)) {
            // make the message
            autonav_msgs::msg::GPSFeedback msg;

            // get the reading
            msg.latitude = compositeData->gnss.gnss1PosLla.value().lat;
            msg.longitude = compositeData->gnss.gnss1PosLla.value().lon;
            //TODO num_sats and status and stuff
            //Gnss1Fix, gnss1NumSats

            this->gpsPublisher->publish(msg);
            log("PUBLISHING!!!!!!", AutoNav::Logging::INFO);
        } else {
            log("Unrecognized VectorNav message", AutoNav::Logging::ERROR);
        }
    }
private:
    // vectornav stuff
    VN::Sensor sensor; // the actual vectornav object (we have a VN200 rugged)
    std::string port = "/dev/ttyUSB0"; //FIXME
    VN::Sensor::BaudRate baudRate = VN::Sensor::BaudRate::Baud115200;
    VN::Registers::System::BinaryOutput1 outputRegister;

    // publishers
    rclcpp::Publisher<autonav_msgs::msg::GPSFeedback>::SharedPtr gpsPublisher;
    rclcpp::TimerBase::SharedPtr publishTimer;
    int gpsRate = 1/5 * 1000; // 5 Hz in milliseconds
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<VectorNavNode> vectornav_node = std::make_shared<VectorNavNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(vectornav_node);
    executor.spin();
    executor.remove_node(vectornav_node);
    rclcpp::shutdown();
    return 0;
}