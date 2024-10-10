#include "autonav_shared/node.hpp"

class VectorNavNode : public AutoNav::Node {
public:
    VectorNavNode() : AutoNav::Node("vectornav_node") {}
    ~VectorNavNode() {
        // disconnect the sensor when we're done with it
        sensor.disconnec();
    }

    void init() override {
        set_device_state(AutoNav::DeviceState::WARMING);

        // connect to the vectornav
        VN::Error e = sensor.connect(this->port, this->baudRate);

        // if the sensor didn't connect, log it
        if (e != VN::Error::None) {
            log("VectorNav Error: " + std::to_string(e), AutoNav::Logging::ERROR);
        }

        // while the sensor is not connected
        while (!sensor.verifyConnectivity()) {
            sleep(2); // wait 2 seconds
            sensor.autoConnect(this->port); // and try again
        }

        log("VectorNav Connected!", AutoNav::Logging::INFO);

        bool needToWriteSettings = false;

        // turn off the stream of data while we're configuring everything
        this->sensor.asyncOutputEnable(VN::asyncOutputEnable::State::Disable);

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
            wait(2);
            this->sensor.reset(); // and so reset the Kalman filter
            wait(2);

            needToWriteSettings = false;
        }

        // set the initial heading of the robot
        this->sensor.setInitialHeading(0.0); //FIXME should be in degrees, north-oriented

        //TODO: publish the message from the data or whatever
        //TODO: publish debug data to UI or something
        //TODO: log data



        // GPS is critical information and should be top priority
        // heading is really nice to have too
        // don't care as much about linear acceleration but that'd be cool to
        // maybe velocity?
        // don't need anything else
        // kill the magnetometer

/**
        ImuStatus
        *gyro status
        *accel status
        AngularRate
        *gyroX
        *gyroY
        *gyroZ
        NumSats
        GnssFix
        GnssPosLla
        GnssStatus
        Ypr
        LinBodyAcc

*/

        // turn the data streams back on
        this->sensor.asyncOutputEnable(VN::asyncOutputEnable::State::Enable);

        set_device_state(AutoNav::DeviceState::READY);
    }
private:
    VN::Sensor sensor; // the actual vectornav object (we have a VN200 rugged)
    std::string port = "COM3"; //FIXME
    VN::Sensor::BaudRate baudRate = VN::Sensor::BaudRate::115200;
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