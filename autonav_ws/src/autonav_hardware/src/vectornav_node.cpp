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

        //TODO read settings from the registers and check if they're correct
        // ascii async data 1: OFF
        // ascii async data 2: OFF
        // binaryOutput 2: OFF
        // binary output 1: matches what we have
        // baudrate = 115200
        // user tag = "danger zone" or "TBD"
        // legacy compatability settings = all off
        // vpe basic control heading mode = relative
        // vpe filtering mode = adaptively filtered
        // vpe tuning mode = adaptive
        // vpe magnetometer basic tuning X, Y, Z confidence level = 0
        // delta theta + delta velocity configuration integration frame = body
        // dt+dv config gyro comp = bias
        // dtdv config accel compensation = gravity
        // dtdv config earth rate comp = gyro rate
        // imu filtering config:
        //  mag filter mode = comp
        //  all others = comp??
        //  window size = ???
        //   mag convergence rate = 3???
        // velocity aiding = 0 / off
        // gnss config
        //  receiver enable = internal gnss reeiv
        //  gnss rate = 5 hz
        //  gnss internal A antenna offset x, y, z = TODO
        //  systems = GPS, SBAS, and none others?
        //  sbas mode = integrity
        //  minCN0 = -32 dB or something idk
        //  minElev = 10 deg or something idk
        //  maxSats = 32 or something idk
        // ins config
        //  scenario = GnssInsWithPressure
        //  ahrsAiding = disable
        // world/gravity model: TODO page 115

        // for each of the configuration registers we want to check
        for (VN::Registers r : this->registers) {
            // read the register
            VN::Error readError = sensor.readRegister(&r);

            // if there's an error
            if (readError != VN::Error::None) {
                // report it and skip this register
                log("Register reading error! " + *VN::errorCodeToString(readError), AutoNav::Logging::ERROR);
                this->set_device_state(AutoNav::DeviceState::ERROR);
            } else {
                // otherwise check if it has the right settings in it
                if (r != r) { //TODO FIXME this is wrong and bad also probably won't compile
                    writeSettings = true; // so we need to update the settings

                    // which means we don't need to check the rest of the settings (if we're writing them all)
                    break;
                }
            }
        }

        // configure the binary output register
        this->outputRegister.rateDivisor = 200;
        this->outputRegister.asyncMode.serial1 = true;

        this->outputRegister.imu.temperature = true;

        this->outputRegister.gnss.numSats = true;
        this->outputRegister.gnss.gnssFix = true;
        this->outputRegister.gnss.gnss1PosLla = true; // TODO do we want gps pos LLA or INS fused pos LLA?
        this->outputRegister.gnss.gnssDop = true;
        this->outputRegister.gnss.gnssStatus = true;

        this->outputRegister.ins.insStatus = true;
        this->outputRegister.ins.posLla = true;

        //TODO need to check if current binary register matches the code and if not then a writeSettings() is needed

        auto latestError = sensor.writeRegister(&this->outputRegister);
        if (latestError != VN::Error::None) {
            log("Register writing error!", AutoNav::Logging::ERROR);
        }

        // if we need to write the settings to flash
        while (needToWriteSettings) {
            if (this->get_system_state() != AutoNav::SystemState::DISABLED) { // only write to flash if we're not moving
                continue;
            }

            //NOTE: this can take anywhere from 500 to 1000ms according to the ICD
            this->sensor.writeSettings();
            log("Writing VectorNav settings, this could take a second", AutoNav::Logging::WARN);

            std::this_thread::sleep_for(std::chrono::seconds(2));
            this->sensor.reset(); // and so reset the on-sensor Kalman filter
            std::this_thread::sleep_for(std::chrono::seconds(2));

            needToWriteSettings = false;
            log("VectorNav settings burned to flash", AutoNav::Logging::INFO);
        }

        // set the initial heading of the robot
        // this->sensor.setInitialHeading(0.0); //FIXME should be in degrees, north-oriented TODO

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

    VN::Registers registers[] = {

    };
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