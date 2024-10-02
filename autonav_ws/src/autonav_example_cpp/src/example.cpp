#include "autonav_shared/node.hpp"

class ExampleCPP : public AutoNav::Node
{
    public:
        ExampleCPP() : AutoNav::Node("example_cpp") 
        {
            log("Hello from ExampleCPP", AutoNav::Logging::DEBUG);
            log("Hello from ExampleCPP", AutoNav::Logging::INFO);
            log("Hello from ExampleCPP", AutoNav::Logging::WARN);
            log("Hello from ExampleCPP", AutoNav::Logging::ERROR);
            log("Hello from ExampleCPP", AutoNav::Logging::FATAL);
        }
        ~ExampleCPP() = default;

        void init() override
        {
            log("Initialized");
            set_device_state(AutoNav::DeviceState::READY);

            perf_start("ExampleCPP::init");
            perf_stop("ExampleCPP::init", true);
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ExampleCPP> example_cpp = std::make_shared<ExampleCPP>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(example_cpp);
    executor.spin();
    executor.remove_node(example_cpp);
    rclcpp::shutdown();
    return 0;
}