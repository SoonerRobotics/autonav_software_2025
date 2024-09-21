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
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExampleCPP>();
    rclcpp::spin(node);
    rclcpp::shutdown(); 
    return 0;
}