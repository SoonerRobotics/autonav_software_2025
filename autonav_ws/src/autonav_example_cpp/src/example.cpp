#include "autonav_shared/node.hpp"


struct ExampleCPPConfig
{
    float alpha;
    std::string beta;
    int gamma;
    bool delta;
    std::vector<float> epsilon;
    std::vector<std::string> zeta;
    std::map<std::string, std::string> eta;
    int rid;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(ExampleCPPConfig, alpha, beta, gamma, delta, epsilon, zeta)
};

class ExampleCPP : public AutoNav::Node
{
    public:
        ExampleCPP() : AutoNav::Node("example_cpp") 
        {
            auto config = ExampleCPPConfig();
            config.alpha = 0.5;
            config.beta = "Hello";
            config.gamma = 42;
            config.delta = true;
            config.epsilon = {0.1, 0.2, 0.3};
            config.zeta = {"A", "B", "C"};
            config.eta = {{"A", "1"}, {"B", "2"}, {"C", "3"}};
            config.rid = 1;
            write_config(config);

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

            request_all_configs();

            log("Beta (CONFIG): " + config["beta"].get<std::string>(), AutoNav::Logging::DEBUG);
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