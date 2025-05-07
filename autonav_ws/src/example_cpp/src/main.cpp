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
        // This will initialize our default config
        auto config = ExampleCPPConfig();
        config.alpha = 0.5;
        config.beta = "Hello";
        config.gamma = 42;
        config.delta = true;
        config.epsilon = {0.1, 0.2, 0.3};
        config.zeta = {"A", "B", "C"};
        config.eta = {{"A", "1"}, {"B", "2"}, {"C", "3"}};
        config.rid = 1;
        this->_config = config;
        
        // If you care about type safety, do this
        this->config = config;

        // DO NOT USE CONFIG VARIABLES IN THE CONSTRUCTOR
        // ^ the config may be updated by the commander. Do all your initialization that requires the config in init()
    }
    ~ExampleCPP() = default;

    void init() override
    {
        log("Initialized");
        set_device_state(AutoNav::DeviceState::READY);

        perf_start("ExampleCPP::init");
        perf_stop("ExampleCPP::init", true);
    }

    void on_config_updated(const json &old_cfg, const json &new_cfg) override
    {
        auto new_config = new_cfg.get<ExampleCPPConfig>();
        this->config = new_config;
        log("Config updated from " + old_cfg.dump() + " to " + new_cfg.dump());
    }

    void on_system_state_updated(AutoNav::SystemState old, AutoNav::SystemState new_state) override
    {
        log("System state changed from " + std::to_string(static_cast<int>(old)) + " to " + std::to_string(static_cast<int>(new_state)));
    }

    void on_mobility_updated(bool old, bool new_state) override
    {
        log("Mobility changed from " + std::to_string(old) + " to " + std::to_string(new_state));
    }

private:
    // If you care about type safety, do this
    ExampleCPPConfig config;
};

int main(int argc, char **argv)
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