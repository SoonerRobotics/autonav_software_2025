#include "autonav_shared/node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"

struct ExpandifyConfig
{
    float vertical_fov;
    float horizontal_fov;
    int map_res;
    float max_range;
	float no_go_percent;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(ExpandifyConfig, vertical_fov, horizontal_fov, map_res, max_range, no_go_percent)
};

struct Circle
{
	int x;
	int y;
	double radius;
};

class Expandify : public AutoNav::Node
{
public:
	Expandify() : AutoNav::Node("zemlin_vision_expandify")
    {
        auto config = ExpandifyConfig();
        config.vertical_fov = 2.75;
        config.horizontal_fov = 3;
        config.map_res = 80;
        config.max_range = 0.55;
        config.no_go_percent = 0.6;
        this->_config = config;

        // If you care about type safety, do this
        this->config = config;
    }

	~Expandify() {}

    void on_config_updated(const json &old_cfg, const json &new_cfg) override
    {
        auto new_config = new_cfg.get<ExpandifyConfig>();
        this->config = new_config;
    }

	void init() override
	{
		log("Expandify initialized", AutoNav::Logging::LogLevel::INFO);

		map = nav_msgs::msg::MapMetaData();
		map.width = 100;
		map.height = 100;
		map.resolution = 0.1;
		map.origin = geometry_msgs::msg::Pose();
		map.origin.position.x = -10.0;
		map.origin.position.y = -10.0;

		auto tempRange = this->config.max_range * this->config.no_go_percent;
		_max_range = (int)(this->config.max_range / (this->config.horizontal_fov / (float)this->config.map_res));
		_no_go_range = (int)(tempRange / (this->config.horizontal_fov / (float)this->config.map_res));

		_circles.push_back(Circle{0, 0, 0});
		for (int x = -_max_range; x <= _max_range; x++)
		{
			for (int y = -_max_range; y <= _max_range; y++)
			{
				if (_max_range * _no_go_percent <= sqrt(x * x + y * y) && sqrt(x * x + y * y) < _max_range)
				{
					_circles.push_back(Circle{x, y, sqrt(x * x + y * y)});
				}
			}
		}

		raw_map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/raw", 20, std::bind(&Expandify::onConfigSpaceReceived, this, std::placeholders::_1));
		expanded_map_pub = create_publisher<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded", 20);
		debug_pub = create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/cfg_space/expanded/image", 20);

        set_device_state(AutoNav::DeviceState::OPERATING);
	}

	void onConfigSpaceReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr cfg)
	{
		std::vector<int8_t> cfg_space = std::vector<int8_t>(this->config.map_res * this->config.map_res);
		std::fill(cfg_space.begin(), cfg_space.end(), 0);

		for (int x = 0; x < this->config.map_res; x++)
		{
			for (int y = 1; y < this->config.map_res; y++)
			{
				if(cfg->data.at(x + y * this->config.map_res) > 0)
				{
					for (Circle& circle : _circles)
					{
						auto idx = (x + circle.x) + this->config.map_res * (y + circle.y);
						auto expr_x = (x + circle.x) < this->config.map_res && (x + circle.x) >= 0;
						auto expr_y = (y + circle.y) < this->config.map_res && (y + circle.y) >= 0;
						if (expr_x && expr_y)
						{
							auto val = cfg_space.at(idx);
							auto linear = 100 - ((circle.radius - _no_go_range) / (_max_range - _no_go_range) * 100);

							if (circle.radius <= _no_go_range)
							{
								cfg_space.at(idx) = 100;
							} else if (cfg_space.at(idx) <= 100 && val <= linear)
							{
								cfg_space.at(idx) = int(linear);
							}
						}
					}
				}
			}
		}

		auto newSpace = nav_msgs::msg::OccupancyGrid();
		newSpace.info = map;
		newSpace.data = cfg_space;
		expanded_map_pub->publish(newSpace);

        // debug stuff

		cv::Mat image = cv::Mat(this->config.map_res, this->config.map_res, CV_8UC1, cfg_space.data());
		cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
		cv::resize(image, image, cv::Size(800, 800), 0, 0, cv::INTER_NEAREST);

		auto compressed = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toCompressedImageMsg();
		compressed->header.stamp = this->now();
		compressed->header.frame_id = "map";
		compressed->format = "jpeg";
		debug_pub->publish(*compressed);
	}

private:
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_map_sub;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr expanded_map_pub;
	rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub;

	nav_msgs::msg::MapMetaData map;
	float _max_range = 0.65;
	float _no_go_percent = 0.70;
	int _no_go_range = 0;
	std::vector<Circle> _circles;

private:
    ExpandifyConfig config;
};

int main(int, char **)
{
	rclcpp::init(0, NULL);
	auto node = std::make_shared<Expandify>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}