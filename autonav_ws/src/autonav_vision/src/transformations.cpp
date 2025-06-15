#include <string>

#include "autonav_shared/node.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

using namespace std;

struct ImageTransformerConfig {
    // HSV
    int lower_hue;
    int lower_sat;
    int lower_val;
    int upper_hue;
    int upper_sat;
    int upper_val;

    // Blur
    int blur_weight;
    int blur_iterations;

    // Perspective transform
    vector<int> src_top_left;
    vector<int> src_top_right;
    vector<int> src_bottom_left;
    vector<int> src_bottom_right;

    vector<int> dest_top_left;
    vector<int> dest_top_right;
    vector<int> dest_bottom_left;
    vector<int> dest_bottom_right;

    // TODO FIXME re-add this
    // Region of disinterest
    // Order: bottom-left,
    // int blank_robot = {
    //     {40, IMAGE_HEIGHT - 5},
    //     {IMAGE_WIDTH - 1, IMAGE_HEIGHT - 5},
    //     {IMAGE_WIDTH - 1, IMAGE_HEIGHT - 1},
    //     {0, IMAGE_HEIGHT - 1}
    // };
    // TODO FIXME re-add this

    // Disabling
    bool disable_blur;
    bool disable_hsv;
    bool disable_region_of_disinterest;
    bool disable_perspective_transform;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(ImageTransformerConfig, lower_hue, lower_sat, lower_val, 
        upper_hue, upper_sat, upper_val, 
        blur_weight, blur_iterations, 
        src_top_left, src_top_right, src_bottom_left, src_bottom_right, 
        dest_top_left, dest_top_right, dest_bottom_left, dest_bottom_right, 
        disable_blur, disable_hsv, disable_region_of_disinterest, disable_perspective_transform);
};

class ImageTransformer : public AutoNav::Node {
private:
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscriber;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr camera_filtered_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr camera_debug_publisher;

    // vars
    string dir;
    ImageTransformerConfig m_config;

public:
    ImageTransformer(string dir) : AutoNav::Node("autonav_vision_transformer_" + dir) {
        auto config = ImageTransformerConfig();
        config.lower_hue = 0;
        config.lower_sat = 0;
        config.lower_val = 0;
        config.upper_hue = 255;
        config.upper_sat = 95;
        config.upper_val = 210;

        config.blur_weight = 5;
        config.blur_iterations = 3;
        
        config.src_top_left = {240, 80};
        config.src_top_right = {380, 80};
        config.src_bottom_left = {0, 480};
        config.src_bottom_right = {640, 480};
        
        config.dest_top_left = {240, 0};
        config.dest_top_right = {380, 0};
        config.dest_bottom_left = {240, 480};
        config.dest_bottom_right = {400, 480};
        config.disable_blur = false;
        config.disable_hsv = false;
        config.disable_region_of_disinterest = false;
        config.disable_perspective_transform = false;

        this->m_config = config;
        this->dir = dir;
    }
    ~ImageTransformer() = default;

    //TODO FIXME
    // void apply_config(config: ):
    //     // HSV
    //     this->config.lower_hue = config["lower_hue"]
    //     this->config.lower_sat = config["lower_sat"]
    //     this->config.lower_val = config["lower_val"]
    //     this->config.upper_hue = config["upper_hue"]
    //     this->config.upper_sat = config["upper_sat"]
    //     this->config.upper_val = config["upper_val"]

    //     // Blur
    //     this->config.blur_weight = config["blur_weight"]
    //     this->config.blur_iterations = config["blur_iterations"]

    //     // Perspective transform
    //     this->config.src_top_left = config["src_top_left"]
    //     this->config.src_top_right = config["src_top_right"]
    //     this->config.src_bottom_left = config["src_bottom_left"]
    //     this->config.src_bottom_right = config["src_bottom_right"]

    //     this->config.dest_top_left = config["dest_top_left"]
    //     this->config.dest_top_right = config["dest_top_right"]
    //     this->config.dest_bottom_left = config["dest_bottom_left"]
    //     this->config.dest_bottom_right = config["dest_bottom_right"]

    //     // Region of disinterest
    //     // Order: top-left, top-right, bottom-right, bottom-left
    //     this->config.blank_robot = config["blank_robot"]

    //     // Disabling
    //     this->config.disable_blur = config["disable_blur"]
    //     this->config.disable_hsv = config["disable_hsv"]
    //     this->config.disable_region_of_disinterest = config["disable_region_of_disinterest"]
    //     this->config.disable_perspective_transform = config["disable_perspective_transform"]

    void init() override {
        this->set_device_state(AutoNav::DeviceState::WARMING);
        
        // subscribers
        camera_subscriber = create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/camera/" + this->dir, 1, std::bind(&ImageTransformer::onImageReceived, this, std::placeholders::_1));
        
        // publishers
        camera_filtered_publisher = create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/vision/filtered/" + this->dir, 1);
        camera_debug_publisher = create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/vision/debug/" + this->dir, 1);
        
        this->set_device_state(AutoNav::DeviceState::READY);
    }

    // Blur
    void apply_blur(cv::Mat img) {
        if (this->m_config.disable_blur) {
            return;
        }
        
        for (int i = 0; i < this->m_config.blur_iterations; i++) {
            cv::blur(img, img, {this->m_config.blur_weight, this->m_config.blur_weight});
        }
    }

    // Threshold
    void apply_hsv(cv::Mat img, cv::Mat mask) {
        if (this->m_config.disable_hsv) {
            return;
        }

        cv::cvtColor(img, img, cv::COLOR_BGR2HSV);

        vector<int> lower = {this->m_config.lower_hue, this->m_config.lower_sat, this->m_config.lower_val};
        vector<int> upper = {this->m_config.upper_hue, this->m_config.upper_sat, this->m_config.upper_val};

        cv::inRange(img, lower, upper, mask);

        // invert the mask, because the mask filters in the ground and we want to drive on the ground
        // this is equivalent to what 'return 255 - mask' did in old python code
        cv::bitwise_not(img, mask);
    }

    // Blank out the robot in the middle of the image
    void apply_region_of_disinterest(cv::Mat img) {
        if (this->m_config.disable_region_of_disinterest) {
            return;
        }

        //TODO FIXME actually make this configurable and like, good
        // mask = np.ones_like(img) * 255
        // cv2.fillPoly(mask, [np.array(this->config.blank_robot)], (0, 0, 0))
        
        // return cv2.bitwise_and(img, mask)

        cv::Rect bottom;
        bottom.x = 0;
        bottom.y = img.rows-150;
        bottom.width = img.cols;
        bottom.height = 150;
        //TODO FIXME replace this line
        cv::rectangle(img, bottom, {0, 0, 0}, cv::FILLED);
    }

    void apply_perspective_transform(cv::Mat img, cv::Mat warped, bool debug=false) {
        if (this->m_config.disable_perspective_transform) {
            return;
        }
        
        //TODO FIXME we could probably stand to make these member variables and then just reinitialize them or something?
        // OR BETTER IDEA: only recalculate the matrix whenever the config changes and just store it in a member variable. bam.
        vector<vector<int>> src_pts = {
            this->m_config.src_top_left,
            this->m_config.src_top_right,
            this->m_config.src_bottom_left,
            this->m_config.src_bottom_right
        };

        vector<vector<int>> dest_pts = {
            this->m_config.dest_top_left,
            this->m_config.dest_top_right,
            this->m_config.dest_bottom_left,
            this->m_config.dest_bottom_right
        };

        auto matrix = cv::getPerspectiveTransform(src_pts, dest_pts);

        if (debug) {
            // alpha channel (0, 0, 0, 1) is important for combining images later
            cv::warpPerspective(img, warped, matrix, {img.cols, img.rows}, cv::INTER_LINEAR, cv::BORDER_CONSTANT, {0, 0, 0, 1}); //TODO FIXME this line is probably wrong, end should be a scalar?
        } else {
            cv::warpPerspective(img, warped, matrix, {img.cols, img.rows}, cv::INTER_LINEAR, cv::BORDER_CONSTANT, {0, 0, 0}); //TODO FIXME this line is probably wrong, end should be a scalar?
        }
    }

    void onImageReceived(const sensor_msgs::msg::CompressedImage msg) {
        if (this->get_device_state() != AutoNav::DeviceState::OPERATING) {
            this->set_device_state(AutoNav::DeviceState::OPERATING);
        }

        // Decompress image (this will be the debug image)
        auto image_ros_msg = cv_bridge::toCvCopy(msg);
        auto image = image_ros_msg->image;

        // make a copy for the filtered view
        cv::Mat filtered_image = image.clone();

        // apply all operations on the cv::Mat that will be the filtered output
        this->apply_blur(filtered_image);
        this->apply_hsv(filtered_image, filtered_image);
        this->apply_region_of_disinterest(filtered_image);
        this->apply_perspective_transform(filtered_image, filtered_image); //FIXME???

        // debug image gets disinterest and warpPerspective
        cv::cvtColor(image, image, cv::COLOR_BGR2BGRA); // transparency so it combines nicely later on
        this->apply_region_of_disinterest(image);
        this->apply_perspective_transform(image, image, true);

        // rotate the image depending on which direction it's facing
        // this step is done last because we don't want to rotate it prior to filtering,
        // so that any side of the robot can be the front.
        // we rotate it now solely for running feelers on the combined image later
        if (this->dir == "left") {
            cv::rotate(filtered_image, filtered_image, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE);
        } else if (this->dir == "right") {
            cv::rotate(filtered_image, filtered_image, cv::ROTATE_90_CLOCKWISE);
            cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
        } else if (this->dir == "back") {
            cv::rotate(filtered_image, filtered_image, cv::ROTATE_180);
            cv::rotate(image, image, cv::ROTATE_180);
        }

        // publish filtered image
        sensor_msgs::msg::CompressedImage::SharedPtr filtered_msg;
        std_msgs::msg::Header hdr;
        filtered_msg = cv_bridge::CvImage(hdr, "bgr8", filtered_image).toCompressedImageMsg();

        this->camera_filtered_publisher->publish(*filtered_msg);

        // publish debug image
        this->camera_debug_publisher->publish(*(image_ros_msg->toCompressedImageMsg()));
    }
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    // nodes = []
    // for direction in ["left", "right", "front", "back"]:
    //     nodes.append(ImageTransformer(direction))    
    // for node in nodes:
    // executor.add_node(node)
    
    // executor.add_node(ImageTransformer("left"));
    // executor.add_node(ImageTransformer("right"));
    // executor.add_node(ImageTransformer("back"));

    std::shared_ptr<ImageTransformer> front_node = std::make_shared<ImageTransformer>("front");
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(front_node);
    
    executor.spin();
    executor.remove_node(front_node);
    rclcpp::shutdown();

    return 0;
}