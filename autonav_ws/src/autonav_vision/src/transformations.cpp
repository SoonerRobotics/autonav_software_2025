#include <string>

#include "autonav_shared/node.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

using namespace std;

// TODO FIXME we don't want to rely on these
const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;

struct ImageTransformerConfig {
    // HSV
    int lower_hue = 0;
    int lower_sat = 0;
    int lower_val = 0;
    int upper_hue = 255;
    int upper_sat = 95;
    int upper_val = 210;

    // Blur
    int blur_weight = 5;
    int blur_iterations = 3;

    // Perspective transform
    cv::Point src_top_left = (240, 80);
    cv::Point src_top_right = (380, 80);
    cv::Point src_bottom_left = (0, 480);
    cv::Point src_bottom_right = (640, 480);

    cv::Point dest_top_left = (240, 0);
    cv::Point dest_top_right = (380, 0);
    cv::Point dest_bottom_left = (240, 480);
    cv::Point dest_bottom_right = (400, 480);

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
    bool disable_blur = false;
    bool disable_hsv = false;
    bool disable_region_of_disinterest = false;
    bool disable_perspective_transform = false;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(); //TODO FIXME
};

class ImageTransformer : public AutoNav::Node {
private:
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscriber;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr camera_filtered_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr camera_debug_publisher;

    // vars
    string m_config;

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
        
        config.src_top_left = (240, 80);
        config.src_top_right = (380, 80);
        config.src_bottom_left = (0, 480);
        config.src_bottom_right = (640, 480);
        
        config.dest_top_left = (240, 0);
        config.dest_top_right = (380, 0);
        config.dest_bottom_left = (240, 480);
        config.dest_bottom_right = (400, 480);
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
    //     this.config.lower_hue = config["lower_hue"]
    //     this.config.lower_sat = config["lower_sat"]
    //     this.config.lower_val = config["lower_val"]
    //     this.config.upper_hue = config["upper_hue"]
    //     this.config.upper_sat = config["upper_sat"]
    //     this.config.upper_val = config["upper_val"]

    //     // Blur
    //     this.config.blur_weight = config["blur_weight"]
    //     this.config.blur_iterations = config["blur_iterations"]

    //     // Perspective transform
    //     this.config.src_top_left = config["src_top_left"]
    //     this.config.src_top_right = config["src_top_right"]
    //     this.config.src_bottom_left = config["src_bottom_left"]
    //     this.config.src_bottom_right = config["src_bottom_right"]

    //     this.config.dest_top_left = config["dest_top_left"]
    //     this.config.dest_top_right = config["dest_top_right"]
    //     this.config.dest_bottom_left = config["dest_bottom_left"]
    //     this.config.dest_bottom_right = config["dest_bottom_right"]

    //     // Region of disinterest
    //     // Order: top-left, top-right, bottom-right, bottom-left
    //     this.config.blank_robot = config["blank_robot"]

    //     // Disabling
    //     this.config.disable_blur = config["disable_blur"]
    //     this.config.disable_hsv = config["disable_hsv"]
    //     this.config.disable_region_of_disinterest = config["disable_region_of_disinterest"]
    //     this.config.disable_perspective_transform = config["disable_perspective_transform"]

    void init() override {
        this->set_device_state(AutoNav::DeviceState::WARMING);
        
        // subscribers
        camera_subscriber = create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/camera/" + this->dir, 1, std::bind(&ImageTransformer::onImageReceived, this, std::placeholders::_1));
        
        // publishers
        camera_filtered_publisher = create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/vision/filtered/{this.dir}", 1);
        camera_debug_publisher = create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/vision/debug/{this.dir}", 1);
        
        this->set_device_state(AutoNav::DeviceState::READY);
    }

    // Blur
    void apply_blur(img) {
        if (this.config.disable_blur) {
            return img;
        }
        
        for (int i = 0; i < this.config.blur_iterations; i++) {
            img = cv::blur(img, (this.config.blur_weight, this.config.blur_weight));
        }
        
        return img;
    }

    // Threshold
    void apply_hsv(img) {
        if (this.config.disable_hsv) {
            return img;
        }

        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV);
        lower = (this.config.lower_hue, this.config.lower_sat, this.config.lower_val);
        upper = (this.config.upper_hue, this.config.upper_sat, this.config.upper_val);
        mask = cv2.inRange(img, lower, upper);

        return 255 - mask;
    }

    // Blank out the robot in the middle of the image
    void apply_region_of_disinterest(img) {
        if (this.config.disable_region_of_disinterest) {
            return img;
        }

        // mask = np.ones_like(img) * 255
        // cv2.fillPoly(mask, [np.array(this.config.blank_robot)], (0, 0, 0))
        
        // return cv2.bitwise_and(img, mask)

        // return cv2.rectangle(img, (0, IMAGE_HEIGHT-80), (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0))
        return cv2.rectangle(img, (0, IMAGE_HEIGHT-150), (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0), cv2.FILLED);
    }

    void apply_perspective_transform(img, debug=False) {
        if (this.config.disable_perspective_transform) {
            return img;
        }
        
        src_pts = np.float32([
            this.config.src_top_left,
            this.config.src_top_right,
            this.config.src_bottom_left,
            this.config.src_bottom_right
        ]);

        dest_pts = np.float32([
            this.config.dest_top_left,
            this.config.dest_top_right,
            this.config.dest_bottom_left,
            this.config.dest_bottom_right
        ]);

        matrix = cv2.getPerspectiveTransform(src_pts, dest_pts);

        if (debug) {
            // alpha channel (0, 0, 0, 1) is important for combining images later
            flattened = cv2.warpPerspective(img, matrix, (640, 480), borderValue=(0, 0, 0, 1));
        } else {
            flattened = cv2.warpPerspective(img, matrix, (640, 480), borderValue=(0, 0, 0));
        }

        return flattened;
    }

    void onImageReceived(msg) {
        if (this.get_device_state() != DeviceState.OPERATING) {
            this.set_device_state(DeviceState.OPERATING);
        }

        // Decompress image
        image = bridge.compressed_imgmsg_to_cv2(msg);

        // separate "filtered_image" variable because we have to do some filtering/drawing on the debug image too
        filtered_image = this.apply_blur(image);
        filtered_image = this.apply_hsv(filtered_image);
        filtered_image = this.apply_region_of_disinterest(filtered_image);
        filtered_image = this.apply_perspective_transform(filtered_image);

        // debug image gets disinterest and warpPerspective
        image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA); // transparency so it combines nicely later on
        image = this.apply_region_of_disinterest(image);
        image = this.apply_perspective_transform(image, True);

        // rotate the image depending on which direction it's facing
        // this step is done last because we don't want to rotate it prior to filtering,
        // so that any side of the robot can be the front.
        // we rotate it now solely for running feelers on the combined image later
        if (this.dir == "left") {
            filtered_image = cv2.rotate(filtered_image, cv2.ROTATE_90_COUNTERCLOCKWISE);
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE);
        } else if (this.dir == "right") {
            filtered_image = cv2.rotate(filtered_image, cv2.ROTATE_90_CLOCKWISE);
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE);
        } else if (this.dir == "back") {
            filtered_image = cv2.rotate(filtered_image, cv2.ROTATE_180);
            image = cv2.rotate(image, cv2.ROTATE_180);
        }

        // publish filtered image
        this.camera_filtered_publisher->publish(bridge.cv2_to_compressed_imgmsg(filtered_image));

        // publish debug image
        this.camera_debug_publisher->publish(bridge.cv2_to_compressed_imgmsg(image));
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
    executor.remove_node(front);
    rclcpp::shutdown();

    return 0;
}