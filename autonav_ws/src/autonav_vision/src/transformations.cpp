#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <chrono>

#include "autonav_shared/node.hpp"
#include "feeler.cpp"
#include "pid.cpp"
#include "cv_bridge/cv_bridge.hpp"
#include "autonav_msgs/msg/position.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_msgs/msg/ultrasonic.hpp"
#include "autonav_msgs/msg/safety_lights.hpp"
#include "autonav_msgs/msg/audible_feedback.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

g_bridge = CvBridge()
g_mapData = MapMetaData()
g_mapData.width = 200
g_mapData.height = 100
g_mapData.resolution = 0.1
g_mapData.origin = Pose()
g_mapData.origin.position.x = -10.0
g_mapData.origin.position.y = -10.0

MAP_RES = 80

FEELERS = False

struct ImageTransformerConfi {
        self.lower_hue_ground = 0
        self.lower_saturation_ground = 0
        self.lower_value_ground = 0
        self.upper_hue_ground = 255
        self.upper_saturation_ground = 140
        self.upper_value_ground = 200
        self.lower_hue_ramp = 80
        self.lower_saturation_ramp = 45
        self.lower_value_ramp = 40
        self.upper_hue_ramp = 220
        self.upper_saturation_ramp = 255
        self.upper_value_ramp = 255
        self.blur = 5
        self.blur_iterations = 3
        self.region_of_disinterest_offset = 30
        self.override_ramp = False
}

class ImageTransformer(Node):
    def __init__(self, dir="front"):
        super().__init__("autonav_vision_transformer")
        self.config = ImageTransformerConfig()
        self.waypointReached = False
        self.dir = dir

    def init(self):
        self.set_device_state(DeviceState.WARMING)

        self.cameraSubscriber = self.create_subscription(CompressedImage, f"/autonav/camera/{self.dir}", self.onImageReceived, 1)
        self.waypointReachedSubscriber = self.create_subscription(
            WaypointReached, "/autonav/waypoint_reached", self.onWaypointReached, 1
        )
        
        self.rawMapPublisher = self.create_publisher(OccupancyGrid, "/autonav/cfg_space/raw", 1)
        self.filteredImagePublisher = self.create_publisher(CompressedImage, "/autonav/cfg_space/raw/image", 1)
        
        self.camera_filtered_publisher = self.create_publisher(CompressedImage, f"/autonav/vision/filtered/{self.dir}", 1)
        self.camera_debug_publisher = self.create_publisher(CompressedImage, f"/autonav/vision/debug/{self.dir}", 1)


        self.set_device_state(DeviceState.READY)

    def apply_config(self, config):
        self.log("Applying new configuration to ImageTransformer")
        self.config.lower_hue_ground = config["lower_hue_ground"]
        self.config.lower_saturation_ground = config["lower_saturation_ground"]
        self.config.lower_value_ground = config["lower_value_ground"]
        self.config.upper_hue_ground = config["upper_hue_ground"]
        self.config.upper_saturation_ground = config["upper_saturation_ground"]
        self.config.upper_value_ground = config["upper_value_ground"]
        self.config.lower_hue_ramp = config["lower_hue_ramp"]
        self.config.lower_saturation_ramp = config["lower_saturation_ramp"]
        self.config.lower_value_ramp = config["lower_value_ramp"]
        self.config.upper_hue_ramp = config["upper_hue_ramp"]
        self.config.upper_saturation_ramp = config["upper_saturation_ramp"]
        self.config.upper_value_ramp = config["upper_value_ramp"]
        self.config.blur = config["blur"]
        self.config.blur_iterations = config["blur_iterations"]
        self.config.region_of_disinterest_offset = config["region_of_disinterest_offset"]
        self.config.override_ramp = config["override_ramp"]

    def on_system_state_changed(self, old, new):
        if old != SystemState.AUTONOMOUS and new == SystemState.AUTONOMOUS:
            self.waypointReached = False

    def getBlur(self):
        blur = self.config.blur
        blur = max(1, blur)
        return (blur, blur)

    def regionOfDisinterest(self, img, vertices):
        mask = np.ones_like(img) * 255
        cv2.fillPoly(mask, vertices, 0)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def flattenImage(self, img, debug=False):
        top_left = (int)(img.shape[1] * 0.26), (int)(img.shape[0])
        top_right = (int)(img.shape[1] - img.shape[1] * 0.26), (int)(img.shape[0])
        bottom_left = 0, 0
        bottom_right = (int)(img.shape[1]), 0

        src_pts = np.float32([[top_left], [top_right], [bottom_left], [bottom_right]])
        dest_pts = np.float32([ [0, 480], [640, 480] ,[0, 0], [640, 0]])

        matrix = cv2.getPerspectiveTransform(dest_pts, src_pts)

        if FEELERS:
            if debug:
                # alpha channel (0, 0, 0, 1) is important for combining images later
                output = cv2.warpPerspective(img, matrix, (640, 480), borderValue=(0, 0, 0, 1))
            else:
                output = cv2.warpPerspective(img, matrix, (640, 480), borderValue=(0, 0, 0))
        else:
            output = cv2.warpPerspective(img, matrix, (640, 480))
        return output

    def publishOccupancyMap(self, img):
        datamap = cv2.resize(img, dsize=(MAP_RES, MAP_RES), interpolation=cv2.INTER_LINEAR) / 2
        flat = list(datamap.flatten().astype(int))
        msg = OccupancyGrid(info=g_mapData, data=flat)
        self.rawMapPublisher.publish(msg)

    def onWaypointReached(self, msg: WaypointReached):
        self.log(f"Waypoint reached: {msg.tag}")
        self.waypointReached = True

    def onImageReceived(self, image: CompressedImage):
        if self.get_device_state() != DeviceState.OPERATING:
            self.set_device_state(DeviceState.OPERATING)

        self.perf_start("Image Transformation")

        # Decompressify
        cv_image = g_bridge.compressed_imgmsg_to_cv2(image)
        feeler_debug_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2BGRA) # transparency so it combines nicely later on

        # Blur it up
        for _ in range(self.config.blur_iterations):
            cv_image = cv2.blur(cv_image, self.getBlur())

        # Apply filter and return a mask
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower = (
            self.config.lower_hue_ground,
            self.config.lower_saturation_ground,
            self.config.lower_value_ground
        )
        upper = (
            self.config.upper_hue_ground,
            self.config.upper_saturation_ground,
            self.config.upper_value_ground
        )
        mask1 = cv2.inRange(img, lower, upper)

        lower = (
            self.config.lower_hue_ramp,
            self.config.lower_saturation_ramp,
            self.config.lower_value_ramp
        )
        upper = (
            self.config.upper_hue_ramp,
            self.config.upper_saturation_ramp,
            self.config.upper_value_ramp
        )
        mask2 = cv2.inRange(img, lower, upper)
        mask = cv2.add(mask1, mask2) if (self.waypointReached or self.config.override_ramp) else mask1
        mask = 255 - mask

        # Apply region of disinterest and flattening
        height = img.shape[0]
        width = img.shape[1]
        region_of_disinterest_vertices=[
            (0, height),
            ((width / 2) - 200, height / 2 + self.config.region_of_disinterest_offset),
            ((width / 2) + 200, height / 2 + self.config.region_of_disinterest_offset),
            (width, height)
        ]
        mask = self.regionOfDisinterest(mask, np.array([region_of_disinterest_vertices], np.int32))
        mask[mask < 250] = 0

        mask = self.flattenImage(mask, FEELERS)
        preview_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

        if FEELERS:
            feeler_debug_image = self.regionOfDisinterest(feeler_debug_image, np.array([region_of_disinterest_vertices], np.int32))
            feeler_debug_image = self.flattenImage(feeler_debug_image, True)

            if self.dir == "left":
                filtered_image = cv2.rotate(filtered_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
                mask = cv2.rotate(mask, cv2.ROTATE_90_COUNTERCLOCKWISE)

            elif self.dir == "right":
                filtered_image = cv2.rotate(filtered_image, cv2.ROTATE_90_CLOCKWISE)
                mask = cv2.rotate(mask, cv2.ROTATE_90_CLOCKWISE)
                
            elif self.dir == "back":
                filtered_image = cv2.rotate(filtered_image, cv2.ROTATE_180)
                mask = cv2.rotate(mask, cv2.ROTATE_180)

            self.camera_filtered_publisher.publish(g_bridge.cv2_to_compressed_imgmsg(mask))
            self.camera_debug_publisher.publish(g_bridge.cv2_to_compressed_imgmsg(feeler_debug_image))
        else: # not feelers

            cv2.polylines(preview_image, np.array([region_of_disinterest_vertices], np.int32), True, (0, 255, 0), 2)
            preview_msg = g_bridge.cv2_to_compressed_imgmsg(preview_image)
            preview_msg.header = image.header
            preview_msg.format = "jpeg"
            self.filteredImagePublisher.publish(preview_msg)

            # Actually generate the map
            self.publishOccupancyMap(mask)

        self.perf_stop("Image Transformation")


def main():
    rclpy.init()
    rclpy.spin(ImageTransformer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
