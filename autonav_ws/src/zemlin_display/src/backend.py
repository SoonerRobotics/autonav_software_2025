#!/usr/bin/env python3

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from autonav_msgs.msg import MotorFeedback, GPSFeedback, MotorInput, DeviceState as DeviceStateMsg, SystemState as SystemStateMsg, SwerveAbsoluteFeedback, ConfigurationUpdate, Position
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import rclpy

from flask import Flask, Response, jsonify
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import threading
from threading import Lock
import logging
import time
import cv2
import numpy as np
import json


class Limiter:
    def __init__(self) -> None:
        self.limits = {}
        self.nextAllowance = {}

    # Sets a limit for how many times per second a key can be used
    def setLimit(self, key, limit):
        self.limits[key] = limit
        self.nextAllowance[key] = 0

    # If it can be used, returns true and decrements the remaining uses
    def use(self, key):
        if key not in self.limits:
            return True

        nextUsageAfter = self.nextAllowance[key]
        if nextUsageAfter == 0:
            self.nextAllowance[key] = time.time() + (1.0 / self.limits[key])
            return True

        if time.time() >= nextUsageAfter:
            self.nextAllowance[key] = time.time() + (1.0 / self.limits[key])
            return True

        return False

class DisplayBackendConfig:
    def __init__(self):
        self.host = "0.0.0.0"
        self.port = 4029
        self.camera_fps = 24

class DisplayBackend(Node):
    def __init__(self):
        super().__init__("autonav_sd_display")
        self.config = DisplayBackendConfig() # Set the default config

        self.camera_last_frame_front = None
        self.camera_last_frame_back = None
        self.camera_last_frame_left = None
        self.camera_last_frame_right = None
        self.filtered_last_frame = None
        self.expanded_last_frame = None
        self.back_cam = None
        self.filtered_image = None

        self.limiter = Limiter()
        self.limiter.setLimit("motor_input", 2)
        self.limiter.setLimit("gps_feedback", 2)
        self.limiter.setLimit("motor_feedback", 2)
        self.limiter.setLimit("position", 2)
        self.limiter.setLimit("absolute", 8)

    def apply_config(self, config: dict):
        self.log(f"Config updated from {self.config} to {config}")
        # This is called BEFORE on_config_update
        # The point of this is to convert from the generic json dict to our config object
        self.config.host = str(config["host"])
        self.config.port = int(config["port"])
        self.config.camera_fps = int(config["camera_fps"])

    def init(self):
        self.log("Initialized")
        self.init_flask_server()
        self.set_device_state(DeviceState.READY)


        # Subscribers
        self.camera_sub_front = self.create_subscription(
            CompressedImage, "/autonav/camera/front", self.camera_callback_front, 10
        )
        self.camera_sub_back = self.create_subscription(
            CompressedImage, "/autonav/camera/back", self.camera_callback_back, 10
        )
        self.camera_sub_left = self.create_subscription(
            CompressedImage, "/autonav/camera/left", self.camera_callback_left, 10
        )
        self.camera_sub_right = self.create_subscription(
            CompressedImage, "/autonav/camera/right", self.camera_callback_right, 10
        )
        self.filered_sub = self.create_subscription(
            CompressedImage, "/autonav/vision/combined/filtered", self.filtered_callback, 10
        )
        # self.expanded_sub = self.create_subscription(
        #     CompressedImage, "/autonav/path_debug_image", self.expanded_callback, 10
        # )

        self.motor_feedback_sub = self.create_subscription(
            MotorFeedback, "/autonav/motor_feedback", self.motor_feedback_callback, 10
        )
        # self.gps_feedback_sub = self.create_subscription(
        #     GPSFeedback, "/autonav/gps", self.gps_feedback, 10
        # )
        self.motor_input_sub = self.create_subscription(
            MotorInput, "/autonav/motor_input", self.motor_input_callback, 10
        )
        self.position_sub = self.create_subscription(
            Position, "/autonav/position", self.position_callback, 10
        )
        # self.device_state_sub = self.create_subscription(
        #     DeviceStateMsg, "/autonav/device_state", self._device_state_callback, 10
        # )
        self.absolute_encoder_sub = self.create_subscription(
            SwerveAbsoluteFeedback, "/autonav/swerve/absolute", self.absolute_encoder_callback, 10
        )        
        # self.config_web_sub = self.create_subscription(
        #     ConfigurationUpdate, "/autonav/shared/config/updates", self.on_web_config_updated, 10
        # )

        # Publishers
        self.load_preset_pub = self.create_publisher(
            String, "/autonav/presets/load", 10
        )
        self.save_preset_pub = self.create_publisher(
            String, "/autonav/presets/save", 10
        )


    def camera_callback_front(self, msg: CompressedImage):
        self.camera_last_frame_front = msg
        
    def camera_callback_back(self, msg: CompressedImage):
        self.camera_last_frame_back = msg
        
    def camera_callback_left(self, msg: CompressedImage):
        self.camera_last_frame_left = msg
        
    def camera_callback_right(self, msg: CompressedImage):
        self.camera_last_frame_right = msg

    def filtered_callback(self, msg):
        self.filtered_last_frame = msg

    def expanded_callback(self, msg):
        self.expanded_last_frame = msg

    def emit(self, type: str, msg: dict):
        packet = {
            "type": type,
            "msg": msg
        }
        self.socketio.emit("message", json.dumps(packet))

    def on_web_config_updated(self, msg: ConfigurationUpdate):
        self.emit("configs", {
            "configs": msg.configs
        })

    def _device_state_callback(self, msg: DeviceState):
        self.emit("device_state", {
            "device": msg.device,
            "state": msg.state
        })

    def on_mobility_updated(self, old, new):
        self.emit("system_state", {
            "state": self.get_system_state(),
            "mobility": new
        })   
    
    def on_system_state_updated(self, old, new):
        self.emit("system_state", {
            "state": new,
            "mobility": self.is_mobility()
        })

    def motor_feedback_callback(self, msg: MotorFeedback):
        if not self.limiter.use("motor_feedback"):
            return

        self.emit("motor_feedback", {
            "delta_x": msg.delta_x,
            "delta_y": msg.delta_y,
            "delta_theta": msg.delta_theta
        })

    def absolute_encoder_callback(self, msg: SwerveAbsoluteFeedback):
        if not self.limiter.use("absolute"):
            return

        self.emit("absolute_encoder", {
            "position_fl": msg.position_fl,
            "position_fr": msg.position_fr,
            "position_bl": msg.position_bl,
            "position_br": msg.position_br
        })

    def position_callback(self, msg: Position):
        if not self.limiter.use("position"):
            return

        self.emit("position", {
            "x": msg.x,
            "y": msg.y,
            "theta": msg.theta
        })

    def motor_input_callback(self, msg: MotorInput):
        if not self.limiter.use("motor_input"):
            return

        self.emit("motor_input", {
            "angular_velocity": msg.angular_velocity,
            "sideways_velocity": msg.sideways_velocity,
            "forward_velocity": msg.forward_velocity
        })

    def gps_feedback(self, msg: GPSFeedback):
        if not self.limiter.use("gps_feedback"):
            return

        self.emit("gps_feedback", {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "gps_fix": msg.gps_fix,
            # "is_locked": msg.is_locked,
            "num_satellites": msg.num_satellites
        })

    def init_flask_server(self):
        """Initialize Flask server in a separate thread."""
        app = Flask(__name__)        
        socketio = SocketIO(app, cors_allowed_origins="*")
        config = self.config
        self.socketio = socketio

        # set log level to error
        flasklog = logging.getLogger("werkzeug")
        flasklog.setLevel(logging.ERROR)

        @app.route("/")
        def get_time():
            """Returns the current time in milliseconds."""
            millis = int(time.time() * 1000)
            return jsonify({"current_time_ms": millis})
        
        # Left camera stream that updates based on the camera fps configuration
        @app.route("/camera_front")
        def front_cam():
            def generate():
                while True:
                    if self.camera_last_frame_front is not None:
                        frame = cv2.imdecode(np.frombuffer(self.camera_last_frame_front.data, np.uint8), cv2.IMREAD_COLOR)
                        _, jpeg = cv2.imencode(".jpg", frame)
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                        )
                    time.sleep(1 / config.camera_fps)
            return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
        
        # Back camera stream that updates based on the camera fps configuration
        @app.route("/camera_back")
        def back_cam():
            def generate():
                while True:
                    if self.camera_last_frame_back is not None:
                        frame = cv2.imdecode(np.frombuffer(self.camera_last_frame_back.data, np.uint8), cv2.IMREAD_COLOR)
                        _, jpeg = cv2.imencode(".jpg", frame)
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                        )
                    time.sleep(1 / config.camera_fps)
            return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
        
        # Left camera stream that updates based on the camera fps configuration
        @app.route("/camera_left")
        def left_cam():
            def generate():
                while True:
                    if self.camera_last_frame_left is not None:
                        frame = cv2.imdecode(np.frombuffer(self.camera_last_frame_left.data, np.uint8), cv2.IMREAD_COLOR)
                        _, jpeg = cv2.imencode(".jpg", frame)
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                        )
                    time.sleep(1 / config.camera_fps)
            return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
        
        # Left camera stream that updates based on the camera fps configuration
        @app.route("/camera_right")
        def right_cam():
            def generate():
                while True:
                    if self.camera_last_frame_right is not None:
                        frame = cv2.imdecode(np.frombuffer(self.camera_last_frame_right.data, np.uint8), cv2.IMREAD_COLOR)
                        _, jpeg = cv2.imencode(".jpg", frame)
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                        )
                    time.sleep(1 / config.camera_fps)
            return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
        
        # Right camera stream that updates based on the camera fps configuration
        @app.route("/filtered")
        def filtered_cam():
            def generate():
                while True:
                    if self.filtered_last_frame is not None:
                        frame = cv2.imdecode(np.frombuffer(self.filtered_last_frame.data, np.uint8), cv2.IMREAD_COLOR)
                        _, jpeg = cv2.imencode(".jpg", frame)
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                        )
                    time.sleep(1 / config.camera_fps)
            return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
        
        # Front camera stream that updates based on the camera fps configuration
        @app.route("/expanded")
        def expanded_cam():
            def generate():
                while True:
                    if self.expanded_last_frame is not None:
                        frame = cv2.imdecode(np.frombuffer(self.expanded_last_frame.data, np.uint8), cv2.IMREAD_COLOR)
                        _, jpeg = cv2.imencode(".jpg", frame)
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                        )
                    time.sleep(1 / config.camera_fps)
            return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
        
        @app.route("/filtered_image")
        def filtered_image():
            def generate():
                while True:
                    if self.filtered_image is not None:
                        frame = cv2.imdecode(np.frombuffer(self.filtered_image.data, np.uint8), cv2.IMREAD_COLOR)
                        _, jpeg = cv2.imencode(".jpg", frame)
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                        )
                    time.sleep(1 / config.camera_fps)
            return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
        
        @socketio.on("load_preset")
        def handle_load_preset(preset_id):
            """Handle loading a preset."""
            if preset_id:
                self.load_preset_pub.publish(String(data=preset_id))
                self.log(f"Loading preset {preset_id}", LogLevel.INFO)
            else:
                self.log("No preset ID provided", LogLevel.ERROR)

        @socketio.on("save_preset")
        def handle_save_preset(preset_id):
            """Handle saving a preset."""
            if preset_id:
                self.save_preset_pub.publish(String(data=preset_id))
                self.log(f"Saving preset {preset_id}", LogLevel.INFO)
            else:
                self.log("No preset ID provided", LogLevel.ERROR)

        @socketio.on("set_system_state")
        def handle_set_system_state(state):
            """Handle setting the system state."""
            if state:
                self.set_system_state(int(state))
                self.log(f"Setting system state to {state}", LogLevel.INFO)

                self.socketio.emit("system_state", json.dumps({
                    "state": state,
                    "mobility": self.is_mobility()
                }))
            else:
                self.log("No state provided", LogLevel.ERROR)

        @socketio.on("set_mobility")
        def handle_set_mobility(mobility):
            """Handle setting the mobility state."""
            if mobility is not None:
                self.set_mobility(bool(mobility))
                self.log(f"Setting mobility to {mobility}", LogLevel.INFO)

                self.socketio.emit("system_state", json.dumps({
                    "state": self.get_system_state(),
                    "mobility": bool(mobility)
                }))
            else:
                self.log("No mobility state provided", LogLevel.ERROR)

        @socketio.on("connect")
        def handle_connect():
            """Handle a new client connection."""
            self.log("Client connected", LogLevel.INFO)
            
            # Send the current system state
            system_state = {
                "state": self.get_system_state(),
                "mobility": self.is_mobility()
            }
            self.emit("system_state", system_state)

            # Loop through the device_states map and send each device state
            for device, state in self.device_states.items():
                device_state = {
                    "device": device,
                    "state": state
                }
                self.emit("device_state", device_state)

            # Send current configs
            self.emit("configs", {
                "configs": self.other_cfgs
            })

        @socketio.on("disconnect")
        def handle_disconnect():
            """Handle a client disconnect."""
            pass
        
        self.log("Flask server initialized", LogLevel.INFO)

        # Run Flask server in a separate thread to avoid blocking the ROS node.
        thread = threading.Thread(
            target=app.run,
            kwargs={
                "host": config.host,
                "port": config.port,
                "debug": False,
                "use_reloader": False,
                "threaded": True,
            },
        )
        thread.daemon = True  # Ensure the thread exits with the main program.
        thread.start()


def main():
    rclpy.init()
    example = DisplayBackend()
    rclpy.spin(example)  # Keep the ROS node running.
    rclpy.shutdown()


if __name__ == "__main__":
    main()
