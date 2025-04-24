#!/usr/bin/env python3

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from autonav_msgs.msg import MotorFeedback, GPSFeedback, MotorInput, DeviceState as DeviceStateMsg, SystemState as SystemStateMsg, SwerveAbsoluteFeedback
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import rclpy

from flask import Flask, Response, jsonify
from flask_socketio import SocketIO, emit
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

        self.left_cam = None
        self.right_cam = None
        self.front_cam = None
        self.back_cam = None

        self.limiter = Limiter()
        self.limiter.setLimit("motor_input", 2)
        self.limiter.setLimit("gps_feedback", 2)
        self.limiter.setLimit("motor_feedback", 4)
        self.limiter.setLimit("absolute", 16)

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
        self.camera_left_sub = self.create_subscription(
            CompressedImage, "/autonav/camera/left", self.camera_left_callback, 10
        )
        self.camera_right_sub = self.create_subscription(
            CompressedImage, "/autonav/camera/right", self.camera_right_callback, 10
        )
        self.camera_front_sub = self.create_subscription(
            CompressedImage, "/autonav/camera/front", self.camera_front_callback, 10
        )
        self.camera_back_sub = self.create_subscription(
            CompressedImage, "/autonav/camera/back", self.camera_back_callback, 10
        )
        self.motor_feedback_sub = self.create_subscription(
            MotorFeedback, "/autonav/motor_feedback", self.motor_feedback_callback, 10
        )
        self.gps_feedback_sub = self.create_subscription(
            MotorFeedback, "/autonav/gps", self.gps_feedback, 10
        )
        self.motor_input_sub = self.create_subscription(
            MotorInput, "/autonav/MotorInput", self.motor_input_callback, 10
        )
        self.system_state_sub = self.create_subscription(
            SystemStateMsg, "/autonav/system_state", self._system_state_callback, 10
        )
        self.device_state_sub = self.create_subscription(
            DeviceStateMsg, "/autonav/device_state", self._device_state_callback, 10
        )
        self.absolute_encoder_sub = self.create_subscription(
            SwerveAbsoluteFeedback, "/autonav/swerve/absolute", self.motor_feedback_callback, 10
        )        

        # Publishers
        self.load_preset_pub = self.create_publisher(
            String, "/autonav/presets/load", 10
        )
        self.save_preset_pub = self.create_publisher(
            String, "/autonav/presets/save", 10
        )


    def camera_left_callback(self, msg):
        self.left_cam = msg

    def camera_right_callback(self, msg):
        self.right_cam = msg

    def camera_front_callback(self, msg):
        self.front_cam = msg

    def camera_back_callback(self, msg):
        self.back_cam = msg

    def _system_state_callback(self, msg: SystemState):
        self.socketio.emit("system_state", json.dumps({
            "state": msg.state,
            "mobility": msg.mobility
        }))

    def _device_state_callback(self, msg: DeviceState):
        self.socketio.emit("device_state", json.dumps({
            "device": msg.device,
            "state": msg.state
        }))

    def motor_feedback_callback(self, msg: MotorFeedback):
        if not self.limiter.use("motor_feedback"):
            return

        self.socketio.emit("motor_feedback", json.dumps({
            "delta_x": msg.delta_x,
            "delta_y": msg.delta_y,
            "delta_theta": msg.delta_theta
        }))

    def motor_input_callback(self, msg: MotorInput):
        if not self.limiter.use("motor_input"):
            return

        self.socketio.emit("motor_input", json.dumps({
            "angular_velocity": msg.angular_velocity,
            "forward_velocity": msg.forward_velocity
        }))

    def gps_feedback(self, msg: GPSFeedback):
        if not self.limiter.use("gps_feedback"):
            return

        self.socketio.emit("gps_feedback", json.dumps({
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "gps_fix": msg.gps_fix,
            "is_locked": msg.is_locked,
            "satellites": msg.satellites
        }))

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
        @app.route("/left_cam")
        def left_cam():
            def generate():
                while True:
                    if self.left_cam is not None:
                        frame = cv2.imdecode(np.frombuffer(self.left_cam.data, np.uint8), cv2.IMREAD_COLOR)
                        _, jpeg = cv2.imencode(".jpg", frame)
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                        )
                    time.sleep(1 / config.camera_fps)
            return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
        
        # Right camera stream that updates based on the camera fps configuration
        @app.route("/right_cam")
        def right_cam():
            def generate():
                while True:
                    if self.right_cam is not None:
                        frame = cv2.imdecode(np.frombuffer(self.right_cam.data, np.uint8), cv2.IMREAD_COLOR)
                        _, jpeg = cv2.imencode(".jpg", frame)
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                        )
                    time.sleep(1 / config.camera_fps)
            return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
        
        # Front camera stream that updates based on the camera fps configuration
        @app.route("/front_cam")
        def front_cam():
            def generate():
                while True:
                    if self.front_cam is not None:
                        frame = cv2.imdecode(np.frombuffer(self.front_cam.data, np.uint8), cv2.IMREAD_COLOR)
                        _, jpeg = cv2.imencode(".jpg", frame)
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                        )
                    time.sleep(1 / config.camera_fps)
            return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
        
        # Back camera stream that updates based on the camera fps configuration
        @app.route("/back_cam")
        def back_cam():
            def generate():
                while True:
                    if self.back_cam is not None:
                        frame = cv2.imdecode(np.frombuffer(self.back_cam.data, np.uint8), cv2.IMREAD_COLOR)
                        _, jpeg = cv2.imencode(".jpg", frame)
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                        )
                    time.sleep(1 / config.camera_fps)
            return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

        # return an image of the current time as a stream
        @app.route("/time")
        def time_stream():
            def generate():
                while True:
                    millis = int(time.time() * 1000)
                    img = np.zeros((100, 300, 3), dtype=np.uint8)
                    cv2.putText(img, str(millis), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    _, jpeg = cv2.imencode(".jpg", img)
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

        @socketio.on("connect")
        def handle_connect():
            """Handle a new client connection."""
            # Send the current system state
            system_state = {
                "state": self.get_system_state(),
                "mobility": self.is_mobility()
            }
            self.socketio.emit("system_state", json.dumps(system_state))

            # Loop through the device_states map and send each device state
            for device, state in self.device_states.items():
                device_state = {
                    "device": device,
                    "state": state
                }
                self.socketio.emit("device_state", json.dumps(device_state))

        @socketio.on("disconnect")
        def handle_disconnect():
            """Handle a client disconnect."""
            pass

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
