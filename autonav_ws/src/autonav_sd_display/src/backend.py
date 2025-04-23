#!/usr/bin/env python3

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from autonav_msgs.msg import MotorFeedback, GPSFeedback, MotorInput
from sensor_msgs.msg import CompressedImage
import rclpy

from flask import Flask, Response, jsonify
from flask_socketio import SocketIO, emit
import threading
from threading import Lock
import time
import cv2
import numpy as np
import json


class DisplayBackendConfig:
    def __init__(self):
        self.host = "0.0.0.0"
        self.port = 4029
        self.camera_fps = 24


class DisplayBackend(Node):
    def __init__(self):
        super().__init__("autonav_sd_display")
        self.write_config(DisplayBackendConfig())

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
            MotorInput, "/autonav/motor_input", self.motor_input_callback, 10
        )

        self.left_cam = None
        self.right_cam = None
        self.front_cam = None
        self.back_cam = None

        self.init_flask_server()

    def init(self):
        self.log("Initialized")
        self.set_device_state(DeviceState.READY)

    def camera_left_callback(self, msg):
        self.left_cam = msg

    def camera_right_callback(self, msg):
        self.right_cam = msg

    def camera_front_callback(self, msg):
        self.front_cam = msg

    def camera_back_callback(self, msg):
        self.back_cam = msg

    def motor_feedback_callback(self, msg: MotorFeedback):
        self.socketio.emit("motor_feedback", json.dumps({
            "delta_x": msg.delta_x,
            "delta_y": msg.delta_y,
            "delta_theta": msg.delta_theta
        }))

    def motor_input_callback(self, msg: MotorInput):
        self.socketio.emit("motor_input", json.dumps({
            "angular_velocity": msg.angular_velocity,
            "forward_velocity": msg.forward_velocity
        }))

    def gps_feedback(self, msg: GPSFeedback):
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
        self.socketio = socketio
        config = self.config

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

        @socketio.on("connect")
        def handle_connect():
            """Handle a new client connection."""
            self.log("Client connected")

        @socketio.on("disconnect")
        def handle_disconnect():
            """Handle a client disconnect."""
            self.log("Client disconnected")

        @socketio.on("test")
        def handle_test(data):
            """Handle a test event."""
            self.log("Received test event: {}".format(data))
            emit("test_response", {"data": "Test response"})

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
