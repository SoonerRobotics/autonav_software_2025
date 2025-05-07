#!/usr/bin/env python3

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from autonav_msgs.msg import MotorFeedback, GPSFeedback, MotorInput, DeviceState as DeviceStateMsg, SystemState as SystemStateMsg, SwerveAbsoluteFeedback, ConfigurationUpdate, Position
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import rclpy

import tkinter as tk
from tkinter import ttk
import threading

from PIL import Image, ImageTk
import cv2
import numpy as np


class AppBackend(Node):
    def __init__(self, app):
        super().__init__("zemlin_test_display")
        self.app = app
        self.app.node = self
        self.set_device_state(DeviceState.OPERATING)

        # Setup ROS subscriber
        self.create_subscription(CompressedImage, "/autonav/camera/compressed", self.camera_compressed_callback, 10)
        self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image", self.filtered_callback, 10)
        self.create_subscription(CompressedImage, "/autonav/path_debug_image", self.expanded_callback, 10)

        self.create_subscription(MotorFeedback, "/autonav/motor_feedback", self.motor_feedback_callback, 10)
        self.create_subscription(GPSFeedback, "/autonav/gps", self.gps_feedback, 10)
        self.create_subscription(MotorInput, "/autonav/motor_input", self.motor_input_callback, 10)
        self.create_subscription(Position, "/autonav/position", self.position_callback, 10)
        # self.create_subscription(DeviceStateMsg, "/autonav/device_state", self._device_state_callback, 10)
        # self.create_subscription(SwerveAbsoluteFeedback, "/autonav/swerve/absolute", self.motor_feedback_callback, 10)        
        # self.create_subscription(ConfigurationUpdate, "/autonav/shared/config/updates", self.on_web_config_updated, 10)

    def update_img_from_msg(self, label, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        # resize to fit the label
        label_width = label.winfo_width()
        label_height = label.winfo_height()
        cv_img = cv2.resize(cv_img, (label_width, label_height), interpolation=cv2.INTER_AREA)

        pil_img = Image.fromarray(cv_img)
        img = ImageTk.PhotoImage(image=pil_img)

        def image_update():
            if label:
                label.config(image=img)
                label.image = img

        self.app.after(0, image_update)

    def update_label(self, label, text):
        if label:
            label.config(text=text)

    def camera_compressed_callback(self, msg):
        if self.app.camera_compressed_label:
            self.update_img_from_msg(self.app.camera_compressed_label, msg)

    def filtered_callback(self, msg):
        if self.app.camera_compressed_label:
            self.update_img_from_msg(self.app.camera_filtered_label, msg)

    def expanded_callback(self, msg):
        if self.app.camera_compressed_label:
            self.update_img_from_msg(self.app.camera_expanded_label, msg)

    def motor_feedback_callback(self, msg: MotorFeedback):
        if self.app.motor_feedback_label:
            self.update_label(self.app.motor_feedback_label, f"Motor Feedback: {msg.delta_x}, {msg.delta_y}, {msg.delta_theta}")

    def gps_feedback(self, msg: GPSFeedback):
        if self.app.gps_feedback_label:
            self.update_label(self.app.gps_feedback_label, f"GPS Feedback: {msg.latitude}, {msg.longitude}, {msg.altitude}")

    def motor_input_callback(self, msg: MotorInput):
        if self.app.motor_input_label:
            self.update_label(self.app.motor_input_label, f"Motor Input: {msg.forward_velocity}, {msg.sideways_velocity}, {msg.angular_velocity}")

    def position_callback(self, msg: Position):
        if self.app.position_label:
            self.update_label(self.app.position_label, f"Position: {msg.x}, {msg.y}, {msg.theta}")

    def on_mobility_updated(self, old, new):
        if self.app.device_state_label:
            self.update_label(self.app.device_state_label, f"Mobility Enabled: {new}")

    def on_system_state_updated(self, old, new):
        if self.app.system_state_label:
            self.update_label(self.app.system_state_label, f"System State: {new}")

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Zemlin Display")
        self.geometry("1280x720")

        # Camera labels
        self.camera_compressed_label = None
        self.camera_filtered_label = None
        self.camera_expanded_label = None

        # Dashboard variables
        self.motor_feedback_label = None
        self.gps_feedback_label = None
        self.motor_input_label = None
        self.position_label = None
        self.system_state_label = None

        # Create the Notebook (tab container)
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # Create tabs
        self.dashboard_tab = self.create_dashboard_tab()
        self.visuals_tab = self.create_visuals_tab()
        self.configuration_tab = self.create_configuration_tab()
        self.performance_tab = self.create_performance_tab()
        self.other_tab = self.create_other_tab()

    def on_mobility_updated(self):
        self.node.set_mobility(self.mobility_var.get())

    def on_system_state_updated(self):
        idx = self.system_state_dropdown.current()
        self.node.set_system_state(SystemState(idx))
        

    def create_dashboard_tab(self):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Dashboard")

        # Title label across the top
        title_label = tk.Label(frame, text="Welcome to the Dashboard", font=("Helvetica", 18))
        title_label.grid(row=0, column=0, columnspan=2, pady=20)

        # Labels in a vertical column on the left
        self.motor_feedback_label = tk.Label(frame, text="Motor Feedback: ")
        self.motor_feedback_label.grid(row=1, column=0, sticky="w", padx=20, pady=5)

        self.gps_feedback_label = tk.Label(frame, text="GPS Feedback: ")
        self.gps_feedback_label.grid(row=2, column=0, sticky="w", padx=20, pady=5)

        self.motor_input_label = tk.Label(frame, text="Motor Input: ")
        self.motor_input_label.grid(row=3, column=0, sticky="w", padx=20, pady=5)

        self.position_label = tk.Label(frame, text="Position: ")
        self.position_label.grid(row=4, column=0, sticky="w", padx=20, pady=5)

        self.system_state_label = tk.Label(frame, text="System State: ")
        self.system_state_label.grid(row=5, column=0, sticky="w", padx=20, pady=5)

        self.device_state_label = tk.Label(frame, text="Mobility Enabled: ")
        self.device_state_label.grid(row=6, column=0, sticky="w", padx=20, pady=5)

        # Checkbox for System Mobility
        self.mobility_var = tk.BooleanVar()
        self.mobility_checkbox = tk.Checkbutton(frame, text="System Mobility", variable=self.mobility_var, command=lambda: self.on_mobility_updated())
        self.mobility_checkbox.grid(row=1, column=1, padx=20, pady=5, sticky="w")

        # Dropdown for System State
        self.system_state_var = tk.StringVar()
        self.system_state_dropdown = ttk.Combobox(frame, textvariable=self.system_state_var, state="readonly")
        self.system_state_dropdown['values'] = ["Disabled", "Manual", "Autonomous", "Shutdown"]
        self.system_state_dropdown.current(0)  # Default to "Disabled"
        self.system_state_dropdown.bind("<<ComboboxSelected>>", lambda event: self.on_system_state_updated())
        self.system_state_dropdown.grid(row=2, column=1, padx=20, pady=5, sticky="w")

        return frame

    def create_configuration_tab(self):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Configuration")
        label = tk.Label(frame, text="Settings", font=("Helvetica", 18))
        label.pack(pady=20)
        tk.Label(frame, text="Option A:").pack()
        tk.Entry(frame).pack()
        return frame

    def create_performance_tab(self):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Performance")
        label = tk.Label(frame, text="Performance Metrics", font=("Helvetica", 18))
        label.pack(pady=20)
        return frame

    def create_other_tab(self):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Other")
        label = tk.Label(frame, text="Miscellaneous", font=("Helvetica", 18))
        label.pack(pady=20)
        tk.Checkbutton(frame, text="Enable feature X").pack()
        return frame


    def create_visuals_tab(self):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Vision")

        self.camera_compressed_label = tk.Label(frame, width=200, height=200)
        self.camera_filtered_label = tk.Label(frame, width=200, height=200)
        self.camera_expanded_label = tk.Label(frame, width=200, height=200)

        # Grid layout in a row
        self.camera_compressed_label.grid(row=0, column=0, padx=5, pady=10)
        self.camera_filtered_label.grid(row=0, column=1, padx=5, pady=10)
        self.camera_expanded_label.grid(row=0, column=2, padx=5, pady=10)

        return frame

def main():
    rclpy.init()

    app = App()
    node = AppBackend(app)

    # Spin ROS in background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Run GUI mainloop in main thread
    app.protocol("WM_DELETE_WINDOW", lambda: (app.destroy(), rclpy.shutdown()))
    app.mainloop()


if __name__ == "__main__":
    main()
