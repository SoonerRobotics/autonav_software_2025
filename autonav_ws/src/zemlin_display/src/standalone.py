#!/usr/bin/env python3

from autonav_shared.node import Node
from autonav_shared.types import LogLevel, DeviceState, SystemState
from autonav_msgs.msg import MotorFeedback, GPSFeedback, MotorInput, DeviceState as DeviceStateMsg, PathingDebug, SystemState as SystemStateMsg, SwerveAbsoluteFeedback, SwerveFeedback, Position, Performance
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import rclpy
import math

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
        self.create_subscription(CompressedImage, "/autonav/camera/compressed/front", self.camera_compressed_callback, 10)
        self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image", self.filtered_callback, 10)
        self.create_subscription(CompressedImage, "/autonav/path_debug_image", self.expanded_callback, 10)

        self.create_subscription(MotorFeedback, "/autonav/motor_feedback", self.motor_feedback_callback, 10)
        self.create_subscription(GPSFeedback, "/autonav/gps", self.gps_feedback, 10)
        self.create_subscription(MotorInput, "/autonav/motor_input", self.motor_input_callback, 10)
        self.create_subscription(Position, "/autonav/position", self.position_callback, 10)
        self.create_subscription(PathingDebug, "/autonav/pathing_debug", self.pathing_debug_callback, 10)
        # self.create_subscription(DeviceStateMsg, "/autonav/device_state", self._device_state_callback, 10)
        self.create_subscription(SwerveAbsoluteFeedback, "/autonav/swerve/absolute", self.absolute_callback, 10)        
        self.create_subscription(SwerveFeedback, "/autonav/swerve/feedback", self.swerve_callback, 10)
        self.create_subscription(Performance, "/autonav/shared/performance", self.performance_callback, 10)
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
            ang = ((msg.theta * -1) * (180 / math.pi) + 360) % 360
            self.update_label(self.app.position_label, f"Position: {msg.x}, {msg.y}, {ang}")

    def on_mobility_updated(self, old, new):
        if self.app.device_state_label:
            self.update_label(self.app.device_state_label, f"Mobility Enabled: {new}")

    def on_system_state_updated(self, old, new):
        if self.app.system_state_label:
            self.update_label(self.app.system_state_label, f"System State: {new}")

    def absolute_callback(self, msg: SwerveAbsoluteFeedback):
        if self.app.motor_feedback_label:
            self.update_label(self.app.motor_feedback_label, f"Absolute Feedback: {msg.position_fl}, {msg.position_fr}, {msg.position_bl}, {msg.position_br}")

    def pathing_debug_callback(self, msg: PathingDebug):
        if self.app.pathing_debug_label:
            self.update_label(self.app.pathing_debug_label, f"Desired Heading: {msg.desired_heading}")
        if self.app.pathing_debug_label2:
            self.update_label(self.app.pathing_debug_label2, f"Desired Latitude: {msg.desired_latitude}")
        if self.app.pathing_debug_label3:
            self.update_label(self.app.pathing_debug_label3, f"Desired Longitude: {msg.desired_longitude}")
        if self.app.pathing_debug_label4:
            self.update_label(self.app.pathing_debug_label4, f"Distance to Waypoint: {msg.distance_to_destination}")
        if self.app.pathing_debug_label5:
            waypoints_str = ", ".join([f"({msg.waypoints[i]}, {msg.waypoints[i+1]})" for i in range(0, len(msg.waypoints), 2)])
            self.update_label(self.app.pathing_debug_label5, f"Waypoint List: {waypoints_str}")
        if self.app.pathing_debug_label6:
            self.update_label(self.app.pathing_debug_label6, f"Time Until Use Waypoints: {msg.time_until_use_waypoints}")

    def performance_callback(self, msg: Performance):
        if self.app.performance_label:
            self.app.performances[msg.name] = msg.elapsed
            self.app.update_performance()

    def swerve_callback(self, msg: SwerveFeedback):
        # update last swerve feedbacks
        self.app.last_swerve_feedbacks[msg.module] = msg
        self.app.update_swerve_display()

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Zemlin Display")
        self.geometry("1280x720")

        # Camera labels
        self.camera_compressed_label = None
        self.camera_filtered_label = None
        self.camera_expanded_label = None

        # canvas for swerve modules
        self.swerve_canvas = None

        # Dashboard variables
        self.motor_feedback_label = None
        self.gps_feedback_label = None
        self.motor_input_label = None
        self.position_label = None
        self.system_state_label = None
        self.device_state_label = None
        self.pathing_debug_label = None
        self.pathing_debug_label2 = None
        self.pathing_debug_label3 = None
        self.pathing_debug_label4 = None
        self.pathing_debug_label5 = None
        self.pathing_debug_label6 = None
        self.performances = {}
        self.performance_label = None
        
        # swerve stuff
        self.last_swerve_feedbacks = [None, None, None, None]

        # Create the Notebook (tab container)
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # Create tabs
        self.dashboard_tab = self.create_dashboard_tab()
        self.visuals_tab = self.create_visuals_tab()
        self.swerve_tab = self.create_swerve_tab()
        self.configuration_tab = self.create_configuration_tab()
        self.performance_tab = self.create_performance_tab()
        self.other_tab = self.create_other_tab()

        # default swerve display
        self.update_swerve_display()

    def update_swerve_display(self):
        mod1 = self.last_swerve_feedbacks[0]
        mod2 = self.last_swerve_feedbacks[1]
        mod3 = self.last_swerve_feedbacks[2]
        mod4 = self.last_swerve_feedbacks[3]

        mod1_data = (mod1.desired_x_vel, mod1.desired_y_vel, mod1.desired_angular_vel, mod1.measured_x_vel, mod1.measured_y_vel, mod1.measured_angular_vel) if mod1 else (0, 0, 0, 0, 0, 0)
        mod2_data = (mod2.desired_x_vel, mod2.desired_y_vel, mod2.desired_angular_vel, mod2.measured_x_vel, mod2.measured_y_vel, mod2.measured_angular_vel) if mod2 else (0, 0, 0, 0, 0, 0)
        mod3_data = (mod3.desired_x_vel, mod3.desired_y_vel, mod3.desired_angular_vel, mod3.measured_x_vel, mod3.measured_y_vel, mod3.measured_angular_vel) if mod3 else (0, 0, 0, 0, 0, 0)
        mod4_data = (mod4.desired_x_vel, mod4.desired_y_vel, mod4.desired_angular_vel, mod4.measured_x_vel, mod4.measured_y_vel, mod4.measured_angular_vel) if mod4 else (0, 0, 0, 0, 0, 0)

        # calculate magnitudes and directions
        def calculate_magnitude_and_direction(x, y):
            magnitude = math.sqrt(x**2 + y**2)
            direction = math.degrees(math.atan2(y, x))
            return magnitude, direction
        
        mod1_mag, mod1_dir = calculate_magnitude_and_direction(mod1_data[0], mod1_data[1])
        mod2_mag, mod2_dir = calculate_magnitude_and_direction(mod2_data[0], mod2_data[1])
        mod3_mag, mod3_dir = calculate_magnitude_and_direction(mod3_data[0], mod3_data[1])
        mod4_mag, mod4_dir = calculate_magnitude_and_direction(mod4_data[0], mod4_data[1])

        # update the canvas
        self.swerve_canvas.delete("all")
        self.draw_robot_swerve_layout([
            (mod1_data[2], mod1_mag, mod1_dir, mod1_data[3], mod1_data[4]),  # FL
            (mod2_data[2], mod2_mag, mod2_dir, mod2_data[3], mod2_data[4]),  # FR
            (mod3_data[2], mod3_mag, mod3_dir, mod3_data[3], mod3_data[4]),  # BL
            (mod4_data[2], mod4_mag, mod4_dir, mod4_data[3], mod4_data[4]),  # BR
        ])
        self.swerve_canvas.update()

    def on_mobility_updated(self):
        self.node.set_mobility(self.mobility_var.get())

    def on_system_state_updated(self):
        idx = self.system_state_dropdown.current()
        self.node.set_system_state(SystemState(idx))
        
    def draw_robot_swerve_layout(self, modules):
        canvas = self.swerve_canvas
        canvas.delete("all")

        cx, cy = 200, 200
        half = 100
        module_radius = 20

        canvas.create_rectangle(cx - half, cy - half, cx + half, cy + half, outline="black", width=2)
        offsets = [(-1, -1), (1, -1), (-1, 1), (1, 1)]
        for i, (rotation, true_mag, true_dir, desired_mag, desired_dir) in enumerate(modules):
            dx, dy = offsets[i]
            mx = cx + dx * half
            my = cy + dy * half

            canvas.create_oval(mx - module_radius, my - module_radius, mx + module_radius, my + module_radius, outline="black", width=2)
            arc_extent = 30
            start_angle = -rotation + 90 - arc_extent / 2
            canvas.create_arc(mx - module_radius, my - module_radius,
                            mx + module_radius, my + module_radius,
                            start=start_angle, extent=arc_extent,
                            fill="red", outline="black")

            def draw_arrow(angle_deg, magnitude, color):
                angle_rad = math.radians(-angle_deg + 90)
                length = 20 + magnitude * 40
                x = mx + length * math.cos(angle_rad)
                y = my + length * math.sin(angle_rad)
                canvas.create_line(mx, my, x, y, arrow=tk.LAST, fill=color, width=2)

            draw_arrow(true_dir, true_mag, "red")
            draw_arrow(desired_dir, desired_mag, "blue")

        # canvas.create_line(cx, cy, cx, cy - 40, arrow=tk.LAST, width=2)

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
        
        # horizontal line
        separator = ttk.Separator(frame, orient='horizontal')
        separator.grid(row=7, column=0, columnspan=2, sticky='ew', padx=20, pady=10)
        
        # pathing debug (desired heading, latitude, longitude, distance to current waypoint, waypoint list, and time_until_use_waypoints)
        self.pathing_debug_label = tk.Label(frame, text="Desired Heading: ")
        self.pathing_debug_label.grid(row=8, column=0, sticky="w", padx=20, pady=5)
        self.pathing_debug_label2 = tk.Label(frame, text="Desured Latitude: ")
        self.pathing_debug_label2.grid(row=9, column=0, sticky="w", padx=20, pady=5)
        self.pathing_debug_label3 = tk.Label(frame, text="Desired Longitude: ")
        self.pathing_debug_label3.grid(row=10, column=0, sticky="w", padx=20, pady=5)
        self.pathing_debug_label4 = tk.Label(frame, text="Distance to Waypoint: ")
        self.pathing_debug_label4.grid(row=11, column=0, sticky="w", padx=20, pady=5)
        self.pathing_debug_label5 = tk.Label(frame, text="Waypoint List: ")
        self.pathing_debug_label5.grid(row=12, column=0, sticky="w", padx=20, pady=5)
        self.pathing_debug_label6 = tk.Label(frame, text="Time Until Use Waypoints: ")
        self.pathing_debug_label6.grid(row=13, column=0, sticky="w", padx=20, pady=5)

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

    def create_swerve_tab(self):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Swerve Modules")

        self.swerve_canvas = tk.Canvas(frame, width=400, height=400, bg="white")
        self.swerve_canvas.pack(padx=20, pady=20)

        return frame


    def create_configuration_tab(self):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Configuration")
        label = tk.Label(frame, text="Settings", font=("Helvetica", 18))
        label.pack(pady=20)
        tk.Label(frame, text="Option A:").pack()
        tk.Entry(frame).pack()
        return frame
    
    def update_performance(self):
        # update the performance label with the latest performance data
        if self.performance_label:
            txt = "Performance:\n"
            for key, value in self.performances.items():
                txt += f"{key}: {value} ms\n"
            self.performance_label.config(text=txt)

    def create_performance_tab(self):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Performance")
        
        # show the map of performance: it should be key(title) to elapsed time (ms)
        txt = "Performance:\n"
        for key, value in self.performances.items():
            txt += f"{key}: {value} ms\n"
            
        self.performance_label = tk.Label(frame, text=txt)
        self.performance_label.pack(pady=20)
        
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

def run_ros(app):
    node = AppBackend(app)
    Node.run_node(node)
    rclpy.shutdown()

def main():
    rclpy.init()

    app = App()

    # Spin ROS in background thread
    ros_thread = threading.Thread(target=run_ros, args=(app,), daemon=True)
    ros_thread.start()

    # Run GUI mainloop in main thread
    app.protocol("WM_DELETE_WINDOW", lambda: (app.destroy(), rclpy.shutdown()))
    app.mainloop()


if __name__ == "__main__":
    main()
