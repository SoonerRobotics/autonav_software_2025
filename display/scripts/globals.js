let systemState = {
    state: 0,
    mode: 0,
    mobility: false,
    estop: false
};
let preferences = {
    gpsFormat: "DD",
    host: "127.0.0.1",
    port: 8080,
    theme: "dark"
};
let config = {};
let conbus = {};
let deviceStates = {};
let logs = [];
let iterator = 0;
const iterators = [];
let development_mode = false;
let connected = false;
let current_preset = "ERROR_NO_PRESET_AUTODETECTED";

const addressKeys = {
    "autonav_serial_imu": {
        "internal_title": "[Serial] IMU",
        "imu_read_rate": "float"
    },

    "autonav_serial_camera_left": {
        "internal_title": "[Serial] Left Camera",
        "refresh_rate": "int",
        "output_width": "int",
        "output_height": "int",
        "scan_rate": "int",
        "flip_horizontal": "bool",
        "flip_vertical": "bool",
        "rotate_clockwise": "bool"
    },

    "autonav_serial_camera_right": {
        "internal_title": "[Serial] Right Camera",
        "refresh_rate": "int",
        "output_width": "int",
        "output_height": "int",
        "scan_rate": "int",
        "flip_horizontal": "bool",
        "flip_vertical": "bool",
        "rotate_clockwise": "bool"
    },

    "autonav_vision_transformer": {
        "internal_title": "[Vision] Transformer",
        "region_of_disinterest_offset": "int",
        "lower_hue": "int",
        "lower_saturation": "int",
        "lower_value": "int",
        "upper_hue": "int",
        "upper_saturation": "int",
        "upper_value": "int",
        "blur": "int",
        "blur_iterations": "int",
        "map_res": "int",
        "left_bottomleft": "point.int",
        "left_bottomright": "point.int",
        "left_topleft": "point.int",
        "left_topright": "point.int",
        "right_bottomleft": "point.int",
        "right_bottomright": "point.int",
        "right_topleft": "point.int",
        "right_topright": "point.int",
        "disable_blur": "bool",
        "disable_hsv": "bool",
        "disable_perspective_transform": "bool",
        "disable_region_of_disinterest": "bool",
        "parallelogram_left": "parallelogram.int",
        "parallelogram_right": "parallelogram.int",
        "top_width": "float",
        "bottom_width": "float",
        "offset": "float"
    },

    "autonav_vision_expandifier": {
        "internal_title": "[Vision] Expandifier",
        "horizontal_fov": "float",
        "map_res": "int",
        "vertical_fov": "float",
        "max_range": "float",
        "no_go_percent": "float",
        "no_go_range": "float",
    },

    "autonav_filters": {
        "internal_title": "[Localization] Filters",
        "filter_type": {
            0: "Deadreckoning",
            1: "Particle Filter"
        }
    },

    "autonav_manual_steamcontroller": {
        "internal_title": "[Manual] Steam Controller",
        "forward_speed": "float",
        "steering_deadzone": "float",
        "throttle_deadzone": "float",
        "turn_speed": "float",
        "max_forward_speed": "float",
        "max_turn_speed": "float",
        "invert_steering": "bool",
        "invert_throttle": "bool",
        "throttle_rate": "float",
        "steering_rate": "float"
    },

    "autonav_nav_astar": {
        "internal_title": "[Navigation] A*",
        "latitude_length": "float",
        "longitude_length": "float",
        "waypoint_pop_distance": "float",
        "waypoint_delay": "float",
        "robot_y": "int",
        "use_only_waypoints": "bool",
        "waypoints": "waypoints",
        "waypointDirection": {
            0: "North",
            1: "South",
            2: "Misc 1",
            3: "Misc 2",
            4: "Misc 3",
            5: "Misc 4",
            6: "Misc 5",
        },
        "calculateWaypointDirection": "bool",
        "vertical_fov": "float",
        "horizontal_fov": "float",
        "waypointMaxWeight": "float",
        "waypointWeight": "float",
    },

    "zemlin_path_resolver": {
        "internal_title": "[Path Resolver]",
        "forward_speed": "float",
        "reverse_speed": "float",
        "angular_aggressiveness": "float",
        "max_angular_speed": "float",
        "radius_multiplier": "float",
        "radius_max": "float",
        "radius_start": "float"
    },

    "zemlin_filters": {
        "internal_title": "[Position Filters]",
        "latitude_length": "float",
        "longitude_length": "float",
        "filter_type": {
            0: "Dead Reckoning",
            1: "Particle Filter",
            2: "Bearing Filter"
        }
    },

    "autonav_nav_resolver": {
        "internal_title": "[Navigation] Resolver",
        "forward_speed": "float",
        "reverse_speed": "float",
        "radius_multiplier": "float",
        "radius_max": "float",
        "radius_start": "float",
        "angular_aggression": "float",
        "max_angular_speed": "float"
    },

    "autonav_image_combiner": {
        "internal_title": "[Image Combiner]",
        "map_res": "int"
    },

    "autonav_playback": {
        "internal_title": "[Playback]",
        "record_imu": "bool",
        "record_gps": "bool",
        "record_position": "bool",
        "record_feedback": "bool",
        "record_motor_debug": "bool",
        "record_raw_cameras": "bool",
        "record_filtered_cameras": "bool",
        "record_astar": "bool",
        "record_autonomous": "bool",
        "record_manual": "bool",
        "frame_rate": "int"
    }
};

const conbusDevices = {
    0x10: {
        title: "Motor Controller",
        registers: {
            0x0: {
                title: "Update Period",
                type: "float",
                readonly: true
            },
            0x1: {
                title: "Pulses Per Radian",
                type: "float",
            },
            0x2: {
                title: "Wheel Radius",
                type: "float",
            },
            0x3: {
                title: "Wheel Base Length",
                type: "float",
            },
            0x4: {
                title: "Slew Rate Limit",
                type: "float",
            },
            0x5: {
                title: "Left Encoder Factor",
                type: "float",
            },
            0x6: {
                title: "Right Encoder Factor",
                type: "float",
            },
            0x10: {
                title: "Velocity Kp",
                type: "float",
            },
            0x11: {
                title: "Velocity Ki",
                type: "float",
            },
            0x12: {
                title: "Velocity Kd",
                type: "float",
            },
            0x13: {
                title: "Velocity Kf",
                type: "float",
            },
            0x20: {
                title: "Angular Kp",
                type: "float",
            },
            0x21: {
                title: "Angular Ki",
                type: "float",
            },
            0x22: {
                title: "Angular Kd",
                type: "float",
            },
            0x23: {
                title: "Angular Kf",
                type: "float",
            },
            0x30: {
                title: "Use Obstacle Avoidance",
                type: "bool",
            },
            0x31: {
                title: "Collision Box Distance",
                type: "uint",
            },
            0x40: {
                title: "Send Statistics",
                type: "bool",
            },
            0x50: {
                title: "Motor Updates Between Deltaodom",
                type: "uint",
            }
        }
    },
    0x11: {
        title: "Safety Lights",
        registers: {
            0x0: {
                title: "Blink Rate",
                type: "uint",
            }
        }
    }
};


// Topic List
const TOPIC_BROADCAST = "/autonav/shared/autonav_display_broadcast";
const TOPIC_SYSTEM_STATE = "/autonav/shared/system";
const TOPIC_DEVICE_STATE = "/autonav/shared/device";
const TOPIC_LOG = "/autonav/shared/log";

// IMU Data
const TOPIC_IMU = "/autonav/imu";
const TOPIC_AUTONAV_GPS = "/autonav/gps";
const TOPIC_MOTOR_INPUT = "/autonav/motor_input";
const TOPIC_POSITION = "/autonav/position";
const TOPIC_CONTROLLER_INPUT = "/autonav/controller_input";

// Motor and System Feedback
const TOPIC_MOTOR_FEEDBACK = "/autonav/motor_feedback";
const TOPIC_NUC_STATISTICS = "/autonav/statistics";
const TOPIC_ULTRASONICS = "/autonav/ultrasonic";
const TOPIC_CONBUS_DATA = "/autonav/conbus/data";
const TOPIC_CONBUS_INSTRUCTION = "/autonav/conbus/instruction";

// Aliases for backward compatibility
const TOPIC_CONBUS = "/autonav/conbus/data";
const TOPIC_SAFETY_LIGHTS = "/autonav/safety_lights";
const TOPIC_PERFORMANCE = "/autonav/performance";

// PID and Motor Statistics
const TOPIC_LINEAR_PID_STATISTICS = "/autonav/linear_pid_statistics";
const TOPIC_ANGULAR_PID_STATISTICS = "/autonav/angular_pid_statistics";
const TOPIC_MOTOR_STATISTICS_FRONT = "/autonav/motor_statistics_front_motors";
const TOPIC_MOTOR_STATISTICS_BACK = "/autonav/motor_statistics_back_motors";
const TOPIC_CAN_STATS = "/autonav/can_stats";
const TOPIC_ZERO_ENCODERS = "/autonav/zero_encoders";

// Raw camera
const TOPIC_RAW_LEFT = "/autonav/camera/left";
const TOPIC_RAW_RIGHT = "/autonav/camera/right";
const TOPIC_RAW_FRONT = "/autonav/camera/front";
const TOPIC_RAW_BACK = "/autonav/camera/back";

// Other Camera Nodes
const TOPIC_COMBINED_IMAGE = "/autonav/vision/combined/filtered";
const TOPIC_FEELERS = "/autonav/feelers/debug";

// Configuration
const TOPIC_CONFIGURATION_BROADCAST = "/autonav/shared/config/requests";
const TOPIC_CONFIGURATION_UPDATE = "/autonav/shared/config/updates";
const TOPIC_CONFIG_PRESTS_LOAD = "/autonav/presets/load";
const TOPIC_CONFIG_PRESTS_SAVE = "/autonav/presets/save";

// Others
const TOPIC_CONFIGURATION = "/scr/configuration"; // Legacy configuration topic
const TOPIC_PLAYBACK = "/autonav/autonav_playback";
const TOPIC_AUDIBLE_FEEDBACK = "/autonav/audible_feedback";
