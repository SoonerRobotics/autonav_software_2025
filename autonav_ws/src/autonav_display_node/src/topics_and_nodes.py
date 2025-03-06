from enum import Enum

class Topics(Enum):
    # Topic Lis
    # teneres
    SYSTEM_STATE = "autonav/shared/system"
    DEVICE_STATE = "autonav/shared/device"

    # IMU Data
    IMU = "/autonav/imu"
    AUTONAV_GPS = "/autonav/gps"
    MOTOR_INPUT = "/autonav/MotorInput"
    POSITION = "/autonav/position"

    MOTOR_FEEDBACK = "/autonav/MotorFeedback"
    NUC_STATISTICS = "/autonav/statistic"
    ULTRASONICS = "/autonav/ultrasonics"
    CONBUS = "/autonav/conbus"
    SAFETY_LIGHTS = "/autonav/safety_lights"
    PERFORMANCE = "autonav/performance"

    # Raw camera
    RAW_LEFT = "autonav/camera/left"
    RAW_RIGHT = "autonav/camera/right"
    RAW_FRONT = "autonav/camera/front"
    RAW_BACK = "autonav/camera/back"

    # Other Camera Nodes
    COMBINED_IMAGE = "/autonav/vision/combined/filtered"
    FEELERS = "/autonav/feelers/debug"  # todo does this transmit an image? (assuming it does for now

    # Others
    CONFIGURATION = "/scr/configuration"  # TODO IS THIS STILL A TOPIC?
    PLAYBACK = "autonav/autonav_playback"  # TODO feed in new data and test if this actually gets data in

    MOTOR_CONTROLLER_DEBUG = ""  # TOD???
