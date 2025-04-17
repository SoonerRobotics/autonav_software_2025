from swerve.swerve_module import SUSwerveDriveModuleConfig

class SUSwerveDriveConfig:
    def __init__(self, front_left: SUSwerveDriveModuleConfig, front_right: SUSwerveDriveModuleConfig, back_left: SUSwerveDriveModuleConfig, back_right: SUSwerveDriveModuleConfig, driveMotorGearRatio: float, angleMotorGearRatio: float, wheelRadius: float):
        self.front_left = front_left
        self.front_right = front_right
        self.back_left = back_left
        self.back_right = back_right

        self.driveMotorGearRatio = driveMotorGearRatio
        self.angleMotorGearRatio = angleMotorGearRatio

        self.wheelRadius = wheelRadius # (meters)

MCP2515_SCK_MOTOR = 14
MCP2515_MOSI_MOTOR = 15
MCP2515_MISO_MOTOR = 12
MCP2515_CS_MOTOR = 13
MCP2515_INT_MOTOR = 11

front_left_module_config = SUSwerveDriveModuleConfig(
    x_pos = 1,
    y_pos = 1,
    drive_motor_id = 1,
    angle_motor_id = 2,
    is_drive_motor_reversed = False,
    is_angle_motor_reversed = False
)

front_right_module_config = SUSwerveDriveModuleConfig(
    x_pos = 1,
    y_pos = -1,
    drive_motor_id = 3,
    angle_motor_id = 4,
    is_drive_motor_reversed = False,
    is_angle_motor_reversed = False
)

back_left_module_config = SUSwerveDriveModuleConfig(
    x_pos = -1,
    y_pos = 1,
    drive_motor_id = 5,
    angle_motor_id = 6,
    is_drive_motor_reversed = False,
    is_angle_motor_reversed = False
)

back_right_module_config = SUSwerveDriveModuleConfig(
    x_pos = -1,
    y_pos = -1,
    drive_motor_id = 7,
    angle_motor_id = 8,
    is_drive_motor_reversed = False,
    is_angle_motor_reversed = False
)

swerve_config = SUSwerveDriveConfig(
    front_left = front_left_module_config,
    front_right = front_right_module_config,
    back_left = back_left_module_config,
    back_right = back_right_module_config,
    driveMotorGearRatio = 16,
    angleMotorGearRatio = 25,
    wheelRadius = 0.1016
)
