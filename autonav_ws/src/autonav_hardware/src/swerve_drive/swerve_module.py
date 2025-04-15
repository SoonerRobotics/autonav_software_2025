from math import atan2, sqrt, cos, sin
from can_spark_max import CanSparkMax

class SUSwerveDriveModuleConfig:
    def __init__(self, x_pos: float, y_pos: float, drive_motor_id: int, angle_motor_id: int, drive_encoder_id: int, angle_encoder_id: int, is_drive_motor_reversed: bool, is_angle_motor_reversed: bool):
        # Module position
        self.x_pos = x_pos # (meters)
        self.y_pos = y_pos # (meters)

        # SPARK Max IDs
        self.drive_motor_id = drive_motor_id
        self.angle_motor_id = angle_motor_id

        # Encoder IDs
        self.drive_encoder_id = drive_encoder_id
        self.angle_encoder_id = angle_encoder_id

        # Module Config
        self.is_drive_motor_reversed = is_drive_motor_reversed
        self.is_angle_motor_reversed = is_angle_motor_reversed

class SUSwerveDriveModuleState:
    def __init__(self, x_vel: float, y_vel: float):
        self.x_vel = x_vel
        self.y_vel = y_vel

class SUSwerveDriveModule:
    def __init__(self, config: SUSwerveDriveModuleConfig, drive_motor: CanSparkMax, angle_motor: CanSparkMax, drive_encoder: AbsoluteEncoder, angle_encoder: AbsoluteEncoder, drive_motor_conversion_factor: float, angle_motor_conversion_factor: float):
        self.config = config
        self.drive_motor = drive_motor
        self.angle_motor = angle_motor
        self.drive_encoder = drive_encoder
        self.angle_encoder = angle_encoder
        self.drive_motor_conversion_factor = drive_motor_conversion_factor
        self.angle_motor_conversion_factor = angle_motor_conversion_factor

    # Update the state of the module
    # state: The desired state of the robot
    # returns: The measured state of the module
    def updateState(self, desired_state: SUSwerveDriveModuleState, period: float) -> SUSwerveDriveModuleState:
        # Compute desired heading as angle of desired_vel (x forward, y left)
        # Compute desired speed as magnitude of desired_vel
        desired_drive_speed = sqrt(desired_state.x_vel * desired_state.x_vel + desired_state.y_vel * desired_state.y_vel)
        desired_angle = atan2(desired_state.x_vel, desired_state.y_vel)

        # TODO: Implement PIDF control to set the module to the desired state
        self.drive_motor_.setPWM(0)
        self.angle_motor_.setPWM(0)

        # Update encoders and calculate measured state
        self.drive_encoder_.update(period)
        self.angle_encoder_.update(period)

        measured_drive_speed = self.drive_encoder_.getVelocity() * self.drive_motor_conversion_factor_
        measured_angle = self.angle_encoder_.getAngle() * self.angle_motor_conversion_factor_

        measured_state = SUSwerveDriveModuleState()
        measured_state.x_vel = measured_drive_speed * cos(measured_angle)
        measured_state.y_vel = measured_drive_speed * sin(measured_angle)

        return measured_state

    def applyConversionFactor(self, drive_motor_reduction: float, angle_motor_reduction: float, wheel_radius: float) -> None:
        # TODO: This is probably not the correct way to calculate the conversion factor
        self.drive_motor_conversion_factor_ = wheel_radius * self.driveMotorGearRatio
        self.angle_motor_conversion_factor_ = self.angleMotorGearRatio