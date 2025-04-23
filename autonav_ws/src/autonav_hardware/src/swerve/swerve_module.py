from math import atan2, sqrt, cos, sin, pi
from swerve.can_spark_max import CanSparkMax

class SUSwerveDriveModuleConfig:
    def __init__(self, x_pos: float, y_pos: float, drive_motor_id: int, angle_motor_id: int, is_drive_motor_reversed: bool, is_angle_motor_reversed: bool):
        # Module position
        self.x_pos = x_pos # (meters)
        self.y_pos = y_pos # (meters)

        # SPARK Max IDs
        self.drive_motor_id = drive_motor_id
        self.angle_motor_id = angle_motor_id

        # Module Config
        self.is_drive_motor_reversed = is_drive_motor_reversed
        self.is_angle_motor_reversed = is_angle_motor_reversed

        # constants
        self.driveMotorGearRatio = 4*4 # 16:1
        self.steerMotorGearRatio = 5*5 # 25:1
        self.wheel_radius = 0.1016 # meters

        # ??? FIXME
        self.drive_motor_conversion_factor_ = self.driveMotorGearRatio * self.wheel_radius
        self.angle_motor_conversion_factor_ = self.steerMotorGearRatio

class SUSwerveDriveModuleState:
    def __init__(self, x_vel: float, y_vel: float):
        self.x_vel = x_vel
        self.y_vel = y_vel

class SUSwerveDriveModule:
    def __init__(self, config: SUSwerveDriveModuleConfig, drive_motor: CanSparkMax, angle_motor: CanSparkMax):
        self.config = config
        
        self.drive_motor_ = drive_motor
        self.angle_motor_ = angle_motor

    # Update the state of the module
    # state: The desired state of the robot
    # returns: The measured state of the module
    def updateState(self, desired_state: SUSwerveDriveModuleState, period: float) -> SUSwerveDriveModuleState:
        # Compute desired heading as angle of desired_vel (x forward, y left)
        # Compute desired speed as magnitude of desired_vel
        desired_drive_speed = sqrt(desired_state.x_vel * desired_state.x_vel + desired_state.y_vel * desired_state.y_vel)
        desired_angle = atan2(desired_state.x_vel, desired_state.y_vel)

        if desired_drive_speed < 0.1:
            # if the desired speed is too low, set the angle to 0
            desired_angle = 0.0

        # use the onboard PIDF controllers of the sparkMAXes to do everything for us
        self.drive_motor_.setVelocity(desired_drive_speed * 42)
        self.angle_motor_.setPosition(desired_angle / (3.14159265358979323846264338327950 * 2))

        # Update encoders and calculate measured state
        # self.drive_encoder_.update(period)
        # self.angle_encoder_.update(period)

        measured_drive_angle = self.angle_motor_.getAbsolutePosition() # the absolute position straight from the sparkmax
        measured_rpm = self.drive_motor_.getRevolutionsPerMinute() # the RPM straight from the sparkmax
        # calculate meters per second using the config and measured_rpm
        # should be: speed (m/s) = RPM * ((pi * diameter) / 60)
        measured_speed_mps = (measured_rpm / 60) * ((2 * pi * 0.1016) / self.driveMotorGearRatio) # m/s


        # measured_drive_speed = self.drive_encoder_.getVelocity() * self.config.drive_motor_conversion_factor_
        # measured_angle = self.angle_encoder_.getAngle() * self.config.angle_motor_conversion_factor_

        measured_state = SUSwerveDriveModuleState()
        measured_state.x_vel = measured_speed_mps * cos(measured_drive_angle)
        measured_state.y_vel = measured_speed_mps * sin(measured_drive_angle)

        return measured_state

    def applyConversionFactor(self, drive_motor_reduction: float, angle_motor_reduction: float, wheel_radius: float) -> None:
        # TODO: This is probably not the correct way to calculate the conversion factor
        self.drive_motor_conversion_factor_ = wheel_radius * self.config.driveMotorGearRatio
        # self.angle_motor_conversion_factor_ = self.angleMotorGearRatio # don't need this as the absolute encoder is directly on the shaft
