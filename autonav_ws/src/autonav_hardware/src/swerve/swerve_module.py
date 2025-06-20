from math import atan2, sqrt, cos, sin, pi, radians
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
        self.driveMotorGearRatio = 1 / ((10/60) * (24/60)) # 15:1 (10t to 60t) * (24t * 60t)
        self.steerMotorGearRatio = 5*5 # 25:1
        self.wheel_radius = 0.1016 # meters

        # final units should be motor rotations per second I think
        self.drive_motor_conversion_factor_ = self.driveMotorGearRatio / (2 * self.wheel_radius * pi)
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
        
        self.last_set_angle_ = 0.0 # radians
        self.drive_motor_last_position = 0.0
        self.current_angle_motor_position = 0.0
        self.last_drive_motor_delta = 0.0

    def getDriveDelta(self) -> float:
        if self.drive_motor_last_position == 0.0:
            self.drive_motor_last_position = self.drive_motor_.getDrivePosition() / self.config.drive_motor_conversion_factor_ 
            return 0.0
        
        # get the current position of the drive motor
        current_drive_motor_position = self.drive_motor_.getDrivePosition() / self.config.drive_motor_conversion_factor_ 

        # calculate the delta
        self.last_drive_motor_delta = (current_drive_motor_position - self.drive_motor_last_position) # meters
        self.drive_motor_last_position = current_drive_motor_position
        self.current_angle_motor_position = self.angle_motor_.getAngle()
        return self.last_drive_motor_delta
        
    def getAngle(self) -> float:
        return self.current_angle_motor_position

    def getState(self) -> float:
        return self.getDriveDelta(), self.getAngle()

    # Update the state of the module
    # state: The desired state of the robot
    def updateState(self, desired_state: SUSwerveDriveModuleState, period: float) -> None:
        # Compute desired heading as angle of desired_vel (x forward, y left)
        # Compute desired speed as magnitude of desired_vel
        desired_drive_speed = sqrt(desired_state.x_vel * desired_state.x_vel + desired_state.y_vel * desired_state.y_vel)
        desired_angle = atan2(desired_state.y_vel, desired_state.x_vel)

        # never turn more than 90 degrees
        angle_error = desired_angle - self.last_set_angle_
        if angle_error > pi:
            angle_error = (2 * pi) - angle_error
        elif angle_error < -pi:
            angle_error = (2 * pi) + angle_error

        if abs(angle_error) > radians(90):
            desired_drive_speed *= -1
            desired_angle = desired_angle + radians(180)

        if desired_angle > pi:
            desired_angle = desired_angle - (2 * pi)
        elif desired_angle < -pi:
            desired_angle = desired_angle + (2 * pi)

        if abs(desired_drive_speed) > 0.3:
            self.last_set_angle_ = desired_angle
            self.angle_motor_.setPosition(desired_angle / (3.14159265358979323846264338327950 * 2) * (-1 if self.config.is_angle_motor_reversed else 1))

        # cosine correction
        desired_drive_speed *= cos(desired_angle - self.angle_motor_.getAngle())

        # use the onboard PIDF controllers of the sparkMAXes to do everything for us
        # self.drive_motor_.setVelocity(desired_drive_speed * self.config.drive_motor_conversion_factor_ * 60 * (-1 if self.config.is_drive_motor_reversed else 1)) # * 60 because drive_motor_ expects RPM but desired_speed is in m/s
        self.drive_motor_.setVelocity(desired_drive_speed * 42 * (-1 if self.config.is_drive_motor_reversed else 1))

    def applyConversionFactor(self, drive_motor_reduction: float, angle_motor_reduction: float, wheel_radius: float) -> None:
        # TODO: This is probably not the correct way to calculate the conversion factor
        self.drive_motor_conversion_factor_ = wheel_radius * self.config.driveMotorGearRatio
        # self.angle_motor_conversion_factor_ = self.angleMotorGearRatio # don't need this as the absolute encoder is directly on the shaft
