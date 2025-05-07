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
        self.driveMotorGearRatio = (10/60) * (24/60) # 15:1 (10t to 60t) * (24t * 60t)
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

    # Update the state of the module
    # state: The desired state of the robot
    # returns: The measured state of the module
    def updateState(self, desired_state: SUSwerveDriveModuleState, period: float) -> SUSwerveDriveModuleState:
        # Compute desired heading as angle of desired_vel (x forward, y left)
        # Compute desired speed as magnitude of desired_vel
        desired_drive_speed = sqrt(desired_state.x_vel * desired_state.x_vel + desired_state.y_vel * desired_state.y_vel)
        desired_angle = atan2(desired_state.x_vel, desired_state.y_vel)

        # never turn more than 90 degrees
        if abs(abs(desired_angle) - abs(self.angle_motor_.getAngle())) > radians(90):
            desired_drive_speed *= -1
            desired_angle -= radians(90)
            
            # clamp FIXME does this work????
            if desired_angle > 2*pi:
                desired_angle -= 2*pi
            elif desired_angle < 0:
                desired_angle += 2*pi

        # use the onboard PIDF controllers of the sparkMAXes to do everything for us
        self.drive_motor_.setVelocity(desired_drive_speed * self.config.drive_motor_conversion_factor_ * 60 * (-1 if self.config.is_drive_motor_reversed else 1)) # * 60 because drive_motor_ expects RPM but desired_speed is in m/s
        #TODO do cosine speed correction based on steer angle

        if desired_drive_speed > 0.2:
            self.angle_motor_.setPosition(desired_angle / (3.14159265358979323846264338327950 * 2) * (-1 if self.config.is_angle_motor_reversed else 1))

        # Update encoders and calculate measured state
        # self.drive_encoder_.update(period)
        # self.angle_encoder_.update(period)

        measured_drive_angle = self.angle_motor_.getAbsolutePosition() # the absolute position straight from the sparkmax

        # remap drive angle from 0 to 1 to radians (i.e. 0 is 0 degrees, 1 is 360 degrees)
        measured_drive_angle = (measured_drive_angle * 2 * pi)

        # remap
        # measured_drive_angle = -measured_drive_angle

        measured_rpm = self.drive_motor_.getRevolutionsPerMinute() # the RPM straight from the sparkmax
        # calculate meters per second using the config and measured_rpm
        # should be: speed (m/s) = RPM * ((pi * diameter) / 60)
        measured_speed_mps = (measured_rpm / 60) * ((2 * pi * self.config.wheel_radius) / self.config.driveMotorGearRatio) # m/s

        # measured_drive_speed = self.drive_encoder_.getVelocity() * self.config.drive_motor_conversion_factor_
        # measured_angle = self.angle_encoder_.getAngle() * self.config.angle_motor_conversion_factor_

        measured_state = SUSwerveDriveModuleState(measured_speed_mps * cos(measured_drive_angle), measured_speed_mps * sin(measured_drive_angle))
        return measured_state

    def applyConversionFactor(self, drive_motor_reduction: float, angle_motor_reduction: float, wheel_radius: float) -> None:
        # TODO: This is probably not the correct way to calculate the conversion factor
        self.drive_motor_conversion_factor_ = wheel_radius * self.config.driveMotorGearRatio
        # self.angle_motor_conversion_factor_ = self.angleMotorGearRatio # don't need this as the absolute encoder is directly on the shaft
