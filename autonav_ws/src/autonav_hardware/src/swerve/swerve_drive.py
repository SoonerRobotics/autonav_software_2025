import numpy as np
from swerve.swerve_module import SUSwerveDriveModule, SUSwerveDriveModuleState
from swerve.swerve_config import SUSwerveDriveConfig

class SUSwerveDriveState:
    def __init__(self, delta_x: float, delta_y: float, delta_theta: float):
        self.delta_x = delta_x             # (delta meters)
        self.delta_y = delta_y             # (delta meters)
        self.delta_theta = delta_theta     # (delta radians) 

class SUSwerveDrive:
    def __init__(self, front_left_module: SUSwerveDriveModule, front_right_module: SUSwerveDriveModule, back_left_module: SUSwerveDriveModule, back_right_module: SUSwerveDriveModule, config: SUSwerveDriveConfig):
        self.front_left_module_ = front_left_module
        self.front_right_module_ = front_right_module
        self.back_left_module_ = back_left_module
        self.back_right_module_ = back_right_module

        self.config = config

        # 8x1 matrix of the state of the modules
        self.desired_modules_state_ = np.zeros((8, 1), np.double)
        self.measured_modules_state_ = np.zeros((8, 1), np.double)

        # 3x1 matrix of the state of the robot
        self.desired_robot_state_ = np.zeros((3, 1), np.double)
        self.measured_robot_state_ = np.zeros((3, 1), np.double)

        # 3x8 matrix of the module positions
        # Create the module positions matrix
        # should appear like:
        # 1 0 -Y1
        # 0 1  X1
        # ...
        mod_pos_mat = [
            [1, 0, -self.config.front_left.y_pos],
            [0, 1, self.config.front_left.x_pos],

            [1, 0, -self.config.front_right.y_pos],
            [0, 1, self.config.front_right.x_pos],

            [1, 0, -self.config.back_left.y_pos],
            [0, 1, self.config.back_left.x_pos],

            [1, 0, -self.config.back_right.y_pos],
            [0, 1, self.config.back_right.x_pos],
        ]
        self.module_positions_matrix_ = np.array(mod_pos_mat, np.double)

        # 8x3 matrix of the inverse of the module positions
        # Pre-compute the pseudo-inverse of the module positions matrix
        self.module_positions_matrix_pinv_ = np.linalg.pinv(self.module_positions_matrix_)

        # Apply the conversion factors to each module
        self.front_left_module_.applyConversionFactor(self.config.driveMotorGearRatio, self.config.angleMotorGearRatio, self.config.wheelRadius)
        self.front_right_module_.applyConversionFactor(self.config.driveMotorGearRatio, self.config.angleMotorGearRatio, self.config.wheelRadius)
        self.back_left_module_.applyConversionFactor(self.config.driveMotorGearRatio, self.config.angleMotorGearRatio, self.config.wheelRadius)
        self.back_right_module_.applyConversionFactor(self.config.driveMotorGearRatio, self.config.angleMotorGearRatio, self.config.wheelRadius)

    # Drive the robot
    # state: The desired state of the robot
    # returns: The actual state of the robot    
    def updateState(self, state: SUSwerveDriveState, period: float) -> SUSwerveDriveState:
        self.desired_robot_state_ = np.array([state.delta_x, state.delta_y, state.delta_theta], np.double)

        # compute the desired module state as module_positions * robot_state
        desired_modules_state_ = self.module_positions_matrix_ @ self.desired_robot_state_

        # update the modules
        self.front_left_module_.updateState(SUSwerveDriveModuleState(desired_modules_state_[0], desired_modules_state_[1]), period)
        self.front_right_module_.updateState(SUSwerveDriveModuleState(desired_modules_state_[2], desired_modules_state_[3]), period)
        self.back_left_module_.updateState(SUSwerveDriveModuleState(desired_modules_state_[4], desired_modules_state_[5]), period)
        self.back_right_module_.updateState(SUSwerveDriveModuleState(desired_modules_state_[6], desired_modules_state_[7]), period)
        
        # grab the states of each module
        # todo: it wouldn't be a terrible idea to move it back into updateState
        front_left_delta = self.front_left_module_.getState() # returns (drive_delta, angle_delta) in meters and radians
        front_right_delta = self.front_right_module_.getState()
        back_left_delta = self.back_left_module_.getState()
        back_right_delta = self.back_right_module_.getState()

        # Extract drive delta (meters) and angle delta (radians) for each module
        modules_deltas = [
            (front_left_delta, self.config.front_left),
            (front_right_delta, self.config.front_right),
            (back_left_delta, self.config.back_left),
            (back_right_delta, self.config.back_right),
        ]

        # our measured state
        measured_modules_state = []
        for (delta_drive, angle), module_config in modules_deltas:
            vx = (delta_drive) * np.cos(angle)
            vy = (delta_drive) * np.sin(angle)
            measured_modules_state.append(vx)
            measured_modules_state.append(vy)

        # a little numpy foolerly
        self.measured_modules_state_ = np.array(measured_modules_state).reshape((8, 1))
        self.measured_robot_state_ = self.module_positions_matrix_pinv_ @ self.measured_modules_state_

        delta_x = self.measured_robot_state_[0, 0]
        delta_y = self.measured_robot_state_[1, 0]

        # I will be honest, this 3 makes no sense to me. Using pi made it accumulate way more error, so 3 it is
        delta_theta = self.measured_robot_state_[2, 0] * 3
        return SUSwerveDriveState(delta_x, delta_y, delta_theta)