import numpy as np
from swerve_drive import SUSwerveDriveModule

class SUSwerveDriveState:
    def __init__(self, x_vel: float, y_vel: float, angular_vel: float):
        self.x_vel = x_vel             # (meters/second)
        self.y_vel = y_vel             # (meters/second)
        self.angular_vel = angular_vel # (radians/second) 

class SUSwerveDrive:
    def __init__(self, front_left_module_: SUSwerveDriveModule, front_right_module_: SUSwerveDriveModule, back_left_module_: SUSwerveDriveModule, back_right_module_: SUSwerveDriveModule):
        self.front_left_module_ = front_left_module_
        self.front_right_module_ = front_right_module_
        self.back_left_module_ = back_left_module_
        self.back_right_module_ = back_right_module_

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
            [0, 1, self.config.back_right.x_pos]
        ]
        self.module_positions_matrix_ = np.array(mod_pos_mat, np.double)

        # 8x3 matrix of the inverse of the module positions
        # Pre-compute the pseudo-inverse of the module positions matrix
        self.module_positions_matrix_pinv_ = self.module_positions_matrix_.completeOrthogonalDecomposition().pseudoInverse()

        # Apply the conversion factors to each module
        front_left_module_.applyConversionFactor(self.config.driveMotorGearRatio, self.config.angleMotorGearRatio, self.config.wheelRadius)
        front_right_module_.applyConversionFactor(self.config.driveMotorGearRatio, self.config.angleMotorGearRatio, self.config.wheelRadius)
        back_left_module_.applyConversionFactor(self.config.driveMotorGearRatio, self.config.angleMotorGearRatio, self.config.wheelRadius)
        back_right_module_.applyConversionFactor(self.config.driveMotorGearRatio, self.config.angleMotorGearRatio, self.config.wheelRadius)

    # Drive the robot
    # state: The desired state of the robot
    # returns: The actual state of the robot
    def updateState(self, state: SUSwerveDriveState, period: float) -> SUSwerveDriveState:
        # Update the state of the robot
        self.desired_robot_state_ = np.array([state.x_vel, state.y_vel, state.angular_vel], np.double)
        
        # Compute the desired module state as module_positions * robot_state
        desired_modules_state_ = self.module_positions_matrix_ @ self.desired_robot_state_
        
        # Update the state of each module
        front_left_measured_state = self.front_left_module_.updateState({desired_modules_state_(0), desired_modules_state_(1)}, period)
        front_right_measured_state = self.front_right_module_.updateState({desired_modules_state_(2), desired_modules_state_(3)}, period)
        back_left_measured_state = self.back_left_module_.updateState({desired_modules_state_(4), desired_modules_state_(5)}, period)
        back_right_measured_state = self.back_right_module_.updateState({desired_modules_state_(6), desired_modules_state_(7)}, period)
        
        # Use the feedback from each module to calculate the state of the robot
        self.measured_modules_state_ = np.array([
            front_left_measured_state.x_vel, front_left_measured_state.y_vel,
            front_right_measured_state.x_vel, front_right_measured_state.y_vel,
            back_left_measured_state.x_vel, back_left_measured_state.y_vel,
            back_right_measured_state.x_vel, back_right_measured_state.y_vel
        ], np.double)
        
        self.measured_robot_state_ = self.module_positions_matrix_pinv_ @ self.measured_modules_state_
        
        return SUSwerveDriveState(self.measured_robot_state_(0), self.measured_robot_state_(1), self.measured_robot_state_(2))