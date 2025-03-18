#ifndef SU_SWERVE_DRIVE_H
#define SU_SWERVE_DRIVE_H

#include "SUSwerveModule.h"

typedef struct{
    SUSwerveDriveModuleConfig front_left;
    SUSwerveDriveModuleConfig front_right;
    SUSwerveDriveModuleConfig back_left;
    SUSwerveDriveModuleConfig back_right;
} SUSwerveDriveConfig;

class SUSwerveDrive {

public:

    SUSwerveDrive(SUSwerveDriveConfig config, float update_period);

    // Drive the robot
    // state: The desired state of the robot
    // returns: The actual state of the robot
    SUSwerveDriveState updateState(SUSwerveDriveState state);

private:
    SUSwerveDriveModule front_left_module_;
    SUSwerveDriveModule front_right_module_;
    SUSwerveDriveModule back_left_module_;
    SUSwerveDriveModule back_right_module_;

    // 8x1 matrix of the state of the modules
    Eigen::Matrix<double, 8, 1> desired_module_state_;
    Eigen::Matrix<double, 8, 1> measured_module_state_;

    // 3x1 matrix of the state of the robot
    Eigen::Matrix<double, 3, 1> desired_robot_state_;
    Eigen::Matrix<double, 3, 1> mesured_robot_state_;

    // 3x8 matrix of the module positions
    Eigen::Matrix<double, 8, 3> module_positions_matrix_;

    // 8x3 matrix of the inverse of the module positions
    Eigen::Matrix<double, 3, 8> module_positions_matrix_pinv_;
};

inline SUSwerveDrive::SUSwerveDrive(SUSwerveDriveConfig config, float update_period) :
    front_left_module_(config.front_left),
    front_right_module_(config.front_right),
    back_left_module_(config.back_left),
    back_right_module_(config.back_right) {

    // Create the module positions matrix
    // should appear like:
    // 0 1 -Y1
    // 0 1  X1
    // ...
    module_positions_matrix_ << 1, 0, -config.front_left.y_pos,
                         0, 1,  config.front_left.x_pos,
                         1, 0, -config.front_right.y_pos,
                         0, 1,  config.front_right.x_pos,
                         1, 0, -config.back_left.y_pos,
                         0, 1,  config.back_left.x_pos,
                         1, 0, -config.back_right.y_pos,
                         0, 1,  config.back_right.x_pos;

    // Calculate the pseudo-inverse of the module positions matrix
    module_positions_matrix_pinv_ = module_positions_.completeOrthogonalDecomposition().pseudoInverse();

    // Initialize the module state matrix
    desired_modules_state_ << 0, 0, 0, 0, 0, 0, 0, 0;
    measured_modules_state_ << 0, 0, 0, 0, 0, 0, 0, 0;

    // Initialize the robot state matrix
    desired_robot_state_ << 0, 0, 0;
    measured_robot_state_ << 0, 0, 0;
}

inline SUSwerveDriveState SUSwerveDrive::updateState(SUSwerveDriveState state) {
    // Update the state of the robot
    desired_robot_state_ << state.x_vel, state.y_vel, state.angular_vel;

    // Compute the desired module state as module_positions * robot_state
    desired_modules_state_ = module_positions_matrix_ * desired_robot_state_;

    // Update the state of each module
    SUSwerveDriveModuleState front_left_measured_state = front_left_module_.updateState({desired_modules_state_(0), desired_modules_state_(1)});
    SUSwerveDriveModuleState front_right_measured_state = front_right_module_.updateState({desired_modules_state_(2), desired_modules_state_(3)});
    SUSwerveDriveModuleState back_left_measured_state = back_left_module_.updateState({desired_modules_state_(4), desired_modules_state_(5)});
    SUSwerveDriveModuleState back_right_measured_state = back_right_module_.updateState({desired_modules_state_(6), desired_modules_state_(7)});
    
    // Use the feedback from each module to calculate the state of the robot
    measured_modules_state_ << front_left_state.x_vel, front_left_state.y_vel,
                     front_right_state.x_vel, front_right_state.y_vel,
                     back_left_state.x_vel, back_left_state.y_vel,
                     back_right_state.x_vel, back_right_state.y_vel;

    measured_robot_state_ = module_positions_matrix_pinv_ * measured_modules_state_;

    return {robot_state_(0), robot_state_(1), robot_state_(2)};
}

#endif