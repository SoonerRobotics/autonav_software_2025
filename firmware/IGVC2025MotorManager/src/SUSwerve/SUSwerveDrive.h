#ifndef SU_SWERVE_DRIVE_H
#define SU_SWERVE_DRIVE_H

#include "ArduinoEigenDense.h"
#include "SUSwerveModule.h"

typedef struct
{
    SUSwerveDriveModuleConfig front_left;
    SUSwerveDriveModuleConfig front_right;
    SUSwerveDriveModuleConfig back_left;
    SUSwerveDriveModuleConfig back_right;

    double driveMotorGearRatio;
    double angleMotorGearRatio;

    double wheelRadius; // (meters)
} SUSwerveDriveConfig;

typedef struct
{
    double x_vel;       // (meters/second)
    double y_vel;       // (meters/second)
    double angular_vel; // (radians/second)
} SUSwerveDriveState;

class SUSwerveDrive
{

public:
    SUSwerveDrive(SUSwerveDriveConfig config, ACAN2515 *driver);

    // Drive the robot
    // state: The desired state of the robot
    // returns: The actual state of the robot
    SUSwerveDriveState updateState(SUSwerveDriveState state, double period);

private:
    SUSwerveDriveModule front_left_module_;
    SUSwerveDriveModule front_right_module_;
    SUSwerveDriveModule back_left_module_;
    SUSwerveDriveModule back_right_module_;

    // 8x1 matrix of the state of the modules
    Eigen::Matrix<double, 8, 1> desired_modules_state_;
    Eigen::Matrix<double, 8, 1> measured_modules_state_;

    // 3x1 matrix of the state of the robot
    Eigen::Matrix<double, 3, 1> desired_robot_state_;
    Eigen::Matrix<double, 3, 1> measured_robot_state_;

    // 3x8 matrix of the module positions
    Eigen::Matrix<double, 8, 3> module_positions_matrix_;

    // 8x3 matrix of the inverse of the module positions
    Eigen::Matrix<double, 3, 8> module_positions_matrix_pinv_;
};

inline SUSwerveDrive::SUSwerveDrive(SUSwerveDriveConfig config, ACAN2515 *driver) : front_left_module_(config.front_left, driver),
                                                                                    front_right_module_(config.front_right, driver),
                                                                                    back_left_module_(config.back_left, driver),
                                                                                    back_right_module_(config.back_right, driver)
{
    // Apply the conversion factors to each module
    front_left_module_.applyConversionFactor(config.driveMotorGearRatio, config.angleMotorGearRatio, config.wheelRadius);
    front_right_module_.applyConversionFactor(config.driveMotorGearRatio, config.angleMotorGearRatio, config.wheelRadius);
    back_left_module_.applyConversionFactor(config.driveMotorGearRatio, config.angleMotorGearRatio, config.wheelRadius);
    back_right_module_.applyConversionFactor(config.driveMotorGearRatio, config.angleMotorGearRatio, config.wheelRadius);

    // Create the module positions matrix
    // should appear like:
    // 1 0 -Y1
    // 0 1  X1
    // ...
    module_positions_matrix_ << 1, 0, -config.front_left.y_pos,
        0, 1, config.front_left.x_pos,
        1, 0, -config.front_right.y_pos,
        0, 1, config.front_right.x_pos,
        1, 0, -config.back_left.y_pos,
        0, 1, config.back_left.x_pos,
        1, 0, -config.back_right.y_pos,
        0, 1, config.back_right.x_pos;

    // Pre-compute the pseudo-inverse of the module positions matrix
    module_positions_matrix_pinv_ = module_positions_matrix_.completeOrthogonalDecomposition().pseudoInverse();

    // Initialize the module state vectors
    desired_modules_state_ << 0, 0, 0, 0, 0, 0, 0, 0;
    measured_modules_state_ << 0, 0, 0, 0, 0, 0, 0, 0;

    // Initialize the robot state vectors
    desired_robot_state_ << 0, 0, 0;
    measured_robot_state_ << 0, 0, 0;
}

inline SUSwerveDriveState SUSwerveDrive::updateState(SUSwerveDriveState state, double period)
{
    // Update the state of the robot
    desired_robot_state_ << state.x_vel, state.y_vel, state.angular_vel;

    // Compute the desired module state as module_positions * robot_state
    desired_modules_state_ = module_positions_matrix_ * desired_robot_state_;

    // Update the state of each module
    SUSwerveDriveModuleState front_left_measured_state = front_left_module_.updateState({desired_modules_state_(0), desired_modules_state_(1)}, period);
    SUSwerveDriveModuleState front_right_measured_state = front_right_module_.updateState({desired_modules_state_(2), desired_modules_state_(3)}, period);
    SUSwerveDriveModuleState back_left_measured_state = back_left_module_.updateState({desired_modules_state_(4), desired_modules_state_(5)}, period);
    SUSwerveDriveModuleState back_right_measured_state = back_right_module_.updateState({desired_modules_state_(6), desired_modules_state_(7)}, period);

    // Use the feedback from each module to calculate the state of the robot
    measured_modules_state_ << front_left_measured_state.x_vel, front_left_measured_state.y_vel,
        front_right_measured_state.x_vel, front_right_measured_state.y_vel,
        back_left_measured_state.x_vel, back_left_measured_state.y_vel,
        back_right_measured_state.x_vel, back_right_measured_state.y_vel;

    measured_robot_state_ = module_positions_matrix_pinv_ * measured_modules_state_;

    return {measured_robot_state_(0), measured_robot_state_(1), measured_robot_state_(2)};
}

#endif