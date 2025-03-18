#ifndef SU_SWERVE_MODULE_H
#define SU_SWERVE_MODULE_H

#include "Eigen.h"
#include <ACAN2515.h>
#include "CanSparkMax.h"
#include "AbsoluteEncoder.h"

typedef struct {
    // Module position
    float x_pos;
    float y_pos;

    // SPARK Max IDs
    int drive_motor_id;
    int angle_motor_id;

    // Encoder IDs
    int drive_encoder_id;
    int angle_encoder_id;
    int i2c_config;

    // Module Config
    bool is_drive_motor_reversed;
    bool is_angle_motor_reversed;
} SUSwerveDriveModuleConfig;

typedef struct{
    float x_vel;
    float y_vel;
    float angular_vel;
} SUSwerveDriveState;

typedef struct {
    float x_vel;
    float y_vel;
} SUSwerveDriveModuleState;

class SUSwerveDriveModule {

public:
    SUSwerveDriveModule(SUSwerveDriveModuleConfig config);

    // Update the state of the module
    // state: The desired state of the robot
    // returns: The measured state of the module
    SUSwerveDriveModuleState updateState(SUSwerveDriveState state);
private:
    SUSwerveDriveModuleConfig config_;

    CanSparkMax drive_motor_;
    CanSparkMax angle_motor_;

    AbsoluteEncoder drive_encoder_;
    AbsoluteEncoder angle_encoder_;
};

inline SUSwerveDriveModule::SUSwerveDriveModule(ACAN2515* can_driver, SUSwerveDriveModuleConfig config) :
    config_(config), drive_motor_(can_driver, config.drive_motor_id), angle_motor_(can_driver, config.angle_motor_id),
    drive_encoder_(config.drive_encoder_id), angle_encoder_(config.angle_encoder_id) {}

inline SUSwerveDriveModuleState SUSwerveDriveModule::updateState(SUSwerveDriveModuleState desired_state) {
    // Compute desired heading as angle of desired_vel (x forward, y left)
    // Compute desired speed as magnitude of desired_vel
    double desired_drive_speed = sqrt(desired_state.x_vel * desired_state.x_vel + desired_state.y_vel * desired_state.y_vel);
    double desired_angle = atan2(desired_state.x_vel, desired_state.y_vel);

    // TODO: Implement PIDF control to set the module to the desired state
    drive_motor_.set(0);
    angle_motor_.set(0);

    // Update encoders and calculate measured state
    drive_encoder_.update();
    angle_encoder_.update();

    double measured_drive_speed = drive_encoder_.getVelocity();
    double measured_angle = angle_encoder_.getAngle();

    SUSwerveDriveModuleState measured_state;
    measured_state.x_vel = measured_drive_speed * cos(measured_angle);
    measured_state.y_vel = measured_drive_speed * sin(measured_angle);

    return measured_state;
}

#endif