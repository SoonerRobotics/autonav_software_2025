#ifndef SU_SWERVE_MODULE_H
#define SU_SWERVE_MODULE_H

#include <ACAN2515.h>
#include "CanSparkMax.h"
#include "AbsoluteEncoder.h"
#include "QuadratureEncoder.h"

typedef struct
{
    // Module position
    double x_pos; // (meters)
    double y_pos; // (meters)

    // SPARK Max IDs
    int drive_motor_id;
    int angle_motor_id;

    // Encoder IDs
    int drive_encoder_pins[2];
    int angle_encoder_id;

    // Module Config
    bool is_drive_motor_reversed;
    bool is_angle_motor_reversed;
} SUSwerveDriveModuleConfig;

typedef struct
{
    double x_vel;
    double y_vel;
} SUSwerveDriveModuleState;

class SUSwerveDriveModule
{

public:
    SUSwerveDriveModule(SUSwerveDriveModuleConfig config, ACAN2515 *can_driver);

    // Update the state of the module
    // state: The desired state of the robot
    // returns: The measured state of the module
    SUSwerveDriveModuleState updateState(SUSwerveDriveModuleState desired_state, double period);

    void applyConversionFactor(double drive_motor_reduction, double angle_motor_reduction, double wheel_radius);

private:
    SUSwerveDriveModuleConfig config_;

    CanSparkMax drive_motor_;
    CanSparkMax angle_motor_;

    QuadratureEncoder drive_encoder_;
    AbsoluteEncoder angle_encoder_;

    double drive_motor_conversion_factor_;
    double angle_motor_conversion_factor_;
};

inline SUSwerveDriveModule::SUSwerveDriveModule(SUSwerveDriveModuleConfig config, ACAN2515 *can_driver) : config_(config), drive_motor_(can_driver, config.drive_motor_id), angle_motor_(can_driver, config.angle_motor_id),
                                                                                                          drive_encoder_(config.drive_encoder_pins), angle_encoder_(config.angle_encoder_id) {}

inline void SUSwerveDriveModule::applyConversionFactor(double driveMotorGearRatio, double angleMotorGearRatio, double wheel_radius)
{
    // TODO: This is probably not the correct way to calculate the conversion factor
    drive_motor_conversion_factor_ = wheel_radius * driveMotorGearRatio;
    angle_motor_conversion_factor_ = angleMotorGearRatio;
}

inline SUSwerveDriveModuleState SUSwerveDriveModule::updateState(SUSwerveDriveModuleState desired_state, double period)
{
    // Compute desired heading as angle of desired_vel (x forward, y left)
    // Compute desired speed as magnitude of desired_vel
    double desired_drive_speed = sqrt(desired_state.x_vel * desired_state.x_vel + desired_state.y_vel * desired_state.y_vel);
    double desired_angle = atan2(desired_state.x_vel, desired_state.y_vel);

    // TODO: Implement PIDF control to set the module to the desired state
    drive_motor_.setPWM(0);
    angle_motor_.setPWM(0);

    // Update encoders and calculate measured state
    drive_encoder_.update(period);
    angle_encoder_.update(period);

    double measured_drive_speed = drive_encoder_.getVelocity() * drive_motor_conversion_factor_;
    double measured_angle = angle_encoder_.getAngle() * angle_motor_conversion_factor_;

    SUSwerveDriveModuleState measured_state;
    measured_state.x_vel = measured_drive_speed * cos(measured_angle);
    measured_state.y_vel = measured_drive_speed * sin(measured_angle);

    return measured_state;
}

#endif