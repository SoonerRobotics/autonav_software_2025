#ifndef SWERVE_DRIVE_H
#define SWERVE_DRIVE_H

#include "motor_with_encoder.h"
#include "common.h"


class SwerveDrive {

public:
    SwerveDrive(MotorWithEncoder swerve_mod1, MotorWithEncoder swerve_mod2, MotorWithEncoder swerve_mod3, MotorWithEncoder swerve_mod4, float update_period);
    // void setup();

    void setOutput(float forward_velocity, float angular_velocity);

    void pulseSwerveModuleOneEncoder();
    void pulseSwerveModuleTwoEncoder();
    void pulseSwerveModuleThreeEncoder();
    void pulseSwerveModuleFourEncoder();

    void updateState(float& delta_x_out, float& delta_y_out, float& delta_theta_out);

    void getSetpoints(PIDSetpoints& pid_setpoints);
    void getControl(PIDControl& pid_control);

    float* getUpdatePeriod();
    float* getPulsesPerRadian();
    float* getWheelRadius();
    float* getWheelbaseLength();
    float* getSlewRateLimit();
    float* getSwerveMod1QuadratureEncoderFactor();
    float* getSwerveMod2QuadratureEncoderFactor();
    float* getSwerveMod3QuadratureEncoderFactor();
    float* getSwerveMod4QuadratureEncoderFactor();
    float* getSwerveMod1AbsoluteEncoderFactor();
    float* getSwerveMod2AbsoluteEncoderFactor();
    float* getSwerveMod3AbsoluteEncoderFactor();
    float* getSwerveMod4AbsoluteEncoderFactor();

    float* getVelocitykP();
    float* getVelocitykI();
    float* getVelocitykD();
    float* getVelocitykF();

    float* getAngularkP();
    float* getAngularkI();
    float* getAngularkD();
    float* getAngularkF();

private:
    MotorWithEncoder swerve_mod1_;
    MotorWithEncoder swerve_mod2_;
    MotorWithEncoder swerve_mod3_;
    MotorWithEncoder swerve_mod4_;

    float update_period_ = 0.025f;
    float pulses_per_radian_ = 600.0f;
    float wheel_radius_ = 0.19f;
    float wheelbase_length_ = 0.575f;

    // input the physical values of the robot
    float length = 1.0f;
    float width = 1.0f;

    float swerve_mod1_quadrature_factor_ = 1.00f;
    float swerve_mod2_quadrature_factor_ = 1.00f;
    float swerve_mod3_quadrature_factor_ = 1.00f;
    float swerve_mod4_quadrature_factor_ = 1.00f;
    float swerve_mod1_angle_factor_ = 0;
    float swerve_mod2_angle_factor_ = 0;
    float swerve_mod3_angle_factor_ = 0;
    float swerve_mod4_angle_factor_ = 0;

    float forward_velocity_setpoint_;
    float angular_velocity_setpoint_;

    float velocity_estimate;
    float angular_estimate;

    // float left_motor_output;
    // float right_motor_output;
    float swerve_mod1_output_drive;
    float swerve_mod2_output_drive;
    float swerve_mod3_output_drive;
    float swerve_mod4_output_drive;

    float swerve_mod1_output_rotate;
    float swerve_mod2_output_rotate;
    float swerve_mod3_output_rotate;
    float swerve_mod4_output_rotate;

    float computeVelocityPID_(float velocity_setpoint, float velocity_current);
    float velocity_kP_ = 0.1;
    float velocity_kI_ = 1.0;
    float velocity_kD_ = 0;
    float velocity_kF_ = 0.15;
    float velocity_integrator_ = 0;
    float velocity_previous_error_ = 0;

    float computeAngularPID_(float angular_setpoint, float angular_current);
    float angular_kP_ = 0.1;
    float angular_kI_ = 0.4;
    float angular_kD_ = 0;
    float angular_kF_ = 0.15;
    float angular_integrator_ = 0;
    float angular_previous_error_ = 0;

    float pulsesToRadians_(int pulses);

    float slewLimit_(float current_output, float desired_output);
    float slew_rate_limit_ = 0.05;
};

// inline DifferentialDrive::DifferentialDrive(MotorWithEncoder left_motor, MotorWithEncoder right_motor, float update_period)
//     : update_period_(update_period), left_motor_(left_motor), right_motor_(right_motor) {}
inline SwerveDrive::SwerveDrive(MotorWithEncoder swerve_mod1, MotorWithEncoder swerve_mod2, MotorWithEncoder swerve_mod3, MotorWithEncoder swerve_mod4, float update_period)
    : swerve_mod1_(swerve_mod1), swerve_mod2_(swerve_mod2), swerve_mod3_(swerve_mod3), swerve_mod4_(swerve_mod4), update_period_(update_period) {}

// inline void DifferentialDrive::setup() {
//   // left_motor_.setup();
//   // right_motor_.setup();
//   swerve_mod1_.setup();
//   swerve_mod2_.setup();
//   swerve_mod3_.setup();
//   swerve_mod4_.setup();
// }

inline void SwerveDrive::setOutput(float forward_velocity, float angular_velocity) {
    forward_velocity_setpoint_ = forward_velocity;
    angular_velocity_setpoint_ = angular_velocity;
}

inline void SwerveDrive::pulseSwerveModuleOneEncoder() {
    swerve_mod1_.pulseEncoder();
    swerve_mod1_.absoluteEncoder();
}

inline void SwerveDrive::pulseSwerveModuleTwoEncoder() {
    swerve_mod2_.pulseEncoder();
    swerve_mod2_.absoluteEncoder();
}

inline void SwerveDrive::pulseSwerveModuleThreeEncoder() {
    swerve_mod3_.pulseEncoder();
    swerve_mod3_.absoluteEncoder();
}

inline void SwerveDrive::pulseSwerveModuleFourEncoder() {
    swerve_mod4_.pulseEncoder();
    swerve_mod4_.absoluteEncoder();
}

inline void SwerveDrive::updateState(float& delta_x_out, float& delta_y_out, float& delta_theta_out) {
    float swerve_mod1_angular_distance = swerve_mod1_.getPulses() / pulses_per_radian_ * swerve_mod1_quadrature_factor_;
    float swerve_mod2_angular_distance = swerve_mod2_.getPulses() / pulses_per_radian_ * swerve_mod2_quadrature_factor_;
    float swerve_mod3_angular_distance = swerve_mod3_.getPulses() / pulses_per_radian_ * swerve_mod3_quadrature_factor_;
    float swerve_mod4_angular_distance = swerve_mod4_.getPulses() / pulses_per_radian_ * swerve_mod4_quadrature_factor_;

    float swerve_mod1_angle = M_PI / 180 * (swerve_mod1_.absoluteEncoder() - swerve_mod1_angle_factor_);
    float swerve_mod2_angle = M_PI / 180 * (swerve_mod2_.absoluteEncoder() - swerve_mod2_angle_factor_);
    float swerve_mod3_angle = M_PI / 180 * (swerve_mod3_.absoluteEncoder() - swerve_mod3_angle_factor_);
    float swerve_mod4_angle = M_PI / 180 * (swerve_mod4_.absoluteEncoder() - swerve_mod4_angle_factor_);

    float vector_swerve_mod1x = swerve_mod1_angular_distance*cos(swerve_mod1_angle); 
    float vector_swerve_mod2x = swerve_mod2_angular_distance*cos(swerve_mod2_angle);
    float vector_swerve_mod3x = swerve_mod3_angular_distance*cos(swerve_mod3_angle);
    float vector_swerve_mod4x = swerve_mod4_angular_distance*cos(swerve_mod4_angle);
    float vx_sum = vector_swerve_mod1x + vector_swerve_mod2x + vector_swerve_mod3x + vector_swerve_mod4x;

    float vector_swerve_mod1y = swerve_mod1_angular_distance*sin(swerve_mod1_angle); 
    float vector_swerve_mod2y = swerve_mod2_angular_distance*sin(swerve_mod2_angle);
    float vector_swerve_mod3y = swerve_mod3_angular_distance*sin(swerve_mod3_angle);
    float vector_swerve_mod4y = swerve_mod4_angular_distance*sin(swerve_mod4_angle);
    float vy_sum = vector_swerve_mod1y + vector_swerve_mod2y + vector_swerve_mod3y + vector_swerve_mod4y;

    float distance_estimate = wheel_radius_ * sqrt(vx_sum * vx_sum + vy_sum * vy_sum);
    float rotation_estimate = 2.0f * wheel_radius_ * (vector_swerve_mod1x - vx_sum) / length; //probably need to fix this

    velocity_estimate = distance_estimate / update_period_;
    angular_estimate = rotation_estimate / update_period_;

    float velocity_control = computeVelocityPID_(forward_velocity_setpoint_, velocity_estimate);
    float angular_control = computeAngularPID_(angular_velocity_setpoint_, angular_estimate);

    if (abs(forward_velocity_setpoint_) < 0.05 && abs(angular_velocity_setpoint_) < 0.05 && abs(velocity_estimate) < 0.05 && abs(angular_estimate) < 0.05) {
      // estop fix
      velocity_integrator_ = 0;
      angular_integrator_ = 0;

      swerve_mod1_output_drive = 0;
      swerve_mod2_output_drive = 0;
      swerve_mod3_output_drive = 0;
      swerve_mod4_output_drive = 0;

      swerve_mod1_output_rotate = 0;
      swerve_mod2_output_rotate = 0;
      swerve_mod3_output_rotate = 0;
      swerve_mod4_output_rotate = 0;
    } else {
      // swerve_mod1_output = slewLimit_(swerve_mod1_output, velocity_control + angular_control);
      // swerve_mod2_output = slewLimit_(swerve_mod2_output, velocity_control + angular_control);
      // swerve_mod3_output = slewLimit_(swerve_mod3_output, velocity_control + angular_control);
      // swerve_mod4_output = slewLimit_(swerve_mod4_output, velocity_control + angular_control);

      swerve_mod1_output_drive = slewLimit_(swerve_mod1_output_drive, velocity_control);
      swerve_mod2_output_drive = slewLimit_(swerve_mod2_output_drive, velocity_control);
      swerve_mod3_output_drive = slewLimit_(swerve_mod3_output_drive, velocity_control);
      swerve_mod4_output_drive = slewLimit_(swerve_mod4_output_drive, velocity_control);

      swerve_mod1_output_rotate = slewLimit_(swerve_mod1_output_rotate, angular_control);
      swerve_mod2_output_rotate = slewLimit_(swerve_mod2_output_rotate, angular_control);
      swerve_mod3_output_rotate = slewLimit_(swerve_mod3_output_rotate, angular_control);
      swerve_mod4_output_rotate = slewLimit_(swerve_mod4_output_rotate, angular_control);

    }

    swerve_mod1_.setOutput(swerve_mod1_output_drive, swerve_mod1_output_rotate);
    swerve_mod2_.setOutput(swerve_mod2_output_drive, swerve_mod2_output_rotate);
    swerve_mod3_.setOutput(swerve_mod3_output_drive, swerve_mod3_output_rotate);
    swerve_mod4_.setOutput(swerve_mod4_output_drive, swerve_mod4_output_rotate);

    float estimated_theta = delta_theta_out + 0.5 * rotation_estimate;
    delta_x_out += distance_estimate * cos(estimated_theta);
    delta_y_out += distance_estimate * sin(estimated_theta);
    delta_theta_out += rotation_estimate;
}

inline float SwerveDrive::pulsesToRadians_(int pulses) {
    return (int)(pulses);
}

inline float SwerveDrive::slewLimit_(float current_output, float desired_output) {
    if (desired_output - current_output > slew_rate_limit_) {
      return current_output + slew_rate_limit_;
    } else if (desired_output - current_output < -slew_rate_limit_) {
      return current_output - slew_rate_limit_;
    } else {
      return desired_output;
    }
}

inline float SwerveDrive::computeVelocityPID_(float velocity_setpoint, float velocity_current) {
    float velocity_error = velocity_setpoint - velocity_current;

    velocity_integrator_ += velocity_error * update_period_;
    float velocity_derivative = (velocity_error - velocity_previous_error_) / update_period_;

    velocity_previous_error_ = velocity_error;

    return velocity_kP_ * velocity_error + velocity_kI_ * velocity_integrator_ + velocity_kD_ * velocity_derivative + velocity_kF_ * velocity_setpoint;
}

inline float SwerveDrive::computeAngularPID_(float angular_setpoint, float angular_current) {
    float angular_error = angular_setpoint - angular_current;

    angular_integrator_ += angular_error * update_period_;
    float angular_derivative = (angular_error - angular_previous_error_) / update_period_;

    angular_previous_error_ = angular_error;

    return angular_kP_ * angular_error + angular_kI_ * angular_integrator_ + angular_kD_ * angular_derivative + angular_kF_ * angular_setpoint;
}

inline float* SwerveDrive::getUpdatePeriod() {
    return &update_period_;
}

inline float* SwerveDrive::getPulsesPerRadian() {
    return &pulses_per_radian_;
}

inline float* SwerveDrive::getWheelRadius() {
    return &wheel_radius_;
}

inline float* SwerveDrive::getWheelbaseLength() {
    return &wheelbase_length_;
}

inline float* SwerveDrive::getSwerveMod1QuadratureEncoderFactor() {
    return &swerve_mod1_quadrature_factor_;
}

inline float* SwerveDrive::getSwerveMod2QuadratureEncoderFactor() {
    return &swerve_mod2_quadrature_factor_;
}

inline float* SwerveDrive::getSwerveMod3QuadratureEncoderFactor() {
    return &swerve_mod3_quadrature_factor_;
}

inline float* SwerveDrive::getSwerveMod4QuadratureEncoderFactor() {
    return &swerve_mod4_quadrature_factor_;
}

inline float* SwerveDrive::getSwerveMod1AbsoluteEncoderFactor() {
    return &swerve_mod1_angle_factor_;
}

inline float* SwerveDrive::getSwerveMod2AbsoluteEncoderFactor() {
    return &swerve_mod2_angle_factor_;
}

inline float* SwerveDrive::getSwerveMod3AbsoluteEncoderFactor() {
    return &swerve_mod3_angle_factor_;
}

inline float* SwerveDrive::getSwerveMod4AbsoluteEncoderFactor() {
    return &swerve_mod4_angle_factor_;
}

inline float* SwerveDrive::getSlewRateLimit() {
    return &slew_rate_limit_;
}

inline float* SwerveDrive::getVelocitykP() {
    return &velocity_kP_;
}

inline float* SwerveDrive::getVelocitykI() {
    return &velocity_kI_;
}

inline float* SwerveDrive::getVelocitykD() {
    return &velocity_kD_;
}

inline float* SwerveDrive::getVelocitykF() {
    return &velocity_kF_;
}

inline float* SwerveDrive::getAngularkP() {
    return &angular_kP_;
}

inline float* SwerveDrive::getAngularkI() {
    return &angular_kI_;
}

inline float* SwerveDrive::getAngularkD() {
    return &angular_kD_;
}

inline float* SwerveDrive::getAngularkF() {
    return &angular_kF_;
}

inline void SwerveDrive::getSetpoints(PIDSetpoints& pid_setpoints) {
  pid_setpoints.forward_current = velocity_estimate * 1000;
  pid_setpoints.forward_setpoint = forward_velocity_setpoint_ * 1000;
  pid_setpoints.angular_current = angular_estimate * 1000;
  pid_setpoints.angular_setpoint = angular_velocity_setpoint_ * 1000;
}

// inline void DifferentialDrive::getControl(PIDControl& pid_control) {
//   pid_control.left_motor_output = left_motor_output * 1000;
//   pid_control.right_motor_output = right_motor_output * 1000;
// }

inline void SwerveDrive::getControl(PIDControl& pid_control) {
  pid_control.swerve_mod1_output_drive = swerve_mod1_output_drive * 1000;
  pid_control.swerve_mod2_output_drive = swerve_mod2_output_drive * 1000;
  pid_control.swerve_mod3_output_drive = swerve_mod3_output_drive * 1000;
  pid_control.swerve_mod4_output_drive = swerve_mod4_output_drive * 1000;

  pid_control.swerve_mod1_output_rotate = swerve_mod1_output_rotate * 1000;
  pid_control.swerve_mod2_output_rotate = swerve_mod2_output_rotate * 1000;
  pid_control.swerve_mod3_output_rotate = swerve_mod3_output_rotate * 1000;
  pid_control.swerve_mod4_output_rotate = swerve_mod4_output_rotate * 1000;
}

#endif