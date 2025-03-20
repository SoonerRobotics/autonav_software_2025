#ifndef CONFIG_H
#define CONFIG_H

#include "SUSwerve/SUSwerveDrive.h"

namespace Config
{

    static const byte MCP2515_SCK_MOTOR = 14;
    static const byte MCP2515_MOSI_MOTOR = 15;
    static const byte MCP2515_MISO_MOTOR = 12;
    static const byte MCP2515_CS_MOTOR = 13;
    static const byte MCP2515_INT_MOTOR = 11;

    SUSwerveDriveModuleConfig front_left_module_config = {
        .x_pos = 0.5,
        .y_pos = 0.5,
        .drive_motor_id = 1,
        .angle_motor_id = 2,
        .drive_encoder_id = 0,
        .angle_encoder_id = 1,
        .is_drive_motor_reversed = false,
        .is_angle_motor_reversed = false};

    SUSwerveDriveModuleConfig front_right_module_config = {
        .x_pos = 0.5,
        .y_pos = -0.5,
        .drive_motor_id = 3,
        .angle_motor_id = 4,
        .drive_encoder_id = 2,
        .angle_encoder_id = 3,
        .is_drive_motor_reversed = false,
        .is_angle_motor_reversed = false};

    SUSwerveDriveModuleConfig back_left_module_config = {
        .x_pos = -0.5,
        .y_pos = 0.5,
        .drive_motor_id = 5,
        .angle_motor_id = 6,
        .drive_encoder_id = 4,
        .angle_encoder_id = 5,
        .is_drive_motor_reversed = false,
        .is_angle_motor_reversed = false};

    SUSwerveDriveModuleConfig back_right_module_config = {
        .x_pos = -0.5,
        .y_pos = -0.5,
        .drive_motor_id = 7,
        .angle_motor_id = 8,
        .drive_encoder_id = 6,
        .angle_encoder_id = 7,
        .is_drive_motor_reversed = false,
        .is_angle_motor_reversed = false};

    SUSwerveDriveConfig swerve_config = {
        .front_left = front_left_module_config,
        .front_right = front_right_module_config,
        .back_left = back_left_module_config,
        .back_right = back_right_module_config};
}

#endif