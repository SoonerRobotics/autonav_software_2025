#include <SPI.h>
#include <ACAN2515.h>
#include "SUSwerve/SUSwerveDrive.h"
#include "Config.h"

ACAN2515 motorsCAN(Config::MCP2515_CS_MOTOR, SPI1, Config::MCP2515_INT_MOTOR);

SUSwerveDrive swerve(Config::swerve_config, &motorsCAN);

void setup() {}

void loop() {}
