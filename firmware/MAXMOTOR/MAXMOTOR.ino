#include <ACAN2515.h>
// #include "SUSwerveModule.h"
#include "SUSwerveDrive.h"
#include "Config.h"
#include "time.h"
#include "common.h"
#include "CONBus.h"
#include "CANBusDriver.h"

// CAN
static const byte MCP2515_SCK = 18;
static const byte MCP2515_MOSI = 19;
static const byte MCP2515_MISO = 16;
static const byte MCP2515_CS = 17;
static const byte MCP2515_INT = 22;

// Quadature Encoder
static const byte MOTOR1_ENC1 = 9; // 1A
static const byte MOTOR1_ENC2 = 8; // 1B

static const byte MOTOR2_ENC1 = 1; // 2A
static const byte MOTOR2_ENC2 = 0; // 2B

static const byte MOTOR3_ENC1 = 2; // 3A
static const byte MOTOR3_ENC2 = 3; // 3B

static const byte MOTOR4_ENC1 = 7; // 4A
static const byte MOTOR4_ENC2 = 6; // 4B

// MCP2515 Driver objects
ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT);
ACAN2515 can_motor (Config::MCP2515_CS_MOTOR, SPI1, Config::MCP2515_INT_MOTOR);

static const uint32_t QUARTZ_FREQUENCY = 16UL * 1000UL * 1000UL ; // 16 MHz

/*
  Add timer
*/
double period = 1.0;

CANMessage frame;
CANMessage outFrame;

CONBus::CONBus conbus;
CONBus::CANBusDriver conbus_can(conbus,0x10);

SUSwerveDriveConfig config;
SUSwerveDrive SwerveDrive(config, &can_motor);
SUSwerveDriveState initial_state = {0.0,0.0,0.0};
SUSwerveDriveState measured_state;

robotStatus_t roboStatus;
distance motorDistances;
MotorCommand motorCommand;

// int motor_updates_in_deltaodom = 0;
// uint32_t motor_updates_between_deltaodom = 3;
// bool canBlinky = false;

bool useObstacleAvoidance = false;
uint32_t collisonBoxDist = 20;
bool isDetectingObstacle = false;
bool sendStatistics = true;

// float delta_x = 0;
// float delta_y = 0;
// float delta_theta = 0;

float desired_forward_velocity;
float desired_angular_velocity;

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(MOTOR1_ENC1, INPUT);
  pinMode(MOTOR1_ENC2, INPUT);
  pinMode(MOTOR2_ENC1, INPUT);
  pinMode(MOTOR2_ENC2, INPUT);
  pinMode(MOTOR3_ENC1, INPUT);
  pinMode(MOTOR3_ENC2, INPUT);
  pinMode(MOTOR4_ENC1, INPUT);
  pinMode(MOTOR4_ENC2, INPUT);

  /*
    Encoder interrupts here
  */

  // motor_update_timer.start();

  configureCan();

  // conbus.addReadOnlyRegister(0x00, drivetrain.getUpdatePeriod());
  // conbus.addRegister(0x01, drivetrain.getPulsesPerRadian());
  conbus.addRegister(0x02, &config.wheelRadius);
  // conbus.addRegister(0x03, drivetrain.getWheelbaseLength());
  // conbus.addRegister(0x04, drivetrain.getSlewRateLimit());
  // conbus.addRegister(0x05, drivetrain.getSwerveMod1QuadratureEncoderFactor());
  // conbus.addRegister(0x06, drivetrain.getSwerveMod2QuadratureEncoderFactor());


  // conbus.addRegister(0x10, drivetrain.getVelocitykP());
  // conbus.addRegister(0x11, drivetrain.getVelocitykI());
  // conbus.addRegister(0x12, drivetrain.getVelocitykD());
  // conbus.addRegister(0x13, drivetrain.getVelocitykF());

  // conbus.addRegister(0x20, drivetrain.getAngularkP());
  // conbus.addRegister(0x21, drivetrain.getAngularkI());
  // conbus.addRegister(0x22, drivetrain.getAngularkD());
  // conbus.addRegister(0x23, drivetrain.getAngularkF());

  conbus.addRegister(0x30, &useObstacleAvoidance);
  conbus.addRegister(0x31, &collisonBoxDist);

  conbus.addRegister(0x40, &sendStatistics);

  // conbus.addRegister(0x50, &motor_updates_between_deltaodom); 

}

void loop() {
  // put your main code here, to run repeatedly:

}

void configureCan() {
  // Set CAN
  SPI.setSCK(MCP2515_SCK);
  SPI.setTX(MCP2515_MOSI);
  SPI.setRX(MCP2515_MISO);
  SPI.setCS(MCP2515_CS);
  SPI.begin();

  // Set Motor CAN
  SPI1.setSCK(Config::MCP2515_SCK_MOTOR);
  SPI1.setTX(Config::MCP2515_MOSI_MOTOR);
  SPI1.setRX(Config::MCP2515_MISO_MOTOR);
  SPI1.setCS(Config::MCP2515_CS_MOTOR);
  SPI1.begin();

  /*
    potential to increase bit rate with the 16MHz crystal
  */
  ACAN2515Settings settings(QUARTZ_FREQUENCY, 100UL * 1000UL);  // CAN bit rate 100 kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode ; // Select Normal mode

  ACAN2515Settings settings_motor(QUARTZ_FREQUENCY, 1000UL * 1000UL);  // CAN bit rate 1 Mb/s
  settings_motor.mRequestedMode = ACAN2515Settings::NormalMode ; // Select Normal mode

  const uint16_t errorCode = can.begin(settings, onCanReceive );
  const uint16_t errorCodeMotor = can_motor.begin(settings_motor, onCanReceiveMotor );
  if (errorCode == 0) {
    Serial.println("CAN Configured");
  }
  else{
    Serial.print("Error: ");
    Serial.println(errorCode);
  }
  if (errorCodeMotor == 0) {
    Serial.println("CAN Configured");
  }
  else{
    Serial.print("Error motor: ");
    Serial.println(errorCode);
  }
}

void onCanReceive() {
  can.isr();

  if (!can.available()) {
    return;
  }

  can.receive(frame);  

  conbus_can.readCanMessage(frame.id, frame.data);

  switch (frame.id) {  
    case 10:
      motorCommand = *(MotorCommand*)(frame.data);


      desired_forward_velocity = (float)motorCommand.setpoint_forward_velocity / SPEED_SCALE_FACTOR;
      desired_angular_velocity = (float)motorCommand.setpoint_angular_velocity / SPEED_SCALE_FACTOR;


      if (useObstacleAvoidance && isDetectingObstacle && desired_forward_velocity > 0) {
        desired_forward_velocity = 0;
      }
      // drivetrain.setOutput(desired_forward_velocity, desired_angular_velocity);
      measured_state = SwerveDrive.updateState(initial_state, period);
      
      break;
    case 20:
      isDetectingObstacle = (frame.data[1] < collisonBoxDist || frame.data[2] < collisonBoxDist || frame.data[3] < collisonBoxDist);

      if (useObstacleAvoidance && isDetectingObstacle && desired_forward_velocity > 0) {
        desired_forward_velocity = 0;
      }
      // drivetrain.setOutput(desired_forward_velocity, desired_angular_velocity);
      measured_state = SwerveDrive.updateState(initial_state, period);
      break;
  }
  
}

void onCanReceiveMotor() {
  can_motor.isr();

  if (!can_motor.available()) {
    return;
  }

  can_motor.receive(frame);
  // conbus and other stuff
}

void sendCanOdomMsgOut(){
  outFrame.id = ODOM_OUT_ID;
  outFrame.len = ODOM_OUT_LEN;

  motorDistances.xn = initial_state.x_vel * ODOM_SCALE_FACTOR;
  motorDistances.yn = initial_state.y_vel * ODOM_SCALE_FACTOR;
  motorDistances.on = initial_state.angular_vel * ODOM_SCALE_FACTOR;

  memcpy(outFrame.data, &motorDistances, ODOM_OUT_LEN );
  can.tryToSend (outFrame) ;

  // if (sendStatistics) {
  //   drivetrain.getSetpoints(pid_setpoints);
  //   outFrame.id = 50;
  //   outFrame.len = 8;
  //   memcpy(outFrame.data, &pid_setpoints, 8);
  //   can.tryToSend(outFrame);

  //   drivetrain.getControl(pid_controls);
  //   outFrame.id = 51;
  //   outFrame.len = 4;
  //   memcpy(outFrame.data, &pid_controls, 4);
  //   can.tryToSend(outFrame);
  // }

}

// void resetDelta(){
//   motorDistances.xn = 0;
//   motorDistances.yn = 0;
//   motorDistances.on = 0;
//   delta_x = 0;
//   delta_y = 0;
//   delta_theta = 0;
// }

// void updateTimers() {
//   motor_update_timer.update();
// }
