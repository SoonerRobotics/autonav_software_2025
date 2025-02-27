#include <SPI.h>
#include "time.h"
#include <Wire.h>
#include <ACAN2515.h>
#include "CONBus.h"
#include "CANBusDriver.h"
#include "common.h"
#include "swerve_drive.h"
#include "motor_with_encoder.h"
#include "CANBusDriver.h"

// CAN
static const byte MCP2515_SCK = 18;
static const byte MCP2515_MOSI = 19;
static const byte MCP2515_MISO = 16;
static const byte MCP2515_CS = 17;
static const byte MCP2515_INT = 22;

// Motor CAN
static const byte MCP2515_SCK_MOTOR = 14;
static const byte MCP2515_MOSI_MOTOR = 15;
static const byte MCP2515_MISO_MOTOR = 12;
static const byte MCP2515_CS_MOTOR = 13;
static const byte MCP2515_INT_MOTOR = 11;

// Motor controller addresses
static const uint32_t MOTOR1_DRIVE = 0x02052C81;
static const uint32_t MOTOR1_ROTATE = 0x02052C8B;
static const uint32_t MOTOR2_DRIVE = 0x02052C82;
static const uint32_t MOTOR2_ROTATE = 0x02052C8C;
static const uint32_t MOTOR3_DRIVE = 0x02052C83;
static const uint32_t MOTOR3_ROTATE = 0x02052C8D;
static const uint32_t MOTOR4_DRIVE = 0x02052C84;
static const uint32_t MOTOR4_ROTATE = 0x02052C8E;

// Absoute Encoder I2C
static const byte SDA_ENC = 20;
static const byte SCL_ENC = 21;
static const byte MCP3428_ADDR = 0b1101000;
static const byte MCP3428_CONFIG1 = 0b11110000;
static const byte MCP3428_CONFIG2 = 0b10110000;
static const byte MCP3428_CONFIG3 = 0b10010000;
static const byte MCP3428_CONFIG4 = 0b11010000;

// Quadature Encoder
static const byte MOTOR1_ENC1 = 9;
static const byte MOTOR1_ENC2 = 8;

static const byte MOTOR2_ENC1 = 1;
static const byte MOTOR2_ENC2 = 0;

static const byte MOTOR3_ENC1 = 2;
static const byte MOTOR3_ENC2 = 3;

static const byte MOTOR4_ENC1 = 7;
static const byte MOTOR4_ENC2 = 6;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Driver objects
//——————————————————————————————————————————————————————————————————————————————

ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT);
ACAN2515 can_motor (MCP2515_CS_MOTOR, SPI1, MCP2515_INT_MOTOR);

static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ; // 8 MHz

TickTwo motor_update_timer(setMotorUpdateFlag, 25);

CANMessage frame;
CANMessage outFrame;

CONBus::CONBus conbus;
CONBus::CANBusDriver conbus_can(conbus,//0xid)

MotorWithEncoder SwerveMod1(can_motor, MOTOR1_DRIVE, MOTOR1_ROTATE, MOTOR1_ENC1, MOTOR1_ENC2, MCP3428_CONFIG1, false);
MotorWithEncoder SwerveMod2(can_motor, MOTOR2_DRIVE, MOTOR2_ROTATE, MOTOR2_ENC1, MOTOR2_ENC2, MCP3428_CONFIG2, false);
MotorWithEncoder SwerveMod3(can_motor, MOTOR3_DRIVE, MOTOR3_ROTATE, MOTOR3_ENC1, MOTOR3_ENC2, MCP3428_CONFIG3, false);
MotorWithEncoder SwerveMod4(can_motor, MOTOR4_DRIVE, MOTOR4_ROTATE, MOTOR4_ENC1, MOTOR4_ENC2, MCP3428_CONFIG4, false);

SwerveDrive drivetrain(SwerveMod1, SwerveMod2, SwerveMod3, SwerveMod4, 0.025);

void onCanReceive() {
  can.isr();

  if (!can.available()) {
    return;
  }

  can.receive(Frame);
  // conbus and other stuff
}

int motor_updates_in_deltaodom = 0;
uint32_t motor_updates_between_deltaodom = 3;
bool canBlinky = false;

bool useObstacleAvoidance = false;
uint32_t collisonBoxDist = 20;
bool isDetectingObstacle = false;

bool sendStatistics = true;

float delta_x = 0;
float delta_y = 0;
float delta_theta = 0;

float desired_forward_velocity;
float desired_angular_velocity;

PIDSetpoints pid_setpoints;
PIDControl pid_controls;

void setup() {
  Serial.begin(9600);

  pinMode(MOTOR1_ENC1, INPUT);
  pinMode(MOTOR1_ENC2, INPUT);
  pinMode(MOTOR2_ENC1, INPUT);
  pinMode(MOTOR2_ENC2, INPUT);
  pinMode(MOTOR3_ENC1, INPUT);
  pinMode(MOTOR3_ENC2, INPUT);
  pinMode(MOTOR4_ENC1, INPUT);
  pinMode(MOTOR4_ENC2, INPUT);

  attachInterrupt(MOTOR1_ENC1, updateSwerveMod1, CHANGE);
  attachInterrupt(MOTOR2_ENC1, updateSwerveMod2, CHANGE);
  attachInterrupt(MOTOR3_ENC1, updateSwerveMod3, CHANGE);
  attachInterrupt(MOTOR4_ENC1, updateSwerveMod4, CHANGE);

  // Set I2C
  Wire.setSDA(SDA_ENC);
  Wire.setSCL(SCL_ENC);
  Wire.begin();

  motor_update_timer.start();

  configureCan();

  conbus.addReadOnlyRegister(0x00, drivetrain.getUpdatePeriod());
  conbus.addRegister(0x01, drivetrain.getPulsesPerRadian());
  conbus.addRegister(0x02, drivetrain.getWheelRadius());
  // conbus.addRegister(0x03, drivetrain.getWheelbaseLength());
  conbus.addRegister(0x04, drivetrain.getSlewRateLimit());
  conbus.addRegister(0x05, drivetrain.getSwerveMod1EncoderFactor());
  conbus.addRegister(0x06, drivetrain.getSwerveMod2EncoderFactor());
  conbus.addRegister(0x07, drivetrain.getSwerveMod3EncoderFactor());
  conbus.addRegister(0x08, drivetrain.getSwerveMod4EncoderFactor());

  conbus.addRegister(0x10, drivetrain.getVelocitykP());
  conbus.addRegister(0x11, drivetrain.getVelocitykI());
  conbus.addRegister(0x12, drivetrain.getVelocitykD());
  conbus.addRegister(0x13, drivetrain.getVelocitykF());

  conbus.addRegister(0x20, drivetrain.getAngularkP());
  conbus.addRegister(0x21, drivetrain.getAngularkI());
  conbus.addRegister(0x22, drivetrain.getAngularkD());
  conbus.addRegister(0x23, drivetrain.getAngularkF());

  conbus.addRegister(0x30, &useObstacleAvoidance);
  conbus.addRegister(0x31, &collisonBoxDist);

  conbus.addRegister(0x40, &sendStatistics);

  conbus.addRegister(0x50, &motor_updates_between_deltaodom);
}

void loop() {
  updateTimers();
  if (MOTOR_UPDATE_FLAG) {

    drivetrain.updateState(delta_x, delta_y, delta_theta);
    motor_updates_in_deltaodom++;

    MOTOR_UPDATE_FLAG = false;
  }

  //loop1
  if (motor_updates_in_deltaodom >= motor_updates_between_deltaodom) {
    motor_updates_in_deltaodom = 0;
    sendCanOdomMsgOut();
    resetDelta();
  }

  if (conbus_can.isReplyReady()) {
    conbus_can.peekReply(outFrame.id, outFrame.len, outFrame.data);

    bool success = can.tryToSend(outFrame);

    if (success) {
      conbus_can.popReply();
    }
  }
}

void updateSwerveMod1()
{
  drivetrain.pulseSwerveModuleOneEncoder();
}

void updateSwerveMod2()
{
  drivetrain.pulseSwerveModuleTwoEncoder();
}

void updateSwerveMod3()
{
  drivetrain.pulseSwerveModuleThreeEncoder();
}

void updateSwerveMod4()
{
  drivetrain.pulseSwerveModuleFourEncoder();
}

void i2cWrite(uint8_t config)
{
  Wire.beginTransmission(MCP3428_ADDR);
  Wire.write(config);
  Wire.endTransmission();
  delay(50);
}

void i2cRead()
{
  Wire.requestFrom(MCP3428_ADDR,2);
  while (Wire.available()) {
    msb = Wire.read();
    lsb = Wire.read();
    total = (int16_t)((msb << 8)| lsb);
    Serial.print(" ADC raw: ");
    Serial.println(total);
    Serial.println(total*2.048/2048);
  }
    delay(50);
}

void configureCan() {
  // Set CAN
  SPI.setSCK(MCP2515_SCK);
  SPI.setTX(MCP2515_MOSI);
  SPI.setRX(MCP2515_MISO);
  SPI.setCS(MCP2515_CS);
  SPI.begin();

  // Set Motor CAN
  SPI1.setSCK(MCP2515_SCK_MOTOR);
  SPI1.setTX(MCP2515_MOSI_MOTOR);
  SPI1.setRX(MCP2515_MISO_MOTOR);
  SPI1.setCS(MCP2515_CS_MOTOR);
  SPI1.begin();

  ACAN2515Settings settings(QUARTZ_FREQUENCY, 100UL * 1000UL);  // CAN bit rate 100 kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode ; // Select Normal mode
  const uint16_t errorCode = can.begin(settings, onCanRecieve );
  const uint16_t errorCodeMotor = can_motor.begin(settings, onCanRecieve );
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

void onCanRecieve() {
  can.isr();

  if (!can.available()) {
    return;
  }

  can.receive(frame);  

  conbus_can.readCanMessage(frame.id, frame.data);

  // printCanMsg(frame);

  switch (frame.id) {  
    case 10:
      motorCommand = *(MotorCommand*)(frame.data);

      desired_forward_velocity = (float)motorCommand.setpoint_forward_velocity / SPEED_SCALE_FACTOR;
      desired_angular_velocity = (float)motorCommand.setpoint_angular_velocity / SPEED_SCALE_FACTOR;


      if (useObstacleAvoidance && isDetectingObstacle && desired_forward_velocity > 0) {
        desired_forward_velocity = 0;
      }

      drivetrain.setOutput(desired_forward_velocity, desired_angular_velocity);
      break;
    case 20:
      isDetectingObstacle = (frame.data[1] < collisonBoxDist || frame.data[2] < collisonBoxDist || frame.data[3] < collisonBoxDist);

      if (useObstacleAvoidance && isDetectingObstacle && desired_forward_velocity > 0) {
        desired_forward_velocity = 0;
      }

      drivetrain.setOutput(desired_forward_velocity, desired_angular_velocity);
      break;
  }
  
}

void sendCanOdomMsgOut(){
  outFrame.id = ODOM_OUT_ID;
  outFrame.len = ODOM_OUT_LEN;

  motorDistances.xn = delta_x * ODOM_SCALE_FACTOR;
  motorDistances.yn = delta_y * ODOM_SCALE_FACTOR;
  motorDistances.on = delta_theta * ODOM_SCALE_FACTOR;

  memcpy(outFrame.data, &motorDistances, ODOM_OUT_LEN );
  const bool ok = can.tryToSend (outFrame) ;

  if (sendStatistics) {
    drivetrain.getSetpoints(pid_setpoints);
    outFrame.id = 50;
    outFrame.len = 8;
    memcpy(outFrame.data, &pid_setpoints, 8);
    can.tryToSend(outFrame);

    drivetrain.getControl(pid_controls);
    outFrame.id = 51;
    outFrame.len = 4;
    memcpy(outFrame.data, &pid_controls, 4);
    can.tryToSend(outFrame);
  }

}

void resetDelta(){
  motorDistances.xn = 0;
  motorDistances.yn = 0;
  motorDistances.on = 0;
  delta_x = 0;
  delta_y = 0;
  delta_theta = 0;
}

void updateTimers() {
  motor_update_timer.update();
}

