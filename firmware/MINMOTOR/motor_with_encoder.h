#ifndef MOTOR_WITH_ENCODER_H
#define MOTOR_WITH_ENCODER_H

#include <stdio.h>
#include <Wire.h>
#include <ACAN2515.h>
#include "common.h"

class MotorWithEncoder {

public:
    MotorWithEncoder(ACAN2515 &can_motor, uint32_t drive, uint32_t rotate, uint8_t pin_Encoder_QA, uint8_t pin_Encoder_QB, uint8_t pin_Encoder_Abs, bool reversed);

    void setup();

    void pulseEncoder();
    int getPulses();
    int absoluteEncoder();

    void setOutput(float control_drive, float control_rotate);
    void setMinOutput(float minControl);

private:
    float minControl_ = 0.01f;
    float maxControl_ = 0.90f;

    //const uint8_t pin_PWM_Control_;
    ACAN2515 *can_motor_;
    const uint32_t drive_;
    const uint32_t rotate_;
    const uint8_t pin_Encoder_QA_;
    const uint8_t pin_Encoder_QB_;
    const uint8_t pin_Encoder_Abs_;
    const bool reversed_;

    const uint32_t motor_address_enable = 0x02052C80;
    int encoder_pulses_ = 0;
    int previous_Encoder_State_ = 0;

    void i2cWrite(uint8_t config);
    int i2cRead();
    const uint8_t MCP3428_ADDR_ = 0b1101000;
};

inline MotorWithEncoder::MotorWithEncoder(ACAN2515 &can_motor, uint32_t drive, uint32_t rotate, uint8_t pin_Encoder_QA, uint8_t pin_Encoder_QB, uint8_t pin_Encoder_Abs, bool reversed)
    : can_motor_(&can_motor), drive_(drive), rotate_(rotate), pin_Encoder_QA_(pin_Encoder_QA), pin_Encoder_QB_(pin_Encoder_QB), pin_Encoder_Abs_(pin_Encoder_Abs), reversed_(reversed) {}


inline void MotorWithEncoder::setOutput(float control_drive, float control_rotate) {
    CANMessage outframe;
    float rotate_factor = 0.5f;
    outframe.ext = true;
    control_rotate = rotate_factor * control_rotate;

    float CANOutD = 0; // Default to braked
    float CANOutR = 0;

    if (control_drive > 1) {
      control_drive = 1;
    }

    if (control_drive < -1) {
      control_drive = -1;
    }

    if (reversed_) {
        control_drive = -control_drive;
    }

    if (abs(control_drive) > maxControl_) { 
      control_drive = abs(control_drive) / control_drive * maxControl_;
    }

    // If we are greater than our min control signal, set the servo output
    if ((control_drive > 0 && control_drive > minControl_) || (control_drive < 0 && control_drive < -minControl_)) {
        CANOutD = control_drive * 0.85;
    }

    if (control_rotate > 1) {
      control_drive = 1;
    }

    if (control_rotate < -1) {
      control_drive = -1;
    }

    if (abs(control_rotate) > maxControl_) { 
      control_rotate = abs(control_rotate) / control_rotate * maxControl_;
    }

    // If we are greater than our min control signal, set the servo output
    if ((control_rotate > 0 && control_rotate > minControl_) || (control_rotate < 0 && control_rotate < -minControl_)) {
        CANOutR = control_rotate * 0.85;
    }

    outframe.id = drive_;
    outframe.dataFloat[0] = CANOutD;
    bool ok = can_motor_->tryToSend(outframe);
    outframe.id = rotate_;
    outframe.dataFloat[0] = CANOutR;
    can_motor_->tryToSend(outframe);

    outframe.id = motor_address_enable; //0x02052C8x
    outframe.data[0] = 0x1E; // motors 1-4 enable: 0b00011110
    outframe.data[1] = 0x78; // motors 11-14 enable: 0b01111000
    outframe.data[2] = 0;
    outframe.data[3] = 0;
    outframe.data[4] = 0;
    outframe.data[5] = 0;
    outframe.data[6] = 0;
    outframe.data[7] = 0;

    can_motor_->tryToSend(outframe);
}

// provides information for whether wheel is spinning forward or reverse
inline void MotorWithEncoder::pulseEncoder() {
    int encoderA = digitalRead(pin_Encoder_QA_);
    int encoderB = digitalRead(pin_Encoder_QB_);

    int current_Encoder_State_ = (encoderA << 1) | (encoderB);

    //11->00->11->00 is counter clockwise rotation or "forward".
    if ((previous_Encoder_State_ == 0x3 && current_Encoder_State_ == 0x0) || (previous_Encoder_State_ == 0x0 && current_Encoder_State_ == 0x3)) {
      encoder_pulses_++;
    }
    //10->01->10->01 is clockwise rotation or "backward".
    else if ((previous_Encoder_State_ == 0x2 && current_Encoder_State_ == 0x1) || (previous_Encoder_State_ == 0x1 && current_Encoder_State_ == 0x2)) {
      encoder_pulses_--;
    }

    previous_Encoder_State_ = current_Encoder_State_;
}

//provide information of the angle of the wheel
inline int MotorWithEncoder::absoluteEncoder() {
    i2cWrite(pin_Encoder_Abs_);
    return i2cRead()*360/2.048;
}

inline int MotorWithEncoder::getPulses() {
    int temp = encoder_pulses_;
    encoder_pulses_ = 0;

    if (reversed_) {
        temp = -temp;
    }

    return temp;
}

inline void MotorWithEncoder::i2cWrite(uint8_t config)
{
    Wire.beginTransmission(MCP3428_ADDR_);
    Wire.write(config);
    Wire.endTransmission();
    delay(25);
}

inline int MotorWithEncoder::i2cRead()
{
    uint8_t msb;
    uint8_t lsb;
    uint16_t total;
    Wire.requestFrom(MCP3428_ADDR_,2);
    while (Wire.available()) {
        msb = Wire.read();
        lsb = Wire.read();
        total = (int16_t)((msb << 8)| lsb);
  }
    return total/1000;
}

#endif