#ifndef MOTOR_WITH_ENCODER_H
#define MOTOR_WITH_ENCODER_H

#include <stdio.h>
#include <Wire.h>
#include <ACAN2515.h>

/*
  changes: no more PWM; now CAN
*/


class MotorWithEncoder {

public:
    //MotorWithEncoder(uint8_t pin_PWM_Control, uint8_t pin_EncoderA, uint8_t pin_EncoderB, bool reversed);
    MotorWithEncoder(ACAN2515 &can_motor, uint32_t drive, uint32_t rotate, uint8_t pin_Encoder_QA, uint8_t pin_Encoder_QB, uint8_t pin_Encoder_Abs, bool reversed);

    void setup();

    void pulseEncoder();
    int getPulses();
    int absoluteEncoder();

    void setOutput(float control);
    void setMinOutput(float minControl);

private:
    float minControl_ = 0.01f;
    float maxControl_ = 0.90f;

    // Servo servoController_;

    //const uint8_t pin_PWM_Control_;
    ACAN2515 *can_motor_;
    const uint32_t drive_;
    const uint32_t rotate_;
    const uint8_t pin_Encoder_QA_;
    const uint8_t pin_Encoder_QB_;
    const uint8_t pin_Encoder_Abs_;
    const bool reversed_;

    const uint32_t motor_address_difference = 0x00002C00; //difference between the addresses of the 2 CAN messages
    int encoder_pulses_ = 0;
    int previous_Encoder_State_ = 0;
    void i2cWrite(uint8_t config);
    int i2cRead();
};

const uint8_t MCP3428_ADDR_ = 0b1101000;
// inline MotorWithEncoder::MotorWithEncoder(uint8_t pin_PWM_Control, uint8_t pin_EncoderA, uint8_t pin_EncoderB, bool reversed)
//     : pin_PWM_Control_(pin_PWM_Control), pin_EncoderA_(pin_EncoderA), pin_EncoderB_(pin_EncoderB), reversed_(reversed) {}

inline MotorWithEncoder::MotorWithEncoder(ACAN2515 &can_motor, uint32_t drive, uint32_t rotate, uint8_t pin_Encoder_QA, uint8_t pin_Encoder_QB, uint8_t pin_Encoder_Abs, bool reversed)
    : can_motor_(&can_motor), drive_(drive), rotate_(rotate), pin_Encoder_QA_(pin_Encoder_QA), pin_Encoder_QB_(pin_Encoder_QB), pin_Encoder_Abs_(pin_Encoder_Abs), reversed_(reversed) {}


// inline void MotorWithEncoder::setup() {
//     servoController_.attach(pin_PWM_Control_, 1000, 2000);
// }

/*
********************************************
servo controller is for servo object that drives the robot for PWM pin input
********************************************
*/

/*
    Set motor output on a scale of -1.0 to 1.0
*/
// inline void MotorWithEncoder::setOutput(float control) {
    
//     int servoOut = 90; // Default to braked

//     if (control > 1) {
//       control = 1;
//     }

//     if (control < -1) {
//       control = -1;
//     }

//     if (reversed_) {
//         control = -control;
//     }

//     if (abs(control) > maxControl_) { 
//       control = abs(control) / control * maxControl_;
//     }

//     // If we are greater than our min control signal, set the servo output
//     if ((control > 0 && control > minControl_) || (control < 0 && control < -minControl_)) {
//         servoOut = 90 + control * 85;
//     }

//     servoController_.write(servoOut);
// }

inline void MotorWithEncoder::setOutput(float control) {
    CANMessage outframe;
    outframe.ext = true;

    float CANOut = 0; // Default to braked

    if (control > 1) {
      control = 1;
    }

    if (control < -1) {
      control = -1;
    }

    if (reversed_) {
        control = -control;
    }

    if (abs(control) > maxControl_) { 
      control = abs(control) / control * maxControl_;
    }

    // If we are greater than our min control signal, set the servo output
    if ((control > 0 && control > minControl_) || (control < 0 && control < -minControl_)) {
        CANOut = control * 0.85;
    }

    outframe.id = drive_;
    outframe.data[0] = 0x02;

    bool ok = can_motor_->tryToSend(outframe);
    if(ok);

    outframe.id = drive_ - motor_address_difference;

    outframe.data[3] = ((uint32_t)CANOut >> 24) & 0xFF;
    outframe.data[2] = ((uint32_t)CANOut >> 16) & 0xFF;
    outframe.data[1] = ((uint32_t)CANOut >> 8) & 0xFF;
    outframe.data[0] = ((uint32_t)CANOut >> 0) & 0xFF;

    ok = can_motor_->tryToSend(outframe);
    if(ok);
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

/*
  inline some kind of encoder stuff for I2C abs encoder data or something
*/

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

void i2cWrite(uint8_t config)
{
    Wire.beginTransmission(MCP3428_ADDR_);
    Wire.write(config);
    Wire.endTransmission();
    delay(25);
}

int i2cRead()
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