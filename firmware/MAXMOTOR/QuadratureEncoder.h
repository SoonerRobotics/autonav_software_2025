#ifndef SU_QUAD_ENCODER_H
#define SU_QUAD_ENCODER_H

#include "Config.h"

class QuadratureEncoder
{
  public:
    QuadratureEncoder(int id[2]);

    void update(double period);
    void pulseEncoder();
    void update();
    // int getPulses();
    double getVelocity();

private:
    int id_[2];
    double velocity_ = 0.0;

    double readEncoder();

    int encoder_pulses_ = 0;
    int previous_Encoder_State_ = 0;
};

inline QuadratureEncoder::QuadratureEncoder(int id[2]) : id_{id[0],id[1]} {}

inline void QuadratureEncoder::pulseEncoder() {
    int encoderA = digitalRead(id_[0]);
    int encoderB = digitalRead(id_[1]);

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

inline void QuadratureEncoder::update(double period) {
    SUSwerveDriveConfig Config;
    velocity_ = 2*3.14159265*Config.wheelRadius*encoder_pulses/period;
    encoder_pulses = 0;
}

inline double QuadratureEncoder::getVelocity() {
    return velocity_;
}

#endif // SU_ABS_ENCODER_H