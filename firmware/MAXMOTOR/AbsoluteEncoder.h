#ifndef SU_ABS_ENCODER_H
#define SU_ABS_ENCODER_H

#include <Wire.h>

#define MCP3428_ADDR 0x68
#define ENCODER_TO_RADIANS (2 * 3.14159265 / 2.048)

class AbsoluteEncoder
{
public:
    AbsoluteEncoder(int id);

    void update(double period);

    double getAngle();
    double getVelocity();

private:
    int id_;
    double angle_ = 0.0;
    double velocity_ = 0.0;

    double readEncoder();
};

inline AbsoluteEncoder::AbsoluteEncoder(int id) : id_(id) {}

inline void AbsoluteEncoder::update(double period)
{
    // Read the encoder values
    double new_angle_ = readEncoder();

    // Calculate the velocity
    double instantaneous_velocity = (new_angle_ - angle_) / period;
    velocity_ = 0.9 * velocity_ + 0.1 * instantaneous_velocity; // Low pass filter
    angle_ = new_angle_;
}

inline double AbsoluteEncoder::getAngle()
{
    return angle_;
}

inline double AbsoluteEncoder::getVelocity()
{
    return velocity_;
}

inline double AbsoluteEncoder::readEncoder()
{
    Wire.beginTransmission(MCP3428_ADDR);
    Wire.write(id_);
    Wire.endTransmission();

    delay(25);

    uint8_t msb;
    uint8_t lsb;
    uint16_t total;

    Wire.requestFrom(MCP3428_ADDR, 2);

    while (Wire.available())
    {
        msb = Wire.read();
        lsb = Wire.read();
        total = (int16_t)((msb << 8) | lsb);
    }

    return total / 1000 * ENCODER_TO_RADIANS;
}

#endif // SU_ABS_ENCODER_H