#ifndef SU_CAN_SPARK_MAX_H
#define SU_CAN_SPARK_MAX_H

#include <ACAN2515.h>

#define CAN_SPARK_MAX_SET 0x02052C81
#define CAN_SPARK_MAX_PWM_OFFSET 0x00002C00

class CanSparkMax
{
public:
    CanSparkMax(ACAN2515 *can_driver, int id) : can_driver_(can_driver)
    {
        id_ = id + CAN_SPARK_MAX_SET;
    }

    void setPWM(float value);

    CanSparkMax reversed(bool reversed)
    {
        reversed_ = reversed;
        return *this;
    }

private:
    ACAN2515 *can_driver_;
    int id_;
    float value_ = 0.0;
    bool reversed_ = false;
};

inline void CanSparkMax::setPWM(float value)
{
    value_ = value;

    // Clamp value to [-1, 1]
    if (value_ > 1)
    {
        value_ = 1;
    }
    else if (value_ < -1)
    {
        value_ = -1;
    }

    // Apply reverse
    if (reversed_)
    {
        value_ = -value_;
    }

    // Create CAN frame and set PWM mode
    CANMessage outframe;
    outframe.ext = true;
    outframe.id = CAN_SPARK_MAX_SET;
    outframe.data[0] = 0x1E;
    outframe.data[1] = 0x78;

    can_driver_->tryToSend(outframe);

    // Send PWM value
    outframe.id = id_ - CAN_SPARK_MAX_PWM_OFFSET;
    outframe.data[3] = ((uint32_t)value_ >> 24) & 0xFF;
    outframe.data[2] = ((uint32_t)value_ >> 16) & 0xFF;
    outframe.data[1] = ((uint32_t)value_ >> 8) & 0xFF;
    outframe.data[0] = ((uint32_t)value_ >> 0) & 0xFF;

    can_driver_->tryToSend(outframe);
}

#endif // SU_CAN_SPARK_MAX_H