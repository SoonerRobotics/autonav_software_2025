from can import ThreadSafeBus, Message

#FIXME?
CAN_SPARK_MAX_OFFSET = 0x02052C81
CAN_SPARK_MAX_PWM_OFFSET = 0x00002C00

class CanSparkMax:
    def __init__(self, id: int, canbus: ThreadSafeBus, revered=False):
        self.id = id + CAN_SPARK_MAX_OFFSET

        self.can = canbus
        self.value_ = 0.0
        self.reversed_ = revered
        

    def setPWM(self, value: float) -> None:
            self.value_ = value

            # Clamp value to [-1, 1]
            if self.value_ < -1:
                self.value_ = -1
            elif self.value_ > 1:
                self.value_ = 1

            # Apply reverse
            if self.reversed_:
                self.value_ *= -1

            #TODO FIXME this is not what we're doing        
            # Create CAN frame and set PWM mode
            outframe = Message()
            outframe.ext = True
            outframe.id = self.id_
            outframe.data[0] = 0x02
        
            ok = self.can_driver_.tryToSend(outframe)
        
            # Send PWM value
            outframe.id = self.id_ - CAN_SPARK_MAX_PWM_OFFSET
            outframe.data[3] = (self.value_ >> 24) & 0xFF
            outframe.data[2] = (self.value_ >> 16) & 0xFF
            outframe.data[1] = (self.value_ >> 8) & 0xFF
            outframe.data[0] = (self.value_ >> 0) & 0xFF
        
            ok = self.can_driver_.tryToSend(outframe)