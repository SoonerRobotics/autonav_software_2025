from can import ThreadSafeBus, Message, Notifier
from swerve.rev_messages import *

class CanSparkMax:
    def __init__(self, id: int, canbus: ThreadSafeBus, reversed=False):
        self.id = id

        self.can = canbus
        self.value_ = 0.0
        self.absolute_position_ = 0.0
        self.revolutions_per_minute_ = 0.0
        self.reversed_ = reversed

        # register CAN callback
        self.notifier = Notifier(self.can, [self.canCallback])

    def set(self, value: float) -> None:
        self.value_ = value

        # Clamp value to [-1, 1]
        if self.value_ < -1:
            self.value_ = -1
        elif self.value_ > 1:
            self.value_ = 1

        # Apply reverse
        if self.reversed_:
            self.value_ *= -1

        #TODO FIXME actually send the message or something
    
    def setPosition(self, value: float) -> None:
        self.can.send(REVMessage(POSITION_API_CLASS, POSITION_API_INDEX, self.id, floatToData(value)))
    
    def setVelocity(self, value: float) -> None:
        self.can.send(REVMessage(VELOCITY_API_CLASS, VELOCITY_API_INDEX, self.id, floatToData(value)))

    def getAbsolutePosition(self) -> float:
        return self.absolute_position_
    
    def getRevolutionsPerMinute(self) -> float:
        return self.revolutions_per_minute_
    
    # if you don't send this at a decent rate (at least 20Hz I think, probably faster if you can) the sparkMAX will disable itself
    def sendHeartbeat(self) -> None:
        self.can.send(REVMessage(NONRIO_HEARTBEAT_API_CLASS, NONRIO_HEARTBEAT_API_INDEX, self.id, NONRIO_HEARTBEAT_DATA))
    
    def canCallback(self, msg: Message) -> None:
        breakdown = REVMessageBreakdown(msg.arbitration_id)

        # First check if we are receiving feedback for a device that is not ours
        if breakdown.device_number != self.id:
            return
        
        if breakdown.api_class == ABSOLUTE_ENCODER_FEEDBACK_API_CLASS and breakdown.api_index == ABSOLUTE_ENCODER_FEEDBACK_API_INDEX:
            absolute_position = dataToFloat(msg.data)
            self.absolute_position_ = absolute_position
            pass
        
        if breakdown.api_classs == DRIVE_ENCODER_FEEDBACK_API_CLASS and breakdown.api_index == DRIVE_ENCODER_FEEDBACK_API_INDEX:
            revolutions_per_minute = dataToFloat(msg.data)
            self.revolutions_per_minute_ = revolutions_per_minute
            pass