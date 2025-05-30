from can import ThreadSafeBus, Message, Notifier
from autonav_shared.node import Node
from swerve.rev_messages import *
import time
import numpy as np

class CanSparkMax:
    def __init__(self, id: int, canbus: ThreadSafeBus, node: Node, reversed=False):
        self.id = id

        self.can = canbus
        self.node = node
        self.value_ = 0.0
        self.absolute_position_ = 0.0
        self.revolutions_per_minute_ = 0.0
        self.drive_position_ = 0.0
        self.reversed_ = reversed
        self.has_set_status = False
        self.set_time = 0.0

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
        pass
    
    def setVelocity(self, value: float) -> None:
        self.can.send(REVMessage(VELOCITY_API_CLASS, VELOCITY_API_INDEX, self.id, floatToData(value)))
        pass

    # deprecated
    def setPeriodicFrameTime(self, status: int, period: int) -> None:
        if period < 0 or period > 65535:
            raise ValueError("period must be between 0 and 65535")

        # convert to bytes
        byt_0 = period & 0xFF
        byt_1 = (period >> 8) & 0xFF
        self.can.send(REVMessage(6, status, self.id, [byt_0, byt_1]))

    def getPeriodicFrameTime(self, status: int) -> None:
        self.can.send(REVMessage(6, status, self.id, []))

    def setParameterUint(self, parameter: Parameter, value: int) -> None:
        api_class = PARAMETER_API_CLASS
        api_index = parameter.value

        # first 4 bytes is the value, an unsigned int
        data = bytearray(pack('<I', value))

        # the next byte is the type
        data.append(ParameterType.kUint.value)
        
        # send it
        self.can.send(REVMessage(api_class, api_index, self.id, data))

    def getParameter(self, parameter: int) -> None:
        api_class = PARAMETER_API_CLASS
        api_index = parameter
        self.can.send(REVMessage(api_class, api_index, self.id, []))

    def getAbsolutePosition(self) -> float:
        return self.absolute_position_

    def getDrivePosition(self) -> float:
        return self.drive_position_

    def encoder_to_radians(self, encoder_reading):
        # Convert 0-1 encoder reading to 0-2π radians
        angle_radians = encoder_reading * 2 * np.pi

        # Normalize to [-π, π]
        angle_radians = (angle_radians + np.pi) % (2 * np.pi) - np.pi

        return angle_radians


    def getAngle(self) -> float:
        # remap from 0 to 1 to radians (i.e. 0 is 0 radians, 1 is 2 * pi radians)
        return self.encoder_to_radians(self.absolute_position_)
    
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

        if not self.has_set_status and self.set_time == 0:
            self.set_time = time.time() + 0.1 + (self.id * 0.1) # 0.1s + 0.1s * id

        if self.set_time != 0 and time.time() > self.set_time:
            # self.node.log(f"Setting periodic status for {self.id}")
            # self.getPeriodicFrameTime(2) # Get Periodic Status 1
            self.setPeriodicFrameTime(2, 50) # Set Periodic Status 2 to 50ms intervals
            self.setPeriodicFrameTime(3, 50) # Set Periodic Status 2 to 50ms intervals
            # self.setPeriodicFrameTime(4, 5000) # Set Periodic Status 4 to 50ms intervals
            self.getParameter(0) # gets the can id of the device
            self.has_set_status = True
            self.set_time = 0
        
        if breakdown.api_class == 6 and breakdown.api_index == 34:
            # this should be the current motor position:
            # take the first 4 bytes
            # self.node.log("received data length: " + str(len(msg.data)))
            try:
                first_four = msg.data[0:4]
                position = dataToFloat(first_four)
                # self.node.log(f"position callback: {position} for {self.id} and class {breakdown.api_class} and index {breakdown.api_index}")
                self.drive_position_ = position
                # print(f"position callback: {position} for {self.id}")
            except Exception as e:
                pass
        # elif breakdown.api_class == 6 and breakdown.device_number == 2 and breakdown.api_index != 33 and breakdown.api_index != 32 and breakdown.api_index == 37:
        #     try:
        #         # this should be the current motor position:
        #         # take the first 4 bytes
        #         first_four = msg.data[0:4]
        #         position = dataToFloat(first_four)
        #         print(f"position callback: {position} for {self.id} and class {breakdown.api_class} and index {breakdown.api_index}")
        #     except Exception as e:
        #         pass

        if breakdown.api_class == 48:
            paramter_idx = breakdown.api_index
            # the next byte is the type
            typ = msg.data[4]
            # the first four is the value
            value = dataToFloat(msg.data[0:4]) if typ == 0 else dataToUint(msg.data[0:4])
            # the last byte is whether it was successful
            success = msg.data[5]
            self.node.log(f"parameter callback: {paramter_idx} = {value} for {self.id} with type {typ} and success {success}")

        # if breakdown.api_class == ABSOLUTE_ENCODER_FEEDBACK_API_CLASS and breakdown.api_index == ABSOLUTE_ENCODER_FEEDBACK_API_INDEX:
        #     absolute_position = dataToFloat(msg.data)
        #     self.absolute_position_ = absolute_position
        #     pass

        if breakdown.api_class == 6 and breakdown.api_index == 37:
            absolute_position = dataToFloat(msg.data)
            self.absolute_position_ = absolute_position
            # print(f"absolute position callback: {absolute_position} for {self.id}")
            pass
        
        if breakdown.api_class == DRIVE_ENCODER_FEEDBACK_API_CLASS and breakdown.api_index == DRIVE_ENCODER_FEEDBACK_API_INDEX:
            revolutions_per_minute = dataToFloat(msg.data)
            self.revolutions_per_minute_ = revolutions_per_minute
            pass