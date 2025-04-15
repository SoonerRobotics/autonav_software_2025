import time
from struct import pack
from can import Message

# from the FRC WPILib docs at https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
DEVICE_TYPE = 2
MANUFACTURER_CODE = 5


# reverse-engineered/from the offline REVLib install
ENABLE_API_CLASS = 11
ENABLE_API_INDEX = 0
ENABLE_DATA = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF] #TODO FIXME not sure if this is right, or if we want to individually enable motors

NONRIO_HEARTBEAT_API_CLASS = 11
NONRIO_HEARTBEAT_API_INDEX = 2
NONRIO_HEARTBEAT_DATA = [0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00] #TODO FIXME are these supposed to individually enable motors? should this bytearray enable all of them? should we only enable if we are in manual?

PERCENT_OUTPUT_API_CLASS = 0
PERCENT_OUTPUT_API_INDEX = 2

ENCODER_API_CLASS = 6 # periodic status frame 2 has encoder position data
ENCODER_API_INDEX = 2

POSITION_API_CLASS = 3
POSITION_API_INDEX = 2

VELOCITY_API_CLASS = 1
VELOCITY_API_INDEX = 2

# PARAMETER_API_CLASS = 48
# PARAMETER_API_INDEX = 0

PARAMETER_API_CLASS = 7
PARAMETER_API_INDEX = 0

def floatToData(f: float) -> bytearray:
    """Converts a given floating-point value into a bytearray that the SparkMAXes can read over CAN"""

    data_array = bytearray(pack('<f', f)) # NEOs expect little-endian format
        
    for i in range(4):
        data_array.append(0x00) #FIXME I don't like this part

    return data_array

class REVMessage(Message):
    def __init__(self, api_class: int, api_index: int, device_number: int, data=[]):
        # see https://docs.google.com/document/d/1ms0ON998f-L-pQZcR1BxyYkEZ_EzRyy0jn7Wsaba6M0/
        super().__init__(
            timestamp=time.time(),
            arbitration_id=
                DEVICE_TYPE << 24
                | MANUFACTURER_CODE << 16
                | api_class << 10
                | api_index << 6
                | device_number,
            data=data,
            check=True
        )