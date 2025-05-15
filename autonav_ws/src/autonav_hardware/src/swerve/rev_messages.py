import time
from enum import Enum
from struct import pack, unpack
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

ABSOLUTE_ENCODER_FEEDBACK_API_CLASS = 6
ABSOLUTE_ENCODER_FEEDBACK_API_INDEX = 37

DRIVE_ENCODER_FEEDBACK_API_CLASS = 6
DRIVE_ENCODER_FEEDBACK_API_INDEX = 33

PERCENT_OUTPUT_API_CLASS = 0
PERCENT_OUTPUT_API_INDEX = 2

ENCODER_API_CLASS = 6 # periodic status frame 2 has encoder position data
ENCODER_API_INDEX = 2

POSITION_API_CLASS = 3
POSITION_API_INDEX = 2

VELOCITY_API_CLASS = 1
VELOCITY_API_INDEX = 2

PARAMETER_API_CLASS = 48
PARAMETER_API_INDEX = 0

# PARAMETER_API_CLASS = 7
# PARAMETER_API_INDEX = 0

# enum parameters
class Parameter(Enum):
    kCanID = 0
    kTest = 63 # I have no clue what this is going to do :)
    kSerialNumberLow = 47 # Low 32-bits of unique 96-bit serial number (READ ONLY)
    kSerialNumberMid = 48 # Mid 32-bits of unique 96-bit serial number (READ ONLY)
    kSerialNumberHigh = 49 # High 32-bits of unique 96-bit serial number (READ ONLY)


class ParameterType(Enum):
    kFloat = 0
    kUint = 1
    kInputMode = 14
    kMotorType = 13
    kSensorType = 9
    kCtrlType = 10
    kIdleMode = 11
    kBool = 12

def floatToData(f: float) -> bytearray:
    """Converts a given floating-point value into a bytearray that the SparkMAXes can read over CAN"""

    data_array = bytearray(pack('<f', f)) # NEOs expect little-endian format
        
    for i in range(4):
        data_array.append(0x00) #FIXME I don't like this part

    return data_array

def dataToFloat(data: bytearray) -> float:
    """Converts a bytearray from the SparkMAXes into a floating-point value"""
    
    if len(data) < 4:
        raise ValueError("Data must be at least 4 bytes long to extract a float")
    
    return unpack('<f', data[:4])[0]

def dataToUint(data: bytearray) -> int:
    """Converts a bytearray from the SparkMAXes into an unsigned integer value"""
    
    if len(data) < 4:
        raise ValueError("Data must be at least 4 bytes long to extract an unsigned int")
    
    return unpack('<I', data[:4])[0]

class REVMessageBreakdown:
    def __init__(self, arbitration_id: int):
        self.device_number = arbitration_id & 0x3F
        self.api_index = (arbitration_id >> 6) & 0x3F
        self.api_class = (arbitration_id >> 10) & 0x3F
        self.manufacturer_code = (arbitration_id >> 16) & 0xFF
        self.device_type = (arbitration_id >> 24) & 0xFF

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