# this script is for testing CAN from canable to canable.
import can

can_channel = input("please enter CAN channel path: ")

my_canable = can.ThreadSafeBus(
    bustype="slcan", channel=can_channel, bitrate=100000
)

with my_canable as bus:
    for msg in bus:
        print(f"message id: {msg.arbitration_id}, data: {msg.data.hex()}")