# this script is for testing CAN from canable to canable.

import can

can_channel = input("please enter CAN channel path: ")

my_canable = can.Bus(
    bustype="slcan", channel=can_channel, bitrate=100000
)

with my_canable as bus:
    while True:
        id = int(input("arbitration id (decimal): "))
        data = bytes(list(map(int, (input("data (bytearray (decimal), delimter='.'): ").split('.')))))
        print(data.hex())
        print(type(data))
        msg = can.Message(
            arbitration_id=id,
            data=data
        )
        try:
            bus.send(msg)
        except can.CanError:
            print("message not sent")