import can

dev_channel = input("enter canable channel:")

my_can_bus = can.ThreadSafeBus(
    bustype="slcan", channel=dev_channel, bitrate=100000
)

with my_can_bus as bus:
    for msg in bus:
        print(msg.data)
        