import can

# can_channel = input("please enter CAN channel (ex 'COM3' or '/dev/ttyACM0'): ")
can_channel = "COM6"

canbus = can.ThreadSafeBus(
    interface="slcan",
    channel=can_channel,
    bitrate=1_000_000 # NEOs (and FRC CAN in general) run at 1 Mbit/sec
)

print(canbus)

motorMessage = can.Message(
    is_extended_id=True,
    # arbitration_id=int(),
    arbitration_id=0x02052C81,
    # data=0x00000000,
    data=0xcdcc4c3e,
    dlc=8
)
enableMessage = can.Message(
    is_extended_id=True,
    arbitration_id=0x0205C81,
    data=0xFFFFFFFF,
    dlc=8
)
ids = set()

broadcastEnable = can.Message(
    arbitration_id=b""
)

with canbus as bus:
    bus.send(motorMessage)
    bus.send(enableMessage)

    for msg in bus:
        if msg.arbitration_id not in ids:
            print(f"message id: {hex(msg.arbitration_id)}, data: {msg.data.hex()}")

        ids.add(msg.arbitration_id)