import tkinter
from tkinter import filedialog

import can
from swerve.can_spark_max import REVMessage, CanSparkMax

print(" == CAN SparkMAX CLI ==")
print("")

bus = can.ThreadSafeBus(
    bustype="slcan", channel="/dev/ttyACM0", bitrate=1_000_000
)

root = tkinter.Tk()
root.withdraw()
filename = filedialog.askopenfilename()

if filename == "":
    raise SystemExit

with open(filename, "r") as f:
    # format like "front,left,drive,1"
    lines = f.readlines()

motors = []

for line in lines:
    front, left, drive, id = line.split()
    motor = CanSparkMax(int(id), bus)
    motor.descriptor = [front, left]

    motors.append(motor)

while True:
    command = input("enter a command:").lower().strip()

    if command == "quit":
        break

print(" == DONE == ")