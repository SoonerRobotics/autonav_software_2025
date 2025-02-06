import PySoundSphere
import os
import time

player = PySoundSphere.AudioPlayer("ffplay")
player.load(os.path.expanduser("~/Documents/vivalavida.wav"))

player.volume = 1.0

print(player)
player.play()

time.sleep(10)