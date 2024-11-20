from just_playback import Playback
import time

playback = Playback()
playback.load_file('/home/tony/Documents/vivalavida.wav')
playback.play()

playback2 = Playback()
playback2.load_file('/home/tony/Documents/metal-pipe.wav')
playback2.play()

while playback.active:
    time.sleep(1)