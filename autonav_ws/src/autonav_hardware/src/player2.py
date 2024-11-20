import numpy as np
import pydub
import pydub.playback
import threading
import os
import time

if __name__ == "__main__":
    sound = pydub.AudioSegment.from_wav(os.path.expanduser("~/Documents/metal-pipe.wav"))
    playback = pydub.playback._play_with_simpleaudio(sound)

    playback.wait_done()

    print("hi")