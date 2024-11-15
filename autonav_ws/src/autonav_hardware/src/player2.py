import numpy as np
from pydub import AudioSegment
from pydub.playback import play
import threading
import os
import time

class AudioPlayer:
    def __init__(self):
        self.song = AudioSegment.from_wav(os.path.expanduser("~/Documents/vivalavida.wav"))
        # self.song_thread = threading.Thread(target=self.song_worker())


    def song_worker(self):
        play(self.song)
# pydub does things in ms
# one_second = 1 * 1000
# first_second = song[:one_second]

if __name__ == "__main__":
    my_player = AudioPlayer()

    my_player.song_worker()
    
    print("hi")