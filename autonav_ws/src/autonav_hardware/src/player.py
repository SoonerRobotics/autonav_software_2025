import wave
import pyaudio
import time

class Player:
    def __init__(self):
        self.chunk_size = 1024


    def play_file(self, filename, duration_seconds):
        with wave.open(filename, 'rb') as wf:

            p = pyaudio.PyAudio()

            stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True)

            # Play samples from the wave file (3)
            start_time = time.time()

            while time.time() - start_time < duration_seconds:
                data = wf.readframes(self.chunk_size)
                nbytes = wf.getsampwidth()
                print(nbytes)
                if not data:
                    break
                stream.write(data)

            # Close stream (4)
            stream.close()

            # Release PortAudio system resources (5)
            p.terminate()

if __name__ == "__main__":
    player = Player()
    player.play_file("./Documents/metal-pipe.wav", 4)