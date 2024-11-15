import threading
import time

def foo():
    t = threading.current_thread()
    while getattr(t, "do_run", True):
        print("working on a task")
        time.sleep(1)
    print("Stopping the Thread after some time.")

# Create a thread
t = threading.Thread(target=foo)
t.start()

# Allow the thread to run for 5 seconds
time.sleep(5)

# Set the termination flag to stop the thread
t.do_run = False