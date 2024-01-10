import signal

def signal_handler(signal, frame):
    print("Ctrl+Z pressed. Handling SIGTSTP signal.")
    # Perform actions or cleanup here, if needed

# Register the signal handler for SIGTSTP
signal.signal(signal.SIGTSTP, signal_handler)
while(True):
    print('1')
    