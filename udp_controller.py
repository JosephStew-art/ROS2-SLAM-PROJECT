import socket
from pynput import keyboard

# Define the IP address and port of the Raspberry Pi
RPI_IP = '192.168.68.61'  # Replace with your Raspberry Pi's IP address
RPI_PORT = 12345

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def on_press(key):
    try:
        # Send key press commands to Raspberry Pi
        if key.char in ['w', 's', 'a', 'd', 'x']:
            print(key)
            sock.sendto(key.char.encode(), (RPI_IP, RPI_PORT))
    except AttributeError:
        pass

# Set up keyboard listener
listener = keyboard.Listener(on_press=on_press)
listener.start()
listener.join()                 