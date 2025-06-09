import serial
import time
from pynput import keyboard

# Initialize serial communication with Arduino
ser = serial.Serial('COM6', 9600)  # Change 'COM3' to the correct port for your Arduino
time.sleep(2)  # Wait for the serial connection to initialize

key_down_char = 'a'
keyDown = False

def on_press(key):
    try:
        # Convert key to string and send to Arduino
        global key_down_char
        key_down_char = key.char
        global keyDown
        keyDown = True
        # if key_down_char is not None:
        #     ser.write(key_down_char.encode())
            # print(f'Sent: {key_down_char}')
    except AttributeError:
        # Handle special keys
        key_str = str(key)
        # ser.write(key_str.encode())
        print(f'Sent: {key_str}')

def on_release(key):
    global keyDown
    keyDown = False
    if key == keyboard.Key.esc:
        # Stop listener
        return False

# Collect events until released
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    while(True):
        if(keyDown):
            ser.write(key_down_char.encode())
            print(f'Sent:', key_down_char)

    listener.join()

# Close the serial connection
ser.close()
