import serial
from touchScreenBasicCoordOutput import Point
import time

ser = serial.Serial('/dev/ttyACM0', 9600)

def read_coords():
    try:
        ser.write(b'S')
        time.sleep(0.1)
        data = ser.readline().decode('utf-8').strip()
        print("got this data: " + data)
    except UnicodeDecodeError as e:
        print(f"Decoding error: {e}")

if __name__ == "__main__":
    while True:
        read_coords()