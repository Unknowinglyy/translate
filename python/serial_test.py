import serial
from touchScreenBasicCoordOutput import Point

ser = serial.Serial('/dev/ttyACM0', 9600)

def read_coords():
    try:
        data = ser.readline().decode('utf-8').strip()
        print("got this data: " + data)
    except UnicodeDecodeError as e:
        print(f"Decoding error: {e}")

if __name__ == "__main__":
    read_coords()