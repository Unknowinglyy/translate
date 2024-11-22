import serial
from touchScreenBasicCoordOutput import Point

ser = serial.Serial('/dev/ttyACM0', 9600)

def read_coords():
    data = ser.readline()
    print("got this data: " + data.decode('utf-8'))