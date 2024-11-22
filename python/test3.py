#!/usr/bin/env python3
import serial

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()

    while True:
        if ser.in_waiting > 0:
            raw_line = ser.readline()
            try:
                # Attempt to decode the line
                line = raw_line.decode('utf-8').rstrip()
                print(line)
            except UnicodeDecodeError:
                # Log invalid data for debugging
                print(f"Invalid data received: {raw_line}")
