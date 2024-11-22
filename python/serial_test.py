import serial

ser = serial.Serial('/dev/ttyACM0', 9600)

while True:
    if ser.in_waiting > 0:
        try:
            data = ser.readline().decode('utf-8').strip()
            print("got this data: " + data)
        except UnicodeDecodeError as e:
            print(f"Decode error: {e}")