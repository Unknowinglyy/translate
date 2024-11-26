import serial

def get_touch_point(serial_port="/dev/ttyACM0", baud_rate =9600):
    with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
        while True:
            line = ser.readline().decode('ascii').strip()
            if line:
                try:
                    x, y, z = map(int, line.split(','))
                    print(f"Touch point - X: {x}, Y: {y}, Z: {z}")
                    return x, y, z
                except ValueError:
                    print(f"Invalid data: {line}")
                    continue
