import serial

def get_touch_point(ser):
    while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                try:
                    x, y, z = map(int, line.split(','))
                    print(f"Touch point - X: {x}, Y: {y}, Z: {z}")
                    return x, y, z
                except ValueError:
                    print(f"Invalid data: {line}")
                    continue
