import evdev
import threading
import time

prev_point = {"x": 0, "y": 0}

class Point:
    def __init__(self, x, y, z=0):
        self.x = x
        self.y = y
        self.z = z

def read_touch_coordinates(device_path='/dev/input/event6'):
    global prev_point
    device = evdev.InputDevice(device_path)

    x, y = None, None

    for event in device.read_loop():
        if event.type == evdev.ecodes.EV_ABS:
            if event.code == evdev.ecodes.ABS_X or event.code == evdev.ecodes.ABS_MT_POSITION_X:
                x = event.value
            elif event.code == evdev.ecodes.ABS_Y or event.code == evdev.ecodes.ABS_MT_POSITION_Y:
                y = event.value
            prev_point["x"] = x
            prev_point["y"] = y
            if x is not None and y is not None:
                return Point(x, y)
        elif event.type == evdev.ecodes.EV_KEY:
            return Point(x, y)
    return Point(0, 0)

def read_coordinates(timeout=0.1):
    result = [None]

    def target():
        result[0] = read_touch_coordinates()

    thread = threading.Thread(target=target)
    thread.start()
    thread.join(timeout)

    if thread.is_alive():
        return Point(prev_point["x"], prev_point["y"])
    else:
        return result[0]

if __name__ == "__main__":
    point = read_coordinates()
    print(f"X: {point.x}, Y: {point.y}")