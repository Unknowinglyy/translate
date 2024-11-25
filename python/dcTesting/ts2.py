import evdev
import time

class Point:
    def __init__(self, x, y, z=0):
        self.x = x
        self.y = y
        self.z = z

def read_touch_coordinates(device_path='/dev/input/event4', timeout=1.0):
    device = evdev.InputDevice(device_path)
    x, y, z = None, None, None
    samples_x, samples_y = [], []
    NUMSAMPLES = 2
    start_time = time.time()

    # Read touch events until a valid point is captured or timeout occurs

    try:
        for event in device.read_one():
            if event.type == evdev.ecodes.EV_ABS:
                if event.code == evdev.ecodes.ABS_X or event.code == evdev.ecodes.ABS_MT_POSITION_X:
                    samples_x.append(event.value)
                    if len(samples_x) > NUMSAMPLES:
                        samples_x.pop(0)
                elif event.code == evdev.ecodes.ABS_Y or event.code == evdev.ecodes.ABS_MT_POSITION_Y:
                    samples_y.append(event.value)
                    if len(samples_y) > NUMSAMPLES:
                        samples_y.pop(0)
                elif event.code == evdev.ecodes.ABS_PRESSURE:
                    z = event.value
            

                # Process samples when we have enough

                if len(samples_x) == NUMSAMPLES and len(samples_y) == NUMSAMPLES:
                    # Sort samples to get the median value
                    samples_x.sort()
                    samples_y.sort()
                    x = samples_x[NUMSAMPLES // 2]
                    y = samples_y[NUMSAMPLES // 2]

                # Return the point with x, y, and z values
                return Point(x, y, z if z is not None else 0)
    except Exception as e:
        print(e)
        return Point(0,0)

if __name__ == "__main__":
    point = read_touch_coordinates()
    print(f"({point.x}, {point.y})")