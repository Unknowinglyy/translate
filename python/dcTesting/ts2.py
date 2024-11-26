import evdev
import time

class Point:
    def __init__(self, x, y, z=0):
        self.x = x
        self.y = y
        self.z = z

def read_touch_coordinates(device_path='/dev/input/event4', timeout=0.5):
    device = evdev.InputDevice(device_path)
    x, y, z = None, None, None
    samples_x, samples_y = [], []
    NUMSAMPLES = 2
    last_event_time = time.time()

    # Read touch events in a loop
    while True:
        try:
            event = device.read_one()  # Non-blocking read
            if event:
                last_event_time = time.time()
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
                        samples_x.sort()
                        samples_y.sort()
                        x = samples_x[NUMSAMPLES // 2]
                        y = samples_y[NUMSAMPLES // 2]

                        yield Point(x, y, z if z is not None else 0)

            # Check for inactivity
            if time.time() - last_event_time > timeout:
                yield Point(0, 0, 0)
                last_event_time = time.time()
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    device_path = '/dev/input/event4'  # Replace with your actual touchscreen device
    for point in read_touch_coordinates(device_path):
        print(f"X: {point.x}, Y: {point.y}, Z: {point.z}")
