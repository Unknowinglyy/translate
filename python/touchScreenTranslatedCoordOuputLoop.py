import evdev

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Define the scaling factors
scale_x = (950 - 50) / (3800 - 250)  # Scaling factor for X axis
scale_y = (950 - 100) / (3950 - 150)  # Scaling factor for Y axis

# Function to translate coordinates
def translate_coordinates(x, y):
    translated_x = int(scale_x * (x - 250) + 50)
    translated_y = int(scale_y * (y - 150) + 100)
    return translated_x, translated_y

def read_touch_coordinates(device_path='/dev/input/event7'):
    device = evdev.InputDevice(device_path)

    x, y = None, None

    # Read touch events in a loop
    for event in device.read_loop():
        # Only process single-touch absolute X and Y events (ignore multitouch)
        if event.type == evdev.ecodes.EV_ABS:
            if event.code == evdev.ecodes.ABS_X or event.code == evdev.ecodes.ABS_MT_POSITION_X:
                x = event.value
            elif event.code == evdev.ecodes.ABS_Y or event.code == evdev.ecodes.ABS_MT_POSITION_Y:
                y = event.value
            # Return translated coordinates when both X and Y are captured
            if x is not None and y is not None:
                translated_x, translated_y = translate_coordinates(x, y)
                return Point(translated_x, translated_y)

if __name__ == "__main__":
    while True:
        try:
            point = read_touch_coordinates()
            print(f"Translated X: {point.x}, Translated Y: {point.y}")
        except KeyboardInterrupt:
            break
