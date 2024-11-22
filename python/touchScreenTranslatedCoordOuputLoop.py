import evdev

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Define the original and target ranges
original_x_min, original_x_max = 250, 3800
original_y_min, original_y_max = 150, 3950

target_x_min, target_x_max = 50, 950
target_y_min, target_y_max = 100, 950

# Calculate the scaling factors
scale_x = (target_x_max - target_x_min) / (original_x_max - original_x_min)
scale_y = (target_y_max - target_y_min) / (original_y_max - original_y_min)

# Function to translate coordinates
def translate_coordinates(x, y):
    # Apply scaling and translation
    translated_x = target_x_min + (x - original_x_min) * scale_x
    translated_y = target_y_min + (y - original_y_min) * scale_y
    return int(translated_x), int(translated_y)

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
