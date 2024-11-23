import numpy as np
import evdev
from scipy.linalg import lstsq

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Define the original and translated points
original_points = [
    (3800, 150),
    (250, 150),
    (3800, 3950),
    (250, 3950)
]

translated_points = [
    (50, 100),
    (50, 950),
    (950, 100),
    (950, 950)
]

# Prepare the matrices for affine transformation
original_coords = []
translated_x = []
translated_y = []

for (ox, oy), (tx, ty) in zip(original_points, translated_points):
    original_coords.append([ox, oy, 1])
    translated_x.append(tx)
    translated_y.append(ty)

# Solve for the affine transformation coefficients
original_coords = np.array(original_coords)
translated_x = np.array(translated_x)
translated_y = np.array(translated_y)

x_coeffs, _, _, _ = lstsq(original_coords, translated_x)
y_coeffs, _, _, _ = lstsq(original_coords, translated_y)

# Define the transformation function
def transform_coordinates(x, y):
    x_transformed = x_coeffs[0] * x + x_coeffs[1] * y + x_coeffs[2]
    y_transformed = y_coeffs[0] * x + y_coeffs[1] * y + y_coeffs[2]
    return Point(x_transformed, y_transformed)

# Function to read touch coordinates
def read_touch_coordinates(device_path='/dev/input/event5'):
    device = evdev.InputDevice(device_path)

    x, y = None, None

    for event in device.read_loop():
        if event.type == evdev.ecodes.EV_ABS:
            if event.code == evdev.ecodes.ABS_X or event.code == evdev.ecodes.ABS_MT_POSITION_X:
                x = event.value
            elif event.code == evdev.ecodes.ABS_Y or event.code == evdev.ecodes.ABS_MT_POSITION_Y:
                y = event.value
            if x is not None and y is not None:
                return Point(x, y)

# Main loop
if __name__ == "__main__":
    while True:
        try:
            # Read the raw touch coordinates
            original_point = read_touch_coordinates()

            # Transform the coordinates
            translated_point = transform_coordinates(original_point.x, original_point.y)

            # Print the original and translated coordinates
            print(f"X: {translated_point.x:.2f}, Y: {translated_point.y:.2f}")
        except KeyboardInterrupt:
            break
