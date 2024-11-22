import digitalio
from adafruit_touchscreen import Touchscreen
import RPi.GPIO as GPIO

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define BCM pin numbers for the touchscreen connections
XL = 13  
XR = 19  
YD = 26  
YU = 16  

# Initialize the touchscreen
# Use BCM pin numbers for XL, XR, YD, and YU
ts = Touchscreen(XL, XR, YD, YU, calibration=((0, 65535), (0, 65535)), size=(320, 240))

while True:
    # Get the touch point
    p = ts.touch_point
    if p:
        print(f"X: {p[0]}, Y: {p[1]}, Pressure: {p[2]}")
