import board
import adafruit_touchscreen
import RPi.GPIO as GPIO

# (pin for left side of screen, pin for right side of screen, pin for bottom of screen, pin for top of screen)


#black on 13 (pin1 )
#white on 19 (pin 2)
#green on 16 (pin 3)
#red on 26 (pin 4)
ts = adafruit_touchscreen.Touchscreen(13, 19, 16, 26, calibration = ((50,50), (960, 960)), size=(320, 240), resistance = 250)

while True:
    p = ts.touch_point
    if p:
        print(p)