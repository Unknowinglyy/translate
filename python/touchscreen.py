import board
import adafruit_touchscreen

# checking what the board pins are set to
print(board.TOUCH_XL)
print(board.TOUCH_XR)
print(board.TOUCH_YD)
print(board.TOUCH_YU)

# ts = adafruit_touchscreen.Touchscreen(board.TOUCH_XL, board.TOUCH_XR, board.TOUCH_YD, board.TOUCH_YU, calibration = ((50,50), (960, 960)), size=(320, 240))

# while True:
#     p = ts.touch_point
#     if p:
#         print(p)