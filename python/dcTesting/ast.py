import time
import math
import RPi.GPIO as GPIO
import serial
from accelstepper import AccelStepper
from multistepper import MultiStepper
from kine2 import Kinematics
from serial_point import get_touch_point

# ----------------------------------------------------------------------------------
'''
NOTES:
    - Kinematics A,B,C == 0,1,2
'''
# ----------------------------------------------------------------------------------
# TIME FUNCTIONS:
start_time = time.perf_counter()

def millis():
    # Returns the number of ms since the program started
    return int((time.perf_counter() - start_time) * 1000)

def constrain(value, minn, maxn):
    return max(min(maxn, value), minn)

kinematics = Kinematics(2, 3.125, 1.75, 3.669291339)
# ----------------------------------------------------------------------------------
# SETUP MOTORS:
stepperA = AccelStepper(AccelStepper.DRIVER, 20, 21)  # Stepper located btwn two corners
stepperB = AccelStepper(AccelStepper.DRIVER, 5, 6)    # Motor 3
stepperC = AccelStepper(AccelStepper.DRIVER, 23, 24)  # Motor 1
steppers = MultiStepper()

pos = [0, 0, 0]
ENA = 17

angOrig = 206.662752199  # Original angle that each leg starts at
speed = [0, 0, 0]
speedPrev = [0, 0, 0]
ks = 20  # Amplifying constant

xoffset = 500
yoffset = 500

# PID constants
kp = 6E-4  # Increased proportional gain
ki = 3E-6  # Increased integral gain
kd = 5E-3  # Decreased derivative gain

error = [0, 0]
errorPrev = [0, 0]
integr = [0, 0]
deriv = [0, 0]

out = [0, 0]

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.output(ENA, GPIO.LOW)

# Configure stepper motor speeds and accelerations
for stepper in [stepperA, stepperB, stepperC]:
    stepper.set_max_speed(800)  # Adjust as needed
    stepper.set_acceleration(30)  # Adjust as needed

# Add steppers to MultiStepper
steppers.add_stepper(stepperA)
steppers.add_stepper(stepperB)
steppers.add_stepper(stepperC)

# Helper Functions
def debug_log(msg):
    print(f"[{time.time():.2f}] {msg}")

def move_to(hz, nx, ny):
    global pos
    debug_log(f"move_to called with hz={hz}, nx={nx}, ny={ny}")

    target_positions = []
    for i, stepper in enumerate([stepperA, stepperB, stepperC]):
        target_angle = kinematics.compute_angle(i, hz, nx, ny)
        pos[i] = round((angOrig - target_angle) * angToStep)  # Calculate position in steps
        target_positions.append(pos[i])
        debug_log(f"Motor {chr(65 + i)}: Target angle={target_angle:.2f}, Steps={pos[i]}")

    # Move all motors concurrently to the calculated positions
    steppers.move_to(target_positions)
    while steppers.run():
        time.sleep(0.001)  # Allow motors to run concurrently

def pid_control(setpoint_x, setpoint_y):
    global detected, error, integr, deriv, out, pos

    # Get touchscreen data from serial_point
    point = get_touch_point()
    if point is not None:
        debug_log(f"Point: ({point.x}, {point.y})")
        
        if point.x != 0 and point.y != 0:
            detected = True
            for i in range(2):  # For X and Y axes
                error[i] = (CENTER_X - point.x - setpoint_x) if i == 0 else (CENTER_Y - point.y - setpoint_y)
                integr[i] += error[i]
                deriv[i] = error[i] - deriv[i]  # Calculate derivative
                out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
                out[i] = max(min(out[i], 0.25), -0.25)  # Constrain output
                debug_log(f"PID output {['X', 'Y'][i]}: error={error[i]}, integr={integr[i]}, deriv={deriv[i]}, out={out[i]}")

            # Update motor positions
            move_to(4.25, -out[0], -out[1])
        else:
            detected = False
            debug_log("Ball not detected.")
    else:
        detected = False
        debug_log("Touchscreen data is None.")

# Main Loop
def balance_ball():
    move_to(4.25, 0, 0)
    debug_log("Starting balance loop...")
    try:
        while True:
            pid_control(0, 0)  # Maintain the ball at the center (0, 0)
            time.sleep(0.01)  # Adjust delay as needed
    except KeyboardInterrupt:
        debug_log("Exiting program...")
    finally:
        GPIO.cleanup()  # Clean up GPIO or any other resources

if __name__ == "__main__":
    debug_log("Initializing...")
    balance_ball()