import time
from accelstepper import AccelStepper
from multistepper import MultiStepper
import RPi.GPIO as GPIO
from kine2 import Kinematics  # Import the Kinematics class
from touchScreenTranslatedCoordOutput import *
import math
'''
    Initalize motor positions to [0,40,65][0,40,40] [0, 120, 100] [0, 135, 80] (motor2, motor1, motor3) (A,B,C) to have flat surface
    angToStep = 1100/360
    angOrig = 206
'''
# Define GPIO pins for the stepper motor
ENA = 17

# Constants and Parameters
CENTER_X, CENTER_Y = 500, 500  # Touchscreen center offsets
angOrig = 206                    # Original angle
angToStep = 1100 / 360           # Steps per degree
ks = 30                         # Speed amplifying constant
kp, ki, kd = .00034, 0, 0.0007    # PID constants

# Global variables for PID control
error = [0, 0]  # Error for X and Y axes
error_prev = [0, 0]  # Previous error for X and Y axes
integr = [0, 0]  # Integral term for X and Y axes
deriv = [0, 0]  # Derivative term for X and Y axes
out = [0, 0]  # PID output for X and Y axes
pos = [0, 0, 0]  # Position of each motor
speed = [0, 0, 0]    # Speed for each motor
speed_prev = [0, 0, 0]  # Previous speed for each motor
detected = False  # Ball detection flag

d, e, f, g = 2, 3.125, 1.75, 3.669291339
kinematics = Kinematics(d, e, f, g)

GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.output(ENA, GPIO.LOW)

# Initialize stepper motors
'''
hardware wiring:
    motor 1: 23 24      
    motor 2: 20 21      13 19
    motor 3: 5 6
working: 
    A: 20 21            13 19
    B: 5 6
    C: 23 24
''' 
stepperA = AccelStepper(AccelStepper.DRIVER, 13, 19)   # aka "Motor A"
stepperB = AccelStepper(AccelStepper.DRIVER, 5, 6)    # aka "Motor B"
stepperC = AccelStepper(AccelStepper.DRIVER, 23, 24)     # aka "Motor C"


# Configure stepper motor speeds and accelerations
for stepper in [stepperA, stepperB, stepperC]:
    stepper.set_max_speed(1000)  # Adjust as needed
    stepper.set_acceleration(150000)  # Adjust as needed

# Create a MultiStepper instance
multi_stepper = MultiStepper()
multi_stepper.add_stepper(stepperA)
multi_stepper.add_stepper(stepperB)
multi_stepper.add_stepper(stepperC)

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
    multi_stepper.move_to(target_positions)
    while multi_stepper.run():
        time.sleep(0.005)  # Allow motors to run concurrently

def pid_control(setpoint_x, setpoint_y):
    global detected, error, error_prev, integr, deriv, out, pos

    # Get touchscreen data (original coordinates)
    orig_point = read_coordinates()
    if orig_point is not None:
        # Transform to translated coordinates
        point = transform_coordinates(orig_point.x, orig_point.y)
        # point = orig_point
        debug_log(f"Point: ({point.x}, {point.y})")

        if point.x != 0 and point.y != 0:
            detected = True
            for i in range(2):  # For X and Y axes
                # Update PID terms
                error_prev[i] = error[i]
                error[i] = (CENTER_X - point.x - setpoint_x) if i == 0 else (CENTER_Y - point.y - setpoint_y)
                integr[i] += error[i] + error_prev[i]
                deriv[i] = error[i] - error_prev[i]
                deriv[i] = 0 if math.isnan(deriv[i]) or math.isinf(deriv[i]) else deriv[i]
                out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
                out[i] = max(min(out[i], 0.25), -0.25)  # Constrain output
                debug_log(f"PID output {['X', 'Y'][i]}: error={error[i]}, integr={integr[i]}, deriv={deriv[i]}, out={out[i]}")
        else:
            detected = False
            debug_log("Ball not detected on first check.")
            time.sleep(0.01)  # 10 ms delay
            orig_point = read_coordinates()  # Check again
            if orig_point is None or (orig_point.x == 0 and orig_point.y == 0):
                detected = False
                debug_log("Ball not detected on second check.")
    else:
        detected = False
        debug_log("Touchscreen data is None.")

    move_to(4.25, -out[0], -out[1])

def balance_ball():
    try:
        while True:
            # Ask the user for 3 numbers
            positions = []
            for i in range(3):
                if i == 0:
                    pos = float(input(f"Enter position for motor A: "))
                elif i == 1:
                    pos = float(input(f"Enter position for motor B: "))
                else:
                    pos = float(input(f"Enter position for motor C: "))
                positions.append(pos)
            
            # Move the motors to the specified positions
            multi_stepper.move_to(positions)
            multi_stepper.run_speed_to_position()
            
            # Optional: Add a small delay to avoid overwhelming the system
            time.sleep(0.1)
    except KeyboardInterrupt:
        debug_log("Exiting program...")
    finally:
        GPIO.cleanup()  # Clean up GPIO or any other resources

if __name__ == "__main__":
    debug_log("Initializing...")
    balance_ball()
