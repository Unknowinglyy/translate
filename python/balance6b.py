import time
from accelstepper import AccelStepper
from multistepper import MultiStepper
import RPi.GPIO as GPIO
from kine2 import Kinematics  # Import the Kinematics class
from touchScreenTranslatedCoordOutput import *
import math

# --------------------------------------------------------------------------------------------------------------
# Define GPIO pins for the stepper motor
ENA = 17

# Constants and Parameters
CENTER_X, CENTER_Y = 500, 500  # Touchscreen center offsets
angOrig = 206.662752199                    # Original angle
angToStep = 1100 / 360           # Steps per degree
ks = 30                         # Speed amplifying constant
kp, ki, kd = .00034, 0.000002, 0.0007    # PID constants

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

# --------------------------------------------------------------------------------------------------------------
# Helper Functions
def debug_log(msg):
    print(f"[{time.time():.2f}] {msg}")

# --------------------------------------------------------------------------------------------------------------
def move_to(hz, nx, ny):
    """
    Moves the platform to the desired position while setting appropriate speed and acceleration.
    """
    global pos, speed

    debug_log(f"move_to called with hz={hz}, nx={nx}, ny={ny}")

    target_positions = []
    for i, stepper in enumerate([stepperA, stepperB, stepperC]):
        target_angle = kinematics.compute_angle(i, hz, nx, ny)
        pos[i] = round((angOrig - target_angle) * angToStep)  # Calculate position in steps
        target_positions.append(pos[i])
        debug_log(f"Motor {chr(65 + i)}: Target angle={target_angle:.2f}, Steps={pos[i]}")

        # Calculate speed based on position difference
        distance = abs(pos[i] - stepper.current_position())
        speed[i] = distance * ks  # Adjust `ks` as needed
        speed[i] = max(min(speed[i], 1000), 0)  # Constrain speed to a reasonable range

        # Set speed and acceleration
        stepper.set_max_speed(speed[i])
        stepper.set_acceleration(speed[i] * 10)  # Adjust acceleration factor as needed

    # Move all motors concurrently to the calculated positions
    multi_stepper.move_to(target_positions)
    while multi_stepper.run():
        time.sleep(0.005)  # Allow motors to run concurrently

def pid_control(setpoint_x, setpoint_y):
    global detected, error, error_prev, integr, deriv, out, pos, speed, speed_prev

    # Get touchscreen data (original coordinates)
    orig_point = read_coordinates()
    if orig_point is not None:
        # Transform to translated coordinates
        point = transform_coordinates(orig_point.x, orig_point.y)
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

            # Calculate stepper motor speeds
            for i, stepper in enumerate([stepperA, stepperB, stepperC]):
                speed_prev[i] = speed[i]
                speed[i] = abs(stepper.current_position() - pos[i]) * ks
                speed[i] = max(min(speed[i], speed_prev[i] + 200), speed_prev[i] - 200)  # Filter speed changes
                speed[i] = max(min(speed[i], 1000), 0)  # Constrain speed
                stepper.set_max_speed(speed[i])
                stepper.set_acceleration(speed[i] * 10)  # Adjust acceleration factor as needed
                debug_log(f"Motor {chr(65 + i)}: Speed={speed[i]}, Acceleration={speed[i] * 10}")

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

def setup():
    while True:
        quit_ = input("Begin offset setup? (y/n): ")
        if quit_.lower() == 'n':
            break    

        # Ask the user for 3 numbers
        positions = []
        for i in range(3):
            user_input = input(f"Enter position for motor {['A', 'B', 'C'][i]} (or 'q' to quit): ")
            if user_input.lower() == 'q':
                print("Exiting setup...")
                return  # Exit the setup function
            try:
                pos = float(user_input)
                positions.append(pos)
            except ValueError:
                print("Invalid input. Please enter a valid number or 'q' to quit.")
                break  # Break the inner loop to re-prompt for the current motor

        if len(positions) == 3:
            # Move the motors to the specified positions
            multi_stepper.move_to(positions)
            multi_stepper.run_speed_to_position()

            # Small Delay
            time.sleep(0.1)

            stepperA.current_position = 0
            stepperB.current_position = 0
            stepperC.current_position = 0
            print(f"Current positions: A={stepperA.current_position}, B={stepperB.current_position}, C={stepperC.current_position}")

# Main Loop
def balance_ball():
    prep_time = 5

    move_to(4.25, 0, 0)
    setup()
    time.sleep(prep_time)
    debug_log(f"Starting balance loop in {prep_time} seconds...")
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