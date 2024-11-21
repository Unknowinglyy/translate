import time
from accelstepper import AccelStepper
from multistepper import MultiStepper
import RPi.GPIO as GPIO
from kine2 import Kinematics  # Import the Kinematics class
from touchScreenBasicCoordOutput import read_touch_coordinates

# Define GPIO pins for the stepper motor
STEP_PIN = 23
DIR_PIN = 24
ENA = 17

# Constants and Parameters
CENTER_X, CENTER_Y = 100, 100  # Touchscreen center offsets
BALL_DETECTION_THRESHOLD = 20    # Ball detection range
angOrig = 206                    # Original angle
angToStep = 1600 / 360           # Steps per degree
ks = 20                          # Speed amplifying constant
kp, ki, kd = 4E-4, 2E-6, 7E-3    # PID constants

# Global variables for PID control
error = [0, 0]  # Error for X and Y axes
integr = [0, 0]  # Integral term for X and Y axes
deriv = [0, 0]  # Derivative term for X and Y axes
out = [0, 0]  # PID output for X and Y axes
pos = [0, 0, 0]  # Position of each motor
detected = False  # Ball detection flag

d, e, f, g = 2, 3.125, 1.75, 3.669291339
kinematics = Kinematics(d, e, f, g)

GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.output(ENA, GPIO.LOW)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# Initialize stepper motors
stepper1 = AccelStepper(AccelStepper.DRIVER, 5, 6)
stepper2 = AccelStepper(AccelStepper.DRIVER, 20, 21)
stepper3 = AccelStepper(AccelStepper.DRIVER, 23, 24)

# Configure stepper motor speeds and accelerations
for stepper in [stepper1, stepper2, stepper3]:
    stepper.set_max_speed(100000)  # Adjust as needed
    stepper.set_acceleration(100)  # Adjust as needed

# Create a MultiStepper instance
multi_stepper = MultiStepper()
multi_stepper.add_stepper(stepper1)
multi_stepper.add_stepper(stepper2)
multi_stepper.add_stepper(stepper3)

# Helper Functions
def debug_log(msg):
    print(f"[{time.time():.2f}] {msg}")

def move_to(hz, nx, ny):
    global pos
    debug_log(f"move_to called with hz={hz}, nx={nx}, ny={ny}")

    target_positions = []
    for i, stepper in enumerate([stepper1, stepper2, stepper3]):
        target_angle = kinematics.compute_angle(chr(65 + i), hz, nx, ny)
        pos[i] = round((angOrig - target_angle) * angToStep)  # Calculate position in steps
        target_positions.append(pos[i])
        debug_log(f"Motor {chr(65 + i)}: Target angle={target_angle:.2f}, Steps={pos[i]}")

    # Move all motors concurrently to the calculated positions
    multi_stepper.move_to(target_positions)
    while multi_stepper.run():
        time.sleep(0.001)  # Allow motors to run concurrently

def pid_control(setpoint_x, setpoint_y):
    global detected, error, integr, deriv, out, pos

    point = read_touch_coordinates()  # Get touchscreen data
    debug_log(f"Touchscreen point: {point.x} {point.y}")
    if point is not None and point.x != 0:
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

# Main Loop
def balance_ball():
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
