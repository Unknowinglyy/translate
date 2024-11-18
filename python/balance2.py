import math
import time
import threading
from accelstepper1 import AccelStepper  # Import AccelStepper
from multistepper import MultiStepper  # Import MultiStepper
from kine2 import Kinematics  # Import the Kinematics class
from touchScreenBasicCoordOutput import read_touch_coordinates

# --------------------------------------------------------------------------------------------
# Parameters
CENTER_X, CENTER_Y = 2025, 2045  # Touchscreen center offsets
BALL_DETECTION_THRESHOLD = 20    # Ball detection range
MAX_TOTAL_STEPS = 250
angOrig = 165                    # Original angle
angToStep = 3200 / 360           # Steps per degree
ks = 20                          # Speed amplifying constant
kp, ki, kd = 4E-8, 2E-6, 7E-3    # PID constants

# Kinematics parameters
d, e, f, g = 2, 3.125, 1.75, 3.669291339
kinematics = Kinematics(d, e, f, g)

# Stepper motor setup using AccelStepper
stepperA = AccelStepper(None, 23, 24)  # (profile=None for now, STEP, DIR)
stepperB = AccelStepper(None, 20, 21)  # Adjust pins for your setup
stepperC = AccelStepper(None, 5, 6)

# MultiStepper for coordinated movement
multi_stepper = MultiStepper()
multi_stepper.add_stepper(stepperA)
multi_stepper.add_stepper(stepperB)
multi_stepper.add_stepper(stepperC)

# --------------------------------------------------------------------------------------------
# Debug Logging
def debug_log(msg):
    """
    Helper function to print debugging messages with a timestamp.
    """
    print(f"[{time.time():.2f}] {msg}")

# Dictionary to track the total steps moved by each motor
total_steps_moved = {'stepperA': 0, 'stepperB': 0, 'stepperC': 0}

def move_to(hz, nx, ny):
    """
    Moves the platform based on calculated motor positions using MultiStepper.
    """
    debug_log(f"move_to called with hz={hz}, nx={nx}, ny={ny}")
    
    absolute_positions = []
    for i, stepper in enumerate([stepperA, stepperB, stepperC]):
        target_angle = kinematics.compute_angle(chr(65 + i), hz, nx, ny)
        steps = round((angOrig - target_angle) * angToStep)  # Calculate position in steps
        
        # Ensure steps do not exceed the maximum total steps
        motor_key = f'stepper{chr(65 + i)}'
        new_total = total_steps_moved[motor_key] + steps
        if abs(new_total) > MAX_TOTAL_STEPS:
            allowed_steps = MAX_TOTAL_STEPS - abs(total_steps_moved[motor_key])
            debug_log(f"{motor_key} step limit reached. Adjusting steps to {allowed_steps}.")
            steps = allowed_steps

        total_steps_moved[motor_key] += steps
        debug_log(f"{motor_key}: Target angle={target_angle:.2f}, Steps={steps}, Total Steps={total_steps_moved[motor_key]}")

        absolute_positions.append(steps)

    # Use MultiStepper to synchronize movement
    multi_stepper.move_to(absolute_positions)
    multi_stepper.run_speed_to_position()

def pid_control(setpoint_x, setpoint_y):
    """
    PID control to move the ball to the specified setpoint.
    """
    global detected, error, integr, deriv, out, speed

    point = read_touch_coordinates()  # Get touchscreen data
    debug_log(f"Touchscreen read: x={point.x}, y={point.y}")
    if point is not None and point.x != 0:
        detected = True
        for i in range(2):
            error[i] = (CENTER_X - point.x - setpoint_x) if i == 0 else (CENTER_Y - point.y - setpoint_y)
            integr[i] += error[i]
            deriv[i] = error[i] - error[i - 1] if i > 0 else 0
            out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
            out[i] = max(min(out[i], 0.25), -0.25)  # Constrain output
            debug_log(f"PID output {['X', 'Y'][i]}: error={error[i]}, integr={integr[i]}, deriv={deriv[i]}, out={out[i]}")
    else:
        detected = False
        debug_log("Ball not detected.")

def balance_ball():
    """
    Main loop to balance the ball using PID and platform movement.
    """
    debug_log("Starting balance loop...")
    try:
        while True:
            pid_control(0, 0)
            move_to(4.25, -out[0], -out[1])
            time.sleep(0.02)  # 50 Hz control loop
    except KeyboardInterrupt:
        debug_log("Exiting program...")
    finally:
        debug_log("Cleaning up...")
        
# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    debug_log("Initializing...")
    balance_ball()
