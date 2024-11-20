# no accel/multistepper yet

import math
import RPi.GPIO as GPIO
import time
from kine2 import Kinematics 
from touchScreenBasicCoordOutput import read_touch_coordinates
import threading

# --------------------------------------------------------------------------------------------
# GPIO setup for stepper motors
MOTOR_PINS = {
    'motor1': {'step': 23, 'dir': 24},
    'motor2': {'step': 20, 'dir': 21},
    'motor3': {'step': 5, 'dir': 6}
}

CENTER_X, CENTER_Y = 2025, 2045  # Touchscreen center offsets
BALL_DETECTION_THRESHOLD = 10    # Ball detection range
MAX_CLOCKWISE_STEPS = 400
angOrig = 165                    # Original angle
angToStep = 3200 / 360           # Steps per degree
ks = 20                          # Speed amplifying constant
kp, ki, kd = 4E-8, 2E-6, 7E-6    # PID constants

# Kinematics parameters
d, e, f, g = 2, 3.125, 1.75, 3.669291339
kinematics = Kinematics(d, e, f, g)

# stepper variables
pos = [0, 0, 0]
speed = [0, 0, 0]
speedPrev = [0, 0, 0]
error = [0, 0]
integr = [0, 0]
deriv = [0, 0]
out = [0, 0]
detected = False

GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

# --------------------------------------------------------------------------------------------
def debug_log(msg):
    """
    Helper function to print debugging messages with a timestamp.
    """
    print(f"[{time.time():.2f}] {msg}")

# Step limits for each motor
motor_positions = {motor: 0 for motor in MOTOR_PINS.keys()}  # Tracks current position of each motor

def move_motor(motor, steps, clockwise):
    """
    Moves a single motor a specified number of steps in a specified direction.
    Restricts movement to within 0 (starting position) and 400 (maximum clockwise steps).
    """
    global motor_positions

    # Calculate the proposed new position
    change = steps if clockwise else -steps
    new_position = motor_positions[motor] + change

    # Enforce movement constraints
    if new_position > MAX_CLOCKWISE_STEPS:
        # Adjust steps to stay within the upper limit
        allowed_steps = MAX_CLOCKWISE_STEPS - motor_positions[motor]
        steps = max(allowed_steps, 0)
        change = steps
        debug_log(f"{motor} clockwise limit reached. Adjusting steps to {allowed_steps}.")
    elif new_position < 0:
        # Prevent moving below the starting position
        allowed_steps = abs(motor_positions[motor])
        steps = max(allowed_steps, 0)
        change = -allowed_steps
        debug_log(f"{motor} counterclockwise limit reached. Adjusting steps to {allowed_steps}.")

    if steps <= 0:
        debug_log(f"{motor} cannot move further. No steps executed.")
        return

    debug_log(f"Moving {motor}: steps={steps}, clockwise={clockwise}, current_position={motor_positions[motor]}")
    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(abs(steps)):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(0.001)

    # Update the motor position
    motor_positions[motor] += change
    debug_log(f"Updated {motor} position to {motor_positions[motor]}")

def move_motors_concurrently(motor_steps):
    """
    Moves the motors concurrently using threading.
    """
    threads = []
    for motor, (steps, clockwise) in motor_steps.items():
        if steps > 0:  # Only move motors with non-zero steps
            t = threading.Thread(target=move_motor, args=(motor, steps, clockwise))
            threads.append(t)
            t.start()
    for t in threads:
        t.join()

def move_to(hz, nx, ny):
    """
    Moves the platform based on calculated motor positions using threading.
    """
    global detected, pos
    debug_log(f"move_to called with hz={hz}, nx={nx}, ny={ny}, detected={detected}")

    motor_steps = {}

    for i, motor in enumerate(MOTOR_PINS.keys()):
        target_angle = kinematics.compute_angle(chr(65 + i), hz, nx, ny)
        pos[i] = round((angOrig - target_angle) * angToStep)  # Calculate position in steps
        steps = abs(pos[i]) // 16  # Adjust step scaling if necessary
        clockwise = pos[i] > 0
        motor_steps[motor] = (steps, clockwise)
        debug_log(f"Motor {chr(65 + i)}: Target angle={target_angle:.2f}, Steps={steps}, Clockwise={clockwise}")

    # Use threading to move motors concurrently
    move_motors_concurrently(motor_steps)

def pid_control(setpoint_x, setpoint_y):
    """
    PID control to move the ball to the specified setpoint.
    """
    global detected, error, integr, deriv, out, speed

    point = read_touch_coordinates() 
    print(f"Touchscreen read: {point.x} {point.y}")
    if point is not None and point.x != 0:
        detected = True
        for i in range(2):
            error[i] = (CENTER_X - point.x - setpoint_x) if i == 0 else (CENTER_Y - point.y - setpoint_y)
            integr[i] += error[i]
            deriv[i] = error[i] - error[i - 1] if i > 0 else 0
            out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
            out[i] = max(min(out[i], 0.25), -0.25) 
            debug_log(f"PID output {['X', 'Y'][i]}: error={error[i]}, integr={integr[i]}, deriv={deriv[i]}, out={out[i]}")

        for i in range(3):
            speedPrev[i] = speed[i]
            speed[i] = abs(pos[i] - int(pos[i])) * ks
            speed[i] = max(min(speed[i], speedPrev[i] + 200), speedPrev[i] - 200)
            speed[i] = max(min(speed[i], 1000), 0)
            debug_log(f"Motor {chr(65 + i)} speed: {speed[i]}")
    else:
        detected = False
        debug_log("Ball not detected. Verifying...")
        time.sleep(0.01)
        point = read_touch_coordinates()
        if point is None or point.x == 0:
            detected = False
            debug_log("Ball confirmed not detected.")

def balance_ball():
    """
    Main loop to balance the ball using PID and platform movement.
    """
    debug_log("Starting balance loop...")
    try:
        while True:
            pid_control(0, 0)
            move_to(4.25, -out[0], -out[1])
            time.sleep(0.001)
    except KeyboardInterrupt:
        debug_log("Exiting program...")
    finally:
        GPIO.cleanup()

# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    debug_log("Initializing...")
    # # Centering motors before starting
    # print("Centering motors...")
    # for _ in range(150):  
    #     move_motor('motor1', 1, True)
    #     move_motor('motor2', 1, True)
    #     move_motor('motor3', 1, True)
    balance_ball()