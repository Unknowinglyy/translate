import math
import RPi.GPIO as GPIO
import time
from kine2 import Kinematics  # Import the Kinematics class
from touchScreenBasicCoordOutput import read_touch_coordinates

# --------------------------------------------------------------------------------------------
# GPIO setup for stepper motors
MOTOR_PINS = {
    'motor1': {'step': 23, 'dir': 24},
    'motor2': {'step': 20, 'dir': 21},
    'motor3': {'step': 5, 'dir': 6}
}

# Parameters
CENTER_X, CENTER_Y = 2025, 2045  # Touchscreen center offsets
BALL_DETECTION_THRESHOLD = 20    # Ball detection range
angOrig = 180          # Original angle
angToStep = 3200 / 360           # Steps per degree
ks = 20                          # Speed amplifying constant
kp, ki, kd = 4E-4, 2E-6, 7E-3    # PID constants

# Kinematics parameters
d, e, f, g = 2, 3.125, 1.75, 3.669291339
kinematics = Kinematics(d, e, f, g)

# Initialize stepper variables
pos = [0, 0, 0]
speed = [0, 0, 0]
speedPrev = [0, 0, 0]
error = [0, 0]
integr = [0, 0]
deriv = [0, 0]
out = [0, 0]
detected = False

# GPIO setup
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

def move_motor(motor, steps, clockwise):
    """
    Moves a single motor a specified number of steps in a specified direction.
    """
    debug_log(f"Moving {motor}: steps={steps}, clockwise={clockwise}")
    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(abs(steps)):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(0.001)

def move_to(hz, nx, ny):
    """
    Moves the platform based on calculated motor positions.
    """
    global detected, pos
    print(f"move_to: hz={hz}, nx={nx}, ny={ny}, detected={detected}")

    for i in range(3):
        target_angle = kinematics.compute_angle(chr(65 + i), hz, nx, ny)
        pos[i] = round((angOrig - target_angle) * angToStep)  # Calculate position in steps
        print(f"Motor {chr(65 + i)}: Target angle={target_angle:.2f}, Steps={pos[i]}")

    for i, motor in enumerate(MOTOR_PINS.keys()):
        steps = abs(pos[i]) // 8
        clockwise = pos[i] > 0
        move_motor(motor, steps, clockwise)

def pid_control(setpoint_x, setpoint_y):
    """
    PID control to move the ball to the specified setpoint.
    """
    global detected, error, integr, deriv, out, speed

    point = read_touch_coordinates()  # Get touchscreen data
    debug_log(f"Touchscreen read: {point}")
    if point is not None and point.x != 0:
        detected = True
        for i in range(2):
            error[i] = (CENTER_X - point.x - setpoint_x) if i == 0 else (CENTER_Y - point.y - setpoint_y)
            integr[i] += error[i]
            deriv[i] = error[i] - error[i - 1] if i > 0 else 0
            out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
            out[i] = max(min(out[i], 0.25), -0.25)  # Constrain output
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
            time.sleep(0.02)
    except KeyboardInterrupt:
        debug_log("Exiting program...")
    finally:
        GPIO.cleanup()

# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    debug_log("Initializing...")
    balance_ball()
