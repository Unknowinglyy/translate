import math
import RPi.GPIO as GPIO
import time
from simple_pid import PID
from touchScreenBasicCoordOutput import read_touch_coordinates
import threading
from kine2 import compute_angle  

# --------------------------------------------------------------------------------------------
# GPIO setup for stepper motors
MOTOR_PINS = {
    'motor1': {'step': 23, 'dir': 24},
    'motor2': {'step': 20, 'dir': 21},
    'motor3': {'step': 5, 'dir': 6}
}

# Center position of the touchscreen
CENTER_X, CENTER_Y = 2005, 2033.5
# Ball detection thresholds
BALL_DETECTION_THRESHOLD = 10

# --------------------------------------------------------------------------------------------
# GPIO Setup
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

# PID controllers for X and Y directions
pid_x = PID(1.2, 0.1, 0.05, setpoint=CENTER_X)
pid_y = PID(1.2, 0.1, 0.05, setpoint=CENTER_Y)

# Configure sample time (update frequency) and output limits
pid_x.sample_time = 0.01  # 10 ms update rate
pid_y.sample_time = 0.01
pid_x.output_limits = (-10, 10)  # Limit to ±10 steps
pid_y.output_limits = (-10, 10)

# Velocity tracking variables
prev_time = time.time()
prev_x, prev_y = CENTER_X, CENTER_Y

# --------------------------------------------------------------------------------------------
def move_motor(motor, steps, clockwise):
    """
    Moves a single motor a specified number of steps in a specified direction.
    """
    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(abs(steps)):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(0.001)

def calculate_motor_steps(ball_x, ball_y, velocity_x, velocity_y):
    """
    Calculates the motor movements required to tilt the plane based on ball position and velocity.
    """
    if abs(ball_x - CENTER_X) < BALL_DETECTION_THRESHOLD and abs(ball_y - CENTER_Y) < BALL_DETECTION_THRESHOLD:
        return {motor: (0, True) for motor in MOTOR_PINS}  # No movement if ball is near center.

    # Adjust PID controllers with velocity inputs (D-term inherently handles velocity response)
    pid_x.set_auto_mode(True, last_output=velocity_x)
    pid_y.set_auto_mode(True, last_output=velocity_y)
    
    # Compute PID outputs
    steps_x = int(pid_x(ball_x))
    steps_y = int(pid_y(ball_y))

    # Use kinematics to compute angles and convert to motor steps
    motor_angles = compute_angle(steps_x, steps_y)  # Returns angles for each motor
    motor_steps = {
        'motor1': (int(motor_angles['A']), motor_angles['A'] > 0),
        'motor2': (int(motor_angles['B']), motor_angles['B'] > 0),
        'motor3': (int(motor_angles['C']), motor_angles['C'] > 0),
    }

    return motor_steps

def move_motors_concurrently(motor_steps):
    """
    Moves the motors concurrently using threading.
    """
    threads = []
    for motor, (steps, clockwise) in motor_steps.items():
        if steps > 0:
            t = threading.Thread(target=move_motor, args=(motor, steps, clockwise))
            threads.append(t)
            t.start()
    for t in threads:
        t.join()

def balance_ball():
    """
    Main loop to balance the ball using PID, motor control, and velocity calculations.
    """
    global prev_time, prev_x, prev_y

    try:
        while True:
            point = read_touch_coordinates()
            if point is None:
                continue

            ball_x, ball_y = point.x, point.y
            current_time = time.time()

            # Calculate velocity
            dt = current_time - prev_time if current_time != prev_time else 0.01
            velocity_x = (ball_x - prev_x) / dt
            velocity_y = (ball_y - prev_y) / dt

            # Update previous values
            prev_x, prev_y = ball_x, ball_y
            prev_time = current_time

            # Calculate motor steps based on position and velocity
            motor_steps = calculate_motor_steps(ball_x, ball_y, velocity_x, velocity_y)

            # Move each motor according to the calculated steps
            move_motors_concurrently(motor_steps)

            time.sleep(0.01)  # Update cycle delay (10 ms)
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        GPIO.cleanup()

# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    # Centering motors before starting
    print("Centering motors...")
    for _ in range(100):  # Arbitrary 100 steps to center
        move_motor('motor1', 1, True)
        move_motor('motor2', 1, True)
        move_motor('motor3', 1, True)
    print("Motors centered. Starting balance loop...")

    balance_ball()
