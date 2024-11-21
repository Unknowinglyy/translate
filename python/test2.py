import time
import math
from accelstepper import AccelStepper
from multistepper import MultiStepper
from touchScreenBasicCoordOutput import read_touch_coordinates
from kine2 import Kinematics

# GPIO Pins for the steppers
STEP_PINS = [23, 20, 5]
DIR_PINS = [24, 21, 6]
ENA_PIN = 17

# Initialize kinematics and steppers
kinematics = Kinematics(2, 3.125, 1.75, 3.669291339)

# Proportional-Integral-Derivative (PID) parameters
kp = 4E-4  # Proportional gain
ki = 2E-6  # Integral gain
kd = 7E-3  # Derivative gain

# Touchscreen center offsets
xoffset = 500
yoffset = 500

# Initial setup
angOrig = 206.662752199  # Original angle of the platform
angToStep = 3200 / 360   # Steps per degree
pos = [0, 0, 0]          # Target positions
error = [0, 0]
errorPrev = [0, 0]
integr = [0, 0]
speed = [0, 0, 0]
speedPrev = [0, 0, 0]
ks = 20  # Speed amplification factor

# Initialize GPIO for enable pin
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA_PIN, GPIO.OUT)
GPIO.output(ENA_PIN, GPIO.LOW)  # Enable drivers

# Initialize steppers
steppers = MultiStepper()
stepper_list = []
for step_pin, dir_pin in zip(STEP_PINS, DIR_PINS):
    stepper = AccelStepper(AccelStepper.DRIVER, step_pin, dir_pin)
    stepper.set_max_speed(1000)
    stepper.set_acceleration(500)
    stepper.set_current_position(0)
    steppers.add_stepper(stepper)
    stepper_list.append(stepper)

def constrain(value, minn, maxn):
    return max(min(maxn, value), minn)

def compute_motor_positions(hz, nx, ny):
    """Compute motor positions based on kinematics."""
    for i in range(3):
        angle = kinematics.compute_angle(i, hz, nx, ny)
        pos[i] = round((angOrig - angle) * angToStep)

def PID_control(setpointX, setpointY):
    """Calculate PID output for balancing."""
    global error, errorPrev, integr, speed, speedPrev

    point = read_touch_coordinates()
    print(f"Touchscreen coordinates: x={point.x}, y={point.y}")

    if point.x != 0:
        detected = True
        for i in range(2):  # Loop for X and Y axes
            errorPrev[i] = error[i]
            error[i] = (xoffset - point.x - setpointX if i == 0 else yoffset - point.y - setpointY)
            integr[i] += error[i] + errorPrev[i]
            deriv = error[i] - errorPrev[i]
            out = kp * error[i] + ki * integr[i] + kd * deriv
            out = constrain(out, -0.25, 0.25)

        compute_motor_positions(4.25, -out[0], -out[1])

        # Calculate and apply motor speeds
        for i, stepper in enumerate(stepper_list):
            speedPrev[i] = speed[i]
            speed[i] = abs(stepper.current_position() - pos[i]) * ks
            speed[i] = constrain(speed[i], speedPrev[i] - 200, speedPrev[i] + 200)
            speed[i] = constrain(speed[i], 0, 1000)

        steppers.move_to(pos)
        while steppers.run():
            time.sleep(0.0001)

    else:
        print("No ball detected, resetting.")
        time.sleep(0.01)

def main():
    """Main loop to balance the ball."""
    print("Starting ball balancing...")
    while True:
        PID_control(0, 0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopping program.")
    finally:
        GPIO.cleanup()
