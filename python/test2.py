import time
import math
from accelstepper import AccelStepper
from multistepper import MultiStepper
import RPi.GPIO as GPIO

# Define GPIO pins for the 3 stepper motors
STEP_PIN1 = 23
DIR_PIN1 = 24

STEP_PIN2 = 34
DIR_PIN2 = 35

STEP_PIN3 = 5
DIR_PIN3 = 6

ENA_PIN = 17

# Initialize GPIO
GPIO.setmode(GPIO.BCM)

GPIO.setup(ENA_PIN, GPIO.OUT)
GPIO.output(ENA_PIN, GPIO.LOW)

# Define stepper motors
motor1 = AccelStepper(AccelStepper.DRIVER, STEP_PIN1, DIR_PIN1)
motor2 = AccelStepper(AccelStepper.DRIVER, STEP_PIN2, DIR_PIN2)
motor3 = AccelStepper(AccelStepper.DRIVER, STEP_PIN3, DIR_PIN3)

# Initialize MultiStepper and add motors
multi_stepper = MultiStepper()
multi_stepper.add_stepper(motor1)
multi_stepper.add_stepper(motor2)
multi_stepper.add_stepper(motor3)

# Set motor properties (max speed, acceleration, etc.)
motor1.set_max_speed(1000)
motor1.set_acceleration(500)

motor2.set_max_speed(1000)
motor2.set_acceleration(500)

motor3.set_max_speed(1000)
motor3.set_acceleration(500)

motor1.set_current_position(0)
motor2.set_current_position(0)
motor3.set_current_position(0)

# Function to calculate wave-like motion
def move_motors_in_wave(amplitude, frequency, duration):
    """
    Moves the motors in a wave-like motion using sine functions.

    :param amplitude: The maximum displacement (steps) from the center position.
    :param frequency: The frequency of the wave (how fast the motors oscillate).
    :param duration: How long the wave motion should run.
    """
    start_time = time.time()
    while time.time() - start_time < duration:
        # Calculate time elapsed
        elapsed_time = time.time() - start_time

        # Calculate the position for each motor based on a sine wave
        position1 = amplitude * math.sin(2 * math.pi * frequency * elapsed_time)
        position2 = amplitude * math.sin(2 * math.pi * frequency * elapsed_time + math.pi / 2)  # 90 degrees phase shift
        position3 = amplitude * math.sin(2 * math.pi * frequency * elapsed_time + math.pi)  # 180 degrees phase shift

        # Move the motors to the calculated positions
        motor1.move_to(round(position1))
        motor2.move_to(round(position2))
        motor3.move_to(round(position3))

        # Start moving the motors
        multi_stepper.run()

        # Small delay to avoid excessive CPU usage
        time.sleep(0.01)

def main():
    try:
        # Move motors in a wave-like motion with a 200-step amplitude, 0.5 Hz frequency, and 10 seconds duration
        move_motors_in_wave(200, 0.5, 10)  # Adjust the amplitude, frequency, and duration as needed

        # After the wave motion, stop motors and clean up
        GPIO.cleanup()

    except KeyboardInterrupt:
        print("Program interrupted.")
        GPIO.cleanup()

if __name__ == "__main__":
    main()
