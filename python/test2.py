import time
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

def move_motors_to_angle(angle1, angle2, angle3):
    # Convert angles to steps based on your system's specifications
    steps1 = angle1 * 3200 / 360  # Example: 3200 steps for 360 degrees
    steps2 = angle2 * 3200 / 360
    steps3 = angle3 * 3200 / 360

    # Move motors to the target positions
    print(f"Moving motors to angles: {angle1}°, {angle2}°, {angle3}°")
    multi_stepper.move_to([steps1, steps2, steps3])

    # Wait until all motors reach their positions
    while multi_stepper.run():
        time.sleep(0.001)  # Small delay for smooth operation

def main():
    try:
        # Move motors to tilt platform to some angle
        move_motors_to_angle(0, 0, 45)  # Example: Tilt all motors to 45 degrees

        time.sleep(1)  # Wait for 1 second

        # Return motors to initial positions (0 degrees)
        print("Returning motors to initial position")
        move_motors_to_angle(0, 0, 0)

        time.sleep(1)  # Wait for 1 second

        # Move motors to a different angle
        move_motors_to_angle(30, 60, 90)  # Example: Tilt motors to different angles

        time.sleep(1)  # Wait for 1 second

    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        print("Cleaning up GPIO")
        GPIO.cleanup()

if __name__ == "__main__":
    main()
