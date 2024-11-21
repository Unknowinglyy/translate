import time
from accelstepper import AccelStepper
from multistepper import MultiStepper
import RPi.GPIO as GPIO 

# Define GPIO pins for the stepper motor
STEP_PIN = 23
DIR_PIN = 24
ENA = 17


GPIO.setmode(GPIO.BCM)

GPIO.setup(ENA, GPIO.OUT)
GPIO.output(ENA, GPIO.LOW)

GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

def main():
    # Initialize the stepper motor
    motor = AccelStepper(AccelStepper.DRIVER, STEP_PIN, DIR_PIN)
    
    # Initialize the MultiStepper and add the motor
    multi_stepper = MultiStepper()
    multi_stepper.add_stepper(motor)

    # Set motor properties
    motor.set_max_speed(1000)  # Maximum speed in steps per second
    motor.set_acceleration(1000)  # Acceleration in steps per second^2
    motor.set_current_position(0)  # Reset current position to 0

    # Move up 20 steps
    print("Moving up 20 steps")
    multi_stepper.move_to([100])  # Target position for the motor
    while multi_stepper.run():  # Keep running until all motors reach their positions
        time.sleep(0.0001)  # Small delay for smooth operation

    time.sleep(1)  # Pause for 1 second at the top

    # Move back down 20 steps
    print("Moving down 20 steps")
    multi_stepper.move_to([0])  # Return to initial position
    while multi_stepper.run():  # Keep running until all motors reach their positions
        time.sleep(0.0001)  # Small delay for smooth operation

    print("Done!")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        print("cleaning up GPIO")
        GPIO.cleanup()
