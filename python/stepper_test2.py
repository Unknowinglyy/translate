import RPi.GPIO as GPIO
from accelstepper import AccelStepper
from multistepper import MultiStepper
import time

# Initialize GPIO
GPIO.setmode(GPIO.BCM)

# Define enable pin
ENA = 17  # GPIO pin for enable

# Initialize stepper motors
stepperA = AccelStepper(AccelStepper.DRIVER, 13, 19)  # (driver type, STEP, DIR) Driver A
stepperB = AccelStepper(AccelStepper.DRIVER, 23, 24)  # (driver type, STEP, DIR) Driver B
stepperC = AccelStepper(AccelStepper.DRIVER, 5, 6)  # (driver type, STEP, DIR) Driver C

# Initialize MultiStepper
steppers = MultiStepper()

# Stepper motor variables
forward_positions = [400, 400, 400]  # Target positions for each stepper motor
backward_positions = [0, 0, 0]

# Setup function
def setup():
    # Set initial maximum speed for the steppers (steps/sec)
    stepperA.set_max_speed(1200)
    stepperB.set_max_speed(400)
    stepperC.set_max_speed(800)
    
    # Add the steppers to the MultiStepper instance
    steppers.add_stepper(stepperA)
    steppers.add_stepper(stepperB)
    steppers.add_stepper(stepperC)
    
    # Configure enable pin
    GPIO.setup(ENA, GPIO.OUT)
    GPIO.output(ENA, GPIO.LOW)  # Enable the stepper drivers
    time.sleep(1)  # Small delay to allow the user to reset the platform

# Move motors simultaneously to their positions
def move_to_positions_simultaneously(steppers, positions):
    for stepper, position in zip(steppers, positions):
        stepper.move_to(position)

    # Continue running all motors until they reach their positions
    while any(stepper.distance_to_go() != 0 for stepper in steppers):
        for stepper in steppers:
            stepper.run_speed()
        time.sleep(0.01)  # Small delay for smooth operation

def loop():
    steppers = [stepperA, stepperB, stepperC]
    
    while True:
        print("Moving to forward positions...")
        move_to_positions_simultaneously(steppers, forward_positions)

        time.sleep(1)  # Pause for 1 second

        print("Moving to backward positions...")
        move_to_positions_simultaneously(steppers, backward_positions)

        time.sleep(1)  # Pause for 1 second

# Run the setup and loop
if __name__ == "__main__":
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        GPIO.cleanup()  # Clean up GPIO resources