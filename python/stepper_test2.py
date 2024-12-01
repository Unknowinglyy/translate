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

def move_to_target(positions):
    steppers.move_to(positions)  # Calculate required speeds for all motors
    while steppers.run():  # Blocks until all steppers reach their target positions
        time.sleep(0.01)

# Main loop
def loop():
    while True:
        print("Moving to forward positions...")
        move_to_target(forward_positions)  # Move to the forward positions
        time.sleep(1)  # Pause for 1 second
        
        print("Moving to backward positions...")
        move_to_target(backward_positions)  # Move back to the starting positions
        time.sleep(1)  # Pause for 1 second

# Run the setup and loop
if __name__ == "__main__":
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        GPIO.cleanup()  # Clean up GPIO resources