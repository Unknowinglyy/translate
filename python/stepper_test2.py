from accelstepper import AccelStepper
from multistepper import MultiStepper
import RPi.GPIO as GPIO
import time 

# Setup GPIO
GPIO.setmode(GPIO.BCM)

# Parameters = driver type, step pin, direction pin
stepper1 = AccelStepper(AccelStepper.DRIVER, 23, 24)
stepper2 = AccelStepper(AccelStepper.DRIVER, 13, 19)
stepper3 = AccelStepper(AccelStepper.DRIVER, 5, 6)

steppers = MultiStepper()

# Target positions for each motor
pos_up = [400, 400, 400]  # Move up
pos_down = [0, 0, 0]      # Move back down

# Enable pin for drivers
ENA = 17
GPIO.setup(ENA, GPIO.OUT)
GPIO.output(ENA, GPIO.LOW)

# Small delay to allow system to settle
time.sleep(1)

# Configure maximum speed for steppers
stepper1.set_max_speed(400)
stepper2.set_max_speed(400)
stepper3.set_max_speed(400)
print("Set all the speeds")

# Add steppers to MultiStepper
steppers.add_stepper(stepper1)
steppers.add_stepper(stepper2)
steppers.add_stepper(stepper3)
print("Added all the steppers")

try:
    # Move to target positions (up)
    steppers.move_to(pos_up)
    print("Moving to target positions (up)...")
    while steppers.run():
        time.sleep(0.01)  # Wait until the movement completes
    print("Reached target positions (up)")

    # Pause before moving back down
    time.sleep(1)

    # Move back to original positions (down)
    steppers.move_to(pos_down)
    print("Moving back to original positions (down)...")
    while steppers.run():
        time.sleep(0.01)  # Wait until the movement completes
    print("Reached original positions (down)")

except KeyboardInterrupt:
    print("Keyboard interrupt")

finally:
    print("Cleaning up GPIO")
    GPIO.cleanup()
