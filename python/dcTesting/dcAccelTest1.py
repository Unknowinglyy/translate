import time
import random
from accelstepper import AccelStepper
import RPi.GPIO as GPIO

# Define GPIO pins for the stepper motor
STEP_PIN = 23
DIR_PIN = 24
ENA_PIN = 17

# Initialize the stepper motor
stepper = AccelStepper(AccelStepper.DRIVER, STEP_PIN, DIR_PIN)

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ENA_PIN, GPIO.OUT)
    GPIO.output(ENA_PIN, GPIO.LOW)  # Enable the stepper driver

def loop():
    while True:
        if stepper.distance_to_go() == 0:
            # Random change to speed, position, and acceleration
            # Make sure we don't get 0 speed or acceleration
            time.sleep(1)
            stepper.move_to(random.randint(100, 200))
            stepper.set_max_speed(random.randint(1, 200) * 100)
            stepper.set_acceleration(random.randint(1, 200))
        stepper.run()

if __name__ == "__main__":
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up")