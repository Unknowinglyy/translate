import time
import math
import RPi.GPIO as GPIO


from accelstepper import AccelStepper
from multistepper import MultiStepper
from touchScreenBasicCoordOutput import read_touch_coordinates, Point
from touchScreenTranslatedCoordOutput import transform_coordinates
from kine2 import Kinematics
# ----------------------------------------------------------------------------------
#stepper motors

stepperB = AccelStepper(AccelStepper.DRIVER, 23, 24) # Stepper B

steppers = MultiStepper()


#stepper motor variables
pos = [400, 400, 400]   #An array to store the target positions for each stepper motor
ENA = 17                    #enable pin for the drivers
# ----------------------------------------------------------------------------------
# Setup one motor:
STEP_PIN = 23
DIR_PIN  = 24
ENA_PIN  = 17
stepper = AccelStepper(AccelStepper.DRIVER, STEP_PIN, DIR_PIN)

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ENA_PIN, GPIO.OUT)
    GPIO.output(ENA_PIN, GPIO.LOW)  # Enable the stepper driver

    # Change these to suit your stepper if you want
    stepper.set_max_speed(800)
    stepper.set_acceleration(20)
    stepper.move_to(400)

def loop():
    while True:
        # If at the end of travel go to the other end
        if stepper.distance_to_go() == 0:
            stepper.move_to(-stepper.current_position())
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