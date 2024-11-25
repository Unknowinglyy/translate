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
stepperA = AccelStepper(AccelStepper.DRIVER, 20, 21) # Stepper A
stepperB = AccelStepper(AccelStepper.DRIVER, 23, 24) # Stepper B
stepperC = AccelStepper(AccelStepper.DRIVER, 5, 6)   # Stepper C
steppers = MultiStepper()


#stepper motor variables
pos = [400, 400, 400]   #An array to store the target positions for each stepper motor
ENA = 17                    #enable pin for the drivers


def setup() {
  #Set iniial maximum speed value for the steppers (steps/sec)
  stepperA.set_max_speed(200)
  stepperB.set_max_speed(200)
  stepperC.set_max_speed(200)


  # Adding the steppers to the steppersControl instance for multi stepper control
  steppers.add_stepper(stepperA);
  steppers.add_stepper(stepperB);
  steppers.add_stepper(stepperC);


  #Enable pin
  GPIO.setup(ENA, GPIO.OUT)  #define enable pin as output
  
  GPIO.output(ENA, LOW)      #set enable pin low to enable the drivers

  print("3 Seconds to setup motors")
  time.sleep(3)             #small delay to allow the user to reset the platform


  #Movemement
  steppers.move_to(pos);  # Calculates the required speed for all motors
  steppers.run_speed_to_position();  # blocks until all steppers reach their target position
}
