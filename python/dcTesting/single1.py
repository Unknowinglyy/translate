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

ENA = 17                    #enable pin for the drivers
# ----------------------------------------------------------------------------------

def setup():
  #Set iniial maximum speed value for the steppers (steps/sec)
  stepperB.set_max_speed(800)

  #Enable pin
  GPIO.setup(ENA, GPIO.OUT)  #define enable pin as output

  print("3 Seconds to setup motors")
  time.sleep(3)             #small delay to allow the user to reset the platform
  
  GPIO.output(ENA, GPIO.LOW)      #set enable pin low to enable the drivers


  #Movemement
  print("Moving Stepper B to 400")
  stepperB.move_to(400);  # Calculates the required speed for all motors
  stepperB.run_speed_to_position();  # blocks until all steppers reach their target position


  


if __name__ == "__main__":
  try:
    setup()
  except KeyboardInterrupt:
    print("Program stopped by user")
  finally:
    GPIO.cleanup()
    print("GPIO cleaned up")