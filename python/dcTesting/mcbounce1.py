import time
import math
import RPi.GPIO as GPIO


from accelstepper import AccelStepper
from multistepper import MultiStepper
from touchScreenBasicCoordOutput import read_touch_coordinates, Point
from touchScreenTranslatedCoordOutput import transform_coordinates
from kine2 import Kinematics
# ----------------------------------------------------------------------------------
'''
NOTE:
    - Testing negative values for move_to
'''
# ----------------------------------------------------------------------------------
#stepper motors

stepperB = AccelStepper(AccelStepper.DRIVER, 23, 24) # Stepper B
steppers = MultiStepper()


ENA = 17                    #enable pin for the drivers
# ----------------------------------------------------------------------------------
def setup():
  #Set iniial maximum speed value for the steppers (steps/sec)
  stepperB.set_max_speed(600)
  steppers.add_stepper(stepperB)

  GPIO.setup(ENA, GPIO.OUT)  
  print("3 Seconds to setup motors")
  time.sleep(3)
  GPIO.output(ENA, GPIO.LOW)      #set enable pin low to enable the drivers
  


def loop():
  print(f"Moving to {400}")
  steppers.move_to([400])

  while True:
    if stepperB.distance_to_go() == 0:
      print(f"Moving to {-stepperB.current_position()}")
      steppers.move_to([-stepperB.current_position()])

    steppers.run_speed_to_position()


  


if __name__ == "__main__":
  try:
    setup()
    loop()
  except KeyboardInterrupt:
    print("Program stopped by user")
  finally:
    GPIO.cleanup()
    print("GPIO cleaned up")