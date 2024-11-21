from accelstepper import AccelStepper
from multistepper import MultiStepper
import RPi.GPIO as GPIO
import time 

GPIO.setmode(GPIO.BCM)
# GPIO.setup(23, GPIO.OUT)
# GPIO.setup(24, GPIO.OUT)
# GPIO.setup(20, GPIO.OUT)
# GPIO.setup(21, GPIO.OUT)
# GPIO.setup(5, GPIO.OUT)
# GPIO.setup(6, GPIO.OUT)

#parameters = driver type, step pin, direction pin
stepper1 = AccelStepper(AccelStepper.DRIVER, 23, 24)
stepper2 = AccelStepper(AccelStepper.DRIVER, 20, 21)
stepper3 = AccelStepper(AccelStepper.DRIVER, 5, 6)

steppers = MultiStepper()

#target positions for each motor
pos = [400, 400, 400]
#enable pin for drivers
ENA = 17

GPIO.setup(ENA, GPIO.OUT)
GPIO.output(ENA, GPIO.LOW)

#small delay to allow system to settle
time.sleep(1)

stepper1.set_max_speed(200)
stepper2.set_max_speed(200)
stepper3.set_max_speed(200)

steppers.add_stepper(stepper1)
steppers.add_stepper(stepper2)
steppers.add_stepper(stepper3)

#move to target positions
steppers.move_to(pos)

steppers.run_speed_to_position()

GPIO.cleanup()