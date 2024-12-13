from accelstepper import AccelStepper
from multistepper import MultiStepper
import RPi.GPIO as GPIO
import time 

GPIO.setmode(GPIO.BCM)
#parameters = driver type, step pin, direction pin
stepper1 = AccelStepper(AccelStepper.DRIVER, 23, 24)
stepper2 = AccelStepper(AccelStepper.DRIVER, 13, 19)
stepper3 = AccelStepper(AccelStepper.DRIVER, 5, 6)

steppers = MultiStepper()

#target positions for each motor
pos = [420, 420, 420]
#enable pin for drivers
ENA = 17

GPIO.setup(ENA, GPIO.OUT)
GPIO.output(ENA, GPIO.LOW)

#small delay to allow system to settle
time.sleep(1)

stepper1.set_max_speed(1000)
stepper2.set_max_speed(1000)
stepper3.set_max_speed(1000)
print("set all the speeds")

steppers.add_stepper(stepper1)
steppers.add_stepper(stepper2)
steppers.add_stepper(stepper3)
print("added all the steppers")

try:
    #move to target positions
    steppers.move_to(pos)
    print("moved to target positions")

    #block until all steppers reach their target positions
    steppers.run_speed_to_position()
    print("ran to target positions")

except KeyboardInterrupt:
    print("Keyboard interrupt")

finally:
    print("cleaning up GPIO")
    GPIO.cleanup()