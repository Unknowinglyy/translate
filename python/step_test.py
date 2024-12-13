from accelstepper import AccelStepper
from multistepper import MultiStepper
import RPi.GPIO as GPIO
import time 

GPIO.setmode(GPIO.BCM)
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

stepper1.set_speed(10)
stepper2.set_speed(10000)
stepper3.set_speed(10000)

stepper1.set_max_speed(100)
stepper2.set_max_speed(10000)
stepper3.set_max_speed(10000)
print("set all the speeds")

steppers.add_stepper(stepper1)
steppers.add_stepper(stepper2)
steppers.add_stepper(stepper3)
print("added all the steppers")

try:
    while True:
        stepper1.step1(1)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Keyboard interrupt")

finally:
    print("cleaning up GPIO")
    GPIO.cleanup()