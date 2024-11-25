import time
import math
import RPi.GPIO as GPIO
from accelstepper import AccelStepper
from multistepper import MultiStepper
from touchScreenBasicCoordOutput import read_touch_coordinates, Point
from touchScreenTranslatedCoordOutput import transform_coordinates
from kine2 import Kinematics
# ----------------------------------------------------------------------------------
# TIME FUNCTIONS:
start_time = time.perf_counter()

def millis():
    return int((time.perf_counter() - start_time) * 1000)

def constrain(value, minn, maxn):
  return max(min(maxn, value), minn)

kinematics = Kinematics(2, 3.125, 1.75, 3.669291339)
# ----------------------------------------------------------------------------------
# SETUP MOTORS:
stepperA = AccelStepper(AccelStepper.DRIVER, 20, 21) # Stepper located btwn two corners
stepperB = AccelStepper(AccelStepper.DRIVER, 23, 24)
stepperC = AccelStepper(AccelStepper.DRIVER, 5, 6)
steppers = MultiStepper()


pos = [0, 0, 0]
ENA = 1


angOrig   = 206.662752199 #original angle that each leg starts at
speed     = [0, 0, 0]
speedPrev = [0, 0, 0]
ks        = 20 # Amplifying constant


xoffset = 500
yoffset = 500


kp = 4E-4
ki = 2E-6
kd = 7E-3


error     = [0, 0]
errorPrev = [0, 0]
integr    = [0, 0]
deriv     = [0, 0]

out = [0, 0]

#variable to capture inital times
timeI = 0

#other variables
angToStep = 3200 / 360 # ~ 8.8889 (8.9 steps per degree)

detected = False
# ----------------------------------------------------------------------------------
def moveTo(hz, nx, ny):
    global detected
    # print("Moving to: " + str(hz) + " " + str(nx) + " " + str(ny))

    if(detected):
        for i in range(3):
            pos[i] = round((angOrig - kinematics.compute_angle(i, hz, nx, ny)) * angToStep)
        
        stepperA.set_max_speed(speed[Kinematics.A])
        stepperB.set_max_speed(speed[Kinematics.B])
        stepperC.set_max_speed(speed[Kinematics.C])

        stepperA.set_acceleration(speed[Kinematics.A] * 30)
        stepperB.set_acceleration(speed[Kinematics.B] * 30)
        stepperC.set_acceleration(speed[Kinematics.C] * 30)

        stepperA.move_to(pos[Kinematics.A])
        stepperB.move_to(pos[Kinematics.B])
        stepperC.move_to(pos[Kinematics.C])

        # print(f"stepperA max_speed {speed[Kinematics.A]}, acceleration {speed[kinematics.A]}, move_to {pos[Kinematics.A]}")stepperB
        stepperA.run()
        stepperB.run()
        stepperC.run()
    else:
        for i in range(3):
            pos[i] = round((angOrig - kinematics.compute_angle(i, hz, 0,0)) * angToStep)

        stepperA.set_max_speed(800)
        stepperB.set_max_speed(800)
        stepperC.set_max_speed(800)

        steppers.move_to(pos)
        steppers.run()






        