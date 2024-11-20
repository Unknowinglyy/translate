import time
import math
from accel.accelstepper import AccelStepper
from multistepper import MultiStepper
from touchScreenBasicCoordOutput import read_touch_coordinates
from kine2 import Kinematics
from accel import __init__

DIRECTION_CCW = 0   # Clockwise
DIRECTION_CW  = 1   # Counter-Clockwise

def constrain(value, minn, maxn):
  return max(min(maxn, value), minn)

kinematics = Kinematics(2, 3.125, 1.75, 3.669291339)

stepper1 = AccelStepper(1, step_pin=23, dir_pin=24)
stepper2 = AccelStepper(1, step_pin=20, dir_pin=21)
stepper3 = AccelStepper(1, step_pin=5, dir_pin=6)

steppers = MultiStepper()

#stores the target positions for each stepper motor
pos = [0, 0, 0]

#enable pin for drivers
ENA = 0

#original angle that each leg starts at
angOrig = 206.662752199

#speed of the stepper motor and the speed amplifying constant
speed = [0, 0, 0]
speedPrev = [0, 0, 0]
ks = 20

#touch screen variables

#x offset for the center position of the touchscreen
xoffset = 500
#y offset for the center position of the touchscreen
yoffset = 500

#PID variables

#proportional gain
kp = 4E-4
#integral gain
ki = 2E-6
#derivative gain
kd = 7E-3

#PID constants
#PID terms for X and Y directions
error = [0, 0]
errorPrev = [0, 0]
integr = [0, 0]
deriv = [0, 0]

out = [0, 0]
#variable to capture inital times
timeI = 0

#other variables
angToStep = 3200 / 360

detected = False

def setup():
    steppers.add_stepper(stepper1)
    steppers.add_stepper(stepper2)
    steppers.add_stepper(stepper3)

    steppers.move_to([4.25,0,0])
    steppers.run_speed_to_position()

def loop():
    PID(0,0)

def moveTo(hz, nx, ny):
    if(detected):
        for i in range(3):
            pos[i] = round((angOrig - kinematics.compute_angle(i, hz, nx, ny)) * angToStep)
        stepper1.set_target_speed(speed[Kinematics.A])
        stepper2.set_target_speed(speed[Kinematics.B])
        stepper3.set_target_speed(speed[Kinematics.C])

        stepper1.set_acceleration(speed[Kinematics.A] * 30)
        stepper2.set_acceleration(speed[Kinematics.B] * 30)
        stepper3.set_acceleration(speed[Kinematics.C] * 30)

        steppers.move_to(pos[Kinematics.A])
        steppers.move_to(pos[Kinematics.B])
        steppers.move_to(pos[Kinematics.C])

        stepper1.run()
        stepper2.run()
        stepper3.run()
    else:
        for i in range(3):
            pos[i] = round((angOrig - kinematics.compute_angle(i, hz, 0,0)) * angToStep)

        stepper1.set_target_speed(800)
        stepper2.set_target_speed(800)
        stepper3.set_target_speed(800)

        steppers.move_to(pos)
        steppers.run()

def PID(setpointX, setpointY):
    point = read_touch_coordinates()

    if(point.x != 0):
        detected = True

        for i in range(2):
            errorPrev[i] = error[i]

            error[i] = (i == 0) * (xoffset - p.x - setpointX) + (i == 1) * (yoffset - p.y - setpointY)
            integr[i] += error[i] + errorPrev[i]
            deriv[i] = error[i] - errorPrev[i]
            deriv[i] = 0 if math.isnan(deriv[i]) or math.isinf(deriv[i]) else deriv[i]
            out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
            out[i] = constrain(out[i], -0.25, 0.25)

        for i in range(3):
            speedPrev[i] = speed[i]

            speed[i] = (stepper1.position() if i == Kinematics.A else 0) + (stepper2.position() if i == Kinematics.B else 0) + (stepper3.position() if i == Kinematics.C else 0)
        
            speed[i] = abs(speed[i] - pos[i]) * ks

            speed[i] = constrain(speed[i], speedPrev[i] - 200, speedPrev[i] + 200)

            speed[i] = constrain(speed[i], 0, 1000)
        print("X OUT: " + str(out[0]) + " Y OUT: " + str(out[1]) + " Speed A: " + speed[Kinematics.A])
    else:
        #delay by 10 ms to double check that there is no ball
        time.sleep(0.01)
        point = read_touch_coordinates()
        if(point.x == 0):
            detected = False
            print("No ball detected")

    # continues moving platform and waits until 20 milliseconds have elapsed
    timeI = time.time() * 1000  # Convert to milliseconds
    while (time.time() * 1000 - timeI) < 20:
        moveTo(4.25, -out[0], -out[1])  # moves the platform

if __name__ == "__main__":
    setup()
    while True:
        loop()
        time.sleep(0.01)    