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
NOTES:
    - Kinematics A,B,C == 0,1,2
'''
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
ENA = 17


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

detected = True
# ----------------------------------------------------------------------------------
def setup():
    steppers.add_stepper(stepperA)
    steppers.add_stepper(stepperB)
    steppers.add_stepper(stepperC)

    # setup enable pin and allow time to manually reset
    GPIO.setup(ENA, GPIO.OUT)
    GPIO.output(ENA, GPIO.HIGH)
    time.sleep(1)

    # turn on motors
    GPIO.output(ENA, GPIO.LOW)
    moveTo(4.25, 0, 0)

    steppers.run_speed_to_position()

def moveTo(hz, nx, ny):
    global detected
    # print("Moving to: " + str(hz) + " " + str(nx) + " " + str(ny))

    if(detected):
        # compute_angle() returns the angle of each leg in degrees
        for i in range(3): 
            pos[i] = round((angOrig - kinematics.compute_angle(i, hz, nx, ny)) * angToStep)
        
        print(f"Pos: {pos}")

        # Set Speed and Acceleration
        a = speed[Kinematics.A] + 5
        b = speed[Kinematics.B] + 5
        c = speed[Kinematics.C] + 5

        d = (speed[Kinematics.A] * 30) + 5
        e = (speed[Kinematics.B] * 30) + 5
        f = (speed[Kinematics.C] * 30) + 5

        g = pos[kinematics.A]
        h = pos[kinematics.B]
        i = pos[kinematics.C]

        stepperA.set_max_speed(a)
        stepperB.set_max_speed(b)
        stepperC.set_max_speed(c)

        stepperA.set_acceleration(d)
        stepperB.set_acceleration(e)
        stepperC.set_acceleration(f)

        # Move to Position
        stepperA.move_to(g)
        stepperB.move_to(h)
        stepperC.move_to(i)

        print(f"""
        Stepper A:
            Max Speed: {a}
            Acceleration: {d}
            Move To: {g}
        Stepper B:
            Max Speed: {b}
            Acceleration: {e}
            Move To: {h}
        Stepper C:
            Max Speed: {c}
            Acceleration: {f}
            Move To: {i}
        """)

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

def PID(setpointX, setpointY):
    global detected
    print("===================================")
    print("starting PID")
    point = read_touch_coordinates()
    translated_point = transform_coordinates(point.x, point.y)
    print("read touch coordinates: " + str(translated_point.x) + " " + str(translated_point.y))
    if(translated_point.x != 0 and translated_point.y != 0):
        detected = True

        for i in range(2):
            errorPrev[i] = error[i]

            error[i] = (i == 0) * (xoffset - translated_point.x - setpointX) + (i == 1) * (yoffset - translated_point.y - setpointY)
            # print(f"Error: {error[i]}")
            integr[i] += error[i] + errorPrev[i]

            deriv[i] = error[i] - errorPrev[i]

            deriv[i] = 0 if (math.isnan(deriv[i]) or math.isinf(deriv[i])) else deriv[i]

            out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]

            out[i] = constrain(out[i], -0.25, 0.25)

        for i in range(3):
            # print(f"speed[{i}] {speed[i]}")
            speedPrev[i] = speed[i]

            speed[i] = (i == Kinematics.A) * stepperA.current_position() + (i == Kinematics.B) * stepperB.current_position() + (i == Kinematics.C) * stepperC.current_position()
        
            speed[i] = abs(speed[i] - pos[i]) * ks

            speed[i] = constrain(speed[i], speedPrev[i] - 200, speedPrev[i] + 200)

            speed[i] = constrain(speed[i], 0, 1000)

        print("X OUT: " + str(out[0]) + " Y OUT: " + str(out[1]) + " Speed A: " + str(speed[Kinematics.A]))
    else:
        #delay by 10 ms to double check that there is no ball
        time.sleep(0.1)
        point = read_touch_coordinates()
        translated_point = transform_coordinates(point.x, point.y)
        if(translated_point.x == 0 and translated_point.y == 0):
            detected = False
            print("No ball detected")

    # continues moving platform and waits until 20 milliseconds have elapsed
    timeI = millis() # Convert to milliseconds
    while (millis() - timeI < 20):
        moveTo(4.25, -out[0], -out[1])  # moves the platform


if __name__ == "__main__":
    try:
        setup()
        print("Setup Complete")
        time.sleep(2)
    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up")
        