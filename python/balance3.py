import time
import math
import RPi.GPIO as GPIO
from accelstepper import AccelStepper, constrain
from multistepper import MultiStepper
from touchScreenBasicCoordOutput import read_touch_coordinates, Point
from touchScreenTranslatedCoordOutput import *
from kine3 import Machine

A = Machine.A
B = Machine.B
C = Machine.C

start_time = time.perf_counter()

# Helper Functions
def debug_log(msg):
    print(f"[{time.time():.2f}] {msg}")

def delay(milliseconds):
    """
    Mimics the Arduino delay() function with reduced CPU usage.
    """
    start_time = time.perf_counter()
    end_time = start_time + (milliseconds / 1000.0)
    
    while time.perf_counter() < end_time:
        # Sleep briefly to reduce CPU load, but not too long
        time.sleep(0.001)  # Sleep for 1 ms

def millis():
    return int((time.perf_counter() - start_time) * 1000)

machine = Machine(2, 3.125, 1.75, 3.669291339)

print("Done with kinematics")


#motor A = motor 2 (pins 13 & 19)
#motor B = motor 3 (pins 5 & 6)
#motor c = motor 1 (pins 23 & 24)

stepperA = AccelStepper(1, 13, 19)
stepperB = AccelStepper(1, 5, 6)
stepperC = AccelStepper(1, 23, 24)

steppers = MultiStepper()
print("Done with steppers")

#stores the target positions for each stepper motor
pos = [0, 0, 0]

#enable pin for drivers
ENA = 17

#original angle that each leg starts at
angOrig = 206.662752199

#speed of the stepper motor and the speed amplifying constant
speed = [1.0, 1.0, 1.0]
speedPrev = [0.0, 0.0, 0.0]
ks = float(20)

#touch screen variables

#x offset for the center position of the touchscreen
xoffset = float(500)
#y offset for the center position of the touchscreen
yoffset = float(500)

#PID variables

#proportional gain
kp = 4E-4
#integral gain
ki = 2E-6
#derivative gain
kd = 7E-3

#PID constants
#PID terms for X and Y directions
error = [0.0, 0.0]
errorPrev = [0.0, 0.0]
integr = [0.0, 0.0]
deriv = [0.0, 0.0]
out = [0.0, 0.0]

#variable to capture inital times
timeI = 0.0

#other variables
angToStep = 1100 / 360

detected = False

print("Done with variables")

def setup():
    steppers.add_stepper(stepperA)
    steppers.add_stepper(stepperB)
    steppers.add_stepper(stepperC)
    
    GPIO.setup(ENA, GPIO.OUT)
    #turn motors off
    GPIO.output(ENA, GPIO.HIGH)

    #sleeping for a second to allow the system to settle
    delay(1000)

    #turn them on
    GPIO.output(ENA, GPIO.LOW)

    moveTo(4.25, 0, 0)

    steppers.run_speed_to_position()

def loop():
    PID(0,0)   

def moveTo(hz, nx, ny):
    global detected, pos, speed, machine, angOrig, stepperA, stepperB, stepperC, steppers, angToStep
    debug_log(f"move_to called with hz={hz}, nx={nx}, ny={ny}")
    if(detected):
        for i in range(3):
            pos[i] = round((angOrig - machine.theta(i, hz, nx, ny)) * angToStep)
            print(f"Motor {i} target position: {pos[i]}")
        stepperA.set_max_speed(speed[A])
        stepperB.set_max_speed(speed[B])
        stepperC.set_max_speed(speed[C])

        stepperA.set_acceleration(speed[A] * 30)
        stepperB.set_acceleration(speed[B] * 30)
        stepperC.set_acceleration(speed[C] * 30)

        stepperA.move_to(pos[A])
        stepperB.move_to(pos[B])
        stepperC.move_to(pos[C])

        # print(f"Stepper1 max_speed {speed[Kinematics.A]}, acceleration {speed[kinematics.A]}, move_to {pos[Kinematics.A]}")

        stepperA.run()
        stepperB.run()
        stepperC.run()
    else:
        print("Ball not detected, moving to original position")
        for i in range(3):
            pos[i] = round((angOrig - machine.theta(i, hz, 0,0)) * angToStep)
            print(f"Motor {i} target position: {pos[i]}")

        stepperA.set_max_speed(800)
        stepperB.set_max_speed(800)
        stepperC.set_max_speed(800)

        steppers.move_to(pos)
        steppers.run()

def PID(setpointX, setpointY):
    global detected, error, errorPrev, integr, deriv, out, speed, speedPrev, timeI, pos, kp, kd, ki, ks, xoffset, yoffset, machine, stepperA, stepperB, stepperC
    print("===================================")
    print("starting PID")
    orig_point = read_coordinates()
    if orig_point is not None:
        point = transform_coordinates(orig_point.x, orig_point.y)
        debug_log(f"Point: ({point.x}, {point.y})")
        if(point.x != 0 and point.y != 0):
            detected = True

            for i in range(2):
                errorPrev[i] = error[i]

                error[i] = (i == 0) * (xoffset - point.x - setpointX) + (i == 1) * (yoffset - point.y - setpointY)
                # print(f"Error: {error[i]}")
                integr[i] += error[i] + errorPrev[i]

                deriv[i] = error[i] - errorPrev[i]

                deriv[i] = 0 if (math.isnan(deriv[i]) or math.isinf(deriv[i])) else deriv[i]

                out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]

                out[i] = constrain(out[i], -0.25, 0.25)

                debug_log(f"PID output {['X', 'Y'][i]}: error={error[i]}, integr={integr[i]}, deriv={deriv[i]}, out={out[i]}")

            for i in range(3):
                # print(f"speed[{i}] {speed[i]}")
                speedPrev[i] = speed[i]

                speed[i] = (i == A) * stepperA.current_position() + (i == B) * stepperB.current_position() + (i == C) * stepperC.current_position()
            
                speed[i] = abs(speed[i] - pos[i]) * ks

                speed[i] = constrain(speed[i], speedPrev[i] - 200, speedPrev[i] + 200)

                speed[i] = constrain(speed[i], 0, 1000)
        else:
            detected = False
            debug_log("Ball not detected on first check.")
            #delay for 10 ms
            delay(10)
            #check for ball again
            orig_point = read_coordinates()
            if orig_point is None or (orig_point.x == 0 and orig_point.y == 0):
                detected = False
                debug_log("Ball not detected on second check.")
    else:
        detected = False
        debug_log("Touchscreen data is None.")
    
    moveTo(4.25, -out[0], -out[1])

if __name__ == "__main__":
    try:
        setup()
        while True:
            loop()

    except KeyboardInterrupt:
        print("Keyboard interrupt")

    finally:
        print("cleaning up GPIO")
        GPIO.cleanup()