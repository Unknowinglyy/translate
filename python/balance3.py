import time
import math
import RPi.GPIO as GPIO
from accelstepper import AccelStepper
from multistepper import MultiStepper
from touchScreenBasicCoordOutput import Point
from kine2 import Kinematics
import serial

ser = serial.Serial('/dev/ttyACM0', 9600)

def read_coords():
    print(f"serial in_waiting: {ser.in_waiting}")
    if ser.in_waiting > 0:
        raw_line = ser.readline()
        try:
            # Attempt to decode the line
            line = raw_line.decode('utf-8', errors='ignore').rstrip()
            print("===================================")
            print(line)
            x, y, z = map(int, line.split(','))
            point = Point(x, y, z)
            ser.reset_input_buffer()
            return point
            
        except UnicodeDecodeError:
            # Log invalid data for debugging
            print(f"Invalid data received: {raw_line}")
            ser.reset_input_buffer()
    return Point(0,-1,-1)
    
    

def millis():
    return int(time.time() * 1000)

def constrain(value, minn, maxn):
  return max(min(maxn, value), minn)

kinematics = Kinematics(2, 3.125, 1.75, 3.669291339)

print("Done with kinematics")

stepper1 = AccelStepper(AccelStepper.DRIVER, 23, 24)
stepper2 = AccelStepper(AccelStepper.DRIVER, 20, 21)
stepper3 = AccelStepper(AccelStepper.DRIVER, 5, 6)

steppers = MultiStepper()
print("Done with steppers")

#stores the target positions for each stepper motor
pos = [0, 0, 0]

#enable pin for drivers
ENA = 17

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

print("Done with variables")

def setup():
    steppers.add_stepper(stepper1)
    steppers.add_stepper(stepper2)
    steppers.add_stepper(stepper3)

    
    GPIO.setup(ENA, GPIO.OUT)
    #turn motors off
    GPIO.output(ENA, GPIO.HIGH)

    #sleeping for a second to allow the system to settle
    time.sleep(1)

    #turn them on
    GPIO.output(ENA, GPIO.LOW)

    moveTo(4.25,0,0)

    steppers.run_speed_to_position()   

def moveTo(hz, nx, ny):
    # print("Moving to: " + str(hz) + " " + str(nx) + " " + str(ny))
    if(detected):
        for i in range(3):
            pos[i] = round((angOrig - kinematics.compute_angle(i, hz, nx, ny)) * angToStep)
        
        stepper1.set_max_speed(speed[Kinematics.A])
        stepper2.set_max_speed(speed[Kinematics.B])
        stepper3.set_max_speed(speed[Kinematics.C])

        stepper1.set_acceleration(speed[Kinematics.A] * 30)
        stepper2.set_acceleration(speed[Kinematics.B] * 30)
        stepper3.set_acceleration(speed[Kinematics.C] * 30)

        stepper1.move_to(pos[Kinematics.A])
        stepper2.move_to(pos[Kinematics.B])
        stepper3.move_to(pos[Kinematics.C])

        stepper1.run()
        stepper2.run()
        stepper3.run()
    else:
        for i in range(3):
            pos[i] = round((angOrig - kinematics.compute_angle(i, hz, 0,0)) * angToStep)

        stepper1.set_max_speed(800)
        stepper2.set_max_speed(800)
        stepper3.set_max_speed(800)

        steppers.move_to(pos)
        steppers.run()

def PID(setpointX, setpointY):
    print("starting PID")
    point = read_coords()
    print("read touch coordinates: " + str(point.x) + " " + str(point.y))
    if(point.x != 0):
        detected = True

        for i in range(2):
            errorPrev[i] = error[i]

            error[i] = (i == 0) * (xoffset - point.x - setpointX) + (i == 1) * (yoffset - point.y - setpointY)

            integr[i] += error[i] + errorPrev[i]

            deriv[i] = error[i] - errorPrev[i]

            deriv[i] = 0 if math.isnan(deriv[i]) or math.isinf(deriv[i]) else deriv[i]

            out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]

            out[i] = constrain(out[i], -0.25, 0.25)

        for i in range(3):
            speedPrev[i] = speed[i]

            speed[i] = (i == Kinematics.A) * stepper1.current_position() + (i == Kinematics.B) * stepper2.current_position() + (i == Kinematics.C) * stepper3.current_position()
        
            speed[i] = abs(speed[i] - pos[i]) * ks

            speed[i] = constrain(speed[i], speedPrev[i] - 200, speedPrev[i] + 200)

            speed[i] = constrain(speed[i], 0, 1000)

        print("X OUT: " + str(out[0]) + " Y OUT: " + str(out[1]) + " Speed A: " + str(speed[Kinematics.A]))
    else:
        #delay by 10 ms to double check that there is no ball
        time.sleep(0.01)
        point = read_coords()
        if(point.x == 0):
            detected = False
            print("No ball detected")

    # continues moving platform and waits until 20 milliseconds have elapsed
    timeI = millis() # Convert to milliseconds
    while (millis() - timeI) < 20:
        moveTo(4.25, -out[0], -out[1])  # moves the platform

if __name__ == "__main__":
    try:
        setup()
        while True:
            PID(0,0)
            print(f"Detected: {detected}")
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        print("cleaning up GPIO")
        GPIO.cleanup()
        ser.close()