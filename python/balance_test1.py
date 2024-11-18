import time
from RpiMotorLib import RpiMotorLib
from touchScreenBasicCoordOutput import read_touch_coordinates, Point
from kine import Machine

# Initialize the machine with the given lengths
machine = Machine(2, 3.125, 1.75, 3.669291339)

# Initialize the stepper motors
stepperA = RpiMotorLib.A4988Nema(23, 24, (-1,-1,-1), "A4988")
stepperB = RpiMotorLib.A4988Nema(20, 21, (-1,-1,-1), "A4988")
stepperC = RpiMotorLib.A4988Nema(5, 6, (-1,-1,-1), "A4988")

# Stepper motor variables
pos = [0, 0, 0]  # Target positions for each stepper motor
angOrig = 206.662752199  # Original angle that each leg starts at
angToStep = 1  # Conversion factor from angle to step (adjust as needed)
ks = 20  # Speed amplifying constant

# PID variables
error = [0, 0]
errorPrev = [0, 0]
integr = [0, 0]
deriv = [0, 0]
detected = 0

# Touchscreen variables
Xoffset = 500  # X offset for the center position of the touchpad
Yoffset = 500  # Y offset for the center position of the touchpad

# Function to move the stepper motors to the target positions
def move_steppers(target_positions):
    stepperA.motor_go(False, "Full", target_positions[0], 0.01, False, 0.05)
    stepperB.motor_go(False, "Full", target_positions[1], 0.01, False, 0.05)
    stepperC.motor_go(False, "Full", target_positions[2], 0.01, False, 0.05)

# PID function to move the ball to the setpoint position
def PID(setpointX, setpointY):
    global detected, error, errorPrev, integr, deriv

    while True:
        point = read_touch_coordinates()
        if point is None:
            break

        # Measure X and Y positions
        p_x = point.x
        p_y = point.y

        # If the ball is detected (the x position will not be 0)
        if p_x != 0:
            detected = 1
            # Calculate PID values
            for i in range(2):
                errorPrev[i] = error[i]  # Set previous error
                error[i] = (i == 0) * (Xoffset - p_x - setpointX) + (i == 1) * (Yoffset - p_y - setpointY)  # Set error
                integr[i] += error[i] + errorPrev[i]  # Calculate the integral of the error
                deriv[i] = error[i] - errorPrev[i]  # Calculate the derivative of the error

            # Calculate target positions for the stepper motors
            for i, leg in enumerate(['A', 'B', 'C']):
                pos[i] = round((angOrig - machine.theta(leg, 0, 0, 0)) * angToStep)

            # Set max speed
            stepperA.motor_go(False, "Full", pos[0], 0.01, False, 0.05)
            stepperB.motor_go(False, "Full", pos[1], 0.01, False, 0.05)
            stepperC.motor_go(False, "Full", pos[2], 0.01, False, 0.05)

            # Move the stepper motors to the target positions
            move_steppers(pos)

            # Add a small delay to avoid overwhelming the motors
            time.sleep(0.1)

# Example usage
if __name__ == "__main__":
    setpointX = 100  # Example setpoint X
    setpointY = 100  # Example setpoint Y
    PID(setpointX, setpointY)