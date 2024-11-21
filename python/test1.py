import time
from accelstepper import AccelStepper
from multistepper import MultiStepper
import RPi.GPIO as GPIO 

# Define GPIO pins for the stepper motor
STEP_PIN = 23
DIR_PIN = 24
ENA = 17


GPIO.setmode(GPIO.BCM)

GPIO.setup(ENA, GPIO.OUT)
GPIO.output(ENA, GPIO.LOW)

GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# ---------------------------------------------------------------------

def main():
    # Initialize the stepper motor
    motor = AccelStepper(AccelStepper.DRIVER, STEP_PIN, DIR_PIN)
    
    # Initialize the MultiStepper and add the motor
    multi_stepper = MultiStepper()
    multi_stepper.add_stepper(motor)

    # Set motor properties
    motor.set_max_speed(20)  # Maximum speed in steps per second
    motor.set_acceleration(500)  # Acceleration in steps per second^2
    motor.set_current_position(0)  # Reset current position to 0

    print(multi_stepper._steppers[0].current_position())
    
    # Move up 20 steps
    print("Moving up 20 steps")
    multi_stepper.move_to([200])  # Target position for the motor
    print(f"Distance_to_go(): {multi_stepper._steppers[0].distance_to_go()}")
    while multi_stepper.run():  # Keep running until all motors reach their positions
        time.sleep(.0001)  # Small delay for smooth operation


    # Return to start position
    print("Returning to initial position")
    multi_stepper.move_to([0])  # Return to initial position
    print(f"Distance_to_go(): {multi_stepper._steppers[0].distance_to_go()}")
    print(f"Current Direction: {multi_stepper._steppers[0].current_direction()}")
    while multi_stepper.run():  # Keep running until all motors reach their positions
        time.sleep(.0001)  # Small delay for smooth operation

    # print("Moving up 20 steps")
    # multi_stepper.move_to([200])  # Target position for the motor
    # while multi_stepper.run():  # Keep running until all motors reach their positions
    #     time.sleep(.001)  # Small delay for smooth operation
    
    time.sleep(5)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        print("cleaning up GPIO")
        GPIO.cleanup()
