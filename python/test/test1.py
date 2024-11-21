import time
from accelstepper import AccelStepper

# Define GPIO pins for the stepper motor
STEP_PIN = 23
DIR_PIN = 24

def main():
    # Initialize the stepper motor
    motor = AccelStepper(AccelStepper.DRIVER, STEP_PIN, DIR_PIN)
    
    # Set motor properties
    motor.set_max_speed(1000)  # Maximum speed in steps per second
    motor.set_acceleration(500)  # Acceleration in steps per second^2
    motor.set_current_position(0)  # Reset current position to 0
    
    # Move up 20 steps
    print("Moving up 20 steps")
    motor.move_to(20)  # Set target position to 20 steps
    while motor.distance_to_go() != 0:
        motor.run()  # Move step by step
        time.sleep(0.01)  # Small delay for smooth operation
    
    time.sleep(1)  # Pause for 1 second at the top

    # Move back down 20 steps
    print("Moving down 20 steps")
    motor.move_to(0)  # Return to initial position
    while motor.distance_to_go() != 0:
        motor.run()  # Move step by step
        time.sleep(0.01)  # Small delay for smooth operation

    print("Done!")

if __name__ == "__main__":
    main()
