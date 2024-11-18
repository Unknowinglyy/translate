import time
import evdev
import logging
from accelstepper import AccelStepper
from kinematics import Kinematics
from spi import SPI
from stepdir import StepperDriver
from touchscreenbasiccoordoutput import read_touch_coordinates, Point

# Initialize logging
logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger(__name__)

# Constants
DIRECTION_CCW = 0   # Clockwise
DIRECTION_CW  = 1   # Counter-Clockwise

# Helper functions
def sleep_microseconds(us_to_sleep):
    time.sleep(us_to_sleep / float(1000000))

def constrain(value, minn, maxn):
    return max(min(maxn, value), minn)

def micros():
    """
    Mimics the Arduino micros() function.
    """
    return int(time.time() * 1000000)

# Main class for ball balancing
class BallBalancing:
    def __init__(self):
        self.spi = SPI()
        self.kinematics = Kinematics(d=10, e=20, f=30, g=40)
        self.stepper_driver = StepperDriver(motor_steps=200, dir_pin=17, step_pin=27, enable_pin=22)
        self.accel_stepper = AccelStepper(profile=None, dir_pin=17, step_pin=27, enable_pin=22)
        self.touchscreen_device_path = '/dev/input/event6'

    def read_touch_coordinates(self):
        return read_touch_coordinates(self.touchscreen_device_path)

    def balance_ball(self):
        while True:
            point = self.read_touch_coordinates()
            log.debug(f"Touch coordinates: X: {point.x}, Y: {point.y}")

            # Compute angles for the legs
            angle_a = self.kinematics.compute_angle('A', hz=0, nx=point.x, ny=point.y)
            angle_b = self.kinematics.compute_angle('B', hz=0, nx=point.x, ny=point.y)
            angle_c = self.kinematics.compute_angle('C', hz=0, nx=point.x, ny=point.y)

            log.debug(f"Angles: A: {angle_a}, B: {angle_b}, C: {angle_c}")

            # Move the stepper motors based on the computed angles
            self.stepper_driver.move_to(angle_a)
            self.stepper_driver.move_to(angle_b)
            self.stepper_driver.move_to(angle_c)

            sleep_microseconds(10000)  # Sleep for 10 milliseconds

if __name__ == "__main__":
    ball_balancing = BallBalancing()
    ball_balancing.balance_ball()