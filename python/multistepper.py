# multi_stepper.py

from accelstepper import AccelStepper


MULTISTEPPER_MAX_STEPPERS = 10

class MultiStepper:
    def __init__(self):
        self._steppers = []
        self._num_steppers = 0

    def add_stepper(self, stepper: AccelStepper) -> bool:
        if self._num_steppers >= MULTISTEPPER_MAX_STEPPERS:
            return False  # No room for more
        self._steppers.append(stepper)
        self._num_steppers += 1
        return True

    def move_to(self, absolute: list):
        # First find the stepper that will take the longest time to move
        longest_time = 0.0

        for i in range(self._num_steppers):
            this_distance = absolute[i] - self._steppers[i].current_position()
            this_time = abs(this_distance) / self._steppers[i].max_speed()
            print("Distance to go: " + str(this_distance) + " Time to go: " + str(this_time))
            if this_time > longest_time:
                longest_time = this_time
        
        if(longest_time > 0.0):
            # Now set the speed of each stepper so they will all arrive at the same time
            for i in range(self._num_steppers):
                this_distance = absolute[i] - self._steppers[i].current_position()
                this_speed = this_distance / longest_time
                self._steppers[i].move_to(absolute[i])
                self._steppers[i].set_speed(this_speed)

    def run(self) -> bool:
        running = False
        for i in range(self._num_steppers):
            if self._steppers[i].distance_to_go() != 0:
                self._steppers[i].run_speed()
                running = True
            #might not be needed
            # else:
            #     self._steppers[i].set_current_position(self._steppers[i].current_position())
        return running

    def run_speed_to_position(self):
        while self.run():
            #do nothing
            pass