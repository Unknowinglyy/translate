import math

def constrain(value, minn, maxn):
    return max(min(maxn, value), minn)

class AccelStepper:
    DIRECTION_CCW = 0
    DIRECTION_CW = 1

    def __init__(self, interface, pin1, pin2, pin3=None, pin4=None, enable_pin=False, enable=False):
        self._interface = interface
        self._currentPos = 0
        self._targetPos = 0
        self._speed = 0
        self._maxSpeed = 1.0
        self._acceleration = 0
        self._sqrt_twoa = 1.0
        self._stepInterval = 0
        self._minPulseWidth = 0
        self._enablePin = 0xff
        self._lastStepTime = 0
        self._pin = [pin1, pin2, pin3, pin4]
        self._enableInverted = False

        self._n = 0
        self._c0 = 0.0
        self._cn = 0.0
        self._cmin = 1.0
        self._direction = self.DIRECTION_CW
        self._pinInverted = [0] * 4
        if enable:
            self.enable_outputs()
        self.set_acceleration(1)
        self.sex_max_speed(1)
    
    def enable_outputs(self):
        #TODO implement this
        pass

    def set_acceleration(self, acceleration):
        self._acceleration = acceleration
 
    def set_max_speed(self, speed):
        if(speed < 0.0):
            speed = -speed
        if self._maxSpeed != speed:
            self._maxSpeed = speed
            self._cmin = 1000000.0 / speed
            if self._n > 0:
                self._n = (self._speed * self._speed) / (2.0 * self._acceleration)
                self.compute_new_speed()

    def maxSpeed(self):
        return self._maxSpeed

    def set_acceleration(self, acceleration):
        if(acceleration == 0.0):
            return
        if(acceleration < 0.0):
            acceleration = -acceleration
        if(self._acceleration != acceleration):
            self._n = self._n * (self._acceleration / acceleration)
            self._c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0
            self._acceleration = acceleration
            self.compute_new_speed()

    def acceleration(self):
        return self._acceleration
    
    def set_speed(self, speed):
        if(self._speed == speed):
            return
        speed = constrain(speed,  -self._maxSpeed, self._maxSpeed)
        if(speed == 0.0):
            _stepInterval = 0
        else:
            _stepInterval = abs(1000000.0 / speed)
            _direction = self.DIRECTION_CW if speed > 0.0 else self.DIRECTION_CCW
            _speed = speed