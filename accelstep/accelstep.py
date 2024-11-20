import math
import time

def constrain(value, minn, maxn):
    return max(min(maxn, value), minn)

def micros():
  """
  Mymics the Arduino micros() function.
  """
  return int(time.time() * 1000000)

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

    def move_to(self, absolute: list):
        if(self._targetPos != absolute):
            self._targetPos = absolute
            self.compute_new_speed()
    
    def move(self, relative):
        self.move_to(self._currentPos + relative)

    def run_speed(self):
        if(not self._stepInterval):
            return False
        time = micros()
        if(time - self._lastStepTime >= self._stepInterval):
            if(self._direction == self.DIRECTION_CW):
                self._currentPos += 1
            else:
                self._currentPos -= 1
            self.step(self._currentPos)
            self._lastStepTime = time
            return True
        else:
            return False
    
    def distance_to_go(self):
        return self._targetPos - self._currentPos
    
    def target_position(self):
        return self._targetPos
    
    def current_position(self):
        return self._currentPos

    def set_current_position(self, position):
        self._targetPos = self._currentPos = position
        self._n = 0
        self._stepInterval = 0
        self._speed = 0.0

    '''
    // Subclasses can override
unsigned long AccelStepper::computeNewSpeed()
{
    long distanceTo = distanceToGo(); // +ve is clockwise from curent location

    long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1)
    {
	// We are at the target and its time to stop
	_stepInterval = 0;
	_speed = 0.0;
	_n = 0;
	return _stepInterval;
    }

    if (distanceTo > 0)
    {
	// We are anticlockwise from the target
	// Need to go clockwise from here, maybe decelerate now
	if (_n > 0)
	{
	    // Currently accelerating, need to decel now? Or maybe going the wrong way?
	    if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
		_n = -stepsToStop; // Start deceleration
	}
	else if (_n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
		_n = -_n; // Start accceleration
	}
    }
    else if (distanceTo < 0)
    {
	// We are clockwise from the target
	// Need to go anticlockwise from here, maybe decelerate
	if (_n > 0)
	{
	    // Currently accelerating, need to decel now? Or maybe going the wrong way?
	    if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
		_n = -stepsToStop; // Start deceleration
	}
	else if (_n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
		_n = -_n; // Start accceleration
	}
    }

    // Need to accelerate or decelerate
    if (_n == 0)
    {
	// First step from stopped
	_cn = _c0;
	_direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else
    {
	// Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
	_cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
	_cn = max(_cn, _cmin); 
    }
    _n++;
    _stepInterval = _cn;
    _speed = 1000000.0 / _cn;
    if (_direction == DIRECTION_CCW)
	_speed = -_speed;

#if 0
    Serial.println(_speed);
    Serial.println(_acceleration);
    Serial.println(_cn);
    Serial.println(_c0);
    Serial.println(_n);
    Serial.println(_stepInterval);
    Serial.println(distanceTo);
    Serial.println(stepsToStop);
    Serial.println("-----");
#endif
    return _stepInterval;
}'''

    def compute_new_speed(self):
        distance_to = self.distance_to_go()
        steps_to_stop = (self._speed * self._speed) / (2.0 * self._acceleration)

        if(distance_to == 0 and steps_to_stop <= 1):
            self._stepInterval = 0
            self._speed = 0.0
            self._n = 0
            return self._stepInterval
        
        if(distance_to > 0):
            if(self._n > 0):
                if(steps_to_stop >= distance_to or self._direction == self.DIRECTION_CCW):
                    self._n = -steps_to_stop
            elif(self._n < 0):
                if(steps_to_stop < distance_to or self._direction == self.DIRECTION_CW):
                    self._n = -self._n

        elif(distance_to < 0):
            if(self._n > 0):
                if(steps_to_stop >= -distance_to or self._direction == self.DIRECTION_CW):
                    self._n = -steps_to_stop
            elif(self._n < 0):
                if(steps_to_stop < -distance_to and self._direction == self.DIRECTION_CCW):
                    self._n = -self._n
        
        if(self._n == 0):
            self._cn = self._c0
            self._direction = self.DIRECTION_CW if distance_to > 0 else self.DIRECTION_CCW
        else:
            self._cn = self._cn - ((2.0 * self._cn) / ((4.0 * self._n) + 1))
            self._cn = max(self._cn, self._cmin)

        self._n += 1
        self._stepInterval = self._cn
        self._speed = 1000000.0 / self._cn 
        if(self._direction == self.DIRECTION_CCW):
            self._speed = -self._speed
        return self._stepInterval
    

    def run(self):
        if(self.run_speed()):
            self.compute_new_speed()
        return self._speed != 0.0 or self.distance_to_go() != 0

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

    def speed(self):
        return self._speed

    def disable_outputs(self):
        if(not self._interface):
            return
        set_output_pins(0)
        if(self._enablePin != 0xff):
            digitalWrite(self._enablePin, not self._enableInverted)

    def enable_outputs(self):
        if (not self._interface):
            return
        pinMode(self._pin[0], OUTPUT)
        pinMode(self._pin[1], OUTPUT)
        if(self._interface == FULL4WIRE || self._interface == HALF4WIRE):
            pinMode(self._pin[2], OUTPUT)
            pinMode(self._pin[3], OUTPUT)
        else if (self._interface == FULL3WIRE || self._interface == HALF3WIRE):
            pinMode(self._pin[2], OUTPUT)
        if (self._enablePin != 0xff):
            pinMode(self._enablePin, OUTPUT)
            digitalWrite(self._enablePin, self._enableInverted)

    def set_min_pulse_width(self, minWidth):
        self._minPulseWidth = minWidth

    def set_enable_pin(self, enable_pin):
        self._enablePin = enable_pin

    def set_pins_inverted(self, directionInvert, stepInvert, enableInvert):
        self._pinInverted[0] = directionInvert
        self._pinInverted[1] = stepInvert
        self._enableInverted = enableInvert
    
    def set_pins_inverted(self, pin1Invert, pin2Invert, pin3Invert, pin4Invert, enableInvert):
        self._pinInverted[0] = pin1Invert
        self._pinInverted[1] = pin2Invert
        self._pinInverted[2] = pin3Invert
        self._pinInverted[3] = pin4Invert
        self._enableInverted = enableInvert

    def run_to_position(self):
        while(run()):
            time.sleep(0.0020) # CHECK THIS

    def run_speed_to_position(self):
        if (self._targetPos == self._currentPos):
            return False;
        if (self._targetPos > self._currentPos):
            self._direction = self.DIRECTION_CW
        else:
            self._direction = self.DIRECTION_CCW 
        return run_speed()

    def run_to_new_position(self, position):
        move_to(position)
        run_to_position()

    def stop(self):
        if (self._speed != 0.0):
            stepsToStop = ((self._speed * self._speed) / (2.0 * self._acceleration)) + 1 

            if (self._speed > 0):
                self.move(stepsToStop)
            else:
                self.move(-stepsToStop)

    def is_running(self):
        return not (self._speed == 0.0 and  self._targetPos == self._currentPos)