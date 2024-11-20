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

    FUNCTION = 0
    DRIVER = 1
    FULL2WIRE = 2
    FULL3WIRE = 3
    FULL4WIRE = 4
    HALF3WIRE = 6
    HALF4WIRE = 8
    
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
        self.set_max_speed(1)

        self._forward = None
        self._backward = None

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
                if(steps_to_stop < distance_to and self._direction == self.DIRECTION_CW):
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
    
    def step(self, step):
        if(self._interface == self.FUNCTION):
            self.step0(step)
        elif(self._interface == self.DRIVER):
            self.step1(step)
        elif(self._interface == self.FULL2WIRE):
            self.step2(step)
        elif(self._interface == self.FULL3WIRE):
            self.step3(step)
        elif(self._interface == self.FULL4WIRE):
            self.step4(step)
        elif(self._interface == self.HALF3WIRE):
            self.step6(step)
        elif(self._interface == self.HALF4WIRE):
            self.step8(step)

    '''// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to _pin[0]
// bit 1 of the mask corresponds to _pin[1]
// ....
void AccelStepper::setOutputPins(uint8_t mask)
{
    uint8_t numpins = 2;
    if (_interface == FULL4WIRE || _interface == HALF4WIRE)
	numpins = 4;
    else if (_interface == FULL3WIRE || _interface == HALF3WIRE)
	numpins = 3;
    uint8_t i;
    for (i = 0; i < numpins; i++)
	digitalWrite(_pin[i], (mask & (1 << i)) ? (HIGH ^ _pinInverted[i]) : (LOW ^ _pinInverted[i]));
}'''

    def set_output_pins(self, mask):
        numpins = 2
        if self._interface == self.FULL4WIRE or self._interface == self.HALF4WIRE:
            numpins = 4
        elif self._interface == self.FULL3WIRE or self._interface == self.HALF3WIRE:
            numpins = 3
        for i in range(numpins):
            # digitalWrite(_pin[i], (mask & (1 << i)) ? (HIGH ^ _pinInverted[i]) : (LOW ^ _pinInverted[i]))
            pass
    def step0(self, step):
        if(self._speed > 0):
            if self._forward:
                self._forward()
        else:
            if self._backward:
                self._backward()


    def step1(self, step):
        self.set_output_pins(0b10 if self._direction else 0b00)
        self.set_output_pins(0b11 if self._direction else 0b01)

        time.sleep(self._minPulseWidth / 1000000.0)
        self.set_output_pins(0b10 if self._direction else 0b00)

    def step2(self, step):
        step = step & 0x3
        if step == 0:
            self.set_output_pins(0b10)
        elif step == 1:
            self.set_output_pins(0b11)
        elif step == 2:
            self.set_output_pins(0b01)
        elif step == 3:
            self.set_output_pins(0b00)

    def step3(self, step):
        step = step % 0x3
        if step == 0:
            self.set_output_pins(0b100)
        elif step == 1:
            self.set_output_pins(0b001)
        elif step == 2:
            self.set_output_pins(0b010)

    def step4(self, step):
        step = step & 0x3
        if step == 0:
            self.set_output_pins(0b0101)
        elif step == 1:
            self.set_output_pins(0b0110)
        elif step == 2:
            self.set_output_pins(0b1010)
        elif step == 3:
            self.set_output_pins(0b1001)

    def step6(self, step):
        step = step % 6
        if step == 0:
            self.set_output_pins(0b100)
        elif step == 1:
            self.set_output_pins(0b101)
        elif step == 2:
            self.set_output_pins(0b001)
        elif step == 3:
            self.set_output_pins(0b011)
        elif step == 4:
            self.set_output_pins(0b010)
        elif step == 5:
            self.set_output_pins(0b110)

    def step8(self, step):
        step = step & 0x7
        if step == 0:
            self.set_output_pins(0b0001)
        elif step == 1:
            self.set_output_pins(0b0101)
        elif step == 2:
            self.set_output_pins(0b0100)
        elif step == 3:
            self.set_output_pins(0b0110)
        elif step == 4:
            self.set_output_pins(0b0010)
        elif step == 5:
            self.set_output_pins(0b1010)
        elif step == 6:
            self.set_output_pins(0b1000)
        elif step == 7:
            self.set_output_pins(0b1001)


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