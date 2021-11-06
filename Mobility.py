import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

class Pwmcheck:
    def bound(self,l,r):
          if l > 99:
              l = l - (l - 99)
              r = r + (l - 99)
          elif r > 99:
              l = r - (r - 99)
              r = l + (l - 99)
          elif l < min_pwm:
              l = l + (min_pwm-l)
              r = r + (min_pwm-l)
          elif r < min_pwm:
              r = r + (min_pwm-r)
              l = l + (min_pwm-r)
          else:
              l = l
              r = r
      
          return l,r

class Motor:
    def __init__(self, Ena, In1, In2):
        self.Ena = Ena
        self.In1 = In1
        self.In2 = In2

        GPIO.setup(self.Ena, GPIO.OUT) # Pin outputs
        GPIO.setup(self.In1, GPIO.OUT)
        GPIO.setup(self.In2, GPIO.OUT)

        self.pwm = GPIO.PWM(self.Ena, 1000) # 1000 is the frequency value 500 to 2000 is recommended for pi software pwm
        self.pwm.start(0)

    def moveB(self, x):  # a function inside a class is called a method
        GPIO.output(self.In1, GPIO.LOW)
        GPIO.output(self.In2, GPIO.HIGH) #High and low order changes the directions i.e this block is back motion
        self.pwm.ChangeDutyCycle(x)


    def moveF(self, x):  # ,t)  # a function inside a class is called a method,
        GPIO.output(self.In1, GPIO.HIGH)
        GPIO.output(self.In2, GPIO.LOW)  # High and low order changes the directions i.e this block is forward motion
        self.pwm.ChangeDutyCycle(x)


    def stop(self):
        self.pwm.ChangeDutyCycle(0)

class encoder:
    def __init__(self, e):
        self.e = GPIO.input(e)
        GPIO.setup(e,GPIO.IN,pull_up_down=GPIO.PUD_UP)
        
    def count(self):
        stateLast = self.e
        rotationCount = 0
        stateCount = 0
        stateCountTotal = 0
        circ = 0.12246 #
        statesPerRotation = 12 * 75
        distanceperstep = circ/statesPerRotation
        timetaken = time.time()
    
        stateCurrent = self.e  # current state of the encoder
        if stateCurrent != stateLast:  # if gpio in does not equal statelast=0
            stateLast = stateCurrent  # then statelastl becomes statecurrent
            stateCount += 1  # statecount begins counting, resets every rotation see below if:
            stateCountTotal += 1  # statecounttotal
        # counter for number of rotations
        if stateCount == statesPerRotation:  # once statecount reaches states per rotation reset count to 0
            rotationCount += 1  # add value to rotation count
            stateCount = 0  # reset the count to 0

        velocity = (distanceperstep * stateCountTotal)/timetaken

        return velocity 

#encoderLA = encoder(7)
#encoderLB = encoder(38)
#encoderRA = encoder(36)
#encoderRB = encoder(40)

class Mobility:
    def __init__(self):
        self.motorL = Motor(32, 16, 18)  # Ena1(pwm pins), in1, in2 pin numbers
        self.motorR = Motor(33, 10, 12)  # Ena2(pwm pins), in3, in4 pin numbers

    def set_velocities(self, x, y):
        #x0, y0 = [x, y]
        max_vel = 0.45
        min_vel = 0.065
        max_rot = 0.785
        
        if x > max_vel:
            x = max_vel
        elif x < -max_vel:
            x = -max_vel
        elif -min_vel < x < -min_vel/2:
            x = -min_vel
        elif -min_vel/2 < x < 0:
            x = 0
        elif min_vel/2 < x < min_vel:
            x = min_vel
        elif 0 < x < min_vel/2:
            x = 0

        if y > max_rot:
            y = max_rot
        elif y < -max_rot:
            y = -max_rot

        # (abs(y) * 0.0665) = 0.0522025 / 2 = 0.02610125 m/s at 0.785 rad/s in opposite directions
        pwm_vel = (((150.65 * abs(x)) + 7.2078)) 
        pwm_rotational = (((150.65 * (abs(y) * 0.0665))) + 7.2078) * 3.5
        left_motor = 0
        right_motor = 0
        #VL = 0
        #VR = 0
        
        check = Pwmcheck()

        if x > 0 and y > 0: # turn right forward
            left_motor = (pwm_vel + pwm_rotational)  # * n  multiply to remove vearing
            right_motor = (pwm_vel - pwm_rotational)
            [left_motor, right_motor] = check.bound(left_motor, right_motor)
            self.motorL.moveF(left_motor) 
            self.motorR.moveF(right_motor)
            #VL = encoderLA.count()
            #VR = encoderRA.count()
        
        elif x < 0 and y > 0: # turn left negative velocity
            left_motor = (pwm_vel + pwm_rotational)
            right_motor = (pwm_vel - pwm_rotational)
            [left_motor, right_motor] = check.bound(left_motor, right_motor)
            self.motorL.moveB(left_motor)
            self.motorR.moveB(right_motor)
            #VL = encoderLB.count()
            #VR = encoderRB.count()
 
        elif x > 0 and y < 0: # turn right forward
            left_motor = (pwm_vel - pwm_rotational)
            right_motor = (pwm_vel + pwm_rotational) 
            [left_motor, right_motor] = check.bound(left_motor, right_motor)
            self.motorL.moveF(left_motor)
            self.motorR.moveF(right_motor)
            #VL = encoderLA.count()
            #VR = encoderRA.count()

        elif x < 0 and y < 0: # turn right negative velocity
            left_motor = (pwm_vel - pwm_rotational)
            right_motor = (pwm_vel + pwm_rotational)
            [left_motor, right_motor] = check.bound(left_motor, right_motor)
            self.motorL.moveB(left_motor)
            self.motorR.moveB(right_motor)
            #VL = encoderLB.count()
            #VR = encoderRB.count()

        elif x == 0 and y > 0:  # turn right onspot
            pwm_rotational = (((288.724 * ((abs(y) * 0.0665))) + 19)) * 2.3
            left_motor = (pwm_rotational)
            right_motor = (pwm_rotational)
            [left_motor, right_motor] = check.bound(left_motor, right_motor)
            self.motorL.moveF(left_motor)
            self.motorR.moveB(right_motor)
            #VL = encoderLA.count()
            #VR = encoderRB.count()

        elif x == 0 and y < 0: # turn left onspot
            pwm_rotational = (((288.724 * ((abs(y) * 0.0665))) + 19)) * 2.3
            left_motor = (pwm_rotational)
            right_motor = (pwm_rotational)
            [left_motor, right_motor] = check.bound(left_motor, right_motor)
            self.motorL.moveB(left_motor)
            self.motorR.moveF(right_motor)
            #VL = encoderLB.count()
            #VR = encoderRA.count()

        elif x > 0 and y == 0:
            left_motor = (pwm_vel)
            right_motor = (pwm_vel)
            [left_motor, right_motor] = check.bound(left_motor, right_motor)
            self.motorL.moveF(left_motor)
            self.motorR.moveF(right_motor)
            #VL = encoderLA.count()
            #VR = encoderRA.count()
            
        elif x < 0 and y == 0:
            left_motor = (pwm_vel)
            right_motor = (pwm_vel)
            [left_motor, right_motor] = check.bound(left_motor, right_motor)
            self.motorL.moveB(left_motor)
            self.motorR.moveB(right_motor)
            #VL = encoderLB.count()
            #VR = encoderRB.count()

        elif x == 0 and y == 0:
            self.motorR.stop()
            self.motorL.stop()
            

        #return VL, VR
        
        

min_pwm = 20


if __name__ == "__main__":
    test = Mobility()
    
    while True:
        vel = float(input("Forward velocity: "))
        rot = float(input("Rotational velocity: "))
        
        test.set_velocities(vel, rot)
        
        input("Press enter to stop running")
        test.set_velocities(0, 0)




