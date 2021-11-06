import RPi.GPIO as GPIO
from optparse import OptionParser
from gpiozero import Servo
from time import sleep

class SubCollector:
    COL_OPEN = -0.8
    COL_CLOSED = 0.7
    
    # TAKEN FROM: https://stackoverflow.com/questions/18265935/python-create-list-with-numbers-between-2-values?rq=1
    def frange(start, stop, step=1.0):
    ''' "range()" like function which accept float type''' 
    i = start
    while i < stop:
        yield i
        i += step

    def __init__(self, pin):
        self.sampleInBay = False
        self.collectorOpen = False

        angleCorrection = 0.45
        maxPW = (2.0 + angleCorrection)/1000
        minPW = (1.0 - angleCorrection)/1000 

        self.servo = Servo(pin,min_pulse_width=minPW,max_pulse_width=maxPW)

    def AngleToAbs(self, angle:int) -> float:
        return (int(angle)/90.0) - 1.0

    def StateToAbs(self, state:bool) -> float:
        if state:
            return self.COL_OPEN
        else:
            return self.COL_CLOSED
    
    def SetServo(self, state, curstate, slow):
        if slow:
            for i in frange(curstate,state,0.1):
                self.servo.value = i
                sleep(0.1)
        else:
            self.servo.value = state
        sleep(0.3)

def Dance(pin1:int, pin2:int):
    pass

def Main():
    parser = OptionParser()

    parser.add_option('-s', '--state', dest='state', action='store',type='float')
    parser.add_option('-c', '--curstate', dest='curstate', action='store', type='float')
    parser.add_option('-p', '--pin', dest='pin', action='store',type='int')
    #parser.add_option('-p2', '--pin2', dest='pin2', action='store', type='int')
    parser.add_option('--slow', dest='slow', action='store_true', default = False)
    parser.add_option('-d', dest='dance', action='store_true', default=False)

    (options,args) = parser.parse_args()
    if options.dance:
        #Dance(options.pin, options.pin2)
        pass
    else:
        col = SubCollector(options.pin)
        col.SetServo(options.state, options.curstate, options.slow) 

if __name__ == '__main__':
    Main()
    exit(0)
