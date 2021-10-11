import RPi.GPIO as GPIO
from gpiozero import Servo
from time import sleep

class SampleCollector:

    COL_OPEN = 1.0
    COL_CLOSED = -1.0

    def __init__(self, collectorOutputPin: int, flipperOutputPin: int):
        self.sampleInBay = False
        self.collectorOpen = False

        angleCorrection = 0.45
        maxPW = (2.0 + angleCorrection)/1000
        minPW = (1.0 - angleCorrection)/1000 

        self.colServo = Servo(collectorOutputPin,min_pulse_width=minPW,max_pulse_width=maxPW)
        self.flipServo = Servo(flipperOutputPin,min_pulse_width=minPW,max_pulse_width=maxPW)

    def AngleToAbs(self, angle:int) -> float:
        return (int(angle)/90.0) - 1.0

    def StateToAbs(self, state:bool) -> float:
        if state:
            return self.COL_OPEN
        else:
            return self.COL_CLOSED

########## SAMPLE COLLECTOR ARM FUNCTIONS ##########

    def Release(self):
        """
        Function specifically for releasing the sample into the lander
        """
        pass

    def OpenCollector(self):
        """
        Opens the collector if it is closed. Else nothing happens.
        """
        if self.collectorOpen:
            pass
        else:
            self.colServo.value = self.COL_OPEN
        self.collectorOpen = True

    def CloseCollector(self):
        """
        Closes the collector if it is open. Else nothing happens.
        """
        if self.collectorOpen:
            self.colServo.value = self.COL_CLOSED
        else:
            pass
        self.collectorOpen = False

    def CloseCollectorSlow(self):
        PRESCALER = 20
        for i in range(-PRESCALER,PRESCALER):
            self.colServo.value = (i/float(PRESCALER))
            print(self.colServo.value)
            sleep(0.1)

    
    def SetCollector(self, collectorState: bool):
        """
        True = open. The collector arm is out ready to collect the sample.
        False = closed. the collector is holding a ball.
        """
        self.colServo.value = self.StateToAbs(collectorState)
        self.collectorOpen = collectorState

    def ToggleCollector(self) -> bool:
        """
        Switched the collectors current state and returns the state it is now in.
        """
        self.collectorOpen = not self.collectorOpen
        self.colServo.value = self.StateToAbs(self.collectorOpen)
        return self.collectorOpen


######### ROCK FLIPPER ARM FUNCTIONS ##########

    FLIPPER_OPEN = 0.0
    FLIPPER_CLOSED = 1.0

    def PrepareArms(self):
        self.SetCollector(True)
        sleep(0.5)
        self.flipServo.value = self.FLIPPER_OPEN


    def LiftRock(self):
        self.FLIPPER_CLOSED
    

