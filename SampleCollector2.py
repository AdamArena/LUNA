from subprocess import call, Popen, PIPE
from time import sleep

class SampleCollector:

    COL_PIN = 26
    FLIP_PIN = 5
    
    COL_OPEN = -0.9
    COL_CLOSED = 0.7
    FLIP_OPEN = -0.68
    FLIP_CLOSED = 0.3
    
    colOpen = False
    flipOpen = False



    def __init__(self):
        self.PrepArms()



    def CloseAll(self):
        self.FlipRock()
        sleep(0.5)
        self.CloseCollector()



    def OpenCollector(self, slow):
        if self.colOpen and not self.flipOpen:
            pass
        else:
            if slow:
                _ = call(['python3', 'CollectorCoProc.py', '--pin', str(self.COL_PIN), '-s', str(self.COL_OPEN), '--slow', '-c', str(self.COL_CLOSED)])
            else:
                _ = call(['python3', 'CollectorCoProc.py', '--pin', str(self.COL_PIN), '-s', str(self.COL_OPEN)])
        self.colOpen = True



    def CloseCollector(self, slow):
        if self.colOpen and not self.flipOpen:
            if slow:
                _ = call(['python3', 'CollectorCoProc.py', '--pin', str(self.COL_PIN), '-s', str(self.COL_CLOSED), '--slow', '-c', str(self.COL_OPEN)])
            else:
                _ = call(['python3', 'CollectorCoProc.py', '--pin', str(self.COL_PIN),'-s', str(self.COL_CLOSED)])
            self.colOpen = False
        else:
            pass
        

    def PrepArms(self):
        self.OpenCollector()
        sleep(0.2)
        _ = call(['python3', 'CollectorCoProc.py', '--pin', str(self.FLIP_PIN), '-s', str(self.FLIP_OPEN)])
        self.flipOpen = True



    def FlipRock(self, half_flip):
        if self.flipOpen and self.colOpen:
            if half_flip:
                _ = call(['python3', 'CollectorCoProc.py', '--pin', str(self.FLIP_PIN), '-s', str(-0.05)])
                sleep(0.5)
                _ = call(['python3', 'CollectorCoProc.py', '--pin', str(self.FLIP_PIN), '-s', str(self.FLIP_CLOSED)])
            else:
                _ = call(['python3', 'CollectorCoProc.py', '--pin', str(self.FLIP_PIN), '-s', str(self.FLIP_CLOSED)])
        else:
            pass
        self.flipOpen = False
    
    
    
    def Dance(self):
        pass
        for i in range(10):
            pass
    
    
    
    def arm_test(self):
        while True:
            val = input("Please input. 1 = prep arms, 2 = flip rock, 3 = close collector, 4 = open collector: ")
            print('\n')
            
            if val == '1':
                self.PrepArms()
            elif val == '2':
                self.FlipRock(True)
            elif val == '3':
                self.CloseCollector(False)
            elif val == '4':
                self.OpenCollector()



if __name__ == "__main__":
    test = SampleCollector()
    test.arm_test()

