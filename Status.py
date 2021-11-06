import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BOARD)
class Status:
    import time
    
    def strobe(self):
        for _ in range(5):
            lst = ['s', 'c', 'r']
            
            for i in range(3):
                self.update_status(lst[i])
                time.sleep(0.2)
                
                
                
    def update_status(self, status):
        GPIO.output(self.search_LED, GPIO.LOW)
        GPIO.output(self.collect_LED, GPIO.LOW)
        GPIO.output(self.return_LED, GPIO.LOW)
        
        #s = white, c = yellow, l = green
        
        
        if status == 's': # s = searching
            GPIO.output(self.search_LED, GPIO.HIGH)
            
        elif status == 'c': # c = collecting
            GPIO.output(self.collect_LED, GPIO.HIGH)
            
        elif status == 'r': # r = returning
            GPIO.output(self.return_LED, GPIO.HIGH)
        
        
        
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        self.search_LED = 3
        self.collect_LED = 5
        self.return_LED = 11
        
        GPIO.setup(self.search_LED, GPIO.OUT)
        GPIO.setup(self.collect_LED, GPIO.OUT)
        GPIO.setup(self.return_LED, GPIO.OUT)
        
        GPIO.output(self.search_LED, GPIO.LOW)
        GPIO.output(self.collect_LED, GPIO.LOW)
        GPIO.output(self.return_LED, GPIO.LOW)



if __name__ == '__main__':
    status = Status()
    lst = ['s', 'c', 'r']
    
    while True:
        val = input("1-3 input is LEDs 1-3. 4 is dance for 3 seconds: ")
        
        if val in ['1', '2', '3']:
            status.update_status(lst[int(val)-1])
        elif val == '4':
            status.strobe()
