### LIBRARIES ###
import io
import numpy as np 
import time
import cv2
import cv2 as cv
from math import atan,sin,cos,pi,floor


### PROCESS IMAGE FUNCTION ###

class Vision:

    def find_objects(self, frame):              
    
        
        #Create kernal to denoise all binary images
        kernel = np.ones((5,5))
        
        #Blur frame to decrease sharp edges and change to HSV colour space
        frame = cv2.flip(frame, 0)
        frame_blur = cv2.GaussianBlur(frame, (3,3),0)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray,(3,3),0)
        cv2.imshow('blur_gray', blur_gray)
    
        # use canny edge detector
        edges = cv2.Canny(blur_gray, threshold1=50, threshold2=150)
        cv2.imshow('Edges', edges)
        
        
        # define the Hough Transform parameters
        rho = 2 # distance resolution in pixels of the Hough grid
        theta = np.pi/180 # angular resolution in radians of the Hough grid
        threshold = 15     # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 50 #minimum number of pixels making up a line
        max_line_gap = 30    # maximum gap in pixels between connectable line segments
    
        # make a blank the same size as the original image to draw on
        line_image = np.copy(frame)*0 
    
        # run Hough on edge detected image
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),min_line_length, max_line_gap)
    
        print(type(lines))
        
        if lines is not None:
            for line in lines:
                for x1,y1,x2,y2 in line:
                    cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),10)
        
        gradient = (x2-x1)/(y2-y1)
        # draw the line on the original image 
        if gradient == 0:
            lines_edges = cv2.addWeighted(frame, 0.8, line_image, 1, 0)
            cv2.imshow('LinesEDGES', lines_edges)
    # want to use the angle
    ### FOCAL CALCULATOR ###
    
        # Calculate percieved focal length
        percWidth = 64
        realDist = 15
        realWidth = 4.67
        #~ percFocal = (percWidth * realDist)/realWidth
        #~ print(percFocal)
        percFocal = 206
        
        width = 0.00368
        pix_deg = atan(240/320)
        fov_div_2 = atan((width/2)/percFocal)
        fov = (180/pi)*fov_div_2*2
        
    ### COLOUR RECOGNITION ###
    
        #Threshold for red colour using two thresholds
        red_mask1_low = (168,100,0)
        red_mask1_upper = (180,255,220)
        
        red_mask2_low = (0, 150, 80)
        red_mask2_upper = (12, 255, 255)
        
        #create two binary masks for red
        red_binary_mask1 = cv2.inRange(hsv_frame, red_mask1_low, red_mask1_upper)
        red_binary_mask2 = cv2.inRange(hsv_frame, red_mask2_low, red_mask2_upper)
        cv2.imshow("red1", red_binary_mask1)
        cv2.imshow("red2", red_binary_mask2)
        
        red_binary_mask1 = cv2.dilate(red_binary_mask1, kernel)
        red_binary_mask2 = cv2.dilate(red_binary_mask2, kernel)
        
        #Combine binary masks for red and denoise signal
        red_binary = cv2.bitwise_or(red_binary_mask1, red_binary_mask2)
        cv2.imshow("bitwise", red_binary)
        red_binary = cv2.morphologyEx(red_binary, cv2.MORPH_OPEN, kernel)
        cv2.imshow("morphology1", red_binary)
        red_binary = cv2.morphologyEx(red_binary, cv2.MORPH_CLOSE, kernel)
        cv2.imshow("morphology1", red_binary)
    
    ### DRAW BOUNDING BOXES ###    
        _,contours,_ = cv2.findContours(red_binary,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        red_range = False
        red_binary = False
        red_x_bearing = False
        
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour) 
            red_mom = cv2.moments(contour)
            if(area > 400):
                x,y,w,h = cv2.boundingRect(contour)
                #~ print(w)
                frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
        
                red_cx = int(red_mom['m10']/red_mom['m00'])
                red_cy = int(red_mom['m01']/red_mom['m00'])
                
                #Define centre of the objects, draw the centroid and bounding box on the frame
                #red
                red_cent = (red_cx,red_cy)
                cv2.circle(frame, red_cent, 5, (0,0,255), -1)
            
    ### RANGE ### 
                red_range = (percFocal*realWidth)/w
                print("red_range:", red_range)
                
                # Show range
                textDist = "D:" + str(round(red_range))
                cv2.putText(frame, textDist, (x - 10, y - 3),
                cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)  
        
    ### BEARING ###        
                # bearing
                red_x_bearing = ((red_cx - 320/2) / (320/ 62.2))
                print("red_bearing:",red_x_bearing)
                
                # Show bearing
                textbearing = "A:" + str(round(red_x_bearing))
                cv2.putText(frame, textbearing, (x + 40, y - 3),
                cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)
        
        #Find centre of image and calculate bearing to it
        ref = (int(frame.shape[1]/2),int(frame.shape[0]/2))
        cv2.circle(frame, ref, 5, (255,255,0), -1)
            
        
        return red_binary,frame,red_range,red_x_bearing
    
        
    def main(self):
        while(1):
            self.ret, frame = self.cap.read()                #Get a frame from the camera 
            if self.ret == True:
                red_binary,frame,red_range,red_x_bearing= self.find_objects(frame)
                cv2.imshow('orig_img', frame)
                cv2.imshow('red_binary_image',red_binary)
            
            cv2.imshow("ROVER OBJECT DETECTION", frame) 
            if cv2.waitKey(10) & 0xFF == ord('q'): 
                cv2.destroyAllWindows() 
                break
            
            
    
    def __init__(self):
        ### INITIATE VIDEO CAPTURE ###
        self.cap = cv2.VideoCapture(0)                               #Connect to camera 0
        self.cap.set(3,320)                                          #set width to 320
        self.cap.set(4,240)                                  #Get a frame from the camera


vision = Vision()
vision.main()
    

  
