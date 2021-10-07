### LIBRARIES ###
import io
import numpy as np 
import time
import cv2
import cv2 as cv
from math import atan,sin,cos,pi,floor
import matplotlib.pyplot as plt


### INITIATE VIDEO CAPTURE ###

cap = cv2.VideoCapture(0)                               #Connect to camera 0
cap.set(3,320)                                          #set width to 320
cap.set(4,240)                                  #Get a frame from the camera


### PROCESS IMAGE FUNCTION ###

def find_objects(frame):              

    #~ # intialisation state
    Orange_img_binary = False
    Orange_Distance = False
    Orange_x_angle = False
    
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
    print(lines)
    
    if(ret):
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),10)
    # draw the line on the original image 
    lines_edges = cv2.addWeighted(frame, 0.8, line_image, 1, 0)
    cv2.imshow('LinesEDGES', lines_edges)

    #Threshold for orange colour using two thresholds
    Orange_mask1_low = (168,100,0)
    Orange_mask1_upper = (180,255,220)
    
    Orange_mask2_low = (0, 150, 80)
    Orange_mask2_upper = (12, 255, 255)
    
    
    #create two binary masks for orange
    Orange_img_binary_mask1 = cv2.inRange(hsv_frame, Orange_mask1_low, Orange_mask1_upper)
    Orange_img_binary_mask2 = cv2.inRange(hsv_frame, Orange_mask2_low, Orange_mask2_upper)
    cv2.imshow("orange1", Orange_img_binary_mask1)
    cv2.imshow("orange2", Orange_img_binary_mask2)
    
    Orange_img_binary_mask1 = cv2.dilate(Orange_img_binary_mask1, kernel)
    Orange_img_binary_mask2 = cv2.dilate(Orange_img_binary_mask2, kernel)
    
    #Combine binary masks for orange and denoise signal
    Orange_img_binary = cv2.bitwise_or(Orange_img_binary_mask1, Orange_img_binary_mask2)
    cv2.imshow("bitwise", Orange_img_binary)
    Orange_img_binary = cv2.morphologyEx(Orange_img_binary, cv2.MORPH_OPEN, kernel)
    cv2.imshow("morphology1", Orange_img_binary)
    Orange_img_binary = cv2.morphologyEx(Orange_img_binary, cv2.MORPH_CLOSE, kernel)
    cv2.imshow("morphology1", Orange_img_binary)

    
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
    
    contours,_ = cv2.findContours(Orange_img_binary,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour) 
        red_mom = cv2.moments(contour)
        if(area > 400):
            x,y,w,h = cv2.boundingRect(contour)
            #~ print(w)
            frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
    
            Orange_cx = int(red_mom['m10']/red_mom['m00'])
            Orange_cy = int(red_mom['m01']/red_mom['m00'])
            
            #Define centre of the objects, draw the centroid and bounding box on the frame
            #Orange
            Orange_cent = (Orange_cx,Orange_cy)
            cv2.circle(frame, Orange_cent, 5, (0,0,255), -1)
        
            # Distance
            Orange_Distance = (percFocal*realWidth)/w
            print("Orange_distance:", Orange_Distance)
            
            # Show Distance
            textDist = "D:" + str(round(Orange_Distance))
            cv2.putText(frame, textDist, (x - 10, y - 3),
            cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)  
    
        
            # Angle
            Orange_x_angle = ((Orange_cx - 320/2) / (320/ 62.2))
            print("Orange_angle:",Orange_x_angle)
            
            # Show Angle
            textAngle = "A:" + str(round(Orange_x_angle))
            cv2.putText(frame, textAngle, (x + 40, y - 3),
            cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)
    
    #Find centre of image and calculate angle to it
    ref = (round(frame.shape[1]/2),round(frame.shape[0]/2))
    cv2.circle(frame, ref, 5, (255,255,0), -1)
        
    
    return lines_edges, Orange_img_binary,frame,Orange_Distance,Orange_x_angle
    
        
while(1):
    ret, frame = cap.read()                #Get a frame from the camera 
    if ret == True:
        lines_edges, Orange_img_binary,frame,Orange_Distance,Orange_x_angle= find_objects(frame)
        cv2.imshow('orig_img', frame)
        cv2.imshow('Orange_binary_image',Orange_img_binary)
    
    cv2.imshow("ROVER OBJECT DETECTION", frame) 
    if cv2.waitKey(10) & 0xFF == ord('q'): 
        cv2.destroyAllWindows() 
        break
    

  
