

def red_filter(frame):
    #Threshold for red colour using two thresholds
    red_mask1_low = (168,100,0)
    red_mask1_upper = (180,255,220)
    
    red_mask2_low = (0, 150, 80)
    red_mask2_upper = (12, 255, 255)
    
    #create two binary masks for red
    red_binary_mask1 = cv2.inRange(hsv_frame, red_mask1_low, red_mask1_upper)
    red_binary_mask2 = cv2.inRange(hsv_frame, red_mask2_low, red_mask2_upper)
    
    red_binary_mask1 = cv2.dilate(red_binary_mask1, kernel)
    red_binary_mask2 = cv2.dilate(red_binary_mask2, kernel)
    red_binary = cv2.bitwise_or(red_binary_mask1, red_binary_mask2)
    cv2.imshow("bitwise", red_binary)
    red_binary = cv2.morphologyEx(red_binary, cv2.MORPH_OPEN, kernel)
    cv2.imshow("morphology1", red_binary)
    red_binary = cv2.morphologyEx(red_binary, cv2.MORPH_CLOSE, kernel)
    cv2.imshow("morphology1", red_binary)
    
    return red_binary
    
    
def red_contours(red_binary)
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
    return red_cent, red_cx, red_cy
   
   
    
def red_range(red_cent,red_cx,frame):
    red_range = (percFocal*realWidth)/w
    print("red_range:", red_range)
            
    # Show range
    textDist = "D:" + str(round(red_range))
    cv2.putText(frame, textDist, (x - 10, y - 3),
    cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)  

def red_bearing(red_cx,frame)
    red_x_bearing = ((red_cx - 320/2) / (320/ 62.2))
    print("red_bearing:",red_x_bearing)
    
    # Show bearing
    textbearing = "A:" + str(round(red_x_bearing))
    cv2.putText(frame, textbearing, (x + 40, y - 3),
    cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)
    
    


def find_colour(frame):

    #Blur frame to decrease sharp edges and change to HSV colour space
    frame = cv2.flip(frame, 0)
    frame_blur = cv2.GaussianBlur(frame, (3,3),0)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    red_filter()
    
    
    
    
    
    
