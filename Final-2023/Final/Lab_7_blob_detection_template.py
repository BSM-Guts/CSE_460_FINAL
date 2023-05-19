### this code works only on the raspberry
import numpy as np
import cv2 as cv
from picamera2 import Picamera2 
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()
from Motor import *            
PWM=Motor()         

try:  
    while True:
        # Camera Part
        frame = picam2.capture_array()
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # Threshold of blue in HSV space
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        # preparing the mask to overlay
        mask = cv.inRange(hsv, lower_yellow, upper_yellow)
        #cv.imshow("m", mask)

        

        # The black region in the mask has the value of 0,
        # so when multiplied with original image removes all non-blue regions
        result = cv.bitwise_and(frame, frame, mask = mask)

        gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
        gray = cv.medianBlur(gray, 5)
        # gray = cv.GaussianBlur(gray,(7,7),2)
        cv.imshow("G", gray)

        params = cv.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 50
        params.maxThreshold = 150


        # Filter by Area.
        params.filterByArea = True
        params.minArea = 400
        params.maxArea = 900000

        
        params.filterByColor = True
        params.blobColor = 255
        
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.3

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        # Create a detector with the parameters
        # OLD: detector = cv.SimpleBlobDetector(params)
        detector = cv.SimpleBlobDetector_create(params)
        
        keypoints=detector.detect(gray)
        print(keypoints)
        
        im_with_keypoints = cv.drawKeypoints(gray, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        # Show keypoints
        cv.imshow("Keypoints", im_with_keypoints)
        
        for keyPoint in keypoints:
            x = keyPoint.pt[0]
            y = keyPoint.pt[1]
            s = keyPoint.size
            print(x,y,s)
            # time.sleep(10)

        # Our operations on the frame come here
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    PWM.setMotorModel(0,0,0,0)       #Left
    cv.destroyAllWindows()
    
PWM.setMotorModel(-0,-0,0,0)       #Left
cv.destroyAllWindows()
print("Close!")
