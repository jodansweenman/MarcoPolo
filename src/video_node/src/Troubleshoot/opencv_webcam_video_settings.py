# Chris Neighbor
# initial version of the webcam detector, can be used to test HSV settings, radius, etc

import cv2
#import time
import numpy as np
#from infer_imagenet import *

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
# load in the video
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT)

# Check if camera opened successfully
if (cap.isOpened() == False):
    print("Error opening video stream or file")

# writing a video file for presentation
#fourcc = cv2.VideoWriter_fourcc(*'MJPG')
#out = cv2.VideoWriter('example_track.avi', fourcc , 30.0, (640, 480),

# Read until video is completed
while cap.isOpened():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if ret == True:

        redball_detected=False

        # resize video for faster processing, add blurr to smooth image, convert to Hue saturation value
        frame = cv2.resize(frame, (640, 480))
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        frameHSV = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)


        # code for later exploring using CNNs for object detection, in this case a tennis ball
        #found = infer_result(frame, 852, model)
        #print('Tennis Ball found?:', found)

        redLow = (0, 140, 140)
        redHigh = (255, 255, 255)

        # other colors such as the green for a tennis ball
        #colorLow = (100, 40, 60)
        #colorHigh = (120, 255, 255)

        # masks the parts of the image which fits the HSV setting, fills in holes using erode/dilate
        mask = cv2.inRange(frameHSV, redLow, redHigh)
        mask = cv2.erode(mask, None, iterations=4)
        mask = cv2.dilate(mask, None, iterations=8)
        mask = cv2.erode(mask, None, iterations=4)

        # copy of the mask for checking if circle
        maskg = np.copy(mask)
        imgg = np.zeros(frame.shape[0:2])


        cv2.imshow('mask', mask)
        cnts, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        center = None
        cv2.drawContours(frame, cnts, -1, (0, 255, 0), 3)

        # Checks to make sure there is a red object
        if len(cnts) < 1:
            cv2.imshow('Frame', frame)
            #cv2.waitKey(10)

            #out.write(frame)
        else:
            c = max(cnts, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if int(M["m00"]) != 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            print('radius', radius)
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # Check to see if the object is a circle by checking mask fill of enclosing circle
                cv2.circle(imgg, center, int(radius), 255, -1)
                masked = cv2.bitwise_and(maskg.astype(np.uint8), maskg.astype(np.uint8), mask=imgg.astype(np.uint8))
                circle_fullness = np.sum(masked) / (np.pi * radius ** 2 * 255)

                if circle_fullness > 0.8:

                    redball_detected=True
                    # draw the circle and centroid on the frame,
                    cv2.circle(frame, (int(x), int(y)), int(radius),
                               (0, 0, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # if large enough of a red object is detected it sends the coordinates
        if redball_detected:
            print('center coordinates', center)
            print(type(center))
        # write to a video file
        #out.write(frame)
        # Display the resulting frame
        print('Redball detected:', redball_detected)
        cv2.imshow('Frame', frame)
        cv2.imshow("test", frameHSV)
        #cv2.waitKey(1)
        # Press Q on keyboard to  exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    # Break the loop
    else:
        break

# When everything done, release the video capture object
cap.release()
#out.release()
# Closes all the frames
cv2.destroyAllWindows()
