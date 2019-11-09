# Chris Neighbor

import cv2
import numpy as np

def setup_video(height=480, width=640, video_path=0):
    """
    resolution of video
    :param video_path: 0 for default webcam, can also use video file
    returns VideoCapture object
    """
    FRAME_WIDTH = width
    FRAME_HEIGHT = height
    # load in the video
    cap = cv2.VideoCapture(video_path)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT)

    # Check if camera opened successfully
    if (cap.isOpened() == False):
        print("Error opening video stream or file")

    return cap

def red_ball_detection(frame):
    """
    function for publishing to the ROS node
    :param frame: opencv image which should be received by the as a BGR numpy array
    :return: int16 array with format [redball_detected(0 or 1), x_coordinate, y_coordinate]
    """
    redball_detected = 0

    # resize video for faster processing, add blurr to smooth image, convert to Hue saturation value
    frame = cv2.resize(frame, (640, 480))
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    frameHSV = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Hue saturation values I found worked best, may need adjustments in different lighting
    #cv2.imshow('HSV', frameHSV)
    #cv2.waitKey(25)
    redLow = (0, 170, 160)
    redHigh = (15, 255, 255)

    # masks the parts of the image which fits the HSV setting, fills in holes using erode/dilate
    mask = cv2.inRange(frameHSV, redLow, redHigh)
    mask = cv2.erode(mask, None, iterations=4)
    mask = cv2.dilate(mask, None, iterations=8)
    mask = cv2.erode(mask, None, iterations=4)

    # stores some images for use in determining if mask is a circle
    maskg = np.copy(mask)
    imgg = np.zeros(frame.shape[0:2])

    # Finds the countours of the mask, contours are the outline of the same value
    _, cnts, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    center = None

    default_return = np.array([0,0,0], dtype='int16')
    #cv2.imshow('frame', frame)
    #cv2.waitKey(25)
    
    #cv2.imshow('mask',mask)
    #cv2.waitKey(25)

    # no mask found at all
    if len(cnts) < 1:
        return default_return
    else:
        # default center
        center = (0,0)
        # find largest masked object
        c = max(cnts, key=cv2.contourArea)

        # encircle the masked object and calculate the center
        (x, y), radius = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if int(M["m00"]) != 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # make sure the object is large enough, can adjust this
        if radius > 30:
            # Check to see if the object is a circle by checking mask fill of enclosing circle
            cv2.circle(imgg, center, int(radius), 255, -1)
            masked = cv2.bitwise_and(maskg.astype(np.uint8), maskg.astype(np.uint8), mask=imgg.astype(np.uint8))
            circle_fullness = np.sum(masked) / (np.pi * radius ** 2 * 255)

            # check if the circle is 80% filled with the mask as threshold
            if circle_fullness > 0.6:
                redball_detected = 1
        if not redball_detected:
            return default_return
        else:
            return np.array([redball_detected, center[0],center[1]], dtype='int16')



if __name__ == '__main__':
    cap = setup_video()
    # use Ctrl+C to end loop
    try:
        while cap.isOpened():
            # Capture frame-by-frame
            ret, frame = cap.read()
            if ret:
                published_coordinates=red_ball_detection(frame)
                print('Published array of coordinates', published_coordinates)
            else:
                break
    except KeyboardInterrupt:
        pass
    # release the video capture object back into the wild from whence it came
    cap.release()
