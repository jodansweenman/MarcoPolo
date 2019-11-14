#!/usr/bin/python


#Chris Neighbor

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_processing_redball import red_ball_detection

"""
should publish to the transform node for 3D, publishes int array 
for now the publishing format will be an int array with format
[red_ball_detected(0 or 1), x_coordinates(0-640), y_coordinates(0-480)]
"""

publisher_node = 'marco/ball_uv'
# camera node for the turtlebot
subscribing_node = '/camera/rgb/image_rect_color'

class image_detector:

    def __init__(self):

        # may want to add a boolean of whether the coordinates are there or not
        self.coordinates_pub = rospy.Publisher(publisher_node, Int16MultiArray, queue_size=10)
        # could publish the mask or converted image using this publisher
        #self.image_pub = rospy.Publisher('image_topic', Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(subscribing_node, Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # checks image for red ball and returns center coordinates
        result = red_ball_detection(cv_image)
        print(result)
        try:
            # sends the int16 array with detected bool and x,y coordinates
            self.coordinates_pub.publish(Int16MultiArray(data=result))
            # displaying an image back
            #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    rospy.init_node('image_detector', anonymous=True)
    id = image_detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
