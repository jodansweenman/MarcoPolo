#!/usr/bin/env python

import rospy
import numpy as np
import time
from std_msgs.msg import Float64
from std_msgs.msg import Bool

bicep = rospy.Publisher('/arm_bicep_joint/command', Float64, queue_size=10)
forearm = rospy.Publisher('/arm_forearm_joint/command', Float64, queue_size=10)
shoulder = rospy.Publisher('/arm_shoulder_pan_joint/command', Float64, queue_size=10)
wrist = rospy.Publisher('/arm_wrist_flex_joint/command', Float64, queue_size=10)
gripper = rospy.Publisher('gripper_joint/command', Float64, queue_size=10)

b_val = np.array([-1.57, -1.0, -1.0, -0.5, 0.0, -0.5, -1.0, -1.0, -1.57])
f_val = np.array([-1.57, -1.0, -0.5, -0.1, 0.0, -0.1, -0.5, -1.0, -1.57])
s_val = np.array([1.57, 1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 1.57, 1.57])
w_val = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
g_val = np.array([0.0, 0.0, 0.0, 0.0, -0.9, -0.9, -0.9, -0.9, -0.9])

count = 0

def arm_callback(data):

    global count

    if data.data == True:
        for i in range(len(b_val)):
            count += 1
            bicep.publish(Float64(b_val[i]))
            forearm.publish(Float64(f_val[i]))
            shoulder.publish(Float64(s_val[i]))
            wrist.publish(Float64(w_val[i]))
            gripper.publish(Float64(g_val[i]))
            time.sleep(1.5)


def arm_control():
    rospy.init_node('arm_control', anonymous=True)
    rospy.Subscriber('/move_goal_true', Bool, arm_callback)
    
    #Initial Position
    bicep.publish(Float64(b_val[count]))
    forearm.publish(Float64(f_val[count]))
    shoulder.publish(Float64(s_val[count]))
    wrist.publish(Float64(w_val[count]))
    gripper.publish(Float64(g_val[count]))
    rospy.spin()

if __name__ == '__main__':
    try:
        arm_control()
    except rospy.ROSInterruptException:
        pass