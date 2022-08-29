#!/usr/bin/env python
from logging.config import listen
import rospy
from ur3_unity_ros.msg import JointAngles

def callback(data):
    rospy.loginfo(data.joint_angles)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("joint_angles", JointAngles, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()