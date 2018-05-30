#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import random,math
import numpy as np
import os
import os.path
import tf
from spencer_tracking_msgs.msg import TrackedPersons
from sfm import SFMSensor
from nav_msgs.msg import Odometry

class tracker:
    def __init__(self):
	#self._track = False
	self._tracker_ID = 0
        self._intention = None
	self._tracker_class = None
	self._tracker_alphas = None
	self._tracker_last_detection_time = 0
	self._tracker_last_detection = None
	self._initilaized = False
        self._matched = False
        self._wait_time = 1
        self._robot_position = np.reshape([0,0],(2,1))
        self._robot_velocity = np.reshape([0,0],(2,1)) 
	self.track_people = rospy.Subscriber("/spencer/perception/tracked_persons", TrackedPersons,self.detection_callback)
	self.track_robot = rospy.Subscriber("/odom", Odometry,self.robot_callback)

    def robot_callback(self,odom):
        self._robot_position = np.reshape([odom.pose.pose.position.x,odom.pose.pose.position.y],(2,1))
        rot = odom.pose.pose.orientation
        rot = (rot.x,rot.y,rot.z,rot.w)
        euler = tf.transformations.euler_from_quaternion(rot)
        theta = euler[2]
        velocity =np.array( [math.cos(theta),math.sin(theta)])*odom.twist.twist.linear.x
        #velocity = odom.twist.twist.linear
        #velocity = tf.quatRotate(rotation,velocity)
        self._robot_velocity = np.reshape([velocity[0],velocity[1]],(2,1))
        print (self._robot_velocity)
    def detection_callback(self,detections):
            time = detections.header.stamp.secs + detections.header.stamp.nsecs * 10 **(-9)
            if len(detections.tracks) == 0:
                    return None 
            human_data = []
            for person in detections.tracks:
                    if not self._initilaized and person.is_matched == True:
                        self._tracker_ID = person.track_id
                        self._tracker_last_detection_time = time
                        self._initilaized = True
                        self._tracker_last_detection = np.reshape([person.pose.pose.position.x,person.pose.pose.position.y],(2,1))
                        self._matched = True
                        self._intention = SFMSensor(self._tracker_last_detection) 
                    elif self._tracker_ID == person.track_id:
                        if person.is_matched == True: 
                            self._matched = True
                            self._tracker_last_detection_time = time
                            self._tracker_last_detection = np.reshape([person.pose.pose.position.x,person.pose.pose.position.y],(2,1))
                        elif person.is_matched == False and time > self._tracker_last_detection_time + self._wait_time:
                            self._initilaized = False
                            self._matched = False
                        else:
                            self._tracker_last_detection = np.reshape([person.pose.pose.position.x,person.pose.pose.position.y],(2,1))
                            self._matched = False

                    else:
                        human_data.append(np.reshape([person.pose.pose.position.x,person.pose.pose.position.y],(2,1)))
            return human_data	

    def intentions(self):
        s


def main(args):
    rospy.init_node('Intentions_node', anonymous=True)
    intentions = tracker()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rospy.sleep(1)

if __name__ == '__main__':
    main(sys.argv)

