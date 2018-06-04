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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class tracker:
	def __init__(self):
	#self._track = False
		self._tracker_ID = -100
		self._tracker_last_detection_time = 0
		self._tracker_last_detection = None
		self._initilaized = False
		self._matched = False
		self._wait_time = 1
		self._time = 0
		self._robot_vel = 0.3
		self._min_angle = 20
		self._angular_ratio = 0.3
		self._robot_position     = np.reshape([0,0],(2,1))
		self.track_people = rospy.Subscriber("/spencer/perception/tracked_persons", TrackedPersons,self.detection_callback)
		self.track_robot = rospy.Subscriber("/odom", Odometry,self.robot_callback)
		self.pub_velocity = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
	def robot_callback(self,odom):
		self._robot_position = np.reshape([odom.pose.pose.position.x,odom.pose.pose.position.y],(2,1))
	def detection_callback(self,detections):
            time = detections.header.stamp.secs + detections.header.stamp.nsecs * 10 **(-9)
            if len(detections.tracks) == 0:
                    return None 
            dt = time - self._time
            self._time = time
            for person in detections.tracks:
                    if not self._initilaized and person.is_matched == True:
                        self._tracker_ID = person.track_id
                        self._tracker_last_detection_time = time
                        self._initilaized = True
                        self._tracker_last_detection = np.reshape([person.pose.pose.position.x,person.pose.pose.position.y],(2,1))
                        self._matched = True
                        return None
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
			return None
	def follow(self):
		vel = Twist()
		vel.linear.x = 0
		vel.angular.z = 0
		if self._initilaized == True:
			vel.linear.x = self._robot_vel
			diff = self._tracker_last_detection - self._robot_position
			angle = math.atan2(diff[1],diff[0])*180/math.pi
			angle = 0 if angle < self._min_angle else angle
			vel.angular.z = self._angular_ratio * angle
			print (vel)
		self.pub_velocity.publish(vel)

def main(args):
    rospy.init_node('Intentions_node', anonymous=True)
    follower = tracker()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
		follower.follow()
		rospy.sleep(1)

if __name__ == '__main__':
    main(sys.argv)

