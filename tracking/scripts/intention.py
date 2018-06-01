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
from ray_cast import ray 
from nav_msgs.msg import Odometry

class tracker:
    def __init__(self):
	#self._track = False
	self._tracker_ID = -100
        self._intention = None
	self._tracker_class = None
	self._tracker_alphas = None
	self._tracker_last_detection_time = 0
	self._tracker_last_detection = None
	self._initilaized = False
        self._matched = False
        self._wait_time = 1
        self._time = 0
        self._sensor = None
        self._predict_intetions = False
        self._max_dis = 3.5
        self._ray = ray(self._max_dis)
        self._robot_position     = np.reshape([0,0],(2,1))
        self._robot_old_position = np.reshape([0,0],(2,1))
        self._robot_velocity     = np.reshape([0,0],(2,1)) 
	self.track_people = rospy.Subscriber("/spencer/perception/tracked_persons", TrackedPersons,self.detection_callback)
	self.track_robot = rospy.Subscriber("/odom", Odometry,self.robot_callback)
    
    def robot_callback(self,odom):
        self._robot_position = np.reshape([odom.pose.pose.position.x,odom.pose.pose.position.y],(2,1))
        #rot = odom.pose.pose.orientation
        #rot = (rot.x,rot.y,rot.z,rot.w)
        #euler = tf.transformations.euler_from_quaternion(rot)
        #theta = euler[2]
        #velocity =np.array( [math.cos(theta),math.sin(theta)])*odom.twist.twist.linear.x
        #velocity = odom.twist.twist.linear
        #velocity = tf.quatRotate(rotation,velocity)
        #self._robot_velocity = np.reshape([velocity[0],velocity[1]],(2,1))
    def detection_callback(self,detections):
            time = detections.header.stamp.secs + detections.header.stamp.nsecs * 10 **(-9)
            if len(detections.tracks) == 0:
                    return None 
            human_data = []
            dt = time - self._time
            self._time = time
            for person in detections.tracks:
                    if not self._initilaized and person.is_matched == True:
                        self._tracker_ID = person.track_id
                        self._tracker_last_detection_time = time
                        self._initilaized = True
                        self._tracker_last_detection = np.reshape([person.pose.pose.position.x,person.pose.pose.position.y],(2,1))
                        self._matched = True
                        self._robot_old_position = self._robot_old_position
                        self._sensor = SFMSensor(self._robot_position,self._tracker_last_detection,np.reshape([0,0],(2,1)))
                        return None
                    elif self._tracker_ID == person.track_id:
                        self._robot_velocity = self._robot_position - self._robot_old_position
                        self._robot_old_position = self._robot_position
                        if person.is_matched == True: 
                            self._matched = True
                            self._tracker_last_detection_time = time
                            self._tracker_last_detection = np.reshape([person.pose.pose.position.x,person.pose.pose.position.y],(2,1))
                            self._sensor._robot_position = self._robot_position
                            self._sensor._robot_velocity = self._robot_velocity
                            self._sensor._human_position = self._tracker_last_detection
                            self._predict_intetions  = True
                        elif person.is_matched == False and time > self._tracker_last_detection_time + self._wait_time:
                            self._initilaized = False
                            self._matched = False
                            self._sensor = None
                        else:
                            self._tracker_last_detection = np.reshape([person.pose.pose.position.x,person.pose.pose.position.y],(2,1))
                            self._matched = False
                            self._sensor._robot_position = self._robot_position
                            self._sensor._robot_velocity = self._robot_velocity
                            self._sensor._human_position = self._tracker_last_detection
                            self._predict_intetions  = True

                    else:
                        human_data.append(np.reshape([person.pose.pose.position.x,person.pose.pose.position.y],(2,1)))
            if self._predict_intetions == True:
                self._predict_intetions = False
                dist,loc = self._ray.find_position2(self._tracker_last_detection)
                if dist == -1:
                    self._sensor._obs_data = {'distance':self._max_dis,'position':(0,0)}
                else:
                    self._sensor._obs_data = {'distance':dist,'position':loc}
                self._sensor._human_data = human_data
                self._sensor.learn_alphas(dt)
                print (self._sensor._alphas)
            #return human_data	


def main(args):
    rospy.init_node('Intentions_node', anonymous=True)
    intentions = tracker()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rospy.sleep(1)

if __name__ == '__main__':
    main(sys.argv)

