#!/usr/bin/env python
# -*- conding:utf8 -*-

from numpy import array
import math

import rospy
import tf

from geometry_msgs.msg import PoseWithCovarianceStamped
from      nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from  morse_helper.msg import PedestrianData

M_PI = 3.14
NEAR_ROT  = M_PI / 12
NEAR_DIS  = 1.0
MAX_SPEED = 2.0
MIN_SPEED = 0.1

class Naito_node:

    def __init__(self):
        rospy.init_node('naito',anonymous=True)
        self._pub = rospy.Publisher('/xbot/cmd_vel', Twist, queue_size=1)
        if(False):
            rospy.Subscriber('/pose_6d',  PoseWithCovarianceStamped, self._robot_callback)
        else:
            rospy.Subscriber('/xbot/odom',  Odometry, self._robot_callback)
        rospy.Subscriber('/pedestrians', PedestrianData, self._pedes_callback)

        self._val = 0
        self._positions = {
            "robot":{"x":0,"y":0,"a":0},
            "woman":{"x":0,"y":0},
            "enemy":{"x":0,"y":0},
        }
        self._goal = {"x":0,"y":0,"a":0}

        self._next_vel = Twist()

        r = rospy.Rate(10)

        while True:

            # self._goal["x"] = -15.0
            # self._goal["y"] =  60.0

            # self._goal["x"] = (self._positions["woman"]["x"]+self._positions["enemy"]["x"])/2
            # self._goal["y"] = (self._positions["woman"]["y"]+self._positions["enemy"]["y"])/2

            print("goal ({x:.2f},{y:.2f},{a:.2f})".format(x=self._goal["x"],y=self._goal["y"],a=self._goal["a"]) )

            self._set_next_vel()
            self._pub.publish(self._next_vel)

            print("robot({x:.2f},{y:.2f},{a:.2f})".format(x=self._positions["robot"]["x"],y=self._positions["robot"]["y"], a=self._positions["robot"]["a"]) )
            # print("woman({x:.2f},{y:.2f})".format(x=self._positions["woman"]["x"],y=self._positions["woman"]["y"]) )
            # print("enemy({x:.2f},{y:.2f})".format(x=self._positions["enemy"]["x"],y=self._positions["enemy"]["y"]) )
            print("\n")

            r.sleep()

    def _robot_callback(self,data):
        self._positions["robot"]["x"] = data.pose.pose.position.x
        self._positions["robot"]["y"] = data.pose.pose.position.y

        euler = tf.transformations.euler_from_quaternion((
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ))
        self._positions["robot"]["a"] = euler[2]

    def _pedes_callback(self,data):
        for odom in data.odoms:
            if (odom.child_frame_id == "Dummy1" or odom.child_frame_id == "Pedestrian_female 42"):
                self._positions["woman"]["x"] = odom.pose.pose.position.x
                self._positions["woman"]["y"] = odom.pose.pose.position.y
            if (odom.child_frame_id == "Scripted Human" or odom.child_frame_id == "Pedestrian_male 02"):
                self._positions["enemy"]["x"] = odom.pose.pose.position.x
                self._positions["enemy"]["y"] = odom.pose.pose.position.y
        self._set_goal()

    def _set_goal(self):
        self._goal["x"] = self._positions["woman"]["x"]
        self._goal["y"] = self._positions["woman"]["y"]

    def _set_next_vel(self):
        to_x = self._goal["x"] - self._positions["robot"]["x"]
        to_y = self._goal["y"] - self._positions["robot"]["y"]
        self._goal["a"] = math.atan2(to_x,to_y)
        to_a = self._goal["a"]

        gap = to_x*to_x + to_y*to_y

        print("gap :({to_x:.2f},{to_y:.2f})={gap:.2f}".format(to_x=to_x,to_y=to_y,gap=gap))

        to_x /= 10
        to_y /= 10

        # if(to_x >= 0):
        #     to_x = max(MIN_SPEED, to_x)
        #     to_x = min(MAX_SPEED, to_x)
        # else:
        #     to_x = min(MIN_SPEED*(-1), to_x)
        #     to_x = max(MAX_SPEED*(-1), to_x)
        #
        # if(to_y >= 0):
        #     to_y = max(MIN_SPEED, to_y)
        #     to_y = min(MAX_SPEED, to_y)
        # else:
        #     to_y = min(MIN_SPEED*(-1), to_y)
        #     to_y = max(MAX_SPEED*(-1), to_y)

        if(self._val < 30 or gap < NEAR_DIS) :
            to_x = 0
            to_y = 0
            self._val += 1
            # print("val :{val:d}".format(val=self._val))

        # print("vel :({x:.2f},{y:.2f})".format(x=to_x,y=to_y))
        # self._next_vel.linear.x = to_x
        # self._next_vel.linear.y = to_y

        gap_a = to_a - self._positions["robot"]["a"]

        if(gap_a * gap_a < 0.03):
            self._next_vel.angular.z = 0.0
        else:
            self._next_vel.angular.z = 0.4








if __name__ == "__main__":
    #help(tf.transformations)
    naito = Naito_node()
