#!/usr/bin/env python
# -*- conding:utf8 -*-

from numpy import array

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg      import Odometry
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
            "robot":{"x":0,"y":0},
            "woman":{"x":0,"y":0},
            "enemy":{"x":0,"y":0},
        }
        self._goal = {"x":0,"y":0}

        self._next_vel = Twist()



        r = rospy.Rate(10)

        while True:

            # self._goal["x"] = -15.0
            # self._goal["y"] =  60.0


            # self._goal["x"] = (self._positions["woman"]["x"]+self._positions["enemy"]["x"])/2
            # self._goal["y"] = (self._positions["woman"]["y"]+self._positions["enemy"]["y"])/2

            print("goal ({x:.2f},{y:.2f})".format(x=self._goal["x"],y=self._goal["y"]) )

            self._set_next_vel()
            self._pub.publish(self._next_vel)

            print("robot({x:.2f},{y:.2f})".format(x=self._positions["robot"]["x"],y=self._positions["robot"]["y"]) )
            print("woman({x:.2f},{y:.2f})".format(x=self._positions["woman"]["x"],y=self._positions["woman"]["y"]) )
            print("enemy({x:.2f},{y:.2f})".format(x=self._positions["enemy"]["x"],y=self._positions["enemy"]["y"]) )
            print("\n")

            r.sleep()

    def _robot_callback(self,data):
        self._positions["robot"]["x"] = data.pose.pose.position.x
        self._positions["robot"]["y"] = data.pose.pose.position.y

    def _pedes_callback(self,data):
        for odom in data.odoms:
            if odom.child_frame_id == "Dummy1":
                self._positions["woman"]["x"] =  odom.pose.pose.position.x
                self._positions["woman"]["y"] =  odom.pose.pose.position.y

                self._goal["x"] = self._positions["woman"]["x"]
                self._goal["y"] = self._positions["woman"]["y"]


            if odom.child_frame_id == "Scripted Human":
                self._positions["enemy"]["x"] =  odom.pose.pose.position.x
                self._positions["enemy"]["y"] =  odom.pose.pose.position.y

    def _set_next_vel(self):
        gap_x = self._goal["x"] - self._positions["robot"]["x"]
        gap_y = self._goal["y"] - self._positions["robot"]["y"]

        gap = gap_x*gap_x + gap_y*gap_y
        print("gap :({gap_x:.2f},{gap_y:.2f})={gap:.2f}".format(gap_x=gap_x,gap_y=gap_y,gap=gap))

        gap_x /= 10
        gap_y /= 10

        # if(gap_x >= 0):
        #     gap_x = max(MIN_SPEED, gap_x)
        #     gap_x = min(MAX_SPEED, gap_x)
        # else:
        #     gap_x = min(MIN_SPEED*(-1), gap_x)
        #     gap_x = max(MAX_SPEED*(-1), gap_x)
        #
        # if(gap_y >= 0):
        #     gap_y = max(MIN_SPEED, gap_y)
        #     gap_y = min(MAX_SPEED, gap_y)
        # else:
        #     gap_y = min(MIN_SPEED*(-1), gap_y)
        #     gap_y = max(MAX_SPEED*(-1), gap_y)

        if(self._val<100 || gap < NEAR_DIS) :
            gap_x = 0
            gap_y = 0

        print("val :"+self._val)
        print("vel :({x:.2f},{y:.2f})".format(x=self._next_vel.linear.x,y=self._next_vel.linear.y))

        self._next_vel.linear.x = gap_x
        self._next_vel.linear.y = gap_y
        #self._next_vel.angular.z = 0.2








if __name__ == "__main__":
    naito = Naito_node()
