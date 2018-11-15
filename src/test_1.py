#!/usr/bin/env python
# -*- conding:utf8 -*-

from numpy import array
import math

import rospy
import tf

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import PoseWithCovarianceStamped
from      nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from  morse_helper.msg import PedestrianData

M_PI = 3.14
NEAR_ROT  = M_PI / 12
NEAR_DIS  = 0.4

SPD_ANG = M_PI / 1
SPD_LIN = 0.8

RATE = 0.3

class Naito_node:

    def __init__(self):
        rospy.init_node('naito',anonymous=True)
        self._vel_pub = rospy.Publisher('/xbot/cmd_vel', Twist, queue_size=1)
        
        self._goal_pub  = rospy.Publisher('/naito/goal',  PoseStamped, queue_size=10)
        self._woman_pub = rospy.Publisher('/naito/woman', PoseStamped, queue_size=10)
        self._enemy_pub = rospy.Publisher('/naito/enemy', PoseStamped, queue_size=10)
        
        if(False):
            rospy.Subscriber('/pose_6d', PoseWithCovarianceStamped, self._robot_callback)
        else:
            rospy.Subscriber('/xbot/odom', Odometry, self._robot_callback)
        
        rospy.Subscriber('/pedestrians', PedestrianData, self._pedes_callback)

        self._phase = 0
        self._reached = False
        
        self._positions = {
            "robot":Point(),
            "woman":Point(),
            "enemy":Point(),
        }
        
        self._goal = Point()
        self._goal_cache = Point()
        self._next_vel = Twist()

        r = rospy.Rate(30)

        while not rospy.is_shutdown():

            self._set_next_vel()
            self._vel_pub.publish(self._next_vel)
            
            # print("robot({a:.2f})".format(a=self._positions["robot"].z))
            # print("robot({x:.2f},{y:.2f},{a:.2f})".format(x=self._positions["robot"].x,y=self._positions["robot"].y, a=self._positions["robot"].z) )
            # print("woman({x:.2f},{y:.2f})".format(x=self._positions["woman"].x,y=self._positions["woman"].y) )
            # print("enemy({x:.2f},{y:.2f})".format(x=self._positions["enemy"].x,y=self._positions["enemy"].y) )
            # print("\n")
            
            try:
                r.sleep()
            except:
                print("time error!")

    def _make_point_pub(self,point):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x  = point.x
        goal.pose.position.y  = point.y
        quaternion = tf.transformations.quaternion_from_euler(0,0,point.z)
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        # print(goal)
        return goal

    def _robot_callback(self,data):
        self._positions["robot"].x = data.pose.pose.position.x
        self._positions["robot"].y = data.pose.pose.position.y

        euler = tf.transformations.euler_from_quaternion((
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ))
        self._positions["robot"].z = euler[2]


    def _pedes_callback(self,data):
        for odom in data.odoms:
            if (odom.child_frame_id in ["Dummy1", "Pedestrian_female 42"]):
                self._positions["woman"].x = odom.pose.pose.position.x
                self._positions["woman"].y = odom.pose.pose.position.y
                self._woman_pub.publish(self._make_point_pub(self._positions["woman"]))
                
            if (odom.child_frame_id in ["Scripted Human", "Pedestrian_male 02"]):
                self._positions["enemy"].x = odom.pose.pose.position.x
                self._positions["enemy"].y = odom.pose.pose.position.y
                self._enemy_pub.publish(self._make_point_pub(self._positions["enemy"]))
            
        self._set_goal()

    def _distance(self,point1,point2):
        return math.sqrt(pow(point1.x - point2.x, 2) +pow(point1.y - point2.y, 2)) 

    def _set_goal(self):
        if(self._distance(self._positions["woman"],self._positions["enemy"]) > 7):  
            self._goal.x = self._positions["woman"].x
            self._goal.y = self._positions["woman"].y
        else:
            self._goal.x = self._positions["woman"].x * (1-RATE) + self._positions["enemy"].x * RATE
            self._goal.y = self._positions["woman"].y * (1-RATE) + self._positions["enemy"].y * RATE
        
        # check goal was updated
        if self._distance(self._goal,self._goal_cache) > 0.1:
            self._goal_cache.x = self._goal.x
            self._goal_cache.y = self._goal.y
            self._goal_pub.publish(self._make_point_pub(self._goal))
            self._reached = False
        


    def _attitude(self,x,y):
        gap_a = math.atan2(y,x) - self._positions["robot"].z
    
        if(gap_a > M_PI):
            gap_a = -2*M_PI + gap_a
        if(gap_a < -1*M_PI):
            gap_a =  2*M_PI - gap_a

        if(gap_a > NEAR_ROT*div):
            self._next_vel.angular.z = SPD_ANG
            return False
        elif(gap_a < -1*NEAR_ROT*div):
            self._next_vel.angular.z = SPD_ANG*(-1)
            return False
        else:
            self._next_vel.angular.z = 0.0
            return True
            

    def _set_next_vel(self):
        to_x = self._goal.x - self._positions["robot"].x
        to_y = self._goal.y - self._positions["robot"].y
        gap = self._distance(self._goal, self._positions["robot"])
        
        if(self._reached):
            self._next_vel.linear.x = 0.0
            self._attitude(
                self._positions["enemy"].x - self._positions["woman"].x,
                self._positions["enemy"].y - self._positions["woman"].y
            )
        else:
            if(self._attitude(to_x,to_y)):
                self._next_vel.linear.x = SPD_LIN
                if(gap < NEAR_DIS) :
                    self._reached = True
            
        
            







if __name__ == "__main__":
    #help(tf.transformations)
    naito = Naito_node()
