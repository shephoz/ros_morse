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
from   sensor_msgs.msg import LaserScan

# Const Definition
M_PI = 3.14
NEAR_ROT  = M_PI / 12
NEAR_DIS  = 0.25

SPD_ANG = M_PI / 1
SPD_LIN = 1.2

RATE = 0.3

PointO = Point()

def norm(point1,point2):
        return math.sqrt( pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) )


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
        rospy.Subscriber('/scan', LaserScan, self._scan_callback)

        self._phase = 0
        self._reached = False

        self._positions = {
            "robot":Point(),
            "woman":Point(),
            "woman_cache":None,
            "enemy_cache":None,
            "enemy":Point(),
        }

        self._woman_vel = Point()
        self._enemy_vel = Point()

        self._goal = Point()
        self._goal_dir = Point()
        self._goal_cache = Point()
        self._next_vel = Twist()

        self._turn_to_avoid = 0

        r = rospy.Rate(30)

        while not rospy.is_shutdown():

            self._set_next_vel()
            self._vel_pub.publish(self._next_vel)

            if(False):
                print("goal ({x:.2f},{y:.2f})".format(x=self._goal.x,y=self._goal.y) )
                print("robot({x:.2f},{y:.2f})".format(x=self._positions["robot"].x,y=self._positions["robot"].y) )
                print("woman({x:.2f},{y:.2f})".format(x=self._positions["woman"].x,y=self._positions["woman"].y) )
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


    def _scan_callback(self,data):
        ranges = data.ranges
        range_max = data.range_max
        right = 0
        mid   = 0
        left  = 0
        width = 60
        pan   = 250

        for i in range(0,1081,15):
            # print("{:4d}".format(i)+(  "-" * int( ranges[i] * 3 )  ))
            if(i in range(540-pan-width,540-pan+width)):
                right += ranges[i]
            if(i in range(540-width,540+width)):
                mid += ranges[i]
            if(i in range(540+pan-width,540+pan+width)):
                left += ranges[i]
        piv    = (width*2.0)/15.0
        right /= piv
        mid   /= piv
        left  /= piv
        print("{:.4f}-{:.4f}-{:.4f}".format(left,mid,right))

        thre = 5.0
        rati = 1.0
        if(mid < thre):
            self._turn_to_avoid = pow(thre - mid, 2)* rati
            if(left < right):
                self._turn_to_avoid *= -1
        else:
            self._turn_to_avoid = 0


    def _pedes_callback(self,data):
        for odom in data.odoms:
            if (odom.child_frame_id in ["Dummy1", "Pedestrian_female 42"]):
                self._positions["woman"].x = odom.pose.pose.position.x
                self._positions["woman"].y = odom.pose.pose.position.y
                self._woman_pub.publish(self._make_point_pub(self._positions["woman"]))
                self._get_woman_vel()

            if (odom.child_frame_id in ["Scripted Human", "Pedestrian_male 02"]):
                self._positions["enemy"].x = odom.pose.pose.position.x
                self._positions["enemy"].y = odom.pose.pose.position.y
                self._enemy_pub.publish(self._make_point_pub(self._positions["enemy"]))

        self._set_goal()

    def _get_woman_vel(self):
        if(self._positions["woman_cache"] is None):
            self._positions["woman_cache"] = Point()
            self._positions["woman_cache"].x = self._positions["woman"].x
            self._positions["woman_cache"].y = self._positions["woman"].y

        length = norm(self._positions["woman"],self._positions["woman_cache"])
        if(length > 0.5):
            vel_x  = self._positions["woman"].x - self._positions["woman_cache"].x
            vel_y  = self._positions["woman"].y - self._positions["woman_cache"].y
            vel_x /= length
            vel_y /= length
            self._woman_vel.x = vel_x
            self._woman_vel.y = vel_y
            self._positions["woman_cache"].x = self._positions["woman"].x
            self._positions["woman_cache"].y = self._positions["woman"].y


    def _set_goal(self):
        if(norm(self._positions["woman"],self._positions["enemy"]) > 7):

            p1 = Point()
            p1.x = self._positions["woman"].x + self._woman_vel.y
            p1.y = self._positions["woman"].y - self._woman_vel.x
            p1_l = norm(self._positions["robot"],p1)

            p2 = Point()
            p2.x = self._positions["woman"].x - self._woman_vel.y
            p2.y = self._positions["woman"].y + self._woman_vel.x
            p2_l = norm(self._positions["robot"],p2)

            if(p1_l < p2_l):
                self._goal.x = p1.x
                self._goal.y = p1.y
            else:
                self._goal.x = p2.x
                self._goal.y = p2.y
            self._goal.z = math.atan2(self._woman_vel.y,self._woman_vel.x)

        else:
            self._goal.x = self._positions["woman"].x * (1-RATE) + self._positions["enemy"].x * RATE
            self._goal.y = self._positions["woman"].y * (1-RATE) + self._positions["enemy"].y * RATE
            self._goal.z = math.atan2(
                self._positions["enemy"].y - self._positions["woman"].y,
                self._positions["enemy"].x - self._positions["woman"].x
            )

        # check goal was updated
        if norm(self._goal,self._goal_cache) > 0.1:
            self._goal_cache.x = self._goal.x
            self._goal_cache.y = self._goal.y
            self._goal_pub.publish(self._make_point_pub(self._goal))
            self._reached = False



    def _attitude(self,goal_z,turn_to_avoid):
        gap_a = goal_z - self._positions["robot"].z

        back_enabled = False
        if(not back_enabled):
            flag = 1
        else:
            if(M_PI * 0.5 <= gap_a < M_PI * 1.5):
                gap_a = gap_a - M_PI
                flag = -1
            elif(M_PI * -1.5 <= gap_a < M_PI * -0.5):
                gap_a = gap_a + M_PI
                flag = -1
            else:
                flag = 1

        if(M_PI < gap_a):
            gap_a = -2 * M_PI + gap_a
        if(gap_a < -1 * M_PI):
            gap_a =  2 * M_PI - gap_a

        if(abs(gap_a) > NEAR_ROT * 2):
            flag = 0

        if(gap_a > NEAR_ROT):
            self._next_vel.angular.z = SPD_ANG
        elif(gap_a < -1*NEAR_ROT):
            self._next_vel.angular.z = SPD_ANG*(-1)
        else:
            self._next_vel.angular.z = 0.0

        if(turn_to_avoid):
            self._next_vel.angular.z += SPD_ANG * self._turn_to_avoid

        return flag


    def _set_next_vel(self):
        to_x = self._goal.x - self._positions["robot"].x
        to_y = self._goal.y - self._positions["robot"].y
        gap = norm(self._goal, self._positions["robot"])

        if(self._reached):
            self._next_vel.linear.x = 0.0
            self._attitude(self._goal.z,False)
        else:
            self._next_vel.linear.x = self._attitude(math.atan2(to_y,to_x), True) * SPD_LIN
            if(gap < NEAR_DIS) :
                self._reached = True










if __name__ == "__main__":
    #help(tf.transformations)
    naito = Naito_node()
