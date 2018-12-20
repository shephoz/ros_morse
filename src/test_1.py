#!/usr/bin/env python
# -*- conding:utf8 -*-

from vector  import Vector
from pose    import Pose
from subject import Subject

import math
import rospy
import tf

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import PoseWithCovarianceStamped
from      nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from  morse_helper.msg import PedestrianData
from   sensor_msgs.msg import LaserScan

# Const Definition
R_PRIVACY = 1.0
VEL_R_MAX = 2.0

NEAR_ROT  = math.pi / 12
NEAR_DIS  = 0.25

SPD_ANG = math.pi / 1
SPD_LIN = 1.2

RATE = 0.3

class Naito_node:

    def __init__(self):
        rospy.init_node('naito', anonymous=True)
        self._vel_pub = rospy.Publisher('/xbot/cmd_vel', Twist, queue_size=1)

        self._goal_pub = rospy.Publisher('/naito/goal',  PoseStamped, queue_size=10)
        self.human_pub = rospy.Publisher('/naito/human', PoseStamped, queue_size=10)
        self.enemy_pub = rospy.Publisher('/naito/enemy', PoseStamped, queue_size=10)

        if(False):
            rospy.Subscriber('/pose_6d',   PoseWithCovarianceStamped, self._robot_callback)
        else:
            rospy.Subscriber('/xbot/odom', Odometry,                  self._robot_callback)

        rospy.Subscriber('/pedestrians', PedestrianData, self._pedes_callback)
        rospy.Subscriber('/scan',        LaserScan,      self._scan_callback)

        self._phase = 0
        self._reached = False

        self.robot = Subject()
        self.human = Subject()
        self.enemy = Subject()

        self.goal = Pose()

        self._goal_dir = Point()
        self._goal_cache = Point()
        self._next_vel = Twist()

        self._turn_to_avoid = 0

        r = rospy.Rate(30)

        while not rospy.is_shutdown():

            self._set_next_vel()
            self._vel_pub.publish(self._next_vel)

            if(False):
                print( "goal ({x:.2f},{y:.2f})".format(x=self.goal.x,      y=self.goal.y     ) )
                print( "robot({x:.2f},{y:.2f})".format(x=self.robot.pose.x, y=self.robot.pose.y) )
                print( "human({x:.2f},{y:.2f})".format(x=self.human.pose.x, y=self.human.pose.y) )
                print( "enemy({x:.2f},{y:.2f})".format(x=self.enemy.pose.x, y=self.enemy.pose.y) )
                print("\n")

            try:
                r.sleep()
            except:
                print("time error!")


    def _get_vel_robot_des(self):
        W_robot = 0.9
        r_protect = W_robot * R_PRIVACY * VEL_R_MAX / max(abs(self.enemy.vel.para - self.human.vel.para), VEL_R_MAX)

        T_robot   = Pose()
        T_robot.vec = self.human.path_to(self.enemy)
        vel_robot_des = W_robot * ( abs(self.enemy.vel.perp - self.human.vel.perp) + 1 ) * (T_robot - P_robot)




    # def _make_point_pub(self,point):
    #     goal = PoseStamped()
    #     goal.header.frame_id = "map"
    #     goal.pose.position.x  = point.x
    #     goal.pose.position.y  = point.y
    #     quaternion = tf.transformations.quaternion_from_euler(0,0,point.z)
    #     goal.pose.orientation.z = quaternion[2]
    #     goal.pose.orientation.w = quaternion[3]
    #     # print(goal)
    #     return goal

    def _robot_callback(self,data):
        self.robot.update_pose(data.pose.pose)

    def _pedes_callback(self,data):
        for odom in data.odoms:
            if (odom.child_frame_id in ["Dummy1", "Pedestrian_female 42"]):
                self.human.update_pose(odom.pose.pose)
                # self.human_pub.publish(self._make_point_pub(self.human.pos.position))
            if (odom.child_frame_id in ["Scripted Human", "Pedestrian_male 02"]):
                self.enemy.update_pose(odom.pose.pose)
                # self.enemy_pub.publish(self._make_point_pub(self.human.pos.position))
            self._set_goal()


    def _scan_callback(self,data):
        ranges    = data.ranges
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


    def _set_goal(self):
        if(self.human.gap_to(self.enemy) > 7):

            p1 = Pose(
                self.human.pose.x + self.human.vel.y,
                self.human.pose.y - self.human.vel.x,
                self.human.vel.tan()
            )
            p1_length = p1.gap_to(self.robot.pose)

            p2 = Pose(
                self.human.pose.x - self.human.vel.y,
                self.human.pose.y + self.human.vel.x,
                self.human.vel.tan()
            )
            p2_length = p2.gap_to(self.robot.pose)
            self.goal = p1 if p1_length < p2_length else p2

        else:
            self.goal = Pose(
                self.human.pose.x * (1-RATE) + self.enemy.pose.x * RATE,
                self.human.pose.y * (1-RATE) + self.enemy.pose.y * RATE,
                self.human.path_to(self.enemy).tan()
            )

        # check goal was updated
        if self.goal.is_moved(self.goal_cache, 0.1):
            self.goal_cache.set(self.goal)
            # self.goal_pub.publish(self._make_point_pub(self._goal))
            self._reached = False



    def _attitude(self,goal_yaw,turn_to_avoid):
        gap_a = goal_yaw - self.robot.pose.yaw

        if(math.pi < gap_a):
            gap_a = -2 * math.pi + gap_a
        if(gap_a < -1 * math.pi):
            gap_a =  2 * math.pi - gap_a

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


    def _set_next_vel(self):
        to_x = self._goal.x - self.robot.pose.x
        to_y = self._goal.y - self.robot.pose.y
        gap = norm(self._goal, self.robot.pos)

        if(self._reached):
            self._next_vel.linear.x = 0.0
            self._attitude(self._goal.z,False)
        else:
            self._attitude(math.atan2(to_y,to_x), True)
            self._next_vel.linear.x = SPD_LIN
            if(gap < NEAR_DIS) :
                self._reached = True






if __name__ == "__main__":
    #help(tf.transformations)
    naito = Naito_node()
