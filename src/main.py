#!/usr/bin/env python
# -*- conding:utf8 -*-

from vector  import Vector
from pose    import Pose
from subject import Subject

import math
import rospy
import tf

# from geometry_msgs.msg import Point
# from geometry_msgs.msg import Quaternion

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from      nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from  morse_helper.msg import PedestrianData
from   sensor_msgs.msg import LaserScan

# Const Definition
#R_PRIVACY = 1.0
VEL_R_MAX = 1.5
W_ROBOT = 0.9
r_protect=0.9

r_social =1.0

NEAR_ROT  = math.pi / 12
NEAR_DIS  = 0.25

SPD_ANG = math.pi / 1
SPD_LIN = 1.2

RATE = 0.3

class Naito_node:

    def __init__(self):
        self.vel_pub   = rospy.Publisher('/xbot/cmd_vel', Twist, queue_size=1)
        # self.goal_pub  = rospy.Publisher('/naito/goal',   PoseStamped, queue_size=10)
        # self.human_pub = rospy.Publisher('/naito/human',  PoseStamped, queue_size=10)
        # self.enemy_pub = rospy.Publisher('/naito/enemy',  PoseStamped, queue_size=10)

        if(False):
            rospy.Subscriber('/pose_6d',   PoseWithCovarianceStamped, self.robot_callback)
        else:
            rospy.Subscriber('/xbot/odom', Odometry,                  self.robot_callback)

        rospy.Subscriber('/pedestrians', PedestrianData, self.pedes_callback)
        rospy.Subscriber('/scan',        LaserScan,      self.scan_callback)

        self.robot = Subject()
        self.human = Subject()
        self.enemy = Subject()

        self.goal       = Pose()
        self.goal_cache = Pose()
        self.reached       = False
        self.turn_to_avoid = 0

        self.next_vel = Twist()


    # def make_point_pub(self,point):
    #     goal = PoseStamped()
    #     goal.header.frame_id = "map"
    #     goal.pose.position.x  = point.x
    #     goal.pose.position.y  = point.y
    #     quaternion = tf.transformations.quaternion_from_euler(0,0,point.z)
    #     goal.pose.orientation.z = quaternion[2]
    #     goal.pose.orientation.w = quaternion[3]
    #     # print(goal)
    #     return goal

    def robot_callback(self,data):
        self.robot.update_pose(data.pose.pose)

    def pedes_callback(self,data):
        for odom in data.odoms:
            if (odom.child_frame_id in ["Dummy1", "Pedestrian_female 42"]):
                self.human.update_pose(odom.pose.pose)
                # self.human_pub.publish(self._make_point_pub(self.human.pos.position))
            if (odom.child_frame_id in ["Scripted Human", "Pedestrian_male 02"]):
                self.enemy.update_pose(odom.pose.pose)
                # self.enemy_pub.publish(self._make_point_pub(self.human.pos.position))
        self.set_goal()


    def scan_callback(self,data):
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
        # print("{:.4f}-{:.4f}-{:.4f}".format(left,mid,right))

        thre = 5.0
        rati = 1.0
        if(mid < thre):
            self.turn_to_avoid = pow(thre - mid, 2)* rati
            if(left < right):
                self.turn_to_avoid *= -1
        else:
            self.turn_to_avoid = 0


    def set_goal(self):
        if(False and self.human.gap_to(self.enemy) > 7):
            # normal mode : no enemy
            p1 = Pose(
                self.human.pose.x + self.human.vel.y,
                self.human.pose.y - self.human.vel.x,
                self.human.vel.rad
            )
            p2 = Pose(
                self.human.pose.x - self.human.vel.y,
                self.human.pose.y + self.human.vel.x,
                self.human.vel.rad
            )
            closer_check = p1.gap_to(self.robot.pose) < p2.gap_to(self.robot.pose)
            self.goal = p1 if closer_check else p2

        else:
            # protect mode : encounter the enemy
            r_protect = W_ROBOT * R_PRIVACY * (VEL_R_MAX / max(abs(self.enemy.para(self.human) - self.human.para(self.enemy)), VEL_R_MAX))

            T_robot = Pose()
            T_robot.vec = self.human.path_to(self.enemy).normalize()
            T_robot.vec = T_robot.vec.scalar_prod(r_protect)
            T_robot.vec = T_robot.vec.plus(self.human.pose.vec)
            T_robot.yaw = self.robot.path_to(self.enemy).rad
            self.goal = T_robot

        # check goal was updated
        if self.goal.is_moved(self.goal_cache, 0.1):
            self.goal_cache.set(self.goal)
            # self.goal_pub.publish(self.make_point_pub(self.goal))
            self.reached = False



    def attitude(self,
        goal_yaw,
        turn_to_avoid
    ):
        gap_a = goal_yaw - self.robot.pose.yaw

        # normalize gap_a to be included by [-PI , PI]
        if(math.pi < gap_a):
            gap_a = -2 * math.pi + gap_a
        if(gap_a < -1 * math.pi):
            gap_a =  2 * math.pi - gap_a

        # returns sign(+/-) of linear velocity for walk_backward("sushiro-aruki")
        walk_backward = 1 if(abs(gap_a) < math.pi - NEAR_ROT) else -1

        if(gap_a > NEAR_ROT):
            self.next_vel.angular.z = SPD_ANG
        elif(gap_a < NEAR_ROT*-1):
            self.next_vel.angular.z = SPD_ANG*(-1)
        else:
            self.next_vel.angular.z = 0.0

        if(abs(gap_a) > NEAR_ROT and turn_to_avoid):
            weight = 1.2
            self.next_vel.angular.z += SPD_ANG * weight * self.turn_to_avoid



    def set_next_vel(self):
        v_r_des = self.robot.pose.vec.path_to(self.goal.vec)
        v_r_des = v_r_des.scalar_prod( W_ROBOT * (abs(self.enemy.perp(self.human) - self.human.perp(self.enemy)) + 1) )

        if(self.reached):
            self.next_vel.linear.x = 0.0
            self.attitude(self.goal.yaw,False)
        else:
            self.attitude(v_r_des.rad, False) #True)
            self.next_vel.linear.x = v_r_des.norm()
            if(v_r_des.norm() < NEAR_DIS) :
                self.reached = True
            else:
                print("not reached : {} : {}".format(v_r_des,v_r_des.norm()))






if __name__ == "__main__":
    #help(tf.transformations)
    rospy.init_node('naito', anonymous=True)
    naito = Naito_node()
    r = rospy.Rate(30)

    while not rospy.is_shutdown():
        if(
            not self.robot.cache is None
            and not self.human.cache is None
            and not self.enemy.cache is None
        ):
            self.set_next_vel()
            self.vel_pub.publish(self.next_vel)
        if(True):
            print( "goal {}".format(self.goal ) )
            print( "robot{}".format(self.robot) )
            print( "human{}".format(self.human) )
            print( "enemy{}".format(self.enemy) )
            print("\n")
        try:
            r.sleep()
        except:
            print("time error!")
