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

from std_msgs.msg import String

# Const Definition
VEL_R_MAX = 1.5
W_ROBOT   = 0.9
R_PROTECT = 0.9
R_SOCIAL  = 1.0

AVOIDING_SLOW_DOWN_RATE = 0.5

NEAR_ROT  = math.pi / 12 # [rad]
NEAR_DIS  = 0.1 # [m]

SPD_ANG = math.pi / 1
SPD_LIN = 1.2

RATE = 0.3

class Naito_node:

    def __init__(self):
        self.vel_pub   = rospy.Publisher('/xbot/cmd_vel', Twist, queue_size=1)
        self.rml_pub   = rospy.Publisher('/naito/rml', String, queue_size=1)
        # self.goal_pub  = rospy.Publisher('/naito/goal',   PoseStamped, queue_size=10)
        # self.human_pub = rospy.Publisher('/naito/human',  PoseStamped, queue_size=10)
        # self.enemy_pub = rospy.Publisher('/naito/enemy',  PoseStamped, queue_size=10)

        #rospy.Subscriber('/pose_6d',   PoseWithCovarianceStamped, self.robot_callback)
        rospy.Subscriber('/xbot/odom',   Odometry,       self.robot_callback)
        rospy.Subscriber('/pedestrians', PedestrianData, self.pedestrian_callback)
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

    def pedestrian_callback(self,data):
        for odom in data.odoms:
            if (odom.child_frame_id in ["Dummy1", "Pedestrian_female 42"]):
                self.human.update_pose(odom.pose.pose)
                # self.human_pub.publish(self._make_point_pub(self.human.pos.position))
            if (odom.child_frame_id in ["Scripted Human", "Pedestrian_male 02"]):
                self.enemy.update_pose(odom.pose.pose)
                # self.enemy_pub.publish(self._make_point_pub(self.human.pos.position))
        self.check_protect_mode()
        self.set_goal()


    def scan_callback(self,data):
        # from scan message
        ranges    = data.ranges
        range_max = data.range_max
        # let : amount of obstacle-index about each direction
        right = 0
        mid   = 0
        left  = 0
        # const : define width & pan of left & right
        width = 60
        pan   = 250

        for i in range(0,1081,15):
            # print("{:4d}".format(i)+(  "-" * int( ranges[i] * 3 )  ))
            # if(i in range(540-pan-width,540-pan+width)):
            #     right += ranges[i]
            # if(i in range(540-width,540+width)):
            #     mid += ranges[i]
            # if(i in range(540+pan-width,540+pan+width)):
            #     left += ranges[i]
            pan = 540
            right += ranges[i] * max((1 - pow(i/pan,2)), 0)
            mid   += ranges[i] * (1 - pow(i/540 - 1,2))
            left  += ranges[i] * max((1 - pow((1080-i)/pan,2)), 0)

        #naito.rml_pub.publish("{:.4f}  {:.4f}  {:.4f}".format(left,mid,right))

        # const : define threshold & ratio to turn_to_avoid
        thre = 600
        rati = 0.01

        if(mid < thre): # if in front of robot is obstacled,
            self.turn_to_avoid = (thre - mid) * rati
            if(left < right):
                self.turn_to_avoid *= -1
        else:
            self.turn_to_avoid = 0

        naito.rml_pub.publish("{:.4f}  {:.4f}".format(mid,self.turn_to_avoid))

    def check_protect_mode(self):
        self.protect_mode = self.human.gap_to(self.enemy) < 7.0
        #self.protect_mode = False

    def set_goal(self):
        if(not self.protect_mode):
            # normal mode : walk aside the partner
            if (self.human.vel.norm() == 0):
                aside_vec = self.human.path_to(self.robot).normalize() * R_SOCIAL
                goal = Pose(self.human.pose.vec + aside_vec, self.human.path_from(self.robot).rad)
                self.goal = goal
            else:
                aside_vec = self.human.vel.rotate(math.pi/2).normalize() * R_SOCIAL
                p1 = Pose(self.human.pose.vec + aside_vec, self.human.vel.rad)
                p2 = Pose(self.human.pose.vec - aside_vec, self.human.vel.rad)
                closer_check = p1.gap_to(self.robot.pose) < p2.gap_to(self.robot.pose)
                self.goal = p1 if closer_check else p2
        else:
            # protect mode : encounter the enemy
            d_protect = R_PROTECT * (VEL_R_MAX / max(abs(self.enemy.para(self.human) - self.human.para(self.enemy)), VEL_R_MAX))
            self.goal = Pose(
                (self.human.path_to(self.enemy).normalize() * d_protect) + self.human.pose.vec,
                self.robot.path_to(self.enemy).rad
            )

        # check goal was updated
        if self.goal.is_moved(self.goal_cache, 0.05):
            self.goal_cache.set(self.goal)
            # self.goal_pub.publish(self.make_point_pub(self.goal))
            self.reached = False



    def set_next_vel(self):
        v_r_des = (self.goal.vec - self.robot.pose.vec) * ( W_ROBOT * (abs(self.enemy.perp(self.human) - self.human.perp(self.enemy)) + 1) )

        if(self.reached):
            #self.next_vel.linear.x = 0.0
            #self.attitude(self.goal.yaw, False)

            self.next_vel.linear.x = 0.0
            self.next_vel.linear.y = 0.0

            return

        #modifier = self.attitude(v_r_des.rad, False)# True)
        #self.next_vel.linear.x = v_r_des.norm() * modifier

        if(self.robot.gap_to(self.human) < R_SOCIAL):

            r_h = self.robot.path_to(self.human)
            theta = v_r_des.rad - r_h.rad
            if(math.pi < theta):
                theta = -2 * math.pi + theta
            if(theta < -1 * math.pi):
                theta =  2 * math.pi - theta
            if(abs(theta)< NEAR_ROT):
                v_r_des = Vector(0.0,0.0)
            else:
                if(math.pi*-0.5 < theta):
                    v_r_des = v_r_des.rotate(math.pi*0.5 - theta)
                if(theta < math.pi*0.5):
                    v_r_des = v_r_des.rotate(math.pi*0.5 - theta)

        v_r_roted = v_r_des.rotate(-self.robot.pose.yaw)
        self.next_vel.linear.x = v_r_roted.x
        self.next_vel.linear.y = v_r_roted.y

        f_r_des = self.robot.path_to(self.enemy).rad if(self.protect_mode) else v_r_des.rad
        gap_a = f_r_des - self.robot.pose.yaw

        # normalize gap_a to be included by [-PI , PI]
        if(math.pi < gap_a):
            gap_a = -2 * math.pi + gap_a
        if(gap_a < -1 * math.pi):
            gap_a =  2 * math.pi - gap_a

        if(abs(gap_a) > NEAR_ROT):
            self.next_vel.angular.z = gap_a
        else:
            self.next_vel.angular.z = 0.0

        if(v_r_des.norm() < NEAR_DIS) :
            self.reached = True
        else:
            print("not reached : {} gap={}".format(v_r_des,v_r_des.norm()))



    def attitude(self,
        goal_yaw,
        turn_to_avoid
    ): # returns modifier to linear speed
        gap_a = goal_yaw - self.robot.pose.yaw

        # normalize gap_a to be included by [-PI , PI]
        if(math.pi < gap_a):
            gap_a = -2 * math.pi + gap_a
        if(gap_a < -1 * math.pi):
            gap_a =  2 * math.pi - gap_a

        # check whether robot should walk_backwards("ushiro-aruki")
        walk_backwards = abs(gap_a) > math.pi * 0.75
        if(walk_backwards):
            gap_a = gap_a - math.pi

        if(gap_a > NEAR_ROT):
            self.next_vel.angular.z = SPD_ANG
        elif(gap_a < NEAR_ROT*-1):
            self.next_vel.angular.z = SPD_ANG*(-1)
        else:
            self.next_vel.angular.z = 0.0

        sign = -1 if walk_backwards else 1
        slow_down = 1.0

        if(turn_to_avoid):
            if(abs(gap_a) > NEAR_ROT):
                self.next_vel.angular.z += SPD_ANG * self.turn_to_avoid
            slow_down = max( slow_down * (1 - self.turn_to_avoid * 0.2), 0)

        return sign * slow_down




if __name__ == "__main__":
    rospy.init_node('naito', anonymous=True)
    naito = Naito_node()
    r = rospy.Rate(30)

    while not rospy.is_shutdown():
        if(
            not naito.robot.cache is None
            and not naito.human.cache is None
            and not naito.enemy.cache is None
        ):
            naito.set_next_vel()
            naito.vel_pub.publish(naito.next_vel)
        if(True):
            print( "goal {}".format(naito.goal ) )
            print( "robot{}".format(naito.robot) )
            print( "human{}".format(naito.human) )
            print( "enemy{}".format(naito.enemy) )
            print("\n")
        try:
            r.sleep()
        except:
            print("time error!")
