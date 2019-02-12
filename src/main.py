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
from rosgraph_msgs.msg import Clock

from layer2.msg import HTEntityList
from layer2.msg import HTEntity
from std_msgs.msg import Header


from std_msgs.msg import String

# Const Definition
VEL_R_MAX = 1.5
W_ROBOT   = 1.2
R_PROTECT = 0.9
R_SOCIAL  = 1.0

AVOIDING_SLOW_DOWN_RATE = 0.5

NEAR_ROT  = math.pi / 12 # [rad]
NEAR_DIS  = 0.1 # [m]

SPD_ANG = math.pi / 6
SPD_LIN = 0.8

RATE = 0.3

class Naito_node:

    def __init__(self):
        #self.vel_pub   = rospy.Publisher('/valocity_to', TwistStamped, queue_size=1)
        self.vel_pub   = rospy.Publisher('/xbot/cmd_vel', Twist, queue_size=1)
        #self.rml_pub   = rospy.Publisher('/naito/rml', String, queue_size=1)

        # rospy.Subscriber('/xbot/odom',   Odometry,       self.robot_callback)
        # rospy.Subscriber('/pedestrians', PedestrianData, self.pedestrian_callback)
        rospy.Subscriber('/pose_6d',   PoseWithCovarianceStamped, self.robot_callback)
        rospy.Subscriber('/human_tracked_l2', HTEntityList , self.humantrack_callback)
        # rospy.Subscriber('/scan',        LaserScan,      self.scan_callback)
        rospy.Subscriber('/clock',       Clock,          self.clock_callback)


        self.robot = Subject()
        self.human = Subject()
        self.enemy = Subject()

        self.human_macro_vel = Vector()

        self.protect_mode = False

        self.goal       = None
        self.goal_cache = None
        self.reached       = False
        self.turn_to_avoid = 0

        self.next_vel = Twist()

        self.clock_cache = None


    def is_ready(self):
        return (not self.robot.cache is None
        and not self.human.cache is None
        and not self.enemy.cache is None
        and not self.human.id is None
        and not self.enemy.id is None)

    def clock_callback(self,data):
        sec = data.clock.secs
        dsec = data.clock.nsecs // (100*1000*1000)
        if(dsec %2 == 0):
            if(self.clock_cache == None or self.clock_cache < sec*10+dsec):
                self.robot.calc_vel()
                self.human.calc_vel()
                if(self.human.vel.norm() > 0.1):
                    self.human_macro_vel.set(self.human.vel)
                self.enemy.calc_vel()

                print( "goal {}".format(naito.goal ) )
                print( "robot{}".format(naito.robot) )
                print( "human{}".format(naito.human) )
                print( "enemy{}".format(naito.enemy) )
                print( "nextv{}".format(naito.next_vel))
                print("\n")

                self.clock_cache = sec*10+dsec


    def robot_callback(self,data):
        #if(data.header.frame_id == "map"):
            self.robot.update_pose(data.pose.pose)


    def humantrack_callback(self,data):
        if(self.human.id is None):
            human = data.list[0]
            enemy = data.list[1]
            if(Pose(enemy.x,enemy.y,0.0).gap_to(self.robot.pose) < Pose(human.x,human.y,0.0).gap_to(self.robot.pose)):
                enemy,human = human,enemy
            self.human.id = human.id
            self.enemy.id = enemy.id
        for entity in data.list:
            if(entity.id == self.human.id):
                self.human.update_pose(Pose(entity.x,entity.y,0.0))
            if(entity.id == self.enemy.id):
                self.enemy.update_pose(Pose(entity.x,entity.y,0.0))
        if(self.is_ready()):
            self.check_protect_mode()
            self.set_goal()






    def pedestrian_callback(self,data):
        for odom in data.odoms:
            if (odom.child_frame_id in ["Dummy1", "Pedestrian_female 42"]):
                self.human.update_pose(odom.pose.pose)
            if (odom.child_frame_id in ["Scripted Human", "Pedestrian_male 02"]):
                self.enemy.update_pose(odom.pose.pose)
        if(self.is_ready()):
            self.check_protect_mode()
            self.set_goal()


    # def scan_callback(self,data):
    #     # from scan message
    #     ranges    = data.ranges
    #     range_max = data.range_max
    #     # let : amount of obstacle-index about each direction
    #     right = 0
    #     mid   = 0
    #     left  = 0
    #     # const : define width & pan of left & right
    #     width = 60
    #     pan   = 250
    #
    #     for i in range(0,1081,15):
    #         # print("{:4d}".format(i)+(  "-" * int( ranges[i] * 3 )  ))
    #         # if(i in range(540-pan-width,540-pan+width)):
    #         #     right += ranges[i]
    #         # if(i in range(540-width,540+width)):
    #         #     mid += ranges[i]
    #         # if(i in range(540+pan-width,540+pan+width)):
    #         #     left += ranges[i]
    #
    #         pan = 540
    #         right += ranges[i] * max((1 - pow(i/pan,2)), 0)
    #         mid   += ranges[i] * (1 - pow(i/540 - 1,2))
    #         left  += ranges[i] * max((1 - pow((1080-i)/pan,2)), 0)
    #
    #     #naito.rml_pub.publish("{:.4f}  {:.4f}  {:.4f}".format(left,mid,right))
    #
    #     # const : define threshold & ratio to turn_to_avoid
    #     thre = 600
    #     rati = 0.01
    #
    #     if(mid < thre): # if in front of robot is obstacled,
    #         self.turn_to_avoid = (thre - mid) * rati
    #         if(left < right):
    #             self.turn_to_avoid *= -1
    #     else:
    #         self.turn_to_avoid = 0
    #
    #     naito.rml_pub.publish("{:.4f}  {:.4f}".format(mid,self.turn_to_avoid))

    def check_protect_mode(self):
        self.protect_mode = self.human.gap_to(self.enemy) < 7.0
        #self.protect_mode = False

    def set_goal(self):
        if(not self.protect_mode):
            # normal mode : walk aside the partner
            if (self.human_macro_vel.norm() == 0):
                aside_vec = self.human.path_to(self.robot).normalize() * R_SOCIAL
                goal = Pose(self.human.pose.vec + aside_vec, self.human.path_from(self.robot).rad)
                self.goal = goal
            else:
                aside_vec = self.human_macro_vel.rotate(math.pi/2).normalize() * R_SOCIAL
                p1 = Pose(self.human.pose.vec + aside_vec, self.human_macro_vel.rad)
                p2 = Pose(self.human.pose.vec - aside_vec, self.human_macro_vel.rad)
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
        if self.goal_cache is None or self.goal.is_moved(self.goal_cache, 0.05):
            self.goal_cache = Pose()
            self.goal_cache.set(self.goal)
            self.reached = False



    def set_next_vel(self):
        v_r_des = (self.goal.vec - self.robot.pose.vec) * ( W_ROBOT * (abs(self.enemy.perp(self.human) - self.human.perp(self.enemy)) + 1) )

        if(self.reached):
            self.next_vel.linear.x = 0.0
            self.next_vel.linear.y = 0.0
            self.next_vel.angular.z = self.attitude(self.goal.yaw,self.goal.yaw)
            return

        # avoiding partner
        p_r = self.human.path_to(self.robot)
        p_r_gap = p_r.norm()
        weight = 1.5
        d = R_PROTECT if self.protect_mode else R_SOCIAL
        avoid_meter = (1 / (p_r_gap * weight))
        v_r_des_modified = v_r_des + (p_r.normalize() * max(avoid_meter, 0.0))

        v_r_roted = v_r_des_modified.rotate(-(self.robot.pose.yaw))


        if(True):
            if(v_r_roted.x > 0):
                v_r_roted.x = min(v_r_roted.x, SPD_LIN)
            if(v_r_roted.x < 0):
                v_r_roted.x = max(v_r_roted.x, -SPD_LIN)
            if(v_r_roted.y > 0):
                v_r_roted.y = min(v_r_roted.y, SPD_LIN)
            if(v_r_roted.y < 0):
                v_r_roted.y = max(v_r_roted.y, -SPD_LIN)

            self.next_vel.linear.x = v_r_roted.x
            self.next_vel.linear.y = v_r_roted.y
        else:
            self.next_vel.linear.x = 0.0
            self.next_vel.linear.y = 0.0


        self.next_vel.angular.z = self.attitude(v_r_des.rad,self.goal.yaw)

        if(v_r_des.norm() < NEAR_DIS):
            self.reached = True
        # else:
        #     print("not reached : {} gap={}".format(v_r_des,v_r_des.norm()))

    def attitude(self,f_r_des_normal,f_r_des_protect):
        f_r_des = f_r_des_protect if self.protect_mode else f_r_des_normal
        gap_a = f_r_des - self.robot.pose.yaw

        # normalize gap_a to be included by [-PI , PI]
        if(math.pi < gap_a):
            gap_a = -2 * math.pi + gap_a
        if(gap_a < -math.pi):
            gap_a =  2 * math.pi - gap_a

        if(gap_a > NEAR_ROT):
            return min(gap_a,SPD_ANG)
        elif(gap_a < -NEAR_ROT):
            return max(gap_a,-SPD_ANG)
        else:
            return 0.0




if __name__ == "__main__":
    rospy.init_node('naito', anonymous=True)
    naito = Naito_node()
    r = rospy.Rate(10)
    listener = tf.TransformListener()
    #rospy.sleep(100)
    # listener.waitForTransform("/pedestrians","/human_tracked_l2",rospy.Time(),rospy.Duration(4.0))





    tracked_pub = rospy.Publisher('/human_tracked_l2', HTEntityList, queue_size=2)

    i=0
    header = Header()
    header.seq = i
    header.stamp = rospy.Time.now()
    header.frame_id = "map"

    htelist = HTEntityList()
    htelist.header = header
    htelist.list.append(HTEntity())
    htelist.list.append(HTEntity())

    htelist.list[0].header = header
    htelist.list[0].id = 1
    htelist.list[0].unique_id = -1
    htelist.list[0].type = 0
    htelist.list[0].x = -20.3821392059
    htelist.list[0].y = 0.953898787498
    htelist.list[0].z = 1.66053318977
    htelist.list[0].body_orientation = 3.06264400482
    htelist.list[0].motion_direction = 3.06264400482
    htelist.list[0].velocity = 0.0554296225309
    htelist.list[0].head_orientation = 3.06264400482
    htelist.list[0].is_speaking = False
    htelist.list[0].option_fields = ''
    htelist.list[0].k2body = ''

    htelist.list[1].header = header
    htelist.list[1].id = 2
    htelist.list[1].unique_id = -1
    htelist.list[1].type = 0
    htelist.list[1].x = -30.2448654175
    htelist.list[1].y = 0.480877876282
    htelist.list[1].z = 1.79357945919
    htelist.list[1].body_orientation = -0.322182565928
    htelist.list[1].motion_direction = -0.322182565928
    htelist.list[1].velocity = 0.213831529021
    htelist.list[1].head_orientation = -0.322182565928
    htelist.list[1].is_speaking = False
    htelist.list[1].option_fields = ''
    htelist.list[1].k2body = ''
    tracked_pub.publish(htelist)

    while not rospy.is_shutdown():
        if (i<=1200):
            header = Header()
            header.seq = i
            header.stamp = rospy.Time.now()
            header.frame_id = "map"

            htelist = HTEntityList()
            htelist.header = header
            htelist.list.append(HTEntity())
            htelist.list.append(HTEntity())

            htelist.list[0].header = header
            htelist.list[0].id = 1
            htelist.list[0].unique_id = -1
            htelist.list[0].type = 0
            htelist.list[0].x = -20.3821392059
            htelist.list[0].y = 0.953898787498
            htelist.list[0].z = 1.66053318977
            htelist.list[0].body_orientation = 3.06264400482
            htelist.list[0].motion_direction = 3.06264400482
            htelist.list[0].velocity = 0.0554296225309
            htelist.list[0].head_orientation = 3.06264400482
            htelist.list[0].is_speaking = False
            htelist.list[0].option_fields = ''
            htelist.list[0].k2body = ''

            htelist.list[1].header = header
            htelist.list[1].id = 2
            htelist.list[1].unique_id = -1
            htelist.list[1].type = 0
            htelist.list[1].x = -30.2448654175
            htelist.list[1].y = 0.480877876282
            htelist.list[1].z = 1.79357945919
            htelist.list[1].body_orientation = -0.322182565928
            htelist.list[1].motion_direction = -0.322182565928
            htelist.list[1].velocity = 0.213831529021
            htelist.list[1].head_orientation = -0.322182565928
            htelist.list[1].is_speaking = False
            htelist.list[1].option_fields = ''
            htelist.list[1].k2body = ''
            tracked_pub.publish(htelist)
            i= i+1
        if(naito.is_ready() and not naito.goal is None):
            naito.set_next_vel()
            naito.vel_pub.publish(naito.next_vel)




        # try:
        #     now = rospy.Time.now()
        #     listener.waitForTransform("/pedestrians","/human_tracked_l2",now,rospy.Duration(3.0))
        #     (trans,rot) = listener.lookupTransform("/pedestrians","/human_tracked_l2",now)
        #     print(trans)
        #     print(rot)
        # except:
        #     print("transform error")

        try:
            r.sleep()
        except:
            print("time error!")
