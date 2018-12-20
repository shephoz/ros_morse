#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from threading import Thread, Lock
from time import sleep

class Test1_node:
    def __init__(self):
        # initialize node
        rospy.init_node('test1_py_node', anonymous=True)
        # subscribers
        odom_sub_ = rospy.Subscriber('xbot/odom', Odometry, self._odom_cb)
        self._odom_data = Odometry()
        # publishers
        self._twist_pub = rospy.Publisher('xbot/cmd_vel', Twist, queue_size=1)

        # processing thread
        self._lock = Lock()
        self._pause_dur = 1.0 / rospy.get_param('hz', 10.0) # get a parameter
        proc_thread_ = Thread(target=self._proc_loop)
        proc_thread_.daemon = True; proc_thread_.start()
        # ROS callback thread
        rospy.spin()

    def _proc_loop(self):
        while True:
            self._lock.acquire()
            # # process data from callbacks here # #
            x_ = self._odom_data.pose.pose.position.x
            y_ = self._odom_data.pose.pose.position.y
            print(x_,y_)
            # # # # #
            self._lock.release()

            # publish a twist message
            twist_msg_ = Twist()
            twist_msg_.linear.x = -3.0
            self._twist_pub.publish(twist_msg_)

            sleep(self._pause_dur) # pause for callbacks

    def _odom_cb(self, msg): # odometry callback
        self._lock.acquire()
        self._odom_data = msg # get odometry data
        self._lock.release()

if __name__ == '__main__':
    test_node_ = Test1_node()
