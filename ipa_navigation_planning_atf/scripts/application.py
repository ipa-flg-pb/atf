#!/usr/bin/python
import unittest
import rospy
import rostest
import sys

from atf_core import ATF

import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


class Application:
    def __init__(self):
        self.atf = ATF()


    def initializeRobot(self):
        self.pose_pub_ = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.goal_pub_ = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pose = PoseStamped()
        self.pose_cov = PoseWithCovarianceStamped()
        self.pose.pose.position.x = 4.0

        self.pose_cov.header.stamp = rospy.Time.now()
        self.pose_cov.header.frame_id = "map"

        self.pose.header.frame_id = "map"

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 1.0)
        self.pose_cov.pose.pose.orientation.x = quat[0]
        self.pose_cov.pose.pose.orientation.y = quat[1]
        self.pose_cov.pose.pose.orientation.z = quat[2]
        self.pose_cov.pose.pose.orientation.w = quat[3]

        self.pose.pose.orientation.x = quat[0]
        self.pose.pose.orientation.y = quat[1]
        self.pose.pose.orientation.z = quat[2]
        self.pose.pose.orientation.w = quat[3]

        for x in range(0,4):
            rospy.sleep(2)
            self.pose_pub_.publish(self.pose_cov)


    def execute(self):

        self.initializeRobot()
        self.atf.start("testblock_01")
        #for x in range(0,4):
        #    rospy.sleep(2)
        #    self.pose_pub_.publish(self.pose_cov)

        self.pose.header.stamp = rospy.Time.now()
        self.goal_pub_.publish(self.pose)
        for x in range(0,4):
            rospy.sleep(4)
            self.pose.header.stamp = rospy.Time.now()
            self.goal_pub_.publish(self.pose)

        self.atf.stop("testblock_01")
#        self.atf.start("testblock_02")
#        rospy.sleep(1)
#        self.pose_pub_.publish(self.pose_cov)
#
#        #self.pose.pose.position.x = 8.0
#        #self.pose.pose.position.y = 0.0
#        self.pose.header.stamp = rospy.Time.now()
#        self.goal_pub_.publish(self.pose)
#        rospy.sleep(20)
#
#        self.atf.stop("testblock_02")
        self.atf.shutdown()

class Test(unittest.TestCase):
    def setUp(self):
        self.app = Application()

    def tearDown(self):
        pass

    def test_Recording(self):
        self.app.execute()

if __name__ == '__main__':
    rospy.init_node('test_name')
    if "standalone" in sys.argv:
        app = Application()
        app.execute()
    else:
        rostest.rosrun('application', 'recording', Test, sysargs=None)
