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


    def initializeRobotAndGoals(self):
        self.pose_pub_ = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.goal_pub_ = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        # get starting pose
        start_pose_information = rospy.get_param("/atf_test/start_pose")
        self.start_pose = PoseWithCovarianceStamped()
        self.start_pose.header.frame_id = "map"
        self.start_pose.pose.pose.position.x = start_pose_information["position_x"]
        self.start_pose.pose.pose.position.y = start_pose_information["position_y"]
        self.start_pose.pose.pose.position.z = 0.0
        self.start_pose.pose.pose.orientation.x = start_pose_information["orientation_x"]
        self.start_pose.pose.pose.orientation.y = start_pose_information["orientation_y"]
        self.start_pose.pose.pose.orientation.z = start_pose_information["orientation_z"]
        self.start_pose.pose.pose.orientation.w = start_pose_information["orientation_w"]

        self.nr_of_goals = rospy.get_param("/atf_test/nr_of_goals")

        # get goals
        self.goals = []
        goal_information = rospy.get_param("/atf_test/goals")
        for x in range(0, self.nr_of_goals):
            goal_to_set = PoseStamped()
            goal_to_set.header.frame_id = "map"
            goal_to_set.pose.position.x = goal_information[x]["position_x"]
            goal_to_set.pose.position.y = goal_information[x]["position_y"]
            goal_to_set.pose.position.z = 0.0
            goal_to_set.pose.orientation.x = goal_information[x]["orientation_x"]
            goal_to_set.pose.orientation.y = goal_information[x]["orientation_y"]
            goal_to_set.pose.orientation.z = goal_information[x]["orientation_z"]
            goal_to_set.pose.orientation.w = goal_information[x]["orientation_w"]
            self.goals.append(goal_to_set)

        rospy.sleep(5)
        self.start_pose.header.stamp = rospy.Time.now()
        self.pose_pub_.publish(self.start_pose)


    def execute(self):
        self.atf.start("testblock_01")

        for x in range(0, self.nr_of_goals):
            self.goals[x].header.stamp = rospy.Time.now()
            self.goal_pub_.publish(self.goals[x])
            rospy.sleep(10) #TODO: send new goal once the previous one was reached and not after a fixed amount of time -> goal_reached_publisher in ipa_eband_planner/eband_trajectory_controller

        self.atf.stop("testblock_01")
        self.atf.shutdown()

class Test(unittest.TestCase):
    def setUp(self):
        self.app = Application()
        self.app.initializeRobotAndGoals()
        rospy.sleep(5)

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
