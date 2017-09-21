#!/usr/bin/python
import unittest
import rospy
import rostest
import sys

from atf_core import ATF

import tf #this should not be necessary
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray


class Application:
    def __init__(self):
        self.atf = ATF()

        self.pose_pub_ = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.goal_pub_ = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self.goal_reached_sub_ = rospy.Subscriber("/move_base/status", GoalStatusArray, self.goal_reached_callback)
        self.reached_last_goal = False
        self.time_threshold = rospy.get_param("/atf_test/time_threshold", 12.0)
        self.loop_rate = rospy.Rate(rospy.get_param("/move_base/controller_frequency", 20))
        self.last_time_stamp = rospy.Time.now()

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

        # get goals
        self.goals = []
        goal_information = rospy.get_param("/atf_test/goals")
        self.nr_of_goals = len(goal_information)
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
        self.goals.reverse()

        # Publish starting pose
        rospy.sleep(5)
        self.start_pose.header.stamp = rospy.Time.now()
        self.pose_pub_.publish(self.start_pose)
        rospy.sleep(5)


    def goal_reached_callback(self, data):
        # make sure callback is not a redundant callback from the previous goal
        if (rospy.Time.now() - self.last_time_stamp).to_sec() >= 1:
            # goal_status == 3 corresponds to SUCCESS
            if data.status_list[-1].status == 3:
                # check if last goal was reached ..
                if self.nr_of_goals == 0:
                    self.reached_last_goal = True
                # .. otherwise send the next one
                else:
                    goal = self.goals.pop()
                    goal.header.stamp = rospy.Time.now()
                    self.goal_pub_.publish(goal)
                    self.nr_of_goals -=1
                    self.last_time_stamp = rospy.Time.now()


    def execute(self):
        self.atf.start("testblock_01")

        if not self.nr_of_goals == 0:
            # Publish initial goal
            goal = self.goals.pop()
            goal.header.stamp = rospy.Time.now()
            self.goal_pub_.publish(goal)
            self.nr_of_goals -=1
            # Publish the remaining goals
            self.last_time_stamp = rospy.Time.now()
            while not self.reached_last_goal:
                if (rospy.Time.now() - self.last_time_stamp).to_sec() > self.time_threshold:
                    rospy.logwarn("Current goal could not be reached; triggering to publish the next goal")
                    # check if last goal was reached ..
                    if self.nr_of_goals == 0:
                        self.reached_last_goal = True
                    # .. otherwise send the next one
                    else:
                        goal = self.goals.pop()
                        goal.header.stamp = rospy.Time.now()
                        self.goal_pub_.publish(goal)
                        self.nr_of_goals -=1
                        self.last_time_stamp = rospy.Time.now()
                else:
                    self.loop_rate.sleep()
        else:
            rospy.logwarn("There are no goals defined; please make sure your robot environment yaml contains a list of goals")

        self.atf.stop("testblock_01")
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
