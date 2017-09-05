#!/usr/bin/env python
import rospy
import math

from nav_msgs.msg import Odometry

class CalculateVelocityMedianOdomParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        pass

    def parse_parameter(self, testblock_name, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        metrics = []
        if type(params) is not list:
            rospy.logerr("metric config not a list")
            return False

        for metric in params:
            # check for optional parameters
            try:
                groundtruth = metric["groundtruth"]
                groundtruth_epsilon = metric["groundtruth_epsilon"]
            except (TypeError, KeyError):
                rospy.logwarn("No groundtruth parameters given, skipping groundtruth evaluation for metric 'velocity_median_odom' in testblock '%s'", testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            metrics.append(CalculateVelocityMedianOdom(metric["odom_topic"], groundtruth, groundtruth_epsilon))
        return metrics

class CalculateVelocityMedianOdom:
    def __init__(self, odom_topic, groundtruth, groundtruth_epsilon):
        """
        Class for calculating the average velocity of a path followed by the given frame in relation to a given root frame.
        The tf data is sent over the tf topic given in the robot_config.yaml.
        :param root_frame: name of the first frame
        :type  root_frame: string
        :param measured_frame: name of the second frame. The distance will be measured in relation to the root_frame.
        :type  measured_frame: string
        """

        self.active = False
        self.velocity_median = []
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.finished = False
        self.odom_topic = odom_topic
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)

    def start(self, timestamp):
        self.active = True

    def stop(self, timestamp):
        self.active = False
        self.finished = True

    def pause(self, timestamp):
        self.active = False

    def purge(self, timestamp):
        pass

    def odom_callback(self, odom):
        if self.active:
            velocity_value = math.sqrt(odom.twist.twist.linear.x ** 2 + odom.twist.twist.linear.y ** 2 + odom.twist.twist.linear.z ** 2)
            self.insert_value(velocity_value)

    def insert_value(self, value):
        self.velocity_median.append(value)
        self.velocity_median.sort()
        # if we have 3 items we can remove the largest and the smallest
        if len(self.velocity_median) > 2:
            self.velocity_median.pop()
            self.velocity_median.pop(0)

    def get_median(self):
        if len(self.velocity_median) is 0:
            # this should not happen
            return 0
        if len(self.velocity_median) % 2 == 0:
            # We need to compute the mean of the two middle values; note that there should not be more than 2 values -> c.f. insert_value()
            return (self.velocity_median[0] + self.velocity_median[1]) / 2
        if len(self.velocity_median) % 2 == 1:
            return self.velocity_median.pop((len(self.velocity_median) - 1) / 2)

    def get_result(self):
        groundtruth_result = None
        details = {"odom_topic": self.odom_topic}
        if self.finished:
            data = round(self.get_median(), 3)
            if self.groundtruth != None and self.groundtruth_epsilon != None:
                if math.fabs(self.groundtruth - data) <= self.groundtruth_epsilon:
                    groundtruth_result = True
                else:
                    groundtruth_result = False
            return "velocity_median_odom", data, groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False
