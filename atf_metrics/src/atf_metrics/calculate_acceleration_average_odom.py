#!/usr/bin/env python
import rospy
import math

from nav_msgs.msg import Odometry

class CalculateAccelerationAverageOdomParamHandler:
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
                rospy.logwarn("No groundtruth parameters given, skipping groundtruth evaluation for metric 'acceleration_average_odom' in testblock '%s'", testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            metrics.append(CalculateAccelerationAverageOdom(metric["odom_topic"], groundtruth, groundtruth_epsilon))
        return metrics

class CalculateAccelerationAverageOdom:
    def __init__(self, odom_topic, groundtruth, groundtruth_epsilon):
        """
        Class for calculating the average acceleration of a path followed by the given frame in relation to a given root frame.
        The tf data is sent over the tf topic given in the robot_config.yaml.
        :param root_frame: name of the first frame
        :type  root_frame: string
        :param measured_frame: name of the second frame. The distance will be measured in relation to the root_frame.
        :type  measured_frame: string
        """

        self.active = False
        self.acceleration_average = 0.0
        self.nr_of_data_points = 0
        self.odom_old = []
        self.first_value = True
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
        self.first_value = True

    def purge(self, timestamp):
        pass

    def odom_callback(self, odom):
        if self.active:
            if self.first_value:
                self.odom_old = odom
                self.first_value = False
            else:
                delta_t_inverse = 1/(odom.header.stamp - odom_old.header.stamp).to_sec()
                acceleration_summand = math.sqrt(((odom.twist.twist.linear.x - odom_old.twist.twist.linear.x) * delta_t) ** 2 +
                                                 ((odom.twist.twist.linear.y - odom_old.twist.twist.linear.y) * delta_t) ** 2 +
                                                 ((odom.twist.twist.linear.z - odom_old.twist.twist.linear.z) * delta_t) ** 2)
                self.acceleration_average = (self.nr_of_data_points * self.acceleration_average + acceleration_summand) / (self.nr_of_data_points +1)
                self.nr_of_data_points +=1
                odom_old = odom

    def get_result(self):
        groundtruth_result = None
        details = {"odom_topic": self.odom_topic}
        if self.finished:
            data = round(self.acceleration_average, 3)
            if self.groundtruth != None and self.groundtruth_epsilon != None:
                if math.fabs(self.groundtruth - data) <= self.groundtruth_epsilon:
                    groundtruth_result = True
                else:
                    groundtruth_result = False
            return "acceleration_average_odom", data, groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False
