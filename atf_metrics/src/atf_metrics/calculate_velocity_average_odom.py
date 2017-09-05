#!/usr/bin/env python
import rospy
import math

from nav_msgs.msg import Odometry

class CalculateVelocityAverageOdomParamHandler:
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
                rospy.logwarn("No groundtruth parameters given, skipping groundtruth evaluation for metric 'velocity_average_odom' in testblock '%s'", testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            metrics.append(CalculateVelocityAverageOdom(metric["odom_topic"], groundtruth, groundtruth_epsilon))
        return metrics

class CalculateVelocityAverageOdom:
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
        self.velocity_average = 0.0
        self.nr_of_data_points = 0
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
            velocity_summand = math.sqrt(odom.twist.twist.linear.x ** 2 + odom.twist.twist.linear.y ** 2 + odom.twist.twist.linear.z ** 2)
            if(self.nr_of_data_points < 1):
                #rospy.logwarn("Transformation: %s, Path Increment: %s",str(trans), str(path_increment))
                self.velocity_average = velocity_summand
                self.nr_of_data_points +=1
            else:
                self.velocity_average = (self.nr_of_data_points * self.velocity_average + velocity_summand) / (self.nr_of_data_points +1)
                self.nr_of_data_points +=1

    def get_result(self):
        groundtruth_result = None
        details = {"odom_topic": self.odom_topic}
        if self.finished:
            data = round(self.velocity_average, 3)
            if self.groundtruth != None and self.groundtruth_epsilon != None:
                if math.fabs(self.groundtruth - data) <= self.groundtruth_epsilon:
                    groundtruth_result = True
                else:
                    groundtruth_result = False
            return "velocity_average_odom", data, groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False
