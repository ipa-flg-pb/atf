#!/usr/bin/env python
import rospy
import tf
import math

class CalculateVelocityMedianParamHandler:
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
                rospy.logwarn("No groundtruth parameters given, skipping groundtruth evaluation for metric 'velocity_median' in testblock '%s'", testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            metrics.append(CalculateVelocityMedian(metric["root_frame"], metric["measured_frame"], groundtruth, groundtruth_epsilon))
        return metrics

class CalculateVelocityMedian:
    def __init__(self, root_frame, measured_frame, groundtruth, groundtruth_epsilon):
        """
        Class for calculating the median smoothness of a path followed by the given frame in relation to a given root frame. i.e. the median difference in rotation between two timestamps
        The tf data is sent over the tf topic given in the robot_config.yaml.
        :param root_frame: name of the first frame
        :type  root_frame: string
        :param measured_frame: name of the second frame. The distance will be measured in relation to the root_frame.
        :type  measured_frame: string
        """

        self.active = False
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        self.velocity_median = []
        self.tf_sampling_freq = 20.0  # Hz
        self.first_value = True
        self.trans_old = []
        self.rot_old = []
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.finished = False

        self.listener = tf.TransformListener()

        rospy.Timer(rospy.Duration.from_sec(1 / self.tf_sampling_freq), self.record_tf)

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

    def record_tf(self, event):
        if self.active:
            try:
                self.listener.waitForTransform(self.root_frame,
                                               self.measured_frame,
                                               rospy.Time(0),
                                               rospy.Duration.from_sec(1 / (2*self.tf_sampling_freq)))
                (trans, rot) = self.listener.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))

            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                rospy.logwarn(e)
                pass
            else:
                if self.first_value:
                    self.trans_old = trans
                    self.rot_old = rot
                    self.first_value = False
                    return
                #print "transformations: \n", "trans[0]", trans[0], "self.trans_old[0]",self.trans_old[0], "trans[1]", trans[1], "self.trans_old[1]",self.trans_old[1], "trans[2]",trans[2], "self.trans_old[2]",self.trans_old[2], "\n ------------------------------------------------ "
                velocity_value = math.sqrt(((trans[0] - self.trans_old[0]) * self.tf_sampling_freq) ** 2 + ((trans[1] - self.trans_old[1]) * self.tf_sampling_freq) ** 2 +
                                           ((trans[2] - self.trans_old[2]) * self.tf_sampling_freq) ** 2)
                self.insert_value(velocity_value)

                self.trans_old = trans
                self.rot_old = rot

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
        details = {"root_frame": self.root_frame, "measured_frame": self.measured_frame}
        if self.finished:
            data = round(self.get_median(), 3)
            if self.groundtruth != None and self.groundtruth_epsilon != None:
                if math.fabs(self.groundtruth - data) <= self.groundtruth_epsilon:
                    groundtruth_result = True
                else:
                    groundtruth_result = False
            return "velocity_median", data, groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False
