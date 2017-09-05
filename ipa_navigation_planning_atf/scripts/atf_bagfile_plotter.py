#!/usr/bin/env python
import rospy
import rosbag

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import numpy
import matplotlib.pyplot as plt

class ATFBagfilePlotter:
    def __init__(self):
        self.bag_directory = "/tmp/ipa_navigation_planning_atf/data/"
        self.bag_name = "ts0_c0_r0_e0_0"
        # vel = velocity, acc = acceleration
        self.color_coding = {'vel': 'r', 'acc': 'b', 'jerk': 'g'}
        self.col_names = {'r': 'red', 'b': 'blue', 'g': 'green'}
        # time_scale in seconds
        self.odom_time_values = []
        self.ekf_time_values = []
        # twist values extracted from odometry
        self.odom_values = []
        self.ekf_values = []
        self.plot_path = True
        self.plot_odom = True
        self.use_grid = True
        self.odometry_frequency = 0.02 # Default is 0.02 corresponding to 50Hz
        self.use_smoothing = True
        self.plot_route = True


    def get_data_from_bagfile(self):
        ret = True
        bag_file = self.bag_directory + self.bag_name + '.bag'
        bag = rosbag.Bag(bag_file, 'r')
        topics = []
        if self.plot_path:
            topics.append('/state_ekf')
        if self.plot_odom:
            topics.append('/base/odometry_controller/odometry')
        # we only want to publish the part where something actually happens
        ekf_first_time_value = None
        odom_first_time_value = None
        for topic, msg, t in bag.read_messages(topics):
            if topic == '/state_ekf':
                if ekf_first_time_value == None:
                    ekf_first_time_value = msg.state.header.stamp
                # check if msg.status inititalized?
                self.ekf_time_values.append(msg.state.header.stamp - ekf_first_time_value)
                # we only want to plot the path so only the position is needed
                self.ekf_values.append(msg.state.pose.pose.position)
            if topic == '/base/odometry_controller/odometry':
                if odom_first_time_value == None:
                    odom_first_time_value = msg.header.stamp
                self.odom_time_values.append(msg.header.stamp - odom_first_time_value)
                self.odom_values.append(msg.twist.twist)
        bag.close()
        # check if values are ok
        if self.plot_odom:
            if (len(self.odom_time_values) == 0) or (len(self.odom_values) == 0):
                rospy.logerr('Found no odometry values in bagfile')
                ret = False
        if self.plot_path:
            if (len(self.ekf_time_values) == 0) or (len(self.ekf_values) == 0):
                rospy.logerr('Found no ekf_state values in bagfile')
                ret = False
        zero_counter = 0.0
        for i in range(1, len(self.ekf_time_values)):
            if (self.ekf_time_values[i] - self.ekf_time_values[i - 1]).to_nsec() == 0:
                zero_counter += 1
        if not zero_counter / len(self.odom_time_values) == 0:
            rospy.loginfo('%f percent of the ekf time differences are zero', 100 * zero_counter / len(self.odom_time_values))
        zero_counter = 0.0
        for i in range(1, len(self.odom_time_values)):
            if (self.odom_time_values[i] - self.odom_time_values[i - 1]).to_nsec() == 0:
                zero_counter += 1
        if not zero_counter / len(self.odom_time_values) == 0:
            rospy.loginfo('%f percent of the odometry time differences are zero', 100 * zero_counter / len(self.odom_time_values))
        self.filter_data()
        return ret

    def get_output(self):
        figure_counter = 0
        # ekf based
        if self.plot_path:
            figure_counter += 1
            plt.figure(figure_counter)
            x_values, y_values = self.get_path_coordinates()
            # Need the axis option so that x and y axis are scaled equally
            axis_lb, axis_ub = self.get_axis_arguments(x_values, y_values)
            plt.axis([axis_lb, axis_ub, axis_lb, axis_ub])
            if self.plot_route:
                start_pose_information = rospy.get_param("/atf_test/start_pose")
                goal_information = rospy.get_param("/atf_test/goals")
                start_x, start_y = [start_pose_information['position_x']], [start_pose_information['position_y']]
                goals_x, goals_y = [], []
                nr_of_goals = len(goal_information)
                for i in range(0, nr_of_goals):
                    goals_x.append(goal_information[i]['position_x'])
                    goals_y.append(goal_information[i]['position_y'])
                plt.xlabel('Path taken by the robot(black), start_pose(blue), goals(red)')
                plt.plot(x_values, y_values, 'k', start_x, start_y, 'bs', goals_x, goals_y, 'ro', alpha = 0.5)
            else:
                plt.xlabel('Path taken by the robot')
                plt.plot(x_values, y_values, 'k')
        # odometry based
        if self.plot_odom:
            figure_counter +=1
            plt.figure(figure_counter)
            time_values = self.get_time_values()
            x_lin_vel, y_lin_vel, ang_vel = self.get_velocity_values()
            x_lin_acc, y_lin_acc, ang_acc = self.get_acceleration_values()
            x_lin_jer, y_lin_jer, ang_jer = self.get_jerk_values()
            if self.use_smoothing:
                acc_weight_vector = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
                jer_weight_vector = acc_weight_vector
                x_lin_acc_s, y_lin_acc_s, ang_acc_s = self.get_smooth_acc(x_lin_acc, y_lin_acc, ang_acc, acc_weight_vector)
                x_lin_jer_s, y_lin_jer_s, ang_jer_s = self.get_smooth_jer(x_lin_jer, y_lin_jer, ang_jer, jer_weight_vector)
                #x_lin_jer_s, y_lin_jer_s, ang_jer_s = self.get_jerk_from_acc(x_lin_jer, y_lin_jer, ang_jer, self.odom_time_values)
            else:
                x_lin_acc_s, y_lin_acc_s, ang_acc_s, x_lin_jer_s, y_lin_jer_s, ang_jer_s = x_lin_acc, y_lin_acc, ang_acc, x_lin_jer, y_lin_jer, ang_jer
            plt.subplot(311)
            plt.ylabel('x')
            plt.grid(self.use_grid)
            plt.plot(time_values, x_lin_vel, self.color_coding['vel'],
                     time_values, x_lin_acc_s, self.color_coding['acc'],
                     time_values, x_lin_jer_s, self.color_coding['jerk']
                     )
            plt.subplot(312)
            plt.ylabel('y')
            plt.grid(self.use_grid)
            plt.plot(time_values, y_lin_vel, self.color_coding['vel'],
                     time_values, y_lin_acc_s, self.color_coding['acc'],
                     time_values, y_lin_jer_s, self.color_coding['jerk']
                     )
            plt.subplot(313)
            plt.xlabel('Angular Vel, Acc, Jerk')
            plt.grid(self.use_grid)
            plt.plot(time_values, ang_vel, self.color_coding['vel'],
                     time_values, ang_acc_s, self.color_coding['acc'],
                     time_values, ang_jer_s, self.color_coding['jerk']
                     )
            plt.xlabel('Time in seconds \n' +
                       'Velocity('+ self.col_names[self.color_coding['vel']] +'), acceleration('+ self.col_names[self.color_coding['acc']] +
                       '), jerk('+ self.col_names[self.color_coding['jerk']] +') in m/s, m/s^2, m/s^3')
            plt.ylabel('theta')
#            plt.ylabel('Velocity(red), acceleration(blue), jerk(green) in m/s, m/s^2, m/s^3')
        rospy.loginfo('Number of figures: %i', figure_counter)
        plt.show()

    def filter_data(self):
        # Make sure the difference of timestamps of two successive msgs is greater than topic_frequency; this is only necessary for odometry for higher precision of differentials
        tmp_x, tmp_y = [], []
        tmp_x.append(self.odom_time_values[0])
        tmp_y.append(self.odom_values[0])
        counter = 0
        for i in range(1, len(self.odom_time_values)):
            if (self.odom_time_values[i] - tmp_x[-1]).to_sec() > self.odometry_frequency:
                tmp_x.append(self.odom_time_values[i])
                tmp_y.append(self.odom_values[i])
            else:
                counter += 1
        rospy.loginfo('%i of the in total %i odometry messages have been discarded', counter, len(self.odom_values))
        self.odom_time_values = tmp_x
        self.odom_values = tmp_y


    def get_path_coordinates(self):
        tmp_x, tmp_y = [], []
        for i in range(0, len(self.ekf_values)):
            tmp_x.append(self.ekf_values[i].x)
            tmp_y.append(self.ekf_values[i].y)
        return tmp_x, tmp_y

    def get_axis_arguments(self, x_values, y_values):
        mini, maxi = -1.0, 1.0
        mini = round(min(mini, min(x_values), min(y_values))) - 0.5
        maxi = round(max(maxi, max(x_values), max(y_values))) + 0.5
        return mini, maxi

    def get_time_values(self):
        tmp = []
        for i in range(0, len(self.odom_time_values)):
            tmp.append(self.odom_time_values[i].to_sec())
        return tmp

    def get_velocity_values(self):
        return self.get_velocity('x'), self.get_velocity('y'), self.get_velocity('theta')

    def get_velocity(self, direction):
        if not ((direction == 'x') or (direction == 'y') or (direction == 'theta')):
            rospy.logerr("get_velocity() only compatible with arguments: 'x', 'y' or 'theta'")
            return []
        tmp = []
        for i in range(0, len(self.odom_values)):
            if direction == 'x':
                tmp.append(self.odom_values[i].linear.x)
            if direction == 'y':
                tmp.append(self.odom_values[i].linear.y)
            if direction == 'theta':
                tmp.append(self.odom_values[i].angular.z)
        return tmp

    def get_acceleration_values(self):
        return self.get_acceleration('x'), self.get_acceleration('y'), self.get_acceleration('theta')

    def get_acceleration(self, direction):
        if not ((direction == 'x') or (direction == 'y') or (direction == 'theta')):
            rospy.logerr("get_velocity() only compatible with arguments: 'x', 'y' or 'theta'")
            return []
        tmp = []
        tmp.append(0)
        for i in range(1, len(self.odom_values) - 1):
            dt_left = 1 / (self.odom_time_values[i] - self.odom_time_values[i-1]).to_sec()
            dt_right = 1 / (self.odom_time_values[i+1] - self.odom_time_values[i]).to_sec()
            vel_left, vel_right = 0, 0
            if direction == 'x':
                vel_left = (self.odom_values[i].linear.x - self.odom_values[i-1].linear.x) * dt_left
                vel_right = (self.odom_values[i+1].linear.x - self.odom_values[i].linear.x) * dt_right
            if direction == 'y':
                vel_left = (self.odom_values[i].linear.y - self.odom_values[i-1].linear.y) * dt_left
                vel_right = (self.odom_values[i+1].linear.y - self.odom_values[i].linear.y) * dt_right
            if direction == 'theta':
                vel_left = (self.odom_values[i].angular.z - self.odom_values[i-1].angular.z) * dt_left
                vel_right = (self.odom_values[i+1].angular.z - self.odom_values[i].angular.z) * dt_right
            # we take the arithmetic mean of left and right difference quotient
            tmp.append(0.5 * (vel_left + vel_right))
        tmp.append(0)
        return tmp

    def get_jerk_values(self):
        return self.get_jerk('x'), self.get_jerk('y'), self.get_jerk('theta')

    def get_jerk(self, direction):
        if not ((direction == 'x') or (direction == 'y') or (direction == 'theta')):
            rospy.logerr("get_jerk() only compatible with arguments: 'x', 'y' or 'theta'")
            return []
        tmp = []
        tmp.append(0)
        for i in range(1, len(self.odom_values) - 1):
            dt2 = (0.5 * (self.odom_time_values[i+1] - self.odom_time_values[i-1]).to_sec()) ** 2
            if direction == 'x':
                tmp.append((self.odom_values[i+1].linear.x - 2 * self.odom_values[i].linear.x + self.odom_values[i-1].linear.x) / dt2)
            if direction == 'y':
                tmp.append((self.odom_values[i+1].linear.y - 2 * self.odom_values[i].linear.y + self.odom_values[i-1].linear.y) / dt2)
            if direction == 'theta':
                tmp.append((self.odom_values[i+1].angular.z - 2 * self.odom_values[i].angular.z + self.odom_values[i-1].angular.z) / dt2)
        tmp.append(0)
        return tmp

    def get_jerk_from_acc(self, x_acc, y_acc, theta_acc, time_values):
        tmp_x_jer, tmp_y_jer, tmp_th_jer = [], [], []
        tmp_x_jer.append(0)
        tmp_y_jer.append(0)
        tmp_th_jer.append(0)
        # Assuming that x_acc, y_acc and theta_acc have the same length since they are all derivated from self.odom_values
        for i in range(1, len(x_acc) - 1):
            dt_l = (time_values[i] - time_values[i-1]).to_sec()
            dt_r = (time_values[i+1] - time_values[i]).to_sec()
            tmp_x_jer.append(0.5 * (self.difference_quotient(x_acc, i, dt_l, 'l') + self.difference_quotient(x_acc, i, dt_r, 'r')))
            tmp_y_jer.append(0.5 * (self.difference_quotient(y_acc, i, dt_l, 'l') + self.difference_quotient(y_acc, i, dt_r, 'r')))
            tmp_th_jer.append(0.5 * (self.difference_quotient(theta_acc, i, dt_l, 'l') + self.difference_quotient(theta_acc, i, dt_r, 'r')))
        tmp_x_jer.append(0)
        tmp_y_jer.append(0)
        tmp_th_jer.append(0)
        return tmp_x_jer, tmp_y_jer, tmp_th_jer

    def difference_quotient(self, values, position, dt, side):
        if dt == 0:
            rospy.logerr('Trying di divide by zero, returning 0.0')
            return 0.0
        if ((side == 'left') or (side == 'l')):
            return (values[position] - values[position-1]) / dt
        if ((side == 'right') or (side == 'r')):
            return (values[position+1] - values[position]) / dt
        if ((side == 'both') or (side == 'b') or (side == 'lr') or (side == 'rl')):
            return 0.5 * (self.difference_quotient(values, position, dt, 'l') + self.difference_quotient(values, position, dt, 'r'))
        rospy.logerr('You did not use a proper argument for side, returning 0.0')
        return 0.0

    # values : list of values to smoothen
    # weights : list of weights to use when determining the 'arithmetic mean of the values', the middle weight is used for the value in position
    #           if an even number of weights is passed the last one is ignored
    def get_smooth_output(self, values, weights, normalize_weights=True):
        if len(values) < 2:
            rospy.loginfo('get_smooth_output(): input is by default smooth')
            return values
        if len(weights) == 0:
            rospy.logwarn('get_smooth_output() : vector of weights was empty; nothing to do')
            return values
        l = 0
        if len(weights) % 2 == 0:
            tmp_weights = []
            for i in range(0, len(weights) - 1):
                tmp_weights.append(weights[i])
            return self.get_smooth_output(values, tmp_weights, normalize_weights)
        else:
            l = int((len(weights) - 1) / 2.0)
        # Make sure sum of weights is equal to number of weights (potentially ignoring the last one)
        weights_adjusted = []
        if normalize_weights:
            weight_sum = 0.0
            for i in range(0, 2 * l + 1):
                weight_sum += weights[i]
            for i in range(0, 2 * l + 1):
                weights_adjusted.append(weights[i] * 2 * l / weight_sum)
        else:
            weights_adjusted.append(weights)
        tmp = []
        for i in range(0, len(values)):
            sum = 0.0
            for j in range(i-l, i+l+1):
                # extrapolate values by values[0] to the left
                if j < 0:
                    sum += values[0] * weights_adjusted[j - i + l]
                # extrapolate values by values[-1] to the right
                elif j >= len(values):
                    sum += values[-1] * weights_adjusted[j - i + l]
                else:
                    sum += values[j] * weights_adjusted[j - i + l]
            tmp.append(sum / len(weights_adjusted))
        return tmp

    def get_smooth_acc(self, x_l_a, y_l_a, th_a_a, acc_weights):
        tmp_x_l_a, tmp_y_l_a, tmp_th_a_a = self.get_smooth_output(x_l_a, acc_weights), self.get_smooth_output(y_l_a, acc_weights), self.get_smooth_output(th_a_a, acc_weights)
        return tmp_x_l_a, tmp_y_l_a, tmp_th_a_a

    def get_smooth_jer(self, x_l_j, y_l_j, th_a_j, jer_weights):
        tmp_x_l_j, tmp_y_l_j, tmp_th_a_j = self.get_smooth_output(x_l_j, jer_weights), self.get_smooth_output(y_l_j, jer_weights), self.get_smooth_output(th_a_j, jer_weights)
        return tmp_x_l_j, tmp_y_l_j, tmp_th_a_j


if __name__ == '__main__':
    rospy.loginfo('Starting atf_bagfile_plotter')
    rospy.init_node('atf_bagfile_plotter')
    abp = ATFBagfilePlotter()
    if abp.get_data_from_bagfile():
        abp.get_output()
