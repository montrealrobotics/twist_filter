#!/usr/bin/env python3
import math
import copy
from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import threading
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class TwistFilter:
    def __init__(self, node, components):
        self.node = node

        self.filters = components

        self.linear_vel_max = 1.0
        self.linear_acc_max = 1.0
        self.angular_vel_max = 1.0
        self.angular_acc_max = 1.0
        self.timeout = 0.25
        self.last_val = 0
        self.lock = threading.Lock()

        self._declare_parameters()

        self.node.add_on_set_parameters_callback(self._parameters_callback)

        self.time_prev = self.node.get_clock().now()
        self.twist_prev = Twist()
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub_cmd_in = self.node.create_subscription(
            Twist,
            'filter_in',
            self.update_twist,
            1
        )
        self.pub_cmd_out = self.node.create_publisher(
            Twist,
            'filter_out',
            qos
        )

        # Uncomment to publish smoothed twist without velocity/acceleration filtering
        # self.pub_cmd_smoothed = self.node.create_publisher(TwistStamped, 'filter_smooth', 1)

        self.stopped = False
        self.cmd = Twist()
        self.prev_time = self.node.get_clock().now()

        timer_period = 0.1
        self.timer = self.node.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.pub_cmd()

    def _declare_parameters(self):
        """Declare all the parameters for the filter"""

        double_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)

        self.node.declare_parameter('linear_vel_max', 1.0, double_descriptor)
        self.node.declare_parameter('linear_acc_max', 1.0, double_descriptor)
        self.node.declare_parameter('angular_vel_max', 1.0, double_descriptor)
        self.node.declare_parameter('angular_acc_max', 1.0, double_descriptor)
        self.node.declare_parameter('timeout', 0.25, double_descriptor)

        self.linear_vel_max = self.node.get_parameter('linear_vel_max').value
        self.linear_acc_max = self.node.get_parameter('linear_acc_max').value
        self.angular_vel_max = self.node.get_parameter('angular_vel_max').value
        self.angular_acc_max = self.node.get_parameter('angular_acc_max').value
        self.timeout = self.node.get_parameter('timeout').value

    def _parameters_callback(self, params):
        """Callback for parameter updates"""

        for param in params:
            if param.name == 'linear_vel_max':
                self.linear_vel_max = param.value
            elif param.name == 'linear_acc_max':
                self.linear_acc_max = param.value
            elif param.name == 'angular_vel_max':
                self.angular_vel_max = param.value
            elif param.name == 'angular_acc_max':
                self.angular_acc_max = param.value
            elif param.name == 'timeout':
                self.timeout = param.value

        if isinstance(self.filters, list):
            self.node.get_logger().info("Updating filter list")
            for filter in self.filters:
                result = filter.update_filters(params)
        else:
            result = self.filters.update_filters(params)

        self.node.get_logger().info(
            f"Filter Reconfigure: Linear vel max: {self.linear_vel_max}, "
            f"Linear acc max: {self.linear_acc_max}, "
            f"Angular vel max: {self.angular_vel_max}, "
            f"Angular acc max: {self.angular_acc_max}, "
            f"Timeout: {self.timeout}"
        )

        result = SetParametersResult()
        if result:
            result.successful = True
        else:
            result.successful = False
        return result

    def update_twist(self, data):
        with self.lock:
            self.cmd = data
            self.prev_time = self.node.get_clock().now()

    def pub_cmd(self):
        with self.lock:
            current_time = self.node.get_clock().now()
            elapsed = (current_time.nanoseconds - self.prev_time.nanoseconds) / 1e9

            if elapsed > self.timeout:
                cmd = self.filter_twist(Twist())
            else:
                cmd = self.filter_twist(self.cmd)

            is_zero_twist = (cmd.linear.x == 0.0 and cmd.linear.y == 0.0 and cmd.linear.z == 0.0 and
                            cmd.angular.x == 0.0 and cmd.angular.y == 0.0 and cmd.angular.z == 0.0)

            if is_zero_twist:
                if not self.stopped:
                    self.pub_cmd_out.publish(cmd)
                    self.stopped = True
            else:
                self.stopped = False
                if cmd is not None:
                    self.pub_cmd_out.publish(cmd)

    def filter_twist(self, data):
        cmd_out = Twist()
        time_filter = self.node.get_clock().now().nanoseconds
        if isinstance(self.filters, list):
            filter_lin = self.filters[0]
            filter_ang = self.filters[1]
        else:
            filter_lin = self.filters
            filter_ang = self.filters

        for key in filter_lin.linear:
            input_val = float(getattr(data.linear, key))
            filtered_val = float(filter_lin.linear[key].filter_signal(input_val, time_filter))
            setattr(cmd_out.linear, key, filtered_val)

        for key in filter_ang.angular:
            input_val = float(getattr(data.angular, key))
            filtered_val = float(filter_ang.angular[key].filter_signal(input_val, time_filter))

            setattr(cmd_out.angular, key, filtered_val)

        # Uncomment to publish smoothed twist without velocity/acceleration filtering
        cmd_out_stamped = TwistStamped()
        cmd_out_stamped.header.stamp = self.node.get_clock().now().to_msg()
        cmd_out_stamped.twist = cmd_out
        # self.pub_cmd_smoothed.publish(cmd_out_stamped)

        time_now = self.node.get_clock().now()
        time_delta = (time_now.nanoseconds - self.time_prev.nanoseconds) / 1e9

        if self.linear_vel_max > 0 or self.angular_vel_max > 0:
            cmd_out = self._saturate_vel(cmd_out, self.linear_vel_max, self.angular_vel_max)

        if (self.linear_acc_max > 0 or self.angular_acc_max > 0) and time_delta > 0.00001:
            cmd_out = self._saturate_acc(cmd_out, self.linear_acc_max, self.angular_acc_max, time_delta)

        self.twist_prev = cmd_out
        self.time_prev = time_now

        return cmd_out

    def _get_twist_mag(self, v):
        '''
        @brief Returns linear and angular magnitudes of a twist

        @param v - Input twist
        @returns lin - Linear magnitude
        @returns ang - Angular magnitude
        '''

        lin = math.sqrt(v.linear.x**2 + v.linear.y**2 + v.linear.z**2)
        ang = math.sqrt(v.angular.x**2 + v.angular.y**2 + v.angular.z**2)

        return lin, ang

    def _get_max_ratios(self, l_mag, a_mag, l_max, a_max):
        '''
        @brief Returns ratio of the vector magnitude to specified max velocity
               and acceleration

        @param l_mag - Linear magnitude
        @param a_mag - Angular magnitude
        @param l_max - Linear maximum limit
        @param a_max - Angular maximum limit
        @returns l_r - Linear ratio
        @returns a_r - Angular ratio
        '''

        if l_mag <= l_max:
            l_r = 1.0
        else:
            l_r = l_max / l_mag

        if a_mag <= a_max:
            a_r = 1.0
        else:
            a_r = a_max / a_mag

        return l_r, a_r

    def _get_scaling_order(self, ratios):
        '''
        @brief Determines order in which to scale the filtered twist. This is
               used so that the minium amount of scaling is done while still
               following any specified constraints.

        @param ratios - Unsorted array of input ratios
        @returns valid - Sorted array of valid ordered ratios (could be empty)
        '''

        valid = []
        for r in ratios:
            if r <= 1.0:
                valid.append(r)

        if len(valid) > 1:
            valid.sort()

        return valid

    def _saturate_vel(self, v, l_max, a_max):
        sat_twist = copy.deepcopy(v)

        mag = self._get_mag(sat_twist.linear)

        if mag > l_max:
            ratio = l_max / mag
            sat_twist.linear.x *= ratio
            sat_twist.linear.y *= ratio
            sat_twist.linear.z *= ratio

        mag = self._get_mag(sat_twist.angular)

        if mag > a_max:
            ratio = a_max / mag
            sat_twist.angular.x *= ratio
            sat_twist.angular.y *= ratio
            sat_twist.angular.z *= ratio
        return sat_twist

    def _get_mag(self, v_comp):
        return math.sqrt(v_comp.x**2 + v_comp.y**2 + v_comp.z**2)

    def _saturate_acc(self, v, l_max, a_max, time_delta):
        sat_twist = copy.deepcopy(v)

        acc = self._get_acc(sat_twist, time_delta)

        mag = self._get_mag(acc.linear)
        if mag > l_max:
            ratio = l_max / mag
            acc.linear.x *= ratio
            acc.linear.y *= ratio
            acc.linear.z *= ratio

        mag = self._get_mag(acc.angular)
        if mag > a_max:
            ratio = a_max / mag
            acc.angular.x *= ratio
            acc.angular.y *= ratio
            acc.angular.z *= ratio

        sat_twist.linear.x = (acc.linear.x * time_delta) + self.twist_prev.linear.x
        sat_twist.linear.y = (acc.linear.y * time_delta) + self.twist_prev.linear.y
        sat_twist.linear.z = (acc.linear.z * time_delta) + self.twist_prev.linear.z
        sat_twist.angular.x = (acc.angular.x * time_delta) + self.twist_prev.angular.x
        sat_twist.angular.y = (acc.angular.y * time_delta) + self.twist_prev.angular.y
        sat_twist.angular.z = (acc.angular.z * time_delta) + self.twist_prev.angular.z

        return sat_twist

    def _get_acc(self, v, time_delta):
        '''
        @brief Returns acceleration of all twist components

        @param v - Current twist
        @param time_delta - Time step
        @returns acc - Twist object that contains acceleration for each component
        '''

        acc = Twist()

        acc.linear.x = self._get_slope(v.linear.x, self.twist_prev.linear.x, time_delta)
        acc.linear.y = self._get_slope(v.linear.y, self.twist_prev.linear.y, time_delta)
        acc.linear.z = self._get_slope(v.linear.z, self.twist_prev.linear.z, time_delta)
        acc.angular.x = self._get_slope(v.angular.x, self.twist_prev.angular.x, time_delta)
        acc.angular.y = self._get_slope(v.angular.y, self.twist_prev.angular.y, time_delta)
        acc.angular.z = self._get_slope(v.angular.z, self.twist_prev.angular.z, time_delta)

        return acc

    def _get_slope(self, current, prev, step):
        '''
        @brief Returns the acceleration over a given time step

        @param current - Current value
        @param prev - Previous value
        @param step - Time step
        @returns a - Slope (acceleration)
        '''

        a = (current - prev) / step
        return a
