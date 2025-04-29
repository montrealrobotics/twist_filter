#!/usr/bin/env python3
import rclpy
import json
import numpy as np


class FIRTwistFilterObject:
    def __init__(self, active_filters, config):
        self.linear = {}
        self.angular = {}

        self.num_samples = config['num_samples']
        self.weights = json.loads(config['weights'])
        lin_config = active_filters['linear']
        ang_config = active_filters['angular']
        for key in lin_config:
            if lin_config[key]:
                self.linear[key] = FIRFilter(self.num_samples, self.weights)
                rclpy.logging.get_logger('FIRTwistFilterObject').info(f'Created component filter for linear.{key}')
        for key in ang_config:
            if ang_config[key]:
                self.angular[key] = FIRFilter(self.num_samples, self.weights)
                rclpy.logging.get_logger('FIRTwistFilterObject').info(f'Created component filter for angular.{key}')


    def update_filters(self, parameters):
        """Update filter parameters from ROS2 parameters"""
        logger = rclpy.logging.get_logger('FIRTwistFilterObject')
        num_samples = None
        weights_str = None
        for param in parameters:
            logger.info(f'param: {param.name}')
            if param.name.endswith('num_samples'):
                num_samples = param.value
                logger.info(f'Filter Reconfigure: Number of samples: {num_samples}')
            elif param.name.endswith('weights'):
                weights_str = param.value
                logger.info(f'Filter Reconfigure: Weights: {weights_str}')
        weights = []
        if num_samples is None and weights_str is None:
            logger.warn('No updates made to FIR filter.')
            return True

        if num_samples is None:
            if weights_str and weights_str != '':
                weights = json.loads(weights_str)
                num_samples = len(weights)

        if not weights:
            weights_str = '[]'
            weights = []

        if num_samples == len(weights) or len(weights) == 0:
            self.num_samples = num_samples
            self.weights = weights

            self.reset_filters(self.num_samples, self.weights)
        else:
            logger.warn('Could not update filter. Make sure number of samples and respective weights match.')

    def reset_filters(self, num_samples, weights):
        for key in self.linear:
            self.linear[key].reset(num_samples, weights)
        for key in self.angular:
            self.angular[key].reset(num_samples, weights)
        rclpy.logging.get_logger('FIRTwistFilterObject').info(f"Component filters reset, Sample number: {num_samples}, weights: {weights}")


class LPTwistFilterObject:
    def __init__(self, active_filters, config):
        self.linear = {}
        self.angular = {}

        self.tau = config['tau']
        self.damping = config['damping']
        lin_config = active_filters['linear']
        ang_config = active_filters['angular']
        for key in lin_config:
            if lin_config[key]:
                self.linear[key] = LPFilter(self.tau, self.damping)
                rclpy.logging.get_logger('LPTwistFilterObject').info(f'Created component filter for linear.{key}')
        for key in ang_config:
            if ang_config[key]:
                self.angular[key] = LPFilter(self.tau, self.damping)
                rclpy.logging.get_logger('LPTwistFilterObject').info(f'Created component filter for angular.{key}')


    def update_filters(self, parameters):
        """Update filter parameters from ROS2 parameters"""
        logger = rclpy.logging.get_logger('LPTwistFilterObject')
        tau = None
        damping = None
        for param in parameters:
            if param.name.endswith('tau'):
                if param.value <= 1.0:
                    tau = param.value
                    logger.info(f'Filter Reconfigure: Tau: {tau}')
            elif param.name.endswith('damping'):
                if param.value <=1.0:
                    damping = param.value
                    logger.info(f'Filter Reconfigure: Damping: {damping}')

        if tau:
            self.reset_filters(tau, self.damping)
        elif damping:
            self.reset_filters(self.tau, damping)
        else:
            logger.warn('No updates made to LP filter.')

    def reset_filters(self, tau, damping):
        for key in self.linear:
            self.linear[key].reset(tau, damping)
        for key in self.angular:
            self.angular[key].reset(tau, damping)
        rclpy.logging.get_logger('LPTwistFilterObject').info(f"Component filters reset, Tau: {tau}, damping: {damping}")


class IIRTwistFilterObject:
    def __init__(self, active_filters, config):
        self.linear = {}
        self.angular = {}

        self.num_samples = config['num_samples']
        self.weights = json.loads(config['weights'])
        self.num_out_samples = config['num_out_samples']
        self.out_weights = json.loads(config['out_weights'])

        lin_config = active_filters['linear']
        ang_config = active_filters['angular']
        for key in lin_config:
            if lin_config[key]:
                self.linear[key] = IIRFilter(self.num_samples, self.weights, self.num_out_samples, self.out_weights)
                rclpy.logging.get_logger('IIRTwistFilterObject').info(f'Created component filter for linear.{key}')
        for key in ang_config:
            if ang_config[key]:
                self.angular[key] = IIRFilter(self.num_samples, self.weights, self.num_out_samples, self.out_weights)
                rclpy.logging.get_logger('IIRTwistFilterObject').info(f'Created component filter for angular.{key}')

    def update_filters(self, parameters):
        """Update filter parameters from ROS2 parameters"""
        logger = rclpy.logging.get_logger('IIRTwistFilterObject')

        num_samples = None
        weights_str = None
        num_out_samples = None
        out_weights_str = None

        for param in parameters:
            if param.name.endswith('num_samples'):
                num_samples = param.value
                logger.info(f'Filter Reconfigure: Number of samples: {num_samples}')
            elif param.name.endswith('weights'):
                weights_str = param.value
                logger.info(f'Filter Reconfigure: Weights: {weights_str}')
            elif param.name.endswith('num_out_samples'):
                num_out_samples = param.value
                logger.info(f'Filter Reconfigure: Number of output samples: {num_out_samples}')
            elif param.name.endswith('out_weights'):
                out_weights_str = param.value
                logger.info(f'Filter Reconfigure: Output weights: {out_weights_str}')

        if num_samples or weights_str:
            weights = []
            if num_samples is None:
                if weights_str and weights_str != '':
                    weights = json.loads(weights_str)
                    num_samples = len(weights)

            if not weights:
                weights_str = '[]'
                weights = []

            if num_samples == len(weights) or len(weights) == 0:
                self.num_samples = num_samples
                self.weights = weights
        elif num_out_samples or out_weights_str:
            logger.info(f'else')
            out_weights = []
            if num_out_samples is None:
                if out_weights_str and out_weights_str != '':
                    out_weights = json.loads(weights_str)
                    num_out_samples = len(out_weights)

            if not out_weights:
                out_weights_str = '[]'
                out_weights = []

            if num_out_samples == len(out_weights) or len(out_weights) == 0:
                self.num_out_samples = num_out_samples
                self.out_weights = out_weights
        else:
            logger.warn('No updates made to IIR filter.')
            return True

        logger.info(f'output {self.num_samples}, {self.weights}, {self.out_weights}, {self.num_out_samples}')

        self.reset_filters(self.num_samples, self.weights,  self.num_out_samples, self.out_weights)

    def reset_filters(self, samples, weights, out_samples, out_weights):
        for key in self.linear:
            self.linear[key].reset(samples, weights, out_samples, out_weights)
        for key in self.angular:
            self.angular[key].reset(samples, weights, out_samples, out_weights)
        rclpy.logging.get_logger('IIRTwistFilterObject').info(f"Component filters reset, Sample number {samples}, weights {weights}, out sample number {out_samples}, out weights {out_weights}")


class FilterBase:
    def __init__(self, num_samples):
        self.num_samples = num_samples
        self.samples = [0] * self.num_samples
        self.last_sent_vel = 0

    def __str__(self):
        '''
        @brief Returns sample array in String form
        '''
        return str(self.samples)

    def update_samples(self, data):
        '''
        @brief Shifts values of entire sample array right 1 space
               and then adds new data element to front

        @param data - New data element
        '''
        if len(self.samples) > 1:
            i = self.num_samples - 2
            while i >= 0:
                self.samples[i+1] = self.samples[i]
                i -= 1

        self.samples[0] = data

    def get_result(self):
        '''
        @brief Returns the filter response. The default behavior is to
               implement a moving average filter
        '''
        result = 0
        for i in range(len(self.samples)):
            result += self.samples[i]
        return result / len(self.samples)

    def filter_signal(self, data, time):
        '''
        @brief Takes in new signal smaple, updates sample array,
               and returns the filtered response

        @param data - New data element
        @returns result
        '''
        self.update_samples(data)
        result = self.get_result()

        return result

    def reset(self, num_samples, weights):
        '''
        @brief Resets filter according to new parameters

        @param data - New configuration
        '''
        self.num_samples = num_samples
        self.samples = [0] * num_samples


class FIRFilter(FilterBase):
    def __init__(self, num_samples, weights):
        super(FIRFilter, self).__init__(num_samples)
        self.weights = weights

    def get_result(self):
        '''
        @brief Computes filtered response and returns it

        @returns - result
        '''
        result = 0
        if len(self.weights) > 0:
            for i in range(len(self.samples)):
                result += self.samples[i] * self.weights[i]
        else:
            for i in range(len(self.samples)):
                result += self.samples[i]
            result = result / len(self.samples)
        return result

    def reset(self, num_samples, weights):
        self.num_samples = num_samples
        self.samples = [0] * num_samples
        self.weights = weights


class IIRFilter(FilterBase):
    def __init__(self, num_samples, weights, num_out_samples, out_weights):
        super(IIRFilter, self).__init__(num_samples)
        self.num_out_samples = num_out_samples
        self.out_samples = [0] * self.num_out_samples
        self.weights = weights
        self.out_weights = out_weights
        self.last_sent_vel = 0

    def update_feedback(self, data):
        '''
        @brief Updates the array of output responses (feedback) with new data point

        @param data - Input feedback
        '''
        if len(self.out_samples) > 1:
            i = self.num_out_samples - 2
            while i >= 0:
                self.out_samples[i+1] = self.out_samples[i]
                i -= 1

        self.out_samples[0] = data

    def get_result(self):
        '''
        @brief Computes filtered response and returns it

        @returns - result
        '''
        result = 0
        input_response = 0
        feedback_response = 0

        if len(self.weights) > 0:
            for i in range(len(self.samples)):
                input_response += self.samples[i] * self.weights[i]
        else:
            for i in range(len(self.samples)):
                input_response += self.samples[i]
            input_response = input_response / len(self.samples)

        if len(self.out_weights) > 0:
            for i in range(len(self.out_samples)):
                feedback_response += self.out_samples[i] * self.out_weights[i]
        else:
            for i in range(len(self.out_samples)):
                feedback_response += self.out_samples[i]
            feedback_response = feedback_response / len(self.out_samples)

        result = input_response - feedback_response
        return result

    def filter_signal(self, data, time):
        '''
        @brief Takes in new signal sample, updates sample array,
               and returns the filtered response. It also updates
               feedback sample array after calculating the response

        @param data - New data element
        @returns result
        '''
        self.update_samples(data)
        result = self.get_result()

        if abs(result) < 0.001:
            result = 0

        self.update_feedback(result)

        return result

    def reset(self, num_samples, weights, num_out_samples, out_weights):
        self.num_samples = num_samples
        self.samples = [0] * num_samples
        self.weights = weights

        self.num_out_samples = num_out_samples
        self.out_samples = [0] * num_out_samples
        self.out_weights = out_weights

class LPFilter:
    def __init__(self, tau, damping_factor):
        """
        tau: The time constant for the low-pass filter on acceleration.
        """
        self.tau = tau
        self.damping_factor = damping_factor
        self.acceleration = 0
        self.prev_acceleration = 0
        self.prev_time = 0
        self.last_sent_velocity = 0

    def filter_signal(self, data, current_time):
        """
        Apply the filter on the velocity signal and smooth out jerk by applying a low-pass filter to acceleration.

        current_velocity: The current velocity at the current time step.
        current_time: The current time (in seconds).
        """
        dt = (current_time - self.prev_time) / 1e9
        if dt <= 0:
            return self.last_sent_velocity

        current_acceleration = (data - self.last_sent_velocity) / dt

        filtered_acceleration = self.low_pass_filter(current_acceleration, dt)
        self.prev_acceleration = filtered_acceleration

        smoothed_velocity = self.last_sent_velocity + filtered_acceleration * dt
        self.prev_time = current_time
        damped_velocity = self.damping_factor * self.last_sent_velocity + (1 - self.damping_factor) * smoothed_velocity

        self.last_sent_velocity = damped_velocity

        return damped_velocity

    def low_pass_filter(self, acceleration, dt):
        """
        Apply a simple low-pass filter to the acceleration signal to smooth it out.

        acceleration: The current acceleration (rate of change of velocity).
        dt: Time step between current and previous velocity samples.
        """
        alpha = 1 - np.exp(-dt / self.tau)

        filtered_acceleration = self.prev_acceleration + alpha * (acceleration - self.prev_acceleration)
        return filtered_acceleration

    def reset(self, tau, damping):
        self.tau = tau
        self.damping = damping
