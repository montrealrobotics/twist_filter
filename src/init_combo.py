#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from component_filters import LPTwistFilterObject, IIRTwistFilterObject, FIRTwistFilterObject
from filter_twist import TwistFilter


class LPFilterNode(Node):
    def __init__(self):
        super().__init__('combo_twist_filter')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('filter_1', 'avg'),
                ('filter_2', 'lp'),
                ('active_filters.linear.x', True),
                ('active_filters.linear.y', False),
                ('active_filters.linear.z', False),
                ('active_filters.angular.x', False),
                ('active_filters.angular.y', False),
                ('active_filters.angular.z', True),
                ('filter_1.tau', 0.2),
                ('filter_1.damping', 0.8),
                ('filter_2.tau', 0.2),
                ('filter_2.damping', 0.8),
                ('filter_1.num_samples', 2),
                ('filter_1.weights', '[]'),
                ('filter_1.num_out_samples', 2),
                ('filter_1.out_weights', '[]'),
                ('filter_2.num_samples', 2),
                ('filter_2.weights', '[]'),
                ('filter_2.num_out_samples', 2),
                ('filter_2.out_weights', '[]'),
            ]
        )

        active_filters = {
            'linear': {
                'x': self.get_parameter('active_filters.linear.x').value,
                'y': self.get_parameter('active_filters.linear.y').value,
                'z': self.get_parameter('active_filters.linear.z').value
            },
            'angular': {
                'x': self.get_parameter('active_filters.angular.x').value,
                'y': self.get_parameter('active_filters.angular.y').value,
                'z': self.get_parameter('active_filters.angular.z').value
            }
        }
        filter_1 = self.get_parameter('filter_1').value
        filter_2 = self.get_parameter('filter_2').value
        
        config_1 = {
            'tau': self.get_parameter('filter_1.tau').value,
            'damping': self.get_parameter('filter_1.damping').value,
            'num_samples': self.get_parameter('filter_1.num_samples').value,
            'weights': self.get_parameter('filter_1.weights').value,
            'num_out_samples': self.get_parameter('filter_1.num_out_samples').value,
            'out_weights': self.get_parameter('filter_1.out_weights').value,
        }
        
        config_2 = {
            'tau': self.get_parameter('filter_2.tau').value,
            'damping': self.get_parameter('filter_2.damping').value,
            'num_samples': self.get_parameter('filter_2.num_samples').value,
            'weights': self.get_parameter('filter_2.weights').value,
            'num_out_samples': self.get_parameter('filter_2.num_out_samples').value,
            'out_weights': self.get_parameter('filter_2.out_weights').value,
        }

        self.get_logger().info(f'Active filters: {active_filters}')
        
        self.component_filters = []
        filter_map = {
            'iir': lambda: IIRTwistFilterObject(active_filters, config_1),
            'fir': lambda: FIRTwistFilterObject(active_filters, config_1),
            'avg': lambda: FIRTwistFilterObject(active_filters, config_1),
            'lp': lambda: LPTwistFilterObject(active_filters, config_1)
        }
        filter_map_2 = {
            'iir': lambda: IIRTwistFilterObject(active_filters, config_2),
            'fir': lambda: FIRTwistFilterObject(active_filters, config_2),
            'avg': lambda: FIRTwistFilterObject(active_filters, config_2),
            'lp': lambda: LPTwistFilterObject(active_filters, config_2)
        }
        if filter_1 in filter_map:
            self.component_filters.append(filter_map[filter_1]())
            self.get_logger().info(f'Added filter 1: {filter_1}')
        else:
            self.component_filters.append(filter_map['avg']())
            raise ValueError(f"Unknown filter type: {filter_1}, adding average filter instead.")

        if filter_2 in filter_map:
            self.component_filters.append(filter_map_2[filter_2]())
            self.get_logger().info(f'Added filter 2: {filter_2}')
        else:
            raise ValueError(f"Unknown filter type: {filter_2}, adding average filter instead.")

        self.twist_filter = TwistFilter(self, self.component_filters)


def main(args=None):
    rclpy.init(args=args)
    node = LPFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
