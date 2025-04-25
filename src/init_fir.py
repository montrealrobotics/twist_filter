#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from component_filters import FIRTwistFilterObject
from filter_twist import TwistFilter


class FIRFilterNode(Node):
    def __init__(self):
        super().__init__('fir_twist_filter')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('active_filters.linear.x', True),
                ('active_filters.linear.y', False),
                ('active_filters.linear.z', False),
                ('active_filters.angular.x', False),
                ('active_filters.angular.y', False),
                ('active_filters.angular.z', True),
                ('num_samples', 2),
                ('weights', '[]')
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

        config = {
            'num_samples': self.get_parameter('num_samples').value,
            'weights': self.get_parameter('weights').value,
        }

        self.get_logger().info(f'Active filters: {active_filters}')

        self.component_filters = FIRTwistFilterObject(active_filters, config)

        self.twist_filter = TwistFilter(self, self.component_filters)


def main(args=None):
    rclpy.init(args=args)
    node = FIRFilterNode()

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
