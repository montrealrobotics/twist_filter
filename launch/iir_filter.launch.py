from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    active_filters_arg = DeclareLaunchArgument(
        'active_filters',
        default_value=PathJoinSubstitution([
            FindPackageShare('twist_filter'),
            'config', 'active_filters_default_iir.yaml'
        ]),
        description='Path to active filters configuration file'
    )
    
    filter_config_arg = DeclareLaunchArgument(
        'filter_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('twist_filter'),
            'config', 'default_iir.yaml'
        ]),
        description='Path to filter configuration file'
    )
    
    filter_name_arg = DeclareLaunchArgument(
        'filter_name',
        default_value='iir_twist_filter',
        description='Name of the filter node'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/cmd_vel',
        description='Input topic name'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/mux/cmd_vel',
        description='Output topic name'
    )

    active_filters = LaunchConfiguration('active_filters')
    filter_config = LaunchConfiguration('filter_config')
    filter_name = LaunchConfiguration('filter_name')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    
    filter_node = Node(
        package='twist_filter',
        executable='init_iir.py',
        name=filter_name,
        parameters=[
            active_filters, 
            filter_config
        ],
        remappings=[
            ('filter_in', input_topic),
            ('filter_out', output_topic)
        ],
        output='screen'
    )
    
    return LaunchDescription([
        active_filters_arg,
        filter_config_arg,
        filter_name_arg,
        input_topic_arg,
        output_topic_arg,
        filter_node
    ])